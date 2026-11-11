// Coolant pump: the PWM output, the task and its logging.
//
// pumpTask (TASK_PRIO_PUMP) runs every PUMP_PERIOD_MS: it reads the VESC temperatures and the
// limits the link task read with COMM_GET_MCCONF (g_state.vesc_limits) under the state lock,
// runs the pure controller from pump_logic.h, sets the LEDC duty and publishes g_state.pump.
// It never logs; the supervisor does, through pump_log_events() / pump_log_summary().
//
// Hardware (README section 2): PIN_PUMP_PWM -> 100 R -> gate of a logic-level N-MOSFET (or
// -> 4.7k -> base of an NPN such as BC337), 10k pull-down from gate/base to GND; drain /
// collector on the pump's PWM wire, which is pulled up to 5 V. The pump wants a 5 V signal and
// may carry a pull-up of its own, so the 3.3 V pin never touches the wire. The stage inverts,
// which the LEDC channel undoes (output_invert = PUMP_PWM_INVERT), and it fails safe: whenever
// this firmware is not driving the pin - the bootloader, a reset, a crash, a failed PWM setup -
// the pull-down turns the transistor off and the pump runs flat out.
//
// Tachometer (PUMP_TACH_ENABLE, README section 2 "Pump tach"): the pump's FG wire pulls
// PIN_PUMP_TACH low through a Schottky diode (pull-up 10k to 3V3 on the pin side), and a PCNT
// unit counts the falling edges. The task hands the running total to the pure logic in
// include/pump_tach.h every tick, before the controller runs, so a stall found this tick
// already sets this tick's duty.
#include "pump.h"

#if PUMP_ENABLE

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>

#include "pump_logic.h"
#include "pump_tach.h"

#if PUMP_TACH_ENABLE
#include <driver/pulse_cnt.h>
#endif

static const char *TAG = "pump";

static constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
static constexpr ledc_channel_t kChannel = LEDC_CHANNEL_0;
static constexpr uint32_t kStopLevel = PUMP_PWM_INVERT ? 1u : 0u;  // the pin level that stops the pump

static PumpCtl s_ctl;          // pumpTask only
static bool s_pwm_ok = false;  // set by pump_start() before the task exists, read-only afterwards

#if PUMP_TACH_ENABLE
static constexpr int kTachHigh = 32767;     // the 16-bit counter's top; the driver accumulates past it
static pcnt_unit_handle_t s_pcnt = nullptr;  // set by pump_start() before the task exists; null = no counter
static PumpTach s_tach;                      // pumpTask only
static uint32_t s_tach_total = 0;            // pumpTask only: the last count read
#endif

// ---------------------------------------------------------------- the output
void pump_gpio_early_init() {
  gpio_set_level((gpio_num_t)PIN_PUMP_PWM, kStopLevel);  // the output latch first: no glitch when the driver turns on
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << PIN_PUMP_PWM;
  io.mode = GPIO_MODE_OUTPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  if (gpio_config(&io) == ESP_OK) gpio_set_level((gpio_num_t)PIN_PUMP_PWM, kStopLevel);
}

// Lets go of the pin: no driver, no internal pull (a weak pull-up would hold the gate half on
// against the stage's pull-down). The MOSFET turns off and the pump runs at full speed.
static void pin_release() {
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << PIN_PUMP_PWM;
  io.mode = GPIO_MODE_DISABLE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io);
}

static bool pwm_setup() {
  ledc_timer_config_t t = {};
  t.speed_mode = kMode;
  t.duty_resolution = (ledc_timer_bit_t)PUMP_PWM_RES_BITS;
  t.timer_num = kTimer;
  t.freq_hz = PUMP_PWM_FREQ_HZ;
  t.clk_cfg = LEDC_AUTO_CLK;
  esp_err_t err = ledc_timer_config(&t);
  if (err == ESP_OK) {
    ledc_channel_config_t ch = {};
    ch.gpio_num = PIN_PUMP_PWM;
    ch.speed_mode = kMode;
    ch.channel = kChannel;
    ch.timer_sel = kTimer;
    ch.duty = 0;  // stopped, the level pump_gpio_early_init() already holds
    ch.hpoint = 0;
    ch.flags.output_invert = PUMP_PWM_INVERT ? 1u : 0u;
    err = ledc_channel_config(&ch);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "LEDC setup on GPIO%d (%d Hz, %d bit) failed: %s - pin released, the pump runs at FULL speed",
             PIN_PUMP_PWM, PUMP_PWM_FREQ_HZ, PUMP_PWM_RES_BITS, esp_err_to_name(err));
    return false;
  }
  return true;
}

// ---------------------------------------------------------------- the tachometer
#if PUMP_TACH_ENABLE
// One PCNT unit, one channel: +1 on every falling edge of the FG input (the pump pulling it
// low through the diode), nothing on the rising one. accum_count with a watch point at the
// top lets the driver carry the 16-bit counter past 32767, so the task reads a running total.
static bool tach_setup() {
  pcnt_unit_handle_t unit = nullptr;
  pcnt_channel_handle_t chan = nullptr;
  const char *step = "unit";
  pcnt_unit_config_t uc = {};
  uc.low_limit = -1;  // the driver wants a limit below 0; the count only ever goes up
  uc.high_limit = kTachHigh;
  uc.flags.accum_count = 1;
  esp_err_t err = pcnt_new_unit(&uc, &unit);
  if (err == ESP_OK && PUMP_TACH_GLITCH_NS > 0) {
    step = "glitch filter";
    pcnt_glitch_filter_config_t gf = {};
    gf.max_glitch_ns = PUMP_TACH_GLITCH_NS;
    err = pcnt_unit_set_glitch_filter(unit, &gf);
  }
  if (err == ESP_OK) {
    step = "channel";
    pcnt_chan_config_t cc = {};
    cc.edge_gpio_num = PIN_PUMP_TACH;
    cc.level_gpio_num = -1;
    err = pcnt_new_channel(unit, &cc, &chan);
  }
  if (err == ESP_OK) {
    step = "edge action";
    err = pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  }
  if (err == ESP_OK) {
    step = "watch point";
    err = pcnt_unit_add_watch_point(unit, kTachHigh);
  }
  // The external 10k does the work; the weak internal pull-up only keeps an unwired input from floating.
  if (err == ESP_OK) {
    step = "pull-up";
    err = gpio_pullup_en((gpio_num_t)PIN_PUMP_TACH);
  }
  if (err == ESP_OK) {
    step = "enable";
    err = pcnt_unit_enable(unit);
  }
  if (err == ESP_OK) {
    step = "clear";
    err = pcnt_unit_clear_count(unit);
  }
  if (err == ESP_OK) {
    step = "start";
    err = pcnt_unit_start(unit);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "tach: PCNT %s on GPIO%d failed: %s - no pump speed, no stall alarm", step, PIN_PUMP_TACH,
             esp_err_to_name(err));
    if (unit) pcnt_unit_disable(unit);  // fails harmlessly when it never got enabled
    if (chan) pcnt_del_channel(chan);
    if (unit) pcnt_del_unit(unit);
    return false;
  }
  s_pcnt = unit;
  return true;
}

// Reads the counter and judges it against the duty driven over the period that just ended.
static uint8_t tach_tick(float applied_duty, uint32_t now) {
  if (!s_pcnt) return PUMP_TACH_ERR;
  int count = 0;
  if (pcnt_unit_get_count(s_pcnt, &count) == ESP_OK) s_tach_total = (uint32_t)count;  // wraps: only differences count
  return pump_tach_step(s_tach, s_tach_total, applied_duty, now);
}
#endif

// ---------------------------------------------------------------- the task
static void publish(const PumpTargets &tg, bool limits_vesc, uint32_t now) {
  PumpStatus p;
  memset(&p, 0, sizeof p);
  p.mode = s_pwm_ok ? s_ctl.mode : (uint8_t)PUMP_MODE_FAULT;
  p.why = s_pwm_ok ? s_ctl.why : (uint8_t)PUMP_WHY_PWM;
  p.duty_pct = s_pwm_ok ? s_ctl.duty : 100.0f;  // a released pin: the hardware's full speed
  p.limits_vesc = limits_vesc;
  p.have_err = s_ctl.have_err;
  p.err_c = s_ctl.err;
  p.integ = s_ctl.integ;
  p.tgt_mot = tg.tgt_mot;
  p.tgt_fet = tg.tgt_fet;
  p.derate_mot = tg.derate_mot;
  p.derate_fet = tg.derate_fet;
  p.t_ms = now ? now : 1u;
#if PUMP_TACH_ENABLE
  p.tach = s_pcnt ? s_tach.state : (uint8_t)PUMP_TACH_ERR;
  p.tach_full = s_pcnt && s_tach.full;
  p.tach_rpm = s_pcnt ? s_tach.rpm : 0.0f;
  p.tach_pulses = s_tach_total;
  p.tach_alarms = s_tach.alarms;
  p.tach_since_ms = s_tach.since;
#endif
  if (state_lock()) {
    g_state.pump = p;
    state_unlock();
  }
}

static void pump_task(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t prev_ms = 0;
  uint32_t written = 0;  // LEDC compare value last set (the channel starts at 0)
  for (;;) {
    hb_touch(hb_pump);

    PumpInputs in;
    VescLimitsState lim;
    uint32_t now = 0;
    bool got = false;
    if (state_lock()) {
      now = state_now_ms();  // inside the lock: every stamp read below is <= now
      in = pump_inputs(g_state.vesc.t, now);
      lim = g_state.vesc_limits;
      state_unlock();
      got = true;
    }
    // A lock timeout (counted by state_lock) keeps the previous duty for one more period.
    if (got) {
      bool limits_vesc = false;
      const PumpTargets tg = pump_targets(pump_limits(lim, &limits_vesc));
      const float dt = prev_ms ? (float)(uint32_t)(now - prev_ms) / 1000.0f : 0.0f;
      prev_ms = now ? now : 1u;
#if PUMP_TACH_ENABLE
      // The duty the pump saw over the last period: a released pin is the hardware's full speed.
      in.tach = tach_tick(s_pwm_ok ? s_ctl.duty : 100.0f, now);
#endif
      const uint32_t count = pump_duty_to_ledc(pump_step(s_ctl, in, tg, now, dt), PUMP_PWM_RES_BITS);
      if (s_pwm_ok && count != written && ledc_set_duty(kMode, kChannel, count) == ESP_OK &&
          ledc_update_duty(kMode, kChannel) == ESP_OK)
        written = count;
      publish(tg, limits_vesc, now);
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(PUMP_PERIOD_MS));
  }
}

// ---------------------------------------------------------------- public API
bool pump_start() {
  pump_init(s_ctl);
#if PUMP_TACH_ENABLE
  pump_tach_init(s_tach);
  const bool tach_ok = tach_setup();  // before the task: it reads s_pcnt unlocked
#endif
  s_pwm_ok = pwm_setup();
  if (!s_pwm_ok) pin_release();
  if (xTaskCreatePinnedToCore(pump_task, "pump", PUMP_TASK_STACK, nullptr, TASK_PRIO_PUMP, nullptr, TASK_CORE_APP) !=
      pdPASS) {
    // Nothing would ever raise the duty from 0: let go of the pin so the pump runs flat out.
    pin_release();
    s_pwm_ok = false;
    ESP_LOGE(TAG, "task create failed: pin released, the pump runs at FULL speed");
    return false;
  }
  ESP_LOGI(TAG,
           "pump started: GPIO%d %d Hz %d bit%s, min %d %%, blind %d %%, Kp %.1f Ki %.2f Kd %.1f, target = derating "
           "start - %d C, limits %s",
           PIN_PUMP_PWM, PUMP_PWM_FREQ_HZ, PUMP_PWM_RES_BITS, PUMP_PWM_INVERT ? " inverted" : "", PUMP_MIN_DUTY_PCT,
           PUMP_BLIND_DUTY_PCT, (double)PUMP_KP, (double)PUMP_KI, (double)PUMP_KD, PUMP_TARGET_MARGIN_C,
           PUMP_READS_MCCONF ? "from the VESC (COMM_GET_MCCONF)" : "PUMP_DEFAULT_* only");
#if PUMP_TACH_ENABLE
  if (tach_ok)
    ESP_LOGI(TAG,
             "tach on GPIO%d: %d pulse(s)/rev, stall below %d rpm for %d ms after a %d ms spin-up -> %s, glitch filter %d ns",
             PIN_PUMP_TACH, PUMP_TACH_PPR, PUMP_TACH_MIN_RPM, PUMP_TACH_STALL_MS, PUMP_TACH_SPINUP_MS,
             PUMP_TACH_STALL_DUTY_PCT ? "full duty" : "alert only", PUMP_TACH_GLITCH_NS);
#endif
  if (PUMP_FORCE_DUTY_PCT >= 0)
    ESP_LOGW(TAG, "PUMP_FORCE_DUTY_PCT %d: fixed duty, the control loop is OFF (commissioning build)",
             PUMP_FORCE_DUTY_PCT);
  return s_pwm_ok;
}

// ---------------------------------------------------------------- the limits (link task context)
uint8_t pump_mcconf_reply(const uint8_t *payload, uint16_t len, uint32_t now) {
  const VescMcconfRead r = vesc_mcconf_parse(payload, len);
  VescLimitsState prev;
  if (!state_lock()) return MCCONF_NONE;
  VescLimitsState &v = g_state.vesc_limits;
  prev = v;
  v.result = r.result;
  v.sig = r.sig;
  memcpy(v.fw, r.fw, sizeof v.fw);
  if (r.result == MCCONF_OK) v.lim = r.lim;
  v.t_ms = now ? now : 1u;
  state_unlock();

  // A refresh that changes nothing stays quiet.
  const bool same_lim = r.result != MCCONF_OK || memcmp(&prev.lim, &r.lim, sizeof r.lim) == 0;
  if (prev.result == r.result && prev.sig == r.sig && same_lim) return r.result;
  const VescTempLimits &l = r.lim;
  switch (r.result) {
    case MCCONF_OK:
      ESP_LOGI(TAG,
               "VESC temp limits (mcconf FW %s): motor %.0f/%.0f C, MOSFET %.0f/%.0f C, accel_dec %.2f -> the VESC "
               "derates from motor %.1f / MOSFET %.1f C",
               r.fw, (double)l.mot_start, (double)l.mot_end, (double)l.fet_start, (double)l.fet_end,
               (double)l.accel_dec, (double)vesc_derate_start(l.mot_start, l.accel_dec),
               (double)vesc_derate_start(l.fet_start, l.accel_dec));
      break;
    case MCCONF_UNKNOWN_SIG:
      ESP_LOGW(TAG, "VESC mcconf signature 0x%08lX (%u B) is no firmware layout this build knows: pump targets from "
                    "PUMP_DEFAULT_* (include/vesc_mcconf.h lists the known ones)",
               (unsigned long)r.sig, (unsigned)len);
      break;
    case MCCONF_IMPLAUSIBLE:
      ESP_LOGW(TAG, "VESC mcconf (FW %s) carries implausible temperature limits: pump targets from PUMP_DEFAULT_*", r.fw);
      break;
    default:
      ESP_LOGW(TAG, "VESC mcconf reply unusable (%s, %u B)", vesc_mcconf_result_str(r.result), (unsigned)len);
      break;
  }
  return r.result;
}

void pump_mcconf_sent() {
  if (state_lock()) {
    g_state.vesc_limits.requests++;
    state_unlock();
  }
}

// ---------------------------------------------------------------- logging (supervisor)
// "58.3" / "--" (stale) / "n/a" (no sensor).
static void fmt_temp(char *buf, size_t n, bool fresh_s4, float t) {
  if (!fresh_s4) snprintf(buf, n, "--");
  else if (!vesc_temp_plausible(t)) snprintf(buf, n, "n/a");
  else snprintf(buf, n, "%.1f", (double)t);
}

void pump_log_events(const SharedState &s, uint32_t now) {
  static uint8_t last_mode = 0xFF, last_why = 0xFF;
  static bool last_lim = false, lim_reported = false;
  const PumpStatus &p = s.pump;
  if (p.t_ms == 0) return;  // the task has not ticked yet

  if (p.mode != last_mode || (p.mode == PUMP_MODE_FULL && p.why != last_why)) {
    const bool f4 = vesc_fresh(&s.vesc.t, VESC_IDX_STATUS_4, now, VESC_STALE_R1_MS);
    char tm[8], tf[8];
    fmt_temp(tm, sizeof tm, f4, s.vesc.t.temp_motor);
    fmt_temp(tf, sizeof tf, f4, s.vesc.t.temp_fet);
    switch (p.mode) {
      case PUMP_MODE_OFF: ESP_LOGI(TAG, "pump OFF (%s): motor %s C, MOSFET %s C", pump_why_str(p.why), tm, tf); break;
      case PUMP_MODE_RUN:
        ESP_LOGI(TAG, "pump RUN (%s) at %.0f %%: motor %s/%.1f C, MOSFET %s/%.1f C", pump_why_str(p.why),
                 (double)p.duty_pct, tm, (double)p.tgt_mot, tf, (double)p.tgt_fet);
        break;
      case PUMP_MODE_FULL:
        if (p.why == PUMP_WHY_DERATE)
          ESP_LOGW(TAG, "pump FULL: the VESC derates from here on - motor %s C (derating %.1f), MOSFET %s C (derating %.1f)",
                   tm, (double)p.derate_mot, tf, (double)p.derate_fet);
        else
          ESP_LOGW(TAG, "pump FULL (%s) at %.0f %%: the tach says it does not turn", pump_why_str(p.why),
                   (double)p.duty_pct);
        break;
      case PUMP_MODE_BLIND:
        ESP_LOGW(TAG, "pump BLIND (%s): %d %% until the VESC temperatures are back", pump_why_str(p.why),
                 PUMP_BLIND_DUTY_PCT);
        break;
      case PUMP_MODE_FORCED: ESP_LOGW(TAG, "pump FORCED to %.0f %% (PUMP_FORCE_DUTY_PCT)", (double)p.duty_pct); break;
      case PUMP_MODE_FAULT:
        ESP_LOGE(TAG, "pump FAULT: no PWM output, the pin is released and the pump runs at FULL speed");
        break;
      default: break;
    }
    last_mode = p.mode;
    last_why = p.why;
  }

#if PUMP_TACH_ENABLE
  // Tach verdicts. WAIT (spin-up / spin-down) is silent, and so is OK after OK: the pump
  // starting and stopping is not news, a stall and its end are.
  static uint8_t last_verdict = PUMP_TACH_NONE;
  if (p.tach != PUMP_TACH_WAIT && p.tach != PUMP_TACH_NONE && p.tach != last_verdict) {
    switch (p.tach) {
      case PUMP_TACH_OK:
        ESP_LOGI(TAG, "pump tach OK%s: %.0f rpm at %.0f %%", last_verdict == PUMP_TACH_NONE ? "" : " again",
                 (double)p.tach_rpm, (double)p.duty_pct);
        break;
      case PUMP_TACH_STALL:
        ESP_LOGW(TAG,
                 "pump tach STALL: %.0f rpm although it should run (blocked, unpowered or unplugged pump, or the FG "
                 "wire) - duty %d %%",
                 (double)p.tach_rpm, PUMP_TACH_STALL_DUTY_PCT ? PUMP_TACH_STALL_DUTY_PCT : (int)(p.duty_pct + 0.5f));
        break;
      case PUMP_TACH_NO_SIGNAL:
        ESP_LOGE(TAG,
                 "pump tach NO SIGNAL: not one FG pulse since boot although the pump should run (FG wire, the diode "
                 "stage, or a dead pump) - duty %d %%",
                 PUMP_TACH_STALL_DUTY_PCT ? PUMP_TACH_STALL_DUTY_PCT : (int)(p.duty_pct + 0.5f));
        break;
      case PUMP_TACH_RUNS_AT_0:
        ESP_LOGW(TAG,
                 "pump tach: %.0f rpm while commanded to 0 %% - the PWM stage does not stop it (transistor, pull-up or "
                 "white wire): it runs at full speed",
                 (double)p.tach_rpm);
        break;
      case PUMP_TACH_ERR: ESP_LOGE(TAG, "pump tach: no pulse counter (see the setup error at boot)"); break;
      default: break;
    }
    last_verdict = p.tach;
  }
#endif

  // The targets move when the limits arrive from the VESC (or stop arriving): say where they are now.
  if (!lim_reported || p.limits_vesc != last_lim) {
    ESP_LOGI(TAG, "pump targets: motor %.1f C, MOSFET %.1f C (the VESC derates from %.1f / %.1f C; limits %s)",
             (double)p.tgt_mot, (double)p.tgt_fet, (double)p.derate_mot, (double)p.derate_fet,
             p.limits_vesc ? "read from the VESC" : "PUMP_DEFAULT_*");
    last_lim = p.limits_vesc;
    lim_reported = true;
  }
}

void pump_log_summary(const SharedState &s, uint32_t now) {
  const PumpStatus &p = s.pump;
  const bool f4 = vesc_fresh(&s.vesc.t, VESC_IDX_STATUS_4, now, VESC_STALE_R1_MS);
  char tm[8], tf[8], err[12];
  fmt_temp(tm, sizeof tm, f4, s.vesc.t.temp_motor);
  fmt_temp(tf, sizeof tf, f4, s.vesc.t.temp_fet);
  if (p.have_err) snprintf(err, sizeof err, "%+.1f", (double)p.err_c);
  else snprintf(err, sizeof err, "--");
  bool from_vesc = false;
  const VescTempLimits l = pump_limits(s.vesc_limits, &from_vesc);
  char src[24];
  if (from_vesc) snprintf(src, sizeof src, "VESC(%s)", s.vesc_limits.fw);
  else if (s.vesc_limits.result == MCCONF_UNKNOWN_SIG)
    snprintf(src, sizeof src, "DEF(sig %08lX)", (unsigned long)s.vesc_limits.sig);
  else snprintf(src, sizeof src, "DEF");
  char tach[80] = "";
#if PUMP_TACH_ENABLE
  // rpm next to the duty on every line: the record a future slow / fast check is built from.
  snprintf(tach, sizeof tach, " | tach %s rpm=%.0f%s pulses=%lu alarms=%lu", pump_tach_str(p.tach), (double)p.tach_rpm,
           p.tach_full ? "" : "~", (unsigned long)p.tach_pulses, (unsigned long)p.tach_alarms);
#endif
  ESP_LOGI(TAG,
           "PUMP %s(%s) %.0f%% Tm=%s/%.1f Tf=%s/%.1f err=%s I=%.1f | lim %s M%.0f/%.0f F%.0f/%.0f dec%.2f "
           "reads=%lu age=%lums%s",
           pump_mode_str(p.mode), pump_why_str(p.why), (double)p.duty_pct, tm, (double)p.tgt_mot, tf,
           (double)p.tgt_fet, err, (double)p.integ, src, (double)l.mot_start, (double)l.mot_end, (double)l.fet_start,
           (double)l.fet_end, (double)l.accel_dec, (unsigned long)s.vesc_limits.requests,
           (unsigned long)age_ms(p.t_ms, now), tach);
}

#endif  // PUMP_ENABLE
