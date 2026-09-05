// CAN / VESC listener: legacy TWAI driver, passive reception of the VESC
// STATUS_1..6 broadcast frames, bus-off recovery and health reporting.
//
// Protocol traps this file is built around (see the plan / README for detail):
//
//  * Legacy driver/twai.h ONLY. The new IDF 5.5 "onchip" TWAI node driver must
//    never be mixed in: IDF's driver-conflict check is compiled out of the
//    precompiled Arduino libraries, so mixing the two fails silently at runtime.
//
//  * TWAI_MODE_NORMAL with tx_queue_len = 0 and no transmit call anywhere in
//    this firmware: the controller only asserts the ACK bit. The VESC's bxCAN
//    has no automatic-retransmission limit, so with a non-ACKing listener as the
//    only other node it goes error-passive and repeats ONE stale STATUS frame
//    forever ("lonely VESC"). CAN_LISTEN_ONLY=1 (true listen-only, no ACK) is
//    only safe when another ACKing node exists on the bus.
//
//  * After a bus-off recovery the legacy driver parks in STOPPED, not RUNNING:
//    twai_start() has to be called again on TWAI_ALERT_BUS_RECOVERED.
//
//  * STATUS_5 wire order is tachometer THEN v_in (handled in vesc_status.h).
//
// Concurrency: the state mutex is held only for field copies; decoding and all
// logging happen outside it. canTask is the sole writer of g_state.vesc and the
// TWAI part of g_state.can.
#include "can_vesc.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/twai.h>
#include <esp_err.h>

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "shared_state.h"
#include "vesc_status.h"

// ---------------------------------------------------------------------------
// Local tunables (not in config.h yet; override with -D if needed)
// ---------------------------------------------------------------------------
#ifndef CAN_TASK_STACK
#define CAN_TASK_STACK 4096         // bytes: ESP-IDF's xTaskCreate() takes the stack size in bytes
#endif
#ifndef CAN_INSTALL_RETRY_MS
#define CAN_INSTALL_RETRY_MS 5000   // retry twai_driver_install()/twai_start() while they keep failing
#endif
#ifndef CAN_RX_TIMEOUT_MS
#define CAN_RX_TIMEOUT_MS 100       // twai_receive() block time; also bounds the heartbeat period
#endif
#ifndef CAN_ERR_LOG_MIN_MS
#define CAN_ERR_LOG_MIN_MS 2000     // rate limit for the noisy alerts (BUS_ERROR, FIFO overrun, RX queue full, warn level)
#endif
#ifndef CAN_ERR_WARN_LEVEL
#define CAN_ERR_WARN_LEVEL 96       // ISO 11898-1 error-warning limit for TEC/REC
#endif

// shared_state.h mirrors twai_state_t so it can stay IDF-free; make sure the mirror never drifts.
static_assert((int)TWAI_STATE_STOPPED == CAN_STATE_STOPPED, "CanState mirror out of sync with twai_state_t");
static_assert((int)TWAI_STATE_RUNNING == CAN_STATE_RUNNING, "CanState mirror out of sync with twai_state_t");
static_assert((int)TWAI_STATE_BUS_OFF == CAN_STATE_BUS_OFF, "CanState mirror out of sync with twai_state_t");
static_assert((int)TWAI_STATE_RECOVERING == CAN_STATE_RECOVERING, "CanState mirror out of sync with twai_state_t");

// ---------------------------------------------------------------------------
// Module state (only touched from canTask, plus the one-shot can_vesc_start())
// ---------------------------------------------------------------------------
static bool s_task_started = false;
static bool s_installed = false;          // twai_driver_install() succeeded (driver may still be STOPPED)
static uint32_t s_last_install_try_ms = 0;
static uint32_t s_last_health_ms = 0;
static bool s_invalid_state_logged = false;
// Rate limiting for alerts that can fire thousands of times per second on a faulty bus.
static uint32_t s_bus_err_pending = 0, s_bus_err_log_ms = 0;
static uint32_t s_qfull_pending = 0, s_qfull_log_ms = 0;
static uint32_t s_overrun_pending = 0, s_overrun_log_ms = 0;
static uint32_t s_warn_log_ms = 0;

// millis() with 0 mapped to 1 so that 0 keeps meaning "never" in every timestamp field.
static inline uint32_t now_nonzero() {
  const uint32_t n = millis();
  return n ? n : 1u;
}

static inline bool due(uint32_t now, uint32_t last, uint32_t period) {
  return last == 0 || (uint32_t)(now - last) >= period;
}

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
const char *can_state_str(int can_state) {
  switch (can_state) {
    case CAN_STATE_UNINSTALLED: return "UNINST";
    case CAN_STATE_STOPPED: return "STOP";
    case CAN_STATE_RUNNING: return "RUN";
    case CAN_STATE_BUS_OFF: return "BUSOFF";
    case CAN_STATE_RECOVERING: return "RECOV";
    default: return "?";
  }
}

static int map_state(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED: return CAN_STATE_STOPPED;
    case TWAI_STATE_RUNNING: return CAN_STATE_RUNNING;
    case TWAI_STATE_BUS_OFF: return CAN_STATE_BUS_OFF;
    case TWAI_STATE_RECOVERING: return CAN_STATE_RECOVERING;
    default: return CAN_STATE_UNINSTALLED;
  }
}

static void set_can_state(int st) {
  if (state_lock()) {
    g_state.can.state = st;
    state_unlock();
  }
}

static void count_dropped() {
  if (state_lock()) {
    g_state.vesc.frames_dropped++;
    state_unlock();
  }
}

[[maybe_unused]] static const char *mode_str() {  // log-only helper (unused when CORE_DEBUG_LEVEL < 3)
  return CAN_LISTEN_ONLY ? "LISTEN_ONLY (no ACK)" : "NORMAL (ACK-only, never transmits)";
}

// Bit timing for the configured bitrate. On the ESP32-C6 (40 MHz XTAL) the 500k
// and 1M presets sample at 80 %, compatible with the VESC's 78.6 %; 125k/250k
// presets sample at 75 %. The actual sample point is logged at install.
static twai_timing_config_t timing_config() {
#if CAN_BITRATE_KBPS == 125
  twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
#elif CAN_BITRATE_KBPS == 250
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
#elif CAN_BITRATE_KBPS == 500
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
#elif CAN_BITRATE_KBPS == 1000
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
#else
#error "CAN_BITRATE_KBPS must be 125, 250, 500 or 1000"
#endif
  return t;
}

// The legacy HAL programs sync(1) + tseg_1 + tseg_2 and ignores prop_seg (it is
// only used by the new node driver), so the sample point is (1 + tseg_1) / total.
[[maybe_unused]] static int sample_point_percent(const twai_timing_config_t &t) {
  const int quanta = 1 + t.tseg_1 + t.tseg_2;
  return quanta > 0 ? (100 * (1 + t.tseg_1)) / quanta : 0;
}

// Exponential moving average, seeded with the first sample of its source message.
static inline float ema_step(float prev, float x, bool first) {
  return first ? x : (CAN_EMA_ALPHA * x + (1.0f - CAN_EMA_ALPHA) * prev);
}

// ---------------------------------------------------------------------------
// Driver install / start
// ---------------------------------------------------------------------------
// Puts an installed driver into RUNNING (idempotent) and mirrors the state.
static bool can_run() {
  [[maybe_unused]] const esp_err_t err = twai_start();  // ESP_ERR_INVALID_STATE when already running: harmless
  twai_status_info_t st{};
  const bool have_st = twai_get_status_info(&st) == ESP_OK;  // fails only when the driver is not installed
  const int cs = have_st ? map_state(st.state) : CAN_STATE_UNINSTALLED;
  set_can_state(cs);
  if (cs != CAN_STATE_RUNNING) log_e("CAN: twai_start failed: %s (state %s)", esp_err_to_name(err), can_state_str(cs));
  return cs == CAN_STATE_RUNNING;
}

static bool can_install() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX,
                                                        CAN_LISTEN_ONLY ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  g.rx_queue_len = CAN_RX_QUEUE_LEN;  // 6 status frames arrive as a burst every 20 ms; the default 5 is too small
  g.tx_queue_len = 0;                 // no TX queue at all: this node only ever asserts the ACK bit
  g.alerts_enabled = TWAI_ALERT_ERR_PASS | TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL |
                     TWAI_ALERT_RX_FIFO_OVERRUN | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                     TWAI_ALERT_ABOVE_ERR_WARN;
  const twai_timing_config_t t = timing_config();
  // Hardware filter wide open: the controller id is filtered in software so we
  // can lock onto whichever VESC talks first (VESC_CAN_ID == -1).
  const twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  const esp_err_t err = twai_driver_install(&g, &t, &f);
  if (err == ESP_ERR_INVALID_STATE) {
    log_w("CAN: driver already installed, reusing it");
  } else if (err != ESP_OK) {
    log_e("CAN: twai_driver_install failed: %s (retry in %lu ms)", esp_err_to_name(err),
          (unsigned long)CAN_INSTALL_RETRY_MS);
    set_can_state(CAN_STATE_UNINSTALLED);
    return false;
  }
  s_installed = true;
  s_invalid_state_logged = false;
  if (!can_run()) return false;

  log_i("CAN: driver installed, state RUNNING: mode=%s bitrate=%dk tx=GPIO%d rx=GPIO%d rx_queue=%d sample_point=%d%%",
        mode_str(), CAN_BITRATE_KBPS, PIN_CAN_TX, PIN_CAN_RX, CAN_RX_QUEUE_LEN, sample_point_percent(t));
  return true;
}

// ---------------------------------------------------------------------------
// Alerts (bus-off recovery, error state changes, queue overflows)
// ---------------------------------------------------------------------------
static void handle_alerts(uint32_t a, uint32_t now) {
  // 1) Driver actions first: they are time-critical, the logging below is not.
  esp_err_t recovery_err = ESP_OK, start_err = ESP_OK;
  if (a & TWAI_ALERT_BUS_OFF) recovery_err = twai_initiate_recovery();
  if (a & TWAI_ALERT_BUS_RECOVERED) start_err = twai_start();  // legacy driver parks in STOPPED after recovery

  twai_status_info_t st{};
  const bool have_st = twai_get_status_info(&st) == ESP_OK;
  [[maybe_unused]] const unsigned long tec = have_st ? (unsigned long)st.tx_error_counter : 0ul;  // log-only
  [[maybe_unused]] const unsigned long rec = have_st ? (unsigned long)st.rx_error_counter : 0ul;

  // 2) Counters under the lock, no I/O.
  if (state_lock()) {
    CanHealth &c = g_state.can;
    if (a & TWAI_ALERT_BUS_OFF) {
      c.bus_off_count++;
      c.state = recovery_err == ESP_OK ? CAN_STATE_RECOVERING : CAN_STATE_BUS_OFF;
    }
    if (a & TWAI_ALERT_BUS_RECOVERED) {
      c.recoveries++;
      c.state = start_err == ESP_OK ? CAN_STATE_RUNNING : CAN_STATE_STOPPED;
    }
    if (a & TWAI_ALERT_RX_QUEUE_FULL) c.queue_full_events++;
    if (a & TWAI_ALERT_ERR_PASS) c.err_passive_events++;
    if (a & TWAI_ALERT_BUS_ERROR) c.bus_error_events++;
    if (have_st) {
      c.tec = st.tx_error_counter;
      c.rec = st.rx_error_counter;
      c.bus_error_count = st.bus_error_count;
      c.rx_missed = st.rx_missed_count;
      c.rx_overrun = st.rx_overrun_count;
    }
    state_unlock();
  }

  // 3) Logs, rate-limited where a faulty bus could flood the console.
  if (a & TWAI_ALERT_BUS_OFF)
    log_e("CAN: BUS_OFF (TEC=%lu REC=%lu), recovery %s", tec, rec,
          recovery_err == ESP_OK ? "started" : esp_err_to_name(recovery_err));
  if (a & TWAI_ALERT_BUS_RECOVERED)
    log_i("CAN: bus recovered, twai_start -> %s", start_err == ESP_OK ? "RUNNING" : esp_err_to_name(start_err));
  if (a & TWAI_ALERT_ERR_PASS) log_w("CAN: error passive (TEC=%lu REC=%lu)", tec, rec);
  if (a & TWAI_ALERT_ERR_ACTIVE) log_i("CAN: error active (TEC=%lu REC=%lu)", tec, rec);

  if (a & TWAI_ALERT_ABOVE_ERR_WARN) {
    if (due(now, s_warn_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_warn_log_ms = now ? now : 1u;
      log_w("CAN: error counter above warning level (TEC=%lu REC=%lu)", tec, rec);
    }
  }
  if (a & TWAI_ALERT_BUS_ERROR) {
    s_bus_err_pending++;
    if (due(now, s_bus_err_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_bus_err_log_ms = now ? now : 1u;
      log_w("CAN: %lu bus error alert(s) (driver bus_error_count=%lu TEC=%lu REC=%lu)", (unsigned long)s_bus_err_pending,
            (unsigned long)st.bus_error_count, tec, rec);
      s_bus_err_pending = 0;
    }
  }
  if (a & TWAI_ALERT_RX_QUEUE_FULL) {
    s_qfull_pending++;
    if (due(now, s_qfull_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_qfull_log_ms = now ? now : 1u;
      log_w("CAN: RX queue full x%lu, frames lost (driver rx_missed=%lu queued=%lu)", (unsigned long)s_qfull_pending,
            (unsigned long)st.rx_missed_count, (unsigned long)st.msgs_to_rx);
      s_qfull_pending = 0;
    }
  }
  if (a & TWAI_ALERT_RX_FIFO_OVERRUN) {
    s_overrun_pending++;
    if (due(now, s_overrun_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_overrun_log_ms = now ? now : 1u;
      log_w("CAN: RX FIFO overrun x%lu (driver rx_overrun=%lu)", (unsigned long)s_overrun_pending,
            (unsigned long)st.rx_overrun_count);
      s_overrun_pending = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// Frames
// ---------------------------------------------------------------------------
static void handle_frame(const twai_message_t &m, uint32_t now) {
  // VESC status frames are 29-bit data frames. Standard-id or remote frames come
  // from some other node and are simply not ours (not counted as drops).
  if (!m.extd || m.rtr) return;
  const uint32_t pkt = VESC_EID_PACKET_ID(m.identifier);
  // Other VESC packet ids (PING/PONG, buffer fill, ...) are legitimate traffic
  // between VESCs or from VESC Tool: ignore silently.
  if (!vesc_is_status_pkt(pkt)) return;
  const int id = VESC_EID_CONTROLLER_ID(m.identifier);
  const int idx = vesc_status_index(pkt);

  if (m.data_length_code < 8) {  // a status id with a short payload is malformed: count it
    count_dropped();
    return;
  }

  // Controller id policy: VESC_CAN_ID >= 0 pins one id; -1 locks onto the first
  // id seen and counts every other id separately (a second VESC on the bus).
  int want = VESC_CAN_ID;
  bool newly_locked = false;
  vesc_telemetry_t t{};
  if (!state_lock()) return;  // lock timeout (counted by state_lock): drop this frame, the next one is 20 ms away
  if (want < 0) {
    if (g_state.vesc.locked_id < 0) {
      g_state.vesc.locked_id = id;
      newly_locked = true;
    }
    want = g_state.vesc.locked_id;
  } else if (g_state.vesc.locked_id < 0 && id == want) {
    g_state.vesc.locked_id = id;  // pinned id: record it once so consumers have a single "which VESC" field
    newly_locked = true;
  }
  if (id != want) {
    g_state.vesc.frames_other_id++;
    state_unlock();
    return;
  }
  t = g_state.vesc.t;  // local copy: decode outside the lock
  state_unlock();
  if (newly_locked) log_i("VESC: locked onto controller id %d", id);

  // Seed (rather than blend) the EMA on the first frame of this STATUS and after a
  // gap longer than the stale window: otherwise, after a cable came back, the
  // display would show a blend of the live value and one that is minutes old.
  const bool first_of_kind = t.t_ms[idx] == 0 || (uint32_t)(now - t.t_ms[idx]) > VESC_STALE_R1_MS;
  if (!vesc_decode_status(&t, m.identifier, true, m.data, m.data_length_code, now, want)) {
    count_dropped();  // cannot happen after the checks above; kept so a decoder change never passes silently
    return;
  }

#if CAN_LOG_RAW_FRAMES
  log_d("CAN rx eid=0x%08lX pkt=%lu id=%d dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]", (unsigned long)m.identifier,
        (unsigned long)pkt, id, (unsigned)m.data_length_code, m.data[0], m.data[1], m.data[2], m.data[3], m.data[4],
        m.data[5], m.data[6], m.data[7]);
#endif

  // Write back: decoded fields, EMA of the displayed quantities, counters.
  if (!state_lock()) return;
  VescState &v = g_state.vesc;
  v.t = t;
  switch (idx) {
    case VESC_IDX_STATUS_1: v.i_motor_ema = ema_step(v.i_motor_ema, t.current_motor, first_of_kind); v.ema_init = true; break;
    case VESC_IDX_STATUS_4: v.i_in_ema = ema_step(v.i_in_ema, t.current_in, first_of_kind); v.ema_init = true; break;
    case VESC_IDX_STATUS_5: v.v_in_ema = ema_step(v.v_in_ema, t.v_in, first_of_kind); v.ema_init = true; break;
    default: break;
  }
  v.frames_total++;
  v.last_frame_ms = now ? now : 1u;
  state_unlock();
}

// ---------------------------------------------------------------------------
// Periodic health snapshot (twai_get_status_info -> g_state.can)
// ---------------------------------------------------------------------------
static void snapshot_health(uint32_t now) {
  twai_status_info_t st{};
  const esp_err_t err = twai_get_status_info(&st);
  if (err != ESP_OK) {  // only ESP_ERR_INVALID_STATE = driver gone: fall back to the install loop
    s_installed = false;
    set_can_state(CAN_STATE_UNINSTALLED);
    log_e("CAN: twai_get_status_info failed: %s, reinstalling", esp_err_to_name(err));
    return;
  }
  const int cs = map_state(st.state);
  if (state_lock()) {
    CanHealth &c = g_state.can;
    c.state = cs;
    c.tec = st.tx_error_counter;
    c.rec = st.rx_error_counter;
    c.bus_error_count = st.bus_error_count;
    c.rx_missed = st.rx_missed_count;
    c.rx_overrun = st.rx_overrun_count;
    c.arb_lost = st.arb_lost_count;
    c.last_health_ms = now ? now : 1u;
    state_unlock();
  }

  const bool unhealthy = cs != CAN_STATE_RUNNING || st.tx_error_counter >= CAN_ERR_WARN_LEVEL ||
                         st.rx_error_counter >= CAN_ERR_WARN_LEVEL;
  if (unhealthy) {
    log_w("CAN: health state=%s TEC=%lu REC=%lu buserr=%lu missed=%lu overrun=%lu arb_lost=%lu queued=%lu",
          can_state_str(cs), (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
          (unsigned long)st.bus_error_count, (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count,
          (unsigned long)st.arb_lost_count, (unsigned long)st.msgs_to_rx);
  } else {
    log_d("CAN: health state=%s TEC=%lu REC=%lu buserr=%lu missed=%lu overrun=%lu", can_state_str(cs),
          (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter, (unsigned long)st.bus_error_count,
          (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count);
  }

  // Self-heal: STOPPED means a missed BUS_RECOVERED alert or a failed start;
  // BUS_OFF means the recovery kicked off by the alert handler did not take.
  // RUNNING is the only state in which a listener is useful, so try again.
  if (cs == CAN_STATE_STOPPED) {
    log_w("CAN: driver is STOPPED, restarting");
    can_run();
  } else if (cs == CAN_STATE_BUS_OFF) {
    const esp_err_t rerr = twai_initiate_recovery();
    if (rerr == ESP_OK) set_can_state(CAN_STATE_RECOVERING);
    log_w("CAN: driver is BUS_OFF, recovery %s", rerr == ESP_OK ? "restarted" : esp_err_to_name(rerr));
  }
}

// ---------------------------------------------------------------------------
// Task
// ---------------------------------------------------------------------------
static void canTask(void *) {
  for (;;) {
    hb_touch(hb_can);
    const uint32_t now = millis();

    if (!s_installed) {  // install failed at boot or the driver disappeared: retry, but keep the heartbeat alive
      if (due(now, s_last_install_try_ms, CAN_INSTALL_RETRY_MS)) {
        s_last_install_try_ms = now ? now : 1u;
        can_install();
      }
      vTaskDelay(pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));
      continue;
    }

    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts) handle_alerts(alerts, now);

    twai_message_t m{};
    const esp_err_t r = twai_receive(&m, pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));
    if (r == ESP_OK) {
      handle_frame(m, now_nonzero());
    } else if (r != ESP_ERR_TIMEOUT) {
      // Anything but a timeout means the driver is unusable. The header documents
      // ESP_ERR_INVALID_STATE for "not installed", but the IDF 5.5 handle-less
      // wrapper actually returns ESP_ERR_INVALID_ARG (NULL handle), and either
      // way the call returns immediately: without this delay the loop would spin
      // at priority 6 on the single core and starve the supervisor / WDT feed.
      if (!s_invalid_state_logged) {
        log_w("CAN: twai_receive failed: %s, recovering", esp_err_to_name(r));
        s_invalid_state_logged = true;
      }
      if (!can_run()) s_installed = false;  // -> periodic reinstall path above
      vTaskDelay(pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));
    }
    // ESP_ERR_TIMEOUT: idle bus, nothing to do.

    if (due(millis(), s_last_health_ms, CAN_HEALTH_LOG_MS)) {
      s_last_health_ms = now_nonzero();
      snapshot_health(s_last_health_ms);
    }
  }
}

bool can_vesc_start() {
  if (s_task_started) return s_installed;
  const bool ok = can_install();
  s_last_install_try_ms = now_nonzero();
  if (xTaskCreate(canTask, "canTask", CAN_TASK_STACK, nullptr, TASK_PRIO_CAN, nullptr) != pdPASS) {
    log_e("CAN: xTaskCreate(canTask) failed");
    return false;
  }
  s_task_started = true;
  return ok;
}

// ---------------------------------------------------------------------------
// Log summary (called from the supervisor on a snapshot)
// ---------------------------------------------------------------------------
static void fmt_age(char *buf, size_t n, uint32_t t_ms, uint32_t now) {
  if (t_ms == 0) snprintf(buf, n, "never");
  else snprintf(buf, n, "%lums", (unsigned long)(now - t_ms));  // unsigned subtraction: wrap-safe
}

// The motor NTC input floats when nothing is wired, so the VESC reports garbage
// (often -273 or > 200): show "n/a" outside a physically plausible range.
static void fmt_tmot(char *buf, size_t n, const vesc_telemetry_t &t) {
  const bool have = t.t_ms[VESC_IDX_STATUS_4] != 0;
  if (!have || t.temp_motor < -40.0f || t.temp_motor > 200.0f) snprintf(buf, n, "n/a");
  else snprintf(buf, n, "%.1fC", t.temp_motor);
}

void can_vesc_log_summary(const SharedState &s, uint32_t now) {
  const vesc_telemetry_t &t = s.vesc.t;
  char age[VESC_STATUS_COUNT][24];
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) {
    // STATUS 1/4/5 are "Rate 1" (VESC_STALE_R1_MS), STATUS 2/3/6 are "Rate 2" (VESC_STALE_R2_MS)
    const bool r1 = (i == VESC_IDX_STATUS_1 || i == VESC_IDX_STATUS_4 || i == VESC_IDX_STATUS_5);
    const uint32_t stale_ms = r1 ? VESC_STALE_R1_MS : VESC_STALE_R2_MS;
    fmt_age(age[i], sizeof age[i], t.t_ms[i], now);
    if (t.t_ms[i] != 0 && (uint32_t)(now - t.t_ms[i]) > stale_ms) strlcat(age[i], "(stale)", sizeof age[i]);
  }
  char tmot[16];
  fmt_tmot(tmot, sizeof tmot, t);

  char line[640];  // worst case (all counters saturated, all ages stale) is ~620 bytes; snprintf truncates safely
  snprintf(line, sizeof line,
           "VESC id=%d frames=%lu other=%lu | erpm=%.0f rpm=%.0f duty=%.3f Im=%.1fA(%.1f) | "
           "Iin=%.1fA(%.1f) Vin=%.1fV(%.1f) P=%.0fW | Tfet=%.1fC Tmot=%s | pid=%.1f | "
           "Ah=%.4f/%.4f Wh=%.4f/%.4f | tacho=%ld revs=%.1f | adc=%.3f/%.3f/%.3f ppm=%.3f | "
           "age s1=%s s2=%s s3=%s s4=%s s5=%s s6=%s | "
           "can=%s tec=%lu rec=%lu buserr=%lu missed=%lu ovr=%lu busoff=%lu qfull=%lu",
           s.vesc.locked_id, (unsigned long)s.vesc.frames_total, (unsigned long)s.vesc.frames_other_id,
           t.erpm, vesc_mech_rpm(&t, VESC_MOTOR_POLES), t.duty, t.current_motor, s.vesc.i_motor_ema,
           t.current_in, s.vesc.i_in_ema, t.v_in, s.vesc.v_in_ema, vesc_power_w(&t),
           t.temp_fet, tmot, t.pid_pos,
           t.amp_hours, t.amp_hours_charged, t.watt_hours, t.watt_hours_charged,
           (long)t.tachometer, vesc_revolutions(&t, VESC_MOTOR_POLES),
           t.adc1, t.adc2, t.adc3, t.ppm,
           age[0], age[1], age[2], age[3], age[4], age[5],
           can_state_str(s.can.state), (unsigned long)s.can.tec, (unsigned long)s.can.rec,
           (unsigned long)s.can.bus_error_count, (unsigned long)s.can.rx_missed, (unsigned long)s.can.rx_overrun,
           (unsigned long)s.can.bus_off_count, (unsigned long)s.can.queue_full_events);
  log_i("%s", line);
}
