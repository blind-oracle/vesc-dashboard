// Dead-man's switch: the cut line, the task and the closed loop.
//
// Three contexts touch this file:
//   * the NimBLE host task (prio 21) calls deadman_beacon_seen() / deadman_adv_other()
//     from gap_cb. Volatile stores only: no lock, no log, no allocation;
//   * the display task (prio 2) calls deadman_request_reset();
//   * deadmanTask (TASK_PRIO_DEADMAN, above CAN) owns everything else. It runs the pure
//     state machine from deadman_logic.h, drives PIN_DEADMAN_CUT and publishes
//     g_state.deadman. It NEVER logs: the USB-Serial/JTAG console can block when a host
//     is attached, and a priority-7 task that blocks misses its tick. All logging is
//     done by the supervisor through deadman_log_events() / deadman_log_summary().
//
// The pin is open drain and wired in PARALLEL with the boat's mechanical kill switch on
// the VESC's ADC2 input, so this firmware can only ever ADD a kill: high-Z lets the
// mechanical switch drive the line, low pulls it under the VESC's 1.65 V threshold. A
// dead, hung or resetting dashboard therefore leaves the motor exactly as the mechanical
// switch has it - the feature can never strand the boat by failing (fail-passive). The
// price, stated so nobody has to rediscover it: a hung dashboard silently removes the
// BLE layer, and a reboot releases a latched cut.
#include "deadman.h"

#if DEADMAN_ENABLE

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>
#include <string.h>

#include "ble_addr.h"
#include "bms_ble.h"
#include "deadman_logic.h"

static const char *TAG = "dm";

// bmsTask touches hb_bms every loop (100 ms); allow a few missed ones before calling the
// radio unhealthy. Shorter than HB_MAX_BMS_MS on purpose: the dead-man wants to know the
// scan stopped long before the supervisor would consider rebooting for it.
static constexpr uint32_t BLE_HB_MAX_MS = 1000;

static_assert(DEADMAN_TICK_MS >= 5 && DEADMAN_TICK_MS <= 100, "DEADMAN_TICK_MS must be 5..100");
static_assert(sizeof(DEADMAN_BEACON_ADDR) == 18,
              "DEADMAN_BEACON_ADDR must be \"aa:bb:cc:dd:ee:ff\" - tag 1's address is mandatory");
static_assert(sizeof(DEADMAN_BEACON_ADDR2) == 18 || sizeof(DEADMAN_BEACON_ADDR2) == 1,
              "DEADMAN_BEACON_ADDR2 must be \"aa:bb:cc:dd:ee:ff\" or \"\" (single-tag boat)");
static_assert(DEADMAN_TAGS == 2, "the tag slots, the log line and the screens are written for exactly two");
static_assert(DEADMAN_ENROL_MS >= 1000 && DEADMAN_ENROL_MS <= 60000,
              "DEADMAN_ENROL_MS must be 1..60 s: it is how long a second tag has to join");
static_assert(HB_MAX_DEADMAN_MS > 5 * DEADMAN_TICK_MS, "HB_MAX_DEADMAN_MS must allow a few missed ticks of jitter");
static_assert(DEADMAN_TIMEOUT_MS >= 1000 && DEADMAN_TIMEOUT_MS <= 60000, "DEADMAN_TIMEOUT_MS must be 1..60 s");

// ---------------------------------------------------------------- host-task -> task
// Single writer (the NimBLE host task), single reader (deadmanTask). Plain volatiles:
// nothing outside deadmanTask derives state from them, and a one-tick-late read is fine.
static volatile uint32_t s_beacon_t_ms[DEADMAN_TAGS] = {0};
static volatile uint32_t s_beacon_reports[DEADMAN_TAGS] = {0};
static volatile int8_t s_beacon_rssi[DEADMAN_TAGS] = {0};
static volatile uint32_t s_adv_other = 0;
static volatile bool s_reset_req = false;

// Which slots have a usable address. Written once in deadman_start(), read-only after.
static bool s_tag_cfg[DEADMAN_TAGS];
static const char *const kTagAddr[DEADMAN_TAGS] = {DEADMAN_BEACON_ADDR, DEADMAN_BEACON_ADDR2};

void deadman_beacon_seen(uint8_t tag, int8_t rssi, uint32_t now_ms) {
  if (tag >= DEADMAN_TAGS) return;
  s_beacon_rssi[tag] = rssi;
  s_beacon_t_ms[tag] = now_ms ? now_ms : 1u;  // 0 means "never"
  s_beacon_reports[tag] = s_beacon_reports[tag] + 1;
}

void deadman_adv_other() { s_adv_other = s_adv_other + 1; }

void deadman_request_reset() { s_reset_req = true; }

// ---------------------------------------------------------------- the cut line
static bool s_gpio_ready = false;

// true = pull ADC2 low (kill), false = high-Z (permit; the mechanical switch owns the line).
static inline void cut_write(bool cut) {
  if (s_gpio_ready) gpio_set_level((gpio_num_t)PIN_DEADMAN_CUT, cut ? 0 : 1);
}

void deadman_gpio_early_init() {
#if PIN_DEADMAN_CUT >= 0
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << PIN_DEADMAN_CUT;
  io.mode = GPIO_MODE_OUTPUT_OD;  // open drain: we can only ever pull the line DOWN
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  if (gpio_config(&io) != ESP_OK) return;
  s_gpio_ready = true;
  // Never gpio_hold_en() this pin: the hold would survive a reset and latch a cut that
  // no running code owns.
  gpio_set_level((gpio_num_t)PIN_DEADMAN_CUT, DEADMAN_BOOT_CUT ? 0 : 1);
#endif
}

// ---------------------------------------------------------------- task state
static DeadmanCalc s_calc;
static DeadmanState s_pub;   // mirror of g_state.deadman, published as one copy
static bool s_disabled;      // start-up refused (bad address / no GPIO): assert the cut and say so
static uint8_t s_vesc_status;  // cached one tick, so the loop takes the mutex only once
static bool s_poll_fresh;

// The watchdog is only meaningful while the radio is actually scanning for the tag.
// hb_bms covers "the BLE task is alive"; bms_ble_scanning() covers "a scan is running".
static bool ble_healthy(uint32_t now) {
  if (!bms_ble_scanning()) return false;
  const uint32_t hb = hb_bms;
  if (hb == 0) return false;  // the BLE task has never run
  return (int32_t)(now - hb) <= (int32_t)BLE_HB_MAX_MS;
}

static void publish_and_sample(uint32_t now) {
  s_pub.state = s_calc.state;
  s_pub.confirm = s_calc.confirm;
  s_pub.cut = s_calc.cut;
  s_pub.state_since_ms = s_calc.state_since_ms;
  s_pub.enrol_open = s_calc.enrol_open;
  s_pub.enrolled_count = (uint8_t)deadman_enrolled_count(s_calc);
  s_pub.gap_max_ms = 0;
  for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
    s_pub.tags[i].t_ms = s_beacon_t_ms[i];
    s_pub.tags[i].reports = s_beacon_reports[i];
    s_pub.tags[i].rssi = s_beacon_rssi[i];
    s_pub.tags[i].gap_max_ms = s_calc.tag[i].gap_max_ms;
    s_pub.tags[i].configured = s_tag_cfg[i];
    s_pub.tags[i].enrolled = s_calc.tag[i].enrolled;
    // The published aggregate is the WORST enrolled tag's own gap, never the union's:
    // that is the conservative number README section 5b tells you to size the timeout from.
    if (s_calc.tag[i].enrolled && s_calc.tag[i].gap_max_ms > s_pub.gap_max_ms)
      s_pub.gap_max_ms = s_calc.tag[i].gap_max_ms;
  }
  s_pub.adv_other = s_adv_other;
  s_pub.trips = s_calc.trips;
  s_pub.resets = s_calc.resets;
  s_pub.resets_refused = s_calc.resets_refused;

  // One short critical section: publish this tick's result and take the VESC fields the
  // closed loop needs next tick. Never blocks - this task outranks every other writer.
  if (state_try_lock()) {
    g_state.deadman = s_pub;
    s_vesc_status = g_state.vesc_ext.status;
    s_poll_fresh = fresh(g_state.vesc_ext.t_ms, now, VESC_EXT_STALE_MS);
    state_unlock();
  } else {
    s_pub.publish_skipped++;
  }
}

static void deadman_task(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    hb_touch(hb_deadman);
    const uint32_t now = state_now_ms();

    DeadmanInputs in = {};
    for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
      in.tag[i].t_ms = s_beacon_t_ms[i];
      in.tag[i].reports = s_beacon_reports[i];
      in.tag[i].rssi = s_beacon_rssi[i];
      in.tag[i].configured = s_tag_cfg[i];
    }
    in.ble_healthy = ble_healthy(now);
    in.poll_fresh = s_poll_fresh;
    in.vesc_status = s_vesc_status;
    if (s_reset_req) {
      in.reset_req = true;
      s_reset_req = false;  // consumed: the gesture is an edge, not a level
    }

    if (s_disabled) {
      s_calc.state = DM_FAULT;  // start-up refused: fail closed and keep saying so
      s_calc.cut = true;
    } else {
      deadman_update(s_calc, in, now);
    }
    cut_write(s_calc.cut);
    publish_and_sample(now);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(DEADMAN_TICK_MS));
  }
}

// ---------------------------------------------------------------- public API
bool deadman_start() {
  memset(&s_pub, 0, sizeof s_pub);
  deadman_init(s_calc);
  s_disabled = false;

#if PIN_DEADMAN_CUT < 0
  ESP_LOGW(TAG, "PIN_DEADMAN_CUT -1: the logic runs and the screens work, but NOTHING is actuated (soak-test build)");
#else
  if (!s_gpio_ready) {
    deadman_gpio_early_init();  // setup() should have done this already; do not run without it
    if (!s_gpio_ready) {
      s_disabled = true;
      ESP_LOGE(TAG, "GPIO%d setup failed: asserting the cut permanently", (int)PIN_DEADMAN_CUT);
    }
  }
#endif

  // Tag 1 is mandatory; tag 2 is optional but, if given, must be valid AND different.
  // A typo must never silently disable a tag, so a malformed non-empty address is fatal.
  uint8_t addr[DEADMAN_TAGS][6];
  for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
    s_tag_cfg[i] = false;
    if (kTagAddr[i][0] == '\0') {
      if (i == 0) {
        s_disabled = true;
        ESP_LOGE(TAG, "DEADMAN_BEACON_ADDR is empty: asserting the cut permanently");
      } else {
        ESP_LOGI(TAG, "tag %u not configured: single-tag boat", i + 1u);
      }
      continue;
    }
    if (!ble_addr_parse(kTagAddr[i], addr[i])) {
      s_disabled = true;
      ESP_LOGE(TAG, "tag %u address \"%s\" is not aa:bb:cc:dd:ee:ff: asserting the cut permanently", i + 1u,
               kTagAddr[i]);
      continue;
    }
    s_tag_cfg[i] = true;
  }
  if (s_tag_cfg[0] && s_tag_cfg[1] && ble_addr_equal(addr[0], addr[1])) {
    // Both slots pointing at one fob would look armed while guarding half as much.
    s_disabled = true;
    ESP_LOGE(TAG, "both tag addresses are %s: they must differ - asserting the cut permanently", kTagAddr[0]);
  }

  if (s_disabled) cut_write(true);

  if (xTaskCreate(deadman_task, "deadman", DEADMAN_TASK_STACK, nullptr, TASK_PRIO_DEADMAN, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "task create failed: asserting the cut permanently");
    cut_write(true);
    return false;
  }
  ESP_LOGI(TAG,
           "dead-man started: tag1=%s tag2=%s timeout %u ms warn %u ms arm %ux/%ums rssi>=%d enrol %u ms "
           "pin=%d (open drain, %s at boot)",
           s_tag_cfg[0] ? kTagAddr[0] : "-", s_tag_cfg[1] ? kTagAddr[1] : "(none)", (unsigned)DEADMAN_TIMEOUT_MS,
           (unsigned)DEADMAN_WARN_MS, (unsigned)DEADMAN_ARM_REPORTS, (unsigned)DEADMAN_ARM_WINDOW_MS,
           (int)DEADMAN_ARM_RSSI, (unsigned)DEADMAN_ENROL_MS, (int)PIN_DEADMAN_CUT,
           DEADMAN_BOOT_CUT ? "CUT" : "permit");
  return !s_disabled;
}

// ---------------------------------------------------------------- logging (supervisor)
// One compact "1:-62/18431 2:off" style description of the tag slots, for the log lines.
static void tags_str(const DeadmanState &d, uint32_t now, char *out, size_t n) {
  out[0] = '\0';
  size_t used = 0;
  for (unsigned i = 0; i < DEADMAN_TAGS && used + 1 < n; ++i) {
    if (!d.tags[i].configured) continue;
    char one[72];
    if (!d.tags[i].enrolled) {
      snprintf(one, sizeof one, "%st%u=%s(not enrolled)", used ? " " : "", i + 1u, kTagAddr[i]);
    } else if (d.tags[i].t_ms == 0) {
      snprintf(one, sizeof one, "%st%u=%s never", used ? " " : "", i + 1u, kTagAddr[i]);
    } else {
      snprintf(one, sizeof one, "%st%u=%s last=%lums rssi=%d n=%lu gapmax=%lums", used ? " " : "", i + 1u,
               kTagAddr[i], (unsigned long)age_ms(d.tags[i].t_ms, now), (int)d.tags[i].rssi,
               (unsigned long)d.tags[i].reports, (unsigned long)d.tags[i].gap_max_ms);
    }
    snprintf(out + used, n - used, "%s", one);
    used = strnlen(out, n);
  }
  if (out[0] == '\0') snprintf(out, n, "no tags configured");
}

void deadman_log_events(const SharedState &s, uint32_t now) {
  static uint8_t last_state = 0xFF;
  static uint8_t last_confirm = 0xFF;
  static uint32_t last_refused = 0, last_resets = 0;
  static bool last_enrol_open = false, enrol_reported = false;
  const DeadmanState &d = s.deadman;
  const uint32_t age = deadman_pub_age(d, now);
  char tags[192];

  if (d.state != last_state) {
    tags_str(d, now, tags, sizeof tags);
    switch (d.state) {
      case DM_ARMED:
        ESP_LOGW(TAG, "ARMED (%u tag%s): %s, %lu s after boot", (unsigned)d.enrolled_count,
                 d.enrolled_count == 1 ? "" : "s", tags, (unsigned long)(now / 1000u));
        break;
      case DM_GRACE:
        ESP_LOGW(TAG, "GRACE: every enrolled tag quiet for %lu ms, cutting in %lu ms", (unsigned long)age,
                 (unsigned long)((uint32_t)DEADMAN_TIMEOUT_MS - age));
        break;
      case DM_TRIPPED:
        ESP_LOGE(TAG,
                 "TRIPPED: every enrolled tag quiet for %lu ms (timeout %u, worst gap while enrolled %lu ms) - "
                 "motor CUT and latched; long press with a tag present to reset. %s",
                 (unsigned long)age, (unsigned)DEADMAN_TIMEOUT_MS, (unsigned long)d.gap_max_ms, tags);
        break;
      case DM_UNAVAILABLE:
        ESP_LOGE(TAG, "NO RADIO: the BLE scan is not running - the dead-man is NOT protecting anything "
                      "(the mechanical kill switch still is). Not cutting: our own failure must not stop the boat");
        break;
      case DM_FAULT:
        ESP_LOGE(TAG, "FAULT: cut asserted but the VESC never reported its kill switch active within %u ms while "
                      "polls were arriving - check PIN_DEADMAN_CUT wiring, the series resistor and the VESC's "
                      "ADC2 kill-switch setting", (unsigned)DEADMAN_CONFIRM_MS);
        break;
      case DM_WAIT_TAG:
        ESP_LOGW(TAG, "NO TAG: waiting for a tag to arm (%s). %s", 
                 DEADMAN_BOOT_CUT ? "motor cut until one appears" : "motor permitted - there is no protection yet",
                 tags);
        enrol_reported = false;  // a fresh enrolment round is coming
        break;
      default: break;
    }
    last_state = d.state;
  }

  // The enrolment window closing is the moment the set is frozen for the run: say which
  // tags are in and which are out, because nothing can change it afterwards.
  if (last_enrol_open && !d.enrol_open && !enrol_reported) {
    enrol_reported = true;
    tags_str(d, now, tags, sizeof tags);
    ESP_LOGW(TAG, "enrolment closed: %u tag%s guarding the motor. %s", (unsigned)d.enrolled_count,
             d.enrolled_count == 1 ? "" : "s", tags);
  }
  last_enrol_open = d.enrol_open;

  if (d.confirm != last_confirm) {
    if (d.confirm == DM_CONFIRM_OK) ESP_LOGW(TAG, "cut confirmed: the VESC reports its kill switch active");
    else if (d.confirm == DM_CONFIRM_STALE) ESP_LOGW(TAG, "cut asserted but no fresh VESC poll: cannot confirm");
    last_confirm = d.confirm;
  }
  if (d.resets != last_resets) {
    ESP_LOGW(TAG, "reset accepted: re-armed (freshest enrolled tag %lu ms old)", (unsigned long)age);
    last_resets = d.resets;
  }
  if (d.resets_refused != last_refused) {
    ESP_LOGE(TAG, "reset REFUSED: no enrolled tag seen for %lu ms (needs %u) - the motor stays cut",
             (unsigned long)age, (unsigned)DEADMAN_RESET_FRESH_MS);
    last_refused = d.resets_refused;
  }
}

void deadman_log_summary(const SharedState &s, uint32_t now) {
  const DeadmanState &d = s.deadman;
  const uint32_t since_s = d.state_since_ms ? (uint32_t)(now - d.state_since_ms) / 1000u : 0u;
  const uint32_t age = deadman_pub_age(d, now);
  const uint32_t margin = (d.state == DM_ARMED || d.state == DM_GRACE) && age < (uint32_t)DEADMAN_TIMEOUT_MS
                              ? (uint32_t)DEADMAN_TIMEOUT_MS - age
                              : 0u;
  char tags[192];
  tags_str(d, now, tags, sizeof tags);
  ESP_LOGI(TAG,
           "DM state=%s since=%lus out=%s enrol=%s/%u gapmax=%lums margin=%lums trips=%lu reset=%lu/%lu "
           "confirm=%s other=%lu skip=%lu | %s",
           deadman_state_str(d.state), (unsigned long)since_s, d.cut ? "CUT" : "permit",
           d.enrol_open ? "open" : "closed", (unsigned)d.enrolled_count, (unsigned long)d.gap_max_ms,
           (unsigned long)margin, (unsigned long)d.trips, (unsigned long)d.resets, (unsigned long)d.resets_refused,
           deadman_confirm_str(d.confirm), (unsigned long)d.adv_other, (unsigned long)d.publish_skipped, tags);
}

#endif  // DEADMAN_ENABLE
