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
              "DEADMAN_BEACON_ADDR must be \"aa:bb:cc:dd:ee:ff\" - the tag address is mandatory");
static_assert(HB_MAX_DEADMAN_MS > 5 * DEADMAN_TICK_MS, "HB_MAX_DEADMAN_MS must allow a few missed ticks of jitter");
static_assert(DEADMAN_TIMEOUT_MS >= 1000 && DEADMAN_TIMEOUT_MS <= 60000, "DEADMAN_TIMEOUT_MS must be 1..60 s");

// ---------------------------------------------------------------- host-task -> task
// Single writer (the NimBLE host task), single reader (deadmanTask). Plain volatiles:
// nothing outside deadmanTask derives state from them, and a one-tick-late read is fine.
static volatile uint32_t s_beacon_t_ms = 0;
static volatile uint32_t s_beacon_reports = 0;
static volatile int8_t s_beacon_rssi = 0;
static volatile uint32_t s_adv_other = 0;
static volatile bool s_reset_req = false;

void deadman_beacon_seen(int8_t rssi, uint32_t now_ms) {
  s_beacon_rssi = rssi;
  s_beacon_t_ms = now_ms ? now_ms : 1u;  // 0 means "never"
  s_beacon_reports = s_beacon_reports + 1;
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
  s_pub.beacon_t_ms = s_beacon_t_ms;
  s_pub.beacon_rssi = s_beacon_rssi;
  s_pub.beacon_reports = s_beacon_reports;
  s_pub.adv_other = s_adv_other;
  s_pub.gap_max_ms = s_calc.gap_max_ms;
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
    in.beacon_t_ms = s_beacon_t_ms;
    in.beacon_reports = s_beacon_reports;
    in.beacon_rssi = s_beacon_rssi;
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

  uint8_t addr[6];
  if (!ble_addr_parse(DEADMAN_BEACON_ADDR, addr)) {
    s_disabled = true;
    ESP_LOGE(TAG, "DEADMAN_BEACON_ADDR \"%s\" is not aa:bb:cc:dd:ee:ff: asserting the cut permanently",
             DEADMAN_BEACON_ADDR);
  }

  if (s_disabled) cut_write(true);

  if (xTaskCreate(deadman_task, "deadman", DEADMAN_TASK_STACK, nullptr, TASK_PRIO_DEADMAN, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "task create failed: asserting the cut permanently");
    cut_write(true);
    return false;
  }
  ESP_LOGI(TAG, "dead-man started: tag %s timeout %u ms warn %u ms arm %ux/%ums rssi>=%d pin=%d (open drain, %s at boot)",
           DEADMAN_BEACON_ADDR, (unsigned)DEADMAN_TIMEOUT_MS, (unsigned)DEADMAN_WARN_MS,
           (unsigned)DEADMAN_ARM_REPORTS, (unsigned)DEADMAN_ARM_WINDOW_MS, (int)DEADMAN_ARM_RSSI,
           (int)PIN_DEADMAN_CUT, DEADMAN_BOOT_CUT ? "CUT" : "permit");
  return !s_disabled;
}

// ---------------------------------------------------------------- logging (supervisor)
void deadman_log_events(const SharedState &s, uint32_t now) {
  static uint8_t last_state = 0xFF;
  static uint8_t last_confirm = 0xFF;
  static uint32_t last_refused = 0, last_resets = 0;
  const DeadmanState &d = s.deadman;

  if (d.state != last_state) {
    switch (d.state) {
      case DM_ARMED:
        ESP_LOGW(TAG, "ARMED: tag %s rssi %d, %lu s after boot", DEADMAN_BEACON_ADDR, (int)d.beacon_rssi,
                 (unsigned long)(now / 1000u));
        break;
      case DM_GRACE:
        ESP_LOGW(TAG, "GRACE: no tag for %lu ms, cutting in %lu ms", (unsigned long)age_ms(d.beacon_t_ms, now),
                 (unsigned long)((uint32_t)DEADMAN_TIMEOUT_MS - age_ms(d.beacon_t_ms, now)));
        break;
      case DM_TRIPPED:
        ESP_LOGE(TAG, "TRIPPED: no tag for %lu ms (timeout %u, worst gap while armed %lu ms) - motor CUT and latched; "
                      "long press on the DEADMAN screen with the tag present to reset",
                 (unsigned long)age_ms(d.beacon_t_ms, now), (unsigned)DEADMAN_TIMEOUT_MS,
                 (unsigned long)d.gap_max_ms);
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
        ESP_LOGW(TAG, "NO TAG: waiting for %s (%s)", DEADMAN_BEACON_ADDR,
                 DEADMAN_BOOT_CUT ? "motor cut until it appears" : "motor permitted - there is no protection yet");
        break;
      default: break;
    }
    last_state = d.state;
  }

  if (d.confirm != last_confirm) {
    if (d.confirm == DM_CONFIRM_OK) ESP_LOGW(TAG, "cut confirmed: the VESC reports its kill switch active");
    else if (d.confirm == DM_CONFIRM_STALE) ESP_LOGW(TAG, "cut asserted but no fresh VESC poll: cannot confirm");
    last_confirm = d.confirm;
  }
  if (d.resets != last_resets) {
    ESP_LOGW(TAG, "reset accepted: re-armed (tag %lu ms old)", (unsigned long)age_ms(d.beacon_t_ms, now));
    last_resets = d.resets;
  }
  if (d.resets_refused != last_refused) {
    ESP_LOGE(TAG, "reset REFUSED: the tag has not been seen for %lu ms (needs %u) - the motor stays cut",
             (unsigned long)age_ms(d.beacon_t_ms, now), (unsigned)DEADMAN_RESET_FRESH_MS);
    last_refused = d.resets_refused;
  }
}

void deadman_log_summary(const SharedState &s, uint32_t now) {
  const DeadmanState &d = s.deadman;
  const uint32_t since_s = d.state_since_ms ? (uint32_t)(now - d.state_since_ms) / 1000u : 0u;
  char age[16];
  if (d.beacon_t_ms) snprintf(age, sizeof age, "%lums", (unsigned long)age_ms(d.beacon_t_ms, now));
  else snprintf(age, sizeof age, "never");
  const uint32_t tag_age = age_ms(d.beacon_t_ms, now);
  const uint32_t margin = (d.state == DM_ARMED || d.state == DM_GRACE) && tag_age < (uint32_t)DEADMAN_TIMEOUT_MS
                              ? (uint32_t)DEADMAN_TIMEOUT_MS - tag_age
                              : 0u;
  ESP_LOGI(TAG,
           "DM state=%s since=%lus out=%s tag=%s last=%s rssi=%d n=%lu gapmax=%lums margin=%lums "
           "trips=%lu reset=%lu/%lu confirm=%s other=%lu skip=%lu",
           deadman_state_str(d.state), (unsigned long)since_s, d.cut ? "CUT" : "permit", DEADMAN_BEACON_ADDR, age,
           (int)d.beacon_rssi, (unsigned long)d.beacon_reports, (unsigned long)d.gap_max_ms, (unsigned long)margin,
           (unsigned long)d.trips, (unsigned long)d.resets, (unsigned long)d.resets_refused,
           deadman_confirm_str(d.confirm), (unsigned long)d.adv_other, (unsigned long)d.publish_skipped);
}

#endif  // DEADMAN_ENABLE
