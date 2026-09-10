// vesc-dashboard: ESP32-C6 (DFRobot FireBeetle 2) VESC CAN monitor + SSD1309 OLED dashboard + u-blox GNSS speed
//
// app_main(): boot banner, shared state, task watchdog, start the worker tasks, then run
// the supervisor loop forever on the main task (priority 1): it feeds the task watchdog
// only while every worker heartbeat is fresh, blinks the LED and prints the periodic
// telemetry / GNSS / trip / system log lines. It never touches the peripherals.
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "can_vesc.h"
#include "config.h"
#include "display.h"
#include "gnss_ubx.h"
#include "shared_state.h"
#include "bms_ble.h"
#include "trip.h"

static const char *TAG = "main";

// Log tags of this project. Their runtime level is raised to the compile-time level of the
// project sources (LOG_LOCAL_LEVEL; platformio.ini may set -DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG),
// so the debug lines can be switched on without touching the IDF components' own tags.
static const char *const kLogTags[] = {"main", "can", "gnss", "oled", "trip", "bms"};

[[maybe_unused]] static const char *resetReasonStr(esp_reset_reason_t r)
{ // log-only
  switch (r)
  {
  case ESP_RST_POWERON:
    return "POWERON";
  case ESP_RST_EXT:
    return "EXT";
  case ESP_RST_SW:
    return "SW";
  case ESP_RST_PANIC:
    return "PANIC";
  case ESP_RST_INT_WDT:
    return "INT_WDT";
  case ESP_RST_TASK_WDT:
    return "TASK_WDT";
  case ESP_RST_WDT:
    return "WDT";
  case ESP_RST_DEEPSLEEP:
    return "DEEPSLEEP";
  case ESP_RST_BROWNOUT:
    return "BROWNOUT";
  case ESP_RST_SDIO:
    return "SDIO";
  default:
    return "UNKNOWN";
  }
}

static void logConfig()
{
  ESP_LOGI(TAG, "cfg CAN : tx=%d rx=%d %d kbit/s mode=%s rxq=%d vesc_id=%d poles=%d poll=%d ms own_id=%d", PIN_CAN_TX,
           PIN_CAN_RX, CAN_BITRATE_KBPS,
           CAN_LISTEN_ONLY ? "LISTEN_ONLY" : (VESC_POLL_MS > 0 ? "NORMAL(ack+poll)" : "NORMAL(ack-only)"),
           CAN_RX_QUEUE_LEN, VESC_CAN_ID, VESC_MOTOR_POLES, VESC_POLL_MS, CAN_OWN_ID);
  ESP_LOGI(TAG, "cfg OLED: sda=%d scl=%d rst=%d addr=0x%02X i2c=%lu Hz %dx%d rot=%d period=%d ms dim_after=%lu ms",
           PIN_OLED_SDA, PIN_OLED_SCL, PIN_OLED_RST, (unsigned)OLED_I2C_ADDR, (unsigned long)OLED_I2C_HZ, OLED_WIDTH,
           OLED_HEIGHT, OLED_ROTATION, OLED_PERIOD_MS, (unsigned long)OLED_IDLE_DIM_MS);
  ESP_LOGI(TAG, "cfg GNSS: rx=%d tx=%d rate=%d ms unit=%s rxbuf=%d", PIN_GNSS_RX, PIN_GNSS_TX, GNSS_RATE_MS,
           SPEED_UNIT_STR, GNSS_RX_BUFFER);
#if BMS_BLE_ENABLE
  ESP_LOGI(TAG, "cfg BMS : ble=1 addr=%s name=%s* proto=%d cells<=%d sign=%s stale=%d ms prio=%d", BMS_BLE_ADDR[0] ? BMS_BLE_ADDR : "(scan)",
           BMS_BLE_NAME_PREFIX, BMS_PROTOCOL, BMS_CELLS_MAX, BMS_CURRENT_SIGN ? "dis+" : "chg+", BMS_STALE_MS, BMS_TASK_PRIO);
#else
  ESP_LOGI(TAG, "cfg BMS : ble=0 (build -e bms to enable)");
#endif
}

static inline void led_set(bool on) { gpio_set_level((gpio_num_t)PIN_LED, on ? 1 : 0); }

static void setup()
{
  // The console is the USB-Serial/JTAG port (sdkconfig): nothing to open, and its output is
  // dropped rather than blocked while no host is attached. The delay gives a host time to
  // re-enumerate after the reset so the boot banner shows up in the monitor.
  vTaskDelay(pdMS_TO_TICKS(SERIAL_BOOT_DELAY_MS));
  for (const char *tag : kLogTags)
    esp_log_level_set(tag, (esp_log_level_t)LOG_LOCAL_LEVEL);

  gpio_config_t led = {};
  led.pin_bit_mask = 1ULL << PIN_LED;
  led.mode = GPIO_MODE_OUTPUT;
  gpio_config(&led);
  led_set(false);

  ESP_LOGI(TAG, "vesc-dashboard %s (built %s %s) reset=%s", FW_VERSION, __DATE__, __TIME__,
           resetReasonStr(esp_reset_reason()));
  logConfig();

  state_init();

  // Task watchdog: ESP-IDF starts the TWDT at boot (CONFIG_ESP_TASK_WDT_INIT, idle task
  // subscribed). Reconfigure it to our timeout without the idle-task check, then subscribe the
  // main task: it feeds the TWDT only while all worker heartbeats are fresh, so a hung
  // CAN/GNSS/display task reboots the board (reset reason TASK_WDT).
  esp_task_wdt_config_t wdt = {};
  wdt.timeout_ms = WDT_TIMEOUT_MS;
  wdt.idle_core_mask = 0;
  wdt.trigger_panic = true;
  esp_err_t err = esp_task_wdt_reconfigure(&wdt);
  if (err != ESP_OK)
    err = esp_task_wdt_init(&wdt); // in case the TWDT was not initialised at boot
  if (err != ESP_OK)
    ESP_LOGE(TAG, "task WDT setup failed: %s", esp_err_to_name(err));
  err = esp_task_wdt_add(NULL);
  if (err != ESP_OK)
    ESP_LOGE(TAG, "task WDT add failed: %s", esp_err_to_name(err));

  // BLE first: the first controller enable may store the PHY calibration blob in NVS, and the TWAI
  // ISR is not cache-safe, so the CAN node must not exist yet while that flash write happens.
  [[maybe_unused]] const bool bmsOk = bms_ble_start();
  [[maybe_unused]] const bool canOk = can_vesc_start(); // log-only
  [[maybe_unused]] const bool gnssOk = gnss_start();
  [[maybe_unused]] const bool dispOk = display_start();
  [[maybe_unused]] const bool tripOk = trip_start();
  ESP_LOGI(TAG, "started: can=%d gnss=%d display=%d trip=%d bms=%d", canOk, gnssOk, dispOk, tripOk, bmsOk);
}

// A heartbeat that was never touched counts as fresh during the first max_age ms
// after boot (tasks may still be starting); afterwards it must be alive.
static bool hbOk(uint32_t hb, uint32_t now, uint32_t max_age)
{
  if (hb == 0)
    return now < max_age;
  // Signed age: a worker task can stamp its heartbeat between our clock read and
  // this comparison (every worker outranks the main task and wakes on the same tick
  // that advances the clock), so now - hb can be -1. Unsigned that is ~4e9 ms and
  // would raise a false "heartbeat stale" alarm and skip one WDT feed.
  return (int32_t)(now - hb) <= (int32_t)max_age;
}

static void loop()
{
  static uint32_t nextVescLog = 0, nextGnssLog = 0, nextTripLog = 0, nextBmsLog = 0, nextSysLog = 0, nextStaleLog = 0, nextBlink = 0;
  static bool led = false;

  // Snapshot BEFORE reading the clock: every timestamp inside s must be <= now,
  // otherwise a frame that lands between the two reads shows up as an age of
  // 4294967295ms in the VESC/GNSS log line and as a stale LED for one cycle.
  const SharedState s = state_snapshot();
  const uint32_t now = state_now_ms();

  // ---- watchdog supervision -------------------------------------------------
  const bool canAlive = hbOk(hb_can, now, HB_MAX_CAN_MS);
  const bool gnssAlive = hbOk(hb_gnss, now, HB_MAX_GNSS_MS);
  const bool dispAlive = hbOk(hb_disp, now, HB_MAX_DISP_MS);
  const bool bmsAlive = (HB_MAX_BMS_MS > 0) ? hbOk(hb_bms, now, HB_MAX_BMS_MS) : true; // 0 = not gated
  if (canAlive && gnssAlive && dispAlive && bmsAlive)
  {
    esp_task_wdt_reset();
  }
  else if ((int32_t)(now - nextStaleLog) >= 0)
  {
    nextStaleLog = now + 1000;
    ESP_LOGE(TAG, "heartbeat stale: can=%lu ms gnss=%lu ms disp=%lu ms (reboot in <= %lu ms)",
             (unsigned long)age_ms(hb_can, now), (unsigned long)age_ms(hb_gnss, now),
             (unsigned long)age_ms(hb_disp, now), (unsigned long)WDT_TIMEOUT_MS);
  }

  // ---- LED: 1 Hz blink when VESC data is fresh, 0.2 Hz otherwise -------------
  const bool vescFresh = vesc_fresh(&s.vesc.t, VESC_IDX_STATUS_1, now, VESC_STALE_R1_MS) ||
                         vesc_fresh(&s.vesc.t, VESC_IDX_STATUS_5, now, VESC_STALE_R1_MS);
  if ((int32_t)(now - nextBlink) >= 0)
  {
    nextBlink = now + (vescFresh ? 500 : 2500);
    led = !led;
    led_set(led);
  }

  // ---- periodic logs (deadline based, never "now % period") ------------------
  if (LOG_VESC_MS && (int32_t)(now - nextVescLog) >= 0)
  {
    nextVescLog = now + LOG_VESC_MS;
    can_vesc_log_summary(s, now);
  }
  if (LOG_GNSS_MS && (int32_t)(now - nextGnssLog) >= 0)
  {
    nextGnssLog = now + LOG_GNSS_MS;
    gnss_log_summary(s, now);
  }
  if (LOG_TRIP_MS && (int32_t)(now - nextTripLog) >= 0)
  {
    nextTripLog = now + LOG_TRIP_MS;
    trip_log_summary(s, now);
  }
  if (LOG_BMS_MS && BMS_BLE_ENABLE && (int32_t)(now - nextBmsLog) >= 0)
  {
    nextBmsLog = now + LOG_BMS_MS;
    bms_log_summary(s, now);
  }
  if (LOG_SYS_MS && (int32_t)(now - nextSysLog) >= 0)
  {
    nextSysLog = now + LOG_SYS_MS;
#if BMS_BLE_ENABLE
    const char *bmsLink = bms_link_str(s.bms.link);
    const uint32_t bmsFrames = s.bms.frames_ok;
#else
    const char *bmsLink = "OFF";
    const uint32_t bmsFrames = 0;
#endif
    ESP_LOGI(TAG,
             "SYS up=%lus reset=%s heap=%lu minheap=%lu lockfail=%lu | can=%s tec=%lu rec=%lu busoff=%lu | disp ok=%d "
             "refreshes=%lu skipped=%lu dim=%d scr=%u btn=%lu | poll sent=%lu ok=%lu bad=%lu to=%lu | hb can=%lu "
             "gnss=%lu disp=%lu | bms=%s frm=%lu hb=%lu",
             (unsigned long)(now / 1000), resetReasonStr(esp_reset_reason()), (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size(), (unsigned long)s.lock_failures, can_state_str(s.can.state),
             (unsigned long)s.can.tec, (unsigned long)s.can.rec, (unsigned long)s.can.bus_off_count, s.disp.init_ok,
             (unsigned long)s.disp.refreshes, (unsigned long)s.disp.skipped_unchanged, s.disp.dimmed,
             (unsigned)s.disp.screen, (unsigned long)s.disp.button_presses, (unsigned long)s.vesc_ext.polls_sent,
             (unsigned long)s.vesc_ext.replies_ok, (unsigned long)s.vesc_ext.replies_bad,
             (unsigned long)s.vesc_ext.timeouts, (unsigned long)age_ms(hb_can, now),
             (unsigned long)age_ms(hb_gnss, now), (unsigned long)age_ms(hb_disp, now), bmsLink, (unsigned long)bmsFrames,
             (unsigned long)(BMS_BLE_ENABLE ? age_ms(hb_bms, now) : 0)); // 0 = no BMS task in this build
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}

extern "C" void app_main()
{
  setup();
  for (;;)
    loop();
}
