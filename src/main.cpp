// boat-motor: ESP32-C6 (DFRobot FireBeetle 2) VESC CAN monitor + OLED/e-ink dashboard (DISPLAY_TYPE) + u-blox GNSS speed
//
// setup(): serial banner, shared state, task watchdog, start the three worker tasks.
// loop():  supervisor (Arduino loopTask, priority 1) - feeds the task watchdog only
//          while every worker heartbeat is fresh, blinks the LED, prints periodic
//          telemetry / GNSS / system log lines. It never touches the peripherals.
#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "can_vesc.h"
#include "config.h"
#include "display.h"
#include "gnss_ubx.h"
#include "trip.h"
#include "shared_state.h"

[[maybe_unused]] static const char *resetReasonStr(esp_reset_reason_t r) {  // log-only (unused below CORE_DEBUG_LEVEL 3)
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

static void logConfig() {
  log_i("cfg CAN : tx=%d rx=%d %d kbit/s mode=%s rxq=%d vesc_id=%d poles=%d poll=%d ms own_id=%d", PIN_CAN_TX,
        PIN_CAN_RX, CAN_BITRATE_KBPS, CAN_LISTEN_ONLY ? "LISTEN_ONLY" : (VESC_POLL_MS > 0 ? "NORMAL(ack+poll)" : "NORMAL(ack-only)"),
        CAN_RX_QUEUE_LEN, VESC_CAN_ID, VESC_MOTOR_POLES, VESC_POLL_MS, CAN_OWN_ID);
#if DISPLAY_TYPE == DISPLAY_TYPE_EPD_GDEY042T81
  log_i("cfg EPD : sck=%d mosi=%d cs=%d dc=%d rst=%d busy=%d spi=%lu Hz rot=%d fastfull=%d full_every=%d/%lu ms",
        PIN_EPD_SCK, PIN_EPD_MOSI, PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY, (unsigned long)EPD_SPI_HZ,
        EPD_ROTATION, EPD_FAST_FULL_UPDATE, EPD_FULL_EVERY_N_PARTIALS, (unsigned long)EPD_FULL_EVERY_MS);
#else
  log_i("cfg OLED: sda=%d scl=%d rst=%d addr=0x%02X i2c=%lu Hz %dx%d rot=%d period=%d ms dim_after=%lu ms",
        PIN_OLED_SDA, PIN_OLED_SCL, PIN_OLED_RST, (unsigned)OLED_I2C_ADDR, (unsigned long)OLED_I2C_HZ, OLED_WIDTH,
        OLED_HEIGHT, OLED_ROTATION, OLED_PERIOD_MS, (unsigned long)OLED_IDLE_DIM_MS);
#endif
  log_i("cfg GNSS: rx=%d tx=%d rate=%d ms unit=%s rxbuf=%d", PIN_GNSS_RX, PIN_GNSS_TX, GNSS_RATE_MS, SPEED_UNIT_STR,
        GNSS_RX_BUFFER);
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // USB-CDC: never block when no host is attached
  delay(SERIAL_BOOT_DELAY_MS);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  log_i("boat-motor %s (built %s %s) reset=%s", FW_VERSION, __DATE__, __TIME__, resetReasonStr(esp_reset_reason()));
  logConfig();

  state_init();

  // Task watchdog: the Arduino core does not subscribe any task by default. loopTask
  // becomes the only subscriber and feeds it only while all worker heartbeats are fresh,
  // so a hung CAN/GNSS/display task reboots the board (reset reason TASK_WDT).
  esp_task_wdt_config_t wdt = {};
  wdt.timeout_ms = WDT_TIMEOUT_MS;
  wdt.idle_core_mask = 0;
  wdt.trigger_panic = true;
  esp_err_t err = esp_task_wdt_reconfigure(&wdt);
  if (err != ESP_OK) err = esp_task_wdt_init(&wdt);  // in case the TWDT was not initialised by the core
  if (err != ESP_OK) log_e("task WDT setup failed: %s", esp_err_to_name(err));
  err = esp_task_wdt_add(NULL);
  if (err != ESP_OK) log_e("task WDT add failed: %s", esp_err_to_name(err));

  [[maybe_unused]] const bool canOk = can_vesc_start();  // log-only below CORE_DEBUG_LEVEL 3
  [[maybe_unused]] const bool gnssOk = gnss_start();
  [[maybe_unused]] const bool dispOk = display_start();
  [[maybe_unused]] const bool tripOk = trip_start();
  log_i("started: can=%d gnss=%d display=%d trip=%d", canOk, gnssOk, dispOk, tripOk);
}

// A heartbeat that was never touched counts as fresh during the first max_age ms
// after boot (tasks may still be starting); afterwards it must be alive.
static bool hbOk(uint32_t hb, uint32_t now, uint32_t max_age) {
  if (hb == 0) return now < max_age;
  // Signed age: a worker task can stamp its heartbeat between our millis() read
  // and this comparison (every worker outranks loop() and wakes on the same tick
  // that advances millis), so now - hb can be -1. Unsigned that is ~4e9 ms and
  // would raise a false "heartbeat stale" alarm and skip one WDT feed.
  return (int32_t)(now - hb) <= (int32_t)max_age;
}

void loop() {
  static uint32_t nextVescLog = 0, nextGnssLog = 0, nextTripLog = 0, nextSysLog = 0, nextStaleLog = 0, nextBlink = 0;
  static bool led = false;

  // Snapshot BEFORE reading the clock: every timestamp inside s must be <= now,
  // otherwise a frame that lands between the two reads shows up as an age of
  // 4294967295ms in the VESC/GNSS log line and as a stale LED for one cycle.
  const SharedState s = state_snapshot();
  const uint32_t now = millis();

  // ---- watchdog supervision -------------------------------------------------
  const bool canAlive = hbOk(hb_can, now, HB_MAX_CAN_MS);
  const bool gnssAlive = hbOk(hb_gnss, now, HB_MAX_GNSS_MS);
  const bool dispAlive = hbOk(hb_disp, now, HB_MAX_DISP_MS);
  if (canAlive && gnssAlive && dispAlive) {
    esp_task_wdt_reset();
  } else if ((int32_t)(now - nextStaleLog) >= 0) {
    nextStaleLog = now + 1000;
    log_e("heartbeat stale: can=%lu ms gnss=%lu ms disp=%lu ms (reboot in <= %lu ms)", (unsigned long)age_ms(hb_can, now),
          (unsigned long)age_ms(hb_gnss, now), (unsigned long)age_ms(hb_disp, now), (unsigned long)WDT_TIMEOUT_MS);
  }

  // ---- LED: 1 Hz blink when VESC data is fresh, 0.2 Hz otherwise -------------
  const bool vescFresh = vesc_fresh(&s.vesc.t, VESC_IDX_STATUS_1, now, VESC_STALE_R1_MS) ||
                         vesc_fresh(&s.vesc.t, VESC_IDX_STATUS_5, now, VESC_STALE_R1_MS);
  if ((int32_t)(now - nextBlink) >= 0) {
    nextBlink = now + (vescFresh ? 500 : 2500);
    led = !led;
    digitalWrite(PIN_LED, led ? HIGH : LOW);
  }

  // ---- periodic logs (deadline based, never "now % period") ------------------
  if (LOG_VESC_MS && (int32_t)(now - nextVescLog) >= 0) {
    nextVescLog = now + LOG_VESC_MS;
    can_vesc_log_summary(s, now);
  }
  if (LOG_GNSS_MS && (int32_t)(now - nextGnssLog) >= 0) {
    nextGnssLog = now + LOG_GNSS_MS;
    gnss_log_summary(s, now);
  }
  if (LOG_TRIP_MS && (int32_t)(now - nextTripLog) >= 0) {
    nextTripLog = now + LOG_TRIP_MS;
    trip_log_summary(s, now);
  }
  if (LOG_SYS_MS && (int32_t)(now - nextSysLog) >= 0) {
    nextSysLog = now + LOG_SYS_MS;
    log_i("SYS up=%lus reset=%s heap=%lu minheap=%lu lockfail=%lu | can=%s tec=%lu rec=%lu busoff=%lu | disp ok=%d "
          "refreshes=%lu skipped=%lu dim=%d scr=%u btn=%lu | poll sent=%lu ok=%lu bad=%lu to=%lu | hb can=%lu gnss=%lu disp=%lu",
          (unsigned long)(now / 1000), resetReasonStr(esp_reset_reason()), (unsigned long)ESP.getFreeHeap(),
          (unsigned long)ESP.getMinFreeHeap(), (unsigned long)s.lock_failures, can_state_str(s.can.state),
          (unsigned long)s.can.tec, (unsigned long)s.can.rec, (unsigned long)s.can.bus_off_count, s.disp.init_ok,
          (unsigned long)s.disp.refreshes, (unsigned long)s.disp.skipped_unchanged, s.disp.dimmed,
          (unsigned)s.disp.screen, (unsigned long)s.disp.button_presses, (unsigned long)s.vesc_ext.polls_sent,
          (unsigned long)s.vesc_ext.replies_ok, (unsigned long)s.vesc_ext.replies_bad, (unsigned long)s.vesc_ext.timeouts,
          (unsigned long)age_ms(hb_can, now),
          (unsigned long)age_ms(hb_gnss, now), (unsigned long)age_ms(hb_disp, now));
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}
