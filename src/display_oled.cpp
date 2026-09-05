// 128x64 SSD1309 OLED dashboard task (DIYables_OLED_SSD1309 over I2C, Adafruit GFX).
//
// Refresh policy:
//   * Every OLED_PERIOD_MS the task builds the display strings from a state
//     snapshot and pushes a frame ONLY when at least one string changed. A
//     frame is the whole 1 KB buffer (~26 ms on the bus at 400 kHz); a skipped
//     tick costs nothing but the memcmp.
//   * Idle: after OLED_IDLE_DIM_MS without a change the contrast drops to
//     OLED_IDLE_CONTRAST (burn-in / power); the next change restores it.
//   * A panel that does not ACK at init is retried every OLED_INIT_RETRY_MS
//     from the task, which keeps its heartbeat alive meanwhile (the supervisor
//     must not reboot the board because a display is unplugged). A panel that
//     stops ACKing later (cable, brown-out) is caught by a zero-length probe
//     every OLED_INIT_RETRY_MS and re-initialised the same way.
//
// Layout (rotation 0, classic 5x7 GFX font: a glyph cell is 6n x 8n px at size n):
//   y  0..23  speed, size 3 (18x24 px glyphs), right-aligned to x 76 (max 4 chars -> x >= 4)
//   x 80..127 right column, size 1: y 0 unit, y 8 fix line, y 16 CAN line (8 chars = 48 px)
//   y 25      horizontal rule
//   y 28..43  row 1: "B/V" caption (0,28)/(0,36) + v_in right-aligned to 62, size 2 (12x16 px);
//                    "B/A" caption (66,28)/(66,36) + i_in right-aligned to 128
//   y 47..62  row 2: "M/A" + i_motor (left), "P/W" + power (right), same geometry
//
// Compiled only when DISPLAY_TYPE == DISPLAY_TYPE_OLED_SSD1309 (the default);
// 'pio run -e epd' builds src/display_epd.cpp instead.
#include "config.h"
#if DISPLAY_TYPE == DISPLAY_TYPE_OLED_SSD1309

#include "display.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <DIYables_OLED_SSD1309.h>

#include <math.h>
#include <string.h>

#include "display_strings_oled.h"
#include "shared_state.h"

// ---- local tunables (candidates for the "Advanced task tunables" block in config.h) ----
#ifndef OLED_TASK_STACK
#define OLED_TASK_STACK 6144     // bytes. snprintf("%f") + GFX + a SharedState copy; the log_d refresh line prints the free minimum
#endif
#ifndef OLED_INIT_RETRY_MS
#define OLED_INIT_RETRY_MS 5000  // retry period while the controller does not ACK
#endif
#ifndef OLED_I2C_TIMEOUT_MS
#define OLED_I2C_TIMEOUT_MS 50   // per I2C transaction (Wire default); a stuck bus costs at most this per transfer
#endif

// The layout below is hard-coded for a 128x64 panel in landscape; the library
// takes uint8_t geometry and an int8_t reset pin.
static_assert(OLED_WIDTH == 128 && OLED_HEIGHT == 64, "display_oled.cpp lays out a 128x64 panel");
static_assert(OLED_ROTATION == 0 || OLED_ROTATION == 2, "OLED_ROTATION must be 0 or 2 (landscape layout)");
static_assert(PIN_OLED_RST >= -1 && PIN_OLED_RST <= 127, "PIN_OLED_RST must be -1 or a GPIO number");
static_assert(OLED_CONTRAST >= 0 && OLED_CONTRAST <= 255 && OLED_IDLE_CONTRAST >= 0 && OLED_IDLE_CONTRAST <= 255,
              "OLED contrast values are 0..255");
// i2cInit() maps 0 Hz to 100 kHz and clamps above 1 MHz; the bus frequency must
// equal OLED_I2C_HZ exactly or the library's setClock() calls below stop being no-ops.
static_assert(OLED_I2C_HZ >= 1 && OLED_I2C_HZ <= 1000000, "OLED_I2C_HZ must be 1..1000000 (SSD1309 max 400 kHz)");

// Both clocks equal: the library brackets every transfer with setClock(clkDuring)
// / setClock(clkAfter); Wire::setClock() short-circuits when the frequency is
// unchanged (on the IDF 5.5 HAL a real change would remove and re-add the device
// handle twice per frame).
static DIYables_OLED_SSD1309 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, PIN_OLED_RST, OLED_I2C_HZ, OLED_I2C_HZ);

// Speed cell: text size 3, advance box ends at x 76 (exclusive). Value cells: size 2.
static constexpr int16_t kSpeedRight = 76;
static constexpr int16_t kRuleY = 25;
static constexpr int16_t kColX = 80;
static constexpr int16_t kRow1Y = 28;
static constexpr int16_t kRow2Y = 47;
static constexpr int16_t kLeftValueRight = 62;
static constexpr int16_t kRightCaptionX = 66;

// ---------------------------------------------------------------- hardware init
// The library's begin() only fails on malloc and never checks for an ACK. Probe
// the controller ourselves so a missing / mis-addressed panel is reported
// instead of silently drawing into the void.
// A ZERO-length transaction on purpose: arduino-esp32's i2cWrite() turns it into
// i2c_master_probe(), which returns ESP_ERR_NOT_FOUND on a NACK -> Wire code 2
// and logs nothing above verbose. A write WITH data that gets NACKed would make
// the IDF driver ESP_LOGE "I2C transaction unexpected nack detected", the HAL
// log_e "i2c_master_transmit failed" and come back as code 4, not 2.
// Returns the Wire error: 0 ACKed, 2 address NACK (nothing at that address),
// 5 timeout (bus stuck / floating: missing pull-ups or power), 4 other.
static uint8_t oled_probe() {
  Wire.beginTransmission((uint8_t)OLED_I2C_ADDR);
  return Wire.endTransmission();
}

// Bus + controller init, safe to call repeatedly (the frame buffer is allocated
// once, the init sequence and the optional RST pulse are re-sent each time).
// Returns 0 when the panel answered, else a Wire error code (see oled_probe).
static uint8_t oled_init() {
  static bool wire_ok = false;
  if (!wire_ok) {
    // Pins FIRST and explicitly: the pinless Wire.begin() would start I2C0 on
    // the board's default SDA/SCL. begin() with the same pins on a started bus
    // only logs a warning and returns true.
    wire_ok = Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL, OLED_I2C_HZ);
    if (!wire_ok) {
      log_e("OLED: Wire.begin(sda=%d, scl=%d, %lu Hz) failed", (int)PIN_OLED_SDA, (int)PIN_OLED_SCL,
            (unsigned long)OLED_I2C_HZ);
      return 4;
    }
    Wire.setTimeOut(OLED_I2C_TIMEOUT_MS);
  }
  // Probe BEFORE begin(): the init sequence is 12 write transactions and every
  // NACKed one would cost an IDF + HAL error line (~25 lines per attempt, every
  // OLED_INIT_RETRY_MS, for an unplugged panel). The probe alone is silent.
  const uint8_t err = oled_probe();
  if (err != 0) return err;
  // periphBegin=false: the bus is ours (pins above). reset: pulse RST only when wired.
  if (!oled.begin(SSD1309_SWITCHCAPVCC, (uint8_t)OLED_I2C_ADDR, PIN_OLED_RST >= 0, false)) {
    log_e("OLED: frame buffer allocation failed (%u bytes)", (unsigned)(OLED_WIDTH * ((OLED_HEIGHT + 7) / 8)));
    return 4;
  }

  oled.setRotation(OLED_ROTATION);
  oled.setContrast((uint8_t)OLED_CONTRAST);
  oled.setFont(nullptr);  // classic 5x7 font: fixed 6n x 8n px cells, cursor = top-left
  oled.setTextColor(SSD1309_WHITE);
  // Never wrap: a clipped extreme value must stop at the edge, not spill a
  // second line into the row below.
  oled.setTextWrap(false);
  return 0;
}

static void log_init_ok() {
  log_i("OLED: init ok (SSD1309 %ux%u, I2C 0x%02X @ %lu Hz, sda=%d scl=%d rst=%d)", (unsigned)OLED_WIDTH,
        (unsigned)OLED_HEIGHT, (unsigned)OLED_I2C_ADDR, (unsigned long)OLED_I2C_HZ, (int)PIN_OLED_SDA,
        (int)PIN_OLED_SCL, (int)PIN_OLED_RST);
}

// ---------------------------------------------------------------- drawing
// Right-aligns s so that its advance box ends at rightEdgeExclusive (classic
// font: 6 * size px per character; the last column of every glyph is blank, so
// the ink ends size px earlier).
static void drawValueRight(const char *s, int16_t rightEdgeExclusive, int16_t y, uint8_t size) {
  const int16_t w = (int16_t)(6 * (int16_t)size * (int16_t)strlen(s));
  oled.setTextSize(size);
  oled.setCursor((int16_t)(rightEdgeExclusive - w), y);
  oled.print(s);
}

// Two stacked size-1 letters left of a size-2 value ("B" over "V" = battery volts).
static void drawCaption(int16_t x, int16_t y, char top, char bottom) {
  oled.setTextSize(1);
  oled.setCursor(x, y);
  oled.print(top);
  oled.setCursor(x, (int16_t)(y + 8));
  oled.print(bottom);
}

// Draws one frame into the (already cleared) buffer.
static void draw(const OledStrings &d) {
  const int16_t W = oled.width();

  // Zone A: speed + right column (y 0..24)
  drawValueRight(d.speed, kSpeedRight, 0, 3);
  oled.setTextSize(1);
  oled.setCursor(kColX, 0);
  oled.print(d.unit);
  oled.setCursor(kColX, 8);
  oled.print(d.fix);
  oled.setCursor(kColX, 16);
  oled.print(d.can);
  oled.drawFastHLine(0, kRuleY, W, SSD1309_WHITE);

  // Zone B: 2x2 values (y 28..62)
  drawCaption(0, kRow1Y, 'B', 'V');
  drawValueRight(d.v_in, kLeftValueRight, kRow1Y, 2);
  drawCaption(kRightCaptionX, kRow1Y, 'B', 'A');
  drawValueRight(d.i_in, W, kRow1Y, 2);
  drawCaption(0, kRow2Y, 'M', 'A');
  drawValueRight(d.i_motor, kLeftValueRight, kRow2Y, 2);
  drawCaption(kRightCaptionX, kRow2Y, 'P', 'W');
  drawValueRight(d.power, W, kRow2Y, 2);
}

// ---------------------------------------------------------------- demo source
#if DISPLAY_DEMO
// Synthetic, slowly varying telemetry so the layout and the refresh policy can be
// validated with no CAN or GNSS attached. Values change every second (so 3 of 4
// ticks are "unchanged"), except for a 30 s "hold" window in every 120 s cycle
// that exercises the idle path (set -DOLED_IDLE_DIM_MS=20000 to see the dimming).
static SharedState demo_state(uint32_t now) {
  SharedState s{};
  const uint32_t sec = now / 1000u;
  const uint32_t cyc = sec % 120u;
  const float t = (float)(cyc >= 90u ? sec - (cyc - 90u) : sec);  // frozen during the hold window
  const uint32_t stamp = now ? now : 1u;

  s.vesc.locked_id = 74;
  s.vesc.t.id = 74;
  s.vesc.v_in_ema = 48.0f + 4.0f * sinf(t / 17.0f);          // 44..52 V
  s.vesc.i_in_ema = 27.5f + 32.5f * sinf(t / 5.0f);          // -5..60 A (regen shows "-5" / "-240")
  s.vesc.i_motor_ema = 60.0f + 60.0f * sinf(t / 7.0f);       // 0..120 A
  s.vesc.t.temp_fet = 38.0f + 6.0f * sinf(t / 23.0f);
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) s.vesc.t.t_ms[i] = stamp;
  s.vesc.last_frame_ms = stamp;
  s.can.state = CAN_STATE_RUNNING;

  s.gnss.phase = GNSS_PHASE_RUN;
  s.gnss.baud = 38400;
  s.gnss.prot_ver_x100 = 1800;
  s.gnss.last_pvt_ms = stamp;
  s.gnss.fix_type = 3;
  s.gnss.fix_ok = true;
  s.gnss.num_sv = (uint8_t)(10 + (int)(2.0f * sinf(t / 13.0f)));  // 8..12
  const float spd = 7.0f + 7.0f * sinf(t / 11.0f);                 // 0..14 in the display unit (kn or km/h)
  s.gnss.gspeed_mm_s = (int32_t)(spd / SPEED_FACTOR);
  s.gnss.sacc_mm_s = 300;
  s.gnss.time_valid = true;
  s.gnss.hour = (uint8_t)((sec / 3600u) % 24u);
  s.gnss.min = (uint8_t)((sec / 60u) % 60u);
  s.gnss.sec = (uint8_t)(sec % 60u);
  return s;
}
#endif

// ---------------------------------------------------------------- task
struct DispCtx {
  OledStrings prev;  // what is on the panel right now
  DisplayStats st;   // counters + flags, mirrored into g_state.disp (e-ink fields stay 0)
};

// Filled by display_start() (boot frame) before the task exists, then owned by the task.
static DispCtx s_ctx;

static void publish_stats(const DisplayStats &st) {
  if (state_lock()) {
    g_state.disp = st;
    state_unlock();
  }
}

// Pushes one frame to the panel and records it as the reference for change detection.
static void render(DispCtx &c, const OledStrings &cur, uint32_t now) {
  const uint32_t t0 = millis();
  oled.clearDisplay();
  draw(cur);
  oled.display();
  [[maybe_unused]] const uint32_t dt = (uint32_t)(millis() - t0);  // log-only

  c.st.refreshes++;
  c.st.last_change_ms = now;
  c.prev = cur;
  // Stack figure = the calling task's minimum free stack (bytes on ESP-IDF): the
  // display task from refresh #2 on (#1 is the boot frame drawn from setup()/loopTask).
  log_d("OLED: refresh %lu ms (#%lu, stack free min %u B)", (unsigned long)dt, (unsigned long)c.st.refreshes,
        (unsigned)uxTaskGetStackHighWaterMark(nullptr));
}

// Nothing changed this tick: count it and step the idle dimming.
static void handle_idle(DispCtx &c, uint32_t now) {
  c.st.skipped_unchanged++;
#if OLED_IDLE_DIM_MS > 0
  const uint32_t idle = (uint32_t)(now - c.st.last_change_ms);
  if (!c.st.dimmed && idle >= (uint32_t)OLED_IDLE_DIM_MS) {
    oled.setContrast((uint8_t)OLED_IDLE_CONTRAST);
    c.st.dimmed = true;
    log_i("OLED: dimmed after %lu s idle", (unsigned long)(idle / 1000u));
  }
#else
  (void)now;
#endif
}

static void display_task(void *) {
  DispCtx &c = s_ctx;
  uint32_t next_retry_ms = millis() + OLED_INIT_RETRY_MS;
  [[maybe_unused]] uint32_t retries = 0;  // log-only
  hb_touch(hb_disp);

  log_i("OLED: task started (period %u ms, demo=%d)", (unsigned)OLED_PERIOD_MS, (int)DISPLAY_DEMO);

  for (;;) {
    const uint32_t tick_start = millis();
    hb_touch(hb_disp);

    // No panel yet: retry the init every OLED_INIT_RETRY_MS, keep the heartbeat alive.
    if (!c.st.init_ok) {
      if ((int32_t)(tick_start - next_retry_ms) >= 0) {
        next_retry_ms = tick_start + OLED_INIT_RETRY_MS;
        ++retries;
        const uint8_t err = oled_init();
        if (err == 0) {
          log_init_ok();
          c.st.init_ok = true;
          c.st.dimmed = false;  // oled_init() programmed OLED_CONTRAST
          OledStrings boot;
          oled_boot_strings(boot);
          render(c, boot, millis());  // the panel RAM is garbage after init: show something now
        } else {
          log_w("OLED: still no response at 0x%02X (retry %lu, Wire error %u)", (unsigned)OLED_I2C_ADDR,
                (unsigned long)retries, (unsigned)err);
        }
      }
      publish_stats(c.st);
      vTaskDelay(pdMS_TO_TICKS(OLED_PERIOD_MS));
      continue;
    }

    // Panel present: re-probe it every OLED_INIT_RETRY_MS (one zero-length
    // transaction, ~30 us). The library swallows every I2C error, so without
    // this a loose connector or a brown-out that reset the controller (Display
    // OFF, RAM cleared) would leave a dark panel until the next reboot. On a
    // miss fall back to the retry path above, which re-initialises and redraws.
    if ((int32_t)(tick_start - next_retry_ms) >= 0) {
      next_retry_ms = tick_start + OLED_INIT_RETRY_MS;
      const uint8_t err = oled_probe();
      if (err != 0) {
        log_w("OLED: lost contact at 0x%02X (Wire error %u), re-initialising", (unsigned)OLED_I2C_ADDR, (unsigned)err);
        c.st.init_ok = false;
        next_retry_ms = tick_start;  // first re-init attempt on the next tick
        publish_stats(c.st);
        vTaskDelay(pdMS_TO_TICKS(OLED_PERIOD_MS));
        continue;
      }
    }

#if DISPLAY_DEMO
    const SharedState s = demo_state(tick_start);
#else
    const SharedState s = state_snapshot();
#endif
    const uint32_t now = millis();
    OledStrings cur;
    oled_build_strings(s, now, cur);

    if (memcmp(&cur, &c.prev, sizeof cur) == 0) {
      handle_idle(c, now);
    } else {
      if (c.st.dimmed) {
        oled.setContrast((uint8_t)OLED_CONTRAST);
        c.st.dimmed = false;
        log_i("OLED: contrast restored (value changed)");
      }
      render(c, cur, now);
    }
    publish_stats(c.st);  // small memcpy under the lock, never while drawing

    // Pace to OLED_PERIOD_MS measured from the tick start (a frame push eats
    // ~26 ms of it); always yield at least a little so loop() keeps running.
    const uint32_t spent = (uint32_t)(millis() - tick_start);
    const uint32_t wait = spent >= OLED_PERIOD_MS ? 10u : (uint32_t)(OLED_PERIOD_MS - spent);
    vTaskDelay(pdMS_TO_TICKS(wait));
  }
}

// ---------------------------------------------------------------- public
bool display_start() {
  DispCtx &c = s_ctx;
  memset(&c, 0, sizeof c);

  const uint8_t err = oled_init();
  if (err == 0) {
    log_init_ok();
    c.st.init_ok = true;
    OledStrings boot;  // all "--", fix line BOOT, CAN line = firmware version
    oled_boot_strings(boot);
    render(c, boot, millis());
  } else {
    log_e("OLED: no response at 0x%02X (check wiring/address jumper), Wire error %u", (unsigned)OLED_I2C_ADDR,
          (unsigned)err);
  }
  publish_stats(c.st);

  // The task always runs: it retries the init and keeps hb_disp fresh either way.
  if (xTaskCreate(display_task, "disp", OLED_TASK_STACK, nullptr, TASK_PRIO_DISP, nullptr) != pdPASS) {
    log_e("OLED: xTaskCreate failed");
    return false;
  }
  return c.st.init_ok;
}

#endif  // DISPLAY_TYPE == DISPLAY_TYPE_OLED_SSD1309
