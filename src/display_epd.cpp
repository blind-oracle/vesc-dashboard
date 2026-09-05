// E-paper dashboard task: GxEPD2 instance, layout, refresh and idle policy.
//
// Refresh policy (ghosting management for the SSD1683):
//   * A frame is drawn only when at least one displayed string changed.
//   * Changes use a full-window FAST PARTIAL update (~0.4 s waveform, no flash).
//   * A (flashing) FULL refresh is forced on the first frame, after a hibernate
//     wake, after EPD_FULL_EVERY_N_PARTIALS partials or EPD_FULL_EVERY_MS.
//   * Idle: powerOff() after EPD_POWEROFF_IDLE_MS (booster off, image stays),
//     hibernate() after EPD_HIBERNATE_IDLE_MS (deep sleep; GxEPD2 resets the
//     panel by itself on the next write, we never touch RST/BUSY directly).
//
// LEGACY: compiled only when DISPLAY_TYPE == DISPLAY_TYPE_EPD_GDEY042T81
// ('pio run -e epd'). The default build uses src/display_oled.cpp.
#include "config.h"
#if DISPLAY_TYPE == DISPLAY_TYPE_EPD_GDEY042T81

#include "display_epd.h"

#include <Arduino.h>
#include <SPI.h>
#include <GxEPD2_BW.h>  // auto-includes gdey/GxEPD2_420_GDEY042T81.h
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

#include <math.h>
#include <string.h>

#include "config.h"
#include "display_strings.h"
#include "shared_state.h"

// 400x300 / 8 = 15 000 byte frame buffer, one page: firstPage()/nextPage() run the draw once.
static GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT>
    display(GxEPD2_420_GDEY042T81(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY));

// GxEPD2 hard-codes a 10 s BUSY timeout for this panel (GxEPD2_420_GDEY042T81 ctor)
// and only reports it with Serial.println("Busy Timeout!"). We detect it ourselves
// from the busy callback: one uninterrupted busy wait lasting almost that long.
static constexpr uint32_t kGxBusyTimeoutMs = 10000;
static constexpr uint32_t kBusyDetectMs = kGxBusyTimeoutMs - 500;
static constexpr uint32_t kBusyGapMs = 500;  // a longer gap between callbacks = a new busy wait

// Written and read only from the display task (the busy callback runs in its context).
static uint32_t s_busy_wait_start_ms = 0;
static uint32_t s_busy_last_ms = 0;
static bool s_busy_flagged = false;
static uint32_t s_busy_timeouts = 0;

// Runs inside GxEPD2's _waitWhileBusy() loop INSTEAD of its delay(1): it must
// yield, or the refresh would spin the CPU for up to 1.2 s (starving loop()).
// It also feeds the display heartbeat so a 10 s busy timeout is not mistaken
// for a hung task by the supervisor.
static void busy_cb(const void *) {
  const uint32_t now = millis();
  if ((uint32_t)(now - s_busy_last_ms) > kBusyGapMs) {
    s_busy_wait_start_ms = now;
    s_busy_flagged = false;
  }
  s_busy_last_ms = now;
  if (!s_busy_flagged && (uint32_t)(now - s_busy_wait_start_ms) >= kBusyDetectMs) {
    s_busy_flagged = true;
    s_busy_timeouts++;
  }
  hb_touch(hb_disp);
  vTaskDelay(1);
}

// ---------------------------------------------------------------- drawing
// Right-aligns text: getTextBounds() returns the ink box relative to the
// cursor (x1 can be negative for glyphs that overhang), so the cursor goes
// at rightX - width - x1, not simply rightX - width.
static void printRight(const char *s, int16_t rightX, int16_t baseline) {
  int16_t x1 = 0, y1 = 0;
  uint16_t w = 0, h = 0;
  display.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((int16_t)(rightX - (int16_t)w - x1), baseline);
  display.print(s);
}

// One 200x62 Zone B tile: framed, small label top-left, big value bottom-right.
static void drawTile(int16_t x, int16_t y, const char *label, const char *value) {
  display.drawRect(x, y, 200, 62, GxEPD_BLACK);
  display.setFont(&FreeSans9pt7b);
  display.setCursor((int16_t)(x + 6), (int16_t)(y + 16));
  display.print(label);
  display.setFont(&FreeSansBold18pt7b);
  printRight(value, (int16_t)(x + 192), (int16_t)(y + 52));
}

// Layout per the plan (rotation 0, 400x300). Called between firstPage()/nextPage(),
// the buffer is already white.
static void draw(const DisplayStrings &d) {
  const int16_t W = display.width();

  // Zone A: speed (y 0..149)
  display.setFont(&FreeSans9pt7b);
  display.setCursor(8, 22);
  display.print("SPEED");
  display.setFont(&FreeSansBold24pt7b);
  display.setTextSize(2);  // 24 pt at 2x: digits ~52 px wide, 70 px tall; "88.8" ~ 180 px
  printRight(d.speed, 300, 118);
  display.setTextSize(1);
  display.setFont(&FreeSans12pt7b);
  display.setCursor(308, 118);
  display.print(d.speed_unit);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(310, 40);
  display.print(d.sats);
  display.setCursor(310, 62);
  display.print(d.fix);
  display.setCursor(310, 84);
  display.print(d.utc);
  display.drawFastHLine(0, 150, W, GxEPD_BLACK);
  display.drawFastHLine(0, 151, W, GxEPD_BLACK);

  // Zone B: 2x2 tiles (y 152..275)
  drawTile(0, 152, "BATTERY V", d.v_in);
  drawTile(200, 152, "BATTERY A", d.i_in);
  drawTile(0, 214, "MOTOR A", d.i_motor);
  drawTile(200, 214, "POWER W", d.power);
  display.drawFastHLine(0, 276, W, GxEPD_BLACK);

  // Zone C: status bar (y 278..300)
  display.setFont(&FreeSans9pt7b);
  display.setCursor(4, 296);
  display.print(d.status_l);
  printRight(d.status_r, 396, 296);
}

// ---------------------------------------------------------------- demo source
#if DISPLAY_DEMO
// Synthetic, slowly varying telemetry so the layout and the refresh policy can be
// validated with no CAN or GNSS attached. Values change every second, except for
// a 30 s "hold" window in every 120 s cycle that exercises the idle powerOff path.
static SharedState demo_state(uint32_t now) {
  SharedState s{};
  const uint32_t sec = now / 1000u;
  const uint32_t cyc = sec % 120u;
  const float t = (float)(cyc >= 90u ? sec - (cyc - 90u) : sec);  // frozen during the hold window
  const uint32_t stamp = now ? now : 1u;

  s.vesc.locked_id = 74;
  s.vesc.t.id = 74;
  s.vesc.v_in_ema = 48.0f + 4.0f * sinf(t / 17.0f);          // 44..52 V
  s.vesc.i_in_ema = 27.5f + 32.5f * sinf(t / 5.0f);          // -5..60 A
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
  DisplayStrings prev;     // what is on the panel right now
  DisplayStats st;         // counters + flags, mirrored into g_state.disp
  uint32_t shown_partials; // partial counter printed in prev.status_r ("#N")
  bool first;              // no frame drawn yet
};

static void publish_stats(const DisplayStats &st) {
  if (state_lock()) {
    g_state.disp = st;
    state_unlock();
  }
}

// Draws one frame. Full: whole-panel waveform with flash (clean image).
// Partial: full-window fast partial (differential, no flash; ghosts slowly).
static void render(DispCtx &c, const DisplayStrings &cur, uint32_t now, bool full) {
  const uint32_t t0 = millis();
  const uint32_t busy_before = s_busy_timeouts;

  if (full) display.setFullWindow();
  else display.setPartialWindow(0, 0, (uint16_t)display.width(), (uint16_t)display.height());
  display.firstPage();
  do {
    draw(cur);
  } while (display.nextPage());

  [[maybe_unused]] const uint32_t dt = millis() - t0;  // log-only
  c.st.refreshes++;
  if (full) {
    c.st.partials = 0;
    c.st.fulls++;
    c.st.last_full_ms = now;
    c.st.hibernating = false;
  } else {
    c.st.partials++;
    c.st.partials_total++;
  }
  c.st.powered_off = false;  // (after a full refresh GxEPD2 powers off internally; the idle powerOff() is then a no-op)
  c.st.last_change_ms = now;
  c.st.busy_timeouts = s_busy_timeouts;
  c.prev = cur;
  c.first = false;

  if (s_busy_timeouts != busy_before) log_w("display: BUSY timeout during refresh (check BUSY wiring / RESE switch)");
  if (full) log_i("display: full refresh %lu ms (fulls=%lu partials_total=%lu)", (unsigned long)dt,
                  (unsigned long)c.st.fulls, (unsigned long)c.st.partials_total);
  else log_d("display: partial refresh %lu ms (#%lu)", (unsigned long)dt, (unsigned long)c.st.partials);
}

// Nothing changed this tick: count it and step the idle state machine.
static void handle_idle(DispCtx &c, uint32_t now) {
  c.st.skipped_unchanged++;
  const uint32_t idle = (uint32_t)(now - c.st.last_change_ms);
  if (!c.st.powered_off && idle >= EPD_POWEROFF_IDLE_MS) {
    display.powerOff();
    c.st.powered_off = true;
    log_i("display: idle %lu ms -> powerOff", (unsigned long)idle);
  }
  if (!c.st.hibernating && idle >= EPD_HIBERNATE_IDLE_MS) {
    display.hibernate();
    c.st.hibernating = true;
    log_i("display: idle %lu ms -> hibernate (next change = full refresh)", (unsigned long)idle);
  }
}

static bool want_full(const DispCtx &c, uint32_t now) {
  return c.first || c.st.hibernating || c.st.partials >= EPD_FULL_EVERY_N_PARTIALS ||
         (uint32_t)(now - c.st.last_full_ms) >= EPD_FULL_EVERY_MS;
}

static void display_task(void *) {
  static DispCtx c;  // static: keeps the two 150-byte string sets off the task stack
  memset(&c, 0, sizeof c);
  c.first = true;
  hb_touch(hb_disp);

  log_i("display: task started, %ux%u, pages=%u, demo=%d", (unsigned)display.width(), (unsigned)display.height(),
        (unsigned)display.pages(), (int)DISPLAY_DEMO);

  // Boot frame: all "--", status BOOT. Costs two full refreshes (GxEPD2's hidden
  // initial clearScreen + ours), ~2.5 s.
  {
    DisplayStrings boot;
    display_boot_strings(boot);
    render(c, boot, millis(), true);
    publish_stats(c.st);
  }

  for (;;) {
    const uint32_t tick_start = millis();
    hb_touch(hb_disp);

#if DISPLAY_DEMO
    const SharedState s = demo_state(tick_start);
#else
    const SharedState s = state_snapshot();
#endif
    const uint32_t now = millis();
    DisplayStrings cur;
    // Change detection must not see the partial counter itself: the panel says
    // "#N" while c.st.partials is already N+1 after that refresh, so a frame
    // built with the live count would differ on EVERY tick and the panel would
    // refresh forever (never idling, flashing a full refresh every 30 partials).
    // Compare against a frame built with the count that is printed on the
    // panel; only when something else changed print the live count.
    display_build_strings(s, now, c.shown_partials, cur);

    if (memcmp(&cur, &c.prev, sizeof cur) == 0) {
      handle_idle(c, now);
    } else {
      if (c.shown_partials != c.st.partials) display_build_strings(s, now, c.st.partials, cur);
      c.shown_partials = c.st.partials;  // the count BEFORE this refresh, per the layout spec
      render(c, cur, now, want_full(c, now));
    }
    c.st.busy_timeouts = s_busy_timeouts;  // idle powerOff()/hibernate() wait on BUSY too
    publish_stats(c.st);                   // small memcpy under the lock, never while drawing

    // Pace to DISPLAY_PERIOD_MS measured from the tick start (a refresh eats
    // 0.4-1.2 s of it); always yield at least a little so loop() keeps running.
    const uint32_t spent = (uint32_t)(millis() - tick_start);
    const uint32_t wait = spent >= DISPLAY_PERIOD_MS ? 100u : (uint32_t)(DISPLAY_PERIOD_MS - spent);
    vTaskDelay(pdMS_TO_TICKS(wait));
  }
}

// ---------------------------------------------------------------- public
bool display_start() {
  // Pins FIRST: epd2.init() calls the pinless SPI.begin(), which returns early
  // once the bus is up. Without this it would start FSPI on the board's default
  // pins. MISO/SS unused (the panel is write-only, CS is driven by GxEPD2).
  SPI.begin(PIN_EPD_SCK, -1, PIN_EPD_MOSI, -1);
  display.init(EPD_DIAG_BAUD, true, EPD_RESET_MS, false);
  display.epd2.selectSPI(SPI, SPISettings(EPD_SPI_HZ, MSBFIRST, SPI_MODE0));
  display.epd2.setBusyCallback(busy_cb, nullptr);
  if (!EPD_FAST_FULL_UPDATE) display.epd2.selectFastFullUpdate(false);  // slow, temperature-compensated waveform
  display.setRotation(EPD_ROTATION);
  display.setTextColor(GxEPD_BLACK);
  // Never wrap: a wide (truncated extreme) value must clip at the edge, not
  // spill a second line into the tile below. getTextBounds() honours the wrap
  // flag as well, so right alignment would otherwise miscompute for such strings.
  display.setTextWrap(false);

  const BaseType_t ok = xTaskCreate(display_task, "disp", 8192, nullptr, TASK_PRIO_DISP, nullptr);
  if (ok != pdPASS) {
    log_e("display: xTaskCreate failed");
    return false;
  }
  log_i("EPD: init ok (GDEY042T81 %ux%u, SPI %lu Hz, cs=%d dc=%d rst=%d busy=%d, fast_full=%d)",
        (unsigned)GxEPD2_420_GDEY042T81::WIDTH, (unsigned)GxEPD2_420_GDEY042T81::HEIGHT, (unsigned long)EPD_SPI_HZ,
        (int)PIN_EPD_CS, (int)PIN_EPD_DC, (int)PIN_EPD_RST, (int)PIN_EPD_BUSY, (int)EPD_FAST_FULL_UPDATE);
  if (state_lock()) {
    g_state.disp.init_ok = true;  // GxEPD2 cannot report a missing panel; BUSY timeouts are counted instead
    state_unlock();
  }
  return true;
}

#endif  // DISPLAY_TYPE == DISPLAY_TYPE_EPD_GDEY042T81
