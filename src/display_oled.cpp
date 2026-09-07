// 128x64 SSD1309 OLED dashboard task (native u8g2 over ESP-IDF driver/i2c_master.h,
// HAL in src/u8g2_hal_idf.cpp).
//
// Screens (include/display_strings_oled.h, OledScreen): MAIN, EFFICIENCY, VESC 1/3,
// VESC 2/3, VESC 3/3, GNSS, SYS. A short press of PIN_BUTTON shows the next screen
// (wrapping), a long press (>= BUTTON_LONG_PRESS_MS) or SCREEN_AUTO_RETURN_MS
// without a press returns to MAIN. PIN_BUTTON -1 compiles no button code at all.
//
// Loop / refresh policy:
//   * The task loop runs every OLED_BUTTON_POLL_MS (20 ms): it samples the
//     button (time-based debounce, so the ~30 ms of a frame push does not matter)
//     and touches its heartbeat.
//   * Every OLED_PERIOD_MS, or immediately when the screen changed, it builds
//     the frame descriptor (OledFrame) for the current screen from a state
//     snapshot and pushes a frame ONLY when the descriptor differs from what is
//     on the panel (memcmp; the screen index is part of the descriptor, so a
//     screen switch always redraws). A frame is the whole 1 KB buffer (~30 ms
//     on the bus at 400 kHz); a skipped tick costs nothing but the memcmp.
//   * Idle: after OLED_IDLE_DIM_MS without a change the contrast drops to
//     OLED_IDLE_CONTRAST (burn-in / power); the next change or any button press
//     restores it.
//   * A panel that does not ACK at init is retried every OLED_INIT_RETRY_MS
//     from the task, which keeps its heartbeat alive meanwhile (the supervisor
//     must not reboot the board because a display is unplugged). A panel that
//     stops ACKing later (cable, brown-out) is caught by the failing frame
//     transfer or by the zero-length probe every OLED_INIT_RETRY_MS, and is
//     re-initialised the same way.
//
// Layout (rotation 0; all text is drawn by BASELINE with U8g2 fonts: a glyph of
// height h with y-offset 0 occupies rows baseline-h .. baseline-1; exact numbers
// in oled_layout, checked there with static_assert):
//   MAIN
//     rows  0..23  speed, u8g2_font_logisoso24_tn (24 px digits, 15 px advance), advance box right-aligned
//                  to x 54 (ink columns <= 52), baseline 24
//     x 56..127    right column, u8g2_font_6x10_tf (6 px advance, 7 px caps), 12 chars, 8 px pitch:
//                  baseline  8 (rows 1..7)   "HH:MM" + ' ' + fix        e.g. "12:34 3D 9sv"
//                  baseline 16 (rows 9..15)  CAN line                   e.g. "VESC 74"
//                  baseline 24 (rows 17..23) speed unit                 "kn" / "km/h"
//     row 26       horizontal rule
//     rows 29..62  4 rows x 2 cells (64 px each), 6x10, baselines 36/45/54/63: label at cell x + 2,
//                  value right-aligned by advance to cell x + 62:  "BV 48.2v | BA 12.4A", "MA 35A | PW 598W",
//                  "TF 45° | TM 32°", "RPM 2350 | WH 56.7"
//   GRID (EFFICIENCY .. SYS)
//     rows  0..9   inverted title bar: u8g2_font_6x13B_tf black on white, title at x 1, page "n/7"
//                  right-aligned to x 127, baseline 9 (rows 0..8)
//     rows 11..62  6 rows of 21 chars, 6x10 at x 1, baselines 18/27/36/45/54/63 (9 px pitch)
//
#include "config.h"

#include "display.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <u8g2.h>

#include <math.h>
#include <string.h>

#include "display_strings_oled.h"
#include "shared_state.h"
#include "u8g2_hal_idf.h"

static const char *TAG = "oled";

// ---- local fallbacks (config.h "Advanced task tunables" defines the same names; these only cover a trimmed config.h) ----
#ifndef OLED_TASK_STACK
#define OLED_TASK_STACK 6144     // bytes. snprintf("%f") + font decoder + a SharedState copy; the debug refresh line prints the free minimum
#endif
#ifndef OLED_INIT_RETRY_MS
#define OLED_INIT_RETRY_MS 5000  // retry period while the controller does not ACK
#endif
#ifndef OLED_I2C_TIMEOUT_MS
#define OLED_I2C_TIMEOUT_MS 50   // per I2C transaction; a stuck bus costs at most this per transfer
#endif
#ifndef OLED_BUTTON_POLL_MS
#define OLED_BUTTON_POLL_MS 20   // task loop period: button sampling granularity (debounce is time based)
#endif

// The layout below is hard-coded for a 128x64 panel in landscape.
static_assert(OLED_WIDTH == 128 && OLED_HEIGHT == 64, "display_oled.cpp lays out a 128x64 panel");
static_assert(OLED_ROTATION == 0 || OLED_ROTATION == 2, "OLED_ROTATION must be 0 or 2 (landscape layout)");
static_assert(PIN_OLED_RST >= -1 && PIN_OLED_RST <= 30, "PIN_OLED_RST must be -1 or an ESP32-C6 GPIO 0..30");
static_assert(OLED_CONTRAST >= 0 && OLED_CONTRAST <= 255 && OLED_IDLE_CONTRAST >= 0 && OLED_IDLE_CONTRAST <= 255,
              "OLED contrast values are 0..255");
static_assert(OLED_I2C_HZ >= 1 && OLED_I2C_HZ <= 1000000, "OLED_I2C_HZ must be 1..1000000 (SSD1309 max 400 kHz)");
static_assert(OLED_I2C_ADDR >= 0x08 && OLED_I2C_ADDR <= 0x77, "OLED_I2C_ADDR is a 7-bit address (0x3C or 0x3D)");
static_assert(PIN_BUTTON >= -1 && PIN_BUTTON <= 30, "PIN_BUTTON must be -1 (none) or an ESP32-C6 GPIO 0..30");
static_assert(BUTTON_DEBOUNCE_MS > 0 && BUTTON_DEBOUNCE_MS < BUTTON_LONG_PRESS_MS,
              "BUTTON_DEBOUNCE_MS must be positive and shorter than BUTTON_LONG_PRESS_MS");
static_assert(SCREEN_AUTO_RETURN_MS >= 0, "SCREEN_AUTO_RETURN_MS must be >= 0 (0 = never)");
static_assert(OLED_BUTTON_POLL_MS > 0 && OLED_BUTTON_POLL_MS <= OLED_PERIOD_MS, "OLED_BUTTON_POLL_MS must be 1..OLED_PERIOD_MS");
static_assert(oled_layout::kWidth == OLED_WIDTH && oled_layout::kHeight == OLED_HEIGHT, "layout geometry matches the panel");

// The u8g2 object owns the 1 KB frame buffer (full-buffer "_f" setup, static memory
// inside u8g2) and the font decoder state -> display task only (display_start() draws
// the boot frame before the task exists).
static u8g2_t s_u8g2;

// ---------------------------------------------------------------- hardware init
// Zero-length transaction: ESP_OK when the controller ACKed its address,
// ESP_ERR_NOT_FOUND when nothing did (wrong address, SDA/SCL swapped, module in SPI
// mode or unpowered), ESP_ERR_TIMEOUT when the bus never idles high (no pull-ups,
// short). The probe is silent; a NACKed write would make the IDF driver log an error.
static esp_err_t oled_probe() { return u8g2_hal_idf_probe(); }

// Bus + controller init, safe to call repeatedly (bus and device handle are created
// once, the reset pulse and the init sequence are re-sent each time). Returns ESP_OK
// when the panel answered and the init sequence went through, else the driver error.
static esp_err_t oled_init() {
  static bool setup_done = false;
  const U8g2HalIdfConfig hal = {PIN_OLED_SDA, PIN_OLED_SCL, PIN_OLED_RST, (uint32_t)OLED_I2C_HZ, (uint8_t)OLED_I2C_ADDR,
                                (uint32_t)OLED_I2C_TIMEOUT_MS};
  esp_err_t err = u8g2_hal_idf_init(hal);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "OLED: I2C bus setup failed (sda=%d scl=%d %lu Hz): %s", (int)PIN_OLED_SDA, (int)PIN_OLED_SCL,
             (unsigned long)OLED_I2C_HZ, esp_err_to_name(err));
    return err;
  }
  // Probe BEFORE the init sequence: its ~12 write transactions would each cost an IDF
  // error line for an unplugged panel, every OLED_INIT_RETRY_MS. The probe alone is silent.
  err = oled_probe();
  if (err != ESP_OK) return err;

  if (!setup_done) {
    // noname0 = no column offset (the panel's 128 columns map to segments 0..127).
    u8g2_Setup_ssd1309_i2c_128x64_noname0_f(&s_u8g2, OLED_ROTATION == 2 ? U8G2_R2 : U8G2_R0, u8g2_hal_idf_byte_cb,
                                            u8g2_hal_idf_gpio_delay_cb);
    u8g2_SetI2CAddress(&s_u8g2, (uint8_t)(OLED_I2C_ADDR << 1));  // u8g2 keeps the 8-bit form
    setup_done = true;
  }
  (void)u8g2_hal_idf_take_last_error();
  u8g2_InitDisplay(&s_u8g2);  // RST pulse (when wired) + init sequence; leaves the panel in power-save
  u8g2_SetPowerSave(&s_u8g2, 0);
  u8g2_SetContrast(&s_u8g2, (uint8_t)OLED_CONTRAST);
  // Text goes through U8g2 fonts only. Transparent mode: glyph background pixels are
  // not painted, so glyphs may overlap rules and the inverted title bar.
  u8g2_SetFontMode(&s_u8g2, 1);
  u8g2_SetFontPosBaseline(&s_u8g2);
  u8g2_SetFontDirection(&s_u8g2, 0);
  u8g2_SetDrawColor(&s_u8g2, 1);
  return u8g2_hal_idf_take_last_error();  // a transfer of the init sequence failed -> not initialised
}

static void log_init_ok() {
  ESP_LOGI(TAG, "OLED: init ok (SSD1309 %ux%u, I2C 0x%02X @ %lu Hz, sda=%d scl=%d rst=%d)", (unsigned)OLED_WIDTH,
           (unsigned)OLED_HEIGHT, (unsigned)OLED_I2C_ADDR, (unsigned long)OLED_I2C_HZ, (int)PIN_OLED_SDA,
           (int)PIN_OLED_SCL, (int)PIN_OLED_RST);
}

// ---------------------------------------------------------------- drawing
// Everything between the OLED_DRAW markers uses only the u8g2 buffer API on
// `s_u8g2`, so the host preview harness can compile it against a stub.
// OLED_DRAW_BEGIN
using namespace oled_layout;

// Always re-assert transparency after a font change (cheap; keeps the invariant local).
static void use_font(const uint8_t *font) {
  u8g2_SetFont(&s_u8g2, font);
  u8g2_SetFontMode(&s_u8g2, 1);
}

// Sum of the glyph advances of an ASCII string in the current font (a missing
// glyph advances 0). Advance-based alignment keeps tabular digits in fixed
// columns: ink-based alignment (u8g2_GetUTF8Width) would shift "10.1" against
// "10.0" because '1' has a narrower bitmap.
static int16_t text_advance(const char *s) {
  int16_t w = 0;
  for (; *s; ++s) w = (int16_t)(w + u8g2_GetGlyphWidth(&s_u8g2, (uint16_t)(uint8_t)*s));
  return w;
}

// Byte-wise (no UTF-8 decoding): the single Latin-1 byte 0xB0 in the strings is the
// degree sign U+00B0 of the *_tf fonts, one byte = one glyph cell.
static void draw_text(int16_t x, int16_t baseline, const char *s) {
  u8g2_DrawStr(&s_u8g2, (u8g2_uint_t)x, (u8g2_uint_t)baseline, s);
}

// Right-aligns by advance: the advance box ends at rightExclusive.
static void draw_text_right(const char *s, int16_t rightExclusive, int16_t baseline) {
  draw_text((int16_t)(rightExclusive - text_advance(s)), baseline, s);
}

// Main screen (see the layout comment at the top).
static void draw_main(const OledMain &m) {
  use_font(u8g2_font_logisoso24_tn);
  draw_text_right(m.speed, kSpeedRight, kSpeedBaseline);

  use_font(u8g2_font_6x10_tf);
  draw_text(kColX, kColBaseline[0], m.clock);
  draw_text((int16_t)(kColX + (kClockChars + 1) * kAdvance), kColBaseline[0], m.fix);
  draw_text(kColX, kColBaseline[1], m.can);
  draw_text(kColX, kColBaseline[2], m.unit);

  u8g2_DrawHLine(&s_u8g2, 0, kRuleY, kWidth);

  for (int r = 0; r < 4; ++r) {
    for (int col = 0; col < 2; ++col) {
      const OledMain::Cell &c = m.cells[r * 2 + col];
      const int16_t x0 = (int16_t)(col * kCellW);
      draw_text((int16_t)(x0 + kCellLabelDx), kCellBaseline[r], c.label);
      // monospace: position by advance (6 px per byte; the degree sign is one byte)
      draw_text((int16_t)(x0 + kCellValueRightDx - kAdvance * (int16_t)strlen(c.value)), kCellBaseline[r], c.value);
    }
  }
}

// Grid screen: inverted title bar + up to 6 monospace rows.
static void draw_grid(const OledGrid &g) {
  u8g2_DrawBox(&s_u8g2, 0, 0, kWidth, kTitleBarH);
  use_font(u8g2_font_6x13B_tf);
  u8g2_SetDrawColor(&s_u8g2, 0);  // transparent mode paints foreground pixels only -> black text on the bar
  draw_text(kTitleX, kTitleBaseline, g.title);
  draw_text((int16_t)(kWidth - 1 - kAdvance * (int16_t)strlen(g.page)), kTitleBaseline, g.page);
  u8g2_SetDrawColor(&s_u8g2, 1);

  use_font(u8g2_font_6x10_tf);
  const int n = g.nrows < kGridRows ? g.nrows : kGridRows;
  for (int r = 0; r < n; ++r) draw_text(kGridX, (int16_t)(kGridBaseline0 + r * kGridPitch), g.rows[r]);
}

// Draws one frame into the (already cleared) buffer.
static void draw_frame(const OledFrame &f) {
  if (f.screen == SCREEN_MAIN) draw_main(f.main);
  else draw_grid(f.grid);
}
// OLED_DRAW_END

// ---------------------------------------------------------------- button
#if PIN_BUTTON >= 0
enum class ButtonEvent : uint8_t { None, Short, Long };

// Polled push button: the raw level must be stable for BUTTON_DEBOUNCE_MS before
// it is accepted (time based, so the poll jitter of a frame push is harmless),
// then the debounced level is edge-detected. Long fires ONCE while the button
// is still held (at BUTTON_LONG_PRESS_MS) and the release after it produces no
// Short. All stamps are state_now_ms(); the differences are unsigned (wrap-safe).
struct Button {
  bool raw_last = false;     // last raw sample (true = pressed)
  uint32_t raw_since = 0;    // time of the last raw change
  bool pressed = false;      // debounced level
  uint32_t pressed_at = 0;   // debounced press time
  bool long_fired = false;
  uint32_t presses = 0;      // debounced presses (-> DisplayStats.button_presses)

  ButtonEvent update(bool raw_pressed, uint32_t now) {
    ButtonEvent ev = ButtonEvent::None;
    if (raw_pressed != raw_last) {
      raw_last = raw_pressed;
      raw_since = now;
    } else if (raw_pressed != pressed && (uint32_t)(now - raw_since) >= (uint32_t)BUTTON_DEBOUNCE_MS) {
      pressed = raw_pressed;  // debounced edge
      if (pressed) {
        pressed_at = now;
        long_fired = false;
        ++presses;
      } else if (!long_fired) {
        ev = ButtonEvent::Short;  // release after a short press
      }
    }
    if (pressed && !long_fired && (uint32_t)(now - pressed_at) >= (uint32_t)BUTTON_LONG_PRESS_MS) {
      long_fired = true;
      ev = ButtonEvent::Long;  // fires while still held
    }
    return ev;
  }
};

// gpio_config() must have run (display_start).
static bool button_raw_pressed() { return (gpio_get_level((gpio_num_t)PIN_BUTTON) == 0) == (BUTTON_ACTIVE_LOW != 0); }
#endif  // PIN_BUTTON >= 0

// ---------------------------------------------------------------- demo source
#if DISPLAY_DEMO
// Synthetic, slowly varying telemetry so every screen and the refresh policy can
// be validated with no CAN or GNSS attached. Values change every second (so most
// 250 ms ticks are "unchanged"), except for a 30 s "hold" window in every 120 s
// cycle that exercises the idle path (set -DOLED_IDLE_DIM_MS=20000 to see the
// dimming). Everything, including the clock and the counters, derives from the
// frozen time so the hold window really holds. A live fault code (OT_FET) shows
// for 10 s per cycle (seconds 40..49); afterwards the VESC 1/3 page shows it as
// the latched "LAST OT_FET <age>" row, whose age is derived from the frozen
// time as well (so it stands still during the hold window).
static SharedState demo_state(uint32_t now) {
  SharedState s{};
  const uint32_t sec = now / 1000u;
  const uint32_t cyc = sec % 120u;
  const uint32_t tsec = cyc >= 90u ? sec - (cyc - 90u) : sec;  // frozen during the hold window
  const uint32_t tcyc = tsec % 120u;                           // 0..90 (frozen at 90 while holding)
  const float t = (float)tsec;
  const uint32_t stamp = now ? now : 1u;

  s.vesc.locked_id = 74;
  s.vesc.t.id = 74;
  s.vesc.v_in_ema = 48.0f + 4.0f * sinf(t / 17.0f);      // 44..52 V
  s.vesc.i_in_ema = 27.5f + 32.5f * sinf(t / 5.0f);      // -5..60 A (regen shows "-5.0A")
  s.vesc.i_motor_ema = 60.0f + 60.0f * sinf(t / 7.0f);   // 0..120 A
  s.vesc.t.v_in = s.vesc.v_in_ema;
  s.vesc.t.current_in = s.vesc.i_in_ema;
  s.vesc.t.current_motor = s.vesc.i_motor_ema;
  const float rpm = 1500.0f + 1500.0f * sinf(t / 9.0f);  // 0..3000 mech rpm
  s.vesc.t.erpm = rpm * (VESC_MOTOR_POLES / 2.0f);
  s.vesc.t.duty = rpm / 3300.0f;
  s.vesc.t.temp_fet = 38.0f + 6.0f * sinf(t / 23.0f);
  s.vesc.t.temp_motor = 31.0f + 9.0f * sinf(t / 29.0f);
  s.vesc.t.amp_hours = t / 3600.0f * 12.0f;
  s.vesc.t.amp_hours_charged = t / 3600.0f * 0.4f;
  s.vesc.t.watt_hours = s.vesc.t.amp_hours * 48.0f;
  s.vesc.t.watt_hours_charged = s.vesc.t.amp_hours_charged * 48.0f;
  s.vesc.t.tachometer = (int32_t)(t * 700.0f);
  s.vesc.t.pid_pos = fmodf(t * 3.0f, 360.0f);
  s.vesc.t.adc1 = 1.5f + 1.5f * sinf(t / 7.0f);
  s.vesc.t.adc2 = 2.1f;
  s.vesc.t.adc3 = 0.0f;
  s.vesc.t.ppm = 0.5f + 0.5f * sinf(t / 7.0f);
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) s.vesc.t.t_ms[i] = stamp;
  s.vesc.last_frame_ms = stamp;
  s.vesc.frames_total = tsec * 300u;
  s.vesc.frames_dropped = tsec / 10u;
  s.can.state = CAN_STATE_RUNNING;

  s.vesc_ext.temp_mos1 = s.vesc.t.temp_fet + 1.0f;
  s.vesc_ext.temp_mos2 = s.vesc.t.temp_fet + 2.5f;
  s.vesc_ext.temp_mos3 = s.vesc.t.temp_fet - 0.5f;
  s.vesc_ext.avg_motor_current = s.vesc.i_motor_ema;
  s.vesc_ext.avg_input_current = s.vesc.i_in_ema;
  s.vesc_ext.avg_id = 0.3f;
  s.vesc_ext.avg_iq = s.vesc.i_motor_ema;
  s.vesc_ext.vd = 1.2f;
  s.vesc_ext.vq = s.vesc.t.duty * s.vesc.v_in_ema;
  s.vesc_ext.tacho_abs = (int32_t)(t * 800.0f);
  s.vesc_ext.fault_code = (tcyc >= 40u && tcyc < 50u) ? 5u : 0u;  // OT_FET for 10 s per cycle
  if (tsec >= 40u) {  // latched by can_vesc from the first fault on: age 0..9 s live, then up to 119 s
    s.vesc_ext.last_fault = 5u;
    const uint32_t age_s = tcyc >= 40u ? tcyc - 40u : tcyc + 80u;
    s.vesc_ext.last_fault_ms = now - age_s * 1000u;
    if (s.vesc_ext.last_fault_ms == 0) s.vesc_ext.last_fault_ms = 1u;
  }
  s.vesc_ext.status = 0;
  s.vesc_ext.vesc_id = 74;
  s.vesc_ext.t_ms = stamp;
  s.vesc_ext.polls_sent = tsec;
  s.vesc_ext.replies_ok = tsec - tsec / 50u - tsec / 200u;
  s.vesc_ext.replies_bad = tsec / 200u;
  s.vesc_ext.timeouts = tsec / 50u;

  s.gnss.phase = GNSS_PHASE_RUN;
  s.gnss.baud = 38400;
  s.gnss.prot_ver_x100 = 3410;
  s.gnss.configured = true;
  s.gnss.last_pvt_ms = stamp;
  s.gnss.fix_type = 3;
  s.gnss.fix_ok = true;
  s.gnss.num_sv = (uint8_t)(10 + (int)(2.0f * sinf(t / 13.0f)));  // 8..12
  const float spd = 7.0f + 7.0f * sinf(t / 11.0f);                 // 0..14 in the display unit (kn or km/h)
  s.gnss.gspeed_mm_s = (int32_t)(spd / SPEED_FACTOR);
  s.gnss.sacc_mm_s = 300;
  s.gnss.pdop_x100 = 150;
  s.gnss.lat_e7 = 594370000 + (int32_t)(t * 30.0f);
  s.gnss.lon_e7 = 247536000 + (int32_t)(t * 50.0f);
  s.gnss.hmsl_mm = 5000 + (int32_t)(500.0f * sinf(t / 19.0f));
  s.gnss.height_mm = s.gnss.hmsl_mm + 20000;
  s.gnss.hacc_mm = 2100;
  s.gnss.vacc_mm = 3000;
  s.gnss.head_mot_e5 = (int32_t)(fmodf(t * 3.0f, 360.0f) * 100000.0f);
  s.gnss.year = 2026;
  s.gnss.month = 9;
  s.gnss.day = 6;
  s.gnss.time_valid = true;
  s.gnss.hour = (uint8_t)((tsec / 3600u) % 24u);
  s.gnss.min = (uint8_t)((tsec / 60u) % 60u);
  s.gnss.sec = (uint8_t)(tsec % 60u);
  s.gnss.good_frames = tsec * 2u;

  // Trip integrator: under way the whole time at 3 m/s (5.8 kn / 10.8 km/h) with
  // ~600 W drawn and ~20 W returned, so the trip efficiency sits near 100 Wh/NM
  // (54 Wh/km) while the window efficiency follows the window power (400..800 W).
  // Everything derives from the frozen time, like the rest of the demo.
  s.trip.t_ms = stamp;
  s.trip.run_s = tsec;
  s.trip.moving_s = tsec;
  s.trip.dist_m = t * 3.0f;
  s.trip.wh = t * (600.0f / 3600.0f);
  s.trip.wh_charged = t * (20.0f / 3600.0f);
  s.trip.win_p_avg_w = 600.0f + 200.0f * sinf(t / 5.0f);
  s.trip.win_fill_s = (uint8_t)(tsec < (uint32_t)EFF_WINDOW_S ? tsec : (uint32_t)EFF_WINDOW_S);
  s.trip.win_dist_m = 3.0f * (float)s.trip.win_fill_s;
  s.trip.win_wh = s.trip.win_p_avg_w * ((float)s.trip.win_fill_s / 3600.0f);
  s.trip.eff_now_valid = s.trip.win_dist_m >= (float)EFF_MIN_DIST_M;
  s.trip.eff_avg_valid = s.trip.dist_m >= (float)EFF_MIN_DIST_M;
  if (s.trip.eff_now_valid) s.trip.eff_now = s.trip.win_wh / (s.trip.win_dist_m / EFF_DIST_UNIT_M);
  if (s.trip.eff_avg_valid) s.trip.eff_avg = (s.trip.wh - s.trip.wh_charged) / (s.trip.dist_m / EFF_DIST_UNIT_M);
  s.trip.energy_from_counters = true;
  s.trip.counter_resets = 0;
  return s;
}
#endif

// ---------------------------------------------------------------- task
struct DispCtx {
  OledFrame prev;          // what is on the panel right now
  DisplayStats st;         // counters + flags, mirrored into g_state.disp
  uint8_t screen;          // OledScreen currently selected
  uint32_t last_press_ms;  // last debounced button event (0 = never), for the auto-return timer
#if PIN_BUTTON >= 0
  Button btn;
#endif
};

// Filled by display_start() (boot frame) before the task exists, then owned by the task.
static DispCtx s_ctx;

static void publish_stats(const DisplayStats &st) {
  if (state_lock()) {
    g_state.disp = st;
    state_unlock();
  }
}

[[maybe_unused]] static const char *reset_reason_str(esp_reset_reason_t r) {  // SYS screen
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

// Pushes one frame to the panel and records it as the reference for change detection.
// Returns false when a transfer failed (panel gone): the caller drops to the re-init path.
static bool render(DispCtx &c, const OledFrame &cur, uint32_t now) {
  const uint32_t t0 = state_now_ms();
  u8g2_ClearBuffer(&s_u8g2);
  draw_frame(cur);
  u8g2_SendBuffer(&s_u8g2);
  [[maybe_unused]] const uint32_t dt = (uint32_t)(state_now_ms() - t0);  // log-only

  const esp_err_t err = u8g2_hal_idf_take_last_error();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "OLED: frame transfer failed (%s), re-initialising", esp_err_to_name(err));
    return false;
  }
  c.st.refreshes++;
  c.st.last_change_ms = now;
  c.prev = cur;
  // Stack figure = the calling task's minimum free stack in bytes: the display task from
  // refresh #2 on (#1 is the boot frame drawn from app_main()).
  ESP_LOGD(TAG, "OLED: refresh %lu ms (#%lu, screen %u, stack free min %u B)", (unsigned long)dt,
           (unsigned long)c.st.refreshes, (unsigned)cur.screen, (unsigned)uxTaskGetStackHighWaterMark(nullptr));
  return true;
}

static void restore_contrast(DispCtx &c, const char *why) {
  if (!c.st.dimmed) return;
  if (c.st.init_ok) u8g2_SetContrast(&s_u8g2, (uint8_t)OLED_CONTRAST);
  c.st.dimmed = false;
  ESP_LOGI(TAG, "OLED: contrast restored (%s)", why);
}

// Nothing changed this tick: count it and step the idle dimming.
static void handle_idle(DispCtx &c, uint32_t now) {
  c.st.skipped_unchanged++;
#if OLED_IDLE_DIM_MS > 0
  const uint32_t idle = (uint32_t)(now - c.st.last_change_ms);
  if (!c.st.dimmed && idle >= (uint32_t)OLED_IDLE_DIM_MS) {
    u8g2_SetContrast(&s_u8g2, (uint8_t)OLED_IDLE_CONTRAST);
    c.st.dimmed = true;
    ESP_LOGI(TAG, "OLED: dimmed after %lu s idle", (unsigned long)(idle / 1000u));
  }
#else
  (void)now;
#endif
}

// Selects a screen; returns true if it changed (the caller then renders at once).
[[maybe_unused]] static bool set_screen(DispCtx &c, uint8_t next, const char *why) {  // unused without a button and auto-return
  if (next >= SCREEN_COUNT) next = SCREEN_MAIN;
  if (next == c.screen) return false;
  c.screen = next;
  c.st.screen = next;
  ESP_LOGI(TAG, "display: screen %u (%s), %s", (unsigned)next, oled_screen_name(next), why);
  return true;
}

// Samples the button and applies the auto-return timer. Returns true when the
// screen changed. Any press counts as activity: it restores the contrast and
// restarts the idle timer.
static bool poll_button(DispCtx &c, uint32_t now) {
  bool changed = false;
  (void)c;  // unused when PIN_BUTTON < 0 and SCREEN_AUTO_RETURN_MS == 0
#if PIN_BUTTON >= 0
  const ButtonEvent ev = c.btn.update(button_raw_pressed(), now);
  c.st.button_presses = c.btn.presses;
  if (ev != ButtonEvent::None) {
    c.last_press_ms = now ? now : 1u;
    c.st.last_change_ms = now;
    restore_contrast(c, "button");
    if (ev == ButtonEvent::Short) changed = set_screen(c, (uint8_t)((c.screen + 1u) % SCREEN_COUNT), "short press");
    else changed = set_screen(c, SCREEN_MAIN, "long press");
  }
#endif
#if SCREEN_AUTO_RETURN_MS > 0
  if (c.screen != SCREEN_MAIN && c.last_press_ms != 0 &&
      (uint32_t)(now - c.last_press_ms) >= (uint32_t)SCREEN_AUTO_RETURN_MS) {
    changed = set_screen(c, SCREEN_MAIN, "auto-return") || changed;
  }
#else
  (void)now;
#endif
  return changed;
}

// One frame tick: snapshot -> descriptor -> push if it differs from the panel.
// Returns false when the push failed (panel gone).
static bool frame_tick(DispCtx &c, uint32_t now) {
#if DISPLAY_DEMO
  SharedState s = demo_state(now);
#else
  SharedState s = state_snapshot();
  // Clock AFTER the snapshot (as trip.cpp and the supervisor do): a producer that
  // stamps between the loop's clock read and the copy would otherwise sit in the
  // future, and the unsigned age test would paint its values "--" for one frame.
  now = state_now_ms();
#endif
  s.disp = c.st;  // the SYS screen shows this task's own counters (the published copy lags one poll)
  OledSysInfo info;
  info.uptime_s = now / 1000u;
  info.heap_free = esp_get_free_heap_size();
  info.heap_min = esp_get_minimum_free_heap_size();
  info.reset_reason = reset_reason_str(esp_reset_reason());

  OledFrame cur;
  oled_build_frame(s, now, c.screen, info, cur);
  if (memcmp(&cur, &c.prev, sizeof cur) == 0) {
    handle_idle(c, now);
    return true;
  }
  restore_contrast(c, "value changed");
  return render(c, cur, now);
}

// The panel stopped answering (failed transfer or probe): back to the retry path, which
// re-initialises and redraws. The button keeps working meanwhile.
static void panel_lost(DispCtx &c, uint32_t now, uint32_t &next_retry_ms) {
  c.st.init_ok = false;
  c.prev = OledFrame{};  // whatever is on the panel now is not what we drew
  next_retry_ms = now;   // first re-init attempt on the next tick
  publish_stats(c.st);
}

static void display_task(void *) {
  DispCtx &c = s_ctx;
  uint32_t next_retry_ms = state_now_ms() + OLED_INIT_RETRY_MS;
  uint32_t next_frame_ms = state_now_ms();  // first frame at once
  [[maybe_unused]] uint32_t retries = 0;    // log-only
  hb_touch(hb_disp);

  ESP_LOGI(TAG, "OLED: task started (poll %u ms, frame %u ms, %u screens, button %s%d, auto-return %lu ms, demo=%d)",
           (unsigned)OLED_BUTTON_POLL_MS, (unsigned)OLED_PERIOD_MS, (unsigned)SCREEN_COUNT,
           PIN_BUTTON >= 0 ? "GPIO" : "none ", (int)PIN_BUTTON, (unsigned long)SCREEN_AUTO_RETURN_MS, (int)DISPLAY_DEMO);

  for (;;) {
    const uint32_t now = state_now_ms();
    hb_touch(hb_disp);

    // The button works whether or not a panel answers (presses are counted either way).
    const bool screen_changed = poll_button(c, now);

    // No panel yet: retry the init every OLED_INIT_RETRY_MS, keep the heartbeat alive.
    if (!c.st.init_ok) {
      if ((int32_t)(now - next_retry_ms) >= 0) {
        next_retry_ms = now + OLED_INIT_RETRY_MS;
        ++retries;
        const esp_err_t err = oled_init();
        if (err == ESP_OK) {
          log_init_ok();
          c.st.init_ok = true;
          c.st.dimmed = false;  // oled_init() programmed OLED_CONTRAST
          OledFrame boot;
          oled_boot_frame(boot);
          if (render(c, boot, state_now_ms())) {  // the panel RAM is garbage after init: show something now
            next_frame_ms = state_now_ms();       // and the live frame on the next tick
          } else {
            panel_lost(c, now, next_retry_ms);
          }
        } else {
          ESP_LOGW(TAG, "OLED: still no response at 0x%02X (retry %lu, %s)", (unsigned)OLED_I2C_ADDR,
                   (unsigned long)retries, esp_err_to_name(err));
        }
      }
      publish_stats(c.st);
      vTaskDelay(pdMS_TO_TICKS(OLED_BUTTON_POLL_MS));
      continue;
    }

    // Panel present: re-probe it every OLED_INIT_RETRY_MS (one zero-length
    // transaction, ~30 us). A frame whose transfer fails is caught in render();
    // the probe additionally catches a panel that vanished while nothing changed
    // on screen. On a miss fall back to the retry path above.
    if ((int32_t)(now - next_retry_ms) >= 0) {
      next_retry_ms = now + OLED_INIT_RETRY_MS;
      const esp_err_t err = oled_probe();
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "OLED: lost contact at 0x%02X (%s), re-initialising", (unsigned)OLED_I2C_ADDR, esp_err_to_name(err));
        panel_lost(c, now, next_retry_ms);
        vTaskDelay(pdMS_TO_TICKS(OLED_BUTTON_POLL_MS));
        continue;
      }
    }

    // Frame tick every OLED_PERIOD_MS, or right away after a screen change.
    if (screen_changed || (int32_t)(now - next_frame_ms) >= 0) {
      next_frame_ms = now + OLED_PERIOD_MS;
      if (!frame_tick(c, now)) {
        panel_lost(c, now, next_retry_ms);
        vTaskDelay(pdMS_TO_TICKS(OLED_BUTTON_POLL_MS));
        continue;
      }
    }
    publish_stats(c.st);  // small memcpy under the lock, never while drawing

    // Fixed short delay rather than vTaskDelayUntil: after a slow I2C transfer
    // there is no catch-up burst that would starve the supervisor (priority 1).
    vTaskDelay(pdMS_TO_TICKS(OLED_BUTTON_POLL_MS));
  }
}

// ---------------------------------------------------------------- public
bool display_start() {
  DispCtx &c = s_ctx;
  c = DispCtx{};  // value-initialised (Button has default member initialisers, so no memset)
  c.screen = SCREEN_MAIN;

#if PIN_BUTTON >= 0
  gpio_config_t btn = {};
  btn.pin_bit_mask = 1ULL << PIN_BUTTON;
  btn.mode = GPIO_MODE_INPUT;
  btn.pull_up_en = BUTTON_ACTIVE_LOW ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  btn.pull_down_en = BUTTON_ACTIVE_LOW ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
  gpio_config(&btn);
  ESP_LOGI(TAG, "OLED: button on GPIO%d (active %s, debounce %u ms, long press %u ms, auto-return %lu ms)", (int)PIN_BUTTON,
           BUTTON_ACTIVE_LOW ? "low, pull-up" : "high, pull-down", (unsigned)BUTTON_DEBOUNCE_MS,
           (unsigned)BUTTON_LONG_PRESS_MS, (unsigned long)SCREEN_AUTO_RETURN_MS);
#else
  ESP_LOGI(TAG, "OLED: no button (PIN_BUTTON -1): main screen only");
#endif

  const esp_err_t err = oled_init();
  if (err == ESP_OK) {
    log_init_ok();
    c.st.init_ok = true;
    OledFrame boot;  // main screen, all "--", clock "--:--", fix line BOOT, CAN line = firmware version
    oled_boot_frame(boot);
    if (!render(c, boot, state_now_ms())) c.st.init_ok = false;  // the task retries
  } else {
    ESP_LOGE(TAG, "OLED: no response at 0x%02X (check wiring/address jumper): %s", (unsigned)OLED_I2C_ADDR,
             esp_err_to_name(err));
  }
  publish_stats(c.st);

  // The task always runs: it retries the init and keeps hb_disp fresh either way.
  if (xTaskCreate(display_task, "disp", OLED_TASK_STACK, nullptr, TASK_PRIO_DISP, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "OLED: xTaskCreate failed");
    return false;
  }
  return c.st.init_ok;
}
