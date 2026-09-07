// Unity tests (pio test -e native) for the pure OLED screen builders in
// include/display_strings_oled.h: main screen cells, clock arithmetic, the six
// grid screens (efficiency, VESC 1-3, GNSS, SYS), the live/latched fault row,
// freshness rules, character budgets, determinism. Fault names come from
// vesc_getvalues.h (checked here as used).
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; includes only Arduino-free headers.
#include <unity.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "display_strings_oled.h"

void setUp() {}
void tearDown() {}

using namespace oled_layout;

// ---------------------------------------------------------------- fixtures
static const uint32_t NOW = 100000;

// Speed expectations depend on the configured unit (config.h SPEED_UNIT_KNOTS, overridable with -D).
#if SPEED_UNIT_KNOTS
static const char *const kUnit = "kn";
static const char *const kSpeed5144 = "10.0";  // 5144 mm/s * 0.00194384 = 9.999 kn
static const char *const kSpeed60000 = "117";  // 60000 mm/s = 116.6 kn -> no decimals at >= 100
static const int32_t kBelowMinMmS = 100;       // 0.19 kn < SPEED_MIN_SHOW (0.3)
#else
static const char *const kUnit = "km/h";
static const char *const kSpeed5144 = "18.5";  // 5144 mm/s * 0.0036 = 18.52 km/h
static const char *const kSpeed60000 = "216";  // 60000 mm/s = 216 km/h
static const int32_t kBelowMinMmS = 50;        // 0.18 km/h < SPEED_MIN_SHOW (0.3)
#endif

// Electrical rpm for a mechanical rpm with the configured pole count (mech = erpm / (poles/2)).
static float erpm_for(float mech_rpm) { return mech_rpm * (VESC_MOTOR_POLES / 2.0f); }

// VESC 48.2 V, 12.4 A in, 35 A motor, 2350 rpm, duty 45 %, Tfet 45.3, Tmot 32.1, Ah 1.234 (+0.012 charged),
// Wh 56.7 (+3.4), tacho 12345, PID 12.3, ADC 1.23/2.10/0.00, PPM 0.52; id 74; all STATUS fresh; CAN running.
// Polled: Tmos 45.1/46.2/44.0 (only the first is displayed), averaged Iin 12.4 / Imot 35.0, Id 0.3, Iq 34.9,
// Vd 1.23, Vq 23.45, tacho abs 23456, no fault, status OK.
static void fill_vesc(SharedState &s, uint32_t t) {
  s.vesc.v_in_ema = 48.2f;
  s.vesc.i_in_ema = 12.4f;
  s.vesc.i_motor_ema = 35.0f;
  s.vesc.locked_id = 74;
  s.vesc.t.id = 74;
  s.vesc.t.erpm = erpm_for(2350.0f);
  s.vesc.t.duty = 0.45f;
  s.vesc.t.amp_hours = 1.234f;
  s.vesc.t.amp_hours_charged = 0.012f;
  s.vesc.t.watt_hours = 56.7f;
  s.vesc.t.watt_hours_charged = 3.4f;
  s.vesc.t.temp_fet = 45.3f;
  s.vesc.t.temp_motor = 32.1f;
  s.vesc.t.current_in = 12.4f;
  s.vesc.t.pid_pos = 12.3f;
  s.vesc.t.tachometer = 12345;
  s.vesc.t.v_in = 48.2f;
  s.vesc.t.adc1 = 1.23f;
  s.vesc.t.adc2 = 2.10f;
  s.vesc.t.adc3 = 0.0f;
  s.vesc.t.ppm = 0.52f;
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) s.vesc.t.t_ms[i] = t;
  s.vesc.frames_total = 12345;
  s.vesc.frames_other_id = 0;
  s.vesc.frames_dropped = 3;
  s.vesc.last_frame_ms = t;
  s.vesc_ext.temp_mos1 = 45.1f;
  s.vesc_ext.temp_mos2 = 46.2f;
  s.vesc_ext.temp_mos3 = 44.0f;
  s.vesc_ext.avg_motor_current = 35.0f;
  s.vesc_ext.avg_input_current = 12.4f;
  s.vesc_ext.avg_id = 0.3f;
  s.vesc_ext.avg_iq = 34.9f;
  s.vesc_ext.vd = 1.23f;
  s.vesc_ext.vq = 23.45f;
  s.vesc_ext.tacho_abs = 23456;
  s.vesc_ext.fault_code = 0;
  s.vesc_ext.status = 0;
  s.vesc_ext.vesc_id = 74;
  s.vesc_ext.t_ms = t;
  s.vesc_ext.polls_sent = 123;
  s.vesc_ext.replies_ok = 120;
  s.vesc_ext.replies_bad = 0;
  s.vesc_ext.timeouts = 3;
  s.can.state = CAN_STATE_RUNNING;
}

// GNSS running at 38400 / PROTVER 34.10, 3D fix, 9 sats, 5144 mm/s (= 10.0 kn), sAcc 0.3 m/s, pDOP 1.5,
// 59.4370123 N 24.7536789 W, 12.3 m MSL, hAcc 2.1 m, vAcc 3.0 m, heading 123.4, 2026-09-06 12:34:56 UTC.
static void fill_gnss(SharedState &s, uint32_t t) {
  s.gnss.phase = GNSS_PHASE_RUN;
  s.gnss.baud = 38400;
  s.gnss.prot_ver_x100 = 3410;
  s.gnss.configured = true;
  s.gnss.last_pvt_ms = t;
  s.gnss.fix_type = 3;
  s.gnss.fix_ok = true;
  s.gnss.num_sv = 9;
  s.gnss.gspeed_mm_s = 5144;
  s.gnss.sacc_mm_s = 300;
  s.gnss.pdop_x100 = 150;
  s.gnss.lat_e7 = 594370123;
  s.gnss.lon_e7 = -247536789;
  s.gnss.hmsl_mm = 12300;
  s.gnss.hacc_mm = 2100;
  s.gnss.vacc_mm = 3000;
  s.gnss.head_mot_e5 = 12340000;
  s.gnss.year = 2026;
  s.gnss.month = 9;
  s.gnss.day = 6;
  s.gnss.hour = 12;
  s.gnss.min = 34;
  s.gnss.sec = 56;
  s.gnss.time_valid = true;
  s.gnss.good_frames = 1234;
  s.gnss.bad_frames = 0;
  s.gnss.redetects = 0;
}

// Trip integrator running: 1h23m20s since start, 1h02m of it moving, 12.34 distance units (NM or km, so
// "dist 12.34" in every build) = a mean of 11.94 units/h while moving, 1234.5 Wh drawn / 4.5 Wh returned,
// window 598 W over the full EFF_WINDOW_S, efficiency now 123.4 / trip 98.5 Wh per unit, energy from STATUS_3.
static void fill_trip(SharedState &s, uint32_t t) {
  s.trip.t_ms = t;
  s.trip.run_s = 5000;
  s.trip.moving_s = 3720;
  s.trip.dist_m = 12.34f * EFF_DIST_UNIT_M;
  s.trip.wh = 1234.5f;
  s.trip.wh_charged = 4.5f;
  s.trip.win_dist_m = 30.0f;
  s.trip.win_wh = 1.66f;
  s.trip.win_p_avg_w = 598.0f;
  s.trip.eff_now = 123.4f;
  s.trip.eff_avg = 98.5f;
  s.trip.eff_now_valid = true;
  s.trip.eff_avg_valid = true;
  s.trip.energy_from_counters = true;
  s.trip.win_fill_s = (uint8_t)EFF_WINDOW_S;
  s.trip.counter_resets = 0;
}

static SharedState make_state(uint32_t t) {
  SharedState s;
  memset(&s, 0, sizeof s);
  s.vesc.locked_id = -1;
  s.can.state = CAN_STATE_UNINSTALLED;
  s.gnss.prot_ver_x100 = -1;
  fill_vesc(s, t);
  fill_gnss(s, t);
  fill_trip(s, t);
  s.disp.button_presses = 12;
  return s;
}

// "<label> <number> <EFF_UNIT_STR>" as the efficiency rows print it ("now 123 Wh/NM" / "now 123 Wh/km").
static const char *eff_row(char *buf, size_t n, const char *label, const char *num) {
  snprintf(buf, n, "%s %s %s", label, num, EFF_UNIT_STR);
  return buf;
}

// A state in which nothing was ever received (fresh boot, no peripherals).
static SharedState never_state() {
  SharedState s;
  memset(&s, 0, sizeof s);
  s.vesc.locked_id = -1;
  s.can.state = CAN_STATE_UNINSTALLED;
  s.gnss.prot_ver_x100 = -1;
  return s;
}

static const OledSysInfo kInfo = {5000, 250 * 1024, 240 * 1024, "POWERON"};  // 1h23m20s up

static OledMain build_main(const SharedState &s, uint32_t now) {
  OledMain m;
  memset(&m, 0xAA, sizeof m);  // prove the builder fully initialises the struct
  oled_build_main(s, now, m);
  return m;
}

static OledGrid build_grid(uint8_t screen, const SharedState &s, uint32_t now, const OledSysInfo &info = kInfo) {
  OledFrame f;
  memset(&f, 0xAA, sizeof f);
  oled_build_frame(s, now, screen, info, f);
  TEST_ASSERT_EQUAL_UINT8(screen, f.screen);
  return f.grid;
}

// ---- budget assertions (the renderer lays out exactly these many 6 px cells) ----
static void assert_main_lengths(const OledMain &m) {
  TEST_ASSERT_TRUE(strnlen(m.speed, sizeof m.speed) <= (size_t)kSpeedChars);
  TEST_ASSERT_TRUE(strnlen(m.unit, sizeof m.unit) <= (size_t)kUnitChars);
  TEST_ASSERT_TRUE(strnlen(m.clock, sizeof m.clock) == (size_t)kClockChars);
  TEST_ASSERT_TRUE(strnlen(m.fix, sizeof m.fix) <= (size_t)kFixChars);
  TEST_ASSERT_TRUE(strnlen(m.can, sizeof m.can) <= (size_t)kCanChars);
  for (int i = 0; i < 8; ++i) {
    TEST_ASSERT_TRUE(strnlen(m.cells[i].label, sizeof m.cells[i].label) <= (size_t)kCellLabelChars);
    TEST_ASSERT_TRUE(strnlen(m.cells[i].label, sizeof m.cells[i].label) >= 2);
    TEST_ASSERT_TRUE(strnlen(m.cells[i].value, sizeof m.cells[i].value) <= (size_t)kCellValueChars);
    TEST_ASSERT_TRUE(strnlen(m.cells[i].value, sizeof m.cells[i].value) >= 1);
  }
  // the speed cell is drawn with a digits-only font: never a letter
  for (const char *p = m.speed; *p; ++p) TEST_ASSERT_TRUE((*p >= '0' && *p <= '9') || *p == '.' || *p == '-');
}

static void assert_grid_lengths(const OledGrid &g) {
  TEST_ASSERT_TRUE(strnlen(g.title, sizeof g.title) <= (size_t)kTitleChars);
  TEST_ASSERT_TRUE(strnlen(g.title, sizeof g.title) >= 3);
  TEST_ASSERT_TRUE(strnlen(g.page, sizeof g.page) <= (size_t)kPageChars);
  TEST_ASSERT_EQUAL_UINT8(kGridRows, g.nrows);
  for (int r = 0; r < kGridRows; ++r) {
    const size_t l = strnlen(g.rows[r], sizeof g.rows[r]);
    TEST_ASSERT_TRUE(l <= (size_t)kGridChars);
    TEST_ASSERT_TRUE(l >= 1);
    TEST_ASSERT_TRUE(l == 0 || g.rows[r][l - 1] != ' ');  // no trailing padding (memcmp stability)
  }
  // the last grid row sits on the panel's last pixel row: its labels carry no descenders
  for (const char *p = g.rows[kGridRows - 1]; *p; ++p) {
    if (*p >= '0' && *p <= '9') continue;
    TEST_ASSERT_TRUE(*p != 'g' && *p != 'j' && *p != 'p' && *p != 'q' && *p != 'y' && *p != ',');
  }
}

// Every byte after the terminating NUL of a string is zero (memset before formatting).
static void assert_zero_tail(const char *buf, size_t n) {
  for (size_t i = strnlen(buf, n) + 1; i < n; ++i) TEST_ASSERT_EQUAL_HEX8(0, (uint8_t)buf[i]);
}

// ---------------------------------------------------------------- main screen: sample
static void test_main_sample_state() {
  OledMain m = build_main(make_state(NOW), NOW);
  TEST_ASSERT_EQUAL_STRING(kSpeed5144, m.speed);
  TEST_ASSERT_EQUAL_STRING(kUnit, m.unit);
  TEST_ASSERT_EQUAL_STRING(SPEED_UNIT_STR, m.unit);
  TEST_ASSERT_EQUAL_STRING("3D 9sv", m.fix);
  TEST_ASSERT_EQUAL_STRING("VESC 74", m.can);
  TEST_ASSERT_EQUAL_STRING("BV", m.cells[0].label);
  TEST_ASSERT_EQUAL_STRING("48.2v", m.cells[0].value);
  TEST_ASSERT_EQUAL_STRING("BA", m.cells[1].label);
  TEST_ASSERT_EQUAL_STRING("12.4A", m.cells[1].value);
  TEST_ASSERT_EQUAL_STRING("MA", m.cells[2].label);
  TEST_ASSERT_EQUAL_STRING("35A", m.cells[2].value);
  TEST_ASSERT_EQUAL_STRING("PW", m.cells[3].label);
  TEST_ASSERT_EQUAL_STRING("598W", m.cells[3].value);  // 48.2 * 12.4 = 597.68
  TEST_ASSERT_EQUAL_STRING("TF", m.cells[4].label);
  TEST_ASSERT_EQUAL_STRING("45\260", m.cells[4].value);  // 45.3 -> "45" + degree sign (single byte 0xB0)
  TEST_ASSERT_EQUAL_STRING("TM", m.cells[5].label);
  TEST_ASSERT_EQUAL_STRING("32\260", m.cells[5].value);
  TEST_ASSERT_EQUAL_STRING("RPM", m.cells[6].label);
  TEST_ASSERT_EQUAL_STRING("2350", m.cells[6].value);  // erpm / (poles/2)
  TEST_ASSERT_EQUAL_STRING("WH", m.cells[7].label);
  TEST_ASSERT_EQUAL_STRING("56.7", m.cells[7].value);  // watt-hours drawn (STATUS_3)
  assert_main_lengths(m);
}

// ---------------------------------------------------------------- main screen: clock
static void test_clock_from_gnss_with_configured_offset() {
  SharedState s = make_state(NOW);
  OledMain m = build_main(s, NOW);
  const OledLocalTime lt = oled_utc_to_local(12, 34, (int32_t)TIME_UTC_OFFSET_MIN);
  char exp[8];
  snprintf(exp, sizeof exp, "%02u:%02u", (unsigned)lt.hour, (unsigned)lt.min);
  TEST_ASSERT_EQUAL_STRING(exp, m.clock);
#if TIME_UTC_OFFSET_MIN == 0
  TEST_ASSERT_EQUAL_STRING("12:34", m.clock);
#endif
  // the clock does not care about the fix type: a time-only fix still has a valid UTC
  s.gnss.fix_type = 5;
  s.gnss.fix_ok = false;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING(exp, m.clock);
  TEST_ASSERT_EQUAL_STRING("--", m.speed);
}

static void test_clock_hidden_without_valid_time_or_fresh_epoch() {
  SharedState s = make_state(NOW);
  s.gnss.time_valid = false;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--:--", m.clock);
  s.gnss.time_valid = true;
  s.gnss.last_pvt_ms = NOW - GNSS_STALE_MS - 1;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--:--", m.clock);
  TEST_ASSERT_EQUAL_STRING("NODATA", m.fix);
  s.gnss.last_pvt_ms = NOW - GNSS_STALE_MS;  // exactly at the limit: still live
  m = build_main(s, NOW);
  TEST_ASSERT_TRUE(strcmp("--:--", m.clock) != 0);
}

static void expect_local(uint8_t h, uint8_t m, int32_t off, unsigned eh, unsigned em, int shift) {
  const OledLocalTime lt = oled_utc_to_local(h, m, off);
  char got[32], exp[32];
  snprintf(got, sizeof got, "%02u:%02u shift %d", (unsigned)lt.hour, (unsigned)lt.min, (int)lt.day_shift);
  snprintf(exp, sizeof exp, "%02u:%02u shift %d", eh, em, shift);
  TEST_ASSERT_EQUAL_STRING(exp, got);
}

static void test_utc_to_local_offsets_and_day_wrap() {
  expect_local(12, 34, 0, 12, 34, 0);
  expect_local(12, 34, 120, 14, 34, 0);     // UTC+2
  expect_local(23, 50, 120, 1, 50, 1);      // wraps into the next day
  expect_local(12, 34, -600, 2, 34, 0);     // UTC-10
  expect_local(0, 15, -60, 23, 15, -1);     // wraps into the previous day
  expect_local(1, 0, -90, 23, 30, -1);      // half-hour offset
  expect_local(23, 59, 1, 0, 0, 1);
  expect_local(0, 0, 0, 0, 0, 0);
  expect_local(12, 0, 1440, 12, 0, 1);      // a full day forward
  expect_local(12, 0, -1440, 12, 0, -1);    // a full day back
  expect_local(5, 30, 840, 19, 30, 0);      // UTC+14 (Line Islands)
  expect_local(5, 30, -720, 17, 30, -1);    // UTC-12
  expect_local(22, 0, 345, 3, 45, 1);       // UTC+5:45 (Nepal)
}

// ---------------------------------------------------------------- main screen: cells
static void test_battery_current_signed_one_decimal() {
  SharedState s = make_state(NOW);
  s.vesc.i_in_ema = -12.4f;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-12.4A", m.cells[1].value);  // 6 chars: the decimal survives the sign
  TEST_ASSERT_EQUAL_STRING("-598W", m.cells[3].value);   // regen
  s.vesc.i_in_ema = -123.4f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-123A", m.cells[1].value);   // "-123.4" would be 6 + 'A'
  s.vesc.i_in_ema = 123.4f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("123.4A", m.cells[1].value);
  s.vesc.i_in_ema = 9.96f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("10.0A", m.cells[1].value);
  s.vesc.i_in_ema = 99.96f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("100.0A", m.cells[1].value);
  s.vesc.i_in_ema = 1234.6f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("1235A", m.cells[1].value);
}

static void test_voltage_cell() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 100.4f;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("100.4v", m.cells[0].value);
  s.vesc.v_in_ema = 3.96f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("4.0v", m.cells[0].value);
  s.vesc.v_in_ema = 1.0e12f;  // absurd: saturates instead of clipping to a plausible number
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("MAXv", m.cells[0].value);
  s.vesc.v_in_ema = -1.0e12f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-MAXv", m.cells[0].value);
}

static void test_motor_current_whole_amps_and_kilo() {
  SharedState s = make_state(NOW);
  s.vesc.i_motor_ema = -35.4f;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-35A", m.cells[2].value);
  s.vesc.i_motor_ema = 1234.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("1234A", m.cells[2].value);
  s.vesc.i_motor_ema = -999.6f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-1000A", m.cells[2].value);
  s.vesc.i_motor_ema = 123456.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("123kA", m.cells[2].value);  // "123.5k" would not fit 5 numeric chars
}

static void test_power_watts_and_kilowatts() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 48.0f;
  s.vesc.i_in_ema = -10.0f;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-480W", m.cells[3].value);
  s.vesc.i_in_ema = -100.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-4800W", m.cells[3].value);
  s.vesc.v_in_ema = 50.0f;
  s.vesc.i_in_ema = 60.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3000W", m.cells[3].value);
  s.vesc.i_in_ema = 246.9f;  // 12345 W -> kilowatts above 9999 W
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("12.3kW", m.cells[3].value);
  s.vesc.v_in_ema = 100.0f;
  s.vesc.i_in_ema = 999.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("99.9kW", m.cells[3].value);
  s.vesc.i_in_ema = 9999.0f;  // 999900 W
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("1000kW", m.cells[3].value);
  s.vesc.i_in_ema = -123.4f;  // -12340 W
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-12kW", m.cells[3].value);
}

static void test_negative_zero_is_normalised() {
  SharedState s = make_state(NOW);
  s.vesc.i_in_ema = -0.01f;
  s.vesc.i_motor_ema = -0.4f;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0A", m.cells[1].value);
  TEST_ASSERT_EQUAL_STRING("0A", m.cells[2].value);
  TEST_ASSERT_EQUAL_STRING("0W", m.cells[3].value);  // 48.2 * -0.01 = -0.48 -> "-0" -> "0"
}

static void test_temperature_cells() {
  SharedState s = make_state(NOW);
  s.vesc.t.temp_fet = -12.6f;
  s.vesc.t.temp_motor = 199.5f;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-13\260", m.cells[4].value);
  TEST_ASSERT_EQUAL_STRING("200\260", m.cells[5].value);
  s.vesc.t.temp_fet = 250.0f;   // implausible MOSFET reading -> hidden
  s.vesc.t.temp_motor = 250.0f; // implausible motor reading = no NTC wired
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[4].value);
  TEST_ASSERT_EQUAL_STRING("n/a", m.cells[5].value);
  s.vesc.t.temp_motor = -273.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("n/a", m.cells[5].value);
  s.vesc.t.temp_motor = 32.1f;
  s.vesc.t.t_ms[VESC_IDX_STATUS_4] = NOW - VESC_STALE_R1_MS - 1;  // stale beats "n/a"
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[4].value);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[5].value);
}

static void test_rpm_cell() {
  SharedState s = make_state(NOW);
  s.vesc.t.erpm = erpm_for(-2350.0f);
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-2350", m.cells[6].value);
  s.vesc.t.erpm = erpm_for(12345.0f);
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("12.3k", m.cells[6].value);  // thousands above 9999
  s.vesc.t.erpm = erpm_for(-12345.0f);
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-12.3k", m.cells[6].value);
  s.vesc.t.erpm = erpm_for(9999.0f);
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("9999", m.cells[6].value);
  s.vesc.t.erpm = erpm_for(0.4f);
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0", m.cells[6].value);
  s.vesc.t.t_ms[VESC_IDX_STATUS_1] = 0;  // never received STATUS_1
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[6].value);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[2].value);
  TEST_ASSERT_EQUAL_STRING("VESC 74", m.can);  // STATUS_5 still fresh
}

// WH cell: one decimal below 1000, whole below 10000, thousands above; STATUS_3 (Rate 2) freshness.
static void test_watt_hours_cell_formats_and_uses_rate2_window() {
  SharedState s = make_state(NOW);
  s.vesc.t.watt_hours = 123.4f;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("123.4", m.cells[7].value);
  s.vesc.t.watt_hours = 999.94f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("999.9", m.cells[7].value);
  s.vesc.t.watt_hours = 999.96f;  // would round to "1000.0": whole watt-hours from here
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("1000", m.cells[7].value);
  s.vesc.t.watt_hours = 1234.6f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("1235", m.cells[7].value);
  s.vesc.t.watt_hours = 9999.4f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("9999", m.cells[7].value);
  s.vesc.t.watt_hours = 9999.6f;  // would round to "10000": thousands from here
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("10.0k", m.cells[7].value);
  s.vesc.t.watt_hours = 12345.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("12.3k", m.cells[7].value);
  s.vesc.t.watt_hours = 123456.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("123.5k", m.cells[7].value);  // 6 chars: the decimal still fits (like the RPM cell)
  s.vesc.t.watt_hours = 1234567.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("1235k", m.cells[7].value);
  s.vesc.t.watt_hours = 0.0f;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", m.cells[7].value);
  // the cell reads STATUS_3, not STATUS_2: a stale STATUS_2 leaves it alone ...
  s.vesc.t.t_ms[VESC_IDX_STATUS_2] = NOW - VESC_STALE_R2_MS - 1;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", m.cells[7].value);
  // ... and STATUS_3 is a Rate-2 message: older than R1 but within R2 is still fresh
  s.vesc.t.t_ms[VESC_IDX_STATUS_3] = NOW - VESC_STALE_R1_MS - 1;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", m.cells[7].value);
  s.vesc.t.t_ms[VESC_IDX_STATUS_3] = NOW - VESC_STALE_R2_MS;  // exactly at the R2 limit: fresh
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", m.cells[7].value);
  s.vesc.t.t_ms[VESC_IDX_STATUS_3] = NOW - VESC_STALE_R2_MS - 1;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[7].value);
  TEST_ASSERT_EQUAL_STRING("2350", m.cells[6].value);  // the neighbouring cells do not care about STATUS_3
}

// ---------------------------------------------------------------- main screen: freshness
static void test_status5_stale_hides_voltage_and_power_only() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_5] = NOW - VESC_STALE_R1_MS - 1;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[0].value);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[3].value);
  TEST_ASSERT_EQUAL_STRING("12.4A", m.cells[1].value);  // STATUS_4 still fresh
  TEST_ASSERT_EQUAL_STRING("35A", m.cells[2].value);
  TEST_ASSERT_EQUAL_STRING("VESC 74", m.can);  // STATUS_1 still fresh
}

static void test_status4_stale_hides_current_power_and_temps() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_4] = NOW - VESC_STALE_R1_MS - 1;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[1].value);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[3].value);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[4].value);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[5].value);
  TEST_ASSERT_EQUAL_STRING("48.2v", m.cells[0].value);
  TEST_ASSERT_EQUAL_STRING("2350", m.cells[6].value);
}

static void test_exactly_at_stale_limit_is_fresh() {
  SharedState s = make_state(NOW - VESC_STALE_R1_MS);  // age == limit -> still fresh (<=)
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("48.2v", m.cells[0].value);
  TEST_ASSERT_EQUAL_STRING("35A", m.cells[2].value);
}

static void test_wraparound_freshness() {
  SharedState s = make_state(0xFFFFFF00u);  // stamped just before millis() wraps
  OledMain m = build_main(s, 0x00000100u);   // 512 ms later, after the wrap
  TEST_ASSERT_EQUAL_STRING("48.2v", m.cells[0].value);
  TEST_ASSERT_EQUAL_STRING(kSpeed5144, m.speed);
  TEST_ASSERT_TRUE(strcmp("--:--", m.clock) != 0);
}

static void test_future_timestamp_is_stale_not_fresh() {
  SharedState s = make_state(NOW + 10);  // producer clock ahead of the display's "now"
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.cells[0].value);  // unsigned age ~4e9 -> stale, never a crash or a lie
  TEST_ASSERT_EQUAL_STRING("NODATA", m.fix);
  TEST_ASSERT_EQUAL_STRING("--:--", m.clock);
}

// ---------------------------------------------------------------- main screen: speed / fix
static void test_speed_rules() {
  SharedState s = make_state(NOW);
  s.gnss.gspeed_mm_s = 60000;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING(kSpeed60000, m.speed);  // >= 100: no decimals
  s.gnss.gspeed_mm_s = kBelowMinMmS;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", m.speed);  // drift at rest hidden
  s.gnss.gspeed_mm_s = -5144;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", m.speed);  // negative oddity clamped, never "-10.0"
  s.gnss.gspeed_mm_s = 5144;
  s.gnss.sacc_mm_s = 3000;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.speed);  // speed accuracy too poor
  TEST_ASSERT_EQUAL_STRING("3D 9sv", m.fix);  // the fix line still reports the receiver state
  s.gnss.sacc_mm_s = 300;
  s.gnss.fix_ok = false;  // gnssFixOK cleared with fixType 3
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.speed);
  s.gnss.fix_ok = true;
  s.gnss.gspeed_mm_s = (int32_t)(10000.0f / SPEED_FACTOR);  // 10000 in the display unit: implausible
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.speed);  // the big font has no letters for "10k"/"MAX"
  s.gnss.gspeed_mm_s = 0x7FFFFFFF;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.speed);
  s.gnss.gspeed_mm_s = (int32_t)(999.4f / SPEED_FACTOR);
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("999", m.speed);
}

static void test_fix_line() {
  SharedState s = make_state(NOW);
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D 9sv", m.fix);  // %2u pads a single digit
  s.gnss.num_sv = 12;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D12sv", m.fix);
  s.gnss.fix_type = 4;  // GNSS + dead reckoning counts as 3D
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D12sv", m.fix);
  s.gnss.fix_type = 2;
  s.gnss.num_sv = 7;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("2D 7sv", m.fix);
  TEST_ASSERT_EQUAL_STRING(kSpeed5144, m.speed);  // fix_ok true: speed shown
  s.gnss.fix_type = 5;  // time only
  s.gnss.fix_ok = false;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("NOFIX", m.fix);
  s.gnss.fix_type = 0;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("NOFIX", m.fix);
  s.gnss.fix_type = 3;
  s.gnss.fix_ok = true;
  s.gnss.num_sv = 255;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D99sv", m.fix);  // capped so the 6-char budget holds
  s.gnss.last_pvt_ms = 0;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("NOGNSS", m.fix);
  TEST_ASSERT_EQUAL_STRING("--", m.speed);
  TEST_ASSERT_EQUAL_STRING("--:--", m.clock);
}

// ---------------------------------------------------------------- main screen: CAN line
static void test_can_states() {
  SharedState s = make_state(NOW);
  OledMain m;
  s.can.state = CAN_STATE_STOPPED;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", m.can);
  s.can.state = CAN_STATE_BUS_OFF;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", m.can);
  TEST_ASSERT_EQUAL_STRING("48.2v", m.cells[0].value);  // driver state and freshness are independent
  s.can.state = CAN_STATE_UNINSTALLED;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", m.can);
  s.can.state = 12345;  // unknown driver state
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", m.can);
  s.can.state = CAN_STATE_RECOVERING;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN RECV", m.can);
  s.can.state = CAN_STATE_RUNNING;
  s.vesc.locked_id = 254;
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 254", m.can);  // exactly 8
}

static void test_can_running_without_fresh_status_is_idle() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_1] = NOW - VESC_STALE_R1_MS - 1;
  s.vesc.t.t_ms[VESC_IDX_STATUS_5] = NOW - VESC_STALE_R1_MS - 1;
  OledMain m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN idle", m.can);
  TEST_ASSERT_EQUAL_STRING("12.4A", m.cells[1].value);  // STATUS_4 alone does not count as "VESC seen"
  s.vesc.t.t_ms[VESC_IDX_STATUS_5] = NOW;                // either STATUS_1 or STATUS_5 fresh -> VESC shown
  m = build_main(s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 74", m.can);
}

// ---------------------------------------------------------------- fault names / screen names
// The names come from vesc_getvalues.h (single source shared with the CAN task's
// log lines); this only pins what the fault row relies on: the mc_fault_code
// order of vedderb/bldc datatypes.h, <= 8 characters, "F<n>" beyond the table.
static void test_fault_names_as_used_by_the_fault_row() {
  char tmp[9];
  TEST_ASSERT_EQUAL_UINT8(34, VESC_FAULT_CODE_COUNT);
  TEST_ASSERT_EQUAL_STRING("NONE", vesc_fault_fmt(tmp, sizeof tmp, 0));
  TEST_ASSERT_EQUAL_STRING("OVER_V", vesc_fault_fmt(tmp, sizeof tmp, 1));
  TEST_ASSERT_EQUAL_STRING("UNDER_V", vesc_fault_fmt(tmp, sizeof tmp, 2));
  TEST_ASSERT_EQUAL_STRING("DRV", vesc_fault_fmt(tmp, sizeof tmp, 3));
  TEST_ASSERT_EQUAL_STRING("ABS_OC", vesc_fault_fmt(tmp, sizeof tmp, 4));
  TEST_ASSERT_EQUAL_STRING("OT_FET", vesc_fault_fmt(tmp, sizeof tmp, 5));
  TEST_ASSERT_EQUAL_STRING("OT_MOT", vesc_fault_fmt(tmp, sizeof tmp, 6));
  TEST_ASSERT_EQUAL_STRING("ENC_HIGH", vesc_fault_fmt(tmp, sizeof tmp, 13));
  TEST_ASSERT_EQUAL_STRING("LV_OUT", vesc_fault_fmt(tmp, sizeof tmp, 29));
  TEST_ASSERT_EQUAL_STRING("ABS_OSPD", vesc_fault_fmt(tmp, sizeof tmp, 33));  // last table entry (FW > 6.05)
  TEST_ASSERT_EQUAL_STRING("F34", vesc_fault_fmt(tmp, sizeof tmp, 34));
  TEST_ASSERT_EQUAL_STRING("F255", vesc_fault_fmt(tmp, sizeof tmp, 255));
  for (unsigned c = 0; c < 256; ++c) {
    TEST_ASSERT_TRUE(strlen(vesc_fault_fmt(tmp, sizeof tmp, (uint8_t)c)) <= 8);
    TEST_ASSERT_TRUE(strlen(vesc_fault_fmt(tmp, sizeof tmp, (uint8_t)c)) >= 3);
    if (c < VESC_FAULT_CODE_COUNT) TEST_ASSERT_EQUAL_STRING(vesc_fault_str((uint8_t)c), tmp);
  }
}

static void test_fmt_age_short() {
  using oled_detail::fmt_age_short;
  char b[8];
  fmt_age_short(b, sizeof b, 0);
  TEST_ASSERT_EQUAL_STRING("0s", b);
  fmt_age_short(b, sizeof b, 34999);
  TEST_ASSERT_EQUAL_STRING("34s", b);
  fmt_age_short(b, sizeof b, 59999);
  TEST_ASSERT_EQUAL_STRING("59s", b);
  fmt_age_short(b, sizeof b, 60000);
  TEST_ASSERT_EQUAL_STRING("1m", b);
  fmt_age_short(b, sizeof b, 3599999);
  TEST_ASSERT_EQUAL_STRING("59m", b);
  fmt_age_short(b, sizeof b, 3600000);
  TEST_ASSERT_EQUAL_STRING("1h", b);
  fmt_age_short(b, sizeof b, 86399999);
  TEST_ASSERT_EQUAL_STRING("23h", b);
  fmt_age_short(b, sizeof b, 86400000);
  TEST_ASSERT_EQUAL_STRING("1d", b);
  fmt_age_short(b, sizeof b, 4294967295u);
  TEST_ASSERT_EQUAL_STRING("49d", b);  // the whole millis() range: never more than 3 chars
}

static void test_screen_names_and_count() {
  TEST_ASSERT_EQUAL_INT(7, (int)SCREEN_COUNT);
  TEST_ASSERT_EQUAL_INT(0, (int)SCREEN_MAIN);
  TEST_ASSERT_EQUAL_INT(1, (int)SCREEN_EFF);  // right after MAIN: one short press from the dashboard
  TEST_ASSERT_EQUAL_INT(2, (int)SCREEN_VESC_A);
  TEST_ASSERT_EQUAL_INT(6, (int)SCREEN_SYS);
  TEST_ASSERT_EQUAL_STRING("MAIN", oled_screen_name(SCREEN_MAIN));
  TEST_ASSERT_EQUAL_STRING("EFF", oled_screen_name(SCREEN_EFF));
  TEST_ASSERT_EQUAL_STRING("VESC 1/3", oled_screen_name(SCREEN_VESC_A));
  TEST_ASSERT_EQUAL_STRING("VESC 2/3", oled_screen_name(SCREEN_VESC_B));
  TEST_ASSERT_EQUAL_STRING("VESC 3/3", oled_screen_name(SCREEN_VESC_C));
  TEST_ASSERT_EQUAL_STRING("GNSS", oled_screen_name(SCREEN_GNSS));
  TEST_ASSERT_EQUAL_STRING("SYS", oled_screen_name(SCREEN_SYS));
  TEST_ASSERT_EQUAL_STRING("?", oled_screen_name(SCREEN_COUNT));
}

// ---------------------------------------------------------------- number formatting helpers
static void test_fmt_num_budgets() {
  using oled_detail::fmt_num;
  char b[16];
  fmt_num(b, sizeof b, 48.2f, 1, 5);
  TEST_ASSERT_EQUAL_STRING("48.2", b);
  fmt_num(b, sizeof b, 99.96f, 1, 4);
  TEST_ASSERT_EQUAL_STRING("100", b);  // "100.0" too wide -> no decimals
  fmt_num(b, sizeof b, 12345.0f, 1, 5);
  TEST_ASSERT_EQUAL_STRING("12345", b);
  fmt_num(b, sizeof b, 12345.0f, 0, 4);
  TEST_ASSERT_EQUAL_STRING("12k", b);
  fmt_num(b, sizeof b, 12345.0f, 0, 5);
  TEST_ASSERT_EQUAL_STRING("12345", b);
  fmt_num(b, sizeof b, 123456.0f, 0, 5);
  TEST_ASSERT_EQUAL_STRING("123k", b);  // "123.5k" would be 6
  fmt_num(b, sizeof b, 123456.0f, 0, 6);
  TEST_ASSERT_EQUAL_STRING("123456", b);
  fmt_num(b, sizeof b, 1234567.0f, 0, 6);
  TEST_ASSERT_EQUAL_STRING("1235k", b);
  fmt_num(b, sizeof b, 12345678.0f, 0, 4);
  TEST_ASSERT_EQUAL_STRING("12M", b);
  fmt_num(b, sizeof b, 1.0e9f, 0, 4);
  TEST_ASSERT_EQUAL_STRING("MAX", b);  // "1000M" does not fit either
  fmt_num(b, sizeof b, -1.0e9f, 0, 4);
  TEST_ASSERT_EQUAL_STRING("-MAX", b);
  fmt_num(b, sizeof b, -0.04f, 1, 5);
  TEST_ASSERT_EQUAL_STRING("0.0", b);
  fmt_num(b, sizeof b, 1.2345f, 3, 7);
  TEST_ASSERT_EQUAL_STRING("1.235", b);  // 1.2345f is slightly above 1.2345 -> rounds up
  fmt_num(b, sizeof b, -123456.0f, 0, 4);
  TEST_ASSERT_EQUAL_STRING("-MAX", b);  // "-0M" would hide the magnitude: saturate instead
  fmt_num(b, sizeof b, 123456.0f, 0, 3);
  TEST_ASSERT_EQUAL_STRING("MAX", b);   // "123k" is 4, "0M" is a lie
  // a tiny buffer never accepts a truncated candidate as "fits"
  char t[6];
  fmt_num(t, sizeof t, 123456.7f, 1, 5);  // "123456.7" truncated would read "12345": rejected
  TEST_ASSERT_EQUAL_STRING("123k", t);
}

static void test_fmt_count_budgets() {
  using oled_detail::fmt_count;
  char b[16];
  fmt_count(b, sizeof b, 999, 5);
  TEST_ASSERT_EQUAL_STRING("999", b);
  fmt_count(b, sizeof b, 123456, 5);
  TEST_ASSERT_EQUAL_STRING("123k", b);
  fmt_count(b, sizeof b, 999999, 3);
  TEST_ASSERT_EQUAL_STRING("1M", b);  // rounded, not "0M"
  fmt_count(b, sizeof b, 4294967295u, 5);
  TEST_ASSERT_EQUAL_STRING("4295M", b);
  fmt_count(b, sizeof b, 4294967295u, 3);
  TEST_ASSERT_EQUAL_STRING("4G", b);
  fmt_count(b, sizeof b, 12345, 3);
  TEST_ASSERT_EQUAL_STRING("12k", b);
  fmt_count(b, sizeof b, 250000, 3);
  TEST_ASSERT_EQUAL_STRING("MAX", b);  // "250k" is 4 chars and "0M" would be a lie
  char t[8];
  fmt_count(t, sizeof t, 4294967295u, 5);  // "4294967295" truncated to 7 chars is rejected
  TEST_ASSERT_EQUAL_STRING("4295M", t);
}

// ---------------------------------------------------------------- grid: EFFICIENCY
static void test_eff_rows_sample_trip() {
  OledGrid g = build_grid(SCREEN_EFF, make_state(NOW), NOW);
  char exp[32];
  TEST_ASSERT_EQUAL_STRING("EFFICIENCY", g.title);
  TEST_ASSERT_EQUAL_STRING("2/7", g.page);
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "now", "123"), g.rows[0]);   // >= 100: whole Wh per unit
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "avg", "98.5"), g.rows[1]);  // < 100: one decimal
  TEST_ASSERT_EQUAL_STRING("dist 12.34 1.23kWh", g.rows[2]);  // 1234.5 - 4.5 = 1230 Wh net
  TEST_ASSERT_EQUAL_STRING("P 598W     win 10s", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("run 1h23m  mov 1h02m", g.rows[4]);
  // mean speed while moving = 12.34 units / (3720 s / 3600) = 11.94 units per hour, in the speed unit
#if SPEED_UNIT_KNOTS && !EFF_UNIT_KM
  TEST_ASSERT_EQUAL_STRING("mean 11.9kn src S3", g.rows[5]);
#elif !SPEED_UNIT_KNOTS
  TEST_ASSERT_EQUAL_STRING("mean 11.9km/h src S3", g.rows[5]);  // 13-char left text pushes the second column
#else
  TEST_ASSERT_EQUAL_STRING("mean 6.4kn src S3", g.rows[5]);  // distance in km, speed in knots: 11.94 km/h = 6.45 kn
#endif
  TEST_ASSERT_TRUE(strstr(g.rows[0], EFF_UNIT_STR) != NULL);
  TEST_ASSERT_TRUE(strcmp(EFF_UNIT_STR, "Wh/NM") == 0 || strcmp(EFF_UNIT_STR, "Wh/km") == 0);
  assert_grid_lengths(g);
}

static void test_eff_rows_regen_and_invalid() {
  SharedState s = make_state(NOW);
  char exp[32];
  // regen: net energy and both efficiencies negative
  s.trip.eff_now = -12.3f;
  s.trip.eff_avg = -123.4f;
  s.trip.wh = 100.0f;
  s.trip.wh_charged = 350.0f;
  s.trip.win_p_avg_w = -480.0f;
  OledGrid g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "now", "-12.3"), g.rows[0]);
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "avg", "-123"), g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("dist 12.34 -250Wh", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("P -480W    win 10s", g.rows[3]);
  assert_grid_lengths(g);
  // rounding points of the efficiency number
  s.trip.eff_now = 99.94f;
  s.trip.eff_avg = 99.96f;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "now", "99.9"), g.rows[0]);
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "avg", "100"), g.rows[1]);
  s.trip.eff_now = -0.04f;  // never "-0.0"
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "now", "0.0"), g.rows[0]);
  // too little distance for a ratio: the integrator clears the valid flags; the rest stays live
  s.trip.eff_now_valid = false;
  s.trip.eff_avg_valid = false;
  s.trip.moving_s = 0;  // nothing moved yet: no mean speed either
  s.trip.dist_m = 0.0f;
  s.trip.wh = 0.4f;
  s.trip.wh_charged = 0.0f;
  s.trip.win_fill_s = 3;
  s.trip.run_s = 754;
  s.trip.energy_from_counters = false;  // STATUS_3 not received: v_in x current_in integration
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("now --", g.rows[0]);  // no unit after "--"
  TEST_ASSERT_EQUAL_STRING("avg --", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("dist 0.00  0Wh", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("P -480W    win 3s", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("run 12m34s mov 0m00s", g.rows[4]);  // 6-char uptimes still leave the second column at 11
  TEST_ASSERT_EQUAL_STRING("mean --    src PxI", g.rows[5]);
  assert_grid_lengths(g);
  // no closed window bucket yet (first second after boot): the integrator's 0 W is no measurement
  s.trip.win_fill_s = 0;
  s.trip.win_p_avg_w = 0.0f;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("P --       win 0s", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("dist 0.00  0Wh", g.rows[2]);  // the trip totals are real zeros, not missing data
  s.trip.win_fill_s = 1;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("P 0W       win 1s", g.rows[3]);
  s.trip.win_fill_s = 3;
  s.trip.win_p_avg_w = -480.0f;
  assert_grid_lengths(g);
  // one valid flag alone
  s.trip.eff_avg_valid = true;
  s.trip.eff_avg = 5.0f;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("now --", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "avg", "5.0"), g.rows[1]);
}

static void test_eff_rows_long_trip_and_energy_formats() {
  SharedState s = make_state(NOW);
  char exp[32];
  s.trip.run_s = 90000;     // 1d 1h
  s.trip.moving_s = 86400;  // 1d 0h
  s.trip.dist_m = 1234.6f * EFF_DIST_UNIT_M;
  s.trip.wh = 123456.0f;
  s.trip.wh_charged = 0.0f;
  s.trip.win_p_avg_w = 12345.0f;
  s.trip.eff_now = 1234.6f;
  s.trip.eff_avg = 100.0f;
  s.trip.energy_from_counters = false;
  OledGrid g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "now", "1235"), g.rows[0]);
  TEST_ASSERT_EQUAL_STRING(eff_row(exp, sizeof exp, "avg", "100"), g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("dist 1235  123.5kWh", g.rows[2]);  // distance drops its decimals, kWh keep one
  TEST_ASSERT_EQUAL_STRING("P 12.3kW   win 10s", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("run 1d01h  mov 1d00h", g.rows[4]);
#if SPEED_UNIT_KNOTS && !EFF_UNIT_KM
  TEST_ASSERT_EQUAL_STRING("mean 51.4kn src PxI", g.rows[5]);  // 1234.6 NM / 24 h
#elif !SPEED_UNIT_KNOTS
  TEST_ASSERT_EQUAL_STRING("mean 51.4km/h src PxI", g.rows[5]);  // exactly 21 chars
#else
  TEST_ASSERT_EQUAL_STRING("mean 27.8kn src PxI", g.rows[5]);  // 51.44 km/h = 27.78 kn
#endif
  assert_grid_lengths(g);
  // 100 days and more: whole days, so the moving time is never clipped to a plausible-looking wrong number
  s.trip.run_s = 100u * 86400u;
  s.trip.moving_s = 4294967295u;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("run 100d   mov 49710d", g.rows[4]);  // exactly 21 chars
  assert_grid_lengths(g);
  s.trip.run_s = 90000;
  s.trip.moving_s = 86400;
  // distance: two decimals below 100, one below 1000, none above (fmt_num within 5 chars)
  s.trip.dist_m = 0.05f * EFF_DIST_UNIT_M;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_TRUE(strncmp("dist 0.05  ", g.rows[2], 11) == 0);
  s.trip.dist_m = 99.996f * EFF_DIST_UNIT_M;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_TRUE(strncmp("dist 100.0 ", g.rows[2], 11) == 0);
  s.trip.dist_m = 999.96f * EFF_DIST_UNIT_M;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_TRUE(strncmp("dist 1000  ", g.rows[2], 11) == 0);
  s.trip.dist_m = 12345.0f * EFF_DIST_UNIT_M;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_TRUE(strncmp("dist 12345 ", g.rows[2], 11) == 0);
  // energy: whole Wh below 1 kWh, kWh with two decimals while they fit
  s.trip.dist_m = 12.34f * EFF_DIST_UNIT_M;
  s.trip.wh = 999.4f;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("dist 12.34 999Wh", g.rows[2]);
  s.trip.wh = 999.6f;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("dist 12.34 1.00kWh", g.rows[2]);
  s.trip.wh = 12350.0f;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("dist 12.34 12.35kWh", g.rows[2]);
  s.trip.wh = 123456.0f;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("dist 12.34 123.5kWh", g.rows[2]);
  s.trip.wh = 1234567.0f;  // from 1 MWh on: megawatt-hours (never "1235kWh" / "1000kkWh")
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("dist 12.34 1.23MWh", g.rows[2]);
  s.trip.wh = 0.0f;
  s.trip.wh_charged = 1234.0f;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("dist 12.34 -1.23kWh", g.rows[2]);
  s.trip.wh_charged = 0.4f;
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("dist 12.34 0Wh", g.rows[2]);  // never "-0Wh"
  assert_grid_lengths(g);
}

static void test_eff_stale_integrator_shows_dashes() {
  SharedState s = make_state(NOW);
  s.trip.t_ms = NOW - TRIP_STALE_MS;  // exactly at the limit: live
  OledGrid g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("P 598W     win 10s", g.rows[3]);
  s.trip.t_ms = NOW - TRIP_STALE_MS - 1;  // integrator stopped ticking: nothing is trustworthy
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("EFFICIENCY", g.title);
  TEST_ASSERT_EQUAL_STRING("now --", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("avg --", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("dist --    --", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("P --       win --", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("run --     mov --", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("mean --    src --", g.rows[5]);
  assert_grid_lengths(g);
  s.trip.t_ms = 0;  // never ran
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("now --", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("mean --    src --", g.rows[5]);
  s.trip.t_ms = NOW + 10;  // stamped after our "now": unsigned age wraps -> stale, never a lie
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("dist --    --", g.rows[2]);
  // the millis() wrap: stamped just before, viewed just after
  s.trip.t_ms = 0xFFFFFF00u;
  g = build_grid(SCREEN_EFF, s, 0x00000100u);
  TEST_ASSERT_EQUAL_STRING("P 598W     win 10s", g.rows[3]);
  // the trip screen ignores VESC / GNSS freshness: the integrator owns those decisions
  s = never_state();
  fill_trip(s, NOW);
  g = build_grid(SCREEN_EFF, s, NOW);
  TEST_ASSERT_EQUAL_STRING("dist 12.34 1.23kWh", g.rows[2]);
}

static void test_fmt_wh_and_energy_helpers() {
  using oled_detail::fmt_energy;
  using oled_detail::fmt_wh;
  char b[16];
  fmt_wh(b, sizeof b, 56.7f, 6);
  TEST_ASSERT_EQUAL_STRING("56.7", b);
  fmt_wh(b, sizeof b, -56.7f, 6);
  TEST_ASSERT_EQUAL_STRING("-56.7", b);
  fmt_wh(b, sizeof b, 999.94f, 6);
  TEST_ASSERT_EQUAL_STRING("999.9", b);
  fmt_wh(b, sizeof b, 999.96f, 6);
  TEST_ASSERT_EQUAL_STRING("1000", b);
  fmt_wh(b, sizeof b, 9999.4f, 6);
  TEST_ASSERT_EQUAL_STRING("9999", b);
  fmt_wh(b, sizeof b, 9999.6f, 6);
  TEST_ASSERT_EQUAL_STRING("10.0k", b);
  fmt_wh(b, sizeof b, 123456.0f, 5);
  TEST_ASSERT_EQUAL_STRING("123k", b);  // a 5-char budget drops the decimal
  fmt_wh(b, sizeof b, 1.0e12f, 6);
  TEST_ASSERT_EQUAL_STRING("MAX", b);
  fmt_energy(b, sizeof b, 567.0f, 8);
  TEST_ASSERT_EQUAL_STRING("567Wh", b);
  fmt_energy(b, sizeof b, -567.4f, 8);
  TEST_ASSERT_EQUAL_STRING("-567Wh", b);
  fmt_energy(b, sizeof b, 999.4f, 8);
  TEST_ASSERT_EQUAL_STRING("999Wh", b);
  fmt_energy(b, sizeof b, 999.6f, 8);
  TEST_ASSERT_EQUAL_STRING("1.00kWh", b);
  fmt_energy(b, sizeof b, 1234.0f, 8);
  TEST_ASSERT_EQUAL_STRING("1.23kWh", b);
  fmt_energy(b, sizeof b, 12350.0f, 8);
  TEST_ASSERT_EQUAL_STRING("12.35kWh", b);
  fmt_energy(b, sizeof b, 123456.0f, 8);
  TEST_ASSERT_EQUAL_STRING("123.5kWh", b);
  fmt_energy(b, sizeof b, -12340.0f, 8);
  TEST_ASSERT_EQUAL_STRING("-12.3kWh", b);  // the sign costs the second decimal
  fmt_energy(b, sizeof b, 999994.0f, 8);
  TEST_ASSERT_EQUAL_STRING("1000kWh", b);  // 999.994 kWh: "999.99" needs 6 numeric chars, "1000" fits
  fmt_energy(b, sizeof b, 999995.0f, 8);
  TEST_ASSERT_EQUAL_STRING("1.00MWh", b);  // megawatt-hours from here on
  fmt_energy(b, sizeof b, 1234567.0f, 8);
  TEST_ASSERT_EQUAL_STRING("1.23MWh", b);
  fmt_energy(b, sizeof b, -1234567.0f, 8);
  TEST_ASSERT_EQUAL_STRING("-1.23MWh", b);  // exactly 8
  fmt_energy(b, sizeof b, 123456789.0f, 8);
  TEST_ASSERT_EQUAL_STRING("123.5MWh", b);
  fmt_energy(b, sizeof b, 1.0e12f, 8);
  TEST_ASSERT_EQUAL_STRING("1000kMWh", b);  // 1e6 MWh: absurd, but bounded (fmt_scaled's thousands step)
  fmt_energy(b, sizeof b, -0.4f, 8);
  TEST_ASSERT_EQUAL_STRING("0Wh", b);
}

// ---------------------------------------------------------------- grid: VESC 1/3
static void test_vesc_a_rows() {
  OledGrid g = build_grid(SCREEN_VESC_A, make_state(NOW), NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 1/3", g.title);
  TEST_ASSERT_EQUAL_STRING("3/7", g.page);
  TEST_ASSERT_EQUAL_STRING("FAULT none", g.rows[0]);  // fresh reply, no fault, nothing latched
  TEST_ASSERT_EQUAL_STRING("Vin 48.2   Ibat 12.4", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("Imot 35    Duty 45%", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("P 598W     RPM 2350", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("Tfet 45.3  Tmot 32.1", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("Ah 1.234   Wh 56.7", g.rows[5]);
  assert_grid_lengths(g);
}

static void test_vesc_a_fault_and_stale() {
  SharedState s = make_state(NOW);
  s.vesc_ext.fault_code = 5;  // live fault in the last reply (can_vesc also latches it; the row prefers the live byte)
  s.vesc_ext.last_fault = 5;
  s.vesc_ext.last_fault_ms = NOW;
  OledGrid g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("FAULT OT_FET", g.rows[0]);
  s.vesc_ext.fault_code = 13;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("FAULT ENC_HIGH", g.rows[0]);
  s.vesc_ext.fault_code = 77;  // beyond the table
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("FAULT F77", g.rows[0]);
  s.vesc_ext.t_ms = NOW - VESC_EXT_STALE_MS;  // exactly at the limit: fresh
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("FAULT F77", g.rows[0]);
  // polled values stale: the live byte is unknown, the latched fault (with its age) is still history worth showing
  s.vesc_ext.t_ms = NOW - VESC_EXT_STALE_MS - 1;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("LAST OT_FET 0s", g.rows[0]);
  s.vesc_ext.last_fault = 0;  // nothing latched, values stale -> unknown, never "none"
  s.vesc_ext.last_fault_ms = 0;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("FAULT --", g.rows[0]);
  s.vesc_ext.t_ms = 0;  // never polled (VESC_POLL_MS 0)
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("FAULT --", g.rows[0]);
  // per-message staleness
  s.vesc.t.t_ms[VESC_IDX_STATUS_2] = NOW - VESC_STALE_R2_MS - 1;
  s.vesc.t.t_ms[VESC_IDX_STATUS_5] = NOW - VESC_STALE_R1_MS - 1;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("Vin --     Ibat 12.4", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("P --       RPM 2350", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("Ah --      Wh 56.7", g.rows[5]);
  s.vesc.t.duty = -1.0f;
  s.vesc.i_motor_ema = -1234.0f;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("Imot -1234 Duty -100%", g.rows[2]);  // exactly 21 chars
  assert_grid_lengths(g);
}

// The VESC clears its live fault byte ~500 ms after the fault, so a 1 Hz poll
// almost always reads 0: the row falls back to the latched code with its age.
static void test_vesc_a_latched_fault_with_age() {
  SharedState s = make_state(NOW);
  s.vesc_ext.fault_code = 0;
  s.vesc_ext.last_fault = 5;
  s.vesc_ext.last_fault_ms = NOW - 34000;
  OledGrid g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("LAST OT_FET 34s", g.rows[0]);
  s.vesc_ext.last_fault_ms = NOW - 59999;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("LAST OT_FET 59s", g.rows[0]);
  s.vesc_ext.last_fault_ms = NOW - 60000;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("LAST OT_FET 1m", g.rows[0]);
  s.vesc_ext.last_fault_ms = NOW - 2 * 3600000u;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("LAST OT_FET 2h", g.rows[0]);
  s.vesc_ext.last_fault = 13;  // the longest names still fit the 21-char row
  s.vesc_ext.last_fault_ms = NOW - 3 * 86400000u;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("LAST ENC_HIGH 3d", g.rows[0]);
  s.vesc_ext.last_fault = 255;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("LAST F255 3d", g.rows[0]);
  // the producer stamped the fault after our "now" was read: 0 s, never 49 days
  s.vesc_ext.last_fault = 5;
  s.vesc_ext.last_fault_ms = NOW + 10;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("LAST OT_FET 0s", g.rows[0]);
  // a live fault always wins over the latched one
  s.vesc_ext.fault_code = 2;
  s.vesc_ext.last_fault_ms = NOW - 34000;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("FAULT UNDER_V", g.rows[0]);
  // a latched code without a timestamp is not a fault (never written by can_vesc; stay defined)
  s.vesc_ext.fault_code = 0;
  s.vesc_ext.last_fault_ms = 0;
  g = build_grid(SCREEN_VESC_A, s, NOW);
  TEST_ASSERT_EQUAL_STRING("FAULT none", g.rows[0]);
  // the millis() wrap: fault stamped just before, viewed just after
  s.vesc_ext.last_fault_ms = 0xFFFFF000u;
  g = build_grid(SCREEN_VESC_A, s, 0x00001000u);  // 8192 ms later
  TEST_ASSERT_EQUAL_STRING("LAST OT_FET 8s", g.rows[0]);
  assert_grid_lengths(g);
}

// ---------------------------------------------------------------- grid: VESC 2/3
static void test_vesc_b_rows() {
  SharedState s = make_state(NOW);
  OledGrid g = build_grid(SCREEN_VESC_B, s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 2/3", g.title);
  TEST_ASSERT_EQUAL_STRING("4/7", g.page);
  TEST_ASSERT_EQUAL_STRING("Tmos 45.1  Iin 12.4", g.rows[0]);  // first MOSFET sensor + the VESC's averaged input current
  TEST_ASSERT_EQUAL_STRING("Id 0.3     Iq 34.9", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("Vd 1.23    Vq 23.45", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("Tach 12345 Abs 23456", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("St OK      id 74", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("AhC 0.012  WhC 3.4", g.rows[5]);
  assert_grid_lengths(g);
  s.vesc_ext.status = 1;
  g = build_grid(SCREEN_VESC_B, s, NOW);
  TEST_ASSERT_EQUAL_STRING("St TIMEOUT id 74", g.rows[4]);
  s.vesc_ext.status = 2;
  g = build_grid(SCREEN_VESC_B, s, NOW);
  TEST_ASSERT_EQUAL_STRING("St KILLSW  id 74", g.rows[4]);
  s.vesc_ext.status = 0xFF;
  g = build_grid(SCREEN_VESC_B, s, NOW);
  TEST_ASSERT_EQUAL_STRING("St TO+KILL id 74", g.rows[4]);
  // the second and third MOSFET sensors are polled (logs) but never displayed
  s.vesc_ext.temp_mos2 = 77.7f;
  s.vesc_ext.temp_mos3 = 88.8f;
  g = build_grid(SCREEN_VESC_B, s, NOW);
  TEST_ASSERT_EQUAL_STRING("Tmos 45.1  Iin 12.4", g.rows[0]);
  for (int r = 0; r < kGridRows; ++r) {
    TEST_ASSERT_NULL(strstr(g.rows[r], "77.7"));
    TEST_ASSERT_NULL(strstr(g.rows[r], "88.8"));
    TEST_ASSERT_NULL(strstr(g.rows[r], "/"));  // the old "a/b/c" form is gone
  }
  // negative / large values keep their columns
  s.vesc_ext.temp_mos1 = -12.3f;
  s.vesc_ext.temp_mos2 = 123.4f;
  s.vesc_ext.temp_mos3 = -0.04f;
  s.vesc_ext.avg_input_current = -123.4f;
  s.vesc.t.tachometer = -2147483647;
  s.vesc_ext.tacho_abs = 2147483647;
  g = build_grid(SCREEN_VESC_B, s, NOW);
  TEST_ASSERT_EQUAL_STRING("Tmos -12.3 Iin -123.4", g.rows[0]);  // exactly 21 chars
  TEST_ASSERT_EQUAL_STRING("Tach -MAX  Abs 2147M", g.rows[3]);   // "-2147M" needs 6 chars, "-2M" would lie
  assert_grid_lengths(g);
  s.vesc_ext.temp_mos1 = 123.4f;
  s.vesc_ext.avg_input_current = 9.96f;
  g = build_grid(SCREEN_VESC_B, s, NOW);
  TEST_ASSERT_EQUAL_STRING("Tmos 123.4 Iin 10.0", g.rows[0]);
  // polled values stale -> "--" everywhere they come from, tacho (STATUS_5) unaffected
  s.vesc_ext.t_ms = NOW - VESC_EXT_STALE_MS - 1;
  g = build_grid(SCREEN_VESC_B, s, NOW);
  TEST_ASSERT_EQUAL_STRING("Tmos --    Iin --", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("Id --      Iq --", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("Vd --      Vq --", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("Tach -MAX  Abs --", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("St --      id --", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("AhC 0.012  WhC 3.4", g.rows[5]);
}

// ---------------------------------------------------------------- grid: VESC 3/3
static void test_vesc_c_rows() {
  SharedState s = make_state(NOW);
  OledGrid g = build_grid(SCREEN_VESC_C, s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 3/3", g.title);
  TEST_ASSERT_EQUAL_STRING("5/7", g.page);
  TEST_ASSERT_EQUAL_STRING("PPM 0.52   PID 12.3", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("ADC 1.23 2.10 0.00", g.rows[1]);
#if VESC_POLL_MS > 0
  TEST_ASSERT_EQUAL_STRING("poll 123   ok 120", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("bad 0      tmo 3", g.rows[3]);
#else
  TEST_ASSERT_EQUAL_STRING("poll off   ok --", g.rows[2]);  // passive build: never polls, never "0 of 0"
  TEST_ASSERT_EQUAL_STRING("bad --     tmo --", g.rows[3]);
#endif
  TEST_ASSERT_EQUAL_STRING("frm 12345  oth 0", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("misc 3     busoff 0", g.rows[5]);
  assert_grid_lengths(g);
  s.vesc.t.adc1 = -32.768f;
  s.vesc.t.adc2 = 32.767f;
  s.vesc.t.adc3 = -1.5f;
  s.vesc_ext.polls_sent = 4294967295u;
  s.vesc_ext.replies_ok = 1234567;
  s.vesc.frames_total = 987654321;
  s.can.bus_off_count = 999999;
  g = build_grid(SCREEN_VESC_C, s, NOW);
  TEST_ASSERT_EQUAL_STRING("ADC -32.8 32.77 -1.50", g.rows[1]);  // exactly 21 chars
#if VESC_POLL_MS > 0
  TEST_ASSERT_EQUAL_STRING("poll 4295M ok 1235k", g.rows[2]);
#else
  TEST_ASSERT_EQUAL_STRING("poll off   ok --", g.rows[2]);
#endif
  TEST_ASSERT_EQUAL_STRING("frm 988M   oth 0", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("misc 3     busoff 1M", g.rows[5]);
  assert_grid_lengths(g);
  s.vesc.t.t_ms[VESC_IDX_STATUS_6] = NOW - VESC_STALE_R2_MS - 1;  // STATUS_6 is Rate 2
  g = build_grid(SCREEN_VESC_C, s, NOW);
  TEST_ASSERT_EQUAL_STRING("PPM --     PID 12.3", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("ADC --", g.rows[1]);
}

// ---------------------------------------------------------------- grid: GNSS
static void test_gnss_rows() {
  OledGrid g = build_grid(SCREEN_GNSS, make_state(NOW), NOW);
  TEST_ASSERT_EQUAL_STRING("GNSS", g.title);
  TEST_ASSERT_EQUAL_STRING("6/7", g.page);
  TEST_ASSERT_EQUAL_STRING("3D 9sv     pDOP 1.5", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("59.43701N 24.75368W", g.rows[1]);  // 5 decimals, rounded, hemisphere letters
  TEST_ASSERT_EQUAL_STRING("Alt 12.3m  hAcc 2.1m", g.rows[2]);
#if SPEED_UNIT_KNOTS
  TEST_ASSERT_EQUAL_STRING("Spd 10.0   sAcc 0.30", g.rows[3]);
#else
  TEST_ASSERT_EQUAL_STRING("Spd 18.5   sAcc 0.30", g.rows[3]);
#endif
  TEST_ASSERT_EQUAL_STRING("Hdg 123.4  vAcc 3.0m", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("2026-09-06 12:34:56Z", g.rows[5]);
  assert_grid_lengths(g);
}

static void test_gnss_coordinates_rounding_and_hemispheres() {
  SharedState s = make_state(NOW);
  s.gnss.lat_e7 = -338688000;   // 33.8688 S
  s.gnss.lon_e7 = 1512093000;   // 151.2093 E
  OledGrid g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("33.86880S 151.20930E", g.rows[1]);
  s.gnss.lat_e7 = 899999950;    // rounds up into the next degree
  s.gnss.lon_e7 = -1799999950;
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("90.00000N 180.00000W", g.rows[1]);  // exactly 20 chars
  s.gnss.lat_e7 = 0;
  s.gnss.lon_e7 = -1;
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.00000N 0.00000W", g.rows[1]);
  s.gnss.lat_e7 = -2147483647;  // garbage: still within the row
  s.gnss.lon_e7 = -2147483647;
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("214.74836S 214.74836W", g.rows[1]);
  assert_grid_lengths(g);
}

static void test_gnss_without_fix_or_receiver() {
  SharedState s = make_state(NOW);
  s.gnss.fix_type = 0;
  s.gnss.fix_ok = false;
  OledGrid g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("NO FIX     pDOP 1.5", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("Pos --", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("Alt --     hAcc 2.1m", g.rows[2]);  // accuracies are receiver estimates: live-gated only
  TEST_ASSERT_EQUAL_STRING("Spd --     sAcc 0.30", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("Hdg --     vAcc 3.0m", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("2026-09-06 12:34:56Z", g.rows[5]);  // time is valid without a position fix
  s.gnss.fix_type = 5;
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("TIME ONLY  pDOP 1.5", g.rows[0]);
  s.gnss.fix_type = 2;
  s.gnss.fix_ok = true;
  s.gnss.num_sv = 12;
  s.gnss.gspeed_mm_s = kBelowMinMmS;  // at rest: heading is noise
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("2D 12sv    pDOP 1.5", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("Spd 0.0    sAcc 0.30", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("Hdg --     vAcc 3.0m", g.rows[4]);
  s.gnss.time_valid = false;
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("UTC --", g.rows[5]);
  // stale epoch
  s = make_state(NOW);
  s.gnss.last_pvt_ms = NOW - GNSS_STALE_MS - 1;
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("NO DATA    pDOP --", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("Pos --", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("Alt --     hAcc --", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("UTC --", g.rows[5]);
  // never any epoch: the receiver phase explains why
  s.gnss.last_pvt_ms = 0;
  s.gnss.phase = GNSS_PHASE_AUTOBAUD;
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("AUTOBAUD   pDOP --", g.rows[0]);
  s.gnss.phase = GNSS_PHASE_CONFIGURE;
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("CONFIG     pDOP --", g.rows[0]);
  s.gnss.phase = GNSS_PHASE_RUN;
  g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("NO GNSS    pDOP --", g.rows[0]);
  assert_grid_lengths(g);
}

static void test_gnss_extreme_values() {
  SharedState s = make_state(NOW);
  s.gnss.hmsl_mm = -123456789;
  s.gnss.hacc_mm = 4294967295u;
  s.gnss.vacc_mm = 4294967295u;
  s.gnss.sacc_mm_s = 1999;  // just below the display gate
  s.gnss.pdop_x100 = 65535;
  s.gnss.head_mot_e5 = 35999999;
  s.gnss.num_sv = 255;
  s.gnss.year = 65535;
  s.gnss.month = 255;
  s.gnss.day = 255;
  s.gnss.hour = 255;
  s.gnss.min = 255;
  s.gnss.sec = 255;
  OledGrid g = build_grid(SCREEN_GNSS, s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D 255sv   pDOP 655.3", g.rows[0]);  // 655.35 as a float is just below .35
  TEST_ASSERT_EQUAL_STRING("Alt -123km hAcc 4Mm", g.rows[2]);    // -123457 m -> "-123k"+"m"; 4294967 m -> "4M"+"m"
  TEST_ASSERT_EQUAL_STRING("Hdg 360.0  vAcc 4Mm", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("65535-255-255 255:255", g.rows[5]);  // garbage date clipped to the row, never overflowing
  assert_grid_lengths(g);
}

// ---------------------------------------------------------------- grid: SYS
static void test_sys_rows() {
  OledGrid g = build_grid(SCREEN_SYS, make_state(NOW), NOW);
  char title[24];
  snprintf(title, sizeof title, "SYS %s", FW_VERSION);
  TEST_ASSERT_EQUAL_STRING(title, g.title);
  TEST_ASSERT_EQUAL_STRING("7/7", g.page);
  TEST_ASSERT_EQUAL_STRING("up 1h23m   btn 12", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("heap 250k  min 240k", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("CAN RUN    T0 R0", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("GNSS RUN 38400 34.10", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("ubx 1234/0 rd 0", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("rst POWERON lock 0", g.rows[5]);
  assert_grid_lengths(g);
}

static void test_sys_variants() {
  SharedState s = make_state(NOW);
  OledSysInfo info = kInfo;
  info.uptime_s = 754;
  info.heap_free = 1234567;
  info.heap_min = 0;
  info.reset_reason = "TASK_WDT";
  s.can.state = CAN_STATE_BUS_OFF;
  s.can.tec = 128;
  s.can.rec = 255;
  s.gnss.phase = GNSS_PHASE_CONFIGURE;
  s.gnss.baud = 230400;
  s.gnss.good_frames = 1234567;
  s.gnss.bad_frames = 12;
  s.gnss.redetects = 3;
  s.lock_failures = 7;
  s.disp.button_presses = 123456;
  OledGrid g = build_grid(SCREEN_SYS, s, NOW, info);
  TEST_ASSERT_EQUAL_STRING("up 12m34s  btn 123k", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("heap 1205k min 0k", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("CAN BUSOFF T128 R255", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("GNSS CONFIG 230400", g.rows[3]);  // PROTVER only while running
  TEST_ASSERT_EQUAL_STRING("ubx 1235k/12 rd 3", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("rst TASK_WDT lock 7", g.rows[5]);
  assert_grid_lengths(g);
  info.uptime_s = 90000;  // 1d 1h
  info.reset_reason = nullptr;
  s.can.state = CAN_STATE_UNINSTALLED;
  s.gnss.phase = GNSS_PHASE_RUN;
  g = build_grid(SCREEN_SYS, s, NOW, info);
  TEST_ASSERT_EQUAL_STRING("up 1d01h   btn 123k", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("CAN UNINST T128 R255", g.rows[2]);  // 10-char left text: second column still at 11
  TEST_ASSERT_EQUAL_STRING("GNSS RUN 230400 34.10", g.rows[3]);  // exactly 21 chars
  TEST_ASSERT_EQUAL_STRING("rst ?      lock 7", g.rows[5]);
  s.gnss.phase = GNSS_PHASE_AUTOBAUD;
  s.gnss.baud = 0;
  s.gnss.prot_ver_x100 = -1;
  g = build_grid(SCREEN_SYS, s, NOW, info);
  TEST_ASSERT_EQUAL_STRING("GNSS AUTOBAUD", g.rows[3]);
  info.reset_reason = "DEEPSLEEP";  // 13-char left text pushes the second column right, row still <= 21
  g = build_grid(SCREEN_SYS, s, NOW, info);
  TEST_ASSERT_EQUAL_STRING("rst DEEPSLEEP lock 7", g.rows[5]);
  info.uptime_s = 100u * 86400u - 1u;
  g = build_grid(SCREEN_SYS, s, NOW, info);
  TEST_ASSERT_EQUAL_STRING("up 99d23h  btn 123k", g.rows[0]);
  info.uptime_s = 100u * 86400u;  // from 100 days on: whole days, so the uptime never exceeds 6 chars
  g = build_grid(SCREEN_SYS, s, NOW, info);
  TEST_ASSERT_EQUAL_STRING("up 100d    btn 123k", g.rows[0]);
  info.uptime_s = 4294967295u;
  g = build_grid(SCREEN_SYS, s, NOW, info);
  TEST_ASSERT_EQUAL_STRING("up 49710d  btn 123k", g.rows[0]);  // the whole uint32_t range stays in the first column
  assert_grid_lengths(g);
}

// ---------------------------------------------------------------- frames: dispatch, determinism, boot
static void test_frame_dispatch_and_determinism() {
  SharedState s = make_state(NOW);
  OledFrame a, b;
  for (uint8_t sc = 0; sc < SCREEN_COUNT; ++sc) {
    memset(&a, 0xAA, sizeof a);
    memset(&b, 0x55, sizeof b);
    oled_build_frame(s, NOW, sc, kInfo, a);
    oled_build_frame(s, NOW + 500, sc, kInfo, b);  // same state a little later: byte-identical
    TEST_ASSERT_EQUAL_UINT8(sc, a.screen);
    TEST_ASSERT_EQUAL_MEMORY(&a, &b, sizeof a);
    if (sc == SCREEN_MAIN) {
      assert_main_lengths(a.main);
      assert_zero_tail(a.main.speed, sizeof a.main.speed);
      assert_zero_tail(a.main.can, sizeof a.main.can);
      for (int i = 0; i < 8; ++i) assert_zero_tail(a.main.cells[i].value, sizeof a.main.cells[i].value);
      // the unused grid half is all zero
      for (size_t i = 0; i < sizeof a.grid; ++i) TEST_ASSERT_EQUAL_HEX8(0, ((const uint8_t *)&a.grid)[i]);
    } else {
      assert_grid_lengths(a.grid);
      // the title starts with the screen name ("SYS" is followed by the firmware version)
      TEST_ASSERT_TRUE(strncmp(a.grid.title, oled_screen_name(sc), strlen(oled_screen_name(sc))) == 0);
      for (int r = 0; r < kGridRows; ++r) assert_zero_tail(a.grid.rows[r], sizeof a.grid.rows[r]);
      for (size_t i = 0; i < sizeof a.main; ++i) TEST_ASSERT_EQUAL_HEX8(0, ((const uint8_t *)&a.main)[i]);
    }
  }
  // every screen differs from every other (the index alone guarantees a redraw)
  OledFrame f[SCREEN_COUNT];
  for (uint8_t sc = 0; sc < SCREEN_COUNT; ++sc) oled_build_frame(s, NOW, sc, kInfo, f[sc]);
  for (int i = 0; i < (int)SCREEN_COUNT; ++i)
    for (int j = i + 1; j < (int)SCREEN_COUNT; ++j) TEST_ASSERT_TRUE(memcmp(&f[i], &f[j], sizeof f[i]) != 0);
  // a real change is detected
  s.vesc.i_in_ema = 12.6f;
  oled_build_frame(s, NOW, SCREEN_MAIN, kInfo, b);
  TEST_ASSERT_TRUE(memcmp(&f[SCREEN_MAIN], &b, sizeof b) != 0);
  // an unknown screen index falls back to the main screen
  oled_build_frame(s, NOW, 200, kInfo, b);
  TEST_ASSERT_EQUAL_UINT8(SCREEN_MAIN, b.screen);
  TEST_ASSERT_EQUAL_STRING("12.6A", b.main.cells[1].value);
}

struct Guarded {
  char pre[16];
  OledFrame f;
  char post[16];
};

static void test_extreme_values_respect_budgets_and_buffers() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 1.0e9f;
  s.vesc.i_in_ema = -1.0e9f;
  s.vesc.i_motor_ema = 1.0e9f;
  s.vesc.t.erpm = erpm_for(-4.29e8f);
  s.vesc.t.duty = -3276.8f;
  s.vesc.t.amp_hours = 214748.36f;
  s.vesc.t.amp_hours_charged = -214748.36f;
  s.vesc.t.watt_hours = 214748.36f;
  s.vesc.t.watt_hours_charged = 214748.36f;
  s.vesc.t.temp_fet = -3276.8f;
  s.vesc.t.temp_motor = 3276.7f;
  s.vesc.t.pid_pos = -655.36f;
  s.vesc.t.tachometer = -2147483647 - 1;
  s.vesc.t.ppm = -32.768f;
  s.vesc.t.adc1 = s.vesc.t.adc2 = s.vesc.t.adc3 = -32.768f;
  s.vesc.locked_id = 254;
  s.vesc.frames_total = s.vesc.frames_other_id = s.vesc.frames_dropped = 4294967295u;
  s.vesc_ext.temp_mos1 = s.vesc_ext.temp_mos2 = s.vesc_ext.temp_mos3 = -3.0e9f;
  s.vesc_ext.avg_id = s.vesc_ext.avg_iq = -3.0e9f;
  s.vesc_ext.vd = s.vesc_ext.vq = 3.0e9f;
  s.vesc_ext.tacho_abs = -2147483647 - 1;
  s.vesc_ext.fault_code = 255;
  s.vesc_ext.status = 255;
  s.vesc_ext.vesc_id = 255;
  s.vesc_ext.polls_sent = s.vesc_ext.replies_ok = s.vesc_ext.replies_bad = s.vesc_ext.timeouts = 4294967295u;
  s.can.tec = s.can.rec = s.can.bus_off_count = 4294967295u;
  s.gnss.gspeed_mm_s = 0x7FFFFFFF;
  s.gnss.sacc_mm_s = 0;
  s.gnss.num_sv = 255;
  s.gnss.baud = 4294967295u;
  s.gnss.prot_ver_x100 = 0x7FFFFFFF;
  s.gnss.good_frames = s.gnss.bad_frames = s.gnss.redetects = 4294967295u;
  s.gnss.phase = 200;
  s.lock_failures = 4294967295u;
  s.disp.button_presses = 4294967295u;
  s.trip.run_s = s.trip.moving_s = 4294967295u;
  s.trip.dist_m = 3.0e9f;
  s.trip.wh = 3.0e9f;
  s.trip.wh_charged = -3.0e9f;
  s.trip.win_p_avg_w = -3.0e9f;
  s.trip.eff_now = -3.0e9f;
  s.trip.eff_avg = 3.0e9f;
  s.trip.win_fill_s = 255;
  s.trip.counter_resets = 4294967295u;
  OledSysInfo info = {4294967295u, 4294967295u, 4294967295u, "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"};

  for (uint8_t sc = 0; sc < SCREEN_COUNT; ++sc) {
    Guarded g;
    memset(&g, 0xAA, sizeof g);
    oled_build_frame(s, NOW, sc, info, g.f);
    if (sc == SCREEN_MAIN) {
      assert_main_lengths(g.f.main);
      TEST_ASSERT_EQUAL_STRING("VESC 254", g.f.main.can);
      TEST_ASSERT_EQUAL_STRING("3D99sv", g.f.main.fix);
      TEST_ASSERT_EQUAL_STRING("--", g.f.main.speed);  // absurd speed, never a plausible number
      TEST_ASSERT_EQUAL_STRING("1000Mv", g.f.main.cells[0].value);
      TEST_ASSERT_EQUAL_STRING("-MAXA", g.f.main.cells[1].value);  // "-1000M" would need 6 numeric chars
      TEST_ASSERT_EQUAL_STRING("1000MA", g.f.main.cells[2].value);
      TEST_ASSERT_EQUAL_STRING("--", g.f.main.cells[4].value);     // implausible temperatures
      TEST_ASSERT_EQUAL_STRING("n/a", g.f.main.cells[5].value);
      TEST_ASSERT_EQUAL_STRING("-429M", g.f.main.cells[6].value);  // -4.29e8 mechanical rpm
      TEST_ASSERT_EQUAL_STRING("214.7k", g.f.main.cells[7].value);  // 214748.36 Wh in thousands
    } else {
      assert_grid_lengths(g.f.grid);
    }
    for (size_t i = 0; i < sizeof g.pre; ++i) {
      TEST_ASSERT_EQUAL_HEX8(0xAA, (uint8_t)g.pre[i]);
      TEST_ASSERT_EQUAL_HEX8(0xAA, (uint8_t)g.post[i]);
    }
  }
}

static void test_never_received_state() {
  SharedState z = never_state();
  OledMain m = build_main(z, NOW);
  TEST_ASSERT_EQUAL_STRING("--", m.speed);
  TEST_ASSERT_EQUAL_STRING("--:--", m.clock);
  TEST_ASSERT_EQUAL_STRING("NOGNSS", m.fix);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", m.can);
  for (int i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_STRING("--", m.cells[i].value);
  OledGrid g = build_grid(SCREEN_VESC_A, z, NOW);
  TEST_ASSERT_EQUAL_STRING("FAULT --", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("Vin --     Ibat --", g.rows[1]);
  g = build_grid(SCREEN_VESC_B, z, NOW);
  TEST_ASSERT_EQUAL_STRING("Tmos --    Iin --", g.rows[0]);
  g = build_grid(SCREEN_EFF, z, NOW);  // integrator never ran: every trip value unknown
  TEST_ASSERT_EQUAL_STRING("now --", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("avg --", g.rows[1]);
  TEST_ASSERT_EQUAL_STRING("dist --    --", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("P --       win --", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("run --     mov --", g.rows[4]);
  TEST_ASSERT_EQUAL_STRING("mean --    src --", g.rows[5]);
  g = build_grid(SCREEN_VESC_C, z, NOW);
#if VESC_POLL_MS > 0
  TEST_ASSERT_EQUAL_STRING("poll 0     ok 0", g.rows[2]);
#else
  TEST_ASSERT_EQUAL_STRING("poll off   ok --", g.rows[2]);
#endif
  g = build_grid(SCREEN_GNSS, z, NOW);
  TEST_ASSERT_EQUAL_STRING("AUTOBAUD   pDOP --", g.rows[0]);
  TEST_ASSERT_EQUAL_STRING("UTC --", g.rows[5]);
  g = build_grid(SCREEN_SYS, z, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN UNINST T0 R0", g.rows[2]);
  TEST_ASSERT_EQUAL_STRING("GNSS AUTOBAUD", g.rows[3]);
  TEST_ASSERT_EQUAL_STRING("ubx 0/0 rd 0", g.rows[4]);
}

static void test_boot_frame() {
  OledFrame f;
  memset(&f, 0x55, sizeof f);
  oled_boot_frame(f);
  TEST_ASSERT_EQUAL_UINT8(SCREEN_MAIN, f.screen);
  TEST_ASSERT_EQUAL_STRING("--", f.main.speed);
  TEST_ASSERT_EQUAL_STRING(SPEED_UNIT_STR, f.main.unit);
  TEST_ASSERT_EQUAL_STRING("--:--", f.main.clock);
  TEST_ASSERT_EQUAL_STRING("BOOT", f.main.fix);
  // the CAN line carries FW_VERSION clipped to its 8-char budget (a long tag such as "9.10.11-rc1" is cut, never overflowed)
  const size_t fw_len = strlen(FW_VERSION) < (size_t)kCanChars ? strlen(FW_VERSION) : (size_t)kCanChars;
  TEST_ASSERT_TRUE(strlen(f.main.can) == fw_len && strncmp(f.main.can, FW_VERSION, fw_len) == 0);
  static const char *const kLabels[8] = {"BV", "BA", "MA", "PW", "TF", "TM", "RPM", "WH"};
  for (int i = 0; i < 8; ++i) {
    TEST_ASSERT_EQUAL_STRING(kLabels[i], f.main.cells[i].label);
    TEST_ASSERT_EQUAL_STRING("--", f.main.cells[i].value);
  }
  assert_main_lengths(f.main);
  for (size_t i = 0; i < sizeof f.grid; ++i) TEST_ASSERT_EQUAL_HEX8(0, ((const uint8_t *)&f.grid)[i]);
  // The boot frame differs from any live frame (so the first live tick redraws) ...
  OledFrame live;
  oled_build_frame(never_state(), NOW, SCREEN_MAIN, kInfo, live);
  TEST_ASSERT_TRUE(memcmp(&f, &live, sizeof f) != 0);
  // ... and from the same boot frame built twice it does not (memcmp stable).
  OledFrame again;
  memset(&again, 0xAA, sizeof again);
  oled_boot_frame(again);
  TEST_ASSERT_EQUAL_MEMORY(&f, &again, sizeof f);
}

// ---------------------------------------------------------------- layout arithmetic (mirrors the static_asserts)
static void test_layout_geometry() {
  // main screen: the widest speed "99.9" (3 digits x 15 px + '.' 9 px) starts at x >= 0
  TEST_ASSERT_TRUE(kSpeedRight - (3 * 15 + 9) >= 0);
  TEST_ASSERT_TRUE(kSpeedBaseline - kBigCap >= 0);
  // right column: 12 cells x 6 px from x 56 end exactly at the panel edge
  TEST_ASSERT_EQUAL_INT(kWidth, kColX + kColChars * kAdvance);
  TEST_ASSERT_EQUAL_INT(kColChars, kClockChars + 1 + kFixChars);
  TEST_ASSERT_TRUE(kColBaseline[2] <= kSpeedBaseline);
  TEST_ASSERT_TRUE(kRuleY > kSpeedBaseline);
  // cells: 10 chars fit between the label x and the value's right edge; 4 rows end on the last panel row
  TEST_ASSERT_TRUE(kCellLabelDx + kCellChars * kAdvance <= kCellValueRightDx);
  TEST_ASSERT_TRUE(kCellLabelChars + 1 + kCellValueChars <= kCellChars);
  TEST_ASSERT_EQUAL_INT(kHeight - 1, kCellBaseline[3]);
  TEST_ASSERT_TRUE(kCellBaseline[0] - kSmallCap > kRuleY);
  // grid: title bar + 6 rows of 21 chars end on the last panel row
  TEST_ASSERT_EQUAL_INT(kHeight - 1, kGridBaseline0 + (kGridRows - 1) * kGridPitch);
  TEST_ASSERT_TRUE(kGridBaseline0 - kSmallCap > kTitleBarH);
  TEST_ASSERT_TRUE(kGridX + kGridChars * kAdvance <= kWidth);
  TEST_ASSERT_TRUE((kTitleChars + 1 + kPageChars) * kAdvance + kTitleX <= kWidth);
  // struct buffers hold their budgets plus the NUL
  OledMain m;
  OledGrid g;
  TEST_ASSERT_TRUE(sizeof m.speed > (size_t)kSpeedChars);
  TEST_ASSERT_TRUE(sizeof m.clock > (size_t)kClockChars);
  TEST_ASSERT_TRUE(sizeof m.fix > (size_t)kFixChars);
  TEST_ASSERT_TRUE(sizeof m.can > (size_t)kCanChars);
  TEST_ASSERT_TRUE(sizeof m.cells[0].label > (size_t)kCellLabelChars);
  TEST_ASSERT_TRUE(sizeof m.cells[0].value > (size_t)kCellValueChars);
  TEST_ASSERT_TRUE(sizeof g.title > (size_t)kTitleChars);
  TEST_ASSERT_TRUE(sizeof g.page > (size_t)kPageChars);
  TEST_ASSERT_TRUE(sizeof g.rows[0] > (size_t)kGridChars);
  TEST_ASSERT_TRUE(sizeof g.rows / sizeof g.rows[0] == (size_t)kGridRows);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_main_sample_state);
  RUN_TEST(test_clock_from_gnss_with_configured_offset);
  RUN_TEST(test_clock_hidden_without_valid_time_or_fresh_epoch);
  RUN_TEST(test_utc_to_local_offsets_and_day_wrap);
  RUN_TEST(test_battery_current_signed_one_decimal);
  RUN_TEST(test_voltage_cell);
  RUN_TEST(test_motor_current_whole_amps_and_kilo);
  RUN_TEST(test_power_watts_and_kilowatts);
  RUN_TEST(test_negative_zero_is_normalised);
  RUN_TEST(test_temperature_cells);
  RUN_TEST(test_rpm_cell);
  RUN_TEST(test_watt_hours_cell_formats_and_uses_rate2_window);
  RUN_TEST(test_status5_stale_hides_voltage_and_power_only);
  RUN_TEST(test_status4_stale_hides_current_power_and_temps);
  RUN_TEST(test_exactly_at_stale_limit_is_fresh);
  RUN_TEST(test_wraparound_freshness);
  RUN_TEST(test_future_timestamp_is_stale_not_fresh);
  RUN_TEST(test_speed_rules);
  RUN_TEST(test_fix_line);
  RUN_TEST(test_can_states);
  RUN_TEST(test_can_running_without_fresh_status_is_idle);
  RUN_TEST(test_fault_names_as_used_by_the_fault_row);
  RUN_TEST(test_fmt_age_short);
  RUN_TEST(test_screen_names_and_count);
  RUN_TEST(test_fmt_num_budgets);
  RUN_TEST(test_fmt_count_budgets);
  RUN_TEST(test_fmt_wh_and_energy_helpers);
  RUN_TEST(test_eff_rows_sample_trip);
  RUN_TEST(test_eff_rows_regen_and_invalid);
  RUN_TEST(test_eff_rows_long_trip_and_energy_formats);
  RUN_TEST(test_eff_stale_integrator_shows_dashes);
  RUN_TEST(test_vesc_a_rows);
  RUN_TEST(test_vesc_a_fault_and_stale);
  RUN_TEST(test_vesc_a_latched_fault_with_age);
  RUN_TEST(test_vesc_b_rows);
  RUN_TEST(test_vesc_c_rows);
  RUN_TEST(test_gnss_rows);
  RUN_TEST(test_gnss_coordinates_rounding_and_hemispheres);
  RUN_TEST(test_gnss_without_fix_or_receiver);
  RUN_TEST(test_gnss_extreme_values);
  RUN_TEST(test_sys_rows);
  RUN_TEST(test_sys_variants);
  RUN_TEST(test_frame_dispatch_and_determinism);
  RUN_TEST(test_extreme_values_respect_budgets_and_buffers);
  RUN_TEST(test_never_received_state);
  RUN_TEST(test_boot_frame);
  RUN_TEST(test_layout_geometry);
  return UNITY_END();
}
