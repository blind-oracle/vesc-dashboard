// Unity tests (pio test -e native) for the pure OLED display string builder.
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; includes only Arduino-free headers.
#include <unity.h>

#include <string.h>

#include "display_strings_oled.h"

void setUp() {}
void tearDown() {}

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

// VESC 48.2 V, 12.4 A in, 35 A motor, id 74, all STATUS fresh; CAN running.
static void fill_vesc(SharedState &s, uint32_t t) {
  s.vesc.v_in_ema = 48.2f;
  s.vesc.i_in_ema = 12.4f;
  s.vesc.i_motor_ema = 35.0f;
  s.vesc.locked_id = 74;
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) s.vesc.t.t_ms[i] = t;
  s.can.state = CAN_STATE_RUNNING;
}

// GNSS running, 3D fix, 9 sats, 5144 mm/s (= 10.0 kn), sAcc 0.3 m/s.
static void fill_gnss(SharedState &s, uint32_t t) {
  s.gnss.phase = GNSS_PHASE_RUN;
  s.gnss.baud = 38400;
  s.gnss.last_pvt_ms = t;
  s.gnss.fix_type = 3;
  s.gnss.fix_ok = true;
  s.gnss.num_sv = 9;
  s.gnss.gspeed_mm_s = 5144;
  s.gnss.sacc_mm_s = 300;
}

static SharedState make_state(uint32_t t) {
  SharedState s;
  memset(&s, 0, sizeof s);
  s.vesc.locked_id = -1;
  s.can.state = CAN_STATE_UNINSTALLED;
  s.gnss.prot_ver_x100 = -1;
  fill_vesc(s, t);
  fill_gnss(s, t);
  return s;
}

static OledStrings build(const SharedState &s, uint32_t now) {
  OledStrings d;
  memset(&d, 0xAA, sizeof d);  // prove the builder fully initialises the struct
  oled_build_strings(s, now, d);
  return d;
}

// Every value cell holds at most 4 glyphs, every right-column cell at most 8.
static void assert_lengths(const OledStrings &d) {
  TEST_ASSERT_TRUE(strnlen(d.speed, sizeof d.speed) <= 4);
  TEST_ASSERT_TRUE(strnlen(d.v_in, sizeof d.v_in) <= 4);
  TEST_ASSERT_TRUE(strnlen(d.i_in, sizeof d.i_in) <= 4);
  TEST_ASSERT_TRUE(strnlen(d.i_motor, sizeof d.i_motor) <= 4);
  TEST_ASSERT_TRUE(strnlen(d.power, sizeof d.power) <= 4);
  TEST_ASSERT_TRUE(strnlen(d.unit, sizeof d.unit) <= 8);
  TEST_ASSERT_TRUE(strnlen(d.fix, sizeof d.fix) <= 8);
  TEST_ASSERT_TRUE(strnlen(d.can, sizeof d.can) <= 8);
}

// ---------------------------------------------------------------- VESC values
static void test_vesc_values_fresh() {
  OledStrings d = build(make_state(NOW), NOW);
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
  TEST_ASSERT_EQUAL_STRING("12.4", d.i_in);
  TEST_ASSERT_EQUAL_STRING("35", d.i_motor);
  TEST_ASSERT_EQUAL_STRING("598", d.power);  // 48.2 * 12.4 = 597.68
  TEST_ASSERT_EQUAL_STRING("VESC 74", d.can);
  assert_lengths(d);
}

static void test_i_in_negative_has_no_decimals() {
  SharedState s = make_state(NOW);
  s.vesc.i_in_ema = -12.4f;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-12", d.i_in);
  TEST_ASSERT_EQUAL_STRING("-598", d.power);  // 48.2 * -12.4 = -597.68 (regen)
}

static void test_i_in_three_digits_has_no_decimals() {
  SharedState s = make_state(NOW);
  s.vesc.i_in_ema = 123.4f;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("123", d.i_in);
}

static void test_i_in_rounds_up_to_one_decimal_that_still_fits() {
  SharedState s = make_state(NOW);
  s.vesc.i_in_ema = 9.96f;  // "%.1f" -> "10.0", 4 chars, keeps the decimal
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("10.0", d.i_in);
}

static void test_i_in_rounding_past_100_drops_the_decimal() {
  SharedState s = make_state(NOW);
  s.vesc.i_in_ema = 99.96f;  // "%.1f" would be "100.0" (5 chars) -> "100"
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("100", d.i_in);
}

static void test_v_in_three_digits_has_no_decimals() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 100.4f;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("100", d.v_in);
}

static void test_power_negative_four_chars() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 48.0f;
  s.vesc.i_in_ema = -10.0f;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-480", d.power);
  TEST_ASSERT_EQUAL_STRING("-10", d.i_in);
}

static void test_power_large_negative_in_kilowatts() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 48.0f;
  s.vesc.i_in_ema = -100.0f;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-5k", d.power);  // -4800 W
  TEST_ASSERT_EQUAL_STRING("-100", d.i_in);
}

static void test_power_five_digits_in_kilowatts() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 50.0f;
  s.vesc.i_in_ema = 246.9f;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("12k", d.power);  // 12345 W
  TEST_ASSERT_EQUAL_STRING("247", d.i_in);
}

static void test_power_four_digits_stays_in_watts() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 50.0f;
  s.vesc.i_in_ema = 60.0f;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3000", d.power);
}

static void test_motor_current_kiloamps() {
  SharedState s = make_state(NOW);
  s.vesc.i_motor_ema = 1234.0f;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("1k", d.i_motor);
  s.vesc.i_motor_ema = 999.0f;  // below the kiloamp threshold: whole amps
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("999", d.i_motor);
  s.vesc.i_motor_ema = 1000.0f;
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("1k", d.i_motor);
  s.vesc.i_motor_ema = -999.0f;
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-999", d.i_motor);
  s.vesc.i_motor_ema = -999.6f;  // "%.0f" -> "-1000" (5 chars) -> "-1k"
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-1k", d.i_motor);
}

static void test_negative_zero_is_normalised() {
  SharedState s = make_state(NOW);
  s.vesc.i_in_ema = -0.01f;
  s.vesc.i_motor_ema = -0.4f;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0", d.i_in);     // negative -> whole amps -> "-0" -> "0"
  TEST_ASSERT_EQUAL_STRING("0", d.i_motor);
  TEST_ASSERT_EQUAL_STRING("0", d.power);    // 48.2 * -0.01 = -0.48 -> "-0" -> "0"
}

static void test_status5_stale_hides_voltage_and_power_only() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_5] = NOW - VESC_STALE_R1_MS - 1;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.v_in);
  TEST_ASSERT_EQUAL_STRING("--", d.power);
  TEST_ASSERT_EQUAL_STRING("12.4", d.i_in);  // STATUS_4 still fresh
  TEST_ASSERT_EQUAL_STRING("35", d.i_motor);
  TEST_ASSERT_EQUAL_STRING("VESC 74", d.can); // STATUS_1 still fresh
}

static void test_status4_stale_hides_current_and_power() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_4] = NOW - VESC_STALE_R1_MS - 1;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.i_in);
  TEST_ASSERT_EQUAL_STRING("--", d.power);
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
}

static void test_status1_never_received_hides_motor_current() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_1] = 0;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.i_motor);
  TEST_ASSERT_EQUAL_STRING("VESC 74", d.can);  // STATUS_5 still fresh
}

static void test_exactly_at_stale_limit_is_fresh() {
  SharedState s = make_state(NOW - VESC_STALE_R1_MS);  // age == limit -> still fresh (<=)
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
}

// ---------------------------------------------------------------- speed
static void test_speed_from_mm_s_and_unit() {
  OledStrings d = build(make_state(NOW), NOW);
  TEST_ASSERT_EQUAL_STRING(kSpeed5144, d.speed);
  TEST_ASSERT_EQUAL_STRING(kUnit, d.unit);
  TEST_ASSERT_EQUAL_STRING(SPEED_UNIT_STR, d.unit);
}

static void test_speed_at_or_above_100_has_no_decimals() {
  SharedState s = make_state(NOW);
  s.gnss.gspeed_mm_s = 60000;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING(kSpeed60000, d.speed);
}

static void test_speed_below_min_show_is_zero() {
  SharedState s = make_state(NOW);
  s.gnss.gspeed_mm_s = kBelowMinMmS;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", d.speed);
  s.gnss.gspeed_mm_s = -5144;  // negative oddity is clamped, never "-10.0"
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", d.speed);
}

static void test_speed_hidden_when_sacc_too_large() {
  SharedState s = make_state(NOW);
  s.gnss.sacc_mm_s = 3000;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  TEST_ASSERT_EQUAL_STRING("3D  9sv", d.fix);  // the fix line still reports the receiver state
}

static void test_speed_hidden_without_fix() {
  SharedState s = make_state(NOW);
  s.gnss.fix_type = 0;
  s.gnss.fix_ok = false;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  TEST_ASSERT_EQUAL_STRING("NO FIX", d.fix);
}

static void test_speed_hidden_when_fix_ok_false_despite_fix_type() {
  SharedState s = make_state(NOW);
  s.gnss.fix_ok = false;  // gnssFixOK cleared (e.g. DOP mask) with fixType 3
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
}

// ---------------------------------------------------------------- fix line
static void test_fix_3d_with_satellite_count() {
  SharedState s = make_state(NOW);
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D  9sv", d.fix);  // %2u pads a single digit
  s.gnss.num_sv = 12;
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D 12sv", d.fix);
  s.gnss.fix_type = 4;  // GNSS + dead reckoning counts as 3D
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D 12sv", d.fix);
}

static void test_fix_2d() {
  SharedState s = make_state(NOW);
  s.gnss.fix_type = 2;
  s.gnss.num_sv = 7;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("2D  7sv", d.fix);
  TEST_ASSERT_EQUAL_STRING(kSpeed5144, d.speed);  // fix_ok true: speed shown
}

static void test_fix_time_only_is_no_fix() {
  SharedState s = make_state(NOW);
  s.gnss.fix_type = 5;
  s.gnss.fix_ok = false;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("NO FIX", d.fix);
}

static void test_no_gnss_ever() {
  SharedState s = make_state(NOW);
  s.gnss.last_pvt_ms = 0;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("NO GNSS", d.fix);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
}

static void test_gnss_stale_is_no_data() {
  SharedState s = make_state(NOW);
  s.gnss.last_pvt_ms = NOW - GNSS_STALE_MS - 1;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("NO DATA", d.fix);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  s.gnss.last_pvt_ms = NOW - GNSS_STALE_MS;  // exactly at the limit: still live
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D  9sv", d.fix);
}

// ---------------------------------------------------------------- CAN line
static void test_can_states() {
  SharedState s = make_state(NOW);
  OledStrings d;

  s.can.state = CAN_STATE_STOPPED;
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", d.can);
  s.can.state = CAN_STATE_BUS_OFF;
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", d.can);
  s.can.state = CAN_STATE_UNINSTALLED;
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", d.can);
  s.can.state = 12345;  // unknown driver state
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", d.can);

  s.can.state = CAN_STATE_RECOVERING;
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN RECV", d.can);

  s.can.state = CAN_STATE_RUNNING;
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 74", d.can);
  s.vesc.locked_id = 254;
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 254", d.can);
}

static void test_can_running_without_fresh_status_is_idle() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_1] = NOW - VESC_STALE_R1_MS - 1;
  s.vesc.t.t_ms[VESC_IDX_STATUS_5] = NOW - VESC_STALE_R1_MS - 1;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN idle", d.can);
  TEST_ASSERT_EQUAL_STRING("12.4", d.i_in);  // STATUS_4 alone does not count as "VESC seen"
  s.vesc.t.t_ms[VESC_IDX_STATUS_5] = NOW;    // either STATUS_1 or STATUS_5 fresh -> VESC shown
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 74", d.can);
}

static void test_can_values_hidden_while_bus_off() {
  // The driver state and the freshness windows are independent: a STOPPED bus
  // with fresh timestamps (just stopped) still shows the fresh values.
  SharedState s = make_state(NOW);
  s.can.state = CAN_STATE_BUS_OFF;
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", d.can);
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
}

// ---------------------------------------------------------------- freshness / wrap
static void test_wraparound_freshness() {
  SharedState s = make_state(0xFFFFFF00u);  // stamped just before millis() wraps
  OledStrings d = build(s, 0x00000100u);     // 512 ms later, after the wrap
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
  TEST_ASSERT_EQUAL_STRING(kSpeed5144, d.speed);
}

static void test_future_timestamp_is_stale_not_fresh() {
  SharedState s = make_state(NOW + 10);  // producer clock ahead of the display's "now"
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.v_in);  // unsigned age ~4e9 -> stale, never a crash or a lie
  TEST_ASSERT_EQUAL_STRING("NO DATA", d.fix);
}

// ---------------------------------------------------------------- determinism / lengths
static void test_deterministic_and_fully_initialised() {
  SharedState s = make_state(NOW);
  OledStrings a = build(s, NOW);
  OledStrings b;
  memset(&b, 0x55, sizeof b);
  oled_build_strings(s, NOW + 500, b);  // same state a little later: byte-identical
  TEST_ASSERT_EQUAL_MEMORY(&a, &b, sizeof a);
  // tail bytes after each NUL are zero (memset before formatting)
  for (size_t i = strlen(a.speed) + 1; i < sizeof a.speed; ++i) TEST_ASSERT_EQUAL_HEX8(0, (uint8_t)a.speed[i]);
  for (size_t i = strlen(a.can) + 1; i < sizeof a.can; ++i) TEST_ASSERT_EQUAL_HEX8(0, (uint8_t)a.can[i]);
  // a real change is detected
  s.vesc.i_in_ema = 12.6f;
  OledStrings c = build(s, NOW);
  TEST_ASSERT_TRUE(memcmp(&a, &c, sizeof c) != 0);
}

struct Guarded {
  char pre[16];
  OledStrings d;
  char post[16];
};

static void test_extreme_values_respect_cell_widths() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 1.0e9f;
  s.vesc.i_in_ema = -1.0e9f;
  s.vesc.i_motor_ema = 1.0e9f;
  s.vesc.locked_id = 254;
  s.gnss.gspeed_mm_s = 0x7FFFFFFF;
  s.gnss.sacc_mm_s = 0;
  s.gnss.num_sv = 255;

  Guarded g;
  memset(&g, 0xAA, sizeof g);
  oled_build_strings(s, NOW, g.d);
  assert_lengths(g.d);
  TEST_ASSERT_EQUAL_STRING("3D 255sv", g.d.fix);  // exactly 8
  TEST_ASSERT_EQUAL_STRING("VESC 254", g.d.can);  // exactly 8
  TEST_ASSERT_EQUAL_CHAR('-', g.d.i_in[0]);        // the sign survives
  TEST_ASSERT_EQUAL_CHAR('-', g.d.power[0]);
  // Past +-999.5k even the 'k' form does not fit: saturate instead of clipping
  // "1000000k" to a plausible-looking "1000".
  TEST_ASSERT_EQUAL_STRING("MAX", g.d.v_in);
  TEST_ASSERT_EQUAL_STRING("-MAX", g.d.i_in);
  TEST_ASSERT_EQUAL_STRING("MAX", g.d.i_motor);
  TEST_ASSERT_EQUAL_STRING("-MAX", g.d.power);
  TEST_ASSERT_EQUAL_STRING("MAX", g.d.speed);     // INT32_MAX mm/s = 4.2e6 kn / 7.7e6 km/h
  for (size_t i = 0; i < sizeof g.pre; ++i) {
    TEST_ASSERT_EQUAL_HEX8(0xAA, (uint8_t)g.pre[i]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, (uint8_t)g.post[i]);
  }

  s.vesc.v_in_ema = -1.0e9f;
  s.vesc.i_in_ema = 1.0e9f;
  s.vesc.i_motor_ema = -1.0e9f;
  s.gnss.gspeed_mm_s = -0x7FFFFFFF;
  memset(&g, 0xAA, sizeof g);
  oled_build_strings(s, NOW, g.d);
  assert_lengths(g.d);
  TEST_ASSERT_EQUAL_STRING("0.0", g.d.speed);
  for (size_t i = 0; i < sizeof g.pre; ++i) {
    TEST_ASSERT_EQUAL_HEX8(0xAA, (uint8_t)g.pre[i]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, (uint8_t)g.post[i]);
  }
}

static void test_kilo_fallback_values() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 100.0f;
  s.vesc.i_in_ema = 999.0f;  // 99.9 kW
  OledStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("100k", d.power);
  TEST_ASSERT_EQUAL_STRING("999", d.i_in);
  s.vesc.i_in_ema = 9999.0f;  // "%.0f" = "9999" fits as-is
  d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("9999", d.i_in);
}

static void test_boot_strings() {
  OledStrings d;
  memset(&d, 0x55, sizeof d);
  oled_boot_strings(d);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  TEST_ASSERT_EQUAL_STRING(SPEED_UNIT_STR, d.unit);
  TEST_ASSERT_EQUAL_STRING("BOOT", d.fix);
  TEST_ASSERT_NOT_NULL(strstr(d.can, FW_VERSION));
  TEST_ASSERT_EQUAL_STRING("--", d.v_in);
  TEST_ASSERT_EQUAL_STRING("--", d.i_in);
  TEST_ASSERT_EQUAL_STRING("--", d.i_motor);
  TEST_ASSERT_EQUAL_STRING("--", d.power);
  assert_lengths(d);
  // The boot frame differs from any live frame (so the first live tick redraws)...
  SharedState s;
  memset(&s, 0, sizeof s);
  s.vesc.locked_id = -1;
  s.can.state = CAN_STATE_UNINSTALLED;
  OledStrings live = build(s, NOW);
  TEST_ASSERT_TRUE(memcmp(&d, &live, sizeof d) != 0);
  // ... and a never-received state is all dashes / NO GNSS / CAN OFF.
  TEST_ASSERT_EQUAL_STRING("--", live.speed);
  TEST_ASSERT_EQUAL_STRING("NO GNSS", live.fix);
  TEST_ASSERT_EQUAL_STRING("CAN OFF", live.can);
  TEST_ASSERT_EQUAL_STRING("--", live.v_in);
  TEST_ASSERT_EQUAL_STRING("--", live.power);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_vesc_values_fresh);
  RUN_TEST(test_i_in_negative_has_no_decimals);
  RUN_TEST(test_i_in_three_digits_has_no_decimals);
  RUN_TEST(test_i_in_rounds_up_to_one_decimal_that_still_fits);
  RUN_TEST(test_i_in_rounding_past_100_drops_the_decimal);
  RUN_TEST(test_v_in_three_digits_has_no_decimals);
  RUN_TEST(test_power_negative_four_chars);
  RUN_TEST(test_power_large_negative_in_kilowatts);
  RUN_TEST(test_power_five_digits_in_kilowatts);
  RUN_TEST(test_power_four_digits_stays_in_watts);
  RUN_TEST(test_motor_current_kiloamps);
  RUN_TEST(test_negative_zero_is_normalised);
  RUN_TEST(test_status5_stale_hides_voltage_and_power_only);
  RUN_TEST(test_status4_stale_hides_current_and_power);
  RUN_TEST(test_status1_never_received_hides_motor_current);
  RUN_TEST(test_exactly_at_stale_limit_is_fresh);
  RUN_TEST(test_speed_from_mm_s_and_unit);
  RUN_TEST(test_speed_at_or_above_100_has_no_decimals);
  RUN_TEST(test_speed_below_min_show_is_zero);
  RUN_TEST(test_speed_hidden_when_sacc_too_large);
  RUN_TEST(test_speed_hidden_without_fix);
  RUN_TEST(test_speed_hidden_when_fix_ok_false_despite_fix_type);
  RUN_TEST(test_fix_3d_with_satellite_count);
  RUN_TEST(test_fix_2d);
  RUN_TEST(test_fix_time_only_is_no_fix);
  RUN_TEST(test_no_gnss_ever);
  RUN_TEST(test_gnss_stale_is_no_data);
  RUN_TEST(test_can_states);
  RUN_TEST(test_can_running_without_fresh_status_is_idle);
  RUN_TEST(test_can_values_hidden_while_bus_off);
  RUN_TEST(test_wraparound_freshness);
  RUN_TEST(test_future_timestamp_is_stale_not_fresh);
  RUN_TEST(test_deterministic_and_fully_initialised);
  RUN_TEST(test_extreme_values_respect_cell_widths);
  RUN_TEST(test_kilo_fallback_values);
  RUN_TEST(test_boot_strings);
  return UNITY_END();
}
