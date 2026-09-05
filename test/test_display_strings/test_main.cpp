// Unity tests (pio test -e native) for the pure display string builder.
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; includes only Arduino-free headers.
#include <unity.h>

#include <stdint.h>
#include <string.h>

#include "display_strings.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- fixtures
static const uint32_t NOW = 100000;

// Speed expectations depend on the configured unit (config.h SPEED_UNIT_KNOTS, overridable with -D).
#if SPEED_UNIT_KNOTS
static const char *const kSpeed5144 = "10.0";  // 5144 mm/s * 0.00194384 = 9.999 kn
static const int32_t kBelowMinMmS = 100;       // 0.19 kn < SPEED_MIN_SHOW (0.3)
#else
static const char *const kSpeed5144 = "18.5";  // 5144 mm/s * 0.0036 = 18.52 km/h
static const int32_t kBelowMinMmS = 50;        // 0.18 km/h < SPEED_MIN_SHOW (0.3)
#endif

// VESC 48.2 V, 12.4 A in, 35 A motor, id 74, Tfet 41 C, all STATUS fresh; CAN running.
static void fill_vesc(SharedState &s, uint32_t t) {
  s.vesc.v_in_ema = 48.2f;
  s.vesc.i_in_ema = 12.4f;
  s.vesc.i_motor_ema = 35.0f;
  s.vesc.t.temp_fet = 41.0f;
  s.vesc.locked_id = 74;
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) s.vesc.t.t_ms[i] = t;
  s.can.state = CAN_STATE_RUNNING;
}

// GNSS running at 38400, 3D fix, 9 sats, 5144 mm/s (= 10.0 kn), sAcc 0.3 m/s, 12:34 UTC.
static void fill_gnss(SharedState &s, uint32_t t) {
  s.gnss.phase = GNSS_PHASE_RUN;
  s.gnss.baud = 38400;
  s.gnss.last_pvt_ms = t;
  s.gnss.fix_type = 3;
  s.gnss.fix_ok = true;
  s.gnss.num_sv = 9;
  s.gnss.gspeed_mm_s = 5144;
  s.gnss.sacc_mm_s = 300;
  s.gnss.time_valid = true;
  s.gnss.hour = 12;
  s.gnss.min = 34;
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

static DisplayStrings build(const SharedState &s, uint32_t now, uint32_t partials = 0) {
  DisplayStrings d;
  memset(&d, 0xAA, sizeof d);  // prove the builder fully initialises the struct
  display_build_strings(s, now, partials, d);
  return d;
}

// ---------------------------------------------------------------- VESC tiles
static void test_vesc_values_fresh() {
  DisplayStrings d = build(make_state(NOW), NOW);
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
  TEST_ASSERT_EQUAL_STRING("12.4", d.i_in);
  TEST_ASSERT_EQUAL_STRING("35", d.i_motor);
  TEST_ASSERT_EQUAL_STRING("598", d.power);  // 48.2 * 12.4 = 597.68
}

static void test_status5_stale_hides_voltage_and_power_only() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_5] = NOW - VESC_STALE_R1_MS - 1;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.v_in);
  TEST_ASSERT_EQUAL_STRING("--", d.power);
  TEST_ASSERT_EQUAL_STRING("12.4", d.i_in);  // STATUS_4 still fresh
  TEST_ASSERT_EQUAL_STRING("35", d.i_motor);
}

static void test_status4_stale_hides_current_power_and_tfet() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_4] = NOW - VESC_STALE_R1_MS - 1;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.i_in);
  TEST_ASSERT_EQUAL_STRING("--", d.power);
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
  TEST_ASSERT_EQUAL_STRING("VESC 74  CAN RUN  Tf --C", d.status_l);
}

static void test_status1_stale_hides_motor_current() {
  SharedState s = make_state(NOW);
  s.vesc.t.t_ms[VESC_IDX_STATUS_1] = 0;  // never received
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.i_motor);
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
}

static void test_exactly_at_stale_limit_is_fresh() {
  SharedState s = make_state(NOW - VESC_STALE_R1_MS);  // age == limit -> still fresh (<=)
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
}

static void test_never_received_all_dashes() {
  SharedState s;
  memset(&s, 0, sizeof s);
  s.vesc.locked_id = -1;
  s.can.state = CAN_STATE_UNINSTALLED;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.v_in);
  TEST_ASSERT_EQUAL_STRING("--", d.i_in);
  TEST_ASSERT_EQUAL_STRING("--", d.i_motor);
  TEST_ASSERT_EQUAL_STRING("--", d.power);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  TEST_ASSERT_EQUAL_STRING("SAT --", d.sats);
  TEST_ASSERT_EQUAL_STRING("NO GNSS", d.fix);
  TEST_ASSERT_EQUAL_STRING("", d.utc);
  TEST_ASSERT_EQUAL_STRING("VESC --  CAN UNINST  Tf --C", d.status_l);
  TEST_ASSERT_EQUAL_STRING("GNSS AUTOBAUD  #0", d.status_r);
}

static void test_signed_current_and_regen_power() {
  SharedState s = make_state(NOW);
  s.vesc.i_in_ema = -3.14f;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("-3.1", d.i_in);
  TEST_ASSERT_EQUAL_STRING("-151", d.power);  // 48.2 * -3.14 = -151.3 (regen)
}

static void test_negative_zero_is_normalised() {
  SharedState s = make_state(NOW);
  s.vesc.i_in_ema = -0.01f;
  s.vesc.i_motor_ema = -0.4f;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", d.i_in);   // not "-0.0"
  TEST_ASSERT_EQUAL_STRING("0", d.i_motor);  // not "-0"
  TEST_ASSERT_EQUAL_STRING("0", d.power);    // 48.2 * -0.01 = -0.48 -> "0"
}

// ---------------------------------------------------------------- GNSS / speed
static void test_speed_knots_from_mm_s() {
  DisplayStrings d = build(make_state(NOW), NOW);
  TEST_ASSERT_EQUAL_STRING(kSpeed5144, d.speed);
  TEST_ASSERT_EQUAL_STRING(SPEED_UNIT_STR, d.speed_unit);
#if SPEED_UNIT_KNOTS
  TEST_ASSERT_EQUAL_STRING("kn", d.speed_unit);
#endif
  TEST_ASSERT_EQUAL_STRING("SAT 09", d.sats);
  TEST_ASSERT_EQUAL_STRING("3D FIX", d.fix);
  TEST_ASSERT_EQUAL_STRING("12:34 UTC", d.utc);
}

static void test_speed_below_min_show_is_zero() {
  SharedState s = make_state(NOW);
  s.gnss.gspeed_mm_s = kBelowMinMmS;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("0.0", d.speed);
}

static void test_speed_hidden_when_sacc_too_large() {
  SharedState s = make_state(NOW);
  s.gnss.sacc_mm_s = 3000;  // > GNSS_MAX_SACC_MM_S (2000)
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  TEST_ASSERT_EQUAL_STRING("3D FIX", d.fix);  // the fix line itself is unaffected
}

static void test_speed_hidden_without_fix() {
  SharedState s = make_state(NOW);
  s.gnss.fix_type = 0;
  s.gnss.fix_ok = false;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  TEST_ASSERT_EQUAL_STRING("NO FIX", d.fix);
  TEST_ASSERT_EQUAL_STRING("SAT 09", d.sats);  // sats are still counted while searching
}

static void test_speed_hidden_when_fix_ok_false_despite_fix_type() {
  SharedState s = make_state(NOW);
  s.gnss.fix_ok = false;  // e.g. gnssFixOK flag clear on PROTVER >= 20
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
}

static void test_fix_2d_and_4() {
  SharedState s = make_state(NOW);
  s.gnss.fix_type = 2;
  DisplayStrings d2 = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("2D FIX", d2.fix);
  s.gnss.fix_type = 4;
  DisplayStrings d4 = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("3D FIX", d4.fix);
  s.gnss.fix_type = 5;  // time-only
  DisplayStrings d5 = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("NO FIX", d5.fix);
}

static void test_no_gnss_ever() {
  SharedState s = make_state(NOW);
  s.gnss.last_pvt_ms = 0;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("NO GNSS", d.fix);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  TEST_ASSERT_EQUAL_STRING("SAT --", d.sats);
  TEST_ASSERT_EQUAL_STRING("", d.utc);
}

static void test_gnss_stale_is_no_data() {
  SharedState s = make_state(NOW);
  s.gnss.last_pvt_ms = NOW - GNSS_STALE_MS - 1;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("NO DATA", d.fix);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  TEST_ASSERT_EQUAL_STRING("SAT --", d.sats);
  TEST_ASSERT_EQUAL_STRING("", d.utc);
}

static void test_utc_empty_without_valid_time() {
  SharedState s = make_state(NOW);
  s.gnss.time_valid = false;
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("", d.utc);
  TEST_ASSERT_EQUAL_STRING(kSpeed5144, d.speed);  // time validity does not gate speed
}

// ---------------------------------------------------------------- status bar
static void test_status_bar_running() {
  DisplayStrings d = build(make_state(NOW), NOW, 123);
  TEST_ASSERT_EQUAL_STRING("VESC 74  CAN RUN  Tf 41C", d.status_l);
  TEST_ASSERT_EQUAL_STRING("GNSS 38400  #123", d.status_r);
}

static void test_status_r_contains_partial_counter() {
  DisplayStrings d = build(make_state(NOW), NOW, 7);
  TEST_ASSERT_NOT_NULL(strstr(d.status_r, "#7"));
  TEST_ASSERT_EQUAL_STRING("GNSS 38400  #7", d.status_r);
}

static void test_status_r_shows_phase_when_not_running() {
  SharedState s = make_state(NOW);
  s.gnss.phase = GNSS_PHASE_DETECT;
  DisplayStrings d_det = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("GNSS DETECT  #0", d_det.status_r);
  s.gnss.phase = GNSS_PHASE_CONFIGURE;
  DisplayStrings d_cfg = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("GNSS CONFIG  #0", d_cfg.status_r);
  s.gnss.phase = GNSS_PHASE_AUTOBAUD;
  DisplayStrings d_ab = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("GNSS AUTOBAUD  #0", d_ab.status_r);
}

static void test_status_l_can_states() {
  SharedState s = make_state(NOW);
  s.can.state = CAN_STATE_BUS_OFF;
  DisplayStrings d_bo = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 74  CAN BUSOFF  Tf 41C", d_bo.status_l);
  s.can.state = CAN_STATE_RECOVERING;
  DisplayStrings d_rc = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 74  CAN RECOV  Tf 41C", d_rc.status_l);
  s.can.state = CAN_STATE_STOPPED;
  DisplayStrings d_st = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 74  CAN STOP  Tf 41C", d_st.status_l);
  s.can.state = CAN_STATE_UNINSTALLED;
  DisplayStrings d_un = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("VESC 74  CAN UNINST  Tf 41C", d_un.status_l);
}

// ---------------------------------------------------------------- time base
static void test_wraparound_freshness() {
  // millis() wrapped: timestamps just below UINT32_MAX, now small. Ages are
  // 601 ms (VESC) and 1101 ms (GNSS): both fresh.
  const uint32_t now = 100;
  SharedState s = make_state(0xFFFFFFFFu - 500u);
  s.gnss.last_pvt_ms = 0xFFFFFFFFu - 1000u;
  DisplayStrings d = build(s, now);
  TEST_ASSERT_EQUAL_STRING("48.2", d.v_in);
  TEST_ASSERT_EQUAL_STRING("598", d.power);
  TEST_ASSERT_EQUAL_STRING(kSpeed5144, d.speed);
  TEST_ASSERT_EQUAL_STRING("3D FIX", d.fix);
}

static void test_wraparound_stale() {
  // Same wrap, but the VESC timestamp is older than the R1 window.
  const uint32_t now = 100;
  SharedState s = make_state(0xFFFFFFFFu - VESC_STALE_R1_MS);  // age = R1 + 101
  DisplayStrings d = build(s, now);
  TEST_ASSERT_EQUAL_STRING("--", d.v_in);
  TEST_ASSERT_EQUAL_STRING("--", d.power);
}

static void test_future_timestamp_is_stale_not_fresh() {
  // A timestamp "ahead" of now (producer stamped after the snapshot's now) wraps
  // to a huge age with unsigned math and must read as stale, never as negative age.
  SharedState s = make_state(NOW + 10);
  DisplayStrings d = build(s, NOW);
  TEST_ASSERT_EQUAL_STRING("--", d.v_in);
}

// ---------------------------------------------------------------- determinism / safety
static void test_deterministic_and_fully_initialised() {
  SharedState s = make_state(NOW);
  DisplayStrings a, b;
  memset(&a, 0x11, sizeof a);
  memset(&b, 0xEE, sizeof b);
  display_build_strings(s, NOW, 5, a);
  display_build_strings(s, NOW, 5, b);
  TEST_ASSERT_EQUAL_MEMORY(&a, &b, sizeof a);  // tail bytes are zeroed, so memcmp works
}

static void test_counter_change_only_changes_status_r() {
  SharedState s = make_state(NOW);
  DisplayStrings a = build(s, NOW, 5);
  DisplayStrings b = build(s, NOW, 6);
  TEST_ASSERT_TRUE(memcmp(&a, &b, sizeof a) != 0);
  memset(a.status_r, 0, sizeof a.status_r);  // blank the one field that may differ ...
  memset(b.status_r, 0, sizeof b.status_r);
  TEST_ASSERT_EQUAL_MEMORY(&a, &b, sizeof a);  // ... and everything else is identical
}

// Contract used by the display task: the panel shows "#N" while the live count is
// already N+1, so it compares against a frame built with the SHOWN counter. Same
// state + shown counter -> identical (no redraw); a real change -> different.
static void test_change_detection_with_shown_counter() {
  SharedState s = make_state(NOW);
  DisplayStrings on_panel = build(s, NOW, 5);     // drawn with partials == 5, live count is now 6
  DisplayStrings probe = build(s, NOW + 1000, 5);  // next tick, nothing changed, built with the shown count
  TEST_ASSERT_EQUAL_MEMORY(&on_panel, &probe, sizeof probe);
  s.vesc.i_in_ema = 12.6f;                          // a real change must still be detected
  DisplayStrings changed = build(s, NOW + 1000, 5);
  TEST_ASSERT_TRUE(memcmp(&on_panel, &changed, sizeof changed) != 0);
}

struct Guarded {
  char pre[16];
  DisplayStrings d;
  char post[16];
};

static void test_extreme_values_do_not_overflow() {
  SharedState s = make_state(NOW);
  s.vesc.v_in_ema = 1.0e30f;
  s.vesc.i_in_ema = -1.0e30f;
  s.vesc.i_motor_ema = -123456789.0f;
  s.vesc.t.temp_fet = 1.0e9f;
  s.vesc.locked_id = 254;
  s.can.state = 12345;  // unknown state value
  s.gnss.gspeed_mm_s = 0x7FFFFFFF;
  s.gnss.sacc_mm_s = 0;
  s.gnss.num_sv = 255;
  s.gnss.hour = 255;
  s.gnss.min = 255;
  s.gnss.baud = 0xFFFFFFFFu;
  s.gnss.phase = 200;  // unknown phase

  Guarded g;
  memset(&g, 0xAA, sizeof g);
  display_build_strings(s, NOW, 0xFFFFFFFFu, g.d);

  // Every string is NUL-terminated inside its own buffer.
  TEST_ASSERT_TRUE(strnlen(g.d.speed, sizeof g.d.speed) < sizeof g.d.speed);
  TEST_ASSERT_TRUE(strnlen(g.d.speed_unit, sizeof g.d.speed_unit) < sizeof g.d.speed_unit);
  TEST_ASSERT_TRUE(strnlen(g.d.sats, sizeof g.d.sats) < sizeof g.d.sats);
  TEST_ASSERT_TRUE(strnlen(g.d.fix, sizeof g.d.fix) < sizeof g.d.fix);
  TEST_ASSERT_TRUE(strnlen(g.d.utc, sizeof g.d.utc) < sizeof g.d.utc);
  TEST_ASSERT_TRUE(strnlen(g.d.v_in, sizeof g.d.v_in) < sizeof g.d.v_in);
  TEST_ASSERT_TRUE(strnlen(g.d.i_in, sizeof g.d.i_in) < sizeof g.d.i_in);
  TEST_ASSERT_TRUE(strnlen(g.d.i_motor, sizeof g.d.i_motor) < sizeof g.d.i_motor);
  TEST_ASSERT_TRUE(strnlen(g.d.power, sizeof g.d.power) < sizeof g.d.power);
  TEST_ASSERT_TRUE(strnlen(g.d.status_l, sizeof g.d.status_l) < sizeof g.d.status_l);
  TEST_ASSERT_TRUE(strnlen(g.d.status_r, sizeof g.d.status_r) < sizeof g.d.status_r);
  // Truncated, but the sign survived, the counter is intact and the guards are untouched.
  TEST_ASSERT_EQUAL_CHAR('-', g.d.i_in[0]);
  TEST_ASSERT_EQUAL_STRING(SPEED_UNIT_STR, g.d.speed_unit);
  TEST_ASSERT_NOT_NULL(strstr(g.d.status_r, "#4294967295"));
  for (size_t i = 0; i < sizeof g.pre; ++i) {
    TEST_ASSERT_EQUAL_HEX8(0xAA, (uint8_t)g.pre[i]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, (uint8_t)g.post[i]);
  }
}

static void test_boot_strings() {
  DisplayStrings d;
  memset(&d, 0x55, sizeof d);
  display_boot_strings(d);
  TEST_ASSERT_EQUAL_STRING("--", d.speed);
  TEST_ASSERT_EQUAL_STRING(SPEED_UNIT_STR, d.speed_unit);
  TEST_ASSERT_EQUAL_STRING("SAT --", d.sats);
  TEST_ASSERT_EQUAL_STRING("NO GNSS", d.fix);
  TEST_ASSERT_EQUAL_STRING("", d.utc);
  TEST_ASSERT_EQUAL_STRING("--", d.v_in);
  TEST_ASSERT_EQUAL_STRING("--", d.i_in);
  TEST_ASSERT_EQUAL_STRING("--", d.i_motor);
  TEST_ASSERT_EQUAL_STRING("--", d.power);
  TEST_ASSERT_EQUAL_STRING("BOOT", d.status_l);
  TEST_ASSERT_NOT_NULL(strstr(d.status_r, FW_VERSION));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_vesc_values_fresh);
  RUN_TEST(test_status5_stale_hides_voltage_and_power_only);
  RUN_TEST(test_status4_stale_hides_current_power_and_tfet);
  RUN_TEST(test_status1_stale_hides_motor_current);
  RUN_TEST(test_exactly_at_stale_limit_is_fresh);
  RUN_TEST(test_never_received_all_dashes);
  RUN_TEST(test_signed_current_and_regen_power);
  RUN_TEST(test_negative_zero_is_normalised);
  RUN_TEST(test_speed_knots_from_mm_s);
  RUN_TEST(test_speed_below_min_show_is_zero);
  RUN_TEST(test_speed_hidden_when_sacc_too_large);
  RUN_TEST(test_speed_hidden_without_fix);
  RUN_TEST(test_speed_hidden_when_fix_ok_false_despite_fix_type);
  RUN_TEST(test_fix_2d_and_4);
  RUN_TEST(test_no_gnss_ever);
  RUN_TEST(test_gnss_stale_is_no_data);
  RUN_TEST(test_utc_empty_without_valid_time);
  RUN_TEST(test_status_bar_running);
  RUN_TEST(test_status_r_contains_partial_counter);
  RUN_TEST(test_status_r_shows_phase_when_not_running);
  RUN_TEST(test_status_l_can_states);
  RUN_TEST(test_wraparound_freshness);
  RUN_TEST(test_wraparound_stale);
  RUN_TEST(test_future_timestamp_is_stale_not_fresh);
  RUN_TEST(test_deterministic_and_fully_initialised);
  RUN_TEST(test_counter_change_only_changes_status_r);
  RUN_TEST(test_change_detection_with_shown_counter);
  RUN_TEST(test_extreme_values_do_not_overflow);
  RUN_TEST(test_boot_strings);
  return UNITY_END();
}
