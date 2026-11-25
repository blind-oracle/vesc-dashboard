// Native Unity tests (pio test -e native) for the peaks-since-boot recorder in include/peaks.h:
// the freshness and plausibility gates, magnitudes for motor current / rpm, drawn-only power and
// battery current, "first sample sets, later ones only raise", non-finite values, and the GNSS
// "usable fix" gate.
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; includes only framework-free headers.
#include <unity.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "peaks.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- fixtures
static const uint32_t NOW = 100000;

static float erpm_for(float mech_rpm) { return mech_rpm * (VESC_MOTOR_POLES / 2.0f); }

// A VESC state with STATUS_1/4/5 stamped at `stamp`: 48 V, 20 A drawn, 60 A motor, 1500 rpm, 40/30 C.
static VescState vesc_at(uint32_t stamp) {
  VescState v;
  memset(&v, 0, sizeof v);
  v.v_in_ema = 48.0f;
  v.i_in_ema = 20.0f;
  v.i_motor_ema = 60.0f;
  v.t.erpm = erpm_for(1500.0f);
  v.t.temp_fet = 40.0f;
  v.t.temp_motor = 30.0f;
  v.t.t_ms[VESC_IDX_STATUS_1] = stamp;
  v.t.t_ms[VESC_IDX_STATUS_4] = stamp;
  v.t.t_ms[VESC_IDX_STATUS_5] = stamp;
  return v;
}

static GnssState gnss_at(uint32_t stamp, int32_t speed_mm_s) {
  GnssState g;
  memset(&g, 0, sizeof g);
  g.last_pvt_ms = stamp;
  g.fix_ok = true;
  g.fix_type = 3;
  g.sacc_mm_s = 300;
  g.gspeed_mm_s = speed_mm_s;
  return g;
}

static PeaksState none() {
  PeaksState p;
  memset(&p, 0, sizeof p);
  return p;
}

static const uint8_t kVescBits = PEAK_POWER | PEAK_IBAT | PEAK_IMOT | PEAK_RPM | PEAK_TFET | PEAK_TMOT;

// ---------------------------------------------------------------- VESC
static void test_first_fresh_sample_sets_every_vesc_peak() {
  PeaksState p = none();
  peaks_vesc(p, vesc_at(NOW), NOW);
  TEST_ASSERT_EQUAL_HEX8(kVescBits, p.have);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 960.0f, p.power_w);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, p.i_bat_a);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, p.i_mot_a);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 1500.0f, p.rpm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, p.temp_fet_c);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, p.temp_mot_c);
}

static void test_max_is_kept_and_raised() {
  PeaksState p = none();
  VescState v = vesc_at(NOW);
  peaks_vesc(p, v, NOW);
  v.i_motor_ema = 25.0f;  // lower: the peak stays
  v.v_in_ema = 47.0f;
  v.i_in_ema = 10.0f;
  v.t.temp_fet = 35.0f;
  peaks_vesc(p, v, NOW + 20);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, p.i_mot_a);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 960.0f, p.power_w);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, p.i_bat_a);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, p.temp_fet_c);
  v.i_motor_ema = 150.0f;  // higher: raised
  v.t.temp_fet = 55.5f;
  peaks_vesc(p, v, NOW + 40);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, p.i_mot_a);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 55.5f, p.temp_fet_c);
}

static void test_reverse_counts_by_magnitude() {
  PeaksState p = none();
  VescState v = vesc_at(NOW);
  v.i_motor_ema = -80.0f;
  v.t.erpm = erpm_for(-2200.0f);
  peaks_vesc(p, v, NOW);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 80.0f, p.i_mot_a);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 2200.0f, p.rpm);
}

static void test_regen_never_counts_as_drawn() {
  PeaksState p = none();
  VescState v = vesc_at(NOW);
  v.i_in_ema = -15.0f;  // regenerating from the very first sample
  peaks_vesc(p, v, NOW);
  TEST_ASSERT_TRUE(p.have & PEAK_POWER);  // seen: it reads 0, not "never"
  TEST_ASSERT_TRUE(p.have & PEAK_IBAT);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, p.power_w);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, p.i_bat_a);
  v.i_in_ema = 30.0f;
  peaks_vesc(p, v, NOW + 20);
  v.i_in_ema = -100.0f;  // heavy regen afterwards neither lowers nor raises the drawn peaks
  peaks_vesc(p, v, NOW + 40);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, p.i_bat_a);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1440.0f, p.power_w);
}

static void test_stale_status_is_ignored() {
  PeaksState p = none();
  VescState v = vesc_at(NOW - VESC_STALE_R1_MS - 1);  // every status too old
  v.i_motor_ema = 500.0f;
  peaks_vesc(p, v, NOW);
  TEST_ASSERT_EQUAL_HEX8(0, p.have);

  // Never received at all (stamps 0)
  p = none();
  v = vesc_at(0);
  peaks_vesc(p, v, NOW);
  TEST_ASSERT_EQUAL_HEX8(0, p.have);

  // Only STATUS_1 fresh: motor current and rpm only
  p = none();
  v = vesc_at(NOW);
  v.t.t_ms[VESC_IDX_STATUS_4] = 0;
  v.t.t_ms[VESC_IDX_STATUS_5] = 0;
  peaks_vesc(p, v, NOW);
  TEST_ASSERT_EQUAL_HEX8(PEAK_IMOT | PEAK_RPM, p.have);

  // STATUS_4 fresh but STATUS_5 stale: current and temperatures, no power (a fresh current
  // times a stale voltage is no measurement)
  p = none();
  v = vesc_at(NOW);
  v.t.t_ms[VESC_IDX_STATUS_5] = NOW - VESC_STALE_R1_MS - 1;
  peaks_vesc(p, v, NOW);
  TEST_ASSERT_EQUAL_HEX8(PEAK_IMOT | PEAK_RPM | PEAK_IBAT | PEAK_TFET | PEAK_TMOT, p.have);
}

static void test_temperatures_need_plausible_readings() {
  PeaksState p = none();
  VescState v = vesc_at(NOW);
  v.t.temp_motor = -273.0f;  // no motor NTC wired
  v.t.temp_fet = 250.0f;     // implausible
  peaks_vesc(p, v, NOW);
  TEST_ASSERT_FALSE(p.have & PEAK_TMOT);
  TEST_ASSERT_FALSE(p.have & PEAK_TFET);
  TEST_ASSERT_TRUE(p.have & PEAK_IBAT);  // STATUS_4 itself was seen

  // A cold start below 0 C is a real value: the first one is recorded, a warmer one raises it
  v.t.temp_fet = -5.0f;
  peaks_vesc(p, v, NOW + 20);
  TEST_ASSERT_TRUE(p.have & PEAK_TFET);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -5.0f, p.temp_fet_c);
  v.t.temp_fet = -10.0f;
  peaks_vesc(p, v, NOW + 40);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -5.0f, p.temp_fet_c);
  v.t.temp_fet = 12.0f;
  peaks_vesc(p, v, NOW + 60);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.0f, p.temp_fet_c);
}

static void test_non_finite_values_are_ignored() {
  PeaksState p = none();
  VescState v = vesc_at(NOW);
  v.i_motor_ema = NAN;
  v.i_in_ema = INFINITY;
  peaks_vesc(p, v, NOW);
  TEST_ASSERT_FALSE(p.have & PEAK_IMOT);
  TEST_ASSERT_FALSE(p.have & PEAK_IBAT);
  TEST_ASSERT_FALSE(p.have & PEAK_POWER);
  TEST_ASSERT_TRUE(p.have & PEAK_RPM);
  v.i_motor_ema = 42.0f;
  peaks_vesc(p, v, NOW + 20);
  v.i_motor_ema = NAN;  // a NaN later never replaces a real peak
  peaks_vesc(p, v, NOW + 40);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 42.0f, p.i_mot_a);
}

static void test_wrapped_clock_freshness() {
  // Stamped just before the 32-bit millisecond clock wraps, read just after: still fresh.
  const uint32_t stamp = 0xFFFFFF00u;
  const uint32_t now = 0x00000010u;
  PeaksState p = none();
  peaks_vesc(p, vesc_at(stamp), now);
  TEST_ASSERT_EQUAL_HEX8(kVescBits, p.have);
}

// ---------------------------------------------------------------- GNSS
static void test_gnss_speed_peak() {
  PeaksState p = none();
  peaks_gnss(p, gnss_at(NOW, 3000), NOW);
  TEST_ASSERT_TRUE(p.have & PEAK_SPEED);
  TEST_ASSERT_EQUAL_INT32(3000, p.speed_mm_s);
  peaks_gnss(p, gnss_at(NOW + 200, 2000), NOW + 200);
  TEST_ASSERT_EQUAL_INT32(3000, p.speed_mm_s);
  peaks_gnss(p, gnss_at(NOW + 400, 7200), NOW + 400);
  TEST_ASSERT_EQUAL_INT32(7200, p.speed_mm_s);
  TEST_ASSERT_EQUAL_HEX8(PEAK_SPEED, p.have);  // nothing VESC-side touched
}

static void test_gnss_needs_a_usable_fix() {
  PeaksState p = none();
  GnssState g = gnss_at(NOW, 9000);
  g.fix_ok = false;
  peaks_gnss(p, g, NOW);
  TEST_ASSERT_FALSE(p.have & PEAK_SPEED);

  g = gnss_at(NOW, 9000);
  g.sacc_mm_s = (uint32_t)GNSS_MAX_SACC_MM_S + 1u;  // a wild speed with a poor accuracy estimate
  peaks_gnss(p, g, NOW);
  TEST_ASSERT_FALSE(p.have & PEAK_SPEED);

  g = gnss_at(NOW - GNSS_STALE_MS - 1, 9000);  // stale epoch
  peaks_gnss(p, g, NOW);
  TEST_ASSERT_FALSE(p.have & PEAK_SPEED);

  // A usable fix at rest is a value (0), and a negative oddity clamps to 0
  peaks_gnss(p, gnss_at(NOW, -50), NOW);
  TEST_ASSERT_TRUE(p.have & PEAK_SPEED);
  TEST_ASSERT_EQUAL_INT32(0, p.speed_mm_s);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_first_fresh_sample_sets_every_vesc_peak);
  RUN_TEST(test_max_is_kept_and_raised);
  RUN_TEST(test_reverse_counts_by_magnitude);
  RUN_TEST(test_regen_never_counts_as_drawn);
  RUN_TEST(test_stale_status_is_ignored);
  RUN_TEST(test_temperatures_need_plausible_readings);
  RUN_TEST(test_non_finite_values_are_ignored);
  RUN_TEST(test_wrapped_clock_freshness);
  RUN_TEST(test_gnss_speed_peak);
  RUN_TEST(test_gnss_needs_a_usable_fix);
  return UNITY_END();
}
