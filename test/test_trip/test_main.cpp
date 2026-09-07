// Native Unity tests (pio test -e native) for the pure trip / efficiency
// integrator in include/trip.h: distance from GNSS ground speed, energy from
// the VESC watt-hour counters or the v_in x current_in fallback, counter reset
// detection, the EFF_WINDOW_S "now" window, validity rules, the dt clamp and
// millis() wrap-around.
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; includes only Arduino-free headers.
// Efficiency expectations follow the configured unit (SPEED_UNIT_KNOTS / EFF_UNIT_KM,
// overridable with -D): the same 100 m / 5.556 Wh trip is 102.9 Wh/NM or 55.6 Wh/km.
#include <unity.h>

#include <stdint.h>
#include <string.h>

#include "trip.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- fixtures
static const uint32_t STEP = TRIP_PERIOD_MS;  // 200 ms integrator tick
static const uint32_t T0 = 100000;            // boot + 100 s

static uint32_t nz(uint32_t t) { return t ? t : 1u; }  // producers store 1 for a millis() of 0

// Efficiency of `wh` watt-hours over `m` metres in the configured display unit.
static float eff_of(double wh, double m) { return (float)(wh / (m / (double)EFF_DIST_UNIT_M)); }

enum Src { SRC_NONE, SRC_COUNTERS, SRC_R1, SRC_BOTH };

// A scenario: the integrator, the snapshot it reads, the emulated VESC energy counters.
struct Sim {
  TripCalc c;
  SharedState s;
  TripState out;
  uint32_t now;
  double cum_wh, cum_whc;  // what the VESC's own STATUS_3 counters would read (it keeps counting during CAN outages)
  explicit Sim(uint32_t t0 = T0) : now(t0), cum_wh(0.0), cum_whc(0.0) {
    trip_calc_init(c);
    memset(&s, 0, sizeof s);
    memset(&out, 0, sizeof out);
    s.vesc.locked_id = -1;
    s.can.state = CAN_STATE_RUNNING;
    s.gnss.prot_ver_x100 = -1;
  }
};

// GNSS: fresh 3D fix at ground speed speed_mm_s, sAcc 0.3 m/s (NAV-PVT stamped now).
static void gnss_fix(Sim &m, int32_t speed_mm_s) {
  GnssState &g = m.s.gnss;
  g.phase = GNSS_PHASE_RUN;
  g.last_pvt_ms = nz(m.now);
  g.fix_type = 3;
  g.fix_ok = true;
  g.num_sv = 9;
  g.gspeed_mm_s = speed_mm_s;
  g.sacc_mm_s = 300;
}

// STATUS_4 / STATUS_5 (and STATUS_1) fresh: 48 V bus, current_in = power / 48.
static void vesc_r1(Sim &m, double power_w) {
  vesc_telemetry_t &t = m.s.vesc.t;
  t.v_in = 48.0f;
  t.current_in = (float)(power_w / 48.0);
  t.t_ms[VESC_IDX_STATUS_1] = nz(m.now);
  t.t_ms[VESC_IDX_STATUS_4] = nz(m.now);
  t.t_ms[VESC_IDX_STATUS_5] = nz(m.now);
}

// STATUS_3 fresh with the emulated counters.
static void vesc_s3(Sim &m) {
  vesc_telemetry_t &t = m.s.vesc.t;
  t.watt_hours = (float)m.cum_wh;
  t.watt_hours_charged = (float)m.cum_whc;
  t.t_ms[VESC_IDX_STATUS_3] = nz(m.now);
}

static void vesc_never(Sim &m) { memset(m.s.vesc.t.t_ms, 0, sizeof m.s.vesc.t.t_ms); }

static void apply_sources(Sim &m, double power_w, Src src) {
  if (src == SRC_COUNTERS || src == SRC_BOTH) vesc_s3(m);
  if (src == SRC_R1 || src == SRC_BOTH) vesc_r1(m, power_w);
}

// First integrator call at m.now (baselines only).
static void start(Sim &m, int32_t speed_mm_s, Src src, bool gnss = true) {
  if (gnss) gnss_fix(m, speed_mm_s);
  apply_sources(m, 0.0, src);
  trip_calc_update(m.c, m.s, m.now, m.out);
}

// One tick: the VESC ran at power_w for `ms`, then the selected sources are refreshed and the integrator runs.
static void tick(Sim &m, uint32_t ms, int32_t speed_mm_s, double power_w, Src src, bool gnss = true) {
  m.now += ms;
  if (power_w >= 0.0) m.cum_wh += power_w * ms / 3.6e6;
  else m.cum_whc += -power_w * ms / 3.6e6;
  if (gnss) gnss_fix(m, speed_mm_s);
  apply_sources(m, power_w, src);
  trip_calc_update(m.c, m.s, m.now, m.out);
}

static void run(Sim &m, unsigned steps, int32_t speed_mm_s, double power_w, Src src, bool gnss = true) {
  for (unsigned i = 0; i < steps; ++i) tick(m, STEP, speed_mm_s, power_w, src, gnss);
}

// ---------------------------------------------------------------- basics
static void test_first_call_only_initialises() {
  Sim m;
  m.cum_wh = 123.4;  // a VESC that has been running for a while: its counters must not become trip energy
  m.cum_whc = 5.6;
  start(m, 5000, SRC_BOTH);
  TEST_ASSERT_EQUAL_UINT32(T0, m.out.t_ms);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.run_s);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.moving_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.dist_m);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.wh);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.wh_charged);
  TEST_ASSERT_TRUE(m.out.energy_from_counters);
  TEST_ASSERT_EQUAL_UINT8(0, m.out.win_fill_s);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  TEST_ASSERT_FALSE(m.out.eff_avg_valid);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.counter_resets);
}

static void test_constant_speed_counters_20s() {
  Sim m;
  start(m, 5000, SRC_BOTH);
  run(m, 100, 5000, 1000.0, SRC_BOTH);  // 20 s at 5 m/s, 1 kW, counters win over the fallback
  TEST_ASSERT_EQUAL_UINT32(T0 + 20000, m.out.t_ms);
  TEST_ASSERT_EQUAL_UINT32(20, m.out.run_s);
  TEST_ASSERT_EQUAL_UINT32(20, m.out.moving_s);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, m.out.dist_m);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 5.5556f, m.out.wh);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.wh_charged);
  TEST_ASSERT_TRUE(m.out.energy_from_counters);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.counter_resets);
  // window: the last 10 closed one-second buckets
  TEST_ASSERT_EQUAL_UINT8(EFF_WINDOW_S, m.out.win_fill_s);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.out.win_dist_m);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f, m.out.win_wh);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 1000.0f, m.out.win_p_avg_w);
  // efficiency: 5.556 Wh per 100 m in the configured unit, "now" == "avg" at constant conditions
  const float expect = eff_of(5.5556, 100.0);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  TEST_ASSERT_TRUE(m.out.eff_avg_valid);
  TEST_ASSERT_FLOAT_WITHIN(expect * 0.01f, expect, m.out.eff_now);
  TEST_ASSERT_FLOAT_WITHIN(expect * 0.01f, expect, m.out.eff_avg);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, m.out.eff_avg, m.out.eff_now);
}

static void test_efficiency_literal_for_configured_unit() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 100, 5000, 1000.0, SRC_COUNTERS);
#if SPEED_UNIT_KNOTS && !EFF_UNIT_KM
  TEST_ASSERT_EQUAL_STRING("Wh/NM", EFF_UNIT_STR);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 102.9f, m.out.eff_avg);  // 5.556 Wh / (100 m / 1852 m)
#else
  TEST_ASSERT_EQUAL_STRING("Wh/km", EFF_UNIT_STR);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 55.56f, m.out.eff_avg);  // 5.556 Wh / 0.1 km
#endif
}

static void test_standing_still_energy_only() {
  Sim m;
  start(m, 0, SRC_COUNTERS);
  run(m, 100, 0, 200.0, SRC_COUNTERS);  // 20 s moored, 200 W (lights, pumps)
  TEST_ASSERT_EQUAL_UINT32(20, m.out.run_s);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.moving_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.dist_m);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.1111f, m.out.wh);
  TEST_ASSERT_EQUAL_UINT8(EFF_WINDOW_S, m.out.win_fill_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.win_dist_m);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.5556f, m.out.win_wh);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, m.out.win_p_avg_w);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  TEST_ASSERT_FALSE(m.out.eff_avg_valid);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.eff_now);  // no division by zero
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.eff_avg);
}

static void test_min_speed_threshold() {
  Sim m;
  start(m, 0, SRC_COUNTERS);
  run(m, 25, EFF_MIN_SPEED_MM_S - 1, 100.0, SRC_COUNTERS);  // GNSS drift just below the threshold
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.dist_m);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.moving_s);
  run(m, 25, EFF_MIN_SPEED_MM_S, 100.0, SRC_COUNTERS);  // exactly at the threshold counts
  TEST_ASSERT_FLOAT_WITHIN(0.001f, (float)EFF_MIN_SPEED_MM_S * 5.0f / 1000.0f, m.out.dist_m);
  TEST_ASSERT_EQUAL_UINT32(5, m.out.moving_s);
  TEST_ASSERT_EQUAL_UINT32(10, m.out.run_s);
}

// ---------------------------------------------------------------- GNSS gating
static void test_gnss_unusable_no_distance() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  // stale NAV-PVT (one ms past the limit at the time of the tick): speed 5 m/s is ignored
  for (int i = 0; i < 5; ++i) {
    m.s.gnss.last_pvt_ms = (m.now + STEP) - GNSS_STALE_MS - 1;
    tick(m, STEP, 5000, 1000.0, SRC_COUNTERS, false);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.dist_m);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.moving_s);
  // exactly at the limit is fresh
  m.s.gnss.last_pvt_ms = (m.now + STEP) - GNSS_STALE_MS;
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS, false);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, m.out.dist_m);
  // fresh but no fix
  gnss_fix(m, 5000);
  m.s.gnss.fix_ok = false;
  run(m, 5, 5000, 1000.0, SRC_COUNTERS, false);  // 1 s: the stamp stays within GNSS_STALE_MS
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, m.out.dist_m);
  // speed accuracy worse than GNSS_MAX_SACC_MM_S
  m.s.gnss.fix_ok = true;
  m.s.gnss.sacc_mm_s = GNSS_MAX_SACC_MM_S + 1;
  m.s.gnss.last_pvt_ms = m.now;
  run(m, 5, 5000, 1000.0, SRC_COUNTERS, false);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, m.out.dist_m);
  // exactly GNSS_MAX_SACC_MM_S is usable
  m.s.gnss.sacc_mm_s = GNSS_MAX_SACC_MM_S;
  m.s.gnss.last_pvt_ms = m.now;
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS, false);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, m.out.dist_m);
  // energy kept flowing the whole time (counters do not depend on the GNSS)
  TEST_ASSERT_TRUE(m.out.wh > 0.5f);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
}

static void test_gnss_loss_invalidates_now_immediately() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 50, 5000, 1000.0, SRC_COUNTERS);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS, false);  // no new NAV-PVT ...
  TEST_ASSERT_TRUE(m.out.eff_now_valid);                // ... still within GNSS_STALE_MS
  m.s.gnss.last_pvt_ms = m.now - GNSS_STALE_MS - 1;   // ... now stale
  trip_calc_update(m.c, m.s, m.now, m.out);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  TEST_ASSERT_TRUE(m.out.win_dist_m > 40.0f);  // the window still holds the distance; only the validity dropped
  TEST_ASSERT_TRUE(m.out.eff_avg_valid);       // the trip average does not depend on the live fix
}

// ---------------------------------------------------------------- energy: counters
static void test_counter_reset_detected() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 50, 5000, 1000.0, SRC_COUNTERS);  // 10 s -> 2.778 Wh
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f, m.out.wh);
  // VESC reboot: its counters restart at (almost) zero
  m.cum_wh = 0.01;
  m.cum_whc = 0.0;
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS);  // cum_wh -> 0.0656
  TEST_ASSERT_EQUAL_UINT32(1, m.out.counter_resets);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f, m.out.wh);  // nothing added on the reset tick
  run(m, 50, 5000, 1000.0, SRC_COUNTERS);               // counting resumes from the new baseline
  TEST_ASSERT_EQUAL_UINT32(1, m.out.counter_resets);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.7778f + 2.7778f, m.out.wh);
  TEST_ASSERT_TRUE(m.out.eff_avg_valid);
  // a reset visible only in the charged counter counts too
  m.cum_whc = 3.0;
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 3.0f, m.out.wh_charged);
  m.cum_whc = 0.0;
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS);
  TEST_ASSERT_EQUAL_UINT32(2, m.out.counter_resets);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 3.0f, m.out.wh_charged);
}

static void test_small_negative_counter_delta_is_noise() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 10, 5000, 1000.0, SRC_COUNTERS);
  const float before = m.out.wh;
  m.cum_wh -= 0.4;  // above -TRIP_COUNTER_RESET_WH: not a reboot, nothing added
  tick(m, STEP, 5000, 0.0, SRC_COUNTERS);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.counter_resets);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, before, m.out.wh);
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS);  // re-baselined: the next delta counts normally
  TEST_ASSERT_FLOAT_WITHIN(0.002f, before + 0.0556f, m.out.wh);
}

static void test_counters_regen_goes_to_wh_charged() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 50, 5000, -500.0, SRC_COUNTERS);  // 10 s of net regen at 5 m/s (sailing, prop turning the motor)
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.wh);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.3889f, m.out.wh_charged);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  TEST_ASSERT_TRUE(m.out.eff_avg_valid);
  const float expect = eff_of(-1.3889, 50.0);
  TEST_ASSERT_TRUE(m.out.eff_now < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(-expect * 0.01f, expect, m.out.eff_now);
  TEST_ASSERT_FLOAT_WITHIN(-expect * 0.01f, expect, m.out.eff_avg);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, -500.0f, m.out.win_p_avg_w);
}

static void test_status3_exactly_at_stale_limit_is_fresh() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  m.now += STEP;
  m.cum_wh += 1000.0 * STEP / 3.6e6;
  gnss_fix(m, 5000);
  vesc_s3(m);
  m.s.vesc.t.t_ms[VESC_IDX_STATUS_3] = m.now - VESC_STALE_R2_MS;  // last frame exactly VESC_STALE_R2_MS ago
  vesc_r1(m, 960.0);
  trip_calc_update(m.c, m.s, m.now, m.out);
  TEST_ASSERT_TRUE(m.out.energy_from_counters);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0556f, m.out.wh);
  m.now += STEP;
  m.s.vesc.t.t_ms[VESC_IDX_STATUS_3] = m.now - VESC_STALE_R2_MS - 1;  // one ms older: stale -> fallback
  gnss_fix(m, 5000);
  vesc_r1(m, 960.0);
  trip_calc_update(m.c, m.s, m.now, m.out);
  TEST_ASSERT_FALSE(m.out.energy_from_counters);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0556f + 0.0533f, m.out.wh);
}

// ---------------------------------------------------------------- energy: fallback
static void test_fallback_integration_960w() {
  Sim m;
  start(m, 5000, SRC_R1);                // STATUS_3 never received
  run(m, 50, 5000, 960.0, SRC_R1);       // 48 V x 20 A for 10 s
  TEST_ASSERT_FALSE(m.out.energy_from_counters);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.6667f, m.out.wh);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.wh_charged);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.out.dist_m);
  TEST_ASSERT_EQUAL_UINT8(EFF_WINDOW_S, m.out.win_fill_s);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 960.0f, m.out.win_p_avg_w);
  const float expect = eff_of(2.6667, 50.0);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  TEST_ASSERT_FLOAT_WITHIN(expect * 0.01f, expect, m.out.eff_now);
  TEST_ASSERT_FLOAT_WITHIN(expect * 0.01f, expect, m.out.eff_avg);
}

static void test_fallback_regen_negative_current() {
  Sim m;
  start(m, 5000, SRC_R1);
  run(m, 50, 5000, -480.0, SRC_R1);  // -10 A at 48 V
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.wh);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.3333f, m.out.wh_charged);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  TEST_ASSERT_TRUE(m.out.eff_now < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, eff_of(-1.3333, 50.0), m.out.eff_now);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, -480.0f, m.out.win_p_avg_w);
}

static void test_fallback_then_counters_rebaseline_no_double_count() {
  Sim m;
  start(m, 5000, SRC_R1);
  run(m, 50, 5000, 960.0, SRC_R1);  // 2.667 Wh integrated; the VESC's counters read the same 2.667 (+ the next tick)
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.6667f, m.out.wh);
  // STATUS_3 appears: the counters' history (2.72 Wh) is NOT added, they only get their
  // baseline; this one step is still integrated from v_in x current_in (no hole in the window).
  tick(m, STEP, 5000, 960.0, SRC_BOTH);
  TEST_ASSERT_TRUE(m.out.energy_from_counters);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.6667f + 0.0533f, m.out.wh);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.counter_resets);
  tick(m, STEP, 5000, 960.0, SRC_BOTH);  // from here the counter deltas count
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.6667f + 2 * 0.05333f, m.out.wh);
  run(m, 48, 5000, 960.0, SRC_BOTH);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.6667f + 50 * 0.05333f, m.out.wh);  // == 20 s x 960 W, nothing lost or doubled
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  const float expect = eff_of(2.6667, 50.0);
  TEST_ASSERT_FLOAT_WITHIN(expect * 0.01f, expect, m.out.eff_now);
}

static void test_outage_then_counters_catch_up_totals_not_window() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 50, 5000, 1000.0, SRC_COUNTERS);  // 10 s: 2.778 Wh
  // CAN outage while the VESC keeps running: no STATUS at all for 10 s
  run(m, 25, 5000, 1000.0, SRC_NONE);      // first 5 s: STATUS_3 not yet stale, counters unchanged -> nothing added
  TEST_ASSERT_TRUE(m.out.energy_from_counters);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f, m.out.wh);
  run(m, 25, 5000, 1000.0, SRC_NONE);      // next 5 s: everything stale, no energy source
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f, m.out.wh);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  TEST_ASSERT_TRUE(m.out.energy_from_counters);  // label kept while no source is active
  TEST_ASSERT_TRUE(m.out.eff_avg_valid);         // distance kept accumulating: 100 m
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, m.out.dist_m);
  // STATUS_3 returns with the counters at 5.556 Wh: the outage energy is recovered into the trip ...
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.counter_resets);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.6111f, m.out.wh);
  const float expect_avg = eff_of(5.6111, 100.2);
  TEST_ASSERT_FLOAT_WITHIN(expect_avg * 0.02f, expect_avg, m.out.eff_avg);
  // ... but not into the window, which still shows gaps
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.out.win_wh);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  // the outage buckets roll out after EFF_WINDOW_S seconds, but the bucket that took the
  // catch-up tick (its own 200 ms of energy is inside the lump) is a gap for one more second
  run(m, EFF_WINDOW_S * 5, 5000, 1000.0, SRC_COUNTERS);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.out.win_dist_m);
  run(m, 5, 5000, 1000.0, SRC_COUNTERS);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  const float expect_now = eff_of(2.7778, 50.0);
  TEST_ASSERT_FLOAT_WITHIN(expect_now * 0.01f, expect_now, m.out.eff_now);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f, m.out.win_wh);
}

static void test_no_energy_source_eff_now_invalid() {
  Sim m;
  start(m, 5000, SRC_NONE);  // sailing with the VESC switched off
  run(m, 50, 5000, 0.0, SRC_NONE);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.out.dist_m);
  TEST_ASSERT_EQUAL_UINT32(10, m.out.moving_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.wh);
  TEST_ASSERT_FALSE(m.out.energy_from_counters);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  TEST_ASSERT_TRUE(m.out.eff_avg_valid);  // per the contract: distance >= EFF_MIN_DIST_M (0 Wh -> 0)
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.eff_avg);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.win_p_avg_w);
}

// ---------------------------------------------------------------- window
static void test_window_rolls_off() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 50, 5000, 1000.0, SRC_COUNTERS);  // 10 s under way
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.out.win_dist_m);
  run(m, 25, 0, 100.0, SRC_COUNTERS);      // 5 s moored: half the window is still the moving half
  TEST_ASSERT_EQUAL_UINT8(EFF_WINDOW_S, m.out.win_fill_s);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, m.out.win_dist_m);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  run(m, 25, 0, 100.0, SRC_COUNTERS);      // 10 s moored: the moving buckets are gone
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.win_dist_m);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.2778f, m.out.win_wh);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, m.out.win_p_avg_w);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  TEST_ASSERT_TRUE(m.out.eff_avg_valid);
  const float expect_avg = eff_of(2.7778 + 0.2778, 50.0);
  TEST_ASSERT_FLOAT_WITHIN(expect_avg * 0.01f, expect_avg, m.out.eff_avg);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.out.dist_m);
}

static void test_window_min_fill_and_min_dist() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 4, 5000, 1000.0, SRC_COUNTERS);  // 0.8 s: no closed bucket yet
  TEST_ASSERT_EQUAL_UINT8(0, m.out.win_fill_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.win_dist_m);  // the open bucket is not part of the sums
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  run(m, 1, 5000, 1000.0, SRC_COUNTERS);  // 1.0 s: first bucket closed
  TEST_ASSERT_EQUAL_UINT8(1, m.out.win_fill_s);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, m.out.win_dist_m);
  run(m, 5, 5000, 1000.0, SRC_COUNTERS);  // 2 s: distance is enough (10 m) but fill < TRIP_EFF_MIN_FILL_S
  TEST_ASSERT_EQUAL_UINT8(2, m.out.win_fill_s);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  run(m, 5, 5000, 1000.0, SRC_COUNTERS);  // 3 s: valid
  TEST_ASSERT_EQUAL_UINT8(3, m.out.win_fill_s);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);

  // slow boat: 2.5 m/s -> 0.5 m per tick, EFF_MIN_DIST_M (10 m) reached exactly after 20 ticks
  Sim slow;
  start(slow, 2500, SRC_COUNTERS);
  run(slow, 19, 2500, 500.0, SRC_COUNTERS);
  TEST_ASSERT_EQUAL_UINT8(3, slow.out.win_fill_s);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.5f, slow.out.win_dist_m);  // 3 closed buckets x 2.5 m
  TEST_ASSERT_FALSE(slow.out.eff_now_valid);
  TEST_ASSERT_FALSE(slow.out.eff_avg_valid);  // trip 9.5 m
  run(slow, 1, 2500, 500.0, SRC_COUNTERS);    // trip 10.0 m, window 4 buckets = 10.0 m
  TEST_ASSERT_TRUE(slow.out.eff_avg_valid);
  TEST_ASSERT_TRUE(slow.out.eff_now_valid);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, slow.out.dist_m);
}

static void test_five_second_step_closes_five_buckets() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  tick(m, 5000, 5000, 1000.0, SRC_COUNTERS);  // one 5 s step (not clamped: exactly TRIP_DT_MAX_MS)
  TEST_ASSERT_EQUAL_UINT32(5, m.out.run_s);
  TEST_ASSERT_EQUAL_UINT32(5, m.out.moving_s);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, m.out.dist_m);
  TEST_ASSERT_EQUAL_UINT8(5, m.out.win_fill_s);  // one bucket with the data, four empty ones
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, m.out.win_dist_m);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.3889f, m.out.win_wh);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 1000.0f, m.out.win_p_avg_w);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  const float expect = eff_of(1.3889, 25.0);
  TEST_ASSERT_FLOAT_WITHIN(expect * 0.01f, expect, m.out.eff_now);
}

static void test_dt_clamp_and_stall_gap() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 25, 5000, 1000.0, SRC_COUNTERS);  // 5 s normal
  tick(m, 60000, 5000, 1000.0, SRC_COUNTERS);  // the task stalled for a minute
  TEST_ASSERT_EQUAL_UINT32(10, m.out.run_s);       // 5 s counted, 55 s dropped
  TEST_ASSERT_EQUAL_UINT32(10, m.out.moving_s);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.out.dist_m);       // not 325 m
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.3889f + 16.6667f, m.out.wh);  // counters are exact regardless of dt
  TEST_ASSERT_EQUAL_UINT8(EFF_WINDOW_S, m.out.win_fill_s);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);  // the stall buckets are gaps
  TEST_ASSERT_TRUE(m.out.eff_avg_valid);
  run(m, EFF_WINDOW_S * 5, 5000, 1000.0, SRC_COUNTERS);  // good data pushes the gaps out
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.out.win_dist_m);

  // fallback source: the clamp bounds the integrated energy as well
  Sim f;
  start(f, 5000, SRC_R1);
  tick(f, 60000, 5000, 960.0, SRC_R1);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 960.0f * 5.0f / 3600.0f, f.out.wh);  // 5 s, not 60 s
  TEST_ASSERT_FALSE(f.out.eff_now_valid);
}

// ---------------------------------------------------------------- clock
static void test_wraparound_now_ms() {
  Sim m(0xFFFFFFFFu - 1099u);  // millis() wraps after 5.5 ticks; no tick lands exactly on 0
  start(m, 5000, SRC_COUNTERS);
  run(m, 10, 5000, 1000.0, SRC_COUNTERS);
  TEST_ASSERT_EQUAL_UINT32(900, m.out.t_ms);
  TEST_ASSERT_EQUAL_UINT32(2, m.out.run_s);
  TEST_ASSERT_EQUAL_UINT32(2, m.out.moving_s);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, m.out.dist_m);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.5556f, m.out.wh);
  TEST_ASSERT_EQUAL_UINT8(2, m.out.win_fill_s);
  TEST_ASSERT_TRUE(m.out.energy_from_counters);
}

static void test_now_zero_stored_as_one() {
  Sim m(0);
  start(m, 0, SRC_NONE, false);
  TEST_ASSERT_EQUAL_UINT32(1, m.out.t_ms);
  m.now = 5;  // a later tick at a non-zero time keeps its own stamp
  trip_calc_update(m.c, m.s, m.now, m.out);
  TEST_ASSERT_EQUAL_UINT32(5, m.out.t_ms);
}

static void test_init_resets_everything() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 100, 5000, 1000.0, SRC_COUNTERS);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  trip_calc_init(m.c);
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS);  // first call after init: baseline only
  TEST_ASSERT_EQUAL_UINT32(0, m.out.run_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.dist_m);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.out.wh);
  TEST_ASSERT_EQUAL_UINT8(0, m.out.win_fill_s);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.counter_resets);
  TEST_ASSERT_FALSE(m.out.eff_now_valid);
  TEST_ASSERT_FALSE(m.out.eff_avg_valid);
  TEST_ASSERT_EQUAL_UINT32(m.now, m.out.t_ms);
}

static void test_no_vesc_ever_then_fallback_only() {
  Sim m;
  vesc_never(m);
  start(m, 5000, SRC_NONE);
  run(m, 10, 5000, 0.0, SRC_NONE);
  TEST_ASSERT_FALSE(m.out.energy_from_counters);
  run(m, 50, 5000, 960.0, SRC_R1);  // STATUS_4/5 start arriving (STATUS_3 disabled in the VESC)
  TEST_ASSERT_FALSE(m.out.energy_from_counters);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.6667f, m.out.wh);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);  // the initial no-source buckets have rolled out
}

static void test_counters_return_after_fallback_overwrite_estimate() {
  Sim m;
  start(m, 5000, SRC_BOTH);
  run(m, 50, 5000, 1000.0, SRC_BOTH);  // 10 s on the counters: 2.778 Wh
  // STATUS_3 lost for 10 s while STATUS_4/5 keep coming: the fallback sees a biased 1200 W
  // (v_in x current_in sampled at 5 Hz) while the VESC's own counters accumulate the true 1000 W.
  // The last STATUS_3 sample counts as fresh for VESC_STALE_R2_MS (5 s): during that time the
  // counters are re-read unchanged (delta 0, nothing added), then the fallback takes over.
  for (int i = 0; i < 50; ++i) {
    m.now += STEP;
    m.cum_wh += 1000.0 * STEP / 3.6e6;
    gnss_fix(m, 5000);
    vesc_r1(m, 1200.0);
    trip_calc_update(m.c, m.s, m.now, m.out);
    if (i == 24) {
      TEST_ASSERT_TRUE(m.out.energy_from_counters);
      TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f, m.out.wh);  // stale-but-fresh sample: nothing added yet
    }
  }
  TEST_ASSERT_FALSE(m.out.energy_from_counters);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f + 1.6667f, m.out.wh);  // 5 s of the 1200 W estimate
  TEST_ASSERT_TRUE(m.out.eff_now_valid);                           // real data, no gap (the window reads low for 5 s)
  // STATUS_3 returns: the counters overwrite the fallback's estimate (nothing doubled, nothing lost)
  tick(m, STEP, 5000, 1000.0, SRC_BOTH);
  TEST_ASSERT_TRUE(m.out.energy_from_counters);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.counter_resets);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 5.6111f, m.out.wh);  // == 20.2 s x 1000 W
  TEST_ASSERT_TRUE(m.out.eff_now_valid);                // the transition tick is not a gap
  run(m, 50, 5000, 1000.0, SRC_BOTH);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.3889f, m.out.wh);  // 30.2 s x 1000 W: steady state is the plain delta again
  const float expect = eff_of(2.7778, 50.0);
  TEST_ASSERT_FLOAT_WITHIN(expect * 0.01f, expect, m.out.eff_now);
}

static void test_outage_split_return_recovers_energy() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 50, 5000, 1000.0, SRC_COUNTERS);  // 10 s: 2.778 Wh
  run(m, 50, 5000, 1000.0, SRC_NONE);      // 10 s CAN outage (the VESC keeps running)
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f, m.out.wh);
  // CAN is back: STATUS_4/5 (50 Hz) land in this tick, STATUS_3 (5 Hz) only in the next one
  tick(m, STEP, 5000, 1000.0, SRC_R1);
  TEST_ASSERT_FALSE(m.out.energy_from_counters);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f + 0.0556f, m.out.wh);  // one fallback step
  tick(m, STEP, 5000, 1000.0, SRC_BOTH);
  TEST_ASSERT_TRUE(m.out.energy_from_counters);
  TEST_ASSERT_EQUAL_UINT32(0, m.out.counter_resets);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 5.6667f, m.out.wh);  // 20.4 s x 1000 W: the outage energy is recovered
  TEST_ASSERT_FALSE(m.out.eff_now_valid);               // ... but the window has the outage gaps
  run(m, EFF_WINDOW_S * 5 + 5, 5000, 1000.0, SRC_BOTH);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.7778f, m.out.win_wh);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.6667f + 2.7778f + 0.2778f, m.out.wh);
}

static void test_counter_reset_tick_is_a_gap() {
  Sim m;
  start(m, 5000, SRC_COUNTERS);
  run(m, 50, 5000, 1000.0, SRC_COUNTERS);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  m.cum_wh = 0.0;  // VESC rebooted
  m.cum_whc = 0.0;
  tick(m, STEP, 5000, 1000.0, SRC_COUNTERS);  // tick 51: first tick of a new (open) bucket
  TEST_ASSERT_EQUAL_UINT32(1, m.out.counter_resets);
  TEST_ASSERT_TRUE(m.out.eff_now_valid);   // only closed buckets count: still the 10 good ones
  run(m, 4, 5000, 1000.0, SRC_COUNTERS);    // tick 55 closes the bucket missing the reset step's energy ...
  TEST_ASSERT_FALSE(m.out.eff_now_valid);  // ... as a gap
  run(m, EFF_WINDOW_S * 5, 5000, 1000.0, SRC_COUNTERS);  // tick 105: the gap bucket is evicted
  TEST_ASSERT_TRUE(m.out.eff_now_valid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.7778f + 54.0f * 0.05556f, m.out.wh);  // only the reset step itself is lost
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_first_call_only_initialises);
  RUN_TEST(test_constant_speed_counters_20s);
  RUN_TEST(test_efficiency_literal_for_configured_unit);
  RUN_TEST(test_standing_still_energy_only);
  RUN_TEST(test_min_speed_threshold);
  RUN_TEST(test_gnss_unusable_no_distance);
  RUN_TEST(test_gnss_loss_invalidates_now_immediately);
  RUN_TEST(test_counter_reset_detected);
  RUN_TEST(test_small_negative_counter_delta_is_noise);
  RUN_TEST(test_counters_regen_goes_to_wh_charged);
  RUN_TEST(test_status3_exactly_at_stale_limit_is_fresh);
  RUN_TEST(test_fallback_integration_960w);
  RUN_TEST(test_fallback_regen_negative_current);
  RUN_TEST(test_fallback_then_counters_rebaseline_no_double_count);
  RUN_TEST(test_outage_then_counters_catch_up_totals_not_window);
  RUN_TEST(test_no_energy_source_eff_now_invalid);
  RUN_TEST(test_window_rolls_off);
  RUN_TEST(test_window_min_fill_and_min_dist);
  RUN_TEST(test_five_second_step_closes_five_buckets);
  RUN_TEST(test_dt_clamp_and_stall_gap);
  RUN_TEST(test_wraparound_now_ms);
  RUN_TEST(test_now_zero_stored_as_one);
  RUN_TEST(test_init_resets_everything);
  RUN_TEST(test_no_vesc_ever_then_fallback_only);
  RUN_TEST(test_counters_return_after_fallback_overwrite_estimate);
  RUN_TEST(test_outage_split_return_recovers_energy);
  RUN_TEST(test_counter_reset_tick_is_a_gap);
  return UNITY_END();
}
