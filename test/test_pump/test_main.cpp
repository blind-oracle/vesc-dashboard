// Unity tests (pio test -e native) for the coolant pump's pure controller in
// include/pump_logic.h: targets from the VESC's limits, the OFF / RUN / FULL / BLIND modes,
// start / stop with hysteresis and after-run, the PI loop with its anti-windup and bumpless
// transfers, the inputs taken from the VESC telemetry and the LEDC duty conversion.
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; the config.h defaults apply (Kp 8,
// Ki 0.1, Kd 0, min 20 %, blind 100 %, margin 10 C, band 15 C, hysteresis 5 C, after-run 60 s).
#include <unity.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "pump_logic.h"

void setUp() {}
void tearDown() {}

static const float DT = PUMP_PERIOD_MS / 1000.0f;

// Defaults 85 / 100 / accel_dec 0.15: the VESC derates acceleration from 76 C, the targets are 66 C.
static PumpTargets def_targets() { return pump_targets(pump_default_limits()); }

static PumpInputs temps(float mot, float fet, bool turning = false) {
  PumpInputs in;
  memset(&in, 0, sizeof in);
  in.fresh = true;
  in.t_mot = mot;
  in.t_fet = fet;
  in.mot_ok = vesc_temp_plausible(mot);
  in.fet_ok = vesc_temp_plausible(fet);
  in.turning = turning;
  return in;
}

static PumpInputs stale() {
  PumpInputs in = temps(30.0f, 30.0f);
  in.fresh = false;
  return in;
}

// Runs `ticks` periods from *now with the same inputs; returns the last duty.
static float run(PumpCtl &c, const PumpInputs &in, const PumpTargets &tg, uint32_t *now, int ticks) {
  float d = 0.0f;
  for (int i = 0; i < ticks; ++i) {
    *now += PUMP_PERIOD_MS;
    d = pump_step(c, in, tg, *now, DT);
  }
  return d;
}

// ---------------------------------------------------------------- targets
static void test_targets_from_default_limits() {
  const PumpTargets tg = def_targets();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 76.0f, tg.derate_mot);  // 85 - 0.15 * (85 - 25)
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 76.0f, tg.derate_fet);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 66.0f, tg.tgt_mot);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 66.0f, tg.tgt_fet);
}

static void test_targets_follow_the_vesc_limits() {
  VescTempLimits l = pump_default_limits();
  l.mot_start = 100.0f;
  l.fet_start = 80.0f;
  l.accel_dec = 0.5f;
  const PumpTargets tg = pump_targets(l);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 62.5f, tg.derate_mot);  // 100 - 0.5 * 75
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 52.5f, tg.derate_fet);  // 80 - 0.5 * 55
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 52.5f, tg.tgt_mot);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 42.5f, tg.tgt_fet);
  l.accel_dec = 0.0f;  // no early acceleration derating: the start itself
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, pump_targets(l).derate_mot);
}

static void test_limits_source() {
  VescLimitsState v;
  memset(&v, 0, sizeof v);
  bool from_vesc = true;
  VescTempLimits l = pump_limits(v, &from_vesc);  // never answered: the defaults
  TEST_ASSERT_FALSE(from_vesc);
  TEST_ASSERT_EQUAL_FLOAT(PUMP_DEFAULT_TEMP_MOTOR_START, l.mot_start);
  v.result = MCCONF_OK;
  v.lim = pump_default_limits();
  v.lim.mot_start = 95.0f;
  l = pump_limits(v, &from_vesc);
  TEST_ASSERT_TRUE(from_vesc);
  TEST_ASSERT_EQUAL_FLOAT(95.0f, l.mot_start);
  v.result = MCCONF_UNKNOWN_SIG;  // a later read of a firmware we do not know: back to the defaults
  l = pump_limits(v, &from_vesc);
  TEST_ASSERT_FALSE(from_vesc);
  TEST_ASSERT_EQUAL_FLOAT(PUMP_DEFAULT_TEMP_MOTOR_START, l.mot_start);
  TEST_ASSERT_EQUAL_FLOAT(PUMP_DEFAULT_TEMP_MOTOR_START, pump_limits(v, nullptr).mot_start);
}

// ---------------------------------------------------------------- no data / blind
static void test_off_until_the_vesc_was_seen() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, run(c, stale(), def_targets(), &now, 20));
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_OFF, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_NO_DATA, c.why);
}

static void test_blind_once_seen_data_goes_stale() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  run(c, temps(30.0f, 30.0f), def_targets(), &now, 2);  // cool and still: off, but seen
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_OFF, c.mode);
  TEST_ASSERT_EQUAL_FLOAT((float)PUMP_BLIND_DUTY_PCT, run(c, stale(), def_targets(), &now, 1));
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_BLIND, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_STALE, c.why);
  // and it stays blind however long the link is gone
  TEST_ASSERT_EQUAL_FLOAT((float)PUMP_BLIND_DUTY_PCT, run(c, stale(), def_targets(), &now, 2000));
}

static void test_blind_without_a_plausible_sensor() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  TEST_ASSERT_EQUAL_FLOAT((float)PUMP_BLIND_DUTY_PCT, run(c, temps(-273.0f, 300.0f), def_targets(), &now, 1));
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_BLIND, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_NO_SENSOR, c.why);
}

static void test_missing_motor_sensor_controls_on_the_fet() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  const float d = run(c, temps(-273.0f, 70.0f), def_targets(), &now, 1);  // FET 4 C over its target
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_WARM, c.why);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, c.err);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, PUMP_KP * 4.0f, d);
}

// ---------------------------------------------------------------- start / stop
static void test_turning_runs_at_minimum_while_cool() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  TEST_ASSERT_EQUAL_FLOAT((float)PUMP_MIN_DUTY_PCT, run(c, temps(25.0f, 28.0f, true), def_targets(), &now, 10));
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_TURNING, c.why);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.integ);  // a cool error only decays the integral, never below 0
}

static void test_after_run_then_off() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  run(c, temps(25.0f, 28.0f, true), def_targets(), &now, 4);
  const uint32_t stopped = now;
  // Motor stopped, everything cool: keeps going for PUMP_MIN_RUN_MS ...
  while ((uint32_t)(now + PUMP_PERIOD_MS - stopped) < (uint32_t)PUMP_MIN_RUN_MS) {
    run(c, temps(25.0f, 28.0f), def_targets(), &now, 1);
    TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
    TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_AFTER_RUN, c.why);
    TEST_ASSERT_EQUAL_FLOAT((float)PUMP_MIN_DUTY_PCT, c.duty);
  }
  // ... then stops.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, run(c, temps(25.0f, 28.0f), def_targets(), &now, 1));
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_OFF, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_COOL, c.why);
}

static void test_warm_start_and_hysteresis() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  const PumpTargets tg = def_targets();  // 66 C: starts above 51 C, may stop below 46 C
  run(c, temps(50.0f, 30.0f), tg, &now, 5);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_OFF, c.mode);  // 16 C below target: not yet
  run(c, temps(51.5f, 30.0f), tg, &now, 1);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_WARM, c.why);
  // Inside the hysteresis band (46..51 C) it keeps running long after the after-run is over.
  run(c, temps(47.0f, 30.0f), tg, &now, 2 * PUMP_MIN_RUN_MS / PUMP_PERIOD_MS);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_AFTER_RUN, c.why);
  // Below it, with the after-run counted from the last WARM tick (long ago): off at once.
  run(c, temps(45.0f, 30.0f), tg, &now, 1);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_OFF, c.mode);
  // and back in the band it stays off
  run(c, temps(49.0f, 30.0f), tg, &now, 20);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_OFF, c.mode);
}

static void test_turning_restarts_the_after_run() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  run(c, temps(25.0f, 28.0f, true), def_targets(), &now, 1);
  run(c, temps(25.0f, 28.0f), def_targets(), &now, (PUMP_MIN_RUN_MS / PUMP_PERIOD_MS) - 10);
  run(c, temps(25.0f, 28.0f, true), def_targets(), &now, 1);  // a short burst of throttle
  run(c, temps(25.0f, 28.0f), def_targets(), &now, 20);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);  // the after-run counts from the burst again
}

// ---------------------------------------------------------------- the PI loop
static void test_pi_ramps_above_target() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  const PumpTargets tg = def_targets();
  const float first = run(c, temps(70.0f, 40.0f, true), tg, &now, 1);  // 4 C over: P = 32 %
  TEST_ASSERT_FLOAT_WITHIN(0.5f, PUMP_KP * 4.0f, first);
  const float later = run(c, temps(70.0f, 40.0f, true), tg, &now, (int)(100.0f / DT));  // 100 s on
  // I = Ki * 4 C * 100 s = 40 %
  TEST_ASSERT_FLOAT_WITHIN(0.5f, PUMP_KP * 4.0f + PUMP_KI * 4.0f * 100.0f, later);
  TEST_ASSERT_TRUE(later > first);
  // Back exactly on target: P is 0, the integral holds the flow that got it there.
  const float on_target = run(c, temps(66.0f, 40.0f, true), tg, &now, 1);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, c.integ, on_target);
  TEST_ASSERT_TRUE(on_target > (float)PUMP_MIN_DUTY_PCT);
}

static void test_anti_windup_at_full_duty() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  const PumpTargets tg = def_targets();
  // 9 C over target (75 C, just below the 76 C derating start): P = 72 %, the integral
  // fills only until the output reaches 100 % and then stops however long it stays there.
  TEST_ASSERT_EQUAL_FLOAT(100.0f, run(c, temps(75.0f, 40.0f, true), tg, &now, (int)(1000.0f / DT)));
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
  TEST_ASSERT_TRUE(c.integ <= 100.0f - PUMP_KP * 9.0f + PUMP_KI * 9.0f * DT + 0.01f);
  // so the duty follows the temperature down at once instead of unwinding for minutes
  const float d = run(c, temps(66.0f, 40.0f, true), tg, &now, 1);
  TEST_ASSERT_TRUE(d < 40.0f);
}

static void test_full_at_derating_start_then_bumpless() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  const PumpTargets tg = def_targets();
  TEST_ASSERT_EQUAL_FLOAT(100.0f, run(c, temps(40.0f, 76.0f, true), tg, &now, 1));  // FET at its derating start
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_FULL, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_DERATE, c.why);
  // Back below: RUN continues from 100 % rather than dropping to P alone ...
  const float d1 = run(c, temps(40.0f, 70.0f, true), tg, &now, 1);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
  TEST_ASSERT_TRUE(d1 >= 99.0f);
  // ... and eases down once it is under target.
  const float d2 = run(c, temps(40.0f, 60.0f, true), tg, &now, (int)(60.0f / DT));
  TEST_ASSERT_TRUE(d2 < d1);
}

static void test_blind_to_run_is_bumpless() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  const PumpTargets tg = def_targets();
  run(c, temps(60.0f, 40.0f, true), tg, &now, 1);
  run(c, stale(), tg, &now, 10);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_BLIND, c.mode);
  // Link back, 6 C under target: from 100 %, not a jump to the minimum.
  const float d = run(c, temps(60.0f, 40.0f, true), tg, &now, 1);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
  TEST_ASSERT_TRUE(d > 50.0f);
  // the integral then decays and the duty settles at the minimum
  TEST_ASSERT_EQUAL_FLOAT((float)PUMP_MIN_DUTY_PCT, run(c, temps(60.0f, 40.0f, true), tg, &now, (int)(600.0f / DT)));
}

static void test_dt_is_clamped() {
  PumpCtl a, b;
  pump_init(a);
  pump_init(b);
  const PumpTargets tg = def_targets();
  pump_step(a, temps(70.0f, 40.0f, true), tg, 1000, 0.0f);
  pump_step(b, temps(70.0f, 40.0f, true), tg, 1000, 0.0f);
  pump_step(a, temps(70.0f, 40.0f, true), tg, 2000, 3600.0f);  // the task stalled for an hour
  pump_step(b, temps(70.0f, 40.0f, true), tg, 2000, 5.0f);
  TEST_ASSERT_EQUAL_FLOAT(b.integ, a.integ);
  pump_step(a, temps(70.0f, 40.0f, true), tg, 3000, -1.0f);  // never integrates backwards
  TEST_ASSERT_EQUAL_FLOAT(b.integ, a.integ);
}

// ---------------------------------------------------------------- inputs / output
static void test_inputs_from_telemetry() {
  vesc_telemetry_t t;
  memset(&t, 0, sizeof t);
  const uint32_t now = 50000;
  t.temp_motor = 41.5f;
  t.temp_fet = 38.0f;
  t.erpm = 150.0f * (VESC_MOTOR_POLES / 2.0f);  // 150 mechanical rpm
  t.t_ms[VESC_IDX_STATUS_4] = now - 100;
  t.t_ms[VESC_IDX_STATUS_1] = now - 100;
  PumpInputs in = pump_inputs(t, now);
  TEST_ASSERT_TRUE(in.fresh && in.mot_ok && in.fet_ok && in.turning);
  TEST_ASSERT_EQUAL_FLOAT(41.5f, in.t_mot);
  t.erpm = -t.erpm;  // reverse turns too
  TEST_ASSERT_TRUE(pump_inputs(t, now).turning);
  t.erpm = 50.0f * (VESC_MOTOR_POLES / 2.0f);
  TEST_ASSERT_FALSE(pump_inputs(t, now).turning);
  t.erpm = 1000.0f * (VESC_MOTOR_POLES / 2.0f);
  t.t_ms[VESC_IDX_STATUS_1] = now - VESC_STALE_R1_MS - 1;  // an rpm that old says nothing
  TEST_ASSERT_FALSE(pump_inputs(t, now).turning);
  t.temp_motor = 260.0f;  // no NTC wired
  in = pump_inputs(t, now);
  TEST_ASSERT_FALSE(in.mot_ok);
  TEST_ASSERT_TRUE(in.fet_ok);
  t.t_ms[VESC_IDX_STATUS_4] = now - VESC_STALE_R1_MS - 1;
  TEST_ASSERT_FALSE(pump_inputs(t, now).fresh);
  t.t_ms[VESC_IDX_STATUS_4] = 0;  // never received
  TEST_ASSERT_FALSE(pump_inputs(t, now).fresh);
}

static void test_duty_to_ledc() {
  TEST_ASSERT_EQUAL_UINT32(0, pump_duty_to_ledc(0.0f, 10));
  TEST_ASSERT_EQUAL_UINT32(1024, pump_duty_to_ledc(100.0f, 10));  // 2^bits = constantly on
  TEST_ASSERT_EQUAL_UINT32(512, pump_duty_to_ledc(50.0f, 10));
  TEST_ASSERT_EQUAL_UINT32(205, pump_duty_to_ledc(20.0f, 10));  // 204.8 rounded
  TEST_ASSERT_EQUAL_UINT32(2048, pump_duty_to_ledc(100.0f, 11));
  TEST_ASSERT_EQUAL_UINT32(0, pump_duty_to_ledc(-5.0f, 10));
  TEST_ASSERT_EQUAL_UINT32(1024, pump_duty_to_ledc(150.0f, 10));
  TEST_ASSERT_EQUAL_UINT32(0, pump_duty_to_ledc(NAN, 10));
}

static void test_mode_and_reason_names() {
  TEST_ASSERT_EQUAL_STRING("RUN", pump_mode_str(PUMP_MODE_RUN));
  TEST_ASSERT_EQUAL_STRING("BLIND", pump_mode_str(PUMP_MODE_BLIND));
  TEST_ASSERT_EQUAL_STRING("?", pump_mode_str(99));
  // every reason fits next to "BLIND 100% " on a 21-char row
  for (uint8_t w = PUMP_WHY_NONE; w <= PUMP_WHY_NO_TACH; ++w) TEST_ASSERT_TRUE(strlen(pump_why_str(w)) <= 9);
  TEST_ASSERT_EQUAL_STRING("no link", pump_why_str(PUMP_WHY_STALE));
  TEST_ASSERT_EQUAL_STRING("stalled", pump_why_str(PUMP_WHY_STALLED));
  TEST_ASSERT_EQUAL_STRING("no tach", pump_why_str(PUMP_WHY_NO_TACH));
}

// ---------------------------------------------------------------- the tach
#if PUMP_TACH_ENABLE
static PumpInputs with_tach(PumpInputs in, uint8_t tach) {
  in.tach = tach;
  return in;
}

static void test_inputs_leave_the_tach_to_the_task() {
  vesc_telemetry_t t;
  memset(&t, 0, sizeof t);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_NONE, pump_inputs(t, 1000).tach);
}

static void test_stall_while_running_forces_the_stall_duty() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  const PumpTargets tg = def_targets();
  run(c, temps(60.0f, 40.0f, true), tg, &now, 10);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
  const float d = run(c, with_tach(temps(60.0f, 40.0f, true), PUMP_TACH_STALL), tg, &now, 1);
  TEST_ASSERT_EQUAL_FLOAT((float)PUMP_TACH_STALL_DUTY_PCT, d);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_FULL, c.mode);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_STALLED, c.why);
  run(c, with_tach(temps(60.0f, 40.0f, true), PUMP_TACH_NO_SIGNAL), tg, &now, 1);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_NO_TACH, c.why);
  // The tach says OK again: RUN continues from the forced duty and eases down (bumpless).
  const float back = run(c, with_tach(temps(60.0f, 40.0f, true), PUMP_TACH_OK), tg, &now, 1);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_RUN, c.mode);
  TEST_ASSERT_TRUE(back > 50.0f);
  TEST_ASSERT_EQUAL_FLOAT((float)PUMP_MIN_DUTY_PCT,
                          run(c, with_tach(temps(60.0f, 40.0f, true), PUMP_TACH_OK), tg, &now, (int)(600.0f / DT)));
}

static void test_stall_never_starts_a_stopped_pump() {
  // A stale alarm must not run a pump that has no reason to run (cool, motor still).
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, run(c, with_tach(temps(25.0f, 28.0f), PUMP_TACH_STALL), def_targets(), &now, 10));
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_OFF, c.mode);
}

static void test_stall_does_not_block_the_stop() {
  // Stalled at full duty, then everything is cool and still: the after-run ends it as usual.
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  run(c, with_tach(temps(25.0f, 28.0f, true), PUMP_TACH_STALL), def_targets(), &now, 2);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_FULL, c.mode);
  run(c, with_tach(temps(25.0f, 28.0f), PUMP_TACH_STALL), def_targets(), &now, PUMP_MIN_RUN_MS / PUMP_PERIOD_MS + 2);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_OFF, c.mode);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.duty);
}

static void test_quiet_tach_states_change_nothing() {
  const uint8_t quiet[] = {PUMP_TACH_NONE, PUMP_TACH_WAIT, PUMP_TACH_OK, PUMP_TACH_RUNS_AT_0, PUMP_TACH_ERR};
  for (uint8_t q : quiet) {
    PumpCtl a, b;
    pump_init(a);
    pump_init(b);
    uint32_t ta = 1000, tb = 1000;
    const float da = run(a, temps(70.0f, 40.0f, true), def_targets(), &ta, 20);
    const float db = run(b, with_tach(temps(70.0f, 40.0f, true), q), def_targets(), &tb, 20);
    TEST_ASSERT_EQUAL_FLOAT(da, db);
    TEST_ASSERT_EQUAL_UINT8(a.mode, b.mode);
  }
}

static void test_derating_and_blind_keep_their_reason() {
  PumpCtl c;
  pump_init(c);
  uint32_t now = 1000;
  run(c, with_tach(temps(40.0f, 76.0f, true), PUMP_TACH_STALL), def_targets(), &now, 1);
  TEST_ASSERT_EQUAL_UINT8(PUMP_WHY_DERATE, c.why);  // the same 100 %, the more urgent reason
  run(c, with_tach(stale(), PUMP_TACH_STALL), def_targets(), &now, 1);
  TEST_ASSERT_EQUAL_UINT8(PUMP_MODE_BLIND, c.mode);
}
#endif  // PUMP_TACH_ENABLE

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_targets_from_default_limits);
  RUN_TEST(test_targets_follow_the_vesc_limits);
  RUN_TEST(test_limits_source);
  RUN_TEST(test_off_until_the_vesc_was_seen);
  RUN_TEST(test_blind_once_seen_data_goes_stale);
  RUN_TEST(test_blind_without_a_plausible_sensor);
  RUN_TEST(test_missing_motor_sensor_controls_on_the_fet);
  RUN_TEST(test_turning_runs_at_minimum_while_cool);
  RUN_TEST(test_after_run_then_off);
  RUN_TEST(test_warm_start_and_hysteresis);
  RUN_TEST(test_turning_restarts_the_after_run);
  RUN_TEST(test_pi_ramps_above_target);
  RUN_TEST(test_anti_windup_at_full_duty);
  RUN_TEST(test_full_at_derating_start_then_bumpless);
  RUN_TEST(test_blind_to_run_is_bumpless);
  RUN_TEST(test_dt_is_clamped);
  RUN_TEST(test_inputs_from_telemetry);
  RUN_TEST(test_duty_to_ledc);
  RUN_TEST(test_mode_and_reason_names);
#if PUMP_TACH_ENABLE
  RUN_TEST(test_inputs_leave_the_tach_to_the_task);
  RUN_TEST(test_stall_while_running_forces_the_stall_duty);
  RUN_TEST(test_stall_never_starts_a_stopped_pump);
  RUN_TEST(test_stall_does_not_block_the_stop);
  RUN_TEST(test_quiet_tach_states_change_nothing);
  RUN_TEST(test_derating_and_blind_keep_their_reason);
#endif
  return UNITY_END();
}
