// Unity tests (pio test -e native) for the coolant pump's tachometer logic in
// include/pump_tach.h: the rpm over the window, the spin-up / spin-down grace, STALL and
// NO_SIGNAL with their recovery, RUNS_AT_0, the alarm counter and the no-verdict band.
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; the config.h defaults apply (period
// 500 ms, window 2000 ms, 1 pulse/rev, min 200 rpm, spin-up 3 s, stall 5 s, spin-down 5 s,
// OK hold 2 s, minimum duty 20 %).
#include <unity.h>

#include <stdint.h>
#include <string.h>

#include "pump_tach.h"

void setUp() {}
void tearDown() {}

static_assert(PUMP_TACH_ENABLE, "the native env builds the tach (PIN_PUMP_TACH defaults to 19)");

// A pump simulator: `rpm` turns into pulses at PUMP_TACH_PPR per revolution.
struct Sim {
  PumpTach t;
  uint32_t now = 1000;
  double pulses = 0.0;  // fractional, so slow speeds still produce pulses over time
  uint32_t total() const { return (uint32_t)pulses; }
  // Runs `ms` at `rpm` with `duty` applied, one PUMP_PERIOD_MS tick at a time.
  uint8_t run(uint32_t ms, float rpm, float duty) {
    uint8_t st = t.state;
    for (uint32_t done = 0; done < ms; done += PUMP_PERIOD_MS) {
      now += PUMP_PERIOD_MS;
      pulses += (double)rpm * PUMP_TACH_PPR * PUMP_PERIOD_MS / 60000.0;
      st = pump_tach_step(t, total(), duty, now);
    }
    return st;
  }
  Sim() { pump_tach_init(t); }
};

// ---------------------------------------------------------------- rpm
static void test_rpm_over_the_window() {
  Sim s;
  s.run(PUMP_PERIOD_MS, 3000.0f, 50.0f);
  TEST_ASSERT_FALSE(s.t.full);
  s.run(PUMP_TACH_WINDOW_MS, 3000.0f, 50.0f);
  TEST_ASSERT_TRUE(s.t.full);
  TEST_ASSERT_FLOAT_WITHIN(31.0f, 3000.0f, s.t.rpm);  // +-1 pulse over 2 s = +-30 rpm
  // A speed change shows fully once the window has passed.
  s.run(PUMP_TACH_WINDOW_MS, 1200.0f, 50.0f);
  TEST_ASSERT_FLOAT_WITHIN(31.0f, 1200.0f, s.t.rpm);
}

static void test_rpm_survives_the_counter_wrap() {
  PumpTach t;
  pump_tach_init(t);
  uint32_t now = 5000, total = 0xFFFFFF00u;  // just below 2^32
  for (int i = 0; i < 10; ++i) {
    now += PUMP_PERIOD_MS;
    total += 50;  // 50 pulses per 500 ms = 6000 rpm, wrapping on the way
    pump_tach_step(t, total, 100.0f, now);
  }
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 6000.0f, t.rpm);
}

static void test_first_tick_is_zero_rpm_and_wait() {
  PumpTach t;
  pump_tach_init(t);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, pump_tach_step(t, 0, 0.0f, 0));  // now == 0 is taken as 1
  TEST_ASSERT_EQUAL_FLOAT(0.0f, t.rpm);
  TEST_ASSERT_NOT_EQUAL(0u, t.since);
}

// ---------------------------------------------------------------- running
static void test_spinup_gives_no_verdict_then_ok() {
  Sim s;
  // Slow during the spin-up: no verdict.
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.run(PUMP_TACH_SPINUP_MS - PUMP_PERIOD_MS, 0.0f, 62.0f));
  // Turning by the end of it: OK.
  s.run(PUMP_PERIOD_MS, 3000.0f, 62.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.t.state);  // the 2 s window still mostly holds the start
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.run(PUMP_TACH_WINDOW_MS, 3000.0f, 62.0f));
  TEST_ASSERT_EQUAL_UINT32(0u, s.t.alarms);
}

static void test_stall_after_the_stall_time() {
  Sim s;
  s.run(10000, 3000.0f, 62.0f);  // running fine
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.t.state);
  // The rotor locks: OK holds through the window and the stall time ...
  s.run(PUMP_TACH_WINDOW_MS, 0.0f, 62.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.t.state);
  s.run(PUMP_TACH_STALL_MS - PUMP_PERIOD_MS - PUMP_PERIOD_MS, 0.0f, 62.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.t.state);
  // ... then it is a STALL (pulses were seen since boot), counted once.
  s.run(4 * PUMP_PERIOD_MS, 0.0f, 62.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_STALL, s.t.state);
  TEST_ASSERT_EQUAL_UINT32(1u, s.t.alarms);
  const uint32_t since = s.t.since;
  s.run(10000, 0.0f, 100.0f);  // stays stalled at the forced full duty: still one alarm
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_STALL, s.t.state);
  TEST_ASSERT_EQUAL_UINT32(1u, s.t.alarms);
  TEST_ASSERT_EQUAL_UINT32(since, s.t.since);
}

static void test_no_signal_when_never_a_pulse() {
  Sim s;
  // Started from boot with the FG wire unplugged: NO_SIGNAL after spin-up + stall time, both
  // counted from the first tick that reports the duty (it covers the period before it).
  s.run(PUMP_TACH_SPINUP_MS + PUMP_TACH_STALL_MS, 0.0f, 20.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.t.state);  // the slow time counts from the end of the spin-up
  s.run(PUMP_PERIOD_MS, 0.0f, 20.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_NO_SIGNAL, s.t.state);
  TEST_ASSERT_EQUAL_UINT32(1u, s.t.alarms);
  TEST_ASSERT_FALSE(s.t.seen);
  // A few pulses appear but it is still slow: the same alarm, now named STALL, not a new one.
  s.run(PUMP_TACH_WINDOW_MS, 60.0f, 100.0f);
  TEST_ASSERT_TRUE(s.t.seen);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_STALL, s.t.state);
  TEST_ASSERT_EQUAL_UINT32(1u, s.t.alarms);
}

static void test_stall_recovers_after_the_ok_hold() {
  Sim s;
  s.run(10000, 3000.0f, 62.0f);
  s.run(PUMP_TACH_WINDOW_MS + PUMP_TACH_STALL_MS + PUMP_PERIOD_MS, 0.0f, 62.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_STALL, s.t.state);
  // The rotor frees itself at full duty: the window must fill with speed, then hold PUMP_TACH_OK_MS.
  s.run(PUMP_TACH_WINDOW_MS / 2, 4000.0f, 100.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_STALL, s.t.state);
  s.run(PUMP_TACH_OK_MS + PUMP_TACH_WINDOW_MS, 4000.0f, 100.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.t.state);
  // A second stall is a second alarm.
  s.run(PUMP_TACH_WINDOW_MS + PUMP_TACH_STALL_MS + PUMP_PERIOD_MS, 0.0f, 62.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_STALL, s.t.state);
  TEST_ASSERT_EQUAL_UINT32(2u, s.t.alarms);
}

static void test_driver_restart_pulses_stay_a_stall() {
  // A locked rotor whose driver retries every few seconds gives a few pulses now and then:
  // averaged over the window that stays under the minimum.
  Sim s;
  s.run(10000, 3000.0f, 62.0f);
  s.run(20000, 45.0f, 100.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_STALL, s.t.state);
}

static void test_stop_ends_an_alarm() {
  Sim s;
  s.run(10000, 3000.0f, 62.0f);
  s.run(PUMP_TACH_WINDOW_MS + PUMP_TACH_STALL_MS + PUMP_PERIOD_MS, 0.0f, 62.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_STALL, s.t.state);
  // The controller stops the pump (cool, motor still): nothing should turn, so no alarm.
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.run(PUMP_PERIOD_MS, 0.0f, 0.0f));
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.run(PUMP_TACH_SPINDOWN_MS, 0.0f, 0.0f));
  // The next start judges again from scratch, and a stall then is a new alarm.
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.run(PUMP_PERIOD_MS, 0.0f, 20.0f));
  s.run(PUMP_TACH_SPINUP_MS + PUMP_TACH_STALL_MS, 0.0f, 20.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_STALL, s.t.state);
  TEST_ASSERT_EQUAL_UINT32(2u, s.t.alarms);
}

static void test_brief_dip_does_not_alarm() {
  Sim s;
  s.run(10000, 3000.0f, 62.0f);
  s.run(PUMP_TACH_WINDOW_MS + 1000, 0.0f, 62.0f);  // 3 s without pulses: under the stall time
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.t.state);
  s.run(PUMP_TACH_WINDOW_MS + PUMP_PERIOD_MS, 3000.0f, 62.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.t.state);
  TEST_ASSERT_EQUAL_UINT32(0u, s.t.alarms);
}

// ---------------------------------------------------------------- stopped
static void test_spindown_then_still_is_ok() {
  Sim s;
  s.run(10000, 3000.0f, 62.0f);
  // Commanded off: the rpm of the coast-down is no verdict ...
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.run(PUMP_PERIOD_MS, 1500.0f, 0.0f));
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.run(PUMP_TACH_SPINDOWN_MS - PUMP_PERIOD_MS, 0.0f, 0.0f));
  // ... after the spin-down, still = OK.
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.run(PUMP_PERIOD_MS, 0.0f, 0.0f));
}

static void test_runs_at_zero_duty() {
  Sim s;
  // The PWM stage does not stop it: full speed at 0 % from boot (the transistor is dead).
  s.run(PUMP_TACH_SPINDOWN_MS, 5000.0f, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.t.state);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_RUNS_AT_0, s.run(PUMP_PERIOD_MS, 5000.0f, 0.0f));
  TEST_ASSERT_EQUAL_UINT32(0u, s.t.alarms);  // reported, not an alarm: it is the cool side
  TEST_ASSERT_FALSE(pump_tach_alarm(s.t.state));
  // Fixed: it stops, and after the window and the hold it reads OK.
  s.run(PUMP_TACH_WINDOW_MS, 0.0f, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_RUNS_AT_0, s.t.state);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.run(PUMP_TACH_OK_MS, 0.0f, 0.0f));
}

static void test_power_on_burst_is_no_verdict() {
  // The pump runs flat out for ~0.3 s at power-up until the firmware stops it: the pulses
  // of that burst and the coast-down fall inside the spin-down and are only "seen".
  Sim s;
  s.run(PUMP_PERIOD_MS, 4000.0f, 0.0f);
  s.run(PUMP_TACH_SPINDOWN_MS, 0.0f, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_OK, s.t.state);
  TEST_ASSERT_TRUE(s.t.seen);
}

// ---------------------------------------------------------------- the band in between
static void test_forced_duty_under_min_has_no_verdict() {
  Sim s;
  s.run(30000, 0.0f, 15.0f);  // a commissioning sweep below PUMP_MIN_DUTY_PCT: it may not turn
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.t.state);
  TEST_ASSERT_EQUAL_UINT32(0u, s.t.alarms);
  s.run(30000, 900.0f, 15.0f);
  TEST_ASSERT_EQUAL_UINT8(PUMP_TACH_WAIT, s.t.state);
  TEST_ASSERT_FLOAT_WITHIN(31.0f, 900.0f, s.t.rpm);  // the speed is still measured and shown
}

static void test_strings_and_alarm_set() {
  TEST_ASSERT_TRUE(pump_tach_alarm(PUMP_TACH_STALL));
  TEST_ASSERT_TRUE(pump_tach_alarm(PUMP_TACH_NO_SIGNAL));
  const uint8_t quiet[] = {PUMP_TACH_NONE, PUMP_TACH_WAIT, PUMP_TACH_OK, PUMP_TACH_RUNS_AT_0, PUMP_TACH_ERR};
  for (uint8_t q : quiet) TEST_ASSERT_FALSE(pump_tach_alarm(q));
  for (uint8_t st = PUMP_TACH_NONE; st <= PUMP_TACH_ERR; ++st) TEST_ASSERT_TRUE(strlen(pump_tach_str(st)) <= 10u);
  TEST_ASSERT_EQUAL_STRING("STALL", pump_tach_str(PUMP_TACH_STALL));
  TEST_ASSERT_EQUAL_STRING("NO TACH", pump_tach_str(PUMP_TACH_NO_SIGNAL));
  TEST_ASSERT_EQUAL_STRING("RUNS AT 0%", pump_tach_str(PUMP_TACH_RUNS_AT_0));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_rpm_over_the_window);
  RUN_TEST(test_rpm_survives_the_counter_wrap);
  RUN_TEST(test_first_tick_is_zero_rpm_and_wait);
  RUN_TEST(test_spinup_gives_no_verdict_then_ok);
  RUN_TEST(test_stall_after_the_stall_time);
  RUN_TEST(test_no_signal_when_never_a_pulse);
  RUN_TEST(test_stall_recovers_after_the_ok_hold);
  RUN_TEST(test_driver_restart_pulses_stay_a_stall);
  RUN_TEST(test_stop_ends_an_alarm);
  RUN_TEST(test_brief_dip_does_not_alarm);
  RUN_TEST(test_spindown_then_still_is_ok);
  RUN_TEST(test_runs_at_zero_duty);
  RUN_TEST(test_power_on_burst_is_no_verdict);
  RUN_TEST(test_forced_duty_under_min_has_no_verdict);
  RUN_TEST(test_strings_and_alarm_set);
  return UNITY_END();
}
