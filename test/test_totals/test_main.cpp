// Native Unity tests (pio test -e native) for the lifetime totals in include/totals.h: the
// record math (base + this boot, rounding, saturation), the motor preset, the blob layout and
// its versioned decode, and when a save is due (stop / run / idle / min gap / retry).
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; includes only framework-free headers.
//
// The preset is 100 h here (before config.h sees it), so the tests that do not look at the
// preset load records above it.
#define TOTALS_MOTOR_PRESET_H 100

#include <unity.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "totals.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- fixtures
static const uint32_t STEP = TRIP_PERIOD_MS;  // the trip task's tick
static const uint32_t T0 = 100000;            // boot + 100 s
static const uint32_t H = 3600;

static TotalsRecord rec_of(uint32_t motor_s, uint32_t on_s, uint32_t dist_m, uint32_t wh, uint32_t whc) {
  TotalsRecord r;
  r.motor_s = motor_s;
  r.on_s = on_s;
  r.dist_m = dist_m;
  r.wh = wh;
  r.whc = whc;
  return r;
}

// A record above the preset: the tests that are not about the preset start clean.
static TotalsRecord used() { return rec_of(200 * H, 300 * H, 5000000, 400000, 2000); }

static void assert_rec(const TotalsRecord &want, const TotalsRecord &got) {
  TEST_ASSERT_EQUAL_UINT32(want.motor_s, got.motor_s);
  TEST_ASSERT_EQUAL_UINT32(want.on_s, got.on_s);
  TEST_ASSERT_EQUAL_UINT32(want.dist_m, got.dist_m);
  TEST_ASSERT_EQUAL_UINT32(want.wh, got.wh);
  TEST_ASSERT_EQUAL_UINT32(want.whc, got.whc);
}

// The trip task: a TripState advanced in STEP ticks, and the flash the record goes to.
struct Sim {
  TotalsCalc c;
  TripState t;
  uint32_t now;
  uint32_t motor_ms, on_ms;  // what trip_add_ms would accumulate
  bool powered;              // the VESC link is fresh
  bool fail;                 // the next writes fail
  TotalsRecord flash;        // what the last successful write stored
  unsigned writes, ok_writes;
  uint8_t last_why;
  uint32_t last_write_at;
  explicit Sim(const TotalsRecord &loaded = used(), bool have = true, uint32_t t0 = T0)
      : t(), now(t0), motor_ms(0), on_ms(0), powered(true), fail(false), flash(loaded), writes(0), ok_writes(0),
        last_why(TOTALS_SAVE_NONE), last_write_at(0) {
    totals_init(c, loaded, have, now);
  }
  // One trip tick: `turning` for the STEP just ended, then the decision and the write.
  uint8_t tick(bool turning) {
    now += STEP;
    if (turning) motor_ms += STEP;
    if (powered || turning) on_ms += STEP;
    t.motor_s = motor_ms / 1000u;
    t.on_s = on_ms / 1000u;
    t.turning = turning;
    TotalsRecord rec;
    const uint8_t why = totals_step(c, t, now, rec);
    if (why != TOTALS_SAVE_NONE) {
      writes++;
      last_why = why;
      last_write_at = now;
      if (!fail) {
        flash = rec;
        ok_writes++;
      }
      totals_done(c, rec, !fail, now);
    }
    return why;
  }
  // ms of ticks; returns how many writes happened meanwhile.
  unsigned run(uint32_t ms, bool turning) {
    const unsigned before = writes;
    for (uint32_t i = 0; i < ms / STEP; ++i) tick(turning);
    return writes - before;
  }
};

// ---------------------------------------------------------------- record math
static void test_lifetime_is_base_plus_trip() {
  TotalsCalc c;
  totals_init(c, used(), true, T0);
  TripState t{};
  t.motor_s = 61;
  t.on_s = 120;
  t.dist_m = 12.4f;
  t.wh = 99.5f;
  t.wh_charged = 0.49f;
  const TotalsRecord r = totals_now(c, t);
  assert_rec(rec_of(200 * H + 61, 300 * H + 120, 5000012, 400100, 2000), r);
  // rounding at .5, and negative or NaN trip energy (a counter catch-up overshoot) adds nothing
  t.dist_m = 12.5f;
  t.wh = -3.0f;
  t.wh_charged = NAN;
  const TotalsRecord q = totals_now(c, t);
  TEST_ASSERT_EQUAL_UINT32(5000013, q.dist_m);
  TEST_ASSERT_EQUAL_UINT32(400000, q.wh);
  TEST_ASSERT_EQUAL_UINT32(2000, q.whc);
}

static void test_saturates_instead_of_wrapping() {
  TotalsCalc c;
  totals_init(c, rec_of(UINT32_MAX - 1u, UINT32_MAX, UINT32_MAX - 10u, 5, 0), true, T0);
  TripState t{};
  t.motor_s = 5;
  t.on_s = 1;
  t.dist_m = 1.0e12f;
  t.wh = 1.0e12f;
  const TotalsRecord r = totals_now(c, t);
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, r.motor_s);
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, r.on_s);
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, r.dist_m);
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, r.wh);
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, totals_round(1.0e20f));
  TEST_ASSERT_EQUAL_UINT32(0, totals_round(-1.0e20f));
  TEST_ASSERT_EQUAL_UINT32(0, totals_round(0.49f));
}

// ---------------------------------------------------------------- the preset (100 h here)
static void test_preset_raises_never_lowers() {
  TEST_ASSERT_EQUAL_UINT32(100 * H, totals_preset_s());
  TotalsCalc c;
  totals_init(c, rec_of(10 * H, 1, 2, 3, 4), true, T0);  // below the preset: raised
  TEST_ASSERT_EQUAL_UINT32(100 * H, c.base.motor_s);
  TEST_ASSERT_EQUAL_UINT32(10 * H, c.saved.motor_s);  // not in flash yet
  TEST_ASSERT_EQUAL_UINT32(1, c.base.on_s);           // the other fields are untouched
  totals_init(c, rec_of(150 * H, 0, 0, 0, 0), true, T0);  // above it: no effect
  TEST_ASSERT_EQUAL_UINT32(150 * H, c.base.motor_s);
  totals_init(c, rec_of(999 * H, 9, 9, 9, 9), false, T0);  // no record: the preset on zeros
  assert_rec(rec_of(100 * H, 0, 0, 0, 0), c.base);
  assert_rec(rec_of(0, 0, 0, 0, 0), c.saved);
}

static void test_preset_gets_saved_once() {
  Sim m(rec_of(10 * H, 0, 0, 0, 0), true);
  m.powered = false;  // nothing else happens: only the preset differs from flash
  TEST_ASSERT_EQUAL_UINT(0, m.run(TOTALS_SAVE_IDLE_S * 1000u - STEP, false));
  TEST_ASSERT_EQUAL_UINT(1, m.run(STEP, false));
  TEST_ASSERT_EQUAL_UINT8(TOTALS_SAVE_IDLE, m.last_why);
  TEST_ASSERT_EQUAL_UINT32(100 * H, m.flash.motor_s);
  TEST_ASSERT_EQUAL_UINT(0, m.run(3600u * 1000u, false));  // and never again
}

// ---------------------------------------------------------------- blob layout
static void test_encode_decode_round_trip() {
  uint8_t b[kTotalsBlobSize];
  TEST_ASSERT_EQUAL_size_t(24, sizeof b);
  const TotalsRecord r = rec_of(0x01020304u, 5, 6, 7, 0xA0B0C0D0u);
  totals_encode(r, b);
  // little-endian u32: version 1, then the fields in order
  const uint8_t head[8] = {1, 0, 0, 0, 0x04, 0x03, 0x02, 0x01};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(head, b, 8);
  TEST_ASSERT_EQUAL_HEX8(0xD0, b[20]);
  TEST_ASSERT_EQUAL_HEX8(0xA0, b[23]);
  TotalsRecord back;
  uint32_t ver = 99;
  TEST_ASSERT_TRUE(totals_decode(b, sizeof b, back, &ver));
  TEST_ASSERT_EQUAL_UINT32(kTotalsVersion, ver);
  assert_rec(r, back);
}

static void test_decode_older_newer_and_garbage() {
  uint8_t b[kTotalsBlobMax];
  memset(b, 0, sizeof b);
  TotalsRecord r;
  uint32_t ver = 0;
  // an older layout with two fields: the rest reads 0
  totals_put_u32(b, 1);
  totals_put_u32(b + 4, 11);
  totals_put_u32(b + 8, 22);
  TEST_ASSERT_TRUE(totals_decode(b, 12, r, &ver));
  assert_rec(rec_of(11, 22, 0, 0, 0), r);
  // a newer layout (version 2, seven fields): the five known ones
  totals_put_u32(b, 2);
  for (uint32_t i = 0; i < 7; ++i) totals_put_u32(b + 4u * (i + 1u), 100u + i);
  TEST_ASSERT_TRUE(totals_decode(b, 32, r, &ver));
  TEST_ASSERT_EQUAL_UINT32(2, ver);
  assert_rec(rec_of(100, 101, 102, 103, 104), r);
  // not a record: too short, not whole u32s, version 0, no buffer
  TEST_ASSERT_FALSE(totals_decode(b, 4, r, &ver));
  TEST_ASSERT_FALSE(totals_decode(b, 0, r, &ver));
  TEST_ASSERT_FALSE(totals_decode(b, 22, r, &ver));
  totals_put_u32(b, 0);
  TEST_ASSERT_FALSE(totals_decode(b, 24, r, &ver));
  TEST_ASSERT_FALSE(totals_decode(nullptr, 24, r, nullptr));
  assert_rec(rec_of(0, 0, 0, 0, 0), r);  // a failed decode leaves a zero record
}

// ---------------------------------------------------------------- when to save
static void test_nothing_changed_never_saves() {
  Sim m;
  m.powered = false;  // the dashboard is on, the VESC is not: nothing counts
  TEST_ASSERT_EQUAL_UINT(0, m.run(3u * 3600u * 1000u, false));
  // the same straight on the calc: an all-zero trip never asks for a write
  TotalsCalc c;
  totals_init(c, used(), true, T0);
  TripState t{};
  TotalsRecord rec;
  for (uint32_t ms = 0; ms < 3600u * 1000u; ms += STEP) TEST_ASSERT_EQUAL_UINT8(TOTALS_SAVE_NONE, totals_step(c, t, T0 + ms, rec));
}

static void test_stop_saves_after_still_time() {
  Sim m;
  TEST_ASSERT_EQUAL_UINT(0, m.run(30000, true));  // 30 s under way: below TOTALS_SAVE_RUN_S
  // the still time starts at the first tick that sees the motor still (one STEP after the stop)
  TEST_ASSERT_EQUAL_UINT(0, m.run(TOTALS_STOP_SAVE_MS, false));
  TEST_ASSERT_EQUAL_UINT(1, m.run(STEP, false));
  TEST_ASSERT_EQUAL_UINT8(TOTALS_SAVE_STOP, m.last_why);
  TEST_ASSERT_EQUAL_UINT32(200 * H + 30, m.flash.motor_s);
  TEST_ASSERT_EQUAL_UINT32(300 * H + 33, m.flash.on_s);
  TEST_ASSERT_EQUAL_UINT(0, m.run(60000, false));  // saved: no second stop save
}

static void test_short_reversal_is_no_stop() {
  Sim m;
  TEST_ASSERT_EQUAL_UINT(0, m.run(30000, true));
  TEST_ASSERT_EQUAL_UINT(0, m.run(1000, false));  // through 0 rpm for 1 s
  // back under way: the run save comes after TOTALS_SAVE_RUN_S of unsaved motor time in all
  const unsigned w = m.run(30000, true);
  TEST_ASSERT_EQUAL_UINT(1, w);
  TEST_ASSERT_EQUAL_UINT8(TOTALS_SAVE_RUN, m.last_why);
  TEST_ASSERT_EQUAL_UINT32(200 * H + TOTALS_SAVE_RUN_S, m.flash.motor_s);
}

static void test_run_saves_every_period() {
  Sim m;
  TEST_ASSERT_EQUAL_UINT(5, m.run(5u * TOTALS_SAVE_RUN_S * 1000u, true));
  TEST_ASSERT_EQUAL_UINT8(TOTALS_SAVE_RUN, m.last_why);
  TEST_ASSERT_EQUAL_UINT32(200 * H + 5 * TOTALS_SAVE_RUN_S, m.flash.motor_s);
  // a power cut now loses less than one period
  m.run(TOTALS_SAVE_RUN_S * 1000u - STEP, true);
  TEST_ASSERT_TRUE(m.motor_ms / 1000u - (m.flash.motor_s - 200 * H) < (uint32_t)TOTALS_SAVE_RUN_S);
}

static void test_idle_saves_powered_time_rarely() {
  Sim m;
  TEST_ASSERT_EQUAL_UINT(0, m.run(TOTALS_SAVE_IDLE_S * 1000u - STEP, false));
  TEST_ASSERT_EQUAL_UINT(1, m.run(STEP, false));
  TEST_ASSERT_EQUAL_UINT8(TOTALS_SAVE_IDLE, m.last_why);
  TEST_ASSERT_EQUAL_UINT32(200 * H, m.flash.motor_s);
  TEST_ASSERT_EQUAL_UINT32(300 * H + TOTALS_SAVE_IDLE_S, m.flash.on_s);
  // a day at the dock with the VESC on: one save per TOTALS_SAVE_IDLE_S
  TEST_ASSERT_EQUAL_UINT(86400u / TOTALS_SAVE_IDLE_S, m.run(86400u * 1000u, false));
}

static void test_drift_distance_saved_by_idle() {
  Sim m;
  m.powered = false;
  m.run(60000, false);
  m.t.dist_m = 250.0f;  // towed, or sailing: distance with the motor off
  TEST_ASSERT_EQUAL_UINT(0, m.run(TOTALS_SAVE_IDLE_S * 1000u - 60000u - STEP, false));
  TEST_ASSERT_EQUAL_UINT(1, m.run(STEP, false));
  TEST_ASSERT_EQUAL_UINT32(5000250, m.flash.dist_m);
}

static void test_min_gap_defers_stop_save() {
  Sim m;
  // the run save at TOTALS_SAVE_RUN_S, then a stop right after it
  TEST_ASSERT_EQUAL_UINT(1, m.run(TOTALS_SAVE_RUN_S * 1000u, true));
  const uint32_t run_save = m.last_write_at;
  TEST_ASSERT_EQUAL_UINT(0, m.run(1000, true));
  // still for TOTALS_STOP_SAVE_MS well inside the gap: the stop save waits for the gap
  TEST_ASSERT_EQUAL_UINT(0, m.run(TOTALS_STOP_SAVE_MS, false));
  const unsigned w = m.run(TOTALS_MIN_GAP_S * 1000u, false);
  TEST_ASSERT_EQUAL_UINT(1, w);
  TEST_ASSERT_EQUAL_UINT8(TOTALS_SAVE_STOP, m.last_why);
  TEST_ASSERT_EQUAL_UINT32(run_save + TOTALS_MIN_GAP_S * 1000u, m.last_write_at);
  TEST_ASSERT_EQUAL_UINT32(200 * H + TOTALS_SAVE_RUN_S + 1, m.flash.motor_s);
}

static void test_gap_also_holds_after_boot() {
  Sim m;
  TEST_ASSERT_EQUAL_UINT(0, m.run(2000, true));
  TEST_ASSERT_EQUAL_UINT(0, m.run(TOTALS_STOP_SAVE_MS, false));  // stopped and still, but 5 s after boot
  TEST_ASSERT_EQUAL_UINT(1, m.run(TOTALS_MIN_GAP_S * 1000u, false));
  TEST_ASSERT_EQUAL_UINT32(T0 + TOTALS_MIN_GAP_S * 1000u, m.last_write_at);
}

static void test_blip_under_a_second_saves_nothing() {
  Sim m;
  m.powered = false;
  m.run(400, true);  // 0.4 s: no whole second, nothing differs
  TEST_ASSERT_EQUAL_UINT(0, m.run(60000, false));
}

static void test_failed_write_retries_later() {
  Sim m;
  m.fail = true;
  m.run(30000, true);
  TEST_ASSERT_EQUAL_UINT(1, m.run(TOTALS_STOP_SAVE_MS + STEP, false));  // the stop save fails
  TEST_ASSERT_TRUE(m.c.failed);
  const uint32_t failed_at = m.last_write_at;
  TEST_ASSERT_EQUAL_UINT(0, m.run(TOTALS_RETRY_S * 1000u - STEP, false));
  m.fail = false;
  TEST_ASSERT_EQUAL_UINT(1, m.run(STEP, false));
  TEST_ASSERT_EQUAL_UINT8(TOTALS_SAVE_RETRY, m.last_why);
  TEST_ASSERT_EQUAL_UINT32(failed_at + TOTALS_RETRY_S * 1000u, m.last_write_at);
  TEST_ASSERT_FALSE(m.c.failed);
  TEST_ASSERT_EQUAL_UINT32(200 * H + 30, m.flash.motor_s);
  TEST_ASSERT_EQUAL_UINT(0, m.run(60000, false));
  // a failing flash while running: one attempt per TOTALS_RETRY_S, not one per tick
  m.fail = true;
  TEST_ASSERT_EQUAL_UINT(10, m.run(10u * TOTALS_RETRY_S * 1000u, true));
}

static void test_clock_wrap() {
  Sim m(used(), true, 0xFFFFFFFFu - 20000u);  // 20 s before the 49.7-day wrap
  m.run(30000, true);
  TEST_ASSERT_EQUAL_UINT(1, m.run(TOTALS_STOP_SAVE_MS + STEP, false));
  TEST_ASSERT_EQUAL_UINT8(TOTALS_SAVE_STOP, m.last_why);
  TEST_ASSERT_EQUAL_UINT32(200 * H + 30, m.flash.motor_s);
}

static void test_reason_and_nvs_strings() {
  TEST_ASSERT_EQUAL_STRING("stop", totals_reason_str(TOTALS_SAVE_STOP));
  TEST_ASSERT_EQUAL_STRING("run", totals_reason_str(TOTALS_SAVE_RUN));
  TEST_ASSERT_EQUAL_STRING("idle", totals_reason_str(TOTALS_SAVE_IDLE));
  TEST_ASSERT_EQUAL_STRING("retry", totals_reason_str(TOTALS_SAVE_RETRY));
  TEST_ASSERT_EQUAL_STRING("none", totals_reason_str(TOTALS_SAVE_NONE));
  TEST_ASSERT_EQUAL_STRING("ok", totals_nvs_str(TOTALS_NVS_OK));
  TEST_ASSERT_EQUAL_STRING("new", totals_nvs_str(TOTALS_NVS_NONE));
  TEST_ASSERT_EQUAL_STRING("ERR", totals_nvs_str(TOTALS_NVS_ERR));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_lifetime_is_base_plus_trip);
  RUN_TEST(test_saturates_instead_of_wrapping);
  RUN_TEST(test_preset_raises_never_lowers);
  RUN_TEST(test_preset_gets_saved_once);
  RUN_TEST(test_encode_decode_round_trip);
  RUN_TEST(test_decode_older_newer_and_garbage);
  RUN_TEST(test_nothing_changed_never_saves);
  RUN_TEST(test_stop_saves_after_still_time);
  RUN_TEST(test_short_reversal_is_no_stop);
  RUN_TEST(test_run_saves_every_period);
  RUN_TEST(test_idle_saves_powered_time_rarely);
  RUN_TEST(test_drift_distance_saved_by_idle);
  RUN_TEST(test_min_gap_defers_stop_save);
  RUN_TEST(test_gap_also_holds_after_boot);
  RUN_TEST(test_blip_under_a_second_saves_nothing);
  RUN_TEST(test_failed_write_retries_later);
  RUN_TEST(test_clock_wrap);
  RUN_TEST(test_reason_and_nvs_strings);
  return UNITY_END();
}
