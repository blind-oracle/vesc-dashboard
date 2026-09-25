// Native Unity tests (pio test -e native) for the pure dead-man's switch logic in
// include/deadman_logic.h: arming (burst + RSSI gate), the timeout boundary, the
// latch and its reset rule, the "no radio" state that must never cut, the closed
// loop against the VESC kill-switch status bit, the BMS radio gate, and millis()
// wrap-around.
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude -DDEADMAN_ENABLE=1 ; Arduino-free.
//
// The one invariant that must never break, asserted after every single step of
// every test: the cut line is asserted ONLY in TRIPPED, in FAULT, or in WAIT_TAG
// with DEADMAN_BOOT_CUT. A bug that asserts it anywhere else stops a boat.
#include <unity.h>

#include <stdint.h>
#include <string.h>

#include "deadman_logic.h"

void setUp() {}
void tearDown() {}

static const uint32_t T0 = 100000;  // boot + 100 s

// ---------------------------------------------------------------- fixture
struct Sim {
  DeadmanCalc c;
  DeadmanInputs in;
  uint32_t now;

  Sim(uint32_t t0 = T0) {
    deadman_init(c);
    memset(&in, 0, sizeof in);
    in.ble_healthy = true;
    in.beacon_rssi = -60;
    in.poll_fresh = true;
    now = t0;
  }

  // Advance the clock without an advert.
  void idle(uint32_t ms) {
    now += ms;
    step();
  }

  // An advert lands right now, then one evaluation.
  void advert(int8_t rssi = -60) {
    in.beacon_t_ms = now ? now : 1u;
    in.beacon_rssi = rssi;
    in.beacon_reports++;
    step();
  }

  void step() {
    deadman_update(c, in, now);
    check_invariant();
  }

  // The invariant, checked after every step of every test.
  void check_invariant() const {
    const bool may_cut = c.state == DM_TRIPPED || c.state == DM_FAULT ||
                         (c.state == DM_WAIT_TAG && DEADMAN_BOOT_CUT);
    if (c.cut) TEST_ASSERT_TRUE_MESSAGE(may_cut, "cut asserted in a state that must never cut");
  }

  // Arm the switch the normal way: a burst of adverts 200 ms apart.
  void arm() {
    for (int i = 0; i < DEADMAN_ARM_REPORTS; ++i) {
      advert();
      if (i + 1 < DEADMAN_ARM_REPORTS) idle(200);
    }
    TEST_ASSERT_EQUAL_INT(DM_ARMED, c.state);
  }
};

// ---------------------------------------------------------------- arming
static void test_boot_state_is_wait_tag() {
  Sim s;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_WAIT_TAG, s.c.state);
  TEST_ASSERT_EQUAL_INT(DEADMAN_BOOT_CUT ? 1 : 0, s.c.cut ? 1 : 0);
  // No tag for a long time must NOT trip: there is nothing to vouch for yet.
  s.idle(10u * DEADMAN_TIMEOUT_MS);
  TEST_ASSERT_EQUAL_INT(DM_WAIT_TAG, s.c.state);
}

static void test_one_advert_does_not_arm() {
  Sim s;
  s.advert();
  TEST_ASSERT_EQUAL_INT(DM_WAIT_TAG, s.c.state);
}

static void test_burst_arms() {
  Sim s;
  s.arm();
  TEST_ASSERT_FALSE(s.c.cut);
}

static void test_adverts_spread_beyond_the_window_do_not_arm() {
  Sim s;
  for (int i = 0; i < DEADMAN_ARM_REPORTS * 3; ++i) {
    s.advert();
    s.idle(DEADMAN_ARM_WINDOW_MS + 100);  // each one starts a fresh burst
    if (s.c.state == DM_TRIPPED) break;   // must not happen: never armed, so never trips
  }
  TEST_ASSERT_EQUAL_INT(DM_WAIT_TAG, s.c.state);
  TEST_ASSERT_EQUAL_UINT32(0, s.c.trips);
}

static void test_weak_adverts_do_not_arm() {
  Sim s;
  for (int i = 0; i < DEADMAN_ARM_REPORTS + 2; ++i) {
    s.advert((int8_t)(DEADMAN_ARM_RSSI - 5));  // a tag in the car park
    s.idle(200);
  }
  TEST_ASSERT_EQUAL_INT(DM_WAIT_TAG, s.c.state);
  // ... but a strong one completing the burst arms immediately
  s.advert((int8_t)DEADMAN_ARM_RSSI);
  TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
}

// ---------------------------------------------------------------- the timeout
static void test_stays_armed_while_the_tag_keeps_advertising() {
  Sim s;
  s.arm();
  for (int i = 0; i < 1000; ++i) {  // ~3.3 minutes at 200 ms
    s.idle(200);
    s.advert();
    TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
    TEST_ASSERT_FALSE(s.c.cut);
  }
  TEST_ASSERT_EQUAL_UINT32(0, s.c.trips);
}

static void test_grace_then_trip_at_the_boundary() {
  Sim s;
  s.arm();
  s.idle(DEADMAN_WARN_MS - 1);
  TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
  s.idle(1);  // exactly at the warn point
  TEST_ASSERT_EQUAL_INT(DM_GRACE, s.c.state);
  TEST_ASSERT_FALSE(s.c.cut);  // GRACE still permits the motor

  s.now = s.in.beacon_t_ms + DEADMAN_TIMEOUT_MS - 1;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_GRACE, s.c.state);
  TEST_ASSERT_FALSE(s.c.cut);

  s.now = s.in.beacon_t_ms + DEADMAN_TIMEOUT_MS;  // exactly at the timeout
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  TEST_ASSERT_TRUE(s.c.cut);
  TEST_ASSERT_EQUAL_UINT32(1, s.c.trips);
}

static void test_grace_recovers_when_the_tag_comes_back() {
  Sim s;
  s.arm();
  s.idle(DEADMAN_WARN_MS);
  TEST_ASSERT_EQUAL_INT(DM_GRACE, s.c.state);
  s.advert();
  TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
  TEST_ASSERT_EQUAL_UINT32(0, s.c.trips);
}

// ---------------------------------------------------------------- the latch
static void test_trip_latches_against_the_tag_returning() {
  Sim s;
  s.arm();
  s.idle(DEADMAN_TIMEOUT_MS);
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  s.in.vesc_status = VESC_STATUS_KILL_SW;  // the VESC sees the cut, so the loop confirms instead of faulting

  for (int i = 0; i < 50; ++i) {  // the tag is back and shouting; the cut must hold
    s.idle(200);
    s.advert();
    TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
    TEST_ASSERT_TRUE(s.c.cut);
  }
}

static void test_reset_needs_the_tag_present() {
  Sim s;
  s.arm();
  s.idle(DEADMAN_TIMEOUT_MS);
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);

  // Reset with the tag still missing: refused, the motor stays cut.
  s.in.reset_req = true;
  s.step();
  s.in.reset_req = false;
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  TEST_ASSERT_TRUE(s.c.cut);
  TEST_ASSERT_EQUAL_UINT32(1, s.c.resets_refused);
  TEST_ASSERT_EQUAL_UINT32(0, s.c.resets);

  // Tag back, reset accepted.
  s.advert();
  s.in.reset_req = true;
  s.step();
  s.in.reset_req = false;
  TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
  TEST_ASSERT_FALSE(s.c.cut);
  TEST_ASSERT_EQUAL_UINT32(1, s.c.resets);
}

static void test_reset_refused_with_a_stale_tag() {
  Sim s;
  s.arm();
  s.idle(DEADMAN_TIMEOUT_MS);
  s.advert();                            // the tag blipped once ...
  s.idle(DEADMAN_RESET_FRESH_MS + 1);    // ... but that was too long ago
  s.in.reset_req = true;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  TEST_ASSERT_EQUAL_UINT32(1, s.c.resets_refused);
}

// ---------------------------------------------------------------- no radio
static void test_unhealthy_radio_never_cuts() {
  Sim s;
  s.arm();
  s.in.ble_healthy = false;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_UNAVAILABLE, s.c.state);
  // However long the radio stays down, our own failure must not stop the boat.
  for (int i = 0; i < 100; ++i) {
    s.idle(1000);
    TEST_ASSERT_EQUAL_INT(DM_UNAVAILABLE, s.c.state);
    TEST_ASSERT_FALSE(s.c.cut);
  }
  TEST_ASSERT_EQUAL_UINT32(0, s.c.trips);
}

static void test_radio_returns_requires_rearming() {
  Sim s;
  s.arm();
  s.in.ble_healthy = false;
  s.step();
  s.in.ble_healthy = true;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_WAIT_TAG, s.c.state);  // the tag must prove itself again
  s.arm();
}

// ---------------------------------------------------------------- closed loop
static void test_confirm_ok_when_the_vesc_agrees() {
  Sim s;
  s.arm();
  s.idle(DEADMAN_TIMEOUT_MS);
  TEST_ASSERT_TRUE(s.c.cut);
  s.in.vesc_status = VESC_STATUS_KILL_SW;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_CONFIRM_OK, s.c.confirm);
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
}

static void test_confirm_failure_becomes_fault() {
  Sim s;
  s.arm();
  s.idle(DEADMAN_TIMEOUT_MS);
  s.in.vesc_status = 0;  // the VESC never sees its kill switch: broken wire or lost config
  s.idle(DEADMAN_CONFIRM_MS - 1);
  TEST_ASSERT_EQUAL_INT(DM_CONFIRM_WAIT, s.c.confirm);
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  s.idle(1);
  TEST_ASSERT_EQUAL_INT(DM_CONFIRM_FAILED, s.c.confirm);
  TEST_ASSERT_EQUAL_INT(DM_FAULT, s.c.state);
  TEST_ASSERT_TRUE(s.c.cut);  // keeps asserting

  // A FAULT never clears by itself, not even with the tag back and a reset.
  s.advert();
  s.in.reset_req = true;
  s.idle(100);
  TEST_ASSERT_EQUAL_INT(DM_FAULT, s.c.state);
  TEST_ASSERT_TRUE(s.c.cut);
}

static void test_dead_can_bus_is_stale_never_fault() {
  Sim s;
  s.arm();
  s.idle(DEADMAN_TIMEOUT_MS);
  s.in.poll_fresh = false;  // the VESC is off or the bus is cut
  s.idle(10u * DEADMAN_CONFIRM_MS);
  TEST_ASSERT_EQUAL_INT(DM_CONFIRM_STALE, s.c.confirm);
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);  // never FAULT: nothing can be concluded
}

// ---------------------------------------------------------------- statistics
static void test_gap_max_tracks_the_worst_gap() {
  Sim s;
  s.arm();
  s.idle(300);
  s.advert();
  s.idle(900);
  s.advert();
  s.idle(400);
  s.advert();
  TEST_ASSERT_EQUAL_UINT32(900, s.c.gap_max_ms);
  // and it restarts with each armed period
  s.idle(DEADMAN_TIMEOUT_MS);
  s.advert();
  s.in.reset_req = true;
  s.step();
  s.in.reset_req = false;
  TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
  TEST_ASSERT_EQUAL_UINT32(0, s.c.gap_max_ms);
}

// ---------------------------------------------------------------- clock wrap
static void test_wraparound_behaves_identically() {
  Sim s(0xFFFFF000u);  // ~4 s before the 32-bit millis() rollover
  s.arm();
  for (int i = 0; i < 100; ++i) {  // straight through the wrap
    s.idle(200);
    s.advert();
    TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
    TEST_ASSERT_FALSE(s.c.cut);
  }
  s.idle(DEADMAN_TIMEOUT_MS);
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  TEST_ASSERT_TRUE(s.c.cut);
}

static void test_never_seen_tag_is_not_a_fresh_tag() {
  Sim s;
  s.step();
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, deadman_tag_age(s.in, s.now));
  TEST_ASSERT_FALSE(deadman_tag_fresh(s.in, s.now, DEADMAN_TIMEOUT_MS));
}

// ---------------------------------------------------------------- BMS radio gate
static void test_connect_gate() {
  const uint32_t now = T0;
  const uint32_t win = 2500;
  SharedState st;
  memset(&st, 0, sizeof st);
  st.deadman.state = DM_ARMED;
  st.deadman.beacon_t_ms = now;

  // Armed, tag fresh, nothing reporting movement -> allowed.
  TEST_ASSERT_TRUE(deadman_connect_allowed(st, now, win));

  // A fresh VESC reporting erpm above the threshold forbids it.
  st.vesc.t.erpm = (float)DEADMAN_STANDSTILL_ERPM + 1.0f;
  st.vesc.t.t_ms[VESC_IDX_STATUS_1] = now;
#if DEADMAN_STANDSTILL_ERPM >= 0
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));
#endif
  // A STALE VESC must not by itself forbid it: absence of evidence is not evidence of motion.
  st.vesc.t.t_ms[VESC_IDX_STATUS_1] = now - VESC_STALE_R1_MS - 1;
  TEST_ASSERT_TRUE(deadman_connect_allowed(st, now, win));

  // A fresh GNSS fix above the speed threshold forbids it.
  st.gnss.last_pvt_ms = now;
  st.gnss.fix_ok = true;
  st.gnss.gspeed_mm_s = DEADMAN_STANDSTILL_MM_S + 1;
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));
  st.gnss.gspeed_mm_s = DEADMAN_STANDSTILL_MM_S;
  TEST_ASSERT_TRUE(deadman_connect_allowed(st, now, win));

  // Too little margin left before the timeout: the blind window would cause the trip.
  st.deadman.beacon_t_ms = now - (DEADMAN_TIMEOUT_MS - win);
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));

  // GRACE never opens a window; TRIPPED and FAULT always may (the motor is already cut).
  st.deadman.beacon_t_ms = now;
  st.deadman.state = DM_GRACE;
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));
  st.deadman.state = DM_TRIPPED;
  TEST_ASSERT_TRUE(deadman_connect_allowed(st, now, win));
  st.deadman.state = DM_FAULT;
  TEST_ASSERT_TRUE(deadman_connect_allowed(st, now, win));
  st.deadman.state = DM_WAIT_TAG;
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));
  st.deadman.state = DM_UNAVAILABLE;
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));
  // A tag that was never seen must never open a window either.
  st.deadman.state = DM_ARMED;
  st.deadman.beacon_t_ms = 0;
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));
}

// ---------------------------------------------------------------- names
static void test_state_names() {
  TEST_ASSERT_EQUAL_STRING("NO TAG", deadman_state_str(DM_WAIT_TAG));
  TEST_ASSERT_EQUAL_STRING("ARMED", deadman_state_str(DM_ARMED));
  TEST_ASSERT_EQUAL_STRING("GRACE", deadman_state_str(DM_GRACE));
  TEST_ASSERT_EQUAL_STRING("TRIPPED", deadman_state_str(DM_TRIPPED));
  TEST_ASSERT_EQUAL_STRING("NO RADIO", deadman_state_str(DM_UNAVAILABLE));
  TEST_ASSERT_EQUAL_STRING("FAULT", deadman_state_str(DM_FAULT));
  TEST_ASSERT_EQUAL_STRING("?", deadman_state_str(200));
  TEST_ASSERT_EQUAL_STRING("OK", deadman_confirm_str(DM_CONFIRM_OK));
  TEST_ASSERT_EQUAL_STRING("STALE", deadman_confirm_str(DM_CONFIRM_STALE));
  // every state name fits the 21-char grid row with room for a value column
  for (unsigned i = 0; i <= DM_FAULT; ++i) TEST_ASSERT_TRUE(strlen(deadman_state_str((uint8_t)i)) <= 8);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_boot_state_is_wait_tag);
  RUN_TEST(test_one_advert_does_not_arm);
  RUN_TEST(test_burst_arms);
  RUN_TEST(test_adverts_spread_beyond_the_window_do_not_arm);
  RUN_TEST(test_weak_adverts_do_not_arm);
  RUN_TEST(test_stays_armed_while_the_tag_keeps_advertising);
  RUN_TEST(test_grace_then_trip_at_the_boundary);
  RUN_TEST(test_grace_recovers_when_the_tag_comes_back);
  RUN_TEST(test_trip_latches_against_the_tag_returning);
  RUN_TEST(test_reset_needs_the_tag_present);
  RUN_TEST(test_reset_refused_with_a_stale_tag);
  RUN_TEST(test_unhealthy_radio_never_cuts);
  RUN_TEST(test_radio_returns_requires_rearming);
  RUN_TEST(test_confirm_ok_when_the_vesc_agrees);
  RUN_TEST(test_confirm_failure_becomes_fault);
  RUN_TEST(test_dead_can_bus_is_stale_never_fault);
  RUN_TEST(test_gap_max_tracks_the_worst_gap);
  RUN_TEST(test_wraparound_behaves_identically);
  RUN_TEST(test_never_seen_tag_is_not_a_fresh_tag);
  RUN_TEST(test_connect_gate);
  RUN_TEST(test_state_names);
  return UNITY_END();
}
