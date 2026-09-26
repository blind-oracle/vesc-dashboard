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

  // `tags` = how many slots are configured: 1 = single-tag boat, 2 = two tags aboard.
  explicit Sim(unsigned tags = 1, uint32_t t0 = T0) {
    deadman_init(c);
    memset(&in, 0, sizeof in);
    in.ble_healthy = true;
    in.poll_fresh = true;
    for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
      in.tag[i].configured = i < tags;
      in.tag[i].rssi = -60;
    }
    now = t0;
  }

  // Advance the clock without an advert.
  void idle(uint32_t ms) {
    now += ms;
    step();
  }

  // An advert from tag 1 lands right now, then one evaluation.
  void advert(int8_t rssi = -60) { advert_tag(0, rssi); }

  // ... or from any slot. Per-tag by construction: this is what proves two tags cannot
  // share an arming burst or mask each other's gap statistic.
  void advert_tag(unsigned i, int8_t rssi = -60) {
    in.tag[i].t_ms = now ? now : 1u;
    in.tag[i].rssi = rssi;
    in.tag[i].reports++;
    step();
  }

  uint32_t tag_age(unsigned i) const { return deadman_tag_age(in, i, now); }

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

  // Enrol one tag the normal way: a burst of its own adverts 200 ms apart.
  void arm(unsigned i = 0) {
    for (int k = 0; k < DEADMAN_ARM_REPORTS; ++k) {
      advert_tag(i);
      if (k + 1 < DEADMAN_ARM_REPORTS) idle(200);
    }
    TEST_ASSERT_TRUE_MESSAGE(c.tag[i].enrolled, "tag did not enrol after a full burst");
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

  s.now = s.in.tag[0].t_ms + DEADMAN_TIMEOUT_MS - 1;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_GRACE, s.c.state);
  TEST_ASSERT_FALSE(s.c.cut);

  s.now = s.in.tag[0].t_ms + DEADMAN_TIMEOUT_MS;  // exactly at the timeout
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
  TEST_ASSERT_EQUAL_UINT32(900, s.c.tag[0].gap_max_ms);
  // and it restarts with each armed period
  s.idle(DEADMAN_TIMEOUT_MS);
  s.advert();
  s.in.reset_req = true;
  s.step();
  s.in.reset_req = false;
  TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
  TEST_ASSERT_EQUAL_UINT32(0, s.c.tag[0].gap_max_ms);
}

// ---------------------------------------------------------------- clock wrap
static void test_wraparound_behaves_identically() {
  Sim s(1, 0xFFFFF000u);  // ~4 s before the 32-bit millis() rollover
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
  Sim s(2);
  s.step();
  // Both the per-tag age and the enrolled-set aggregate report "never", and the aggregate
  // must stay UINT32_MAX rather than collapsing to 0 - a never-seen tag must not look fresh.
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, s.tag_age(0));
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, s.tag_age(1));
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, deadman_age(s.c, s.in, s.now));
  TEST_ASSERT_EQUAL_UINT32(0u, deadman_enrolled_count(s.c));
  // An out-of-range slot is defined too, so a loop bug cannot read rubbish.
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, deadman_tag_age(s.in, DEADMAN_TAGS, s.now));
}

// ---------------------------------------------------------------- BMS radio gate
static void test_connect_gate() {
  const uint32_t now = T0;
  const uint32_t win = 2500;
  SharedState st;
  memset(&st, 0, sizeof st);
  st.deadman.state = DM_ARMED;
  st.deadman.enrolled_count = 1;
  st.deadman.tags[0].configured = true;
  st.deadman.tags[0].enrolled = true;
  st.deadman.tags[0].t_ms = now;

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
  st.deadman.tags[0].t_ms = now - (DEADMAN_TIMEOUT_MS - win);
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));

  // GRACE never opens a window; TRIPPED and FAULT always may (the motor is already cut).
  st.deadman.tags[0].t_ms = now;
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
  st.deadman.tags[0].t_ms = 0;
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


// ================================================================ two tags
// Each of these targets one specific way a second tag can go wrong. The first three are
// the ones that would look armed while guarding nothing.

// Two tags each contributing half a burst must arm NOTHING: a shared arming counter would
// enrol a system where neither tag is actually aboard.
static void test_two_tags_cannot_share_an_arming_burst() {
  Sim s(2);
  for (int round = 0; round < 4; ++round) {
    s.advert_tag(0);
    s.idle(50);
    s.advert_tag(1);
    s.idle(50);
    // Each tag has seen `round+1` adverts of its own - never enough on its own until
    // DEADMAN_ARM_REPORTS, and the two must not be added together.
    if (round + 1 < DEADMAN_ARM_REPORTS) {
      TEST_ASSERT_EQUAL_INT(DM_WAIT_TAG, s.c.state);
      TEST_ASSERT_FALSE(s.c.tag[0].enrolled);
      TEST_ASSERT_FALSE(s.c.tag[1].enrolled);
    }
  }
  // After DEADMAN_ARM_REPORTS rounds both have legitimately armed on their own adverts.
  TEST_ASSERT_TRUE(s.c.tag[0].enrolled);
  TEST_ASSERT_TRUE(s.c.tag[1].enrolled);
}

// A weak tag must not be carried in by a strong one: the RSSI gate is per tag.
static void test_weak_tag_is_not_armed_by_a_strong_one() {
  Sim s(2);
  for (int i = 0; i < DEADMAN_ARM_REPORTS + 2; ++i) {
    s.advert_tag(0, (int8_t)(DEADMAN_ARM_RSSI - 10));  // tag 1 is far away
    s.advert_tag(1, -50);                              // tag 2 is right here
    s.idle(100);
  }
  TEST_ASSERT_FALSE(s.c.tag[0].enrolled);
  TEST_ASSERT_TRUE(s.c.tag[1].enrolled);
  TEST_ASSERT_EQUAL_UINT32(1u, deadman_enrolled_count(s.c));
}

// Tag 2's adverts must not mask tag 1's own worst gap. A shared prev_advert_ms would
// measure the gap in the UNION of both streams, which is smaller than either tag's, and
// the README tells the reader to size DEADMAN_TIMEOUT_MS from this number.
static void test_gap_is_measured_per_tag() {
  Sim s(2);
  s.arm(0);
  s.arm(1);
  s.advert_tag(0);  // re-stamp tag 1's reference: arm(1) itself advanced the clock
  // Tag 2 chatters every 100 ms while tag 1 goes quiet for 900 ms.
  for (int i = 0; i < 9; ++i) {
    s.idle(100);
    s.advert_tag(1);
  }
  s.advert_tag(0);
  TEST_ASSERT_EQUAL_UINT32(900, s.c.tag[0].gap_max_ms);
  TEST_ASSERT_TRUE(s.c.tag[1].gap_max_ms <= 100);
}

// The headline rule: with both enrolled, either one alone keeps the motor running, and the
// cut comes only once BOTH have gone quiet - timed from the LAST one to fall silent.
static void test_either_tag_alone_keeps_the_motor_alive() {
  Sim s(2);
  s.arm(0);
  s.arm(1);
  TEST_ASSERT_EQUAL_UINT32(2u, deadman_enrolled_count(s.c));

  // Tag 1 dies. Tag 2 alone holds the motor for far longer than the timeout.
  for (int i = 0; i < 100; ++i) {
    s.idle(200);
    s.advert_tag(1);
    TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
    TEST_ASSERT_FALSE(s.c.cut);
  }
  TEST_ASSERT_EQUAL_UINT32(0, s.c.trips);

  // Now tag 2 goes too: the countdown starts from tag 2's last advert, not tag 1's.
  s.now = s.in.tag[1].t_ms + DEADMAN_TIMEOUT_MS - 1;
  s.step();
  TEST_ASSERT_FALSE(s.c.cut);
  s.now = s.in.tag[1].t_ms + DEADMAN_TIMEOUT_MS;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  TEST_ASSERT_TRUE(s.c.cut);
}

// A configured tag that is never present must neither trip anything nor block a trip:
// it simply never enrols, so it is not in the aggregate at all.
static void test_absent_second_tag_is_harmless() {
  Sim s(2);
  s.arm(0);  // only tag 1 is aboard
  TEST_ASSERT_EQUAL_UINT32(1u, deadman_enrolled_count(s.c));
  TEST_ASSERT_FALSE(s.c.tag[1].enrolled);

  // It behaves exactly like a single-tag boat: tag 1 going quiet trips on time.
  for (int i = 0; i < 20; ++i) {
    s.idle(200);
    s.advert_tag(0);
    TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
  }
  s.idle(DEADMAN_TIMEOUT_MS);
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  TEST_ASSERT_TRUE(s.c.cut);
}

// The enrolment window closes and freezes the set. A tag that turns up afterwards - the one
// left in the car, still in range at the dock - must never join, and must never extend the
// margin: that is the whole point of deciding the set once.
static void test_late_tag_never_joins() {
  Sim s(2);
  s.arm(0);
  TEST_ASSERT_TRUE(s.c.enrol_open);
  while (s.c.enrol_open) {  // keep tag 1 alive while the window closes
    s.idle(200);
    s.advert_tag(0);
  }
  TEST_ASSERT_FALSE(s.c.enrol_open);

  // Tag 2 now shouts continuously, loudly, with tag 1 still aboard. It must not enrol ...
  for (int i = 0; i < 50; ++i) {
    s.idle(100);
    s.advert_tag(1, -40);
    s.advert_tag(0);
  }
  TEST_ASSERT_FALSE(s.c.tag[1].enrolled);
  TEST_ASSERT_EQUAL_UINT32(1u, deadman_enrolled_count(s.c));

  // ... and must not keep the motor alive once tag 1 goes, however loud it is.
  s.in.vesc_status = VESC_STATUS_KILL_SW;  // the VESC confirms, so the cut latches rather than faulting
  for (int i = 0; i < 60; ++i) {
    s.idle(100);
    s.advert_tag(1, -40);
  }
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  TEST_ASSERT_TRUE(s.c.cut);
}

// A second tag that arrives while the window is still open DOES join.
static void test_second_tag_joins_inside_the_window() {
  Sim s(2);
  s.arm(0);
  TEST_ASSERT_TRUE(s.c.enrol_open);
  s.arm(1);  // still inside DEADMAN_ENROL_MS
  TEST_ASSERT_EQUAL_UINT32(2u, deadman_enrolled_count(s.c));
  s.idle(DEADMAN_ENROL_MS);
  TEST_ASSERT_FALSE(s.c.enrol_open);
  TEST_ASSERT_EQUAL_UINT32(2u, deadman_enrolled_count(s.c));
}

// A radio outage throws the whole set away: while it was down we could not tell which tags
// were still aboard, so everything must prove itself again.
static void test_radio_outage_clears_the_enrolment_set() {
  Sim s(2);
  s.arm(0);
  s.arm(1);
  s.in.ble_healthy = false;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_UNAVAILABLE, s.c.state);
  s.in.ble_healthy = true;
  s.step();
  TEST_ASSERT_EQUAL_INT(DM_WAIT_TAG, s.c.state);
  TEST_ASSERT_EQUAL_UINT32(0u, deadman_enrolled_count(s.c));
  TEST_ASSERT_EQUAL_UINT32(0u, s.c.tag[0].arm_seen);
  TEST_ASSERT_EQUAL_UINT32(0u, s.c.tag[1].arm_seen);
  // and a fresh enrolment round works
  s.arm(0);
  TEST_ASSERT_TRUE(s.c.enrol_open);
}

// Either enrolled tag being present is enough to clear the latch.
static void test_reset_accepted_with_either_tag() {
  Sim s(2);
  s.arm(0);
  s.arm(1);
  s.idle(DEADMAN_TIMEOUT_MS);
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  s.advert_tag(1);  // only tag 2 came back
  s.in.reset_req = true;
  s.step();
  s.in.reset_req = false;
  TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
  TEST_ASSERT_FALSE(s.c.cut);
  TEST_ASSERT_EQUAL_UINT32(1, s.c.resets);
}

// The BMS radio gate measures its margin against the tag that is actually keeping the
// motor alive, not against the stalest one.
static void test_connect_gate_uses_the_freshest_tag() {
  const uint32_t now = T0, win = 2500;
  SharedState st;
  memset(&st, 0, sizeof st);
  st.deadman.state = DM_ARMED;
  st.deadman.enrolled_count = 2;
  for (unsigned i = 0; i < 2; ++i) {
    st.deadman.tags[i].configured = true;
    st.deadman.tags[i].enrolled = true;
  }
  // Tag 1 is nearly stale, tag 2 is fresh: the margin comes from tag 2, so this is allowed.
  st.deadman.tags[0].t_ms = now - (DEADMAN_TIMEOUT_MS - 100);
  st.deadman.tags[1].t_ms = now;
  TEST_ASSERT_TRUE(deadman_connect_allowed(st, now, win));
  // Both nearly stale: no margin for the blind window.
  st.deadman.tags[1].t_ms = now - (DEADMAN_TIMEOUT_MS - 100);
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));
  // A tag that is NOT enrolled must not lend its freshness.
  st.deadman.tags[1].enrolled = false;
  st.deadman.tags[1].t_ms = now;
  TEST_ASSERT_FALSE(deadman_connect_allowed(st, now, win));
}

// Everything above, through the 32-bit millis() rollover.
static void test_two_tags_across_the_wrap() {
  Sim s(2, 0xFFFFF000u);
  s.arm(0);
  s.arm(1);
  for (int i = 0; i < 100; ++i) {
    s.idle(200);
    s.advert_tag(i % 2);  // the two tags alternate straight through the wrap
    TEST_ASSERT_EQUAL_INT(DM_ARMED, s.c.state);
    TEST_ASSERT_FALSE(s.c.cut);
  }
  s.idle(DEADMAN_TIMEOUT_MS);
  TEST_ASSERT_EQUAL_INT(DM_TRIPPED, s.c.state);
  TEST_ASSERT_TRUE(s.c.cut);
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
  RUN_TEST(test_two_tags_cannot_share_an_arming_burst);
  RUN_TEST(test_weak_tag_is_not_armed_by_a_strong_one);
  RUN_TEST(test_gap_is_measured_per_tag);
  RUN_TEST(test_either_tag_alone_keeps_the_motor_alive);
  RUN_TEST(test_absent_second_tag_is_harmless);
  RUN_TEST(test_late_tag_never_joins);
  RUN_TEST(test_second_tag_joins_inside_the_window);
  RUN_TEST(test_radio_outage_clears_the_enrolment_set);
  RUN_TEST(test_reset_accepted_with_either_tag);
  RUN_TEST(test_connect_gate_uses_the_freshest_tag);
  RUN_TEST(test_two_tags_across_the_wrap);
  return UNITY_END();
}
