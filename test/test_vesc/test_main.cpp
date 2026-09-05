// Native Unity tests for the clean-room VESC STATUS_1..6 decoder (include/vesc_status.h).
// Run with:  pio test -e native
//
// Every vector below is a real wire image (big-endian two's complement) built
// from the scale factors in the plan, so a regression in byte order or scaling
// shows up as a numeric mismatch, not as a "test passes because both sides
// share the bug".
#include <unity.h>

#include <stdint.h>
#include <string.h>

#include "vesc_status.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- helpers
static uint32_t eid(uint32_t packet_id, uint8_t controller_id) { return (packet_id << 8) | controller_id; }

static vesc_telemetry_t zeroed() {
  vesc_telemetry_t t;
  memset(&t, 0, sizeof t);
  return t;
}

// Decodes an extended, DLC 8 frame with want_id = -1 unless told otherwise.
static bool feed(vesc_telemetry_t *t, uint32_t packet_id, uint8_t id, const uint8_t (&d)[8], uint32_t now_ms = 1000,
                 int want_id = -1, bool is_ext = true, uint8_t dlc = 8) {
  return vesc_decode_status(t, eid(packet_id, id), is_ext, d, dlc, now_ms, want_id);
}

static void assert_only_timestamp(const vesc_telemetry_t &t, int idx, uint32_t expected) {
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) {
    if (i == idx) TEST_ASSERT_EQUAL_UINT32(expected, t.t_ms[i]);
    else TEST_ASSERT_EQUAL_UINT32(0u, t.t_ms[i]);
  }
}

// ---------------------------------------------------------------- raw helpers
static void test_be_helpers() {
  const uint8_t p16[] = {0xFF, 0x85};              // -123
  const uint8_t p16b[] = {0x7F, 0xFF};             // 32767
  const uint8_t p32[] = {0xFF, 0xFF, 0xEC, 0x78};  // -5000
  const uint8_t p32b[] = {0x00, 0x00, 0x03, 0xE8}; // 1000
  TEST_ASSERT_EQUAL_INT(-123, vesc_be_i16(p16));
  TEST_ASSERT_EQUAL_INT(32767, vesc_be_i16(p16b));
  TEST_ASSERT_EQUAL_INT32(-5000, vesc_be_i32(p32));
  TEST_ASSERT_EQUAL_INT32(1000, vesc_be_i32(p32b));
}

static void test_eid_macros_and_index() {
  const uint32_t e = eid(VESC_CAN_PACKET_STATUS, 0x4A);
  TEST_ASSERT_EQUAL_HEX32(0x094A, e);
  TEST_ASSERT_EQUAL_UINT8(0x4A, VESC_EID_CONTROLLER_ID(e));
  TEST_ASSERT_EQUAL_UINT32(9u, VESC_EID_PACKET_ID(e));

  TEST_ASSERT_EQUAL_INT(VESC_IDX_STATUS_1, vesc_status_index(9));
  TEST_ASSERT_EQUAL_INT(VESC_IDX_STATUS_2, vesc_status_index(14));
  TEST_ASSERT_EQUAL_INT(VESC_IDX_STATUS_3, vesc_status_index(15));
  TEST_ASSERT_EQUAL_INT(VESC_IDX_STATUS_4, vesc_status_index(16));
  TEST_ASSERT_EQUAL_INT(VESC_IDX_STATUS_5, vesc_status_index(27));
  TEST_ASSERT_EQUAL_INT(VESC_IDX_STATUS_6, vesc_status_index(58));
  TEST_ASSERT_EQUAL_INT(-1, vesc_status_index(0));
  TEST_ASSERT_EQUAL_INT(-1, vesc_status_index(10));   // between STATUS and STATUS_2
  TEST_ASSERT_EQUAL_INT(-1, vesc_status_index(17));   // CAN_PACKET_PING
  TEST_ASSERT_EQUAL_INT(-1, vesc_status_index(59));
  TEST_ASSERT_TRUE(vesc_is_status_pkt(27));
  TEST_ASSERT_FALSE(vesc_is_status_pkt(28));
}

// ---------------------------------------------------------------- STATUS_1 (pkt 9)
static void test_status1_plan_vector() {
  // erpm 1000 = 0x000003E8, current 10.0 A = 100 = 0x0064, duty 0.5 = 500 = 0x01F4
  const uint8_t d[8] = {0x00, 0x00, 0x03, 0xE8, 0x00, 0x64, 0x01, 0xF4};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS, 0x4A, d, 1234));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, t.erpm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, t.current_motor);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, t.duty);
  TEST_ASSERT_EQUAL_UINT8(0x4A, t.id);
  assert_only_timestamp(t, VESC_IDX_STATUS_1, 1234);
  // untouched fields stay zero
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, t.v_in);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, t.current_in);
}

static void test_status1_negative_values() {
  // erpm -50000 = 0xFFFF3CB0, current -12.3 A = -123 = 0xFF85, duty -0.75 = -750 = 0xFD12
  const uint8_t d[8] = {0xFF, 0xFF, 0x3C, 0xB0, 0xFF, 0x85, 0xFD, 0x12};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS, 0x01, d));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -50000.0f, t.erpm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -12.3f, t.current_motor);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.75f, t.duty);
  TEST_ASSERT_EQUAL_UINT8(0x01, t.id);
}

// ---------------------------------------------------------------- STATUS_2 (pkt 14)
static void test_status2_amp_hours() {
  // amp_hours 1.2345 Ah = 12345 = 0x00003039, amp_hours_charged -0.5 Ah = -5000 = 0xFFFFEC78
  const uint8_t d[8] = {0x00, 0x00, 0x30, 0x39, 0xFF, 0xFF, 0xEC, 0x78};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_2, 0x4A, d, 2000));
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, 1.2345f, t.amp_hours);
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, -0.5f, t.amp_hours_charged);
  assert_only_timestamp(t, VESC_IDX_STATUS_2, 2000);
}

// ---------------------------------------------------------------- STATUS_3 (pkt 15)
static void test_status3_watt_hours() {
  // watt_hours 100.0001 Wh = 1000001 = 0x000F4241, watt_hours_charged 2.5 Wh = 25000 = 0x000061A8
  const uint8_t d[8] = {0x00, 0x0F, 0x42, 0x41, 0x00, 0x00, 0x61, 0xA8};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_3, 0x4A, d, 3000));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0001f, t.watt_hours);
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, 2.5f, t.watt_hours_charged);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 97.5001f, vesc_wh_net(&t));
  assert_only_timestamp(t, VESC_IDX_STATUS_3, 3000);
}

// ---------------------------------------------------------------- STATUS_4 (pkt 16)
static void test_status4_plan_vector() {
  // temp_fet 40.0 = 400 = 0x0190, temp_motor 20.0 = 200 = 0x00C8, current_in 5.0 A = 50 = 0x0032, pid_pos 10.0 deg = 500 = 0x01F4
  const uint8_t d[8] = {0x01, 0x90, 0x00, 0xC8, 0x00, 0x32, 0x01, 0xF4};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_4, 0x4A, d, 4000));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 40.0f, t.temp_fet);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, t.temp_motor);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, t.current_in);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, t.pid_pos);
  assert_only_timestamp(t, VESC_IDX_STATUS_4, 4000);
}

static void test_status4_negative_regen_and_cold() {
  // temp_fet -5.5 = -55 = 0xFFC9, temp_motor -273.0 (no NTC) = -2730 = 0xF556, current_in -20.0 A (regen) = -200 = 0xFF38, pid_pos -0.02 = -1 = 0xFFFF
  const uint8_t d[8] = {0xFF, 0xC9, 0xF5, 0x56, 0xFF, 0x38, 0xFF, 0xFF};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_4, 0x4A, d));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -5.5f, t.temp_fet);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -273.0f, t.temp_motor);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -20.0f, t.current_in);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.02f, t.pid_pos);
}

// ---------------------------------------------------------------- STATUS_5 (pkt 27)
static void test_status5_wire_order_tacho_then_vin() {
  // tachometer 42 = 0x0000002A in bytes 0..3, v_in 50.0 V = 500 = 0x01F4 in bytes 4..5, 6..7 reserved.
  // A decoder following the VESC struct order (v_in first) would read v_in = 0.0 and tacho = 0x2A01F400.
  const uint8_t d[8] = {0x00, 0x00, 0x00, 0x2A, 0x01, 0xF4, 0x00, 0x00};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_5, 0x4A, d, 5000));
  TEST_ASSERT_EQUAL_INT32(42, t.tachometer);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, t.v_in);
  assert_only_timestamp(t, VESC_IDX_STATUS_5, 5000);
}

static void test_status5_negative_tacho_reserved_ignored() {
  // tachometer -100000 = 0xFFFE7960, v_in 3.3 V = 33 = 0x0021, reserved bytes set to junk
  const uint8_t d[8] = {0xFF, 0xFE, 0x79, 0x60, 0x00, 0x21, 0xDE, 0xAD};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_5, 0x4A, d));
  TEST_ASSERT_EQUAL_INT32(-100000, t.tachometer);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.3f, t.v_in);
}

// ---------------------------------------------------------------- STATUS_6 (pkt 58)
static void test_status6_adc_ppm() {
  // adc1 3.3 V = 3300 = 0x0CE4, adc2 1.65 V = 1650 = 0x0672, adc3 0 V, ppm -0.25 = -250 = 0xFF06
  const uint8_t d[8] = {0x0C, 0xE4, 0x06, 0x72, 0x00, 0x00, 0xFF, 0x06};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_6, 0x4A, d, 6000));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.3f, t.adc1);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.65f, t.adc2);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, t.adc3);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.25f, t.ppm);
  assert_only_timestamp(t, VESC_IDX_STATUS_6, 6000);
}

// ---------------------------------------------------------------- rejections
static void test_short_dlc_rejected() {
  const uint8_t d[8] = {0x00, 0x00, 0x03, 0xE8, 0x00, 0x64, 0x01, 0xF4};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_FALSE(feed(&t, VESC_CAN_PACKET_STATUS, 0x4A, d, 1000, -1, true, 7));
  TEST_ASSERT_FALSE(feed(&t, VESC_CAN_PACKET_STATUS, 0x4A, d, 1000, -1, true, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, t.erpm);
  assert_only_timestamp(t, -1, 0);  // no timestamp written
  TEST_ASSERT_EQUAL_UINT8(0, t.id);
}

static void test_standard_id_rejected() {
  // An 11-bit id that happens to look like (9 << 8) | 0x4A must not be decoded.
  const uint8_t d[8] = {0x00, 0x00, 0x03, 0xE8, 0x00, 0x64, 0x01, 0xF4};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_FALSE(feed(&t, VESC_CAN_PACKET_STATUS, 0x4A, d, 1000, -1, false));
  assert_only_timestamp(t, -1, 0);
}

static void test_unknown_packet_rejected() {
  const uint8_t d[8] = {0x00, 0x00, 0x03, 0xE8, 0x00, 0x64, 0x01, 0xF4};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_FALSE(feed(&t, 0, 0x4A, d));    // SET_DUTY
  TEST_ASSERT_FALSE(feed(&t, 10, 0x4A, d));   // between the status ids
  TEST_ASSERT_FALSE(feed(&t, 17, 0x4A, d));   // PING
  TEST_ASSERT_FALSE(feed(&t, 59, 0x4A, d));   // just past STATUS_6
  TEST_ASSERT_FALSE(feed(&t, 0x1FFFFF, 0x4A, d));  // largest 29-bit packet field
  assert_only_timestamp(t, -1, 0);
}

static void test_null_pointers_rejected() {
  const uint8_t d[8] = {0};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_FALSE(vesc_decode_status(&t, eid(9, 1), true, 0, 8, 1000, -1));
  TEST_ASSERT_FALSE(vesc_decode_status(0, eid(9, 1), true, d, 8, 1000, -1));
}

// ---------------------------------------------------------------- controller id filter
static void test_want_id_filtering() {
  const uint8_t d[8] = {0x00, 0x00, 0x03, 0xE8, 0x00, 0x64, 0x01, 0xF4};
  vesc_telemetry_t t = zeroed();
  // pinned to 0x10, frame from 0x4A -> rejected and untouched
  TEST_ASSERT_FALSE(feed(&t, VESC_CAN_PACKET_STATUS, 0x4A, d, 1000, 0x10));
  assert_only_timestamp(t, -1, 0);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, t.erpm);
  // pinned to 0x4A, frame from 0x4A -> accepted
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS, 0x4A, d, 1000, 0x4A));
  TEST_ASSERT_EQUAL_UINT8(0x4A, t.id);
  // -1 accepts any id, including 0 and 254
  vesc_telemetry_t u = zeroed();
  TEST_ASSERT_TRUE(feed(&u, VESC_CAN_PACKET_STATUS, 0x00, d, 1000, -1));
  TEST_ASSERT_EQUAL_UINT8(0x00, u.id);
  TEST_ASSERT_TRUE(feed(&u, VESC_CAN_PACKET_STATUS, 0xFE, d, 1000, -1));
  TEST_ASSERT_EQUAL_UINT8(0xFE, u.id);
  // want_id 0 pins id 0 (not "any")
  vesc_telemetry_t w = zeroed();
  TEST_ASSERT_TRUE(feed(&w, VESC_CAN_PACKET_STATUS, 0x00, d, 1000, 0));
  TEST_ASSERT_FALSE(feed(&w, VESC_CAN_PACKET_STATUS, 0x01, d, 1000, 0));
}

// ---------------------------------------------------------------- timestamps / freshness
static void test_now_zero_is_stored_as_one() {
  const uint8_t d[8] = {0x00, 0x00, 0x00, 0x2A, 0x01, 0xF4, 0x00, 0x00};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_5, 0x4A, d, 0));
  TEST_ASSERT_EQUAL_UINT32(1u, t.t_ms[VESC_IDX_STATUS_5]);
  // Stored as 1 it counts as received: 49 ms later it is fresh. Had it been stored
  // as 0 the same call would return false ("never"), which is what the next line pins.
  TEST_ASSERT_TRUE(vesc_fresh(&t, VESC_IDX_STATUS_5, 50, 100));
  t.t_ms[VESC_IDX_STATUS_5] = 0;
  TEST_ASSERT_FALSE(vesc_fresh(&t, VESC_IDX_STATUS_5, 50, 100));
}

static void test_fresh_wraparound_and_never() {
  vesc_telemetry_t t = zeroed();
  t.t_ms[VESC_IDX_STATUS_1] = 0xFFFFFFF0u;
  // now wrapped past zero: age = 5 - 0xFFFFFFF0 = 21 ms (unsigned) -> fresh
  TEST_ASSERT_TRUE(vesc_fresh(&t, VESC_IDX_STATUS_1, 5, 100));
  TEST_ASSERT_FALSE(vesc_fresh(&t, VESC_IDX_STATUS_1, 5, 20));   // 21 > 20
  TEST_ASSERT_TRUE(vesc_fresh(&t, VESC_IDX_STATUS_1, 5, 21));    // boundary inclusive
  // never received -> never fresh, even with a huge max_age
  TEST_ASSERT_FALSE(vesc_fresh(&t, VESC_IDX_STATUS_2, 5, 0xFFFFFFFFu));
  // exact boundary without wrap
  t.t_ms[VESC_IDX_STATUS_4] = 1000;
  TEST_ASSERT_TRUE(vesc_fresh(&t, VESC_IDX_STATUS_4, 3000, 2000));
  TEST_ASSERT_FALSE(vesc_fresh(&t, VESC_IDX_STATUS_4, 3001, 2000));
  // out-of-range index
  TEST_ASSERT_FALSE(vesc_fresh(&t, -1, 3000, 2000));
  TEST_ASSERT_FALSE(vesc_fresh(&t, VESC_STATUS_COUNT, 3000, 2000));
}

// ---------------------------------------------------------------- derived values
static void test_derived_values() {
  vesc_telemetry_t t = zeroed();
  t.v_in = 50.0f;
  t.current_in = 5.0f;
  t.erpm = 1000.0f;
  t.tachometer = 42;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 250.0f, vesc_power_w(&t));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 142.857f, vesc_mech_rpm(&t, 14));  // 1000 / 7 pole pairs
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, vesc_revolutions(&t, 14)); // 42 / (3 * 14)
  // regen: negative battery current gives negative power
  t.current_in = -5.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -250.0f, vesc_power_w(&t));
  // degenerate pole counts do not divide by zero
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, vesc_mech_rpm(&t, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, vesc_revolutions(&t, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, vesc_mech_rpm(&t, -14));
}

// ---------------------------------------------------------------- accumulation
static void test_frames_accumulate_into_one_struct() {
  const uint8_t s1[8] = {0x00, 0x00, 0x03, 0xE8, 0x00, 0x64, 0x01, 0xF4};
  const uint8_t s4[8] = {0x01, 0x90, 0x00, 0xC8, 0x00, 0x32, 0x01, 0xF4};
  const uint8_t s5[8] = {0x00, 0x00, 0x00, 0x2A, 0x01, 0xF4, 0x00, 0x00};
  vesc_telemetry_t t = zeroed();
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS, 0x4A, s1, 100));
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_4, 0x4A, s4, 110));
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS_5, 0x4A, s5, 120));
  // each message only touches its own fields
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, t.erpm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, t.current_in);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, t.v_in);
  TEST_ASSERT_EQUAL_INT32(42, t.tachometer);
  TEST_ASSERT_EQUAL_UINT32(100u, t.t_ms[VESC_IDX_STATUS_1]);
  TEST_ASSERT_EQUAL_UINT32(110u, t.t_ms[VESC_IDX_STATUS_4]);
  TEST_ASSERT_EQUAL_UINT32(120u, t.t_ms[VESC_IDX_STATUS_5]);
  TEST_ASSERT_EQUAL_UINT32(0u, t.t_ms[VESC_IDX_STATUS_2]);
  TEST_ASSERT_EQUAL_UINT32(0u, t.t_ms[VESC_IDX_STATUS_3]);
  TEST_ASSERT_EQUAL_UINT32(0u, t.t_ms[VESC_IDX_STATUS_6]);
  // the plan's bench scenario: 50.0 V * 5.0 A = 250 W
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 250.0f, vesc_power_w(&t));
  // a later frame overwrites, the timestamp moves forward
  const uint8_t s1b[8] = {0x00, 0x00, 0x07, 0xD0, 0xFF, 0x9C, 0x00, 0x00};  // erpm 2000, -10.0 A, duty 0
  TEST_ASSERT_TRUE(feed(&t, VESC_CAN_PACKET_STATUS, 0x4A, s1b, 130));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2000.0f, t.erpm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.0f, t.current_motor);
  TEST_ASSERT_EQUAL_UINT32(130u, t.t_ms[VESC_IDX_STATUS_1]);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_be_helpers);
  RUN_TEST(test_eid_macros_and_index);
  RUN_TEST(test_status1_plan_vector);
  RUN_TEST(test_status1_negative_values);
  RUN_TEST(test_status2_amp_hours);
  RUN_TEST(test_status3_watt_hours);
  RUN_TEST(test_status4_plan_vector);
  RUN_TEST(test_status4_negative_regen_and_cold);
  RUN_TEST(test_status5_wire_order_tacho_then_vin);
  RUN_TEST(test_status5_negative_tacho_reserved_ignored);
  RUN_TEST(test_status6_adc_ppm);
  RUN_TEST(test_short_dlc_rejected);
  RUN_TEST(test_standard_id_rejected);
  RUN_TEST(test_unknown_packet_rejected);
  RUN_TEST(test_null_pointers_rejected);
  RUN_TEST(test_want_id_filtering);
  RUN_TEST(test_now_zero_is_stored_as_one);
  RUN_TEST(test_fresh_wraparound_and_never);
  RUN_TEST(test_derived_values);
  RUN_TEST(test_frames_accumulate_into_one_struct);
  return UNITY_END();
}
