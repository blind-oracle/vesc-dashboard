// Unity tests (pio test -e native) for include/vesc_mcconf.h: the temperature limits out of a
// COMM_GET_MCCONF reply for every firmware layout in the table (byte strings laid out as
// confgenerator_serialize_mcconf() of each release writes them), the rejection of unknown,
// short and implausible replies, the derating point, and the read scheduler both VESC links share.
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; includes only framework-free headers.
#include <unity.h>

#include <stdint.h>
#include <string.h>

#include "vesc_mcconf.h"

void setUp() {}
void tearDown() {}

static const uint32_t SIG_503 = 0x968A9B08u;  // 2525666056
static const uint32_t SIG_602 = 0x2E43A161u;  // 776184161 (6.00 too)
static const uint32_t SIG_605 = 0x3F829CF7u;  // 1065524471
static const uint32_t SIG_606 = 0x2EFD0142u;  // 788332866
static const uint32_t SIG_700 = 0x57AD8F53u;  // 1470992211

static const size_t kReplyLen = 480;  // about what 6.05 / 6.06 send
static uint8_t g_buf[kReplyLen];

static void put_be16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)((uint16_t)v >> 8);
  p[1] = (uint8_t)v;
}

// A whole reply: [14, signature BE, filler ...] with the limits written in the given layout.
// The filler is 0xA5 so that a parser reading the wrong offset sees garbage, not zeros.
static const uint8_t *reply(uint32_t sig, bool f16, float fet_s, float fet_e, float mot_s, float mot_e, float dec) {
  memset(g_buf, 0xA5, sizeof g_buf);
  g_buf[0] = VESC_COMM_GET_MCCONF;
  g_buf[1] = (uint8_t)(sig >> 24);
  g_buf[2] = (uint8_t)(sig >> 16);
  g_buf[3] = (uint8_t)(sig >> 8);
  g_buf[4] = (uint8_t)sig;
  uint8_t *t = g_buf + 1 + 63;  // serialized offset 63, after the command byte
  if (f16) {
    put_be16(t + 0, (int16_t)(fet_s * 10.0f));
    put_be16(t + 2, (int16_t)(fet_e * 10.0f));
    put_be16(t + 4, (int16_t)(mot_s * 10.0f));
    put_be16(t + 6, (int16_t)(mot_e * 10.0f));
    put_be16(t + 8, (int16_t)(dec * 10000.0f));
  } else {
    t[0] = (uint8_t)fet_s;
    t[1] = (uint8_t)fet_e;
    t[2] = (uint8_t)mot_s;
    t[3] = (uint8_t)mot_e;
    put_be16(t + 4, (int16_t)(dec * 10000.0f));
  }
  return g_buf;
}

// ---------------------------------------------------------------- parse
static void test_parse_u8_layouts() {
  const uint32_t sigs[3] = {SIG_605, SIG_606, SIG_700};
  const char *const fws[3] = {"6.05", "6.06", "7.00"};
  for (int i = 0; i < 3; ++i) {
    const VescMcconfRead r = vesc_mcconf_parse(reply(sigs[i], false, 85, 100, 90, 110, 0.15f), kReplyLen);
    TEST_ASSERT_EQUAL_UINT8(MCCONF_OK, r.result);
    TEST_ASSERT_EQUAL_HEX32(sigs[i], r.sig);
    TEST_ASSERT_EQUAL_STRING(fws[i], r.fw);
    TEST_ASSERT_EQUAL_FLOAT(85.0f, r.lim.fet_start);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.lim.fet_end);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, r.lim.mot_start);
    TEST_ASSERT_EQUAL_FLOAT(110.0f, r.lim.mot_end);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.15f, r.lim.accel_dec);
  }
}

static void test_parse_f16_layouts() {
  const VescMcconfRead r = vesc_mcconf_parse(reply(SIG_602, true, 85.5f, 100.0f, 80.0f, 95.5f, 0.25f), kReplyLen);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_OK, r.result);
  TEST_ASSERT_EQUAL_STRING("6.00/02", r.fw);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 85.5f, r.lim.fet_start);  // float16 x10 keeps the tenth
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, r.lim.fet_end);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 80.0f, r.lim.mot_start);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 95.5f, r.lim.mot_end);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.25f, r.lim.accel_dec);
  const VescMcconfRead r5 = vesc_mcconf_parse(reply(SIG_503, true, 80, 95, 80, 95, 0.0f), kReplyLen);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_OK, r5.result);
  TEST_ASSERT_EQUAL_STRING("5.03", r5.fw);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r5.lim.accel_dec);
}

// The same bytes read with the other encoding are garbage, which the range check catches:
// a table entry pointing at the wrong encoding could not slip through as plausible.
static void test_wrong_encoding_is_implausible() {
  // a float16 x10 layout (85.0 -> 0x0352) parsed as uint8 reads fet 3 / 82
  reply(SIG_602, true, 85, 100, 85, 100, 0.15f);
  g_buf[1] = (uint8_t)(SIG_606 >> 24);
  g_buf[2] = (uint8_t)(SIG_606 >> 16);
  g_buf[3] = (uint8_t)(SIG_606 >> 8);
  g_buf[4] = (uint8_t)SIG_606;
  TEST_ASSERT_EQUAL_UINT8(MCCONF_IMPLAUSIBLE, vesc_mcconf_parse(g_buf, kReplyLen).result);
  // and a uint8 layout parsed as float16 reads the bytes 85, 100 as 0x5564 = 2186 C
  reply(SIG_606, false, 85, 100, 85, 100, 0.15f);
  g_buf[1] = (uint8_t)(SIG_602 >> 24);
  g_buf[2] = (uint8_t)(SIG_602 >> 16);
  g_buf[3] = (uint8_t)(SIG_602 >> 8);
  g_buf[4] = (uint8_t)SIG_602;
  TEST_ASSERT_EQUAL_UINT8(MCCONF_IMPLAUSIBLE, vesc_mcconf_parse(g_buf, kReplyLen).result);
}

static void test_unknown_signature() {
  const VescMcconfRead r = vesc_mcconf_parse(reply(0x1C17AFEEu, false, 85, 100, 85, 100, 0.15f), kReplyLen);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_UNKNOWN_SIG, r.result);
  TEST_ASSERT_EQUAL_HEX32(0x1C17AFEEu, r.sig);  // kept for the log
  TEST_ASSERT_EQUAL_STRING("", r.fw);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.lim.mot_start);  // nothing guessed
}

static void test_not_mcconf_and_short() {
  reply(SIG_606, false, 85, 100, 85, 100, 0.15f);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_NOT_MCCONF, vesc_mcconf_parse(nullptr, 10).result);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_NOT_MCCONF, vesc_mcconf_parse(g_buf, 0).result);
  g_buf[0] = 4;  // COMM_GET_VALUES
  TEST_ASSERT_EQUAL_UINT8(MCCONF_NOT_MCCONF, vesc_mcconf_parse(g_buf, kReplyLen).result);
  g_buf[0] = VESC_COMM_GET_MCCONF;
  TEST_ASSERT_EQUAL_UINT8(MCCONF_SHORT, vesc_mcconf_parse(g_buf, 4).result);  // no whole signature
  // uint8 layout: needs the command byte + 63 + 4 limits + 2 accel_dec = 70 bytes
  TEST_ASSERT_EQUAL_UINT8(MCCONF_SHORT, vesc_mcconf_parse(g_buf, 69).result);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_OK, vesc_mcconf_parse(g_buf, 70).result);
  // float16 layout: 1 + 63 + 8 + 2 = 74
  reply(SIG_602, true, 85, 100, 85, 100, 0.15f);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_SHORT, vesc_mcconf_parse(g_buf, 73).result);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_OK, vesc_mcconf_parse(g_buf, 74).result);
}

static void test_implausible_limits() {
  TEST_ASSERT_EQUAL_UINT8(MCCONF_IMPLAUSIBLE, vesc_mcconf_parse(reply(SIG_606, false, 100, 85, 85, 100, 0.15f), kReplyLen).result);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_IMPLAUSIBLE, vesc_mcconf_parse(reply(SIG_606, false, 85, 85, 85, 100, 0.15f), kReplyLen).result);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_IMPLAUSIBLE, vesc_mcconf_parse(reply(SIG_606, false, 85, 100, 10, 100, 0.15f), kReplyLen).result);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_IMPLAUSIBLE, vesc_mcconf_parse(reply(SIG_606, false, 85, 100, 85, 100, 1.5f), kReplyLen).result);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_IMPLAUSIBLE, vesc_mcconf_parse(reply(SIG_602, true, 85, 100, 85, 100, -0.1f), kReplyLen).result);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_IMPLAUSIBLE, vesc_mcconf_parse(reply(SIG_602, true, -20, 100, 85, 100, 0.1f), kReplyLen).result);
  // the edges are fine: limits switched off by setting them very high, no early acceleration derating
  TEST_ASSERT_EQUAL_UINT8(MCCONF_OK, vesc_mcconf_parse(reply(SIG_606, false, 20, 250, 149, 150, 0.0f), kReplyLen).result);
  TEST_ASSERT_EQUAL_UINT8(MCCONF_OK, vesc_mcconf_parse(reply(SIG_606, false, 85, 100, 85, 100, 1.0f), kReplyLen).result);
}

static void test_derate_start() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 76.0f, vesc_derate_start(85.0f, 0.15f));  // the VESC defaults
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, vesc_derate_start(100.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, vesc_derate_start(85.0f, 1.0f));
}

static void test_result_names() {
  TEST_ASSERT_EQUAL_STRING("ok", vesc_mcconf_result_str(MCCONF_OK));
  TEST_ASSERT_EQUAL_STRING("unknown signature", vesc_mcconf_result_str(MCCONF_UNKNOWN_SIG));
  TEST_ASSERT_EQUAL_STRING("?", vesc_mcconf_result_str(200));
}

// ---------------------------------------------------------------- scheduler
static void test_sched_reads_at_link_up_only() {
  VescMcconfSched s;
  mcconf_sched_init(s);
  TEST_ASSERT_FALSE(mcconf_sched_due(s, false, 1000));  // no VESC: nothing to ask
  TEST_ASSERT_FALSE(mcconf_sched_due(s, false, 50000));
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 60000));  // first answered poll: read now
  mcconf_sched_on_sent(s);
  TEST_ASSERT_FALSE(mcconf_sched_due(s, true, 60100));  // in flight: one at a time
}

static void test_sched_refresh_after_an_answer() {
  VescMcconfSched s;
  mcconf_sched_init(s);
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 1000));
  mcconf_sched_on_sent(s);
  mcconf_sched_on_result(s, MCCONF_OK, 1050);
  TEST_ASSERT_FALSE(mcconf_sched_due(s, true, 1050 + VESC_MCCONF_REFRESH_MS - 1));
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 1050 + VESC_MCCONF_REFRESH_MS));
  // an unknown firmware is an answer too: asking again sooner would not change it
  mcconf_sched_on_sent(s);
  mcconf_sched_on_result(s, MCCONF_UNKNOWN_SIG, 5000000);
  TEST_ASSERT_FALSE(mcconf_sched_due(s, true, 5000000 + VESC_MCCONF_RETRY_MS));
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 5000000 + VESC_MCCONF_REFRESH_MS));
}

static void test_sched_retry_after_failure() {
  VescMcconfSched s;
  mcconf_sched_init(s);
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 1000));
  mcconf_sched_on_sent(s);
  mcconf_sched_on_timeout(s, 1300);
  TEST_ASSERT_FALSE(mcconf_sched_due(s, true, 1300 + VESC_MCCONF_RETRY_MS - 1));
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 1300 + VESC_MCCONF_RETRY_MS));
  // a broken reply and a lock timeout (MCCONF_NONE: nothing stored) retry the same way
  mcconf_sched_on_sent(s);
  mcconf_sched_on_result(s, MCCONF_SHORT, 20000);
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 20000 + VESC_MCCONF_RETRY_MS));
  mcconf_sched_on_sent(s);
  mcconf_sched_on_result(s, MCCONF_NONE, 40000);
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 40000 + VESC_MCCONF_RETRY_MS));
}

static void test_sched_rereads_after_a_lost_link() {
  VescMcconfSched s;
  mcconf_sched_init(s);
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 1000));
  mcconf_sched_on_sent(s);
  mcconf_sched_on_result(s, MCCONF_OK, 1050);
  TEST_ASSERT_FALSE(mcconf_sched_due(s, true, 2000));
  TEST_ASSERT_FALSE(mcconf_sched_due(s, false, 3000));  // VESC gone (maybe reconfigured meanwhile)
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, 4000));    // back: read at once, refresh or not
}

static void test_sched_wraps() {
  VescMcconfSched s;
  mcconf_sched_init(s);
  const uint32_t t0 = 0xFFFFFF00u;
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, t0));
  mcconf_sched_on_sent(s);
  mcconf_sched_on_timeout(s, t0);
  TEST_ASSERT_FALSE(mcconf_sched_due(s, true, t0 + 100u));  // past the wrap, not yet due
  TEST_ASSERT_TRUE(mcconf_sched_due(s, true, t0 + VESC_MCCONF_RETRY_MS));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_parse_u8_layouts);
  RUN_TEST(test_parse_f16_layouts);
  RUN_TEST(test_wrong_encoding_is_implausible);
  RUN_TEST(test_unknown_signature);
  RUN_TEST(test_not_mcconf_and_short);
  RUN_TEST(test_implausible_limits);
  RUN_TEST(test_derate_start);
  RUN_TEST(test_result_names);
  RUN_TEST(test_sched_reads_at_link_up_only);
  RUN_TEST(test_sched_refresh_after_an_answer);
  RUN_TEST(test_sched_retry_after_failure);
  RUN_TEST(test_sched_rereads_after_a_lost_link);
  RUN_TEST(test_sched_wraps);
  return UNITY_END();
}
