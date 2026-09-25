// Native Unity tests for include/ble_addr.h: the printed-vs-NimBLE byte order (the bug
// this header exists to stop repeating) and every way a hand-typed MAC can be wrong.
#include <unity.h>

#include <string.h>

#include "ble_addr.h"

void setUp() {}
void tearDown() {}

static void test_parse_keeps_nimble_order() {
  uint8_t v[6];
  TEST_ASSERT_TRUE(ble_addr_parse("C8:47:8C:12:34:56", v));
  TEST_ASSERT_EQUAL_HEX8(0x56, v[0]);  // val[0] is the LAST printed octet
  TEST_ASSERT_EQUAL_HEX8(0x34, v[1]);
  TEST_ASSERT_EQUAL_HEX8(0x12, v[2]);
  TEST_ASSERT_EQUAL_HEX8(0x8c, v[3]);
  TEST_ASSERT_EQUAL_HEX8(0x47, v[4]);
  TEST_ASSERT_EQUAL_HEX8(0xc8, v[5]);
}

static void test_round_trip() {
  uint8_t v[6];
  char s[18];
  TEST_ASSERT_TRUE(ble_addr_parse("e1:23:45:67:89:ab", v));
  ble_addr_str(v, s);
  TEST_ASSERT_EQUAL_STRING("e1:23:45:67:89:ab", s);
  // upper case in, lower case out, same bytes
  uint8_t u[6];
  TEST_ASSERT_TRUE(ble_addr_parse("E1:23:45:67:89:AB", u));
  TEST_ASSERT_TRUE(ble_addr_equal(u, v));
}

static void test_dash_separators() {
  uint8_t a[6], b[6];
  TEST_ASSERT_TRUE(ble_addr_parse("00-11-22-33-44-55", a));
  TEST_ASSERT_TRUE(ble_addr_parse("00:11:22:33:44:55", b));
  TEST_ASSERT_TRUE(ble_addr_equal(a, b));
}

static void test_rejects_bad_input() {
  uint8_t v[6];
  TEST_ASSERT_FALSE(ble_addr_parse(nullptr, v));
  TEST_ASSERT_FALSE(ble_addr_parse("", v));                     // the "no tag configured" default
  TEST_ASSERT_FALSE(ble_addr_parse("C8:47:8C:12:34", v));        // too short
  TEST_ASSERT_FALSE(ble_addr_parse("C8:47:8C:12:34:56:78", v));  // too long
  TEST_ASSERT_FALSE(ble_addr_parse("C8:47:8C:12:34:5G", v));     // not hex
  TEST_ASSERT_FALSE(ble_addr_parse("C8/47/8C/12/34/56", v));     // wrong separator
  TEST_ASSERT_FALSE(ble_addr_parse("C847.8C123456", v));
  TEST_ASSERT_FALSE(ble_addr_parse("                 ", v));     // 17 chars of nothing
}

static void test_failure_zeroes_the_output() {
  uint8_t v[6];
  TEST_ASSERT_TRUE(ble_addr_parse("ff:ff:ff:ff:ff:ff", v));
  TEST_ASSERT_FALSE(ble_addr_parse("ff:ff:ff:ff:ff:fg", v));  // a half-parsed address must not survive
  for (int i = 0; i < 6; ++i) TEST_ASSERT_EQUAL_HEX8(0, v[i]);
}

static void test_extremes() {
  uint8_t v[6];
  char s[18];
  TEST_ASSERT_TRUE(ble_addr_parse("00:00:00:00:00:00", v));
  ble_addr_str(v, s);
  TEST_ASSERT_EQUAL_STRING("00:00:00:00:00:00", s);
  TEST_ASSERT_TRUE(ble_addr_parse("ff:ff:ff:ff:ff:ff", v));
  ble_addr_str(v, s);
  TEST_ASSERT_EQUAL_STRING("ff:ff:ff:ff:ff:ff", s);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_parse_keeps_nimble_order);
  RUN_TEST(test_round_trip);
  RUN_TEST(test_dash_separators);
  RUN_TEST(test_rejects_bad_input);
  RUN_TEST(test_failure_zeroes_the_output);
  RUN_TEST(test_extremes);
  return UNITY_END();
}
