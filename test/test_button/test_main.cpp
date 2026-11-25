// Native Unity tests (pio test -e native) for the push-button gesture decoder in
// include/button_logic.h: debounce, Short after the double-click window, Double, a slow second
// press, a click followed by a hold, the instant Short with the window off, and the long /
// very long holds. The button is sampled every 20 ms, as the display task does.
// Compiles with: -std=c++17 -DUNIT_TEST -Iinclude ; includes only framework-free headers.
#include <unity.h>

#include <stdint.h>

#include "button_logic.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- harness
static const uint32_t POLL = 20;  // OLED_BUTTON_POLL_MS
static const uint32_t T0 = 1000;

struct Sim {
  Button b;
  uint32_t now = T0;
  unsigned shorts = 0, doubles = 0, longs = 0, very_longs = 0;
  uint32_t last_short_at = 0, last_double_at = 0, last_long_at = 0;

  explicit Sim(uint32_t window = (uint32_t)BUTTON_DOUBLE_CLICK_MS) { b.double_ms = window; }

  // Holds the raw level for `ms`, sampling every POLL.
  void level(bool pressed, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += POLL) {
      switch (b.update(pressed, now)) {
        case ButtonEvent::Short: shorts++; last_short_at = now; break;
        case ButtonEvent::Double: doubles++; last_double_at = now; break;
        case ButtonEvent::Long: longs++; last_long_at = now; break;
        case ButtonEvent::VeryLong: very_longs++; break;
        default: break;
      }
      now += POLL;
    }
  }
  void press(uint32_t ms) { level(true, ms); }
  void release(uint32_t ms) { level(false, ms); }
  unsigned events() const { return shorts + doubles + longs + very_longs; }
};

static const uint32_t WINDOW = (uint32_t)BUTTON_DOUBLE_CLICK_MS;

// ---------------------------------------------------------------- tests
static void test_config_sanity() {
  TEST_ASSERT_TRUE(WINDOW > (uint32_t)BUTTON_DEBOUNCE_MS);
  TEST_ASSERT_TRUE(WINDOW < (uint32_t)BUTTON_LONG_PRESS_MS);
}

static void test_single_click_is_a_short_after_the_window() {
  Sim s;
  s.release(100);
  s.press(100);
  const uint32_t released_at = s.now;
  s.release(WINDOW - 100);  // still inside the window (debounce + part of it): nothing yet
  TEST_ASSERT_EQUAL_UINT(0, s.events());
  s.release(400);
  TEST_ASSERT_EQUAL_UINT(1, s.shorts);
  TEST_ASSERT_EQUAL_UINT(1, s.events());
  // committed one window after the debounced release (+ at most one poll)
  const uint32_t debounced_release = released_at + (uint32_t)BUTTON_DEBOUNCE_MS;
  TEST_ASSERT_TRUE(s.last_short_at >= debounced_release + WINDOW);
  TEST_ASSERT_TRUE(s.last_short_at <= debounced_release + WINDOW + 2 * POLL);
  TEST_ASSERT_EQUAL_UINT32(1, s.b.presses);
}

static void test_double_click_is_a_double_and_no_short() {
  Sim s;
  s.release(100);
  s.press(100);
  s.release(120);
  s.press(100);
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(1, s.doubles);
  TEST_ASSERT_EQUAL_UINT(0, s.shorts);
  TEST_ASSERT_EQUAL_UINT(1, s.events());
  TEST_ASSERT_EQUAL_UINT32(2, s.b.presses);
}

static void test_second_press_debouncing_at_the_window_edge_still_doubles() {
  // The second raw press arrives just before the window ends; its debounce runs past the end.
  // The window must not close under a press that is already on its way.
  Sim s;
  s.release(100);
  s.press(100);
  // The debounced release lands on the first poll >= BUTTON_DEBOUNCE_MS (40 ms at a 20 ms poll), so the
  // raw press below arrives one poll before the window closes and is debounced only after it has.
  s.release((uint32_t)BUTTON_DEBOUNCE_MS + WINDOW);
  TEST_ASSERT_EQUAL_UINT(0, s.events());
  s.press(100);
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(1, s.doubles);
  TEST_ASSERT_EQUAL_UINT(0, s.shorts);
}

static void test_slow_second_press_is_two_shorts() {
  Sim s;
  s.release(100);
  s.press(100);
  s.release(WINDOW + 200);
  TEST_ASSERT_EQUAL_UINT(1, s.shorts);
  s.press(100);
  s.release(WINDOW + 200);
  TEST_ASSERT_EQUAL_UINT(2, s.shorts);
  TEST_ASSERT_EQUAL_UINT(0, s.doubles);
}

static void test_triple_click_is_a_double_then_a_short() {
  Sim s;
  s.release(100);
  for (int i = 0; i < 3; ++i) {
    s.press(80);
    s.release(100);
  }
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(1, s.doubles);
  TEST_ASSERT_EQUAL_UINT(1, s.shorts);
  TEST_ASSERT_TRUE(s.last_double_at < s.last_short_at);
}

static void test_click_then_hold_is_a_long_only() {
  Sim s;
  s.release(100);
  s.press(100);
  s.release(120);
  s.press((uint32_t)BUTTON_LONG_PRESS_MS + 200);  // second press held into a long press
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(1, s.longs);
  TEST_ASSERT_EQUAL_UINT(0, s.shorts);
  TEST_ASSERT_EQUAL_UINT(0, s.doubles);
}

static void test_long_press_fires_while_held_and_release_is_silent() {
  Sim s;
  s.release(100);
  const uint32_t start = s.now;
  s.press((uint32_t)BUTTON_LONG_PRESS_MS + 300);
  TEST_ASSERT_EQUAL_UINT(1, s.longs);
  TEST_ASSERT_TRUE(s.last_long_at >= start + (uint32_t)BUTTON_LONG_PRESS_MS);
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(1, s.events());
}

static void test_very_long_hold() {
  Sim s;
  s.release(100);
  s.press((uint32_t)OTA_LONG_PRESS_MS + 300);
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(1, s.longs);
#if OTA_ENABLE
  TEST_ASSERT_EQUAL_UINT(1, s.very_longs);
  TEST_ASSERT_EQUAL_UINT(2, s.events());
#else
  TEST_ASSERT_EQUAL_UINT(0, s.very_longs);
  TEST_ASSERT_EQUAL_UINT(1, s.events());
#endif
}

static void test_bounce_is_rejected() {
  Sim s;
  s.release(100);
  // a 20 ms blip (shorter than the debounce) is no press at all
  s.press(POLL);
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(0, s.events());
  TEST_ASSERT_EQUAL_UINT32(0, s.b.presses);
  // a press with contact bounce on its leading and trailing edges is ONE click
  s.press(POLL);
  s.release(POLL);
  s.press(120);
  s.release(POLL);
  s.press(POLL);
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(1, s.shorts);
  TEST_ASSERT_EQUAL_UINT(0, s.doubles);
  TEST_ASSERT_EQUAL_UINT32(1, s.b.presses);
}

static void test_window_off_gives_an_instant_short() {
  Sim s(0);
  s.release(100);
  s.press(100);
  const uint32_t released_at = s.now;
  s.release(100);
  TEST_ASSERT_EQUAL_UINT(1, s.shorts);
  TEST_ASSERT_TRUE(s.last_short_at <= released_at + (uint32_t)BUTTON_DEBOUNCE_MS + POLL);
  // two quick presses are two Shorts, never a Double
  s.press(100);
  s.release(100);
  s.press(100);
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(3, s.shorts);
  TEST_ASSERT_EQUAL_UINT(0, s.doubles);
}

static void test_clock_wrap() {
  Sim s;
  s.now = 0xFFFFFF00u;  // the window spans the 32-bit wrap
  s.release(100);
  s.press(100);
  s.release(120);
  s.press(100);
  s.release(1000);
  TEST_ASSERT_EQUAL_UINT(1, s.doubles);
  TEST_ASSERT_EQUAL_UINT(0, s.shorts);
  s.now = 0xFFFFFF80u;
  s.press(100);
  s.release(1000);  // a single click whose window ends after the wrap
  TEST_ASSERT_EQUAL_UINT(1, s.shorts);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_config_sanity);
  RUN_TEST(test_single_click_is_a_short_after_the_window);
  RUN_TEST(test_double_click_is_a_double_and_no_short);
  RUN_TEST(test_second_press_debouncing_at_the_window_edge_still_doubles);
  RUN_TEST(test_slow_second_press_is_two_shorts);
  RUN_TEST(test_triple_click_is_a_double_then_a_short);
  RUN_TEST(test_click_then_hold_is_a_long_only);
  RUN_TEST(test_long_press_fires_while_held_and_release_is_silent);
  RUN_TEST(test_very_long_hold);
  RUN_TEST(test_bounce_is_rejected);
  RUN_TEST(test_window_off_gives_an_instant_short);
  RUN_TEST(test_clock_wrap);
  return UNITY_END();
}
