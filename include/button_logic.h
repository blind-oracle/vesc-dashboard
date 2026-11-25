// Push-button gesture decoder (pure part): debounce, short press, double click, long press and
// the very long (update-mode) hold, from raw level samples and a millisecond clock.
//
// The display task (src/display_oled.cpp) samples the GPIO every OLED_BUTTON_POLL_MS and feeds
// it here; it maps the events to screens: Short = next, Double = previous, Long = MAIN (or the
// overlay's own action), VeryLong = update mode.
//
//   * Debounce: the raw level must be stable for BUTTON_DEBOUNCE_MS before it is accepted
//     (time based, so the poll jitter of a frame push is harmless); the debounced level is
//     then edge-detected.
//   * Long fires ONCE while the button is still held (at BUTTON_LONG_PRESS_MS) and the release
//     after it produces nothing; VeryLong fires once more if the same hold reaches
//     OTA_LONG_PRESS_MS (OTA_ENABLE builds only).
//   * Short / Double (BUTTON_DOUBLE_CLICK_MS > 0): the release of a short press opens a window
//     of BUTTON_DOUBLE_CLICK_MS. A second short press released after a press that began inside
//     it is a Double (and no Short). If the window passes with the button up, the first press
//     becomes a Short - so a single press acts BUTTON_DOUBLE_CLICK_MS after its release. A
//     second press that is held into a Long drops the pending click (the Long wins). While the
//     raw level already reads pressed (a second press still debouncing) the window is not
//     closed, so the debounce time never splits a fast double click into two Shorts.
//   * BUTTON_DOUBLE_CLICK_MS 0: no Double; a short press is a Short at its release, at once.
//
// All stamps come from one millisecond clock (state_now_ms()); the differences are unsigned,
// so a clock wrap is harmless. At most one event per update() call.
//
// Framework-free (only config.h) so the native Unity tests (test/test_button) can drive it.
#pragma once

#include <stdint.h>

#include "config.h"

#ifndef BUTTON_DOUBLE_CLICK_MS
#define BUTTON_DOUBLE_CLICK_MS 0  // an older config.h: no double click
#endif

enum class ButtonEvent : uint8_t { None, Short, Double, Long, VeryLong };

struct Button {
  bool raw_last = false;       // last raw sample (true = pressed)
  uint32_t raw_since = 0;      // time of the last raw change
  bool pressed = false;        // debounced level
  uint32_t pressed_at = 0;     // debounced press time
  bool long_fired = false;
  bool very_long_fired = false;
  bool click_pending = false;  // a short press was released and may still become a Double
  uint32_t click_ms = 0;       // its release time (valid with click_pending)
  uint32_t presses = 0;        // debounced presses (-> DisplayStats.button_presses)
  uint32_t double_ms = (uint32_t)BUTTON_DOUBLE_CLICK_MS;  // the window (0 = no Double); a member so tests can vary it

  ButtonEvent update(bool raw_pressed, uint32_t now) {
    ButtonEvent ev = ButtonEvent::None;
    if (raw_pressed != raw_last) {
      raw_last = raw_pressed;
      raw_since = now;
    } else if (raw_pressed != pressed && (uint32_t)(now - raw_since) >= (uint32_t)BUTTON_DEBOUNCE_MS) {
      pressed = raw_pressed;  // debounced edge
      if (pressed) {
        pressed_at = now;
        long_fired = false;
        very_long_fired = false;
        ++presses;
      } else if (!long_fired) {
        // release after a short press
        if (double_ms == 0) {
          ev = ButtonEvent::Short;
        } else if (click_pending) {
          click_pending = false;
          ev = ButtonEvent::Double;
        } else {
          click_pending = true;
          click_ms = now;
        }
      }
    }
    if (pressed && !long_fired && (uint32_t)(now - pressed_at) >= (uint32_t)BUTTON_LONG_PRESS_MS) {
      long_fired = true;
      click_pending = false;  // a click followed by a hold: the long press wins
      ev = ButtonEvent::Long;  // fires while still held
    }
#if OTA_ENABLE
    else if (pressed && long_fired && !very_long_fired && (uint32_t)(now - pressed_at) >= (uint32_t)OTA_LONG_PRESS_MS) {
      very_long_fired = true;
      ev = ButtonEvent::VeryLong;  // the same hold, still held (OTA_LONG_PRESS_MS > BUTTON_LONG_PRESS_MS)
    }
#endif
    // The window closed with the button up: the single press stands.
    if (ev == ButtonEvent::None && click_pending && !pressed && !raw_last && (uint32_t)(now - click_ms) >= double_ms) {
      click_pending = false;
      ev = ButtonEvent::Short;
    }
    return ev;
  }
};
