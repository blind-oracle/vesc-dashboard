// Status LED on PIN_LED, driven by the supervisor (main.cpp): a 1 Hz blink while VESC
// data is fresh, 0.2 Hz otherwise, 4 Hz (green) in update mode. Two kinds of LED (config.h):
//   LED_WS2812 0  a plain GPIO LED, active high (an external one, say);
//   LED_WS2812 1  one WS2812 RGB LED (default: the S3-Zero's on-board one, GPIO21), which
//                 also shows the freshness as a colour: green = VESC data fresh, red = none.
// Framework-free header; the implementation is src/status_led.cpp.
#pragma once

// Configures the pin (and for a WS2812 the RMT channel) and switches the LED off. Returns
// false if the LED could not be set up; status_led_set() is then a no-op.
bool status_led_init();

// on = lit; fresh selects the WS2812 colour (ignored by a plain LED).
void status_led_set(bool on, bool fresh);
