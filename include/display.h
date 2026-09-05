// Display module entry point. Exactly one implementation is compiled, selected
// by DISPLAY_TYPE in config.h:
//   DISPLAY_TYPE_OLED_SSD1309   -> src/display_oled.cpp (128x64 OLED, I2C)  [default]
//   DISPLAY_TYPE_EPD_GDEY042T81 -> src/display_epd.cpp  (400x300 e-ink, SPI) [legacy]
// Both run their own FreeRTOS task, read state_snapshot(), touch hb_disp and
// publish DisplayStats into g_state.disp.
#pragma once

// Initialises the display hardware and starts the display task. Returns false if
// the hardware did not respond (the task still runs and shows what it can).
bool display_start();
