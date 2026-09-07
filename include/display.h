// Display module entry point (src/display_oled.cpp: 128x64 SSD1309 OLED over I2C).
// The display task reads state_snapshot(), touches hb_disp and publishes
// DisplayStats into g_state.disp.
#pragma once

// Initialises the display hardware and starts the display task. Returns false if
// the hardware did not respond (the task still runs, retries, and shows what it can).
bool display_start();
