// JK BMS over Bluetooth LE: public entry points of the BMS task (src/bms_ble.cpp).
//
// Framework-free header (only <stdint.h> + project headers) so main.cpp, the display
// builders and the native tests can include it. When BMS_BLE_ENABLE is 0 the functions
// are inline no-ops, so callers need no #if.
#pragma once

#include <stdint.h>

#include "config.h"
#include "shared_state.h"

#if BMS_BLE_ENABLE
// Initialises NVS + the NimBLE host and starts bmsTask. Returns false if BLE could not be
// brought up (everything else keeps running; the BMS screens then show "BLE OFF").
bool bms_ble_start();
// One ESP_LOGI line with the link state and every decoded value (call from the supervisor
// with a state snapshot; take `now` after the snapshot).
void bms_log_summary(const SharedState &s, uint32_t now);
// BmsLink -> "OFF" / "IDLE" / "SCAN" / "CONN" / "SETUP" / "STREAM".
const char *bms_link_str(uint8_t link);
// Tells the BMS task whether a screen that shows BMS data is on the panel
// (oled_screen_needs_bms()). Called by the display task on every screen change and
// once at start-up; safe from any task (one volatile flag, bmsTask reads it in its
// own loop, so it acts on a change within one message-buffer poll, 100 ms).
//   true  -> connect to the BMS (direct to the last known address, else scan) and poll it
//   false -> terminate the link, stop scanning and stay idle (BMS_LINK_IDLE)
// With BMS_LINK_ON_DEMAND 0 the link is held from boot on and this call does nothing.
void bms_set_active(bool active);
#else
inline bool bms_ble_start() { return false; }
inline void bms_log_summary(const SharedState &, uint32_t) {}
inline const char *bms_link_str(uint8_t) { return "OFF"; }
inline void bms_set_active(bool) {}
#endif
