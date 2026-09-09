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
// BmsLink -> "OFF" / "SCAN" / "CONN" / "SETUP" / "STREAM".
const char *bms_link_str(uint8_t link);
#else
inline bool bms_ble_start() { return false; }
inline void bms_log_summary(const SharedState &, uint32_t) {}
inline const char *bms_link_str(uint8_t) { return "OFF"; }
#endif
