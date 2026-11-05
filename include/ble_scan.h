// BLE radio for the dead-man's switch: public entry points of the beacon scan (src/ble_scan.cpp).
//
// Framework-free header (only <stdbool.h> + project headers) so main.cpp, the dead-man task
// and the native tests can include it. With DEADMAN_ENABLE 0 there is no radio code at all and
// the functions are inline no-ops, so callers need no #if.
#pragma once

#include <stdbool.h>

#include "config.h"

#if DEADMAN_ENABLE
// Initialises NVS + the NimBLE host and starts the ble task, which keeps the beacon watchdog
// scan running for as long as the host is synced. Returns false if BLE could not be brought
// up; the dead-man then reports NO RADIO (no protection, and it does not cut for it).
bool ble_scan_start();
// True while the watchdog scan is running. Read by the dead-man task to decide whether it is
// actually protecting anything - it must never call a NimBLE API itself, the ble task owns those.
bool ble_scan_active();
#else
inline bool ble_scan_start() { return false; }
inline bool ble_scan_active() { return false; }
#endif
