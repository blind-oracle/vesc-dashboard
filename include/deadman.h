// Dead-man's switch: public entry points of the deadman task (src/deadman.cpp).
//
// Framework-free header (only <stdint.h>/<stdbool.h> + project headers) so main.cpp,
// the display task and the native tests can include it. With DEADMAN_ENABLE 0 every
// function is an inline no-op, so callers need no #if.
//
// The decision logic itself is pure and lives in include/deadman_logic.h; this module
// is the FreeRTOS/GPIO shell around it.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "shared_state.h"

#if DEADMAN_ENABLE
// Puts PIN_DEADMAN_CUT into its boot state (DEADMAN_BOOT_CUT). Call this as the FIRST
// statement of setup(), before any delay: until it runs the pin is a floating input and
// the line rests wherever the mechanical switch and its pull-down leave it.
void deadman_gpio_early_init();

// Starts deadmanTask (TASK_PRIO_DEADMAN). Returns false if the task could not be created
// or the beacon address is unusable; the cut is then asserted permanently and logged.
bool deadman_start();

// Called from the NimBLE host task for every advert that matches DEADMAN_BEACON_ADDR.
// Three volatile stores: no lock, no log, no allocation. Deliberately NOT routed through
// the BMS message buffer, so the safety timestamp never depends on bmsTask draining it.
void deadman_beacon_seen(int8_t rssi, uint32_t now_ms);
// Called from the same place for every advert that does not match (crowded-marina counter).
void deadman_adv_other();

// Called from the display task when the reset gesture is made (a long press while the
// DEADMAN screen is shown). The task consumes it on its next tick and may refuse it.
void deadman_request_reset();

// Immediate transition lines (call every supervisor loop) and the periodic summary
// (call on the LOG_DEADMAN_MS deadline). All dead-man logging happens here, never in
// the task itself: the console can block, and a blocked priority-7 task misses its tick.
void deadman_log_events(const SharedState &s, uint32_t now);
void deadman_log_summary(const SharedState &s, uint32_t now);
#else
inline void deadman_gpio_early_init() {}
inline bool deadman_start() { return false; }
inline void deadman_beacon_seen(int8_t, uint32_t) {}
inline void deadman_adv_other() {}
inline void deadman_request_reset() {}
inline void deadman_log_events(const SharedState &, uint32_t) {}
inline void deadman_log_summary(const SharedState &, uint32_t) {}
#endif
