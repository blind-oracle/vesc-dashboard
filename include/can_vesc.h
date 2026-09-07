// CAN / VESC listener: passive (ACK-only) reception of the VESC STATUS_1..6
// broadcast frames into g_state.vesc, optional active polling of the values no
// broadcast carries (COMM_GET_VALUES_SELECTIVE, VESC_POLL_MS) into g_state.vesc_ext,
// TWAI bus health into g_state.can.
//
// This header is framework-free (only <stdint.h> + shared_state.h + vesc_getvalues.h)
// so main.cpp, the display code and the native tests can include it; everything
// that touches the TWAI driver or FreeRTOS lives in src/can_vesc.cpp.
#pragma once

#include <stdint.h>

#include "shared_state.h"
#include "vesc_getvalues.h"  // vesc_fault_str() / vesc_fault_name(), VESC_STATUS_* bits for the consumers of g_state.vesc_ext

// Creates and enables the TWAI node (ESP-IDF esp_twai_onchip.h) and spawns
// canTask (TASK_PRIO_CAN). Returns false if the node could not be created or
// enabled, or the task could not be created. On a driver failure the task is
// still created and retries every few seconds, so the caller only needs to
// log the result.
bool can_vesc_start();

// Emits ONE info log line with every VESC field, the derived values (mech rpm,
// power, revolutions), the smoothed values in parentheses, the age of each
// STATUS message ("never" when not yet received), the polled values with their
// counters ("| ext: ..." or "| poll=off") and the CAN health counters.
// Works on a snapshot: call it from the supervisor with state_snapshot().
void can_vesc_log_summary(const SharedState &s, uint32_t now);

// CanState -> "UNINST" / "STOP" / "RUN" / "BUSOFF" / "RECOV" (or "?").
const char *can_state_str(int can_state);

// True when this build transmits COMM_GET_VALUES_SELECTIVE requests (VESC_POLL_MS > 0)
// and polling has not been suspended at runtime (a VESC was seen broadcasting under
// CAN_OWN_ID). False = strictly passive node; g_state.vesc_ext then stays all zero.
bool can_vesc_polling_enabled();

// Latched fault: the last NON-ZERO fault code seen in a poll reply (0 = none so far)
// and, through t_ms_or_null, when it was seen (state_now_ms(), 0 = never). The VESC clears
// its live fault byte after "fault stop time" (default 500 ms), so a 1 Hz poll only
// sees a fault by luck; g_state.vesc_ext.fault_code is that live byte, the latch is
// the memory. Convenience reader of g_state.vesc_ext.last_fault / last_fault_ms for
// code without a snapshot at hand (takes the state mutex briefly; safe from any
// task). Consumers working on a state_snapshot() should read those fields directly.
uint8_t can_vesc_last_fault(uint32_t *t_ms_or_null);
