// CAN / VESC listener: passive (ACK-only) reception of the VESC STATUS_1..6
// broadcast frames into g_state.vesc, TWAI bus health into g_state.can.
//
// This header is Arduino-free (only <stdint.h> + shared_state.h) so main.cpp
// and the native tests can include it; everything that touches the TWAI
// driver or FreeRTOS lives in src/can_vesc.cpp.
#pragma once

#include <stdint.h>

#include "shared_state.h"

// Installs the legacy TWAI driver (driver/twai.h), starts it and spawns
// canTask (TASK_PRIO_CAN). Returns false if the driver could not be installed
// or started, or the task could not be created. On a driver failure the task
// is still created and retries the install every few seconds, so the caller
// only needs to log the result.
bool can_vesc_start();

// Emits ONE log_i line with every VESC field, the derived values (mech rpm,
// power, revolutions), the smoothed values in parentheses, the age of each
// STATUS message ("never" when not yet received) and the CAN health counters.
// Works on a snapshot: call it from the supervisor with state_snapshot().
void can_vesc_log_summary(const SharedState &s, uint32_t now);

// CanState -> "UNINST" / "STOP" / "RUN" / "BUSOFF" / "RECOV" (or "?").
const char *can_state_str(int can_state);
