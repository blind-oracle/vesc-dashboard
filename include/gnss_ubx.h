// GNSS module: u-blox UBX receiver on HP UART1 (Serial1).
//
// gnss_start() configures Serial1 and launches gnssTask, which runs the phase
// machine AUTOBAUD -> DETECT (MON-VER / PROTVER) -> CONFIGURE (VALSET for
// PROTVER >= 27, CFG-MSG/CFG-RATE otherwise, NAV-VELNED fallback for u-blox 6)
// -> RUN (NAV-PVT into g_state.gnss). No valid UBX frame for GNSS_REDETECT_MS
// while running sends it back to AUTOBAUD, so a power-cycled or swapped module
// recovers without an ESP32 reboot.
//
// All UART and FreeRTOS code lives in src/gnss_ubx.cpp; the protocol parser
// and builders are in the Arduino-free include/ubx_min.h.
#pragma once

#include <stdint.h>

#include "shared_state.h"

// Sets the RX ring size, opens Serial1 on PIN_GNSS_RX/PIN_GNSS_TX and creates
// the "gnss" task at TASK_PRIO_GNSS. Never blocks setup(): all receiver I/O
// happens in the task. Returns false only if the task could not be created.
bool gnss_start();

// One log_i line summarizing s.gnss (phase, baud, PROTVER, fix, sats, speed in
// the display unit, sAcc, pDOP, UTC, NAV-PVT age, frame counters, redetects).
// Call from the supervisor with a snapshot, never under the state mutex.
void gnss_log_summary(const SharedState &s, uint32_t now);

// "AUTOBAUD" / "DETECT" / "CONFIG" / "RUN" for a GnssPhase value ("?" otherwise).
const char *gnss_phase_str(uint8_t phase);
