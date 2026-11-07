// Coolant pump: public entry points of the pump task (src/pump.cpp).
//
// Framework-free header (only <stdint.h>/<stdbool.h> + project headers) so main.cpp and the
// native tests can include it. With PUMP_ENABLE 0 every function is an inline no-op, so
// callers need no #if.
//
// The control logic itself is pure and lives in include/pump_logic.h; this module is the
// FreeRTOS/LEDC shell around it.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "shared_state.h"

#if PUMP_ENABLE
// Drives PIN_PUMP_PWM to the "pump stopped" level (high through the inverting MOSFET stage).
// Call early in setup(), in both boot modes: OTA update mode never starts the pump task, so
// this is what keeps the pump stopped there. Until it runs the pin floats and the hardware
// runs the pump at full speed (the gate pull-down), which is the safe default.
void pump_gpio_early_init();

// Configures the LEDC PWM output and starts pumpTask (TASK_PRIO_PUMP). If the PWM cannot be
// set up the pin is released (floating = full speed), the mode reads FAULT and false comes
// back; the task still runs so the screen and the log keep saying so.
bool pump_start();

// Immediate mode transitions (call every supervisor loop) and the periodic summary (call on
// the LOG_PUMP_MS deadline). The task itself never logs.
void pump_log_events(const SharedState &s, uint32_t now);
void pump_log_summary(const SharedState &s, uint32_t now);

// Called by the VESC link task (src/can_vesc.cpp or src/uart_vesc.cpp) with every
// COMM_GET_MCCONF reply payload: parses the temperature limits, publishes
// g_state.vesc_limits and logs what changed. Returns the VescMcconfResult for the link's read
// scheduler (MCCONF_NONE when the state lock timed out: nothing stored, ask again soon).
uint8_t pump_mcconf_reply(const uint8_t *payload, uint16_t len, uint32_t now);
// Called by the link task for every COMM_GET_MCCONF request it sent (a counter).
void pump_mcconf_sent();
#else
inline void pump_gpio_early_init() {}
inline bool pump_start() { return false; }
inline void pump_log_events(const SharedState &, uint32_t) {}
inline void pump_log_summary(const SharedState &, uint32_t) {}
inline uint8_t pump_mcconf_reply(const uint8_t *, uint16_t, uint32_t) { return 0; }
inline void pump_mcconf_sent() {}
#endif
