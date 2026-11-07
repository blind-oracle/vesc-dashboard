// Firmware update: the pure part of the entry gate (no FreeRTOS, no IDF, no I/O).
//
// Holding the button for OTA_LONG_PRESS_MS asks for update mode, which reboots the board and
// leaves it without the VESC link, the GNSS and the dead-man's tag watch until it reboots
// again. ota_entry_check() decides from a state snapshot whether that is acceptable right
// now; src/ota.cpp adds the two checks that need the flash (a spare app slot exists, the
// running image is not on probation) and owns the reboot.
//
// Framework-free (<math.h>/<stdint.h> + project headers) so the native Unity tests
// (test/test_display_strings_oled) can drive it, and the FW UPDATE screen can name a refusal.
#pragma once

#include <math.h>
#include <stdint.h>

#include "config.h"
#include "shared_state.h"
#include "vesc_status.h"

enum OtaRefusal : uint8_t {
  OTA_OK = 0,
  OTA_REFUSED_MOTOR,        // fresh VESC data: the motor is turning
  OTA_REFUSED_MOVING,       // a fresh, accurate GNSS fix: the boat is moving
  OTA_REFUSED_DEADMAN,      // the dead-man alarm owns the panel (TRIPPED / FAULT)
  OTA_REFUSED_NO_SLOT,      // the flashed partition table has no second app slot (src/ota.cpp)
  OTA_REFUSED_UNCONFIRMED,  // the running image is still on probation (src/ota.cpp)
};

// Stale or never-received data counts as stationary: a VESC that is switched off at the dock,
// or a GNSS indoors, must not block an update. Only data that positively shows motion refuses.
// The dead-man alarm refuses for a different reason: update mode followed by the reboot out of
// it would clear a trip without the tag being present, which the reset gesture never allows.
inline OtaRefusal ota_entry_check(const SharedState &s, uint32_t now) {
#if DEADMAN_UI_ENABLE
  if (s.deadman.state == DM_TRIPPED || s.deadman.state == DM_FAULT) return OTA_REFUSED_DEADMAN;
#endif
  const vesc_telemetry_t &t = s.vesc.t;
  if (vesc_fresh(&t, VESC_IDX_STATUS_1, now, VESC_STALE_R1_MS) &&
      fabsf(vesc_mech_rpm(&t, VESC_MOTOR_POLES)) > (float)OTA_MAX_RPM)
    return OTA_REFUSED_MOTOR;
  // The same trust rule as the speed on the main screen: a fresh epoch, a usable fix and a sane
  // speed-accuracy estimate (a poor fix at the dock reports random speeds).
  const GnssState &g = s.gnss;
  if (fresh(g.last_pvt_ms, now, GNSS_STALE_MS) && g.fix_ok && g.sacc_mm_s <= (uint32_t)GNSS_MAX_SACC_MM_S &&
      (g.gspeed_mm_s > OTA_MAX_SPEED_MM_S || g.gspeed_mm_s < -OTA_MAX_SPEED_MM_S))
    return OTA_REFUSED_MOVING;
  return OTA_OK;
}

// Second row of the "UPDATE REFUSED" screen (<= 21 chars): what is wrong.
inline const char *ota_refusal_text(uint8_t r) {
  switch (r) {
    case OTA_REFUSED_MOTOR: return "motor is turning";
    case OTA_REFUSED_MOVING: return "boat is moving";
    case OTA_REFUSED_DEADMAN: return "dead-man alarm is on";
    case OTA_REFUSED_NO_SLOT: return "no spare app slot";
    case OTA_REFUSED_UNCONFIRMED: return "new image unconfirmed";
    default: return "?";
  }
}

// Third row (<= 21 chars): what to do about it.
inline const char *ota_refusal_hint(uint8_t r) {
  switch (r) {
    case OTA_REFUSED_MOTOR: return "stop the motor";
    case OTA_REFUSED_MOVING: return "stop the boat";
    case OTA_REFUSED_DEADMAN: return "clear the alarm first";
    case OTA_REFUSED_NO_SLOT: return "flash once by USB";
    case OTA_REFUSED_UNCONFIRMED: return "retry in a minute";
    default: return "";
  }
}

#if OTA_ENABLE
// Log name of a phase.
inline const char *ota_phase_name(uint8_t phase) {
  switch (phase) {
    case OTA_PHASE_OFF: return "OFF";
    case OTA_PHASE_STARTING: return "STARTING";
    case OTA_PHASE_WAITING: return "WAITING";
    case OTA_PHASE_RECEIVING: return "RECEIVING";
    case OTA_PHASE_DONE: return "DONE";
    case OTA_PHASE_FAILED: return "FAILED";
    default: return "?";
  }
}
#endif
