// Pure, deterministic builder of the strings shown on the e-paper dashboard.
//
// Arduino-free (only <stdint.h>/<stdio.h>/<string.h>): the display task calls
// it on a state snapshot, and the native unit tests exercise it on the host.
// Header-only because the "native" test environment compiles no src/ files.
//
// Contract:
//   * Every value that is stale (older than its freshness window) or was never
//     received is rendered as "--"; nothing stale is ever shown as live data.
//   * The output is fully deterministic for a given (state, now, partials):
//     the struct is zeroed before formatting, so the display task can memcmp
//     two results to decide whether the panel needs a refresh at all.
//   * Every string is produced with snprintf into its own fixed buffer, so
//     extreme values truncate instead of overflowing.
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "shared_state.h"
#include "vesc_status.h"

struct DisplayStrings {
  char speed[8];        // "10.0" / "--"
  char speed_unit[6];   // SPEED_UNIT_STR ("kn" / "km/h")
  char sats[12];        // "SAT 09" / "SAT --"
  char fix[10];         // "3D FIX" / "2D FIX" / "NO FIX" / "NO DATA" / "NO GNSS"
  char utc[12];         // "12:34 UTC" or "" when the receiver has no valid time
  char v_in[10];        // "48.2" / "--"
  char i_in[10];        // "12.4" / "-3.1" / "--"   (battery current, signed)
  char i_motor[10];     // "35" / "--"
  char power[10];       // "598" / "--"             (v_in * i_in, signed)
  char status_l[48];    // "VESC 74  CAN RUN  Tf 41C"
  char status_r[48];    // "GNSS 38400  #123"
};

namespace display_detail {

// Formats a float with 0 or 1 decimals, then strips the sign from a negative
// zero ("-0.0" -> "0.0"). WHY: a smoothed current hovering around 0 A would
// otherwise flip between "0.0" and "-0.0", each flip costing a needless
// e-paper refresh. Literal format strings keep -Wformat happy.
inline void fmt_num(char *buf, size_t n, int decimals, float v) {
  if (decimals == 0) snprintf(buf, n, "%.0f", (double)v);
  else snprintf(buf, n, "%.1f", (double)v);
  if (buf[0] != '-') return;
  for (const char *p = buf + 1; *p; ++p) {
    if (*p != '0' && *p != '.') return;  // a real non-zero digit: keep the sign
  }
  memmove(buf, buf + 1, strlen(buf));    // drops the '-' and moves the NUL along
}

inline void fmt_dashes(char *buf, size_t n) { snprintf(buf, n, "--"); }

inline const char *can_state_name(int state) {
  switch (state) {
    case CAN_STATE_RUNNING: return "RUN";
    case CAN_STATE_STOPPED: return "STOP";
    case CAN_STATE_BUS_OFF: return "BUSOFF";
    case CAN_STATE_RECOVERING: return "RECOV";
    case CAN_STATE_UNINSTALLED: return "UNINST";
    default: return "?";
  }
}

inline const char *gnss_phase_name(uint8_t phase) {
  switch (phase) {
    case GNSS_PHASE_AUTOBAUD: return "AUTOBAUD";
    case GNSS_PHASE_DETECT: return "DETECT";
    case GNSS_PHASE_CONFIGURE: return "CONFIG";
    case GNSS_PHASE_RUN: return "RUN";
    default: return "?";
  }
}

// ---- VESC tiles -------------------------------------------------------------
inline void build_vesc(const VescState &v, uint32_t now, DisplayStrings &out) {
  const bool s1 = fresh(v.t.t_ms[VESC_IDX_STATUS_1], now, VESC_STALE_R1_MS);
  const bool s4 = fresh(v.t.t_ms[VESC_IDX_STATUS_4], now, VESC_STALE_R1_MS);
  const bool s5 = fresh(v.t.t_ms[VESC_IDX_STATUS_5], now, VESC_STALE_R1_MS);

  if (s5) fmt_num(out.v_in, sizeof out.v_in, 1, v.v_in_ema);
  else fmt_dashes(out.v_in, sizeof out.v_in);

  if (s4) fmt_num(out.i_in, sizeof out.i_in, 1, v.i_in_ema);
  else fmt_dashes(out.i_in, sizeof out.i_in);

  if (s1) fmt_num(out.i_motor, sizeof out.i_motor, 0, v.i_motor_ema);
  else fmt_dashes(out.i_motor, sizeof out.i_motor);

  // Power needs both factors live: a fresh current times a stale voltage is a lie.
  if (s4 && s5) fmt_num(out.power, sizeof out.power, 0, v.v_in_ema * v.i_in_ema);
  else fmt_dashes(out.power, sizeof out.power);
}

// ---- GNSS column + speed -----------------------------------------------------
inline void build_gnss(const GnssState &g, uint32_t now, DisplayStrings &out) {
  const bool ever = g.last_pvt_ms != 0;
  const bool live = fresh(g.last_pvt_ms, now, GNSS_STALE_MS);

  snprintf(out.speed_unit, sizeof out.speed_unit, "%s", SPEED_UNIT_STR);

  // Speed is gated three ways: recent epoch, receiver says the fix is usable,
  // and the receiver's own speed-accuracy estimate is sane (a fresh 2D fix
  // with sAcc 5 m/s would otherwise show random knots at the dock).
  if (!live || !g.fix_ok || g.sacc_mm_s > (uint32_t)GNSS_MAX_SACC_MM_S) {
    fmt_dashes(out.speed, sizeof out.speed);
  } else {
    float v = (float)g.gspeed_mm_s * SPEED_FACTOR;
    if (v < SPEED_MIN_SHOW) v = 0.0f;  // hides drift at rest (and any negative oddity)
    fmt_num(out.speed, sizeof out.speed, 1, v);
  }

  if (live) snprintf(out.sats, sizeof out.sats, "SAT %02u", (unsigned)g.num_sv);
  else snprintf(out.sats, sizeof out.sats, "SAT --");

  if (!ever) snprintf(out.fix, sizeof out.fix, "NO GNSS");
  else if (!live) snprintf(out.fix, sizeof out.fix, "NO DATA");
  else if (g.fix_type == 3 || g.fix_type == 4) snprintf(out.fix, sizeof out.fix, "3D FIX");
  else if (g.fix_type == 2) snprintf(out.fix, sizeof out.fix, "2D FIX");
  else snprintf(out.fix, sizeof out.fix, "NO FIX");

  if (live && g.time_valid) snprintf(out.utc, sizeof out.utc, "%02u:%02u UTC", (unsigned)g.hour, (unsigned)g.min);
  else out.utc[0] = '\0';
}

// ---- status bar --------------------------------------------------------------
inline void build_status(const SharedState &s, uint32_t now, uint32_t partials, DisplayStrings &out) {
  char id[8];
  if (s.vesc.locked_id < 0) snprintf(id, sizeof id, "--");
  else snprintf(id, sizeof id, "%u", (unsigned)(s.vesc.locked_id & 0xFF));  // 0..254

  char tf[12];
  if (fresh(s.vesc.t.t_ms[VESC_IDX_STATUS_4], now, VESC_STALE_R1_MS)) fmt_num(tf, sizeof tf, 0, s.vesc.t.temp_fet);
  else fmt_dashes(tf, sizeof tf);

  snprintf(out.status_l, sizeof out.status_l, "VESC %s  CAN %s  Tf %sC", id, can_state_name(s.can.state), tf);

  char gnss[12];
  if (s.gnss.phase == GNSS_PHASE_RUN) snprintf(gnss, sizeof gnss, "%lu", (unsigned long)s.gnss.baud);
  else snprintf(gnss, sizeof gnss, "%s", gnss_phase_name(s.gnss.phase));

  snprintf(out.status_r, sizeof out.status_r, "GNSS %s  #%lu", gnss, (unsigned long)partials);
}

}  // namespace display_detail

// Builds every displayed string from a state snapshot.
//   now                 current millis() (same clock as the timestamps in s)
//   partials_for_status partial-refresh counter to print in the status bar. Pass
//                       the value BEFORE incrementing it for the refresh being
//                       decided, so the counter alone never triggers a redraw.
inline void display_build_strings(const SharedState &s, uint32_t now, uint32_t partials_for_status,
                                  DisplayStrings &out) {
  memset(&out, 0, sizeof out);  // deterministic tail bytes: the caller memcmp()s whole structs
  display_detail::build_vesc(s.vesc, now, out);
  display_detail::build_gnss(s.gnss, now, out);
  display_detail::build_status(s, now, partials_for_status, out);
}

// The frame shown before any data arrived: every value "--", status "BOOT".
inline void display_boot_strings(DisplayStrings &out) {
  memset(&out, 0, sizeof out);
  display_detail::fmt_dashes(out.speed, sizeof out.speed);
  snprintf(out.speed_unit, sizeof out.speed_unit, "%s", SPEED_UNIT_STR);
  snprintf(out.sats, sizeof out.sats, "SAT --");
  snprintf(out.fix, sizeof out.fix, "NO GNSS");
  display_detail::fmt_dashes(out.v_in, sizeof out.v_in);
  display_detail::fmt_dashes(out.i_in, sizeof out.i_in);
  display_detail::fmt_dashes(out.i_motor, sizeof out.i_motor);
  display_detail::fmt_dashes(out.power, sizeof out.power);
  snprintf(out.status_l, sizeof out.status_l, "BOOT");
  snprintf(out.status_r, sizeof out.status_r, "FW %s", FW_VERSION);
}
