// Pure, deterministic builder of the strings shown on the 128x64 OLED dashboard.
//
// Arduino-free (only <stdint.h>/<stdio.h>/<string.h>): the display task calls
// it on a state snapshot, and the native unit tests exercise it on the host.
// Header-only because the "native" test environment compiles no src/ files.
//
// Contract:
//   * Every value that is stale (older than its freshness window) or was never
//     received is rendered as "--"; nothing stale is ever shown as live data.
//   * Every VALUE string (speed, v_in, i_in, i_motor, power) is at most 4
//     characters, every RIGHT-COLUMN string (unit, fix, can) at most 8: the
//     layout in display_oled.cpp reserves exactly that many glyph cells.
//     A number that does not fit its 4 cells falls back to fewer decimals,
//     then to thousands with a 'k' suffix ("12k", "-5k"), then is clipped.
//   * The output is fully deterministic for a given (state, now): the struct
//     is zeroed before formatting, so the display task can memcmp two results
//     to decide whether the panel needs a refresh at all.
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "shared_state.h"
#include "vesc_status.h"

struct OledStrings {
  char speed[6];    // "10.0" / "117" / "0.0" / "--"          (<= 4 chars, size-3 font)
  char unit[6];     // SPEED_UNIT_STR ("kn" / "km/h")
  char fix[10];     // "3D  9sv" / "2D 12sv" / "NO FIX" / "NO DATA" / "NO GNSS" / "BOOT"   (<= 8 chars)
  char can[10];     // "VESC 74" / "CAN idle" / "CAN RECV" / "CAN OFF" / FW_VERSION at boot (<= 8 chars)
  char v_in[6];     // "48.2" / "100" / "--"                  (<= 4 chars)
  char i_in[6];     // "12.4" / "-12" / "123" / "--"          (<= 4 chars, battery current, signed)
  char i_motor[6];  // "35" / "-35" / "1k" / "--"             (<= 4 chars)
  char power[6];    // "598" / "-480" / "-5k" / "12k" / "--"  (<= 4 chars, v_in * i_in, signed)
};

namespace oled_detail {

// Glyph cells reserved by the layout (display_oled.cpp): 4 for a value, 8 for
// the right column (8 * 6 px = 48 px, x 80..127 at text size 1).
constexpr size_t kValueChars = 4;
constexpr size_t kColumnChars = 8;

// Hard length guarantee: the caller memcmp()s and lays out fixed-width cells, so
// nothing may ever exceed max_chars (buffers are >= max_chars + 1 bytes).
inline void clip(char *buf, size_t n, size_t max_chars) {
  if (max_chars < n) buf[max_chars] = '\0';
}

// Strips the sign from a negative zero ("-0.0" -> "0.0", "-0" -> "0"). WHY: a
// smoothed current hovering around 0 A would otherwise flip between "0" and
// "-0", each flip costing a needless frame push.
inline void strip_negative_zero(char *buf) {
  if (buf[0] != '-') return;
  for (const char *p = buf + 1; *p; ++p) {
    if (*p != '0' && *p != '.') return;  // a real non-zero digit: keep the sign
  }
  memmove(buf, buf + 1, strlen(buf));    // drops the '-' and moves the NUL along
}

// Thousands with a 'k' suffix: "12k", "-5k", "100k". Used when a number does
// not fit its 4 cells, and for motor currents of 1000 A and more. Beyond
// +-999.5k the cell saturates to "MAX" / "-MAX": clipping "10737k" to "1073"
// would print a plausible-looking wrong number. (VESC fields are int16/10, so
// only the power product can get there: 3276.7 V * 3276.7 A = 10.7 MW.)
inline void fmt_kilo(char *buf, size_t n, float v) {
  snprintf(buf, n, "%.0fk", (double)(v / 1000.0f));
  strip_negative_zero(buf);
  if (strlen(buf) > kValueChars) snprintf(buf, n, "%s", v < 0.0f ? "-MAX" : "MAX");
  clip(buf, n, kValueChars);
}

// Formats v into a 4-character value cell. decimals = 1 tries "%.1f" first
// ("48.2", "-9.9", "10.0"); decimals = 0 (or a 1-decimal result that is too
// wide, e.g. "100.0") uses "%.0f" ("100", "-480", "9999"); anything still too
// wide is shown in thousands ("12k", "-5k"); a final clip guards the length.
// Literal format strings keep -Wformat happy; (double) casts keep -Wdouble-promotion quiet.
inline void fmt_value(char *buf, size_t n, float v, int decimals) {
  if (decimals > 0) {
    snprintf(buf, n, "%.1f", (double)v);
    strip_negative_zero(buf);
    if (strlen(buf) <= kValueChars) return;
  }
  snprintf(buf, n, "%.0f", (double)v);
  strip_negative_zero(buf);
  if (strlen(buf) <= kValueChars) return;
  fmt_kilo(buf, n, v);
}

inline void fmt_dashes(char *buf, size_t n) { snprintf(buf, n, "--"); }

// ---- VESC values ------------------------------------------------------------
inline void build_vesc(const VescState &v, uint32_t now, OledStrings &out) {
  const bool s1 = fresh(v.t.t_ms[VESC_IDX_STATUS_1], now, VESC_STALE_R1_MS);
  const bool s4 = fresh(v.t.t_ms[VESC_IDX_STATUS_4], now, VESC_STALE_R1_MS);
  const bool s5 = fresh(v.t.t_ms[VESC_IDX_STATUS_5], now, VESC_STALE_R1_MS);

  if (s5) fmt_value(out.v_in, sizeof out.v_in, v.v_in_ema, 1);
  else fmt_dashes(out.v_in, sizeof out.v_in);

  // Battery current: one decimal while positive and < 100 A ("12.4"); regen /
  // charging (negative) and >= 100 A in whole amps so the sign or third digit fits.
  if (s4) fmt_value(out.i_in, sizeof out.i_in, v.i_in_ema, v.i_in_ema >= 0.0f ? 1 : 0);
  else fmt_dashes(out.i_in, sizeof out.i_in);

  // Motor current in whole amps; from 1000 A (either sign) in kiloamps ("1k").
  if (!s1) fmt_dashes(out.i_motor, sizeof out.i_motor);
  else if (v.i_motor_ema >= 1000.0f || v.i_motor_ema <= -1000.0f) fmt_kilo(out.i_motor, sizeof out.i_motor, v.i_motor_ema);
  else fmt_value(out.i_motor, sizeof out.i_motor, v.i_motor_ema, 0);

  // Power needs both factors live: a fresh current times a stale voltage is a lie.
  if (s4 && s5) fmt_value(out.power, sizeof out.power, v.v_in_ema * v.i_in_ema, 0);
  else fmt_dashes(out.power, sizeof out.power);

  clip(out.v_in, sizeof out.v_in, kValueChars);
  clip(out.i_in, sizeof out.i_in, kValueChars);
  clip(out.i_motor, sizeof out.i_motor, kValueChars);
  clip(out.power, sizeof out.power, kValueChars);
}

// ---- GNSS: speed + fix line ---------------------------------------------------
inline void build_gnss(const GnssState &g, uint32_t now, OledStrings &out) {
  const bool ever = g.last_pvt_ms != 0;
  const bool live = fresh(g.last_pvt_ms, now, GNSS_STALE_MS);

  snprintf(out.unit, sizeof out.unit, "%s", SPEED_UNIT_STR);

  // Speed is gated three ways: recent epoch, receiver says the fix is usable,
  // and the receiver's own speed-accuracy estimate is sane (a fresh 2D fix
  // with sAcc 5 m/s would otherwise show random knots at the dock).
  if (!live || !g.fix_ok || g.sacc_mm_s > (uint32_t)GNSS_MAX_SACC_MM_S) {
    fmt_dashes(out.speed, sizeof out.speed);
  } else {
    const float v = (float)g.gspeed_mm_s * SPEED_FACTOR;
    if (v < SPEED_MIN_SHOW) snprintf(out.speed, sizeof out.speed, "0.0");  // hides drift at rest (and any negative oddity)
    else fmt_value(out.speed, sizeof out.speed, v, 1);                     // "10.0" below 100, "117" above
  }

  if (!ever) snprintf(out.fix, sizeof out.fix, "NO GNSS");
  else if (!live) snprintf(out.fix, sizeof out.fix, "NO DATA");
  else if (g.fix_type == 3 || g.fix_type == 4) snprintf(out.fix, sizeof out.fix, "3D %2usv", (unsigned)g.num_sv);
  else if (g.fix_type == 2) snprintf(out.fix, sizeof out.fix, "2D %2usv", (unsigned)g.num_sv);
  else snprintf(out.fix, sizeof out.fix, "NO FIX");

  clip(out.speed, sizeof out.speed, kValueChars);
  clip(out.unit, sizeof out.unit, kColumnChars);
  clip(out.fix, sizeof out.fix, kColumnChars);
}

// ---- CAN / VESC line -----------------------------------------------------------
inline void build_can(const SharedState &s, uint32_t now, OledStrings &out) {
  switch (s.can.state) {
    case CAN_STATE_RUNNING: {
      // Same freshness test as the LED in main.cpp: STATUS_1 or STATUS_5 within VESC_STALE_R1_MS.
      const bool s1 = fresh(s.vesc.t.t_ms[VESC_IDX_STATUS_1], now, VESC_STALE_R1_MS);
      const bool s5 = fresh(s.vesc.t.t_ms[VESC_IDX_STATUS_5], now, VESC_STALE_R1_MS);
      if (!s1 && !s5) snprintf(out.can, sizeof out.can, "CAN idle");
      else if (s.vesc.locked_id >= 0) snprintf(out.can, sizeof out.can, "VESC %u", (unsigned)(s.vesc.locked_id & 0xFF));  // 0..254
      else snprintf(out.can, sizeof out.can, "VESC ?");  // fresh frames but no id locked: cannot happen, stay defined
      break;
    }
    case CAN_STATE_RECOVERING: snprintf(out.can, sizeof out.can, "CAN RECV"); break;
    default: snprintf(out.can, sizeof out.can, "CAN OFF"); break;  // STOPPED, BUS_OFF, UNINSTALLED, unknown
  }
  clip(out.can, sizeof out.can, kColumnChars);
}

}  // namespace oled_detail

// Builds every displayed string from a state snapshot.
//   now  current millis() (same clock as the timestamps in s)
inline void oled_build_strings(const SharedState &s, uint32_t now, OledStrings &out) {
  memset(&out, 0, sizeof out);  // deterministic tail bytes: the caller memcmp()s whole structs
  oled_detail::build_vesc(s.vesc, now, out);
  oled_detail::build_gnss(s.gnss, now, out);
  oled_detail::build_can(s, now, out);
}

// The frame shown before any data arrived: every value "--", fix line "BOOT",
// CAN line = firmware version.
inline void oled_boot_strings(OledStrings &out) {
  memset(&out, 0, sizeof out);
  oled_detail::fmt_dashes(out.speed, sizeof out.speed);
  snprintf(out.unit, sizeof out.unit, "%s", SPEED_UNIT_STR);
  snprintf(out.fix, sizeof out.fix, "BOOT");
  snprintf(out.can, sizeof out.can, "%s", FW_VERSION);
  oled_detail::fmt_dashes(out.v_in, sizeof out.v_in);
  oled_detail::fmt_dashes(out.i_in, sizeof out.i_in);
  oled_detail::fmt_dashes(out.i_motor, sizeof out.i_motor);
  oled_detail::fmt_dashes(out.power, sizeof out.power);
  oled_detail::clip(out.unit, sizeof out.unit, oled_detail::kColumnChars);
  oled_detail::clip(out.can, sizeof out.can, oled_detail::kColumnChars);
}
