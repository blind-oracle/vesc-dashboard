// Peaks since boot (pure part): the largest power, battery / motor current, rpm, MOSFET / motor
// temperature and GNSS ground speed seen since power-on, for the PEAKS screen and the log.
//
// Sampling: the producers call these inside their own locked writes of g_state, so every
// VESC frame (CAN STATUS_1/4/5 at up to 50 Hz, or each UART GET_VALUES reply) and every GNSS
// epoch is seen; nothing is lost to a slower consumer's sampling. The values are the smoothed
// ones the dashboard shows (VescState::*_ema), so a switching spike in the raw STATUS_4
// current never becomes a "peak" nobody could have read off the screen.
//
// What counts (the same freshness and plausibility gates as the MAIN screen):
//   * motor current, rpm   STATUS_1 fresh: the MAGNITUDE (reverse counts as much as ahead);
//   * battery current      STATUS_4 fresh: drawn only, max(I, 0) (regen never counts);
//   * power                STATUS_4 and STATUS_5 fresh: drawn only, max(V x I, 0);
//   * temperatures         STATUS_4 fresh and vesc_temp_plausible (no motor NTC = never set);
//   * ground speed         trip_gnss_usable (fresh epoch, fix_ok, sAcc <= GNSS_MAX_SACC_MM_S),
//                          max(gSpeed, 0).
// The first accepted sample sets a field (and its PeakBit), later ones only raise it, so a
// drawn-only field first seen while regenerating reads 0 rather than staying "never seen".
// Non-finite values are ignored.
//
// Framework-free (<math.h> + shared_state.h + trip.h + vesc_status.h) so the native Unity
// tests (test/test_peaks) can drive it. The log line lives in src/peaks.cpp.
#pragma once

#include <math.h>
#include <stdint.h>

#include "config.h"
#include "shared_state.h"
#include "trip.h"  // trip_gnss_usable(): the same "usable fix" test the trip distance uses
#include "vesc_status.h"

// Raises one field: the first finite sample sets it, later ones only push it up.
inline void peak_raise(float &slot, uint8_t &have, uint8_t bit, float v) {
  if (!isfinite(v)) return;
  if (!(have & bit) || v > slot) slot = v;
  have = (uint8_t)(have | bit);
}

// One VESC update (CAN status frame or UART GET_VALUES reply). v = the state just written;
// now = the clock the producer stamped it with.
inline void peaks_vesc(PeaksState &p, const VescState &v, uint32_t now) {
  const vesc_telemetry_t &t = v.t;
  const bool s1 = vesc_fresh(&t, VESC_IDX_STATUS_1, now, VESC_STALE_R1_MS);
  const bool s4 = vesc_fresh(&t, VESC_IDX_STATUS_4, now, VESC_STALE_R1_MS);
  const bool s5 = vesc_fresh(&t, VESC_IDX_STATUS_5, now, VESC_STALE_R1_MS);
  if (s1) {
    peak_raise(p.i_mot_a, p.have, PEAK_IMOT, fabsf(v.i_motor_ema));
    peak_raise(p.rpm, p.have, PEAK_RPM, fabsf(vesc_mech_rpm(&t, VESC_MOTOR_POLES)));
  }
  if (s4) {
    peak_raise(p.i_bat_a, p.have, PEAK_IBAT, fmaxf(v.i_in_ema, 0.0f));
    if (vesc_temp_plausible(t.temp_fet)) peak_raise(p.temp_fet_c, p.have, PEAK_TFET, t.temp_fet);
    if (vesc_temp_plausible(t.temp_motor)) peak_raise(p.temp_mot_c, p.have, PEAK_TMOT, t.temp_motor);
  }
  if (s4 && s5) peak_raise(p.power_w, p.have, PEAK_POWER, fmaxf(v.v_in_ema * v.i_in_ema, 0.0f));
}

// One GNSS epoch (NAV-PVT, or NAV-VELNED on a u-blox 6). g = the state just written.
inline void peaks_gnss(PeaksState &p, const GnssState &g, uint32_t now) {
  if (!trip_gnss_usable(g, now)) return;
  const int32_t v = g.gspeed_mm_s > 0 ? g.gspeed_mm_s : 0;
  if (!(p.have & PEAK_SPEED) || v > p.speed_mm_s) p.speed_mm_s = v;
  p.have = (uint8_t)(p.have | PEAK_SPEED);
}

// ---------------------------------------------------------------------------
// Task API (src/peaks.cpp)
// ---------------------------------------------------------------------------
// Supervisor: one log line with every peak ("--" for a field never seen).
void peaks_log_summary(const SharedState &s);
