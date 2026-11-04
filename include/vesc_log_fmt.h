// The all-fields "VESC ..." log line, shared by both VESC links (src/can_vesc.cpp and,
// with VESC_LINK_UART, src/uart_vesc.cpp): every telemetry field, the derived values
// (mech rpm, power, revolutions), the smoothed values in parentheses, the age of each
// STATUS slot ("never" when not yet received), the polled values with their counters
// ("| ext: ..." or "| poll=off"), then the link's own health text.
//
// Firmware-only: it uses strlcat, which the native host libc may lack.
#pragma once

#include <stdio.h>
#include <string.h>

#include "shared_state.h"
#include "vesc_getvalues.h"
#include "vesc_status.h"

namespace vesc_log_detail {

inline void fmt_age(char *buf, size_t n, uint32_t t_ms, uint32_t now) {
  if (t_ms == 0) snprintf(buf, n, "never");
  else snprintf(buf, n, "%lums", (unsigned long)(now - t_ms));  // unsigned subtraction: wrap-safe
}

// The motor NTC input floats when nothing is wired, so the VESC reports garbage
// (often -273 or > 200): show "n/a" outside a physically plausible range.
inline void fmt_tmot(char *buf, size_t n, const vesc_telemetry_t &t) {
  const bool have = t.t_ms[VESC_IDX_STATUS_4] != 0;
  if (!have || t.temp_motor < -40.0f || t.temp_motor > 200.0f) snprintf(buf, n, "n/a");
  else snprintf(buf, n, "%.1fC", t.temp_motor);
}

#if VESC_POLL_MS > 0
// "| ext: ..." section of the summary: the polled values, their counters and age.
inline void fmt_ext(char *buf, size_t n, const SharedState &s, uint32_t now) {
  const VescExt &x = s.vesc_ext;
  char age[32];
  fmt_age(age, sizeof age, x.t_ms, now);
  if (x.t_ms != 0 && (uint32_t)(now - x.t_ms) > VESC_EXT_STALE_MS) strlcat(age, "(stale)", sizeof age);
  char fc[12], lfc[12];  // "F<code>" for fault codes newer than our table instead of a bare "F?"
  vesc_fault_fmt(fc, sizeof fc, x.fault_code);
  char last[40];  // latched fault straight from the snapshot: no lock in the log path
  if (x.last_fault == 0 || x.last_fault_ms == 0) snprintf(last, sizeof last, "none");
  else snprintf(last, sizeof last, "%s@%lums", vesc_fault_fmt(lfc, sizeof lfc, x.last_fault), (unsigned long)(now - x.last_fault_ms));
  const char *st = (x.status & VESC_STATUS_TIMEOUT) ? ((x.status & VESC_STATUS_KILL_SW) ? "TO+KILL" : "TO")
                                                    : ((x.status & VESC_STATUS_KILL_SW) ? "KILL" : "-");
  snprintf(buf, n,
           "| ext: fault=%s last=%s Tmos=%.1f/%.1f/%.1fC Iavg=%.2f/%.2fA Id/Iq=%.2f/%.2fA Vd/Vq=%.3f/%.3fV "
           "tachoAbs=%ld st=%s id=%u polls=%lu ok=%lu bad=%lu to=%lu age=%s",
           fc, last, x.temp_mos1, x.temp_mos2, x.temp_mos3, x.avg_motor_current,
           x.avg_input_current, x.avg_id, x.avg_iq, x.vd, x.vq, (long)x.tacho_abs, st, (unsigned)x.vesc_id,
           (unsigned long)x.polls_sent, (unsigned long)x.replies_ok, (unsigned long)x.replies_bad,
           (unsigned long)x.timeouts, age);
}
#endif

}  // namespace vesc_log_detail

// Formats the whole line into line[n]; link is the transport's health text, appended
// after the last '|'. Typical line ~650 bytes; the worst case (all counters saturated,
// all ages stale) is ~950 bytes, so callers pass 1024. snprintf truncates safely.
inline void vesc_log_line(char *line, size_t n, const SharedState &s, uint32_t now, const char *link) {
  using namespace vesc_log_detail;
  const vesc_telemetry_t &t = s.vesc.t;
  char age[VESC_STATUS_COUNT][24];
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) {
    // STATUS 1/4/5 are "Rate 1" (VESC_STALE_R1_MS), STATUS 2/3/6 are "Rate 2" (VESC_STALE_R2_MS)
    const bool r1 = (i == VESC_IDX_STATUS_1 || i == VESC_IDX_STATUS_4 || i == VESC_IDX_STATUS_5);
    const uint32_t stale_ms = r1 ? VESC_STALE_R1_MS : VESC_STALE_R2_MS;
    fmt_age(age[i], sizeof age[i], t.t_ms[i], now);
    if (t.t_ms[i] != 0 && (uint32_t)(now - t.t_ms[i]) > stale_ms) strlcat(age[i], "(stale)", sizeof age[i]);
  }
  char tmot[16];
  fmt_tmot(tmot, sizeof tmot, t);
  // Polled values: ~200 bytes typical, ~330 with every counter saturated.
  char ext[352];
#if VESC_POLL_MS > 0
  fmt_ext(ext, sizeof ext, s, now);
#else
  snprintf(ext, sizeof ext, "| poll=off");
#endif

  snprintf(line, n,
           "VESC id=%d frames=%lu other=%lu | erpm=%.0f rpm=%.0f duty=%.3f Im=%.1fA(%.1f) | "
           "Iin=%.1fA(%.1f) Vin=%.1fV(%.1f) P=%.0fW | Tfet=%.1fC Tmot=%s | pid=%.1f | "
           "Ah=%.4f/%.4f Wh=%.4f/%.4f | tacho=%ld revs=%.1f | adc=%.3f/%.3f/%.3f ppm=%.3f | "
           "age s1=%s s2=%s s3=%s s4=%s s5=%s s6=%s %s | %s",
           s.vesc.locked_id, (unsigned long)s.vesc.frames_total, (unsigned long)s.vesc.frames_other_id,
           t.erpm, vesc_mech_rpm(&t, VESC_MOTOR_POLES), t.duty, t.current_motor, s.vesc.i_motor_ema,
           t.current_in, s.vesc.i_in_ema, t.v_in, s.vesc.v_in_ema, vesc_power_w(&t),
           t.temp_fet, tmot, t.pid_pos,
           t.amp_hours, t.amp_hours_charged, t.watt_hours, t.watt_hours_charged,
           (long)t.tachometer, vesc_revolutions(&t, VESC_MOTOR_POLES),
           t.adc1, t.adc2, t.adc3, t.ppm,
           age[0], age[1], age[2], age[3], age[4], age[5], ext, link);
}
