// Coolant pump: the pure decision logic of the pump task (src/pump.cpp).
//
// Framework-free (only <stdint.h>/<math.h>/<string.h> + project headers), so the native
// tests drive it with synthetic temperatures and a synthetic clock.
//
// The pump (Kamoer KDLP1200-B12-T) cools the motor and the VESC. Its targets come from the
// VESC's own limits (include/vesc_mcconf.h): per sensor, the point where the VESC starts
// cutting acceleration current, minus PUMP_TARGET_MARGIN_C. One error drives the loop:
// the larger of (motor - motor target) and (MOSFET - MOSFET target) over the sensors that
// read plausibly, so whichever runs closer to its limit sets the flow.
//
// Modes, in order of precedence:
//   FORCED  PUMP_FORCE_DUTY_PCT >= 0 (commissioning): that duty, nothing else
//   OFF     no fresh temperatures since boot (VESC off at the dock): 0 %
//   BLIND   temperatures were seen and then went stale, or none reads plausibly: the motor
//           may still be working and we cannot see it, so PUMP_BLIND_DUTY_PCT (100 %)
//   FULL    a temperature at or above its derating start: 100 % (a backstop; the PID is
//           normally there already)
//   FULL    the pump should run but its tach says it does not turn (STALL / NO_SIGNAL,
//           include/pump_tach.h): PUMP_TACH_STALL_DUTY_PCT. Only the duty changes: the
//           start / stop decisions below still apply, so a cool, still motor stops it
//   RUN     the PID, clamped to PUMP_MIN_DUTY_PCT..100 %. A stopped pump starts when the
//           motor turns or the error rises above -PUMP_ON_BAND_C; it stops again only with
//           the motor still, the error below -(PUMP_ON_BAND_C + PUMP_HYST_C) and
//           PUMP_MIN_RUN_MS gone since the last of those reasons (Kamoer: frequent
//           start/stop wears the pump)
//   OFF     everything else
//
// The PID: u = Kp * err + I + Kd * d(err)/dt (low-passed by PUMP_D_ALPHA; Kd is 0 by default).
// I stays in 0..100 and does not integrate while the output is pinned at 100 % with the error
// still positive, so it never winds up; at the bottom it simply decays to 0. Entering RUN from
// FULL or BLIND preloads I so the duty continues from where it was (bumpless) and eases down.
#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "pump_tach.h"
#include "shared_state.h"
#include "vesc_mcconf.h"
#include "vesc_status.h"

#if PUMP_ENABLE

static_assert(PUMP_KP >= 0.0f && PUMP_KI >= 0.0f && PUMP_KD >= 0.0f, "PUMP_KP / PUMP_KI / PUMP_KD must be >= 0");
static_assert(PUMP_D_ALPHA > 0.0f && PUMP_D_ALPHA <= 1.0f, "PUMP_D_ALPHA must be in (0, 1]");

inline const char *pump_mode_str(uint8_t m) {
  switch (m) {
    case PUMP_MODE_OFF: return "OFF";
    case PUMP_MODE_RUN: return "RUN";
    case PUMP_MODE_FULL: return "FULL";
    case PUMP_MODE_BLIND: return "BLIND";
    case PUMP_MODE_FORCED: return "FORCED";
    case PUMP_MODE_FAULT: return "FAULT";
    default: return "?";
  }
}

// <= 9 chars: the PUMP screen shows it next to the mode and the duty.
inline const char *pump_why_str(uint8_t w) {
  switch (w) {
    case PUMP_WHY_NO_DATA: return "no data";
    case PUMP_WHY_COOL: return "cool";
    case PUMP_WHY_WARM: return "warm";
    case PUMP_WHY_TURNING: return "turning";
    case PUMP_WHY_AFTER_RUN: return "after-run";
    case PUMP_WHY_DERATE: return "derating";
    case PUMP_WHY_STALE: return "no link";
    case PUMP_WHY_NO_SENSOR: return "no sensor";
    case PUMP_WHY_FORCED: return "forced";
    case PUMP_WHY_PWM: return "pwm setup";
    case PUMP_WHY_STALLED: return "stalled";
    case PUMP_WHY_NO_TACH: return "no tach";
    default: return "";
  }
}

// ---------------------------------------------------------------- targets
struct PumpTargets {
  float derate_mot, derate_fet;  // where the VESC starts cutting acceleration current, C
  float tgt_mot, tgt_fet;        // derate - PUMP_TARGET_MARGIN_C
};

inline VescTempLimits pump_default_limits() {
  VescTempLimits l;
  l.fet_start = PUMP_DEFAULT_TEMP_FET_START;
  l.fet_end = PUMP_DEFAULT_TEMP_FET_END;
  l.mot_start = PUMP_DEFAULT_TEMP_MOTOR_START;
  l.mot_end = PUMP_DEFAULT_TEMP_MOTOR_END;
  l.accel_dec = PUMP_DEFAULT_TEMP_ACCEL_DEC;
  return l;
}

// The limits the targets come from: the VESC's own once it answered COMM_GET_MCCONF with a
// layout this code knows, else the PUMP_DEFAULT_* values.
inline VescTempLimits pump_limits(const VescLimitsState &v, bool *from_vesc) {
  const bool ok = v.result == MCCONF_OK;
  if (from_vesc) *from_vesc = ok;
  return ok ? v.lim : pump_default_limits();
}

inline PumpTargets pump_targets(const VescTempLimits &l) {
  PumpTargets t;
  t.derate_mot = vesc_derate_start(l.mot_start, l.accel_dec);
  t.derate_fet = vesc_derate_start(l.fet_start, l.accel_dec);
  t.tgt_mot = t.derate_mot - (float)PUMP_TARGET_MARGIN_C;
  t.tgt_fet = t.derate_fet - (float)PUMP_TARGET_MARGIN_C;
  return t;
}

// ---------------------------------------------------------------- inputs
struct PumpInputs {
  bool fresh;         // STATUS_4 (both temperatures) inside VESC_STALE_R1_MS
  bool mot_ok, fet_ok;  // the reading is plausible (vesc_temp_plausible)
  float t_mot, t_fet;   // C
  bool turning;       // fresh STATUS_1 with |mechanical rpm| > PUMP_TURNING_RPM
  uint8_t tach;       // PumpTachState from the tach (PUMP_TACH_NONE without one); set by the pump task
};

// From the VESC telemetry, read under the state lock with `now` taken inside it (a frame
// stamped after an earlier clock read would otherwise look 49 days old: one false BLIND tick).
inline PumpInputs pump_inputs(const vesc_telemetry_t &t, uint32_t now) {
  PumpInputs in;
  in.fresh = vesc_fresh(&t, VESC_IDX_STATUS_4, now, VESC_STALE_R1_MS);
  in.t_mot = t.temp_motor;
  in.t_fet = t.temp_fet;
  in.mot_ok = vesc_temp_plausible(t.temp_motor);
  in.fet_ok = vesc_temp_plausible(t.temp_fet);
  in.turning = vesc_fresh(&t, VESC_IDX_STATUS_1, now, VESC_STALE_R1_MS) &&
               fabsf(vesc_mech_rpm(&t, VESC_MOTOR_POLES)) > (float)PUMP_TURNING_RPM;
  in.tach = PUMP_TACH_NONE;  // the pump task fills it from its own tach state
  return in;
}

// ---------------------------------------------------------------- controller
struct PumpCtl {
  uint8_t mode;           // PumpMode
  uint8_t why;            // PumpWhy
  bool seen;              // temperatures were fresh at least once since boot
  bool have_err;          // err holds the previous tick's error (derivative history)
  float err;              // max over the plausible sensors of (temperature - target), C
  float d_filt;           // low-passed d(err)/dt, C/s
  float integ;            // integral term, % duty
  float duty;             // last output, %
  uint32_t last_need_ms;  // last tick something asked for flow (turning / warm / derating); 0 = never
};

inline void pump_init(PumpCtl &c) { memset(&c, 0, sizeof c); }

namespace pump_detail {
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline float set(PumpCtl &c, uint8_t mode, uint8_t why, float duty) {
  c.mode = mode;
  c.why = why;
  c.duty = duty;
  return duty;
}

inline void pid_reset(PumpCtl &c) {
  c.integ = 0.0f;
  c.d_filt = 0.0f;
  c.have_err = false;
}
}  // namespace pump_detail

// One control tick: returns the duty (0..100 %) and updates c.mode / c.why. dt_s is the time
// since the previous tick (clamped to 0..5 s, so a stalled task never integrates a burst).
inline float pump_step(PumpCtl &c, const PumpInputs &in, const PumpTargets &tg, uint32_t now, float dt_s) {
  using namespace pump_detail;
  dt_s = clampf(dt_s, 0.0f, 5.0f);
#if PUMP_FORCE_DUTY_PCT >= 0
  (void)in;
  (void)tg;
  (void)now;
  return set(c, PUMP_MODE_FORCED, PUMP_WHY_FORCED, (float)PUMP_FORCE_DUTY_PCT);
#else
  const uint8_t prev = c.mode;
  const float prev_duty = c.duty;
  if (in.fresh) c.seen = true;

  // ---- blind: nothing to control on ----
  if (!in.fresh || (!in.mot_ok && !in.fet_ok)) {
    pid_reset(c);
    if (!c.seen) return set(c, PUMP_MODE_OFF, PUMP_WHY_NO_DATA, 0.0f);
    return set(c, PUMP_MODE_BLIND, in.fresh ? PUMP_WHY_NO_SENSOR : PUMP_WHY_STALE, (float)PUMP_BLIND_DUTY_PCT);
  }

  // ---- the error: whichever sensor runs closest to (or furthest past) its target ----
  float err = -1.0e9f;
  bool derate = false;
  if (in.mot_ok) {
    err = fmaxf(err, in.t_mot - tg.tgt_mot);
    derate = derate || in.t_mot >= tg.derate_mot;
  }
  if (in.fet_ok) {
    err = fmaxf(err, in.t_fet - tg.tgt_fet);
    derate = derate || in.t_fet >= tg.derate_fet;
  }
  if (c.have_err && dt_s > 0.0f) c.d_filt += PUMP_D_ALPHA * ((err - c.err) / dt_s - c.d_filt);
  else c.d_filt = 0.0f;
  c.err = err;
  c.have_err = true;

  const bool warm = err > -(float)PUMP_ON_BAND_C;
  if (in.turning || warm || derate) c.last_need_ms = now ? now : 1u;

  // ---- backstop: the VESC is about to (or already does) cut power ----
  if (derate) return set(c, PUMP_MODE_FULL, PUMP_WHY_DERATE, 100.0f);

  // ---- start / stop ----
  const bool running = prev == PUMP_MODE_RUN || prev == PUMP_MODE_FULL || prev == PUMP_MODE_BLIND;
  if (!running) {
    if (!in.turning && !warm) {
      pid_reset(c);
      c.have_err = true;  // keep the error as derivative history
      return set(c, PUMP_MODE_OFF, PUMP_WHY_COOL, 0.0f);
    }
    c.integ = 0.0f;  // a fresh start: the minimum, plus P once it is warm
  } else {
    const bool cold = err < -(float)(PUMP_ON_BAND_C + PUMP_HYST_C);
    const bool after_run_over = c.last_need_ms == 0 || (uint32_t)(now - c.last_need_ms) >= (uint32_t)PUMP_MIN_RUN_MS;
    if (!in.turning && cold && after_run_over) {
      pid_reset(c);
      c.have_err = true;
      return set(c, PUMP_MODE_OFF, PUMP_WHY_COOL, 0.0f);
    }
    // Bumpless from FULL / BLIND: continue at the duty it had and let the loop ease it down.
    if (prev != PUMP_MODE_RUN) c.integ = clampf(prev_duty - PUMP_KP * err - PUMP_KD * c.d_filt, 0.0f, 100.0f);
  }

#if PUMP_TACH_ENABLE && PUMP_TACH_STALL_DUTY_PCT > 0
  // ---- it should run, but the tach says it does not turn: the most torque to restart, and
  // a broken FG wire costs noise, never cooling. Coming back, RUN eases down from here. ----
  if (pump_tach_alarm(in.tach))
    return set(c, PUMP_MODE_FULL, in.tach == PUMP_TACH_STALL ? PUMP_WHY_STALLED : PUMP_WHY_NO_TACH,
               (float)PUMP_TACH_STALL_DUTY_PCT);
#endif

  // ---- PID ----
  float u = PUMP_KP * err + c.integ + PUMP_KD * c.d_filt;
  if (!(u >= 100.0f && err > 0.0f)) {  // anti-windup: no integrating further into the top
    c.integ = clampf(c.integ + PUMP_KI * err * dt_s, 0.0f, 100.0f);
    u = PUMP_KP * err + c.integ + PUMP_KD * c.d_filt;
  }
  const uint8_t why = warm ? PUMP_WHY_WARM : (in.turning ? PUMP_WHY_TURNING : PUMP_WHY_AFTER_RUN);
  return set(c, PUMP_MODE_RUN, why, clampf(u, (float)PUMP_MIN_DUTY_PCT, 100.0f));
#endif
}

// Duty in % -> LEDC compare value at `bits` resolution. LEDC takes 0..2^bits, 2^bits being
// constantly high; the MOSFET stage's inversion is the channel's output_invert flag, not here.
inline uint32_t pump_duty_to_ledc(float pct, unsigned bits) {
  const uint32_t full = 1u << bits;
  if (!(pct > 0.0f)) return 0;  // also NaN
  if (pct >= 100.0f) return full;
  return (uint32_t)(pct * (float)full / 100.0f + 0.5f);
}

#endif  // PUMP_ENABLE
