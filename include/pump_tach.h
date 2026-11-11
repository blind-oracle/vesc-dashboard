// Coolant pump: the pure tachometer logic of the pump task (src/pump.cpp).
//
// Framework-free (only <stdint.h>/<string.h> + project headers), so the native tests drive it
// with synthetic pulse counts and a synthetic clock.
//
// The pump's yellow FG wire gives PUMP_TACH_PPR pulses per revolution (Kamoer: 1). The PCNT
// peripheral counts them; every PUMP_PERIOD_MS the pump task hands the running total, the
// duty it applied over the last period and the clock to pump_tach_step(), which keeps the
// rpm over the last PUMP_TACH_WINDOW_MS and judges it against the duty:
//
//   duty >= PUMP_MIN_DUTY_PCT  the pump should turn. No verdict for PUMP_TACH_SPINUP_MS, then
//                              below PUMP_TACH_MIN_RPM for PUMP_TACH_STALL_MS = STALL (or
//                              NO_SIGNAL when not one pulse arrived since boot), an alarm that
//                              holds until the rpm stays up for PUMP_TACH_OK_MS. Otherwise OK.
//   duty == 0                  the pump should be still. No verdict for PUMP_TACH_SPINDOWN_MS
//                              (this also ends an alarm), then at or above PUMP_TACH_MIN_RPM
//                              for PUMP_TACH_OK_MS = RUNS_AT_0: the PWM stage does not stop
//                              it. Otherwise OK.
//   anything in between        a FORCED commissioning duty below the minimum: no verdict.
//
// The pump task turns STALL / NO_SIGNAL into PUMP_TACH_STALL_DUTY_PCT (pump_logic.h) and the
// display into the alarm overlay; RUNS_AT_0 is only reported (the pump then runs flat out,
// which is the cool side).
#pragma once

#include <stdint.h>
#include <string.h>

#include "config.h"
#include "shared_state.h"

#if PUMP_ENABLE

// The two tach states that mean "the coolant does not flow although it should".
inline bool pump_tach_alarm(uint8_t s) { return s == PUMP_TACH_STALL || s == PUMP_TACH_NO_SIGNAL; }

// For the log and, for the states that need attention, the PUMP page (<= 10 chars).
inline const char *pump_tach_str(uint8_t s) {
  switch (s) {
    case PUMP_TACH_NONE: return "none";
    case PUMP_TACH_WAIT: return "wait";
    case PUMP_TACH_OK: return "OK";
    case PUMP_TACH_STALL: return "STALL";
    case PUMP_TACH_NO_SIGNAL: return "NO TACH";
    case PUMP_TACH_RUNS_AT_0: return "RUNS AT 0%";
    case PUMP_TACH_ERR: return "tach ERR";
    default: return "?";
  }
}

#if PUMP_TACH_ENABLE

// One sample more than the window holds periods: the rpm is the difference across it.
static constexpr unsigned kPumpTachSlots = PUMP_TACH_WINDOW_MS / PUMP_PERIOD_MS + 1;

struct PumpTach {
  uint32_t pulses[kPumpTachSlots];  // ring of pulse totals ...
  uint32_t t_ms[kPumpTachSlots];    // ... and when each was read
  uint8_t head;                     // next slot to write (= the oldest once the ring is full)
  uint8_t n;                        // samples held
  uint8_t state;                    // PumpTachState
  bool full;                        // rpm averages a whole window
  bool seen;                        // at least one pulse since boot
  float rpm;
  uint32_t run_since;   // applied duty >= PUMP_MIN_DUTY_PCT without a break since (0 = not running)
  uint32_t stop_since;  // applied duty == 0 since (0 = not stopped)
  uint32_t low_since;   // rpm below PUMP_TACH_MIN_RPM since (0 = it is not)
  uint32_t high_since;  // rpm at or above PUMP_TACH_MIN_RPM since (0 = it is not)
  uint32_t alarms;      // entries into STALL / NO_SIGNAL
  uint32_t since;       // when `state` was entered (0 = never)
};

inline void pump_tach_init(PumpTach &t) {
  memset(&t, 0, sizeof t);
  t.state = PUMP_TACH_WAIT;
}

namespace pump_tach_detail {
inline uint32_t elapsed(uint32_t since, uint32_t now) { return since ? (uint32_t)(now - since) : 0u; }
}  // namespace pump_tach_detail

// One tick. pulses_total = the counter since it was set up (wraps at 2^32: only differences
// are used), duty_pct = what the pump was driven with over the period that just ended.
// Returns the new state (also in t.state).
inline uint8_t pump_tach_step(PumpTach &t, uint32_t pulses_total, float duty_pct, uint32_t now) {
  using pump_tach_detail::elapsed;
  if (now == 0) now = 1;  // 0 means "never" in every *_since field

  // ---- the rpm over the window ----
  t.pulses[t.head] = pulses_total;
  t.t_ms[t.head] = now;
  t.head = (uint8_t)((t.head + 1u) % kPumpTachSlots);
  if (t.n < kPumpTachSlots) t.n++;
  const unsigned newest = (t.head + kPumpTachSlots - 1u) % kPumpTachSlots;
  const unsigned oldest = (t.head + kPumpTachSlots - t.n) % kPumpTachSlots;
  const uint32_t dt = (uint32_t)(t.t_ms[newest] - t.t_ms[oldest]);
  const uint32_t dp = (uint32_t)(t.pulses[newest] - t.pulses[oldest]);
  t.rpm = dt ? (float)dp * 60000.0f / ((float)PUMP_TACH_PPR * (float)dt) : 0.0f;
  t.full = t.n == kPumpTachSlots;
  if (pulses_total != 0) t.seen = true;  // the counter starts at 0 when it is set up

  // ---- what the pump was asked to do, and since when ----
  const bool run = duty_pct >= (float)PUMP_MIN_DUTY_PCT;
  const bool stop = !(duty_pct > 0.0f);  // also NaN
  if (!run) t.run_since = 0;
  else if (!t.run_since) t.run_since = now;
  if (!stop) t.stop_since = 0;
  else if (!t.stop_since) t.stop_since = now;
  const bool turning = t.rpm >= (float)PUMP_TACH_MIN_RPM;
  if (turning) {
    t.low_since = 0;
    if (!t.high_since) t.high_since = now;
  } else {
    t.high_since = 0;
    if (!t.low_since) t.low_since = now;
  }

  // ---- the verdict ----
  uint8_t next = t.state;
  if (run) {
    if (pump_tach_alarm(t.state)) {
      if (elapsed(t.high_since, now) >= (uint32_t)PUMP_TACH_OK_MS) next = PUMP_TACH_OK;
      else next = t.seen ? PUMP_TACH_STALL : PUMP_TACH_NO_SIGNAL;  // pulses since: the same alarm, better named
    } else if (elapsed(t.run_since, now) < (uint32_t)PUMP_TACH_SPINUP_MS) {
      next = PUMP_TACH_WAIT;
    } else if (turning) {
      next = PUMP_TACH_OK;
    } else {
      // Count the slow time from the end of the spin-up only: a start is slow by nature.
      uint32_t from = t.low_since;
      const uint32_t settled = t.run_since + (uint32_t)PUMP_TACH_SPINUP_MS;
      if ((int32_t)(settled - from) > 0) from = settled;
      if ((uint32_t)(now - from) >= (uint32_t)PUMP_TACH_STALL_MS) next = t.seen ? PUMP_TACH_STALL : PUMP_TACH_NO_SIGNAL;
      else if (next != PUMP_TACH_OK) next = PUMP_TACH_WAIT;  // a dip under the minimum: OK holds until it counts
    }
  } else if (stop) {
    if (elapsed(t.stop_since, now) < (uint32_t)PUMP_TACH_SPINDOWN_MS) next = PUMP_TACH_WAIT;  // ends an alarm too
    else if (t.state == PUMP_TACH_RUNS_AT_0) next = elapsed(t.low_since, now) >= (uint32_t)PUMP_TACH_OK_MS ? PUMP_TACH_OK : PUMP_TACH_RUNS_AT_0;
    else if (elapsed(t.high_since, now) >= (uint32_t)PUMP_TACH_OK_MS) next = PUMP_TACH_RUNS_AT_0;
    else if (!turning) next = PUMP_TACH_OK;
    else if (next != PUMP_TACH_OK) next = PUMP_TACH_WAIT;
  } else {
    next = PUMP_TACH_WAIT;  // a FORCED duty under the minimum: the pump may or may not turn there
  }

  if (pump_tach_alarm(next) && !pump_tach_alarm(t.state)) t.alarms++;
  if (next != t.state || t.since == 0) t.since = now;
  t.state = next;
  return next;
}

#endif  // PUMP_TACH_ENABLE
#endif  // PUMP_ENABLE
