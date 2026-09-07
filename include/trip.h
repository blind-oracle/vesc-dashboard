// Trip / efficiency integrator (pure part).
//
// Turns snapshots of the shared VESC + GNSS state into trip distance, trip
// energy and efficiency in Wh per EFF_DIST_UNIT_M (Wh/NM on the knots build,
// Wh/km otherwise, see config.h):
//   "avg"  since the integrator started (boot),
//   "now"  over the last EFF_WINDOW_S seconds (ring of one-second buckets).
//
// Arduino-free (<stdint.h>/<stdbool.h> + shared_state.h + vesc_status.h) so the
// native Unity tests (test/test_trip) can drive it with synthetic snapshots. The
// FreeRTOS task that calls it every TRIP_PERIOD_MS and publishes the result into
// g_state.trip lives in src/trip.cpp. Everything here is inline: no dynamic
// allocation, no I/O, no locking.
//
// Semantics of trip_calc_update(c, s, now_ms, out), called with a monotonically
// increasing millis() clock (unsigned wrap-safe arithmetic throughout):
//
//  * Time step: dt = now - last, clamped to TRIP_DT_MAX_MS (5 s) so a stalled
//    task never integrates one huge step. run_s counts integrator time from the
//    first call; the first call only initialises (dt = 0).
//
//  * Distance: while the GNSS is usable (NAV-PVT fresh within GNSS_STALE_MS,
//    fix_ok, sAcc <= GNSS_MAX_SACC_MM_S) and gspeed_mm_s >= EFF_MIN_SPEED_MM_S:
//    dist_m += gspeed_mm_s * dt / 1e6 with the LATEST speed sample (rectangle
//    rule, no interpolation) and moving_s accumulates dt. Otherwise nothing is
//    added (moored, drifting below the threshold, or no usable fix).
//
//  * Energy, two sources, checked in this order every tick:
//    (A) STATUS_3 fresh (VESC_STALE_R2_MS): the VESC's own counters watt_hours /
//        watt_hours_charged (monotonic, mc_interface.c only ever adds to them).
//        Every counter tick stores a BASELINE (the counter values) and an ANCHOR
//        (the trip totals at that instant); the next counter tick sets
//        totals = anchor + (counter - baseline). In steady state that is the
//        plain per-step delta. The first sample only sets baseline + anchor. A
//        delta below -TRIP_COUNTER_RESET_WH means the VESC rebooted (or its
//        counters were reset from VESC Tool): counter_resets++, nothing added,
//        re-baselined, and the bucket is a gap (that step's energy is unknown).
//        A tiny negative delta above the threshold is treated as noise: nothing
//        added, re-baselined. energy_from_counters = true.
//    (B) STATUS_3 stale but STATUS_4 and STATUS_5 fresh (VESC_STALE_R1_MS):
//        integrate P = v_in * current_in (raw fields, not the display EMA) over
//        dt; P > 0 goes to wh, P < 0 to wh_charged. energy_from_counters = false.
//        Baseline and anchor are KEPT: when STATUS_3 returns, the counters
//        overwrite the fallback's estimate for the whole time they were away
//        (totals = anchor + delta), so nothing is counted twice and the totals
//        end up exactly counter-driven. The window keeps the fallback's view of
//        those seconds (real data, no gap); the transition tick's own share goes
//        into the window as v_in x current_in x dt (STATUS_4/5 are fresh).
//    (C) Neither fresh: nothing is added; baseline and anchor are KEPT as well,
//        so a CAN outage is recovered from the counters when STATUS_3 returns,
//        whether STATUS_4/5 reappear in the same tick, one tick earlier (they
//        are broadcast 10x more often) or never. That catch-up delta goes to
//        the trip totals only, not into the "now" window (its buckets cover
//        time the delta does not belong to); the bucket receiving that tick is
//        flagged as a gap (see below).
//    Net energy = wh - wh_charged. Negative (net regen) is allowed; the display
//    decides how to show it. energy_from_counters keeps its last value while no
//    source is active (no label flapping during an outage). Because the
//    counters win, the totals can step once (by the fallback's error) when
//    STATUS_3 comes back after a long fallback period.
//
//  * Window ("now"): a ring of EFF_WINDOW_S one-second buckets of (distance,
//    net Wh, gap flag). A bucket closes every 1000 ms of INTEGRATOR time
//    (accumulated dt, so wall-clock jitter never skews it); a clamped 5 s step
//    closes five buckets (one with the data, four empty). Only closed buckets
//    count: win_dist_m / win_wh are their sums, win_fill_s their number
//    (0..EFF_WINDOW_S), win_p_avg_w = win_wh * 3600 / win_fill_s (0 when the
//    window is empty). A bucket carries a gap flag when, during any of its
//    ticks, the GNSS was unusable or no energy source was active, when it took
//    the catch-up tick after an outage, or when it was produced by a clamped
//    (stalled) step: its distance or energy is then incomplete and the ratio
//    would be wrong, not just noisy.
//    eff_now = win_wh / (win_dist_m / EFF_DIST_UNIT_M), eff_now_valid iff
//    win_fill_s >= TRIP_EFF_MIN_FILL_S && win_dist_m >= EFF_MIN_DIST_M &&
//    no gap bucket in the window && GNSS usable now && an energy source is
//    active now.
//
//  * Trip: eff_avg = (wh - wh_charged) / (dist_m / EFF_DIST_UNIT_M),
//    eff_avg_valid iff dist_m >= EFF_MIN_DIST_M.
//
//  * Accumulators are double inside TripCalc (no drift over days at 5 Hz);
//    TripState stores float. out.t_ms = now_ms (1 when now_ms is 0, so 0 keeps
//    meaning "never").
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "shared_state.h"
#include "vesc_status.h"

// ---------------------------------------------------------------------------
// Tunables. config.h ("Trip / efficiency") defines the first group; the fallbacks
// only keep this header self-contained against an older config.h.
// ---------------------------------------------------------------------------
#ifndef TRIP_PERIOD_MS
#define TRIP_PERIOD_MS 200
#endif
#ifndef EFF_WINDOW_S
#define EFF_WINDOW_S 10
#endif
#ifndef EFF_MIN_SPEED_MM_S
#define EFF_MIN_SPEED_MM_S 500
#endif
#ifndef EFF_MIN_DIST_M
#define EFF_MIN_DIST_M 10
#endif
#ifndef EFF_DIST_UNIT_M
#define EFF_DIST_UNIT_M 1000.0f
#endif
// Local constants (candidates for config.h; overridable with -D).
#ifndef TRIP_DT_MAX_MS
#define TRIP_DT_MAX_MS 5000        // longest step integrated after a stall; beyond it time is dropped, not integrated
#endif
#ifndef TRIP_COUNTER_RESET_WH
#define TRIP_COUNTER_RESET_WH 0.5f // a watt-hour counter delta below -this = the VESC rebooted (counters restarted)
#endif
#ifndef TRIP_EFF_MIN_FILL_S
#define TRIP_EFF_MIN_FILL_S 3      // closed window buckets needed before eff_now is shown
#endif

static_assert(EFF_WINDOW_S >= 2 && EFF_WINDOW_S <= 255, "EFF_WINDOW_S must fit TripState::win_fill_s (uint8_t)");
static_assert(TRIP_EFF_MIN_FILL_S >= 1 && TRIP_EFF_MIN_FILL_S <= EFF_WINDOW_S, "TRIP_EFF_MIN_FILL_S must be 1..EFF_WINDOW_S");
static_assert(TRIP_DT_MAX_MS >= 1000, "TRIP_DT_MAX_MS must be at least one window bucket (1000 ms)");

// ---------------------------------------------------------------------------
// Integrator state. Plain aggregate: trip_calc_init() zeroes it.
// ---------------------------------------------------------------------------
struct TripCalc {
  // clock
  bool started;            // first call seen (last_ms valid)
  uint32_t last_ms;        // now_ms of the previous call
  uint32_t run_s;          // integrator time since the first call, whole seconds ...
  uint32_t run_rem_ms;     // ... plus this remainder (0..999)
  uint32_t moving_s;       // time with ground speed >= EFF_MIN_SPEED_MM_S, whole seconds ...
  uint32_t moving_rem_ms;  // ... plus this remainder
  // trip totals
  double dist_m;           // GNSS ground speed integrated while moving
  double wh;               // energy drawn from the battery
  double wh_charged;       // energy returned to the battery (regen)
  // energy source
  bool have_baseline;      // base_* / anchor_* are valid (a STATUS_3 sample has been seen)
  bool prev_counters;      // the previous tick applied the counters (its delta spans exactly one step)
  bool from_counters;      // last active source: true = STATUS_3 counters, false = v_in x current_in
  bool energy_active;      // this tick had an energy source (counters fresh or fallback fresh)
  double base_wh;          // STATUS_3 watt_hours at the last counter tick ...
  double base_whc;         // ... and watt_hours_charged
  double anchor_wh;        // trip wh at that same tick: totals = anchor + (counter - base) at the next counter tick
  double anchor_whc;       // (kept across fallback / outage so the counters overwrite the estimate, never add to it)
  uint32_t counter_resets; // VESC reboots detected from the counters
  // "now" window: ring of closed one-second buckets ...
  double ring_dist[EFF_WINDOW_S];
  double ring_wh[EFF_WINDOW_S];
  bool ring_gap[EFF_WINDOW_S];
  uint8_t head;            // next slot to write
  uint8_t filled;          // closed buckets present (0..EFF_WINDOW_S)
  // ... and the bucket being filled
  uint32_t bucket_ms;      // integrator time accumulated into the open bucket
  double cur_dist;
  double cur_wh;
  bool cur_gap;
};

// True when the GNSS snapshot may be integrated: a fresh NAV-PVT with a fix and
// a plausible speed accuracy.
inline bool trip_gnss_usable(const GnssState &g, uint32_t now_ms) {
  return fresh(g.last_pvt_ms, now_ms, GNSS_STALE_MS) && g.fix_ok && g.sacc_mm_s <= (uint32_t)GNSS_MAX_SACC_MM_S;
}

// seconds/remainder accumulator: keeps whole seconds exact without 64-bit math.
inline void trip_add_ms(uint32_t &sec, uint32_t &rem_ms, uint32_t dt_ms) {
  rem_ms += dt_ms;
  sec += rem_ms / 1000u;
  rem_ms %= 1000u;
}

inline void trip_calc_init(TripCalc &c) { c = TripCalc{}; }

// Fallback energy of one step: P = v_in x current_in over dt_ms (W * ms -> Wh, signed, regen negative).
inline double trip_step_wh(const vesc_telemetry_t &t, uint32_t dt_ms) {
  return (double)t.v_in * (double)t.current_in * (double)dt_ms / 3.6e6;
}

// Adds one fallback step to the trip totals (P > 0 -> wh, P < 0 -> wh_charged); returns the signed Wh for the window.
inline double trip_integrate_power(TripCalc &c, const vesc_telemetry_t &t, uint32_t dt_ms) {
  const double e = trip_step_wh(t, dt_ms);
  if (e >= 0.0) c.wh += e;
  else c.wh_charged -= e;
  return e;
}

// Advance the integrator to time now_ms using the snapshot s; writes the derived TripState into out.
inline void trip_calc_update(TripCalc &c, const SharedState &s, uint32_t now_ms, TripState &out) {
  const vesc_telemetry_t &t = s.vesc.t;
  const GnssState &g = s.gnss;

  // ---- time step (unsigned: wrap-safe; clamped: a stall drops time instead of integrating it) ----
  uint32_t dt = 0;
  bool clamped = false;
  if (c.started) {
    dt = now_ms - c.last_ms;
    if (dt > (uint32_t)TRIP_DT_MAX_MS) {
      dt = TRIP_DT_MAX_MS;
      clamped = true;
    }
  }
  c.started = true;
  c.last_ms = now_ms;
  trip_add_ms(c.run_s, c.run_rem_ms, dt);

  // ---- distance: latest ground speed sample x dt while moving ----
  const bool gnss_ok = trip_gnss_usable(g, now_ms);
  const bool moving = gnss_ok && g.gspeed_mm_s >= (int32_t)EFF_MIN_SPEED_MM_S;
  double d_dist = 0.0;
  if (moving && dt) {
    d_dist = (double)g.gspeed_mm_s * (double)dt * 1e-6;  // mm/s * ms -> m
    c.dist_m += d_dist;
    trip_add_ms(c.moving_s, c.moving_rem_ms, dt);
  }

  // ---- energy: (A) STATUS_3 counters, (B) v_in x current_in fallback, (C) none ----
  const bool s3 = vesc_fresh(&t, VESC_IDX_STATUS_3, now_ms, VESC_STALE_R2_MS);
  const bool r1 = vesc_fresh(&t, VESC_IDX_STATUS_4, now_ms, VESC_STALE_R1_MS) &&
                  vesc_fresh(&t, VESC_IDX_STATUS_5, now_ms, VESC_STALE_R1_MS);
  const bool prev_active = c.energy_active;  // the previous tick had a source (with prev_counters false: the fallback)
  double d_net = 0.0;                        // this tick's net energy that belongs to the window
  if (s3) {
    if (c.have_baseline) {
      const double d_wh = (double)t.watt_hours - c.base_wh;
      const double d_whc = (double)t.watt_hours_charged - c.base_whc;
      if (d_wh < -(double)TRIP_COUNTER_RESET_WH || d_whc < -(double)TRIP_COUNTER_RESET_WH) {
        c.counter_resets++;  // VESC rebooted: counters restarted, nothing to add, re-baseline below
        c.cur_gap = true;    // this step's energy (and the VESC's own boot-time energy) is unknown
      } else {
        const double a = d_wh > 0.0 ? d_wh : 0.0;  // monotonic counters: a tiny negative is noise
        const double b = d_whc > 0.0 ? d_whc : 0.0;
        // Totals anchored to the counters: whatever the fallback integrated since the anchor
        // (STATUS_3 missing for a while) is replaced by the counters' own figure, so nothing is
        // counted twice and a CAN outage is caught up. In steady state this is "+= delta".
        c.wh = c.anchor_wh + a;
        c.wh_charged = c.anchor_whc + b;
        if (c.prev_counters) {
          d_net = a - b;  // the delta spans exactly this step
        } else if (prev_active && r1 && dt) {
          // Fallback -> counters: the window already holds the fallback's view of the missing
          // seconds; this step's share comes from v_in x current_in as well (fresh, no gap).
          d_net = trip_step_wh(t, dt);
        } else {
          c.cur_gap = true;  // catch-up after an outage: this step's own share is inseparable from the lump
        }
      }
    } else if (r1 && dt) {
      // First STATUS_3 ever while the fallback was in use: the counters only get their baseline
      // now, so this step still comes from v_in x current_in (fresh: the fallback ran a tick ago).
      d_net = trip_integrate_power(c, t, dt);
    }
    c.base_wh = (double)t.watt_hours;
    c.base_whc = (double)t.watt_hours_charged;
    c.anchor_wh = c.wh;
    c.anchor_whc = c.wh_charged;
    c.have_baseline = true;
    c.from_counters = true;
    c.energy_active = true;
  } else if (r1) {
    // Baseline and anchor are kept: when STATUS_3 returns the counters overwrite this estimate.
    if (dt) d_net = trip_integrate_power(c, t, dt);
    c.from_counters = false;
    c.energy_active = true;
  } else {
    c.energy_active = false;  // baseline and anchor kept: a CAN outage is caught up from the counters
  }
  c.prev_counters = s3;

  // ---- window: accumulate into the open bucket, close buckets on integrator time ----
  c.cur_dist += d_dist;
  c.cur_wh += d_net;
  if (dt) {
    if (clamped || !gnss_ok || !c.energy_active) c.cur_gap = true;
    c.bucket_ms += dt;
  }
  while (c.bucket_ms >= 1000u) {
    c.ring_dist[c.head] = c.cur_dist;
    c.ring_wh[c.head] = c.cur_wh;
    c.ring_gap[c.head] = c.cur_gap;
    c.head = (uint8_t)((c.head + 1u) % (unsigned)EFF_WINDOW_S);
    if (c.filled < EFF_WINDOW_S) c.filled++;
    c.bucket_ms -= 1000u;
    c.cur_dist = 0.0;
    c.cur_wh = 0.0;
    // After a clamped step the buckets it closes beyond the first, and the partial
    // remainder, hold no data for time that did pass: mark them as gaps.
    c.cur_gap = clamped && c.bucket_ms > 0u;
  }

  // ---- derived output ----
  double win_dist = 0.0, win_wh = 0.0;
  unsigned gaps = 0;
  for (unsigned i = 0; i < c.filled; ++i) {  // slots 0..filled-1 are the valid ones until the ring is full
    win_dist += c.ring_dist[i];
    win_wh += c.ring_wh[i];
    if (c.ring_gap[i]) gaps++;
  }
  const double net = c.wh - c.wh_charged;

  out = TripState{};
  out.t_ms = now_ms ? now_ms : 1u;
  out.run_s = c.run_s;
  out.moving_s = c.moving_s;
  out.dist_m = (float)c.dist_m;
  out.wh = (float)c.wh;
  out.wh_charged = (float)c.wh_charged;
  out.win_dist_m = (float)win_dist;
  out.win_wh = (float)win_wh;
  out.win_fill_s = c.filled;
  out.win_p_avg_w = c.filled ? (float)(win_wh * 3600.0 / (double)c.filled) : 0.0f;
  out.eff_now = win_dist > 0.0 ? (float)(win_wh / (win_dist / (double)EFF_DIST_UNIT_M)) : 0.0f;
  out.eff_now_valid = c.filled >= TRIP_EFF_MIN_FILL_S && win_dist >= (double)EFF_MIN_DIST_M && gaps == 0 &&
                      gnss_ok && c.energy_active;
  out.eff_avg = c.dist_m > 0.0 ? (float)(net / (c.dist_m / (double)EFF_DIST_UNIT_M)) : 0.0f;
  out.eff_avg_valid = c.dist_m >= (double)EFF_MIN_DIST_M;
  out.energy_from_counters = c.from_counters;
  out.counter_resets = c.counter_resets;
}

// ---------------------------------------------------------------------------
// Task API (implemented in src/trip.cpp; Arduino/FreeRTOS side)
// ---------------------------------------------------------------------------
// Starts the trip integrator task (TASK_PRIO_TRIP, TRIP_TASK_STACK); it runs
// trip_calc_update() every TRIP_PERIOD_MS on a state snapshot and publishes
// g_state.trip. Returns false if the task could not be created.
bool trip_start();
// One log_i line with the trip / efficiency summary (call from the supervisor).
void trip_log_summary(const SharedState &s, uint32_t now);
