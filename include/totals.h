// Lifetime totals (pure part): motor running hours, powered hours, GNSS distance and energy,
// summed over every boot and kept in flash.
//
// Lifetime = the record loaded at boot (base) + what the trip integrator (include/trip.h)
// counted since: TripState.motor_s / on_s / dist_m / wh / wh_charged. Nothing is integrated
// twice, and a VESC reboot or a counter catch-up is handled where the trip handles it.
//
// When to save (totals_step), checked on every trip tick, first match wins:
//   * nothing differs from the record in flash -> never (no write is ever wasted);
//   * less than TOTALS_MIN_GAP_S since the last attempt (or boot) -> not yet (a waiting stop
//     save is kept, not dropped);
//   * the last write failed -> only TOTALS_RETRY_S after it (RETRY);
//   * the motor stopped and has been still for TOTALS_STOP_SAVE_MS -> STOP. A reversal through
//     0 rpm is shorter and is no stop. A normal shutdown comes after a stop, so it loses nothing;
//   * running with TOTALS_SAVE_RUN_S of unsaved motor time -> RUN (bounds what a power cut
//     while running loses);
//   * TOTALS_SAVE_IDLE_S since the last attempt (or boot) -> IDLE (powered time at the dock,
//     distance while drifting, a preset that is not in flash yet).
//
// The record is a 24-byte blob of little-endian u32: version, then the TotalsRecord fields.
// Fields are only ever appended: a shorter (older) blob reads with the missing fields 0, a
// longer (newer) one with the fields this firmware knows.
//
// Framework-free (<stdint.h> + shared_state.h) so the native Unity tests (test/test_totals)
// can drive it. The NVS side lives in src/totals.cpp.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "shared_state.h"

#if TOTALS_ENABLE

constexpr uint32_t kTotalsVersion = 1;
constexpr size_t kTotalsFields = 5;                          // TotalsRecord members
constexpr size_t kTotalsBlobSize = 4u * (1u + kTotalsFields);  // version + fields
constexpr size_t kTotalsBlobMax = 64;                        // the largest blob read back (a newer layout)

static_assert(sizeof(TotalsRecord) == 4u * kTotalsFields, "TotalsRecord must be kTotalsFields x uint32_t");
static_assert((double)TOTALS_MOTOR_PRESET_H >= 0.0 && (double)TOTALS_MOTOR_PRESET_H < 1000000.0,
              "TOTALS_MOTOR_PRESET_H must be 0..999999 hours");

struct TotalsCalc {
  TotalsRecord base;        // lifetime before this boot (the loaded record, preset applied)
  TotalsRecord saved;       // what is in flash: the loaded record (0s if none), then the last write
  bool running;             // the motor turned at the previous step
  bool stop_pending;        // it stopped since: save once still for TOTALS_STOP_SAVE_MS
  bool failed;              // the last write failed
  uint32_t still_since_ms;  // when it stopped (valid with stop_pending)
  uint32_t last_write_ms;   // last write attempt, or boot: the gap, idle and retry timers
};

// ---- record helpers ------------------------------------------------------------------------
inline uint32_t totals_field(const TotalsRecord &r, size_t i) {
  switch (i) {
    case 0: return r.motor_s;
    case 1: return r.on_s;
    case 2: return r.dist_m;
    case 3: return r.wh;
    default: return r.whc;
  }
}

inline void totals_set_field(TotalsRecord &r, size_t i, uint32_t v) {
  switch (i) {
    case 0: r.motor_s = v; break;
    case 1: r.on_s = v; break;
    case 2: r.dist_m = v; break;
    case 3: r.wh = v; break;
    default: r.whc = v; break;
  }
}

inline bool totals_equal(const TotalsRecord &a, const TotalsRecord &b) {
  for (size_t i = 0; i < kTotalsFields; ++i)
    if (totals_field(a, i) != totals_field(b, i)) return false;
  return true;
}

inline void totals_put_u32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

inline uint32_t totals_get_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// out must hold kTotalsBlobSize bytes.
inline void totals_encode(const TotalsRecord &r, uint8_t *out) {
  totals_put_u32(out, kTotalsVersion);
  for (size_t i = 0; i < kTotalsFields; ++i) totals_put_u32(out + 4u * (i + 1u), totals_field(r, i));
}

// False for anything that is not a record: shorter than version + one field, not whole u32s,
// or version 0. *version gets the blob's version (> kTotalsVersion = written by newer firmware).
inline bool totals_decode(const uint8_t *in, size_t len, TotalsRecord &out, uint32_t *version) {
  out = TotalsRecord{};
  if (version) *version = 0;
  if (in == nullptr || len < 8u || len % 4u != 0u) return false;
  const uint32_t v = totals_get_u32(in);
  if (v == 0u) return false;
  if (version) *version = v;
  const size_t n = len / 4u - 1u;
  for (size_t i = 0; i < kTotalsFields && i < n; ++i) totals_set_field(out, i, totals_get_u32(in + 4u * (i + 1u)));
  return true;
}

inline uint32_t totals_sat_add(uint32_t a, uint32_t b) { return a > UINT32_MAX - b ? UINT32_MAX : a + b; }

// A trip float to whole units: rounded, negative (and NaN) -> 0, saturated.
inline uint32_t totals_round(float v) {
  if (!(v > 0.0f)) return 0u;
  const double d = (double)v + 0.5;
  return d >= 4294967295.0 ? UINT32_MAX : (uint32_t)d;
}

inline uint32_t totals_preset_s() { return (uint32_t)((double)TOTALS_MOTOR_PRESET_H * 3600.0 + 0.5); }

// ---- state machine ---------------------------------------------------------------------------
// loaded / have_record: what NVS gave at boot. now: boot time (the gap and idle timers start here).
inline void totals_init(TotalsCalc &c, const TotalsRecord &loaded, bool have_record, uint32_t now) {
  c = TotalsCalc{};
  if (have_record) c.saved = loaded;
  c.base = c.saved;
  const uint32_t preset = totals_preset_s();
  if (c.base.motor_s < preset) c.base.motor_s = preset;  // never lowers what was counted
  c.last_write_ms = now;
}

// Lifetime values right now: base + this boot.
inline TotalsRecord totals_now(const TotalsCalc &c, const TripState &t) {
  TotalsRecord r;
  r.motor_s = totals_sat_add(c.base.motor_s, t.motor_s);
  r.on_s = totals_sat_add(c.base.on_s, t.on_s);
  r.dist_m = totals_sat_add(c.base.dist_m, totals_round(t.dist_m));
  r.wh = totals_sat_add(c.base.wh, totals_round(t.wh));
  r.whc = totals_sat_add(c.base.whc, totals_round(t.wh_charged));
  return r;
}

// One trip tick: rec = the lifetime values; returns why they should be written now
// (TOTALS_SAVE_NONE = not now). The caller writes and then reports with totals_done().
inline uint8_t totals_step(TotalsCalc &c, const TripState &t, uint32_t now, TotalsRecord &rec) {
  rec = totals_now(c, t);
  if (t.turning) {
    c.running = true;
    c.stop_pending = false;  // still turning, or a reversal through 0 rpm that was too short
  } else if (c.running) {
    c.running = false;
    c.stop_pending = true;
    c.still_since_ms = now;
  }
  if (totals_equal(rec, c.saved)) {
    c.stop_pending = false;  // nothing to save (a stop with no new whole second)
    return TOTALS_SAVE_NONE;
  }
  const uint32_t since = now - c.last_write_ms;
  if (since < (uint32_t)TOTALS_MIN_GAP_S * 1000u) return TOTALS_SAVE_NONE;
  if (c.failed) return since >= (uint32_t)TOTALS_RETRY_S * 1000u ? TOTALS_SAVE_RETRY : TOTALS_SAVE_NONE;
  if (c.stop_pending && (uint32_t)(now - c.still_since_ms) >= (uint32_t)TOTALS_STOP_SAVE_MS) return TOTALS_SAVE_STOP;
  if (c.running && rec.motor_s >= c.saved.motor_s && rec.motor_s - c.saved.motor_s >= (uint32_t)TOTALS_SAVE_RUN_S)
    return TOTALS_SAVE_RUN;
  if (since >= (uint32_t)TOTALS_SAVE_IDLE_S * 1000u) return TOTALS_SAVE_IDLE;
  return TOTALS_SAVE_NONE;
}

// The write of rec (from totals_step) finished at now; ok = it is in flash.
inline void totals_done(TotalsCalc &c, const TotalsRecord &rec, bool ok, uint32_t now) {
  c.last_write_ms = now;
  c.failed = !ok;
  if (!ok) return;  // a pending stop stays pending: the retry carries it
  c.saved = rec;
  c.stop_pending = false;
}

inline const char *totals_reason_str(uint8_t r) {
  switch (r) {
    case TOTALS_SAVE_STOP: return "stop";
    case TOTALS_SAVE_RUN: return "run";
    case TOTALS_SAVE_IDLE: return "idle";
    case TOTALS_SAVE_RETRY: return "retry";
    default: return "none";
  }
}

inline const char *totals_nvs_str(uint8_t n) {
  switch (n) {
    case TOTALS_NVS_OK: return "ok";
    case TOTALS_NVS_NONE: return "new";
    default: return "ERR";
  }
}

#endif  // TOTALS_ENABLE

// ---------------------------------------------------------------------------
// Task API (src/totals.cpp; no-ops with TOTALS_ENABLE 0)
// ---------------------------------------------------------------------------
// Brings NVS up and reads the record. app_main calls it once, before the BLE controller and
// the CAN node exist: NVS init may finish an interrupted write (a flash write).
void totals_load();
// One trip tick (the trip task, outside the state lock): decides, writes the record when due,
// and fills *out with what to publish into g_state.totals.
void totals_tick(const TripState &t, uint32_t now, TotalsState *out);
// Supervisor: a line per save (debug) or failed save (error), and the periodic summary.
void totals_log_events(const SharedState &s, uint32_t now);
void totals_log_summary(const SharedState &s, uint32_t now);
