// Dead-man's switch: the pure decision logic (no FreeRTOS, no IDF, no I/O).
//
// A BLE tag worn by the helmsman is watched by the radio the BMS client owns
// (src/bms_ble.cpp timestamps every matching advert). This header turns that
// timestamp into one boolean - "assert the cut" - and the state the screens and
// the log show. The FreeRTOS task that samples it, drives PIN_DEADMAN_CUT and
// publishes g_state.deadman lives in src/deadman.cpp.
//
// Framework-free (<stdint.h>/<stdbool.h> + config.h + shared_state.h) so the
// native Unity tests (test/test_deadman) can drive the whole state machine with
// synthetic inputs. Everything is inline: no allocation, no locking, no logging.
// All time arithmetic is unsigned and wrap-safe, like fresh() / age_ms().
//
// States (DmState), and what the cut line does in each:
//
//   WAIT_TAG     the tag has not been seen yet. With DEADMAN_BOOT_CUT 0 (default)
//                the motor is permitted: the firmware has nothing to vouch for and
//                fail-passive says it must not be the reason the boat cannot move.
//                The screen says NO TAG, loudly, because there is no protection.
//                With DEADMAN_BOOT_CUT 1 the cut is held instead, so the tag cannot
//                be forgotten - at the price of a dead tag battery immobilising the
//                boat until the dashboard is powered down.
//   ARMED        the tag is fresh. Motor permitted.
//   GRACE        the tag has been quiet for DEADMAN_WARN_MS but not yet for
//                DEADMAN_TIMEOUT_MS. Motor still permitted; the screen counts down.
//   TRIPPED      the timeout expired. The cut is asserted and LATCHED: the tag
//                coming back does not release it, only a deliberate reset with the
//                tag present does (you must not be able to restart the motor while
//                the person is still in the water).
//   UNAVAILABLE  the radio is not delivering adverts at all (the BLE host is stale
//                or not scanning). This does NOT trip: under fail-passive our own
//                failure must not stop the boat, because the mechanical lanyard
//                still covers the real hazard. It must be shown loudly instead.
//   FAULT        the cut was asserted but the VESC never reported its kill switch
//                active within DEADMAN_CONFIRM_MS while its poll replies were
//                arriving. The wiring, the series resistor or the VESC's ADC2
//                configuration is broken. The cut keeps being asserted and the
//                state never clears by itself.
//
// Arming deliberately needs DEADMAN_ARM_REPORTS adverts inside DEADMAN_ARM_WINDOW_MS
// with the last one at least DEADMAN_ARM_RSSI: arming on one -95 dBm packet from the
// car park would mean the first thing that happens when you leave the dock is a trip.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "shared_state.h"
#include "vesc_getvalues.h"  // VESC_STATUS_KILL_SW: the bit the VESC reports its kill switch in
#include "vesc_status.h"     // vesc_fresh() / VESC_IDX_STATUS_1 for the standstill test

#if DEADMAN_UI_ENABLE

// ---------------------------------------------------------------- inputs
// Everything the decision needs, so the tests can drive it without a radio or a VESC.
struct DeadmanInputs {
  uint32_t beacon_t_ms;    // last matching advert (0 = never seen)
  uint32_t beacon_reports; // total matching adverts, monotonic
  int8_t beacon_rssi;      // RSSI of the last one
  bool ble_healthy;        // the BLE host is alive AND a scan is running
  bool reset_req;          // a reset gesture happened since the last call (edge, not level)
  bool poll_fresh;         // a VESC poll reply is fresh (vesc_ext.t_ms within VESC_EXT_STALE_MS)
  uint8_t vesc_status;     // VescExt::status, for the confirmation loop
};

// ---------------------------------------------------------------- state
struct DeadmanCalc {
  uint8_t state;             // DmState
  uint32_t state_since_ms;   // when it was entered (0 = never; stamped like every other producer)
  bool cut;                  // what PIN_DEADMAN_CUT should assert right now

  uint32_t arm_first_ms;     // first advert of the current arming burst (0 = none in progress)
  uint32_t arm_seen;         // adverts in that burst

  uint32_t seen_reports;     // last beacon_reports we acted on (edge detection on new adverts)
  uint32_t prev_advert_ms;   // previous advert time, for the gap statistic
  uint32_t gap_max_ms;       // longest gap between adverts while armed: the number that sizes DEADMAN_TIMEOUT_MS

  uint32_t cut_since_ms;     // when cut was first asserted (0 = not asserting)
  uint8_t confirm;           // DmConfirm

  uint32_t trips;            // TRIPPED entries
  uint32_t resets;           // accepted resets
  uint32_t resets_refused;   // refused because the tag was not present
};

inline void deadman_init(DeadmanCalc &c) {
  c = DeadmanCalc{};
  c.state = DM_WAIT_TAG;
  c.cut = DEADMAN_BOOT_CUT ? true : false;
  c.confirm = DM_CONFIRM_UNKNOWN;
}

// ---------------------------------------------------------------- helpers
// Age of the last advert. UINT32_MAX when the tag has never been seen, 0 when the
// producer stamped after our clock read (never 49 days).
inline uint32_t deadman_tag_age(const DeadmanInputs &in, uint32_t now) {
  if (in.beacon_t_ms == 0) return 0xFFFFFFFFu;
  const uint32_t age = (uint32_t)(now - in.beacon_t_ms);
  return (int32_t)age < 0 ? 0u : age;
}

inline bool deadman_tag_fresh(const DeadmanInputs &in, uint32_t now, uint32_t within_ms) {
  return deadman_tag_age(in, now) <= within_ms;
}

// ---------------------------------------------------------------- the machine
inline void deadman_update(DeadmanCalc &c, const DeadmanInputs &in, uint32_t now) {
  const uint32_t age = deadman_tag_age(in, now);
  const bool new_advert = in.beacon_reports != c.seen_reports;

  // ---- advert bookkeeping (runs in every state; the gap statistic only while armed)
  if (new_advert) {
    if (c.prev_advert_ms != 0 && (c.state == DM_ARMED || c.state == DM_GRACE)) {
      const uint32_t gap = (uint32_t)(in.beacon_t_ms - c.prev_advert_ms);
      if ((int32_t)gap > 0 && gap > c.gap_max_ms) c.gap_max_ms = gap;
    }
    c.prev_advert_ms = in.beacon_t_ms;
    c.seen_reports = in.beacon_reports;
  }

  const uint8_t was = c.state;

  switch (c.state) {
    case DM_WAIT_TAG:
    case DM_UNAVAILABLE: {
      if (!in.ble_healthy) {
        // No radio: no protection. Never trip on our own failure (fail-passive).
        // WAIT_TAG keeps its boot cut; UNAVAILABLE never asserts one of its own.
        if (c.state == DM_WAIT_TAG) break;
        c.state = DM_UNAVAILABLE;
        break;
      }
      if (c.state == DM_UNAVAILABLE) {  // radio back: re-arm from scratch, the tag must prove itself again
        c.state = DM_WAIT_TAG;
        c.arm_first_ms = 0;
        c.arm_seen = 0;
      }
      // Arming burst: DEADMAN_ARM_REPORTS adverts inside DEADMAN_ARM_WINDOW_MS, last one strong enough.
      if (new_advert) {
        if (c.arm_first_ms == 0 || (uint32_t)(now - c.arm_first_ms) > (uint32_t)DEADMAN_ARM_WINDOW_MS) {
          c.arm_first_ms = now ? now : 1u;  // 0 means "no burst in progress"
          c.arm_seen = 1;
        } else {
          c.arm_seen++;
        }
        if (c.arm_seen >= (uint32_t)DEADMAN_ARM_REPORTS && in.beacon_rssi >= (int8_t)DEADMAN_ARM_RSSI) {
          c.state = DM_ARMED;
          c.arm_first_ms = 0;
          c.arm_seen = 0;
          c.gap_max_ms = 0;  // the statistic covers this armed period only
        }
      }
      break;
    }

    case DM_ARMED:
    case DM_GRACE: {
      if (!in.ble_healthy) {  // the radio died under us: no protection, but do not stop the boat
        c.state = DM_UNAVAILABLE;
        break;
      }
      if (age >= (uint32_t)DEADMAN_TIMEOUT_MS) {
        c.state = DM_TRIPPED;
        c.trips++;
      } else if (age >= (uint32_t)DEADMAN_WARN_MS) {
        c.state = DM_GRACE;
      } else {
        c.state = DM_ARMED;
      }
      break;
    }

    case DM_TRIPPED: {
      // Latched. Only a deliberate reset with the tag actually present releases it.
      if (in.reset_req) {
        if (deadman_tag_fresh(in, now, (uint32_t)DEADMAN_RESET_FRESH_MS)) {
          c.resets++;
          c.state = DM_ARMED;
          c.gap_max_ms = 0;
        } else {
          c.resets_refused++;
        }
      }
      break;
    }

    case DM_FAULT:
    default:
      break;  // never clears by itself: the cut stays on until the board is restarted and the wiring fixed
  }

  // ---- the output. Only TRIPPED and FAULT assert a cut; WAIT_TAG may hold the boot cut.
  const bool want_cut = (c.state == DM_TRIPPED) || (c.state == DM_FAULT) ||
                        (c.state == DM_WAIT_TAG && DEADMAN_BOOT_CUT);
  if (want_cut && !c.cut) c.cut_since_ms = now ? now : 1u;
  if (!want_cut) {
    c.cut_since_ms = 0;
    c.confirm = DM_CONFIRM_UNKNOWN;
  }
  c.cut = want_cut;

  // ---- closed loop: the VESC must agree that its kill switch is active.
  // "Not confirmed" only counts while poll replies are arriving - a dead CAN bus
  // must never raise a false FAULT.
#if DEADMAN_CONFIRM_ENABLE
  if (c.cut) {
    if (!in.poll_fresh) {
      c.confirm = DM_CONFIRM_STALE;
    } else if (in.vesc_status & VESC_STATUS_KILL_SW) {
      c.confirm = DM_CONFIRM_OK;
    } else if (c.cut_since_ms != 0 && (uint32_t)(now - c.cut_since_ms) >= (uint32_t)DEADMAN_CONFIRM_MS) {
      c.confirm = DM_CONFIRM_FAILED;
      c.state = DM_FAULT;  // keeps asserting; the screen and the log say the cut did not take
    } else {
      c.confirm = DM_CONFIRM_WAIT;
    }
  }
#endif

  if (c.state != was) c.state_since_ms = now ? now : 1u;
}

// ---------------------------------------------------------------- BMS radio gate
// A BLE connect attempt blinds the beacon watchdog for the whole attempt (NimBLE keeps
// one master context: ble_gap_connect() and ble_gap_disc() exclude each other). So the
// BMS may only start one when the motor cannot be running, and when the tag is fresh
// enough that the blind window cannot itself cause a trip.
//
// "Standstill" means no source that is currently reporting says we are moving: a stale
// VESC or a stale GNSS must not by itself forbid the link, but a fresh one that says
// "moving" must. Always allowed while TRIPPED - the motor is already cut, so reading
// the BMS then is harmless and useful.
// Reads the PUBLISHED state (g_state.deadman), not the task's own DeadmanCalc, so
// src/bms_ble.cpp can call it without reaching into another module's internals. The copy
// is at most DEADMAN_TICK_MS old, which is nothing against a multi-second margin.
inline bool deadman_connect_allowed(const SharedState &s, uint32_t now, uint32_t connect_window_ms) {
  const uint8_t st = s.deadman.state;
  if (st == DM_TRIPPED || st == DM_FAULT) return true;
  if (st != DM_ARMED) return false;  // never open a blind window from GRACE: the margin is already shrinking

  const uint32_t age = (uint32_t)(now - s.deadman.beacon_t_ms);
  if (s.deadman.beacon_t_ms == 0 || (uint32_t)DEADMAN_TIMEOUT_MS <= age) return false;
  if ((uint32_t)DEADMAN_TIMEOUT_MS - age <= connect_window_ms) return false;  // the window would eat the margin

#if DEADMAN_STANDSTILL_ERPM >= 0
  if (vesc_fresh(&s.vesc.t, VESC_IDX_STATUS_1, now, VESC_STALE_R1_MS)) {
    const float erpm = s.vesc.t.erpm < 0 ? -s.vesc.t.erpm : s.vesc.t.erpm;
    if (erpm > (float)DEADMAN_STANDSTILL_ERPM) return false;
  }
#endif
  if (fresh(s.gnss.last_pvt_ms, now, GNSS_STALE_MS) && s.gnss.fix_ok) {
    const int32_t sp = s.gnss.gspeed_mm_s < 0 ? -s.gnss.gspeed_mm_s : s.gnss.gspeed_mm_s;
    if (sp > (int32_t)DEADMAN_STANDSTILL_MM_S) return false;
  }
  return true;
}

// ---------------------------------------------------------------- names (log + screens)
inline const char *deadman_state_str(uint8_t st) {
  switch (st) {
    case DM_WAIT_TAG: return "NO TAG";
    case DM_ARMED: return "ARMED";
    case DM_GRACE: return "GRACE";
    case DM_TRIPPED: return "TRIPPED";
    case DM_UNAVAILABLE: return "NO RADIO";
    case DM_FAULT: return "FAULT";
    default: return "?";
  }
}

inline const char *deadman_confirm_str(uint8_t cf) {
  switch (cf) {
    case DM_CONFIRM_UNKNOWN: return "-";
    case DM_CONFIRM_WAIT: return "WAIT";
    case DM_CONFIRM_OK: return "OK";
    case DM_CONFIRM_FAILED: return "FAILED";
    case DM_CONFIRM_STALE: return "STALE";
    default: return "?";
  }
}

#endif  // DEADMAN_UI_ENABLE
