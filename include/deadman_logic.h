// Dead-man's switch: the pure decision logic (no FreeRTOS, no IDF, no I/O).
//
// Up to DEADMAN_TAGS BLE tags worn by the crew are watched by the radio the BMS client
// owns (src/bms_ble.cpp timestamps every matching advert). This header turns those
// timestamps into one boolean - "assert the cut" - and the state the screens and the log
// show. The FreeRTOS task that samples it, drives PIN_DEADMAN_CUT and publishes
// g_state.deadman lives in src/deadman.cpp.
//
// Framework-free (<stdint.h>/<stdbool.h> + config.h + shared_state.h) so the native Unity
// tests (test/test_deadman) can drive the whole state machine with synthetic inputs.
// Everything is inline: no allocation, no locking, no logging. All time arithmetic is
// unsigned and wrap-safe, like fresh() / age_ms().
//
// WHICH TAGS COUNT is decided once, when protection begins. The first tag to complete its
// arming burst enrols and opens a DEADMAN_ENROL_MS window; any other configured tag that
// arms inside that window joins; then the set is frozen for the run. A tag that turns up
// afterwards - the one left in the car, still in range at the dock - can never join and
// silently keep the switch alive. A radio outage throws the set away: we lost track, so
// every tag must prove itself again.
//
// WITH TWO TAGS ENROLLED the cut comes only when BOTH have gone quiet: the machine works
// on the age of the FRESHEST enrolled tag. Either crew member aboard keeps the motor
// running. That is weaker than requiring both - a tag left on the boat by someone who
// then goes over the side keeps it running too - and is the rule that was asked for.
//
// States (DmState), and what the cut line does in each:
//
//   WAIT_TAG     no tag has enrolled yet. With DEADMAN_BOOT_CUT 0 (default) the motor is
//                permitted: the firmware has nothing to vouch for and fail-passive says it
//                must not be the reason the boat cannot move. The screen says so. With
//                DEADMAN_BOOT_CUT 1 the cut is held instead, so a tag cannot be forgotten,
//                at the price of a flat tag battery immobilising the boat.
//   ARMED        at least one enrolled tag is fresh. Motor permitted.
//   GRACE        every enrolled tag has been quiet for DEADMAN_WARN_MS but not yet for
//                DEADMAN_TIMEOUT_MS. Motor still permitted; the screen counts down.
//   TRIPPED      the timeout expired. The cut is asserted and LATCHED: a tag coming back
//                does not release it, only a deliberate reset with a tag present does (you
//                must not be able to restart the motor while the person is still in the
//                water). A radio outage while TRIPPED does NOT release it either.
//   UNAVAILABLE  the radio is not delivering adverts at all. This does NOT trip: under
//                fail-passive our own failure must not stop the boat, because the
//                mechanical lanyard still covers the real hazard. Shown loudly instead.
//   FAULT        the cut was asserted but the VESC never reported its kill switch active
//                within DEADMAN_CONFIRM_MS while its poll replies were arriving. The
//                wiring, the series resistor or the VESC's ADC2 configuration is broken.
//                The cut keeps being asserted and the state never clears by itself.
//
// Arming is PER TAG and deliberately needs DEADMAN_ARM_REPORTS adverts from that one tag
// inside DEADMAN_ARM_WINDOW_MS with its last one at least DEADMAN_ARM_RSSI. Sharing one
// burst counter between tags would let two tags each contribute half a burst and arm a
// system where neither is actually aboard.
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
  struct Tag {
    uint32_t t_ms;    // last advert from this tag (0 = never seen)
    uint32_t reports; // adverts from this tag, monotonic (edge-detects a new one)
    int8_t rssi;      // RSSI of the last one
    bool configured;  // a usable address is set for this slot
  } tag[DEADMAN_TAGS];
  bool ble_healthy;    // the BLE host is alive AND a scan is running
  bool reset_req;      // a reset gesture happened since the last call (edge, not level)
  bool poll_fresh;     // a VESC poll reply is fresh (vesc_ext.t_ms within VESC_EXT_STALE_MS)
  uint8_t vesc_status; // VescExt::status, for the confirmation loop
};

// ---------------------------------------------------------------- state
struct DeadmanCalc {
  uint8_t state;           // DmState
  uint32_t state_since_ms; // when it was entered (0 = never; stamped like every other producer)
  bool cut;                // what PIN_DEADMAN_CUT should assert right now

  struct Tag {
    uint32_t arm_first_ms;   // first advert of this tag's arming burst (0 = none in progress)
    uint32_t arm_seen;       // adverts in that burst
    uint32_t seen_reports;   // last reports value we acted on
    uint32_t prev_advert_ms; // previous advert FROM THIS TAG, for its own gap statistic
    uint32_t gap_max_ms;     // longest gap between its own adverts while enrolled
    bool enrolled;           // it guards the motor
  } tag[DEADMAN_TAGS];

  uint32_t enrol_close_ms; // when the enrolment window closes (0 = never opened)
  bool enrol_open;         // a second tag can still join

  uint32_t cut_since_ms; // when cut was first asserted (0 = not asserting)
  uint8_t confirm;       // DmConfirm

  uint32_t trips;          // TRIPPED entries
  uint32_t resets;         // accepted resets
  uint32_t resets_refused; // refused because no enrolled tag was present
};

inline void deadman_init(DeadmanCalc &c) {
  c = DeadmanCalc{};
  c.state = DM_WAIT_TAG;
  c.cut = DEADMAN_BOOT_CUT ? true : false;
  c.confirm = DM_CONFIRM_UNKNOWN;
}

// ---------------------------------------------------------------- helpers
// Age of ONE tag's last advert. UINT32_MAX when it has never been seen, 0 when the
// producer stamped after our clock read (never 49 days).
inline uint32_t deadman_tag_age(const DeadmanInputs &in, unsigned i, uint32_t now) {
  if (i >= DEADMAN_TAGS || in.tag[i].t_ms == 0) return 0xFFFFFFFFu;
  const uint32_t age = (uint32_t)(now - in.tag[i].t_ms);
  return (int32_t)age < 0 ? 0u : age;
}

inline unsigned deadman_enrolled_count(const DeadmanCalc &c) {
  unsigned n = 0;
  for (unsigned i = 0; i < DEADMAN_TAGS; ++i)
    if (c.tag[i].enrolled) n++;
  return n;
}

// Age of the FRESHEST enrolled tag - the number the timeout works on, i.e. "how long since
// anyone the switch is guarding was last heard". UINT32_MAX when nothing is enrolled, which
// is why the state machine only reaches this once enrolled_count > 0. Restricting the
// aggregate to enrolled tags is also what keeps a configured-but-absent tag harmless: it is
// not in the set, so it can neither trip nor block a trip.
inline uint32_t deadman_age(const DeadmanCalc &c, const DeadmanInputs &in, uint32_t now) {
  uint32_t best = 0xFFFFFFFFu;
  for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
    if (!c.tag[i].enrolled) continue;
    const uint32_t age = deadman_tag_age(in, i, now);
    if (age < best) best = age;
  }
  return best;
}

// The same aggregate over the PUBLISHED state, for consumers outside the task (the screens,
// the log and the BMS radio gate). Kept next to deadman_age() so the two cannot drift.
inline uint32_t deadman_pub_age(const DeadmanState &d, uint32_t now) {
  uint32_t best = 0xFFFFFFFFu;
  for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
    if (!d.tags[i].enrolled || d.tags[i].t_ms == 0) continue;
    const uint32_t ms = (uint32_t)(now - d.tags[i].t_ms);
    const uint32_t age = (int32_t)ms < 0 ? 0u : ms;
    if (age < best) best = age;
  }
  return best;
}

// Throw the enrolment set and every burst away: used when the radio comes back, because
// while it was down we could not tell which tags were still aboard.
inline void deadman_forget_tags(DeadmanCalc &c) {
  for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
    c.tag[i].enrolled = false;
    c.tag[i].arm_first_ms = 0;
    c.tag[i].arm_seen = 0;
  }
  c.enrol_open = false;
  c.enrol_close_ms = 0;
}

// ---------------------------------------------------------------- the machine
inline void deadman_update(DeadmanCalc &c, const DeadmanInputs &in, uint32_t now) {
  const uint8_t was = c.state;

  // ---- radio health first, so the per-tag work below never runs on a stale set.
  // TRIPPED and FAULT are deliberately immune: losing the radio must not release a latch.
  if (!in.ble_healthy) {
    if (c.state == DM_ARMED || c.state == DM_GRACE) c.state = DM_UNAVAILABLE;
  } else if (c.state == DM_UNAVAILABLE) {
    c.state = DM_WAIT_TAG;
    deadman_forget_tags(c);
  }

  // ---- per-tag advert bookkeeping and arming bursts.
  // A tag may still enrol while the window is open, which is how a second tag joins.
  const bool may_enrol = in.ble_healthy && (c.state == DM_WAIT_TAG || ((c.state == DM_ARMED || c.state == DM_GRACE) &&
                                                                       c.enrol_open));
  for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
    DeadmanCalc::Tag &t = c.tag[i];
    if (!in.tag[i].configured) continue;
    if (in.tag[i].reports == t.seen_reports) continue;  // no new advert from this tag
    t.seen_reports = in.tag[i].reports;

    // Its OWN gap, measured only while it is enrolled and the machine is live.
    if (t.prev_advert_ms != 0 && t.enrolled && (c.state == DM_ARMED || c.state == DM_GRACE)) {
      const uint32_t gap = (uint32_t)(in.tag[i].t_ms - t.prev_advert_ms);
      if ((int32_t)gap > 0 && gap > t.gap_max_ms) t.gap_max_ms = gap;
    }
    t.prev_advert_ms = in.tag[i].t_ms;

    if (t.enrolled || !may_enrol) continue;
    // Arming burst, per tag: DEADMAN_ARM_REPORTS of ITS adverts inside
    // DEADMAN_ARM_WINDOW_MS, and ITS last one strong enough.
    if (t.arm_first_ms == 0 || (uint32_t)(now - t.arm_first_ms) > (uint32_t)DEADMAN_ARM_WINDOW_MS) {
      t.arm_first_ms = now ? now : 1u;  // 0 means "no burst in progress"
      t.arm_seen = 1;
    } else {
      t.arm_seen++;
    }
    if (t.arm_seen >= (uint32_t)DEADMAN_ARM_REPORTS && in.tag[i].rssi >= (int8_t)DEADMAN_ARM_RSSI) {
      t.enrolled = true;
      t.arm_first_ms = 0;
      t.arm_seen = 0;
      t.gap_max_ms = 0;  // its statistic covers this enrolled period only
    }
  }

  // ---- the enrolment window: opened by the first tag to enrol, frozen when it closes.
  const unsigned enrolled = deadman_enrolled_count(c);
  if (enrolled > 0 && c.enrol_close_ms == 0) {
    c.enrol_close_ms = now ? now : 1u;
    c.enrol_close_ms += (uint32_t)DEADMAN_ENROL_MS;
    if (c.enrol_close_ms == 0) c.enrol_close_ms = 1u;
    c.enrol_open = true;
  }
  if (c.enrol_open && (int32_t)(now - c.enrol_close_ms) >= 0) c.enrol_open = false;

  // ---- the state machine
  switch (c.state) {
    case DM_WAIT_TAG:
      if (enrolled > 0) c.state = DM_ARMED;
      break;

    case DM_ARMED:
    case DM_GRACE: {
      const uint32_t age = deadman_age(c, in, now);
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

    case DM_TRIPPED:
      // Latched. Only a deliberate reset with an enrolled tag actually present releases it.
      if (in.reset_req) {
        if (deadman_age(c, in, now) <= (uint32_t)DEADMAN_RESET_FRESH_MS) {
          c.resets++;
          c.state = DM_ARMED;
          for (unsigned i = 0; i < DEADMAN_TAGS; ++i) c.tag[i].gap_max_ms = 0;
        } else {
          c.resets_refused++;
        }
      }
      break;

    case DM_UNAVAILABLE:
    case DM_FAULT:
    default:
      break;  // FAULT never clears by itself: the cut stays until the board is restarted
  }

  // ---- the output. Only TRIPPED and FAULT assert a cut; WAIT_TAG may hold the boot cut.
  const bool want_cut =
      (c.state == DM_TRIPPED) || (c.state == DM_FAULT) || (c.state == DM_WAIT_TAG && DEADMAN_BOOT_CUT);
  if (want_cut && !c.cut) c.cut_since_ms = now ? now : 1u;
  if (!want_cut) {
    c.cut_since_ms = 0;
    c.confirm = DM_CONFIRM_UNKNOWN;
  }
  c.cut = want_cut;

  // ---- closed loop: the VESC must agree that its kill switch is active.
  // "Not confirmed" only counts while poll replies are arriving - a dead CAN bus must
  // never raise a false FAULT.
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
// A BLE connect attempt blinds the beacon watchdog for the whole attempt (NimBLE keeps one
// master context: ble_gap_connect() and ble_gap_disc() exclude each other). So the BMS may
// only start one when the motor cannot be running, and when the freshest enrolled tag has
// enough margin that the blind window cannot itself cause a trip.
//
// "Standstill" means no source that is currently reporting says we are moving: a stale VESC
// or a stale GNSS must not by itself forbid the link, but a fresh one that says "moving"
// must. Always allowed while TRIPPED or FAULT - the motor is already cut, so reading the
// BMS then is harmless and useful.
//
// Reads the PUBLISHED state (g_state.deadman), not the task's own DeadmanCalc, so
// src/bms_ble.cpp can call it without reaching into another module's internals. The copy is
// at most DEADMAN_TICK_MS old, which is nothing against a multi-second margin.
inline bool deadman_connect_allowed(const SharedState &s, uint32_t now, uint32_t connect_window_ms) {
  const uint8_t st = s.deadman.state;
  if (st == DM_TRIPPED || st == DM_FAULT) return true;
  if (st != DM_ARMED) return false;  // never open a blind window from GRACE: the margin is already shrinking

  const uint32_t age = deadman_pub_age(s.deadman, now);
  if (age >= (uint32_t)DEADMAN_TIMEOUT_MS) return false;               // includes "nothing enrolled" (UINT32_MAX)
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
