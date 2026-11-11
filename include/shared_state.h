// Thread-safe shared telemetry state and task heartbeats.
//
// Producers (canTask, gnssTask) write under the mutex; consumers (displayTask,
// the supervisor/logger in loop()) copy a snapshot under the mutex and work on
// the copy. Every locked region is a memcpy or a few field writes: no I/O.
//
// This header is framework-free (only <stdint.h>) so the pure display-string
// builder and the native unit tests can include it. The implementation
// (src/shared_state.cpp) uses FreeRTOS + esp_timer.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "vesc_mcconf.h"
#include "vesc_status.h"

// ---------------------------------------------------------------- VESC / CAN
struct VescState
{
  vesc_telemetry_t t;       // raw decoded fields + per-status timestamps t_ms[6]
  float i_in_ema;           // smoothed STATUS_4 current_in (CAN_EMA_ALPHA)
  float i_motor_ema;        // smoothed STATUS_1 current_motor
  float v_in_ema;           // smoothed STATUS_5 v_in
  bool ema_init;            // first sample seeds the EMAs
  int locked_id;            // controller id we follow (-1 until the first frame when VESC_CAN_ID == -1)
  uint32_t frames_total;    // accepted status frames
  uint32_t frames_other_id; // status frames from a different controller id (ignored)
  uint32_t frames_dropped;  // extended frames with unexpected DLC / non-status ids
  uint32_t last_frame_ms;   // time of the last accepted frame (0 = never)
};

// Values only available by actively polling the VESC (COMM_GET_VALUES_SELECTIVE),
// see can_vesc.cpp / vesc_getvalues.h. All zero until the first complete reply.
struct VescExt {
  float temp_mos1, temp_mos2, temp_mos3; // per-MOSFET temperatures, degC
  float avg_motor_current;   // A (average, unfiltered)
  float avg_input_current;   // A
  float avg_id, avg_iq;      // d/q axis currents, A
  float vd, vq;              // d/q axis voltages, V
  int32_t tacho_abs;         // absolute tachometer steps
  uint8_t fault_code;        // mc_fault_code, 0 = FAULT_CODE_NONE
  uint8_t status;            // bit0 timeout active, bit1 kill switch active
  uint8_t vesc_id;           // controller id reported in the reply
  uint32_t t_ms;             // time of the last complete, CRC-valid reply (0 = never)
  uint8_t last_fault;        // latched: last non-zero fault code seen (the VESC clears the live byte ~500 ms after a fault)
  uint32_t last_fault_ms;    // when it was seen (0 = never)
  uint32_t polls_sent;       // requests transmitted
  uint32_t replies_ok;       // complete replies decoded
  uint32_t replies_bad;      // CRC / format failures
  uint32_t timeouts;         // requests without a complete reply within VESC_POLL_TIMEOUT_MS
};

// CAN node lifecycle as canTask sees it (IDF-free; the values are also what the
// display strings and the tests use): UNINSTALLED = no node, STOPPED = node
// created but disabled, RUNNING = enabled and on the bus, BUS_OFF = the node
// went bus-off, RECOVERING = recovery started, waiting for error-active again.
enum CanState : int
{
  CAN_STATE_UNINSTALLED = -1,
  CAN_STATE_STOPPED = 0,
  CAN_STATE_RUNNING = 1,
  CAN_STATE_BUS_OFF = 2,
  CAN_STATE_RECOVERING = 3,
};

struct CanHealth
{
  int state;                   // CanState
  uint32_t tec, rec;           // transmit / receive error counters (twai_node_get_info)
  uint32_t bus_error_count;    // cumulative bus errors since the node was enabled (driver record)
  uint32_t rx_missed;          // frames dropped because the ISR -> canTask queue was full
  uint32_t arb_lost;           // arbitration-lost events (on_error callback)
  uint32_t bus_off_count;      // bus-off events
  uint32_t recoveries;         // completed bus-off recoveries (node error-active again)
  uint32_t err_passive_events; // transitions into the error-passive state
  uint32_t last_health_ms;     // time of the last twai_node_get_info() snapshot
};

// ---------------------------------------------------------------- GNSS
enum GnssPhase : uint8_t
{
  GNSS_PHASE_AUTOBAUD = 0,
  GNSS_PHASE_DETECT = 1,
  GNSS_PHASE_CONFIGURE = 2,
  GNSS_PHASE_RUN = 3,
};

struct GnssState
{
  uint8_t phase;          // GnssPhase
  uint32_t baud;          // detected UART baud (0 = none yet)
  int prot_ver_x100;      // UBX protocol version * 100 from MON-VER (-1 unknown, e.g. 1800 = 18.00)
  bool configured;        // configuration messages were acknowledged
  bool use_velned;        // receiver has no NAV-PVT (u-blox 6): speed comes from NAV-VELNED
  uint32_t last_pvt_ms;   // time of the last valid NAV-PVT / NAV-VELNED (0 = never)
  uint8_t fix_type;       // NAV-PVT fixType: 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
  uint8_t flags;          // NAV-PVT flags (bit0 gnssFixOK, only meaningful for PROTVER >= 20)
  uint8_t num_sv;         // satellites used
  bool fix_ok;            // fixType in {2,3,4} (and gnssFixOK when PROTVER >= 20)
  int32_t gspeed_mm_s;    // ground speed, mm/s (signed as transmitted)
  uint32_t sacc_mm_s;     // speed accuracy estimate, mm/s
  uint16_t pdop_x100;     // position DOP * 100
  int32_t lat_e7, lon_e7; // degrees * 1e7
  int32_t height_mm;             // height above ellipsoid, mm
  int32_t hmsl_mm;               // height above mean sea level, mm
  uint32_t hacc_mm, vacc_mm;     // horizontal / vertical accuracy estimates, mm
  int32_t head_mot_e5;           // heading of motion, degrees * 1e5
  int32_t vel_d_mm_s;            // down velocity, mm/s
  uint16_t year;                 // UTC date of the last fix
  uint8_t month, day;
  uint8_t hour, min, sec; // UTC time of the last fix
  bool time_valid;        // NAV-PVT valid.validTime && validDate
  uint32_t good_frames;   // checksum-valid UBX frames
  uint32_t bad_frames;    // checksum failures / oversized frames
  uint32_t redetects;     // times the task fell back to autobaud
};

// ---------------------------------------------------------------- Trip / efficiency
// Written by the trip integrator task (src/trip.cpp) from snapshots of the VESC
// and GNSS state; read by the EFFICIENCY screen and the logs.
struct TripState {
  uint32_t t_ms;             // last integrator update (0 = never)
  uint32_t run_s;            // seconds since the integrator started
  uint32_t moving_s;         // seconds with ground speed >= EFF_MIN_SPEED_MM_S
  float dist_m;              // trip distance, m (GNSS ground speed integrated while moving)
  float wh;                  // trip energy drawn from the battery, Wh
  float wh_charged;          // trip energy returned to the battery (regen), Wh
  float win_dist_m;          // distance over the last EFF_WINDOW_S seconds
  float win_wh;              // net energy over the last EFF_WINDOW_S seconds
  float win_p_avg_w;         // average electrical power over the window
  float eff_now;             // net Wh per EFF_DIST_UNIT_M over the window (valid: eff_now_valid)
  float eff_avg;             // net Wh per EFF_DIST_UNIT_M since startup (valid: eff_avg_valid)
  bool eff_now_valid;        // window distance >= EFF_MIN_DIST_M and data fresh
  bool eff_avg_valid;        // trip distance >= EFF_MIN_DIST_M
  bool energy_from_counters; // true = VESC STATUS_3 watt-hour counters, false = v_in x current_in integration fallback
  uint8_t win_fill_s;        // seconds of data in the window (0..EFF_WINDOW_S)
  uint32_t counter_resets;   // VESC watt-hour counter resets detected (VESC rebooted)
};

// ---------------------------------------------------------------- Display
struct DisplayStats
{
  uint32_t refreshes;         // frames pushed to the panel
  uint32_t skipped_unchanged; // ticks where nothing changed
  uint32_t last_change_ms;
  bool dimmed;                   // contrast lowered after OLED_IDLE_DIM_MS without changes
  bool init_ok; // controller answered at init
  uint8_t screen;                // currently shown screen index (0 = main)
  uint32_t button_presses;       // debounced presses seen
  uint8_t ota_refused;           // OtaRefusal of the last refused update-mode hold (0 = none)
  uint32_t ota_refused_ms;       // when it was refused (0 = never): the refusal shows for OTA_REFUSED_SHOW_MS
  uint32_t pump_ack;             // PumpStatus::tach_alarms the helmsman dismissed with a press (the pump alarm overlay)
};

#if OTA_ENABLE
// ---------------------------------------------------------------- firmware update (update mode)
// Written by src/ota.cpp (its HTTP handler, the Wi-Fi event handler and ota_tick()) while the
// board is in update mode; read by the FW UPDATE screen and the supervisor's log line. All
// zero (OTA_PHASE_OFF) in normal operation.
enum OtaPhase : uint8_t {
  OTA_PHASE_OFF = 0,    // normal operation
  OTA_PHASE_STARTING,   // update mode: bringing up Wi-Fi
  OTA_PHASE_WAITING,    // access point started (or failed to: ap_up), no upload in progress
  OTA_PHASE_RECEIVING,  // an upload is being written to the spare slot
  OTA_PHASE_DONE,       // written, verified and selected for boot: rebooting into it
  OTA_PHASE_FAILED,     // the last upload was rejected (detail says why); another one may follow
};

struct OtaStatus {
  uint8_t phase;              // OtaPhase
  bool ap_up;                 // WIFI_EVENT_AP_START seen, no AP_STOP since
  uint8_t stations;           // phones joined right now (the AP admits one)
  char ssid[20];              // OTA_AP_SSID
  char pass[9];               // this session's 8-digit WPA2 passphrase
  char ip[16];                // "192.168.4.1"
  char run_ver[18];           // version of the running image (esp_app_desc, i.e. git describe)
  char new_ver[18];           // version of the image being received ("" until its header arrived)
  char detail[24];            // why the last upload failed
  uint32_t received, total;   // bytes of the upload in progress (or the last one)
  uint32_t idle_deadline_ms;  // update mode reboots to normal at this state_now_ms() unless an upload runs
};
#endif

#if PUMP_ENABLE
// ---------------------------------------------------------------- coolant pump
// The VESC's temperature limits as the link task (canTask / vescUartTask, the only writer)
// last read them with COMM_GET_MCCONF (include/vesc_mcconf.h). The pump task uses them only
// with result == MCCONF_OK and falls back to the PUMP_DEFAULT_* values otherwise.
struct VescLimitsState {
  VescTempLimits lim;  // valid with result == MCCONF_OK
  uint8_t result;      // VescMcconfResult of the last reply (MCCONF_NONE = never answered)
  uint32_t sig;        // MCCONF_SIGNATURE of the last reply (0 = none)
  char fw[8];          // firmware label of a known layout ("6.06"), "" otherwise
  uint32_t t_ms;       // when the last reply arrived (0 = never)
  uint32_t requests;   // COMM_GET_MCCONF requests sent
};

// Written by the pump task (src/pump.cpp); read by the PUMP screen and the log. The decision
// logic is pure and lives in include/pump_logic.h.
enum PumpMode : uint8_t {
  PUMP_MODE_OFF = 0,  // stopped: no VESC temperatures since boot, or cool, still and the after-run over
  PUMP_MODE_RUN,      // closed loop, PUMP_MIN_DUTY_PCT..100 %
  PUMP_MODE_FULL,     // a temperature reached the VESC's derating start (100 %), or the tach says it stalled (PUMP_TACH_STALL_DUTY_PCT)
  PUMP_MODE_BLIND,    // temperatures were seen, then went stale (or none reads plausibly): PUMP_BLIND_DUTY_PCT
  PUMP_MODE_FORCED,   // PUMP_FORCE_DUTY_PCT (commissioning)
  PUMP_MODE_FAULT,    // the PWM output could not be set up: the pin floats, the hardware runs the pump flat out
};

// Why the pump is in its mode (log line and screen).
enum PumpWhy : uint8_t {
  PUMP_WHY_NONE = 0,
  PUMP_WHY_NO_DATA,    // OFF: no VESC temperatures since boot (VESC off at the dock)
  PUMP_WHY_COOL,       // OFF: cool, motor still, after-run over
  PUMP_WHY_WARM,       // RUN: a temperature is within PUMP_ON_BAND_C of its target, or above it
  PUMP_WHY_TURNING,    // RUN: the motor turns
  PUMP_WHY_AFTER_RUN,  // RUN: neither, but the after-run / hysteresis keeps it going
  PUMP_WHY_DERATE,     // FULL: at or above the VESC's derating start
  PUMP_WHY_STALE,      // BLIND: the temperatures stopped arriving
  PUMP_WHY_NO_SENSOR,  // BLIND: fresh data, but neither temperature is plausible
  PUMP_WHY_FORCED,     // FORCED
  PUMP_WHY_PWM,        // FAULT
  PUMP_WHY_STALLED,    // FULL: the tach says the pump does not turn although it should (PUMP_TACH_STALL)
  PUMP_WHY_NO_TACH,    // FULL: no tach pulse since boot although the pump should turn (PUMP_TACH_NO_SIGNAL)
};

// What the pump's tachometer (FG wire, include/pump_tach.h) says about the pump.
enum PumpTachState : uint8_t {
  PUMP_TACH_NONE = 0,   // no tach in this build (PIN_PUMP_TACH -1), or the task has not ticked yet
  PUMP_TACH_WAIT,       // no verdict: spin-up / spin-down, or a duty between 0 and PUMP_MIN_DUTY_PCT
  PUMP_TACH_OK,         // turning while it should, or still while it should be stopped
  PUMP_TACH_STALL,      // should run, pulses were seen since boot, but below PUMP_TACH_MIN_RPM now: ALARM
  PUMP_TACH_NO_SIGNAL,  // should run, and not one pulse since boot: the FG wire or a dead pump: ALARM
  PUMP_TACH_RUNS_AT_0,  // commanded to 0 %, but it turns: the PWM stage does not stop it (full speed)
  PUMP_TACH_ERR,        // the pulse counter could not be set up
};

struct PumpStatus {
  uint8_t mode;                  // PumpMode
  uint8_t why;                   // PumpWhy
  bool limits_vesc;              // targets from the VESC's limits (false = the PUMP_DEFAULT_* values)
  bool have_err;                 // err_c is meaningful (fresh, plausible temperatures)
  float duty_pct;                // commanded duty, 0..100
  float err_c;                   // max over the plausible sensors of (temperature - target), C
  float integ;                   // integral term, % duty
  float tgt_mot, tgt_fet;        // targets, C
  float derate_mot, derate_fet;  // where the VESC starts cutting acceleration current, C
  uint32_t t_ms;                 // last pump task tick (0 = never)
  // The tachometer (all zero / PUMP_TACH_NONE without PUMP_TACH_ENABLE)
  uint8_t tach;                  // PumpTachState
  bool tach_full;                // tach_rpm averages a full PUMP_TACH_WINDOW_MS
  float tach_rpm;                // pump rpm over the window (pulses / PUMP_TACH_PPR)
  uint32_t tach_pulses;          // FG pulses counted since boot
  uint32_t tach_alarms;          // entries into STALL / NO_SIGNAL since boot: the alarm overlay's sequence number
  uint32_t tach_since_ms;        // when the tach state was entered (0 = never)
};
#endif  // PUMP_ENABLE

#if DEADMAN_UI_ENABLE
// ---------------------------------------------------------------- dead-man's switch
// Written by deadmanTask (src/deadman.cpp) from the beacon timestamps the BLE host
// task stamps (src/ble_scan.cpp); read by the DEADMAN screen, the MAIN screen's cut
// banner and the log.
// The decision logic itself is pure and lives in include/deadman_logic.h.
enum DmState : uint8_t {
  DM_WAIT_TAG = 0,   // the tag has not been seen yet: no protection (see DEADMAN_BOOT_CUT)
  DM_ARMED,          // tag fresh, motor permitted
  DM_GRACE,          // tag quiet past DEADMAN_WARN_MS, still permitted, screen counting down
  DM_TRIPPED,        // timed out: cut asserted and latched until a reset with the tag present
  DM_UNAVAILABLE,    // the radio is not delivering adverts: no protection, and we do NOT cut for it
  DM_FAULT,          // the cut was asserted but the VESC never reported its kill switch active
};

// Result of the closed loop against VescExt::status bit 1 (VESC_STATUS_KILL_SW).
enum DmConfirm : uint8_t {
  DM_CONFIRM_UNKNOWN = 0, // not asserting a cut
  DM_CONFIRM_WAIT,        // asserted, still inside DEADMAN_CONFIRM_MS
  DM_CONFIRM_OK,          // the VESC reports its kill switch active
  DM_CONFIRM_FAILED,      // it never did, while polls were arriving -> DM_FAULT
  DM_CONFIRM_STALE,       // no fresh poll reply, so nothing can be concluded (never a FAULT)
};

// One tag slot as the screens and the log see it. Up to DEADMAN_TAGS of these; a slot whose
// address is empty or unparsable is inert (configured == false) and is simply not shown.
struct DeadmanTag {
  uint32_t t_ms;       // last advert from THIS tag (0 = never)
  uint32_t reports;    // adverts from this tag since boot
  uint32_t gap_max_ms; // longest gap between ITS OWN adverts while enrolled - never the gap in the
                       // union of both tags, which would under-report and make the README's
                       // "size the timeout at 5x gapmax" advice unsafe
  int8_t rssi;
  bool configured;     // a usable address is set for this slot
  bool enrolled;       // it was present when the enrolment window closed, so it guards the motor
};

struct DeadmanState {
  uint8_t state;             // DmState
  uint8_t confirm;           // DmConfirm
  bool cut;                  // PIN_DEADMAN_CUT is being pulled low right now
  bool enrol_open;           // the enrolment window is still open (a second tag can still join)
  uint8_t enrolled_count;    // tags currently guarding the motor
  uint32_t state_since_ms;   // when the state was entered (0 = never)
  DeadmanTag tags[DEADMAN_TAGS];
  uint32_t adv_other;        // non-matching adverts seen (a crowded-marina indicator)
  uint32_t gap_max_ms;       // max over the enrolled tags' own gaps: the number that sizes DEADMAN_TIMEOUT_MS
  uint32_t trips, resets, resets_refused;
  uint32_t publish_skipped;  // state_lock() misses from the deadman task
};
#endif  // DEADMAN_UI_ENABLE

struct SharedState
{
  VescState vesc;
  VescExt vesc_ext;
  CanHealth can;
  GnssState gnss;
  TripState trip;
#if DEADMAN_UI_ENABLE
  DeadmanState deadman;
#endif
#if OTA_ENABLE
  OtaStatus ota;
#endif
#if PUMP_ENABLE
  VescLimitsState vesc_limits;
  PumpStatus pump;
#endif
  DisplayStats disp;
  uint32_t lock_failures; // state_lock() timeouts (should stay 0)
};

extern SharedState g_state;

void state_init();
// Acquire the state mutex. Returns false on timeout (and counts it).
bool state_lock(uint32_t timeout_ms = 50);
// Non-blocking variant that does NOT touch the shared failure counter: for the dead-man
// task, which runs above every other task and must never block or write a counter it
// races everyone for. A miss is counted in DeadmanState::publish_skipped instead.
bool state_try_lock();
void state_unlock();
// Copy of g_state taken under the mutex (on lock failure returns the previous snapshot).
SharedState state_snapshot();

// ---------------------------------------------------------------- Heartbeats
// Plain state_now_ms() stamps written by each task; read by the supervisor.
extern volatile uint32_t hb_can;
extern volatile uint32_t hb_gnss;
extern volatile uint32_t hb_disp;
extern volatile uint32_t hb_ble;   // ble task, dead-man builds (logged and checked by the dead-man; never gates the watchdog)
#if DEADMAN_ENABLE
extern volatile uint32_t hb_deadman;  // deadmanTask: safety-relevant, so it ALWAYS gates the watchdog (HB_MAX_DEADMAN_MS)
#endif
#if PUMP_ENABLE
extern volatile uint32_t hb_pump;  // pumpTask (HB_MAX_PUMP_MS)
#endif

// Milliseconds since boot (32-bit, wraps like millis() did); esp_timer underneath, so headers stay framework-free.
uint32_t state_now_ms();

inline void hb_touch(volatile uint32_t &hb) { hb = state_now_ms(); }

// True if t_ms != 0 and it is at most max_age ms old (wrap-safe).
inline bool fresh(uint32_t t_ms, uint32_t now, uint32_t max_age)
{
  return t_ms != 0 && (uint32_t)(now - t_ms) <= max_age;
}

// Age in ms of a timestamp (UINT32_MAX if never set).
inline uint32_t age_ms(uint32_t t_ms, uint32_t now) { return t_ms ? (uint32_t)(now - t_ms) : 0xFFFFFFFFu; }
