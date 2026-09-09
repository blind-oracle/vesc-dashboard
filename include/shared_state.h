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
};

#if BMS_UI_ENABLE
// ---------------------------------------------------------------- BMS (JK BMS over BLE)
// Written by bmsTask (src/bms_ble.cpp) after every decoded cell-info frame; read by the
// BMS / CELLS screens and the logs. Present when the BLE client is built or in the demo.
enum BmsLink : uint8_t {
  BMS_LINK_OFF = 0,     // BLE not started / init failed
  BMS_LINK_SCANNING,    // looking for the BMS (or waiting out a back-off)
  BMS_LINK_CONNECTING,  // connect request in flight
  BMS_LINK_SETUP,       // connected: MTU, discovery, subscribe, 0x97/0x96 sent, waiting for the first frame
  BMS_LINK_STREAM,      // cell-info frames arriving
};

enum BmsProto : uint8_t { BMS_PROTO_UNKNOWN = 0, BMS_PROTO_JK02_24S = 1, BMS_PROTO_JK02_32S = 2 };

struct BmsState {
  uint32_t t_ms;                  // last decoded cell-info frame (0 = never)
  uint8_t link;                   // BmsLink
  uint8_t proto;                  // BmsProto (0 while unknown / implausible)
  uint8_t cell_count;             // non-zero cells in the frame (<= 32; only the first BMS_CELLS_MAX are stored)
  uint16_t cell_mv[BMS_CELLS_MAX];
  uint16_t cell_min_mv, cell_max_mv, cell_avg_mv, cell_delta_mv;
  uint8_t cell_min_idx, cell_max_idx; // 1-based, 0 = none (computed from non-zero cells)
  uint32_t pack_mv;
  int32_t current_ma;             // JK sign: positive = charging (BMS_CURRENT_SIGN flips it for the screens)
  int32_t power_mw;               // pack_mv x current_ma, same sign as current_ma
  int16_t t1_d, t2_d, mos_d;      // temperatures, 0.1 degC, signed
  uint8_t soc_pct, soh_pct;
  uint32_t remaining_mah, nominal_mah, cycle_count;
  int16_t balance_ma;
  uint8_t balance_action;
  bool chg_mos_on, dis_mos_on;
  uint32_t errors;                // alarm bitmask (16 valid bits on 24S, 32 on 32S)
  char model[17], sw[9];          // from the device-info frame ("" until received)
  char addr[18];                  // peer address "aa:bb:cc:dd:ee:ff" ("" until seen)
  int8_t rssi;
  uint16_t mtu;
  uint8_t last_disc_reason;
  uint32_t link_since_ms;         // when the current link state was entered (0 = never)
  uint32_t frames_ok, frames_crc_bad, frames_resync, notify_dropped, connects, disconnects;
};
#endif  // BMS_UI_ENABLE

struct SharedState
{
  VescState vesc;
  VescExt vesc_ext;
  CanHealth can;
  GnssState gnss;
  TripState trip;
#if BMS_UI_ENABLE
  BmsState bms;
#endif
  DisplayStats disp;
  uint32_t lock_failures; // state_lock() timeouts (should stay 0)
};

extern SharedState g_state;

void state_init();
// Acquire the state mutex. Returns false on timeout (and counts it).
bool state_lock(uint32_t timeout_ms = 50);
void state_unlock();
// Copy of g_state taken under the mutex (on lock failure returns the previous snapshot).
SharedState state_snapshot();

// ---------------------------------------------------------------- Heartbeats
// Plain state_now_ms() stamps written by each task; read by the supervisor.
extern volatile uint32_t hb_can;
extern volatile uint32_t hb_gnss;
extern volatile uint32_t hb_disp;
extern volatile uint32_t hb_bms;   // bmsTask (logged; gates the watchdog only when HB_MAX_BMS_MS > 0)

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
