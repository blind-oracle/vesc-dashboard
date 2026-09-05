// Thread-safe shared telemetry state and task heartbeats.
//
// Producers (canTask, gnssTask) write under the mutex; consumers (displayTask,
// the supervisor/logger in loop()) copy a snapshot under the mutex and work on
// the copy. Every locked region is a memcpy or a few field writes: no I/O.
//
// This header is Arduino-free (only <stdint.h>) so the pure display-string
// builder and the native unit tests can include it. The implementation
// (src/shared_state.cpp) uses FreeRTOS + millis().
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

// Legacy TWAI driver states (mirrors twai_state_t so this header stays IDF-free).
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
  int state;         // CanState
  uint32_t tec, rec; // transmit / receive error counters
  uint32_t bus_error_count;
  uint32_t rx_missed;  // frames lost because the RX queue was full (driver counter)
  uint32_t rx_overrun; // hardware FIFO overruns
  uint32_t arb_lost;
  uint32_t bus_off_count;      // BUS_OFF alerts seen
  uint32_t recoveries;         // successful bus-off recoveries
  uint32_t queue_full_events;  // RX_QUEUE_FULL alerts
  uint32_t err_passive_events; // ERR_PASS alerts
  uint32_t bus_error_events;   // BUS_ERROR alerts
  uint32_t last_health_ms;     // time of the last twai_get_status_info() snapshot
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
  uint8_t hour, min, sec; // UTC time of the last fix
  bool time_valid;        // NAV-PVT valid.validTime && validDate
  uint32_t good_frames;   // checksum-valid UBX frames
  uint32_t bad_frames;    // checksum failures / oversized frames
  uint32_t redetects;     // times the task fell back to autobaud
};

// ---------------------------------------------------------------- Display
struct DisplayStats
{
  // generic (both display types)
  uint32_t refreshes;         // frames pushed to the panel (any kind)
  uint32_t skipped_unchanged; // ticks where nothing changed
  uint32_t last_change_ms;
  bool dimmed;  // OLED: contrast lowered after OLED_IDLE_DIM_MS without changes
  bool init_ok; // controller answered at init
  // e-ink specific (zero on the OLED build)
  uint32_t partials; // partial refreshes since the last full refresh
  uint32_t partials_total;
  uint32_t fulls;
  uint32_t last_full_ms;
  uint32_t busy_timeouts;
  bool hibernating;
  bool powered_off;
};

struct SharedState
{
  VescState vesc;
  CanHealth can;
  GnssState gnss;
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
// Plain millis() stamps written by each task; read by the supervisor.
extern volatile uint32_t hb_can;
extern volatile uint32_t hb_gnss;
extern volatile uint32_t hb_disp;

// millis() wrapper so headers stay Arduino-free.
uint32_t state_now_ms();

inline void hb_touch(volatile uint32_t &hb) { hb = state_now_ms(); }

// True if t_ms != 0 and it is at most max_age ms old (wrap-safe).
inline bool fresh(uint32_t t_ms, uint32_t now, uint32_t max_age)
{
  return t_ms != 0 && (uint32_t)(now - t_ms) <= max_age;
}

// Age in ms of a timestamp (UINT32_MAX if never set).
inline uint32_t age_ms(uint32_t t_ms, uint32_t now) { return t_ms ? (uint32_t)(now - t_ms) : 0xFFFFFFFFu; }
