// Clean-room decoder for the VESC CAN "status" broadcast frames (STATUS_1..6).
// Framework-agnostic: depends only on <stdint.h>/<stdbool.h>, so it compiles in
// the native unit tests as well as on the ESP32.
//
// Wire format (verified against vedderb/bldc master, comm/comm_can.c):
//   29-bit extended ID: eid = (packet_id << 8) | controller_id
//   All multi-byte fields are big-endian two's-complement, DLC = 8.
//
//   pkt 9  STATUS   : [0..3] i32 erpm            [4..5] i16 current_motor*10  [6..7] i16 duty*1000
//   pkt 14 STATUS_2 : [0..3] i32 amp_hours*1e4   [4..7] i32 amp_hours_charged*1e4
//   pkt 15 STATUS_3 : [0..3] i32 watt_hours*1e4  [4..7] i32 watt_hours_charged*1e4
//   pkt 16 STATUS_4 : [0..1] i16 temp_fet*10     [2..3] i16 temp_motor*10     [4..5] i16 current_in*10  [6..7] i16 pid_pos*50
//   pkt 27 STATUS_5 : [0..3] i32 tachometer      [4..5] i16 v_in*10           [6..7] reserved
//                     (wire order is tacho THEN v_in, unlike the VESC's can_status_msg_5 struct)
//   pkt 58 STATUS_6 : [0..1] i16 adc1*1000       [2..3] i16 adc2*1000         [4..5] i16 adc3*1000      [6..7] i16 ppm*1000
//
// Notes: erpm is ELECTRICAL rpm (mechanical = erpm / (poles/2)). STATUS_1 current
// is the filtered q-axis motor current (signed). STATUS_4 current_in is the
// battery (bus) current. Electrical power = v_in * current_in.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VESC_CAN_PACKET_STATUS 9u
#define VESC_CAN_PACKET_STATUS_2 14u
#define VESC_CAN_PACKET_STATUS_3 15u
#define VESC_CAN_PACKET_STATUS_4 16u
#define VESC_CAN_PACKET_STATUS_5 27u
#define VESC_CAN_PACKET_STATUS_6 58u

#define VESC_EID_CONTROLLER_ID(eid) ((uint8_t)((eid) & 0xFFu))
#define VESC_EID_PACKET_ID(eid) ((uint32_t)((eid) >> 8))

// Index into vesc_telemetry_t::t_ms for each status message.
enum {
  VESC_IDX_STATUS_1 = 0,
  VESC_IDX_STATUS_2 = 1,
  VESC_IDX_STATUS_3 = 2,
  VESC_IDX_STATUS_4 = 3,
  VESC_IDX_STATUS_5 = 4,
  VESC_IDX_STATUS_6 = 5,
  VESC_STATUS_COUNT = 6
};

typedef struct {
  uint8_t id;          // controller id of the last decoded frame
  float erpm;          // STATUS_1: electrical rpm
  float current_motor; // STATUS_1: motor (q-axis) current, A, signed
  float duty;          // STATUS_1: duty cycle, -1..1
  float amp_hours;     // STATUS_2: Ah drawn
  float amp_hours_charged;
  float watt_hours;    // STATUS_3: Wh drawn
  float watt_hours_charged;
  float temp_fet;      // STATUS_4: MOSFET temperature, degC
  float temp_motor;    // STATUS_4: motor temperature, degC (implausible when no NTC is wired)
  float current_in;    // STATUS_4: battery/bus current, A, signed
  float pid_pos;       // STATUS_4: position, deg
  int32_t tachometer;  // STATUS_5: tachometer steps (revolutions = steps / (3 * poles))
  float v_in;          // STATUS_5: input voltage, V
  float adc1, adc2, adc3; // STATUS_6: external ADC inputs, V
  float ppm;           // STATUS_6: PPM/servo input, ~-1..1
  uint32_t t_ms[VESC_STATUS_COUNT]; // time of the last STATUS_1..6, 0 = never received
} vesc_telemetry_t;

static inline int16_t vesc_be_i16(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline int32_t vesc_be_i32(const uint8_t *p) {
  return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3]);
}

// Maps a packet id to 0..5, or -1 if it is not a status frame.
static inline int vesc_status_index(uint32_t packet_id) {
  switch (packet_id) {
    case VESC_CAN_PACKET_STATUS: return VESC_IDX_STATUS_1;
    case VESC_CAN_PACKET_STATUS_2: return VESC_IDX_STATUS_2;
    case VESC_CAN_PACKET_STATUS_3: return VESC_IDX_STATUS_3;
    case VESC_CAN_PACKET_STATUS_4: return VESC_IDX_STATUS_4;
    case VESC_CAN_PACKET_STATUS_5: return VESC_IDX_STATUS_5;
    case VESC_CAN_PACKET_STATUS_6: return VESC_IDX_STATUS_6;
    default: return -1;
  }
}

static inline bool vesc_is_status_pkt(uint32_t packet_id) { return vesc_status_index(packet_id) >= 0; }

// Decodes one CAN frame into *t. Returns true if the frame was a VESC status
// frame (extended id, DLC >= 8, packet id 9/14/15/16/27/58) and, when
// want_id >= 0, came from that controller id. On success t->id and the
// matching t->t_ms[] are updated (a now_ms of 0 is stored as 1 so that 0 keeps
// meaning "never").
static inline bool vesc_decode_status(vesc_telemetry_t *t, uint32_t eid, bool is_ext, const uint8_t *d, uint8_t dlc,
                                      uint32_t now_ms, int want_id) {
  if (!is_ext || dlc < 8 || d == 0 || t == 0) return false;
  const int idx = vesc_status_index(VESC_EID_PACKET_ID(eid));
  if (idx < 0) return false;
  const uint8_t id = VESC_EID_CONTROLLER_ID(eid);
  if (want_id >= 0 && id != (uint8_t)want_id) return false;

  switch (idx) {
    case VESC_IDX_STATUS_1:
      t->erpm = (float)vesc_be_i32(d + 0);
      t->current_motor = vesc_be_i16(d + 4) / 10.0f;
      t->duty = vesc_be_i16(d + 6) / 1000.0f;
      break;
    case VESC_IDX_STATUS_2:
      t->amp_hours = vesc_be_i32(d + 0) / 10000.0f;
      t->amp_hours_charged = vesc_be_i32(d + 4) / 10000.0f;
      break;
    case VESC_IDX_STATUS_3:
      t->watt_hours = vesc_be_i32(d + 0) / 10000.0f;
      t->watt_hours_charged = vesc_be_i32(d + 4) / 10000.0f;
      break;
    case VESC_IDX_STATUS_4:
      t->temp_fet = vesc_be_i16(d + 0) / 10.0f;
      t->temp_motor = vesc_be_i16(d + 2) / 10.0f;
      t->current_in = vesc_be_i16(d + 4) / 10.0f;
      t->pid_pos = vesc_be_i16(d + 6) / 50.0f;
      break;
    case VESC_IDX_STATUS_5:
      t->tachometer = vesc_be_i32(d + 0);   // bytes 0..3 (wire order!)
      t->v_in = vesc_be_i16(d + 4) / 10.0f; // bytes 4..5; 6..7 reserved
      break;
    case VESC_IDX_STATUS_6:
      t->adc1 = vesc_be_i16(d + 0) / 1000.0f;
      t->adc2 = vesc_be_i16(d + 2) / 1000.0f;
      t->adc3 = vesc_be_i16(d + 4) / 1000.0f;
      t->ppm = vesc_be_i16(d + 6) / 1000.0f;
      break;
    default:
      return false;
  }
  t->id = id;
  t->t_ms[idx] = now_ms ? now_ms : 1u;
  return true;
}

// ---- derived values ----
static inline float vesc_power_w(const vesc_telemetry_t *t) { return t->v_in * t->current_in; } // signed, negative = regen
static inline float vesc_mech_rpm(const vesc_telemetry_t *t, int motor_poles) {
  return motor_poles > 0 ? t->erpm / (motor_poles / 2.0f) : 0.0f;
}
static inline float vesc_revolutions(const vesc_telemetry_t *t, int motor_poles) {
  return motor_poles > 0 ? (float)t->tachometer / (3.0f * (float)motor_poles) : 0.0f;
}
static inline float vesc_wh_net(const vesc_telemetry_t *t) { return t->watt_hours - t->watt_hours_charged; }

// True if STATUS message idx (0..5) was received and is at most max_age_ms old.
// Uses unsigned arithmetic so it is correct across the 49.7-day millis() wrap.
static inline bool vesc_fresh(const vesc_telemetry_t *t, int idx, uint32_t now_ms, uint32_t max_age_ms) {
  if (idx < 0 || idx >= VESC_STATUS_COUNT) return false;
  const uint32_t ts = t->t_ms[idx];
  return ts != 0 && (uint32_t)(now_ms - ts) <= max_age_ms;
}

#ifdef __cplusplus
}
#endif
