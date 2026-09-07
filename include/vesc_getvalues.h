// Clean-room codec for ACTIVE polling of a VESC over CAN: COMM_GET_VALUES /
// COMM_GET_VALUES_SELECTIVE request building, reassembly of the fragmented reply
// (FILL_RX_BUFFER / FILL_RX_BUFFER_LONG / PROCESS_RX_BUFFER / PROCESS_SHORT_BUFFER)
// and decoding of the mask-ordered payload into VescExt (+ optionally the
// vesc_telemetry_t fields the reply also carries).
//
// Framework-agnostic: only <stdint.h>/<stdbool.h>/<string.h> plus the project's
// Arduino-free headers, so it compiles in the native unit tests as well as on
// the ESP32. Everything that touches the TWAI driver lives in src/can_vesc.cpp.
//
// Wire protocol (verified against vedderb/bldc master 4b4dd30, FW 7.01,
// comm/comm_can.c comm_can_send_buffer()/decode_msg(), comm/commands.c, util/crc.c):
//
//   REQUEST  one extended frame  eid = (CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | target_id
//            payload [own_id, send = 0, COMM packet...]:
//              COMM_GET_VALUES            [own, 0, 4]                          DLC 3
//              COMM_GET_VALUES_SELECTIVE  [own, 0, 50, mask >> 24 ... mask]    DLC 7  (mask big-endian)
//            send = 0 makes the VESC process the command and answer to own_id with send = 1.
//            The VESC only decodes extended frames whose low eid byte is its own id (or 255).
//
//   REPLY    comm_can_send_buffer(own_id, data, len, 1) on the VESC:
//              len <= 6 : PROCESS_SHORT_BUFFER  eid (8 << 8) | own  [vesc_id, 1, data...]
//              else     : FILL_RX_BUFFER        eid (5 << 8) | own  [offset u8, up to 7 bytes]   offsets 0, 7, 14, ... <= 255
//                         FILL_RX_BUFFER_LONG   eid (6 << 8) | own  [off_hi, off_lo, up to 6 bytes] for offsets > 255
//                         PROCESS_RX_BUFFER     eid (7 << 8) | own  [vesc_id, 1, len_hi, len_lo, crc_hi, crc_lo]
//            crc16 = CRC-16/XMODEM (poly 0x1021, init 0, no reflection) over the reassembled payload only.
//            vesc_id in byte 0 is the VESC's PRIMARY controller id (differs from the polled id on
//            dual-motor hardware when motor 2 was addressed): never reject a reply because of it.
//
//   PAYLOAD  [packet id (4 | 50)] [mask u32 BE, SELECTIVE only] then the fields IN BIT ORDER, each
//            present only if its mask bit is set (plain GET_VALUES = every bit 0..21):
//              bit  0 temp_fet           i16 /10     bit 11 watt_hours          i32 /1e4
//              bit  1 temp_motor         i16 /10     bit 12 watt_hours_charged  i32 /1e4
//              bit  2 avg_motor_current  i32 /100    bit 13 tachometer          i32
//              bit  3 avg_input_current  i32 /100    bit 14 tachometer_abs      i32
//              bit  4 avg_id             i32 /100    bit 15 fault_code          u8
//              bit  5 avg_iq             i32 /100    bit 16 pid_pos             i32 /1e6
//              bit  6 duty               i16 /1000   bit 17 controller_id       u8
//              bit  7 erpm               i32         bit 18 temp_mos1, 2, 3     3 x i16 /10 (ONE bit, 6 bytes)
//              bit  8 v_in               i16 /10     bit 19 vd                  i32 /1000
//              bit  9 amp_hours          i32 /1e4    bit 20 vq                  i32 /1000
//              bit 10 amp_hours_charged  i32 /1e4    bit 21 status              u8 (bit0 timeout, bit1 kill switch)
//            Bit 21 exists from FW 5.03; older firmware echoes the requested mask but omits the
//            field, i.e. the reply is SHORTER than the mask implies: the decoder stops at the end
//            of the data and reports how many fields it got (see vesc_decode_getvalues).
//            All integers are big-endian two's complement (buffer_append_int16/int32).
//
//   CAVEAT  commands_process_packet() on the VESC also points its global send_func at the CAN
//            reply path, so after the first poll any unsolicited output (COMM_PRINT from
//            commands_printf / LispBM print, terminal replies) arrives addressed to own_id
//            through the same FILL/PROCESS frames. Check the packet id byte before decoding.
//
// Semantics worth knowing: bits 2..5, 19, 20 are read-and-reset averages since the
// previous read by ANY client (VESC Tool included), the fault byte is cleared by the
// VESC after its "fault stop time" (default 500 ms) so a 1 Hz poll can miss a fault,
// and temp_mos1..3 read 0.0 on hardware without per-MOSFET sensors.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "shared_state.h"  // VescExt (Arduino-free)
#include "vesc_status.h"   // vesc_be_i16 / vesc_be_i32, vesc_telemetry_t

// ---------------------------------------------------------------- protocol constants
// CAN_PACKET_ID (datatypes.h) used by the request/reply transport.
#define VESC_CAN_PACKET_FILL_RX_BUFFER 5u
#define VESC_CAN_PACKET_FILL_RX_BUFFER_LONG 6u
#define VESC_CAN_PACKET_PROCESS_RX_BUFFER 7u
#define VESC_CAN_PACKET_PROCESS_SHORT_BUFFER 8u
// COMM_PACKET_ID (datatypes.h) carried inside PROCESS_SHORT_BUFFER / the reassembled payload.
#define VESC_COMM_GET_VALUES 4u
#define VESC_COMM_GET_VALUES_SELECTIVE 50u

// eid = (packet_id << 8) | controller_id, same layout as the status frames.
#define VESC_EID(packet_id, controller_id) ((((uint32_t)(packet_id)) << 8) | ((uint32_t)(controller_id) & 0xFFu))
#define VESC_GETVALUES_REQUEST_EID(target_id) VESC_EID(VESC_CAN_PACKET_PROCESS_SHORT_BUFFER, (target_id))

// Mask bits of COMM_GET_VALUES_SELECTIVE (commands.c, case COMM_GET_VALUES_SELECTIVE).
enum {
  VESC_GV_TEMP_FET = 0,
  VESC_GV_TEMP_MOTOR = 1,
  VESC_GV_AVG_MOTOR_CURRENT = 2,
  VESC_GV_AVG_INPUT_CURRENT = 3,
  VESC_GV_AVG_ID = 4,
  VESC_GV_AVG_IQ = 5,
  VESC_GV_DUTY = 6,
  VESC_GV_RPM = 7,
  VESC_GV_V_IN = 8,
  VESC_GV_AMP_HOURS = 9,
  VESC_GV_AMP_HOURS_CHARGED = 10,
  VESC_GV_WATT_HOURS = 11,
  VESC_GV_WATT_HOURS_CHARGED = 12,
  VESC_GV_TACHO = 13,
  VESC_GV_TACHO_ABS = 14,
  VESC_GV_FAULT = 15,
  VESC_GV_PID_POS = 16,
  VESC_GV_VESC_ID = 17,
  VESC_GV_TEMP_MOS123 = 18,
  VESC_GV_VD = 19,
  VESC_GV_VQ = 20,
  VESC_GV_STATUS = 21,
  VESC_GV_BIT_COUNT = 22
};
#define VESC_GV_MASK_ALL 0x003FFFFFu  // every field of a plain COMM_GET_VALUES reply (bits 0..21)
// Fields the STATUS_1..6 broadcasts also carry (updated in vesc_telemetry_t when polled).
#define VESC_GV_MASK_TELEMETRY                                                                                 \
  ((1u << VESC_GV_TEMP_FET) | (1u << VESC_GV_TEMP_MOTOR) | (1u << VESC_GV_DUTY) | (1u << VESC_GV_RPM) |      \
   (1u << VESC_GV_V_IN) | (1u << VESC_GV_AMP_HOURS) | (1u << VESC_GV_AMP_HOURS_CHARGED) |                    \
   (1u << VESC_GV_WATT_HOURS) | (1u << VESC_GV_WATT_HOURS_CHARGED) | (1u << VESC_GV_TACHO) |                 \
   (1u << VESC_GV_PID_POS))

// The mask this firmware requests: exactly the fields that are NOT in any STATUS
// broadcast (bits 2,3,4,5,14,15,17,18,19,20,21) -> 42-byte reply = 6 FILL_RX_BUFFER
// frames + 1 PROCESS_RX_BUFFER. Override with -DVESC_GETVALUES_MASK=0x...; 0 selects
// plain COMM_GET_VALUES (74-byte reply, every field). A VESC with firmware < 5.03
// omits bit 21 (status): its 41-byte replies are still accepted, the status byte
// simply keeps its previous value (0) and can_vesc.cpp warns once.
// (config.h "Advanced task tunables" is the primary definition; this is the fallback.)
#ifndef VESC_GETVALUES_MASK
#define VESC_GETVALUES_MASK 0x003EC03Cu
#endif

// VescExt::status bits (timeout.c: timeout_has_timeout() | timeout_kill_sw_active() << 1).
#define VESC_STATUS_TIMEOUT 0x01u  // no control input for longer than the app "Timeout": timeout brake active
#define VESC_STATUS_KILL_SW 0x02u  // kill switch input asserted

// Reassembly buffer size. The VESC can push at most PACKET_MAX_PL_LEN = 512 bytes
// through comm_can_send_buffer (RX_BUFFER_SIZE on its side); a GET_VALUES reply is
// <= 78 bytes, but mirroring the VESC's limit keeps the FILL_RX_BUFFER_LONG path
// (offsets > 255) exercisable and tolerates longer replies from future firmware.
// (config.h "Advanced task tunables" is the primary definition; this is the fallback.)
#ifndef VESC_RX_BUFFER_SIZE
#define VESC_RX_BUFFER_SIZE 512
#endif

// ---------------------------------------------------------------- CRC
// CRC-16/XMODEM (util/crc.c crc16(): poly 0x1021, init 0x0000, no reflection, no
// final xor). Bitwise, no table: a 78-byte reply costs ~600 iterations, once per poll.
// Check value: vesc_crc16("123456789", 9) == 0x31C3.
static inline uint16_t vesc_crc16(const uint8_t *p, size_t n) {
  uint16_t c = 0;
  for (size_t i = 0; i < n; ++i) {
    c = (uint16_t)(c ^ (uint16_t)((uint16_t)p[i] << 8));
    for (int b = 0; b < 8; ++b) c = (uint16_t)((c & 0x8000u) ? ((uint16_t)(c << 1) ^ 0x1021u) : (uint16_t)(c << 1));
  }
  return c;
}

// ---------------------------------------------------------------- request
// Fills the payload of the PROCESS_SHORT_BUFFER request frame (eid =
// VESC_GETVALUES_REQUEST_EID(target)) and returns its length (DLC): 7 for
// COMM_GET_VALUES_SELECTIVE with the given mask, 3 for plain COMM_GET_VALUES when
// mask == 0. out8 must hold 8 bytes. Never set mask bits above 21: the decoder
// would not know their size and the reply would be rejected.
static inline size_t vesc_build_getvalues_request(uint8_t *out8, uint8_t own_id, uint32_t mask) {
  size_t n = 0;
  out8[n++] = own_id;  // rx_buffer_last_id on the VESC: the reply is addressed to this id
  out8[n++] = 0;       // send = 0: process the command, answer over CAN with send = 1
  if (mask == 0) {
    out8[n++] = (uint8_t)VESC_COMM_GET_VALUES;
  } else {
    out8[n++] = (uint8_t)VESC_COMM_GET_VALUES_SELECTIVE;
    out8[n++] = (uint8_t)(mask >> 24);
    out8[n++] = (uint8_t)(mask >> 16);
    out8[n++] = (uint8_t)(mask >> 8);
    out8[n++] = (uint8_t)mask;
  }
  return n;
}

// True for the four packet ids a reply to our request can arrive with (5..8).
static inline bool vesc_is_getvalues_reply_pkt(uint32_t packet_id) {
  return packet_id >= VESC_CAN_PACKET_FILL_RX_BUFFER && packet_id <= VESC_CAN_PACKET_PROCESS_SHORT_BUFFER;
}

// ---------------------------------------------------------------- reassembly
// One reply in flight. Only frames whose low eid byte is OUR id may be fed (the
// caller filters); frames from/to other nodes never touch the buffer.
struct VescRxBuffer {
  uint8_t data[VESC_RX_BUFFER_SIZE];
  uint16_t len_filled;  // bytes received so far = the offset the next FILL frame must carry
};

static inline void vesc_rx_reset(VescRxBuffer *b) { b->len_filled = 0; }

// Feeds one FILL_RX_BUFFER (is_long = false: frame[0] = offset, 1..7 data bytes) or
// FILL_RX_BUFFER_LONG (is_long = true: frame[0..1] = offset BE, 1..6 data bytes).
// Mirrors decode_msg() on the VESC: offset 0 (re)starts the buffer, an offset equal
// to the bytes filled so far appends, an offset BELOW it is a duplicate of a fragment
// already received (CAN retransmits a frame the receiver already accepted when the
// transmitter sees an error in the last EOF bit) and is ignored - the VESC ignores it
// too, and the CRC in PROCESS_RX_BUFFER still verifies the result. Returns false (and
// resets the buffer) on a gap - a lost frame -, an overflow or a frame without data:
// the whole reply is lost. True means the buffer is still consistent.
static inline bool vesc_rx_fill(VescRxBuffer *b, const uint8_t *frame, uint8_t dlc, bool is_long) {
  const uint8_t hdr = is_long ? 2u : 1u;
  if (dlc <= hdr || dlc > 8) {
    vesc_rx_reset(b);
    return false;
  }
  const uint32_t off = is_long ? (((uint32_t)frame[0] << 8) | frame[1]) : frame[0];
  const uint32_t n = (uint32_t)dlc - hdr;
  if (off == 0) vesc_rx_reset(b);
  if (off != 0 && off < b->len_filled) return true;  // duplicate fragment: keep what we have
  if (off != b->len_filled || off + n > sizeof b->data) {
    vesc_rx_reset(b);
    return false;
  }
  memcpy(b->data + off, frame + hdr, n);
  b->len_filled = (uint16_t)(off + n);
  return true;
}

// Consumes a PROCESS_RX_BUFFER frame [vesc_id, send, len_hi, len_lo, crc_hi, crc_lo]:
// checks that the declared length equals the bytes filled and that the CRC-16
// matches. Returns 1 and points *payload/*len at the reassembled reply (valid until
// the next fill), 0 on a length or CRC mismatch (count it as a bad reply), -1 when
// the frame is not a reply at all (DLC < 6, or send != 1: a command forwarded TO our
// id, which a display never executes). The buffer is reset in every case.
static inline int vesc_rx_process(VescRxBuffer *b, const uint8_t *frame, uint8_t dlc, const uint8_t **payload,
                                  uint16_t *len) {
  const uint16_t filled = b->len_filled;
  vesc_rx_reset(b);
  if (dlc < 6 || frame[1] != 1) return -1;
  const uint16_t declared = (uint16_t)(((uint16_t)frame[2] << 8) | frame[3]);
  const uint16_t crc = (uint16_t)(((uint16_t)frame[4] << 8) | frame[5]);
  if (declared == 0 || declared != filled || vesc_crc16(b->data, declared) != crc) return 0;
  *payload = b->data;
  *len = declared;
  return 1;
}

// ---------------------------------------------------------------- decode
// Wire size of one mask bit's field.
static inline uint16_t vesc_getvalues_field_size(int bit) {
  switch (bit) {
    case VESC_GV_TEMP_FET:
    case VESC_GV_TEMP_MOTOR:
    case VESC_GV_DUTY:
    case VESC_GV_V_IN: return 2;
    case VESC_GV_FAULT:
    case VESC_GV_VESC_ID:
    case VESC_GV_STATUS: return 1;
    case VESC_GV_TEMP_MOS123: return 6;
    default: return 4;
  }
}

// Length of a complete reply payload (packet id + [mask] + fields) for a request
// mask; mask 0 = plain COMM_GET_VALUES (74). Our default mask -> 42; 0x3FFFFF -> 78.
static inline uint16_t vesc_getvalues_expected_len(uint32_t mask) {
  const bool selective = mask != 0;
  if (!selective) mask = VESC_GV_MASK_ALL;
  uint16_t n = selective ? 5 : 1;
  for (int b = 0; b < VESC_GV_BIT_COUNT; ++b)
    if (mask & (1u << b)) n = (uint16_t)(n + vesc_getvalues_field_size(b));
  return n;
}

// Number of fields (set bits 0..21) a request mask asks for; mask 0 = plain
// COMM_GET_VALUES = all 22. Compare with the value vesc_decode_getvalues() returns
// to detect a reply from firmware that does not implement every requested bit.
static inline int vesc_getvalues_field_count(uint32_t mask) {
  if (mask == 0) mask = VESC_GV_MASK_ALL;
  int n = 0;
  for (int b = 0; b < VESC_GV_BIT_COUNT; ++b)
    if (mask & (1u << b)) ++n;
  return n;
}

// True if the first byte of a reassembled payload is a packet id this decoder
// understands (COMM_GET_VALUES / COMM_GET_VALUES_SELECTIVE). Anything else that
// arrives addressed to our id (COMM_PRINT, COMM_FW_VERSION, ...) is unsolicited
// output the VESC routed to its last requester: not a bad reply, just not ours.
static inline bool vesc_is_getvalues_payload(const uint8_t *payload, uint16_t len) {
  return payload != 0 && len >= 1 &&
         (payload[0] == VESC_COMM_GET_VALUES || payload[0] == VESC_COMM_GET_VALUES_SELECTIVE);
}

// Field mask a reply payload announces: the echoed mask for COMM_GET_VALUES_SELECTIVE,
// VESC_GV_MASK_ALL for COMM_GET_VALUES, 0 if the payload is neither (or too short).
static inline uint32_t vesc_getvalues_mask(const uint8_t *payload, uint16_t len) {
  if (payload == 0 || len < 1) return 0;
  if (payload[0] == VESC_COMM_GET_VALUES) return VESC_GV_MASK_ALL;
  if (payload[0] == VESC_COMM_GET_VALUES_SELECTIVE && len >= 5) return (uint32_t)vesc_be_i32(payload + 1);
  return 0;
}

// Decodes a complete, CRC-verified reply payload (first byte = COMM packet id 4 or 50)
// into *out. Fields the reply does not carry are left untouched, so the caller can
// pass its current VescExt and keep older values; out->t_ms and the counters are
// never written here. When also_update is non-null, the fields that the STATUS
// broadcasts also carry (VESC_GV_MASK_TELEMETRY) are written into it as well -
// values only, never its t_ms[] freshness stamps, which stay tied to the broadcasts.
// Returns the number of fields decoded (>= 0). The VESC echoes the REQUESTED mask
// but only appends the fields its firmware knows (bit 21 needs FW >= 5.03), so the
// payload can end before the announced field list does: decoding stops at the last
// complete field and the count is below vesc_getvalues_field_count(mask) - the
// leading fields are valid, the missing ones keep their previous value. A field cut
// in half is never decoded. Returns -1, with *out untouched, if the packet id is
// unknown, the header is incomplete or the mask has bits above 21 (unknown field
// sizes would misalign everything after them). Trailing extra bytes are tolerated.
static inline int vesc_decode_getvalues(const uint8_t *payload, uint16_t len, VescExt *out,
                                        vesc_telemetry_t *also_update_or_null) {
  if (payload == 0 || out == 0 || len < 1) return -1;
  uint32_t mask;
  uint16_t i;
  if (payload[0] == VESC_COMM_GET_VALUES_SELECTIVE) {
    if (len < 5) return -1;
    mask = (uint32_t)vesc_be_i32(payload + 1);
    i = 5;
  } else if (payload[0] == VESC_COMM_GET_VALUES) {
    mask = VESC_GV_MASK_ALL;
    i = 1;
  } else {
    return -1;
  }
  if (mask & ~VESC_GV_MASK_ALL) return -1;

  int nfields = 0;
  vesc_telemetry_t *t = also_update_or_null;
  for (int b = 0; b < VESC_GV_BIT_COUNT; ++b) {
    if (!(mask & (1u << b))) continue;
    const uint16_t sz = vesc_getvalues_field_size(b);
    if ((uint32_t)i + sz > len) break;  // data ends inside the announced list: older firmware
    const uint8_t *p = payload + i;
    switch (b) {
      case VESC_GV_TEMP_FET: if (t) t->temp_fet = vesc_be_i16(p) / 10.0f; break;
      case VESC_GV_TEMP_MOTOR: if (t) t->temp_motor = vesc_be_i16(p) / 10.0f; break;
      case VESC_GV_AVG_MOTOR_CURRENT: out->avg_motor_current = vesc_be_i32(p) / 100.0f; break;
      case VESC_GV_AVG_INPUT_CURRENT: out->avg_input_current = vesc_be_i32(p) / 100.0f; break;
      case VESC_GV_AVG_ID: out->avg_id = vesc_be_i32(p) / 100.0f; break;
      case VESC_GV_AVG_IQ: out->avg_iq = vesc_be_i32(p) / 100.0f; break;
      case VESC_GV_DUTY: if (t) t->duty = vesc_be_i16(p) / 1000.0f; break;
      case VESC_GV_RPM: if (t) t->erpm = (float)vesc_be_i32(p); break;
      case VESC_GV_V_IN: if (t) t->v_in = vesc_be_i16(p) / 10.0f; break;
      case VESC_GV_AMP_HOURS: if (t) t->amp_hours = vesc_be_i32(p) / 10000.0f; break;
      case VESC_GV_AMP_HOURS_CHARGED: if (t) t->amp_hours_charged = vesc_be_i32(p) / 10000.0f; break;
      case VESC_GV_WATT_HOURS: if (t) t->watt_hours = vesc_be_i32(p) / 10000.0f; break;
      case VESC_GV_WATT_HOURS_CHARGED: if (t) t->watt_hours_charged = vesc_be_i32(p) / 10000.0f; break;
      case VESC_GV_TACHO: if (t) t->tachometer = vesc_be_i32(p); break;
      case VESC_GV_TACHO_ABS: out->tacho_abs = vesc_be_i32(p); break;
      case VESC_GV_FAULT: out->fault_code = p[0]; break;
      case VESC_GV_PID_POS: if (t) t->pid_pos = vesc_be_i32(p) / 1000000.0f; break;
      case VESC_GV_VESC_ID: out->vesc_id = p[0]; break;
      case VESC_GV_TEMP_MOS123:
        out->temp_mos1 = vesc_be_i16(p) / 10.0f;
        out->temp_mos2 = vesc_be_i16(p + 2) / 10.0f;
        out->temp_mos3 = vesc_be_i16(p + 4) / 10.0f;
        break;
      case VESC_GV_VD: out->vd = vesc_be_i32(p) / 1000.0f; break;
      case VESC_GV_VQ: out->vq = vesc_be_i32(p) / 1000.0f; break;
      case VESC_GV_STATUS: out->status = p[0]; break;
      default: break;
    }
    i = (uint16_t)(i + sz);
    ++nfields;
  }
  return nfields;
}

// ---------------------------------------------------------------- fault codes
// mc_fault_code (datatypes.h, master: 34 values). 30..33 are newer than FW 6.05.
#define VESC_FAULT_CODE_COUNT 34u

// Short form for the 128x64 display, at most 8 characters. Unknown codes -> "F?"
// (use vesc_fault_fmt() to get "F<code>" instead).
static inline const char *vesc_fault_str(uint8_t code) {
  static const char *const k[VESC_FAULT_CODE_COUNT] = {
      "NONE",      // 0  FAULT_CODE_NONE
      "OVER_V",    // 1  FAULT_CODE_OVER_VOLTAGE
      "UNDER_V",   // 2  FAULT_CODE_UNDER_VOLTAGE
      "DRV",       // 3  FAULT_CODE_DRV (gate driver chip fault)
      "ABS_OC",    // 4  FAULT_CODE_ABS_OVER_CURRENT
      "OT_FET",    // 5  FAULT_CODE_OVER_TEMP_FET
      "OT_MOT",    // 6  FAULT_CODE_OVER_TEMP_MOTOR
      "GATE_OV",   // 7  FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE
      "GATE_UV",   // 8  FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE
      "MCU_UV",    // 9  FAULT_CODE_MCU_UNDER_VOLTAGE
      "WDT_RST",   // 10 FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET
      "ENC_SPI",   // 11 FAULT_CODE_ENCODER_SPI
      "ENC_LOW",   // 12 FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE
      "ENC_HIGH",  // 13 FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE
      "FLASH",     // 14 FAULT_CODE_FLASH_CORRUPTION
      "I_OFFS1",   // 15 FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1
      "I_OFFS2",   // 16 FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2
      "I_OFFS3",   // 17 FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3
      "I_UNBAL",   // 18 FAULT_CODE_UNBALANCED_CURRENTS
      "BRK",       // 19 FAULT_CODE_BRK
      "RSLV_LOT",  // 20 FAULT_CODE_RESOLVER_LOT
      "RSLV_DOS",  // 21 FAULT_CODE_RESOLVER_DOS
      "RSLV_LOS",  // 22 FAULT_CODE_RESOLVER_LOS
      "FLASH_AP",  // 23 FAULT_CODE_FLASH_CORRUPTION_APP_CFG
      "FLASH_MC",  // 24 FAULT_CODE_FLASH_CORRUPTION_MC_CFG
      "ENC_NMAG",  // 25 FAULT_CODE_ENCODER_NO_MAGNET
      "ENC_SMAG",  // 26 FAULT_CODE_ENCODER_MAGNET_TOO_STRONG
      "PH_FILT",   // 27 FAULT_CODE_PHASE_FILTER
      "ENC_FLT",   // 28 FAULT_CODE_ENCODER_FAULT
      "LV_OUT",    // 29 FAULT_CODE_LV_OUTPUT_FAULT
      "ENC_SLIP",  // 30 FAULT_CODE_ENCODER_SLIP
      "OVERSPD",   // 31 FAULT_CODE_OVERSPEED
      "UNDERSPD",  // 32 FAULT_CODE_UNDERSPEED
      "ABS_OSPD",  // 33 FAULT_CODE_ABS_OVERSPEED
  };
  return code < VESC_FAULT_CODE_COUNT ? k[code] : "F?";
}

// Full enumerator name without the FAULT_CODE_ prefix (as VESC Tool prints it).
// Unknown codes -> "UNKNOWN".
static inline const char *vesc_fault_name(uint8_t code) {
  static const char *const k[VESC_FAULT_CODE_COUNT] = {
      "NONE",                               // 0
      "OVER_VOLTAGE",                       // 1
      "UNDER_VOLTAGE",                      // 2
      "DRV",                                // 3
      "ABS_OVER_CURRENT",                   // 4
      "OVER_TEMP_FET",                      // 5
      "OVER_TEMP_MOTOR",                    // 6
      "GATE_DRIVER_OVER_VOLTAGE",           // 7
      "GATE_DRIVER_UNDER_VOLTAGE",          // 8
      "MCU_UNDER_VOLTAGE",                  // 9
      "BOOTING_FROM_WATCHDOG_RESET",        // 10
      "ENCODER_SPI",                        // 11
      "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE", // 12
      "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE", // 13
      "FLASH_CORRUPTION",                   // 14
      "HIGH_OFFSET_CURRENT_SENSOR_1",       // 15
      "HIGH_OFFSET_CURRENT_SENSOR_2",       // 16
      "HIGH_OFFSET_CURRENT_SENSOR_3",       // 17
      "UNBALANCED_CURRENTS",                // 18
      "BRK",                                // 19
      "RESOLVER_LOT",                       // 20
      "RESOLVER_DOS",                       // 21
      "RESOLVER_LOS",                       // 22
      "FLASH_CORRUPTION_APP_CFG",           // 23
      "FLASH_CORRUPTION_MC_CFG",            // 24
      "ENCODER_NO_MAGNET",                  // 25
      "ENCODER_MAGNET_TOO_STRONG",          // 26
      "PHASE_FILTER",                       // 27
      "ENCODER_FAULT",                      // 28
      "LV_OUTPUT_FAULT",                    // 29
      "ENCODER_SLIP",                       // 30
      "OVERSPEED",                          // 31
      "UNDERSPEED",                         // 32
      "ABS_OVERSPEED",                      // 33
  };
  return code < VESC_FAULT_CODE_COUNT ? k[code] : "UNKNOWN";
}

// Writes the short fault string, or "F<code>" (e.g. "F37") for unknown codes, into
// buf (n >= 9 recommended; always NUL-terminated when n > 0). Returns buf.
// No <stdio.h>: the number is formatted by hand so the header stays minimal.
static inline const char *vesc_fault_fmt(char *buf, size_t n, uint8_t code) {
  if (n == 0) return buf;
  if (code < VESC_FAULT_CODE_COUNT) {
    const char *s = vesc_fault_str(code);
    size_t i = 0;
    for (; s[i] != '\0' && i + 1 < n; ++i) buf[i] = s[i];
    buf[i] = '\0';
    return buf;
  }
  char tmp[8];
  size_t k = 0;
  tmp[k++] = 'F';
  unsigned v = code;
  char digits[3];
  int nd = 0;
  do {
    digits[nd++] = (char)('0' + (v % 10u));
    v /= 10u;
  } while (v != 0);
  while (nd > 0) tmp[k++] = digits[--nd];
  size_t i = 0;
  for (; i < k && i + 1 < n; ++i) buf[i] = tmp[i];
  buf[i] = '\0';
  return buf;
}
