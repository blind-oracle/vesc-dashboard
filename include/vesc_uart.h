// Clean-room codec for polling a VESC over its UART (the COMM-header port) instead
// of CAN: framing of the COMM_GET_VALUES request, reassembly of the replies from
// the byte stream, and the update of the shared VESC state from one reply.
//
// Framework-agnostic: only <stdint.h>/<stdbool.h>/<string.h> plus the project's
// framework-free headers, so it compiles in the native unit tests as well as on
// the ESP32. Everything that touches the UART driver lives in src/uart_vesc.cpp.
//
// Wire protocol (verified against vedderb/bldc master, comm/packet.c
// packet_send_packet() / packet_process_byte(), comm/commands.c):
//
//   FRAME    len <= 255 : [2, len u8,             payload, crc_hi, crc_lo, 3]
//            len >  255 : [3, len_hi, len_lo,     payload, crc_hi, crc_lo, 3]
//            crc16 = CRC-16/XMODEM (vesc_crc16) over the payload only. PACKET_MAX_PL_LEN
//            = 512, so the 24-bit form (start byte 4) never occurs: the VESC rejects it too.
//
//   REQUEST  payload [COMM_GET_VALUES] -> the 6 bytes 02 01 04 40 84 03. Plain
//            COMM_GET_VALUES rather than _SELECTIVE: over UART nothing is broadcast, so
//            every field is wanted, and it works on every firmware.
//
//   REPLY    payload [4, fields bit 0..21 in order] (74 bytes on FW >= 5.03), the very
//            same payload the CAN reply carries: vesc_decode_getvalues() decodes it.
//            commands_process_packet() also points the VESC's global send_func at the
//            last requester, so unsolicited packets (COMM_PRINT, ...) can arrive between
//            the replies: check the packet id before decoding.
//
// Read-only: COMM_GET_VALUES does not call timeout_reset() on the VESC, so a polling
// dashboard can never keep the motor's control timeout alive.
//
// What a GET_VALUES reply cannot provide: STATUS_6 (raw ADC1..3 pin voltages and the
// servo input). Its t_ms[] slot is never stamped, so those values stay "--". Motor and
// battery current are the VESC's read-and-reset averages since the previous read (by
// any client), not the filtered values STATUS_1/STATUS_4 carry.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "shared_state.h"    // VescState, VescExt, VESC_STALE_R1_MS
#include "vesc_getvalues.h"  // vesc_crc16, vesc_decode_getvalues, VESC_GV_*, VESC_RX_BUFFER_SIZE
#include "vesc_status.h"     // vesc_telemetry_t, VESC_IDX_*

// ---------------------------------------------------------------- framing
#define VESC_UART_START_SHORT 2u
#define VESC_UART_START_LONG 3u
#define VESC_UART_STOP 3u
#define VESC_UART_MAX_PAYLOAD 512u           // PACKET_MAX_PL_LEN on the VESC
#define VESC_UART_GETVALUES_REQUEST_LEN 6u   // 02 01 04 40 84 03

// Wraps payload (1..VESC_UART_MAX_PAYLOAD bytes) into one frame in out. Returns the
// frame length (len + 5, or len + 6 above 255 bytes), or 0 if the length is out of
// range or the frame does not fit cap.
static inline size_t vesc_uart_frame(uint8_t *out, size_t cap, const uint8_t *payload, uint16_t len) {
  if (out == 0 || payload == 0 || len == 0 || len > VESC_UART_MAX_PAYLOAD) return 0;
  const bool is_long = len > 255u;
  const size_t total = (size_t)len + (is_long ? 6u : 5u);
  if (cap < total) return 0;
  size_t i = 0;
  if (is_long) {
    out[i++] = (uint8_t)VESC_UART_START_LONG;
    out[i++] = (uint8_t)(len >> 8);
    out[i++] = (uint8_t)len;
  } else {
    out[i++] = (uint8_t)VESC_UART_START_SHORT;
    out[i++] = (uint8_t)len;
  }
  memcpy(out + i, payload, len);
  i += len;
  const uint16_t crc = vesc_crc16(payload, len);
  out[i++] = (uint8_t)(crc >> 8);
  out[i++] = (uint8_t)crc;
  out[i++] = (uint8_t)VESC_UART_STOP;
  return i;
}

// The COMM_GET_VALUES request frame; out must hold VESC_UART_GETVALUES_REQUEST_LEN bytes.
static inline size_t vesc_uart_build_getvalues_request(uint8_t *out) {
  const uint8_t id = (uint8_t)VESC_COMM_GET_VALUES;
  return vesc_uart_frame(out, VESC_UART_GETVALUES_REQUEST_LEN, &id, 1);
}

// A request frame carrying the one-byte command cmd (COMM_GET_MCCONF: 02 01 0E crc crc 03);
// out must hold VESC_UART_CMD_REQUEST_LEN bytes.
#define VESC_UART_CMD_REQUEST_LEN 6u
static inline size_t vesc_uart_build_cmd_request(uint8_t *out, uint8_t cmd) {
  return vesc_uart_frame(out, VESC_UART_CMD_REQUEST_LEN, &cmd, 1);
}

// ---------------------------------------------------------------- reassembly
// The VESC's own decoder works on a buffer, not a byte-wise state machine, and so does
// this one: a frame is only accepted once it is complete and its stop byte and CRC
// match; any failure drops ONE byte and rescans. That matters because the stop byte (3)
// is also the long-form start byte, and a payload byte can look like either: a parser
// that threw away the whole failed candidate would also throw away the real frame
// hiding inside it.
struct VescUartRx {
  uint8_t buf[VESC_RX_BUFFER_SIZE + 6];  // raw bytes not yet consumed: one longest frame
  uint8_t pkt[VESC_RX_BUFFER_SIZE];      // the payload last returned by vesc_uart_rx_next()
  uint16_t n;                            // bytes in buf
  uint32_t junk;                         // bytes dropped outside any valid frame (noise, partial frames, overflow)
  uint32_t bad;                          // complete candidate frames whose stop byte or CRC did not match
};

static inline void vesc_uart_rx_reset(VescUartRx *r) { r->n = 0; }

static inline void vesc_uart_rx_drop(VescUartRx *r, uint16_t k) {
  if (k >= r->n) {
    r->n = 0;
    return;
  }
  memmove(r->buf, r->buf + k, (size_t)(r->n - k));
  r->n = (uint16_t)(r->n - k);
}

// Appends n received bytes. When they do not fit, the OLDEST bytes go first (counted
// as junk): a full buffer can only mean a bogus frame header waiting for bytes that
// will never complete it, and dropping it is exactly the resync that is needed.
static inline void vesc_uart_rx_push(VescUartRx *r, const uint8_t *data, size_t n) {
  const size_t cap = sizeof r->buf;
  if (n >= cap) {  // only the newest cap bytes can matter
    r->junk += (uint32_t)(r->n + (n - cap));
    data += n - cap;
    n = cap;
    r->n = 0;
  } else if (r->n + n > cap) {
    const uint16_t over = (uint16_t)(r->n + n - cap);
    r->junk += over;
    vesc_uart_rx_drop(r, over);
  }
  memcpy(r->buf + r->n, data, n);
  r->n = (uint16_t)(r->n + n);
}

// Extracts the next complete, CRC-valid frame. Returns true and points *payload/*len
// at its payload (valid until the next call), false when the buffer holds no complete
// frame. flush = true means the line has gone idle: an incomplete frame can then never
// complete, so it is rescanned byte by byte (and ends up as junk) instead of waited for.
static inline bool vesc_uart_rx_next(VescUartRx *r, bool flush, const uint8_t **payload, uint16_t *len) {
  while (r->n > 0) {
    const uint8_t s = r->buf[0];
    if (s != VESC_UART_START_SHORT && s != VESC_UART_START_LONG) {
      vesc_uart_rx_drop(r, 1);
      r->junk++;
      continue;
    }
    const uint16_t hdr = s == VESC_UART_START_SHORT ? 2u : 3u;
    if (r->n < hdr) {
      if (!flush) return false;
      vesc_uart_rx_drop(r, 1);
      r->junk++;
      continue;
    }
    const uint16_t plen =
        s == VESC_UART_START_SHORT ? r->buf[1] : (uint16_t)(((uint16_t)r->buf[1] << 8) | r->buf[2]);
    // The long form is only ever used above 255 bytes (the VESC rejects it otherwise too).
    const bool len_ok = plen != 0 && plen <= VESC_UART_MAX_PAYLOAD && plen <= sizeof r->pkt &&
                        (s == VESC_UART_START_SHORT || plen > 255u);
    if (!len_ok) {
      vesc_uart_rx_drop(r, 1);
      r->junk++;
      continue;
    }
    const uint16_t total = (uint16_t)(hdr + plen + 3u);
    if (r->n < total) {
      if (!flush) return false;
      vesc_uart_rx_drop(r, 1);
      r->junk++;
      continue;
    }
    const uint8_t *p = r->buf + hdr;
    const uint16_t crc = (uint16_t)(((uint16_t)r->buf[hdr + plen] << 8) | r->buf[hdr + plen + 1]);
    if (r->buf[total - 1] != VESC_UART_STOP || vesc_crc16(p, plen) != crc) {
      vesc_uart_rx_drop(r, 1);
      r->bad++;
      continue;
    }
    memcpy(r->pkt, p, plen);
    vesc_uart_rx_drop(r, total);
    *payload = r->pkt;
    *len = plen;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------- state update
// The GET_VALUES fields each STATUS broadcast carries. A STATUS slot (t_ms[]) is stamped
// fresh only when every one of its fields was in the reply, so a reply cut short by older
// firmware never makes a stale value look live. STATUS_1/STATUS_4 currents are the
// averages (bits 2/3): GET_VALUES has no filtered currents.
#define VESC_UART_FIELDS_S1 ((1u << VESC_GV_RPM) | (1u << VESC_GV_AVG_MOTOR_CURRENT) | (1u << VESC_GV_DUTY))
#define VESC_UART_FIELDS_S2 ((1u << VESC_GV_AMP_HOURS) | (1u << VESC_GV_AMP_HOURS_CHARGED))
#define VESC_UART_FIELDS_S3 ((1u << VESC_GV_WATT_HOURS) | (1u << VESC_GV_WATT_HOURS_CHARGED))
#define VESC_UART_FIELDS_S4                                                                            \
  ((1u << VESC_GV_TEMP_FET) | (1u << VESC_GV_TEMP_MOTOR) | (1u << VESC_GV_AVG_INPUT_CURRENT) |        \
   (1u << VESC_GV_PID_POS))
#define VESC_UART_FIELDS_S5 ((1u << VESC_GV_TACHO) | (1u << VESC_GV_V_IN))

// The fields a reply actually carried: vesc_decode_getvalues() decodes in bit order and
// stops at the end of the data, so they are the first nfields set bits of the mask.
static inline uint32_t vesc_getvalues_decoded_mask(uint32_t mask, int nfields) {
  uint32_t out = 0;
  for (int b = 0; b < VESC_GV_BIT_COUNT && nfields > 0; ++b) {
    if (!(mask & (1u << b))) continue;
    out |= 1u << b;
    --nfields;
  }
  return out;
}

// Exponential moving average, seeded (not blended) on the first sample of its source.
static inline float vesc_uart_ema(float prev, float x, bool seed, float alpha) {
  return seed ? x : (alpha * x + (1.0f - alpha) * prev);
}

#define VESC_UART_NOT_GETVALUES (-2)  // unsolicited packet (COMM_PRINT, ...): neither good nor bad
#define VESC_UART_BAD_LAYOUT (-1)     // a GET_VALUES packet the decoder cannot lay out: count as bad

// Applies one CRC-valid reply payload to the VESC state, exactly as the CAN path would
// after the matching STATUS frames plus a poll reply:
//   * every decoded field into v->t / x (fields the reply lacks keep their value),
//   * the averaged currents into t.current_motor / t.current_in,
//   * t_ms[STATUS_1..5] stamped per complete field group, STATUS_6 never,
//   * the display EMAs (i_motor / i_in / v_in) stepped with alpha, seeded on the first
//     sample and after a gap longer than VESC_STALE_R1_MS (a reconnect must not blend
//     in a value that is minutes old),
//   * locked_id / t.id from the controller id field, frames_total / last_frame_ms,
//   * x->t_ms, x->replies_ok and the fault latch (last non-zero code and its time).
// Returns the number of fields decoded (compare with vesc_getvalues_field_count() of the
// reply's mask to spot older firmware), or VESC_UART_NOT_GETVALUES / VESC_UART_BAD_LAYOUT
// with *v and *x untouched. The counters polls_sent / timeouts / replies_bad belong to the
// caller.
static inline int vesc_uart_apply_getvalues(VescState *v, VescExt *x, const uint8_t *payload, uint16_t len,
                                            uint32_t now, float alpha) {
  if (!vesc_is_getvalues_payload(payload, len)) return VESC_UART_NOT_GETVALUES;
  vesc_telemetry_t t = v->t;
  VescExt e = *x;
  const int nfields = vesc_decode_getvalues(payload, len, &e, &t);
  if (nfields < 0) return VESC_UART_BAD_LAYOUT;
  const uint32_t got = vesc_getvalues_decoded_mask(vesc_getvalues_mask(payload, len), nfields);
  const uint32_t ts = now ? now : 1u;  // 0 keeps meaning "never"

  if (got & (1u << VESC_GV_AVG_MOTOR_CURRENT)) t.current_motor = e.avg_motor_current;
  if (got & (1u << VESC_GV_AVG_INPUT_CURRENT)) t.current_in = e.avg_input_current;

  static const uint32_t kGroups[VESC_IDX_STATUS_5 + 1] = {VESC_UART_FIELDS_S1, VESC_UART_FIELDS_S2, VESC_UART_FIELDS_S3,
                                                          VESC_UART_FIELDS_S4, VESC_UART_FIELDS_S5};
  bool stamped[VESC_IDX_STATUS_5 + 1];
  bool seed[VESC_IDX_STATUS_5 + 1];
  for (int i = 0; i <= VESC_IDX_STATUS_5; ++i) {
    seed[i] = t.t_ms[i] == 0 || (uint32_t)(ts - t.t_ms[i]) > VESC_STALE_R1_MS;
    stamped[i] = (got & kGroups[i]) == kGroups[i];
    if (stamped[i]) t.t_ms[i] = ts;
  }
  if (stamped[VESC_IDX_STATUS_1]) {
    v->i_motor_ema = vesc_uart_ema(v->i_motor_ema, t.current_motor, seed[VESC_IDX_STATUS_1], alpha);
    v->ema_init = true;
  }
  if (stamped[VESC_IDX_STATUS_4]) {
    v->i_in_ema = vesc_uart_ema(v->i_in_ema, t.current_in, seed[VESC_IDX_STATUS_4], alpha);
    v->ema_init = true;
  }
  if (stamped[VESC_IDX_STATUS_5]) {
    v->v_in_ema = vesc_uart_ema(v->v_in_ema, t.v_in, seed[VESC_IDX_STATUS_5], alpha);
    v->ema_init = true;
  }
  if (got & (1u << VESC_GV_VESC_ID)) {
    t.id = e.vesc_id;
    v->locked_id = e.vesc_id;
  }

  e.t_ms = ts;
  e.replies_ok++;
  if (e.fault_code != 0) {  // latch: the VESC clears the live byte after its fault stop time
    e.last_fault = e.fault_code;
    e.last_fault_ms = ts;
  }
  v->t = t;
  v->frames_total++;
  v->last_frame_ms = ts;
  *x = e;
  return nfields;
}
