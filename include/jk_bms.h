// JK (Jikong) BMS Bluetooth LE protocol codec: command frame builder, 300-byte
// response frame assembler (GATT notification chunks -> CRC-checked frames),
// cell-info / device-info decoders, JK02_24S vs JK02_32S layout detection and
// the error bit names.
//
// Framework-agnostic on purpose (only <stdint.h>/<stddef.h>/<string.h>) so it
// compiles in the native unit tests as well as on the ESP32. Everything that
// touches NimBLE (scan, connect, service 0xFFE0 / characteristic 0xFFE1
// subscribe + write-without-response) lives in src/bms_ble.cpp.
//
// Verified against syssi/esphome-jk-bms components/jk_bms_ble/jk_bms_ble.cpp
// (build_frame, assemble, decode_jk02_cell_info_, decode_device_info_) and the
// recorded frames in test/test_jk_bms/jk_frames.h.
//
// COMMAND (20 bytes, written to 0xFFE1):
//   AA 55 90 EB | reg | len | value LE32 | 9 x 00 | sum8(bytes 0..18)
//   reg 0x97 = device info (answer type 0x03), 0x96 = cell info (answer type
//   0x02; afterwards the BMS streams cell info by itself every ~1 s on JK02 and
//   also settings frames on some firmware). The JK app fills the 9 padding
//   bytes with junk; only bytes 0..9 and the checksum matter.
//
// RESPONSE (exactly 300 bytes):
//   55 AA EB 90 | type @4 | counter @5 | payload ... | sum8(bytes 0..298) @299
//   type 0x01 settings, 0x02 cell info, 0x03 device info, 0x05 logbook.
//   The BMS splits a frame into notifications of whatever size the negotiated
//   MTU allows: 20 (MTU 23: 15 notifications), 128+128+44, 150+150,
//   128+128+22+22 have all been seen. A command-triggered frame is often
//   followed by a 20-byte echo of the ACK (AA 55 90 EB C8 01 01 00x12 44), so
//   "320-byte frames" exist; JK-PB modules also emit 4-byte "AT\r\n"
//   notifications, sometimes glued in front of a preamble in ONE notification.
//   The assembler therefore never trusts notification boundaries: it scans for
//   the preamble at any offset and only ever checks the CRC at index 299 of a
//   preamble-aligned 300-byte window.
//
// CELL INFO (type 0x02) byte offsets, JK02_24S layout / JK02_32S layout:
//   6+2i     u16 mV cell i (24 cells on 24S, 32 on 32S)
//   118/150  u32 mV pack voltage
//   126/158  i32 mA current, POSITIVE = CHARGING (kept as is; the display flips)
//   130/162  i16 0.1 C temperature sensor 1
//   132/164  i16 0.1 C temperature sensor 2
//   134/144  i16 0.1 C MOSFET temperature (24S @134, 32S @112+32)
//   136/166  errors: u16 on 24S @136, u32 on 32S @134+32
//   138/170  i16 mA balance current
//   140/172  u8 balancing action (0 off, 1 charging balancer, 2 discharging)
//   141/173  u8 % SoC
//   142/174  u32 mAh remaining capacity
//   146/178  u32 mAh nominal (full charge) capacity
//   150/182  u32 cycle count
//   158/190  u8 % SOH
//   162/194  u32 s total runtime
//   166/198  u8 charge MOSFET on
//   167/199  u8 discharge MOSFET on
//   (32S: +16 for the fields at 54..111, +32 for the fields >= 112.)
//   Not used on purpose: the BMS's own avg/delta/min/max cell fields @58..63
//   (unreliable), the "precharging"/"balancer" bytes @168/169 and the power
//   field @122 (unsigned). Min/max/avg/delta are computed from the non-zero
//   cells and power = pack_mv * current_ma / 1000 (mW, int64 intermediate).
//
// DEVICE INFO (type 0x03): model @6 (16 chars, NUL-padded, may fill all 16),
//   hardware version @22 (8), software version @30 (8), uptime s u32 @38,
//   power-on count u32 @42, device name @46 (16).
//
// LAYOUT: the same firmware family answers with either the 24-cell or the
//   32-cell layout; there is no field that says which. Guess from the device
//   info (software major >= 11 or a JK_PB / JK-PB model -> 32S, else 24S) and
//   confirm with the first cell-info frame: the sum of the non-zero cells must
//   match the pack voltage within 2 %, see jk_layout_plausible().
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------- constants
// Same numbering as BmsProto in shared_state.h (stored there as a uint8_t).
enum JkProto : uint8_t { JK_PROTO_UNKNOWN = 0, JK_PROTO_JK02_24S = 1, JK_PROTO_JK02_32S = 2 };

constexpr uint8_t JK_CMD_LEN = 20;
constexpr uint16_t JK_FRAME_LEN = 300;
constexpr uint16_t JK_ASM_BUF = 400;  // one frame + the remainder of the notification that completed it

constexpr uint8_t JK_REG_DEVICE_INFO = 0x97;
constexpr uint8_t JK_REG_CELL_INFO = 0x96;

constexpr uint8_t JK_FRAME_SETTINGS = 0x01, JK_FRAME_CELL_INFO = 0x02, JK_FRAME_DEVICE_INFO = 0x03,
                  JK_FRAME_LOGBOOK = 0x05;

// Response preamble and command header (the same four bytes, swapped pairwise:
// the ACK echo AA 55 90 EB never matches the response preamble scan).
constexpr uint8_t JK_RSP_PREAMBLE[4] = {0x55, 0xAA, 0xEB, 0x90};
constexpr uint8_t JK_CMD_HEADER[4] = {0xAA, 0x55, 0x90, 0xEB};

// Cell slots per layout and the byte shift of the fields at >= 112.
constexpr uint8_t JK_CELLS_24S = 24;
constexpr uint8_t JK_CELLS_32S = 32;

// ---------------------------------------------------------------- byte access
// Little-endian, byte-wise: the assembler buffer is not 4-byte aligned, never
// cast to uint16_t*/uint32_t*. Prefixed to coexist with ubx_min.h's rdU2 etc.
static inline uint16_t jk_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static inline uint32_t jk_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int16_t jk_i16(const uint8_t *p) { return (int16_t)jk_u16(p); }
static inline int32_t jk_i32(const uint8_t *p) { return (int32_t)jk_u32(p); }
static inline void jk_wr_u32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

// ---------------------------------------------------------------- checksum
// Both directions use the same "CRC": the sum of the bytes modulo 256.
inline uint8_t jk_sum8(const uint8_t *p, size_t n) {
  uint8_t s = 0;
  for (size_t i = 0; i < n; ++i) s = (uint8_t)(s + p[i]);
  return s;
}

// Builds the 20-byte command frame {AA 55 90 EB, reg, len, value LE32, 9 x 00,
// sum8}. Device info: jk_build_cmd(out, JK_REG_DEVICE_INFO, 0, 0) ->
// AA 55 90 EB 97 00 00x13 11; cell info: ... 96 00 00x13 10. len is the size
// of value in bytes for register writes (0 for the two read requests).
inline void jk_build_cmd(uint8_t out[JK_CMD_LEN], uint8_t reg, uint32_t value, uint8_t len) {
  memset(out, 0, JK_CMD_LEN);
  memcpy(out, JK_CMD_HEADER, 4);
  out[4] = reg;
  out[5] = len;
  jk_wr_u32(out + 6, value);
  out[JK_CMD_LEN - 1] = jk_sum8(out, JK_CMD_LEN - 1);
}

// True when frame[299] == sum8(frame[0..298]). Does not check the preamble.
inline bool jk_frame_crc_ok(const uint8_t frame[JK_FRAME_LEN]) {
  return jk_sum8(frame, JK_FRAME_LEN - 1) == frame[JK_FRAME_LEN - 1];
}

// ---------------------------------------------------------------- assembler
// Collects notification chunks into CRC-checked 300-byte frames.
//   frames_ok       frames delivered to the callback
//   frames_crc_bad  preamble-aligned 300-byte windows whose sum8 did not match
//   resyncs         times bytes were thrown away: garbage / ACK echoes / "AT\r\n"
//                   before a preamble, the remains of a CRC-bad or truncated frame
//                   (an ACK echo after every command DOES count, so a rising
//                   resyncs counter alone is not a link problem)
struct JkAssembler {
  uint8_t buf[JK_ASM_BUF];
  uint16_t len;
  uint32_t frames_ok, frames_crc_bad, resyncs;
};

// Zeroes everything, counters included.
inline void jk_asm_reset(JkAssembler &a) { memset(&a, 0, sizeof a); }

// Drops the buffered bytes, keeps the counters. Call on (re)connect so a
// half-received frame from the previous connection cannot corrupt the first
// frame of the new one.
inline void jk_asm_clear(JkAssembler &a) { a.len = 0; }

// Called once per CRC-valid frame. frame points into the assembler buffer and
// is only valid during the call (copy it if needed); frame[4] is the type, the
// callback dispatches on it (settings/logbook frames are delivered too). Do
// not feed the same assembler from inside the callback.
typedef void (*JkFrameCb)(const uint8_t frame[JK_FRAME_LEN], void *ctx);

static inline bool jk_is_preamble(const uint8_t *p) {
  return p[0] == JK_RSP_PREAMBLE[0] && p[1] == JK_RSP_PREAMBLE[1] && p[2] == JK_RSP_PREAMBLE[2] &&
         p[3] == JK_RSP_PREAMBLE[3];
}

// Index of the first byte of buf[0..len) worth keeping: the first complete
// preamble; else the start of a trailing partial preamble (1..3 bytes equal to
// the beginning of 55 AA EB 90, which the next notification may complete);
// else len (nothing worth keeping).
static inline uint16_t jk_asm_keep_from(const uint8_t *buf, uint16_t len) {
  for (uint16_t i = 0; i + 4 <= len; ++i)
    if (jk_is_preamble(buf + i)) return i;
  for (uint16_t k = 3; k >= 1; --k) {
    if (k > len) continue;
    if (memcmp(buf + len - k, JK_RSP_PREAMBLE, k) == 0) return (uint16_t)(len - k);
  }
  return len;
}

// Aligns the buffer to a preamble and delivers every complete frame in it.
// Leaves fewer than 300 bytes behind (the next frame's beginning at most).
static inline void jk_asm_process(JkAssembler &a, JkFrameCb cb, void *ctx) {
  for (;;) {
    const uint16_t keep = jk_asm_keep_from(a.buf, a.len);
    if (keep > 0) {
      a.resyncs++;
      a.len = (uint16_t)(a.len - keep);
      memmove(a.buf, a.buf + keep, a.len);
    }
    // buf now starts with a full preamble, or holds a partial one (< 4 bytes).
    if (a.len < JK_FRAME_LEN) return;
    if (jk_frame_crc_ok(a.buf)) {
      a.frames_ok++;
      if (cb) cb(a.buf, ctx);
      a.len = (uint16_t)(a.len - JK_FRAME_LEN);
      memmove(a.buf, a.buf + JK_FRAME_LEN, a.len);  // remainder: next frame's start, or an ACK echo (discarded above)
    } else {
      // Wrong CRC: a lost or foreign chunk inside the window (e.g. the ACK
      // echo between two 20-byte chunks). Drop one byte and rescan: the
      // rescan usually finds no preamble and throws the rest away (resync),
      // or finds the next frame's preamble inside the window and aligns to it.
      a.frames_crc_bad++;
      a.len--;
      memmove(a.buf, a.buf + 1, a.len);
    }
  }
}

// Feeds one notification (any length, any alignment). cb may be called zero,
// one or several times per call. A buffer that fills up (400 bytes) without
// completing a frame cannot normally happen (jk_asm_process always leaves
// < 300 bytes); if it does, it is cleared and counted as a resync.
inline void jk_feed(JkAssembler &a, const uint8_t *data, size_t n, JkFrameCb cb, void *ctx) {
  if (data == 0) return;
  size_t i = 0;
  while (i < n) {
    size_t room = (size_t)JK_ASM_BUF - a.len;
    if (room == 0) {
      a.len = 0;
      a.resyncs++;
      room = JK_ASM_BUF;
    }
    size_t take = n - i;
    if (take > room) take = room;
    memcpy(a.buf + a.len, data + i, take);
    a.len = (uint16_t)(a.len + take);
    i += take;
    jk_asm_process(a, cb, ctx);
  }
}

// ---------------------------------------------------------------- layout helpers
static inline uint8_t jk_cell_slots(JkProto proto) { return proto == JK_PROTO_JK02_32S ? JK_CELLS_32S : JK_CELLS_24S; }
// Byte shift of the fields at >= 112 (pack voltage, current, temperatures, ...).
static inline uint8_t jk_off32(JkProto proto) { return proto == JK_PROTO_JK02_32S ? 32 : 0; }

// ---------------------------------------------------------------- cell info
struct JkCellInfo {
  uint8_t counter;                    // frame[5], increments per frame
  uint8_t cell_count;                 // non-zero cells among the layout's 24/32 slots
  uint16_t cell_mv[32];               // every slot, 0 = unused
  uint16_t cell_min_mv, cell_max_mv, cell_avg_mv, cell_delta_mv;  // over the non-zero cells
  uint8_t cell_min_idx, cell_max_idx; // 1-based, first occurrence, 0 = none
  uint32_t pack_mv;
  int32_t current_ma;                 // positive = charging (JK convention)
  int32_t power_mw;                   // pack_mv * current_ma / 1000, same sign as current
  int16_t t1_d, t2_d, mos_d;          // 0.1 C
  uint32_t errors;                    // bit i -> jk_error_name(i); 16 bits on 24S, 32 on 32S
  int16_t balance_ma;
  uint8_t balance_action;             // 0 off, 1 charging balancer, 2 discharging balancer
  uint8_t soc_pct;
  uint32_t remaining_mah, nominal_mah, cycle_count;
  uint8_t soh_pct;
  uint32_t runtime_s;
  bool chg_mos_on, dis_mos_on;
};

// Decodes a CRC-valid frame with the given layout. Zero-fills out first;
// returns false (out all zero) when proto is UNKNOWN or frame[4] != 0x02.
// Does NOT check plausibility: call jk_detect_proto() first when proto is a
// guess, decoding with the wrong layout yields nonsense (pack 0, 65535 mV
// "cells" from the enabled-cells bitmask, ...).
inline bool jk_decode_cell_info(const uint8_t frame[JK_FRAME_LEN], JkProto proto, JkCellInfo &out) {
  memset(&out, 0, sizeof out);
  if (frame == 0 || proto == JK_PROTO_UNKNOWN || frame[4] != JK_FRAME_CELL_INFO) return false;
  const uint8_t slots = jk_cell_slots(proto);
  const uint16_t o = jk_off32(proto);
  out.counter = frame[5];

  uint32_t sum = 0;
  uint16_t mn = 0xFFFF, mx = 0;
  for (uint8_t i = 0; i < slots; ++i) {
    const uint16_t mv = jk_u16(frame + 6 + 2 * i);
    out.cell_mv[i] = mv;
    if (mv == 0) continue;  // unused slot, or a cell the BMS reports as 0 (skipped, indices keep the gap)
    out.cell_count++;
    sum += mv;
    if (mv < mn) {
      mn = mv;
      out.cell_min_idx = (uint8_t)(i + 1);
    }
    if (mv > mx) {
      mx = mv;
      out.cell_max_idx = (uint8_t)(i + 1);
    }
  }
  if (out.cell_count) {
    out.cell_min_mv = mn;
    out.cell_max_mv = mx;
    out.cell_avg_mv = (uint16_t)(sum / out.cell_count);
    out.cell_delta_mv = (uint16_t)(mx - mn);
  }

  out.pack_mv = jk_u32(frame + 118 + o);
  out.current_ma = jk_i32(frame + 126 + o);
  const int64_t p = (int64_t)out.pack_mv * (int64_t)out.current_ma / 1000;
  out.power_mw = p > INT32_MAX ? INT32_MAX : (p < INT32_MIN ? INT32_MIN : (int32_t)p);
  out.t1_d = jk_i16(frame + 130 + o);
  out.t2_d = jk_i16(frame + 132 + o);
  if (proto == JK_PROTO_JK02_32S) {
    out.mos_d = jk_i16(frame + 112 + 32);  // 144
    out.errors = jk_u32(frame + 134 + 32); // 166..169
  } else {
    out.mos_d = jk_i16(frame + 134);
    out.errors = jk_u16(frame + 136);
  }
  out.balance_ma = jk_i16(frame + 138 + o);
  out.balance_action = frame[140 + o];
  out.soc_pct = frame[141 + o];
  out.remaining_mah = jk_u32(frame + 142 + o);
  out.nominal_mah = jk_u32(frame + 146 + o);
  out.cycle_count = jk_u32(frame + 150 + o);
  out.soh_pct = frame[158 + o];
  out.runtime_s = jk_u32(frame + 162 + o);
  out.chg_mos_on = frame[166 + o] != 0;
  out.dis_mos_on = frame[167 + o] != 0;
  return true;
}

// ---------------------------------------------------------------- device info
struct JkDevInfo {
  uint8_t counter;
  char model[17];  // "JK-B2A24S20P", "JK_B2A8S20P", "JK_PB2A16S15P", "JK-B2A16S" (JK04)
  char hw[9];      // "10.XG", "11.XW", "14.XA", "3.0"
  char sw[9];      // "10.07", "11.17", "14.20", "3.3.0"
  uint32_t uptime_s;
  uint32_t power_on_count;
  char name[17];   // BLE device name as configured in the app
};

// Copies a fixed-width NUL-padded field into dst (width + 1 bytes), stopping
// at the first NUL, always NUL-terminated. Bytes outside printable ASCII
// become '.' so a corrupt frame cannot put control characters on the display.
static inline void jk_copy_str(const uint8_t *src, size_t width, char *dst) {
  size_t n = 0;
  for (; n < width && src[n] != 0; ++n) {
    const uint8_t c = src[n];
    dst[n] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
  }
  dst[n] = 0;
}

// Decodes a CRC-valid device info frame. Zero-fills out first; returns false
// when frame[4] != 0x03.
inline bool jk_decode_device_info(const uint8_t frame[JK_FRAME_LEN], JkDevInfo &out) {
  memset(&out, 0, sizeof out);
  if (frame == 0 || frame[4] != JK_FRAME_DEVICE_INFO) return false;
  out.counter = frame[5];
  jk_copy_str(frame + 6, 16, out.model);
  jk_copy_str(frame + 22, 8, out.hw);
  jk_copy_str(frame + 30, 8, out.sw);
  out.uptime_s = jk_u32(frame + 38);
  out.power_on_count = jk_u32(frame + 42);
  jk_copy_str(frame + 46, 16, out.name);
  return true;
}

// Leading decimal number of a software version string: "11.17" -> 11,
// "10.07" -> 10, "3.3.0" -> 3, "14.20" -> 14. -1 when it does not start with
// a digit ("", "V7"). Capped at 99999 (no int overflow on garbage).
inline int jk_sw_major(const char *sw) {
  if (sw == 0 || sw[0] < '0' || sw[0] > '9') return -1;
  int v = 0;
  for (size_t i = 0; sw[i] >= '0' && sw[i] <= '9'; ++i) {
    v = v * 10 + (sw[i] - '0');
    if (v > 99999) return 99999;
  }
  return v;
}

static inline bool jk_starts_with(const char *s, const char *prefix) {
  return s != 0 && strncmp(s, prefix, strlen(prefix)) == 0;
}

// Layout guess from the device info: JK_PB* / JK-PB* (the inverter-oriented
// PB series) and software major >= 11 use the 32-cell layout, everything else
// (JK-B* 10.xx, JK04 3.x) the 24-cell one. Confirm with jk_detect_proto().
inline JkProto jk_guess_proto(const JkDevInfo &d) {
  if (jk_starts_with(d.model, "JK_PB") || jk_starts_with(d.model, "JK-PB")) return JK_PROTO_JK02_32S;
  if (jk_sw_major(d.sw) >= 11) return JK_PROTO_JK02_32S;
  return JK_PROTO_JK02_24S;
}

// True when a cell info frame makes sense in the given layout: at least one
// non-zero cell, no cell above 5000 mV, pack voltage above 1000 mV and the
// sum of the non-zero cells within 2 % of the pack voltage. In the wrong
// layout the pack field reads 0 (24S view of a 32S frame) or the extra cell
// slots contain the enabled-cells bitmask 0xFFFF (32S view of a 24S frame).
inline bool jk_layout_plausible(const uint8_t frame[JK_FRAME_LEN], JkProto proto) {
  if (frame == 0 || proto == JK_PROTO_UNKNOWN || frame[4] != JK_FRAME_CELL_INFO) return false;
  const uint8_t slots = jk_cell_slots(proto);
  uint32_t sum = 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < slots; ++i) {
    const uint16_t mv = jk_u16(frame + 6 + 2 * i);
    if (mv == 0) continue;
    if (mv > 5000) return false;
    sum += mv;
    count++;
  }
  if (count == 0 || count > 32) return false;
  const uint32_t pack = jk_u32(frame + 118 + jk_off32(proto));
  if (pack <= 1000) return false;
  const uint32_t diff = sum > pack ? sum - pack : pack - sum;
  return diff < pack / 50;
}

// Layout of a cell info frame: the guess when it is plausible, else the other
// layout when that one is, else UNKNOWN (a device info frame, a JK04 frame, a
// frame with a zero pack voltage). guess may be UNKNOWN: then 24S is tried
// first, then 32S.
inline JkProto jk_detect_proto(const uint8_t frame[JK_FRAME_LEN], JkProto guess) {
  const JkProto first = (guess == JK_PROTO_JK02_32S) ? JK_PROTO_JK02_32S : JK_PROTO_JK02_24S;
  const JkProto second = (first == JK_PROTO_JK02_32S) ? JK_PROTO_JK02_24S : JK_PROTO_JK02_32S;
  if (jk_layout_plausible(frame, first)) return first;
  if (jk_layout_plausible(frame, second)) return second;
  return JK_PROTO_UNKNOWN;
}

// ---------------------------------------------------------------- errors
// Error/alarm bit names (bit 0 first), in the order of the JK app's alarm list
// as used by syssi's DEFAULT_ERRORS_JK02. At most 6 characters for the OLED.
// The 24S layout only carries bits 0..15.
inline const char *jk_error_name(uint8_t bit) {
  static const char *const k[32] = {
      "WIRE_R",  // 0  wire resistance (a sense wire / cell resistance out of range)
      "MOS_OT",  // 1  MOSFET over temperature
      "CELLNO",  // 2  cell count mismatch
      "BIT3",    // 3  current sensor anomaly
      "FULL",    // 4  cell over voltage (charge complete)
      "PACKOV",  // 5  pack over voltage
      "CHG_OC",  // 6  charge over current
      "CHG_SC",  // 7  charge short circuit
      "CHG_OT",  // 8  charge over temperature
      "CHG_UT",  // 9  charge under temperature
      "COPROC",  // 10 CPU / auxiliary communication error
      "CELLUV",  // 11 cell under voltage
      "PACKUV",  // 12 pack under voltage
      "DIS_OC",  // 13 discharge over current
      "DIS_SC",  // 14 discharge short circuit
      "DIS_OT",  // 15 discharge over temperature
      "CMOSAB",  // 16 charge MOSFET abnormal
      "DMOSAB",  // 17 discharge MOSFET abnormal
      "GPS",     // 18 GPS disconnected
      "PASSWD",  // 19 modify password in time
      "DISON",   // 20 discharge on failed
      "BAT_OT",  // 21 battery over temperature alarm
      "TSENS",   // 22 temperature sensor anomaly
      "PLMOD",   // 23 PLC module anomaly
      "SCPREL",  // 24 short circuit protection release
      "DOCP2",   // 25 discharge over current protection 2
      "DOCP3",   // 26 discharge over current protection 3
      "DIS_UT",  // 27 discharge under temperature
      "GPSLCK",  // 28 GPS lock
      "BIT29",   // 29 reserved
      "BIT30",   // 30 reserved
      "BIT31",   // 31 reserved
  };
  return bit < 32 ? k[bit] : "?";
}

// Lowest set bit (0..31) or -1 when errors == 0: the one to show first.
inline int jk_first_error(uint32_t errors) {
  for (int b = 0; b < 32; ++b)
    if (errors & (1u << b)) return b;
  return -1;
}

inline uint8_t jk_error_count(uint32_t errors) {
  uint8_t n = 0;
  while (errors) {
    n = (uint8_t)(n + (errors & 1u));
    errors >>= 1;
  }
  return n;
}
