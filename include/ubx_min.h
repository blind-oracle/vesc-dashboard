// Minimal u-blox UBX protocol support: frame parser, NAV-PVT / NAV-VELNED
// decoders, MON-VER PROTVER extraction and small frame/VALSET builders.
//
// Framework-agnostic on purpose (only <stdint.h>/<stdbool.h>/<string.h>) so it
// compiles in the native unit tests as well as on the ESP32. Everything that
// touches the UART lives in src/gnss_ubx.cpp.
//
// Frame:  B5 62 | cls | id | lenLo lenHi | payload[len] | ckA ckB
// Fletcher-8 checksum (ckA += b; ckB += ckA) over cls, id, len and payload.
//
// Verified against the u-blox 7 / M8 / M9 / M10 interface descriptions:
//   NAV-PVT  0x01 0x07  len 84 (u-blox 7) or 92 (M8/M9/M10); the first 84
//            bytes are identical, so decode with len >= 84.
//   NAV-VELNED 0x01 0x12 len 36 (u-blox 6 fallback, cm/s instead of mm/s).
//   MON-VER  0x0A 0x04  swVersion[30] @0, hwVersion[10] @30, then 30-byte
//            extension strings @40+30n; PROTVER is spelled "PROTVER=18.00"
//            (M8 FW 3.x, M9, M10) or "PROTVER 15.00" (u-blox 6/7, M8 FW 2.01).
//   ACK-ACK 0x05 0x01 / ACK-NAK 0x05 0x00, payload {cls, id} of the acked msg.
//   CFG-VALSET 0x06 0x8A (PROTVER >= 27): {0 version, layers, 2 reserved,
//            key U4 LE, value(size from key bits 28..30) ...}.
//   CFG-PRT  0x06 0x00  len 20, UART port configuration of u-blox 6/7/M8
//            (PROTVER < 27): {portID U1, reserved U1, txReady X2, mode X4,
//            baudRate U4, inProtoMask X2, outProtoMask X2, flags X2, res U1[2]}.
//   CFG-RATE 0x06 0x08  len 6: {measRate U2 ms, navRate U2 cycles, timeRef U2}.
//            measRate >= 50 ms below PROTVER 24 (>= 25 ms from 24 on); navRate
//            is fixed to 1 below PROTVER 18. A rate the module cannot sustain
//            is ACKed anyway and shows up as missing epochs, never as a NAK.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------- constants
#define UBX_SYNC1 0xB5u
#define UBX_SYNC2 0x62u

#define UBX_CLASS_NAV 0x01u
#define UBX_NAV_PVT 0x07u
#define UBX_NAV_VELNED 0x12u

#define UBX_CLASS_ACK 0x05u
#define UBX_ACK_NAK 0x00u
#define UBX_ACK_ACK 0x01u

#define UBX_CLASS_CFG 0x06u
#define UBX_CFG_PRT 0x00u
#define UBX_CFG_MSG 0x01u
#define UBX_CFG_RATE 0x08u
#define UBX_CFG_NAV5 0x24u
#define UBX_CFG_VALSET 0x8Au

#define UBX_CLASS_MON 0x0Au
#define UBX_MON_VER 0x04u

// Wildcard for UbxParser frame matching helpers (cls or id "don't care").
#define UBX_ANY 0xFFu

// Longest payload we accept. MON-VER on an M10 with ~10 extension strings is
// 40 + 30*10 = 340 bytes; anything larger than 512 is treated as a corrupt
// length (which is how a wrong-baud byte stream usually shows up).
#define UBX_MAX_PAYLOAD 512u
// Frame overhead: 2 sync + cls + id + 2 len + 2 checksum.
#define UBX_FRAME_OVERHEAD 8u

// CFG-VALSET configuration keys (u-blox generation 9+ interface description).
#define UBX_KEY_CFG_UART1OUTPROT_UBX 0x10740001u          // L
#define UBX_KEY_CFG_UART1OUTPROT_NMEA 0x10740002u         // L
#define UBX_KEY_CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007u  // U1 (rate per epoch)
#define UBX_KEY_CFG_RATE_MEAS 0x30210001u                 // U2 ms
#define UBX_KEY_CFG_RATE_NAV 0x30210002u                  // U2 cycles
#define UBX_KEY_CFG_NAVSPG_DYNMODEL 0x20110021u           // U1 (E1), 5 = SEA
#define UBX_KEY_CFG_UART1_BAUDRATE 0x40520001u            // U4 bit/s; the port switches as soon as the message is processed

// VALSET header bytes: version 0, layers RAM|BBR (never FLASH: we reconfigure
// on every boot instead of wearing the receiver's flash), 2 reserved.
#define UBX_VALSET_LAYER_RAM 0x01u
#define UBX_VALSET_LAYER_BBR 0x02u
#define UBX_VALSET_HEADER_LEN 4u

// CFG-PRT UART payload (20 bytes, u-blox 6/7/M8 receiver descriptions).
// mode 0x000008D0: bit4 reserved1 = 1 (Antaris compatibility), charLen bits
// 6-7 = 11 (8 data bits), parity bits 9-11 = 100 (none), nStopBits bits 12-13
// = 00 (1 stop bit): the 8N1 constant gpsd and PX4 send. Protocol masks: bit0
// UBX, bit1 NMEA, bit2 RTCM2 (input only; M8 factory inProtoMask is 0x0007,
// outProtoMask 0x0003). The change is RAM-only on these receivers: without a
// CFG-CFG save they boot at their factory baud again (autobaud re-finds them).
#define UBX_CFG_PRT_LEN 20u
#define UBX_CFG_PRT_UART1 0x01u
#define UBX_CFG_PRT_MODE_8N1 0x000008D0u
#define UBX_CFG_PRT_PROTO_UBX 0x0001u
#define UBX_CFG_PRT_PROTO_NMEA 0x0002u
#define UBX_CFG_PRT_PROTO_RTCM 0x0004u

// CFG-RATE payload (6 bytes); timeRef 0 = UTC, 1 = GPS time.
#define UBX_CFG_RATE_LEN 6u
#define UBX_CFG_RATE_TIMEREF_UTC 0u
#define UBX_CFG_RATE_TIMEREF_GPS 1u

// NAV-PVT fixType values.
#define UBX_FIX_NONE 0u
#define UBX_FIX_DR 1u
#define UBX_FIX_2D 2u
#define UBX_FIX_3D 3u
#define UBX_FIX_GNSS_DR 4u
#define UBX_FIX_TIME 5u

// ---------------------------------------------------------------- byte access
// Little-endian, byte-wise: UBX payloads are packed and the parser buffer is
// not guaranteed to be 4-byte aligned, so never cast to uint32_t*.
static inline uint8_t rdU1(const uint8_t *p) { return p[0]; }
static inline uint16_t rdU2(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static inline uint32_t rdU4(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t rdI4(const uint8_t *p) { return (int32_t)rdU4(p); }

static inline void wrU2(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)(v >> 8);
}
static inline void wrU4(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

// ---------------------------------------------------------------- checksum
// Fletcher-8 over n bytes starting at the class byte (cls, id, len, payload).
static inline void ubxChecksum(const uint8_t *frameFromCls, size_t n, uint8_t &ckA, uint8_t &ckB) {
  ckA = 0;
  ckB = 0;
  for (size_t i = 0; i < n; ++i) {
    ckA = (uint8_t)(ckA + frameFromCls[i]);
    ckB = (uint8_t)(ckB + ckA);
  }
}

// Builds a complete frame into out. Returns the frame length (len + 8) or 0 if
// outCap is too small. pl may be null when len == 0 (a poll).
static inline size_t ubxBuildFrame(uint8_t *out, size_t outCap, uint8_t cls, uint8_t id, const uint8_t *pl,
                                   uint16_t len) {
  const size_t total = (size_t)len + UBX_FRAME_OVERHEAD;
  if (out == 0 || outCap < total || (len > 0 && pl == 0)) return 0;
  out[0] = (uint8_t)UBX_SYNC1;
  out[1] = (uint8_t)UBX_SYNC2;
  out[2] = cls;
  out[3] = id;
  wrU2(out + 4, len);
  if (len) memcpy(out + 6, pl, len);
  ubxChecksum(out + 2, (size_t)len + 4, out[6 + len], out[7 + len]);
  return total;
}

// ---------------------------------------------------------------- parser
// Byte-at-a-time state machine. feed() returns true exactly once per
// checksum-valid frame; cls()/id()/len()/payload() are then valid until the
// next call to feed().
class UbxParser {
 public:
  // Returns true when b completed a valid frame.
  bool feed(uint8_t b) {
    switch (state_) {
      case 0:  // waiting for sync 1
        if (b == UBX_SYNC1) state_ = 1;
        return false;
      case 1:  // waiting for sync 2
        if (b == UBX_SYNC2) {
          state_ = 2;
          ckA_ = 0;
          ckB_ = 0;
        } else if (b == UBX_SYNC1) {
          state_ = 1;  // "B5 B5 62": the second B5 is the real sync
        } else {
          state_ = 0;
        }
        return false;
      case 2:
        cls_ = b;
        ck(b);
        state_ = 3;
        return false;
      case 3:
        id_ = b;
        ck(b);
        state_ = 4;
        return false;
      case 4:
        len_ = b;
        ck(b);
        state_ = 5;
        return false;
      case 5:
        len_ = (uint16_t)(len_ | ((uint16_t)b << 8));
        ck(b);
        if (len_ > UBX_MAX_PAYLOAD) {
          // Implausible length: almost always a wrong-baud byte stream that
          // happened to contain B5 62. Count it and resync instead of waiting
          // for 60 000 bytes that will never come.
          badFrames++;
          state_ = 0;
          return false;
        }
        pos_ = 0;
        state_ = (len_ == 0) ? 7 : 6;
        return false;
      case 6:  // payload
        payload_[pos_++] = b;
        ck(b);
        if (pos_ >= len_) state_ = 7;
        return false;
      case 7:  // CK_A
        if (b != ckA_) {
          badFrames++;
          state_ = 0;
          return false;
        }
        state_ = 8;
        return false;
      case 8:  // CK_B
        state_ = 0;
        if (b != ckB_) {
          badFrames++;
          return false;
        }
        goodFrames++;
        return true;
      default:
        state_ = 0;
        return false;
    }
  }

  uint8_t cls() const { return cls_; }
  uint8_t id() const { return id_; }
  uint16_t len() const { return len_; }
  const uint8_t *payload() const { return payload_; }

  // True if the last completed frame has this class/id (UBX_ANY = wildcard).
  // Both wildcards are needed: waiting for "any ACK" is (0x05, UBX_ANY).
  bool matches(uint8_t cls, uint8_t id) const {
    return (cls == UBX_ANY || cls_ == cls) && (id == UBX_ANY || id_ == id);
  }

  uint32_t goodFrames = 0;  // checksum-valid frames
  uint32_t badFrames = 0;   // CK_A or CK_B mismatches + oversized lengths

  // Drops any partially received frame (e.g. after a baud change). Counters
  // are kept: the caller compares before/after.
  void reset() {
    state_ = 0;
    pos_ = 0;
  }

 private:
  void ck(uint8_t b) {
    ckA_ = (uint8_t)(ckA_ + b);
    ckB_ = (uint8_t)(ckB_ + ckA_);
  }

  uint8_t state_ = 0;  // 0 sync1, 1 sync2, 2 cls, 3 id, 4 lenLo, 5 lenHi, 6 payload, 7 ckA, 8 ckB
  uint8_t cls_ = 0;
  uint8_t id_ = 0;
  uint16_t len_ = 0;
  uint16_t pos_ = 0;
  uint8_t ckA_ = 0;
  uint8_t ckB_ = 0;
  uint8_t payload_[UBX_MAX_PAYLOAD];
};

// ---------------------------------------------------------------- NAV-PVT
struct NavPvt {
  uint32_t iTOW;      // ms of GPS week
  uint16_t year;
  uint8_t month, day, hour, min, sec;
  uint8_t valid;      // bit0 validDate, bit1 validTime, bit2 fullyResolved, bit3 validMag
  uint8_t fixType;    // 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
  uint8_t flags;      // bit0 gnssFixOK (PROTVER >= 20 only), bit1 diffSoln, bits 2-4 psmState, bit5 headVehValid
  uint8_t flags2;
  uint8_t numSV;
  int32_t lon, lat;   // 1e-7 deg
  int32_t height;     // mm above ellipsoid
  int32_t hMSL;       // mm above mean sea level
  uint32_t hAcc, vAcc;  // mm
  int32_t velN, velE, velD;  // mm/s
  int32_t gSpeed;     // mm/s, 2-D ground speed
  int32_t headMot;    // 1e-5 deg
  uint32_t sAcc;      // mm/s speed accuracy estimate
  uint32_t headAcc;   // 1e-5 deg
  uint16_t pDOP;      // 0.01
  uint16_t payloadLen;  // 84 (u-blox 7) or 92 (M8+)
};

// Decodes the parser's last frame as NAV-PVT. Accepts len >= 84 because the
// u-blox 7 message is 84 bytes and the M8/M9/M10 one is 92 (headVeh, magDec
// appended); the fields below live in the common first 84 bytes.
static inline bool decodeNavPvt(const UbxParser &p, NavPvt &o) {
  if (p.cls() != UBX_CLASS_NAV || p.id() != UBX_NAV_PVT || p.len() < 84) return false;
  const uint8_t *b = p.payload();
  o.iTOW = rdU4(b + 0);
  o.year = rdU2(b + 4);
  o.month = rdU1(b + 6);
  o.day = rdU1(b + 7);
  o.hour = rdU1(b + 8);
  o.min = rdU1(b + 9);
  o.sec = rdU1(b + 10);
  o.valid = rdU1(b + 11);
  // 12..19: tAcc U4, nano I4 (unused)
  o.fixType = rdU1(b + 20);
  o.flags = rdU1(b + 21);
  o.flags2 = rdU1(b + 22);
  o.numSV = rdU1(b + 23);
  o.lon = rdI4(b + 24);
  o.lat = rdI4(b + 28);
  o.height = rdI4(b + 32);
  o.hMSL = rdI4(b + 36);
  o.hAcc = rdU4(b + 40);
  o.vAcc = rdU4(b + 44);
  o.velN = rdI4(b + 48);
  o.velE = rdI4(b + 52);
  o.velD = rdI4(b + 56);
  o.gSpeed = rdI4(b + 60);
  o.headMot = rdI4(b + 64);
  o.sAcc = rdU4(b + 68);
  o.headAcc = rdU4(b + 72);
  o.pDOP = rdU2(b + 76);
  o.payloadLen = p.len();
  return true;
}

// NAV-VELNED (u-blox 6 has no NAV-PVT): gSpeed U4 @20 and sAcc U4 @28 are in
// cm/s; converted to mm/s to match NAV-PVT.
static inline bool decodeNavVelned(const UbxParser &p, int32_t &gSpeed_mm_s, uint32_t &sAcc_mm_s) {
  if (p.cls() != UBX_CLASS_NAV || p.id() != UBX_NAV_VELNED || p.len() != 36) return false;
  const uint8_t *b = p.payload();
  const uint32_t gs_cm = rdU4(b + 20);
  const uint32_t sa_cm = rdU4(b + 28);
  gSpeed_mm_s = (gs_cm > 0x0CCCCCCCu) ? 0x7FFFFFFF : (int32_t)(gs_cm * 10u);
  sAcc_mm_s = (sa_cm > 0x19999999u) ? 0xFFFFFFFFu : sa_cm * 10u;
  return true;
}

// Fix gate. flags bit0 (gnssFixOK) only exists from PROTVER 20 on: older
// receivers leave the bit 0, so requiring it there would never show a fix.
static inline bool pvtFixOk(const NavPvt &o, int protVerX100) {
  const bool typeOk = (o.fixType == UBX_FIX_2D || o.fixType == UBX_FIX_3D || o.fixType == UBX_FIX_GNSS_DR);
  if (!typeOk) return false;
  if (protVerX100 < 2000) return true;
  return (o.flags & 0x01u) != 0;
}

// ---------------------------------------------------------------- MON-VER
// Tiny "dd.dd" -> x100 parser (no atof: no locale, no libc float formatting
// dependency on the target). Accepts "18", "18.0", "18.00", "27.11", stops at
// the first character that is not a digit or the first '.'.
static inline int ubxParseDecimalX100(const uint8_t *s, size_t n) {
  int whole = 0;
  size_t i = 0;
  bool any = false;
  while (i < n && s[i] >= '0' && s[i] <= '9') {
    whole = whole * 10 + (s[i] - '0');
    any = true;
    ++i;
    // Real PROTVERs are two digits; a longer run is garbage. Stop before the
    // int can overflow (signed overflow is UB) and report "unknown".
    if (whole > 9999) return 0;
  }
  if (!any) return 0;
  int frac = 0;
  if (i < n && s[i] == '.') {
    ++i;
    int digits = 0;
    while (i < n && s[i] >= '0' && s[i] <= '9' && digits < 2) {
      frac = frac * 10 + (s[i] - '0');
      ++digits;
      ++i;
    }
    if (digits == 1) frac *= 10;  // "18.5" -> 1850
  }
  return whole * 100 + frac;
}

// Scans the MON-VER extension strings (30 bytes each at 40 + 30n) for
// "PROTVER=" or "PROTVER " and returns the version * 100 (1800 = 18.00).
// Returns 0 if not found (u-blox 6 firmware older than 7.03 has no PROTVER).
static inline int ubxParseProtVer(const uint8_t *monVerPayload, uint16_t len) {
  if (monVerPayload == 0) return 0;
  static const char kTag[] = "PROTVER";
  const size_t tagLen = sizeof(kTag) - 1;
  for (uint16_t off = 40; off + 30 <= len; off = (uint16_t)(off + 30)) {
    const uint8_t *ext = monVerPayload + off;
    // Bounded scan inside the 30-byte field: it is NUL-padded but treat it as
    // untrusted (a truncated or garbage frame must not run off the buffer).
    for (size_t i = 0; i + tagLen + 1 < 30; ++i) {
      if (ext[i] == 0) break;
      if (memcmp(ext + i, kTag, tagLen) != 0) continue;
      const uint8_t sep = ext[i + tagLen];
      if (sep != '=' && sep != ' ') continue;
      const size_t start = i + tagLen + 1;
      const int v = ubxParseDecimalX100(ext + start, 30 - start);
      if (v > 0) return v;
    }
  }
  return 0;
}

// Copies a fixed-width MON-VER string field (swVersion 30 B @0, hwVersion
// 10 B @30, extension 30 B @40+30n) into a NUL-terminated buffer.
static inline void ubxCopyVerField(const uint8_t *payload, uint16_t len, uint16_t off, uint8_t width, char *out,
                                   size_t outCap) {
  if (outCap == 0) return;
  size_t n = 0;
  while (n < width && n + 1 < outCap && (uint16_t)(off + n) < len && payload[off + n] != 0) {
    const uint8_t c = payload[off + n];
    out[n] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
    ++n;
  }
  out[n] = 0;
}

// ---------------------------------------------------------------- VALSET
// Value size encoded in the key (bits 28..30): 1 = L (1 byte), 2 = U1, 3 = U2,
// 4 = U4, 5 = U8. Deriving it from the key removes a whole class of "sent a U1
// for a U2 key -> NAK" mistakes.
static inline uint8_t ubxValsetValueSize(uint32_t key) {
  switch ((key >> 28) & 0x7u) {
    case 1: return 1;
    case 2: return 1;
    case 3: return 2;
    case 4: return 4;
    case 5: return 8;
    default: return 0;
  }
}

// Writes the 4-byte VALSET header {version 0, layers, 0, 0} and returns the
// payload length so far (4).
static inline uint16_t ubxValsetBegin(uint8_t *pl, uint16_t cap, uint8_t layers = UBX_VALSET_LAYER_RAM | UBX_VALSET_LAYER_BBR) {
  if (pl == 0 || cap < UBX_VALSET_HEADER_LEN) return 0;
  pl[0] = 0x00;  // message version
  pl[1] = layers;
  pl[2] = 0x00;
  pl[3] = 0x00;
  return UBX_VALSET_HEADER_LEN;
}

// Appends key (U4 LE) + value (1/2/4 bytes LE by key size) to pl at *len.
// Returns false (and leaves pl untouched) if the key size is unknown/8 or cap
// would be exceeded.
static inline bool ubxValsetAppend(uint8_t *pl, uint16_t &len, uint16_t cap, uint32_t key, uint32_t value) {
  const uint8_t vs = ubxValsetValueSize(key);
  if (pl == 0 || vs == 0 || vs > 4) return false;
  if ((uint32_t)len + 4u + vs > cap) return false;
  wrU4(pl + len, key);
  len = (uint16_t)(len + 4);
  switch (vs) {
    case 1: pl[len] = (uint8_t)value; break;
    case 2: wrU2(pl + len, (uint16_t)value); break;
    default: wrU4(pl + len, value); break;
  }
  len = (uint16_t)(len + vs);
  return true;
}

// ---------------------------------------------------------------- legacy CFG builders
// CFG-PRT payload for UART1: 8N1 at baud, with the given input/output protocol
// masks (UBX_CFG_PRT_PROTO_*). txReady, flags and the reserved bytes are 0.
// Returns UBX_CFG_PRT_LEN (20), or 0 when payload20 is null. Sending it
// reconfigures the port immediately, so the ACK normally leaves at the NEW
// baud (or is corrupted): callers verify with a poll at the new rate instead.
static inline size_t ubxBuildCfgPrtUart1(uint8_t *payload20, uint32_t baud, uint16_t inProto, uint16_t outProto) {
  if (payload20 == 0) return 0;
  memset(payload20, 0, UBX_CFG_PRT_LEN);
  payload20[0] = (uint8_t)UBX_CFG_PRT_UART1;  // portID
  wrU4(payload20 + 4, UBX_CFG_PRT_MODE_8N1);  // mode
  wrU4(payload20 + 8, baud);                  // baudRate
  wrU2(payload20 + 12, inProto);              // inProtoMask
  wrU2(payload20 + 14, outProto);             // outProtoMask
  return UBX_CFG_PRT_LEN;
}

// CFG-RATE payload: measurement period ms, navigation cycles per measurement
// (1 on every generation that matters), time reference. Returns
// UBX_CFG_RATE_LEN (6), or 0 when payload6 is null.
static inline size_t ubxBuildCfgRate(uint8_t *payload6, uint16_t measMs, uint16_t navCycles, uint16_t timeRef) {
  if (payload6 == 0) return 0;
  wrU2(payload6 + 0, measMs);
  wrU2(payload6 + 2, navCycles);
  wrU2(payload6 + 4, timeRef);
  return UBX_CFG_RATE_LEN;
}
