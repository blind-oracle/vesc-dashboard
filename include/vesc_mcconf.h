// The VESC's configured temperature limits, read with COMM_GET_MCCONF: the coolant pump's
// thresholds (include/pump_logic.h). Parser plus the read scheduler both transports share
// (src/can_vesc.cpp, src/uart_vesc.cpp).
//
// Framework-free (only <stdint.h>/<stddef.h>/<string.h>), so the native tests exercise it.
//
// Wire format (verified against vedderb/bldc release_5_03 .. release_7_00, comm/commands.c
// commands_send_mcconf() and confgenerator.c confgenerator_serialize_mcconf()):
//
//   REQUEST  payload [14] (COMM_GET_MCCONF). Read-only: the handler copies the live config
//            and answers, nothing else. Over CAN: PROCESS_SHORT_BUFFER [own_id, 0, 14]; the
//            ~480-byte answer comes back as ~76 FILL_RX_BUFFER(_LONG) frames + PROCESS_RX_BUFFER.
//   REPLY    payload [14, MCCONF_SIGNATURE u32 BE, the serialized mc_configuration ...].
//            The temperature limits sit at serialized offset 63 (payload index 64) in EVERY
//            firmware from 5.03 to 7.00 - the fields before them were reshuffled in 6.05,
//            but their byte count stayed 63 - and only their encoding changed:
//              5.03 / 6.00 / 6.02  float16 x10:  fet_start 63, fet_end 65, motor_start 67,
//                                  motor_end 69, then l_temp_accel_dec float16 x10000 at 71
//              6.05 / 6.06 / 7.00  uint8 (whole C, truncated): 63, 64, 65, 66, then
//                                  l_temp_accel_dec float16 x10000 at 67
//            MCCONF_SIGNATURE is a CRC over the names, types and encodings of every field,
//            so it changes with ANY layout change. Only signatures in the table are parsed;
//            an unknown one means "firmware this code has not seen": the caller falls back
//            to its defaults rather than guessing at an offset.
//
// Where the VESC actually starts derating: mc_interface.c update_override_limits() cuts the
// ACCELERATION current from utils_map(accel_dec, 0, 1, start, 25) = start - accel_dec *
// (start - 25), which is 76 C for the defaults 85 / 0.15 - braking current only at 85 C.
// A boat's forward thrust is all acceleration current, so 76 C is where it loses power.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "config.h"

#define VESC_COMM_GET_MCCONF 14u

// Serialized offset of l_temp_fet_start, counted from the first signature byte.
#define VESC_MCCONF_TEMP_OFFSET 63u

enum VescMcconfResult : uint8_t {
  MCCONF_NONE = 0,     // nothing read yet
  MCCONF_OK,           // limits decoded and plausible
  MCCONF_NOT_MCCONF,   // not a COMM_GET_MCCONF packet (the caller routes by payload[0]; kept for completeness)
  MCCONF_SHORT,        // shorter than the layout its signature names
  MCCONF_UNKNOWN_SIG,  // a firmware whose layout is not in the table
  MCCONF_IMPLAUSIBLE,  // a known layout carrying values out of range (start >= end, ...)
};

inline const char *vesc_mcconf_result_str(uint8_t r) {
  switch (r) {
    case MCCONF_NONE: return "none";
    case MCCONF_OK: return "ok";
    case MCCONF_NOT_MCCONF: return "not mcconf";
    case MCCONF_SHORT: return "short";
    case MCCONF_UNKNOWN_SIG: return "unknown signature";
    case MCCONF_IMPLAUSIBLE: return "implausible";
    default: return "?";
  }
}

struct VescTempLimits {
  float fet_start, fet_end;  // l_temp_fet_start / _end, C: the VESC derates the MOSFETs from start, stops at end
  float mot_start, mot_end;  // l_temp_motor_start / _end, C
  float accel_dec;           // l_temp_accel_dec, 0..1: how much earlier the acceleration current is derated
};

enum VescMcconfEnc : uint8_t {
  MCCONF_ENC_F16_X10,  // four int16 BE / 10 at 63, 65, 67, 69; accel_dec at 71
  MCCONF_ENC_U8,       // four uint8 at 63..66; accel_dec at 67
};

struct VescMcconfLayout {
  uint32_t sig;
  const char *fw;  // firmware label (<= 7 chars: the PUMP screen shows it)
  uint8_t enc;     // VescMcconfEnc
};

// MCCONF_SIGNATURE per release (confgenerator.h of each release_* branch).
inline const VescMcconfLayout *vesc_mcconf_layout(uint32_t sig) {
  static const VescMcconfLayout kLayouts[] = {
      {0x968A9B08u, "5.03", MCCONF_ENC_F16_X10},
      {0x2E43A161u, "6.00/02", MCCONF_ENC_F16_X10},  // 6.00 and 6.02 share one layout
      {0x3F829CF7u, "6.05", MCCONF_ENC_U8},
      {0x2EFD0142u, "6.06", MCCONF_ENC_U8},
      {0x57AD8F53u, "7.00", MCCONF_ENC_U8},
  };
  for (const VescMcconfLayout &l : kLayouts)
    if (l.sig == sig) return &l;
  return nullptr;
}

struct VescMcconfRead {
  uint8_t result;     // VescMcconfResult
  uint32_t sig;       // MCCONF_SIGNATURE of the reply (0 when too short to carry one)
  char fw[8];         // firmware label of a known layout, "" otherwise
  VescTempLimits lim; // valid only with MCCONF_OK
};

namespace vesc_mcconf_detail {
inline uint32_t be_u32(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
inline float be_f16(const uint8_t *p, float scale) { return (float)(int16_t)(((uint16_t)p[0] << 8) | p[1]) / scale; }
inline bool pair_ok(float start, float end) { return start >= 20.0f && start < end && end <= 250.0f; }
}  // namespace vesc_mcconf_detail

// Decodes the four temperature limits and l_temp_accel_dec from a COMM_GET_MCCONF reply
// payload (payload[0] = 14). Every outcome other than MCCONF_OK leaves lim zeroed.
inline VescMcconfRead vesc_mcconf_parse(const uint8_t *payload, size_t len) {
  using namespace vesc_mcconf_detail;
  VescMcconfRead r;
  memset(&r, 0, sizeof r);
  if (payload == nullptr || len < 1 || payload[0] != VESC_COMM_GET_MCCONF) {
    r.result = MCCONF_NOT_MCCONF;
    return r;
  }
  if (len < 5) {
    r.result = MCCONF_SHORT;
    return r;
  }
  const uint8_t *conf = payload + 1;  // serialized offsets count from here (the signature)
  const size_t conf_len = len - 1;
  r.sig = be_u32(conf);
  const VescMcconfLayout *l = vesc_mcconf_layout(r.sig);
  if (l == nullptr) {
    r.result = MCCONF_UNKNOWN_SIG;
    return r;
  }
  for (size_t i = 0; i + 1 < sizeof r.fw && l->fw[i] != '\0'; ++i) r.fw[i] = l->fw[i];
  const size_t o = VESC_MCCONF_TEMP_OFFSET;
  VescTempLimits lim;
  if (l->enc == MCCONF_ENC_F16_X10) {
    if (conf_len < o + 10) {
      r.result = MCCONF_SHORT;
      return r;
    }
    lim.fet_start = be_f16(conf + o, 10.0f);
    lim.fet_end = be_f16(conf + o + 2, 10.0f);
    lim.mot_start = be_f16(conf + o + 4, 10.0f);
    lim.mot_end = be_f16(conf + o + 6, 10.0f);
    lim.accel_dec = be_f16(conf + o + 8, 10000.0f);
  } else {
    if (conf_len < o + 6) {
      r.result = MCCONF_SHORT;
      return r;
    }
    lim.fet_start = conf[o];
    lim.fet_end = conf[o + 1];
    lim.mot_start = conf[o + 2];
    lim.mot_end = conf[o + 3];
    lim.accel_dec = be_f16(conf + o + 4, 10000.0f);
  }
  if (!pair_ok(lim.fet_start, lim.fet_end) || !pair_ok(lim.mot_start, lim.mot_end) || lim.accel_dec < 0.0f ||
      lim.accel_dec > 1.0f) {
    r.result = MCCONF_IMPLAUSIBLE;
    return r;
  }
  r.lim = lim;
  r.result = MCCONF_OK;
  return r;
}

// Where the VESC starts cutting acceleration current for one sensor (see the header comment).
inline float vesc_derate_start(float start, float accel_dec) { return start - accel_dec * (start - 25.0f); }

// ---------------------------------------------------------------- read scheduler
// Decides when the transport spends one poll slot on COMM_GET_MCCONF instead of GET_VALUES:
// at every link-up (the first answered poll after boot or after the link was lost - the VESC
// may have been reconfigured meanwhile), VESC_MCCONF_RETRY_MS after a request that went
// unanswered or came back broken, and every VESC_MCCONF_REFRESH_MS while connected (0 = never)
// once an answer was parsed, whatever it said: an unknown signature will not change by asking
// again sooner. One request at a time; the transport reports what became of it.
struct VescMcconfSched {
  bool link_was_up;  // link_up at the previous due() call
  bool armed;        // a read is wanted at next_ms; false while one is in flight (or refresh is off)
  uint32_t next_ms;
};

inline void mcconf_sched_init(VescMcconfSched &s) { memset(&s, 0, sizeof s); }

// Call when a poll slot is due. link_up = the VESC answered the last GET_VALUES poll.
// True = send COMM_GET_MCCONF in this slot, then call mcconf_sched_on_sent().
inline bool mcconf_sched_due(VescMcconfSched &s, bool link_up, uint32_t now) {
  if (!link_up) {
    s.link_was_up = false;
    return false;
  }
  if (!s.link_was_up) {  // (re)connected: read now, whatever was scheduled
    s.link_was_up = true;
    s.armed = true;
    s.next_ms = now;
  }
  return s.armed && (int32_t)(now - s.next_ms) >= 0;
}

inline void mcconf_sched_on_sent(VescMcconfSched &s) { s.armed = false; }

// A reply was parsed. MCCONF_SHORT / MCCONF_NOT_MCCONF count as broken (retry soon).
inline void mcconf_sched_on_result(VescMcconfSched &s, uint8_t result, uint32_t now) {
  const bool answered = result == MCCONF_OK || result == MCCONF_UNKNOWN_SIG || result == MCCONF_IMPLAUSIBLE;
  if (answered && VESC_MCCONF_REFRESH_MS == 0) {
    s.armed = false;
    return;
  }
  s.armed = true;
  s.next_ms = now + (answered ? (uint32_t)VESC_MCCONF_REFRESH_MS : (uint32_t)VESC_MCCONF_RETRY_MS);
}

// The request went unanswered within VESC_MCCONF_TIMEOUT_MS, or its frames were broken.
inline void mcconf_sched_on_timeout(VescMcconfSched &s, uint32_t now) {
  s.armed = true;
  s.next_ms = now + (uint32_t)VESC_MCCONF_RETRY_MS;
}
