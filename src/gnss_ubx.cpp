// GNSS task: u-blox UBX receiver on HP UART1 (Serial1).
//
// Phase machine (runs entirely inside gnssTask, setup() never waits):
//   AUTOBAUD  try each baud in GNSS_BAUDS: listen for any valid UBX frame,
//             else poll MON-VER (factory modules are NMEA-only but still
//             answer UBX polls). Retry forever if nothing answers.
//   DETECT    MON-VER -> PROTVER (both "PROTVER=" and "PROTVER " spellings).
//   CONFIGURE PROTVER >= 27: CFG-VALSET (RAM+BBR). Otherwise (or when every
//             VALSET is refused): CFG-MSG / CFG-RATE, NMEA off first,
//             NAV-VELNED fallback when NAV-PVT is unavailable (u-blox 6).
//   RUN       pump bytes -> UbxParser -> g_state.gnss. No valid frame for
//             GNSS_REDETECT_MS -> back to AUTOBAUD (module power-cycled).
//
// Rules kept throughout: the heartbeat hb_gnss is touched at least every
// ~100 ms (every wait is a loop of short vTaskDelay), no logging or UART I/O
// while holding the state mutex, timestamps are millis() with 0 = never.
#include "gnss_ubx.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "shared_state.h"
#include "ubx_min.h"

// Local fallbacks for tunables that config.h does not (yet) define.
#ifndef GNSS_AUTOBAUD_LISTEN_MS
#define GNSS_AUTOBAUD_LISTEN_MS 1200   // passive listen per baud (a UBX-configured module emits 1 Hz NAV-PVT)
#endif
#ifndef GNSS_MONVER_TIMEOUT_MS
#define GNSS_MONVER_TIMEOUT_MS 1500    // MON-VER reply wait (the reply is long and queues behind NMEA at 9600)
#endif
#ifndef GNSS_AUTOBAUD_RETRY_MS
#define GNSS_AUTOBAUD_RETRY_MS 2000    // pause between failed autobaud passes
#endif
#ifndef GNSS_TASK_STACK
#define GNSS_TASK_STACK 4096           // bytes (Arduino's xTaskCreate takes bytes)
#endif

namespace {

// Serial1 == HP UART1 on the ESP32-C6. Serial2 is the LP UART (16-byte FIFO,
// different clock): never use it here.
HardwareSerial &GNSS = Serial1;

constexpr uint32_t kBauds[] = GNSS_BAUDS;
constexpr size_t kNumBauds = sizeof(kBauds) / sizeof(kBauds[0]);
constexpr uint32_t kPumpSliceMs = 10;    // idle delay between RX pumps
constexpr uint32_t kHbMaxGapMs = 100;    // longest sleep without a heartbeat touch
constexpr uint32_t kSettleMs = 50;       // after a baud change: let the UART and the receiver's current byte finish

// NMEA standard message ids (class 0xF0) switched off on legacy receivers.
constexpr uint8_t kNmeaIds[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x08, 0x41};  // GGA GLL GSA GSV RMC VTG ZDA TXT

// Per-baud RX statistics for the "nothing answered" diagnostic.
struct BaudStat {
  uint32_t bytes;
  uint32_t dollars;  // '$' count: NMEA sentence starts (receiver alive, wrong protocol)
};

// Everything the task needs. Static so the 512-byte parser buffer and the
// MON-VER copy do not live on the task stack.
struct GnssCtx {
  UbxParser parser;
  uint8_t phase = GNSS_PHASE_AUTOBAUD;
  uint32_t baud = 0;
  int prot_ver_x100 = -1;    // -1 unknown
  bool configured = false;
  bool use_velned = false;
  uint32_t last_good_frame_ms = 0;
  uint32_t redetects = 0;
  // RX diagnostics for the current autobaud attempt
  uint32_t rx_bytes = 0;
  uint32_t rx_dollars = 0;
  // MON-VER reply (payload copy) for DETECT
  bool have_mon_ver = false;
  uint16_t mon_ver_len = 0;
  uint8_t mon_ver[UBX_MAX_PAYLOAD];
};
GnssCtx s_ctx;

// ---------------------------------------------------------------- small helpers
uint32_t now_nz() {
  const uint32_t t = millis();
  return t ? t : 1u;  // 0 is reserved for "never"
}

// PROTVER used for decisions: unknown (no MON-VER / no PROTVER string) is
// treated as a legacy 15.00 receiver, which is the safe assumption.
int effective_prot_ver() { return s_ctx.prot_ver_x100 > 0 ? s_ctx.prot_ver_x100 : 1500; }

[[maybe_unused]] const char *ack_str(int r) { return r > 0 ? "ACK" : (r == 0 ? "NAK" : "timeout"); }  // log-only

// vTaskDelay in slices so the supervisor keeps seeing a live heartbeat.
void delay_hb(uint32_t ms) {
  while (ms > 0) {
    const uint32_t slice = ms > kHbMaxGapMs ? kHbMaxGapMs : ms;
    vTaskDelay(pdMS_TO_TICKS(slice));
    hb_touch(hb_gnss);
    ms -= slice;
  }
}

// Drops whatever is in the RX ring right now (bounded by the snapshot so a
// streaming receiver cannot keep us here).
void flush_rx() {
  int n = GNSS.available();
  while (n-- > 0) {
    if (GNSS.read() < 0) break;
  }
}

// Publishes the phase-level fields of the context into g_state.gnss.
void publish_phase(uint8_t phase) {
  s_ctx.phase = phase;
  if (!state_lock()) return;
  GnssState &g = g_state.gnss;
  g.phase = phase;
  g.baud = s_ctx.baud;
  g.prot_ver_x100 = s_ctx.prot_ver_x100;
  g.configured = s_ctx.configured;
  g.use_velned = s_ctx.use_velned;
  g.redetects = s_ctx.redetects;
  g.good_frames = s_ctx.parser.goodFrames;
  g.bad_frames = s_ctx.parser.badFrames;
  state_unlock();
}

// ---------------------------------------------------------------- frame handling
// Called for EVERY checksum-valid frame, including the ones that arrive while
// a configuration step waits for its ACK (NAV-PVT must never be dropped).
void handle_frame() {
  const UbxParser &p = s_ctx.parser;
  const uint32_t now = now_nz();
  s_ctx.last_good_frame_ms = now;

  if (p.matches(UBX_CLASS_MON, UBX_MON_VER)) {
    s_ctx.mon_ver_len = p.len();
    memcpy(s_ctx.mon_ver, p.payload(), p.len());
    s_ctx.have_mon_ver = true;
  }

  // Decode outside the lock; write inside.
  NavPvt pvt{};
  bool have_pvt = false;
  int32_t vel_gspeed = 0;
  uint32_t vel_sacc = 0;
  bool have_vel = false;
  if (p.matches(UBX_CLASS_NAV, UBX_NAV_PVT)) {
    have_pvt = decodeNavPvt(p, pvt);
  } else if (s_ctx.use_velned && p.matches(UBX_CLASS_NAV, UBX_NAV_VELNED)) {
    have_vel = decodeNavVelned(p, vel_gspeed, vel_sacc);
  }
  const int prot_ver = effective_prot_ver();

  if (!state_lock()) return;
  GnssState &g = g_state.gnss;
  g.good_frames = p.goodFrames;  // copied, not incremented: the parser is the single source of truth
  g.bad_frames = p.badFrames;
  if (have_pvt) {
    g.last_pvt_ms = now;
    g.fix_type = pvt.fixType;
    g.flags = pvt.flags;
    g.num_sv = pvt.numSV;
    g.fix_ok = pvtFixOk(pvt, prot_ver);
    g.gspeed_mm_s = pvt.gSpeed;
    g.sacc_mm_s = pvt.sAcc;
    g.pdop_x100 = pvt.pDOP;
    g.lat_e7 = pvt.lat;
    g.lon_e7 = pvt.lon;
    g.hour = pvt.hour;
    g.min = pvt.min;
    g.sec = pvt.sec;
    g.time_valid = (pvt.valid & 0x03u) == 0x03u;  // validDate && validTime
  } else if (have_vel) {
    // u-blox 6: no fixType available from this message; the accuracy estimate
    // is the only fix-quality indicator (sAcc is huge without a fix).
    g.last_pvt_ms = now;
    g.gspeed_mm_s = vel_gspeed;
    g.sacc_mm_s = vel_sacc;
    g.fix_ok = vel_sacc < (uint32_t)GNSS_MAX_SACC_MM_S;
  }
  state_unlock();
}

// Feeds buffered bytes to the parser and STOPS right after the first complete
// frame that matches (cls, id). Bytes are taken from the driver ring one at a
// time so nothing read is lost: on a true return the parser still holds the
// matching frame (ubx_send_ack inspects its payload) and the unread tail stays
// in the ring for the next call. Draining everything instead would let a
// NAV-PVT that arrives right behind an ACK overwrite the ACK before the caller
// looks at it, turning ACKs into "timeouts" whenever the receiver is already
// streaming (redetect of a live module, or every step after PVT is enabled).
// Bounded by the byte count seen at entry, so a flooding receiver cannot
// starve the heartbeat. Returns true if a matching frame was completed.
bool pump_rx(uint8_t cls, uint8_t id) {
  int n = GNSS.available();
  while (n-- > 0) {
    const int c = GNSS.read();
    if (c < 0) break;
    s_ctx.rx_bytes++;
    if (c == '$') s_ctx.rx_dollars++;
    if (!s_ctx.parser.feed((uint8_t)c)) continue;
    handle_frame();
    if (s_ctx.parser.matches(cls, id)) return true;
  }
  return false;
}

// ---------------------------------------------------------------- UBX I/O
// Builds and writes one frame. Payloads we send are small (CFG-NAV5 is the
// largest at 36 bytes), so the UART FIFO absorbs them without blocking.
bool ubx_send(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len) {
  uint8_t frame[64 + UBX_FRAME_OVERHEAD];
  const size_t n = ubxBuildFrame(frame, sizeof(frame), cls, id, pl, len);
  if (n == 0) {
    log_e("GNSS: frame %02X %02X len %u does not fit the TX buffer", cls, id, (unsigned)len);
    return false;
  }
  return GNSS.write(frame, n) == n;
}

// Pumps RX for up to timeout_ms, handing every frame to handle_frame(), and
// returns true as soon as one matches (cls, id). UBX_ANY wildcards either
// field: waiting for "any ACK-class message" is (UBX_CLASS_ACK, UBX_ANY).
// The reference snippet this replaces only wildcarded cls and therefore never
// matched an ACK; both wildcards are required.
bool ubx_wait_for(uint8_t cls, uint8_t id, uint32_t timeout_ms) {
  const uint32_t start = millis();
  for (;;) {
    hb_touch(hb_gnss);
    if (pump_rx(cls, id)) return true;
    if ((uint32_t)(millis() - start) >= timeout_ms) return false;
    vTaskDelay(pdMS_TO_TICKS(kPumpSliceMs));
  }
}

// Sends a message and waits GNSS_ACK_TIMEOUT_MS for the matching ACK-ACK /
// ACK-NAK (payload {cls, id} must match: an ACK for an earlier message is
// skipped). Returns +1 ACK, 0 NAK, -1 timeout / send failure.
int ubx_send_ack(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len) {
  if (!ubx_send(cls, id, pl, len)) return -1;
  const uint32_t start = millis();
  for (;;) {
    const uint32_t elapsed = (uint32_t)(millis() - start);
    if (elapsed >= (uint32_t)GNSS_ACK_TIMEOUT_MS) return -1;
    if (!ubx_wait_for(UBX_CLASS_ACK, UBX_ANY, (uint32_t)GNSS_ACK_TIMEOUT_MS - elapsed)) return -1;
    const UbxParser &p = s_ctx.parser;
    if (p.len() >= 2 && p.payload()[0] == cls && p.payload()[1] == id) return p.id() == UBX_ACK_ACK ? 1 : 0;
    // ACK for something else: keep waiting with the remaining budget.
  }
}

bool poll_mon_ver(uint32_t timeout_ms) {
  ubx_send(UBX_CLASS_MON, UBX_MON_VER, nullptr, 0);  // B5 62 0A 04 00 00 0E 34
  return ubx_wait_for(UBX_CLASS_MON, UBX_MON_VER, timeout_ms);
}

// ---------------------------------------------------------------- AUTOBAUD
// One baud attempt: passive listen, then an active MON-VER poll.
bool autobaud_try(uint32_t baud, BaudStat &stat) {
  GNSS.updateBaudRate(baud);
  delay_hb(kSettleMs);
  flush_rx();
  s_ctx.parser.reset();
  s_ctx.rx_bytes = 0;
  s_ctx.rx_dollars = 0;

  // 1) A module already talking UBX (configured on a previous boot with BBR
  //    kept by its backup battery) betrays itself within one epoch.
  const uint32_t good0 = s_ctx.parser.goodFrames;
  ubx_wait_for(UBX_ANY, UBX_ANY, GNSS_AUTOBAUD_LISTEN_MS);
  bool found = s_ctx.parser.goodFrames != good0;

  // 2) Factory configs are usually NMEA-only, but the UBX *input* protocol is
  //    enabled by default, so a MON-VER poll gets a UBX reply even then. This
  //    is the path that normally succeeds on a fresh module.
  if (!found) {
    flush_rx();
    s_ctx.parser.reset();
    found = poll_mon_ver(GNSS_MONVER_TIMEOUT_MS);
  }

  stat.bytes = s_ctx.rx_bytes;
  stat.dollars = s_ctx.rx_dollars;
  log_d("GNSS: autobaud %lu: %s (%lu bytes, %lu '$')", (unsigned long)baud, found ? "UBX" : "nothing",
        (unsigned long)stat.bytes, (unsigned long)stat.dollars);
  return found;
}

// Formats "38400:0B 9600:612B/$12 ..." for the failure log.
void format_baud_stats(const BaudStat *stats, char *out, size_t cap) {
  size_t used = 0;
  out[0] = 0;
  for (size_t i = 0; i < kNumBauds && used < cap; ++i) {
    const int n = snprintf(out + used, cap - used, "%s%lu:%luB/$%lu", i ? " " : "", (unsigned long)kBauds[i],
                           (unsigned long)stats[i].bytes, (unsigned long)stats[i].dollars);
    if (n < 0) break;
    used += (size_t)n;
  }
}

void phase_autobaud() {
  s_ctx.have_mon_ver = false;
  s_ctx.baud = 0;  // unknown again (0 = none yet), also after a redetect
  s_ctx.configured = false;
  s_ctx.use_velned = false;
  s_ctx.prot_ver_x100 = -1;
  publish_phase(GNSS_PHASE_AUTOBAUD);

  BaudStat stats[kNumBauds];
  for (;;) {
    for (size_t i = 0; i < kNumBauds; ++i) {
      stats[i] = BaudStat{0, 0};
      if (autobaud_try(kBauds[i], stats[i])) {
        s_ctx.baud = kBauds[i];
        log_i("GNSS: UBX detected at %lu baud (%s)", (unsigned long)s_ctx.baud,
              s_ctx.have_mon_ver ? "MON-VER reply" : "unsolicited frame");
        publish_phase(GNSS_PHASE_AUTOBAUD);  // baud now known
        return;
      }
    }
    char detail[160];
    format_baud_stats(stats, detail, sizeof(detail));
    // '$' counts > 0 mean a live NMEA receiver that ignores UBX input on this
    // port (UBX inProtoMask disabled); 0 bytes everywhere means wiring/power.
    log_e("GNSS: no UBX reply at any baud [%s]; retrying in %u ms", detail, (unsigned)GNSS_AUTOBAUD_RETRY_MS);
    delay_hb(GNSS_AUTOBAUD_RETRY_MS);
  }
}

// ---------------------------------------------------------------- DETECT
void log_mon_ver() {
  char sw[31], hw[11], ext[31];
  ubxCopyVerField(s_ctx.mon_ver, s_ctx.mon_ver_len, 0, 30, sw, sizeof(sw));
  ubxCopyVerField(s_ctx.mon_ver, s_ctx.mon_ver_len, 30, 10, hw, sizeof(hw));
  const unsigned n_ext = s_ctx.mon_ver_len > 40 ? (unsigned)((s_ctx.mon_ver_len - 40) / 30) : 0u;
  log_i("GNSS: MON-VER sw=\"%s\" hw=\"%s\" extensions=%u", sw, hw, n_ext);
  for (unsigned i = 0; i < n_ext; ++i) {
    ubxCopyVerField(s_ctx.mon_ver, s_ctx.mon_ver_len, (uint16_t)(40 + 30 * i), 30, ext, sizeof(ext));
    log_i("GNSS:   ext[%u] \"%s\"", i, ext);
  }
}

void phase_detect() {
  publish_phase(GNSS_PHASE_DETECT);
  for (int attempt = 0; attempt < 2 && !s_ctx.have_mon_ver; ++attempt) {
    poll_mon_ver(GNSS_MONVER_TIMEOUT_MS);
  }
  if (!s_ctx.have_mon_ver) {
    s_ctx.prot_ver_x100 = -1;
    log_w("GNSS: no MON-VER reply; assuming a legacy (PROTVER < 27) receiver");
  } else {
    log_mon_ver();
    const int pv = ubxParseProtVer(s_ctx.mon_ver, s_ctx.mon_ver_len);
    s_ctx.prot_ver_x100 = pv > 0 ? pv : -1;
    if (pv > 0) {
      log_i("GNSS: PROTVER %d.%02d -> %s configuration", pv / 100, pv % 100, pv >= 2700 ? "VALSET" : "legacy CFG");
    } else {
      log_w("GNSS: MON-VER has no PROTVER string (u-blox 6?); assuming legacy protocol");
    }
  }
  publish_phase(GNSS_PHASE_DETECT);
}

// ---------------------------------------------------------------- CONFIGURE
// Generation 9+ (M9/M10): CFG-VALSET into RAM+BBR, three separate frames so a
// single refused key cannot take the others down with it (a VALSET is
// all-or-nothing per frame). Returns false when every essential step failed,
// which means "this is not really a VALSET receiver": use the legacy path.
bool configure_valset() {
  uint8_t pl[UBX_VALSET_HEADER_LEN + 2 * (4 + 4)];
  uint16_t len;

  // (1) UART1 output protocols: UBX on, NMEA off (NMEA would waste half the
  //     bandwidth at 9600 and delay every ACK).
  len = ubxValsetBegin(pl, sizeof(pl));
  ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_UART1OUTPROT_UBX, 1);
  ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_UART1OUTPROT_NMEA, 0);
  const int r_prot = ubx_send_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, pl, len);
  log_i("GNSS: VALSET UART1OUTPROT UBX=1 NMEA=0: %s", ack_str(r_prot));

  // (2) NAV-PVT every navigation epoch on UART1.
  len = ubxValsetBegin(pl, sizeof(pl));
  ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_MSGOUT_UBX_NAV_PVT_UART1, 1);
  const int r_msg = ubx_send_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, pl, len);
  log_i("GNSS: VALSET MSGOUT NAV-PVT UART1=1: %s", ack_str(r_msg));

  // (3) Measurement / navigation rate.
  len = ubxValsetBegin(pl, sizeof(pl));
  ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_RATE_MEAS, (uint32_t)GNSS_RATE_MS);
  ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_RATE_NAV, 1);
  const int r_rate = ubx_send_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, pl, len);
  log_i("GNSS: VALSET RATE MEAS=%u NAV=1: %s", (unsigned)GNSS_RATE_MS, ack_str(r_rate));

  if (r_prot <= 0 && r_msg <= 0 && r_rate <= 0) {
    log_w("GNSS: every VALSET refused despite PROTVER >= 27; falling back to legacy CFG messages");
    return false;
  }

#if GNSS_DYNMODEL_SEA
  // (4) Dynamic platform model SEA: assumes zero vertical velocity and sea
  //     level altitude, which steadies the speed estimate on a boat.
  len = ubxValsetBegin(pl, sizeof(pl));
  ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_NAVSPG_DYNMODEL, 5);
  const int r_dyn = ubx_send_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, pl, len);
  log_i("GNSS: VALSET NAVSPG-DYNMODEL=5 (SEA): %s", ack_str(r_dyn));
  (void)r_dyn;  // log-only (log_i is empty below CORE_DEBUG_LEVEL 3)
#endif

  s_ctx.configured = (r_msg == 1);  // the message-enable step is what makes data flow
  return true;
}

// u-blox 6/7/M8 (PROTVER < 27) and the VALSET fallback.
void configure_legacy(int prot_ver) {
  // NMEA off FIRST: on a factory module at 9600 the NMEA burst occupies most
  // of the line, so every later ACK would queue behind it.
  int nmea_off = 0;
  for (size_t i = 0; i < sizeof(kNmeaIds); ++i) {
    // CFG-MSG with a 3-byte payload sets the rate on the current port only.
    const uint8_t pl[3] = {0xF0, kNmeaIds[i], 0x00};
    const int r = ubx_send_ack(UBX_CLASS_CFG, UBX_CFG_MSG, pl, 3);
    if (r == 1) nmea_off++;
    log_i("GNSS: CFG-MSG NMEA F0 %02X off: %s", kNmeaIds[i], ack_str(r));
  }
  log_i("GNSS: NMEA off: %d/%u ACKed", nmea_off, (unsigned)sizeof(kNmeaIds));
  (void)nmea_off;  // log-only

  // Navigation rate: measRate ms, navRate 1 cycle, timeRef 1 = GPS time.
  uint8_t rate[6];
  wrU2(rate + 0, (uint16_t)GNSS_RATE_MS);
  wrU2(rate + 2, 1);
  wrU2(rate + 4, 1);
  const int r_rate = ubx_send_ack(UBX_CLASS_CFG, UBX_CFG_RATE, rate, 6);
  log_i("GNSS: CFG-RATE meas=%u nav=1: %s", (unsigned)GNSS_RATE_MS, ack_str(r_rate));
  (void)r_rate;  // log-only: the rate is a nicety, data flow does not depend on it

  // NAV-PVT exists from PROTVER 14 (u-blox 7). Older firmware NAKs the
  // unknown message id, in which case NAV-VELNED (u-blox 6) carries the speed.
  int r_pvt = -1;
  if (prot_ver >= 1400) {
    const uint8_t pl[3] = {UBX_CLASS_NAV, UBX_NAV_PVT, 0x01};
    r_pvt = ubx_send_ack(UBX_CLASS_CFG, UBX_CFG_MSG, pl, 3);
    log_i("GNSS: CFG-MSG NAV-PVT on: %s", ack_str(r_pvt));
  } else {
    log_i("GNSS: PROTVER %d.%02d < 14.00, NAV-PVT unavailable", prot_ver / 100, prot_ver % 100);
  }
  int r_vel = -1;
  if (r_pvt != 1) {
    const uint8_t pl[3] = {UBX_CLASS_NAV, UBX_NAV_VELNED, 0x01};
    r_vel = ubx_send_ack(UBX_CLASS_CFG, UBX_CFG_MSG, pl, 3);
    s_ctx.use_velned = true;
    log_i("GNSS: CFG-MSG NAV-VELNED on (fallback): %s", ack_str(r_vel));
  }

#if GNSS_DYNMODEL_SEA
  // CFG-NAV5: mask bit0 (dyn) selects only the dynModel field for update.
  uint8_t nav5[36];
  memset(nav5, 0, sizeof(nav5));
  wrU2(nav5 + 0, 0x0001);  // mask: apply dynamic model setting
  nav5[2] = 5;             // dynModel SEA
  const int r_nav5 = ubx_send_ack(UBX_CLASS_CFG, UBX_CFG_NAV5, nav5, 36);
  log_i("GNSS: CFG-NAV5 dynModel=5 (SEA): %s", ack_str(r_nav5));
  (void)r_nav5;  // log-only
#endif

  s_ctx.configured = (r_pvt == 1) || (r_vel == 1);
}

void phase_configure() {
  s_ctx.configured = false;
  s_ctx.use_velned = false;
  publish_phase(GNSS_PHASE_CONFIGURE);

  const int prot_ver = effective_prot_ver();
  bool done = false;
  if (prot_ver >= 2700) done = configure_valset();
  if (!done) configure_legacy(prot_ver);

  log_i("GNSS: configured=%d velned=%d (baud %lu, PROTVER %d.%02d%s)", s_ctx.configured ? 1 : 0,
        s_ctx.use_velned ? 1 : 0, (unsigned long)s_ctx.baud, prot_ver / 100, prot_ver % 100,
        s_ctx.prot_ver_x100 > 0 ? "" : " assumed");
  publish_phase(GNSS_PHASE_CONFIGURE);
}

// ---------------------------------------------------------------- RUN
void phase_run() {
  publish_phase(GNSS_PHASE_RUN);
  // Grace period starts now, not at the last config ACK, so a receiver that
  // needs a moment after reconfiguration is not immediately re-detected.
  s_ctx.last_good_frame_ms = now_nz();
  for (;;) {
    hb_touch(hb_gnss);
    // pump_rx returns after each frame; drain what queued up (e.g. behind a
    // multi-second e-paper refresh) with a bound so a flood cannot hold us.
    for (int frames = 0; frames < 64 && pump_rx(UBX_ANY, UBX_ANY); ++frames) {
    }
    const uint32_t silent_ms = (uint32_t)(millis() - s_ctx.last_good_frame_ms);
    if (silent_ms > (uint32_t)GNSS_REDETECT_MS) {
      s_ctx.redetects++;
      log_w("GNSS: no valid UBX frame for %lu ms (module reset or baud lost); redetect #%lu",
            (unsigned long)silent_ms, (unsigned long)s_ctx.redetects);
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(kPumpSliceMs));
  }
}

void gnss_task(void *) {
  for (;;) {
    phase_autobaud();
    phase_detect();
    phase_configure();
    phase_run();
  }
}

}  // namespace

// ---------------------------------------------------------------- public API
bool gnss_start() {
  // The RX ring must be sized BEFORE begin(): setRxBufferSize() returns 0 (and
  // keeps the 256-byte default) once the driver is running. 2048 bytes bridge a
  // multi-second e-paper refresh at 1 Hz NAV-PVT (100 bytes/epoch).
  const size_t rx_size = GNSS.setRxBufferSize(GNSS_RX_BUFFER);
  GNSS.begin(kBauds[0], SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
  if (rx_size == 0) {
    log_w("GNSS: setRxBufferSize(%u) returned 0 (called after begin?); RX ring stays at the default", (unsigned)GNSS_RX_BUFFER);
  } else {
    log_i("GNSS: Serial1 (HP UART1) RX=GPIO%d TX=GPIO%d ring=%u B, first baud %lu", PIN_GNSS_RX, PIN_GNSS_TX,
          (unsigned)rx_size, (unsigned long)kBauds[0]);
  }

  const BaseType_t ok = xTaskCreate(gnss_task, "gnss", GNSS_TASK_STACK, nullptr, TASK_PRIO_GNSS, nullptr);
  if (ok != pdPASS) {
    log_e("GNSS: xTaskCreate failed (%ld)", (long)ok);
    return false;
  }
  hb_touch(hb_gnss);  // the task has not run yet; do not look dead to the supervisor in the meantime
  return true;
}

const char *gnss_phase_str(uint8_t phase) {
  switch (phase) {
    case GNSS_PHASE_AUTOBAUD: return "AUTOBAUD";
    case GNSS_PHASE_DETECT: return "DETECT";
    case GNSS_PHASE_CONFIGURE: return "CONFIG";
    case GNSS_PHASE_RUN: return "RUN";
    default: return "?";
  }
}

void gnss_log_summary(const SharedState &s, uint32_t now) {
  const GnssState &g = s.gnss;
  char age[16];
  if (g.last_pvt_ms == 0) {
    snprintf(age, sizeof(age), "never");
  } else {
    snprintf(age, sizeof(age), "%lums", (unsigned long)age_ms(g.last_pvt_ms, now));
  }
  char protver[12];
  if (g.prot_ver_x100 > 0) {
    snprintf(protver, sizeof(protver), "%d.%02d", g.prot_ver_x100 / 100, g.prot_ver_x100 % 100);
  } else {
    snprintf(protver, sizeof(protver), "?");
  }
  [[maybe_unused]] const float speed = (float)g.gspeed_mm_s * SPEED_FACTOR;  // log-only
  log_i("GNSS phase=%s baud=%lu protver=%s cfg=%d%s fix=%u ok=%d sats=%u spd=%.2f%s sAcc=%.2fm/s pDOP=%.2f "
        "utc=%02u:%02u:%02u%s age=%s good=%lu bad=%lu redetect=%lu",
        gnss_phase_str(g.phase), (unsigned long)g.baud, protver, g.configured ? 1 : 0, g.use_velned ? " velned" : "",
        (unsigned)g.fix_type, g.fix_ok ? 1 : 0, (unsigned)g.num_sv, (double)speed, SPEED_UNIT_STR,
        (double)g.sacc_mm_s / 1000.0, (double)g.pdop_x100 / 100.0, (unsigned)g.hour, (unsigned)g.min,
        (unsigned)g.sec, g.time_valid ? "" : "?", age, (unsigned long)g.good_frames, (unsigned long)g.bad_frames,
        (unsigned long)g.redetects);
}
