// Native (host) unit tests for include/ubx_min.h: UBX framing, checksum,
// NAV-PVT / NAV-VELNED decoding, MON-VER PROTVER parsing, VALSET building.
//   pio test -e native
// Arduino-free: only ubx_min.h and <unity.h>.
#include <unity.h>

#include <stdint.h>
#include <string.h>

#include "ubx_min.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- helpers
// Feeds a whole buffer; returns how many complete frames feed() reported.
static int feedAll(UbxParser &p, const uint8_t *b, size_t n) {
  int frames = 0;
  for (size_t i = 0; i < n; ++i)
    if (p.feed(b[i])) frames++;
  return frames;
}

// Synthetic NAV-PVT payload (84 or 92 bytes): fixType 3, flags 1 (gnssFixOK),
// numSV 9, gSpeed 5144 mm/s (= 10.0 kn), sAcc 300 mm/s, pDOP 1.50,
// valid 0x07, 12:34:56, lat/lon of a marina in Kiel.
static void makeNavPvtPayload(uint8_t *pl, uint16_t len) {
  memset(pl, 0, len);
  wrU4(pl + 0, 123456789u);  // iTOW
  wrU2(pl + 4, 2026);        // year
  pl[6] = 9;                 // month
  pl[7] = 4;                 // day
  pl[8] = 12;                // hour
  pl[9] = 34;                // min
  pl[10] = 56;               // sec
  pl[11] = 0x07;             // valid: date, time, fullyResolved
  pl[20] = 3;                // fixType 3D
  pl[21] = 0x01;             // flags: gnssFixOK
  pl[23] = 9;                // numSV
  wrU4(pl + 24, (uint32_t)101340000);   // lon 10.134 deg
  wrU4(pl + 28, (uint32_t)543210000);   // lat 54.321 deg
  wrU4(pl + 32, (uint32_t)45000);       // height mm
  wrU4(pl + 36, (uint32_t)2000);        // hMSL mm
  wrU4(pl + 40, 3500);                  // hAcc
  wrU4(pl + 44, 5000);                  // vAcc
  wrU4(pl + 48, (uint32_t)3000);        // velN
  wrU4(pl + 52, (uint32_t)(-4180));     // velE
  wrU4(pl + 56, (uint32_t)(-50));       // velD
  wrU4(pl + 60, (uint32_t)5144);        // gSpeed mm/s
  wrU4(pl + 64, (uint32_t)30500000);    // headMot 305.00000 deg
  wrU4(pl + 68, 300);                   // sAcc mm/s
  wrU4(pl + 72, 1500000);               // headAcc
  wrU2(pl + 76, 150);                   // pDOP 1.50
}

static size_t makeNavPvtFrame(uint8_t *out, size_t cap, uint16_t payloadLen) {
  uint8_t pl[92];
  makeNavPvtPayload(pl, payloadLen);
  return ubxBuildFrame(out, cap, UBX_CLASS_NAV, UBX_NAV_PVT, pl, payloadLen);
}

// ---------------------------------------------------------------- checksum / builder
void test_monver_poll_checksum() {
  uint8_t out[16];
  const size_t n = ubxBuildFrame(out, sizeof(out), UBX_CLASS_MON, UBX_MON_VER, nullptr, 0);
  const uint8_t expected[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
  TEST_ASSERT_EQUAL_UINT32(8, (uint32_t)n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, 8);

  uint8_t a = 0, b = 0;
  ubxChecksum(expected + 2, 4, a, b);
  TEST_ASSERT_EQUAL_HEX8(0x0E, a);
  TEST_ASSERT_EQUAL_HEX8(0x34, b);
}

void test_build_frame_rejects_small_buffer() {
  uint8_t out[7];
  TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)ubxBuildFrame(out, sizeof(out), UBX_CLASS_MON, UBX_MON_VER, nullptr, 0));
  uint8_t pl[3] = {1, 2, 3};
  TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)ubxBuildFrame(out, 10, UBX_CLASS_CFG, UBX_CFG_MSG, pl, 3));
  TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)ubxBuildFrame(out, 32, UBX_CLASS_CFG, UBX_CFG_MSG, nullptr, 3));
}

// VALSET reference frame from the plan: NMEA=0, UBX=1, MSGOUT NAV-PVT=1,
// RATE-MEAS=1000, RATE-NAV=1 in one frame (production sends them as three
// frames; the byte layout per key/value is what this pins down).
void test_valset_reference_frame() {
  uint8_t pl[64];
  uint16_t len = ubxValsetBegin(pl, sizeof(pl));
  TEST_ASSERT_EQUAL_UINT16(4, len);
  TEST_ASSERT_TRUE(ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_UART1OUTPROT_NMEA, 0));
  TEST_ASSERT_TRUE(ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_UART1OUTPROT_UBX, 1));
  TEST_ASSERT_TRUE(ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_MSGOUT_UBX_NAV_PVT_UART1, 1));
  TEST_ASSERT_TRUE(ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_RATE_MEAS, 1000));
  TEST_ASSERT_TRUE(ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_RATE_NAV, 1));
  TEST_ASSERT_EQUAL_UINT16(0x1F, len);

  uint8_t frame[80];
  const size_t n = ubxBuildFrame(frame, sizeof(frame), UBX_CLASS_CFG, UBX_CFG_VALSET, pl, len);
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x8A, 0x1F, 0x00,             // header
                              0x00, 0x03, 0x00, 0x00,                          // version, layers RAM+BBR, reserved
                              0x02, 0x00, 0x74, 0x10, 0x00,                    // UART1OUTPROT-NMEA = 0
                              0x01, 0x00, 0x74, 0x10, 0x01,                    // UART1OUTPROT-UBX = 1
                              0x07, 0x00, 0x91, 0x20, 0x01,                    // MSGOUT-UBX_NAV_PVT_UART1 = 1
                              0x01, 0x00, 0x21, 0x30, 0xE8, 0x03,              // RATE-MEAS = 1000
                              0x02, 0x00, 0x21, 0x30, 0x01, 0x00,              // RATE-NAV = 1
                              0x08, 0x31};                                     // checksum
  TEST_ASSERT_EQUAL_UINT32(sizeof(expected), (uint32_t)n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, sizeof(expected));
}

void test_valset_value_sizes_and_capacity() {
  TEST_ASSERT_EQUAL_UINT8(1, ubxValsetValueSize(UBX_KEY_CFG_UART1OUTPROT_UBX));   // L
  TEST_ASSERT_EQUAL_UINT8(1, ubxValsetValueSize(UBX_KEY_CFG_NAVSPG_DYNMODEL));    // U1/E1
  TEST_ASSERT_EQUAL_UINT8(2, ubxValsetValueSize(UBX_KEY_CFG_RATE_MEAS));          // U2
  TEST_ASSERT_EQUAL_UINT8(4, ubxValsetValueSize(0x40000001u));                    // U4
  TEST_ASSERT_EQUAL_UINT8(0, ubxValsetValueSize(0x00000001u));                    // invalid
  uint8_t pl[9];
  uint16_t len = ubxValsetBegin(pl, sizeof(pl));
  TEST_ASSERT_TRUE(ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_NAVSPG_DYNMODEL, 5));  // 4 + 5 = 9 fits
  TEST_ASSERT_EQUAL_UINT16(9, len);
  TEST_ASSERT_FALSE(ubxValsetAppend(pl, len, sizeof(pl), UBX_KEY_CFG_NAVSPG_DYNMODEL, 5));  // no room
  TEST_ASSERT_EQUAL_UINT16(9, len);
  TEST_ASSERT_FALSE(ubxValsetAppend(pl, len, 64, 0x50000001u, 1));  // U8 unsupported
}

// ---------------------------------------------------------------- NAV-PVT
void test_navpvt_92_bytes_decodes() {
  uint8_t frame[128];
  const size_t n = makeNavPvtFrame(frame, sizeof(frame), 92);
  TEST_ASSERT_EQUAL_UINT32(100, (uint32_t)n);

  UbxParser p;
  bool done = false;
  for (size_t i = 0; i < n; ++i) {
    const bool r = p.feed(frame[i]);
    if (i + 1 < n) TEST_ASSERT_FALSE(r);  // only the last byte completes the frame
    done = r;
  }
  TEST_ASSERT_TRUE(done);
  TEST_ASSERT_EQUAL_UINT32(1, p.goodFrames);
  TEST_ASSERT_EQUAL_UINT32(0, p.badFrames);
  TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_NAV, p.cls());
  TEST_ASSERT_EQUAL_HEX8(UBX_NAV_PVT, p.id());
  TEST_ASSERT_EQUAL_UINT16(92, p.len());
  TEST_ASSERT_TRUE(p.matches(UBX_CLASS_NAV, UBX_NAV_PVT));
  TEST_ASSERT_TRUE(p.matches(UBX_ANY, UBX_ANY));
  TEST_ASSERT_TRUE(p.matches(UBX_CLASS_NAV, UBX_ANY));
  TEST_ASSERT_FALSE(p.matches(UBX_CLASS_ACK, UBX_ANY));

  NavPvt o;
  TEST_ASSERT_TRUE(decodeNavPvt(p, o));
  TEST_ASSERT_EQUAL_UINT32(123456789u, o.iTOW);
  TEST_ASSERT_EQUAL_UINT16(2026, o.year);
  TEST_ASSERT_EQUAL_UINT8(9, o.month);
  TEST_ASSERT_EQUAL_UINT8(4, o.day);
  TEST_ASSERT_EQUAL_UINT8(12, o.hour);
  TEST_ASSERT_EQUAL_UINT8(34, o.min);
  TEST_ASSERT_EQUAL_UINT8(56, o.sec);
  TEST_ASSERT_EQUAL_HEX8(0x07, o.valid);
  TEST_ASSERT_EQUAL_UINT8(3, o.fixType);
  TEST_ASSERT_EQUAL_HEX8(0x01, o.flags);
  TEST_ASSERT_EQUAL_UINT8(9, o.numSV);
  TEST_ASSERT_EQUAL_INT32(101340000, o.lon);
  TEST_ASSERT_EQUAL_INT32(543210000, o.lat);
  TEST_ASSERT_EQUAL_INT32(45000, o.height);
  TEST_ASSERT_EQUAL_INT32(2000, o.hMSL);
  TEST_ASSERT_EQUAL_UINT32(3500, o.hAcc);
  TEST_ASSERT_EQUAL_UINT32(5000, o.vAcc);
  TEST_ASSERT_EQUAL_INT32(3000, o.velN);
  TEST_ASSERT_EQUAL_INT32(-4180, o.velE);
  TEST_ASSERT_EQUAL_INT32(-50, o.velD);
  TEST_ASSERT_EQUAL_INT32(5144, o.gSpeed);
  TEST_ASSERT_EQUAL_INT32(30500000, o.headMot);
  TEST_ASSERT_EQUAL_UINT32(300, o.sAcc);
  TEST_ASSERT_EQUAL_UINT32(1500000, o.headAcc);
  TEST_ASSERT_EQUAL_UINT16(150, o.pDOP);
  TEST_ASSERT_EQUAL_UINT16(92, o.payloadLen);
  TEST_ASSERT_TRUE(pvtFixOk(o, 1800));
  TEST_ASSERT_TRUE(pvtFixOk(o, 3410));
}

void test_navpvt_84_bytes_accepted() {
  uint8_t frame[128];
  const size_t n = makeNavPvtFrame(frame, sizeof(frame), 84);
  TEST_ASSERT_EQUAL_UINT32(92, (uint32_t)n);
  UbxParser p;
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  NavPvt o;
  TEST_ASSERT_TRUE(decodeNavPvt(p, o));
  TEST_ASSERT_EQUAL_UINT16(84, o.payloadLen);
  TEST_ASSERT_EQUAL_INT32(5144, o.gSpeed);
  TEST_ASSERT_EQUAL_UINT32(300, o.sAcc);
  TEST_ASSERT_EQUAL_UINT16(150, o.pDOP);
  TEST_ASSERT_EQUAL_UINT8(3, o.fixType);
}

void test_navpvt_short_or_wrong_id_rejected() {
  uint8_t pl[92];
  makeNavPvtPayload(pl, 92);
  uint8_t frame[128];
  UbxParser p;
  NavPvt o;
  // 83 bytes: valid frame, but too short to be NAV-PVT.
  size_t n = ubxBuildFrame(frame, sizeof(frame), UBX_CLASS_NAV, UBX_NAV_PVT, pl, 83);
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  TEST_ASSERT_FALSE(decodeNavPvt(p, o));
  // Same payload under a different id.
  n = ubxBuildFrame(frame, sizeof(frame), UBX_CLASS_NAV, 0x35, pl, 92);
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  TEST_ASSERT_FALSE(decodeNavPvt(p, o));
  TEST_ASSERT_EQUAL_UINT32(2, p.goodFrames);
}

// ---------------------------------------------------------------- framing robustness
void test_corrupted_ckb_counts_bad_frame() {
  uint8_t frame[128];
  const size_t n = makeNavPvtFrame(frame, sizeof(frame), 92);
  frame[n - 1] ^= 0xFF;  // CK_B
  UbxParser p;
  TEST_ASSERT_EQUAL_INT(0, feedAll(p, frame, n));
  TEST_ASSERT_EQUAL_UINT32(1, p.badFrames);
  TEST_ASSERT_EQUAL_UINT32(0, p.goodFrames);
}

void test_corrupted_cka_counts_bad_frame() {
  uint8_t frame[128];
  const size_t n = makeNavPvtFrame(frame, sizeof(frame), 92);
  frame[n - 2] ^= 0x01;  // CK_A
  UbxParser p;
  TEST_ASSERT_EQUAL_INT(0, feedAll(p, frame, n));
  TEST_ASSERT_EQUAL_UINT32(1, p.badFrames);
  TEST_ASSERT_EQUAL_UINT32(0, p.goodFrames);
}

void test_corrupted_payload_counts_bad_frame() {
  uint8_t frame[128];
  const size_t n = makeNavPvtFrame(frame, sizeof(frame), 92);
  frame[6 + 60] ^= 0x10;  // flip a bit in gSpeed
  UbxParser p;
  TEST_ASSERT_EQUAL_INT(0, feedAll(p, frame, n));
  TEST_ASSERT_EQUAL_UINT32(1, p.badFrames);
  TEST_ASSERT_EQUAL_UINT32(0, p.goodFrames);
}

void test_resync_after_garbage() {
  // NMEA-ish garbage including stray sync bytes, then a valid frame.
  const uint8_t garbage[] = {'$', 'G', 'N', 'R', 'M', 'C', ',', 0xB5, 0x00, 0x62, 0xB5, 0xB5, 0x13, '*', '4', 'A', '\r', '\n'};
  uint8_t frame[128];
  const size_t n = makeNavPvtFrame(frame, sizeof(frame), 92);
  UbxParser p;
  TEST_ASSERT_EQUAL_INT(0, feedAll(p, garbage, sizeof(garbage)));
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  TEST_ASSERT_EQUAL_UINT32(1, p.goodFrames);
  NavPvt o;
  TEST_ASSERT_TRUE(decodeNavPvt(p, o));
  TEST_ASSERT_EQUAL_INT32(5144, o.gSpeed);
  // A second frame right behind the first.
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  TEST_ASSERT_EQUAL_UINT32(2, p.goodFrames);
  TEST_ASSERT_EQUAL_UINT32(0, p.badFrames);
}

void test_double_sync_byte_resyncs() {
  // "B5 B5 62 ..." : the second B5 is the real sync 1.
  uint8_t frame[128];
  const size_t n = makeNavPvtFrame(frame, sizeof(frame), 84);
  UbxParser p;
  TEST_ASSERT_FALSE(p.feed(0xB5));
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  TEST_ASSERT_EQUAL_UINT32(1, p.goodFrames);
}

void test_oversized_length_rejected() {
  // Header claiming len 513 must be dropped immediately (badFrames++), and the
  // parser must accept the next valid frame.
  const uint8_t bogus[] = {0xB5, 0x62, 0x01, 0x07, 0x01, 0x02};  // len = 0x0201 = 513
  UbxParser p;
  TEST_ASSERT_EQUAL_INT(0, feedAll(p, bogus, sizeof(bogus)));
  TEST_ASSERT_EQUAL_UINT32(1, p.badFrames);
  uint8_t frame[128];
  const size_t n = makeNavPvtFrame(frame, sizeof(frame), 92);
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  TEST_ASSERT_EQUAL_UINT32(1, p.goodFrames);
}

void test_max_length_frame_accepted() {
  // Exactly 512 bytes of payload is the accepted maximum (long MON-VER).
  static uint8_t pl[512];
  for (size_t i = 0; i < sizeof(pl); ++i) pl[i] = (uint8_t)(i * 7);
  static uint8_t frame[512 + 8];
  const size_t n = ubxBuildFrame(frame, sizeof(frame), UBX_CLASS_MON, UBX_MON_VER, pl, 512);
  TEST_ASSERT_EQUAL_UINT32(520, (uint32_t)n);
  UbxParser p;
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  TEST_ASSERT_EQUAL_UINT16(512, p.len());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(pl, p.payload(), 512);
}

void test_zero_length_frame_and_ack_matching() {
  // A poll (len 0) parses, and an ACK-ACK for CFG-MSG carries {cls,id}.
  UbxParser p;
  const uint8_t poll[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, poll, sizeof(poll)));
  TEST_ASSERT_EQUAL_UINT16(0, p.len());
  TEST_ASSERT_TRUE(p.matches(UBX_CLASS_MON, UBX_MON_VER));

  uint8_t ackPl[2] = {UBX_CLASS_CFG, UBX_CFG_MSG};
  uint8_t frame[16];
  const size_t n = ubxBuildFrame(frame, sizeof(frame), UBX_CLASS_ACK, UBX_ACK_ACK, ackPl, 2);
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  TEST_ASSERT_TRUE(p.matches(UBX_CLASS_ACK, UBX_ANY));   // "any ACK class message"
  TEST_ASSERT_TRUE(p.matches(UBX_CLASS_ACK, UBX_ACK_ACK));
  TEST_ASSERT_FALSE(p.matches(UBX_CLASS_ACK, UBX_ACK_NAK));
  TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_CFG, p.payload()[0]);
  TEST_ASSERT_EQUAL_HEX8(UBX_CFG_MSG, p.payload()[1]);
}

// ---------------------------------------------------------------- NAV-VELNED
void test_navvelned_decodes_cm_to_mm() {
  uint8_t pl[36];
  memset(pl, 0, sizeof(pl));
  wrU4(pl + 20, 514);  // gSpeed 514 cm/s
  wrU4(pl + 28, 30);   // sAcc 30 cm/s
  uint8_t frame[64];
  const size_t n = ubxBuildFrame(frame, sizeof(frame), UBX_CLASS_NAV, UBX_NAV_VELNED, pl, 36);
  UbxParser p;
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n));
  int32_t gs = 0;
  uint32_t sa = 0;
  TEST_ASSERT_TRUE(decodeNavVelned(p, gs, sa));
  TEST_ASSERT_EQUAL_INT32(5140, gs);
  TEST_ASSERT_EQUAL_UINT32(300, sa);
  // Wrong length is rejected.
  const size_t n2 = ubxBuildFrame(frame, sizeof(frame), UBX_CLASS_NAV, UBX_NAV_VELNED, pl, 35);
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, frame, n2));
  TEST_ASSERT_FALSE(decodeNavVelned(p, gs, sa));
}

// ---------------------------------------------------------------- MON-VER
static void putExt(uint8_t *payload, int n, const char *s) {
  uint8_t *e = payload + 40 + 30 * n;
  memset(e, 0, 30);
  const size_t l = strlen(s);
  memcpy(e, s, l < 30 ? l : 30);
}

void test_protver_equals_spelling() {
  uint8_t pl[40 + 30 * 3];
  memset(pl, 0, sizeof(pl));
  memcpy(pl, "EXT CORE 3.01 (107900)", 22);
  memcpy(pl + 30, "00080000", 8);
  putExt(pl, 0, "FWVER=SPG 3.01");
  putExt(pl, 1, "PROTVER=18.00");
  putExt(pl, 2, "GPS;GLO;GAL;BDS");
  TEST_ASSERT_EQUAL_INT(1800, ubxParseProtVer(pl, sizeof(pl)));
  char sw[31], hw[11], ext[31];
  ubxCopyVerField(pl, sizeof(pl), 0, 30, sw, sizeof(sw));
  ubxCopyVerField(pl, sizeof(pl), 30, 10, hw, sizeof(hw));
  ubxCopyVerField(pl, sizeof(pl), 40, 30, ext, sizeof(ext));
  TEST_ASSERT_EQUAL_STRING("EXT CORE 3.01 (107900)", sw);
  TEST_ASSERT_EQUAL_STRING("00080000", hw);
  TEST_ASSERT_EQUAL_STRING("FWVER=SPG 3.01", ext);
}

void test_protver_space_spelling() {
  uint8_t pl[40 + 30 * 2];
  memset(pl, 0, sizeof(pl));
  putExt(pl, 0, "PROTVER 15.00");
  putExt(pl, 1, "ANTSUPERV=AC SD PDoS SR");
  TEST_ASSERT_EQUAL_INT(1500, ubxParseProtVer(pl, sizeof(pl)));
}

void test_protver_missing_or_odd() {
  uint8_t pl[40 + 30 * 2];
  memset(pl, 0, sizeof(pl));
  memcpy(pl, "7.03 (45969)", 12);
  putExt(pl, 0, "FWVER=SPG 1.00");
  putExt(pl, 1, "MOD=NEO-6M");
  TEST_ASSERT_EQUAL_INT(0, ubxParseProtVer(pl, sizeof(pl)));
  TEST_ASSERT_EQUAL_INT(0, ubxParseProtVer(pl, 40));   // no extensions at all
  TEST_ASSERT_EQUAL_INT(0, ubxParseProtVer(pl, 0));
  TEST_ASSERT_EQUAL_INT(0, ubxParseProtVer(nullptr, 100));
  // Partial trailing extension (len not a multiple of 30) is ignored, earlier ones still parse.
  putExt(pl, 0, "PROTVER=27.11");
  TEST_ASSERT_EQUAL_INT(2711, ubxParseProtVer(pl, 40 + 30 + 17));
  // Other decimal shapes.
  putExt(pl, 0, "PROTVER=34.10");
  TEST_ASSERT_EQUAL_INT(3410, ubxParseProtVer(pl, sizeof(pl)));
  putExt(pl, 0, "PROTVER=23");
  TEST_ASSERT_EQUAL_INT(2300, ubxParseProtVer(pl, sizeof(pl)));
  putExt(pl, 0, "PROTVER=18.5");
  TEST_ASSERT_EQUAL_INT(1850, ubxParseProtVer(pl, sizeof(pl)));
  putExt(pl, 0, "PROTVER=");
  TEST_ASSERT_EQUAL_INT(0, ubxParseProtVer(pl, sizeof(pl)));
  // Garbage digit runs must not overflow the int: unknown (0), never UB.
  putExt(pl, 0, "PROTVER=99999999999999999999");
  TEST_ASSERT_EQUAL_INT(0, ubxParseProtVer(pl, sizeof(pl)));
  putExt(pl, 0, "PROTVER=12345.00");
  TEST_ASSERT_EQUAL_INT(0, ubxParseProtVer(pl, sizeof(pl)));
}

// ---------------------------------------------------------------- fix gate
void test_pvt_fix_ok_rules() {
  NavPvt o;
  memset(&o, 0, sizeof(o));
  o.fixType = 3;
  o.flags = 0;
  TEST_ASSERT_TRUE(pvtFixOk(o, 1500));    // pre-PROTVER 20: flags bit0 does not exist -> fixType alone
  TEST_ASSERT_TRUE(pvtFixOk(o, 1999));
  TEST_ASSERT_FALSE(pvtFixOk(o, 2000));   // gnssFixOK required from PROTVER 20 on
  TEST_ASSERT_FALSE(pvtFixOk(o, 2700));
  o.flags = 0x01;
  TEST_ASSERT_TRUE(pvtFixOk(o, 2700));
  o.fixType = 0;
  TEST_ASSERT_FALSE(pvtFixOk(o, 1500));
  TEST_ASSERT_FALSE(pvtFixOk(o, 2700));
  o.fixType = 2;
  TEST_ASSERT_TRUE(pvtFixOk(o, 2700));
  o.fixType = 4;
  TEST_ASSERT_TRUE(pvtFixOk(o, 2700));
  o.fixType = 1;  // dead reckoning only
  TEST_ASSERT_FALSE(pvtFixOk(o, 1500));
  o.fixType = 5;  // time only
  TEST_ASSERT_FALSE(pvtFixOk(o, 1500));
}

// ---------------------------------------------------------------- byte access
void test_little_endian_readers() {
  const uint8_t b[] = {0x78, 0x56, 0x34, 0x12, 0xFF, 0xFF, 0xFF, 0xFF};
  TEST_ASSERT_EQUAL_HEX8(0x78, rdU1(b));
  TEST_ASSERT_EQUAL_HEX16(0x5678, rdU2(b));
  TEST_ASSERT_EQUAL_HEX32(0x12345678u, rdU4(b));
  TEST_ASSERT_EQUAL_INT32(-1, rdI4(b + 4));
  uint8_t w[6];
  wrU2(w, 0xBEEF);
  wrU4(w + 2, 0xDEADC0DEu);
  const uint8_t expected[] = {0xEF, 0xBE, 0xDE, 0xC0, 0xAD, 0xDE};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, w, 6);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_monver_poll_checksum);
  RUN_TEST(test_build_frame_rejects_small_buffer);
  RUN_TEST(test_valset_reference_frame);
  RUN_TEST(test_valset_value_sizes_and_capacity);
  RUN_TEST(test_navpvt_92_bytes_decodes);
  RUN_TEST(test_navpvt_84_bytes_accepted);
  RUN_TEST(test_navpvt_short_or_wrong_id_rejected);
  RUN_TEST(test_corrupted_ckb_counts_bad_frame);
  RUN_TEST(test_corrupted_cka_counts_bad_frame);
  RUN_TEST(test_corrupted_payload_counts_bad_frame);
  RUN_TEST(test_resync_after_garbage);
  RUN_TEST(test_double_sync_byte_resyncs);
  RUN_TEST(test_oversized_length_rejected);
  RUN_TEST(test_max_length_frame_accepted);
  RUN_TEST(test_zero_length_frame_and_ack_matching);
  RUN_TEST(test_navvelned_decodes_cm_to_mm);
  RUN_TEST(test_protver_equals_spelling);
  RUN_TEST(test_protver_space_spelling);
  RUN_TEST(test_protver_missing_or_odd);
  RUN_TEST(test_pvt_fix_ok_rules);
  RUN_TEST(test_little_endian_readers);
  return UNITY_END();
}
