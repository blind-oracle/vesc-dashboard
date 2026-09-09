// Native (host) unit tests for include/jk_bms.h: command builder / sum8, frame
// CRC, the notification assembler (every chunking seen in the field, ACK
// echoes, "AT\r\n", garbage, partial preambles), cell info / device info
// decoding of the recorded frames in jk_frames.h, layout detection and the
// error bit names.
//   pio test -e native -f test_jk_bms
// Framework-free: only jk_bms.h, jk_frames.h and <unity.h>.
#include <unity.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "jk_bms.h"
#include "jk_frames.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- helpers
// The 20-byte ACK echo the BMS appends to command-triggered frames.
static const uint8_t kAckEcho[20] = {0xAA, 0x55, 0x90, 0xEB, 0xC8, 0x01, 0x01, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44};
static const uint8_t kAt[4] = {'A', 'T', '\r', '\n'};

// Callback sink: keeps up to 4 delivered frames.
struct Sink {
  int count;
  uint8_t frames[4][JK_FRAME_LEN];
};

static void sinkCb(const uint8_t frame[JK_FRAME_LEN], void *ctx) {
  Sink *s = (Sink *)ctx;
  if (s->count < 4) memcpy(s->frames[s->count], frame, JK_FRAME_LEN);
  s->count++;
}

static void sinkReset(Sink &s) { memset(&s, 0, sizeof s); }

// Feeds buf in chunks of the given sizes (sizes must sum to n).
static void feedChunks(JkAssembler &a, const uint8_t *buf, const size_t *sizes, size_t nsizes, Sink &s) {
  size_t off = 0;
  for (size_t i = 0; i < nsizes; ++i) {
    jk_feed(a, buf + off, sizes[i], sinkCb, &s);
    off += sizes[i];
  }
}

// Copies a recorded frame, lets the caller patch it, then fixes the CRC.
static void fixCrc(uint8_t frame[JK_FRAME_LEN]) { frame[JK_FRAME_LEN - 1] = jk_sum8(frame, JK_FRAME_LEN - 1); }

static void wr16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

// ---------------------------------------------------------------- sum8 / command builder
void test_sum8_and_build_cmd_reads() {
  const uint8_t abc[] = {0x01, 0x02, 0xFF};
  TEST_ASSERT_EQUAL_HEX8(0x02, jk_sum8(abc, 3));  // 0x102 mod 256
  TEST_ASSERT_EQUAL_HEX8(0x00, jk_sum8(abc, 0));

  uint8_t out[JK_CMD_LEN];
  memset(out, 0xEE, sizeof out);
  jk_build_cmd(out, JK_REG_DEVICE_INFO, 0, 0);
  const uint8_t dev[JK_CMD_LEN] = {0xAA, 0x55, 0x90, 0xEB, 0x97, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(dev, out, JK_CMD_LEN);
  TEST_ASSERT_EQUAL_HEX8(out[19], jk_sum8(out, 19));

  jk_build_cmd(out, JK_REG_CELL_INFO, 0, 0);
  const uint8_t cell[JK_CMD_LEN] = {0xAA, 0x55, 0x90, 0xEB, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(cell, out, JK_CMD_LEN);

  // The ACK echo is a sum8-consistent command frame too (reg 0xC8, len 1, value 1).
  TEST_ASSERT_EQUAL_HEX8(kAckEcho[19], jk_sum8(kAckEcho, 19));
}

void test_build_cmd_value_and_app_padding_variant() {
  uint8_t out[JK_CMD_LEN];
  jk_build_cmd(out, 0x12, 0x12345678u, 4);
  TEST_ASSERT_EQUAL_HEX8(0xAA, out[0]);
  TEST_ASSERT_EQUAL_HEX8(0x55, out[1]);
  TEST_ASSERT_EQUAL_HEX8(0x90, out[2]);
  TEST_ASSERT_EQUAL_HEX8(0xEB, out[3]);
  TEST_ASSERT_EQUAL_HEX8(0x12, out[4]);
  TEST_ASSERT_EQUAL_HEX8(0x04, out[5]);
  TEST_ASSERT_EQUAL_HEX8(0x78, out[6]);
  TEST_ASSERT_EQUAL_HEX8(0x56, out[7]);
  TEST_ASSERT_EQUAL_HEX8(0x34, out[8]);
  TEST_ASSERT_EQUAL_HEX8(0x12, out[9]);
  for (int i = 10; i < 19; ++i) TEST_ASSERT_EQUAL_HEX8(0x00, out[i]);
  // 0xAA+0x55+0x90+0xEB+0x12+0x04+0x78+0x56+0x34+0x12 = 932 = 0x3A4 -> 0xA4
  TEST_ASSERT_EQUAL_HEX8(0xA4, out[19]);
  TEST_ASSERT_EQUAL_HEX8(jk_sum8(out, 19), out[19]);

  // The JK app fills bytes 10..18 with junk; such a frame is still sum8-consistent
  // and identical to ours in bytes 0..9.
  uint8_t app[JK_CMD_LEN];
  memcpy(app, out, JK_CMD_LEN);
  const uint8_t junk[9] = {0x5A, 0x13, 0xC7, 0x00, 0xFF, 0x81, 0x22, 0x9E, 0x40};
  memcpy(app + 10, junk, 9);
  app[19] = jk_sum8(app, 19);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(out, app, 10);
  TEST_ASSERT_EQUAL_HEX8(jk_sum8(app, 19), app[19]);
  TEST_ASSERT_NOT_EQUAL(out[19], app[19]);  // the padding changes the checksum, so the app's frame differs
}

// ---------------------------------------------------------------- frame CRC
static const uint8_t *const kAllFrames[] = {
    kFrame24S_Comment16S,      kFrame24S_Dev_B2A24S20P,   kFrame24S_Cell_B2A24S20P,      kFrame24S_Cell_B1A20S15P_13S,
    kFrame32S_Dev_B2A8S20P,    kFrame32S_Cell_B2A8S20P,   kFrame32S_Settings_B2A8S20P,   kFrameDev_JK04_B2A16S_330,
    kFrameDev_24S_B2A24S15P_1007, kFrameDev_32S_PB2A16S15P_1420,
};
static const size_t kAllFramesCount = sizeof(kAllFrames) / sizeof(kAllFrames[0]);

void test_crc_all_recorded_frames_pass() {
  for (size_t i = 0; i < kAllFramesCount; ++i) {
    TEST_ASSERT_TRUE_MESSAGE(jk_frame_crc_ok(kAllFrames[i]), "recorded frame CRC");
    TEST_ASSERT_TRUE(jk_is_preamble(kAllFrames[i]));
  }
  TEST_ASSERT_EQUAL_HEX8(0xCD, kFrame24S_Comment16S[299]);
}

void test_crc_flipped_byte_fails() {
  uint8_t f[JK_FRAME_LEN];
  for (size_t i = 0; i < kAllFramesCount; ++i) {
    memcpy(f, kAllFrames[i], JK_FRAME_LEN);
    f[7] ^= 0x01;
    TEST_ASSERT_FALSE(jk_frame_crc_ok(f));
    f[7] ^= 0x01;
    f[299] ^= 0x80;
    TEST_ASSERT_FALSE(jk_frame_crc_ok(f));
  }
}

// ---------------------------------------------------------------- assembler: chunkings
static void assertOneFrame(JkAssembler &a, Sink &s, const uint8_t *expected) {
  TEST_ASSERT_EQUAL_INT(1, s.count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, s.frames[0], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_crc_bad);
  TEST_ASSERT_EQUAL_UINT16(0, a.len);
}

void test_asm_chunkings_seen_in_the_field() {
  JkAssembler a;
  Sink s;
  const uint8_t *f = kFrame24S_Comment16S;

  // MTU 23: 15 x 20.
  jk_asm_reset(a);
  sinkReset(s);
  for (int i = 0; i < 15; ++i) {
    jk_feed(a, f + 20 * i, 20, sinkCb, &s);
    if (i < 14) TEST_ASSERT_EQUAL_INT(0, s.count);
  }
  assertOneFrame(a, s, f);
  TEST_ASSERT_EQUAL_UINT32(0, a.resyncs);

  // 128 + 128 + 44.
  jk_asm_reset(a);
  sinkReset(s);
  const size_t c1[] = {128, 128, 44};
  feedChunks(a, f, c1, 3, s);
  assertOneFrame(a, s, f);

  // 150 + 150.
  jk_asm_reset(a);
  sinkReset(s);
  const size_t c2[] = {150, 150};
  feedChunks(a, f, c2, 2, s);
  assertOneFrame(a, s, f);

  // 128 + 128 + 22 + 22.
  jk_asm_reset(a);
  sinkReset(s);
  const size_t c3[] = {128, 128, 22, 22};
  feedChunks(a, f, c3, 4, s);
  assertOneFrame(a, s, f);

  // One byte at a time.
  jk_asm_reset(a);
  sinkReset(s);
  for (size_t i = 0; i < JK_FRAME_LEN; ++i) {
    jk_feed(a, f + i, 1, sinkCb, &s);
    if (i + 1 < JK_FRAME_LEN) TEST_ASSERT_EQUAL_INT(0, s.count);
  }
  assertOneFrame(a, s, f);
  TEST_ASSERT_EQUAL_UINT32(0, a.resyncs);

  // All 300 at once.
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, f, JK_FRAME_LEN, sinkCb, &s);
  assertOneFrame(a, s, f);
  TEST_ASSERT_EQUAL_UINT32(0, a.resyncs);
}

void test_asm_frame_plus_ack_echo_in_one_blob() {
  static uint8_t blob[320];
  memcpy(blob, kFrame24S_Comment16S, 300);
  memcpy(blob + 300, kAckEcho, 20);
  JkAssembler a;
  Sink s;
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, blob, 320, sinkCb, &s);
  assertOneFrame(a, s, kFrame24S_Comment16S);
  TEST_ASSERT_EQUAL_UINT32(1, a.resyncs);  // the echo was thrown away
  // The next frame still decodes cleanly.
  jk_feed(a, kFrame24S_Cell_B2A24S20P, 300, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(2, s.count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kFrame24S_Cell_B2A24S20P, s.frames[1], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_UINT32(2, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_crc_bad);
  TEST_ASSERT_EQUAL_UINT16(0, a.len);

  // The 320-byte blob in 20-byte chunks (MTU 23) behaves the same.
  jk_asm_reset(a);
  sinkReset(s);
  for (int i = 0; i < 16; ++i) jk_feed(a, blob + 20 * i, 20, sinkCb, &s);
  assertOneFrame(a, s, kFrame24S_Comment16S);
}

void test_asm_echo_and_at_as_separate_notifications() {
  JkAssembler a;
  Sink s;
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, kFrame32S_Dev_B2A8S20P, 300, sinkCb, &s);
  jk_feed(a, kAckEcho, 20, sinkCb, &s);
  jk_feed(a, kAt, 4, sinkCb, &s);
  jk_feed(a, kFrame32S_Cell_B2A8S20P, 300, sinkCb, &s);
  jk_feed(a, kAt, 4, sinkCb, &s);
  jk_feed(a, kFrame32S_Settings_B2A8S20P, 300, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(3, s.count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kFrame32S_Dev_B2A8S20P, s.frames[0], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kFrame32S_Cell_B2A8S20P, s.frames[1], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kFrame32S_Settings_B2A8S20P, s.frames[2], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_UINT32(3, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_crc_bad);
  TEST_ASSERT_EQUAL_UINT32(3, a.resyncs);  // echo, AT, AT
  TEST_ASSERT_EQUAL_UINT16(0, a.len);
}

void test_asm_at_glued_to_preamble_in_one_notification() {
  static uint8_t blob[304];
  memcpy(blob, kAt, 4);
  memcpy(blob + 4, kFrame32S_Cell_B2A8S20P, 300);
  JkAssembler a;
  Sink s;
  jk_asm_reset(a);
  sinkReset(s);
  // As one notification ...
  jk_feed(a, blob, 304, sinkCb, &s);
  assertOneFrame(a, s, kFrame32S_Cell_B2A8S20P);
  TEST_ASSERT_EQUAL_UINT32(1, a.resyncs);
  // ... and as 20-byte chunks (the glued AT shifts every chunk boundary).
  jk_asm_reset(a);
  sinkReset(s);
  for (size_t off = 0; off < 304; off += 20) {
    const size_t n = (304 - off) < 20 ? (304 - off) : 20;
    jk_feed(a, blob + off, n, sinkCb, &s);
  }
  assertOneFrame(a, s, kFrame32S_Cell_B2A8S20P);
  TEST_ASSERT_EQUAL_UINT32(1, a.resyncs);
}

void test_asm_echo_interleaved_between_frame_chunks() {
  // Chunks 1..5 of frame A, the ACK echo, chunks 6..15 of frame A: A is
  // unrecoverable and must never reach the callback; frame B afterwards must.
  JkAssembler a;
  Sink s;
  jk_asm_reset(a);
  sinkReset(s);
  const uint8_t *fa = kFrame24S_Comment16S;
  for (int i = 0; i < 5; ++i) jk_feed(a, fa + 20 * i, 20, sinkCb, &s);
  jk_feed(a, kAckEcho, 20, sinkCb, &s);
  for (int i = 5; i < 15; ++i) jk_feed(a, fa + 20 * i, 20, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(0, s.count);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_ok);
  TEST_ASSERT_TRUE(a.frames_crc_bad + a.resyncs >= 1);
  TEST_ASSERT_TRUE(a.len < JK_FRAME_LEN);

  const uint8_t *fb = kFrame24S_Cell_B2A24S20P;
  for (int i = 0; i < 15; ++i) jk_feed(a, fb + 20 * i, 20, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(1, s.count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(fb, s.frames[0], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT16(0, a.len);

  // Same with a lost chunk instead of an inserted one (chunk 7 of A missing).
  jk_asm_reset(a);
  sinkReset(s);
  for (int i = 0; i < 15; ++i)
    if (i != 7) jk_feed(a, fa + 20 * i, 20, sinkCb, &s);
  for (int i = 0; i < 15; ++i) jk_feed(a, fb + 20 * i, 20, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(1, s.count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(fb, s.frames[0], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_crc_bad);  // the 280 + 20 window was CRC-checked once
  TEST_ASSERT_EQUAL_UINT16(0, a.len);
}

void test_asm_crc_corrupt_frame_then_good() {
  uint8_t bad[JK_FRAME_LEN];
  memcpy(bad, kFrame24S_Comment16S, JK_FRAME_LEN);
  bad[141] ^= 0x10;  // SoC bit flipped, CRC not updated
  JkAssembler a;
  Sink s;
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, bad, JK_FRAME_LEN, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(0, s.count);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_crc_bad);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_ok);
  TEST_ASSERT_TRUE(a.len < 4);  // the corrupt frame was discarded (at most a partial-preamble tail is kept)
  jk_feed(a, kFrame24S_Comment16S, JK_FRAME_LEN, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(1, s.count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kFrame24S_Comment16S, s.frames[0], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_crc_bad);

  // Corrupt frame in 20-byte chunks, good frame in 20-byte chunks.
  jk_asm_reset(a);
  sinkReset(s);
  for (int i = 0; i < 15; ++i) jk_feed(a, bad + 20 * i, 20, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(0, s.count);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_crc_bad);
  for (int i = 0; i < 15; ++i) jk_feed(a, kFrame24S_Comment16S + 20 * i, 20, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(1, s.count);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_ok);
}

void test_asm_garbage_before_preamble() {
  static uint8_t blob[7 + 300];
  const uint8_t garbage[7] = {0x00, 0x55, 0xAA, 0x12, 0xEB, 0x90, 0x55};  // stray preamble fragments included
  memcpy(blob, garbage, 7);
  memcpy(blob + 7, kFrame24S_Cell_B1A20S15P_13S, 300);
  JkAssembler a;
  Sink s;
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, blob, sizeof blob, sinkCb, &s);
  assertOneFrame(a, s, kFrame24S_Cell_B1A20S15P_13S);
  TEST_ASSERT_EQUAL_UINT32(1, a.resyncs);

  // Garbage as its own notification first, then the frame in two halves.
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, garbage, 7, sinkCb, &s);
  TEST_ASSERT_TRUE(a.len <= 3);  // trailing 0x55 kept as a possible preamble start
  jk_feed(a, kFrame24S_Cell_B1A20S15P_13S, 150, sinkCb, &s);
  jk_feed(a, kFrame24S_Cell_B1A20S15P_13S + 150, 150, sinkCb, &s);
  assertOneFrame(a, s, kFrame24S_Cell_B1A20S15P_13S);
}

void test_asm_two_frames_in_one_600_byte_feed() {
  static uint8_t blob[600];
  memcpy(blob, kFrame24S_Comment16S, 300);
  memcpy(blob + 300, kFrame24S_Cell_B2A24S20P, 300);
  JkAssembler a;
  Sink s;
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, blob, 600, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(2, s.count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kFrame24S_Comment16S, s.frames[0], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kFrame24S_Cell_B2A24S20P, s.frames[1], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_UINT32(2, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_crc_bad);
  TEST_ASSERT_EQUAL_UINT32(0, a.resyncs);
  TEST_ASSERT_EQUAL_UINT16(0, a.len);

  // Frame boundary in the middle of a notification: 300 bytes = second half of A + first half of B.
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, blob, 150, sinkCb, &s);
  jk_feed(a, blob + 150, 300, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(1, s.count);
  TEST_ASSERT_EQUAL_UINT16(150, a.len);
  jk_feed(a, blob + 450, 150, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(2, s.count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kFrame24S_Cell_B2A24S20P, s.frames[1], JK_FRAME_LEN);
  TEST_ASSERT_EQUAL_UINT16(0, a.len);
}

void test_asm_settings_frame_is_delivered_as_type_01() {
  JkAssembler a;
  Sink s;
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, kFrame32S_Settings_B2A8S20P, 300, sinkCb, &s);
  assertOneFrame(a, s, kFrame32S_Settings_B2A8S20P);
  TEST_ASSERT_EQUAL_HEX8(JK_FRAME_SETTINGS, s.frames[0][4]);
  // Dispatch is the caller's job: the cell-info decoder rejects it.
  JkCellInfo ci;
  TEST_ASSERT_FALSE(jk_decode_cell_info(s.frames[0], JK_PROTO_JK02_32S, ci));
  TEST_ASSERT_EQUAL_UINT32(0, ci.pack_mv);
}

void test_asm_500_bytes_garbage_never_overflows() {
  static uint8_t garbage[500];
  for (size_t i = 0; i < sizeof garbage; ++i) garbage[i] = (uint8_t)(i * 37u + 11u);
  // Make sure the pattern contains no full preamble (it would be a legitimate frame start).
  for (size_t i = 0; i + 4 <= sizeof garbage; ++i) TEST_ASSERT_FALSE(jk_is_preamble(garbage + i));
  JkAssembler a;
  Sink s;
  jk_asm_reset(a);
  sinkReset(s);
  jk_feed(a, garbage, sizeof garbage, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(0, s.count);
  TEST_ASSERT_TRUE(a.len <= JK_ASM_BUF);
  TEST_ASSERT_TRUE(a.len < 4);
  TEST_ASSERT_TRUE(a.resyncs >= 1);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_ok);
  // Byte by byte as well (len must never grow past a partial preamble).
  jk_asm_reset(a);
  for (size_t i = 0; i < sizeof garbage; ++i) {
    jk_feed(a, garbage + i, 1, sinkCb, &s);
    TEST_ASSERT_TRUE(a.len < 4);
  }
  TEST_ASSERT_EQUAL_INT(0, s.count);
  // A frame after all that still decodes.
  jk_feed(a, kFrame24S_Comment16S, 300, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(1, s.count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kFrame24S_Comment16S, s.frames[0], JK_FRAME_LEN);
}

void test_asm_partial_preamble_tail_across_notifications() {
  // "... 55 AA" ends one notification, "EB 90 ..." starts the next.
  JkAssembler a;
  Sink s;
  const uint8_t *f = kFrame24S_Comment16S;
  jk_asm_reset(a);
  sinkReset(s);
  const uint8_t junkThen55AA[6] = {0x01, 0x02, 0x03, 0x04, 0x55, 0xAA};
  jk_feed(a, junkThen55AA, 6, sinkCb, &s);
  TEST_ASSERT_EQUAL_UINT16(2, a.len);
  TEST_ASSERT_EQUAL_UINT32(1, a.resyncs);  // the 4 junk bytes
  jk_feed(a, f + 2, 298, sinkCb, &s);
  assertOneFrame(a, s, f);
  TEST_ASSERT_EQUAL_UINT32(1, a.resyncs);

  // Split after 1 and after 3 preamble bytes, frame-only (no junk).
  for (size_t split = 1; split <= 3; ++split) {
    jk_asm_reset(a);
    sinkReset(s);
    jk_feed(a, f, split, sinkCb, &s);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)split, a.len);
    TEST_ASSERT_EQUAL_UINT32(0, a.resyncs);
    jk_feed(a, f + split, JK_FRAME_LEN - split, sinkCb, &s);
    assertOneFrame(a, s, f);
    TEST_ASSERT_EQUAL_UINT32(0, a.resyncs);
  }

  // A kept tail that turns out NOT to be a preamble is discarded.
  jk_asm_reset(a);
  sinkReset(s);
  const uint8_t tail55AA[2] = {0x55, 0xAA};
  jk_feed(a, tail55AA, 2, sinkCb, &s);
  TEST_ASSERT_EQUAL_UINT16(2, a.len);
  jk_feed(a, kAckEcho, 20, sinkCb, &s);
  TEST_ASSERT_EQUAL_UINT16(0, a.len);
  TEST_ASSERT_EQUAL_UINT32(1, a.resyncs);
  jk_feed(a, f, JK_FRAME_LEN, sinkCb, &s);
  assertOneFrame(a, s, f);
}

void test_asm_reset_and_clear_semantics() {
  JkAssembler a;
  Sink s;
  memset(&a, 0xAB, sizeof a);
  jk_asm_reset(a);
  TEST_ASSERT_EQUAL_UINT16(0, a.len);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_crc_bad);
  TEST_ASSERT_EQUAL_UINT32(0, a.resyncs);
  sinkReset(s);
  jk_feed(a, kFrame24S_Comment16S, 300, sinkCb, &s);
  jk_feed(a, kFrame24S_Comment16S, 100, sinkCb, &s);  // half a frame pending
  TEST_ASSERT_EQUAL_UINT16(100, a.len);
  jk_asm_clear(a);  // disconnect
  TEST_ASSERT_EQUAL_UINT16(0, a.len);
  TEST_ASSERT_EQUAL_UINT32(1, a.frames_ok);  // counters survive
  // After the reconnect a fresh frame decodes without the stale half interfering.
  jk_feed(a, kFrame24S_Cell_B2A24S20P, 300, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(2, s.count);
  TEST_ASSERT_EQUAL_UINT32(2, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT32(0, a.frames_crc_bad);
  // Null data / zero length are no-ops.
  jk_feed(a, 0, 10, sinkCb, &s);
  jk_feed(a, kFrame24S_Comment16S, 0, sinkCb, &s);
  TEST_ASSERT_EQUAL_INT(2, s.count);
  TEST_ASSERT_EQUAL_UINT16(0, a.len);
  // Null callback: frames are counted and consumed, nobody is called.
  jk_feed(a, kFrame24S_Comment16S, 300, 0, 0);
  TEST_ASSERT_EQUAL_UINT32(3, a.frames_ok);
  TEST_ASSERT_EQUAL_UINT16(0, a.len);
}

// ---------------------------------------------------------------- decoding, JK02_24S
void test_decode_24s_comment16s() {
  JkCellInfo c;
  TEST_ASSERT_TRUE(jk_decode_cell_info(kFrame24S_Comment16S, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_HEX8(0x8C, c.counter);
  TEST_ASSERT_EQUAL_UINT8(16, c.cell_count);
  const uint16_t cells[16] = {3327, 3329, 3329, 3327, 3329, 3329, 3327, 3329,
                              3329, 3329, 3329, 3327, 3329, 3329, 3329, 3329};
  TEST_ASSERT_EQUAL_UINT16_ARRAY(cells, c.cell_mv, 16);
  for (int i = 16; i < 32; ++i) TEST_ASSERT_EQUAL_UINT16(0, c.cell_mv[i]);
  TEST_ASSERT_EQUAL_UINT16(3327, c.cell_min_mv);
  TEST_ASSERT_EQUAL_UINT16(3329, c.cell_max_mv);
  TEST_ASSERT_EQUAL_UINT8(1, c.cell_min_idx);  // first 3327
  TEST_ASSERT_EQUAL_UINT8(2, c.cell_max_idx);  // first 3329
  TEST_ASSERT_EQUAL_UINT16(2, c.cell_delta_mv);
  TEST_ASSERT_TRUE(c.cell_avg_mv >= 3327 && c.cell_avg_mv <= 3329);
  TEST_ASSERT_EQUAL_UINT16(3328, c.cell_avg_mv);  // 53256 / 16 = 3328.5 -> 3328
  TEST_ASSERT_EQUAL_UINT32(53251, c.pack_mv);
  TEST_ASSERT_EQUAL_INT32(0, c.current_ma);
  TEST_ASSERT_EQUAL_INT32(0, c.power_mw);
  TEST_ASSERT_EQUAL_INT16(190, c.t1_d);
  TEST_ASSERT_EQUAL_INT16(191, c.t2_d);
  TEST_ASSERT_EQUAL_INT16(210, c.mos_d);
  TEST_ASSERT_EQUAL_HEX32(0, c.errors);
  TEST_ASSERT_EQUAL_INT16(0, c.balance_ma);
  TEST_ASSERT_EQUAL_UINT8(0, c.balance_action);
  TEST_ASSERT_EQUAL_UINT8(84, c.soc_pct);
  TEST_ASSERT_EQUAL_UINT32(68494, c.remaining_mah);
  TEST_ASSERT_EQUAL_UINT32(81000, c.nominal_mah);
  TEST_ASSERT_EQUAL_UINT32(0, c.cycle_count);
  TEST_ASSERT_EQUAL_UINT8(100, c.soh_pct);
  TEST_ASSERT_EQUAL_UINT32(1049546, c.runtime_s);
  TEST_ASSERT_TRUE(c.chg_mos_on);
  TEST_ASSERT_TRUE(c.dis_mos_on);
}

void test_decode_24s_b2a24s20p() {
  JkCellInfo c;
  TEST_ASSERT_TRUE(jk_decode_cell_info(kFrame24S_Cell_B2A24S20P, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_HEX8(0xFC, c.counter);
  TEST_ASSERT_EQUAL_UINT8(24, c.cell_count);
  TEST_ASSERT_EQUAL_UINT16(3284, c.cell_mv[0]);
  TEST_ASSERT_EQUAL_UINT16(3281, c.cell_mv[23]);
  for (int i = 24; i < 32; ++i) TEST_ASSERT_EQUAL_UINT16(0, c.cell_mv[i]);
  TEST_ASSERT_EQUAL_UINT16(3279, c.cell_min_mv);
  TEST_ASSERT_EQUAL_UINT8(3, c.cell_min_idx);
  TEST_ASSERT_EQUAL_UINT16(3285, c.cell_max_mv);
  TEST_ASSERT_EQUAL_UINT8(23, c.cell_max_idx);
  TEST_ASSERT_EQUAL_UINT16(6, c.cell_delta_mv);
  TEST_ASSERT_EQUAL_UINT16(3280, c.cell_avg_mv);  // 78736 / 24 = 3280.67 -> 3280
  TEST_ASSERT_EQUAL_UINT32(78735, c.pack_mv);
  TEST_ASSERT_EQUAL_INT32(0, c.current_ma);
  TEST_ASSERT_EQUAL_INT16(207, c.t1_d);
  TEST_ASSERT_EQUAL_INT16(207, c.t2_d);
  TEST_ASSERT_EQUAL_INT16(234, c.mos_d);
  TEST_ASSERT_EQUAL_HEX32(0, c.errors);
  TEST_ASSERT_EQUAL_UINT8(85, c.soc_pct);
  TEST_ASSERT_EQUAL_UINT32(42804, c.remaining_mah);
  TEST_ASSERT_EQUAL_UINT32(50000, c.nominal_mah);
  TEST_ASSERT_EQUAL_UINT32(0, c.cycle_count);
  TEST_ASSERT_EQUAL_UINT8(100, c.soh_pct);
  TEST_ASSERT_EQUAL_UINT32(269340, c.runtime_s);
  TEST_ASSERT_TRUE(c.chg_mos_on);
  TEST_ASSERT_FALSE(c.dis_mos_on);
}

void test_decode_24s_b1a20s15p_13s() {
  JkCellInfo c;
  TEST_ASSERT_TRUE(jk_decode_cell_info(kFrame24S_Cell_B1A20S15P_13S, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_HEX8(0x73, c.counter);
  TEST_ASSERT_EQUAL_UINT8(13, c.cell_count);
  TEST_ASSERT_EQUAL_UINT16(3292, c.cell_mv[0]);
  TEST_ASSERT_EQUAL_UINT16(3291, c.cell_mv[12]);
  TEST_ASSERT_EQUAL_UINT16(0, c.cell_mv[13]);
  TEST_ASSERT_EQUAL_UINT16(3288, c.cell_min_mv);
  TEST_ASSERT_EQUAL_UINT8(5, c.cell_min_idx);
  TEST_ASSERT_EQUAL_UINT16(3292, c.cell_max_mv);
  TEST_ASSERT_EQUAL_UINT8(1, c.cell_max_idx);
  TEST_ASSERT_EQUAL_UINT16(4, c.cell_delta_mv);
  TEST_ASSERT_EQUAL_UINT32(42786, c.pack_mv);
  TEST_ASSERT_EQUAL_INT32(0, c.current_ma);
  TEST_ASSERT_EQUAL_INT16(-31, c.t1_d);
  TEST_ASSERT_EQUAL_INT16(-29, c.t2_d);
  TEST_ASSERT_EQUAL_INT16(5, c.mos_d);
  TEST_ASSERT_EQUAL_UINT8(47, c.soc_pct);
  TEST_ASSERT_EQUAL_UINT32(63168, c.remaining_mah);
  TEST_ASSERT_EQUAL_UINT32(132000, c.nominal_mah);
  TEST_ASSERT_EQUAL_UINT32(56, c.cycle_count);
  TEST_ASSERT_EQUAL_UINT8(100, c.soh_pct);
  TEST_ASSERT_FALSE(c.chg_mos_on);
  TEST_ASSERT_FALSE(c.dis_mos_on);
}

// ---------------------------------------------------------------- decoding, JK02_32S
void test_decode_32s_b2a8s20p() {
  JkCellInfo c;
  TEST_ASSERT_TRUE(jk_decode_cell_info(kFrame32S_Cell_B2A8S20P, JK_PROTO_JK02_32S, c));
  TEST_ASSERT_EQUAL_HEX8(0x8B, c.counter);
  TEST_ASSERT_EQUAL_UINT8(8, c.cell_count);
  const uint16_t cells[8] = {3287, 3291, 3285, 3327, 3328, 3285, 3284, 3296};
  TEST_ASSERT_EQUAL_UINT16_ARRAY(cells, c.cell_mv, 8);
  for (int i = 8; i < 32; ++i) TEST_ASSERT_EQUAL_UINT16(0, c.cell_mv[i]);
  TEST_ASSERT_EQUAL_UINT16(3284, c.cell_min_mv);
  TEST_ASSERT_EQUAL_UINT8(7, c.cell_min_idx);
  TEST_ASSERT_EQUAL_UINT16(3328, c.cell_max_mv);
  TEST_ASSERT_EQUAL_UINT8(5, c.cell_max_idx);
  TEST_ASSERT_EQUAL_UINT16(44, c.cell_delta_mv);
  TEST_ASSERT_EQUAL_UINT16(3297, c.cell_avg_mv);  // 26383 / 8 = 3297.875 -> 3297
  TEST_ASSERT_EQUAL_UINT32(26381, c.pack_mv);
  TEST_ASSERT_EQUAL_INT32(0, c.current_ma);
  TEST_ASSERT_EQUAL_INT32(0, c.power_mw);
  TEST_ASSERT_EQUAL_INT16(297, c.t1_d);
  TEST_ASSERT_EQUAL_INT16(298, c.t2_d);
  TEST_ASSERT_EQUAL_INT16(361, c.mos_d);
  TEST_ASSERT_EQUAL_HEX32(0, c.errors);
  TEST_ASSERT_EQUAL_INT16(-2019, c.balance_ma);
  TEST_ASSERT_EQUAL_UINT8(2, c.balance_action);
  TEST_ASSERT_EQUAL_UINT8(69, c.soc_pct);
  TEST_ASSERT_EQUAL_UINT32(69554, c.remaining_mah);
  TEST_ASSERT_EQUAL_UINT32(100000, c.nominal_mah);
  TEST_ASSERT_EQUAL_UINT32(0, c.cycle_count);
  TEST_ASSERT_EQUAL_UINT8(100, c.soh_pct);
  TEST_ASSERT_EQUAL_UINT32(10989, c.runtime_s);
  TEST_ASSERT_TRUE(c.chg_mos_on);
  TEST_ASSERT_TRUE(c.dis_mos_on);
}

// ---------------------------------------------------------------- layout detection
void test_layout_plausible_and_detect() {
  // Right layouts pass.
  TEST_ASSERT_TRUE(jk_layout_plausible(kFrame24S_Comment16S, JK_PROTO_JK02_24S));
  TEST_ASSERT_TRUE(jk_layout_plausible(kFrame24S_Cell_B2A24S20P, JK_PROTO_JK02_24S));
  TEST_ASSERT_TRUE(jk_layout_plausible(kFrame24S_Cell_B1A20S15P_13S, JK_PROTO_JK02_24S));
  TEST_ASSERT_TRUE(jk_layout_plausible(kFrame32S_Cell_B2A8S20P, JK_PROTO_JK02_32S));
  // Wrong layouts fail.
  TEST_ASSERT_FALSE(jk_layout_plausible(kFrame24S_Comment16S, JK_PROTO_JK02_32S));
  TEST_ASSERT_FALSE(jk_layout_plausible(kFrame24S_Cell_B2A24S20P, JK_PROTO_JK02_32S));
  TEST_ASSERT_FALSE(jk_layout_plausible(kFrame24S_Cell_B1A20S15P_13S, JK_PROTO_JK02_32S));
  TEST_ASSERT_FALSE(jk_layout_plausible(kFrame32S_Cell_B2A8S20P, JK_PROTO_JK02_24S));
  // UNKNOWN and non-cell-info frames are never plausible.
  TEST_ASSERT_FALSE(jk_layout_plausible(kFrame24S_Comment16S, JK_PROTO_UNKNOWN));
  TEST_ASSERT_FALSE(jk_layout_plausible(kFrame24S_Dev_B2A24S20P, JK_PROTO_JK02_24S));
  TEST_ASSERT_FALSE(jk_layout_plausible(kFrame32S_Settings_B2A8S20P, JK_PROTO_JK02_32S));

  // Detector: right guess kept, wrong guess corrected, UNKNOWN guess resolved.
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_24S, jk_detect_proto(kFrame24S_Comment16S, JK_PROTO_JK02_24S));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_24S, jk_detect_proto(kFrame24S_Comment16S, JK_PROTO_JK02_32S));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_24S, jk_detect_proto(kFrame24S_Comment16S, JK_PROTO_UNKNOWN));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_24S, jk_detect_proto(kFrame24S_Cell_B1A20S15P_13S, JK_PROTO_JK02_32S));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_32S, jk_detect_proto(kFrame32S_Cell_B2A8S20P, JK_PROTO_JK02_32S));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_32S, jk_detect_proto(kFrame32S_Cell_B2A8S20P, JK_PROTO_JK02_24S));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_32S, jk_detect_proto(kFrame32S_Cell_B2A8S20P, JK_PROTO_UNKNOWN));

  // Wrong-layout decode yields nonsense (which is why the detector exists).
  JkCellInfo c;
  TEST_ASSERT_TRUE(jk_decode_cell_info(kFrame32S_Cell_B2A8S20P, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_UINT32(0, c.pack_mv);
  TEST_ASSERT_TRUE(jk_decode_cell_info(kFrame24S_Comment16S, JK_PROTO_JK02_32S, c));
  TEST_ASSERT_EQUAL_UINT16(0xFFFF, c.cell_mv[24]);  // enabled-cells bitmask read as a cell
}

void test_detect_unknown_when_both_layouts_fail() {
  // Device info frame: not cell info at all.
  TEST_ASSERT_EQUAL(JK_PROTO_UNKNOWN, jk_detect_proto(kFrame24S_Dev_B2A24S20P, JK_PROTO_JK02_24S));
  TEST_ASSERT_EQUAL(JK_PROTO_UNKNOWN, jk_detect_proto(kFrameDev_JK04_B2A16S_330, JK_PROTO_UNKNOWN));
  // Settings frame.
  TEST_ASSERT_EQUAL(JK_PROTO_UNKNOWN, jk_detect_proto(kFrame32S_Settings_B2A8S20P, JK_PROTO_JK02_32S));
  // Cell info with the pack voltage zeroed (CRC recomputed).
  uint8_t f[JK_FRAME_LEN];
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  memset(f + 118, 0, 4);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_frame_crc_ok(f));
  TEST_ASSERT_FALSE(jk_layout_plausible(f, JK_PROTO_JK02_24S));
  TEST_ASSERT_FALSE(jk_layout_plausible(f, JK_PROTO_JK02_32S));
  TEST_ASSERT_EQUAL(JK_PROTO_UNKNOWN, jk_detect_proto(f, JK_PROTO_JK02_24S));
  TEST_ASSERT_EQUAL(JK_PROTO_UNKNOWN, jk_detect_proto(f, JK_PROTO_UNKNOWN));
  // Pack off by more than 2 % (a wrong cell count would do this).
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  jk_wr_u32(f + 118, 53251 + 1100);
  fixCrc(f);
  TEST_ASSERT_FALSE(jk_layout_plausible(f, JK_PROTO_JK02_24S));
  jk_wr_u32(f + 118, 53251 + 1000);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_layout_plausible(f, JK_PROTO_JK02_24S));
  // A cell above 5000 mV.
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  wr16(f + 6, 5001);
  fixCrc(f);
  TEST_ASSERT_FALSE(jk_layout_plausible(f, JK_PROTO_JK02_24S));
  // All cells zero.
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  memset(f + 6, 0, 48);
  fixCrc(f);
  TEST_ASSERT_FALSE(jk_layout_plausible(f, JK_PROTO_JK02_24S));
}

void test_decode_cell_info_rejects_unknown_and_wrong_type() {
  JkCellInfo c;
  memset(&c, 0xAB, sizeof c);
  TEST_ASSERT_FALSE(jk_decode_cell_info(kFrame24S_Comment16S, JK_PROTO_UNKNOWN, c));
  TEST_ASSERT_EQUAL_UINT8(0, c.cell_count);  // zero-filled even on failure
  TEST_ASSERT_EQUAL_UINT32(0, c.pack_mv);
  TEST_ASSERT_EQUAL_UINT8(0, c.soc_pct);
  memset(&c, 0xAB, sizeof c);
  TEST_ASSERT_FALSE(jk_decode_cell_info(kFrame24S_Dev_B2A24S20P, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_UINT32(0, c.pack_mv);
  TEST_ASSERT_FALSE(jk_decode_cell_info(kFrame32S_Dev_B2A8S20P, JK_PROTO_JK02_32S, c));
  TEST_ASSERT_FALSE(jk_decode_cell_info(kFrame32S_Settings_B2A8S20P, JK_PROTO_JK02_32S, c));
  TEST_ASSERT_FALSE(jk_decode_cell_info(0, JK_PROTO_JK02_24S, c));
}

// ---------------------------------------------------------------- device info
void test_device_info_recorded_frames() {
  JkDevInfo d;
  TEST_ASSERT_TRUE(jk_decode_device_info(kFrame24S_Dev_B2A24S20P, d));
  TEST_ASSERT_EQUAL_HEX8(0xFC, d.counter);
  TEST_ASSERT_EQUAL_STRING("JK-B2A24S20P", d.model);
  TEST_ASSERT_EQUAL_STRING("10.XG", d.hw);
  TEST_ASSERT_EQUAL_STRING("10.07", d.sw);
  TEST_ASSERT_EQUAL_UINT32(269100, d.uptime_s);
  TEST_ASSERT_EQUAL_UINT32(1, d.power_on_count);
  TEST_ASSERT_EQUAL_STRING("JK-B2A24S20P", d.name);

  TEST_ASSERT_TRUE(jk_decode_device_info(kFrame32S_Dev_B2A8S20P, d));
  TEST_ASSERT_EQUAL_HEX8(0x8B, d.counter);
  TEST_ASSERT_EQUAL_STRING("JK_B2A8S20P", d.model);
  TEST_ASSERT_EQUAL_STRING("11.XW", d.hw);
  TEST_ASSERT_EQUAL_STRING("11.17", d.sw);
  TEST_ASSERT_EQUAL_UINT32(10800, d.uptime_s);
  TEST_ASSERT_EQUAL_UINT32(1, d.power_on_count);
  TEST_ASSERT_EQUAL_STRING("JK_B2A8S20P", d.name);

  TEST_ASSERT_TRUE(jk_decode_device_info(kFrameDev_JK04_B2A16S_330, d));
  TEST_ASSERT_EQUAL_STRING("JK-B2A16S", d.model);
  TEST_ASSERT_EQUAL_STRING("3.0", d.hw);
  TEST_ASSERT_EQUAL_STRING("3.3.0", d.sw);
  TEST_ASSERT_EQUAL_UINT32(36867600, d.uptime_s);
  TEST_ASSERT_EQUAL_UINT32(19, d.power_on_count);
  TEST_ASSERT_EQUAL_STRING("BMS", d.name);

  TEST_ASSERT_TRUE(jk_decode_device_info(kFrameDev_24S_B2A24S15P_1007, d));
  TEST_ASSERT_EQUAL_STRING("JK-B2A24S15P", d.model);
  TEST_ASSERT_EQUAL_STRING("10.XW", d.hw);
  TEST_ASSERT_EQUAL_STRING("10.07", d.sw);
  TEST_ASSERT_EQUAL_UINT32(110400, d.uptime_s);
  TEST_ASSERT_EQUAL_UINT32(6, d.power_on_count);

  TEST_ASSERT_TRUE(jk_decode_device_info(kFrameDev_32S_PB2A16S15P_1420, d));
  TEST_ASSERT_EQUAL_STRING("JK_PB2A16S15P", d.model);
  TEST_ASSERT_EQUAL_STRING("14.XA", d.hw);
  TEST_ASSERT_EQUAL_STRING("14.20", d.sw);
  TEST_ASSERT_EQUAL_UINT32(124500, d.uptime_s);
  TEST_ASSERT_EQUAL_UINT32(156, d.power_on_count);
  TEST_ASSERT_EQUAL_STRING("JK_PB2A16S15P", d.name);

  // Not a device info frame.
  memset(&d, 0xAB, sizeof d);
  TEST_ASSERT_FALSE(jk_decode_device_info(kFrame24S_Comment16S, d));
  TEST_ASSERT_EQUAL_STRING("", d.model);  // zero-filled on failure
  TEST_ASSERT_FALSE(jk_decode_device_info(kFrame32S_Settings_B2A8S20P, d));
  TEST_ASSERT_FALSE(jk_decode_device_info(0, d));
}

void test_device_info_16_char_fields_are_terminated() {
  uint8_t f[JK_FRAME_LEN];
  memcpy(f, kFrame24S_Dev_B2A24S20P, JK_FRAME_LEN);
  memcpy(f + 6, "JK_PB2A16S20P-XY", 16);   // model fills all 16 bytes, no NUL
  memcpy(f + 22, "14.XA123", 8);           // hw fills all 8
  memcpy(f + 30, "14.20.99", 8);           // sw fills all 8
  memcpy(f + 46, "My Boat Battery!", 16);  // name fills all 16
  f[62] = 0x7F;                            // byte after the name must not leak in
  fixCrc(f);
  JkDevInfo d;
  memset(&d, 0xAB, sizeof d);
  TEST_ASSERT_TRUE(jk_decode_device_info(f, d));
  TEST_ASSERT_EQUAL_UINT32(16, (uint32_t)strlen(d.model));
  TEST_ASSERT_EQUAL_STRING("JK_PB2A16S20P-XY", d.model);
  TEST_ASSERT_EQUAL_CHAR(0, d.model[16]);
  TEST_ASSERT_EQUAL_STRING("14.XA123", d.hw);
  TEST_ASSERT_EQUAL_CHAR(0, d.hw[8]);
  TEST_ASSERT_EQUAL_STRING("14.20.99", d.sw);
  TEST_ASSERT_EQUAL_CHAR(0, d.sw[8]);
  TEST_ASSERT_EQUAL_STRING("My Boat Battery!", d.name);
  TEST_ASSERT_EQUAL_CHAR(0, d.name[16]);
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_32S, jk_guess_proto(d));
  // Control characters are neutralised.
  f[6] = 0x01;
  f[7] = 0xC3;
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_device_info(f, d));
  TEST_ASSERT_EQUAL_STRING(".._PB2A16S20P-XY", d.model);
}

void test_sw_major() {
  TEST_ASSERT_EQUAL_INT(10, jk_sw_major("10.07"));
  TEST_ASSERT_EQUAL_INT(11, jk_sw_major("11.17"));
  TEST_ASSERT_EQUAL_INT(3, jk_sw_major("3.3.0"));
  TEST_ASSERT_EQUAL_INT(14, jk_sw_major("14.20"));
  TEST_ASSERT_EQUAL_INT(15, jk_sw_major("15"));
  TEST_ASSERT_EQUAL_INT(-1, jk_sw_major(""));
  TEST_ASSERT_EQUAL_INT(-1, jk_sw_major("V7"));
  TEST_ASSERT_EQUAL_INT(-1, jk_sw_major(".5"));
  TEST_ASSERT_EQUAL_INT(-1, jk_sw_major(0));
  TEST_ASSERT_EQUAL_INT(99999, jk_sw_major("99999999999999999999"));  // capped, no UB
}

void test_guess_proto() {
  JkDevInfo d;
  TEST_ASSERT_TRUE(jk_decode_device_info(kFrame24S_Dev_B2A24S20P, d));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_24S, jk_guess_proto(d));  // JK-B2A24S20P 10.07
  TEST_ASSERT_TRUE(jk_decode_device_info(kFrame32S_Dev_B2A8S20P, d));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_32S, jk_guess_proto(d));  // JK_B2A8S20P 11.17
  TEST_ASSERT_TRUE(jk_decode_device_info(kFrameDev_32S_PB2A16S15P_1420, d));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_32S, jk_guess_proto(d));  // JK_PB2A16S15P 14.20
  TEST_ASSERT_TRUE(jk_decode_device_info(kFrameDev_24S_B2A24S15P_1007, d));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_24S, jk_guess_proto(d));  // JK-B2A24S15P 10.07
  TEST_ASSERT_TRUE(jk_decode_device_info(kFrameDev_JK04_B2A16S_330, d));
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_24S, jk_guess_proto(d));  // JK04 3.3.0: guessed 24S, detector says UNKNOWN

  memset(&d, 0, sizeof d);
  strcpy(d.model, "JK_PB2A16S20P");  // model alone decides when sw is empty
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_32S, jk_guess_proto(d));
  strcpy(d.model, "JK-PB2A16S20P");
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_32S, jk_guess_proto(d));
  strcpy(d.model, "JK-B2A24S15P");
  strcpy(d.sw, "10.07");
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_24S, jk_guess_proto(d));
  strcpy(d.sw, "11.00");
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_32S, jk_guess_proto(d));
  memset(&d, 0, sizeof d);  // nothing known at all -> 24S (the detector corrects it)
  TEST_ASSERT_EQUAL(JK_PROTO_JK02_24S, jk_guess_proto(d));
}

// ---------------------------------------------------------------- synthetic frames
void test_synthetic_negative_temperature_and_current() {
  // 24S layout.
  uint8_t f[JK_FRAME_LEN];
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  wr16(f + 130, 0xFF9C);                 // t1 = -100 = -10.0 C
  wr16(f + 132, 0xFFFF);                 // t2 = -0.1 C
  wr16(f + 134, 0xFE0C);                 // mos = -500 = -50.0 C
  jk_wr_u32(f + 126, (uint32_t)-15250);  // discharging 15.25 A
  wr16(f + 138, (uint16_t)-1200);        // balance -1.2 A
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_frame_crc_ok(f));
  TEST_ASSERT_TRUE(jk_layout_plausible(f, JK_PROTO_JK02_24S));
  JkCellInfo c;
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_INT16(-100, c.t1_d);
  TEST_ASSERT_EQUAL_INT16(-1, c.t2_d);
  TEST_ASSERT_EQUAL_INT16(-500, c.mos_d);
  TEST_ASSERT_EQUAL_INT32(-15250, c.current_ma);
  TEST_ASSERT_EQUAL_INT32(-812077, c.power_mw);  // 53251 * -15250 / 1000 = -812077.75 -> -812077 (truncation toward 0)
  TEST_ASSERT_EQUAL_INT16(-1200, c.balance_ma);

  // 32S layout: the same fields at +32.
  memcpy(f, kFrame32S_Cell_B2A8S20P, JK_FRAME_LEN);
  wr16(f + 162, 0xFF9C);
  wr16(f + 144, (uint16_t)-25);
  jk_wr_u32(f + 158, (uint32_t)-120000);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_layout_plausible(f, JK_PROTO_JK02_32S));
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_32S, c));
  TEST_ASSERT_EQUAL_INT16(-100, c.t1_d);
  TEST_ASSERT_EQUAL_INT16(298, c.t2_d);
  TEST_ASSERT_EQUAL_INT16(-25, c.mos_d);
  TEST_ASSERT_EQUAL_INT32(-120000, c.current_ma);
  TEST_ASSERT_EQUAL_INT32(-3165720, c.power_mw);  // 26381 * -120000 / 1000

  // Positive (charging) current keeps its sign.
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  jk_wr_u32(f + 126, 20000);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_INT32(20000, c.current_ma);
  TEST_ASSERT_EQUAL_INT32(1065020, c.power_mw);
}

void test_synthetic_zero_mid_pack_cell() {
  uint8_t f[JK_FRAME_LEN];
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  wr16(f + 6 + 2 * 2, 0);  // cell 3 of 16 reads 0
  fixCrc(f);
  JkCellInfo c;
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_UINT8(15, c.cell_count);
  TEST_ASSERT_EQUAL_UINT16(0, c.cell_mv[2]);
  TEST_ASSERT_EQUAL_UINT16(3327, c.cell_mv[3]);  // slot 4 keeps its place
  TEST_ASSERT_EQUAL_UINT16(3329, c.cell_mv[15]);
  TEST_ASSERT_EQUAL_UINT16(3327, c.cell_min_mv);
  TEST_ASSERT_EQUAL_UINT8(1, c.cell_min_idx);
  TEST_ASSERT_EQUAL_UINT16(3329, c.cell_max_mv);
  TEST_ASSERT_EQUAL_UINT8(2, c.cell_max_idx);
  TEST_ASSERT_EQUAL_UINT16(2, c.cell_delta_mv);
  TEST_ASSERT_EQUAL_UINT16(3328, c.cell_avg_mv);  // (53256 - 3329) / 15 = 3328.47 -> 3328
  // The pack no longer matches the cell sum: implausible, as it should be.
  TEST_ASSERT_FALSE(jk_layout_plausible(f, JK_PROTO_JK02_24S));

  // Zero the minimum-holding cell 1 instead: indices move to the next occurrences.
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  wr16(f + 6, 0);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_UINT8(15, c.cell_count);
  TEST_ASSERT_EQUAL_UINT8(4, c.cell_min_idx);  // next 3327 is cell 4
  TEST_ASSERT_EQUAL_UINT8(2, c.cell_max_idx);

  // All cells zero: count 0, no indices, no division by zero.
  memset(f + 6, 0, 48);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_UINT8(0, c.cell_count);
  TEST_ASSERT_EQUAL_UINT8(0, c.cell_min_idx);
  TEST_ASSERT_EQUAL_UINT8(0, c.cell_max_idx);
  TEST_ASSERT_EQUAL_UINT16(0, c.cell_min_mv);
  TEST_ASSERT_EQUAL_UINT16(0, c.cell_max_mv);
  TEST_ASSERT_EQUAL_UINT16(0, c.cell_avg_mv);
  TEST_ASSERT_EQUAL_UINT16(0, c.cell_delta_mv);
  TEST_ASSERT_EQUAL_UINT32(53251, c.pack_mv);  // the rest still decodes
}

void test_synthetic_power_large_negative_no_overflow() {
  uint8_t f[JK_FRAME_LEN];
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  jk_wr_u32(f + 118, 53000);
  jk_wr_u32(f + 126, (uint32_t)-120000);  // 53000 mV * -120000 mA = -6.36e9 (overflows int32 before / 1000)
  fixCrc(f);
  JkCellInfo c;
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_UINT32(53000, c.pack_mv);
  TEST_ASSERT_EQUAL_INT32(-120000, c.current_ma);
  TEST_ASSERT_EQUAL_INT32(-6360000, c.power_mw);
  // Charging at the same magnitude.
  jk_wr_u32(f + 126, 120000);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_INT32(6360000, c.power_mw);
  // 32S: 100 V * -500 A = -50 kW.
  memcpy(f, kFrame32S_Cell_B2A8S20P, JK_FRAME_LEN);
  jk_wr_u32(f + 150, 100000);
  jk_wr_u32(f + 158, (uint32_t)-500000);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_32S, c));
  TEST_ASSERT_EQUAL_INT32(-50000000, c.power_mw);
  // Absurd values saturate instead of wrapping.
  jk_wr_u32(f + 150, 4000000000u);
  jk_wr_u32(f + 158, (uint32_t)-2000000000);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_32S, c));
  TEST_ASSERT_EQUAL_INT32(INT32_MIN, c.power_mw);
  jk_wr_u32(f + 158, 2000000000u);
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_32S, c));
  TEST_ASSERT_EQUAL_INT32(INT32_MAX, c.power_mw);
}

void test_synthetic_errors_field_per_layout() {
  uint8_t f[JK_FRAME_LEN];
  JkCellInfo c;
  // 24S: u16 @136.
  memcpy(f, kFrame24S_Comment16S, JK_FRAME_LEN);
  wr16(f + 136, 0x0801);  // WIRE_R + CELLUV
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_24S, c));
  TEST_ASSERT_EQUAL_HEX32(0x00000801, c.errors);
  TEST_ASSERT_EQUAL_INT16(210, c.mos_d);  // MOS temperature @134 untouched
  // 32S: u32 @166.
  memcpy(f, kFrame32S_Cell_B2A8S20P, JK_FRAME_LEN);
  jk_wr_u32(f + 166, 0x80000800u);  // CELLUV + BIT31
  fixCrc(f);
  TEST_ASSERT_TRUE(jk_decode_cell_info(f, JK_PROTO_JK02_32S, c));
  TEST_ASSERT_EQUAL_HEX32(0x80000800u, c.errors);
  TEST_ASSERT_EQUAL_INT16(361, c.mos_d);  // MOS temperature @144 untouched
  TEST_ASSERT_EQUAL_INT(11, jk_first_error(c.errors));
  TEST_ASSERT_EQUAL_UINT8(2, jk_error_count(c.errors));
  TEST_ASSERT_EQUAL_STRING("CELLUV", jk_error_name((uint8_t)jk_first_error(c.errors)));
}

// ---------------------------------------------------------------- error names
void test_error_names() {
  for (int b = 0; b < 32; ++b) {
    const char *s = jk_error_name((uint8_t)b);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_TRUE_MESSAGE(strcmp(s, "?") != 0, "every bit 0..31 has a name");
    const size_t n = strlen(s);
    TEST_ASSERT_TRUE(n >= 1 && n <= 6);
    for (int b2 = 0; b2 < b; ++b2) TEST_ASSERT_TRUE_MESSAGE(strcmp(s, jk_error_name((uint8_t)b2)) != 0, "names unique");
  }
  TEST_ASSERT_EQUAL_STRING("WIRE_R", jk_error_name(0));
  TEST_ASSERT_EQUAL_STRING("MOS_OT", jk_error_name(1));
  TEST_ASSERT_EQUAL_STRING("CELLNO", jk_error_name(2));
  TEST_ASSERT_EQUAL_STRING("BIT3", jk_error_name(3));
  TEST_ASSERT_EQUAL_STRING("FULL", jk_error_name(4));
  TEST_ASSERT_EQUAL_STRING("PACKOV", jk_error_name(5));
  TEST_ASSERT_EQUAL_STRING("CHG_OC", jk_error_name(6));
  TEST_ASSERT_EQUAL_STRING("CHG_SC", jk_error_name(7));
  TEST_ASSERT_EQUAL_STRING("CHG_OT", jk_error_name(8));
  TEST_ASSERT_EQUAL_STRING("CHG_UT", jk_error_name(9));
  TEST_ASSERT_EQUAL_STRING("COPROC", jk_error_name(10));
  TEST_ASSERT_EQUAL_STRING("CELLUV", jk_error_name(11));
  TEST_ASSERT_EQUAL_STRING("PACKUV", jk_error_name(12));
  TEST_ASSERT_EQUAL_STRING("DIS_OC", jk_error_name(13));
  TEST_ASSERT_EQUAL_STRING("DIS_SC", jk_error_name(14));
  TEST_ASSERT_EQUAL_STRING("DIS_OT", jk_error_name(15));
  TEST_ASSERT_EQUAL_STRING("CMOSAB", jk_error_name(16));
  TEST_ASSERT_EQUAL_STRING("DMOSAB", jk_error_name(17));
  TEST_ASSERT_EQUAL_STRING("GPS", jk_error_name(18));
  TEST_ASSERT_EQUAL_STRING("PASSWD", jk_error_name(19));
  TEST_ASSERT_EQUAL_STRING("DISON", jk_error_name(20));
  TEST_ASSERT_EQUAL_STRING("BAT_OT", jk_error_name(21));
  TEST_ASSERT_EQUAL_STRING("TSENS", jk_error_name(22));
  TEST_ASSERT_EQUAL_STRING("PLMOD", jk_error_name(23));
  TEST_ASSERT_EQUAL_STRING("SCPREL", jk_error_name(24));
  TEST_ASSERT_EQUAL_STRING("DOCP2", jk_error_name(25));
  TEST_ASSERT_EQUAL_STRING("DOCP3", jk_error_name(26));
  TEST_ASSERT_EQUAL_STRING("DIS_UT", jk_error_name(27));
  TEST_ASSERT_EQUAL_STRING("GPSLCK", jk_error_name(28));
  TEST_ASSERT_EQUAL_STRING("BIT29", jk_error_name(29));
  TEST_ASSERT_EQUAL_STRING("BIT30", jk_error_name(30));
  TEST_ASSERT_EQUAL_STRING("BIT31", jk_error_name(31));
  TEST_ASSERT_EQUAL_STRING("?", jk_error_name(32));
  TEST_ASSERT_EQUAL_STRING("?", jk_error_name(255));
}

void test_first_error_and_count() {
  TEST_ASSERT_EQUAL_INT(-1, jk_first_error(0));
  TEST_ASSERT_EQUAL_INT(11, jk_first_error(0x800));
  TEST_ASSERT_EQUAL_INT(0, jk_first_error(0x801));
  TEST_ASSERT_EQUAL_INT(31, jk_first_error(0x80000000u));
  TEST_ASSERT_EQUAL_INT(0, jk_first_error(0xFFFFFFFFu));
  TEST_ASSERT_EQUAL_UINT8(0, jk_error_count(0));
  TEST_ASSERT_EQUAL_UINT8(1, jk_error_count(0x800));
  TEST_ASSERT_EQUAL_UINT8(2, jk_error_count(0x00000801u));
  TEST_ASSERT_EQUAL_UINT8(32, jk_error_count(0xFFFFFFFFu));
  TEST_ASSERT_EQUAL_UINT8(16, jk_error_count(0xAAAAAAAAu));
}

// ---------------------------------------------------------------- byte access
void test_little_endian_readers() {
  const uint8_t b[] = {0x78, 0x56, 0x34, 0x12, 0xFF, 0xFF, 0xFF, 0xFF, 0x9C, 0xFF};
  TEST_ASSERT_EQUAL_HEX16(0x5678, jk_u16(b));
  TEST_ASSERT_EQUAL_HEX32(0x12345678u, jk_u32(b));
  TEST_ASSERT_EQUAL_INT32(-1, jk_i32(b + 4));
  TEST_ASSERT_EQUAL_INT16(-1, jk_i16(b + 4));
  TEST_ASSERT_EQUAL_INT16(-100, jk_i16(b + 8));
  uint8_t w[4];
  jk_wr_u32(w, 0xDEADC0DEu);
  const uint8_t expected[] = {0xDE, 0xC0, 0xAD, 0xDE};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, w, 4);
  TEST_ASSERT_EQUAL_UINT8(24, jk_cell_slots(JK_PROTO_JK02_24S));
  TEST_ASSERT_EQUAL_UINT8(32, jk_cell_slots(JK_PROTO_JK02_32S));
  TEST_ASSERT_EQUAL_UINT8(0, jk_off32(JK_PROTO_JK02_24S));
  TEST_ASSERT_EQUAL_UINT8(32, jk_off32(JK_PROTO_JK02_32S));
  TEST_ASSERT_EQUAL_UINT8(0, (uint8_t)JK_PROTO_UNKNOWN);  // same numbering as BmsProto
  TEST_ASSERT_EQUAL_UINT8(1, (uint8_t)JK_PROTO_JK02_24S);
  TEST_ASSERT_EQUAL_UINT8(2, (uint8_t)JK_PROTO_JK02_32S);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_sum8_and_build_cmd_reads);
  RUN_TEST(test_build_cmd_value_and_app_padding_variant);
  RUN_TEST(test_crc_all_recorded_frames_pass);
  RUN_TEST(test_crc_flipped_byte_fails);
  RUN_TEST(test_asm_chunkings_seen_in_the_field);
  RUN_TEST(test_asm_frame_plus_ack_echo_in_one_blob);
  RUN_TEST(test_asm_echo_and_at_as_separate_notifications);
  RUN_TEST(test_asm_at_glued_to_preamble_in_one_notification);
  RUN_TEST(test_asm_echo_interleaved_between_frame_chunks);
  RUN_TEST(test_asm_crc_corrupt_frame_then_good);
  RUN_TEST(test_asm_garbage_before_preamble);
  RUN_TEST(test_asm_two_frames_in_one_600_byte_feed);
  RUN_TEST(test_asm_settings_frame_is_delivered_as_type_01);
  RUN_TEST(test_asm_500_bytes_garbage_never_overflows);
  RUN_TEST(test_asm_partial_preamble_tail_across_notifications);
  RUN_TEST(test_asm_reset_and_clear_semantics);
  RUN_TEST(test_decode_24s_comment16s);
  RUN_TEST(test_decode_24s_b2a24s20p);
  RUN_TEST(test_decode_24s_b1a20s15p_13s);
  RUN_TEST(test_decode_32s_b2a8s20p);
  RUN_TEST(test_layout_plausible_and_detect);
  RUN_TEST(test_detect_unknown_when_both_layouts_fail);
  RUN_TEST(test_decode_cell_info_rejects_unknown_and_wrong_type);
  RUN_TEST(test_device_info_recorded_frames);
  RUN_TEST(test_device_info_16_char_fields_are_terminated);
  RUN_TEST(test_sw_major);
  RUN_TEST(test_guess_proto);
  RUN_TEST(test_synthetic_negative_temperature_and_current);
  RUN_TEST(test_synthetic_zero_mid_pack_cell);
  RUN_TEST(test_synthetic_power_large_negative_no_overflow);
  RUN_TEST(test_synthetic_errors_field_per_layout);
  RUN_TEST(test_error_names);
  RUN_TEST(test_first_error_and_count);
  RUN_TEST(test_little_endian_readers);
  return UNITY_END();
}
