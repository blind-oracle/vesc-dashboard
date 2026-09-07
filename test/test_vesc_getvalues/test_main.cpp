// Native Unity tests for include/vesc_getvalues.h: CRC-16/XMODEM, the
// COMM_GET_VALUES(_SELECTIVE) request builder, reassembly of the fragmented
// CAN reply (FILL_RX_BUFFER / FILL_RX_BUFFER_LONG / PROCESS_RX_BUFFER) and the
// mask-ordered payload decoder, plus the fault-code string tables.
//   pio test -e native
// Arduino-free: only vesc_getvalues.h (-> shared_state.h, vesc_status.h) and <unity.h>.
//
// The reference frames (request for target 74 from own id 120, and the 7-frame
// reply for mask 0x003EC03C with CRC 0x9113) are byte images produced by a host
// emulation of vedderb/bldc comm_can_send_buffer()/crc16(), so a regression in
// offsets, endianness or the CRC shows up as a byte mismatch, not as a test that
// passes because both sides share the bug.
#include <unity.h>

#include <stdint.h>
#include <string.h>

#include "vesc_getvalues.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- helpers
static const uint8_t kOwnId = 120;
static const uint8_t kVescId = 74;

struct Frame {
  uint32_t eid;
  uint8_t dlc;
  uint8_t d[8];
};

static void put_i16(uint8_t *&p, int32_t v) {
  *p++ = (uint8_t)((uint32_t)v >> 8);
  *p++ = (uint8_t)v;
}
static void put_i32(uint8_t *&p, int32_t v) {
  *p++ = (uint8_t)((uint32_t)v >> 24);
  *p++ = (uint8_t)((uint32_t)v >> 16);
  *p++ = (uint8_t)((uint32_t)v >> 8);
  *p++ = (uint8_t)v;
}

// Mirror of comm_can_send_buffer(controller_id = own, data, len, send = 1) as run by a
// VESC with id vesc_id: len <= 6 -> one PROCESS_SHORT_BUFFER; otherwise FILL_RX_BUFFER
// (7-byte chunks while the offset is <= 255), FILL_RX_BUFFER_LONG (6-byte chunks
// above) and a closing PROCESS_RX_BUFFER with length + CRC. Returns the frame count.
static int emit_reply(Frame *out, int cap, uint8_t vesc_id, uint8_t own, const uint8_t *data, unsigned len) {
  // frames needed: 1 (short) or ceil(min(len,259)/7) short + ceil(max(len-259,0)/6) long + 1; the
  // short loop stops at offset 252 (i <= 255), i.e. after 259 bytes. Checked BEFORE writing.
  const unsigned short_bytes = len < 259u ? len : 259u;
  const unsigned need = len <= 6 ? 1u : (short_bytes + 6u) / 7u + (len > 259u ? (len - 259u + 5u) / 6u : 0u) + 1u;
  TEST_ASSERT_TRUE_MESSAGE(need <= (unsigned)cap, "emit_reply: frame array too small");
  int n = 0;
  if (len <= 6) {
    Frame &f = out[n++];
    f.eid = VESC_EID(VESC_CAN_PACKET_PROCESS_SHORT_BUFFER, own);
    f.d[0] = vesc_id;
    f.d[1] = 1;
    memcpy(f.d + 2, data, len);
    f.dlc = (uint8_t)(len + 2);
    return n;
  }
  unsigned end_a = 0;
  for (unsigned i = 0; i < len; i += 7) {
    if (i > 255) break;
    end_a = i + 7;
    const unsigned sl = (i + 7 <= len) ? 7 : len - i;
    Frame &f = out[n++];
    f.eid = VESC_EID(VESC_CAN_PACKET_FILL_RX_BUFFER, own);
    f.d[0] = (uint8_t)i;
    memcpy(f.d + 1, data + i, sl);
    f.dlc = (uint8_t)(sl + 1);
  }
  for (unsigned i = end_a; i < len; i += 6) {
    const unsigned sl = (i + 6 <= len) ? 6 : len - i;
    Frame &f = out[n++];
    f.eid = VESC_EID(VESC_CAN_PACKET_FILL_RX_BUFFER_LONG, own);
    f.d[0] = (uint8_t)(i >> 8);
    f.d[1] = (uint8_t)i;
    memcpy(f.d + 2, data + i, sl);
    f.dlc = (uint8_t)(sl + 2);
  }
  const uint16_t crc = vesc_crc16(data, len);
  Frame &f = out[n++];
  f.eid = VESC_EID(VESC_CAN_PACKET_PROCESS_RX_BUFFER, own);
  f.d[0] = vesc_id;
  f.d[1] = 1;
  f.d[2] = (uint8_t)(len >> 8);
  f.d[3] = (uint8_t)len;
  f.d[4] = (uint8_t)(crc >> 8);
  f.d[5] = (uint8_t)crc;
  f.dlc = 6;
  TEST_ASSERT_EQUAL_INT((int)need, n);
  return n;
}

// Feeds a frame sequence into the reassembler the way can_vesc.cpp does; returns the
// vesc_rx_process() result of the closing frame (-2 if a fill failed first).
static int feed_reply(VescRxBuffer &b, const Frame *fr, int n, const uint8_t **payload, uint16_t *len) {
  for (int i = 0; i < n; ++i) {
    const uint32_t pkt = VESC_EID_PACKET_ID(fr[i].eid);
    if (pkt == VESC_CAN_PACKET_FILL_RX_BUFFER || pkt == VESC_CAN_PACKET_FILL_RX_BUFFER_LONG) {
      if (!vesc_rx_fill(&b, fr[i].d, fr[i].dlc, pkt == VESC_CAN_PACKET_FILL_RX_BUFFER_LONG)) return -2;
    } else if (pkt == VESC_CAN_PACKET_PROCESS_RX_BUFFER) {
      return vesc_rx_process(&b, fr[i].d, fr[i].dlc, payload, len);
    }
  }
  return -3;  // no PROCESS_RX_BUFFER in the sequence
}

// Reference reply (emulated VESC 74 -> us 120) for mask 0x003EC03C: avg_motor_current
// 12.34 A, avg_input_current -5.67 A, avg_id 0.12, avg_iq 12.00, tacho_abs 123456789,
// fault 5, id 74, mos 41.5/42.0/-1.5 C, vd 1.500 V, vq -24.123 V, status 0x02.
static const Frame kRefReply[7] = {
    {0x578, 8, {0x00, 0x32, 0x00, 0x3E, 0xC0, 0x3C, 0x00, 0x00}},
    {0x578, 8, {0x07, 0x04, 0xD2, 0xFF, 0xFF, 0xFD, 0xC9, 0x00}},
    {0x578, 8, {0x0E, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x04, 0xB0}},
    {0x578, 8, {0x15, 0x07, 0x5B, 0xCD, 0x15, 0x05, 0x4A, 0x01}},
    {0x578, 8, {0x1C, 0x9F, 0x01, 0xA4, 0xFF, 0xF1, 0x00, 0x00}},
    {0x578, 8, {0x23, 0x05, 0xDC, 0xFF, 0xFF, 0xA1, 0xC5, 0x02}},
    {0x778, 6, {0x4A, 0x01, 0x00, 0x2A, 0x91, 0x13, 0x00, 0x00}},
};
static const uint8_t kRefPayload[42] = {0x32, 0x00, 0x3E, 0xC0, 0x3C, 0x00, 0x00, 0x04, 0xD2, 0xFF, 0xFF, 0xFD, 0xC9, 0x00,
                                        0x00, 0x00, 0x0C, 0x00, 0x00, 0x04, 0xB0, 0x07, 0x5B, 0xCD, 0x15, 0x05, 0x4A, 0x01,
                                        0x9F, 0x01, 0xA4, 0xFF, 0xF1, 0x00, 0x00, 0x05, 0xDC, 0xFF, 0xFF, 0xA1, 0xC5, 0x02};

// SELECTIVE payload (mask 0x003EC03C) with the values named in the task: fault 5
// (OT_FET), tmos 41.2/42.3/40.1, id/iq 0.12/12.00, vd/vq 1.500/-24.123, tacho_abs
// 123456789, status 0x02. Returns the length (42).
static uint16_t make_selective_payload(uint8_t *buf) {
  uint8_t *p = buf;
  *p++ = (uint8_t)VESC_COMM_GET_VALUES_SELECTIVE;
  put_i32(p, (int32_t)0x003EC03C);
  put_i32(p, 1234);       // avg_motor_current 12.34 A
  put_i32(p, -567);       // avg_input_current -5.67 A
  put_i32(p, 12);         // avg_id 0.12 A
  put_i32(p, 1200);       // avg_iq 12.00 A
  put_i32(p, 123456789);  // tacho_abs
  *p++ = 5;               // fault OVER_TEMP_FET
  *p++ = kVescId;         // controller id
  put_i16(p, 412);        // temp_mos1 41.2
  put_i16(p, 423);        // temp_mos2 42.3
  put_i16(p, 401);        // temp_mos3 40.1
  put_i32(p, 1500);       // vd 1.500 V
  put_i32(p, -24123);     // vq -24.123 V
  *p++ = 0x02;            // status: kill switch
  return (uint16_t)(p - buf);
}

// Plain COMM_GET_VALUES payload: all 22 fields, 74 bytes.
static uint16_t make_full_payload(uint8_t *buf) {
  uint8_t *p = buf;
  *p++ = (uint8_t)VESC_COMM_GET_VALUES;
  put_i16(p, 410);      // temp_fet 41.0
  put_i16(p, 355);      // temp_motor 35.5
  put_i32(p, 1000);     // avg_motor_current 10.00
  put_i32(p, 500);      // avg_input_current 5.00
  put_i32(p, -50);      // avg_id -0.50
  put_i32(p, 1000);     // avg_iq 10.00
  put_i16(p, 500);      // duty 0.500
  put_i32(p, 1000);     // erpm 1000
  put_i16(p, 482);      // v_in 48.2
  put_i32(p, 12345);    // amp_hours 1.2345
  put_i32(p, 1000);     // amp_hours_charged 0.1
  put_i32(p, 1000001);  // watt_hours 100.0001
  put_i32(p, 25000);    // watt_hours_charged 2.5
  put_i32(p, 42);       // tachometer
  put_i32(p, 4242);     // tachometer_abs
  *p++ = 3;             // fault DRV
  put_i32(p, 12500000); // pid_pos 12.5
  *p++ = 7;             // controller id
  put_i16(p, 301);      // temp_mos1 30.1
  put_i16(p, 302);      // temp_mos2 30.2
  put_i16(p, 303);      // temp_mos3 30.3
  put_i32(p, 250);      // vd 0.250
  put_i32(p, -12750);   // vq -12.750
  *p++ = 0x01;          // status: timeout
  return (uint16_t)(p - buf);
}

static VescExt zeroed_ext() {
  VescExt e;
  memset(&e, 0, sizeof e);
  return e;
}

// ---------------------------------------------------------------- CRC
static void test_crc16_check_values() {
  const uint8_t tv[] = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x31C3, vesc_crc16(tv, 9));  // CRC-16/XMODEM check value
  const uint8_t a[] = "A";
  TEST_ASSERT_EQUAL_HEX16(0x58E5, vesc_crc16(a, 1));
  TEST_ASSERT_EQUAL_HEX16(0x0000, vesc_crc16(a, 0));
  TEST_ASSERT_EQUAL_HEX16(0x9113, vesc_crc16(kRefPayload, sizeof kRefPayload));  // from the emulated VESC
}

// ---------------------------------------------------------------- request builder
static void test_request_selective_bytes() {
  uint8_t d[8];
  memset(d, 0xEE, sizeof d);
  const size_t n = vesc_build_getvalues_request(d, kOwnId, VESC_GETVALUES_MASK);
  // EID (8 << 8) | 74, payload [own=120, send=0, COMM_GET_VALUES_SELECTIVE=50, mask BE]
  TEST_ASSERT_EQUAL_HEX32(0x0000084A, VESC_GETVALUES_REQUEST_EID(kVescId));
  TEST_ASSERT_EQUAL_UINT32(7, (uint32_t)n);
  const uint8_t expected[7] = {0x78, 0x00, 0x32, 0x00, 0x3E, 0xC0, 0x3C};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, d, 7);
  TEST_ASSERT_EQUAL_HEX8(0xEE, d[7]);  // untouched beyond the DLC
}

static void test_request_plain_getvalues_bytes() {
  uint8_t d[8];
  const size_t n = vesc_build_getvalues_request(d, kOwnId, 0);  // mask 0 = plain COMM_GET_VALUES
  TEST_ASSERT_EQUAL_UINT32(3, (uint32_t)n);
  const uint8_t expected[3] = {0x78, 0x00, 0x04};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, d, 3);
}

static void test_default_mask_and_lengths() {
  TEST_ASSERT_EQUAL_HEX32(0x003EC03C, VESC_GETVALUES_MASK);
  TEST_ASSERT_EQUAL_UINT16(42, vesc_getvalues_expected_len(VESC_GETVALUES_MASK));
  TEST_ASSERT_EQUAL_UINT16(78, vesc_getvalues_expected_len(VESC_GV_MASK_ALL));  // SELECTIVE, every bit
  TEST_ASSERT_EQUAL_UINT16(74, vesc_getvalues_expected_len(0));                 // plain GET_VALUES
  TEST_ASSERT_EQUAL_UINT16(6, vesc_getvalues_expected_len(1u << VESC_GV_FAULT));
  // the default mask requests exactly the fields no STATUS frame carries
  TEST_ASSERT_EQUAL_HEX32(0, VESC_GETVALUES_MASK & VESC_GV_MASK_TELEMETRY);
  TEST_ASSERT_TRUE(vesc_is_getvalues_reply_pkt(5));
  TEST_ASSERT_TRUE(vesc_is_getvalues_reply_pkt(8));
  TEST_ASSERT_FALSE(vesc_is_getvalues_reply_pkt(4));
  TEST_ASSERT_FALSE(vesc_is_getvalues_reply_pkt(9));  // STATUS_1
  // field counts the decoder's return value is compared against
  TEST_ASSERT_EQUAL_INT(11, vesc_getvalues_field_count(VESC_GETVALUES_MASK));
  TEST_ASSERT_EQUAL_INT(22, vesc_getvalues_field_count(VESC_GV_MASK_ALL));
  TEST_ASSERT_EQUAL_INT(22, vesc_getvalues_field_count(0));  // plain GET_VALUES
  TEST_ASSERT_EQUAL_INT(1, vesc_getvalues_field_count(1u << VESC_GV_FAULT));
  // packet-id gate: only 4 and 50 are answers to us, COMM_PRINT (21) etc. are unsolicited output
  const uint8_t gv[1] = {4}, sel[1] = {50}, prt[6] = {21, 'h', 'e', 'l', 'l', 'o'};
  TEST_ASSERT_TRUE(vesc_is_getvalues_payload(gv, 1));
  TEST_ASSERT_TRUE(vesc_is_getvalues_payload(sel, 1));
  TEST_ASSERT_FALSE(vesc_is_getvalues_payload(prt, 6));
  TEST_ASSERT_FALSE(vesc_is_getvalues_payload(gv, 0));
  TEST_ASSERT_FALSE(vesc_is_getvalues_payload(nullptr, 1));
}

// ---------------------------------------------------------------- reassembly
static void test_reassembly_reference_reply() {
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_EQUAL_INT(1, feed_reply(b, kRefReply, 7, &payload, &len));
  TEST_ASSERT_NOT_NULL(payload);
  TEST_ASSERT_EQUAL_UINT16(42, len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kRefPayload, payload, 42);
  TEST_ASSERT_EQUAL_UINT16(0, b.len_filled);  // consumed
  TEST_ASSERT_EQUAL_HEX32(0x003EC03C, vesc_getvalues_mask(payload, len));

  // the emulator reproduces the reference frames byte for byte
  Frame fr[16];
  const int n = emit_reply(fr, 16, kVescId, kOwnId, kRefPayload, sizeof kRefPayload);
  TEST_ASSERT_EQUAL_INT(7, n);
  for (int i = 0; i < 7; ++i) {
    TEST_ASSERT_EQUAL_HEX32(kRefReply[i].eid, fr[i].eid);
    TEST_ASSERT_EQUAL_UINT8(kRefReply[i].dlc, fr[i].dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kRefReply[i].d, fr[i].d, kRefReply[i].dlc);
  }
}

static void test_reassembly_lost_fill_frame() {
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, kRefReply[0].d, kRefReply[0].dlc, false));
  TEST_ASSERT_EQUAL_UINT16(7, b.len_filled);
  // frame with offset 14 arrives while only 7 bytes are filled -> gap -> reply lost
  TEST_ASSERT_FALSE(vesc_rx_fill(&b, kRefReply[2].d, kRefReply[2].dlc, false));
  TEST_ASSERT_EQUAL_UINT16(0, b.len_filled);
  // the closing frame then fails on the declared length (42 != 0)
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_EQUAL_INT(0, vesc_rx_process(&b, kRefReply[6].d, kRefReply[6].dlc, &payload, &len));
}

// A duplicated fragment (CAN retransmits a frame the receiver already accepted when
// the transmitter saw an error in the last EOF bit) must not cost the reply: the
// VESC's own decode_msg() ignores a FILL whose offset matches no buffer, so do we.
static void test_reassembly_duplicate_fill_ignored() {
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  for (int i = 0; i < 3; ++i) TEST_ASSERT_TRUE(vesc_rx_fill(&b, kRefReply[i].d, kRefReply[i].dlc, false));
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, kRefReply[2].d, kRefReply[2].dlc, false));  // offset 14 again
  TEST_ASSERT_EQUAL_UINT16(21, b.len_filled);                                     // unchanged
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, kRefReply[1].d, kRefReply[1].dlc, false));  // older duplicate, offset 7
  TEST_ASSERT_EQUAL_UINT16(21, b.len_filled);
  TEST_ASSERT_EQUAL_INT(1, feed_reply(b, kRefReply + 3, 4, &payload, &len));  // the rest completes it
  TEST_ASSERT_EQUAL_UINT16(42, len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kRefPayload, payload, 42);

  // a duplicate whose payload differs (a foreign packet interleaved at a matching offset)
  // is ignored the same way: the CRC at the end decides
  memset(&b, 0, sizeof b);
  for (int i = 0; i < 3; ++i) TEST_ASSERT_TRUE(vesc_rx_fill(&b, kRefReply[i].d, kRefReply[i].dlc, false));
  uint8_t alien[8];
  memcpy(alien, kRefReply[1].d, 8);
  alien[3] ^= 0xFF;
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, alien, 8, false));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kRefPayload, b.data, 21);  // original bytes kept
  TEST_ASSERT_EQUAL_INT(1, feed_reply(b, kRefReply + 3, 4, &payload, &len));

  // a duplicate of the FIRST fragment restarts the buffer (offset 0 always does), so the
  // remaining fragments have to follow again; here they do not -> gap -> lost
  memset(&b, 0, sizeof b);
  for (int i = 0; i < 3; ++i) TEST_ASSERT_TRUE(vesc_rx_fill(&b, kRefReply[i].d, kRefReply[i].dlc, false));
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, kRefReply[0].d, kRefReply[0].dlc, false));
  TEST_ASSERT_EQUAL_UINT16(7, b.len_filled);
  TEST_ASSERT_FALSE(vesc_rx_fill(&b, kRefReply[3].d, kRefReply[3].dlc, false));
  TEST_ASSERT_EQUAL_UINT16(0, b.len_filled);
}

static void test_reassembly_offset_zero_restarts() {
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  // half a reply, then the VESC starts a new one: offset 0 restarts the buffer
  for (int i = 0; i < 3; ++i) TEST_ASSERT_TRUE(vesc_rx_fill(&b, kRefReply[i].d, kRefReply[i].dlc, false));
  TEST_ASSERT_EQUAL_UINT16(21, b.len_filled);
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_EQUAL_INT(1, feed_reply(b, kRefReply, 7, &payload, &len));
  TEST_ASSERT_EQUAL_UINT16(42, len);
}

static void test_reassembly_bad_crc_and_bad_length() {
  Frame fr[8];
  memcpy(fr, kRefReply, sizeof kRefReply);
  fr[3].d[4] ^= 0x01;  // one flipped payload bit
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_EQUAL_INT(0, feed_reply(b, fr, 7, &payload, &len));
  TEST_ASSERT_EQUAL_UINT16(0, b.len_filled);

  memcpy(fr, kRefReply, sizeof kRefReply);
  fr[6].d[3] = 0x29;  // declared length 41 while 42 bytes were filled
  TEST_ASSERT_EQUAL_INT(0, feed_reply(b, fr, 7, &payload, &len));

  memcpy(fr, kRefReply, sizeof kRefReply);
  fr[6].d[4] ^= 0x80;  // corrupted CRC field
  TEST_ASSERT_EQUAL_INT(0, feed_reply(b, fr, 7, &payload, &len));
}

static void test_process_frame_not_a_reply() {
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  for (int i = 0; i < 6; ++i) TEST_ASSERT_TRUE(vesc_rx_fill(&b, kRefReply[i].d, kRefReply[i].dlc, false));
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  // send = 0: a command forwarded TO our id, never a reply -> -1, buffer dropped
  uint8_t cmd[6] = {0x4A, 0x00, 0x00, 0x2A, 0x91, 0x13};
  TEST_ASSERT_EQUAL_INT(-1, vesc_rx_process(&b, cmd, 6, &payload, &len));
  TEST_ASSERT_EQUAL_UINT16(0, b.len_filled);
  // short PROCESS_RX_BUFFER frame
  TEST_ASSERT_EQUAL_INT(-1, vesc_rx_process(&b, kRefReply[6].d, 5, &payload, &len));
  // declared length 0 is never valid
  uint8_t zero[6] = {0x4A, 0x01, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_EQUAL_INT(0, vesc_rx_process(&b, zero, 6, &payload, &len));
}

static void test_fill_rejects_empty_and_oversized_frames() {
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  uint8_t f[8] = {0};
  TEST_ASSERT_FALSE(vesc_rx_fill(&b, f, 1, false));  // offset only, no data
  TEST_ASSERT_FALSE(vesc_rx_fill(&b, f, 2, true));   // long offset only, no data
  TEST_ASSERT_FALSE(vesc_rx_fill(&b, f, 9, false));  // DLC > 8 is not a classic CAN frame
  TEST_ASSERT_FALSE(vesc_rx_fill(&b, f, 0, false));
  // a long offset just past the buffer end must not write out of bounds
  b.len_filled = (uint16_t)(VESC_RX_BUFFER_SIZE - 3);
  f[0] = (uint8_t)((VESC_RX_BUFFER_SIZE - 3) >> 8);
  f[1] = (uint8_t)(VESC_RX_BUFFER_SIZE - 3);
  TEST_ASSERT_FALSE(vesc_rx_fill(&b, f, 8, true));  // offset + 6 > size
  TEST_ASSERT_EQUAL_UINT16(0, b.len_filled);
  // exactly filling the buffer is fine
  b.len_filled = (uint16_t)(VESC_RX_BUFFER_SIZE - 6);
  f[0] = (uint8_t)((VESC_RX_BUFFER_SIZE - 6) >> 8);
  f[1] = (uint8_t)(VESC_RX_BUFFER_SIZE - 6);
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, f, 8, true));
  TEST_ASSERT_EQUAL_UINT16(VESC_RX_BUFFER_SIZE, b.len_filled);
}

// Edge cases of the reassembler that the reference reply does not exercise: a closing
// frame with no fragments at all, duplicate / restarting LONG fragments, the last
// short-offset frame (offset 255 + 7 = 262 must still fit), a SELECTIVE header cut
// short, and the id masking of the EID macro.
static void test_reassembly_edge_offsets() {
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  // PROCESS_RX_BUFFER on a fresh buffer (every FILL lost): length mismatch, never a payload
  TEST_ASSERT_EQUAL_INT(0, vesc_rx_process(&b, kRefReply[6].d, kRefReply[6].dlc, &payload, &len));
  TEST_ASSERT_NULL(payload);
  TEST_ASSERT_EQUAL_UINT16(0, b.len_filled);

  // a short FILL at offset 255 carrying 7 bytes (the VESC's loop stops at i > 255, so
  // offset 255 itself never occurs, but 252 + 7 = 259 does; both must fit the buffer)
  uint8_t f[8] = {0xFF, 1, 2, 3, 4, 5, 6, 7};
  b.len_filled = 255;
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, f, 8, false));
  TEST_ASSERT_EQUAL_UINT16(262, b.len_filled);
  TEST_ASSERT_EQUAL_HEX8(7, b.data[261]);

  // LONG duplicate (offset below the fill level) is ignored, LONG at the fill level appends
  uint8_t l[8] = {0x01, 0x06, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6};  // offset 262
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, l, 8, true));
  TEST_ASSERT_EQUAL_UINT16(268, b.len_filled);
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, l, 8, true));  // same frame again
  TEST_ASSERT_EQUAL_UINT16(268, b.len_filled);
  TEST_ASSERT_EQUAL_HEX8(0xA6, b.data[267]);
  // LONG with a gap (offset 300 while 268 are filled) drops the reply
  l[0] = 0x01;
  l[1] = 0x2C;
  TEST_ASSERT_FALSE(vesc_rx_fill(&b, l, 8, true));
  TEST_ASSERT_EQUAL_UINT16(0, b.len_filled);
  // LONG at offset 0 (re)starts the buffer like a short offset 0 does
  b.len_filled = 100;
  l[0] = 0;
  l[1] = 0;
  TEST_ASSERT_TRUE(vesc_rx_fill(&b, l, 5, true));  // 3 data bytes
  TEST_ASSERT_EQUAL_UINT16(3, b.len_filled);
  TEST_ASSERT_EQUAL_HEX8(0xA1, b.data[0]);

  // SELECTIVE payload cut inside the mask: not decodable, mask unknown
  const uint8_t hdr4[4] = {0x32, 0x00, 0x3E, 0xC0};
  TEST_ASSERT_EQUAL_HEX32(0, vesc_getvalues_mask(hdr4, 4));
  TEST_ASSERT_TRUE(vesc_is_getvalues_payload(hdr4, 4));  // packet id alone is ours ...
  VescExt e = zeroed_ext();
  TEST_ASSERT_EQUAL_INT(-1, vesc_decode_getvalues(hdr4, 4, &e, nullptr));  // ... but the header is incomplete

  // EID helpers keep the controller id to 8 bits and the packet id above it
  TEST_ASSERT_EQUAL_HEX32(0x0000084A, VESC_GETVALUES_REQUEST_EID(0x14A));
  TEST_ASSERT_EQUAL_HEX32(0x00000578, VESC_EID(VESC_CAN_PACKET_FILL_RX_BUFFER, kOwnId));
  TEST_ASSERT_EQUAL_HEX32(0x00000778, VESC_EID(VESC_CAN_PACKET_PROCESS_RX_BUFFER, kOwnId));
  TEST_ASSERT_EQUAL_UINT8(kOwnId, VESC_EID_CONTROLLER_ID(0x00000778));
  TEST_ASSERT_EQUAL_UINT32(7, VESC_EID_PACKET_ID(0x00000778));
}

// A SELECTIVE reply that carries STATUS-frame fields (bits 0 and 8: someone changed the
// mask) updates the telemetry VALUES only and leaves VescExt alone.
static void test_decode_selective_with_telemetry_bits() {
  uint8_t buf[16];
  uint8_t *p = buf;
  *p++ = (uint8_t)VESC_COMM_GET_VALUES_SELECTIVE;
  put_i32(p, (int32_t)((1u << VESC_GV_TEMP_FET) | (1u << VESC_GV_V_IN)));
  put_i16(p, 512);  // temp_fet 51.2
  put_i16(p, 470);  // v_in 47.0
  const uint16_t len = (uint16_t)(p - buf);
  TEST_ASSERT_EQUAL_UINT16(9, len);
  TEST_ASSERT_EQUAL_UINT16(9, vesc_getvalues_expected_len((1u << VESC_GV_TEMP_FET) | (1u << VESC_GV_V_IN)));
  TEST_ASSERT_TRUE((vesc_getvalues_mask(buf, len) & VESC_GV_MASK_TELEMETRY) != 0);  // -> can_vesc.cpp copies t back
  VescExt e = zeroed_ext();
  e.tacho_abs = 77;
  vesc_telemetry_t t;
  memset(&t, 0, sizeof t);
  t.t_ms[VESC_IDX_STATUS_4] = 555;
  t.t_ms[VESC_IDX_STATUS_5] = 556;
  TEST_ASSERT_EQUAL_INT(2, vesc_decode_getvalues(buf, len, &e, &t));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 51.2f, t.temp_fet);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 47.0f, t.v_in);
  TEST_ASSERT_EQUAL_UINT32(555, t.t_ms[VESC_IDX_STATUS_4]);  // freshness stamps untouched
  TEST_ASSERT_EQUAL_UINT32(556, t.t_ms[VESC_IDX_STATUS_5]);
  TEST_ASSERT_EQUAL_INT32(77, e.tacho_abs);  // nothing of VescExt requested
  TEST_ASSERT_EQUAL_UINT8(0, e.fault_code);
  // without a telemetry target the fields are skipped, not misaligned: still 2 fields
  TEST_ASSERT_EQUAL_INT(2, vesc_decode_getvalues(buf, len, &e, nullptr));
}

// 265-byte payload: 37 FILL_RX_BUFFER frames (offsets 0..252 step 7), one
// FILL_RX_BUFFER_LONG at offset 259 (6 bytes) and PROCESS_RX_BUFFER [.. 01 09 crc].
static void test_reassembly_long_frames_past_offset_255() {
  uint8_t big[265];
  for (unsigned i = 0; i < sizeof big; ++i) big[i] = (uint8_t)(i * 7u + 3u);
  Frame fr[48];
  const int n = emit_reply(fr, 48, kVescId, kOwnId, big, sizeof big);
  TEST_ASSERT_EQUAL_INT(39, n);
  TEST_ASSERT_EQUAL_HEX32(VESC_EID(VESC_CAN_PACKET_FILL_RX_BUFFER, kOwnId), fr[36].eid);
  TEST_ASSERT_EQUAL_HEX8(0xFC, fr[36].d[0]);  // last short frame at offset 252
  TEST_ASSERT_EQUAL_HEX32(VESC_EID(VESC_CAN_PACKET_FILL_RX_BUFFER_LONG, kOwnId), fr[37].eid);
  TEST_ASSERT_EQUAL_HEX8(0x01, fr[37].d[0]);  // offset 259 = 0x0103
  TEST_ASSERT_EQUAL_HEX8(0x03, fr[37].d[1]);
  TEST_ASSERT_EQUAL_UINT8(8, fr[37].dlc);
  TEST_ASSERT_EQUAL_HEX8(0x01, fr[38].d[2]);  // len 265 = 0x0109
  TEST_ASSERT_EQUAL_HEX8(0x09, fr[38].d[3]);

  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_EQUAL_INT(1, feed_reply(b, fr, n, &payload, &len));
  TEST_ASSERT_EQUAL_UINT16(265, len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(big, payload, 265);

  // a lost LONG frame: feed the short ones, skip the LONG one -> length mismatch
  memset(&b, 0, sizeof b);
  for (int i = 0; i < 37; ++i) TEST_ASSERT_TRUE(vesc_rx_fill(&b, fr[i].d, fr[i].dlc, false));
  TEST_ASSERT_EQUAL_INT(0, vesc_rx_process(&b, fr[38].d, fr[38].dlc, &payload, &len));
}

// 7-byte payload is the smallest fragmented reply: one FILL + PROCESS.
static void test_reassembly_seven_byte_boundary() {
  const uint8_t p7[7] = {0x32, 0x00, 0x00, 0x00, 0x04, 0x00, 0x2A};  // SELECTIVE, mask bit 2 only
  Frame fr[4];
  const int n = emit_reply(fr, 4, kVescId, kOwnId, p7, 7);
  TEST_ASSERT_EQUAL_INT(2, n);
  TEST_ASSERT_EQUAL_UINT8(8, fr[0].dlc);
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_EQUAL_INT(1, feed_reply(b, fr, n, &payload, &len));
  TEST_ASSERT_EQUAL_UINT16(7, len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(p7, payload, 7);
}

// ---------------------------------------------------------------- decode
static void test_decode_selective_payload() {
  uint8_t buf[64];
  const uint16_t len = make_selective_payload(buf);
  TEST_ASSERT_EQUAL_UINT16(42, len);
  TEST_ASSERT_EQUAL_UINT16(vesc_getvalues_expected_len(VESC_GETVALUES_MASK), len);

  VescExt e = zeroed_ext();
  e.t_ms = 999;
  e.polls_sent = 5;
  vesc_telemetry_t t;
  memset(&t, 0, sizeof t);
  TEST_ASSERT_EQUAL_INT(11, vesc_decode_getvalues(buf, len, &e, &t));  // every requested field present
  TEST_ASSERT_EQUAL_INT(vesc_getvalues_field_count(VESC_GETVALUES_MASK), 11);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.34f, e.avg_motor_current);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -5.67f, e.avg_input_current);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.12f, e.avg_id);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, e.avg_iq);
  TEST_ASSERT_EQUAL_INT32(123456789, e.tacho_abs);
  TEST_ASSERT_EQUAL_UINT8(5, e.fault_code);
  TEST_ASSERT_EQUAL_STRING("OT_FET", vesc_fault_str(e.fault_code));
  TEST_ASSERT_EQUAL_STRING("OVER_TEMP_FET", vesc_fault_name(e.fault_code));
  TEST_ASSERT_EQUAL_UINT8(kVescId, e.vesc_id);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 41.2f, e.temp_mos1);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 42.3f, e.temp_mos2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 40.1f, e.temp_mos3);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.5f, e.vd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -24.123f, e.vq);
  TEST_ASSERT_EQUAL_HEX8(0x02, e.status);
  TEST_ASSERT_TRUE(e.status & VESC_STATUS_KILL_SW);
  TEST_ASSERT_FALSE(e.status & VESC_STATUS_TIMEOUT);
  // bookkeeping fields are the caller's business
  TEST_ASSERT_EQUAL_UINT32(999, e.t_ms);
  TEST_ASSERT_EQUAL_UINT32(5, e.polls_sent);
  // the mask carries no STATUS field: telemetry stays untouched
  vesc_telemetry_t zero;
  memset(&zero, 0, sizeof zero);
  TEST_ASSERT_TRUE(memcmp(&t, &zero, sizeof t) == 0);
  // also fine without the telemetry pointer
  VescExt e2 = zeroed_ext();
  TEST_ASSERT_EQUAL_INT(11, vesc_decode_getvalues(buf, len, &e2, nullptr));
  TEST_ASSERT_EQUAL_INT32(123456789, e2.tacho_abs);
}

static void test_decode_reference_reply_payload() {
  VescExt e = zeroed_ext();
  TEST_ASSERT_EQUAL_INT(11, vesc_decode_getvalues(kRefPayload, sizeof kRefPayload, &e, nullptr));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.34f, e.avg_motor_current);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -5.67f, e.avg_input_current);
  TEST_ASSERT_EQUAL_INT32(123456789, e.tacho_abs);
  TEST_ASSERT_EQUAL_UINT8(5, e.fault_code);
  TEST_ASSERT_EQUAL_UINT8(74, e.vesc_id);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 41.5f, e.temp_mos1);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 42.0f, e.temp_mos2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.5f, e.temp_mos3);  // negative i16
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.5f, e.vd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -24.123f, e.vq);
  TEST_ASSERT_EQUAL_HEX8(0x02, e.status);
}

static void test_decode_full_getvalues_payload() {
  uint8_t buf[96];
  const uint16_t len = make_full_payload(buf);
  TEST_ASSERT_EQUAL_UINT16(74, len);
  TEST_ASSERT_EQUAL_HEX32(VESC_GV_MASK_ALL, vesc_getvalues_mask(buf, len));

  VescExt e = zeroed_ext();
  vesc_telemetry_t t;
  memset(&t, 0, sizeof t);
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) t.t_ms[i] = 777;  // freshness stamps belong to the broadcasts
  t.adc1 = 1.25f;                                              // not in GET_VALUES: must survive
  TEST_ASSERT_EQUAL_INT(22, vesc_decode_getvalues(buf, len, &e, &t));
  // VescExt part
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, e.avg_motor_current);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, e.avg_input_current);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, e.avg_id);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, e.avg_iq);
  TEST_ASSERT_EQUAL_INT32(4242, e.tacho_abs);
  TEST_ASSERT_EQUAL_UINT8(3, e.fault_code);
  TEST_ASSERT_EQUAL_STRING("DRV", vesc_fault_str(e.fault_code));
  TEST_ASSERT_EQUAL_UINT8(7, e.vesc_id);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.1f, e.temp_mos1);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.2f, e.temp_mos2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.3f, e.temp_mos3);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.25f, e.vd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -12.75f, e.vq);
  TEST_ASSERT_EQUAL_HEX8(0x01, e.status);
  // telemetry part (values only)
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 41.0f, t.temp_fet);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 35.5f, t.temp_motor);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, t.duty);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, t.erpm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 48.2f, t.v_in);
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, 1.2345f, t.amp_hours);
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.1f, t.amp_hours_charged);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0001f, t.watt_hours);
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, 2.5f, t.watt_hours_charged);
  TEST_ASSERT_EQUAL_INT32(42, t.tachometer);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 12.5f, t.pid_pos);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.25f, t.adc1);
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) TEST_ASSERT_EQUAL_UINT32(777, t.t_ms[i]);
  // the same payload fragments into 11 FILL frames + PROCESS and comes back intact
  Frame fr[16];
  const int n = emit_reply(fr, 16, 7, kOwnId, buf, len);
  TEST_ASSERT_EQUAL_INT(12, n);
  VescRxBuffer b;
  memset(&b, 0, sizeof b);
  const uint8_t *payload = nullptr;
  uint16_t plen = 0;
  TEST_ASSERT_EQUAL_INT(1, feed_reply(b, fr, n, &payload, &plen));
  TEST_ASSERT_EQUAL_UINT16(74, plen);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, payload, 74);
}

// Firmware older than the requested mask echoes the mask but appends only the fields
// it knows: the decoder returns how many it got, the leading ones are valid and a
// field cut in half is never decoded.
static void test_decode_truncated_reply_is_partial() {
  uint8_t buf[64];
  const uint16_t len = make_selective_payload(buf);
  VescExt e = zeroed_ext();
  e.status = 0xAA;
  // one byte short (what a VESC with firmware < 5.03 sends: mask echoed, bit 21 omitted)
  TEST_ASSERT_EQUAL_INT(10, vesc_decode_getvalues(buf, (uint16_t)(len - 1), &e, nullptr));
  TEST_ASSERT_TRUE(vesc_decode_getvalues(buf, (uint16_t)(len - 1), &e, nullptr) <
                   vesc_getvalues_field_count(VESC_GETVALUES_MASK));  // what can_vesc.cpp warns about
  TEST_ASSERT_EQUAL_HEX8(0xAA, e.status);  // the missing field keeps its previous value
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -24.123f, e.vq);  // the last complete one was decoded
  TEST_ASSERT_EQUAL_INT32(123456789, e.tacho_abs);
  // cut inside the first 4-byte field: header ok, nothing decoded, *out untouched
  e = zeroed_ext();
  TEST_ASSERT_EQUAL_INT(0, vesc_decode_getvalues(buf, 7, &e, nullptr));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, e.avg_motor_current);
  // header only
  TEST_ASSERT_EQUAL_INT(0, vesc_decode_getvalues(buf, 5, &e, nullptr));
  // cut inside the header / empty / null: hard errors
  TEST_ASSERT_EQUAL_INT(-1, vesc_decode_getvalues(buf, 4, &e, nullptr));
  TEST_ASSERT_EQUAL_INT(-1, vesc_decode_getvalues(buf, 0, &e, nullptr));
  TEST_ASSERT_EQUAL_INT(-1, vesc_decode_getvalues(nullptr, len, &e, nullptr));
  TEST_ASSERT_EQUAL_INT(-1, vesc_decode_getvalues(buf, len, nullptr, nullptr));
  // plain GET_VALUES from FW < 5.03: 73 bytes = 21 of 22 fields
  uint8_t full[96];
  const uint16_t flen = make_full_payload(full);
  e = zeroed_ext();
  e.status = 0x55;
  TEST_ASSERT_EQUAL_INT(21, vesc_decode_getvalues(full, (uint16_t)(flen - 1), &e, nullptr));
  TEST_ASSERT_EQUAL_HEX8(0x55, e.status);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -12.75f, e.vq);
  TEST_ASSERT_EQUAL_INT(0, vesc_decode_getvalues(full, 1, &e, nullptr));
  TEST_ASSERT_EQUAL_INT(0, vesc_decode_getvalues(full, 2, &e, nullptr));  // temp_fet needs 2 bytes after the id
}

static void test_decode_unknown_packet_and_unknown_bits_rejected() {
  uint8_t buf[64];
  const uint16_t len = make_selective_payload(buf);
  VescExt e = zeroed_ext();
  buf[0] = 47;  // COMM_GET_VALUES_SETUP: different layout, never decode it as GET_VALUES
  TEST_ASSERT_EQUAL_INT(-1, vesc_decode_getvalues(buf, len, &e, nullptr));
  TEST_ASSERT_EQUAL_HEX32(0, vesc_getvalues_mask(buf, len));
  TEST_ASSERT_FALSE(vesc_is_getvalues_payload(buf, len));
  buf[0] = (uint8_t)VESC_COMM_GET_VALUES_SELECTIVE;
  buf[1] = 0x01;  // mask bit 24: unknown field size -> reject instead of misaligning
  TEST_ASSERT_EQUAL_INT(-1, vesc_decode_getvalues(buf, len, &e, nullptr));
  TEST_ASSERT_EQUAL_INT32(0, e.tacho_abs);  // nothing written on a hard error
}

static void test_decode_trailing_bytes_tolerated_and_untouched_fields_kept() {
  uint8_t buf[64];
  uint16_t len = make_selective_payload(buf);
  buf[len++] = 0xDE;  // a future firmware appending something after the known fields
  buf[len++] = 0xAD;
  VescExt e = zeroed_ext();
  TEST_ASSERT_EQUAL_INT(11, vesc_decode_getvalues(buf, len, &e, nullptr));
  TEST_ASSERT_EQUAL_HEX8(0x02, e.status);

  // a mask without bit 21 leaves status (and everything else not requested) as it was
  uint8_t small[8];
  uint8_t *p = small;
  *p++ = (uint8_t)VESC_COMM_GET_VALUES_SELECTIVE;
  put_i32(p, (int32_t)(1u << VESC_GV_FAULT));
  *p++ = 19;  // BRK
  VescExt keep = zeroed_ext();
  keep.status = 0xAA;
  keep.vesc_id = 74;
  keep.tacho_abs = -1;
  TEST_ASSERT_EQUAL_INT(1, vesc_decode_getvalues(small, (uint16_t)(p - small), &keep, nullptr));  // 6 bytes = a PROCESS_SHORT_BUFFER reply
  TEST_ASSERT_EQUAL_UINT8(19, keep.fault_code);
  TEST_ASSERT_EQUAL_STRING("BRK", vesc_fault_str(keep.fault_code));
  TEST_ASSERT_EQUAL_HEX8(0xAA, keep.status);
  TEST_ASSERT_EQUAL_UINT8(74, keep.vesc_id);
  TEST_ASSERT_EQUAL_INT32(-1, keep.tacho_abs);
  TEST_ASSERT_EQUAL_UINT16(6, vesc_getvalues_expected_len(1u << VESC_GV_FAULT));
}

// ---------------------------------------------------------------- fault strings
static void test_fault_strings() {
  TEST_ASSERT_EQUAL_STRING("NONE", vesc_fault_str(0));
  TEST_ASSERT_EQUAL_STRING("OVER_V", vesc_fault_str(1));
  TEST_ASSERT_EQUAL_STRING("UNDER_V", vesc_fault_str(2));
  TEST_ASSERT_EQUAL_STRING("DRV", vesc_fault_str(3));
  TEST_ASSERT_EQUAL_STRING("ABS_OC", vesc_fault_str(4));
  TEST_ASSERT_EQUAL_STRING("OT_FET", vesc_fault_str(5));
  TEST_ASSERT_EQUAL_STRING("OT_MOT", vesc_fault_str(6));
  TEST_ASSERT_EQUAL_STRING("LV_OUT", vesc_fault_str(29));
  TEST_ASSERT_EQUAL_STRING("ABS_OSPD", vesc_fault_str(33));
  TEST_ASSERT_EQUAL_STRING("F?", vesc_fault_str(34));
  TEST_ASSERT_EQUAL_STRING("F?", vesc_fault_str(255));
  TEST_ASSERT_EQUAL_STRING("NONE", vesc_fault_name(0));
  TEST_ASSERT_EQUAL_STRING("OVER_VOLTAGE", vesc_fault_name(1));
  TEST_ASSERT_EQUAL_STRING("ABS_OVER_CURRENT", vesc_fault_name(4));
  TEST_ASSERT_EQUAL_STRING("BOOTING_FROM_WATCHDOG_RESET", vesc_fault_name(10));
  TEST_ASSERT_EQUAL_STRING("ABS_OVERSPEED", vesc_fault_name(33));
  TEST_ASSERT_EQUAL_STRING("UNKNOWN", vesc_fault_name(34));
  TEST_ASSERT_EQUAL_STRING("UNKNOWN", vesc_fault_name(255));
  // every short string fits the 8-glyph display cell, every name is non-empty
  for (unsigned c = 0; c < 256; ++c) {
    TEST_ASSERT_TRUE(strlen(vesc_fault_str((uint8_t)c)) <= 8);
    TEST_ASSERT_TRUE(strlen(vesc_fault_str((uint8_t)c)) >= 2);
    TEST_ASSERT_TRUE(strlen(vesc_fault_name((uint8_t)c)) >= 3);
  }
}

static void test_fault_fmt() {
  char buf[16];
  TEST_ASSERT_EQUAL_STRING("OT_FET", vesc_fault_fmt(buf, sizeof buf, 5));
  TEST_ASSERT_EQUAL_STRING("NONE", vesc_fault_fmt(buf, sizeof buf, 0));
  TEST_ASSERT_EQUAL_STRING("F34", vesc_fault_fmt(buf, sizeof buf, 34));
  TEST_ASSERT_EQUAL_STRING("F37", vesc_fault_fmt(buf, sizeof buf, 37));
  TEST_ASSERT_EQUAL_STRING("F100", vesc_fault_fmt(buf, sizeof buf, 100));
  TEST_ASSERT_EQUAL_STRING("F255", vesc_fault_fmt(buf, sizeof buf, 255));
  // truncation to the buffer, always NUL-terminated
  TEST_ASSERT_EQUAL_STRING("RSL", vesc_fault_fmt(buf, 4, 20));
  TEST_ASSERT_EQUAL_STRING("F2", vesc_fault_fmt(buf, 3, 255));
  TEST_ASSERT_EQUAL_STRING("", vesc_fault_fmt(buf, 1, 5));
  for (unsigned c = 0; c < 256; ++c) TEST_ASSERT_TRUE(strlen(vesc_fault_fmt(buf, sizeof buf, (uint8_t)c)) <= 8);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_check_values);
  RUN_TEST(test_request_selective_bytes);
  RUN_TEST(test_request_plain_getvalues_bytes);
  RUN_TEST(test_default_mask_and_lengths);
  RUN_TEST(test_reassembly_reference_reply);
  RUN_TEST(test_reassembly_lost_fill_frame);
  RUN_TEST(test_reassembly_duplicate_fill_ignored);
  RUN_TEST(test_reassembly_offset_zero_restarts);
  RUN_TEST(test_reassembly_bad_crc_and_bad_length);
  RUN_TEST(test_process_frame_not_a_reply);
  RUN_TEST(test_fill_rejects_empty_and_oversized_frames);
  RUN_TEST(test_reassembly_edge_offsets);
  RUN_TEST(test_reassembly_long_frames_past_offset_255);
  RUN_TEST(test_reassembly_seven_byte_boundary);
  RUN_TEST(test_decode_selective_with_telemetry_bits);
  RUN_TEST(test_decode_selective_payload);
  RUN_TEST(test_decode_reference_reply_payload);
  RUN_TEST(test_decode_full_getvalues_payload);
  RUN_TEST(test_decode_truncated_reply_is_partial);
  RUN_TEST(test_decode_unknown_packet_and_unknown_bits_rejected);
  RUN_TEST(test_decode_trailing_bytes_tolerated_and_untouched_fields_kept);
  RUN_TEST(test_fault_strings);
  RUN_TEST(test_fault_fmt);
  return UNITY_END();
}
