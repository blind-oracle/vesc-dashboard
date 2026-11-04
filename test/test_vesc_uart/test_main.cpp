// Native Unity tests for include/vesc_uart.h: the UART framing of the COMM_GET_VALUES
// request, the buffer-based reply reassembler (junk, damaged frames, the stop byte that
// doubles as the long-form start byte, idle flush, overflow) and the state update from
// one reply (field mapping, STATUS slot stamping, EMA seeding, fault latch). Plus the
// UART link labels of the display builders, which only exist with VESC_LINK_UART.
//   pio test -e native
// Arduino-free: vesc_uart.h (-> vesc_getvalues.h, shared_state.h), display_strings_oled.h
// and <unity.h>.
//
// The request reference bytes 02 01 04 40 84 03 are the frame vedderb/bldc
// packet_send_packet() emits for the one-byte payload COMM_GET_VALUES (CRC-16/XMODEM of
// 0x04 = 0x4084), the same bytes every VESC UART client sends.

// This suite covers the UART build: every test directory is compiled on its own, so the
// switch only affects this file (config.h defaults are all #ifndef).
#define VESC_LINK_UART 1

#include <unity.h>

#include <stdint.h>
#include <string.h>

#include "display_strings_oled.h"
#include "vesc_uart.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- helpers
static const uint8_t kVescId = 74;

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

struct Reply {
  int16_t v_in_x10 = 503;
  uint8_t fault = 5;  // OT_FET
};

// A plain COMM_GET_VALUES reply payload carrying the first nfields fields (22 = all,
// 74 bytes; 21 = firmware older than 5.03, without the status byte).
static uint16_t build_reply(uint8_t *out, int nfields, const Reply &r = Reply()) {
  uint8_t *p = out;
  *p++ = VESC_COMM_GET_VALUES;
  for (int b = 0; b < nfields && b < VESC_GV_BIT_COUNT; ++b) {
    switch (b) {
      case VESC_GV_TEMP_FET: put_i16(p, 415); break;
      case VESC_GV_TEMP_MOTOR: put_i16(p, 332); break;
      case VESC_GV_AVG_MOTOR_CURRENT: put_i32(p, 5234); break;
      case VESC_GV_AVG_INPUT_CURRENT: put_i32(p, 2107); break;
      case VESC_GV_AVG_ID: put_i32(p, -25); break;
      case VESC_GV_AVG_IQ: put_i32(p, 5210); break;
      case VESC_GV_DUTY: put_i16(p, 456); break;
      case VESC_GV_RPM: put_i32(p, 12345); break;
      case VESC_GV_V_IN: put_i16(p, r.v_in_x10); break;
      case VESC_GV_AMP_HOURS: put_i32(p, 12345); break;
      case VESC_GV_AMP_HOURS_CHARGED: put_i32(p, 123); break;
      case VESC_GV_WATT_HOURS: put_i32(p, 612345); break;
      case VESC_GV_WATT_HOURS_CHARGED: put_i32(p, 5000); break;
      case VESC_GV_TACHO: put_i32(p, 98765); break;
      case VESC_GV_TACHO_ABS: put_i32(p, 123456); break;
      case VESC_GV_FAULT: *p++ = r.fault; break;
      case VESC_GV_PID_POS: put_i32(p, 12500000); break;
      case VESC_GV_VESC_ID: *p++ = kVescId; break;
      case VESC_GV_TEMP_MOS123: put_i16(p, 410); put_i16(p, 425); put_i16(p, 405); break;
      case VESC_GV_VD: put_i32(p, 1234); break;
      case VESC_GV_VQ: put_i32(p, 23456); break;
      case VESC_GV_STATUS: *p++ = VESC_STATUS_KILL_SW; break;
      default: break;
    }
  }
  return (uint16_t)(p - out);
}

static uint16_t build_reply_frame(uint8_t *out, size_t cap, int nfields = 22, const Reply &r = Reply()) {
  uint8_t pl[128];
  const uint16_t n = build_reply(pl, nfields, r);
  return (uint16_t)vesc_uart_frame(out, cap, pl, n);
}

static VescUartRx g_rx;  // ~1 KB: static, as in the firmware

static void fresh_state(VescState &v, VescExt &x) {
  memset(&v, 0, sizeof v);
  memset(&x, 0, sizeof x);
  v.locked_id = -1;  // as state_init() sets it
}

// ---------------------------------------------------------------- framing
static void test_request_bytes() {
  uint8_t req[VESC_UART_GETVALUES_REQUEST_LEN];
  const uint8_t expect[] = {0x02, 0x01, 0x04, 0x40, 0x84, 0x03};
  TEST_ASSERT_EQUAL_UINT32(6, vesc_uart_build_getvalues_request(req));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, req, 6);
}

static void test_frame_rejects_bad_sizes() {
  uint8_t out[600], pl[600] = {4};
  TEST_ASSERT_EQUAL_UINT32(0, vesc_uart_frame(out, sizeof out, pl, 0));
  TEST_ASSERT_EQUAL_UINT32(0, vesc_uart_frame(out, sizeof out, pl, 513));
  TEST_ASSERT_EQUAL_UINT32(0, vesc_uart_frame(out, 5, pl, 1));  // needs 6
  TEST_ASSERT_EQUAL_UINT32(6, vesc_uart_frame(out, 6, pl, 1));
  TEST_ASSERT_EQUAL_UINT32(518, vesc_uart_frame(out, sizeof out, pl, 512));  // largest frame: long form, 512 + 6
}

static void test_short_frame_round_trip() {
  uint8_t f[128];
  const uint16_t n = build_reply_frame(f, sizeof f);
  TEST_ASSERT_EQUAL_UINT16(79, n);  // 74-byte payload + 5
  TEST_ASSERT_EQUAL_HEX8(0x02, f[0]);
  TEST_ASSERT_EQUAL_UINT8(74, f[1]);
  TEST_ASSERT_EQUAL_HEX8(0x03, f[78]);
  vesc_uart_rx_reset(&g_rx);
  vesc_uart_rx_push(&g_rx, f, n);
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(74, len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(f + 2, p, 74);
  TEST_ASSERT_FALSE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(0, g_rx.n);
}

static void test_long_frame_round_trip() {
  uint8_t pl[300], f[320];
  for (unsigned i = 0; i < sizeof pl; ++i) pl[i] = (uint8_t)(i * 7u + 3u);  // contains plenty of 2s and 3s
  const size_t n = vesc_uart_frame(f, sizeof f, pl, sizeof pl);
  TEST_ASSERT_EQUAL_UINT32(306, n);
  TEST_ASSERT_EQUAL_HEX8(0x03, f[0]);
  TEST_ASSERT_EQUAL_HEX8(0x01, f[1]);
  TEST_ASSERT_EQUAL_HEX8(0x2C, f[2]);
  vesc_uart_rx_reset(&g_rx);
  vesc_uart_rx_push(&g_rx, f, n);
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(300, len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(pl, p, 300);
}

// ---------------------------------------------------------------- reassembly
static void test_parser_byte_at_a_time() {
  uint8_t f[128];
  const uint16_t n = build_reply_frame(f, sizeof f);
  vesc_uart_rx_reset(&g_rx);
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  for (uint16_t i = 0; i < n; ++i) {
    TEST_ASSERT_FALSE(vesc_uart_rx_next(&g_rx, false, &p, &len));
    vesc_uart_rx_push(&g_rx, f + i, 1);
  }
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(74, len);
}

static void test_parser_chunks_and_back_to_back() {
  uint8_t f[256];
  const uint16_t a = build_reply_frame(f, sizeof f);
  uint8_t req[6];
  vesc_uart_build_getvalues_request(req);
  memcpy(f + a, req, 6);  // a second, different packet right behind the first
  const uint16_t n = (uint16_t)(a + 6);
  vesc_uart_rx_reset(&g_rx);
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  vesc_uart_rx_push(&g_rx, f, 40);
  TEST_ASSERT_FALSE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  vesc_uart_rx_push(&g_rx, f + 40, (size_t)(n - 40));
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(74, len);
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(1, len);
  TEST_ASSERT_EQUAL_HEX8(VESC_COMM_GET_VALUES, p[0]);
  TEST_ASSERT_FALSE(vesc_uart_rx_next(&g_rx, false, &p, &len));
}

static void test_parser_skips_leading_junk() {
  uint8_t f[128];
  const uint8_t junk[] = {0x00, 0xFF, 0x55, 0x10};
  vesc_uart_rx_reset(&g_rx);
  const uint32_t junk0 = g_rx.junk;
  vesc_uart_rx_push(&g_rx, junk, sizeof junk);
  vesc_uart_rx_push(&g_rx, f, build_reply_frame(f, sizeof f));
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(74, len);
  TEST_ASSERT_EQUAL_UINT32(4, g_rx.junk - junk0);
}

static void test_parser_bad_crc_then_resync() {
  uint8_t f[256];
  const uint16_t a = build_reply_frame(f, sizeof f);
  f[a - 2] ^= 0x01;  // CRC low byte
  const uint16_t b = build_reply_frame(f + a, sizeof f - a);
  vesc_uart_rx_reset(&g_rx);
  const uint32_t bad0 = g_rx.bad;
  vesc_uart_rx_push(&g_rx, f, (size_t)(a + b));
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  // flush: the line is idle after the burst. Bytes of the damaged frame can form a bogus
  // header that would otherwise wait for more data (see the idle-flush test).
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, true, &p, &len));  // the second, intact copy
  TEST_ASSERT_EQUAL_UINT16(74, len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(f + a + 2, p, 74);
  TEST_ASSERT_TRUE(g_rx.bad - bad0 >= 1);
  TEST_ASSERT_FALSE(vesc_uart_rx_next(&g_rx, false, &p, &len));
}

static void test_parser_bad_stop_then_resync() {
  uint8_t f[256];
  const uint16_t a = build_reply_frame(f, sizeof f);
  f[a - 1] = 0x00;  // stop byte
  const uint16_t b = build_reply_frame(f + a, sizeof f - a);
  vesc_uart_rx_reset(&g_rx);
  const uint32_t bad0 = g_rx.bad;
  vesc_uart_rx_push(&g_rx, f, (size_t)(a + b));
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, true, &p, &len));  // idle after the burst, as above
  TEST_ASSERT_EQUAL_UINT16(74, len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(f + a + 2, p, 74);
  TEST_ASSERT_TRUE(g_rx.bad - bad0 >= 1);
}

static void test_parser_rejects_bad_lengths() {
  // [2, 0] (empty), a long form announcing <= 255 bytes, a long form above 512: each start
  // byte is dropped and the request right behind them is still found.
  const uint8_t bogus[] = {0x02, 0x00, 0x03, 0x00, 0x10, 0x03, 0x02, 0x01};
  uint8_t req[6];
  vesc_uart_build_getvalues_request(req);
  vesc_uart_rx_reset(&g_rx);
  vesc_uart_rx_push(&g_rx, bogus, sizeof bogus);
  vesc_uart_rx_push(&g_rx, req, sizeof req);
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(1, len);
  TEST_ASSERT_EQUAL_HEX8(VESC_COMM_GET_VALUES, p[0]);
}

static void test_parser_idle_flush_resyncs_past_a_bogus_header() {
  // A stray stop byte (3) followed by two bytes that read as a plausible long length (300)
  // makes the parser wait for 306 bytes. The real frame behind it is held, not lost, and
  // the idle flush rescans past the bogus header to find it.
  const uint8_t bogus[] = {0x03, 0x01, 0x2C};
  uint8_t req[6];
  vesc_uart_build_getvalues_request(req);
  vesc_uart_rx_reset(&g_rx);
  vesc_uart_rx_push(&g_rx, bogus, sizeof bogus);
  vesc_uart_rx_push(&g_rx, req, sizeof req);
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_FALSE(vesc_uart_rx_next(&g_rx, false, &p, &len));  // still waiting for the bogus frame
  TEST_ASSERT_EQUAL_UINT16(9, g_rx.n);
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, true, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(1, len);
  TEST_ASSERT_EQUAL_HEX8(VESC_COMM_GET_VALUES, p[0]);
}

static void test_parser_flush_discards_partial_frame() {
  uint8_t f[128];
  const uint16_t n = build_reply_frame(f, sizeof f);
  vesc_uart_rx_reset(&g_rx);
  const uint32_t junk0 = g_rx.junk;
  vesc_uart_rx_push(&g_rx, f, 30);  // the VESC went away mid-reply
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_FALSE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(30, g_rx.n);
  TEST_ASSERT_FALSE(vesc_uart_rx_next(&g_rx, true, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(0, g_rx.n);
  TEST_ASSERT_EQUAL_UINT32(30, g_rx.junk - junk0);
  vesc_uart_rx_push(&g_rx, f, n);  // the next reply is unaffected
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(74, len);
}

static void test_parser_overflow_drops_oldest() {
  static uint8_t noise[2000];
  memset(noise, 0x55, sizeof noise);
  vesc_uart_rx_reset(&g_rx);
  const uint32_t junk0 = g_rx.junk;
  vesc_uart_rx_push(&g_rx, noise, 300);
  vesc_uart_rx_push(&g_rx, noise, 300);  // 600 > 518: the oldest 82 go
  TEST_ASSERT_EQUAL_UINT16(sizeof g_rx.buf, g_rx.n);
  TEST_ASSERT_EQUAL_UINT32(600 - sizeof g_rx.buf, g_rx.junk - junk0);
  vesc_uart_rx_push(&g_rx, noise, sizeof noise);  // larger than the whole buffer
  TEST_ASSERT_EQUAL_UINT16(sizeof g_rx.buf, g_rx.n);
  TEST_ASSERT_EQUAL_UINT32(600 + sizeof noise - sizeof g_rx.buf, g_rx.junk - junk0);
  uint8_t f[128];
  vesc_uart_rx_push(&g_rx, f, build_reply_frame(f, sizeof f));
  const uint8_t *p = nullptr;
  uint16_t len = 0;
  TEST_ASSERT_TRUE(vesc_uart_rx_next(&g_rx, false, &p, &len));
  TEST_ASSERT_EQUAL_UINT16(74, len);
}

// ---------------------------------------------------------------- state update
static void test_decoded_mask() {
  TEST_ASSERT_EQUAL_HEX32(VESC_GV_MASK_ALL, vesc_getvalues_decoded_mask(VESC_GV_MASK_ALL, 22));
  TEST_ASSERT_EQUAL_HEX32(VESC_GV_MASK_ALL & ~(1u << VESC_GV_STATUS), vesc_getvalues_decoded_mask(VESC_GV_MASK_ALL, 21));
  TEST_ASSERT_EQUAL_HEX32(0x0000001Cu, vesc_getvalues_decoded_mask(0x003EC03Cu, 3));
  TEST_ASSERT_EQUAL_HEX32(0, vesc_getvalues_decoded_mask(VESC_GV_MASK_ALL, 0));
}

static void test_apply_full_reply() {
  VescState v;
  VescExt x;
  fresh_state(v, x);
  uint8_t pl[128];
  const uint16_t n = build_reply(pl, 22);
  TEST_ASSERT_EQUAL_UINT16(74, n);
  TEST_ASSERT_EQUAL_INT(22, vesc_uart_apply_getvalues(&v, &x, pl, n, 1000, 0.5f));

  const vesc_telemetry_t &t = v.t;
  TEST_ASSERT_EQUAL_FLOAT(41.5f, t.temp_fet);
  TEST_ASSERT_EQUAL_FLOAT(33.2f, t.temp_motor);
  TEST_ASSERT_EQUAL_FLOAT(52.34f, t.current_motor);  // the average stands in for STATUS_1's current
  TEST_ASSERT_EQUAL_FLOAT(21.07f, t.current_in);     // ... and for STATUS_4's
  TEST_ASSERT_EQUAL_FLOAT(0.456f, t.duty);
  TEST_ASSERT_EQUAL_FLOAT(12345.0f, t.erpm);
  TEST_ASSERT_EQUAL_FLOAT(50.3f, t.v_in);
  TEST_ASSERT_EQUAL_FLOAT(1.2345f, t.amp_hours);
  TEST_ASSERT_EQUAL_FLOAT(0.0123f, t.amp_hours_charged);
  TEST_ASSERT_EQUAL_FLOAT(61.2345f, t.watt_hours);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, t.watt_hours_charged);
  TEST_ASSERT_EQUAL_INT32(98765, t.tachometer);
  TEST_ASSERT_EQUAL_FLOAT(12.5f, t.pid_pos);
  TEST_ASSERT_EQUAL_UINT8(kVescId, t.id);
  for (int i = VESC_IDX_STATUS_1; i <= VESC_IDX_STATUS_5; ++i) TEST_ASSERT_EQUAL_UINT32(1000, t.t_ms[i]);
  TEST_ASSERT_EQUAL_UINT32(0, t.t_ms[VESC_IDX_STATUS_6]);  // never: no UART equivalent

  TEST_ASSERT_EQUAL_FLOAT(41.0f, x.temp_mos1);
  TEST_ASSERT_EQUAL_FLOAT(42.5f, x.temp_mos2);
  TEST_ASSERT_EQUAL_FLOAT(40.5f, x.temp_mos3);
  TEST_ASSERT_EQUAL_FLOAT(-0.25f, x.avg_id);
  TEST_ASSERT_EQUAL_FLOAT(52.1f, x.avg_iq);
  TEST_ASSERT_EQUAL_FLOAT(1.234f, x.vd);
  TEST_ASSERT_EQUAL_FLOAT(23.456f, x.vq);
  TEST_ASSERT_EQUAL_INT32(123456, x.tacho_abs);
  TEST_ASSERT_EQUAL_UINT8(5, x.fault_code);
  TEST_ASSERT_EQUAL_HEX8(VESC_STATUS_KILL_SW, x.status);
  TEST_ASSERT_EQUAL_UINT8(kVescId, x.vesc_id);
  TEST_ASSERT_EQUAL_UINT32(1000, x.t_ms);
  TEST_ASSERT_EQUAL_UINT32(1, x.replies_ok);
  TEST_ASSERT_EQUAL_UINT8(5, x.last_fault);
  TEST_ASSERT_EQUAL_UINT32(1000, x.last_fault_ms);

  TEST_ASSERT_EQUAL_INT(kVescId, v.locked_id);
  TEST_ASSERT_EQUAL_UINT32(1, v.frames_total);
  TEST_ASSERT_EQUAL_UINT32(1000, v.last_frame_ms);
  TEST_ASSERT_TRUE(v.ema_init);
  TEST_ASSERT_EQUAL_FLOAT(52.34f, v.i_motor_ema);  // seeded, not blended with the zeroed start value
  TEST_ASSERT_EQUAL_FLOAT(21.07f, v.i_in_ema);
  TEST_ASSERT_EQUAL_FLOAT(50.3f, v.v_in_ema);
}

static void test_apply_now_zero_is_stamped_as_one() {
  VescState v;
  VescExt x;
  fresh_state(v, x);
  uint8_t pl[128];
  const uint16_t n = build_reply(pl, 22);
  vesc_uart_apply_getvalues(&v, &x, pl, n, 0, 0.5f);
  TEST_ASSERT_EQUAL_UINT32(1, v.t.t_ms[VESC_IDX_STATUS_1]);
  TEST_ASSERT_EQUAL_UINT32(1, x.t_ms);
  TEST_ASSERT_EQUAL_UINT32(1, v.last_frame_ms);
}

static void test_apply_without_status_byte() {
  VescState v;
  VescExt x;
  fresh_state(v, x);
  x.status = 0x01;  // a previous value the short reply must not overwrite
  uint8_t pl[128];
  const uint16_t n = build_reply(pl, 21);
  TEST_ASSERT_EQUAL_UINT16(73, n);
  TEST_ASSERT_EQUAL_INT(21, vesc_uart_apply_getvalues(&v, &x, pl, n, 500, 0.5f));
  TEST_ASSERT_EQUAL_INT(22, vesc_getvalues_field_count(vesc_getvalues_mask(pl, n)));
  TEST_ASSERT_EQUAL_HEX8(0x01, x.status);
  for (int i = VESC_IDX_STATUS_1; i <= VESC_IDX_STATUS_5; ++i) TEST_ASSERT_EQUAL_UINT32(500, v.t.t_ms[i]);
}

static void test_apply_truncated_mid_telemetry_stamps_complete_groups_only() {
  VescState v;
  VescExt x;
  fresh_state(v, x);
  v.i_in_ema = 7.0f;
  v.v_in_ema = 8.0f;
  uint8_t pl[128];
  const uint16_t n = build_reply(pl, 10);  // bits 0..9: ends after amp_hours
  TEST_ASSERT_EQUAL_INT(10, vesc_uart_apply_getvalues(&v, &x, pl, n, 700, 0.5f));
  TEST_ASSERT_EQUAL_UINT32(700, v.t.t_ms[VESC_IDX_STATUS_1]);  // erpm, avg motor current, duty: all there
  TEST_ASSERT_EQUAL_UINT32(0, v.t.t_ms[VESC_IDX_STATUS_2]);    // amp_hours_charged missing
  TEST_ASSERT_EQUAL_UINT32(0, v.t.t_ms[VESC_IDX_STATUS_3]);
  TEST_ASSERT_EQUAL_UINT32(0, v.t.t_ms[VESC_IDX_STATUS_4]);    // pid_pos missing
  TEST_ASSERT_EQUAL_UINT32(0, v.t.t_ms[VESC_IDX_STATUS_5]);    // tachometer missing
  TEST_ASSERT_EQUAL_FLOAT(50.3f, v.t.v_in);                    // decoded, but its slot stays unstamped
  TEST_ASSERT_EQUAL_FLOAT(52.34f, v.i_motor_ema);
  TEST_ASSERT_EQUAL_FLOAT(7.0f, v.i_in_ema);  // STATUS_4 / STATUS_5 not stamped: their EMAs do not move
  TEST_ASSERT_EQUAL_FLOAT(8.0f, v.v_in_ema);
  TEST_ASSERT_EQUAL_INT(-1, v.locked_id);     // no controller id field
  TEST_ASSERT_EQUAL_UINT32(1, x.replies_ok);
}

static void test_apply_unsolicited_and_bad_layout_leave_state_alone() {
  VescState v, v0;
  VescExt x, x0;
  fresh_state(v, x);
  v.frames_total = 3;
  x.replies_ok = 3;
  v0 = v;
  x0 = x;
  const uint8_t print[] = {21, 'h', 'i'};  // COMM_PRINT
  TEST_ASSERT_EQUAL_INT(VESC_UART_NOT_GETVALUES, vesc_uart_apply_getvalues(&v, &x, print, sizeof print, 900, 0.5f));
  const uint8_t unknown_bit[] = {VESC_COMM_GET_VALUES_SELECTIVE, 0x00, 0x80, 0x00, 0x00};  // mask bit 23
  TEST_ASSERT_EQUAL_INT(VESC_UART_BAD_LAYOUT,
                        vesc_uart_apply_getvalues(&v, &x, unknown_bit, sizeof unknown_bit, 900, 0.5f));
  TEST_ASSERT_EQUAL_MEMORY(&v0, &v, sizeof v);
  TEST_ASSERT_EQUAL_MEMORY(&x0, &x, sizeof x);
}

static void test_apply_ema_blends_and_reseeds_after_gap() {
  VescState v;
  VescExt x;
  fresh_state(v, x);
  uint8_t pl[128];
  Reply r;
  uint16_t n = build_reply(pl, 22, r);
  vesc_uart_apply_getvalues(&v, &x, pl, n, 1000, 0.5f);
  TEST_ASSERT_EQUAL_FLOAT(50.3f, v.v_in_ema);
  r.v_in_x10 = 400;
  n = build_reply(pl, 22, r);
  vesc_uart_apply_getvalues(&v, &x, pl, n, 1100, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 45.15f, v.v_in_ema);  // 0.5 * 40.0 + 0.5 * 50.3
  r.v_in_x10 = 300;
  n = build_reply(pl, 22, r);
  vesc_uart_apply_getvalues(&v, &x, pl, n, 1100 + VESC_STALE_R1_MS + 1, 0.5f);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, v.v_in_ema);  // a reconnect after a gap seeds afresh
}

static void test_apply_fault_latch() {
  VescState v;
  VescExt x;
  fresh_state(v, x);
  uint8_t pl[128];
  Reply r;
  uint16_t n = build_reply(pl, 22, r);
  vesc_uart_apply_getvalues(&v, &x, pl, n, 1000, 0.5f);
  r.fault = 0;  // the VESC cleared its live byte
  n = build_reply(pl, 22, r);
  vesc_uart_apply_getvalues(&v, &x, pl, n, 1100, 0.5f);
  TEST_ASSERT_EQUAL_UINT8(0, x.fault_code);
  TEST_ASSERT_EQUAL_UINT8(5, x.last_fault);
  TEST_ASSERT_EQUAL_UINT32(1000, x.last_fault_ms);
  TEST_ASSERT_EQUAL_UINT32(2, x.replies_ok);
  TEST_ASSERT_EQUAL_UINT32(2, v.frames_total);
}

// ---------------------------------------------------------------- display labels
static void test_main_link_labels() {
  SharedState s;
  memset(&s, 0, sizeof s);
  s.vesc.locked_id = -1;
  OledMain m;
  s.can.state = CAN_STATE_UNINSTALLED;
  oled_build_main(s, 10000, m);
  TEST_ASSERT_EQUAL_STRING("UART OFF", m.can);
  s.can.state = CAN_STATE_RUNNING;
  oled_build_main(s, 10000, m);
  TEST_ASSERT_EQUAL_STRING("NO VESC", m.can);
  uint8_t pl[128];
  const uint16_t n = build_reply(pl, 22);
  vesc_uart_apply_getvalues(&s.vesc, &s.vesc_ext, pl, n, 9900, 0.5f);
  oled_build_main(s, 10000, m);
  TEST_ASSERT_EQUAL_STRING("VESC 74", m.can);
  oled_build_main(s, 9900 + VESC_STALE_R1_MS + 1, m);
  TEST_ASSERT_EQUAL_STRING("NO VESC", m.can);
}

static void test_sys_link_row() {
  SharedState s;
  memset(&s, 0, sizeof s);
  s.can.state = CAN_STATE_RUNNING;
  const OledSysInfo info = {60, 250 * 1024, 240 * 1024, "POWERON"};
  OledGrid g;
  oled_build_sys(s, 1000, info, g);
  TEST_ASSERT_EQUAL_STRING("UART RUN   115200", g.rows[2]);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_request_bytes);
  RUN_TEST(test_frame_rejects_bad_sizes);
  RUN_TEST(test_short_frame_round_trip);
  RUN_TEST(test_long_frame_round_trip);
  RUN_TEST(test_parser_byte_at_a_time);
  RUN_TEST(test_parser_chunks_and_back_to_back);
  RUN_TEST(test_parser_skips_leading_junk);
  RUN_TEST(test_parser_bad_crc_then_resync);
  RUN_TEST(test_parser_bad_stop_then_resync);
  RUN_TEST(test_parser_rejects_bad_lengths);
  RUN_TEST(test_parser_idle_flush_resyncs_past_a_bogus_header);
  RUN_TEST(test_parser_flush_discards_partial_frame);
  RUN_TEST(test_parser_overflow_drops_oldest);
  RUN_TEST(test_decoded_mask);
  RUN_TEST(test_apply_full_reply);
  RUN_TEST(test_apply_now_zero_is_stamped_as_one);
  RUN_TEST(test_apply_without_status_byte);
  RUN_TEST(test_apply_truncated_mid_telemetry_stamps_complete_groups_only);
  RUN_TEST(test_apply_unsolicited_and_bad_layout_leave_state_alone);
  RUN_TEST(test_apply_ema_blends_and_reseeds_after_gap);
  RUN_TEST(test_apply_fault_latch);
  RUN_TEST(test_main_link_labels);
  RUN_TEST(test_sys_link_row);
  return UNITY_END();
}
