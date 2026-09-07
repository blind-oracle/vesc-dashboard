// CAN / VESC listener: legacy TWAI driver, passive reception of the VESC
// STATUS_1..6 broadcast frames, optional active polling of the values no
// broadcast carries (COMM_GET_VALUES_SELECTIVE every VESC_POLL_MS ms), bus-off
// recovery and health reporting.
//
// Protocol traps this file is built around (see the plan / README for detail):
//
//  * Legacy driver/twai.h ONLY. The new IDF 5.5 "onchip" TWAI node driver must
//    never be mixed in: IDF's driver-conflict check is compiled out of the
//    precompiled Arduino libraries, so mixing the two fails silently at runtime.
//
//  * TWAI_MODE_NORMAL: the controller asserts the ACK bit. With VESC_POLL_MS == 0
//    the TX queue is disabled and no transmit call exists in the binary (strictly
//    passive node); with VESC_POLL_MS > 0 the ONLY frames this node ever transmits
//    are the single-shot COMM_GET_VALUES_SELECTIVE requests built in poll_tick().
//    The ACK matters either way: the VESC's bxCAN has no automatic-retransmission
//    limit, so with a non-ACKing listener as the only other node it goes
//    error-passive and repeats ONE stale STATUS frame forever ("lonely VESC").
//    CAN_LISTEN_ONLY=1 (true listen-only, no ACK, no polling) is only safe when
//    another ACKing node exists on the bus.
//
//  * Polling is the mirror image of that trap: the request is sent single-shot
//    (twai_message_t.ss) so a powered-down VESC never turns this node into a
//    retransmitting "lonely display". Replies are addressed to CAN_OWN_ID (low EID
//    byte), fragmented into FILL_RX_BUFFER frames and closed by PROCESS_RX_BUFFER
//    with length + CRC-16 (vesc_getvalues.h). The VESC clears its fault byte after
//    ~500 ms, so the last non-zero fault code is latched here (can_vesc_last_fault).
//
//  * After a bus-off recovery the legacy driver parks in STOPPED, not RUNNING:
//    twai_start() has to be called again on TWAI_ALERT_BUS_RECOVERED.
//
//  * STATUS_5 wire order is tachometer THEN v_in (handled in vesc_status.h).
//
// Concurrency: the state mutex is held only for field copies; decoding and all
// logging happen outside it. canTask is the sole writer of g_state.vesc,
// g_state.vesc_ext and the TWAI part of g_state.can.
#include "can_vesc.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/twai.h>
#include <esp_err.h>

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "shared_state.h"
#include "vesc_getvalues.h"
#include "vesc_status.h"

// ---------------------------------------------------------------------------
// Task tunables: config.h ("Advanced task tunables") defines all of these; the
// #ifndef fallbacks below only keep this file compiling against an older config.h.
// ---------------------------------------------------------------------------
#ifndef CAN_TASK_STACK
#define CAN_TASK_STACK 4096         // bytes: ESP-IDF's xTaskCreate() takes the stack size in bytes
#endif
#ifndef CAN_INSTALL_RETRY_MS
#define CAN_INSTALL_RETRY_MS 5000   // retry twai_driver_install()/twai_start() while they keep failing
#endif
#ifndef CAN_RX_TIMEOUT_MS
#define CAN_RX_TIMEOUT_MS 100       // twai_receive() block time; also bounds the heartbeat and poll-tick period
#endif
#ifndef CAN_ERR_LOG_MIN_MS
#define CAN_ERR_LOG_MIN_MS 2000     // rate limit for the noisy alerts (BUS_ERROR, FIFO overrun, RX queue full, warn level)
#endif
#ifndef CAN_ERR_WARN_LEVEL
#define CAN_ERR_WARN_LEVEL 96       // ISO 11898-1 error-warning limit for TEC/REC
#endif
#ifndef CAN_TX_WAIT_MS
#define CAN_TX_WAIT_MS 10           // twai_transmit() block time for a poll request (the TX queue is normally empty)
#endif
#ifndef VESC_POLL_LOG_MIN_MS
#define VESC_POLL_LOG_MIN_MS 10000  // rate limit for "no reply" / "bad reply" / "not transmitted" warnings (counts aggregated)
#endif
#ifndef VESC_POLL_BACKOFF_AFTER
#define VESC_POLL_BACKOFF_AFTER 3   // consecutive polls without a reply before slowing down to VESC_POLL_BACKOFF_MS
#endif
#ifndef VESC_POLL_BACKOFF_MS
#define VESC_POLL_BACKOFF_MS 5000   // poll period while the VESC does not answer (off / wrong id); a STATUS frame resumes at once
#endif

#if VESC_POLL_MS > 0
#if CAN_LISTEN_ONLY
#error "VESC_POLL_MS > 0 requires CAN_LISTEN_ONLY 0: twai_transmit() is not supported in listen-only mode (set VESC_POLL_MS 0 for a passive node)"
#endif
static_assert(CAN_TX_QUEUE_LEN >= 1, "CAN_TX_QUEUE_LEN must be >= 1 while VESC_POLL_MS > 0");
static_assert(VESC_POLL_TIMEOUT_MS > 0, "VESC_POLL_TIMEOUT_MS must be > 0");
static_assert((VESC_GETVALUES_MASK & ~VESC_GV_MASK_ALL) == 0, "VESC_GETVALUES_MASK has bits above 21 (unknown to the decoder)");
// Effective period once the VESC stopped answering: never faster than the normal one.
#define VESC_POLL_SLOW_MS ((VESC_POLL_BACKOFF_MS) > (VESC_POLL_MS) ? (VESC_POLL_BACKOFF_MS) : (VESC_POLL_MS))
#endif

// shared_state.h mirrors twai_state_t so it can stay IDF-free; make sure the mirror never drifts.
static_assert((int)TWAI_STATE_STOPPED == CAN_STATE_STOPPED, "CanState mirror out of sync with twai_state_t");
static_assert((int)TWAI_STATE_RUNNING == CAN_STATE_RUNNING, "CanState mirror out of sync with twai_state_t");
static_assert((int)TWAI_STATE_BUS_OFF == CAN_STATE_BUS_OFF, "CanState mirror out of sync with twai_state_t");
static_assert((int)TWAI_STATE_RECOVERING == CAN_STATE_RECOVERING, "CanState mirror out of sync with twai_state_t");

// ---------------------------------------------------------------------------
// Module state (only touched from canTask, plus the one-shot can_vesc_start())
// ---------------------------------------------------------------------------
static bool s_task_started = false;
static bool s_installed = false;          // twai_driver_install() succeeded (driver may still be STOPPED)
static uint32_t s_last_install_try_ms = 0;
static uint32_t s_last_health_ms = 0;
static bool s_invalid_state_logged = false;
// Rate limiting for alerts that can fire thousands of times per second on a faulty bus.
static uint32_t s_bus_err_pending = 0, s_bus_err_log_ms = 0;
static uint32_t s_qfull_pending = 0, s_qfull_log_ms = 0;
static uint32_t s_overrun_pending = 0, s_overrun_log_ms = 0;
static uint32_t s_warn_log_ms = 0;

#if VESC_POLL_MS > 0
// Active polling: one request in flight at a time, one reply being reassembled.
static VescRxBuffer s_rx;                         // FILL_RX_BUFFER reassembly (canTask only)
static bool s_poll_outstanding = false;           // request queued, complete reply not yet seen
static uint32_t s_poll_sent_ms = 0;               // when the outstanding request was queued
static uint32_t s_last_poll_ms = 0;               // 0 = poll at the next tick
static uint32_t s_consec_fail = 0;                // polls in a row without a complete reply (drives the back-off)
static bool s_poll_suspended = false;             // a VESC broadcasts under CAN_OWN_ID: stay passive (handle_frame)
static uint32_t s_tx_err_pending = 0, s_tx_err_log_ms = 0;    // twai_transmit() returned an error
static uint32_t s_tx_fail_pending = 0, s_tx_fail_log_ms = 0;  // TWAI_ALERT_TX_FAILED: single shot not acknowledged
static uint32_t s_to_pending = 0, s_to_log_ms = 0;            // reply timeouts
static uint32_t s_bad_pending = 0, s_bad_log_ms = 0;          // bad replies (length / CRC / layout)
static bool s_partial_logged = false;             // "firmware answers fewer fields than requested" warned once
static uint8_t s_prev_fault = 0;                  // live fault byte of the last STORED reply, for edge logging
// The fault latch itself (last non-zero code + time) lives in g_state.vesc_ext.last_fault / last_fault_ms
// so that every consumer gets it with the snapshot; can_vesc_last_fault() is a convenience reader.
#endif

// millis() with 0 mapped to 1 so that 0 keeps meaning "never" in every timestamp field.
static inline uint32_t now_nonzero() {
  const uint32_t n = millis();
  return n ? n : 1u;
}

static inline bool due(uint32_t now, uint32_t last, uint32_t period) {
  return last == 0 || (uint32_t)(now - last) >= period;
}

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
const char *can_state_str(int can_state) {
  switch (can_state) {
    case CAN_STATE_UNINSTALLED: return "UNINST";
    case CAN_STATE_STOPPED: return "STOP";
    case CAN_STATE_RUNNING: return "RUN";
    case CAN_STATE_BUS_OFF: return "BUSOFF";
    case CAN_STATE_RECOVERING: return "RECOV";
    default: return "?";
  }
}

bool can_vesc_polling_enabled() {
#if VESC_POLL_MS > 0
  return !s_poll_suspended;
#else
  return false;
#endif
}

uint8_t can_vesc_last_fault(uint32_t *t_ms_or_null) {
  uint8_t code = 0;
  uint32_t ms = 0;
#if VESC_POLL_MS > 0
  if (state_lock()) {
    code = g_state.vesc_ext.last_fault;
    ms = g_state.vesc_ext.last_fault_ms;
    state_unlock();
  }
#endif
  if (t_ms_or_null) *t_ms_or_null = ms;
  return code;
}

static int map_state(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED: return CAN_STATE_STOPPED;
    case TWAI_STATE_RUNNING: return CAN_STATE_RUNNING;
    case TWAI_STATE_BUS_OFF: return CAN_STATE_BUS_OFF;
    case TWAI_STATE_RECOVERING: return CAN_STATE_RECOVERING;
    default: return CAN_STATE_UNINSTALLED;
  }
}

static void set_can_state(int st) {
  if (state_lock()) {
    g_state.can.state = st;
    state_unlock();
  }
}

static void count_dropped() {
  if (state_lock()) {
    g_state.vesc.frames_dropped++;
    state_unlock();
  }
}

[[maybe_unused]] static const char *mode_str() {  // log-only helper (unused when CORE_DEBUG_LEVEL < 3)
#if VESC_POLL_MS > 0
  return CAN_LISTEN_ONLY ? "LISTEN_ONLY (no ACK)" : "NORMAL (ACK, transmits only GET_VALUES_SELECTIVE polls)";
#else
  return CAN_LISTEN_ONLY ? "LISTEN_ONLY (no ACK)" : "NORMAL (ACK-only, never transmits)";
#endif
}

// Bit timing for the configured bitrate. On the ESP32-C6 (40 MHz XTAL) the 500k
// and 1M presets sample at 80 %, compatible with the VESC's 78.6 %; 125k/250k
// presets sample at 75 %. The actual sample point is logged at install.
static twai_timing_config_t timing_config() {
#if CAN_BITRATE_KBPS == 125
  twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
#elif CAN_BITRATE_KBPS == 250
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
#elif CAN_BITRATE_KBPS == 500
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
#elif CAN_BITRATE_KBPS == 1000
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
#else
#error "CAN_BITRATE_KBPS must be 125, 250, 500 or 1000"
#endif
  return t;
}

// The legacy HAL programs sync(1) + tseg_1 + tseg_2 and ignores prop_seg (it is
// only used by the new node driver), so the sample point is (1 + tseg_1) / total.
[[maybe_unused]] static int sample_point_percent(const twai_timing_config_t &t) {
  const int quanta = 1 + t.tseg_1 + t.tseg_2;
  return quanta > 0 ? (100 * (1 + t.tseg_1)) / quanta : 0;
}

// Exponential moving average, seeded with the first sample of its source message.
static inline float ema_step(float prev, float x, bool first) {
  return first ? x : (CAN_EMA_ALPHA * x + (1.0f - CAN_EMA_ALPHA) * prev);
}

// ---------------------------------------------------------------------------
// Driver install / start
// ---------------------------------------------------------------------------
// Puts an installed driver into RUNNING (idempotent) and mirrors the state.
static bool can_run() {
  [[maybe_unused]] const esp_err_t err = twai_start();  // ESP_ERR_INVALID_STATE when already running: harmless
  twai_status_info_t st{};
  const bool have_st = twai_get_status_info(&st) == ESP_OK;  // fails only when the driver is not installed
  const int cs = have_st ? map_state(st.state) : CAN_STATE_UNINSTALLED;
  set_can_state(cs);
  if (cs != CAN_STATE_RUNNING) log_e("CAN: twai_start failed: %s (state %s)", esp_err_to_name(err), can_state_str(cs));
  return cs == CAN_STATE_RUNNING;
}

static bool can_install() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX,
                                                        CAN_LISTEN_ONLY ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  g.rx_queue_len = CAN_RX_QUEUE_LEN;  // 6 status frames arrive as a burst every 20 ms; the default 5 is too small
  // TX queue only while polling: with 0 the driver cannot transmit at all and this node
  // only ever asserts the ACK bit. With a queue, twai_transmit() never fails with ESP_FAIL
  // ("another message is transmitting") even if a request coincides with a retry.
  g.tx_queue_len = (VESC_POLL_MS > 0) ? CAN_TX_QUEUE_LEN : 0;
  g.alerts_enabled = TWAI_ALERT_ERR_PASS | TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL |
                     TWAI_ALERT_RX_FIFO_OVERRUN | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                     TWAI_ALERT_ABOVE_ERR_WARN
#if VESC_POLL_MS > 0
                     | TWAI_ALERT_TX_FAILED  // a single-shot poll request that got no ACK (VESC off)
#endif
      ;
  const twai_timing_config_t t = timing_config();
  // Hardware filter wide open: the controller id is filtered in software so we
  // can lock onto whichever VESC talks first (VESC_CAN_ID == -1); the poll replies
  // (low EID byte == CAN_OWN_ID) pass the same way.
  const twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  const esp_err_t err = twai_driver_install(&g, &t, &f);
  if (err == ESP_ERR_INVALID_STATE) {
    log_w("CAN: driver already installed, reusing it");
  } else if (err != ESP_OK) {
    log_e("CAN: twai_driver_install failed: %s (retry in %lu ms)", esp_err_to_name(err),
          (unsigned long)CAN_INSTALL_RETRY_MS);
    set_can_state(CAN_STATE_UNINSTALLED);
    return false;
  }
  s_installed = true;
  s_invalid_state_logged = false;
  if (!can_run()) return false;

#if VESC_POLL_MS > 0
  log_i("CAN: driver installed, state RUNNING: mode=%s bitrate=%dk tx=GPIO%d rx=GPIO%d rx_queue=%d tx_queue=%d "
        "sample_point=%d%% poll=%d ms own_id=%d mask=0x%08lX",
        mode_str(), CAN_BITRATE_KBPS, PIN_CAN_TX, PIN_CAN_RX, CAN_RX_QUEUE_LEN, (int)g.tx_queue_len,
        sample_point_percent(t), VESC_POLL_MS, CAN_OWN_ID, (unsigned long)VESC_GETVALUES_MASK);
#else
  log_i("CAN: driver installed, state RUNNING: mode=%s bitrate=%dk tx=GPIO%d rx=GPIO%d rx_queue=%d tx_queue=%d "
        "sample_point=%d%% poll=off",
        mode_str(), CAN_BITRATE_KBPS, PIN_CAN_TX, PIN_CAN_RX, CAN_RX_QUEUE_LEN, (int)g.tx_queue_len,
        sample_point_percent(t));
#endif
  return true;
}

// ---------------------------------------------------------------------------
// Alerts (bus-off recovery, error state changes, queue overflows)
// ---------------------------------------------------------------------------
static void handle_alerts(uint32_t a, uint32_t now) {
  // 1) Driver actions first: they are time-critical, the logging below is not.
  esp_err_t recovery_err = ESP_OK, start_err = ESP_OK;
  if (a & TWAI_ALERT_BUS_OFF) recovery_err = twai_initiate_recovery();
  if (a & TWAI_ALERT_BUS_RECOVERED) start_err = twai_start();  // legacy driver parks in STOPPED after recovery

  twai_status_info_t st{};
  const bool have_st = twai_get_status_info(&st) == ESP_OK;
  [[maybe_unused]] const unsigned long tec = have_st ? (unsigned long)st.tx_error_counter : 0ul;  // log-only
  [[maybe_unused]] const unsigned long rec = have_st ? (unsigned long)st.rx_error_counter : 0ul;

  // 2) Counters under the lock, no I/O.
  if (state_lock()) {
    CanHealth &c = g_state.can;
    if (a & TWAI_ALERT_BUS_OFF) {
      c.bus_off_count++;
      c.state = recovery_err == ESP_OK ? CAN_STATE_RECOVERING : CAN_STATE_BUS_OFF;
    }
    if (a & TWAI_ALERT_BUS_RECOVERED) {
      c.recoveries++;
      c.state = start_err == ESP_OK ? CAN_STATE_RUNNING : CAN_STATE_STOPPED;
    }
    if (a & TWAI_ALERT_RX_QUEUE_FULL) c.queue_full_events++;
    if (a & TWAI_ALERT_ERR_PASS) c.err_passive_events++;
    if (a & TWAI_ALERT_BUS_ERROR) c.bus_error_events++;
    if (have_st) {
      c.tec = st.tx_error_counter;
      c.rec = st.rx_error_counter;
      c.bus_error_count = st.bus_error_count;
      c.rx_missed = st.rx_missed_count;
      c.rx_overrun = st.rx_overrun_count;
    }
    state_unlock();
  }

  // 3) Logs, rate-limited where a faulty bus could flood the console.
  if (a & TWAI_ALERT_BUS_OFF)
    log_e("CAN: BUS_OFF (TEC=%lu REC=%lu), recovery %s", tec, rec,
          recovery_err == ESP_OK ? "started" : esp_err_to_name(recovery_err));
  if (a & TWAI_ALERT_BUS_RECOVERED)
    log_i("CAN: bus recovered, twai_start -> %s", start_err == ESP_OK ? "RUNNING" : esp_err_to_name(start_err));
  if (a & TWAI_ALERT_ERR_PASS) log_w("CAN: error passive (TEC=%lu REC=%lu)", tec, rec);
  if (a & TWAI_ALERT_ERR_ACTIVE) log_i("CAN: error active (TEC=%lu REC=%lu)", tec, rec);

  if (a & TWAI_ALERT_ABOVE_ERR_WARN) {
    if (due(now, s_warn_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_warn_log_ms = now ? now : 1u;
      log_w("CAN: error counter above warning level (TEC=%lu REC=%lu)", tec, rec);
    }
  }
  if (a & TWAI_ALERT_BUS_ERROR) {
    s_bus_err_pending++;
    if (due(now, s_bus_err_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_bus_err_log_ms = now ? now : 1u;
      log_w("CAN: %lu bus error alert(s) (driver bus_error_count=%lu TEC=%lu REC=%lu)", (unsigned long)s_bus_err_pending,
            (unsigned long)st.bus_error_count, tec, rec);
      s_bus_err_pending = 0;
    }
  }
  if (a & TWAI_ALERT_RX_QUEUE_FULL) {
    s_qfull_pending++;
    if (due(now, s_qfull_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_qfull_log_ms = now ? now : 1u;
      log_w("CAN: RX queue full x%lu, frames lost (driver rx_missed=%lu queued=%lu)", (unsigned long)s_qfull_pending,
            (unsigned long)st.rx_missed_count, (unsigned long)st.msgs_to_rx);
      s_qfull_pending = 0;
    }
  }
  if (a & TWAI_ALERT_RX_FIFO_OVERRUN) {
    s_overrun_pending++;
    if (due(now, s_overrun_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_overrun_log_ms = now ? now : 1u;
      log_w("CAN: RX FIFO overrun x%lu (driver rx_overrun=%lu)", (unsigned long)s_overrun_pending,
            (unsigned long)st.rx_overrun_count);
      s_overrun_pending = 0;
    }
  }
#if VESC_POLL_MS > 0
  if (a & TWAI_ALERT_TX_FAILED) {
    // Single-shot request not acknowledged: nobody listens on the bus (VESC off, cable,
    // bitrate). The reply timeout below counts it; this only explains the silence.
    s_tx_fail_pending++;
    if (due(now, s_tx_fail_log_ms, VESC_POLL_LOG_MIN_MS)) {
      s_tx_fail_log_ms = now ? now : 1u;
      log_w("VESC poll: %lu request(s) not acknowledged on the bus (VESC off? TEC=%lu)", (unsigned long)s_tx_fail_pending,
            tec);
      s_tx_fail_pending = 0;
    }
  }
#endif
}

// ---------------------------------------------------------------------------
// Active polling: request, reply reassembly, decode (VESC_POLL_MS > 0 only)
// ---------------------------------------------------------------------------
#if VESC_POLL_MS > 0
// A complete-looking reply turned out unusable (length/CRC mismatch, or a GET_VALUES
// payload whose layout the decoder rejects): count it, warn rate-limited and - only
// while a request is actually outstanding - end that request and count the failure
// towards the back-off. Without a request in flight the frames were unsolicited (a
// COMM_PRINT whose fragment got lost, a late reply after the timeout that already
// counted): they must not push the poller into its slow period. Unsolicited packets
// with other packet ids never get here.
static void reply_bad(uint32_t now, [[maybe_unused]] const char *why, [[maybe_unused]] unsigned len) {
  if (s_poll_outstanding) {
    s_poll_outstanding = false;
    if (s_consec_fail != 0xFFFFFFFFu) s_consec_fail++;
  }
  if (state_lock()) {
    g_state.vesc_ext.replies_bad++;
    state_unlock();
  }
  s_bad_pending++;
  if (due(now, s_bad_log_ms, VESC_POLL_LOG_MIN_MS)) {
    s_bad_log_ms = now ? now : 1u;
    log_w("VESC poll: %lu bad reply/replies, last: %s (len %u, expected %u for mask 0x%08lX)", (unsigned long)s_bad_pending,
          why, len, (unsigned)vesc_getvalues_expected_len(VESC_GETVALUES_MASK), (unsigned long)VESC_GETVALUES_MASK);
    s_bad_pending = 0;
  }
}

// Frames addressed to OUR id with packet id 5..8 (handle_frame already checked both).
static void handle_reply_frame(const twai_message_t &m, uint32_t pkt, uint32_t now) {
#if CAN_LOG_RAW_FRAMES
  log_d("CAN rx reply eid=0x%08lX pkt=%lu dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]", (unsigned long)m.identifier,
        (unsigned long)pkt, (unsigned)m.data_length_code, m.data[0], m.data[1], m.data[2], m.data[3], m.data[4],
        m.data[5], m.data[6], m.data[7]);
#endif
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  switch (pkt) {
    case VESC_CAN_PACKET_FILL_RX_BUFFER:
    case VESC_CAN_PACKET_FILL_RX_BUFFER_LONG:
      if (!vesc_rx_fill(&s_rx, m.data, m.data_length_code, pkt == VESC_CAN_PACKET_FILL_RX_BUFFER_LONG)) {
        // A gap (lost frame) or a malformed fragment: the buffer was dropped, the closing
        // PROCESS_RX_BUFFER will fail its length check and count the reply as bad.
        // (A duplicate of a fragment already received is ignored inside vesc_rx_fill.)
        log_d("VESC poll: reply fragment out of sequence (pkt %lu dlc %u offset %u)", (unsigned long)pkt,
              (unsigned)m.data_length_code, (unsigned)m.data[0]);
      }
      return;
    case VESC_CAN_PACKET_PROCESS_RX_BUFFER: {
      const int r = vesc_rx_process(&s_rx, m.data, m.data_length_code, &payload, &len);
      if (r < 0) return;  // send flag 0 / short frame: a command forwarded TO our id, not a reply
      if (r == 0) {
        reply_bad(now, "length or CRC mismatch", ((unsigned)m.data[2] << 8) | m.data[3]);
        return;
      }
      break;
    }
    case VESC_CAN_PACKET_PROCESS_SHORT_BUFFER:  // reply payload <= 6 bytes: [vesc_id, send=1, data...]
      if (m.data_length_code < 3 || m.data[1] != 1) return;  // send != 1: a command addressed to us
      payload = m.data + 2;
      len = (uint16_t)(m.data_length_code - 2u);
      break;
    default: return;
  }

  // A complete, CRC-valid payload addressed to us - but not necessarily an answer:
  // commands_process_packet() on the VESC points its global send_func at the CAN reply
  // path, so after our first request every unsolicited packet (COMM_PRINT from
  // commands_printf / LispBM print, terminal output) travels the same way. Only the
  // two GET_VALUES packet ids are ours; anything else is neither good nor bad.
  if (!vesc_is_getvalues_payload(payload, len)) {
    log_d("VESC poll: unsolicited packet id %u (%u B) from id %u ignored", (unsigned)payload[0], (unsigned)len,
          (unsigned)m.data[0]);
    return;
  }

  // Copy-modify-write so that fields the mask does not carry keep their previous value;
  // canTask is the sole writer of both structs, so the copy cannot go stale in between.
  VescExt ext;
  vesc_telemetry_t t;
  if (!state_lock()) return;  // lock timeout (counted by state_lock): the next poll is at most VESC_POLL_MS away
  ext = g_state.vesc_ext;
  t = g_state.vesc.t;
  state_unlock();

  ext.vesc_id = m.data[0];  // PROCESS_*_BUFFER byte 0 = the VESC's primary id; mask bit 17 (if requested) overrides it
  const uint32_t mask = vesc_getvalues_mask(payload, len);
  const int nfields = vesc_decode_getvalues(payload, len, &ext, &t);
  if (nfields < 0) {  // incomplete header or mask bits above 21: the CRC passed, so this is a layout we do not know
    reply_bad(now, "unknown layout", len);
    return;
  }
  const int expected = vesc_getvalues_field_count(mask);
  if (nfields < expected && !s_partial_logged) {
    // Older firmware echoes the requested mask but omits the bits it does not implement
    // (bit 21 'status' needs FW >= 5.03). The leading fields are valid, the rest keep
    // their previous value; say so once instead of rejecting every reply.
    s_partial_logged = true;
    log_w("VESC poll: firmware answers %d of %d requested fields (%u of %u B, mask 0x%08lX): missing fields stay at "
          "their previous value (bit 21 'status' needs VESC FW >= 5.03)",
          nfields, expected, (unsigned)len,
          (unsigned)vesc_getvalues_expected_len(payload[0] == VESC_COMM_GET_VALUES ? 0u : mask), (unsigned long)mask);
  }
  [[maybe_unused]] const uint32_t latency = s_poll_outstanding ? (uint32_t)(now - s_poll_sent_ms) : 0u;
  s_poll_outstanding = false;  // the reply did arrive, whatever happens to the store below
  s_consec_fail = 0;
  ext.t_ms = now ? now : 1u;
  ext.replies_ok++;
  if (ext.fault_code != 0) {  // latch: the VESC clears the live byte after its fault stop time
    ext.last_fault = ext.fault_code;
    ext.last_fault_ms = ext.t_ms;
  }

  // Store first (fields only), then log the fault edge against the value that was
  // actually stored: a lock timeout (counted by state_lock) leaves s_prev_fault alone
  // so the next reply reports the edge instead of losing it.
  const uint8_t prev_fault = s_prev_fault;
  if (!state_lock()) return;
  g_state.vesc_ext = ext;
  if (mask & VESC_GV_MASK_TELEMETRY) g_state.vesc.t = t;  // values only; t_ms[] freshness stays with the broadcasts
  s_prev_fault = ext.fault_code;
  state_unlock();

  if (ext.fault_code != prev_fault) {
    if (ext.fault_code != 0)
      log_w("VESC: fault %s (%s, code %u) reported by id %u", vesc_fault_str(ext.fault_code),
            vesc_fault_name(ext.fault_code), (unsigned)ext.fault_code, (unsigned)ext.vesc_id);
    else
      log_i("VESC: fault cleared (was %s)", vesc_fault_str(prev_fault));
  }

  log_d("VESC poll: reply %u B from id %u in %lu ms: fault=%s Tmos=%.1f/%.1f/%.1f Iavg=%.2f/%.2f Id/Iq=%.2f/%.2f "
        "Vd/Vq=%.3f/%.3f tachoAbs=%ld status=0x%02X",
        (unsigned)len, (unsigned)ext.vesc_id, (unsigned long)latency, vesc_fault_str(ext.fault_code), ext.temp_mos1,
        ext.temp_mos2, ext.temp_mos3, ext.avg_motor_current, ext.avg_input_current, ext.avg_id, ext.avg_iq, ext.vd,
        ext.vq, (long)ext.tacho_abs, (unsigned)ext.status);
}

// Called every loop iteration (<= CAN_RX_TIMEOUT_MS apart): times out a stale request,
// then sends the next one when due. Never broadcasts to id 255 (every VESC on the bus
// would answer and their fragments would interleave in one buffer): a target id must be
// known, either pinned (VESC_CAN_ID) or locked from the status frames.
static void poll_tick(uint32_t now) {
  if (s_poll_outstanding && (uint32_t)(now - s_poll_sent_ms) >= VESC_POLL_TIMEOUT_MS) {
    s_poll_outstanding = false;
    vesc_rx_reset(&s_rx);
    if (s_consec_fail != 0xFFFFFFFFu) s_consec_fail++;
    if (state_lock()) {
      g_state.vesc_ext.timeouts++;
      state_unlock();
    }
    s_to_pending++;
    if (due(now, s_to_log_ms, VESC_POLL_LOG_MIN_MS)) {
      s_to_log_ms = now ? now : 1u;
      log_w("VESC poll: %lu request(s) without a complete reply within %d ms (VESC off, wrong id, CAN mode not VESC, "
            "or firmware without COMM_GET_VALUES_SELECTIVE); polling every %lu ms",
            (unsigned long)s_to_pending, VESC_POLL_TIMEOUT_MS,
            (unsigned long)(s_consec_fail >= VESC_POLL_BACKOFF_AFTER ? VESC_POLL_SLOW_MS : VESC_POLL_MS));
      s_to_pending = 0;
    }
  }

  const uint32_t period = s_consec_fail >= VESC_POLL_BACKOFF_AFTER ? (uint32_t)VESC_POLL_SLOW_MS : (uint32_t)VESC_POLL_MS;
  if (!due(now, s_last_poll_ms, period)) return;
  if (s_poll_outstanding || s_poll_suspended) return;  // outstanding only when VESC_POLL_TIMEOUT_MS > VESC_POLL_MS

  int target = VESC_CAN_ID;
  int cs = CAN_STATE_UNINSTALLED;
  if (!state_lock()) return;
  if (target < 0) target = g_state.vesc.locked_id;
  cs = g_state.can.state;
  state_unlock();
  if (target < 0 || cs != CAN_STATE_RUNNING) return;  // no VESC known yet / bus not usable: try again next period
  s_last_poll_ms = now ? now : 1u;

  twai_message_t m{};
  m.extd = 1;  // the VESC only decodes 29-bit ids
  m.ss = 1;    // single shot: no automatic retransmission when nobody acknowledges (VESC off)
  m.identifier = VESC_GETVALUES_REQUEST_EID(target);
  m.data_length_code = (uint8_t)vesc_build_getvalues_request(m.data, (uint8_t)CAN_OWN_ID, VESC_GETVALUES_MASK);
  const esp_err_t err = twai_transmit(&m, pdMS_TO_TICKS(CAN_TX_WAIT_MS));
  if (err != ESP_OK) {
    // ESP_ERR_INVALID_STATE (driver not running), ESP_ERR_TIMEOUT (queue full): the
    // health snapshot / alert handler restore the driver, so only report it.
    s_tx_err_pending++;
    if (due(now, s_tx_err_log_ms, VESC_POLL_LOG_MIN_MS)) {
      s_tx_err_log_ms = now ? now : 1u;
      log_w("VESC poll: twai_transmit failed x%lu: %s (state %s)", (unsigned long)s_tx_err_pending, esp_err_to_name(err),
            can_state_str(cs));
      s_tx_err_pending = 0;
    }
    return;
  }
  s_poll_outstanding = true;
  s_poll_sent_ms = now ? now : 1u;
  vesc_rx_reset(&s_rx);
  if (state_lock()) {
    g_state.vesc_ext.polls_sent++;
    state_unlock();
  }
#if CAN_LOG_RAW_FRAMES
  log_d("CAN tx eid=0x%08lX dlc=%u [%02X %02X %02X %02X %02X %02X %02X]", (unsigned long)m.identifier,
        (unsigned)m.data_length_code, m.data[0], m.data[1], m.data[2], m.data[3], m.data[4], m.data[5], m.data[6]);
#endif
}
#endif  // VESC_POLL_MS > 0

// ---------------------------------------------------------------------------
// Frames
// ---------------------------------------------------------------------------
static void handle_frame(const twai_message_t &m, uint32_t now) {
  // VESC frames are 29-bit data frames with DLC <= 8. Standard-id or remote frames
  // come from some other node and are simply not ours (not counted as drops); a
  // non-compliant DLC 9..15 (the driver flags it dlc_non_comp) still only carries
  // 8 data bytes, so it must never reach the decoders that trust the DLC.
  if (!m.extd || m.rtr || m.data_length_code > 8) return;
  const uint32_t pkt = VESC_EID_PACKET_ID(m.identifier);
  const int id = VESC_EID_CONTROLLER_ID(m.identifier);
#if VESC_POLL_MS > 0
  // Addressed to OUR id with a buffer packet id: the VESC answering a poll.
  if (id == CAN_OWN_ID && vesc_is_getvalues_reply_pkt(pkt)) {
    handle_reply_frame(m, pkt, now);
    return;
  }
#endif
  // Other VESC packet ids (PING/PONG, buffer traffic between VESCs or from VESC
  // Tool, anything else sent to our id) are legitimate: ignore silently.
  if (!vesc_is_status_pkt(pkt)) return;
  const int idx = vesc_status_index(pkt);

  if (m.data_length_code < 8) {  // a status id with a short payload is malformed: count it
    count_dropped();
    return;
  }
#if VESC_POLL_MS > 0
  if (id == CAN_OWN_ID && !s_poll_suspended) {
    // A VESC broadcasts STATUS under our own id (APPCONF_CONTROLLER_ID -1 derives ids
    // from the UUID): its command parser would swallow our reply frames and it would
    // answer requests meant for us. Stay passive; the status path below is unaffected.
    s_poll_suspended = true;
    log_w("VESC: controller id %d equals CAN_OWN_ID, polling suspended (change CAN_OWN_ID or the VESC id)", id);
  }
#endif

  // Controller id policy: VESC_CAN_ID >= 0 pins one id; -1 locks onto the first
  // id seen and counts every other id separately (a second VESC on the bus).
  int want = VESC_CAN_ID;
  bool newly_locked = false;
  vesc_telemetry_t t{};
  if (!state_lock()) return;  // lock timeout (counted by state_lock): drop this frame, the next one is 20 ms away
  if (want < 0) {
    if (g_state.vesc.locked_id < 0) {
      g_state.vesc.locked_id = id;
      newly_locked = true;
    }
    want = g_state.vesc.locked_id;
  } else if (g_state.vesc.locked_id < 0 && id == want) {
    g_state.vesc.locked_id = id;  // pinned id: record it once so consumers have a single "which VESC" field
    newly_locked = true;
  }
  if (id != want) {
    g_state.vesc.frames_other_id++;
    state_unlock();
    return;
  }
  t = g_state.vesc.t;  // local copy: decode outside the lock
  state_unlock();
  if (newly_locked) log_i("VESC: locked onto controller id %d", id);

  // Seed (rather than blend) the EMA on the first frame of this STATUS and after a
  // gap longer than the stale window: otherwise, after a cable came back, the
  // display would show a blend of the live value and one that is minutes old.
  const bool first_of_kind = t.t_ms[idx] == 0 || (uint32_t)(now - t.t_ms[idx]) > VESC_STALE_R1_MS;
#if VESC_POLL_MS > 0
  // The VESC is (back) on the bus: leave the poll back-off at once instead of waiting
  // out the slow period.
  if (first_of_kind && s_consec_fail >= VESC_POLL_BACKOFF_AFTER) s_last_poll_ms = 0;
#endif
  if (!vesc_decode_status(&t, m.identifier, true, m.data, m.data_length_code, now, want)) {
    count_dropped();  // cannot happen after the checks above; kept so a decoder change never passes silently
    return;
  }

#if CAN_LOG_RAW_FRAMES
  log_d("CAN rx eid=0x%08lX pkt=%lu id=%d dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]", (unsigned long)m.identifier,
        (unsigned long)pkt, id, (unsigned)m.data_length_code, m.data[0], m.data[1], m.data[2], m.data[3], m.data[4],
        m.data[5], m.data[6], m.data[7]);
#endif

  // Write back: decoded fields, EMA of the displayed quantities, counters.
  if (!state_lock()) return;
  VescState &v = g_state.vesc;
  v.t = t;
  switch (idx) {
    case VESC_IDX_STATUS_1: v.i_motor_ema = ema_step(v.i_motor_ema, t.current_motor, first_of_kind); v.ema_init = true; break;
    case VESC_IDX_STATUS_4: v.i_in_ema = ema_step(v.i_in_ema, t.current_in, first_of_kind); v.ema_init = true; break;
    case VESC_IDX_STATUS_5: v.v_in_ema = ema_step(v.v_in_ema, t.v_in, first_of_kind); v.ema_init = true; break;
    default: break;
  }
  v.frames_total++;
  v.last_frame_ms = now ? now : 1u;
  state_unlock();
}

// ---------------------------------------------------------------------------
// Periodic health snapshot (twai_get_status_info -> g_state.can)
// ---------------------------------------------------------------------------
static void snapshot_health(uint32_t now) {
  twai_status_info_t st{};
  const esp_err_t err = twai_get_status_info(&st);
  if (err != ESP_OK) {  // only ESP_ERR_INVALID_STATE = driver gone: fall back to the install loop
    s_installed = false;
    set_can_state(CAN_STATE_UNINSTALLED);
    log_e("CAN: twai_get_status_info failed: %s, reinstalling", esp_err_to_name(err));
    return;
  }
  const int cs = map_state(st.state);
  if (state_lock()) {
    CanHealth &c = g_state.can;
    c.state = cs;
    c.tec = st.tx_error_counter;
    c.rec = st.rx_error_counter;
    c.bus_error_count = st.bus_error_count;
    c.rx_missed = st.rx_missed_count;
    c.rx_overrun = st.rx_overrun_count;
    c.arb_lost = st.arb_lost_count;
    c.last_health_ms = now ? now : 1u;
    state_unlock();
  }

  const bool unhealthy = cs != CAN_STATE_RUNNING || st.tx_error_counter >= CAN_ERR_WARN_LEVEL ||
                         st.rx_error_counter >= CAN_ERR_WARN_LEVEL;
  if (unhealthy) {
    log_w("CAN: health state=%s TEC=%lu REC=%lu buserr=%lu missed=%lu overrun=%lu arb_lost=%lu queued=%lu",
          can_state_str(cs), (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
          (unsigned long)st.bus_error_count, (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count,
          (unsigned long)st.arb_lost_count, (unsigned long)st.msgs_to_rx);
  } else {
    log_d("CAN: health state=%s TEC=%lu REC=%lu buserr=%lu missed=%lu overrun=%lu", can_state_str(cs),
          (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter, (unsigned long)st.bus_error_count,
          (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count);
  }

  // Self-heal: STOPPED means a missed BUS_RECOVERED alert or a failed start;
  // BUS_OFF means the recovery kicked off by the alert handler did not take.
  // RUNNING is the only state in which a listener is useful, so try again.
  if (cs == CAN_STATE_STOPPED) {
    log_w("CAN: driver is STOPPED, restarting");
    can_run();
  } else if (cs == CAN_STATE_BUS_OFF) {
    const esp_err_t rerr = twai_initiate_recovery();
    if (rerr == ESP_OK) set_can_state(CAN_STATE_RECOVERING);
    log_w("CAN: driver is BUS_OFF, recovery %s", rerr == ESP_OK ? "restarted" : esp_err_to_name(rerr));
  }
}

// ---------------------------------------------------------------------------
// Task
// ---------------------------------------------------------------------------
static void canTask(void *) {
  for (;;) {
    hb_touch(hb_can);
    const uint32_t now = millis();

    if (!s_installed) {  // install failed at boot or the driver disappeared: retry, but keep the heartbeat alive
      if (due(now, s_last_install_try_ms, CAN_INSTALL_RETRY_MS)) {
        s_last_install_try_ms = now ? now : 1u;
        can_install();
      }
      vTaskDelay(pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));
      continue;
    }

    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts) handle_alerts(alerts, now);

    twai_message_t m{};
    const esp_err_t r = twai_receive(&m, pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));
    if (r == ESP_OK) {
      handle_frame(m, now_nonzero());
    } else if (r != ESP_ERR_TIMEOUT) {
      // Anything but a timeout means the driver is unusable. The header documents
      // ESP_ERR_INVALID_STATE for "not installed", but the IDF 5.5 handle-less
      // wrapper actually returns ESP_ERR_INVALID_ARG (NULL handle), and either
      // way the call returns immediately: without this delay the loop would spin
      // at priority 6 on the single core and starve the supervisor / WDT feed.
      if (!s_invalid_state_logged) {
        log_w("CAN: twai_receive failed: %s, recovering", esp_err_to_name(r));
        s_invalid_state_logged = true;
      }
      if (!can_run()) s_installed = false;  // -> periodic reinstall path above
      vTaskDelay(pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));
    }
    // ESP_ERR_TIMEOUT: idle bus, nothing to do.

#if VESC_POLL_MS > 0
    poll_tick(now_nonzero());  // after the receive: a fresh timestamp, and the reply of the previous poll is in
#endif

    if (due(millis(), s_last_health_ms, CAN_HEALTH_LOG_MS)) {
      s_last_health_ms = now_nonzero();
      snapshot_health(s_last_health_ms);
    }
  }
}

bool can_vesc_start() {
  if (s_task_started) return s_installed;
  const bool ok = can_install();
  s_last_install_try_ms = now_nonzero();
  if (xTaskCreate(canTask, "canTask", CAN_TASK_STACK, nullptr, TASK_PRIO_CAN, nullptr) != pdPASS) {
    log_e("CAN: xTaskCreate(canTask) failed");
    return false;
  }
  s_task_started = true;
  return ok;
}

// ---------------------------------------------------------------------------
// Log summary (called from the supervisor on a snapshot)
// ---------------------------------------------------------------------------
static void fmt_age(char *buf, size_t n, uint32_t t_ms, uint32_t now) {
  if (t_ms == 0) snprintf(buf, n, "never");
  else snprintf(buf, n, "%lums", (unsigned long)(now - t_ms));  // unsigned subtraction: wrap-safe
}

// The motor NTC input floats when nothing is wired, so the VESC reports garbage
// (often -273 or > 200): show "n/a" outside a physically plausible range.
static void fmt_tmot(char *buf, size_t n, const vesc_telemetry_t &t) {
  const bool have = t.t_ms[VESC_IDX_STATUS_4] != 0;
  if (!have || t.temp_motor < -40.0f || t.temp_motor > 200.0f) snprintf(buf, n, "n/a");
  else snprintf(buf, n, "%.1fC", t.temp_motor);
}

#if VESC_POLL_MS > 0
// "| ext: ..." section of the summary: the polled values, their counters and age.
static void fmt_ext(char *buf, size_t n, const SharedState &s, uint32_t now) {
  const VescExt &x = s.vesc_ext;
  char age[32];
  fmt_age(age, sizeof age, x.t_ms, now);
  if (x.t_ms != 0 && (uint32_t)(now - x.t_ms) > VESC_EXT_STALE_MS) strlcat(age, "(stale)", sizeof age);
  char fc[12], lfc[12];  // "F<code>" for fault codes newer than our table instead of a bare "F?"
  vesc_fault_fmt(fc, sizeof fc, x.fault_code);
  char last[40];  // latched fault straight from the snapshot: no lock in the log path
  if (x.last_fault == 0 || x.last_fault_ms == 0) snprintf(last, sizeof last, "none");
  else snprintf(last, sizeof last, "%s@%lums", vesc_fault_fmt(lfc, sizeof lfc, x.last_fault), (unsigned long)(now - x.last_fault_ms));
  const char *st = (x.status & VESC_STATUS_TIMEOUT) ? ((x.status & VESC_STATUS_KILL_SW) ? "TO+KILL" : "TO")
                                                    : ((x.status & VESC_STATUS_KILL_SW) ? "KILL" : "-");
  snprintf(buf, n,
           "| ext: fault=%s last=%s Tmos=%.1f/%.1f/%.1fC Iavg=%.2f/%.2fA Id/Iq=%.2f/%.2fA Vd/Vq=%.3f/%.3fV "
           "tachoAbs=%ld st=%s id=%u polls=%lu ok=%lu bad=%lu to=%lu age=%s",
           fc, last, x.temp_mos1, x.temp_mos2, x.temp_mos3, x.avg_motor_current,
           x.avg_input_current, x.avg_id, x.avg_iq, x.vd, x.vq, (long)x.tacho_abs, st, (unsigned)x.vesc_id,
           (unsigned long)x.polls_sent, (unsigned long)x.replies_ok, (unsigned long)x.replies_bad,
           (unsigned long)x.timeouts, age);
}
#endif

void can_vesc_log_summary(const SharedState &s, uint32_t now) {
  const vesc_telemetry_t &t = s.vesc.t;
  char age[VESC_STATUS_COUNT][24];
  for (int i = 0; i < VESC_STATUS_COUNT; ++i) {
    // STATUS 1/4/5 are "Rate 1" (VESC_STALE_R1_MS), STATUS 2/3/6 are "Rate 2" (VESC_STALE_R2_MS)
    const bool r1 = (i == VESC_IDX_STATUS_1 || i == VESC_IDX_STATUS_4 || i == VESC_IDX_STATUS_5);
    const uint32_t stale_ms = r1 ? VESC_STALE_R1_MS : VESC_STALE_R2_MS;
    fmt_age(age[i], sizeof age[i], t.t_ms[i], now);
    if (t.t_ms[i] != 0 && (uint32_t)(now - t.t_ms[i]) > stale_ms) strlcat(age[i], "(stale)", sizeof age[i]);
  }
  char tmot[16];
  fmt_tmot(tmot, sizeof tmot, t);
  // Polled values: ~200 bytes typical, ~330 with every counter saturated.
  char ext[352];
#if VESC_POLL_MS > 0
  fmt_ext(ext, sizeof ext, s, now);
#else
  snprintf(ext, sizeof ext, "| poll=off");
#endif

  // Typical line ~650 bytes; the worst case (all counters saturated, all ages stale)
  // is ~950 bytes. snprintf truncates safely; loopTask has an 8 KB stack.
  char line[1024];
  snprintf(line, sizeof line,
           "VESC id=%d frames=%lu other=%lu | erpm=%.0f rpm=%.0f duty=%.3f Im=%.1fA(%.1f) | "
           "Iin=%.1fA(%.1f) Vin=%.1fV(%.1f) P=%.0fW | Tfet=%.1fC Tmot=%s | pid=%.1f | "
           "Ah=%.4f/%.4f Wh=%.4f/%.4f | tacho=%ld revs=%.1f | adc=%.3f/%.3f/%.3f ppm=%.3f | "
           "age s1=%s s2=%s s3=%s s4=%s s5=%s s6=%s %s | "
           "can=%s tec=%lu rec=%lu buserr=%lu missed=%lu ovr=%lu busoff=%lu qfull=%lu",
           s.vesc.locked_id, (unsigned long)s.vesc.frames_total, (unsigned long)s.vesc.frames_other_id,
           t.erpm, vesc_mech_rpm(&t, VESC_MOTOR_POLES), t.duty, t.current_motor, s.vesc.i_motor_ema,
           t.current_in, s.vesc.i_in_ema, t.v_in, s.vesc.v_in_ema, vesc_power_w(&t),
           t.temp_fet, tmot, t.pid_pos,
           t.amp_hours, t.amp_hours_charged, t.watt_hours, t.watt_hours_charged,
           (long)t.tachometer, vesc_revolutions(&t, VESC_MOTOR_POLES),
           t.adc1, t.adc2, t.adc3, t.ppm,
           age[0], age[1], age[2], age[3], age[4], age[5], ext,
           can_state_str(s.can.state), (unsigned long)s.can.tec, (unsigned long)s.can.rec,
           (unsigned long)s.can.bus_error_count, (unsigned long)s.can.rx_missed, (unsigned long)s.can.rx_overrun,
           (unsigned long)s.can.bus_off_count, (unsigned long)s.can.queue_full_events);
  log_i("%s", line);
}
