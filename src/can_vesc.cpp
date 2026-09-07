// CAN / VESC listener on the ESP-IDF TWAI node driver (esp_twai_onchip.h): passive
// reception of the VESC STATUS_1..6 broadcast frames, optional active polling of the
// values no broadcast carries (COMM_GET_VALUES_SELECTIVE every VESC_POLL_MS ms),
// bus-off recovery and health reporting.
//
// Protocol traps this file is built around (see the README for detail):
//
//  * Normal mode: the controller asserts the ACK bit. With VESC_POLL_MS == 0 no
//    transmit call exists in the binary (strictly passive node); with VESC_POLL_MS > 0
//    the ONLY frames this node ever transmits are the COMM_GET_VALUES_SELECTIVE
//    requests built in poll_tick(). The ACK matters either way: the VESC's bxCAN has
//    no automatic-retransmission limit, so with a non-ACKing listener as the only
//    other node it goes error-passive and repeats ONE stale STATUS frame forever
//    ("lonely VESC"). CAN_LISTEN_ONLY=1 (true listen-only, no ACK, no polling) is
//    only safe when another ACKing node exists on the bus.
//
//  * Polling is the mirror image of that trap: the node is configured single-shot
//    (fail_retry_cnt 0: no automatic retransmission) so a powered-down VESC never
//    turns this node into a retransmitting "lonely display". Replies are addressed
//    to CAN_OWN_ID (low EID byte), fragmented into FILL_RX_BUFFER frames and closed
//    by PROCESS_RX_BUFFER with length + CRC-16 (vesc_getvalues.h). The VESC clears
//    its fault byte after ~500 ms, so the last non-zero fault code is latched here
//    (can_vesc_last_fault).
//
//  * Driver model: the node driver has no RX queue and no alert bits; its event
//    callbacks run in the TWAI ISR. on_rx_done copies the frame into our own
//    FreeRTOS queue (CAN_RX_QUEUE_LEN deep), the other callbacks only bump counters.
//    canTask blocks on that queue (CAN_RX_TIMEOUT_MS = heartbeat cadence) and turns
//    the counter deltas into the actions the legacy alert handler used to take
//    (bus-off -> twai_node_recover(), state and error logging). Nothing in the
//    callbacks logs, allocates or takes the state mutex.
//
//  * STATUS_5 wire order is tachometer THEN v_in (handled in vesc_status.h).
//
// Concurrency: the state mutex is held only for field copies; decoding and all
// logging happen outside it. canTask is the sole writer of g_state.vesc,
// g_state.vesc_ext and g_state.can; the ISR callbacks write only s_isr.
#include "can_vesc.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_twai.h>
#include <esp_twai_onchip.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "shared_state.h"
#include "vesc_getvalues.h"
#include "vesc_status.h"

static const char *TAG = "can";

// ---------------------------------------------------------------------------
// Task tunables: config.h ("Advanced task tunables") defines all of these; the
// #ifndef fallbacks below only keep this file compiling against an older config.h.
// ---------------------------------------------------------------------------
#ifndef CAN_TASK_STACK
#define CAN_TASK_STACK 4096         // bytes: ESP-IDF's xTaskCreate() takes the stack size in bytes
#endif
#ifndef CAN_INSTALL_RETRY_MS
#define CAN_INSTALL_RETRY_MS 5000   // retry creating / enabling the node while it keeps failing
#endif
#ifndef CAN_RX_TIMEOUT_MS
#define CAN_RX_TIMEOUT_MS 100       // RX queue wait; also bounds the heartbeat and poll-tick period
#endif
#ifndef CAN_ERR_LOG_MIN_MS
#define CAN_ERR_LOG_MIN_MS 2000     // rate limit for the noisy events (bus errors, RX queue full, warning level)
#endif
#ifndef CAN_ERR_WARN_LEVEL
#define CAN_ERR_WARN_LEVEL 96       // ISO 11898-1 error-warning limit for TEC/REC
#endif
#ifndef CAN_TX_WAIT_MS
#define CAN_TX_WAIT_MS 10           // twai_node_transmit() block time for a poll request (the TX queue is normally empty)
#endif
#ifndef CAN_SAMPLE_POINT_PERMILL
#define CAN_SAMPLE_POINT_PERMILL 800 // 80 %: what the legacy 500k/1M presets gave; compatible with the VESC's 78.6 %
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

static_assert(CAN_TX_QUEUE_LEN >= 1, "CAN_TX_QUEUE_LEN must be >= 1 (the node driver needs a TX queue even when it is never used)");
static_assert(CAN_RX_QUEUE_LEN >= 8, "CAN_RX_QUEUE_LEN must hold at least one 6-frame STATUS burst");
static_assert(CAN_SAMPLE_POINT_PERMILL >= 500 && CAN_SAMPLE_POINT_PERMILL <= 900, "CAN_SAMPLE_POINT_PERMILL must be 500..900");

#if VESC_POLL_MS > 0
#if CAN_LISTEN_ONLY
#error "VESC_POLL_MS > 0 requires CAN_LISTEN_ONLY 0: a listen-only node cannot transmit (set VESC_POLL_MS 0 for a passive node)"
#endif
static_assert(VESC_POLL_TIMEOUT_MS > 0, "VESC_POLL_TIMEOUT_MS must be > 0");
static_assert((VESC_GETVALUES_MASK & ~VESC_GV_MASK_ALL) == 0, "VESC_GETVALUES_MASK has bits above 21 (unknown to the decoder)");
// Effective period once the VESC stopped answering: never faster than the normal one.
#define VESC_POLL_SLOW_MS ((VESC_POLL_BACKOFF_MS) > (VESC_POLL_MS) ? (VESC_POLL_BACKOFF_MS) : (VESC_POLL_MS))
#endif

// ---------------------------------------------------------------------------
// Frame as handed from the ISR to canTask (driver-independent)
// ---------------------------------------------------------------------------
struct CanFrame {
  uint32_t id;                        // 11- or 29-bit identifier
  uint8_t dlc;                        // raw DLC 0..15; a classic frame never carries more than 8 data bytes
  bool ext;                           // 29-bit identifier
  bool rtr;                           // remote frame (the node is configured not to receive them; kept for the checks)
  uint8_t data[TWAI_FRAME_MAX_LEN];
};

// ---------------------------------------------------------------------------
// Module state (only touched from canTask, plus the one-shot can_vesc_start())
// ---------------------------------------------------------------------------
static bool s_task_started = false;
static twai_node_handle_t s_node = nullptr;  // created by can_install()
static bool s_enabled = false;               // twai_node_enable() succeeded
static QueueHandle_t s_rx_q = nullptr;       // CanFrame, CAN_RX_QUEUE_LEN deep (ISR -> canTask)
static int s_state = CAN_STATE_UNINSTALLED;  // CanState lifecycle, mirrored into g_state.can.state
static uint32_t s_last_install_try_ms = 0;
static uint32_t s_last_health_ms = 0;
// Rate limiting for events that can fire thousands of times per second on a faulty bus.
static uint32_t s_bus_err_pending = 0, s_bus_err_log_ms = 0;
static uint32_t s_qfull_pending = 0, s_qfull_log_ms = 0;
static uint32_t s_warn_log_ms = 0;

// Counters written in the TWAI ISR callbacks and read by canTask. Monotonic 32-bit
// values: the task keeps the value it processed last and acts on the difference.
// Single core, aligned 32-bit stores are atomic on RV32, so no lock is needed.
struct IsrCounters {
  uint32_t rx_dropped;   // on_rx_done: our queue was full, frame lost
  uint32_t bus_off;      // on_state_change -> TWAI_ERROR_BUS_OFF
  uint32_t err_passive;  // on_state_change -> TWAI_ERROR_PASSIVE
  uint32_t err_warning;  // on_state_change -> TWAI_ERROR_WARNING
  uint32_t err_active;   // on_state_change -> TWAI_ERROR_ACTIVE (also the end of a bus-off recovery)
  uint32_t bus_errors;   // on_error, any flag
  uint32_t ack_err;      // on_error: no acknowledge (our single-shot poll request found nobody)
  uint32_t arb_lost;     // on_error: arbitration lost
  uint32_t tx_failed;    // on_tx_done with is_tx_success == false
};
static volatile IsrCounters s_isr = {};
static IsrCounters s_seen = {};                      // what canTask has already processed
static volatile int s_last_drv_state = -1;           // last twai_error_state_t reported by on_state_change (-1 = none yet)

#if VESC_POLL_MS > 0
// Active polling: one request in flight at a time, one reply being reassembled.
static VescRxBuffer s_rx;                         // FILL_RX_BUFFER reassembly (canTask only)
static bool s_poll_outstanding = false;           // request queued, complete reply not yet seen
static uint32_t s_poll_sent_ms = 0;               // when the outstanding request was queued
static uint32_t s_last_poll_ms = 0;               // 0 = poll at the next tick
static uint32_t s_consec_fail = 0;                // polls in a row without a complete reply (drives the back-off)
static bool s_poll_suspended = false;             // a VESC broadcasts under CAN_OWN_ID: stay passive (handle_frame)
static uint32_t s_tx_err_pending = 0, s_tx_err_log_ms = 0;    // twai_node_transmit() returned an error
static uint32_t s_tx_fail_pending = 0, s_tx_fail_log_ms = 0;  // on_tx_done: single shot not acknowledged
static uint32_t s_to_pending = 0, s_to_log_ms = 0;            // reply timeouts
static uint32_t s_bad_pending = 0, s_bad_log_ms = 0;          // bad replies (length / CRC / layout)
static bool s_partial_logged = false;             // "firmware answers fewer fields than requested" warned once
static uint8_t s_prev_fault = 0;                  // live fault byte of the last STORED reply, for edge logging
// The TX frame and its buffer must stay valid until on_tx_done: one request in flight by design.
static uint8_t s_tx_buf[TWAI_FRAME_MAX_LEN];
static twai_frame_t s_tx_frame;
static volatile bool s_tx_in_flight = false;      // set before twai_node_transmit(), cleared by on_tx_done
// The fault latch itself (last non-zero code + time) lives in g_state.vesc_ext.last_fault / last_fault_ms
// so that every consumer gets it with the snapshot; can_vesc_last_fault() is a convenience reader.
#endif

// state_now_ms() with 0 mapped to 1 so that 0 keeps meaning "never" in every timestamp field.
static inline uint32_t now_nonzero() {
  const uint32_t n = state_now_ms();
  return n ? n : 1u;
}

static inline bool due(uint32_t now, uint32_t last, uint32_t period) {
  return last == 0 || (uint32_t)(now - last) >= period;
}

// ---------------------------------------------------------------------------
// ISR callbacks (TWAI interrupt context: no logging, no mutex, no allocation)
// ---------------------------------------------------------------------------
static bool on_rx_done(twai_node_handle_t node, const twai_rx_done_event_data_t *, void *) {
  CanFrame f;
  twai_frame_t rx = {};
  rx.buffer = f.data;
  rx.buffer_len = sizeof f.data;
  if (twai_node_receive_from_isr(node, &rx) != ESP_OK) return false;
  f.id = rx.header.id;
  f.dlc = (uint8_t)(rx.header.dlc > 15 ? 15 : rx.header.dlc);
  f.ext = rx.header.ide;
  f.rtr = rx.header.rtr;
  BaseType_t hpw = pdFALSE;
  if (xQueueSendFromISR(s_rx_q, &f, &hpw) != pdTRUE) s_isr.rx_dropped = s_isr.rx_dropped + 1;
  return hpw == pdTRUE;
}

static bool on_tx_done(twai_node_handle_t, const twai_tx_done_event_data_t *edata, void *) {
  if (!edata->is_tx_success) s_isr.tx_failed = s_isr.tx_failed + 1;
#if VESC_POLL_MS > 0
  s_tx_in_flight = false;
#endif
  return false;
}

static bool on_state_change(twai_node_handle_t, const twai_state_change_event_data_t *edata, void *) {
  switch (edata->new_sta) {
    case TWAI_ERROR_BUS_OFF: s_isr.bus_off = s_isr.bus_off + 1; break;
    case TWAI_ERROR_PASSIVE: s_isr.err_passive = s_isr.err_passive + 1; break;
    case TWAI_ERROR_WARNING: s_isr.err_warning = s_isr.err_warning + 1; break;
    case TWAI_ERROR_ACTIVE: s_isr.err_active = s_isr.err_active + 1; break;
    default: break;
  }
  s_last_drv_state = (int)edata->new_sta;
  return false;
}

static bool on_error(twai_node_handle_t, const twai_error_event_data_t *edata, void *) {
  s_isr.bus_errors = s_isr.bus_errors + 1;
  if (edata->err_flags.ack_err) s_isr.ack_err = s_isr.ack_err + 1;
  if (edata->err_flags.arb_lost) s_isr.arb_lost = s_isr.arb_lost + 1;
  return false;
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

[[maybe_unused]] static const char *drv_state_str(int s) {  // log-only
  switch (s) {
    case TWAI_ERROR_ACTIVE: return "active";
    case TWAI_ERROR_WARNING: return "warning";
    case TWAI_ERROR_PASSIVE: return "passive";
    case TWAI_ERROR_BUS_OFF: return "bus-off";
    default: return "?";
  }
}

static void set_can_state(int st) {
  s_state = st;
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

[[maybe_unused]] static const char *mode_str() {  // log-only helper
#if VESC_POLL_MS > 0
  return CAN_LISTEN_ONLY ? "LISTEN_ONLY (no ACK)" : "NORMAL (ACK, transmits only GET_VALUES_SELECTIVE polls)";
#else
  return CAN_LISTEN_ONLY ? "LISTEN_ONLY (no ACK)" : "NORMAL (ACK-only, never transmits)";
#endif
}

// Exponential moving average, seeded with the first sample of its source message.
static inline float ema_step(float prev, float x, bool first) {
  return first ? x : (CAN_EMA_ALPHA * x + (1.0f - CAN_EMA_ALPHA) * prev);
}

// TEC / REC / cumulative bus errors from the driver, for logs and the health snapshot.
static bool node_info(twai_node_status_t &st, twai_node_record_t &rec) {
  st = twai_node_status_t{};
  rec = twai_node_record_t{};
  return s_node && twai_node_get_info(s_node, &st, &rec) == ESP_OK;
}

// ---------------------------------------------------------------------------
// Node creation / enable
// ---------------------------------------------------------------------------
// Enables a created node (idempotent) and mirrors the state.
static bool can_run() {
  if (!s_node) return false;
  if (!s_enabled) {
    const esp_err_t err = twai_node_enable(s_node);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "CAN: twai_node_enable failed: %s (retry in %lu ms)", esp_err_to_name(err),
               (unsigned long)CAN_INSTALL_RETRY_MS);
      set_can_state(CAN_STATE_STOPPED);
      return false;
    }
    s_enabled = true;
  }
  set_can_state(CAN_STATE_RUNNING);
  return true;
}

static bool can_install() {
  if (!s_node) {
    twai_onchip_node_config_t cfg = {};
    cfg.io_cfg.tx = (gpio_num_t)PIN_CAN_TX;
    cfg.io_cfg.rx = (gpio_num_t)PIN_CAN_RX;
    cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    cfg.bit_timing.bitrate = (uint32_t)CAN_BITRATE_KBPS * 1000u;
    cfg.bit_timing.sp_permill = CAN_SAMPLE_POINT_PERMILL;
    cfg.fail_retry_cnt = 0;  // single shot: a request nobody acknowledges is not retransmitted (see the header comment)
    cfg.tx_queue_depth = CAN_TX_QUEUE_LEN;
    cfg.intr_priority = 0;
    cfg.flags.enable_listen_only = CAN_LISTEN_ONLY ? 1 : 0;
    cfg.flags.no_receive_rtr = 1;  // remote frames are never ours
    esp_err_t err = twai_new_node_onchip(&cfg, &s_node);
    if (err != ESP_OK) {
      s_node = nullptr;
      ESP_LOGE(TAG, "CAN: twai_new_node_onchip failed: %s (retry in %lu ms)", esp_err_to_name(err),
               (unsigned long)CAN_INSTALL_RETRY_MS);
      set_can_state(CAN_STATE_UNINSTALLED);
      return false;
    }
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = on_rx_done;
    cbs.on_tx_done = on_tx_done;
    cbs.on_state_change = on_state_change;
    cbs.on_error = on_error;
    err = twai_node_register_event_callbacks(s_node, &cbs, nullptr);
    if (err == ESP_OK) {
      // Hardware filter wide open: the controller id is filtered in software so we
      // can lock onto whichever VESC talks first (VESC_CAN_ID == -1); the poll replies
      // (low EID byte == CAN_OWN_ID) pass the same way. The VESC only uses 29-bit ids.
      twai_mask_filter_config_t f = {};
      f.id = 0;
      f.mask = 0;
      f.is_ext = 1;
      err = twai_node_config_mask_filter(s_node, 0, &f);
    }
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "CAN: node setup failed: %s (retry in %lu ms)", esp_err_to_name(err), (unsigned long)CAN_INSTALL_RETRY_MS);
      twai_node_delete(s_node);
      s_node = nullptr;
      set_can_state(CAN_STATE_UNINSTALLED);
      return false;
    }
    set_can_state(CAN_STATE_STOPPED);
  }
  if (!can_run()) return false;

#if VESC_POLL_MS > 0
  ESP_LOGI(TAG, "CAN: node enabled: mode=%s bitrate=%dk sample_point=%u permill single-shot tx=GPIO%d rx=GPIO%d "
                "rx_queue=%d tx_queue=%d poll=%d ms own_id=%d mask=0x%08lX",
           mode_str(), CAN_BITRATE_KBPS, (unsigned)CAN_SAMPLE_POINT_PERMILL, PIN_CAN_TX, PIN_CAN_RX, CAN_RX_QUEUE_LEN,
           CAN_TX_QUEUE_LEN, VESC_POLL_MS, CAN_OWN_ID, (unsigned long)VESC_GETVALUES_MASK);
#else
  ESP_LOGI(TAG, "CAN: node enabled: mode=%s bitrate=%dk sample_point=%u permill single-shot tx=GPIO%d rx=GPIO%d "
                "rx_queue=%d tx_queue=%d poll=off",
           mode_str(), CAN_BITRATE_KBPS, (unsigned)CAN_SAMPLE_POINT_PERMILL, PIN_CAN_TX, PIN_CAN_RX, CAN_RX_QUEUE_LEN,
           CAN_TX_QUEUE_LEN);
#endif
  return true;
}

// ---------------------------------------------------------------------------
// ISR events (bus-off recovery, error state changes, queue overflows)
// ---------------------------------------------------------------------------
static void handle_events(uint32_t now) {
  // One snapshot of the ISR counters; the differences to the last pass are this pass's events.
  const IsrCounters cur = {s_isr.rx_dropped, s_isr.bus_off,   s_isr.err_passive, s_isr.err_warning, s_isr.err_active,
                           s_isr.bus_errors, s_isr.ack_err,   s_isr.arb_lost,    s_isr.tx_failed};
  const uint32_t rx_dropped = cur.rx_dropped - s_seen.rx_dropped;
  const uint32_t bus_off = cur.bus_off - s_seen.bus_off;
  const uint32_t err_passive = cur.err_passive - s_seen.err_passive;
  const uint32_t err_warning = cur.err_warning - s_seen.err_warning;
  const uint32_t err_active = cur.err_active - s_seen.err_active;
  const uint32_t bus_errors = cur.bus_errors - s_seen.bus_errors;
  const uint32_t ack_err = cur.ack_err - s_seen.ack_err;
  [[maybe_unused]] const uint32_t tx_failed = cur.tx_failed - s_seen.tx_failed;
  if (!(rx_dropped | bus_off | err_passive | err_warning | err_active | bus_errors | tx_failed)) return;
  s_seen = cur;
  const int drv_state = s_last_drv_state;

  // 1) Driver actions first: they are time-critical, the logging below is not.
  esp_err_t recovery_err = ESP_OK;
  bool recovered = false;
  if (bus_off) {
    // ESP_ERR_INVALID_STATE here means the node has already left bus-off again (fast bus).
    recovery_err = twai_node_recover(s_node);
    s_state = (recovery_err == ESP_OK || drv_state == TWAI_ERROR_ACTIVE) ? CAN_STATE_RECOVERING : CAN_STATE_BUS_OFF;
  }
  if (err_active && (s_state == CAN_STATE_RECOVERING || s_state == CAN_STATE_BUS_OFF) && drv_state == TWAI_ERROR_ACTIVE) {
    recovered = true;  // the controller saw 129 x 11 recessive bits and rejoined the bus; no re-enable needed
    s_state = CAN_STATE_RUNNING;
  }

  twai_node_status_t st;
  twai_node_record_t rec;
  const bool have_st = node_info(st, rec);
  [[maybe_unused]] const unsigned long tec = have_st ? (unsigned long)st.tx_error_count : 0ul;  // log-only
  [[maybe_unused]] const unsigned long recnt = have_st ? (unsigned long)st.rx_error_count : 0ul;

  // 2) Counters under the lock, no I/O.
  if (state_lock()) {
    CanHealth &c = g_state.can;
    c.state = s_state;
    c.bus_off_count += bus_off;
    if (recovered) c.recoveries++;
    c.err_passive_events += err_passive;
    c.rx_missed = cur.rx_dropped;
    c.arb_lost = cur.arb_lost;
    if (have_st) {
      c.tec = st.tx_error_count;
      c.rec = st.rx_error_count;
      c.bus_error_count = rec.bus_err_num;
    }
    state_unlock();
  }

  // 3) Logs, rate-limited where a faulty bus could flood the console.
  if (bus_off)
    ESP_LOGE(TAG, "CAN: BUS_OFF x%lu (TEC=%lu REC=%lu), recovery %s", (unsigned long)bus_off, tec, recnt,
             recovery_err == ESP_OK ? "started" : esp_err_to_name(recovery_err));
  if (recovered) ESP_LOGI(TAG, "CAN: bus recovered, node error-active again (TEC=%lu REC=%lu)", tec, recnt);
  if (err_passive) ESP_LOGW(TAG, "CAN: error passive (TEC=%lu REC=%lu)", tec, recnt);
  if (err_active && !recovered) ESP_LOGI(TAG, "CAN: error active (TEC=%lu REC=%lu)", tec, recnt);

  if (err_warning) {
    if (due(now, s_warn_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_warn_log_ms = now ? now : 1u;
      ESP_LOGW(TAG, "CAN: error counter above warning level (TEC=%lu REC=%lu)", tec, recnt);
    }
  }
  // Bus errors other than the missing ACK of our own single-shot poll (those are reported below).
  const uint32_t bus_errors_other = bus_errors > ack_err ? bus_errors - ack_err : 0u;
  if (bus_errors_other) {
    s_bus_err_pending += bus_errors_other;
    if (due(now, s_bus_err_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_bus_err_log_ms = now ? now : 1u;
      ESP_LOGW(TAG, "CAN: %lu bus error(s) (driver total=%lu arb_lost=%lu TEC=%lu REC=%lu)", (unsigned long)s_bus_err_pending,
               (unsigned long)rec.bus_err_num, (unsigned long)cur.arb_lost, tec, recnt);
      s_bus_err_pending = 0;
    }
  }
  if (rx_dropped) {
    s_qfull_pending += rx_dropped;
    if (due(now, s_qfull_log_ms, CAN_ERR_LOG_MIN_MS)) {
      s_qfull_log_ms = now ? now : 1u;
      ESP_LOGW(TAG, "CAN: RX queue full, %lu frame(s) lost (total %lu, queue %d)", (unsigned long)s_qfull_pending,
               (unsigned long)cur.rx_dropped, CAN_RX_QUEUE_LEN);
      s_qfull_pending = 0;
    }
  }
#if VESC_POLL_MS > 0
  if (tx_failed) {
    // Single-shot request not acknowledged: nobody listens on the bus (VESC off, cable,
    // bitrate). The reply timeout in poll_tick counts it; this only explains the silence.
    s_tx_fail_pending += tx_failed;
    if (due(now, s_tx_fail_log_ms, VESC_POLL_LOG_MIN_MS)) {
      s_tx_fail_log_ms = now ? now : 1u;
      ESP_LOGW(TAG, "VESC poll: %lu request(s) not acknowledged on the bus (VESC off? ack errors=%lu TEC=%lu)",
               (unsigned long)s_tx_fail_pending, (unsigned long)cur.ack_err, tec);
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
    ESP_LOGW(TAG, "VESC poll: %lu bad reply/replies, last: %s (len %u, expected %u for mask 0x%08lX)",
             (unsigned long)s_bad_pending, why, len, (unsigned)vesc_getvalues_expected_len(VESC_GETVALUES_MASK),
             (unsigned long)VESC_GETVALUES_MASK);
    s_bad_pending = 0;
  }
}

// Frames addressed to OUR id with packet id 5..8 (handle_frame already checked both).
static void handle_reply_frame(const CanFrame &m, uint32_t pkt, uint32_t now) {
#if CAN_LOG_RAW_FRAMES
  ESP_LOGD(TAG, "CAN rx reply eid=0x%08lX pkt=%lu dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]", (unsigned long)m.id,
           (unsigned long)pkt, (unsigned)m.dlc, m.data[0], m.data[1], m.data[2], m.data[3], m.data[4], m.data[5],
           m.data[6], m.data[7]);
#endif
  const uint8_t *payload = nullptr;
  uint16_t len = 0;
  switch (pkt) {
    case VESC_CAN_PACKET_FILL_RX_BUFFER:
    case VESC_CAN_PACKET_FILL_RX_BUFFER_LONG:
      if (!vesc_rx_fill(&s_rx, m.data, m.dlc, pkt == VESC_CAN_PACKET_FILL_RX_BUFFER_LONG)) {
        // A gap (lost frame) or a malformed fragment: the buffer was dropped, the closing
        // PROCESS_RX_BUFFER will fail its length check and count the reply as bad.
        // (A duplicate of a fragment already received is ignored inside vesc_rx_fill.)
        ESP_LOGD(TAG, "VESC poll: reply fragment out of sequence (pkt %lu dlc %u offset %u)", (unsigned long)pkt,
                 (unsigned)m.dlc, (unsigned)m.data[0]);
      }
      return;
    case VESC_CAN_PACKET_PROCESS_RX_BUFFER: {
      const int r = vesc_rx_process(&s_rx, m.data, m.dlc, &payload, &len);
      if (r < 0) return;  // send flag 0 / short frame: a command forwarded TO our id, not a reply
      if (r == 0) {
        reply_bad(now, "length or CRC mismatch", ((unsigned)m.data[2] << 8) | m.data[3]);
        return;
      }
      break;
    }
    case VESC_CAN_PACKET_PROCESS_SHORT_BUFFER:  // reply payload <= 6 bytes: [vesc_id, send=1, data...]
      if (m.dlc < 3 || m.data[1] != 1) return;  // send != 1: a command addressed to us
      payload = m.data + 2;
      len = (uint16_t)(m.dlc - 2u);
      break;
    default: return;
  }

  // A complete, CRC-valid payload addressed to us - but not necessarily an answer:
  // commands_process_packet() on the VESC points its global send_func at the CAN reply
  // path, so after our first request every unsolicited packet (COMM_PRINT from
  // commands_printf / LispBM print, terminal output) travels the same way. Only the
  // two GET_VALUES packet ids are ours; anything else is neither good nor bad.
  if (!vesc_is_getvalues_payload(payload, len)) {
    ESP_LOGD(TAG, "VESC poll: unsolicited packet id %u (%u B) from id %u ignored", (unsigned)payload[0], (unsigned)len,
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
    ESP_LOGW(TAG, "VESC poll: firmware answers %d of %d requested fields (%u of %u B, mask 0x%08lX): missing fields stay at "
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
      ESP_LOGW(TAG, "VESC: fault %s (%s, code %u) reported by id %u", vesc_fault_str(ext.fault_code),
               vesc_fault_name(ext.fault_code), (unsigned)ext.fault_code, (unsigned)ext.vesc_id);
    else
      ESP_LOGI(TAG, "VESC: fault cleared (was %s)", vesc_fault_str(prev_fault));
  }

  ESP_LOGD(TAG, "VESC poll: reply %u B from id %u in %lu ms: fault=%s Tmos=%.1f/%.1f/%.1f Iavg=%.2f/%.2f Id/Iq=%.2f/%.2f "
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
      ESP_LOGW(TAG, "VESC poll: %lu request(s) without a complete reply within %d ms (VESC off, wrong id, CAN mode not VESC, "
                    "or firmware without COMM_GET_VALUES_SELECTIVE); polling every %lu ms",
               (unsigned long)s_to_pending, VESC_POLL_TIMEOUT_MS,
               (unsigned long)(s_consec_fail >= VESC_POLL_BACKOFF_AFTER ? VESC_POLL_SLOW_MS : VESC_POLL_MS));
      s_to_pending = 0;
    }
  }

  const uint32_t period = s_consec_fail >= VESC_POLL_BACKOFF_AFTER ? (uint32_t)VESC_POLL_SLOW_MS : (uint32_t)VESC_POLL_MS;
  if (!due(now, s_last_poll_ms, period)) return;
  if (s_poll_outstanding || s_poll_suspended) return;  // outstanding only when VESC_POLL_TIMEOUT_MS > VESC_POLL_MS
  if (s_tx_in_flight) return;  // the previous request has not left the controller yet (a few ms at most): next tick

  int target = VESC_CAN_ID;
  int cs = CAN_STATE_UNINSTALLED;
  if (!state_lock()) return;
  if (target < 0) target = g_state.vesc.locked_id;
  cs = g_state.can.state;
  state_unlock();
  if (target < 0 || cs != CAN_STATE_RUNNING) return;  // no VESC known yet / bus not usable: try again next period
  s_last_poll_ms = now ? now : 1u;

  const uint8_t len = (uint8_t)vesc_build_getvalues_request(s_tx_buf, (uint8_t)CAN_OWN_ID, VESC_GETVALUES_MASK);
  s_tx_frame = twai_frame_t{};
  s_tx_frame.header.id = VESC_GETVALUES_REQUEST_EID(target);
  s_tx_frame.header.ide = 1;  // the VESC only decodes 29-bit ids
  s_tx_frame.header.dlc = len;
  s_tx_frame.buffer = s_tx_buf;
  s_tx_frame.buffer_len = len;
  s_tx_in_flight = true;  // BEFORE the call: on_tx_done can run before twai_node_transmit() returns
  const esp_err_t err = twai_node_transmit(s_node, &s_tx_frame, CAN_TX_WAIT_MS);
  if (err != ESP_OK) {
    // ESP_ERR_INVALID_STATE (node disabled / bus-off), ESP_ERR_TIMEOUT (queue full): the
    // health snapshot / event handler restore the node, so only report it.
    s_tx_in_flight = false;
    s_tx_err_pending++;
    if (due(now, s_tx_err_log_ms, VESC_POLL_LOG_MIN_MS)) {
      s_tx_err_log_ms = now ? now : 1u;
      ESP_LOGW(TAG, "VESC poll: twai_node_transmit failed x%lu: %s (state %s)", (unsigned long)s_tx_err_pending,
               esp_err_to_name(err), can_state_str(cs));
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
  ESP_LOGD(TAG, "CAN tx eid=0x%08lX dlc=%u [%02X %02X %02X %02X %02X %02X %02X]", (unsigned long)s_tx_frame.header.id,
           (unsigned)len, s_tx_buf[0], s_tx_buf[1], s_tx_buf[2], s_tx_buf[3], s_tx_buf[4], s_tx_buf[5], s_tx_buf[6]);
#endif
}
#endif  // VESC_POLL_MS > 0

// ---------------------------------------------------------------------------
// Frames
// ---------------------------------------------------------------------------
static void handle_frame(const CanFrame &m, uint32_t now) {
  // VESC frames are 29-bit data frames with DLC <= 8. Standard-id or remote frames
  // come from some other node and are simply not ours (not counted as drops); a
  // non-compliant DLC 9..15 still only carries 8 data bytes, so it must never reach
  // the decoders that trust the DLC.
  if (!m.ext || m.rtr || m.dlc > 8) return;
  const uint32_t pkt = VESC_EID_PACKET_ID(m.id);
  const int id = VESC_EID_CONTROLLER_ID(m.id);
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

  if (m.dlc < 8) {  // a status id with a short payload is malformed: count it
    count_dropped();
    return;
  }
#if VESC_POLL_MS > 0
  if (id == CAN_OWN_ID && !s_poll_suspended) {
    // A VESC broadcasts STATUS under our own id (APPCONF_CONTROLLER_ID -1 derives ids
    // from the UUID): its command parser would swallow our reply frames and it would
    // answer requests meant for us. Stay passive; the status path below is unaffected.
    s_poll_suspended = true;
    ESP_LOGW(TAG, "VESC: controller id %d equals CAN_OWN_ID, polling suspended (change CAN_OWN_ID or the VESC id)", id);
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
  if (newly_locked) ESP_LOGI(TAG, "VESC: locked onto controller id %d", id);

  // Seed (rather than blend) the EMA on the first frame of this STATUS and after a
  // gap longer than the stale window: otherwise, after a cable came back, the
  // display would show a blend of the live value and one that is minutes old.
  const bool first_of_kind = t.t_ms[idx] == 0 || (uint32_t)(now - t.t_ms[idx]) > VESC_STALE_R1_MS;
#if VESC_POLL_MS > 0
  // The VESC is (back) on the bus: leave the poll back-off at once instead of waiting
  // out the slow period.
  if (first_of_kind && s_consec_fail >= VESC_POLL_BACKOFF_AFTER) s_last_poll_ms = 0;
#endif
  if (!vesc_decode_status(&t, m.id, true, m.data, m.dlc, now, want)) {
    count_dropped();  // cannot happen after the checks above; kept so a decoder change never passes silently
    return;
  }

#if CAN_LOG_RAW_FRAMES
  ESP_LOGD(TAG, "CAN rx eid=0x%08lX pkt=%lu id=%d dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]", (unsigned long)m.id,
           (unsigned long)pkt, id, (unsigned)m.dlc, m.data[0], m.data[1], m.data[2], m.data[3], m.data[4], m.data[5],
           m.data[6], m.data[7]);
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
// Periodic health snapshot (twai_node_get_info -> g_state.can) and self-heal
// ---------------------------------------------------------------------------
static void snapshot_health(uint32_t now) {
  twai_node_status_t st;
  twai_node_record_t rec;
  if (!node_info(st, rec)) {  // the handle is unusable: drop to the (re)enable path, rate-limited there
    s_enabled = false;
    set_can_state(CAN_STATE_STOPPED);
    ESP_LOGE(TAG, "CAN: twai_node_get_info failed, re-enabling the node");
    return;
  }

  // Reconcile our lifecycle view with the driver's error state in case an ISR event
  // was missed: bus-off without a recovery under way, or a recovery that completed.
  if (st.state == TWAI_ERROR_BUS_OFF && s_state == CAN_STATE_RUNNING) s_state = CAN_STATE_BUS_OFF;
  if (st.state == TWAI_ERROR_ACTIVE && (s_state == CAN_STATE_RECOVERING || s_state == CAN_STATE_BUS_OFF)) {
    s_state = CAN_STATE_RUNNING;
    if (state_lock()) {
      g_state.can.recoveries++;
      state_unlock();
    }
    ESP_LOGI(TAG, "CAN: bus recovered (seen by the health snapshot)");
  }

  const int cs = s_state;
  if (state_lock()) {
    CanHealth &c = g_state.can;
    c.state = cs;
    c.tec = st.tx_error_count;
    c.rec = st.rx_error_count;
    c.bus_error_count = rec.bus_err_num;
    c.rx_missed = s_isr.rx_dropped;
    c.arb_lost = s_isr.arb_lost;
    c.last_health_ms = now ? now : 1u;
    state_unlock();
  }

  const bool unhealthy = cs != CAN_STATE_RUNNING || st.tx_error_count >= CAN_ERR_WARN_LEVEL ||
                         st.rx_error_count >= CAN_ERR_WARN_LEVEL;
  if (unhealthy) {
    ESP_LOGW(TAG, "CAN: health state=%s drv=%s TEC=%lu REC=%lu buserr=%lu missed=%lu arb_lost=%lu txq_free=%lu",
             can_state_str(cs), drv_state_str((int)st.state), (unsigned long)st.tx_error_count,
             (unsigned long)st.rx_error_count, (unsigned long)rec.bus_err_num, (unsigned long)s_isr.rx_dropped,
             (unsigned long)s_isr.arb_lost, (unsigned long)st.tx_queue_remaining);
  } else {
    ESP_LOGD(TAG, "CAN: health state=%s drv=%s TEC=%lu REC=%lu buserr=%lu missed=%lu", can_state_str(cs),
             drv_state_str((int)st.state), (unsigned long)st.tx_error_count, (unsigned long)st.rx_error_count,
             (unsigned long)rec.bus_err_num, (unsigned long)s_isr.rx_dropped);
  }

  // Self-heal: BUS_OFF means the recovery kicked off by the event handler did not
  // take (or the event was missed). RUNNING is the only state in which a listener
  // is useful, so try again.
  if (cs == CAN_STATE_BUS_OFF) {
    const esp_err_t rerr = twai_node_recover(s_node);
    if (rerr == ESP_OK) set_can_state(CAN_STATE_RECOVERING);
    ESP_LOGW(TAG, "CAN: node is BUS_OFF, recovery %s", rerr == ESP_OK ? "restarted" : esp_err_to_name(rerr));
  }
}

// ---------------------------------------------------------------------------
// Task
// ---------------------------------------------------------------------------
static void canTask(void *) {
  for (;;) {
    hb_touch(hb_can);
    const uint32_t now = state_now_ms();

    if (!s_node || !s_enabled) {  // creation / enable failed or the node was lost: retry, but keep the heartbeat alive
      if (due(now, s_last_install_try_ms, CAN_INSTALL_RETRY_MS)) {
        s_last_install_try_ms = now ? now : 1u;
        can_install();
      }
      vTaskDelay(pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));
      continue;
    }

    handle_events(now);

    // Wait for one frame (or the heartbeat period), then drain the burst that came with
    // it without waiting again (the 6 STATUS frames arrive within ~1 ms); bounded so a
    // flooding bus cannot hold the loop.
    CanFrame m;
    if (xQueueReceive(s_rx_q, &m, pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS)) == pdTRUE) {
      handle_frame(m, now_nonzero());
      for (int n = 1; n < CAN_RX_QUEUE_LEN && xQueueReceive(s_rx_q, &m, 0) == pdTRUE; ++n) handle_frame(m, now_nonzero());
    }

#if VESC_POLL_MS > 0
    poll_tick(now_nonzero());  // after the receive: a fresh timestamp, and the reply of the previous poll is in
#endif

    if (due(state_now_ms(), s_last_health_ms, CAN_HEALTH_LOG_MS)) {
      s_last_health_ms = now_nonzero();
      snapshot_health(s_last_health_ms);
    }
  }
}

bool can_vesc_start() {
  if (s_task_started) return s_enabled;
  if (!s_rx_q) s_rx_q = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CanFrame));
  if (!s_rx_q) {
    ESP_LOGE(TAG, "CAN: xQueueCreate(%d frames) failed", CAN_RX_QUEUE_LEN);
    return false;
  }
  const bool ok = can_install();
  s_last_install_try_ms = now_nonzero();
  if (xTaskCreate(canTask, "canTask", CAN_TASK_STACK, nullptr, TASK_PRIO_CAN, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "CAN: xTaskCreate(canTask) failed");
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
  // is ~950 bytes. snprintf truncates safely; the main task has an 8 KB stack.
  char line[1024];
  snprintf(line, sizeof line,
           "VESC id=%d frames=%lu other=%lu | erpm=%.0f rpm=%.0f duty=%.3f Im=%.1fA(%.1f) | "
           "Iin=%.1fA(%.1f) Vin=%.1fV(%.1f) P=%.0fW | Tfet=%.1fC Tmot=%s | pid=%.1f | "
           "Ah=%.4f/%.4f Wh=%.4f/%.4f | tacho=%ld revs=%.1f | adc=%.3f/%.3f/%.3f ppm=%.3f | "
           "age s1=%s s2=%s s3=%s s4=%s s5=%s s6=%s %s | "
           "can=%s tec=%lu rec=%lu buserr=%lu missed=%lu arb=%lu busoff=%lu recov=%lu",
           s.vesc.locked_id, (unsigned long)s.vesc.frames_total, (unsigned long)s.vesc.frames_other_id,
           t.erpm, vesc_mech_rpm(&t, VESC_MOTOR_POLES), t.duty, t.current_motor, s.vesc.i_motor_ema,
           t.current_in, s.vesc.i_in_ema, t.v_in, s.vesc.v_in_ema, vesc_power_w(&t),
           t.temp_fet, tmot, t.pid_pos,
           t.amp_hours, t.amp_hours_charged, t.watt_hours, t.watt_hours_charged,
           (long)t.tachometer, vesc_revolutions(&t, VESC_MOTOR_POLES),
           t.adc1, t.adc2, t.adc3, t.ppm,
           age[0], age[1], age[2], age[3], age[4], age[5], ext,
           can_state_str(s.can.state), (unsigned long)s.can.tec, (unsigned long)s.can.rec,
           (unsigned long)s.can.bus_error_count, (unsigned long)s.can.rx_missed, (unsigned long)s.can.arb_lost,
           (unsigned long)s.can.bus_off_count, (unsigned long)s.can.recoveries);
  ESP_LOGI(TAG, "%s", line);
}
