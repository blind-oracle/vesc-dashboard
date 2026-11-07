// VESC over UART (VESC_LINK_UART 1): polls the VESC's COMM-header UART with
// COMM_GET_VALUES every VESC_POLL_MS and fills the same g_state.vesc / g_state.vesc_ext
// fields the CAN link does, so every consumer works unchanged. include/can_vesc.h is
// the API (src/can_vesc.cpp compiles to nothing in this build), include/vesc_uart.h
// the codec and the state update.
//
// Things this file is built around (see the README, section 3b):
//
//  * HP UART VESC_UART_PORT (config.h board block). FireBeetle C6: UART0 - UART1 is the
//    GNSS, the LP UART's pins are fixed to the GNSS's 4/5 and the console is USB-Serial/
//    JTAG (config.h refuses a UART console), so UART0 is free once the ROM is done with
//    it. S3-Zero: UART2, which leaves UART0 and its ROM boot-log pads alone. Either way
//    it is routed to the CAN transceiver's old pins.
//
//  * The only bytes ever transmitted are the 6-byte COMM_GET_VALUES request and, for the
//    coolant pump's thresholds (PUMP_READS_MCCONF), a 6-byte COMM_GET_MCCONF in one poll
//    slot at link-up and every VESC_MCCONF_REFRESH_MS. Both are read-only and neither resets
//    the VESC's control timeout, so the dashboard can never keep a motor alive. One request
//    is in flight at a time.
//
//  * Replies are reassembled from whatever uart_read_bytes() returns (vesc_uart_rx_*).
//    A read that returns nothing for VESC_UART_RX_WAIT_MS means the line is idle: a
//    partial frame can then never complete and is flushed rather than waited for.
//    Unsolicited packets (the VESC routes COMM_PRINT to its last requester) are
//    ignored; a late reply that arrives after its timeout is still good data and is
//    stored.
//
//  * Nothing is broadcast over UART, so the back-off for a silent VESC is short
//    (VESC_POLL_BACKOFF_MS, 1 s): no STATUS frame would announce its return.
//
// Concurrency: as in can_vesc.cpp, the state mutex is held only for field copies;
// decoding and all logging happen outside it. vescUartTask is the sole writer of
// g_state.vesc, g_state.vesc_ext and g_state.can.
#include "can_vesc.h"

#if VESC_LINK_UART

#include <driver/gpio.h>
#include <driver/uart.h>
#include <soc/soc_caps.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>

#include "config.h"
#include "pump.h"
#include "shared_state.h"
#include "vesc_getvalues.h"
#include "vesc_log_fmt.h"
#include "vesc_mcconf.h"
#include "vesc_uart.h"

static const char *TAG = "vesc";

constexpr uart_port_t kUart = (uart_port_t)VESC_UART_PORT;

static_assert(VESC_UART_PORT != 1 && VESC_UART_PORT >= 0 && VESC_UART_PORT < SOC_UART_HP_NUM,
              "VESC_UART_PORT must be an HP UART of this chip other than UART1 (the GNSS)");

static_assert(VESC_POLL_TIMEOUT_MS > 0, "VESC_POLL_TIMEOUT_MS must be > 0");
static_assert(VESC_UART_RX_BUFFER > 128, "VESC_UART_RX_BUFFER must exceed the 128-byte hardware FIFO (uart_driver_install)");
static_assert(VESC_UART_RX_WAIT_MS >= 1 && VESC_UART_RX_WAIT_MS <= 100, "VESC_UART_RX_WAIT_MS must be 1..100 ms");
static_assert(VESC_UART_EMA_ALPHA > 0.0f && VESC_UART_EMA_ALPHA <= 1.0f, "VESC_UART_EMA_ALPHA must be in (0, 1]");
// Effective period once the VESC stopped answering: never faster than the normal one.
#define VESC_POLL_SLOW_MS ((VESC_POLL_BACKOFF_MS) > (VESC_POLL_MS) ? (VESC_POLL_BACKOFF_MS) : (VESC_POLL_MS))

// ---------------------------------------------------------------------------
// Module state (only touched from vescUartTask, plus the one-shot can_vesc_start())
// ---------------------------------------------------------------------------
static bool s_task_started = false;
static bool s_installed = false;              // driver installed, pins routed
static uint32_t s_last_install_try_ms = 0;
static VescUartRx s_rx;                       // ~1 KB: static, not on the task stack
static uint8_t s_req[VESC_UART_GETVALUES_REQUEST_LEN];
static size_t s_req_len = 0;
static bool s_poll_outstanding = false;       // request written, complete reply not yet seen
static uint32_t s_poll_sent_ms = 0;
static uint32_t s_last_poll_ms = 0;           // 0 = poll at the next tick
static uint32_t s_consec_fail = 0;            // polls in a row without a complete reply (drives the back-off)
static bool s_partial_logged = false;         // "firmware answers fewer fields" warned once
static bool s_seen_reply = false;             // first reply logged
static uint8_t s_prev_fault = 0;              // live fault byte of the last STORED reply, for edge logging
static uint32_t s_seen_junk = 0, s_seen_bad = 0;              // s_rx counters already published
static uint32_t s_to_pending = 0, s_to_log_ms = 0;            // reply timeouts
static uint32_t s_bad_pending = 0, s_bad_log_ms = 0;          // bad frames / unknown layouts
static uint32_t s_tx_err_pending = 0, s_tx_err_log_ms = 0;    // uart_write_bytes() failed
// The pump's COMM_GET_MCCONF read (include/vesc_mcconf.h). Dead code unless PUMP_READS_MCCONF.
static constexpr bool kReadMcconf = PUMP_READS_MCCONF;
static uint8_t s_req_mc[VESC_UART_CMD_REQUEST_LEN];
static size_t s_req_mc_len = 0;
static VescMcconfSched s_mc;
static bool s_out_is_mc = false;              // the outstanding request is COMM_GET_MCCONF, not GET_VALUES
static uint32_t s_mc_to_log_ms = 0;

// state_now_ms() with 0 mapped to 1 so that 0 keeps meaning "never" in every timestamp field.
static inline uint32_t now_nonzero() {
  const uint32_t n = state_now_ms();
  return n ? n : 1u;
}

static inline bool due(uint32_t now, uint32_t last, uint32_t period) {
  return last == 0 || (uint32_t)(now - last) >= period;
}

// ---------------------------------------------------------------------------
// Public helpers (can_vesc.h)
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

bool can_vesc_polling_enabled() { return true; }

uint8_t can_vesc_last_fault(uint32_t *t_ms_or_null) {
  uint8_t code = 0;
  uint32_t ms = 0;
  if (state_lock()) {
    code = g_state.vesc_ext.last_fault;
    ms = g_state.vesc_ext.last_fault_ms;
    state_unlock();
  }
  if (t_ms_or_null) *t_ms_or_null = ms;
  return code;
}

static void set_link_state(int st) {
  if (state_lock()) {
    g_state.can.state = st;
    state_unlock();
  }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
static bool uart_install() {
  uart_config_t cfg = {};
  cfg.baud_rate = VESC_UART_BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  // RX ring only: the 6-byte request always fits the 128-byte hardware FIFO, so
  // uart_write_bytes() never blocks.
  esp_err_t err = uart_driver_install(kUart, VESC_UART_RX_BUFFER, 0, 0, nullptr, 0);
  const bool have_driver = err == ESP_OK;
  if (err == ESP_OK) err = uart_param_config(kUart, &cfg);
  if (err == ESP_OK) err = uart_set_pin(kUart, PIN_VESC_UART_TX, PIN_VESC_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  // An unpowered or unplugged VESC must read as an idle (high) line, not as noise.
  if (err == ESP_OK) err = gpio_pullup_en((gpio_num_t)PIN_VESC_UART_RX);
  if (err != ESP_OK) {
    if (have_driver) uart_driver_delete(kUart);
    ESP_LOGE(TAG, "VESC: UART%d setup failed: %s (tx=GPIO%d rx=GPIO%d, retry in %lu ms)", VESC_UART_PORT,
             esp_err_to_name(err), PIN_VESC_UART_TX, PIN_VESC_UART_RX, (unsigned long)CAN_INSTALL_RETRY_MS);
    set_link_state(CAN_STATE_UNINSTALLED);
    return false;
  }
  s_installed = true;
  set_link_state(CAN_STATE_RUNNING);
  ESP_LOGI(TAG, "VESC: UART%d tx=GPIO%d rx=GPIO%d %d baud ring=%u B, COMM_GET_VALUES every %d ms (timeout %d ms)",
           VESC_UART_PORT, PIN_VESC_UART_TX, PIN_VESC_UART_RX, VESC_UART_BAUD, (unsigned)VESC_UART_RX_BUFFER,
           VESC_POLL_MS, VESC_POLL_TIMEOUT_MS);
  return true;
}

// ---------------------------------------------------------------------------
// Replies
// ---------------------------------------------------------------------------
// n unusable replies (stop byte / CRC mismatch, or a GET_VALUES layout the decoder
// rejects): count them, warn rate-limited and - only while a request is actually
// outstanding - end that request and count the failure towards the back-off.
// Ends an outstanding COMM_GET_MCCONF that went unanswered or broken: the read scheduler
// retries it, and it never counts towards the GET_VALUES back-off. False if the outstanding
// request was a GET_VALUES poll.
static bool end_mcconf_request(uint32_t now) {
  if (!s_out_is_mc) return false;
  s_out_is_mc = false;
  mcconf_sched_on_timeout(s_mc, now);
  return true;
}

static void reply_bad(uint32_t now, uint32_t n, [[maybe_unused]] const char *why) {
  if (s_poll_outstanding) {
    s_poll_outstanding = false;
    if (!end_mcconf_request(now) && s_consec_fail != 0xFFFFFFFFu) s_consec_fail++;
  }
  if (state_lock()) {
    g_state.vesc_ext.replies_bad += n;
    state_unlock();
  }
  s_bad_pending += n;
  if (due(now, s_bad_log_ms, VESC_POLL_LOG_MIN_MS)) {
    s_bad_log_ms = now;
    ESP_LOGW(TAG, "VESC poll: %lu bad reply/replies, last: %s (wiring noise? baud %d must match VESC Tool)",
             (unsigned long)s_bad_pending, why, VESC_UART_BAUD);
    s_bad_pending = 0;
  }
}

static void handle_packet(const uint8_t *payload, uint16_t len, uint32_t now) {
  if (kReadMcconf && len >= 1 && payload[0] == VESC_COMM_GET_MCCONF) {
    if (s_out_is_mc) {  // the answer to the outstanding request (a late one only reschedules)
      s_out_is_mc = false;
      s_poll_outstanding = false;
    }
    mcconf_sched_on_result(s_mc, pump_mcconf_reply(payload, len, now), now);
    return;
  }

  VescState v;
  VescExt x;
  if (!state_lock()) return;  // lock timeout (counted by state_lock): the next reply is VESC_POLL_MS away
  v = g_state.vesc;
  x = g_state.vesc_ext;
  state_unlock();

  const int nfields = vesc_uart_apply_getvalues(&v, &x, payload, len, now, VESC_UART_EMA_ALPHA);
  if (nfields == VESC_UART_NOT_GETVALUES) {
    ESP_LOGD(TAG, "VESC: unsolicited packet id %u (%u B) ignored", (unsigned)payload[0], (unsigned)len);
    return;
  }
  if (nfields < 0) {
    reply_bad(now, 1, "unknown GET_VALUES layout");
    return;
  }
  const int expected = vesc_getvalues_field_count(vesc_getvalues_mask(payload, len));
  if (nfields < expected && !s_partial_logged) {
    // Older firmware omits the fields it does not implement (bit 21 'status' needs
    // FW >= 5.03): the leading ones are valid, the rest keep their previous value.
    s_partial_logged = true;
    ESP_LOGW(TAG, "VESC poll: firmware answers %d of %d fields (%u B): missing fields stay at their previous value "
                  "(bit 21 'status', needed for the KILLSW check, is FW >= 5.03)",
             nfields, expected, (unsigned)len);
  }
  [[maybe_unused]] const uint32_t latency = s_poll_outstanding ? (uint32_t)(now - s_poll_sent_ms) : 0u;
  const uint32_t fails = s_consec_fail;
  if (!s_out_is_mc) s_poll_outstanding = false;  // the reply did arrive (a late one leaves a COMM_GET_MCCONF in flight)
  s_consec_fail = 0;

  // Store first, then log the fault edge against the value actually stored: a lock
  // timeout leaves s_prev_fault alone so the next reply reports the edge instead.
  const uint8_t prev_fault = s_prev_fault;
  if (!state_lock()) return;
  g_state.vesc = v;
  g_state.vesc_ext = x;
  s_prev_fault = x.fault_code;
  state_unlock();

  if (!s_seen_reply) {
    s_seen_reply = true;
    ESP_LOGI(TAG, "VESC: replying over UART, controller id %d (%u B, %d fields)", v.locked_id, (unsigned)len, nfields);
  } else if (fails >= VESC_POLL_BACKOFF_AFTER) {
    ESP_LOGI(TAG, "VESC: replying again after %lu failed polls", (unsigned long)fails);
  }
  if (x.fault_code != prev_fault) {
    if (x.fault_code != 0)
      ESP_LOGW(TAG, "VESC: fault %s (%s, code %u) reported by id %u", vesc_fault_str(x.fault_code),
               vesc_fault_name(x.fault_code), (unsigned)x.fault_code, (unsigned)x.vesc_id);
    else
      ESP_LOGI(TAG, "VESC: fault cleared (was %s)", vesc_fault_str(prev_fault));
  }
  ESP_LOGD(TAG, "VESC poll: reply %u B in %lu ms: fault=%s Vin=%.1f Iin=%.2f Im=%.2f erpm=%.0f status=0x%02X",
           (unsigned)len, (unsigned long)latency, vesc_fault_str(x.fault_code), v.t.v_in, v.t.current_in,
           v.t.current_motor, v.t.erpm, (unsigned)x.status);
}

// Publishes what the reassembler threw away since the last pass: stray bytes into
// g_state.vesc.frames_dropped (the "misc" counter on VESC 3/3), damaged frames as bad replies.
static void publish_rx_errors(uint32_t now) {
  const uint32_t junk = s_rx.junk - s_seen_junk;
  const uint32_t bad = s_rx.bad - s_seen_bad;
  if (!(junk | bad)) return;
  s_seen_junk = s_rx.junk;
  s_seen_bad = s_rx.bad;
  if (junk && state_lock()) {
    g_state.vesc.frames_dropped += junk;
    state_unlock();
  }
  if (bad) reply_bad(now, bad, "stop byte or CRC mismatch");
}

// ---------------------------------------------------------------------------
// Polling
// ---------------------------------------------------------------------------
// Called every loop iteration (<= VESC_UART_RX_WAIT_MS apart): times out a stale
// request, then writes the next one when due.
static void poll_tick(uint32_t now) {
  const uint32_t timeout = s_out_is_mc ? (uint32_t)VESC_MCCONF_TIMEOUT_MS : (uint32_t)VESC_POLL_TIMEOUT_MS;
  if (s_poll_outstanding && (uint32_t)(now - s_poll_sent_ms) >= timeout && end_mcconf_request(now)) {
    s_poll_outstanding = false;
    if (due(now, s_mc_to_log_ms, VESC_POLL_LOG_MIN_MS)) {
      s_mc_to_log_ms = now;
      ESP_LOGW(TAG, "VESC: no complete answer to COMM_GET_MCCONF within %d ms; pump limits unchanged, retry in %d s",
               VESC_MCCONF_TIMEOUT_MS, VESC_MCCONF_RETRY_MS / 1000);
    }
  }
  if (s_poll_outstanding && !s_out_is_mc && (uint32_t)(now - s_poll_sent_ms) >= VESC_POLL_TIMEOUT_MS) {
    s_poll_outstanding = false;
    if (s_consec_fail != 0xFFFFFFFFu) s_consec_fail++;
    if (state_lock()) {
      g_state.vesc_ext.timeouts++;
      state_unlock();
    }
    s_to_pending++;
    if (due(now, s_to_log_ms, VESC_POLL_LOG_MIN_MS)) {
      s_to_log_ms = now;
      ESP_LOGW(TAG, "VESC poll: %lu request(s) without a complete reply within %d ms (VESC off, TX/RX swapped, App to Use "
                    "without UART, or VESC baud != %d); polling every %lu ms",
               (unsigned long)s_to_pending, VESC_POLL_TIMEOUT_MS, VESC_UART_BAUD,
               (unsigned long)(s_consec_fail >= VESC_POLL_BACKOFF_AFTER ? VESC_POLL_SLOW_MS : VESC_POLL_MS));
      s_to_pending = 0;
    }
  }

  const uint32_t period = s_consec_fail >= VESC_POLL_BACKOFF_AFTER ? (uint32_t)VESC_POLL_SLOW_MS : (uint32_t)VESC_POLL_MS;
  if (!due(now, s_last_poll_ms, period)) return;
  if (s_poll_outstanding) return;  // VESC_POLL_TIMEOUT_MS > VESC_POLL_MS, or a COMM_GET_MCCONF in flight
  s_last_poll_ms = now;

  // The pump's limits read borrows this slot at link-up (the VESC answered the last poll) and
  // every VESC_MCCONF_REFRESH_MS.
  const bool mc = kReadMcconf && mcconf_sched_due(s_mc, s_seen_reply && s_consec_fail == 0, now);
  const uint8_t *req = mc ? s_req_mc : s_req;
  const size_t req_len = mc ? s_req_mc_len : s_req_len;
  const int w = uart_write_bytes(kUart, req, req_len);
  if (w != (int)req_len) {
    s_tx_err_pending++;
    if (due(now, s_tx_err_log_ms, VESC_POLL_LOG_MIN_MS)) {
      s_tx_err_log_ms = now;
      ESP_LOGW(TAG, "VESC poll: uart_write_bytes failed x%lu (returned %d)", (unsigned long)s_tx_err_pending, w);
      s_tx_err_pending = 0;
    }
    return;
  }
  s_poll_outstanding = true;
  s_poll_sent_ms = now;
  s_out_is_mc = mc;
  if (mc) {
    mcconf_sched_on_sent(s_mc);
    pump_mcconf_sent();
    return;
  }
  if (state_lock()) {
    g_state.vesc_ext.polls_sent++;
    state_unlock();
  }
}

// ---------------------------------------------------------------------------
// Task
// ---------------------------------------------------------------------------
static void vescUartTask(void *) {
  uint8_t chunk[128];
  for (;;) {
    hb_touch(hb_can);  // the VESC link's heartbeat, whichever transport (HB_MAX_CAN_MS)

    if (!s_installed) {  // driver setup failed: retry, but keep the heartbeat alive
      const uint32_t now = now_nonzero();
      if (due(now, s_last_install_try_ms, CAN_INSTALL_RETRY_MS)) {
        s_last_install_try_ms = now;
        uart_install();
      }
      vTaskDelay(pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));
      continue;
    }

    // Blocks until the chunk is full or VESC_UART_RX_WAIT_MS has passed; a reply (79 B)
    // therefore arrives in one or two reads. Nothing at all = the line is idle.
    const int n = uart_read_bytes(kUart, chunk, sizeof chunk, pdMS_TO_TICKS(VESC_UART_RX_WAIT_MS));
    if (n > 0) vesc_uart_rx_push(&s_rx, chunk, (size_t)n);
    const uint8_t *payload = nullptr;
    uint16_t len = 0;
    while (vesc_uart_rx_next(&s_rx, n <= 0, &payload, &len)) handle_packet(payload, len, now_nonzero());
    publish_rx_errors(now_nonzero());

    poll_tick(now_nonzero());  // after the receive: the reply of the previous poll is in
  }
}

bool can_vesc_start() {
  if (s_task_started) return s_installed;
  s_req_len = vesc_uart_build_getvalues_request(s_req);
  s_req_mc_len = vesc_uart_build_cmd_request(s_req_mc, (uint8_t)VESC_COMM_GET_MCCONF);
  mcconf_sched_init(s_mc);
  vesc_uart_rx_reset(&s_rx);
  const bool ok = uart_install();
  s_last_install_try_ms = now_nonzero();
  if (xTaskCreatePinnedToCore(vescUartTask, "vescUart", CAN_TASK_STACK, nullptr, TASK_PRIO_CAN, nullptr,
                              TASK_CORE_APP) != pdPASS) {
    ESP_LOGE(TAG, "VESC: xTaskCreate(vescUartTask) failed");
    return false;
  }
  s_task_started = true;
  return ok;
}

// ---------------------------------------------------------------------------
// Log summary (called from the supervisor on a snapshot)
// ---------------------------------------------------------------------------
void can_vesc_log_summary(const SharedState &s, uint32_t now) {
  char link[64];
  snprintf(link, sizeof link, "uart=%s baud=%d junk=%lu", can_state_str(s.can.state), VESC_UART_BAUD,
           (unsigned long)s.vesc.frames_dropped);
  char line[1024];  // the main task has an 8 KB stack
  vesc_log_line(line, sizeof line, s, now, link);
  ESP_LOGI(TAG, "%s", line);
}

#endif  // VESC_LINK_UART
