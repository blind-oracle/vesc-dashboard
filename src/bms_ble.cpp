// JK BMS over Bluetooth LE: NimBLE central (GATT client) and the bmsTask state machine.
//
// Two tasks touch this file:
//   * the NimBLE host task (prio 21, Kconfig) runs gap_cb and the GATT callbacks below. They
//     only copy a small record into a FreeRTOS message buffer: no logging, no mutex, no heap,
//     no NimBLE calls (a failed send bumps s_dropped);
//   * bmsTask ("bms", BMS_TASK_PRIO) drains that buffer, owns the state machine and EVERY
//     NimBLE API call, feeds the JK frame assembler (jk_bms.h), publishes g_state.bms as one
//     struct copy under state_lock(50) and logs every transition at INFO.
//
// Link state machine (BmsLink is what the display shows; the internal Phase is finer):
//   OFF ---bms_ble_start()---> wait for host sync ---SYNC---> SCANNING, or CONNECTING when
//        BMS_BLE_ADDR is set
//   SCANNING: fast preset for BMS_SCAN_FAST_MS, then the slow preset (restarted every 5 min so
//        the controller's duplicate filter forgets); an advert that matches -> CONNECTING
//   CONNECTING: ble_gap_connect(BMS_CONNECT_TIMEOUT_MS) -> SETUP, or failure + back-off
//   SETUP: MTU exchange -> service 0xFFE0 -> characteristics 0xFFE1 -> descriptors -> CCCD
//        write 01 00 -> 0x97 (device info) -> 0x96 (cell info, re-sent every BMS_POLL_MS)
//        -> first plausible cell-info frame -> STREAM. Deadlines: BMS_SETUP_TIMEOUT_MS from
//        the connect event until 0x96 is sent, BMS_FIRST_FRAME_TIMEOUT_MS from then on.
//   STREAM: notifications -> jk_feed -> decode -> publish; a stalled stream is re-polled after
//        BMS_FIRST_FRAME_TIMEOUT_MS and dropped after twice that.
//   Disconnect / any failure -> back-off (link SCANNING, BMS_RECONNECT_MS doubling up to
//        BMS_RECONNECT_MAX_MS, reset when STREAM is reached) -> direct reconnect to the known
//        address while consecutive failures < BMS_RECONNECT_FAILS, else scan (fast preset).
//   Host RESET -> everything forgotten, wait for the next SYNC.
//
// GATT facts (syssi/esphome-jk-bms + captures): service 0xFFE0; the old TI module and JK-PB
// expose ONE 0xFFE1 (write | write-no-rsp | notify), the newer module TWO 0xFFE1 (one write,
// one read | notify). The notify characteristic is the first 0xFFE1 with NOTIFY, the command
// characteristic the first 0xFFE1 with WRITE_NO_RSP or WRITE (may be the same one). No
// pairing, no connection-parameter update (the C6 controller asserted on that, esp-idf #16010).
#include "bms_ble.h"

#if BMS_BLE_ENABLE

#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/message_buffer.h>
#include <freertos/task.h>
#include <host/ble_att.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_hs_adv.h>
#include <host/ble_hs_id.h>
#include <host/ble_hs_mbuf.h>
#include <host/ble_uuid.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <nvs_flash.h>
#include <sdkconfig.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "jk_bms.h"
#include "shared_state.h"

extern "C" void ble_store_config_init(void);

static const char *TAG = "bms";

// ---------------------------------------------------------------- constants
static constexpr uint16_t JK_SVC_UUID16 = 0xFFE0;
static constexpr uint16_t JK_CHR_UUID16 = 0xFFE1;
// Largest notification we accept: one ATT payload (MTU - 3) plus slack; longer ones are dropped.
static constexpr uint16_t NOTIFY_MAX = CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU + 8;
static constexpr uint8_t MAX_CHRS = 8;                    // characteristics remembered per 0xFFE0 service
static constexpr uint32_t CMD_GAP_MS = 100;               // CCCD write -> 0x97 -> 0x96 spacing
static constexpr uint32_t SCAN_SLOW_REFRESH_MS = 300000;  // restart the slow scan (fresh duplicate filter)
static constexpr uint32_t SCAN_MIN_LIFE_MS = 1000;        // a scan that ends sooner is not restarted directly
static constexpr uint32_t CONNECT_EVENT_GRACE_MS = 5000;  // CONNECT event must follow ble_gap_connect() by timeout + this
static constexpr uint32_t TERMINATE_GRACE_MS = 3000;      // DISCONNECT event must follow ble_gap_terminate() within this
static constexpr uint32_t STREAM_STALL_MS = BMS_FIRST_FRAME_TIMEOUT_MS;  // no frame in STREAM: re-poll; 2x: drop the link
static constexpr uint32_t CRC_WARN_PERIOD_MS = 10000;
static constexpr uint32_t RSSI_POLL_MS = 5000;

// Unsigned copies of the tunables so the wrap-safe arithmetic below never mixes signedness.
static constexpr uint32_t k_reconnect_ms = BMS_RECONNECT_MS;
static constexpr uint32_t k_reconnect_max_ms = BMS_RECONNECT_MAX_MS;
static constexpr uint32_t k_reconnect_fails = BMS_RECONNECT_FAILS;
static constexpr uint32_t k_connect_timeout_ms = BMS_CONNECT_TIMEOUT_MS;
static constexpr uint32_t k_setup_timeout_ms = BMS_SETUP_TIMEOUT_MS;
static constexpr uint32_t k_first_frame_timeout_ms = BMS_FIRST_FRAME_TIMEOUT_MS;
static constexpr uint32_t k_poll_ms = BMS_POLL_MS;
static constexpr uint32_t k_scan_fast_ms = BMS_SCAN_FAST_MS;
static constexpr uint32_t k_scan_fast_itvl_ms = BMS_SCAN_FAST_ITVL_MS;
static constexpr uint32_t k_scan_fast_window_ms = BMS_SCAN_FAST_WINDOW_MS;
static constexpr uint32_t k_scan_slow_itvl_ms = BMS_SCAN_SLOW_ITVL_MS;
static constexpr uint32_t k_scan_slow_window_ms = BMS_SCAN_SLOW_WINDOW_MS;

static const ble_uuid16_t s_uuid_ffe0 = {{BLE_UUID_TYPE_16}, JK_SVC_UUID16};
static const uint8_t k_cccd_notify[2] = {0x01, 0x00};

// ---------------------------------------------------------------- host task -> bmsTask records
// Every record starts with a uint8_t type so the receiver can dispatch on MsgAny::type.
enum MsgType : uint8_t {
  MSG_SYNC = 1,
  MSG_RESET,
  MSG_DISC,
  MSG_DISC_COMPLETE,
  MSG_CONNECT,
  MSG_DISCONNECT,
  MSG_MTU,
  MSG_SVC,
  MSG_SVC_DONE,
  MSG_CHR,
  MSG_CHR_DONE,
  MSG_DSC,
  MSG_DSC_DONE,
  MSG_WRITE_DONE,
  MSG_NOTIFY,
};
struct MsgHdr {
  uint8_t type;
};
struct MsgReason {  // RESET, DISC_COMPLETE, DISCONNECT
  uint8_t type;
  int reason;
};
struct MsgDisc {
  uint8_t type;
  uint8_t addr_type;
  uint8_t addr[6];  // NimBLE order: val[0] = last printed octet
  int8_t rssi;
  uint8_t event_type;
  uint8_t name_len;
  bool has_ffe0;
  char name[31];
};
struct MsgConnect {
  uint8_t type;
  uint16_t conn_handle;
  int status;
};
struct MsgMtu {
  uint8_t type;
  uint16_t status;
  uint16_t value;
};
struct MsgSvc {
  uint8_t type;
  uint16_t start;
  uint16_t end;
};
struct MsgStatus {  // SVC_DONE, CHR_DONE, DSC_DONE, WRITE_DONE
  uint8_t type;
  uint16_t status;
};
struct MsgChr {
  uint8_t type;
  uint8_t properties;
  uint16_t def_handle;
  uint16_t val_handle;
  uint16_t uuid16;  // 0 for 32/128-bit UUIDs
};
struct MsgDsc {
  uint8_t type;
  uint16_t handle;
  uint16_t uuid16;
};
struct MsgNotify {  // only offsetof(data) + len bytes are queued
  uint8_t type;
  uint8_t pad;
  uint16_t len;
  uint8_t data[NOTIFY_MAX];
};
union MsgAny {
  uint8_t type;
  MsgHdr hdr;
  MsgReason reason;
  MsgDisc disc;
  MsgConnect connect;
  MsgMtu mtu;
  MsgSvc svc;
  MsgStatus status;
  MsgChr chr;
  MsgDsc dsc;
  MsgNotify notify;
};

static MessageBufferHandle_t s_mb = nullptr;
static volatile uint32_t s_dropped = 0;  // records the host task could not queue (buffer full / oversized notification)
static MsgNotify s_tx_notify;            // host-task scratch (the host task is its only user)

// Host-task side. Never blocks, never logs.
static void post(const void *msg, size_t len) {
  if (s_mb == nullptr || xMessageBufferSend(s_mb, msg, len, 0) != len) s_dropped = s_dropped + 1;
}

static void post_status(uint8_t type, uint16_t status) {
  MsgStatus m = {};
  m.type = type;
  m.status = status;
  post(&m, sizeof m);
}

// ---------------------------------------------------------------- NimBLE callbacks (host task)
static int gap_cb(struct ble_gap_event *event, void *) {
  switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
      const struct ble_gap_disc_desc &d = event->disc;
      if (d.event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND && d.event_type != BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP) break;
      MsgDisc m = {};
      m.type = MSG_DISC;
      m.addr_type = d.addr.type;
      memcpy(m.addr, d.addr.val, sizeof m.addr);
      m.rssi = d.rssi;
      m.event_type = d.event_type;
      struct ble_hs_adv_fields f;
      if (d.data != nullptr && ble_hs_adv_parse_fields(&f, d.data, d.length_data) == 0) {
        if (f.name != nullptr && f.name_len > 0) {
          m.name_len = f.name_len < sizeof m.name ? f.name_len : (uint8_t)sizeof m.name;
          memcpy(m.name, f.name, m.name_len);
        }
        if (f.uuids16 != nullptr) {
          for (uint8_t i = 0; i < f.num_uuids16; ++i)
            if (f.uuids16[i].value == JK_SVC_UUID16) m.has_ffe0 = true;
        }
      }
      post(&m, sizeof m);
      break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE: {
      MsgReason m = {};
      m.type = MSG_DISC_COMPLETE;
      m.reason = event->disc_complete.reason;
      post(&m, sizeof m);
      break;
    }
    case BLE_GAP_EVENT_CONNECT: {
      MsgConnect m = {};
      m.type = MSG_CONNECT;
      m.status = event->connect.status;
      m.conn_handle = event->connect.conn_handle;
      post(&m, sizeof m);
      break;
    }
    case BLE_GAP_EVENT_DISCONNECT: {
      MsgReason m = {};
      m.type = MSG_DISCONNECT;
      m.reason = event->disconnect.reason;
      post(&m, sizeof m);
      break;
    }
    case BLE_GAP_EVENT_NOTIFY_RX: {
      const uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
      uint16_t out = 0;
      if (len == 0 || len > NOTIFY_MAX ||
          ble_hs_mbuf_to_flat(event->notify_rx.om, s_tx_notify.data, NOTIFY_MAX, &out) != 0) {
        s_dropped = s_dropped + 1;
        break;
      }
      s_tx_notify.type = MSG_NOTIFY;
      s_tx_notify.pad = 0;
      s_tx_notify.len = out;
      post(&s_tx_notify, offsetof(MsgNotify, data) + out);
      break;
    }
    default:
      // MTU is taken from the exchange callback; CONN_UPDATE_REQ / L2CAP_UPDATE_REQ: returning 0
      // accepts whatever the peer asks for (we never initiate an update ourselves).
      break;
  }
  return 0;
}

static int mtu_cb(uint16_t, const struct ble_gatt_error *error, uint16_t mtu, void *) {
  MsgMtu m = {};
  m.type = MSG_MTU;
  m.status = error ? error->status : 0;
  m.value = mtu;
  post(&m, sizeof m);
  return 0;
}

static int svc_cb(uint16_t, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc, void *) {
  const uint16_t st = error ? error->status : 0;
  if (st == 0 && svc != nullptr) {
    MsgSvc m = {};
    m.type = MSG_SVC;
    m.start = svc->start_handle;
    m.end = svc->end_handle;
    post(&m, sizeof m);
  } else {
    post_status(MSG_SVC_DONE, st);  // BLE_HS_EDONE = normal completion
  }
  return 0;
}

static int chr_cb(uint16_t, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *) {
  const uint16_t st = error ? error->status : 0;
  if (st == 0 && chr != nullptr) {
    MsgChr m = {};
    m.type = MSG_CHR;
    m.properties = chr->properties;
    m.def_handle = chr->def_handle;
    m.val_handle = chr->val_handle;
    m.uuid16 = ble_uuid_u16(&chr->uuid.u);
    post(&m, sizeof m);
  } else {
    post_status(MSG_CHR_DONE, st);
  }
  return 0;
}

static int dsc_cb(uint16_t, const struct ble_gatt_error *error, uint16_t, const struct ble_gatt_dsc *dsc, void *) {
  const uint16_t st = error ? error->status : 0;
  if (st == 0 && dsc != nullptr) {
    MsgDsc m = {};
    m.type = MSG_DSC;
    m.handle = dsc->handle;
    m.uuid16 = ble_uuid_u16(&dsc->uuid.u);
    post(&m, sizeof m);
  } else {
    post_status(MSG_DSC_DONE, st);
  }
  return 0;
}

static int write_cb(uint16_t, const struct ble_gatt_error *error, struct ble_gatt_attr *, void *) {
  post_status(MSG_WRITE_DONE, error ? error->status : 0);
  return 0;
}

static void on_sync() {
  MsgHdr m = {MSG_SYNC};
  post(&m, sizeof m);
}

static void on_reset(int reason) {
  MsgReason m = {};
  m.type = MSG_RESET;
  m.reason = reason;
  post(&m, sizeof m);
}

static void host_task(void *) {
  nimble_port_run();  // returns only after nimble_port_stop()
  nimble_port_freertos_deinit();
}

// ---------------------------------------------------------------- bmsTask state
enum Phase : uint8_t { PH_IDLE, PH_BACKOFF, PH_SCAN, PH_CONNECT, PH_SETUP, PH_STREAM };
enum SetupStep : uint8_t { ST_MTU, ST_SVC, ST_CHR, ST_DSC, ST_CCCD, ST_CMD_INFO, ST_CMD_CELL, ST_WAIT_FRAME };

static const char *phase_str(uint8_t p) {
  switch (p) {
    case PH_IDLE: return "idle";
    case PH_BACKOFF: return "backoff";
    case PH_SCAN: return "scan";
    case PH_CONNECT: return "connect";
    case PH_SETUP: return "setup";
    case PH_STREAM: return "stream";
    default: return "?";
  }
}

static const char *step_str(uint8_t st) {
  switch (st) {
    case ST_MTU: return "mtu";
    case ST_SVC: return "service discovery";
    case ST_CHR: return "characteristic discovery";
    case ST_DSC: return "descriptor discovery";
    case ST_CCCD: return "cccd write";
    case ST_CMD_INFO: return "cmd 0x97";
    case ST_CMD_CELL: return "cmd 0x96";
    case ST_WAIT_FRAME: return "first frame";
    default: return "?";
  }
}

static const char *proto_str(uint8_t p) {
  switch (p) {
    case JK_PROTO_JK02_24S: return "JK02_24S";
    case JK_PROTO_JK02_32S: return "JK02_32S";
    default: return "UNKNOWN";
  }
}

struct ChrRec {
  uint16_t def_handle, val_handle, uuid16;
  uint8_t properties;
};

struct Ctx {
  uint32_t now;  // state_now_ms() taken once per loop iteration, after the message receive
  Phase phase;
  SetupStep step;
  bool synced;
  bool connected;    // between CONNECT(status 0) and DISCONNECT
  bool terminating;  // ble_gap_terminate() sent, DISCONNECT pending
  uint8_t own_addr_type;
  // peer
  bool cfg_addr_valid;  // BMS_BLE_ADDR parsed
  ble_addr_t cfg_addr;
  bool peer_known;  // peer holds an address to connect to directly
  ble_addr_t peer;
  char peer_name[32];
  // scan
  bool scan_fast;
  uint32_t scan_started_ms;
  // connection
  uint16_t conn_handle;
  uint32_t connected_ms;
  uint16_t svc_start, svc_end;
  ChrRec chrs[MAX_CHRS];
  uint8_t n_chrs;
  uint16_t notify_val, cmd_val, cccd, dsc_end;
  bool cmd_no_rsp;  // command characteristic supports write-without-response
  // deadlines (state_now_ms() based, compared with signed differences)
  uint32_t deadline_ms;  // connect event / setup / terminate
  uint32_t next_cmd_ms;
  uint32_t next_poll_ms;
  uint32_t first_frame_deadline_ms;
  uint32_t last_frame_ms;
  uint32_t next_rssi_ms;
  bool stall_polled;
  // back-off
  uint32_t backoff_ms;
  uint32_t backoff_until_ms;
  uint32_t fails;  // consecutive attempts that did not reach STREAM
  bool force_scan;
  // layout
  JkProto guess;  // from the device-info frame (or BMS_PROTOCOL)
  JkProto proto;  // confirmed layout of this connection
  bool dev_logged, layout_warned;
  uint32_t frames_conn;  // decoded cell-info frames in this connection
  // log rate limits
  uint32_t last_crc_bad, last_crc_warn_ms;
  uint32_t publish_skipped;
};

// Large objects live at file scope, not on the 4 KB task stack.
static Ctx s;
static JkAssembler s_asm;
static BmsState s_bms;  // mirror of g_state.bms, published as one copy
static JkCellInfo s_ci;
static JkDevInfo s_dev;
static MsgAny s_rx;
static uint8_t s_frame[JK_FRAME_LEN];
static uint32_t s_heap_before = 0;

// ---------------------------------------------------------------- helpers
static inline bool due(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }
static inline uint32_t stamp(uint32_t t) { return t ? t : 1; }  // 0 means "never"
static inline uint16_t ms_to_0625(uint32_t ms) { return (uint16_t)(ms * 1000u / 625u); }

// "aa:bb:cc:dd:ee:ff" in printed order (val[5] first).
static void mac_str(const uint8_t val[6], char out[18]) {
  snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x", val[5], val[4], val[3], val[2], val[1], val[0]);
}

// "C8:47:8C:12:34:56" (':' or '-' separators) -> ble_addr_t with val[0] = LAST printed octet, type public.
static bool parse_addr(const char *str, ble_addr_t &out) {
  memset(&out, 0, sizeof out);
  if (str == nullptr || strlen(str) != 17) return false;
  for (int i = 0; i < 6; ++i) {
    const char *p = str + 3 * i;
    unsigned v = 0;
    for (int k = 0; k < 2; ++k) {
      const char c = p[k];
      unsigned d;
      if (c >= '0' && c <= '9') d = (unsigned)(c - '0');
      else if (c >= 'a' && c <= 'f') d = (unsigned)(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F') d = (unsigned)(c - 'A' + 10);
      else return false;
      v = v * 16u + d;
    }
    if (i < 5 && p[2] != ':' && p[2] != '-') return false;
    out.val[5 - i] = (uint8_t)v;
  }
  out.type = BLE_ADDR_PUBLIC;
  return true;
}

// One whole-struct copy under the mutex; skipped (and counted) when the lock is busy.
static void publish() {
  s_bms.frames_ok = s_asm.frames_ok;
  s_bms.frames_crc_bad = s_asm.frames_crc_bad;
  s_bms.frames_resync = s_asm.resyncs;
  s_bms.notify_dropped = s_dropped;
  if (state_lock(50)) {
    g_state.bms = s_bms;
    state_unlock();
  } else {
    s.publish_skipped++;
  }
}

// link_since_ms only moves when the visible link state changes (back-off and scanning are both
// SCANNING, so "since" is the time without a BMS, which the display's hint wants).
static void set_link(uint8_t link) {
  if (s_bms.link != link) {
    s_bms.link = link;
    s_bms.link_since_ms = stamp(s.now);
  }
  publish();
}

static void forget_connection() {
  s.connected = false;
  s.terminating = false;
  s.conn_handle = 0;
  s.svc_start = s.svc_end = 0;
  s.n_chrs = 0;
  s.notify_val = s.cmd_val = s.cccd = s.dsc_end = 0;
  s.cmd_no_rsp = true;
  s.guess = JK_PROTO_UNKNOWN;
  s.proto = JK_PROTO_UNKNOWN;
  s.dev_logged = s.layout_warned = false;
  s.frames_conn = 0;
  s.stall_polled = false;
  jk_asm_clear(s_asm);  // a half frame from the old connection must not pollute the next
}

static void stop_scan() {
  (void)ble_gap_disc_cancel();  // BLE_HS_EALREADY when nothing is running: fine
}

// ---------------------------------------------------------------- transitions
static void start_scan(bool fast);
static void start_connect();

static void begin_backoff(const char *why) {
  const uint32_t delay = s.backoff_ms;
  ESP_LOGI(TAG, "%s, retry in %lu ms", why, (unsigned long)delay);
  s.phase = PH_BACKOFF;
  s.backoff_until_ms = s.now + delay;
  s.backoff_ms = (delay * 2u > k_reconnect_max_ms) ? k_reconnect_max_ms : delay * 2u;
  set_link(BMS_LINK_SCANNING);
}

static void after_backoff() {
  if (!s.synced) {
    s.phase = PH_IDLE;  // a host reset happened meanwhile: SYNC restarts everything
    return;
  }
  if (s.peer_known && !s.force_scan && s.fails < k_reconnect_fails) {
    start_connect();
  } else {
    s.force_scan = false;
    start_scan(true);
  }
}

// Gives up on the current attempt. With an open connection: terminate (the DISCONNECT event then
// runs the back-off); otherwise back off right away.
static void fail_conn(const char *why, bool count_fail) {
  if (count_fail) s.fails++;
  char buf[128];
  snprintf(buf, sizeof buf, "%s (fails=%lu)", why, (unsigned long)s.fails);
  if (s.connected && !s.terminating) {
    const int rc = ble_gap_terminate(s.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc == 0) {
      ESP_LOGW(TAG, "%s: terminating", buf);
      s.terminating = true;
      s.deadline_ms = s.now + TERMINATE_GRACE_MS;
      return;
    }
    ESP_LOGW(TAG, "%s: ble_gap_terminate rc=%d", buf, rc);
    s_bms.disconnects++;  // the link is already gone; a late DISCONNECT event is ignored below
  }
  forget_connection();
  begin_backoff(buf);
}

static void start_scan(bool fast) {
  struct ble_gap_disc_params p = {};
  p.itvl = ms_to_0625(fast ? k_scan_fast_itvl_ms : k_scan_slow_itvl_ms);
  p.window = ms_to_0625(fast ? k_scan_fast_window_ms : k_scan_slow_window_ms);
  p.filter_policy = 0;
  p.limited = 0;
  // By address: passive is enough. By name: active, the name is often only in the scan response.
  p.passive = s.cfg_addr_valid ? 1 : 0;
  p.filter_duplicates = 1;
  // The legacy ble_gap_disc() of this NimBLE does not check the master state (it never returns
  // EALREADY / EBUSY): over a pending connect attempt it would overwrite that state, over a
  // running scan the controller rejects the parameters. Check here instead.
  if (ble_gap_conn_active()) {
    s.force_scan = true;
    begin_backoff("scan start: a connect attempt is still pending");
    return;
  }
  if (ble_gap_disc_active()) (void)ble_gap_disc_cancel();  // a stale scan (an earlier cancel failed)
  const int rc = ble_gap_disc(s.own_addr_type, BLE_HS_FOREVER, &p, gap_cb, nullptr);
  if (rc != 0) {
    char buf[48];
    snprintf(buf, sizeof buf, "ble_gap_disc rc=%d", rc);
    s.force_scan = true;
    begin_backoff(buf);
    return;
  }
  s.phase = PH_SCAN;
  s.scan_fast = fast;
  s.scan_started_ms = s.now;
  ESP_LOGI(TAG, "scan start (%s, %s) interval %lu ms window %lu ms, match by %s", fast ? "fast" : "slow",
           p.passive ? "passive" : "active", (unsigned long)(fast ? k_scan_fast_itvl_ms : k_scan_slow_itvl_ms),
           (unsigned long)(fast ? k_scan_fast_window_ms : k_scan_slow_window_ms),
           s.cfg_addr_valid ? "address" : "name prefix \"" BMS_BLE_NAME_PREFIX "\" / uuid 0xFFE0");
  set_link(BMS_LINK_SCANNING);
}

static void start_connect() {
  char mac[18];
  mac_str(s.peer.val, mac);
  snprintf(s_bms.addr, sizeof s_bms.addr, "%s", mac);
  // NULL params = controller defaults; never ble_gap_update_params / set_prefered_* (C6 assert history).
  const int rc = ble_gap_connect(s.own_addr_type, &s.peer, (int32_t)k_connect_timeout_ms, nullptr, gap_cb, nullptr);
  if (rc != 0) {
    s.fails++;
    if (rc == BLE_HS_EALREADY || rc == BLE_HS_EBUSY) {
      (void)ble_gap_conn_cancel();
      (void)ble_gap_disc_cancel();
    }
    char buf[80];
    snprintf(buf, sizeof buf, "ble_gap_connect %s rc=%d (fails=%lu)", mac, rc, (unsigned long)s.fails);
    begin_backoff(buf);
    return;
  }
  s.phase = PH_CONNECT;
  s.deadline_ms = s.now + k_connect_timeout_ms + CONNECT_EVENT_GRACE_MS;
  ESP_LOGI(TAG, "connecting %s (addr type %u, timeout %lu ms, fails=%lu)", mac, s.peer.type,
           (unsigned long)k_connect_timeout_ms, (unsigned long)s.fails);
  set_link(BMS_LINK_CONNECTING);
}

static void enter_stream() {
  s.phase = PH_STREAM;
  s.step = ST_WAIT_FRAME;
  s.fails = 0;
  s.backoff_ms = k_reconnect_ms;
  s.force_scan = false;
  s.next_rssi_ms = s.now + RSSI_POLL_MS;
  ESP_LOGI(TAG, "stream: %.3f V %+.3f A (JK sign, +=charge) %+ld W soc %u%% %u cells layout %s, %lu ms after connect",
           s_ci.pack_mv / 1000.0, s_ci.current_ma / 1000.0, (long)(s_ci.power_mw / 1000), (unsigned)s_ci.soc_pct,
           (unsigned)s_ci.cell_count, proto_str(s.proto), (unsigned long)(s.now - s.connected_ms));
  set_link(BMS_LINK_STREAM);
}

// ---------------------------------------------------------------- JK commands
// The only two registers this firmware ever writes: exactly these two jk_build_cmd() call sites.
static bool write_cmd(const uint8_t cmd[JK_CMD_LEN], const char *what) {
  int rc;
  if (s.cmd_no_rsp) rc = ble_gattc_write_no_rsp_flat(s.conn_handle, s.cmd_val, cmd, JK_CMD_LEN);
  else rc = ble_gattc_write_flat(s.conn_handle, s.cmd_val, cmd, JK_CMD_LEN, write_cb, nullptr);
  if (rc != 0) ESP_LOGW(TAG, "%s: write rc=%d", what, rc);
  else ESP_LOGD(TAG, "%s sent", what);
  return rc == 0;
}

static bool send_cmd_device_info() {
  uint8_t cmd[JK_CMD_LEN];
  jk_build_cmd(cmd, JK_REG_DEVICE_INFO, 0, 0);
  return write_cmd(cmd, "cmd 0x97 device info");
}

static bool send_cmd_cell_info() {
  uint8_t cmd[JK_CMD_LEN];
  jk_build_cmd(cmd, JK_REG_CELL_INFO, 0, 0);
  return write_cmd(cmd, "cmd 0x96 cell info");
}

// ---------------------------------------------------------------- frames (called from jk_feed on bmsTask)
static void handle_device_info(const uint8_t *frame) {
  if (!jk_decode_device_info(frame, s_dev)) return;
  s.guess = BMS_PROTOCOL != 0 ? (JkProto)BMS_PROTOCOL : jk_guess_proto(s_dev);
  snprintf(s_bms.model, sizeof s_bms.model, "%s", s_dev.model);
  snprintf(s_bms.sw, sizeof s_bms.sw, "%s", s_dev.sw);
  if (!s.dev_logged) {
    s.dev_logged = true;
    ESP_LOGI(TAG, "device %s hw %s sw %s uptime %lu s power-ons %lu name '%s' -> layout %s%s", s_dev.model, s_dev.hw,
             s_dev.sw, (unsigned long)s_dev.uptime_s, (unsigned long)s_dev.power_on_count, s_dev.name,
             proto_str(s.guess), BMS_PROTOCOL != 0 ? " (forced by BMS_PROTOCOL)" : " (guess)");
  }
  publish();
}

// Layout of this connection: forced by BMS_PROTOCOL (plausibility only warns) or detected from the
// frame with the device-info guess as the first candidate. UNKNOWN = frame counted, nothing decoded.
static bool ensure_layout(const uint8_t *frame) {
  if (s.proto != JK_PROTO_UNKNOWN) return true;
  JkProto p;
  if (BMS_PROTOCOL != 0) {
    p = (JkProto)BMS_PROTOCOL;
    if (!jk_layout_plausible(frame, p) && !s.layout_warned) {
      s.layout_warned = true;
      ESP_LOGW(TAG, "forced layout %s fails the cell-sum check (model %s sw %s): decoding anyway, check BMS_PROTOCOL",
               proto_str(p), s_bms.model, s_bms.sw);
    }
  } else {
    p = jk_detect_proto(frame, s.guess);
    if (p == JK_PROTO_UNKNOWN) {
      if (!s.layout_warned) {
        s.layout_warned = true;
        ESP_LOGW(TAG, "layout unknown: model %s sw %s, cell-info frame plausible in neither 24S nor 32S (pack24 %lu mV pack32 %lu mV)",
                 s_bms.model[0] ? s_bms.model : "?", s_bms.sw[0] ? s_bms.sw : "?", (unsigned long)jk_u32(frame + 118),
                 (unsigned long)jk_u32(frame + 150));
      }
      s_bms.proto = JK_PROTO_UNKNOWN;
      publish();
      return false;
    }
  }
  s.proto = p;
  uint8_t cells = 0;
  for (uint8_t i = 0; i < jk_cell_slots(p); ++i)
    if (jk_u16(frame + 6 + 2 * i) != 0) cells++;
  ESP_LOGI(TAG, "layout confirmed %s (%u cells)%s", proto_str(p), (unsigned)cells,
           (s.guess != JK_PROTO_UNKNOWN && s.guess != p) ? " - differs from the device-info guess" : "");
  return true;
}

static void handle_cell_info(const uint8_t *frame) {
  if (!ensure_layout(frame)) return;
  if (!jk_decode_cell_info(frame, s.proto, s_ci)) return;
  const JkCellInfo &c = s_ci;
  if (c.errors != s_bms.errors) {
    const int fe = jk_first_error(c.errors);
    ESP_LOGI(TAG, "errors 0x%08lx -> 0x%08lx (%s, %u set)", (unsigned long)s_bms.errors, (unsigned long)c.errors,
             fe >= 0 ? jk_error_name((uint8_t)fe) : "none", (unsigned)jk_error_count(c.errors));
  }
  s_bms.proto = s.proto;
  s_bms.cell_count = c.cell_count;
  memcpy(s_bms.cell_mv, c.cell_mv, sizeof s_bms.cell_mv);  // the first BMS_CELLS_MAX slots
  s_bms.cell_min_mv = c.cell_min_mv;
  s_bms.cell_max_mv = c.cell_max_mv;
  s_bms.cell_avg_mv = c.cell_avg_mv;
  s_bms.cell_delta_mv = c.cell_delta_mv;
  s_bms.cell_min_idx = c.cell_min_idx;
  s_bms.cell_max_idx = c.cell_max_idx;
  s_bms.pack_mv = c.pack_mv;
  s_bms.current_ma = c.current_ma;  // JK sign kept (positive = charging); the display applies BMS_CURRENT_SIGN
  s_bms.power_mw = c.power_mw;
  s_bms.t1_d = c.t1_d;
  s_bms.t2_d = c.t2_d;
  s_bms.mos_d = c.mos_d;
  s_bms.soc_pct = c.soc_pct;
  s_bms.soh_pct = c.soh_pct;
  s_bms.remaining_mah = c.remaining_mah;
  s_bms.nominal_mah = c.nominal_mah;
  s_bms.cycle_count = c.cycle_count;
  s_bms.balance_ma = c.balance_ma;
  s_bms.balance_action = c.balance_action;
  s_bms.chg_mos_on = c.chg_mos_on;
  s_bms.dis_mos_on = c.dis_mos_on;
  s_bms.errors = c.errors;
  s_bms.t_ms = stamp(s.now);
  s.last_frame_ms = s.now;
  s.stall_polled = false;
  s.frames_conn++;
  if (s.phase == PH_SETUP) enter_stream();  // publishes
  else publish();
  ESP_LOGD(TAG, "frame #%u %lu mV %ld mA soc %u%% cells %u delta %u mV", (unsigned)c.counter, (unsigned long)c.pack_mv,
           (long)c.current_ma, (unsigned)c.soc_pct, (unsigned)c.cell_count, (unsigned)c.cell_delta_mv);
}

// jk_feed callback: the pointer is into the assembler buffer, valid only during the call -> copy.
static void on_frame(const uint8_t frame[JK_FRAME_LEN], void *) {
  memcpy(s_frame, frame, JK_FRAME_LEN);
  switch (s_frame[4]) {
    case JK_FRAME_DEVICE_INFO: handle_device_info(s_frame); break;
    case JK_FRAME_CELL_INFO: handle_cell_info(s_frame); break;
    default: break;  // 0x01 settings, 0x05 logbook: nothing to show
  }
}

// ---------------------------------------------------------------- message handlers (bmsTask)
static void on_sync_msg() {
  s.synced = true;
  if (s.phase != PH_IDLE) {
    ESP_LOGW(TAG, "sync while %s: ignored", phase_str(s.phase));
    return;
  }
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) ESP_LOGW(TAG, "ble_hs_util_ensure_addr rc=%d", rc);
  rc = ble_hs_id_infer_auto(0, &s.own_addr_type);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d, using the public address", rc);
    s.own_addr_type = BLE_ADDR_PUBLIC;
  }
  uint8_t own[6] = {};
  char mac[18];
  (void)ble_hs_id_copy_addr(s.own_addr_type, own, nullptr);
  mac_str(own, mac);
  ESP_LOGI(TAG, "host synced, own addr %s (type %u), heap %lu (was %lu before BLE init)", mac, (unsigned)s.own_addr_type,
           (unsigned long)esp_get_free_heap_size(), (unsigned long)s_heap_before);
  s.fails = 0;
  s.backoff_ms = k_reconnect_ms;
  s.force_scan = false;
  if (s.cfg_addr_valid) {
    s.peer = s.cfg_addr;
    s.peer_known = true;
    start_connect();
  } else {
    start_scan(true);
  }
}

static void on_reset_msg(int reason) {
  ESP_LOGW(TAG, "host reset reason=%d while %s; waiting for sync", reason, phase_str(s.phase));
  s.synced = false;
  if (s.connected) {
    s_bms.disconnects++;
    s_bms.last_disc_reason = (uint8_t)(reason & 0xFF);
  }
  forget_connection();
  s.phase = PH_IDLE;
  set_link(BMS_LINK_SCANNING);
}

static void on_disc(const MsgDisc &d) {
  if (s.phase != PH_SCAN) return;
  char mac[18];
  mac_str(d.addr, mac);
  char name[32];
  const uint8_t nl = d.name_len < 31 ? d.name_len : 31;
  memcpy(name, d.name, nl);
  name[nl] = 0;
  const size_t plen = strlen(BMS_BLE_NAME_PREFIX);
  const bool name_match = plen > 0 && (size_t)nl >= plen && strncmp(name, BMS_BLE_NAME_PREFIX, plen) == 0;
  bool match;
  if (s.cfg_addr_valid) match = memcmp(d.addr, s.cfg_addr.val, 6) == 0;
  else match = name_match || d.has_ffe0;
  if (!match) {
    if (name_match || d.has_ffe0 || strncmp(name, "JK", 2) == 0)
      ESP_LOGD(TAG, "other JK-ish device %s rssi %d name '%s'%s", mac, (int)d.rssi, name, d.has_ffe0 ? " uuid 0xFFE0" : "");
    return;
  }
  s.peer.type = d.addr_type;
  memcpy(s.peer.val, d.addr, sizeof s.peer.val);
  s.peer_known = true;
  if (nl) snprintf(s.peer_name, sizeof s.peer_name, "%s", name);
  s_bms.rssi = d.rssi;
  ESP_LOGI(TAG, "match %s rssi %d name '%s'%s (%s, addr type %u) - pin it with -DBMS_BLE_ADDR", mac, (int)d.rssi,
           name[0] ? name : s.peer_name, d.has_ffe0 ? " uuid 0xFFE0" : "",
           d.event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP ? "scan rsp" : "adv", (unsigned)d.addr_type);
  stop_scan();
  start_connect();
}

static void on_disc_complete(int reason) {
  if (s.phase != PH_SCAN) return;  // ble_gap_disc_cancel() emits no event, so this is an unexpected end
  const uint32_t life = s.now - s.scan_started_ms;
  ESP_LOGW(TAG, "scan ended reason=%d after %lu ms", reason, (unsigned long)life);
  if (life < SCAN_MIN_LIFE_MS) {
    s.force_scan = true;
    begin_backoff("scan keeps ending");
    return;
  }
  start_scan(s.scan_fast);
}

static void start_svc_disc();

static void on_connect(const MsgConnect &m) {
  if (s.phase != PH_CONNECT) {
    if (m.status == 0) {
      ESP_LOGW(TAG, "unexpected connection handle=%u while %s: terminating", (unsigned)m.conn_handle, phase_str(s.phase));
      (void)ble_gap_terminate(m.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return;
  }
  if (m.status != 0) {
    s.fails++;
    char buf[128];
    snprintf(buf, sizeof buf, "connect failed status=%d%s (fails=%lu)", m.status,
             m.status == BLE_HS_ETIMEOUT ? " = timeout: BMS off, out of range or the JK app is connected" : "",
             (unsigned long)s.fails);
    begin_backoff(buf);
    return;
  }
  forget_connection();  // clean handle table + assembler for this connection
  s.connected = true;
  s.conn_handle = m.conn_handle;
  s.connected_ms = s.now;
  s.phase = PH_SETUP;
  s.step = ST_MTU;
  s.deadline_ms = s.now + k_setup_timeout_ms;
  s_bms.connects++;
  s_bms.mtu = 0;
  ESP_LOGI(TAG, "connected handle=%u (connects=%lu)", (unsigned)m.conn_handle, (unsigned long)s_bms.connects);
  set_link(BMS_LINK_SETUP);
  const int rc = ble_gattc_exchange_mtu(s.conn_handle, mtu_cb, nullptr);
  if (rc == BLE_HS_EALREADY) {
    // The peer ran the MTU exchange first (ble_att_svr_rx_mtu sets TXED_MTU): no callback will
    // come, the negotiated value is already in the channel.
    s_bms.mtu = ble_att_mtu(s.conn_handle);
    ESP_LOGI(TAG, "mtu %u (exchanged by the peer)", (unsigned)s_bms.mtu);
    start_svc_disc();
  } else if (rc != 0) {
    char buf[48];
    snprintf(buf, sizeof buf, "exchange_mtu rc=%d", rc);
    fail_conn(buf, true);
  }
}

static void on_disconnect(int reason) {
  if (!s.connected) {
    ESP_LOGD(TAG, "stray disconnect reason=0x%03x", (unsigned)reason);
    return;
  }
  const uint32_t dur_s = (uint32_t)(s.now - s.connected_ms) / 1000u;
  const bool streamed = s.phase == PH_STREAM;
  const bool local = s.terminating;
  s_bms.disconnects++;
  s_bms.last_disc_reason = (uint8_t)(reason & 0xFF);
  if (!streamed && !local) s.fails++;  // the peer dropped us before the stream started
  forget_connection();
  char buf[128];
  snprintf(buf, sizeof buf, "disconnect reason=0x%03x%s (after %lu s, %s, fails=%lu)", (unsigned)reason,
           local ? " local" : "", (unsigned long)dur_s, streamed ? "was streaming" : "no stream", (unsigned long)s.fails);
  begin_backoff(buf);
}

static bool in_setup(SetupStep st) { return s.phase == PH_SETUP && s.step == st && !s.terminating; }

// ENOTCONN means the link is already gone: ble_gap_conn_broken() delivers the pending GATT
// callbacks (status ENOTCONN) BEFORE the DISCONNECT event, and a synchronous ENOTCONN means that
// event is already queued. Let on_disconnect() do the accounting instead of a futile terminate
// (the setup deadline still covers a lost event).
static void setup_fail_status(const char *what, uint16_t status) {
  if (status == BLE_HS_ENOTCONN) {
    ESP_LOGD(TAG, "%s: connection gone, waiting for the disconnect event", what);
    return;
  }
  char buf[64];
  snprintf(buf, sizeof buf, "%s status=0x%03x", what, (unsigned)status);
  fail_conn(buf, true);
}

static void setup_fail_rc(const char *what, int rc) {
  if (rc == BLE_HS_ENOTCONN) {
    ESP_LOGD(TAG, "%s: connection gone, waiting for the disconnect event", what);
    return;
  }
  char buf[64];
  snprintf(buf, sizeof buf, "%s rc=%d", what, rc);
  fail_conn(buf, true);
}

static void start_svc_disc() {
  s.step = ST_SVC;
  const int rc = ble_gattc_disc_svc_by_uuid(s.conn_handle, &s_uuid_ffe0.u, svc_cb, nullptr);
  if (rc != 0) setup_fail_rc("disc_svc_by_uuid", rc);
}

static void on_mtu(const MsgMtu &m) {
  if (!in_setup(ST_MTU)) return;
  if (m.status == 0) {
    s_bms.mtu = m.value;
    ESP_LOGI(TAG, "mtu %u", (unsigned)m.value);
  } else if (m.status == BLE_HS_ENOTCONN) {
    return;  // link gone during the exchange: the DISCONNECT event follows
  } else {
    s_bms.mtu = 23;  // exchange refused: default ATT MTU, 20-byte notifications
    ESP_LOGW(TAG, "mtu exchange status=0x%03x, assuming 23", (unsigned)m.status);
  }
  start_svc_disc();
}

static void on_svc(const MsgSvc &m) {
  if (!in_setup(ST_SVC)) return;
  if (s.svc_start == 0) {
    s.svc_start = m.start;
    s.svc_end = m.end;
  } else {
    ESP_LOGW(TAG, "second 0xFFE0 service 0x%04x..0x%04x ignored", (unsigned)m.start, (unsigned)m.end);
  }
}

static void on_svc_done(uint16_t status) {
  if (!in_setup(ST_SVC)) return;
  if (status != BLE_HS_EDONE && status != 0) {
    setup_fail_status("service discovery", status);
    return;
  }
  if (s.svc_start == 0) {
    fail_conn("service 0xFFE0 not found (not a JK BMS?)", true);
    return;
  }
  s.step = ST_CHR;
  const int rc = ble_gattc_disc_all_chrs(s.conn_handle, s.svc_start, s.svc_end, chr_cb, nullptr);
  if (rc != 0) setup_fail_rc("disc_all_chrs", rc);
}

static void on_chr(const MsgChr &m) {
  if (!in_setup(ST_CHR)) return;
  if (s.n_chrs >= MAX_CHRS) return;
  ChrRec &r = s.chrs[s.n_chrs++];
  r.def_handle = m.def_handle;
  r.val_handle = m.val_handle;
  r.uuid16 = m.uuid16;
  r.properties = m.properties;
}

static void write_cccd() {
  s.step = ST_CCCD;
  ESP_LOGI(TAG, "svc 0x%04x..0x%04x notify 0x%04x cmd 0x%04x%s cccd 0x%04x mtu %u", (unsigned)s.svc_start,
           (unsigned)s.svc_end, (unsigned)s.notify_val, (unsigned)s.cmd_val, s.cmd_no_rsp ? "" : " (write with response)",
           (unsigned)s.cccd, (unsigned)s_bms.mtu);
  const int rc = ble_gattc_write_flat(s.conn_handle, s.cccd, k_cccd_notify, sizeof k_cccd_notify, write_cb, nullptr);
  if (rc != 0) setup_fail_rc("cccd write", rc);
}

static void on_chr_done(uint16_t status) {
  if (!in_setup(ST_CHR)) return;
  if (status != BLE_HS_EDONE && status != 0) {
    setup_fail_status("characteristic discovery", status);
    return;
  }
  uint8_t notify_i = 0xFF, cmd_i = 0xFF;
  for (uint8_t i = 0; i < s.n_chrs; ++i) {
    const ChrRec &r = s.chrs[i];
    if (r.uuid16 != JK_CHR_UUID16) continue;
    if (notify_i == 0xFF && (r.properties & BLE_GATT_CHR_PROP_NOTIFY)) notify_i = i;
    if (cmd_i == 0xFF && (r.properties & (BLE_GATT_CHR_PROP_WRITE_NO_RSP | BLE_GATT_CHR_PROP_WRITE))) cmd_i = i;
  }
  if (notify_i == 0xFF || cmd_i == 0xFF) {
    ESP_LOGW(TAG, "no usable 0xFFE1 (notify %s, write %s); %u characteristics in 0x%04x..0x%04x:",
             notify_i == 0xFF ? "missing" : "ok", cmd_i == 0xFF ? "missing" : "ok", (unsigned)s.n_chrs,
             (unsigned)s.svc_start, (unsigned)s.svc_end);
    for (uint8_t i = 0; i < s.n_chrs; ++i)
      ESP_LOGW(TAG, "  chr def 0x%04x val 0x%04x uuid16 0x%04x props 0x%02x", (unsigned)s.chrs[i].def_handle,
               (unsigned)s.chrs[i].val_handle, (unsigned)s.chrs[i].uuid16, (unsigned)s.chrs[i].properties);
    fail_conn("no notify/write characteristic", true);
    return;
  }
  const ChrRec &nc = s.chrs[notify_i];
  s.notify_val = nc.val_handle;
  s.cmd_val = s.chrs[cmd_i].val_handle;
  s.cmd_no_rsp = (s.chrs[cmd_i].properties & BLE_GATT_CHR_PROP_WRITE_NO_RSP) != 0;
  // Descriptor range: value handle + 1 .. next characteristic definition - 1, else the service end.
  s.dsc_end = s.svc_end;
  for (uint8_t i = 0; i < s.n_chrs; ++i)
    if (s.chrs[i].def_handle > nc.def_handle && (uint16_t)(s.chrs[i].def_handle - 1) < s.dsc_end)
      s.dsc_end = (uint16_t)(s.chrs[i].def_handle - 1);
  if (s.dsc_end <= s.notify_val) {
    s.cccd = (uint16_t)(s.notify_val + 1);
    ESP_LOGW(TAG, "notify characteristic 0x%04x has no descriptor range, guessing cccd 0x%04x", (unsigned)s.notify_val,
             (unsigned)s.cccd);
    write_cccd();
    return;
  }
  s.step = ST_DSC;
  const int rc = ble_gattc_disc_all_dscs(s.conn_handle, s.notify_val, s.dsc_end, dsc_cb, nullptr);
  if (rc != 0) setup_fail_rc("disc_all_dscs", rc);
}

static void on_dsc(const MsgDsc &m) {
  if (!in_setup(ST_DSC)) return;
  if (m.uuid16 == BLE_GATT_DSC_CLT_CFG_UUID16 && s.cccd == 0) s.cccd = m.handle;
}

static void on_dsc_done(uint16_t status) {
  if (!in_setup(ST_DSC)) return;
  if (status != BLE_HS_EDONE && status != 0) {
    setup_fail_status("descriptor discovery", status);
    return;
  }
  if (s.cccd == 0) {
    s.cccd = (uint16_t)(s.notify_val + 1);
    ESP_LOGW(TAG, "no CCCD (0x2902) in 0x%04x..0x%04x, guessing 0x%04x", (unsigned)(s.notify_val + 1),
             (unsigned)s.dsc_end, (unsigned)s.cccd);
  }
  write_cccd();
}

static void on_write_done(uint16_t status) {
  if (!s.connected || s.terminating) return;
  if (s.phase == PH_SETUP && s.step == ST_CCCD) {
    if (status != 0) {
      setup_fail_status("cccd write", status);
      return;
    }
    ESP_LOGI(TAG, "subscribed (cccd 0x%04x <- 01 00), commands in %lu ms", (unsigned)s.cccd, (unsigned long)CMD_GAP_MS);
    s.step = ST_CMD_INFO;
    s.next_cmd_ms = s.now + CMD_GAP_MS;
    return;
  }
  // A command written with response (command characteristic without WRITE_NO_RSP).
  if (status != 0) ESP_LOGW(TAG, "command write status=0x%03x", (unsigned)status);
}

static void on_notify(const MsgNotify &m, size_t rec_len) {
  if (!s.connected || s.terminating) return;
  const size_t avail = rec_len > offsetof(MsgNotify, data) ? rec_len - offsetof(MsgNotify, data) : 0;
  size_t len = m.len;
  if (len > avail) len = avail;
  if (len == 0) return;
  jk_feed(s_asm, m.data, len, on_frame, nullptr);  // on_frame never re-enters jk_feed
  if (s_asm.frames_crc_bad != s.last_crc_bad && due(s.now, s.last_crc_warn_ms + CRC_WARN_PERIOD_MS)) {
    ESP_LOGW(TAG, "crc bad +%lu (total %lu), resyncs %lu (ACK echoes count), frames ok %lu, mtu %u",
             (unsigned long)(s_asm.frames_crc_bad - s.last_crc_bad), (unsigned long)s_asm.frames_crc_bad,
             (unsigned long)s_asm.resyncs, (unsigned long)s_asm.frames_ok, (unsigned)s_bms.mtu);
    s.last_crc_bad = s_asm.frames_crc_bad;
    s.last_crc_warn_ms = s.now;
  }
}

static void handle_msg(const MsgAny &m, size_t len) {
  switch (m.type) {
    case MSG_SYNC: on_sync_msg(); break;
    case MSG_RESET: on_reset_msg(m.reason.reason); break;
    case MSG_DISC: on_disc(m.disc); break;
    case MSG_DISC_COMPLETE: on_disc_complete(m.reason.reason); break;
    case MSG_CONNECT: on_connect(m.connect); break;
    case MSG_DISCONNECT: on_disconnect(m.reason.reason); break;
    case MSG_MTU: on_mtu(m.mtu); break;
    case MSG_SVC: on_svc(m.svc); break;
    case MSG_SVC_DONE: on_svc_done(m.status.status); break;
    case MSG_CHR: on_chr(m.chr); break;
    case MSG_CHR_DONE: on_chr_done(m.status.status); break;
    case MSG_DSC: on_dsc(m.dsc); break;
    case MSG_DSC_DONE: on_dsc_done(m.status.status); break;
    case MSG_WRITE_DONE: on_write_done(m.status.status); break;
    case MSG_NOTIFY: on_notify(m.notify, len); break;
    default: ESP_LOGE(TAG, "bad message type %u len %u", (unsigned)m.type, (unsigned)len); break;
  }
}

// ---------------------------------------------------------------- deadlines (every loop iteration)
static void setup_timeout() {
  char buf[64];
  snprintf(buf, sizeof buf, "setup timeout at %s", step_str(s.step));
  fail_conn(buf, true);
}

static void setup_tick() {
  switch (s.step) {
    case ST_CMD_INFO:
      if (due(s.now, s.next_cmd_ms)) {
        (void)send_cmd_device_info();  // optional: the layout is detected from the frames anyway
        s.step = ST_CMD_CELL;
        s.next_cmd_ms = s.now + CMD_GAP_MS;
      } else if (due(s.now, s.deadline_ms)) {
        setup_timeout();
      }
      break;
    case ST_CMD_CELL:
      if (due(s.now, s.next_cmd_ms)) {
        (void)send_cmd_cell_info();
        s.step = ST_WAIT_FRAME;
        s.next_poll_ms = s.now + k_poll_ms;
        s.first_frame_deadline_ms = s.now + k_first_frame_timeout_ms;
      } else if (due(s.now, s.deadline_ms)) {
        setup_timeout();
      }
      break;
    case ST_WAIT_FRAME:
      if (due(s.now, s.first_frame_deadline_ms)) {
        char buf[80];
        snprintf(buf, sizeof buf, "no plausible cell-info frame within %lu ms (frames ok %lu)",
                 (unsigned long)k_first_frame_timeout_ms, (unsigned long)s_asm.frames_ok);
        fail_conn(buf, true);
      } else if (due(s.now, s.next_poll_ms)) {
        ESP_LOGI(TAG, "no cell-info frame yet, re-sending 0x96");
        (void)send_cmd_cell_info();
        s.next_poll_ms = s.now + k_poll_ms;
      }
      break;
    default:  // ST_MTU .. ST_CCCD: GATT procedures under the setup deadline
      if (due(s.now, s.deadline_ms)) setup_timeout();
      break;
  }
}

static void stream_tick() {
  const uint32_t age = s.now - s.last_frame_ms;
  if (age >= 2u * STREAM_STALL_MS) {
    char buf[64];
    snprintf(buf, sizeof buf, "stream stalled for %lu ms", (unsigned long)age);
    fail_conn(buf, false);  // a stall after a good stream is not a connect failure
    return;
  }
  if (age >= STREAM_STALL_MS && !s.stall_polled) {
    ESP_LOGW(TAG, "no cell-info frame for %lu ms, re-sending 0x96", (unsigned long)age);
    (void)send_cmd_cell_info();
    s.stall_polled = true;
  }
  if (due(s.now, s.next_rssi_ms)) {
    s.next_rssi_ms = s.now + RSSI_POLL_MS;
    int8_t rssi = 0;
    if (ble_gap_conn_rssi(s.conn_handle, &rssi) == 0 && rssi != 127) s_bms.rssi = rssi;  // published with the next frame
  }
}

static void tick() {
  switch (s.phase) {
    case PH_IDLE: break;
    case PH_BACKOFF:
      if (due(s.now, s.backoff_until_ms)) after_backoff();
      break;
    case PH_SCAN:
      if (s.scan_fast) {
        if (due(s.now, s.scan_started_ms + k_scan_fast_ms)) {
          stop_scan();
          start_scan(false);
        }
      } else if (due(s.now, s.scan_started_ms + SCAN_SLOW_REFRESH_MS)) {
        ESP_LOGD(TAG, "slow scan refresh");
        stop_scan();
        start_scan(false);
      }
      break;
    case PH_CONNECT:
      if (due(s.now, s.deadline_ms)) {
        (void)ble_gap_conn_cancel();
        s.fails++;
        begin_backoff("no connect event from the host");
      }
      break;
    case PH_SETUP:
    case PH_STREAM:
      if (s.terminating) {
        if (due(s.now, s.deadline_ms)) {
          ESP_LOGW(TAG, "no disconnect event %lu ms after terminate, dropping the connection state",
                   (unsigned long)TERMINATE_GRACE_MS);
          s_bms.disconnects++;
          forget_connection();
          begin_backoff("terminate timed out");
        }
      } else if (s.phase == PH_STREAM) {
        stream_tick();
      } else {
        setup_tick();
      }
      break;
    default: break;
  }
}

// ---------------------------------------------------------------- task
static void bms_task(void *) {
  s.now = state_now_ms();
  if (BMS_BLE_ADDR[0] != 0) {
    if (parse_addr(BMS_BLE_ADDR, s.cfg_addr)) {
      s.cfg_addr_valid = true;
      ESP_LOGI(TAG, "target %s: direct connect, passive scan by address after %lu failures", BMS_BLE_ADDR,
               (unsigned long)k_reconnect_fails);
    } else {
      ESP_LOGE(TAG, "BMS_BLE_ADDR \"%s\" is not aa:bb:cc:dd:ee:ff: scanning by name / 0xFFE0 instead", BMS_BLE_ADDR);
    }
  }
  for (;;) {
    hb_touch(hb_bms);
    const size_t n = xMessageBufferReceive(s_mb, &s_rx, sizeof s_rx, pdMS_TO_TICKS(100));
    s.now = state_now_ms();
    if (n > 0) handle_msg(s_rx, n);
    tick();
  }
}

// ---------------------------------------------------------------- public API
bool bms_ble_start() {
  s_heap_before = esp_get_free_heap_size();
  memset(&s, 0, sizeof s);
  memset(&s_bms, 0, sizeof s_bms);
  jk_asm_reset(s_asm);
  s.phase = PH_IDLE;
  s.backoff_ms = k_reconnect_ms;
  s.cmd_no_rsp = true;
  s_mb = xMessageBufferCreate(BMS_MSG_BUF_BYTES);
  if (s_mb == nullptr) {
    ESP_LOGE(TAG, "message buffer (%d bytes) alloc failed", BMS_MSG_BUF_BYTES);
    return false;
  }
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "nvs: erasing (%s)", esp_err_to_name(err));
    if (nvs_flash_erase() == ESP_OK) err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init: %s", esp_err_to_name(err));
    return false;
  }
  err = nimble_port_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err));
    return false;
  }
  ble_hs_cfg.reset_cb = on_reset;
  ble_hs_cfg.sync_cb = on_sync;
  ble_hs_cfg.sm_bonding = 0;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 0;
  ble_store_config_init();
  nimble_port_freertos_init(host_task);
  if (xTaskCreate(bms_task, "bms", BMS_TASK_STACK, nullptr, BMS_TASK_PRIO, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "bms task create failed");
    return false;
  }
  ESP_LOGI(TAG, "BLE init ok, heap %lu -> %lu, msg buf %d B, notify cap %u B", (unsigned long)s_heap_before,
           (unsigned long)esp_get_free_heap_size(), BMS_MSG_BUF_BYTES, (unsigned)NOTIFY_MAX);
  return true;
}

const char *bms_link_str(uint8_t link) {
  switch (link) {
    case BMS_LINK_OFF: return "OFF";
    case BMS_LINK_SCANNING: return "SCAN";
    case BMS_LINK_CONNECTING: return "CONN";
    case BMS_LINK_SETUP: return "SETUP";
    case BMS_LINK_STREAM: return "STREAM";
    default: return "?";
  }
}

void bms_log_summary(const SharedState &st, uint32_t now) {
  const BmsState &b = st.bms;
  const uint32_t since_s = b.link_since_ms ? (uint32_t)(now - b.link_since_ms) / 1000u : 0;
  char age[16];
  if (b.t_ms) snprintf(age, sizeof age, "%lums", (unsigned long)age_ms(b.t_ms, now));
  else snprintf(age, sizeof age, "never");
  const int fe = jk_first_error(b.errors);
  ESP_LOGI(TAG,
           "BMS link=%s since=%lus addr=%s rssi=%d mtu=%u proto=%s dev=%s/%s age=%s "
           "pack=%.3fV cur=%+.3fA (JK sign, +=charge) pwr=%+ldW soc=%u%% soh=%u%% cap=%.1f/%.1fAh cyc=%lu "
           "cells=%u min=%umV#%u max=%umV#%u delta=%umV T1=%.1f T2=%.1f MOS=%.1f err=0x%08lx(%s) bal=%dmA/%u "
           "mos=C%u/D%u frames ok=%lu bad=%lu resync=%lu ndrop=%lu conn=%lu/%lu lastdisc=0x%02x heap=%lu",
           bms_link_str(b.link), (unsigned long)since_s, b.addr[0] ? b.addr : "-", (int)b.rssi, (unsigned)b.mtu,
           proto_str(b.proto), b.model[0] ? b.model : "-", b.sw[0] ? b.sw : "-", age, b.pack_mv / 1000.0,
           b.current_ma / 1000.0, (long)(b.power_mw / 1000), (unsigned)b.soc_pct, (unsigned)b.soh_pct,
           b.remaining_mah / 1000.0, b.nominal_mah / 1000.0, (unsigned long)b.cycle_count, (unsigned)b.cell_count,
           (unsigned)b.cell_min_mv, (unsigned)b.cell_min_idx, (unsigned)b.cell_max_mv, (unsigned)b.cell_max_idx,
           (unsigned)b.cell_delta_mv, b.t1_d / 10.0, b.t2_d / 10.0, b.mos_d / 10.0, (unsigned long)b.errors,
           fe >= 0 ? jk_error_name((uint8_t)fe) : "-", (int)b.balance_ma, (unsigned)b.balance_action,
           (unsigned)b.chg_mos_on, (unsigned)b.dis_mos_on, (unsigned long)b.frames_ok, (unsigned long)b.frames_crc_bad,
           (unsigned long)b.frames_resync, (unsigned long)b.notify_dropped, (unsigned long)b.connects,
           (unsigned long)b.disconnects, (unsigned)b.last_disc_reason, (unsigned long)esp_get_free_heap_size());
}

#endif  // BMS_BLE_ENABLE
