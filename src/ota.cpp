// Firmware update over the air (include/ota.h): the reboot request, the update-mode access
// point + upload server, and the probation of a freshly flashed image.
//
// Contexts:
//   * the display task calls ota_request_update_mode() (the long hold; it reboots) and
//     ota_request_cancel() (a volatile flag);
//   * the supervisor (app_main) calls ota_tick() in update mode and ota_confirm_if_healthy()
//     in normal operation;
//   * in update mode the HTTP server task runs the upload handler and the default event loop
//     task runs wifi_event(). Both publish into g_state.ota under the state mutex and stamp
//     s_last_activity_ms (a single 32-bit volatile).
//
// The update-mode boot runs nothing of the normal firmware but the display task (main.cpp),
// so the upload may erase flash freely: no TWAI or BLE interrupt is waiting behind the
// disabled cache. The image is written with OTA_WITH_SEQUENTIAL_WRITES, one sector erase at a
// time inside esp_ota_write(), never OTA_SIZE_UNKNOWN, which would erase the whole slot in one
// blocking call and freeze the screen and the TCP connection for seconds.
#include "ota.h"

#if OTA_ENABLE

#include <esp_app_desc.h>
#include <esp_app_format.h>
#include <esp_attr.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_ota_ops.h>
#include <esp_phy_init.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>
#include <string.h>

#include "nvs_util.h"
#include "shared_state.h"

static const char *TAG = "ota";

// The request to enter update mode crosses the reboot in RTC memory, so deciding to write
// flash involves no flash write of its own. RTC_NOINIT_ATTR, not RTC_DATA_ATTR: the startup
// code clears .rtc.bss on every reset but a deep-sleep wake, so an RTC_DATA_ATTR variable would
// be zeroed by the very reboot that is meant to carry it. .rtc_noinit is never cleared, which
// is why the value is a magic word and is only honoured after a software reset: on a cold boot
// it holds whatever the RTC RAM held. It is cleared as soon as it is read.
static constexpr uint32_t OTA_REQUEST_MAGIC = 0x52455551u;  // "REQU"
static RTC_NOINIT_ATTR uint32_t s_boot_request;

static constexpr int RECV_CHUNK = 2048;            // bytes read from the socket per iteration
static constexpr uint32_t MIN_IMAGE_BYTES = 4096;  // anything smaller cannot be an app image
static constexpr int MAX_RECV_STALLS = 3;          // consecutive recv_wait_timeout expiries before giving up

static bool s_update_mode;                    // set once in ota_update_mode_start(), before the server exists
static volatile uint8_t s_phase;              // OtaPhase, the copy ota_tick() reads without the mutex
static volatile uint32_t s_last_activity_ms;  // page loads, a phone joining, upload progress
static volatile bool s_cancel_req;            // display task -> ota_tick()
static volatile uint32_t s_probe_reqs;        // event task only (log throttle)
static volatile bool s_ap_up;                 // event task -> ota_update_mode_start()'s wait
static uint32_t s_published_deadline;         // supervisor only
static httpd_handle_t s_httpd;

// ---------------------------------------------------------------- status
// Every field of g_state.ota is written under the state mutex, like every other producer's.
// A missed lock (50 ms) loses one screen update, never the phase ota_tick() acts on.
template <typename F>
static void publish(F &&fn) {
  if (state_lock()) {
    fn(g_state.ota);
    state_unlock();
  }
}

static void set_phase(uint8_t phase) {
  s_phase = phase;
  publish([phase](OtaStatus &o) { o.phase = phase; });
}

static void touch() { s_last_activity_ms = state_now_ms(); }

// ---------------------------------------------------------------- slots
static const char *img_state_name(esp_ota_img_states_t st) {
  switch (st) {
    case ESP_OTA_IMG_NEW: return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending-verify";
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_INVALID: return "invalid";
    case ESP_OTA_IMG_ABORTED: return "aborted";
    default: return "undefined";  // a wired flash: the empty otadata gives the slot no state at all
  }
}

// True while this image still has to prove itself, or the next reset rolls back. esp_ota_begin()
// refuses to overwrite the only valid fallback in that state, which is why update mode is
// refused then rather than failing at the first byte.
static bool image_on_probation() {
  const esp_partition_t *run = esp_ota_get_running_partition();
  esp_ota_img_states_t st;
  if (!run || esp_ota_get_state_partition(run, &st) != ESP_OK) return false;
  return st == ESP_OTA_IMG_PENDING_VERIFY;
}

[[noreturn]] static void reboot_normal() {
  s_boot_request = 0;
  vTaskDelay(pdMS_TO_TICKS(150));  // let the log drain
  esp_restart();
}

// ---------------------------------------------------------------- boot, request, probation
bool ota_boot_is_update_mode() {
  const bool requested = s_boot_request == OTA_REQUEST_MAGIC && esp_reset_reason() == ESP_RST_SW;
  s_boot_request = 0;  // one shot: a power cycle or any later reset comes up normally
  return requested;
}

void ota_log_boot_state() {
  const esp_partition_t *run = esp_ota_get_running_partition();
  if (!run) {
    ESP_LOGW(TAG, "no running partition reported");
    return;
  }
  esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
  esp_ota_get_state_partition(run, &st);  // fails for a wired flash: stays "undefined"
  ESP_LOGI(TAG, "running %s at 0x%06lx (%lu KB slot), image %s, version %s, %u app slots, hold %u ms for update mode",
           run->label, (unsigned long)run->address, (unsigned long)(run->size / 1024), img_state_name(st),
           esp_app_get_description()->version, (unsigned)esp_ota_get_app_partition_count(), (unsigned)OTA_LONG_PRESS_MS);
  if (st == ESP_OTA_IMG_PENDING_VERIFY)
    ESP_LOGW(TAG, "new image on probation: confirmed after %lu s with every task alive; a reset before that rolls back",
             (unsigned long)(OTA_CONFIRM_MS / 1000));
}

bool ota_update_mode_active() { return s_update_mode; }

OtaRefusal ota_request_update_mode() {
  if (s_update_mode) return OTA_OK;  // already there: the hold means nothing more
  const SharedState s = state_snapshot();
  const uint32_t now = state_now_ms();  // after the snapshot, as everywhere else
  OtaRefusal r = ota_entry_check(s, now);
  if (r == OTA_OK && esp_ota_get_app_partition_count() < 2) r = OTA_REFUSED_NO_SLOT;
  if (r == OTA_OK && image_on_probation()) r = OTA_REFUSED_UNCONFIRMED;
  if (r != OTA_OK) {
    ESP_LOGW(TAG, "update mode refused: %s", ota_refusal_text(r));
    return r;
  }
  ESP_LOGW(TAG, "entering update mode: rebooting");
  s_boot_request = OTA_REQUEST_MAGIC;
  vTaskDelay(pdMS_TO_TICKS(150));  // let the log drain
  esp_restart();
}

void ota_request_cancel() { s_cancel_req = true; }

void ota_tick(uint32_t now) {
  if (!s_update_mode) return;
  const uint8_t phase = s_phase;
  const bool busy = phase == OTA_PHASE_RECEIVING || phase == OTA_PHASE_DONE;
  if (s_cancel_req) {
    s_cancel_req = false;
    if (busy) {
      ESP_LOGW(TAG, "exit refused: an upload is being written");
    } else {
      ESP_LOGW(TAG, "update mode left from the button: rebooting");
      reboot_normal();
    }
  }
  // Never leave the access point up by accident; the timer stands still while an upload runs.
  if (busy) s_last_activity_ms = now;
  const uint32_t deadline = s_last_activity_ms + (uint32_t)OTA_IDLE_TIMEOUT_MS;
  if ((int32_t)(now - deadline) >= 0) {
    ESP_LOGW(TAG, "idle for %lu s: leaving update mode", (unsigned long)(OTA_IDLE_TIMEOUT_MS / 1000));
    reboot_normal();
  }
  if (deadline != s_published_deadline) {
    s_published_deadline = deadline;
    publish([deadline](OtaStatus &o) { o.idle_deadline_ms = deadline; });
  }
}

void ota_confirm_if_healthy(bool healthy, uint32_t now) {
  static bool s_done = false;
  if (s_done || !healthy || now < (uint32_t)OTA_CONFIRM_MS) return;
  s_done = true;
  if (!image_on_probation()) return;
  // One otadata sector write. The TWAI and RMT interrupts are not flagged IRAM, so they are
  // masked (not crashed into) for the ~50 ms the flash is busy and a few CAN frames may be
  // dropped; the dead-man task misses two or three ticks. Once per update, a minute after boot.
  const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err == ESP_OK)
    ESP_LOGW(TAG, "new image confirmed after %lu s with every task alive: rollback cancelled",
             (unsigned long)(OTA_CONFIRM_MS / 1000));
  else
    ESP_LOGE(TAG, "could not confirm the running image (%s): the next reset rolls back", esp_err_to_name(err));
}

// ---------------------------------------------------------------- HTTP server
static const char k_page[] =
    "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>vesc-dashboard update</title>"
    "<style>body{font:16px system-ui;margin:0;padding:24px;background:#111;color:#eee}"
    "h1{font-size:20px;margin:0 0 4px}p{color:#aaa;margin:4px 0 20px}"
    "input,button{font:inherit;width:100%;box-sizing:border-box;padding:12px;margin:6px 0}"
    "button{background:#fb0;border:0;border-radius:6px;font-weight:600}"
    "progress{width:100%;height:24px}#s{margin-top:14px}</style>"
    "<h1>vesc-dashboard</h1><p id=v></p>"
    "<input id=f type=file accept='.bin'><button onclick='go()'>Flash firmware</button>"
    "<progress id=p value=0 max=100 hidden></progress><div id=s></div>"
    "<script>"
    "fetch('/info').then(r=>r.text()).then(t=>v.textContent=t);"
    "function go(){var fl=f.files[0];if(!fl){s.textContent='Pick a .bin first';return;}"
    "p.hidden=false;var x=new XMLHttpRequest();x.open('POST','/update');"
    "x.upload.onprogress=function(e){if(e.lengthComputable)p.value=e.loaded*100/e.total;};"
    "x.onload=function(){s.textContent=x.responseText;};"
    "x.onerror=function(){s.textContent='Upload failed';};"
    "x.send(fl);}"
    "</script>";

static esp_err_t page_get(httpd_req_t *req) {
  touch();
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_sendstr(req, k_page);
}

static esp_err_t info_get(httpd_req_t *req) {
  touch();
  const esp_app_desc_t *app = esp_app_get_description();
  const esp_partition_t *run = esp_ota_get_running_partition();
  const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
  char buf[192];
  snprintf(buf, sizeof buf, "%s (%s) on %s: running from %s, will write %s (%lu KB slot)", app->version, FW_VERSION,
           BOARD_NAME, run ? run->label : "?", next ? next->label : "?", (unsigned long)(next ? next->size / 1024 : 0));
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, buf);
}

static esp_err_t fail(httpd_req_t *req, const char *why) {
  set_phase(OTA_PHASE_FAILED);
  publish([why](OtaStatus &o) {
    snprintf(o.detail, sizeof o.detail, "%s", why);
    o.new_ver[0] = '\0';
  });
  ESP_LOGW(TAG, "upload failed: %s", why);
  httpd_resp_set_status(req, "400 Bad Request");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, why);
}

// Static, not on the stack: the server runs a single worker, so one upload at a time.
static char s_recv_buf[RECV_CHUNK];

static esp_err_t update_post(httpd_req_t *req) {
  touch();
  const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
  if (!target) return fail(req, "no spare app slot");
  if (req->content_len == 0) return fail(req, "empty upload");
  const uint32_t total = (uint32_t)req->content_len;
  if (total > target->size) return fail(req, "larger than the slot");
  if (total < MIN_IMAGE_BYTES) return fail(req, "too small for firmware");

  esp_ota_handle_t h = 0;
  esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &h);
  if (err != ESP_OK) return fail(req, esp_err_to_name(err));

  ESP_LOGW(TAG, "receiving %lu KB into %s", (unsigned long)(total / 1024), target->label);
  set_phase(OTA_PHASE_RECEIVING);
  publish([total](OtaStatus &o) {
    o.received = 0;
    o.total = total;
    o.detail[0] = '\0';
    o.new_ver[0] = '\0';
  });

  uint32_t written = 0;
  size_t remaining = req->content_len;
  bool described = false;
  int stalls = 0;
  while (remaining > 0) {
    const int n = httpd_req_recv(req, s_recv_buf, remaining < (size_t)RECV_CHUNK ? remaining : (size_t)RECV_CHUNK);
    if (n == HTTPD_SOCK_ERR_TIMEOUT) {
      // Each call already waited recv_wait_timeout: a phone that walked out of range must not
      // hold the handler, and update mode, open for ever.
      if (++stalls > MAX_RECV_STALLS) {
        esp_ota_abort(h);
        return fail(req, "upload stalled");
      }
      continue;
    }
    stalls = 0;
    if (n <= 0) {
      esp_ota_abort(h);
      return fail(req, "connection dropped");
    }
    err = esp_ota_write(h, s_recv_buf, (size_t)n);
    if (err != ESP_OK) {
      esp_ota_abort(h);
      // esp_ota_write() rejects a first byte that is not the image magic 0xE9.
      return fail(req, err == ESP_ERR_OTA_VALIDATE_FAILED ? "not a firmware image" : esp_err_to_name(err));
    }
    written += (uint32_t)n;
    remaining -= (size_t)n;
    touch();
    publish([written](OtaStatus &o) { o.received = written; });

    // Once the app descriptor has landed: refuse anything that is not this project (an image for
    // another chip already fails esp_ota_end()), and show which version is arriving.
    if (!described &&
        written >= sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
      described = true;
      esp_app_desc_t incoming;
      const esp_app_desc_t *cur = esp_app_get_description();
      if (esp_ota_get_partition_description(target, &incoming) != ESP_OK ||
          strncmp(incoming.project_name, cur->project_name, sizeof incoming.project_name) != 0) {
        esp_ota_abort(h);
        return fail(req, "not vesc-dashboard fw");
      }
      ESP_LOGW(TAG, "image is %s (built %s %s), running %s", incoming.version, incoming.date, incoming.time,
               cur->version);
      publish([&incoming](OtaStatus &o) { snprintf(o.new_ver, sizeof o.new_ver, "%.17s", incoming.version); });
    }
  }

  err = esp_ota_end(h);  // verifies the image: checksum, SHA-256, chip id
  if (err != ESP_OK) return fail(req, err == ESP_ERR_OTA_VALIDATE_FAILED ? "image failed checks" : esp_err_to_name(err));
  err = esp_ota_set_boot_partition(target);
  if (err != ESP_OK) return fail(req, esp_err_to_name(err));

  set_phase(OTA_PHASE_DONE);
  ESP_LOGW(TAG, "wrote %lu KB to %s, rebooting into it", (unsigned long)(written / 1024), target->label);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "Flashed. The dashboard is rebooting into the new firmware.");
  vTaskDelay(pdMS_TO_TICKS(800));  // let the response reach the phone and the screen show DONE
  reboot_normal();
}

// ---------------------------------------------------------------- Wi-Fi
// WIFI_EVENT_AP_START, which arrives asynchronously after esp_wifi_start(), is the only
// affirmative statement that the access point is beaconing; the screen keys off it. A probe
// request proves the receive path works and that a phone scans this channel, which separates
// "we are not transmitting" from "nobody is looking".
static void wifi_event(void *, esp_event_base_t, int32_t id, void *data) {
  switch (id) {
    case WIFI_EVENT_AP_START:
      s_ap_up = true;
      publish([](OtaStatus &o) { o.ap_up = true; });
      ESP_LOGI(TAG, "access point up");
      break;
    case WIFI_EVENT_AP_STOP:
      s_ap_up = false;
      publish([](OtaStatus &o) { o.ap_up = false; });
      ESP_LOGW(TAG, "access point stopped");
      break;
    case WIFI_EVENT_AP_STACONNECTED:
      touch();
      publish([](OtaStatus &o) { o.stations++; });
      ESP_LOGI(TAG, "phone joined (aid %u)", (unsigned)((const wifi_event_ap_staconnected_t *)data)->aid);
      break;
    case WIFI_EVENT_AP_STADISCONNECTED:
      publish([](OtaStatus &o) {
        if (o.stations) o.stations--;
      });
      ESP_LOGI(TAG, "phone left");
      break;
    case WIFI_EVENT_AP_PROBEREQRECVED:
      if (s_probe_reqs < 5)  // only the first few: a scanning phone would flood the log
        ESP_LOGI(TAG, "probe request, rssi %d", ((const wifi_event_ap_probe_req_rx_t *)data)->rssi);
      s_probe_reqs = s_probe_reqs + 1;
      break;
    default: break;
  }
}

// httpd_register_uri_handler() copies the descriptor and the path.
static esp_err_t add_uri(const char *uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *)) {
  httpd_uri_t u;
  memset(&u, 0, sizeof u);
  u.uri = uri;
  u.method = method;
  u.handler = handler;
  return httpd_register_uri_handler(s_httpd, &u);
}

// A fresh passphrase per session, shown on the screen and never stored. Digits only: it is read
// off a small display and typed on a phone. WPA2 needs at least eight characters.
static void make_passphrase(char *out, size_t len) {
  for (size_t i = 0; i + 1 < len; ++i) out[i] = (char)('0' + esp_random() % 10u);
  out[len - 1] = '\0';
}

#define OTA_CHECK(x, what)                                            \
  do {                                                                \
    const esp_err_t e_ = (x);                                         \
    if (e_ != ESP_OK) {                                               \
      ESP_LOGE(TAG, "%s: %s", what, esp_err_to_name(e_));             \
      return false;                                                   \
    }                                                                 \
  } while (0)

bool ota_update_mode_start() {
  s_update_mode = true;
  touch();
  char pass[9];
  make_passphrase(pass, sizeof pass);
  publish([&pass](OtaStatus &o) {
    memset(&o, 0, sizeof o);
    snprintf(o.ssid, sizeof o.ssid, "%s", OTA_AP_SSID);
    memcpy(o.pass, pass, sizeof o.pass);
    snprintf(o.run_ver, sizeof o.run_ver, "%.17s", esp_app_get_description()->version);
  });
  set_phase(OTA_PHASE_STARTING);

  if (!nvs_bring_up(TAG)) return false;  // the PHY calibration lives there
  OTA_CHECK(esp_netif_init(), "netif init");
  OTA_CHECK(esp_event_loop_create_default(), "event loop");
  esp_netif_t *netif = esp_netif_create_default_wifi_ap();
  if (!netif) {
    ESP_LOGE(TAG, "no AP netif");
    return false;
  }
  OTA_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, nullptr), "wifi events");

  // Force a full RF calibration by throwing the cached one away: the blob is only checked for
  // integrity (format, MAC, length), never for correctness, so one taken under a sagging supply
  // is reused on every later boot and looks exactly like a dead radio. ~100 ms, in a rare mode.
  const esp_err_t cal = esp_phy_erase_cal_data_in_nvs();
  ESP_LOGI(TAG, "cached RF calibration %s, forcing a full one", cal == ESP_OK ? "erased" : "could not be erased");

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  OTA_CHECK(esp_wifi_init(&cfg), "wifi init");
  OTA_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM), "wifi storage");
  OTA_CHECK(esp_wifi_set_event_mask(WIFI_EVENT_MASK_NONE), "wifi event mask");  // unmask probe requests

  wifi_config_t wc;
  memset(&wc, 0, sizeof wc);
  const size_t ssid_len = strlen(OTA_AP_SSID);
  memcpy(wc.ap.ssid, OTA_AP_SSID, ssid_len);
  wc.ap.ssid_len = (uint8_t)ssid_len;
  memcpy(wc.ap.password, pass, sizeof pass);
  wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
  wc.ap.max_connection = 1;
  wc.ap.channel = 1;
  OTA_CHECK(esp_wifi_set_mode(WIFI_MODE_AP), "wifi mode");
  OTA_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc), "wifi config");
  OTA_CHECK(esp_wifi_start(), "wifi start");
  const esp_err_t pw = esp_wifi_set_max_tx_power((int8_t)OTA_AP_TX_POWER_QDBM);
  if (pw != ESP_OK) ESP_LOGW(TAG, "tx power not set: %s", esp_err_to_name(pw));

  esp_netif_ip_info_t ip;
  memset(&ip, 0, sizeof ip);
  OTA_CHECK(esp_netif_get_ip_info(netif, &ip), "netif ip");
  char ip_str[16];
  snprintf(ip_str, sizeof ip_str, IPSTR, IP2STR(&ip.ip));

  // Priority 1, like the supervisor: on the single-core C6 an upload that arrives faster than
  // the flash takes it would otherwise run the handler back to back, starve the supervisor and
  // let the task watchdog fire mid-write. Time slicing shares the core; the display (2) and
  // the Wi-Fi / lwIP tasks stay above it.
  httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
  hc.task_priority = 1;
  hc.core_id = TASK_CORE_APP;
  hc.stack_size = 6144;
  hc.max_uri_handlers = 4;
  hc.lru_purge_enable = true;
  hc.recv_wait_timeout = 20;
  hc.send_wait_timeout = 20;
  OTA_CHECK(httpd_start(&s_httpd, &hc), "httpd start");
  OTA_CHECK(add_uri("/", HTTP_GET, page_get), "uri /");
  OTA_CHECK(add_uri("/info", HTTP_GET, info_get), "uri /info");
  OTA_CHECK(add_uri("/update", HTTP_POST, update_post), "uri /update");

  publish([&ip_str](OtaStatus &o) { memcpy(o.ip, ip_str, sizeof o.ip); });

  // AP_START is posted from the Wi-Fi task: give it a moment. ESP_OK from everything above does
  // not mean the radio is beaconing; without it update mode stays up so the screen can say so.
  for (int i = 0; i < 20 && !s_ap_up; ++i) vTaskDelay(pdMS_TO_TICKS(50));
  set_phase(OTA_PHASE_WAITING);
  if (!s_ap_up) {
    ESP_LOGE(TAG, "*** the access point did NOT come up: check the supply, try a lower OTA_AP_TX_POWER_QDBM ***");
    return true;
  }
  ESP_LOGW(TAG, "update mode: join \"%s\" with passphrase %s, then open http://%s", OTA_AP_SSID, pass, ip_str);
  return true;
}

#endif  // OTA_ENABLE
