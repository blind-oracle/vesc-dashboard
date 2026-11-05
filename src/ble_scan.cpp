// BLE radio for the dead-man's switch: the NimBLE host (observer role only) and the ble task
// that keeps one passive scan running for the crew's tags.
//
// Two tasks touch this file:
//   * the NimBLE host task (prio 21, Kconfig) runs gap_cb, on_sync and on_reset. gap_cb
//     timestamps a matching tag straight into the dead-man (deadman_beacon_seen(): volatile
//     stores) and counts every other advert; apart from that the callbacks only wake the ble
//     task. No logging, no mutex, no heap, no NimBLE calls;
//   * the ble task ("ble", TASK_PRIO_BLE) owns EVERY NimBLE API call and all logging: on each
//     new host sync it settles the own address, and once per loop (TASK_POLL_MS, or at once when
//     woken) it makes sure the scan is running - self-healing after a host reset, a controller
//     hiccup or a scan that ended for any other reason.
//
// The scan: passive, DEADMAN_SCAN_ITVL_MS / DEADMAN_SCAN_WINDOW_MS, no duplicate filter (a
// liveness watchdog needs EVERY advert, not one per scan instance), BLE_HS_FOREVER. Nothing else
// ever uses the radio - no connections, no advertising - so nothing interrupts the scan.
//
// On the dual-core S3 both tasks run on TASK_CORE_RADIO (core 0) and the dead-man task on core 1.
// What crosses the cores is single-writer, 32 bits or narrower and volatile (Xtensa GCC brackets
// volatile accesses with memw), and the dead-man reads every age with signed arithmetic.
#include "ble_scan.h"

#if DEADMAN_ENABLE

#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <host/ble_hs_id.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <nvs_flash.h>

#include "ble_addr.h"
#include "deadman.h"  // deadman_beacon_seen() / deadman_adv_other() from the host task
#include "shared_state.h"

static const char *TAG = "ble";

static constexpr uint32_t TASK_POLL_MS = 100;  // ensure_scan() period while nothing wakes the task

// Parsed once in ble_scan_start(), before the host task exists, then read-only: the host task
// compares every advert against it without a lock.
static uint8_t s_tag_addr[DEADMAN_TAGS][6];  // all-zero = that slot is inert and can never match

// Host task -> ble task (written only by on_sync / on_reset).
static volatile bool s_synced = false;
static volatile uint32_t s_sync_gen = 0;  // +1 per sync: a reset + sync between two loops is still seen
static volatile int s_reset_reason = 0;
// ble task -> dead-man task, through ble_scan_active(). A volatile bool keeps every NimBLE call
// inside the ble task, which is the invariant this file is built on.
static volatile bool s_scanning = false;

static TaskHandle_t s_task = nullptr;
static uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static uint32_t s_heap_before = 0;

static inline uint16_t ms_to_0625(uint32_t ms) { return (uint16_t)(ms * 1000u / 625u); }

// ---------------------------------------------------------------- NimBLE callbacks (host task)
static void wake() {
  if (s_task != nullptr) xTaskNotifyGive(s_task);
}

static int gap_cb(struct ble_gap_event *event, void *) {
  switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
      const struct ble_gap_disc_desc &d = event->disc;
      // Legacy PDU types a beacon uses: ADV_NONCONN_IND / SCAN_IND are what most of them send;
      // DIR_IND is never addressed to us. (Extended-advertising beacons are invisible either
      // way: CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT is off.)
      if (d.event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND && d.event_type != BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP &&
          d.event_type != BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND && d.event_type != BLE_HCI_ADV_RPT_EVTYPE_SCAN_IND)
        break;
      // The tag is timestamped HERE, in the host task, so the safety timestamp never waits
      // for another task. An unconfigured slot is all-zero, so it never matches a real advert.
      for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
        if (!ble_addr_equal(d.addr.val, s_tag_addr[i])) continue;
        deadman_beacon_seen((uint8_t)i, d.rssi, state_now_ms());
        return 0;
      }
      deadman_adv_other();
      break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
      wake();  // the scan ended (it runs forever, so something cancelled it): restart it now
      break;
    default: break;
  }
  return 0;
}

static void on_sync() {
  s_sync_gen = s_sync_gen + 1;
  s_synced = true;
  wake();
}

static void on_reset(int reason) {
  s_reset_reason = reason;
  s_synced = false;
  wake();
}

static void host_task(void *) {
  nimble_port_run();  // returns only after nimble_port_stop()
  nimble_port_freertos_deinit();
}

// ---------------------------------------------------------------- ble task
static void settle_addr() {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) ESP_LOGW(TAG, "ble_hs_util_ensure_addr rc=%d", rc);
  rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d, using the public address", rc);
    s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
  }
  uint8_t own[6] = {};
  char mac[18];
  (void)ble_hs_id_copy_addr(s_own_addr_type, own, nullptr);
  ble_addr_str(own, mac);
  ESP_LOGI(TAG, "host synced, own addr %s (type %u), heap %lu (was %lu before BLE init)", mac,
           (unsigned)s_own_addr_type, (unsigned long)esp_get_free_heap_size(), (unsigned long)s_heap_before);
}

// Called once per loop, so it is self-healing: it restarts a scan that a host reset, a
// controller hiccup or anything else left stopped. ble_gap_disc() returns EALREADY when one is
// already running, which is success as far as we are concerned.
static void ensure_scan() {
  if (ble_gap_disc_active()) {
    s_scanning = true;
    return;
  }
  struct ble_gap_disc_params p = {};
  p.itvl = ms_to_0625(DEADMAN_SCAN_ITVL_MS);
  p.window = ms_to_0625(DEADMAN_SCAN_WINDOW_MS);
  p.passive = 1;            // the tags are known by address: no need to ask for scan responses
  p.filter_duplicates = 0;  // a liveness watchdog needs EVERY advert, not one per scan instance
  const int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &p, gap_cb, nullptr);
  if (rc == 0 || rc == BLE_HS_EALREADY) {
    if (!s_scanning)
      ESP_LOGI(TAG, "watchdog scan up (%u/%u ms, passive, no duplicate filter)", (unsigned)DEADMAN_SCAN_ITVL_MS,
               (unsigned)DEADMAN_SCAN_WINDOW_MS);
    s_scanning = true;
  } else if (s_scanning) {
    // The dead-man task sees this within a tick and goes NO RADIO: no protection, but it
    // must not cut for our own failure - the mechanical kill switch still covers that.
    ESP_LOGE(TAG, "watchdog scan will not start (rc=%d): the dead-man is blind", rc);
    s_scanning = false;
  }
}

static void ble_task(void *) {
  uint32_t gen = 0;  // last host sync acted on
  bool was_synced = false;
  for (;;) {
    hb_touch(hb_ble);
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TASK_POLL_MS));
    if (!s_synced) {
      if (was_synced) ESP_LOGW(TAG, "host reset reason=%d: scan down, waiting for sync", s_reset_reason);
      was_synced = false;
      s_scanning = false;
      continue;
    }
    was_synced = true;
    const uint32_t g = s_sync_gen;
    if (g != gen) {
      gen = g;
      s_scanning = false;  // a new host: whatever ran before is gone
      settle_addr();
    }
    ensure_scan();
  }
}

// ---------------------------------------------------------------- public API
bool ble_scan_start() {
  s_heap_before = esp_get_free_heap_size();
  // Parsed here, before nimble_port_freertos_init() creates the host task, so gap_cb can
  // compare against them without a lock or an initialisation race. A bad tag address is
  // fatal for the dead-man (deadman_start() refuses and holds the cut), so only warn here.
  {
    const char *const tag_addr[DEADMAN_TAGS] = {DEADMAN_BEACON_ADDR, DEADMAN_BEACON_ADDR2};
    for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
      if (tag_addr[i][0] == '\0') continue;  // slot not configured: left all-zero, never matches
      if (!ble_addr_parse(tag_addr[i], s_tag_addr[i]))
        ESP_LOGE(TAG, "tag %u address \"%s\" is unusable: no advert can ever match it", i + 1u, tag_addr[i]);
    }
  }
  // The controller stores its PHY calibration in NVS at the first enable.
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "nvs: erasing (%s)", esp_err_to_name(err));
    if (nvs_flash_erase() == ESP_OK) err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init: %s", esp_err_to_name(err));
    return false;
  }
  // The task exists before the host does, so the first sync can always wake it.
  if (xTaskCreatePinnedToCore(ble_task, "ble", BLE_TASK_STACK, nullptr, TASK_PRIO_BLE, &s_task, TASK_CORE_RADIO) !=
      pdPASS) {
    ESP_LOGE(TAG, "ble task create failed");
    return false;
  }
  err = nimble_port_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err));
    return false;  // the task keeps waiting for a sync that never comes: ble_scan_active() stays false
  }
  ble_hs_cfg.reset_cb = on_reset;
  ble_hs_cfg.sync_cb = on_sync;
  nimble_port_freertos_init(host_task);
  ESP_LOGI(TAG, "BLE init ok, heap %lu -> %lu", (unsigned long)s_heap_before, (unsigned long)esp_get_free_heap_size());
  return true;
}

bool ble_scan_active() { return s_scanning; }

#endif  // DEADMAN_ENABLE
