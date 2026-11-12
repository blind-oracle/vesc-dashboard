// NVS bring-up shared by its three users: the lifetime totals (src/totals.cpp, the record that
// survives power cycles), the BLE controller (src/ble_scan.cpp), which stores its PHY
// calibration at the first enable, and update mode's Wi-Fi (src/ota.cpp). nvs_flash_init() is
// idempotent, so whoever comes first initialises it. A partition that is full or was written
// by a newer NVS version is erased and initialised again. That loses the lifetime totals too
// (src/totals.cpp then logs "no saved record"), but neither case happens with what this
// firmware stores: one 24-byte record and one calibration blob in 20 KB, on IDF 6.
// IDF header: firmware sources only, never the native tests.
#pragma once

#include <esp_log.h>
#include <nvs_flash.h>

inline bool nvs_bring_up(const char *tag) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(tag, "nvs: erasing (%s)", esp_err_to_name(err));
    if (nvs_flash_erase() == ESP_OK) err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    ESP_LOGE(tag, "nvs_flash_init: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}
