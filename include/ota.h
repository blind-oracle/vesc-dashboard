// Firmware update over the air: public entry points of src/ota.cpp.
//
// Holding the button for OTA_LONG_PRESS_MS reboots into UPDATE MODE: the dashboard brings up
// its own WPA2 access point, serves a small upload page and writes whatever image is posted
// to the spare app slot; then it reboots into it. The phone side is any browser.
//
// Rebooting into update mode, rather than starting Wi-Fi next to everything else, is
// deliberate: the update-mode boot starts only the OLED, the button and the Wi-Fi, so no
// radio teardown code exists, the heap is at its largest, and nothing that is not cache-safe
// (the TWAI ISR, the BLE controller) runs while flash sectors are erased. The request crosses
// the reboot in RTC memory, so deciding to write flash involves no flash write of its own.
//
// Framework-free header (only <stdbool.h>/<stdint.h> + project headers) so main.cpp and the
// display task can include it. With OTA_ENABLE 0 every function is an inline no-op, so
// callers need no #if.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "ota_logic.h"

#if OTA_ENABLE
// True when this boot was asked for by ota_request_update_mode(). One shot: it clears the
// request, so a power cycle always comes up normally. Call it once, first thing in setup().
bool ota_boot_is_update_mode();

// Logs the running slot, its OTA state and the image version.
void ota_log_boot_state();

// Brings up the access point and the upload server. Only after ota_boot_is_update_mode()
// returned true, and after state_init(). Returns false if Wi-Fi or the server could not be
// started (the caller then reboots to normal operation).
bool ota_update_mode_start();

// True while this boot is serving updates.
bool ota_update_mode_active();

// Asks for update mode (from the display task, on the long hold): applies ota_entry_check()
// to a fresh snapshot plus the flash checks, then reboots. Returns only when refused.
OtaRefusal ota_request_update_mode();

// Asks update mode to end (a long press while the FW UPDATE screen is shown). Handled by
// ota_tick(), which refuses it while an upload is being written.
void ota_request_cancel();

// Update-mode housekeeping, every supervisor loop: a pending cancel, the idle timeout
// (OTA_IDLE_TIMEOUT_MS, paused while an upload runs) and the countdown on the screen.
void ota_tick(uint32_t now);

// Normal-mode housekeeping, every supervisor loop: once the image has run OTA_CONFIRM_MS with
// `healthy` true (every task heartbeat fresh), a probationary image is marked valid and the
// rollback is cancelled. Until then a crash, a watchdog reset or a power cycle boots the
// previous image again.
void ota_confirm_if_healthy(bool healthy, uint32_t now);
#else
inline bool ota_boot_is_update_mode() { return false; }
inline void ota_log_boot_state() {}
inline bool ota_update_mode_start() { return false; }
inline bool ota_update_mode_active() { return false; }
inline OtaRefusal ota_request_update_mode() { return OTA_REFUSED_NO_SLOT; }
inline void ota_request_cancel() {}
inline void ota_tick(uint32_t) {}
inline void ota_confirm_if_healthy(bool, uint32_t) {}
#endif
