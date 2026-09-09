#include "shared_state.h"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <string.h>

SharedState g_state{};

volatile uint32_t hb_can = 0;
volatile uint32_t hb_gnss = 0;
volatile uint32_t hb_disp = 0;
volatile uint32_t hb_bms = 0;

static SemaphoreHandle_t s_mutex = nullptr;
static SharedState s_last_snapshot{};

void state_init() {
  if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
  memset(&g_state, 0, sizeof(g_state));
  g_state.vesc.locked_id = -1;
  g_state.can.state = CAN_STATE_UNINSTALLED;
  g_state.gnss.prot_ver_x100 = -1;
  g_state.gnss.phase = GNSS_PHASE_AUTOBAUD;
}

bool state_lock(uint32_t timeout_ms) {
  if (!s_mutex) return false;
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) return true;
  // Counted without the lock: a lost increment here is acceptable, a deadlock is not.
  g_state.lock_failures++;
  return false;
}

void state_unlock() {
  if (s_mutex) xSemaphoreGive(s_mutex);
}

SharedState state_snapshot() {
  // Copy into the caller's object while the lock is held: two consumers (the
  // display task and the supervisor) share s_last_snapshot, and a copy taken from
  // it after unlocking can be preempted mid-memcpy by the other consumer
  // refreshing it, handing back a torn mix of two snapshots.
  SharedState out;
  if (state_lock(50)) {
    s_last_snapshot = g_state;
    out = s_last_snapshot;
    state_unlock();
  } else {
    out = s_last_snapshot;  // lock timeout (counted): best effort, previous snapshot
  }
  return out;
}

// Milliseconds since boot, truncated to 32 bits (wraps after ~49.7 days, exactly like
// Arduino's millis() did). Every age in the project is an unsigned difference, so the
// wrap is harmless; 0 stays reserved for "never" by the callers that need it.
uint32_t state_now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }
