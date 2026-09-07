// Trip / efficiency integrator task.
//
// Every TRIP_PERIOD_MS: take a state snapshot, advance the pure integrator in
// include/trip.h (distance from GNSS ground speed, energy from the VESC
// watt-hour counters or the v_in x current_in fallback, "now" window and trip
// averages) and publish the result into g_state.trip for the EFFICIENCY screen
// and the logs. The task owns no peripheral and is not watched by the
// supervisor (nothing depends on it for safety); it sleeps with vTaskDelay
// between ticks, so it can never busy-loop.
//
// Rules kept: no logging or I/O under state_lock() (the locked region is one
// struct copy), timestamps are state_now_ms() with 0 = never.
#include "trip.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <math.h>
#include <stdio.h>

#include "config.h"
#include "shared_state.h"

static const char *TAG = "trip";

// Local fallbacks for tunables that config.h does not (yet) define.
#ifndef TASK_PRIO_TRIP
#define TASK_PRIO_TRIP 3       // between the display (TASK_PRIO_DISP 2) and the data producers (GNSS 5, CAN 6)
#endif
#ifndef TRIP_TASK_STACK
#define TRIP_TASK_STACK 4096   // bytes (ESP-IDF's xTaskCreate() takes bytes); holds one SharedState snapshot (~0.5 KB)
#endif

namespace {

TripCalc s_calc;
bool s_started = false;

void trip_task(void *) {
  ESP_LOGI(TAG, "trip: task started (period %u ms, window %u s, unit %s, min speed %u mm/s)", (unsigned)TRIP_PERIOD_MS,
           (unsigned)EFF_WINDOW_S, EFF_UNIT_STR, (unsigned)EFF_MIN_SPEED_MM_S);
  trip_calc_init(s_calc);
  for (;;) {
    // Snapshot BEFORE reading the clock so every timestamp inside s is <= now
    // (a frame landing between the two reads must not look like a wrapped age).
    const SharedState s = state_snapshot();
    const uint32_t now = state_now_ms();
    TripState tmp;
    trip_calc_update(s_calc, s, now, tmp);
    if (state_lock()) {  // lock timeout (counted by state_lock): this tick's result is published next time
      g_state.trip = tmp;
      state_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(TRIP_PERIOD_MS));
  }
}

// "--" for an invalid efficiency, otherwise e.g. "103Wh/NM" (rounded, -0 normalised).
void fmt_eff(char *buf, size_t n, float eff, bool valid) {
  if (!valid) {
    snprintf(buf, n, "--");
    return;
  }
  if (fabsf(eff) < 0.5f) eff = 0.0f;
  snprintf(buf, n, "%.0f%s", (double)eff, EFF_UNIT_STR);
}

}  // namespace

bool trip_start() {
  if (s_started) return true;
  if (xTaskCreate(trip_task, "trip", TRIP_TASK_STACK, nullptr, TASK_PRIO_TRIP, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "trip: xTaskCreate failed");
    return false;
  }
  s_started = true;
  return true;
}

void trip_log_summary(const SharedState &s, uint32_t now) {
  const TripState &t = s.trip;
  char eff_now[24], eff_avg[24];
  fmt_eff(eff_now, sizeof eff_now, t.eff_now, t.eff_now_valid);
  fmt_eff(eff_avg, sizeof eff_avg, t.eff_avg, t.eff_avg_valid);
  // Nobody else watches this task: flag a state that stopped updating.
  char stale[32] = "";
  if (t.t_ms == 0) {
    snprintf(stale, sizeof stale, " (never updated)");
  } else if ((uint32_t)(now - t.t_ms) > 5u * (uint32_t)TRIP_PERIOD_MS) {
    snprintf(stale, sizeof stale, " (stale %lums)", (unsigned long)(now - t.t_ms));
  }
  ESP_LOGI(TAG, "TRIP run=%lus moving=%lus dist=%.3f%s wh=%.1f whc=%.1f now=%s avg=%s win=%us P=%.0fW src=%s resets=%lu%s",
           (unsigned long)t.run_s, (unsigned long)t.moving_s, (double)(t.dist_m / EFF_DIST_UNIT_M), EFF_DIST_UNIT_STR,
           (double)t.wh, (double)t.wh_charged, eff_now, eff_avg, (unsigned)t.win_fill_s, (double)t.win_p_avg_w,
           t.energy_from_counters ? "counters" : "integ", (unsigned long)t.counter_resets, stale);
}
