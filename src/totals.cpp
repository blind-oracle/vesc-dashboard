// Lifetime totals: the NVS record behind include/totals.h.
//
//   * totals_load()  app_main, once, before the BLE controller and the CAN node exist: brings
//                    NVS up (nvs_bring_up is idempotent; NVS init may finish an interrupted
//                    write), opens namespace "totals" and reads key "rec". Logs the outcome.
//   * totals_tick()  the trip task, every TRIP_PERIOD_MS, outside the state lock: the pure
//                    decision (totals_step), the write when one is due (nvs_set_blob +
//                    nvs_commit, ~1-3 ms with the flash cache off; a sector erase every ~145
//                    saves takes ~50 ms), and the TotalsState the trip task publishes. No logging.
//   * totals_log_*() the supervisor: a debug line per save, an error line per failed one, and
//                    the periodic summary.
//
// NVS is power-loss safe: a write cut short leaves the previous record valid. When NVS cannot
// be brought up, or the record is unreadable, the totals run in RAM only and nothing is ever
// written, so a record this firmware does not understand is never overwritten.
#include "totals.h"

#if TOTALS_ENABLE

#include <esp_log.h>
#include <nvs.h>

#include <stdio.h>

#include "nvs_util.h"

static const char *TAG = "totals";
static const char *kNamespace = "totals";
static const char *kKey = "rec";

namespace {

bool s_loaded = false;
bool s_have_handle = false;
nvs_handle_t s_nvs = 0;
uint8_t s_nvs_state = TOTALS_NVS_ERR;
TotalsCalc s_calc;
// Counters published with every tick (the trip task owns them after boot).
uint32_t s_last_save_ms = 0, s_saves = 0, s_errors = 0;
int32_t s_last_err = 0;
uint8_t s_last_reason = TOTALS_SAVE_NONE;

void fmt_record(char *buf, size_t n, const TotalsRecord &r) {
  snprintf(buf, n, "motor=%.1fh powered=%.1fh dist=%.1f%s net=%.2fkWh (drawn %luWh, regen %luWh)",
           (double)r.motor_s / 3600.0, (double)r.on_s / 3600.0, (double)r.dist_m / (double)EFF_DIST_UNIT_M,
           EFF_DIST_UNIT_STR, ((double)r.wh - (double)r.whc) / 1000.0, (unsigned long)r.wh, (unsigned long)r.whc);
}

// The stored blob: NONE when there is none, OK with *rec filled, ERR (logged) otherwise.
uint8_t read_record(TotalsRecord *rec) {
  size_t len = 0;
  esp_err_t err = nvs_get_blob(s_nvs, kKey, nullptr, &len);
  if (err == ESP_ERR_NVS_NOT_FOUND) return TOTALS_NVS_NONE;
  if (err != ESP_OK || len == 0 || len > kTotalsBlobMax) {
    ESP_LOGE(TAG, "record unreadable (%s, %u bytes)", esp_err_to_name(err), (unsigned)len);
    return TOTALS_NVS_ERR;
  }
  uint8_t buf[kTotalsBlobMax];
  err = nvs_get_blob(s_nvs, kKey, buf, &len);
  uint32_t version = 0;
  if (err != ESP_OK || !totals_decode(buf, len, *rec, &version)) {
    ESP_LOGE(TAG, "record unreadable (%s, %u bytes, version %lu)", esp_err_to_name(err), (unsigned)len,
             (unsigned long)version);
    return TOTALS_NVS_ERR;
  }
  if (version > kTotalsVersion)
    ESP_LOGW(TAG, "record v%lu was written by newer firmware: reading its first %u fields, the next save writes v%lu",
             (unsigned long)version, (unsigned)kTotalsFields, (unsigned long)kTotalsVersion);
  return TOTALS_NVS_OK;
}

}  // namespace

void totals_load() {
  if (s_loaded) return;
  const uint32_t now = state_now_ms();
  TotalsRecord rec{};
  if (!nvs_bring_up(TAG)) {
    s_nvs_state = TOTALS_NVS_ERR;
  } else {
    const esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &s_nvs);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_open(%s): %s", kNamespace, esp_err_to_name(err));
      s_nvs_state = TOTALS_NVS_ERR;
    } else {
      s_have_handle = true;
      s_nvs_state = read_record(&rec);
    }
  }
  totals_init(s_calc, rec, s_nvs_state == TOTALS_NVS_OK, now);
  s_loaded = true;

  char line[160];
  fmt_record(line, sizeof line, s_calc.base);
  switch (s_nvs_state) {
    case TOTALS_NVS_OK: ESP_LOGI(TAG, "totals: %s (record v%lu)", line, (unsigned long)kTotalsVersion); break;
    case TOTALS_NVS_NONE:
      ESP_LOGW(TAG, "totals: no saved record (first boot, or the NVS partition was erased): starting at %s", line);
      break;
    default:
      ESP_LOGE(TAG, "totals: NVS unavailable, counting in RAM only and never writing (this boot only): %s", line);
      break;
  }
  if (totals_preset_s() > 0)
    ESP_LOGI(TAG, "totals: motor preset %.1fh (TOTALS_MOTOR_PRESET_H) %s", (double)totals_preset_s() / 3600.0,
             s_calc.base.motor_s > s_calc.saved.motor_s ? "applied" : "below the saved hours: no effect");
}

void totals_tick(const TripState &t, uint32_t now, TotalsState *out) {
  if (!s_loaded || out == nullptr) return;
  TotalsRecord rec;
  const uint8_t why = totals_step(s_calc, t, now, rec);
  if (why != TOTALS_SAVE_NONE && s_have_handle && s_nvs_state != TOTALS_NVS_ERR) {
    uint8_t buf[kTotalsBlobSize];
    totals_encode(rec, buf);
    esp_err_t err = nvs_set_blob(s_nvs, kKey, buf, sizeof buf);
    if (err == ESP_OK) err = nvs_commit(s_nvs);
    const uint32_t done = state_now_ms();  // after the flash write: the gap counts from its end
    totals_done(s_calc, rec, err == ESP_OK, done);
    s_last_reason = why;
    if (err == ESP_OK) {
      s_saves++;
      s_last_save_ms = done ? done : 1u;
    } else {
      s_errors++;
      s_last_err = (int32_t)err;
    }
  }
  TotalsState &o = *out;
  o = TotalsState{};
  o.t_ms = now ? now : 1u;
  o.life = rec;
  o.last_save_ms = s_last_save_ms;
  o.saves = s_saves;
  o.errors = s_errors;
  o.last_err = s_last_err;
  o.nvs = s_nvs_state;
  o.last_reason = s_last_reason;
  o.dirty = !totals_equal(rec, s_calc.saved);
  o.failed = s_calc.failed;
}

void totals_log_events(const SharedState &s, uint32_t now) {
  (void)now;
  static uint32_t s_seen_saves = 0, s_seen_errors = 0;
  const TotalsState &o = s.totals;
  if (o.saves != s_seen_saves) {
    s_seen_saves = o.saves;
    char line[160];
    fmt_record(line, sizeof line, o.life);
    ESP_LOGD(TAG, "totals saved (%s, #%lu): %s", totals_reason_str(o.last_reason), (unsigned long)o.saves, line);
  }
  if (o.errors != s_seen_errors) {
    s_seen_errors = o.errors;
    ESP_LOGE(TAG, "totals: save failed (%s, %s): kept in RAM, next try in %u s (%lu failures)",
             esp_err_to_name((esp_err_t)o.last_err), totals_reason_str(o.last_reason), (unsigned)TOTALS_RETRY_S,
             (unsigned long)o.errors);
  }
}

void totals_log_summary(const SharedState &s, uint32_t now) {
  const TotalsState &o = s.totals;
  if (o.t_ms == 0) {
    ESP_LOGW(TAG, "TOTALS not published yet (trip task not running)");
    return;
  }
  char line[160], last[40];
  fmt_record(line, sizeof line, o.life);
  if (o.last_save_ms) snprintf(last, sizeof last, "%lus ago (%s)", (unsigned long)((now - o.last_save_ms) / 1000u),
                               totals_reason_str(o.last_reason));
  else snprintf(last, sizeof last, "none this boot");
  ESP_LOGI(TAG, "TOTALS %s | boot mot=%lus on=%lus | nvs=%s saves=%lu err=%lu last=%s%s%s", line,
           (unsigned long)s.trip.motor_s, (unsigned long)s.trip.on_s, totals_nvs_str(o.nvs), (unsigned long)o.saves,
           (unsigned long)o.errors, last, o.dirty ? " unsaved" : "", o.failed ? " FAILED" : "");
}

#else  // !TOTALS_ENABLE

void totals_load() {}
void totals_tick(const TripState &, uint32_t, TotalsState *) {}
void totals_log_events(const SharedState &, uint32_t) {}
void totals_log_summary(const SharedState &, uint32_t) {}

#endif  // TOTALS_ENABLE
