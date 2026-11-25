// Peaks since boot: the supervisor's log line. The recorder itself is pure (include/peaks.h)
// and runs inside the producers' locked writes; nothing here touches g_state.
#include "peaks.h"

#include <esp_log.h>

#include <stdio.h>

static const char *TAG = "peaks";

namespace {

// "%.<dec>f<unit>" when the field holds a value, "--" otherwise.
void fmt_peak(char *buf, size_t n, bool have, float v, int dec, const char *unit) {
  if (have) snprintf(buf, n, "%.*f%s", dec, (double)v, unit);
  else snprintf(buf, n, "--");
}

}  // namespace

void peaks_log_summary(const SharedState &s) {
  const PeaksState &p = s.peaks;
  char pw[16], ib[16], im[16], rpm[16], tf[16], tm[16], spd[24];
  fmt_peak(pw, sizeof pw, p.have & PEAK_POWER, p.power_w, 0, "W");
  fmt_peak(ib, sizeof ib, p.have & PEAK_IBAT, p.i_bat_a, 1, "A");
  fmt_peak(im, sizeof im, p.have & PEAK_IMOT, p.i_mot_a, 1, "A");
  fmt_peak(rpm, sizeof rpm, p.have & PEAK_RPM, p.rpm, 0, "");
  fmt_peak(tf, sizeof tf, p.have & PEAK_TFET, p.temp_fet_c, 1, "C");
  fmt_peak(tm, sizeof tm, p.have & PEAK_TMOT, p.temp_mot_c, 1, "C");
  fmt_peak(spd, sizeof spd, p.have & PEAK_SPEED, (float)p.speed_mm_s * SPEED_FACTOR, 1, SPEED_UNIT_STR);
  ESP_LOGI(TAG, "PEAKS P=%s Ibat=%s Imot=%s rpm=%s Tfet=%s Tmot=%s spd=%s", pw, ib, im, rpm, tf, tm, spd);
}
