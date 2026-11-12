// Pure, deterministic builders of everything shown on the 128x64 OLED: the main
// dashboard (OledMain), the detail "grid" screens (OledGrid: efficiency, three
// VESC pages, the coolant pump, GNSS, SYS), the overlays (dead-man alarm, FW UPDATE, update refused),
// the screen enum, the layout constants shared by the renderer and the host tests
// and the UTC -> local clock arithmetic.
// Fault-code names come from vesc_getvalues.h (the CAN codec's 34-entry
// mc_fault_code table), so there is a single source.
//
// Arduino-free (only <stdint.h>/<stdio.h>/<string.h>/<math.h>): the display task
// calls the builders on a state snapshot, and the native unit tests exercise
// them on the host. Header-only because the "native" test environment compiles
// no src/ files.
//
// Contract:
//   * Every value that is stale (older than its freshness window) or was never
//     received is rendered as "--"; nothing stale is ever shown as live data.
//     Freshness windows: STATUS 1/4/5 -> VESC_STALE_R1_MS, STATUS 2/3/6 ->
//     VESC_STALE_R2_MS, polled values (g_state.vesc_ext) -> VESC_EXT_STALE_MS,
//     GNSS -> GNSS_STALE_MS, trip integrator (g_state.trip) -> TRIP_STALE_MS.
//   * Character budgets are hard limits enforced by clip(): the renderer lays
//     out fixed-width cells with a 6 px monospace font (see oled_layout). A
//     number that does not fit its budget drops decimals, then switches to
//     thousands ("12.3k", "12k"), millions ("12M") and finally "MAX"/"-MAX";
//     clipping a long number to a plausible-looking shorter one never happens.
//   * The output is fully deterministic for a given input: every struct is
//     zeroed before formatting, so the display task can memcmp two results to
//     decide whether the panel needs a refresh at all.
//   * The degree sign is the single Latin-1 byte 0xB0 ("\260"): the u8g2 UTF-8
//     decoder maps a lone 0x80..0xBF byte to that code point, so it costs one
//     byte = one glyph cell, which keeps the character budgets exact.
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "shared_state.h"
#if DEADMAN_UI_ENABLE
#include "deadman_logic.h"  // deadman_pub_age(): the freshest enrolled tag, for the main row
#endif
#if OTA_ENABLE
#include "ota_logic.h"  // the refusal wording of the update-mode hold
#endif
#if PUMP_ENABLE
#include "pump_logic.h"  // pump_mode_str / pump_why_str / pump_limits
#endif
#include "vesc_getvalues.h"  // vesc_fault_fmt(): "OT_FET" / "F37" (<= 8 chars), Arduino-free
#include "vesc_status.h"

// Freshness window of the trip integrator's stamp (TripState::t_ms, written
// every TRIP_PERIOD_MS): older than this (or never set) means the integrator is
// not running and the whole EFFICIENCY screen shows "--". Local fallback in
// case config.h does not define it.
#ifndef TRIP_STALE_MS
#define TRIP_STALE_MS 5000
#endif

// ---------------------------------------------------------------- screens
// Cycled by short presses of PIN_BUTTON (wrapping); a long press (and the
// SCREEN_AUTO_RETURN_MS timer, off by default) goes back to SCREEN_MAIN.
enum OledScreen : uint8_t {
  SCREEN_MAIN = 0,  // speed, clock, fix, CAN line + 8 telemetry cells
  SCREEN_EFF,       // "EFFICIENCY": Wh per NM/km now (EFF_WINDOW_S) and per trip, trip distance / energy / times, mean speed
  SCREEN_VESC_A,    // "VESC 1/3": fault (live, or the last latched one with its age), live electrical values, temperatures, Ah/Wh
  SCREEN_VESC_B,    // "VESC 2/3": polled values (MOSFET 1 temp, avg input current, id/iq, vd/vq, tacho abs, status)
  SCREEN_VESC_C,    // "VESC 3/3": inputs (PPM/ADC/PID) and CAN / poll counters
#if PUMP_ENABLE
  SCREEN_PUMP,      // "PUMP": mode / duty / reason, temperatures against their targets, the VESC's limits, PID terms
#endif
  SCREEN_GNSS,      // fix, position, accuracies, speed/heading, UTC date-time
  SCREEN_SYS,       // firmware, uptime, heap, CAN/GNSS driver state, button/lock counters
#if TOTALS_ENABLE
  SCREEN_TOTALS,    // "TOTALS": lifetime motor / powered hours, distance, net energy, this boot's motor time, last save
#endif
  SCREEN_COUNT
};

inline const char *oled_screen_name(uint8_t screen) {
  switch (screen) {
    case SCREEN_MAIN: return "MAIN";
    case SCREEN_EFF: return "EFF";
    case SCREEN_VESC_A: return "VESC 1/3";
    case SCREEN_VESC_B: return "VESC 2/3";
    case SCREEN_VESC_C: return "VESC 3/3";
#if PUMP_ENABLE
    case SCREEN_PUMP: return "PUMP";
#endif
    case SCREEN_GNSS: return "GNSS";
    case SCREEN_SYS: return "SYS";
#if TOTALS_ENABLE
    case SCREEN_TOTALS: return "TOTALS";
#endif
    default: return "?";
  }
}

// ---------------------------------------------------------------- layout
// Pixel geometry of the 128x64 panel, shared by display_oled.cpp (drawing) and
// the host tests (arithmetic check). Fonts (U8g2_for_Adafruit_GFX, drawn by
// BASELINE; a glyph of height h with y-offset 0 occupies rows baseline-h ..
// baseline-1, i.e. the baseline row itself stays empty):
//   big   u8g2_font_logisoso24_tn  digits 24 px tall, tabular 15 px advance, '.' 9 px; only " *+,-./0-9:"
//   small u8g2_font_6x10_tf        6 px advance, caps 7 px (rows B-7..B-1), '.' ':' ',' dip to row B,
//                                  g/j/p/q/y descend to row B+1; has the degree sign
//   bold  u8g2_font_6x13B_tf       6 px advance, caps 9 px (rows B-9..B-1)
namespace oled_layout {
constexpr int kWidth = 128;
constexpr int kHeight = 64;
constexpr int kAdvance = 6;      // small and bold fonts (monospace)
constexpr int kSmallCap = 7;     // small font cap height
constexpr int kBoldCap = 9;      // bold font cap height
constexpr int kBigCap = 24;      // big font digit height

// ---- main screen ----
// Zone A (rows 0..23): speed in the big font, its advance box right-aligned to
// kSpeedRight (exclusive), digits on baseline 24 -> rows 0..23. "99.9" is 54 px
// of advance (ink columns 2..52 at x 0). Right column at kColX: three small
// rows without descenders at 8 px pitch: [clock ' ' fix] / [can] / [unit].
constexpr int kSpeedRight = 54;
constexpr int kSpeedBaseline = 24;
constexpr int kSpeedChars = 4;
constexpr int kColX = 56;
constexpr int kColChars = 12;                    // 12 * 6 = 72 px -> x 56..127
constexpr int kColBaseline[3] = {8, 16, 24};     // caps rows 1..7, 9..15, 17..23
constexpr int kClockChars = 5;                   // "HH:MM"
constexpr int kFixChars = 6;                     // clock + ' ' + fix = kColChars
constexpr int kCanChars = 8;
#if DEADMAN_UI_ENABLE
constexpr int kUnitChars = kColChars;            // the dead-man row takes the whole line (see build_main_unit)
#else
constexpr int kUnitChars = 4;                    // "km/h"
#endif
// Zone B: rule, then 4 rows x 2 cells. A cell is 64 px: label at x0 + 2, value
// right-aligned by advance to x0 + 62 (last ink column <= x0 + 60, so the left
// value and the right label are >= 5 px apart). 10 chars = 60 px per cell.
constexpr int kRuleY = 26;
constexpr int kCellW = 64;
constexpr int kCellLabelDx = 2;
constexpr int kCellValueRightDx = 62;
constexpr int kCellBaseline[4] = {36, 45, 54, 63};  // caps rows 29..35, 38..44, 47..53, 56..62
constexpr int kCellLabelChars = 3;
constexpr int kCellValueChars = 6;
constexpr int kCellChars = 10;                   // label + >= 1 space + value

// ---- grid screens ----
// Inverted title bar rows 0..9 (bold text baseline 9 -> rows 0..8, page "n/N"
// right-aligned to x 127), then 6 small rows at 9 px pitch: baselines 18, 27,
// 36, 45, 54, 63 -> caps 11..17 ... 56..62. Rows are 21 chars = 126 px at x 1.
constexpr int kTitleBarH = 10;
constexpr int kTitleBaseline = 9;
constexpr int kTitleX = 1;
constexpr int kTitleChars = 15;
constexpr int kPageChars = 5;
constexpr int kGridRows = 6;
constexpr int kGridChars = 21;
constexpr int kGridX = 1;
constexpr int kGridBaseline0 = 18;
constexpr int kGridPitch = 9;
constexpr int kGridCol2 = 11;                    // second column starts at character 11 (x 67)

// ---- arithmetic checks (the same numbers the renderer draws with) ----
static_assert(kSpeedBaseline - kBigCap >= 0 && kSpeedBaseline <= kHeight, "speed row inside the panel");
static_assert(kSpeedRight - 15 * 3 - 9 >= 0, "'99.9' (3 digits + '.') starts at x >= 0");
static_assert(kSpeedRight + 2 <= kColX, "gap between the speed and the right column");
static_assert(kColX + kColChars * kAdvance <= kWidth, "right column fits 12 characters");
static_assert(kClockChars + 1 + kFixChars == kColChars, "clock + space + fix fill the first right-column line");
static_assert(kCanChars <= kColChars && kUnitChars <= kColChars, "can/unit lines fit");
static_assert(kColBaseline[0] - kSmallCap >= 0, "first right-column row starts at row >= 0");
static_assert(kColBaseline[1] - kSmallCap > kColBaseline[0] - 1, "right-column rows do not overlap");
static_assert(kColBaseline[2] - kSmallCap > kColBaseline[1] - 1, "right-column rows do not overlap");
static_assert(kColBaseline[2] <= kSpeedBaseline, "right column ends with the speed row");
static_assert(kRuleY > kSpeedBaseline, "rule below Zone A");
static_assert(kCellBaseline[0] - kSmallCap > kRuleY, "first cell row below the rule");
static_assert(kCellBaseline[3] <= kHeight - 1, "last cell row inside the panel ('.' dips to the baseline row)");
static_assert(kCellBaseline[1] - kSmallCap > kCellBaseline[0] && kCellBaseline[2] - kSmallCap > kCellBaseline[1] &&
                  kCellBaseline[3] - kSmallCap > kCellBaseline[2],
              "cell rows leave >= 1 blank row (one row may carry a '.' on its baseline row)");
static_assert(2 * kCellW == kWidth, "two cells per row");
static_assert(kCellLabelDx + kCellChars * kAdvance <= kCellValueRightDx, "10 characters fit between label x and value edge");
static_assert(kCellLabelChars + 1 + kCellValueChars <= kCellChars, "label + space + value fit a cell");
static_assert(kTitleBaseline - kBoldCap >= 0 && kTitleBaseline <= kTitleBarH, "title text inside the bar");
static_assert(kTitleX + (kTitleChars + 1 + kPageChars) * kAdvance <= kWidth, "title and page indicator fit the bar");
static_assert(kGridBaseline0 - kSmallCap > kTitleBarH, ">= 1 blank row between the bar and the first grid row");
static_assert(kGridPitch >= kSmallCap + 2, "grid rows: caps never overlap the previous row's baseline row");
static_assert(kGridBaseline0 + (kGridRows - 1) * kGridPitch <= kHeight - 1, "last grid row inside the panel");
static_assert(kGridX + kGridChars * kAdvance <= kWidth, "21 characters fit a grid row");
static_assert(kGridCol2 <= kGridChars - 1, "second column inside the row");
}  // namespace oled_layout

// ---------------------------------------------------------------- output structs
struct OledMain {
  char speed[6];  // "10.0" / "117" / "0.0" / "--"                    (<= 4 chars, big font: digits '.' '-' only)
  char unit[14];  // SPEED_UNIT_STR ("kn" / "km/h"), or in a DEADMAN_UI_ENABLE build the dead-man
                  // row instead ("1:-62 2:-70" / "CUT IN 3s" / "NO RADIO")   (<= kUnitChars)
  char clock[6];  // "14:34" local (UTC + TIME_UTC_OFFSET_MIN) / "--:--" (5)
  char fix[8];    // "3D 9sv" / "3D12sv" / "2D 8sv" / "NOFIX" / "NODATA" / "NOGNSS" / "BOOT" (<= 6)
  char can[10];   // "VESC 74" / "CAN idle" / "CAN RECV" / "CAN OFF" / FW_VERSION at boot   (<= 8)
                  // (VESC_LINK_UART: "VESC 74" / "NO VESC" / "UART OFF")
  struct Cell {
    char label[4];  // "BV" "BA" "MA" "PW" "TF" "TM" "RPM" "WH"        (<= 3)
    char value[8];  // "48.2v" "12.4A" "35A" "598W" "45\260" "2350" "56.7" / "--" / "n/a" (<= 6)
  } cells[8];       // row-major: [BV BA] [MA PW] [TF TM] [RPM WH]
};

struct OledGrid {
  char title[16];    // "EFFICIENCY" / "VESC 1/3" / "SYS 0.1.0"  (<= 15, bold)
  char page[6];      // "2/7"                                   (<= 5, right-aligned in the bar)
  char rows[6][22];  // <= 21 monospace chars each
  uint8_t nrows;     // rows in use (always 6 here)
};

// Everything the renderer compares to decide whether to push a frame.
struct OledFrame {
  uint8_t screen;   // OledScreen; selects which member is valid
  uint8_t overlay;  // OledOverlay: when set it REPLACES the screen (the dead-man alarm)
  OledMain main;    // SCREEN_MAIN
  OledGrid grid;    // every other screen, and every overlay
};

// An overlay takes the whole panel whatever screen is selected: a cut motor must not be
// something you can page away from, and in update mode there is nothing else to show.
enum OledOverlay : uint8_t {
  OVERLAY_NONE = 0,
  OVERLAY_DEADMAN,      // the dead-man's switch is holding the motor cut (TRIPPED or FAULT)
  OVERLAY_OTA,          // update mode: access point, passphrase, upload progress
  OVERLAY_OTA_REFUSED,  // a hold for update mode was just refused (OTA_REFUSED_SHOW_MS)
  OVERLAY_PUMP,         // the pump's tach says it stalled (or gives no signal) and nobody dismissed it yet
};

// Host-side facts the Arduino-free builders cannot fetch themselves.
struct OledSysInfo {
  uint32_t uptime_s;
  uint32_t heap_free;        // bytes
  uint32_t heap_min;         // bytes (minimum ever free)
  const char *reset_reason;  // "POWERON" / "TASK_WDT" / ... (nullptr -> "?")
};

// ---------------------------------------------------------------- clock
struct OledLocalTime {
  uint8_t hour, min;
  int8_t day_shift;  // -1 / 0 / +1 day relative to the UTC date
};

// UTC hh:mm + offset (minutes, may be negative or beyond a day) -> local hh:mm
// with a floor-division day wrap (C++ '/' truncates toward zero).
inline OledLocalTime oled_utc_to_local(uint8_t h, uint8_t m, int32_t offset_min) {
  int32_t t = (int32_t)h * 60 + (int32_t)m + offset_min;
  int32_t days = t / 1440;
  int32_t r = t - days * 1440;
  if (r < 0) {
    r += 1440;
    days -= 1;
  }
  OledLocalTime lt;
  lt.hour = (uint8_t)(r / 60);
  lt.min = (uint8_t)(r % 60);
  lt.day_shift = (int8_t)days;
  return lt;
}

namespace oled_detail {

using namespace oled_layout;

// Hard length guarantee: the caller memcmp()s and lays out fixed-width cells, so
// nothing may ever exceed max_chars (buffers are >= max_chars + 1 bytes).
inline void clip(char *buf, size_t n, size_t max_chars) {
  if (max_chars < n) buf[max_chars] = '\0';
}

// Bounded copy / append (always NUL-terminated). Used instead of snprintf("%s")
// wherever the source buffer is larger than the destination: GCC's
// -Wformat-truncation reasons about "%s" by the source array's bound.
inline void copy_str(char *dst, size_t n, const char *src) {
  size_t i = 0;
  while (i + 1 < n && src[i] != '\0') {
    dst[i] = src[i];
    ++i;
  }
  dst[i] = '\0';
}

inline void append_str(char *dst, size_t n, const char *src) {
  size_t l = strlen(dst);
  if (l >= n) return;
  copy_str(dst + l, n - l, src);
}

// A formatted candidate that was truncated by snprintf must never be accepted
// as "fits": with max_chars <= n - 2 a truncated result (n - 1 chars) is always
// longer than max_chars and rejected, so the next (shorter) form is tried.
inline size_t safe_max(size_t n, size_t max_chars) { return n >= 3 && max_chars > n - 2 ? n - 2 : max_chars; }

// Strips the sign from a negative zero ("-0.0" -> "0.0", "-0" -> "0"). WHY: a
// smoothed current hovering around 0 A would otherwise flip between "0" and
// "-0", each flip costing a needless frame push.
inline void strip_negative_zero(char *buf) {
  if (buf[0] != '-') return;
  for (const char *p = buf + 1; *p; ++p) {
    if (*p != '0' && *p != '.') return;  // a real non-zero digit: keep the sign
  }
  memmove(buf, buf + 1, strlen(buf));  // drops the '-' and moves the NUL along
}

inline void fmt_dashes(char *buf, size_t n) { snprintf(buf, n, "--"); }

// True when the numeric part of a scaled candidate ("0k", "-0.0M", "0G") is
// zero: such a candidate would hide a large value behind a "0", so it is skipped.
inline bool zero_candidate(const char *buf) {
  for (const char *p = buf; *p; ++p) {
    if (*p >= '1' && *p <= '9') return false;
    if (*p != '0' && *p != '.' && *p != '-') break;  // suffix reached
  }
  return true;
}

// Accepts a scaled candidate when it fits and is not a bare zero.
inline bool scaled_ok(const char *buf, size_t max_chars) { return strlen(buf) <= max_chars && !zero_candidate(buf); }

// Thousands / millions / saturation steps: "12.3k", "12k", "12M", then "MAX" or
// "-MAX" (clipped to max_chars). Clipping "10737k" to "1073" would print a
// plausible-looking wrong number, hence the explicit saturation.
inline void fmt_scaled(char *buf, size_t n, float v, size_t max_chars) {
  max_chars = safe_max(n, max_chars);
  snprintf(buf, n, "%.1fk", (double)(v / 1000.0f));
  strip_negative_zero(buf);
  if (scaled_ok(buf, max_chars)) return;
  snprintf(buf, n, "%.0fk", (double)(v / 1000.0f));
  strip_negative_zero(buf);
  if (scaled_ok(buf, max_chars)) return;
  snprintf(buf, n, "%.0fM", (double)(v / 1000000.0f));
  strip_negative_zero(buf);
  if (scaled_ok(buf, max_chars)) return;
  snprintf(buf, n, "%s", v < 0.0f ? "-MAX" : "MAX");
  clip(buf, n, max_chars);
}

// Formats v with at most `dec` decimals into at most max_chars characters:
// drops decimals one by one ("48.2" -> "100" when "100.0" is too wide), then
// falls back to fmt_scaled. Literal format strings keep -Wformat happy; the
// (double) casts keep -Wdouble-promotion quiet.
inline void fmt_num(char *buf, size_t n, float v, int dec, size_t max_chars) {
  max_chars = safe_max(n, max_chars);
  for (int d = dec; d >= 0; --d) {
    snprintf(buf, n, "%.*f", d, (double)v);
    strip_negative_zero(buf);
    if (strlen(buf) <= max_chars) return;
  }
  fmt_scaled(buf, n, v, max_chars);
}

// Rounded integer division (no overflow for any uint32_t v).
inline uint32_t div_round(uint32_t v, uint32_t d) { return v / d + (v % d >= d / 2u ? 1u : 0u); }

// Unsigned counter into at most max_chars characters: "12345", "1234k", "123M",
// "4G" (rounded at each step, so 999999 in 3 chars is "1M"); a step that would
// read "0k"/"0M" is skipped and the value ends as "MAX".
inline void fmt_count(char *buf, size_t n, uint32_t v, size_t max_chars) {
  max_chars = safe_max(n, max_chars);
  snprintf(buf, n, "%lu", (unsigned long)v);
  if (strlen(buf) <= max_chars) return;
  snprintf(buf, n, "%luk", (unsigned long)div_round(v, 1000u));
  if (scaled_ok(buf, max_chars)) return;
  snprintf(buf, n, "%luM", (unsigned long)div_round(v, 1000000u));
  if (scaled_ok(buf, max_chars)) return;
  snprintf(buf, n, "%luG", (unsigned long)div_round(v, 1000000000u));
  if (scaled_ok(buf, max_chars)) return;
  snprintf(buf, n, "MAX");
  clip(buf, n, max_chars);
}

// Number followed by a unit suffix, the whole thing at most max_chars wide.
inline void fmt_num_unit(char *buf, size_t n, float v, int dec, size_t max_chars, const char *unit) {
  const size_t ul = strlen(unit);
  fmt_num(buf, n, v, dec, max_chars > ul ? max_chars - ul : 0);
  append_str(buf, n, unit);
  clip(buf, n, max_chars);
}

// Electrical power: watts below 10 kW ("598W", "-4800W"), else kilowatts
// ("12.3kW", "100kW", "-12kW"); >= 6 chars are never exceeded.
inline void fmt_power(char *buf, size_t n, float p, size_t max_chars) {
  if (fabsf(p) < 9999.5f) fmt_num(buf, n, p, 0, max_chars - 1);
  else fmt_scaled(buf, n, p, max_chars - 1);
  append_str(buf, n, "W");
  clip(buf, n, max_chars);
}

// Mechanical rpm: whole rpm below 10000 ("2350", "-2350"), thousands above ("12.3k").
inline void fmt_rpm(char *buf, size_t n, float rpm, size_t max_chars) {
  if (fabsf(rpm) < 9999.5f) fmt_num(buf, n, rpm, 0, max_chars);
  else fmt_scaled(buf, n, rpm, max_chars);
}

// Watt-hours: one decimal below 1000 ("56.7", "123.4"), whole below 10000
// ("1235", "9999"), thousands above ("12.3k", "123k"). The thresholds are the
// rounding points, so "1000.0" / "10000" are never produced.
inline void fmt_wh(char *buf, size_t n, float wh, size_t max_chars) {
  if (fabsf(wh) < 999.95f) fmt_num(buf, n, wh, 1, max_chars);
  else if (fabsf(wh) < 9999.5f) fmt_num(buf, n, wh, 0, max_chars);
  else fmt_scaled(buf, n, wh, max_chars);
}

// Trip energy with its unit: whole Wh below 1 kWh ("567Wh", "-250Wh"), kilowatt-hours
// below 1 MWh with two decimals while they fit the budget ("1.23kWh", "12.35kWh", then
// "123.5kWh" as fmt_num drops decimals), megawatt-hours above ("1.23MWh", "123.5MWh":
// a 49-day trip at 1 kW; the k-step of fmt_scaled would otherwise print "1000kkWh").
inline void fmt_energy(char *buf, size_t n, float wh, size_t max_chars) {
  if (fabsf(wh) < 999.5f) fmt_num_unit(buf, n, wh, 0, max_chars, "Wh");
  else if (fabsf(wh) < 999995.0f) fmt_num_unit(buf, n, wh / 1000.0f, 2, max_chars, "kWh");  // < 999.995 kWh -> never "1000.00kWh"
  else fmt_num_unit(buf, n, wh / 1000000.0f, 2, max_chars, "MWh");
}

// Efficiency (Wh per distance unit): one decimal below 100 ("98.5", "-12.3" for
// regen), whole above ("123", "-123").
inline void fmt_eff(char *buf, size_t n, float v, size_t max_chars) {
  fmt_num(buf, n, v, fabsf(v) < 99.95f ? 1 : 0, max_chars);
}

inline bool temp_plausible(float t) { return vesc_temp_plausible(t); }

// ---- freshness ----------------------------------------------------------------
struct VescFresh {
  bool s1, s2, s3, s4, s5, s6;  // STATUS_1..6 within their windows
  bool ext;                     // polled values within VESC_EXT_STALE_MS
};

inline VescFresh vesc_freshness(const SharedState &s, uint32_t now) {
  VescFresh f;
  const vesc_telemetry_t &t = s.vesc.t;
  f.s1 = vesc_fresh(&t, VESC_IDX_STATUS_1, now, VESC_STALE_R1_MS);
  f.s2 = vesc_fresh(&t, VESC_IDX_STATUS_2, now, VESC_STALE_R2_MS);
  f.s3 = vesc_fresh(&t, VESC_IDX_STATUS_3, now, VESC_STALE_R2_MS);
  f.s4 = vesc_fresh(&t, VESC_IDX_STATUS_4, now, VESC_STALE_R1_MS);
  f.s5 = vesc_fresh(&t, VESC_IDX_STATUS_5, now, VESC_STALE_R1_MS);
  f.s6 = vesc_fresh(&t, VESC_IDX_STATUS_6, now, VESC_STALE_R2_MS);
  f.ext = fresh(s.vesc_ext.t_ms, now, VESC_EXT_STALE_MS);
  return f;
}

// The trip integrator stamps TripState::t_ms on every tick; a stale or never-set
// stamp means it is not running (nothing on the EFFICIENCY screen is trustworthy).
inline bool trip_live(const TripState &t, uint32_t now) { return fresh(t.t_ms, now, TRIP_STALE_MS); }

// ---- shared GNSS pieces -------------------------------------------------------
inline bool gnss_live(const GnssState &g, uint32_t now) { return fresh(g.last_pvt_ms, now, GNSS_STALE_MS); }

// Speed is gated three ways: recent epoch, receiver says the fix is usable, and
// the receiver's own speed-accuracy estimate is sane (a fresh 2D fix with sAcc
// 5 m/s would otherwise show random knots at the dock). <= 4 chars, digits/'.'
// only (the big font has no letters): speeds >= 9999.5 in the display unit are
// implausible for a boat and shown as "--" instead of a 'k' or "MAX" the font
// could not draw.
inline void fmt_speed(char *buf, size_t n, const GnssState &g, uint32_t now) {
  if (!gnss_live(g, now) || !g.fix_ok || g.sacc_mm_s > (uint32_t)GNSS_MAX_SACC_MM_S) {
    fmt_dashes(buf, n);
    return;
  }
  const float v = (float)g.gspeed_mm_s * SPEED_FACTOR;
  if (v < SPEED_MIN_SHOW) snprintf(buf, n, "0.0");  // hides drift at rest (and any negative oddity)
  else if (v >= 9999.5f) fmt_dashes(buf, n);
  else fmt_num(buf, n, v, 1, kSpeedChars);  // "10.0" below 100, "117" above
  clip(buf, n, kSpeedChars);
}

// "HH:MM" local time while the receiver reports a valid UTC date and time and
// the epoch is fresh, else "--:--". fixType is irrelevant (time-only fixes count).
inline void fmt_clock(char *buf, size_t n, const GnssState &g, uint32_t now) {
  if (!g.time_valid || !gnss_live(g, now)) {
    snprintf(buf, n, "--:--");
    return;
  }
  const OledLocalTime lt = oled_utc_to_local(g.hour, g.min, (int32_t)TIME_UTC_OFFSET_MIN);
  // % 24 / % 60 are no-ops (already in range) that let GCC bound the width for -Wformat-truncation
  snprintf(buf, n, "%02u:%02u", (unsigned)lt.hour % 24u, (unsigned)lt.min % 60u);
}

inline const char *gnss_phase_name(uint8_t phase) {
  switch (phase) {
    case GNSS_PHASE_AUTOBAUD: return "AUTOBAUD";
    case GNSS_PHASE_DETECT: return "DETECT";
    case GNSS_PHASE_CONFIGURE: return "CONFIG";
    case GNSS_PHASE_RUN: return "RUN";
    default: return "?";
  }
}

inline const char *can_state_name(int state) {
  switch (state) {
    case CAN_STATE_UNINSTALLED: return "UNINST";
    case CAN_STATE_STOPPED: return "STOP";
    case CAN_STATE_RUNNING: return "RUN";
    case CAN_STATE_BUS_OFF: return "BUSOFF";
    case CAN_STATE_RECOVERING: return "RECOV";
    default: return "?";
  }
}

// ---- main screen ----------------------------------------------------------------
inline void set_cell(OledMain::Cell &c, const char *label) {
  snprintf(c.label, sizeof c.label, "%s", label);
  clip(c.label, sizeof c.label, kCellLabelChars);
}

inline void cell_dashes(OledMain::Cell &c) { fmt_dashes(c.value, sizeof c.value); }

inline void build_main_cells(const SharedState &s, const VescFresh &f, OledMain &out) {
  const VescState &v = s.vesc;
  OledMain::Cell *c = out.cells;
  set_cell(c[0], "BV");
  set_cell(c[1], "BA");
  set_cell(c[2], "MA");
  set_cell(c[3], "PW");
  set_cell(c[4], "TF");
  set_cell(c[5], "TM");
  set_cell(c[6], "RPM");
  set_cell(c[7], "WH");

  if (f.s5) fmt_num_unit(c[0].value, sizeof c[0].value, v.v_in_ema, 1, kCellValueChars, "v");
  else cell_dashes(c[0]);

  // Battery current, signed: "12.4A", "-12.4A", "123.4A", "-123A".
  if (f.s4) fmt_num_unit(c[1].value, sizeof c[1].value, v.i_in_ema, 1, kCellValueChars, "A");
  else cell_dashes(c[1]);

  // Motor current in whole amps: "35A", "-35A", "1234A", "123kA".
  if (f.s1) fmt_num_unit(c[2].value, sizeof c[2].value, v.i_motor_ema, 0, kCellValueChars, "A");
  else cell_dashes(c[2]);

  // Power needs both factors live: a fresh current times a stale voltage is a lie.
  if (f.s4 && f.s5) fmt_power(c[3].value, sizeof c[3].value, v.v_in_ema * v.i_in_ema, kCellValueChars);
  else cell_dashes(c[3]);

  // MOSFET temperature: "--" when stale or implausible.
  if (f.s4 && temp_plausible(v.t.temp_fet)) fmt_num_unit(c[4].value, sizeof c[4].value, v.t.temp_fet, 0, kCellValueChars, "\260");
  else cell_dashes(c[4]);

  // Motor temperature: "n/a" when the VESC has no motor NTC (implausible reading).
  if (!f.s4) cell_dashes(c[5]);
  else if (!temp_plausible(v.t.temp_motor)) snprintf(c[5].value, sizeof c[5].value, "n/a");
  else fmt_num_unit(c[5].value, sizeof c[5].value, v.t.temp_motor, 0, kCellValueChars, "\260");

  if (f.s1) fmt_rpm(c[6].value, sizeof c[6].value, vesc_mech_rpm(&v.t, VESC_MOTOR_POLES), kCellValueChars);
  else cell_dashes(c[6]);

  // Watt hours drawn (STATUS_3, Rate 2): "56.7", "123.4", "1235", "9999", "12.3k", "123k".
  if (f.s3) fmt_wh(c[7].value, sizeof c[7].value, v.t.watt_hours, kCellValueChars);
  else cell_dashes(c[7]);

  for (int i = 0; i < 8; ++i) clip(c[i].value, sizeof c[i].value, kCellValueChars);
}

inline void build_main_fix(const GnssState &g, uint32_t now, OledMain &out) {
  const bool ever = g.last_pvt_ms != 0;
  const bool live = gnss_live(g, now);
  const unsigned sv = g.num_sv > 99 ? 99u : (unsigned)g.num_sv;  // 6 chars: "3D99sv" at most
  if (!ever) snprintf(out.fix, sizeof out.fix, "NOGNSS");
  else if (!live) snprintf(out.fix, sizeof out.fix, "NODATA");
  else if (g.fix_type == 3 || g.fix_type == 4) snprintf(out.fix, sizeof out.fix, "3D%2usv", sv);
  else if (g.fix_type == 2) snprintf(out.fix, sizeof out.fix, "2D%2usv", sv);
  else snprintf(out.fix, sizeof out.fix, "NOFIX");
  clip(out.fix, sizeof out.fix, kFixChars);
}

inline void build_main_can(const SharedState &s, const VescFresh &f, OledMain &out) {
#if VESC_LINK_UART
  // UART link: the state only says whether the driver is up; freshness says whether the VESC answers.
  if (s.can.state != CAN_STATE_RUNNING) snprintf(out.can, sizeof out.can, "UART OFF");
  else if (!f.s1 && !f.s5) snprintf(out.can, sizeof out.can, "NO VESC");
  else if (s.vesc.locked_id >= 0) snprintf(out.can, sizeof out.can, "VESC %u", (unsigned)(s.vesc.locked_id & 0xFF));
  else snprintf(out.can, sizeof out.can, "VESC ?");  // a reply without the controller id field (very old firmware)
#else
  switch (s.can.state) {
    case CAN_STATE_RUNNING:
      // Same freshness test as the LED in main.cpp: STATUS_1 or STATUS_5 within VESC_STALE_R1_MS.
      if (!f.s1 && !f.s5) snprintf(out.can, sizeof out.can, "CAN idle");
      else if (s.vesc.locked_id >= 0) snprintf(out.can, sizeof out.can, "VESC %u", (unsigned)(s.vesc.locked_id & 0xFF));
      else snprintf(out.can, sizeof out.can, "VESC ?");  // fresh frames but no id locked: cannot happen, stay defined
      break;
    case CAN_STATE_RECOVERING: snprintf(out.can, sizeof out.can, "CAN RECV"); break;
    default: snprintf(out.can, sizeof out.can, "CAN OFF"); break;  // STOPPED, BUS_OFF, UNINSTALLED, unknown
  }
#endif
  clip(out.can, sizeof out.can, kCanChars);
}

// ---- grid helpers ---------------------------------------------------------------
// Zeroes the grid and fills the title bar: title left, "n/N" (1-based screen index) right.
// The index is clamped to the last screen: a page indicator never reads above N/N, and the
// bound keeps "%u/%u" within the 5-char budget for -Wformat-truncation (SCREEN_COUNT <= 11).
inline void grid_begin(OledGrid &g, const char *title, uint8_t screen) {
  memset(&g, 0, sizeof g);
  copy_str(g.title, sizeof g.title, title);
  clip(g.title, sizeof g.title, kTitleChars);
  const unsigned page = screen < SCREEN_COUNT ? (unsigned)screen + 1u : (unsigned)SCREEN_COUNT;
  snprintf(g.page, sizeof g.page, "%u/%u", page, (unsigned)SCREEN_COUNT);
  clip(g.page, sizeof g.page, kPageChars);
  g.nrows = kGridRows;
}

// One full-width row (clipped to 21 chars). `row` is an OledGrid::rows[] entry (22 bytes).
inline void row1(char *row, const char *text) { copy_str(row, kGridChars + 1, text); }

// Two-column row: `left` at character 0, `right` at character kGridCol2 (or
// one space after a left text longer than 10 chars). Clipped to 21 chars, no
// trailing padding.
inline void row2(char *row, const char *left, const char *right) {
  const size_t n = (size_t)kGridChars + 1;
  copy_str(row, n, left);
  size_t pos = strlen(row);
  pos = pos < (size_t)kGridCol2 ? (size_t)kGridCol2 : pos + 1;
  if (pos >= n - 1) return;  // left text already fills the row
  memset(row + strlen(row), ' ', pos - strlen(row));
  copy_str(row + pos, n - pos, right);
  // a right text that did not fit at all leaves trailing padding: trim it
  size_t l = strlen(row);
  while (l > 0 && row[l - 1] == ' ') row[--l] = '\0';
}

// "<label> <number>" or "<label> --".
inline void labeled(char *buf, size_t n, const char *label, bool ok, float v, int dec, size_t max_chars,
                    const char *unit = "") {
  char num[16];
  if (ok) fmt_num_unit(num, sizeof num, v, dec, max_chars, unit);
  else fmt_dashes(num, sizeof num);
  copy_str(buf, n, label);
  append_str(buf, n, " ");
  append_str(buf, n, num);
}

inline void labeled_count(char *buf, size_t n, const char *label, uint32_t v, size_t max_chars) {
  char num[16];
  fmt_count(num, sizeof num, v, max_chars);
  copy_str(buf, n, label);
  append_str(buf, n, " ");
  append_str(buf, n, num);
}

// Age of a past event as "34s" / "12m" / "3h" / "2d" (<= 3 chars: the uint32_t
// millis() range ends at 49 days).
inline void fmt_age_short(char *buf, size_t n, uint32_t ms) {
  const uint32_t s = ms / 1000u;
  if (s < 60u) snprintf(buf, n, "%lus", (unsigned long)s);
  else if (s < 3600u) snprintf(buf, n, "%lum", (unsigned long)(s / 60u));
  else if (s < 86400u) snprintf(buf, n, "%luh", (unsigned long)(s / 3600u));
  else snprintf(buf, n, "%lud", (unsigned long)(s / 86400u));
}

// Fault row of "VESC 1/3" (<= 21 chars). The VESC clears its live fault byte
// ~500 ms after a fault, so at 1 Hz polling it reads 0 almost always; can_vesc
// latches the last non-zero code (VescExt::last_fault / last_fault_ms):
//   "FAULT <name>"       live byte non-zero in a fresh reply       (<= 14 chars)
//   "LAST <name> <age>"  no live fault, a fault was latched earlier (<= 17 chars)
//   "FAULT none"         fresh reply, no fault, nothing latched
//   "FAULT --"           polled values stale or never polled (VESC_POLL_MS 0), nothing latched
// <name> is vesc_fault_fmt(): "OT_FET" / "ENC_HIGH" / "F37" for codes beyond the table.
inline void build_fault_row(const VescExt &e, bool ext_fresh, uint32_t now, char *row) {
  char name[9], age[8], buf[32];
  if (ext_fresh && e.fault_code != 0) {
    snprintf(buf, sizeof buf, "FAULT %s", vesc_fault_fmt(name, sizeof name, e.fault_code));
  } else if (e.last_fault != 0 && e.last_fault_ms != 0) {
    uint32_t ms = (uint32_t)(now - e.last_fault_ms);
    if ((int32_t)ms < 0) ms = 0;  // producer stamped after our `now` was read: 0 s, not 49 d
    fmt_age_short(age, sizeof age, ms);
    snprintf(buf, sizeof buf, "LAST %s %s", vesc_fault_fmt(name, sizeof name, e.last_fault), age);
  } else if (ext_fresh) {
    snprintf(buf, sizeof buf, "FAULT none");
  } else {
    snprintf(buf, sizeof buf, "FAULT --");
  }
  row1(row, buf);
}

// ---- GNSS pieces used by the grid -------------------------------------------------
// degrees * 1e7 -> "59.12345N" style (5 decimals, hemisphere letter), integer only.
inline void fmt_coord(char *buf, size_t n, int32_t e7, char pos, char neg) {
  const uint32_t a = e7 < 0 ? (uint32_t)(-(int64_t)e7) : (uint32_t)e7;
  uint32_t deg = a / 10000000u;
  uint32_t frac5 = (a % 10000000u + 50u) / 100u;  // rounded to 5 decimals
  if (frac5 >= 100000u) {
    frac5 -= 100000u;
    deg += 1u;
  }
  snprintf(buf, n, "%lu.%05lu%c", (unsigned long)deg, (unsigned long)frac5, e7 < 0 ? neg : pos);
}

// Uptime as "12m34s" / "1h23m" / "12d03h", whole days from 100 days on ("100d",
// "49710d" = the uint32_t range): always <= 6 chars, so two of them fill a
// two-column row ("run 12d03h mov 12d03h") without ever clipping the second one
// to a plausible-looking shorter number.
inline void fmt_uptime(char *buf, size_t n, uint32_t sec) {
  if (sec < 3600u) snprintf(buf, n, "%lum%02lus", (unsigned long)(sec / 60u), (unsigned long)(sec % 60u));
  else if (sec < 86400u) snprintf(buf, n, "%luh%02lum", (unsigned long)(sec / 3600u), (unsigned long)((sec / 60u) % 60u));
  else if (sec < 100u * 86400u) snprintf(buf, n, "%lud%02luh", (unsigned long)(sec / 86400u), (unsigned long)((sec / 3600u) % 24u));
  else snprintf(buf, n, "%lud", (unsigned long)(sec / 86400u));
}


// In-place ASCII upper case: the panel's last grid row has no room for descenders.
inline void upper_str(char *buf) {
  for (char *p = buf; *p; ++p)
    if (*p >= 'a' && *p <= 'z') *p = (char)(*p - 'a' + 'A');
}

}  // namespace oled_detail

#if DEADMAN_UI_ENABLE
// The dead-man row REPLACES the speed unit on the main screen. A unit that never changes is
// worth less than knowing whether the switch is protecting you and how much signal margin
// each tag has; the unit is a build-time constant and is still shown on the EFFICIENCY page
// ("mean 5.9kn"). Having this row is also what lets the CAN cell go back to always showing
// the CAN state - the dead-man used to hijack it for want of anywhere else.
//
// Priority order, first match wins (12 chars):
//   "NO RADIO"     the watchdog scan is not running: no protection at all
//   "CUT IN 3s"    GRACE, counting down to the cut
//   "1:-62 2:-70"  each configured tag and its RSSI
//
// A slot reads " --" while its tag is quiet (older than DEADMAN_WARN_MS) and "off" once the
// enrolment window has closed without it - that is the "disabled because it was missing at
// startup" state. An unconfigured slot is not shown at all, so a single-tag boat reads
// "1:-62". All three slot forms are 3 characters, so the row never jitters in width.
inline void build_main_deadman(const SharedState &s, uint32_t now, OledMain &out) {
  using namespace oled_detail;
  const DeadmanState &d = s.deadman;
  if (d.state == DM_UNAVAILABLE) {
    copy_str(out.unit, sizeof out.unit, "NO RADIO");
    return;
  }
  if (d.state == DM_GRACE) {
    const uint32_t age = deadman_pub_age(d, now);
    if (age < (uint32_t)DEADMAN_TIMEOUT_MS) {
      char a[4];  // fmt_age_short is at most 3 chars
      fmt_age_short(a, sizeof a, (uint32_t)DEADMAN_TIMEOUT_MS - age);
      snprintf(out.unit, sizeof out.unit, "CUT IN %s", a);
      return;
    }
  }
  out.unit[0] = '\0';
  for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
    if (!d.tags[i].configured) continue;
    char val[6], one[10];
    if (!d.tags[i].enrolled && !d.enrol_open) {
      copy_str(val, sizeof val, "off");  // it was not there when the set was frozen
    } else if (d.tags[i].t_ms == 0 || age_ms(d.tags[i].t_ms, now) > (uint32_t)DEADMAN_WARN_MS) {
      copy_str(val, sizeof val, " --");
    } else {
      // Clamped to -99: a 4-digit RSSI would push two slots past the 12-char row, and
      // anything below -99 dBm is unusable for a watchdog anyway.
      int rssi = (int)d.tags[i].rssi;
      if (rssi < -99) rssi = -99;
      if (rssi > 0) rssi = 0;
      snprintf(val, sizeof val, "%3d", rssi);
    }
    snprintf(one, sizeof one, "%s%u:%s", out.unit[0] ? " " : "", i + 1u, val);
    append_str(out.unit, sizeof out.unit, one);
  }
  if (out.unit[0] == '\0') copy_str(out.unit, sizeof out.unit, "NO TAG");  // nothing configured
  clip(out.unit, sizeof out.unit, kUnitChars);
}
#endif  // DEADMAN_UI_ENABLE

// ================================================================ public builders
// Main screen. now = current millis() (same clock as the timestamps in s).
inline void oled_build_main(const SharedState &s, uint32_t now, OledMain &out) {
  using namespace oled_detail;
  memset(&out, 0, sizeof out);  // deterministic tail bytes: the caller memcmp()s whole structs
  const VescFresh f = vesc_freshness(s, now);
  fmt_speed(out.speed, sizeof out.speed, s.gnss, now);
  snprintf(out.unit, sizeof out.unit, "%s", SPEED_UNIT_STR);
  clip(out.unit, sizeof out.unit, kUnitChars);
  fmt_clock(out.clock, sizeof out.clock, s.gnss, now);
  build_main_fix(s.gnss, now, out);
  build_main_can(s, f, out);  // never overridden any more: the dead-man has its own row
#if DEADMAN_UI_ENABLE
  build_main_deadman(s, now, out);  // replaces the speed unit written above
#endif
  build_main_cells(s, f, out);
}

// "VESC 1/3": fault (live or latched, see build_fault_row), live electrical values,
// duty, power/rpm, temperatures, Ah/Wh.
inline void oled_build_vesc_a(const SharedState &s, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  grid_begin(g, "VESC 1/3", SCREEN_VESC_A);
  const VescFresh f = vesc_freshness(s, now);
  const vesc_telemetry_t &t = s.vesc.t;
  char l[32], r[32], tmp[8];

  build_fault_row(s.vesc_ext, f.ext, now, g.rows[0]);

  labeled(l, sizeof l, "Vin", f.s5, s.vesc.v_in_ema, 1, 5);
  labeled(r, sizeof r, "Ibat", f.s4, s.vesc.i_in_ema, 1, 5);
  row2(g.rows[1], l, r);

  labeled(l, sizeof l, "Imot", f.s1, s.vesc.i_motor_ema, 0, 5);
  labeled(r, sizeof r, "Duty", f.s1, t.duty * 100.0f, 0, 5, "%");
  row2(g.rows[2], l, r);

  if (f.s4 && f.s5) fmt_power(tmp, sizeof tmp, s.vesc.v_in_ema * s.vesc.i_in_ema, 6);
  else fmt_dashes(tmp, sizeof tmp);
  snprintf(l, sizeof l, "P %s", tmp);
  if (f.s1) fmt_rpm(tmp, sizeof tmp, vesc_mech_rpm(&t, VESC_MOTOR_POLES), 6);
  else fmt_dashes(tmp, sizeof tmp);
  snprintf(r, sizeof r, "RPM %s", tmp);
  row2(g.rows[3], l, r);

  labeled(l, sizeof l, "Tfet", f.s4, t.temp_fet, 1, 5);
  labeled(r, sizeof r, "Tmot", f.s4, t.temp_motor, 1, 5);
  row2(g.rows[4], l, r);

  labeled(l, sizeof l, "Ah", f.s2, t.amp_hours, 3, 7);
  labeled(r, sizeof r, "Wh", f.s3, t.watt_hours, 1, 7);
  row2(g.rows[5], l, r);
}

// "VESC 2/3": values only available by polling (COMM_GET_VALUES_SELECTIVE) + tacho + charged Ah/Wh.
inline void oled_build_vesc_b(const SharedState &s, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  grid_begin(g, "VESC 2/3", SCREEN_VESC_B);
  const VescFresh f = vesc_freshness(s, now);
  const VescExt &e = s.vesc_ext;
  char l[32], r[32];

  // Only the first MOSFET sensor is shown (temp_mos2/3 stay polled for the logs);
  // the right column carries the VESC's own averaged input current, the one polled
  // value no other screen has (STATUS_4 current_in is unfiltered).
  labeled(l, sizeof l, "Tmos", f.ext, e.temp_mos1, 1, 5);
  labeled(r, sizeof r, "Iin", f.ext, e.avg_input_current, 1, 6);
  row2(g.rows[0], l, r);

  labeled(l, sizeof l, "Id", f.ext, e.avg_id, 1, 6);
  labeled(r, sizeof r, "Iq", f.ext, e.avg_iq, 1, 6);
  row2(g.rows[1], l, r);

  labeled(l, sizeof l, "Vd", f.ext, e.vd, 2, 6);
  labeled(r, sizeof r, "Vq", f.ext, e.vq, 2, 6);
  row2(g.rows[2], l, r);

  labeled(l, sizeof l, "Tach", f.s5, (float)s.vesc.t.tachometer, 0, 5);
  labeled(r, sizeof r, "Abs", f.ext, (float)e.tacho_abs, 0, 6);
  row2(g.rows[3], l, r);

  // status: bit0 timeout active, bit1 kill switch (output disabled)
  const char *st = "--";
  if (f.ext) {
    switch (e.status & 0x03u) {
      case 0: st = "OK"; break;
      case 1: st = "TIMEOUT"; break;
      case 2: st = "KILLSW"; break;
      default: st = "TO+KILL"; break;
    }
  }
  snprintf(l, sizeof l, "St %s", st);
  if (f.ext) snprintf(r, sizeof r, "id %u", (unsigned)e.vesc_id);
  else snprintf(r, sizeof r, "id --");
  row2(g.rows[4], l, r);

  labeled(l, sizeof l, "AhC", f.s2, s.vesc.t.amp_hours_charged, 3, 6);
  labeled(r, sizeof r, "WhC", f.s3, s.vesc.t.watt_hours_charged, 1, 6);
  row2(g.rows[5], l, r);
}

// "VESC 3/3": inputs (PPM, ADC, PID position), poll counters ("poll off" when VESC_POLL_MS is 0) and CAN frame counters.
inline void oled_build_vesc_c(const SharedState &s, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  grid_begin(g, "VESC 3/3", SCREEN_VESC_C);
  const VescFresh f = vesc_freshness(s, now);
  const vesc_telemetry_t &t = s.vesc.t;
  char l[32], r[32], a[8], b[8], c[8];

  labeled(l, sizeof l, "PPM", f.s6, t.ppm, 2, 6);
  labeled(r, sizeof r, "PID", f.s4, t.pid_pos, 1, 6);
  row2(g.rows[0], l, r);

  if (f.s6) {
    fmt_num(a, sizeof a, t.adc1, 2, 5);
    fmt_num(b, sizeof b, t.adc2, 2, 5);
    fmt_num(c, sizeof c, t.adc3, 2, 5);
    snprintf(l, sizeof l, "ADC %s %s %s", a, b, c);
  } else {
    snprintf(l, sizeof l, "ADC --");
  }
  row1(g.rows[1], l);

  // Poll counters (requests sent / complete replies / CRC-format failures /
  // reply timeouts). A strictly passive build (VESC_POLL_MS 0) never polls:
  // say so instead of showing zeros that look like a dead link.
#if VESC_POLL_MS > 0
  labeled_count(l, sizeof l, "poll", s.vesc_ext.polls_sent, 5);
  labeled_count(r, sizeof r, "ok", s.vesc_ext.replies_ok, 5);
  row2(g.rows[2], l, r);

  labeled_count(l, sizeof l, "bad", s.vesc_ext.replies_bad, 5);
  labeled_count(r, sizeof r, "tmo", s.vesc_ext.timeouts, 5);
  row2(g.rows[3], l, r);
#else
  row2(g.rows[2], "poll off", "ok --");
  row2(g.rows[3], "bad --", "tmo --");
#endif

  labeled_count(l, sizeof l, "frm", s.vesc.frames_total, 5);
  labeled_count(r, sizeof r, "oth", s.vesc.frames_other_id, 5);
  row2(g.rows[4], l, r);

  // last row: no descenders (its baseline is the panel's last row): "misc" = non-status / odd-DLC frames
  labeled_count(l, sizeof l, "misc", s.vesc.frames_dropped, 5);
  labeled_count(r, sizeof r, "busoff", s.can.bus_off_count, 3);
  row2(g.rows[5], l, r);
}

// "GNSS": fix/pDOP, position, altitude/hAcc, speed/sAcc, heading/vAcc, UTC date-time.
inline void oled_build_gnss(const SharedState &s, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  grid_begin(g, "GNSS", SCREEN_GNSS);
  const GnssState &gn = s.gnss;
  const bool live = gnss_live(gn, now);
  const bool pos = live && gn.fix_ok;
  char l[32], r[32], a[12], b[12];

  if (!live) {
    if (gn.last_pvt_ms != 0) snprintf(l, sizeof l, "NO DATA");
    else if (gn.phase == GNSS_PHASE_RUN) snprintf(l, sizeof l, "NO GNSS");
    else snprintf(l, sizeof l, "%s", gnss_phase_name(gn.phase));  // AUTOBAUD / DETECT / CONFIG
  } else if (gn.fix_type == 3 || gn.fix_type == 4) {
    snprintf(l, sizeof l, "3D %usv", (unsigned)gn.num_sv);
  } else if (gn.fix_type == 2) {
    snprintf(l, sizeof l, "2D %usv", (unsigned)gn.num_sv);
  } else if (gn.fix_type == 5) {
    snprintf(l, sizeof l, "TIME ONLY");
  } else {
    snprintf(l, sizeof l, "NO FIX");
  }
  labeled(r, sizeof r, "pDOP", live, (float)gn.pdop_x100 / 100.0f, 1, 5);
  row2(g.rows[0], l, r);

  if (pos) {
    fmt_coord(a, sizeof a, gn.lat_e7, 'N', 'S');
    fmt_coord(b, sizeof b, gn.lon_e7, 'E', 'W');
    snprintf(l, sizeof l, "%s %s", a, b);
  } else {
    snprintf(l, sizeof l, "Pos --");
  }
  row1(g.rows[1], l);

  labeled(l, sizeof l, "Alt", pos, (float)gn.hmsl_mm / 1000.0f, 1, 6, "m");
  labeled(r, sizeof r, "hAcc", live, (float)gn.hacc_mm / 1000.0f, 1, 4, "m");
  row2(g.rows[2], l, r);

  fmt_speed(a, sizeof a, gn, now);
  snprintf(l, sizeof l, "Spd %s", a);
  labeled(r, sizeof r, "sAcc", live, (float)gn.sacc_mm_s / 1000.0f, 2, 5);
  row2(g.rows[3], l, r);

  // heading of motion is noise at rest: shown only while the displayed speed is non-zero
  const bool moving = pos && (float)gn.gspeed_mm_s * SPEED_FACTOR >= SPEED_MIN_SHOW &&
                      gn.sacc_mm_s <= (uint32_t)GNSS_MAX_SACC_MM_S;
  labeled(l, sizeof l, "Hdg", moving, (float)gn.head_mot_e5 / 100000.0f, 1, 5);
  labeled(r, sizeof r, "vAcc", live, (float)gn.vacc_mm / 1000.0f, 1, 4, "m");
  row2(g.rows[4], l, r);

  if (live && gn.time_valid) {
    snprintf(l, sizeof l, "%04u-%02u-%02u %02u:%02u:%02uZ", (unsigned)gn.year, (unsigned)gn.month, (unsigned)gn.day,
             (unsigned)gn.hour, (unsigned)gn.min, (unsigned)gn.sec);
  } else {
    snprintf(l, sizeof l, "UTC --");
  }
  row1(g.rows[5], l);
}

// "SYS <FW_VERSION>": uptime / button presses, heap, CAN state + error counters,
// GNSS driver state, UBX frame counters, reset reason / lock failures. Nothing
// here depends on the display's own refresh counters: a screen that shows the
// number of frames pushed would change with every push and refresh forever.
inline void oled_build_sys(const SharedState &s, uint32_t now, const OledSysInfo &info, OledGrid &g) {
  using namespace oled_detail;
  (void)now;
  char title[24], l[32], r[32], a[12], b[12], c[12];  // counters: "%lu" of a uint32 is <= 10 chars + suffix
  snprintf(title, sizeof title, "SYS %s", FW_VERSION);
  grid_begin(g, title, SCREEN_SYS);

  fmt_uptime(a, sizeof a, info.uptime_s);
  snprintf(l, sizeof l, "up %s", a);
  labeled_count(r, sizeof r, "btn", s.disp.button_presses, 5);
  row2(g.rows[0], l, r);

  fmt_count(a, sizeof a, info.heap_free / 1024u, 4);
  fmt_count(b, sizeof b, info.heap_min / 1024u, 4);
  copy_str(l, sizeof l, "heap ");
  append_str(l, sizeof l, a);
  append_str(l, sizeof l, "k");
  copy_str(r, sizeof r, "min ");
  append_str(r, sizeof r, b);
  append_str(r, sizeof r, "k");
  row2(g.rows[1], l, r);

#if VESC_LINK_UART
  snprintf(l, sizeof l, "UART %s", can_state_name(s.can.state));
  snprintf(r, sizeof r, "%lu", (unsigned long)VESC_UART_BAUD);  // no error counters: CRC failures are "bad" on VESC 3/3
#else
  snprintf(l, sizeof l, "CAN %s", can_state_name(s.can.state));
  snprintf(r, sizeof r, "T%lu R%lu", (unsigned long)s.can.tec, (unsigned long)s.can.rec);  // TEC / REC error counters
#endif
  row2(g.rows[2], l, r);

  // "GNSS RUN 38400 34.10" (phase, baud when known, PROTVER while running)
  int len = snprintf(l, sizeof l, "GNSS %s", gnss_phase_name(s.gnss.phase));
  if (s.gnss.baud != 0 && len > 0 && (size_t)len < sizeof l)
    len += snprintf(l + len, sizeof l - (size_t)len, " %lu", (unsigned long)s.gnss.baud);
  if (s.gnss.phase == GNSS_PHASE_RUN && s.gnss.prot_ver_x100 > 0 && len > 0 && (size_t)len < sizeof l)
    snprintf(l + len, sizeof l - (size_t)len, " %d.%02d", s.gnss.prot_ver_x100 / 100, s.gnss.prot_ver_x100 % 100);
  row1(g.rows[3], l);

  // "ubx <good>/<bad> rd <redetects>"
  fmt_count(a, sizeof a, s.gnss.good_frames, 5);
  fmt_count(b, sizeof b, s.gnss.bad_frames, 4);
  fmt_count(c, sizeof c, s.gnss.redetects, 3);
  copy_str(l, sizeof l, "ubx ");
  append_str(l, sizeof l, a);
  append_str(l, sizeof l, "/");
  append_str(l, sizeof l, b);
  append_str(l, sizeof l, " rd ");
  append_str(l, sizeof l, c);
  row1(g.rows[4], l);

  // last row: no descenders (its baseline is the panel's last row)
  snprintf(l, sizeof l, "rst %s", info.reset_reason ? info.reset_reason : "?");
  labeled_count(r, sizeof r, "lock", s.lock_failures, 3);  // state_lock() timeouts
  row2(g.rows[5], l, r);
}

// "EFFICIENCY": net Wh per distance unit over the last EFF_WINDOW_S ("now") and
// since startup ("avg"), trip distance and net energy, window power and fill,
// run / moving time, mean speed while moving and the energy source ("S3" = VESC
// watt-hour counters, "PxI" = v_in x current_in integration). Every value is
// "--" while the integrator is not running (trip_live), the efficiencies also
// while their own valid flags are clear (too little distance), the window power
// while the window holds no data yet, the mean speed while nothing has moved yet.
inline void oled_build_eff(const SharedState &s, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  grid_begin(g, "EFFICIENCY", SCREEN_EFF);
  const TripState &t = s.trip;
  const bool live = trip_live(t, now);
  char l[32], r[32], a[16];
  // "now 123 Wh/NM": label + space (4) + number + space (1) + unit fill the 21-char row
  const size_t eff_chars = (size_t)kGridChars - 5u - strlen(EFF_UNIT_STR);

  if (live && t.eff_now_valid) {
    fmt_eff(a, sizeof a, t.eff_now, eff_chars);
    snprintf(l, sizeof l, "now %s %s", a, EFF_UNIT_STR);
  } else {
    snprintf(l, sizeof l, "now --");
  }
  row1(g.rows[0], l);

  if (live && t.eff_avg_valid) {
    fmt_eff(a, sizeof a, t.eff_avg, eff_chars);
    snprintf(l, sizeof l, "avg %s %s", a, EFF_UNIT_STR);
  } else {
    snprintf(l, sizeof l, "avg --");
  }
  row1(g.rows[1], l);

  // distance in EFF_DIST_UNIT_M units: "12.34", "123.4", "1234", "12345" (fmt_num drops the decimals);
  // the unit is the one of the efficiency rows. Net energy = drawn - returned.
  labeled(l, sizeof l, "dist", live, t.dist_m / EFF_DIST_UNIT_M, 2, 5);
  if (live) fmt_energy(r, sizeof r, t.wh - t.wh_charged, 8);
  else fmt_dashes(r, sizeof r);
  row2(g.rows[2], l, r);

  // window: average electrical power and how many of its EFF_WINDOW_S seconds hold data.
  // With no closed bucket yet (win_fill_s 0, first second after boot) the integrator
  // publishes 0 W, which is no measurement: "--" rather than a live-looking "0W".
  if (live && t.win_fill_s > 0) fmt_power(a, sizeof a, t.win_p_avg_w, 6);
  else fmt_dashes(a, sizeof a);
  snprintf(l, sizeof l, "P %s", a);
  if (live) snprintf(r, sizeof r, "win %us", (unsigned)t.win_fill_s);
  else snprintf(r, sizeof r, "win --");
  row2(g.rows[3], l, r);

  // "run" (not "time"): a 6-char uptime ("12m34s", "12d03h") after a 4-char label would push the second column
  if (live) {
    fmt_uptime(a, sizeof a, t.run_s);
    snprintf(l, sizeof l, "run %s", a);
    fmt_uptime(a, sizeof a, t.moving_s);
    snprintf(r, sizeof r, "mov %s", a);
  } else {
    snprintf(l, sizeof l, "run --");
    snprintf(r, sizeof r, "mov --");
  }
  row2(g.rows[4], l, r);

  // last row (no descenders): mean speed while moving = distance / moving time in the
  // speed unit ("mean 5.4kn", "mean 12.3km/h"), energy source
  const bool mean_ok = live && t.moving_s > 0;
  const float mean = mean_ok ? t.dist_m / (float)t.moving_s * 1000.0f * SPEED_FACTOR : 0.0f;  // m/s -> mm/s -> unit
  labeled(l, sizeof l, "mean", mean_ok, mean, 1, 4 + strlen(SPEED_UNIT_STR), SPEED_UNIT_STR);
  if (live) snprintf(r, sizeof r, "src %s", t.energy_from_counters ? "S3" : "PxI");
  else snprintf(r, sizeof r, "src --");
  row2(g.rows[5], l, r);
}


#if TOTALS_ENABLE
// "TOTALS": the lifetime record (include/totals.h) as the trip task last published it.
//   motor      1234.5 h      motor running (|rpm| > TOTALS_MIN_RPM), every boot together
//   powered    1502.3 h      VESC link fresh
//   dist       4321.0 NM     GNSS distance while moving, in the efficiency's distance unit
//   energy     812.40kWh     net = drawn - returned (fmt_energy)
//   this boot  mot 1h02m     this boot's motor time (the trip integrator)
//   SAVED 12M34S AGO         what is in flash (totals_save_row)
// Every value is "--" while the trip task has not published (totals_live).
inline bool totals_live(const TotalsState &t, uint32_t now) { return fresh(t.t_ms, now, TRIP_STALE_MS); }

// The last row (upper case, <= 21 chars), first match wins:
//   "NVS ERROR: RAM ONLY"    NVS unavailable or the record unreadable: nothing is written
//   "SAVE FAILED, RETRYING"  the last write failed (next try TOTALS_RETRY_S later)
//   "SAVED 12M34S AGO"       a write succeeded this boot
//   "NOT SAVED YET"          changes waiting for their first save
//   "UP TO DATE"             nothing to save: flash holds these values
inline void totals_save_row(const TotalsState &t, uint32_t now, bool live, char *out, size_t n) {
  using namespace oled_detail;
  if (!live) {
    fmt_dashes(out, n);
  } else if (t.nvs == TOTALS_NVS_ERR) {
    copy_str(out, n, "NVS ERROR: RAM ONLY");
  } else if (t.failed) {
    copy_str(out, n, "SAVE FAILED, RETRYING");
  } else if (t.last_save_ms != 0) {
    // Signed age: the trip task stamps the save after its own clock read, which may lead `now`.
    const int32_t age_ms = (int32_t)(now - t.last_save_ms);
    char a[12];
    fmt_uptime(a, sizeof a, age_ms > 0 ? (uint32_t)age_ms / 1000u : 0u);
    copy_str(out, n, "SAVED ");
    append_str(out, n, a);
    append_str(out, n, " AGO");
    upper_str(out);
  } else {
    copy_str(out, n, t.dirty ? "NOT SAVED YET" : "UP TO DATE");
  }
  clip(out, n, kGridChars);
}

inline void oled_build_totals(const SharedState &s, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  grid_begin(g, "TOTALS", SCREEN_TOTALS);
  const TotalsState &t = s.totals;
  const bool live = totals_live(t, now);
  const TotalsRecord &r = t.life;
  char v[24], l[32];
  // the value column starts at kGridCol2: 10 chars
  const size_t vw = (size_t)(kGridChars - kGridCol2);

  if (live) fmt_num_unit(v, sizeof v, (float)((double)r.motor_s / 3600.0), 1, vw, " h");
  else fmt_dashes(v, sizeof v);
  row2(g.rows[0], "motor", v);

  if (live) fmt_num_unit(v, sizeof v, (float)((double)r.on_s / 3600.0), 1, vw, " h");
  else fmt_dashes(v, sizeof v);
  row2(g.rows[1], "powered", v);

  if (live) {
    char unit[8] = " ";
    append_str(unit, sizeof unit, EFF_DIST_UNIT_STR);
    fmt_num_unit(v, sizeof v, (float)((double)r.dist_m / (double)EFF_DIST_UNIT_M), 1, vw, unit);
  } else {
    fmt_dashes(v, sizeof v);
  }
  row2(g.rows[2], "dist", v);

  if (live) fmt_energy(v, sizeof v, (float)((double)r.wh - (double)r.whc), vw);
  else fmt_dashes(v, sizeof v);
  row2(g.rows[3], "energy", v);

  if (trip_live(s.trip, now)) {
    char a[12];
    fmt_uptime(a, sizeof a, s.trip.motor_s);
    copy_str(v, sizeof v, "mot ");
    append_str(v, sizeof v, a);
  } else {
    copy_str(v, sizeof v, "mot --");
  }
  row2(g.rows[4], "this boot", v);

  // last row: no descenders (its baseline is the panel's last row)
  totals_save_row(t, now, live, l, sizeof l);
  row1(g.rows[5], l);
}
#endif  // TOTALS_ENABLE

#if PUMP_ENABLE
// "PUMP" (every row <= 21 chars):
//   RUN 62% warm            mode, commanded duty, reason ("--" while the pump task is silent)
//   Tmot 58.3  tgt 66.0     motor temperature ("--" stale, "n/a" no sensor) and its target
//   Tfet 41.0  tgt 66.0     MOSFET temperature and its target
//   Lim M85/100 F85/100     the VESC's l_temp_motor/fet_start/end in use
//   dec 0.15   3450rpm      l_temp_accel_dec, and with a tach (PUMP_TACH_ENABLE) the pump's
//                           speed, or what is wrong with it (pump_tach_row). Without a tach the
//                           right side says where the limits came from: the VESC (its firmware,
//                           "VESC 6.06"), or DEF = PUMP_DEFAULT_* (wait / cfg / new FW / bad ...).
//                           With a tach that source is only in the PUMP log line.
//   err +7.7   int 12.0     the loop's error (C over target, max of both sensors) and I term
// Where the pump's limits came from (<= 12 chars): "VESC 6.06", or DEF = PUMP_DEFAULT_* and why.
inline void pump_limits_src(const VescLimitsState &v, char *out, size_t n) {
  using namespace oled_detail;
  if (v.result == MCCONF_OK) {
    copy_str(out, n, "VESC ");
    append_str(out, n, v.fw);
    return;
  }
  switch (v.result) {
    case MCCONF_NONE: copy_str(out, n, PUMP_READS_MCCONF ? "DEF wait" : "DEF cfg"); break;
    case MCCONF_UNKNOWN_SIG: copy_str(out, n, "DEF new FW"); break;
    case MCCONF_IMPLAUSIBLE: copy_str(out, n, "DEF bad lim"); break;
    default: copy_str(out, n, "DEF bad rep"); break;
  }
}

#if PUMP_TACH_ENABLE
// The tach on the PUMP page (<= 10 chars): the word when something needs attention, the rpm
// otherwise ("~" while the window is still filling), "--" while the pump task is silent.
inline void pump_tach_row(const PumpStatus &p, bool live, char *out, size_t n) {
  using namespace oled_detail;
  if (!live || p.tach == PUMP_TACH_NONE) {
    fmt_dashes(out, n);
  } else if (p.tach == PUMP_TACH_STALL || p.tach == PUMP_TACH_NO_SIGNAL || p.tach == PUMP_TACH_RUNS_AT_0 ||
             p.tach == PUMP_TACH_ERR) {
    copy_str(out, n, pump_tach_str(p.tach));
  } else {
    fmt_num_unit(out, n, p.tach_rpm, 0, p.tach_full ? 10 : 9, "rpm");
    if (!p.tach_full) append_str(out, n, "~");
  }
  clip(out, n, 10);
}
#endif

inline void oled_build_pump(const SharedState &s, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  grid_begin(g, "PUMP", SCREEN_PUMP);
  const PumpStatus &p = s.pump;
  const VescFresh f = vesc_freshness(s, now);
  const bool live = fresh(p.t_ms, now, HB_MAX_PUMP_MS);
  char l[32], r[32], buf[40];

  if (live) snprintf(buf, sizeof buf, "%s %.0f%% %s", pump_mode_str(p.mode), (double)p.duty_pct, pump_why_str(p.why));
  else fmt_dashes(buf, sizeof buf);
  row1(g.rows[0], buf);

  const float temps[2] = {s.vesc.t.temp_motor, s.vesc.t.temp_fet};
  const float tgts[2] = {p.tgt_mot, p.tgt_fet};
  const char *const labels[2] = {"Tmot", "Tfet"};
  for (int i = 0; i < 2; ++i) {
    if (f.s4 && !temp_plausible(temps[i])) snprintf(l, sizeof l, "%s n/a", labels[i]);
    else labeled(l, sizeof l, labels[i], f.s4, temps[i], 1, 5);
    labeled(r, sizeof r, "tgt", p.t_ms != 0, tgts[i], 1, 5);
    row2(g.rows[1 + i], l, r);
  }

  bool from_vesc = false;
  const VescTempLimits lim = pump_limits(s.vesc_limits, &from_vesc);
  snprintf(buf, sizeof buf, "Lim M%.0f/%.0f F%.0f/%.0f", (double)lim.mot_start, (double)lim.mot_end,
           (double)lim.fet_start, (double)lim.fet_end);
  row1(g.rows[3], buf);

#if PUMP_TACH_ENABLE
  (void)from_vesc;  // the source is in the log line; the row shows the pump's speed instead
  snprintf(l, sizeof l, "dec %.2f", (double)lim.accel_dec);
  pump_tach_row(p, live, r, sizeof r);
  row2(g.rows[4], l, r);
#else
  char src[16];
  pump_limits_src(s.vesc_limits, src, sizeof src);
  snprintf(buf, sizeof buf, "dec %.2f %s", (double)lim.accel_dec, src);
  row1(g.rows[4], buf);
#endif

  char num[8];
  if (live && p.have_err) fmt_num(num, sizeof num, p.err_c, 1, 5);
  else fmt_dashes(num, sizeof num);
  copy_str(l, sizeof l, live && p.have_err && p.err_c > 0.0f ? "err +" : "err ");
  append_str(l, sizeof l, num);
  labeled(r, sizeof r, "int", live, p.integ, 1, 5);
  row2(g.rows[5], l, r);
}
#endif  // PUMP_ENABLE

// Builds the frame for `screen` (unknown indices fall back to the main screen).
#if DEADMAN_UI_ENABLE
// The dead-man alarm: shown INSTEAD of whatever page is selected, for as long as the
// firmware is holding the motor cut. It answers, in order, the four questions the person
// at the helm actually has: is the motor off, why, did the cut really reach the VESC, and
// how do I get going again.
//
//   [MOTOR CUT         !]
//   TRIPPED    no tag 12s      (or "FAULT      CUT FAILED")
//   worst gap while armed      diagnostic for sizing DEADMAN_TIMEOUT_MS
//   tag -62dBm  n 18431
//   VESC KILLSW OK             the closed loop: OK / WAIT / FAILED / no poll
//   hold btn: reset            or "hold btn: need tag" when the tag is not present
inline void oled_build_deadman_alarm(const SharedState &s, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  const DeadmanState &d = s.deadman;
  memset(&g, 0, sizeof g);
  copy_str(g.title, sizeof g.title, "MOTOR CUT");
  copy_str(g.page, sizeof g.page, "!");
  g.nrows = kGridRows;

  char l[32], r[32], a[12];
  const uint32_t age = deadman_pub_age(d, now);  // the freshest ENROLLED tag

  if (d.state == DM_FAULT) {
    row2(g.rows[0], "FAULT", "CUT FAILED");
  } else if (age != 0xFFFFFFFFu) {
    fmt_age_short(a, sizeof a, age);
    snprintf(r, sizeof r, "no tag %s", a);
    row2(g.rows[0], "TRIPPED", r);
  } else {
    row2(g.rows[0], "TRIPPED", "never seen");
  }

  fmt_count(a, sizeof a, d.gap_max_ms, 5);
  snprintf(l, sizeof l, "gapmax %sms", a);
  row1(g.rows[1], l);

  // Both tags on one row, same "1:-62" encoding as the main screen, plus the total
  // advert count so a tag that has gone deaf is obvious next to one that has not.
  {
    char slots[16] = {0};
    uint32_t total = 0;
    for (unsigned i = 0; i < DEADMAN_TAGS; ++i) {
      if (!d.tags[i].configured) continue;
      total += d.tags[i].reports;
      char val[6], one[10];
      if (!d.tags[i].enrolled) copy_str(val, sizeof val, "off");
      else if (d.tags[i].t_ms == 0 || age_ms(d.tags[i].t_ms, now) > (uint32_t)DEADMAN_WARN_MS)
        copy_str(val, sizeof val, " --");
      else {
        int rssi = (int)d.tags[i].rssi;
        if (rssi < -99) rssi = -99;
        if (rssi > 0) rssi = 0;
        snprintf(val, sizeof val, "%3d", rssi);
      }
      snprintf(one, sizeof one, "%s%u:%s", slots[0] ? " " : "", i + 1u, val);
      append_str(slots, sizeof slots, one);
    }
    if (slots[0] == '\0') copy_str(slots, sizeof slots, "no tags");
    fmt_count(a, sizeof a, total, 5);
    snprintf(r, sizeof r, "n %s", a);
    row2(g.rows[2], slots, r);
  }

  // The closed loop. "NOT CUT" is the one that means "the wiring or the VESC config is
  // wrong and the motor may still be live" - it must not read like a mere warning.
  switch (d.confirm) {
    case DM_CONFIRM_OK: row1(g.rows[3], "VESC: KILLSW ok"); break;
    case DM_CONFIRM_WAIT: row1(g.rows[3], "VESC: waiting..."); break;
    case DM_CONFIRM_FAILED: row1(g.rows[3], "VESC: NOT CUT!"); break;
    case DM_CONFIRM_STALE: row1(g.rows[3], "VESC: no reply"); break;
    default: row1(g.rows[3], "VESC: --"); break;
  }

  // Row 4: how often this has happened, plus refused resets when there have been any
  // (that pair is the "I keep pressing the button and nothing happens" diagnosis).
  char b[8];
  if (d.resets_refused) {
    fmt_count(a, sizeof a, d.trips, 4);
    fmt_count(b, sizeof b, d.resets_refused, 4);
    snprintf(l, sizeof l, "trips %s", a);
    snprintf(r, sizeof r, "refused %s", b);
    row2(g.rows[4], l, r);
  } else {
    fmt_count(a, sizeof a, d.trips, 5);
    snprintf(l, sizeof l, "trips %s", a);
    row1(g.rows[4], l);
  }

  // Row 5 sits on the panel's last pixel row, so it must carry no descenders (g/j/p/q/y):
  // all upper case, which is what an alarm wants anyway. It says whether the reset gesture
  // can succeed right now, so nobody holds the button wondering why nothing happens.
  if (d.state == DM_FAULT) row1(g.rows[5], "FIX WIRING - NO RESET");
  else if (age <= (uint32_t)DEADMAN_RESET_FRESH_MS) row1(g.rows[5], "HOLD BTN: RESET");
  else row1(g.rows[5], "HOLD BTN: NEED TAG");
}
#endif  // DEADMAN_UI_ENABLE

#if PUMP_TACH_ENABLE
// The pump alarm: shown INSTEAD of whatever page is selected while the tach reports STALL or
// NO_SIGNAL, until a press dismisses that alarm (DisplayStats::pump_ack = its sequence number,
// PumpStatus::tach_alarms). A later stall is a new number and shows again. It says what is
// wrong, what the firmware does about it, what to check and what it means for the ride.
//
//   [PUMP              !]
//   PUMP STALLED        12s    (or "NO PUMP TACH SIGNAL": not one pulse since boot)
//   duty 100%        0rpm
//   forced to 100%             (or "alert only")
//   chk fuse/plug/hose         (NO SIGNAL: "chk FG wire / diode")
//   motor may derate
//   PRESS BTN: DISMISS
inline bool pump_alarm_active(const SharedState &s) {
  return pump_tach_alarm(s.pump.tach) && s.pump.tach_alarms != s.disp.pump_ack;
}

inline void oled_build_pump_alarm(const SharedState &s, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  const PumpStatus &p = s.pump;
  memset(&g, 0, sizeof g);
  copy_str(g.title, sizeof g.title, "PUMP");
  copy_str(g.page, sizeof g.page, "!");
  g.nrows = kGridRows;

  const bool stalled = p.tach == PUMP_TACH_STALL;
  char l[32], r[32], a[8];
  if (stalled) {
    if (p.tach_since_ms) {
      // Signed: the pump task stamps with its own clock read, which may be a little after `now`.
      const int32_t age = (int32_t)(now - p.tach_since_ms);
      fmt_age_short(a, sizeof a, age > 0 ? (uint32_t)age : 0u);
      row2(g.rows[0], "PUMP STALLED", a);
    } else {
      row1(g.rows[0], "PUMP STALLED");
    }
  } else {
    row1(g.rows[0], "NO PUMP TACH SIGNAL");
  }

  char num[8];
  fmt_num(num, sizeof num, p.duty_pct, 0, 3);
  snprintf(l, sizeof l, "duty %s%%", num);
  fmt_num_unit(r, sizeof r, p.tach_rpm, 0, 10, "rpm");
  row2(g.rows[1], l, r);

  if (PUMP_TACH_STALL_DUTY_PCT > 0) {
    snprintf(l, sizeof l, "forced to %d%%", PUMP_TACH_STALL_DUTY_PCT);
    row1(g.rows[2], l);
  } else {
    row1(g.rows[2], "alert only");
  }
  row1(g.rows[3], stalled ? "chk fuse/plug/hose" : "chk FG wire / diode");
  row1(g.rows[4], "motor may derate");
  // The last grid row sits on the panel's bottom pixel row: no descenders, so upper case.
  row1(g.rows[5], "PRESS BTN: DISMISS");
}
#endif  // PUMP_TACH_ENABLE

#if OTA_ENABLE
// FW UPDATE: shown for the whole of update mode (src/ota.cpp publishes g_state.ota).
//
//   [FW UPDATE      45%]       the percentage while receiving
//   AP VESC-DASH-OTA
//   PASS 48213907
//   http://192.168.4.1
//   RX 612K/1105K              or STARTING WI-FI / AP NOT UP - CHECK PWR / WAITING FOR PHONE /
//                              PHONE JOINED / DONE - REBOOTING / UPLOAD FAILED
//   new 8ea95aa                the version arriving, else "run <running version>", or why it failed
//   HOLD BTN: EXIT 4:32        the idle countdown; DO NOT POWER OFF while the image is written
inline void oled_build_ota(const OtaStatus &o, uint32_t now, OledGrid &g) {
  using namespace oled_detail;
  memset(&g, 0, sizeof g);
  copy_str(g.title, sizeof g.title, "FW UPDATE");
  g.nrows = kGridRows;
  char l[40], a[12], b[12];

  if (o.phase == OTA_PHASE_RECEIVING && o.total != 0) {
    const uint32_t pct = (uint32_t)((uint64_t)o.received * 100u / o.total);
    snprintf(g.page, sizeof g.page, "%u%%", (unsigned)(pct > 100u ? 100u : pct));
  } else if (o.phase == OTA_PHASE_DONE) {
    copy_str(g.page, sizeof g.page, "100%");
  }

  snprintf(l, sizeof l, "AP %s", o.ssid);
  row1(g.rows[0], l);
  snprintf(l, sizeof l, "PASS %s", o.pass[0] ? o.pass : "--");
  row1(g.rows[1], l);
  snprintf(l, sizeof l, "http://%s", o.ip[0] ? o.ip : "--");
  row1(g.rows[2], l);

  switch (o.phase) {
    case OTA_PHASE_STARTING: row1(g.rows[3], "STARTING WI-FI"); break;
    case OTA_PHASE_RECEIVING:
      fmt_count(a, sizeof a, o.received / 1024u, 5);
      fmt_count(b, sizeof b, o.total / 1024u, 5);
      snprintf(l, sizeof l, "RX %sK/%sK", a, b);
      row1(g.rows[3], l);
      break;
    case OTA_PHASE_DONE: row1(g.rows[3], "DONE - REBOOTING"); break;
    case OTA_PHASE_FAILED: row1(g.rows[3], "UPLOAD FAILED"); break;
    default:  // WAITING
      if (!o.ap_up) row1(g.rows[3], "AP NOT UP - CHECK PWR");
      else if (o.stations == 0) row1(g.rows[3], "WAITING FOR PHONE");
      else row1(g.rows[3], "PHONE JOINED");
      break;
  }

  if (o.phase == OTA_PHASE_FAILED && o.detail[0]) {
    row1(g.rows[4], o.detail);
  } else {
    const char *ver = o.new_ver[0] ? o.new_ver : o.run_ver[0] ? o.run_ver : "--";
    snprintf(l, sizeof l, "%s %s", o.new_ver[0] ? "new" : "run", ver);
    row1(g.rows[4], l);
  }

  // Last row: on the panel's last pixel row, so upper case only (no descenders).
  if (o.phase == OTA_PHASE_RECEIVING || o.phase == OTA_PHASE_DONE) {
    row1(g.rows[5], "DO NOT POWER OFF");
  } else if (o.idle_deadline_ms == 0) {
    row1(g.rows[5], "HOLD BTN: EXIT");
  } else {
    const int32_t left = (int32_t)(o.idle_deadline_ms - now);
    const uint32_t s = left > 0 ? ((uint32_t)left + 999u) / 1000u : 0u;  // round up: "0:00" only at the very end
    snprintf(l, sizeof l, "HOLD BTN: EXIT %lu:%02lu", (unsigned long)(s / 60u), (unsigned long)(s % 60u));
    row1(g.rows[5], l);
  }
}

// UPDATE REFUSED: what stopped the hold and what to do about it, for OTA_REFUSED_SHOW_MS.
inline void oled_build_ota_refused(uint8_t reason, OledGrid &g) {
  using namespace oled_detail;
  memset(&g, 0, sizeof g);
  copy_str(g.title, sizeof g.title, "FW UPDATE");
  copy_str(g.page, sizeof g.page, "!");
  g.nrows = kGridRows;
  row1(g.rows[0], "UPDATE REFUSED");
  row1(g.rows[1], ota_refusal_text(reason));
  row1(g.rows[2], ota_refusal_hint(reason));
}
#endif  // OTA_ENABLE

inline void oled_build_frame(const SharedState &s, uint32_t now, uint8_t screen, const OledSysInfo &info,
                             OledFrame &out) {
  memset(&out, 0, sizeof out);
  out.screen = screen < SCREEN_COUNT ? screen : (uint8_t)SCREEN_MAIN;
#if OTA_ENABLE
  // Update mode runs nothing that could feed another screen (and no dead-man: the cut is held).
  if (s.ota.phase != OTA_PHASE_OFF) {
    out.overlay = OVERLAY_OTA;
    oled_build_ota(s.ota, now, out.grid);
    return;
  }
#endif
#if DEADMAN_UI_ENABLE
  // A cut motor takes the whole panel, whatever page the helmsman had selected, and the
  // long press that would normally go back to MAIN clears the latch instead.
  if (s.deadman.state == DM_TRIPPED || s.deadman.state == DM_FAULT) {
    out.overlay = OVERLAY_DEADMAN;
    oled_build_deadman_alarm(s, now, out.grid);
    return;
  }
#endif
#if OTA_ENABLE
  // Signed age: the display task stamps the refusal with its loop clock, which may trail `now`.
  if (s.disp.ota_refused != OTA_OK && s.disp.ota_refused_ms != 0 &&
      (int32_t)(now - s.disp.ota_refused_ms) < (int32_t)OTA_REFUSED_SHOW_MS) {
    out.overlay = OVERLAY_OTA_REFUSED;
    oled_build_ota_refused(s.disp.ota_refused, out.grid);
    return;
  }
#endif
#if PUMP_TACH_ENABLE
  // A pump that should run but does not turn: the motor heats up from here, so it takes the
  // panel until a press acknowledges it (the PUMP page keeps saying STALL after that).
  if (pump_alarm_active(s)) {
    out.overlay = OVERLAY_PUMP;
    oled_build_pump_alarm(s, now, out.grid);
    return;
  }
#endif
  switch (out.screen) {
    case SCREEN_EFF: oled_build_eff(s, now, out.grid); break;
    case SCREEN_VESC_A: oled_build_vesc_a(s, now, out.grid); break;
    case SCREEN_VESC_B: oled_build_vesc_b(s, now, out.grid); break;
    case SCREEN_VESC_C: oled_build_vesc_c(s, now, out.grid); break;
#if PUMP_ENABLE
    case SCREEN_PUMP: oled_build_pump(s, now, out.grid); break;
#endif
    case SCREEN_GNSS: oled_build_gnss(s, now, out.grid); break;
    case SCREEN_SYS: oled_build_sys(s, now, info, out.grid); break;
#if TOTALS_ENABLE
    case SCREEN_TOTALS: oled_build_totals(s, now, out.grid); break;
#endif
    default: oled_build_main(s, now, out.main); break;
  }
}

// The frame shown before any data arrived: main screen with every value "--",
// clock "--:--", fix line "BOOT", CAN line = firmware version.
inline void oled_boot_frame(OledFrame &out) {
  using namespace oled_detail;
  memset(&out, 0, sizeof out);
  out.screen = SCREEN_MAIN;
  OledMain &m = out.main;
  fmt_dashes(m.speed, sizeof m.speed);
  snprintf(m.unit, sizeof m.unit, "%s", SPEED_UNIT_STR);
  clip(m.unit, sizeof m.unit, kUnitChars);
  snprintf(m.clock, sizeof m.clock, "--:--");
  snprintf(m.fix, sizeof m.fix, "BOOT");
  snprintf(m.can, sizeof m.can, "%s", FW_VERSION);
  clip(m.can, sizeof m.can, kCanChars);
  static const char *const kLabels[8] = {"BV", "BA", "MA", "PW", "TF", "TM", "RPM", "WH"};
  for (int i = 0; i < 8; ++i) {
    set_cell(m.cells[i], kLabels[i]);
    cell_dashes(m.cells[i]);
  }
}
