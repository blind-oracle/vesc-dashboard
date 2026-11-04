// Status LED (include/status_led.h): a plain GPIO, or with LED_WS2812 one WS2812 RGB LED
// driven by an RMT TX channel and a bytes encoder.
//
// WS2812 notes:
//  * 10 MHz RMT resolution; bit 0 = 0.3 us high / 0.9 us low, bit 1 = 0.9 us high /
//    0.3 us low (the values of Espressif's led_strip example), MSB first, colour order
//    G-R-B. The line idles low, and the >= 280 us latch gap is implicit: the supervisor
//    writes the LED at most twice a second.
//  * Non-blocking transmit (queue_nonblocking): a full queue skips one blink rather than
//    stalling the supervisor that feeds the task watchdog.
//  * status_led_init() sends one "off" frame and waits for it, so no RMT interrupt is in
//    flight when the first BLE controller enable writes its PHY calibration to flash (the
//    same caution main.cpp takes for the TWAI node).
// Only called from the supervisor task (app_main), so no locking.
#include "status_led.h"

#include <esp_err.h>
#include <esp_log.h>

#include "config.h"

#if LED_WS2812
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#else
#include <driver/gpio.h>
#endif

static_assert(PIN_LED >= 0 && PIN_LED <= BOARD_GPIO_MAX, "PIN_LED must be a GPIO of this board");
static_assert(LED_WS2812_LEVEL >= 1 && LED_WS2812_LEVEL <= 255, "LED_WS2812_LEVEL must be 1..255");

static const char *TAG = "main";

static bool s_ok = false;

#if LED_WS2812

constexpr uint32_t kResolutionHz = 10 * 1000 * 1000;  // 0.1 us per tick
constexpr uint16_t kT0H = 3, kT0L = 9, kT1H = 9, kT1L = 3;  // ticks

static rmt_channel_handle_t s_chan = nullptr;
static rmt_encoder_handle_t s_enc = nullptr;
static uint8_t s_grb[3];  // must stay valid until the transmission is done: static

static void ws2812_write(uint8_t r, uint8_t g, uint8_t b, bool wait) {
  s_grb[0] = g;
  s_grb[1] = r;
  s_grb[2] = b;
  rmt_transmit_config_t tx = {};
  tx.loop_count = 0;
  tx.flags.eot_level = 0;
  tx.flags.queue_nonblocking = 1;
  if (rmt_transmit(s_chan, s_enc, s_grb, sizeof s_grb, &tx) == ESP_OK && wait) rmt_tx_wait_all_done(s_chan, 10);
}

bool status_led_init() {
  rmt_tx_channel_config_t cfg = {};
  cfg.gpio_num = (gpio_num_t)PIN_LED;
  cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  cfg.resolution_hz = kResolutionHz;
  cfg.mem_block_symbols = 48;  // one frame is 24 symbols; 48 is the smallest block on every RMT-capable chip
  cfg.trans_queue_depth = 2;
  esp_err_t err = rmt_new_tx_channel(&cfg, &s_chan);
  if (err == ESP_OK) {
    rmt_bytes_encoder_config_t enc = {};
    enc.bit0.level0 = 1;
    enc.bit0.duration0 = kT0H;
    enc.bit0.level1 = 0;
    enc.bit0.duration1 = kT0L;
    enc.bit1.level0 = 1;
    enc.bit1.duration0 = kT1H;
    enc.bit1.level1 = 0;
    enc.bit1.duration1 = kT1L;
    enc.flags.msb_first = 1;
    err = rmt_new_bytes_encoder(&enc, &s_enc);
  }
  if (err == ESP_OK) err = rmt_enable(s_chan);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "status LED: WS2812 on GPIO%d: RMT setup failed: %s (no status LED)", PIN_LED, esp_err_to_name(err));
    return false;
  }
  s_ok = true;
  ws2812_write(0, 0, 0, true);
  return true;
}

void status_led_set(bool on, bool fresh) {
  if (!s_ok) return;
  const uint8_t v = on ? (uint8_t)LED_WS2812_LEVEL : 0u;
  ws2812_write(fresh ? 0u : v, fresh ? v : 0u, 0u, false);
}

#else  // plain GPIO LED, active high

bool status_led_init() {
  gpio_config_t led = {};
  led.pin_bit_mask = 1ULL << PIN_LED;
  led.mode = GPIO_MODE_OUTPUT;
  const esp_err_t err = gpio_config(&led);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "status LED: GPIO%d setup failed: %s (no status LED)", PIN_LED, esp_err_to_name(err));
    return false;
  }
  s_ok = true;
  gpio_set_level((gpio_num_t)PIN_LED, 0);
  return true;
}

void status_led_set(bool on, bool) {
  if (s_ok) gpio_set_level((gpio_num_t)PIN_LED, on ? 1 : 0);
}

#endif  // LED_WS2812
