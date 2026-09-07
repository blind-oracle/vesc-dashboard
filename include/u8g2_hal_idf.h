// u8g2 hardware abstraction for ESP-IDF: an SSD13xx-class OLED on the driver/i2c_master.h
// bus, an optional reset GPIO, FreeRTOS / ROM delays. Used by src/display_oled.cpp only
// (the display task is the sole caller after display_start()).
#pragma once

#include <stdint.h>

#include <esp_err.h>
#include <u8g2.h>

struct U8g2HalIdfConfig {
  int sda;              // GPIO numbers
  int scl;
  int rst;              // reset GPIO, -1 = not wired (RES tied high on the module)
  uint32_t i2c_hz;      // SCL frequency
  uint8_t addr7;        // 7-bit device address (0x3C / 0x3D)
  uint32_t timeout_ms;  // per I2C transaction; a stuck bus costs at most this per transfer
};

// Creates the I2C master bus and the device handle once (later calls return ESP_OK).
esp_err_t u8g2_hal_idf_init(const U8g2HalIdfConfig &cfg);

// Zero-length probe of the display address: ESP_OK = ACK, ESP_ERR_NOT_FOUND = nothing at
// that address, ESP_ERR_TIMEOUT = the bus never idles (no pull-ups, short, unpowered module),
// ESP_ERR_INVALID_STATE = u8g2_hal_idf_init() has not succeeded.
esp_err_t u8g2_hal_idf_probe();

// Error of the failed transfers since the previous call (ESP_OK when every transfer went
// through); the call clears it. u8g2 itself ignores transfer results, this is how the
// display task learns that a frame or the init sequence did not reach the panel.
esp_err_t u8g2_hal_idf_take_last_error();

// The two u8x8 callbacks for u8g2_Setup_*_i2c_*().
uint8_t u8g2_hal_idf_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_hal_idf_gpio_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
