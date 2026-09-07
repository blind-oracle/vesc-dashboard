// u8g2 HAL for ESP-IDF (see include/u8g2_hal_idf.h).
//
// Byte protocol: u8g2 brackets every I2C transaction with START_TRANSFER / END_TRANSFER
// and hands the bytes over in one or more SEND calls in between (the control byte 0x00 /
// 0x40 first, then commands or a run of frame-buffer bytes). The u8x8 CAD layer never
// puts more than 32 bytes into one transaction, so the accumulated transaction is sent
// with a single i2c_master_transmit() at END_TRANSFER.
#include "u8g2_hal_idf.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

namespace {

U8g2HalIdfConfig s_cfg{};
i2c_master_bus_handle_t s_bus = nullptr;
i2c_master_dev_handle_t s_dev = nullptr;
esp_err_t s_last_err = ESP_OK;

// One transaction. 32 bytes is u8g2's own limit; the margin turns a library change into
// a logged ESP_ERR_INVALID_SIZE instead of a buffer overrun.
uint8_t s_buf[128];
size_t s_len = 0;
bool s_overflow = false;

void note_error(esp_err_t err) {
  if (err != ESP_OK) s_last_err = err;
}

}  // namespace

esp_err_t u8g2_hal_idf_init(const U8g2HalIdfConfig &cfg) {
  if (s_dev) return ESP_OK;
  s_cfg = cfg;
  if (!s_bus) {
    i2c_master_bus_config_t bus = {};
    bus.i2c_port = I2C_NUM_0;
    bus.sda_io_num = (gpio_num_t)cfg.sda;
    bus.scl_io_num = (gpio_num_t)cfg.scl;
    bus.clk_source = I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt = 7;
    bus.flags.enable_internal_pullup = true;  // modules carry their own pull-ups; the weak internal ones do no harm
    const esp_err_t err = i2c_new_master_bus(&bus, &s_bus);
    if (err != ESP_OK) {
      s_bus = nullptr;
      return err;
    }
  }
  i2c_device_config_t dev = {};
  dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev.device_address = cfg.addr7;
  dev.scl_speed_hz = cfg.i2c_hz;
  const esp_err_t err = i2c_master_bus_add_device(s_bus, &dev, &s_dev);
  if (err != ESP_OK) s_dev = nullptr;
  return err;
}

esp_err_t u8g2_hal_idf_probe() {
  if (!s_bus) return ESP_ERR_INVALID_STATE;
  return i2c_master_probe(s_bus, s_cfg.addr7, (int)s_cfg.timeout_ms);
}

esp_err_t u8g2_hal_idf_take_last_error() {
  const esp_err_t e = s_last_err;
  s_last_err = ESP_OK;
  return e;
}

uint8_t u8g2_hal_idf_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
  (void)u8x8;  // the device address comes from s_cfg (set once), not from u8x8->i2c_address
  switch (msg) {
    case U8X8_MSG_BYTE_INIT:  // bus and device are created in u8g2_hal_idf_init()
      return s_dev != nullptr;
    case U8X8_MSG_BYTE_SET_DC:  // no D/C line on I2C
      return 1;
    case U8X8_MSG_BYTE_START_TRANSFER:
      s_len = 0;
      s_overflow = false;
      return 1;
    case U8X8_MSG_BYTE_SEND: {
      if (s_len + arg_int > sizeof s_buf) {
        s_overflow = true;
        return 0;
      }
      memcpy(s_buf + s_len, arg_ptr, arg_int);
      s_len += arg_int;
      return 1;
    }
    case U8X8_MSG_BYTE_END_TRANSFER: {
      if (!s_dev) {
        note_error(ESP_ERR_INVALID_STATE);
        return 0;
      }
      if (s_overflow) {
        note_error(ESP_ERR_INVALID_SIZE);
        return 0;
      }
      if (s_len == 0) return 1;
      const esp_err_t err = i2c_master_transmit(s_dev, s_buf, s_len, (int)s_cfg.timeout_ms);
      note_error(err);
      return err == ESP_OK;
    }
    default:
      return 0;
  }
}

uint8_t u8g2_hal_idf_gpio_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
  (void)arg_ptr;
  switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      if (s_cfg.rst >= 0) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << s_cfg.rst;
        io.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io);
        gpio_set_level((gpio_num_t)s_cfg.rst, 1);  // not in reset
      }
      return 1;
    case U8X8_MSG_DELAY_MILLI:  // reset pulse timing and the init sequence's waits
      if (arg_int) vTaskDelay(pdMS_TO_TICKS(arg_int));
      return 1;
    case U8X8_MSG_DELAY_10MICRO:
      esp_rom_delay_us(10u * arg_int);
      return 1;
    case U8X8_MSG_DELAY_100NANO:  // 100 ns steps are below what we can time; 1 us is safe
    case U8X8_MSG_DELAY_NANO:
      esp_rom_delay_us(1);
      return 1;
    case U8X8_MSG_DELAY_I2C:  // only the software I2C byte function uses this
      esp_rom_delay_us(arg_int <= 2 ? 5 : 2);
      return 1;
    case U8X8_MSG_GPIO_RESET:
      if (s_cfg.rst >= 0) gpio_set_level((gpio_num_t)s_cfg.rst, arg_int ? 1 : 0);
      return 1;
    default:
      // CS / DC / SPI lines and menu keys: unused with hardware I2C. Report "high" / not pressed.
      u8x8_SetGPIOResult(u8x8, 1);
      return 1;
  }
}
