# Project sources: every .cpp in src/; headers in ../include. ESP-IDF 6 no longer lets the
# "driver" umbrella component pull in the individual drivers, so the ones this firmware uses
# are listed explicitly (TWAI, I2C master, UART, GPIO, RMT for a WS2812 status LED), plus
# esp_timer for the millisecond clock, the u8g2 component from components/u8g2, and bt +
# nvs_flash for the dead-man's BLE beacon scan (bt compiles to nothing while CONFIG_BT_ENABLED
# is off).
file(GLOB app_sources ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "../include"
                       PRIV_REQUIRES esp_driver_twai esp_driver_i2c esp_driver_uart esp_driver_gpio esp_driver_rmt esp_timer log u8g2 bt nvs_flash)
