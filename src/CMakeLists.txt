# Project sources: every .cpp in src/; headers in ../include. ESP-IDF 6 no longer lets the
# "driver" umbrella component pull in the individual drivers, so the ones this firmware uses
# are listed explicitly (TWAI, I2C master, UART, GPIO, RMT for a WS2812 status LED, LEDC for
# the coolant pump's PWM, PCNT for its tachometer), plus esp_timer for the millisecond clock, the u8g2 component from
# components/u8g2, nvs_flash for the lifetime totals' record (src/totals.cpp), bt for the dead-man's BLE
# beacon scan (it compiles to nothing while CONFIG_BT_ENABLED is off), and for update mode (src/ota.cpp) the Wi-Fi access point, the HTTP server and the
# OTA slot API (esp_wifi, esp_netif, esp_event, esp_phy, esp_http_server, app_update,
# esp_app_format). The linker drops all of those from a build with OTA_ENABLE 0.
file(GLOB app_sources ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "../include"
                       PRIV_REQUIRES esp_driver_twai esp_driver_i2c esp_driver_uart esp_driver_gpio esp_driver_rmt esp_driver_ledc esp_driver_pcnt esp_timer log u8g2 bt nvs_flash
                                     esp_wifi esp_netif esp_event esp_phy esp_http_server app_update esp_app_format)
