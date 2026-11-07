# AI-warning

Be advised that this whole repo is AI-generated, so take it with a grain of salt. But the code generally works very well for me.

# vesc-dashboard

ESP32-C6 firmware that listens to a VESC motor controller on CAN, polls it once a second for the few values its broadcasts lack (fault code, per-MOSFET temperatures, id/iq, vd/vq, absolute tachometer), reads ground speed and time from a u-blox GNSS receiver at 5 Hz, shows everything on a 128x64 monochrome OLED (SSD1309 controller, I2C) as seven screens cycled by a push button, and integrates speed and energy into a trip log with the boat's efficiency in Wh per nautical mile (or km). Built for the DFRobot FireBeetle 2 ESP32-C6 (DFR1075) with an MCP2551-class CAN transceiver, any 2.42" or 1.54" SSD1309 module switched to I2C mode and any receiver that speaks UBX (u-blox 6/7, M8, M9, M10 or a compatible clone). On the CAN bus the firmware acknowledges frames and transmits exactly one thing: a 7-byte `COMM_GET_VALUES_SELECTIVE` request every `VESC_POLL_MS` (1 s). It never sends a command or a setting; `-DVESC_POLL_MS=0` makes it strictly passive (ACK only). Optionally the C6's own Bluetooth LE radio runs a dead-man's switch (`pio run -e deadman`, section 5): a BLE tag worn by the helmsman, and if it stops advertising the firmware pulls the VESC's own kill-switch input low, in parallel with the boat's mechanical lanyard switch; the default build contains no BLE code at all. That path uses a GPIO, not CAN - what the firmware transmits on the bus is unchanged. Without a CAN transceiver, `pio run -e deadman_uart` talks to the VESC over its COMM-header UART instead (section 3b): one `COMM_GET_VALUES` poll every 100 ms replaces both the broadcasts and the CAN poll, and everything except the ADC/PPM row stays available. The same UART build also runs on a Waveshare ESP32-S3-Zero (`pio run -e s3zero_deadman_uart`, section 2b).

## 1. What it does

Five FreeRTOS tasks (tick 1 ms; the dead-man builds add `deadmanTask`, the `ble` task and the two NimBLE tasks). Priorities guarantee that CAN reception is never starved by a frame push or by a stuck I2C bus (50 ms timeout per transaction). On the single-core C6 everything shares core 0. The dual-core S3-Zero runs the radio (BLE controller, NimBLE host, `ble` task) on core 0 and everything else, `app_main` included, on core 1 (`TASK_CORE_RADIO` / `TASK_CORE_APP`), so the firmware's own tasks keep the same priorities among themselves on both boards and only the radio moved away.

| Task | Priority | Stack | Job |
|---|---|---|---|
| `canTask` (`src/can_vesc.cpp`) | `TASK_PRIO_CAN` = 6 | 4096 | take frames from the TWAI ISR queue, keep VESC STATUS 1..6 frames, decode, smooth (EMA), write `g_state.vesc`; every `VESC_POLL_MS` send one `COMM_GET_VALUES_SELECTIVE` request, reassemble and CRC-check the reply into `g_state.vesc_ext`, latch the last fault; bus-off recovery and health counters |
| `gnssTask` (`src/gnss_ubx.cpp`) | `TASK_PRIO_GNSS` = 5 | 4096 | AUTOBAUD -> DETECT (MON-VER) -> BAUD (move the receiver's UART to `GNSS_TARGET_BAUD`) -> CONFIGURE (`GNSS_RATE_MS`, NAV-PVT on, NMEA off) -> RUN; parses NAV-PVT (position, speed, accuracies, heading, UTC date and time) into `g_state.gnss`; back to autobaud after `GNSS_REDETECT_MS` of silence or when the receiver does not answer at the new baud |
| `displayTask` (`src/display_oled.cpp`) | `TASK_PRIO_DISP` = 2 | 6144 (`OLED_TASK_STACK`) | loop every `OLED_BUTTON_POLL_MS` (20 ms): debounce the button, switch screens; every `OLED_PERIOD_MS` (or at once after a screen change) build the frame for the current screen from a snapshot and push it only when something changed; dim after `OLED_IDLE_DIM_MS` idle; re-probe the controller every 5 s and re-initialise a panel that stopped answering |
| `tripTask` (`src/trip.cpp`) | `TASK_PRIO_TRIP` = 3 | 4096 (`TRIP_TASK_STACK`) | every `TRIP_PERIOD_MS` (200 ms): snapshot, integrate GNSS ground speed into distance and the VESC watt-hour counters (or `v_in x current_in`) into energy, keep the `EFF_WINDOW_S` ring, publish `g_state.trip` (section "Efficiency and trip"); a pure consumer that owns no peripheral and is not watched by the watchdog |
| `deadmanTask` (`src/deadman.cpp`, `deadman` env only) | `TASK_PRIO_DEADMAN` = 7 | 2560 (`DEADMAN_TASK_STACK`) | every `DEADMAN_TICK_MS` (20 ms): run the pure state machine in `include/deadman_logic.h` against the beacon timestamp the NimBLE host task stamps (`src/ble_scan.cpp`), drive `PIN_DEADMAN_CUT` (open drain, in parallel with the mechanical kill switch on VESC ADC2), confirm the cut against the polled `VESC_STATUS_KILL_SW` bit and publish `g_state.deadman`. Runs ABOVE `canTask` on purpose and never logs - the supervisor does that for it |
| `ble` task (`src/ble_scan.cpp`, `deadman` env only) | `TASK_PRIO_BLE` = 4 | 4096 (`BLE_TASK_STACK`) | owns every NimBLE call: settles the own address after each host sync and, every 100 ms or at once when the host reports a change, makes sure the passive watchdog scan is running (`DEADMAN_SCAN_ITVL_MS` / `DEADMAN_SCAN_WINDOW_MS`, no duplicate filter). The host task's scan callback timestamps a matching tag straight into the dead-man and counts every other advert. The NimBLE host and controller tasks (priorities 21 / 23) run their own short slices |
| supervisor loop in `app_main()` (`src/main.cpp`) | 1 | 8192 (`CONFIG_ESP_MAIN_TASK_STACK_SIZE`) | sole Task-WDT subscriber (`WDT_TIMEOUT_MS`), fed only while the CAN, GNSS and display heartbeats (and in a dead-man build the dead-man's) are fresh; the trip and `ble` tasks do not gate it (the dead-man watches the `ble` heartbeat itself and reports `NO RADIO`); 1 Hz VESC and GNSS log lines, 5 s TRIP and DM lines (`LOG_TRIP_MS`, `LOG_DEADMAN_MS`), 10 s SYS line with the display, button and poll counters and the BLE scan state; LED |

Data flow:

```
 VESC ---CAN---> transceiver -> TWAI ISR -> RX queue (64 frames) -> canTask --+
      <--------- 1 request frame / s (single shot) <--------------- canTask   +--> g_state (one mutex) --> displayTask -> u8g2 (fonts, frame buffer) -> I2C master -> OLED
 GNSS ---UART--> UART1 ISR -> 2048 B ring -------------------------> gnssTask -+       |    ^             \-> app_main(): logs, watchdog, LED
      <--------- UBX configuration: 115200 baud, 5 Hz NAV-PVT <---- gnssTask   |       +----+-- tripTask (200 ms snapshots): speed x dt -> distance, Wh counters -> energy -> g_state.trip
 BLE tag ~adv~> NimBLE host (gap_cb): volatile timestamp ---------> deadmanTask -> PIN_DEADMAN_CUT (deadman envs only)
 button (GPIO7) ---------------------------------------------------> displayTask (20 ms poll)
```

Every hold of the mutex is a memcpy or a few field writes, never I/O; consumers work on snapshots. Every timestamp is a `state_now_ms()` value (milliseconds since boot, 32-bit) where 0 means "never received" and all age arithmetic is wrap-safe.

Files:

| Path | Purpose |
|---|---|
| `include/config.h` | every pin and tunable; the only file you normally edit |
| `include/vesc_status.h` | header-only VESC STATUS_1..6 decoder (framework-free, unit-tested) |
| `include/vesc_getvalues.h` | header-only codec for active polling: request builder, `FILL_RX_BUFFER` / `PROCESS_RX_BUFFER` reassembly, CRC-16/XMODEM, mask-ordered `COMM_GET_VALUES(_SELECTIVE)` decoder, the 34-entry fault-code table (framework-free, unit-tested) |
| `include/ubx_min.h` | header-only UBX frame parser, NAV-PVT / NAV-VELNED decoders and the CFG-PRT / CFG-RATE / VALSET frame builders (framework-free, unit-tested) |
| `include/shared_state.h`, `src/shared_state.cpp` | `g_state` (`vesc`, `vesc_ext`, `can`, `gnss`, `trip`, `disp`, plus `deadman` with `DEADMAN_UI_ENABLE` and `ota` with `OTA_ENABLE`), mutex, snapshots, heartbeats, `fresh()` |
| `include/can_vesc.h`, `src/can_vesc.cpp` | TWAI node setup (`esp_twai_onchip.h`), ISR callbacks and RX queue, canTask, poll state machine, bus-off recovery |
| `include/gnss_ubx.h`, `src/gnss_ubx.cpp` | UART1 setup, autobaud, MON-VER detection, baud switch, VALSET/legacy configuration |
| `include/trip.h`, `src/trip.cpp` | header-only trip integrator (distance, energy sources, `EFF_WINDOW_S` window, efficiency; framework-free, unit-tested) and the 200 ms task that feeds it snapshots and publishes `g_state.trip` |
| `include/display.h` | `display_start()`, the display task's one entry point (implemented by `src/display_oled.cpp`) |
| `include/display_strings_oled.h` | pure builders of all OLED screens (seven, plus the dead-man row and alarm of a `deadman` build and the FW UPDATE / UPDATE REFUSED overlays; character budgets, stale -> `--`, UTC -> local clock), unit-tested |
| `src/display_oled.cpp` | I2C bring-up and presence probe, u8g2 rendering, button debounce, screen switching, change-detected refresh, idle dimming |
| `include/u8g2_hal_idf.h`, `src/u8g2_hal_idf.cpp` | u8g2 HAL for ESP-IDF: I2C master byte callback, reset GPIO, delays, address probe |
| `components/u8g2/` | u8g2 library (git submodule, tag 2.37.1): fonts, frame buffer, SSD1309 init sequence; only the referenced fonts are linked |
| `include/ble_scan.h`, `src/ble_scan.cpp` | the dead-man's radio (the only file that includes NimBLE): NVS + NimBLE host init, the scan callback that timestamps the tags, the `ble` task that keeps the watchdog scan running; inline no-ops when `DEADMAN_ENABLE` is 0 |
| `include/ota.h`, `include/ota_logic.h`, `src/ota.cpp` | over-the-air update (section 7): the reboot request across RTC memory, the update-mode access point and upload server, the probation of a new image; the entry gate (motor turning, boat moving, dead-man alarm) is the framework-free, unit-tested `ota_logic.h`; inline no-ops when `OTA_ENABLE` is 0 |
| `include/nvs_util.h` | NVS bring-up shared by the BLE controller and update mode's Wi-Fi |
| `src/main.cpp` | `app_main()`: start-up and the supervisor loop (a smaller one in update mode) |
| `platformio.ini`, `boards/dfrobot_firebeetle2_esp32c6.json`, `boards/waveshare_esp32s3_zero.json`, `sdkconfig.defaults`, `sdkconfig.defaults.ble`, `sdkconfig.defaults.esp32s3`, `partitions.csv`, `CMakeLists.txt`, `src/CMakeLists.txt` | build: pinned platform and envs, board definitions, ESP-IDF configuration (the `.ble` fragment adds the NimBLE host, observer role only, for the dead-man envs; ESP-IDF applies `.esp32s3` by itself on the S3-Zero), partition table, IDF project files |
| `test/test_vesc/`, `test/test_vesc_getvalues/`, `test/test_vesc_uart/`, `test/test_ubx/`, `test/test_trip/`, `test/test_display_strings_oled/`, `test/test_deadman/`, `test/test_ble_addr/` | Unity tests (one `test_main.cpp` each, 208 test cases), run on the host with `pio test -e native` |

## 2. Hardware and wiring

### Wiring diagram

```
                                    USB-C: console + power (GPIO12/13)
                                                    |
 +------------------+             +-----------------+------------------+        +-----------------+
 | u-blox GNSS      |             |  FireBeetle 2 ESP32-C6 (DFR1075)   |        | SSD1309 OLED    |
 | UBX, 3.3 V UART  |             |   silkscreen label = GPIO number   |        | 128x64 I2C 0x3C |
 |              VCC |-------------| 3V3                            3V3 |--------| VCC             |
 |              GND |-------------| GND                            GND |--------| GND             |
 |               TX |------------>| 4  PIN_GNSS_RX    PIN_OLED_SCL  23 |------->| SCL             |
 |               RX |<------------| 5  PIN_GNSS_TX    PIN_OLED_SDA  22 |------->| SDA             |
 +------------------+             |                   PIN_OLED_RST  14 |- opt ->| RES             |
                                  |  on-board LED = GPIO15 (PIN_LED)   |        | DC -> GND: 0x3C |
 +------------------+             |                                    |        | CS -> GND       |
 | CAN transceiver  |             |                                    |        +-----------------+
 | MCP2551 (5 V IO) |             |                   PIN_BUTTON     7 |----o   push button
 | or 3.3 V-IO part |             |                                    |     \  to GND
 |              TXD |<---[1k]-----| 3  PIN_CAN_TX                  GND |----o   (internal pull-up)
 |              RXD |---[1k]--+-->| 2  PIN_CAN_RX                      |
 | RS  -> GND       |         |   |                                    |
 | VDD <- 5V (VESC) |      [2.2k] |                                    |
 |              GND |---------+---| GND                                |
 |  CANH      CANL  |             +------------------------------------+
 +---+--------+-----+
     |        |
  [120R]      |   <- 120 Ohm terminator at this end of the bus
     |        |
     +========+=== twisted pair ===> VESC CAN connector: CANH, CANL, GND (second 120 Ohm there)
```

- Resistors: 1 kOhm in series with TXD (TXD has a 25 kOhm pull-up to 5 V inside the MCP2551; the resistor limits the current into GPIO3 while the ESP32 is in reset). The 1 kOhm / 2.2 kOhm divider exists because the MCP2551 is a 5 V part and drives RXD to 5 V, while ESP32-C6 pads take VDD + 0.3 V at most; the divider gives ~3.4 V at a 5.0 V supply (check it, see below). A 3.3 V-IO transceiver (MCP2562 with VIO = 3V3, TJA1051T/3, SN65HVD230) connects RXD to GPIO2 directly. 120 Ohm across CANH/CANL at each end of the bus.
- 5 V: the transceiver's VDD comes from the VESC's 5 V pin (or another regulated 5 V on the boat), never from the FireBeetle 3V3; every GND is commoned. The FireBeetle itself runs from USB-C (also the console) or its battery connector; the OLED and the GNSS module take 3V3 from the board.
- Free header pins: GPIO1, 8 and 18 (CS, DC, SD_CS on the SPI header) are not used by anything. Alternatives: the FireBeetle's default I2C pins 19 (SDA) / 20 (SCL) with `-DPIN_OLED_SDA=19 -DPIN_OLED_SCL=20`; the on-board BOOT button with `-DPIN_BUTTON=9`; `-DPIN_OLED_RST=-1` when RES is tied high on the module.

Silkscreen labels on the FireBeetle 2 ESP32-C6 equal GPIO numbers.

| Signal | GPIO (`config.h`) | FireBeetle silkscreen | Module pin |
|---|---|---|---|
| CAN TX | 3 (`PIN_CAN_TX`) | 3 / A2 | MCP2551 TXD (pin 1), through 1 kOhm |
| CAN RX | 2 (`PIN_CAN_RX`) | 2 / A1 | MCP2551 RXD (pin 4), through the level-shift divider |
| OLED SCL | 23 (`PIN_OLED_SCL`) | 23 / SCK | OLED SCL (CLK / D0) |
| OLED SDA | 22 (`PIN_OLED_SDA`) | 22 / MOSI | OLED SDA (DIN / D1) |
| OLED RST | 14 (`PIN_OLED_RST`, optional: -1 = not wired) | 14 / RES | OLED RES |
| Button | 7 (`PIN_BUTTON`, -1 = none, 9 = on-board BOOT) | 7 / INT | momentary switch to GND, no resistor |
| GNSS RX | 4 (`PIN_GNSS_RX`) | 4 / LP_RX | GNSS module TX |
| GNSS TX | 5 (`PIN_GNSS_TX`) | 5 / LP_TX | GNSS module RX |
| LED | 15 (`PIN_LED`) | 15 | on-board LED, nothing to wire |

The OLED module and the GNSS module go to the board's 3V3 and GND. The CAN transceiver needs more care.

### Button

A plain momentary switch between pin 7 and GND; the firmware enables the C6's internal pull-up (~45 kOhm) with `pinMode(INPUT_PULLUP)`, so no external resistor is needed (a 100 nF capacitor across the switch does no harm on a noisy boat; the software debounce is `BUTTON_DEBOUNCE_MS` = 30 ms anyway). GPIO7 is a plain GPIO on this board: silkscreen "7"/"INT", routed to the unused GDI display connector, not a strapping or USB pin. `BUTTON_ACTIVE_LOW 1` (default) = pressed reads LOW; set it to 0 for a switch to 3V3 (the firmware then uses `INPUT_PULLDOWN`). `-DPIN_BUTTON=9` uses the on-board BOOT button instead (10 kOhm pull-up already on the board): fine at runtime, but holding it while the board resets enters the ROM download mode. `-DPIN_BUTTON=-1` compiles the button code out; the main screen is then the only one.

### CAN transceiver: the 5 V problem

The MCP2551 is a 5 V part. ESP32-C6 pads are not 5 V tolerant (absolute maximum VDD + 0.3 V), so its RXD output must never reach GPIO2 directly.

- Preferred: a transceiver with 3.3 V I/O and no divider at all: MCP2562 (VDD 5 V, VIO pin to 3V3), TJA1051T/3 (VIO to 3V3) or SN65HVD230 (3.3 V supply, needs no 5 V).
- If you already have an MCP2551: RXD (pin 4) -> 1 kOhm -> GPIO2, plus 2.2 kOhm from GPIO2 to GND. That gives 5.0 V x 2.2/3.2 = 3.44 V at a nominal supply but 3.61 V at 5.25 V, which is over the absolute maximum. Measure your 5 V rail, then measure the idle level at the divider output before plugging it into the board: 3.2..3.5 V, never above 3.6 V.
- TXD (pin 1) accepts 3.3 V directly (VIH 2.0 V). Put 1 kOhm in series anyway: TXD has an internal 25 kOhm pull-up to 5 V and the resistor limits the current into GPIO3 while the ESP32 is in reset. TXD must be wired: it carries the ACK bit for every received frame and the one poll request per second.
- RS (pin 8) to GND (high-speed slope mode).
- 120 Ohm termination at both ends of the bus. Most VESC boards have no on-board terminator; with everything powered off you should measure about 60 Ohm between CANH and CANL once both terminators are in place (120 Ohm means only one).
- Power the transceiver from the VESC's 5 V pin (or another regulated 5 V) and common the grounds. The FireBeetle VIN pin is dead when the board runs from its battery connector, and USB 5 V is only there while you debug.

### OLED: 128x64 SSD1309 over I2C

Any 128x64 SSD1309 module works (Waveshare, DIYables, LCDWIKI, Diymore / ACEIRMC 2.42" or 1.54", Adafruit 2719); the firmware drives it with u8g2 (SSD1309 `noname0` driver over the ESP-IDF I2C master, see `src/u8g2_hal_idf.cpp`). Most 7-pin modules ship in 4-wire SPI mode and must be switched to I2C first.

| Module pin | Role in I2C mode | Connect to | Notes |
|---|---|---|---|
| GND | ground | GND | commoned with the CAN transceiver and the GNSS |
| VCC | 3.3 V into the module's boost converter (the SSD1309 has no charge pump; its 7..16 V panel voltage is generated on the module) | 3V3 | modules are marked 3.3 V / 5 V; use 3.3 V so the on-board pull-ups sit at 3.3 V (C6 pads: VDD + 0.3 V max). Budget ~200 mA at all-pixels-on, 15..50 mA for this mostly dark layout |
| SCL (CLK, D0) | I2C clock | 23 / SCK -> GPIO23 | `PIN_OLED_SCL` |
| SDA (DIN, D1 + D2) | I2C data | 22 / MOSI -> GPIO22 | `PIN_OLED_SDA` |
| RES (RST) | reset, active low | 14 / RES -> GPIO14 | `PIN_OLED_RST`; the library pulses it at init. Never leave it floating: the panel then starts blank or goes dark at random. If you do not wire it, tie RES to 3V3 through 4.7..10 kOhm and set `-DPIN_OLED_RST=-1` (4-pin "IIC" modules with an on-board RC reset need neither) |
| DC (D/C#) | address select SA0 | GND -> 0x3C, 3V3 -> 0x3D | not driven by the MCU in I2C mode; must match `OLED_I2C_ADDR`. 4-pin modules fix it with a solder resistor ("0x78" / "0x7A" side = 0x3C / 0x3D) |
| CS (CS#) | chip select | GND | tie low in I2C mode; some boards ground it with a solder resistor (LCDWIKI R8) so the pin can stay open |
| back-side solder resistors | interface select | - | Waveshare 2.42": both 0R resistors to the "I2C" position. LCDWIKI MSP154X: 4.7k on R4 + R9 only (remove R5). Diymore / ACEIRMC 2.42": move the 4.7k from R4 to R3, bridge R5. Adafruit 2719: BS1/BS2 jumpers. 4-pin "IIC" modules come pre-configured |

- Pull-ups: the SSD1309 datasheet requires external pull-ups on SDA and SCL; the C6's internal ones (~45 kOhm) are far too weak for 400 kHz. Most I2C-configured modules carry 4.7 kOhm (on LCDWIKI boards the R4/R9 parts are the pull-ups); measure SDA and SCL to VCC and add 2.2..4.7 kOhm to 3V3 if there are none. If the display is flaky, build with `-DOLED_I2C_HZ=100000` (a frame then takes ~105 ms instead of ~26 ms, still fine at 4 Hz).
- Why the SPI header pins: the OLED sits on the FireBeetle's SPI header pins (23 / SCK, 22 / MOSI, 14 / RES), which the C6 routes to I2C through its GPIO matrix (22 and 23 are unrestricted "P2" pins); nothing else uses SPI. The neighbouring CS / DC / SD_CS pins (GPIO1, 8, 18) are free. If you prefer the FireBeetle's default I2C pins (19 = SDA, 20 = SCL, also on the GDI connector), build with `-DPIN_OLED_SDA=19 -DPIN_OLED_SCL=20`.
- Burn-in: white SSD1309 glass is rated ~20,000 h to half luminance (yellow ~50,000 h) and static images leave residual ghosts. The layout is black with sparse text (the only filled area is the 10-px title bar of the detail screens), and the contrast drops to `OLED_IDLE_CONTRAST` after `OLED_IDLE_DIM_MS` without a change (section 6). If the panel is still to be bought, a yellow one lasts longer.

### GNSS receiver

Any u-blox receiver or clone with a 3.3 V UART: NEO-6M/7M, NEO-M8N, SAM-M8Q, NEO-M9N, MAX-M10S and the like. Module TX -> GPIO4, module RX <- GPIO5, VCC 3V3, GND. Some breakout boards take 5 V in and regulate it down but keep 3.3 V logic; a few have 5 V logic. Check the datasheet: the C6 pins are not 5 V tolerant. The board has a 499 Ohm series resistor on GPIO5; that is fine for UART.

### Pins to leave alone

| GPIO | Why |
|---|---|
| 12, 13 | USB D-/D+: the USB-Serial/JTAG console (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`). `config.h` refuses to build if a peripheral uses them |
| 16, 17 | UART0, the ROM/IDF boot console. GPIO16 is driven by UART0 TX for the whole run |
| 9 | BOOT button, boot-mode strapping. Usable as the screen button (`-DPIN_BUTTON=9`) because the level is only latched at reset |
| 0 | battery ADC, not on the header |
| 8, 15 | strapping pins (boot mode, JTAG select). MCU-driven outputs are fine (the on-board LED is on 15); never a peripheral-driven input |
| 4, 5 | strapping only for the SDIO edge, harmless; used for GNSS |

## 2b. Waveshare ESP32-S3-Zero

`pio run -e s3zero_deadman_uart` is the `deadman_uart` build (VESC over UART, dead-man's switch) for a Waveshare ESP32-S3-Zero instead of the FireBeetle (`BOARD_S3_ZERO=1`, board JSON `boards/waveshare_esp32s3_zero.json`). The module is an ESP32-S3FH4R2: 4 MB flash, and 2 MB PSRAM that the firmware leaves unused. It has native USB only (GPIO19/20), a WS2812 RGB LED on GPIO21 and a BOOT button on GPIO0. It is powered through its 5V pin (3.7..6 V in) and has 3V3 out from an ME6217 regulator (800 mA).

Everything uses the FireBeetle's GPIO numbers wherever the module has them. Only the OLED moves (the S3-Zero has no GPIO22/23), and the LED becomes the on-board WS2812:

| S3-Zero pin | Function | FireBeetle GPIO |
|---|---|---|
| GP1 | dead-man cut, open drain (section 5) | 1 |
| GP2 | VESC UART RX <- 1 kOhm <- VESC TX (section 3b) | 2 |
| GP3 | VESC UART TX -> 1 kOhm -> VESC RX | 3 |
| GP4 / GP5 | GNSS RX / TX | 4 / 5 |
| GP7 | button to GND | 7 |
| GP8 / GP9 / GP10 | OLED SDA / SCL / RES | 22 / 23 / 14 |
| GP21 (on board) | WS2812 status LED | 15 (plain LED) |

All of these are front-edge pins. Notes:

- **Status LED:** it blinks green at 1 Hz while VESC data is fresh and red at 0.2 Hz while it is not, at `LED_WS2812_LEVEL` (16 of 255). It is driven by the RMT peripheral (`src/status_led.cpp`).
- **VESC UART:** the link runs on UART2 (`VESC_UART_PORT`). UART1 is the GNSS, and UART0 is left alone, together with its TX/RX pads (GPIO43/44), where the ROM prints its boot log.
- **Both cores:** `sdkconfig.defaults.esp32s3` (applied automatically for this target) splits the S3 by job, at 160 MHz. Core 0 (`TASK_CORE_RADIO`) runs the radio: the BLE controller, the NimBLE host and the `ble` task. Core 1 (`TASK_CORE_APP`) runs `app_main` and every other task of the firmware, and with them the UART, I2C and RMT interrupts they install. The firmware's own tasks therefore keep exactly the priorities they have on the single-core C6 (the dead-man task above everything), and the radio's tasks at priorities 21/23 no longer preempt them. The only data crossing the cores is the dead-man's beacon timestamps and the scan flag: single-writer, 32-bit, `volatile`. `config.h` refuses a configuration that puts anything on the wrong core, and the boot banner prints `cores=2 app=1 radio=0`.
- **Pins to leave alone:**
  - GPIO0, 3, 45 and 46 are the S3's strapping pins. `config.h` refuses them for the dead-man cut. GPIO3 only selects the JTAG source when the `STRAP_JTAG_SEL` eFuse is burned (it is not from the factory), so the VESC TX output there is fine.
  - GPIO19/20 are USB, and GPIO26..37 are the in-package flash / PSRAM. `config.h` refuses both, as it refuses 12/13 on the C6.
- **Flashing:** over the USB-C port (USB-Serial/JTAG, which is also the console). If the port does not enumerate, hold BOOT while plugging in.
- **Overrides:** every pin is still an `#ifndef` default, so `-D` overrides work as on the FireBeetle.

## 3. VESC configuration (VESC Tool)

A VESC broadcasts nothing on CAN until you enable status messages. Connect VESC Tool to the VESC, then App Settings > General:

| Setting | Value |
|---|---|
| CAN Baud Rate | 500K (must match `CAN_BITRATE_KBPS`) |
| CAN Mode | VESC (the poll replies need it too: a VESC in UAVCAN or Comm Bridge mode ignores requests) |
| Can Messages Rate 1 (FW 6.x) | Status 1, Status 4, Status 5 at 10..50 Hz |
| Can Messages Rate 2 (FW 6.x) | Status 2, Status 3 (and Status 6 if you want the ADC/PPM values) at 1..5 Hz |
| Can Status Message Mode (FW <= 5.03) | CAN_STATUS_1_2_3_4_5, status rate 10..50 Hz |
| VESC ID | note it; leave `VESC_CAN_ID -1` to lock onto whatever appears first, or pin that id. It must differ from `CAN_OWN_ID` (120) |

Then "Write App Configuration". Also note Motor Settings > Additional Info > Motor Poles and put it in `VESC_MOTOR_POLES`: the `RPM` cell and the log show mechanical rpm = erpm / (`VESC_MOTOR_POLES` / 2).

### What each status frame carries

| Frame | packet id | Fields (resolution per LSB) | Rate group | Shown as |
|---|---|---|---|---|
| STATUS_1 | 9 | erpm, motor current (0.1 A, signed), duty (0.001) | Rate 1 | `MA`, `RPM` cells; `Imot`, `Duty`, `RPM` on VESC 1/3; CAN line freshness |
| STATUS_2 | 14 | Ah drawn, Ah charged (0.1 mAh) | Rate 2 | `Ah` / `AhC` on VESC 1/3 and 2/3 |
| STATUS_3 | 15 | Wh drawn, Wh charged (0.1 mWh) | Rate 2 | `WH` cell; `Wh` / `WhC`; trip energy (section "Efficiency and trip") |
| STATUS_4 | 16 | FET temp, motor temp (0.1 C), battery current (0.1 A, signed), PID position | Rate 1 | `BA`, `PW`, `TF`, `TM` cells; `Ibat`, `Tfet`, `Tmot`, `PID`; trip energy fallback (with STATUS_5) |
| STATUS_5 | 27 | tachometer, input voltage (0.1 V) | Rate 1 | `BV`, `PW` cells; `Vin`, `Tach`; CAN line freshness |
| STATUS_6 | 58 | ADC1..3 (1 mV), PPM (0.001) | Rate 2 | `ADC`, `PPM` on VESC 3/3 |

STATUS 1/4/5 older than `VESC_STALE_R1_MS` (2 s) and STATUS 2/3/6 older than `VESC_STALE_R2_MS` (5 s) are shown as `--`. Currents and voltage are EMA-smoothed for the display (`CAN_EMA_ALPHA`, 0.15; 1.0 = off).

### Active polling: `COMM_GET_VALUES_SELECTIVE`

Some values exist only in the reply to a request. Every `VESC_POLL_MS` (1000 ms) canTask sends one extended, single-shot frame `CAN_PACKET_PROCESS_SHORT_BUFFER` to the VESC id (EID `(8 << 8) | id`, payload `[CAN_OWN_ID, 0, 50, mask]`, 7 bytes) with mask `VESC_GETVALUES_MASK` = `0x003EC03C`: average motor and input current, id/iq, vd/vq, absolute tachometer, fault code, controller id, MOSFET 1/2/3 temperatures and the status byte (timeout / kill switch). The VESC answers with six `FILL_RX_BUFFER` frames and one `PROCESS_RX_BUFFER` (42 bytes, CRC-16/XMODEM), addressed to `CAN_OWN_ID`; the reply is checked, decoded into `g_state.vesc_ext` and shown on VESC 1/3 (fault) and VESC 2/3 (everything else), stale after `VESC_EXT_STALE_MS` (3 s). Eight frames per second is about 0.2 % of a 500 kbit/s bus.

- `CAN_OWN_ID` (120) is this board's controller id on the VESC bus. It must be 1..254 and differ from every VESC id (255 is the broadcast address; both rules are `#error`s in `config.h`). A VESC that was never given an id derives one from its chip UUID, so check it in VESC Tool. If a STATUS frame ever arrives under our own id the firmware logs `VESC: controller id 120 equals CAN_OWN_ID, polling suspended (change CAN_OWN_ID or the VESC id)` once and stays passive for the rest of the run.
- Only requests are ever transmitted, and a request cannot change anything: `COMM_GET_VALUES_SELECTIVE` is the same read-only query VESC Tool's realtime page uses. Polling starts once a target id is known (`VESC_CAN_ID`, or the id locked from the first STATUS frame) and never broadcasts to 255. A VESC with status messages disabled therefore needs `VESC_CAN_ID` pinned.
- Strictly passive node: build with `-DVESC_POLL_MS=0`. The compiled object then contains no reference to `twai_node_transmit`, and every polled value shows `--` (`FAULT --`, `poll off`). `CAN_LISTEN_ONLY 1` requires this (a listen-only controller cannot transmit; `#error` otherwise).
- Fault latch: the VESC clears its live fault byte about 500 ms after a fault (its "fault stop time"), so a 1 Hz poll usually sees a `0`. canTask therefore latches the last non-zero code and its time in `VescExt::last_fault` / `last_fault_ms`; VESC 1/3 shows `FAULT <name>` while the live byte is set and `LAST <name> <age>` afterwards (section 6). Names (34 codes, `vesc_getvalues.h`): `NONE OVER_V UNDER_V DRV ABS_OC OT_FET OT_MOT GATE_OV GATE_UV MCU_UV WDT_RST ENC_SPI ENC_LOW ENC_HIGH FLASH I_OFFS1 I_OFFS2 I_OFFS3 I_UNBAL BRK RSLV_LOT RSLV_DOS RSLV_LOS FLASH_AP FLASH_MC ENC_NMAG ENC_SMAG PH_FILT ENC_FLT LV_OUT ENC_SLIP OVERSPD UNDERSPD ABS_OSPD`; a code beyond the table prints as `F<code>`. The log names both forms: `VESC: fault OT_FET (OVER_TEMP_FET, code 5) reported by id 74` and later `VESC: fault cleared (was OT_FET)`.
- Timing: a request without a complete reply within `VESC_POLL_TIMEOUT_MS` (500 ms) counts as a timeout; after `VESC_POLL_BACKOFF_AFTER` (3) consecutive failures the period stretches to `VESC_POLL_BACKOFF_MS` (5 s) until a reply arrives or a STATUS frame shows the VESC is back. Warnings are rate-limited to one per `VESC_POLL_LOG_MIN_MS` (10 s). The averages (id/iq, vd/vq, Iavg) are read-and-reset on the VESC, i.e. averages since the previous read by any client, so with VESC Tool polling at the same time their windows shrink.
- Firmware: `COMM_GET_VALUES_SELECTIVE` exists from FW 3.42; the status byte (mask bit 21) from FW 5.03. Older firmware answers 41 bytes, which is accepted (the status keeps 0) with one `VESC poll: firmware answers 10 of 11 requested fields ...` warning. For FW < 3.42 build with `-DVESC_GETVALUES_MASK=0` (plain `COMM_GET_VALUES`, 74-byte reply, 12 frames).

### Why the board acknowledges

`CAN_LISTEN_ONLY 0` (default) runs the TWAI node in normal mode, so it generates the ACK bit for every frame it receives. That ACK is required when the VESC and this board are the only two nodes: the VESC's STM32 bxCAN has no retransmit limit, so with a non-acknowledging listener it goes error-passive and repeats the same stale STATUS frame forever ("lonely VESC"). The node is configured single-shot (`fail_retry_cnt 0`, no automatic retransmission) for the mirror-image reason: with the VESC powered off a request is simply lost (`VESC poll: N request(s) not acknowledged on the bus (VESC off? ack errors=N TEC=N)`, error-passive after ~16 polls, never bus-off) instead of being retransmitted for ever. Set `-DCAN_LISTEN_ONLY=1 -DVESC_POLL_MS=0` (no ACK, TX electrically silent) only when another node acknowledges anyway: a second VESC, a BMS, a permanently attached VESC Tool CAN adapter.

## 3b. VESC over UART (no CAN)

`pio run -e deadman_uart` is the `deadman` build with the VESC link moved from CAN to a UART (`VESC_LINK_UART=1`; task in `src/uart_vesc.cpp`, codec in `include/vesc_uart.h`). There is no transceiver, and the TWAI driver is not even linked. Every `VESC_POLL_MS` (100 ms) the board sends the 6-byte request `02 01 04 40 84 03` (`COMM_GET_VALUES`). The VESC answers with one 79-byte frame (74-byte payload, ~7 ms at 115200). That frame holds every field of STATUS_1..5 and every value the CAN build polls: fault, MOSFET temperatures, id/iq, vd/vq, absolute tachometer, controller id, and the timeout / kill-switch status byte. The main screen, VESC 1/3 and 2/3, the trip log and the dead-man's switch, including its `KILLSW` closed loop, therefore work unchanged.

### Wiring

| FireBeetle | series | VESC COMM header (75_300: USART3) |
|---|---|---|
| GPIO3 (`PIN_VESC_UART_TX`) | 1 kOhm | RX (PB11) |
| GPIO2 (`PIN_VESC_UART_RX`) | 1 kOhm | TX (PB10) |
| GND | | GND |

- Both sides are 3.3 V logic: wire them directly, and never to the header's 5 V pin.
- The series resistors limit the current through the input protection diodes when only one side is powered. A UART line idles high, so without them an unpowered VESC would be fed from the dashboard's TX, and vice versa.
- GPIO2/3 are the CAN transceiver's pins. This build does not use CAN, so remove the transceiver or leave it disconnected.
- RX has the internal pull-up enabled, so an unplugged VESC reads as an idle line.
- The link runs on the C6's UART0 (UART2 on the S3-Zero, section 2b): UART1 is the GNSS, and the console stays on USB-Serial/JTAG (`config.h` refuses a UART console in this build).

### On the VESC (VESC Tool)

| Setting | Value |
|---|---|
| App Settings > General > App to Use | a mode that includes UART: `ADC and UART` with an ADC throttle, `PPM and UART` with PPM. The throttle keeps working; the UART is added |
| App Settings > UART > Baudrate | 115200 (= `VESC_UART_BAUD`) |
| CAN status messages | not needed (they may stay on; nothing listens) |

Then "Write App Configuration".

- The COMM-header UART only runs its packet interface in those app modes (`app.c`: `APP_UART` / `APP_PPM_UART` / `APP_ADC_UART`).
- Its pads double as I2C2, which the VESC switches off when the UART starts.
- Some hardware has a second, "permanent" UART (`HW_UART_P_DEV`). On the 75_300 R2/R3 that is UART4 on PC10/PC11, at a fixed 115200, labelled "for NRF51" in the firmware. It speaks the protocol whatever the app, but only if nothing else is on it, typically a Bluetooth module: two transmitters on one RX line corrupt each other.
- `COMM_GET_VALUES` is read-only and does not call `timeout_reset()` on the VESC, so polling can never keep the motor's control timeout alive.

### What differs from the CAN build

| | CAN build | UART build |
|---|---|---|
| PPM / ADC1..3 (VESC 3/3) | STATUS_6 | always `--`. GET_VALUES does not carry the raw pin voltages, and `COMM_GET_DECODED_ADC/PPM` only report the apps' processed values (ADC only in ADC modes, no ADC3) |
| Motor / battery current (`MA`, `BA`, `PW`) | STATUS_1/4 filtered currents, EMA `CAN_EMA_ALPHA` 0.15 at 50 Hz | the VESC's averages over each 100 ms poll, EMA `VESC_UART_EMA_ALPHA` 0.5 |
| Update rate | Rate 1/2 broadcasts + 1 Hz poll | everything at 10 Hz (`VESC_POLL_MS` 100, `VESC_POLL_TIMEOUT_MS` 50) |
| Main screen link text | `VESC 74` / `CAN idle` / `CAN OFF` | `VESC 74` / `NO VESC` / `UART OFF` |
| SYS row 3 | `CAN RUN T0 R0` (TEC / REC) | `UART RUN 115200` |
| VESC 3/3 counters | `frm` status frames, `misc` odd frames, `busoff` | `frm` replies, `misc` bytes outside any frame, `busoff` always 0; `bad` = frames with a CRC or stop-byte mismatch |
| VESC id | locked from the first STATUS frame (`VESC_CAN_ID`) | the controller id field of each reply |
| Silent VESC | back-off to 5 s; a STATUS frame resumes polling at once | `tmo` counts up, back-off to `VESC_POLL_BACKOFF_MS` (1 s) |
| Log | tag `can`, `VESC id=...` line ending in `can=RUN tec=...` | tag `vesc`, the same line ending in `uart=RUN baud=115200 junk=N` |

The averages are read-and-reset on the VESC: VESC Tool's realtime page running at the same time (over USB or Bluetooth) shortens their window, as on CAN.

A healthy start:

```
I (1513) main: cfg VESC: uart0 tx=3 rx=2 115200 baud poll=100 ms timeout=50 ms poles=10 (no CAN)
I (1530) vesc: VESC: UART0 tx=GPIO3 rx=GPIO2 115200 baud ring=1024 B, COMM_GET_VALUES every 100 ms (timeout 50 ms)
I (1640) vesc: VESC: replying over UART, controller id 74 (74 B, 22 fields)
```

No answer at all shows up as `VESC poll: N request(s) without a complete reply within 50 ms (VESC off, TX/RX swapped, App to Use without UART, or VESC baud != 115200)`, repeated every 10 s.

## 4. GNSS

The receiver is driven with the binary UBX protocol only; NMEA output is switched off. gnssTask runs AUTOBAUD -> DETECT -> BAUD -> CONFIGURE -> RUN on its own, `setup()` never waits for it, and the receiver's flash is never written.

- Generations: u-blox 7, M8, M9, M10 and clones via NAV-PVT; u-blox 6 (no NAV-PVT) via NAV-VELNED. The generation is read from MON-VER's PROTVER: >= 27 is configured with VALSET (CFG-UART1-BAUDRATE, CFG-UART1OUTPROT, CFG-MSGOUT, CFG-RATE), 14..26 with the legacy CFG-PRT / CFG-MSG / CFG-RATE messages, below 14 or on a NAK for NAV-PVT the task enables NAV-VELNED instead. No MON-VER reply, or every VALSET refused, also falls back to the legacy messages.
- Autobaud: `GNSS_BAUDS` = 115200, 38400, 9600, 57600, 230400, tried in that order: the target baud first (a receiver this firmware configured earlier is still there while it stays powered or battery-backed), then the factory defaults 38400 (M9/M10 firmware) and 9600 (M8, u-blox 6/7, MAX-M10S modules). A rate is accepted when a checksum-valid UBX frame arrives within `GNSS_AUTOBAUD_LISTEN_MS` (1.2 s) or the receiver answers a MON-VER poll within `GNSS_MONVER_TIMEOUT_MS` (1.5 s). Factory modules are NMEA-only, so the MON-VER path is the one that normally succeeds, about 7 s after boot for a 9600-baud module. If no rate works the task pauses `GNSS_AUTOBAUD_RETRY_MS` (2 s) and starts over, forever, without blocking anything else.
- Baud switch: after DETECT the receiver's UART1 is moved to `GNSS_TARGET_BAUD` (115200, the fastest rate every generation from u-blox 6 up supports; 0 = keep the detected baud; nothing is sent when the receiver already runs at the target). PROTVER >= 27 gets a VALSET of CFG-UART1-BAUDRATE into RAM and BBR, older receivers a CFG-PRT for UART1 (8N1, input UBX+NMEA+RTCM, output UBX only, which also mutes NMEA on the spot). The ACK of a baud change is unreliable by design: the receiver switches as soon as it has processed the frame and the acknowledge leaves at the new rate, so it is not awaited. The task flushes the frame onto the wire, pauses `GNSS_BAUD_SWITCH_SETTLE_MS` (100 ms, u-blox's recommendation), reprograms UART1, discards the RX ring and proves the switch with up to two MON-VER polls at the new rate: `GNSS: switching UART1 9600 -> 115200 baud (CFG-PRT)` (or `(VALSET)`) followed by `GNSS: UART1 switched to 115200 baud (MON-VER reply)`, about 0.2 s. No reply logs `GNSS: baud switch to 115200 failed (no reply to 2 MON-VER polls at the new rate, attempt 1/2); redetecting from 9600, redetect #1` and restarts autobaud, which finds the receiver wherever it ended up since both bauds are in `GNSS_BAUDS`; a switch that worked but lost its proof therefore costs one autobaud pass and then logs `GNSS: UART1 switched to 115200 baud (found there by autobaud after an unconfirmed switch)`. After `GNSS_BAUD_SWITCH_ATTEMPTS` (2) unconfirmed switches in a row the detected baud is kept (`GNSS: baud switch to 115200 failed (2 attempts without a reply at the new rate), staying at 9600`) and configuration continues there; a module reset or swap (RUN-phase redetect) earns fresh attempts. Why bother: at 9600 the factory NMEA burst fills the line and every ACK queues behind it, and NAV-PVT (100 bytes per epoch) at 10 Hz alone is 104 % of a 9600 line; CONFIGURE warns when the configured rate would take more than half the line in use (`GNSS: NAV-PVT every 100 ms needs 10000 bit/s = 104% of 9600 baud; expect dropped epochs`).
- Navigation rate: `GNSS_RATE_MS` 200 = 5 Hz (measRate 200 ms, navRate 1, GPS time reference; `config.h` accepts 50..10000). 5 Hz is the data-sheet maximum of a stock NEO-M8N with its default GPS+GLONASS set and inside the limits of NEO-6 (5 Hz), NEO-7 (10 Hz) and NEO-M9N (25 Hz); a MAX-M10S is rated 3 Hz on its default GPS+Galileo+BeiDou set (10 Hz on GPS+Galileo), so at 5 Hz it accepts the rate but may skip an epoch now and then. The trip integrator ticks at the same period. `-DGNSS_RATE_MS=100` (10 Hz) is for a NEO-M9N, an M8Q/M8M, or an M8N/M10 reduced to a single constellation, and only makes sense when the baud switch succeeded (10 Hz NAV-PVT is 10 kbit/s: impossible at 9600, a quarter of a 38400 line); `gnss_start()` then warns once (`GNSS: GNSS_RATE_MS 100 (> 5 Hz) is not guaranteed on a stock NEO-M8N / NEO-6 / default MAX-M10S`). A rate the module cannot sustain is ACKed anyway and shows up as skipped epochs, never as an error. `GNSS_DYNMODEL_SEA 1` sets the dynamic model to SEA (5), which suits a boat.
- Persistence and power cycles: everything is written to RAM (VALSET also to BBR, the battery-backed RAM), never to the receiver's flash, and the receiver is reconfigured on every boot and after every redetect. An M9/M10 with a backup battery comes back from a power cycle at 115200 with NAV-PVT on and NMEA off and is found by the first autobaud attempt within a second (`GNSS: UBX detected at 115200 baud (unsolicited frame)`). Any receiver without V_BCKP, and every u-blox 6/7/M8 even with one (the legacy CFG-PRT / CFG-MSG changes are RAM-only), boots at its factory baud talking NMEA: the running task sees no valid UBX frame for `GNSS_REDETECT_MS` (20 s), logs `GNSS: no valid UBX frame for 20010 ms (module reset or baud lost); redetect #1` and goes through autobaud, switch and configuration again without an ESP32 reboot, so the speed cell shows `--` for roughly half a minute. A configuration saved to the receiver's flash with u-center is left alone; note that a battery-backed M9/M10 keeps the 115200 / UBX-only port settings for u-center as well until they are changed or the battery drains.
- `SPEED_UNIT_KNOTS 1` = knots, 0 = km/h (the examples in this README use knots). Speed shows `--` without a 2D/3D fix, when the receiver's own speed-accuracy estimate is worse than `GNSS_MAX_SACC_MM_S`, or when no NAV-PVT arrived for `GNSS_STALE_MS`. Speeds below `SPEED_MIN_SHOW` are shown as 0.0 to hide drift at the mooring; the heading on the GNSS screen is hidden (`--`) in the same case.
- Clock: NAV-PVT carries UTC date and time with validity flags. The main screen shows `HH:MM` = UTC + `TIME_UTC_OFFSET_MIN` (e.g. 120 for UTC+2, -300 for UTC-5; negative and multi-day offsets wrap correctly, no DST logic) whenever the receiver flags both date and time valid and the epoch is fresh, even without a position fix (a time-only fix counts); otherwise `--:--`. The GNSS screen shows the raw UTC date-time with a `Z` suffix.
- The receiver is on HP UART1 (`driver/uart.h`, `UART_NUM_1`) with a `GNSS_RX_BUFFER` (2048 byte) driver receive ring filled from the UART interrupt: 4 s of 5 Hz NAV-PVT (100 bytes per epoch), so no epoch is lost while the display task holds the CPU (an OLED frame is ~26 ms of I2C). At 10 Hz it still holds 2 s. The LP UART of the C6 is not used.

### Efficiency and trip

tripTask (`src/trip.cpp`; the arithmetic is the header-only, unit-tested `include/trip.h`) wakes every `TRIP_PERIOD_MS` (200 ms), takes a state snapshot, advances the integrator and publishes `g_state.trip` for the EFFICIENCY screen and the `TRIP` log line. Everything starts from zero at boot: nothing is stored, so a reboot (or a watchdog reset) starts a new trip.

- Distance: the latest NAV-PVT ground speed times the tick length (rectangle rule), integrated only while the GNSS is usable (NAV-PVT within `GNSS_STALE_MS`, fix OK, sAcc within `GNSS_MAX_SACC_MM_S`: the same gate as the speed cell) and the speed is at least `EFF_MIN_SPEED_MM_S` (500 mm/s, about 1 kn); the same ticks count as moving time, so drift at the mooring adds nothing. Caveats: GNSS Doppler speed is good to a few cm/s under way but noisy at walking pace, where the threshold also cuts off real movement, so slow legs are underestimated; the 5 Hz epochs and the 200 ms ticks are not synchronised, so a sample is occasionally used twice or skipped, which averages out over a trip.
- Energy, checked in this order every tick: (A) STATUS_3 fresh (`VESC_STALE_R2_MS`): the VESC's own watt-hour counters (drawn and charged, 0.1 mWh per LSB, monotonic since the VESC booted). The trip totals are anchored to them: in steady state every tick adds the counter delta, and a counter that jumps back by more than `TRIP_COUNTER_RESET_WH` (0.5 Wh) means the VESC rebooted or its counters were cleared in VESC Tool: `counter_resets` (`resets=` in the log) increments, that step's energy is lost and the integrator re-baselines. (B) STATUS_3 stale but STATUS_4 and STATUS_5 fresh (`VESC_STALE_R1_MS`): `v_in x current_in x dt` from the raw frame values (not the display EMA), positive to drawn, negative to charged; the screen shows `src PxI`, the log `src=integ`. When STATUS_3 returns, the counters overwrite this estimate for the whole time they were away, so nothing is counted twice and the totals may step once by the estimate's error. (C) neither fresh: nothing is added; when the counters come back the outage is caught up from them into the trip totals (not into the window). Net energy = drawn - charged, so a net-regenerating window or trip shows a negative efficiency.
- Efficiency: `avg` = net Wh / (trip distance / `EFF_DIST_UNIT_M`), valid from `EFF_MIN_DIST_M` (10 m) of trip distance. `now` is the same ratio over a ring of `EFF_WINDOW_S` (10) one-second buckets closed on integrator time; only closed buckets count, so it moves once a second and lags up to one. A bucket is a gap when the GNSS was unusable or no energy source was active during it, when it took an outage catch-up or a counter reset, or when it came from a tick longer than `TRIP_DT_MAX_MS` (5 s, only if the task itself stalls). `now` is valid with `TRIP_EFF_MIN_FILL_S` (3) closed buckets, `EFF_MIN_DIST_M` of window distance, no gap in the window, a usable GNSS and an active energy source right now; otherwise `--`. The unit follows the speed unit (knots -> Wh/NM, km/h -> Wh/km); `EFF_UNIT_KM 1` forces Wh/km on a knots build.
- Resolution: STATUS_3 arrives at the VESC's Rate 2 (1..5 Hz, section 3), so between frames the counters stand still and the window fills in steps; a STATUS_3 dropout shorter than `VESC_STALE_R2_MS` (5 s) is invisible to the freshness test and makes `now` read low until the frames resume (window and totals self-correct). A VESC reboot while its counters were still below 0.5 Wh is not detected (at most 0.5 Wh lost).
- Log: `TRIP run=83s moving=62s dist=0.101NM wh=10.1 whc=0.1 now=101Wh/NM avg=99Wh/NM win=10s P=598W src=counters resets=0` every `LOG_TRIP_MS` (5 s; 0 = off), `now=--` / `avg=--` while invalid, ` (never updated)` or ` (stale 1234ms)` appended when the trip state stopped updating. The task owns no peripheral and is not watched by the watchdog; a dead trip task shows as `--` on the EFFICIENCY screen after `TRIP_STALE_MS` (5 s) and as the `stale` suffix in the log.

## 5. Dead-man's switch (BLE beacon -> the VESC's kill-switch input)

Optional: `pio run -e deadman -t upload` adds a dead-man's switch to the default build. A BLE tag
worn by the helmsman is watched by the board's own Bluetooth LE radio; when its advertisements
stop arriving for `DEADMAN_TIMEOUT_MS` (5 s) the firmware pulls `PIN_DEADMAN_CUT` low and the motor
stops. **Nothing is sent over CAN** - the cut goes through the VESC's own kill-switch input, which
is also why it works regardless of where the throttle comes from (section 9).

**This is not a certified man-overboard device and does not replace a mechanical lanyard kill
switch.** It is a second, additive layer on top of one. A spurious cut near rocks, in a lock or in
a shipping lane is itself dangerous; so is trusting a 2.4 GHz link that water and a human body
attenuate heavily. Read the whole section before wiring anything.

### Wiring

The ESP32 GPIO goes in **parallel** with the boat's existing mechanical kill switch, on the same
VESC ADC2 input, as an **open-drain** output: high-Z means "permit" and lets the mechanical switch
drive the line, low means "kill". The firmware can therefore only ever ADD a kill. A dashboard that
is in reset, crashed or unplugged leaves the line exactly as the mechanical switch has it, so this
feature cannot strand the boat by failing. A dashboard that is **unpowered but still wired in** is
the exception, unless you use the optocoupler or a shared supply ("Unpowered dashboard" below).

The diagram is for a switch whose contact is **closed while running** (normally closed, NC:
pulling the lanyard opens it). For one that is open while running, see "Normally-open switch"
below; the firmware is the same for both.

```
 3V3 --[ Rs ]--+--------+-- VESC ADC2     Rs is the one change to the harness: without it,
               |        |                 pulling ADC2 low shorts 3V3 to GND through the
 mech. NC sw --+        |  [ Rpd ]        closed mechanical switch.
                        |     |
   ESP32 PIN_DEADMAN_CUT+    GND          open drain: high-Z = permit, driven low = kill
      + a common ground with the VESC (already there via the CAN transceiver)
```

**Sizing Rs against the existing pull-down is a safety constraint, not a detail.** The VESC's
threshold is a hard-coded 1.65 V **with no hysteresis**, sampled every 10 ms, so with the ESP32
high-Z and the switch closed, `3.3 x Rpd/(Rs+Rpd)` must sit clear of it or the motor will chatter:

| Rpd (measure yours) | with Rs 4.7k | verdict |
|---|---|---|
| 10 k | 2.24 V | ok |
| 4.7 k | 1.65 V | **on the threshold - chatters** |
| 100 k | 3.15 V | ok |

Rule of thumb: **Rs <= Rpd/2**. Measure ADC2 with the switch closed before and after fitting Rs,
and confirm the `KILLSW` indicator on the VESC 2/3 page follows both the mechanical switch and
`PIN_DEADMAN_CUT`. `PIN_DEADMAN_CUT` defaults to GPIO1 (GPIO1, 8 and 18 are the free header pins);
it must never be a strapping pin (8/9) or 15, because a glitch low at reset would be a spurious
engine cut - `config.h` refuses to build those.

**Normally-open switch.** "Normally open" names the contact's state without the clip, so check
what yours does with the lanyard attached. Closed while running is electrically the NC case above:
nothing changes. Open while running, closing when the lanyard is pulled, goes to the **ground
side** of the line, as a wired-OR with the ESP32 - both of them pull ADC2 to GND:

```
 VESC 3V3 --[ Rpu ]--+-------------- VESC ADC2     Rpu 1k..2.2k, from the VESC's own 3V3 (never
                     |                             the dashboard's: unpowered, it would read as kill)
   mech. NO sw ------+------ GND     closes when the lanyard is pulled -> ADC2 = 0 V -> kill
                     |
   ESP32 PIN_DEADMAN_CUT             open drain: high-Z = permit, driven low = kill
```

- Firmware and VESC: no change. The cut is still "low = kill", the VESC stays on `ADC2 Low`, and
  the closed loop on `KILLSW` works the same. Remove Rs and the old pull-down; Rpu is the only
  resistor and also limits what the ESP32 sinks (1.5..3.3 mA).
- Rpu 1k..2.2k rather than 10k+: the switch contact sees that current when it closes (a marine
  lanyard contact is rarely gold-plated, and a fraction of a mA through an oxidised one may not
  register a kill), and a low-impedance line resists motor noise, which matters because the
  VESC's threshold has no hysteresis.
- With the switch open and the ESP32 high-Z, ADC2 must read at least 2.5 V. If it does not,
  something pulls the pin down; the rule is then **Rpu <= Rpd/2**, the mirror of the NC case.
- **Never** wire a normally-open switch to 3V3 with `Kill Switch Mode = ADC2 High`. An open-drain
  output can only pull low, which in that mode means "permit": the firmware could never cut, and
  while it held the line low it would fight the switch through the resistor and could mask the
  mechanical kill.
- The trade-off: with a contact that is open while running, a broken wire or an unplugged
  connector reads as "run". The mechanical kill is then silently gone, and nothing can detect it,
  because the VESC only reports a kill once the line actually goes low. The NC wiring fails toward
  a kill instead. If your lanyard switch also has an NC contact (many are SPDT), use that one with
  the wiring above. Otherwise pull the lanyard at the dock before every outing and check that
  `KILLSW` appears on VESC 2/3.

**Unpowered dashboard.** The FireBeetle runs from its own USB-C or battery, not from the VESC. When
it is wired in but unpowered, the line's run level (2.2..3.3 V through Rs or Rpu) meets the GPIO's
input protection diode into the board's dead 3V3 rail (the same diodes the UART series resistors
are there for, section 3b). That can drag ADC2 below 1.65 V and **cut the motor** when the
dashboard's battery or USB supply dies. Commissioning step 5 tests exactly this. Two fixes, neither
needs a firmware change:

- Power the dashboard from the VESC's 5 V (or the same switched supply), so it is never unpowered
  while the VESC runs.
- Or put an optocoupler (PC817 or similar) between the dashboard and ADC2. Its transistor replaces
  the GPIO on the line, so an unpowered, resetting or hung dashboard leaves it truly untouched, and
  the cut line becomes galvanically isolated. The polarity is the same as today (GPIO low = LED on =
  cut):

```
 dashboard 3V3 --[ 330R ]-->|-- PIN_DEADMAN_CUT (open drain)     PC817 LED, ~6 mA while cutting
                         LED
 VESC ADC2 ------------ C   E ------------ VESC GND              PC817 transistor, across the line
```

  The transistor must be able to sink the line's current: a PC817 (CTR >= 50 % at ~6 mA) sinks at
  least 3 mA, so use Rpu >= 2.2k with a normally-open switch; Rs 4.7k (about 0.7 mA) with the NC
  wiring is fine.

### On the VESC

Nothing to change if the mechanical switch already works: VESC Tool -> App Settings -> General ->
**Kill Switch Mode** = `ADC2 Low` for a 3.3 V-high "run" line, which both wirings above produce.
`ADC2 High` does not work with this design (see "Normally-open switch"). On the entry edge the VESC's timeout
thread releases the motor immediately (`mc_interface_release_motor_override()`, then
`mc_interface_ignore_input_both()` re-armed every 10 ms), which overrides the local PPM/ADC
throttle rather than racing it. **Cut latency is about 10 ms, not the app timeout.** The VESC
reports the state back as status bit 1, which this firmware already polls and shows as `KILLSW` -
that is what closes the loop below.

### States

| State | Cut | Meaning |
|---|---|---|
| `NO TAG` | `DEADMAN_BOOT_CUT` | no tag has enrolled yet. With the default 0 the motor runs and there is **no protection**; with 1 the tag cannot be forgotten, at the price of a dead tag battery immobilising the boat until the dashboard is powered down |
| `ARMED` | no | at least one enrolled tag is fresh |
| `GRACE` | no | every enrolled tag quiet past `DEADMAN_WARN_MS`; the tag row counts down (`CUT IN 3s`) |
| `TRIPPED` | **yes** | timed out. Latched: the tag coming back does not release it |
| `NO RADIO` | no | the BLE scan is not running. **Does not cut** - our own failure must not stop the boat, the mechanical switch still covers the real hazard - but it means there is no protection, so it takes the tag row and logs at ERROR. It also throws the enrolment set away: when the radio returns, every tag must prove itself again |
| `FAULT` | **yes** | the cut was asserted but the VESC never reported `KILLSW` within `DEADMAN_CONFIRM_MS` while its polls were arriving. Wiring, series resistor or VESC config. Never clears by itself |

Arming deliberately needs `DEADMAN_ARM_REPORTS` (3) adverts inside `DEADMAN_ARM_WINDOW_MS` (5 s)
with the last at `DEADMAN_ARM_RSSI` (-85 dBm) or better: arming on one weak packet from the car
park would mean the first thing that happens when you leave the dock is a trip.

While the motor is cut a full-screen **alarm replaces whatever page was selected** - it is not
something you can page away from - and the long press that normally returns to MAIN clears the
latch instead. The reset is **refused** unless the tag has been seen within
`DEADMAN_RESET_FRESH_MS` (1 s), so the motor cannot be restarted while the person is still in the
water; the alarm's last row says which of the two it will do.

### Two tags

`DEADMAN_BEACON_ADDR2` adds a second tag for a second crew member; leave it empty for a one-tag
boat and nothing below applies. **With two tags enrolled the motor is cut only when BOTH have gone
quiet** - either one aboard keeps it running, so a flat battery in one tag is survivable.

Which tags count is decided **once**, when protection begins. The first tag to complete its arming
burst enrols and opens a `DEADMAN_ENROL_MS` (5 s) window; any other configured tag that arms inside
that window joins; then the set is frozen for the run and the log says who is in:

```
W (48120) dm: enrolment closed: 2 tags guarding the motor. t1=f2:28:3c:03:e4:ad last=110ms rssi=-62 n=41 gapmax=0ms t2=de:ad:be:ef:00:02 last=90ms rssi=-70 n=38 gapmax=0ms
```

Freezing the set is the point: a tag that turns up afterwards - the one left in the car, still in
range at the dock - can never join and silently keep the switch alive. It shows as `off` on the
main screen and is ignored entirely. The window opens on the first sighting rather than at boot, so
powering the dashboard up before you put the tag on still works.

Arming is **per tag**: each needs `DEADMAN_ARM_REPORTS` of its own adverts inside
`DEADMAN_ARM_WINDOW_MS` with its own last one at `DEADMAN_ARM_RSSI` or better. Two tags cannot
contribute half a burst each, and a weak tag is never carried in by a strong one. Each tag's
`gapmax` is measured over its own adverts, never the union of both, so the "size the timeout at 5x
gapmax" rule below stays valid if you later carry only one tag.

What this buys and what it costs: a dead tag battery no longer looks like a man overboard. But the
guarantee is weaker - a tag left lying on the boat by someone who then goes over the side keeps the
motor running. That is inherent in "cut only when both are gone", and it is why the mechanical
lanyard remains the primary protection.

The two addresses must differ; the firmware refuses to run and holds the cut if they do not, since
both slots pointing at one fob would look armed while guarding half as much.

### The radio

In normal operation the radio does exactly one thing: one passive scan (`src/ble_scan.cpp`), continuous
(`DEADMAN_SCAN_ITVL_MS` = `DEADMAN_SCAN_WINDOW_MS` = 100 ms), with `filter_duplicates` off - a
liveness watchdog needs every advert, not one per scan instance. NimBLE runs in the observer role
only (`sdkconfig.defaults.ble`): no connections, no advertising, no GATT, so nothing ever
interrupts the scan. The `ble` task restarts it within 100 ms if anything stops it (a host reset,
a controller hiccup); while it is down the switch reads `NO RADIO` and does not cut. Tags are known
by address only (`DEADMAN_BEACON_ADDR`, `DEADMAN_BEACON_ADDR2`): there is no discovery mode.

Two consequences of the BLE stack as configured: the tag must use **legacy advertising on the 1M
PHY** (`CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT` is off, so extended-advertising beacons are invisible),
and a dead tag battery is indistinguishable from a man overboard. (Update mode, section 7, uses
the radio for Wi-Fi instead, in a separate boot with no BLE and the cut held.)

### Commissioning

Do this in order; the first step costs nothing and catches most of the mistakes.

1. **Run it with the cut line disconnected** for several normal outings. The firmware arms, trips
   and resets for real without touching the motor. Requirement before connecting the wire: zero
   spurious trips over hours, with the tag carried where it will actually be carried. Watch
   `gapmax` in the `DM` log line - **size `DEADMAN_TIMEOUT_MS` at 5x the worst gap you ever see**,
   not by guessing. After connecting it, measure ADC2 with a meter: at least 2.5 V while running
   with the dashboard high-Z, about 0 V with the lanyard pulled.
2. Connect the line. On the bench, prop out of the water: walk the tag away, confirm the motor
   stops, `St KILLSW` appears on VESC 2/3 within two poll periods and the alarm says `VESC: KILLSW
   ok`. Time "tag gone" to "motor stopped" with a clamp meter.
3. **Negative test**: disable the kill switch in VESC Tool and repeat. The firmware must reach
   `FAULT` and say `VESC: NOT CUT!`. This is the test that proves the closed loop is real - without
   it, a broken cut wire looks identical to a working one until the first real trip.
4. Confirm the mechanical lanyard still works on its own with the dashboard powered off. This is
   the fail-passive property and the single most important test here.
5. Mid-run, pull the dashboard's power (USB / battery) **with the cut line still connected**, then,
   powered again, force a watchdog reboot: the motor must keep running both times, and ADC2 must
   stay at least 2.5 V. If it drops with the dashboard unpowered, fit the optocoupler or power the
   dashboard from the VESC ("Unpowered dashboard" above).
6. With two tags: check the `enrolment closed` line names both. Walk tag 2 out of range - the motor
   must keep running and the row must show `2: --`. Then walk tag 1 out too: the cut must come
   `DEADMAN_TIMEOUT_MS` after the **second** tag went, not the first. Separately, power up with only
   tag 1 present and then bring tag 2 into range: it must show `off` and never join.
7. Only then a man-overboard rehearsal: tethered, low speed, open water, warm, a second person at
   the helm, mechanical lanyard still worn.

With a normally-open mechanical switch, add one check before **every** outing: pull the lanyard
at the dock and confirm that `KILLSW` appears on VESC 2/3. A broken wire to that switch is
otherwise invisible.

Update mode (section 7) runs no dead-man and holds the cut instead. Check that once at the dock
with the cut line connected: hold the button 5 s; while the FW UPDATE screen is up ADC2 must read
about 0 V and VESC Tool must show the kill switch active (the dashboard cannot confirm it there:
update mode has no VESC link). A long press leaves update mode.

Log line (tag `dm`, every `LOG_DEADMAN_MS`), plus an immediate line on every transition:

```
I (63120) dm: DM state=ARMED since=412s out=permit tag=e1:23:45:67:89:ab last=120ms rssi=-62 n=18431 gapmax=340ms margin=4880ms trips=0 reset=0/0 confirm=- other=48219 skip=0
E (71200) dm: TRIPPED: no tag for 5012 ms (timeout 5000, worst gap while armed 340 ms) - motor CUT and latched; long press on the DEADMAN screen with the tag present to reset
```

## 6. Display behaviour

128 x 64, white on black, `OLED_ROTATION 0`. Text is drawn by u8g2 (the C library in `components/u8g2`, full frame buffer, SSD1309 `noname0` driver, HAL in `src/u8g2_hal_idf.cpp`): `u8g2_font_logisoso24_tn` for the speed (24 px digits, tabular 15 px advance; the font has digits, `-`, `.` and `:` only), `u8g2_font_6x10_tf` for all other text (6 px monospace, 7 px caps, has the degree sign) and `u8g2_font_6x13B_tf` (bold, same 6 px advance) for the title bars. Only the three referenced fonts are linked (~4.7 KB of flash); the rest of the U8g2 font file costs nothing. A row of small text is 21 characters wide.

Seven screens, cycled by the button (section "Button" below); the detail screens carry their page number `n/7` in the title bar. All value fields are fixed-width cells; `--` means stale or never received.

```
 MAIN (screen 1)              EFFICIENCY (2/7)             VESC 1/3 (3/7)               VESC 2/3 (4/7)
 +---------------------+      +---------------------+      +---------------------+      +---------------------+
 |   6.2   12:34 3D 9sv|      |[EFFICIENCY      2/7]|      |[VESC 1/3        3/7]|      |[VESC 2/3        4/7]|
 | (24 px) VESC 74     |      |now 101 Wh/NM        |      |LAST OT_FET 34s      |      |Tmos 45.1  Iin 12.4  |
 |         kn          |      |avg 98.5 Wh/NM       |      |Vin 48.2   Ibat 12.4 |      |Id 0.3     Iq 34.9   |
 |---------------------|      |dist 6.10  601Wh     |      |Imot 35    Duty 45%  |      |Vd 1.23    Vq 23.45  |
 |BV   48.2v BA   12.4A|      |P 598W     win 10s   |      |P 598W     RPM 2350  |      |Tach 12345 Abs 23456 |
 |MA     35A PW    598W|      |run 1h23m  mov 1h02m |      |Tfet 45.3  Tmot 32.1 |      |St OK      id 74     |
 |TF     45° TM     32°|      |mean 5.9kn src S3    |      |Ah 12.566  Wh 605.7  |      |AhC 0.098  WhC 4.7   |
 |RPM   2350 WH   605.7|      +---------------------+      +---------------------+      +---------------------+
 +---------------------+
 VESC 3/3 (5/7)               GNSS (6/7)                   SYS (7/7)
 +---------------------+      +---------------------+      +---------------------+
 |[VESC 3/3        5/7]|      |[GNSS            6/7]|      |[SYS 0.1.0       7/7]|
 |PPM 0.52   PID 12.3  |      |3D 9sv     pDOP 1.5  |      |up 1h23m   btn 12    |
 |ADC 1.23 2.10 0.00   |      |59.43701N 24.75368E  |      |heap 250k  min 240k  |
 |poll 123   ok 120    |      |Alt 12.3m  hAcc 2.1m |      |CAN RUN    T0 R0     |
 |bad 0      tmo 3     |      |Spd 6.2    sAcc 0.30 |      |GNSS RUN 115200 34.10|
 |frm 12345  oth 0     |      |Hdg 123.4  vAcc 3.0m |      |ubx 1234/0 rd 0      |
 |misc 3     busoff 0  |      |2026-09-06 12:34:56Z |      |rst POWERON lock 0   |
 +---------------------+      +---------------------+      +---------------------+
```

- **MAIN.** Zone A (rows 0..23): ground speed right-aligned in the big font (max 4 characters), then a 12-character column at x 56 with three lines: the local clock `HH:MM` (or `--:--`) followed by the fix field, the CAN line, and the speed unit (`kn` / `km/h`). A 1-px rule at row 26, then Zone B: four rows of two 64-px cells, label left, value right-aligned (up to 6 characters). Fix field: `3D 9sv` / `3D12sv` / `2D 7sv` (fix type, satellites used), `NOFIX` (receiver alive, no 2D/3D fix), `NODATA` (no epoch for `GNSS_STALE_MS`), `NOGNSS` (nothing received since boot), `BOOT` on the first frame. CAN line: `VESC 74` (controller running, STATUS_1 or STATUS_5 within `VESC_STALE_R1_MS`), `CAN idle` (running, no fresh frame: VESC off or status messages not enabled), `CAN RECV` (bus-off recovery), `CAN OFF` (STOPPED / BUS_OFF / driver not installed); the boot frame shows `FW_VERSION` here.
- **Cell glossary.** `BV` battery voltage (STATUS_5, one decimal, `48.2v`); `BA` battery current (STATUS_4, signed, one decimal while it fits: `12.4A`, `-12.4A`, `-123A`); `MA` motor current (STATUS_1, whole amps: `35A`, `-1000A`); `PW` electrical power = voltage x battery current, signed, only when both are fresh (`598W`, `-4800W`, `12.3kW` from 10 kW); `TF` MOSFET temperature (STATUS_4, whole degrees, `--` outside -40..200 C); `TM` motor temperature (`n/a` when the VESC reports an implausible value because no motor NTC is wired); `RPM` mechanical rpm = erpm / (`VESC_MOTOR_POLES` / 2) (`2350`, `12.3k` from 10000); `WH` watt-hours drawn since the VESC booted (STATUS_3: one decimal below 1000 `605.7`, whole below 10000 `1235`, then `12.3k`, `123.5k`).
- **EFFICIENCY** (from `g_state.trip`, section "Efficiency and trip"). `now` = net Wh per distance unit over the last `EFF_WINDOW_S` (10 s) and `avg` since boot, one decimal below 100 (`98.5`, `-12.3` while regenerating), whole above (`123`); the unit is `Wh/NM` on a knots build and `Wh/km` on a km/h build or with `EFF_UNIT_KM 1`. `dist` trip distance in that unit (two decimals while they fit: `6.10`, `123.4`, `1235`) and net trip energy (`601Wh`, `-250Wh`, `1.23kWh`, `12.35kWh`, `1.23MWh`); `P` average electrical power over the window (`598W`, `-480W`, `12.3kW`) and `win` the seconds of data in it (0..10); `run` integrator time since boot and `mov` time above `EFF_MIN_SPEED_MM_S` (`12m34s`, `1h23m`, `12d03h`, whole days from 100 d); `mean` speed while moving (distance / moving time, in the speed unit) and `src`: `S3` = energy from the VESC watt-hour counters, `PxI` = the `v_in x current_in` fallback. `--` rules: `now --` until the window holds `TRIP_EFF_MIN_FILL_S` (3) closed seconds and `EFF_MIN_DIST_M` (10 m) of distance with no gap (GNSS unusable, no energy source, counter reset or a stalled tick inside the window); `avg --` below `EFF_MIN_DIST_M` of trip distance; `P --` before the first window second closes; `mean --` while nothing has moved; the whole screen `--` when the integrator has not stamped `trip.t_ms` for `TRIP_STALE_MS` (5 s) or never ran.
- **VESC 1/3.** Row 1 is the fault row: `FAULT <name>` while the live fault byte of a fresh poll reply is non-zero, `LAST <name> <age>` when a fault was latched earlier (`34s`, `12m`, `3h`, `2d`), `FAULT none` (fresh reply, no fault, nothing latched), `FAULT --` (polled values stale, or `VESC_POLL_MS 0`). Then input voltage / battery current (one decimal), motor current / duty %, power / rpm, FET and motor temperature with one decimal (raw, no plausibility filter), Ah / Wh drawn.
- **VESC 2/3** (polled values, `VESC_EXT_STALE_MS` window): `Tmos`, the first MOSFET temperature (the VESC reports three; MOSFET 2 and 3 stay in the poll and the log but left the screen) with `Iin`, the VESC's own averaged input current (`Tmos --    Iin --` when stale), average d/q currents, d/q voltages, tachometer (STATUS_5) / absolute tachometer (polled), status `St OK` / `TIMEOUT` (no control input, timeout brake active) / `KILLSW` / `TO+KILL` and the controller id from the reply, charged Ah / Wh.
- **VESC 3/3**: PPM input / PID position, `ADC` 1/2/3 volts (`ADC --` when STATUS_6 is stale), poll counters `poll` (requests sent) / `ok` (complete replies) / `bad` (CRC or format failures) / `tmo` (reply timeouts) - `poll off   ok --` and `bad --     tmo --` in a `VESC_POLL_MS 0` build -, `frm` accepted status frames / `oth` frames from another controller id, `misc` non-status or odd-DLC frames / `busoff` events.
- **GNSS**: fix and satellites (`3D 9sv`, `2D 12sv`, `TIME ONLY`, `NO FIX`, `NO DATA`, `NO GNSS`, or the phase `AUTOBAUD` / `DETECT` / `CONFIG` while nothing has been received) with pDOP; latitude/longitude with five decimals (`Pos --` without a usable fix); height above sea level / horizontal accuracy; speed (same rules as the main screen) / speed accuracy in m/s; heading of motion (only while moving) / vertical accuracy; UTC date-time (`UTC --` when unknown).
- **SYS `FW_VERSION`**: uptime (`12m34s`, `1h23m`, `12d03h`) / debounced button presses, free heap / minimum free heap in kB, TWAI state (`UNINST` / `STOP` / `RUN` / `BUSOFF` / `RECOV`) with the TEC and REC error counters, GNSS phase, baud and PROTVER, good/bad UBX frames and redetects, reset reason / `state_lock()` timeouts.
- **Dead-man overlay** (`deadman` build, section 5). While the firmware is holding the motor cut
  (`TRIPPED` or `FAULT`) a full-screen alarm **replaces** the selected page, so a cut motor is not
  something you can page away from: title `MOTOR CUT`, then the state and how long the tag has been
  missing, the worst advert gap while armed, the tag's RSSI and advert count, the closed-loop result
  against the VESC (`VESC: KILLSW ok` / `waiting...` / `NOT CUT!` / `no reply`), the trip and
  refused-reset counters, and a last row saying whether a long press will reset right now
  (`HOLD BTN: RESET`) or be refused because the tag is absent (`HOLD BTN: NEED TAG`). A `FAULT`
  cannot be reset from the panel at all (`FIX WIRING - NO RESET`). The MAIN screen's CAN cell is
  never taken over any more: the dead-man has its own row (below), which is what let the CAN line
  have its cell back.
- **Dead-man tag row** (`deadman` build). The third right-column line - the one the art above shows
  as `kn` - carries the dead-man instead of the speed unit. A unit that never changes is worth less
  than knowing whether the switch is protecting you and how much signal margin each tag has, and
  the unit is a build-time constant that is still on the EFFICIENCY page (`mean 5.9kn`). First
  match wins: `NO RADIO` (the scan is down, so there is no protection at all), `CUT IN 3s` (the
  `GRACE` countdown), else one slot per configured tag: `1:-62 2:-70`. A slot reads ` --` while its
  tag is quiet (older than `DEADMAN_WARN_MS`) and `off` once the enrolment window closed without it
  - that is the "it was missing at startup, so it guards nothing" state. A single-tag boat reads
  `1:-62`, and `NO TAG` means nothing is configured. RSSI is clamped to -99 dBm so two slots always
  fit the 12-character line.
- **Formatting rules.** A number that does not fit its cell drops decimals one by one (`48.2` -> `100`), then switches to thousands (`12.3k`, `12k`), millions (`12M`) and finally `MAX` / `-MAX`; a clipped-but-plausible wrong number is never shown. Counters: `12345`, `1234k`, `123M`, `4G`. `-0.0` is printed as `0.0` so a current hovering around zero does not flicker. Speed: `0.0` below `SPEED_MIN_SHOW`, one decimal below 100, whole above, `--` without a usable fix or with a speed-accuracy estimate above `GNSS_MAX_SACC_MM_S`. Freshness windows: STATUS 1/4/5 `VESC_STALE_R1_MS`, STATUS 2/3/6 `VESC_STALE_R2_MS`, polled values `VESC_EXT_STALE_MS`, GNSS `GNSS_STALE_MS`, trip integrator `TRIP_STALE_MS`; anything older, or never received, is `--`.

### Button

The display task samples `PIN_BUTTON` every `OLED_BUTTON_POLL_MS` (20 ms) and accepts a level once it has been stable for `BUTTON_DEBOUNCE_MS` (30 ms). Releasing before `BUTTON_LONG_PRESS_MS` (1500 ms) is a short press: next screen, wrapping from SYS back to MAIN. Holding for `BUTTON_LONG_PRESS_MS` is a long press: back to MAIN, fired once while the button is still held (the release then does nothing). The selected screen then stays up indefinitely: `SCREEN_AUTO_RETURN_MS` is 0 (off) by default, and setting it to a number of milliseconds brings back the automatic return to MAIN that long after the last button event. Every press counts (`btn` on the SYS screen, `btn=` in the SYS log line), restores the contrast if the panel was dimmed and restarts the idle timer; a screen change is drawn immediately, not at the next tick, and logged as `display: screen 1 (EFF), short press` (screen indices 0..6 = MAIN, EFF, VESC 1/3, VESC 2/3, VESC 3/3, GNSS, SYS; reasons: `short press`, `long press`, `auto-return`). The button works even while no panel answers.

Holding on to `OTA_LONG_PRESS_MS` (5 s; the long press at 1.5 s has already gone back to MAIN by then) asks for update mode, section 7: logged as `display: 5000 ms hold -> update mode requested`. Refused, the panel shows `UPDATE REFUSED` and the reason for `OTA_REFUSED_SHOW_MS` (3 s); accepted, the board reboots. On the dead-man alarm the long press still requests the reset and the 5 s hold is always refused. In update mode a long press leaves it.

Refresh and idle policy (`config.h`):

| Define | Default | Effect |
|---|---|---|
| `OLED_PERIOD_MS` | 250 | frame tick (4 Hz); each tick renders a snapshot into the frame descriptor of the current screen and compares it with what is on the panel; nothing changed = no I2C traffic. A changed frame pushes the whole 1 KB buffer, ~26 ms at 400 kHz |
| `OLED_BUTTON_POLL_MS` | 20 | task loop period = button sampling granularity (must not exceed `OLED_PERIOD_MS`) |
| `OLED_CONTRAST` | 255 | normal contrast (0..255) |
| `OLED_IDLE_DIM_MS` | 600000 | nothing changed and no press for 10 min -> contrast drops to `OLED_IDLE_CONTRAST` (log `OLED: dimmed after N s idle`); the next change or press restores it (`OLED: contrast restored (value changed)` / `(button)`). 0 = never dim. A page with a live clock or a fault age changes every minute or second and never dims while shown |
| `OLED_IDLE_CONTRAST` | 8 | contrast while dimmed; segment current falls roughly 15..40x versus 255 |
| `OLED_ROTATION` | 0 | 0 = connector at the top, 2 = upside down. 1 and 3 (portrait) are rejected at compile time: the layout is landscape-only |
| `OLED_I2C_HZ` | 400000 | bus clock; the SSD1309 is specified to 400 kHz. 100000 for long or noisy wiring |
| `OLED_WIDTH`, `OLED_HEIGHT` | 128, 64 | panel geometry; the layout is hard-coded for it (`static_assert`) |

The panel is probed with a zero-length I2C transaction before every init, so a missing display is reported instead of drawn into blindly: `display_start()` logs `OLED: no response at 0x3C (check wiring/address jumper): ESP_ERR_NOT_FOUND` and the task retries every 5 s (`OLED: still no response at 0x3C (retry N, ESP_ERR_NOT_FOUND)`); while running it re-probes every 5 s and re-initialises a panel that stopped answering (`OLED: lost contact at 0x3C (ESP_ERR_NOT_FOUND), re-initialising`), as it does at once when a frame transfer fails (`OLED: frame transfer failed (ESP_ERR_INVALID_RESPONSE), re-initialising`), so hot-plugging works in both directions. After every (re)init the panel shows the boot frame (main screen, `--` everywhere, `--:--`, `BOOT`, firmware version) and the live frame on the next tick. `-DDISPLAY_DEMO=1` (the `demo` env, the default build plus the demo, no BLE) feeds synthetic changing values to all seven screens (the EFFICIENCY page runs a synthetic trip at 3 m/s and ~600 W) with a 30 s hold every 2 min, a live `OT_FET` fault for 10 s per cycle and the latched `LAST OT_FET <age>` row afterwards, so you can check every layout and the idle dimming with no VESC or GNSS attached (`-DOLED_IDLE_DIM_MS=20000` to see the dimming sooner).

## 7. Building, flashing, testing

Prerequisites: PlatformIO Core (`pio` on the PATH, e.g. `~/.platformio/penv/bin/pio`), git (u8g2 is a submodule) and a USB-C cable to the FireBeetle's USB port. The firmware is native ESP-IDF 6.1, no Arduino core: `platformio.ini` pins the official PlatformIO platform `espressif32 @ 7.1.1` (ESP-IDF 6.1.0, GCC 15.2 toolchain; `7.0.1` = ESP-IDF 6.0.1 is the fallback). The board definition is `boards/dfrobot_firebeetle2_esp32c6.json` (the official platform has none for this board), the ESP-IDF configuration is `sdkconfig.defaults` (the builder derives the gitignored `sdkconfig.<env>` from it; the dead-man envs append `sdkconfig.defaults.ble`, section 5, and ESP-IDF adds `sdkconfig.defaults.esp32s3` by itself on the S3-Zero) and the partition table is `partitions.csv`. The first build downloads the toolchain and ESP-IDF (about 1 GB), creates the IDF Python environment under `~/.platformio/penv/.espidf-6.1.0` and compiles ESP-IDF from source (a little over a minute on a recent laptop); later builds take seconds.

| Command | What it does |
|---|---|
| `git submodule update --init` | fetch u8g2 into `components/u8g2` (once per checkout) |
| `pio run` | build the default firmware (env `dfrobot_firebeetle2_esp32c6`) |
| `pio run -t upload` | build and flash over USB (`/dev/cu.usbmodem*`); hold BOOT while plugging in only if the port does not enumerate |
| `pio device monitor` | serial console at 115200 with exception decoder and timestamps |
| `pio run -e deadman -t upload` | the default build plus the BLE dead-man's switch (section 5; NimBLE host from `sdkconfig.defaults.ble`). `DEADMAN_BEACON_ADDR` (and `DEADMAN_BEACON_ADDR2` for a second tag) must be set to your hardware first: a tag address that never matches leaves the switch in `NO TAG` for ever |
| `pio run -e deadman_uart -t upload` | the `deadman` build with the VESC on its COMM-header UART instead of CAN (section 3b): no transceiver; App to Use must include UART |
| `pio run -e s3zero_deadman_uart -t upload` | `deadman_uart` on a Waveshare ESP32-S3-Zero (section 2b); the first build compiles ESP-IDF for the S3 |
| `pio test -e native` | host unit tests in eight directories (`test/test_vesc`, `test/test_vesc_getvalues`, `test/test_vesc_uart`, `test/test_ubx`, `test/test_trip`, `test/test_display_strings_oled`, `test/test_deadman`, `test/test_ble_addr`; 208 test cases): STATUS decoder, poll codec (request bytes, reassembly, CRC, decode, fault table), UART link codec (request frame, reassembly through junk, damaged frames and bogus headers, the state update from a reply), UBX parser and the CFG-PRT / CFG-RATE / VALSET frame builders (byte-exact against the u-blox frames), trip integrator (distance gate, both energy sources, counter resets, outages, window and gap rules, 32-bit clock wrap), the dead-man state machine (arming, enrolment of two tags, timeout, latch and reset, no radio, the closed loop), BLE address parsing, all OLED screens and overlays, the update-mode entry gate, and the clock arithmetic; no hardware |
| `pio run -e demo -t upload` | flash the layout demo (`DISPLAY_DEMO=1` on top of the default env: all seven screens and the dead-man row with synthetic data, dimming after 20 s; no BLE) |
| `pio run -t menuconfig` | ESP-IDF menuconfig on the env's generated `sdkconfig`; put lasting changes into `sdkconfig.defaults` |
| `pio project init --ide vscode` | regenerate IntelliSense after changing `platformio.ini` |

A healthy boot (ESP-IDF log format `L (ms) tag: message`; numbers and the `...` parts vary, the phrases are the ones the code prints) looks like:

```
I (1512) main: vesc-dashboard 0.1.0 (built Sep  7 2026 19:35:02) board=DFRobot FireBeetle 2 ESP32-C6 cores=1 app=0 radio=0 reset=POWERON
I (1513) main: cfg CAN : tx=3 rx=2 500 kbit/s mode=NORMAL(ack+poll) rxq=64 vesc_id=-1 poles=10 poll=1000 ms own_id=120
I (1514) main: cfg OLED: sda=22 scl=23 rst=14 addr=0x3C i2c=400000 Hz 128x64 rot=0 period=250 ms dim_after=600000 ms
I (1515) main: cfg GNSS: rx=4 tx=5 rate=200 ms unit=km/h rxbuf=2048
I (1530) can: CAN: node enabled: mode=NORMAL (ACK, transmits only GET_VALUES_SELECTIVE polls) bitrate=500k sample_point=800 permill single-shot tx=GPIO3 rx=GPIO2 rx_queue=64 tx_queue=4 poll=1000 ms own_id=120 mask=0x003EC03C
I (1535) gnss: GNSS: UART1 RX=GPIO4 TX=GPIO5 ring=2048 B, first baud 115200, target baud 115200, rate 200 ms
I (1540) oled: OLED: button on GPIO7 (active low, pull-up, debounce 30 ms, long press 1500 ms, auto-return 0 ms)
I (1575) oled: OLED: init ok (SSD1309 128x64, I2C 0x3C @ 400000 Hz, sda=22 scl=23 rst=14)
I (1611) main: started: can=1 gnss=1 display=1 trip=1 ble=0 deadman=0
I (1612) trip: trip: task started (period 200 ms, window 10 s, unit Wh/km, min speed 500 mm/s)
I (1640) oled: OLED: task started (poll 20 ms, frame 250 ms, 7 screens, button GPIO7, auto-return 0 ms, demo=0)
I (1650) can: VESC: locked onto controller id 74
I (5000) can: VESC id=74 frames=212 other=0 | erpm=0 rpm=0 duty=0.000 Im=0.0A(0.0) | Iin=0.2A(0.2) Vin=50.1V(50.1) P=10W | Tfet=41.0C Tmot=n/a | ... | age s1=12ms s2=340ms s3=341ms s4=15ms s5=9ms s6=never | ext: fault=NONE last=none Tmos=40.5/41.0/40.0C Iavg=0.00/0.20A Id/Iq=0.00/0.00A Vd/Vq=0.000/0.000V tachoAbs=0 st=- id=74 polls=3 ok=3 bad=0 to=0 age=410ms | can=RUN tec=0 rec=0 buserr=0 missed=0 arb=0 busoff=0 recov=0
I (6612) trip: TRIP run=5s moving=0s dist=0.000km wh=0.0 whc=0.0 now=-- avg=-- win=4s P=10W src=counters resets=0
I (8420) gnss: GNSS: UBX detected at 9600 baud (MON-VER reply)
I (8630) gnss: GNSS: PROTVER 18.00 -> legacy CFG configuration
I (8631) gnss: GNSS: switching UART1 9600 -> 115200 baud (CFG-PRT)
I (8850) gnss: GNSS: UART1 switched to 115200 baud (MON-VER reply)
I (8930) gnss: GNSS: NMEA off: 8/8 ACKed
I (8940) gnss: GNSS: CFG-RATE meas=200 nav=1: ACK
I (8950) gnss: GNSS: CFG-MSG NAV-PVT on: ACK
I (8970) gnss: GNSS: configured=1 velned=0 (baud 115200, PROTVER 18.00)
I (9001) gnss: GNSS phase=RUN baud=115200 protver=18.00 cfg=1 fix=3 ok=1 sats=9 spd=0.00km/h sAcc=0.40m/s ... age=110ms good=57 bad=0 redetect=0
I (10000) main: SYS up=10s reset=POWERON heap=... minheap=... lockfail=0 | can=RUN tec=0 rec=0 busoff=0 | disp ok=1 refreshes=.. skipped=.. dim=0 scr=0 btn=0 | poll sent=9 ok=9 bad=0 to=0 | hb can=.. gnss=.. disp=.. | ble=OFF hb=0
```

An M9/M10 receiver logs `GNSS: PROTVER 34.10 -> VALSET configuration`, `GNSS: switching UART1 38400 -> 115200 baud (VALSET)` and four `GNSS: VALSET ...: ACK` lines (UART1OUTPROT, MSGOUT, RATE, NAVSPG-DYNMODEL) instead of the CFG-MSG/CFG-RATE ones; one that still runs at 115200 from an earlier boot (backup battery) is found within a second and skips the switch. The first `TRIP` line shows `now=-- avg=--` until the boat has covered `EFF_MIN_DIST_M`. Before a VESC is connected the VESC line prints `never` for every age, `can=RUN` with `frames=0` and `polls=0` (no id to poll yet), the cells show `--` and the CAN line `CAN idle`; that is expected. With `-DVESC_POLL_MS=0` the install line ends in `poll=off` and the VESC line carries `| poll=off` instead of the `| ext:` section. The LED blinks at 1 Hz while VESC data is fresh and slowly otherwise. `OLED: refresh N ms (#N, screen N, stack free min N B)` per frame and `VESC poll: reply 42 B from id 74 in N ms: ...` per reply need `-DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG` in `build_flags` (debug lines of this project's own tags only; the IDF components stay at info).

### Over-the-air update

Every build with a button updates over Wi-Fi (`OTA_ENABLE`, on whenever `PIN_BUTTON` is set). The radio is only ever switched on for this, in a boot of its own:

1. At rest, hold the button for 5 s (`OTA_LONG_PRESS_MS`; the long press at 1.5 s returns to MAIN on the way, keep holding). The board reboots into update mode: the LED blinks at 4 Hz and the OLED shows the FW UPDATE screen with the access point `VESC-DASH-OTA` (`OTA_AP_SSID`), an 8-digit passphrase that is new every time, and `http://192.168.4.1`.
2. Join that network from a phone or laptop, open the address and pick the `firmware.bin` of the env you run (`.pio/build/<env>/firmware.bin`, copied to the phone beforehand).
3. Flash firmware. The screen counts `RX 612K/1105K` with the percentage in the title bar, the page shows a progress bar. The image goes into the other app slot, is verified (checksum, SHA-256, chip) and selected, and the board reboots into it.
4. The new image runs on probation and is confirmed after `OTA_CONFIRM_MS` (60 s) with every task heartbeat fresh, in a dead-man build also with the beacon scan running (`ota: new image confirmed after 60 s with every task alive: rollback cancelled`). A panic, a watchdog reset **or a power cycle** before that boots the previous image again (bootloader rollback), so leave it powered for a minute after an update.

Update mode starts nothing but the display, the button and the Wi-Fi: no VESC link, no GNSS, no trip, no BLE. In a dead-man build **the cut is held** (ADC2 low) from the first milliseconds of the update-mode boot until the reboot out of it, because nothing watches the tags. It ends with a long press (refused while an image is being written), after `OTA_IDLE_TIMEOUT_MS` (5 min) without an upload in progress (the countdown is on the last row), or with the reboot after a successful upload. A power cycle always comes up normally: the request crosses the reboot in RTC memory, is honoured only after a software reset, and only once.

The hold is refused, with the reason on screen for 3 s, while fresh VESC data shows the motor turning (more than `OTA_MAX_RPM`, 100 rpm), a fresh and accurate GNSS fix shows more than `OTA_MAX_SPEED_MM_S` (500 mm/s, about 1 kn), the dead-man alarm is up (the reboot out of update mode would clear a trip without the tag), the running image is still on probation, or the partition table has no second slot. Stale data counts as stationary, so a VESC that is switched off at the dock does not block an update.

Rejected uploads leave the running image untouched and show `UPLOAD FAILED` with the reason: a file that does not start with the image magic (`not a firmware image`), an image of another ESP-IDF project (`not vesc-dashboard fw`, by the project name in its app descriptor), one for the other chip or a truncated one (`image failed checks`), one larger than the slot. Two builds of this project for the same chip look alike, though: a `dfrobot_firebeetle2_esp32c6` image flashed onto a dead-man boat removes the dead-man. Pick the env deliberately.

The first OTA-capable firmware needs one wired flash: the partition table (two 1.94 MB app slots, no filesystem) and the rollback-enabled bootloader only arrive with `pio run -e <env> -t upload` over USB. A wired upload also writes an empty otadata, so it always boots the wired image from app0, whatever the last update selected.

A session in the log (numbers vary):

```
W (61210) oled: display: 5000 ms hold -> update mode requested
W (61211) ota: entering update mode: rebooting
...
I (1515) ota: running app0 at 0x010000 (1984 KB slot), image undefined, version 8ea95aa, 2 app slots, hold 5000 ms for update mode
W (1516) main: *** UPDATE MODE: no VESC link, GNSS or BLE, motor cut held; waiting for a firmware upload ***
I (2610) ota: access point up
W (2650) ota: update mode: join "VESC-DASH-OTA" with passphrase 48213907, then open http://192.168.4.1
I (31200) ota: phone joined (aid 1)
W (45400) ota: receiving 1105 KB into app1
W (45420) ota: image is 1a2b3c4 (built Nov  7 2026 18:02:11), running 8ea95aa
W (61800) ota: wrote 1105 KB to app1, rebooting into it
...
I (1515) ota: running app1 at 0x200000 (1984 KB slot), image pending-verify, version 1a2b3c4, 2 app slots, hold 5000 ms for update mode
W (1516) ota: new image on probation: confirmed after 60 s with every task alive; a reset before that rolls back
W (60100) ota: new image confirmed after 60 s with every task alive: rollback cancelled
```

### Troubleshooting

| Symptom | Check |
|---|---|
| Panel dark; boot log `OLED: no response at 0x3C (check wiring/address jumper): ESP_ERR_NOT_FOUND`, then `OLED: still no response at 0x3C (retry N, ESP_ERR_NOT_FOUND)` every 5 s | `ESP_ERR_NOT_FOUND` = nothing ACKs at that address: the module's DC/SA0 jumper selects 0x3D (`-DOLED_I2C_ADDR=0x3D`), SDA/SCL swapped, module still in SPI mode (solder resistors), CS not grounded, module unpowered |
| Same lines with `ESP_ERR_TIMEOUT` | the bus never idles high: no pull-ups, SDA or SCL shorted, module unpowered. `OLED: I2C bus setup failed (sda=22 scl=23 400000 Hz): ...` = the I2C master itself could not be created (pin numbers, port in use) |
| `OLED: init ok` but the panel stays dark or dies at random | RES floating (wire GPIO14 or tie RES to 3V3); VCC not 3.3 V; a `-DOLED_CONTRAST` override near 0 |
| `OLED: frame transfer failed (ESP_ERR_INVALID_RESPONSE), re-initialising` or `OLED: lost contact at 0x3C (ESP_ERR_NOT_FOUND), re-initialising` | loose connector or a brown-out that reset the controller; the task re-initialises the panel by itself. A few `i2c.master` error lines from the IDF driver for the transfer that was in flight are expected |
| Garbage, an offset or mirrored image, only half the panel lit | an SSD1306 module (different init sequence, 128x32 variants; this build is 128x64 SSD1309 only) or an `OLED_WIDTH` / `OLED_HEIGHT` override. Upside down: `-DOLED_ROTATION=2` |
| Frames missing or flicker at 400 kHz | pull-ups missing or long wiring; `-DOLED_I2C_HZ=100000` |
| Button does nothing, `btn` on the SYS screen (and `btn=` in the SYS log line) stays 0 | boot log must show `OLED: button on GPIO7 (active low, ...)`, not `OLED: no button (PIN_BUTTON -1): main screen only`; switch wired to GND (or `-DBUTTON_ACTIVE_LOW=0` for a switch to 3V3); a press shorter than `BUTTON_DEBOUNCE_MS` is ignored. A long press always returns to MAIN, so a switch stuck closed looks like "nothing happens" |
| Clock shows `--:--` although the fix line is `3D 9sv` | the receiver has not flagged date and time valid yet (cold start: wait a minute); wrong local time = `TIME_UTC_OFFSET_MIN` (minutes, e.g. 120 for UTC+2) |
| `can=RUN` but `frames=0` and every age `never` in the VESC log line, cells `--`, CAN line `CAN idle` | status messages not enabled in VESC Tool (section 3); bitrate mismatch (`CAN_BITRATE_KBPS` vs VESC Tool); RXD level shifting missing or the divider output too low; TXD not wired (no ACK, the VESC then stalls, see section 3) |
| Status frames fine, but VESC 2/3 shows `--`, VESC 1/3 `FAULT --`, `tmo` climbs and every 10 s `VESC poll: N request(s) without a complete reply within 500 ms (VESC off, wrong id, CAN mode not VESC, or firmware without COMM_GET_VALUES_SELECTIVE); polling every 5000 ms` | VESC Tool > App Settings > General > CAN Mode must be VESC (not UAVCAN / Comm Bridge); `CAN_OWN_ID` equals the VESC id (then also `VESC: controller id 120 equals CAN_OWN_ID, polling suspended ...`); FW < 3.42 has no `COMM_GET_VALUES_SELECTIVE` (`-DVESC_GETVALUES_MASK=0`); a `VESC_CAN_ID` that no VESC has. With the VESC off these lines plus `VESC poll: N request(s) not acknowledged on the bus (VESC off? ack errors=N TEC=N)` and `CAN: error passive` are expected and stop as soon as it is back |
| `VESC poll: N bad reply/replies, last: length or CRC mismatch (len N, expected 42 for mask 0x003EC03C)` now and then, `bad` on VESC 3/3 | bus errors during a reply, or another CAN master (VESC Tool over CAN, a second display) whose traffic interleaves with ours; the next poll recovers. Constant: a `VESC_GETVALUES_MASK` override the decoder does not know (`unknown layout`) |
| `VESC poll: firmware answers 10 of 11 requested fields (41 of 42 B, ...)` once | VESC firmware < 5.03 has no status byte; harmless, `St` on VESC 2/3 stays `OK` |
| Repeated `CAN: BUS_OFF`, `CAN: error passive` or `CAN: N bus error(s)` with the VESC on | termination (60 Ohm across CANH/CANL), CANH/CANL swapped, grounds not commoned, RS not grounded, bitrate mismatch |
| `CAN: RX queue full, N frame(s) lost` warnings | something floods the bus, or canTask was held off; raise `CAN_RX_QUEUE_LEN` |
| GNSS stuck in AUTOBAUD, log repeats `GNSS: no UBX reply at any baud [115200:0B/$0 38400:0B/$0 9600:...]; retrying in 2000 ms` | `0B` at every baud: no bytes at all, so TX/RX swapped (module TX must go to GPIO4), power or ground. Bytes and `$` counts at one baud: an NMEA talker that ignores UBX input on this port (re-enable the UBX input protocol in u-center). Nothing sensible at any rate: a baud not in `GNSS_BAUDS` (add it) or 5 V logic |
| GNSS detected, `NOFIX` for minutes | antenna indoors; a cold first fix can take a few minutes outdoors |
| `GNSS: baud switch to 115200 failed (no reply to 2 MON-VER polls at the new rate, attempt 1/2); redetecting from 9600, redetect #1`, later `GNSS: baud switch to 115200 failed (2 attempts without a reply at the new rate), staying at 9600` | the receiver refused or ignored the new baud (a clone without CFG-PRT / CFG-UART1-BAUDRATE support, or a wiring fault that only shows at 115200): it is configured at the detected baud and works, only slower (5 Hz NAV-PVT is half a 9600 line; do not combine with `GNSS_RATE_MS 100`). A single `attempt 1/2` line followed by `GNSS: UART1 switched to 115200 baud (found there by autobaud after an unconfirmed switch)` is harmless: the switch worked and only its proof was lost. `-DGNSS_TARGET_BAUD=0` disables the switch |
| EFFICIENCY screen all `--` | trip task not running: the boot log must show `trip: task started (...)` and a `TRIP ...` line every 5 s without a ` (stale ...)` suffix. Only `now --`: fewer than 3 s in the window, less than `EFF_MIN_DIST_M` covered in the last 10 s, speed below `EFF_MIN_SPEED_MM_S`, speed cell `--`, or a gap in the window (CAN or GNSS dropout, VESC reboot). `now --` and `avg --` with a fix and the motor running: not yet `EFF_MIN_DIST_M` of trip distance, or the boat stays below ~1 kn |
| `NO RADIO` on the dead-man row in a `deadman` build, `started: ... ble=0` | NimBLE init failed: see the `ble:` error lines at boot (`nvs_flash_init`, `nimble_port_init`, task create). `ble=1` but `NO RADIO` later: the log shows `watchdog scan will not start` or `host reset`; the `ble` task retries every 100 ms |
| `firmware.bin` grew by about 300 KB | expected for the dead-man envs (NimBLE host and controller); Wi-Fi and the update server add more to every build with a button. Everything fits the 1.94 MB app slot |
| `UPDATE REFUSED` after the 5 s hold | the second row says why: stop the motor or the boat (fresh data above `OTA_MAX_RPM` / `OTA_MAX_SPEED_MM_S`), clear the dead-man alarm first, wait a minute after an update (the new image is not confirmed yet), or `no spare app slot` = the board still has an old partition table: flash once by USB |
| FW UPDATE shows `AP NOT UP - CHECK PWR`, log `*** the access point did NOT come up ...` | the Wi-Fi start draws the largest current peak on the board: a weak USB port or supply. Better supply, or a lower `OTA_AP_TX_POWER_QDBM` (34 = 8.5 dBm) |
| The phone does not list `VESC-DASH-OTA`, or keeps leaving it | 2.4 GHz channel 1 only; the AP admits one client at a time; many phones abandon a network without internet: tell it to stay connected, or switch mobile data off for the upload |
| `UPLOAD FAILED` with `not a firmware image` / `not vesc-dashboard fw` / `image failed checks` | wrong file: upload `firmware.bin` (not `bootloader.bin`, `partitions.bin` or the `.elf`) of this project, built for this board's chip; nothing was changed on the board |
| The previous version is back after an update (banner `ota: running app0 ... image valid` instead of the new one) | the new image did not survive `OTA_CONFIRM_MS` (a panic or watchdog reset, see `reset=`), or the power was cut before it was confirmed: the bootloader rolled back. Leave it powered for a minute after an update |
| `fatal error: u8g2.h: No such file or directory` | the submodule is empty: `git submodule update --init` |
| `#error This firmware targets the ESP32-C6 (ESP-IDF)` | the board or platform in `platformio.ini` was changed, `sdkconfig.defaults` lost `CONFIG_IDF_TARGET`, or the wrong env is selected |
| A change in `sdkconfig.defaults`, `sdkconfig.defaults.ble` or `sdkconfig.defaults.esp32s3` has no effect | the generated `sdkconfig.<env>` (`sdkconfig.dfrobot_firebeetle2_esp32c6`, `sdkconfig.deadman`, ...) already exists, and a value in it wins over the defaults: delete it and rebuild |
| The build stops inside `components/u8g2` with a warning reported as an error | ESP-IDF 6 turns the default warnings into errors for IDF-built components; add `CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y` to `sdkconfig.defaults` |
| `#error CAN_OWN_ID must be 1..254` / `CAN_OWN_ID must differ from VESC_CAN_ID` / `VESC_POLL_MS > 0 requires CAN_LISTEN_ONLY 0` | fix the offending override; a listen-only build needs `-DVESC_POLL_MS=0` as well |
| `#error DEADMAN_ENABLE needs the NimBLE host` | `-DDEADMAN_ENABLE=1` in an env whose sdkconfig lacks `sdkconfig.defaults.ble`: extend `env:deadman`, or give your env the same `board_build.cmake_extra_args` (and delete its stale `sdkconfig.<env>`) |
| `#error The S3-Zero runs dual-core ...` or `... must run on TASK_CORE_RADIO` | the generated `sdkconfig.s3zero_deadman_uart` predates `sdkconfig.defaults.esp32s3` (for example a single-core one from an older checkout): delete it and rebuild |

## 8. Changing pins and tunables

Everything lives in `include/config.h` as `#ifndef NAME / #define NAME value`. Either edit the file or override from `platformio.ini` without touching it, in your own environment that inherits the main one (the same pattern the `demo` env uses):

```ini
[env:myboat]
extends = env:dfrobot_firebeetle2_esp32c6
build_flags =
    ${env:dfrobot_firebeetle2_esp32c6.build_flags}
    -DPIN_CAN_TX=7 -DPIN_CAN_RX=6 -DPIN_BUTTON=9
    -DSPEED_UNIT_KNOTS=0 -DTIME_UTC_OFFSET_MIN=120 -DOLED_I2C_ADDR=0x3D
```

Then build with `pio run -e myboat -t upload`. For a boat with the dead-man's switch extend `env:deadman` (and inherit `${env:deadman.build_flags}`) instead; the NimBLE fragment comes along with the inherited `board_build.cmake_extra_args`.

Compile-time checks stop you from using GPIO12/13 (on the S3-Zero: the USB pins 19/20 and the flash / PSRAM pins 26..37), from a dead-man cut on a strapping pin of the board, from sharing a GPIO between two peripherals (the button pin takes part; `PIN_OLED_RST` is checked separately because it may be -1), from an unsupported CAN bitrate, from a `CAN_OWN_ID` outside 1..254 or equal to `VESC_CAN_ID`, from polling in listen-only mode, from a `GNSS_RATE_MS` outside 50..10000 ms, from an `EFF_WINDOW_S` outside 2..120 s, from a `TASK_PRIO_BLE` at or above `TASK_PRIO_GNSS`, from a BLE scan window longer than its interval and from `DEADMAN_ENABLE 1` without the NimBLE host in the sdkconfig (`sdkconfig.defaults.ble`), on the S3-Zero from an sdkconfig that is single-core or puts `app_main` or the radio on the wrong core, and, with `VESC_LINK_UART 1`, from `VESC_POLL_MS 0`, from equal UART pins and from a UART console in the sdkconfig. `src/gnss_ubx.cpp` adds `static_assert`s that `GNSS_TARGET_BAUD` is 0 or one of `GNSS_BAUDS` and that `GNSS_BAUD_SWITCH_ATTEMPTS` is 1..255. `src/display_oled.cpp` adds `static_assert`s for 128x64, `OLED_ROTATION` 0 or 2, `PIN_OLED_RST` in -1..30, contrast 0..255, `OLED_I2C_ADDR` a 7-bit address, `OLED_I2C_HZ` in 1..1000000, `PIN_BUTTON` in -1..30, `BUTTON_DEBOUNCE_MS` shorter than `BUTTON_LONG_PRESS_MS` and `OLED_BUTTON_POLL_MS` not above `OLED_PERIOD_MS`.

| Group | Defines |
|---|---|
| Board | `BOARD_S3_ZERO` (1 = Waveshare ESP32-S3-Zero, set by `[env:s3zero_deadman_uart]`; it moves the OLED / LED defaults and sets `VESC_UART_PORT` 2), `LED_WS2812` (1 = `PIN_LED` is a WS2812), `LED_WS2812_LEVEL` (1..255), `VESC_UART_PORT` (0 on the C6, 2 on the S3; never 1, the GNSS). Fixed per board, not overridable: `TASK_CORE_APP` / `TASK_CORE_RADIO` (S3 1 / 0, C6 0 / 0), the cores every task is pinned to |
| Display demo | `DISPLAY_DEMO` (1 = synthetic changing values on every screen, the `demo` env) |
| OLED | `PIN_OLED_SCL`, `PIN_OLED_SDA`, `PIN_OLED_RST` (-1 = not wired), `OLED_I2C_ADDR`, `OLED_I2C_HZ`, `OLED_WIDTH`, `OLED_HEIGHT`, `OLED_ROTATION`, `OLED_PERIOD_MS`, `OLED_CONTRAST`, `OLED_IDLE_DIM_MS`, `OLED_IDLE_CONTRAST` |
| Screens, button, clock | `PIN_BUTTON` (-1 = none, 9 = BOOT button), `BUTTON_ACTIVE_LOW`, `BUTTON_DEBOUNCE_MS`, `BUTTON_LONG_PRESS_MS`, `SCREEN_AUTO_RETURN_MS` (0 = stay, the default), `TIME_UTC_OFFSET_MIN` |
| CAN / VESC | `PIN_CAN_TX`, `PIN_CAN_RX`, `CAN_BITRATE_KBPS` (125/250/500/1000), `CAN_LISTEN_ONLY`, `CAN_RX_QUEUE_LEN`, `VESC_CAN_ID` (-1 = any), `VESC_MOTOR_POLES`, `VESC_STALE_R1_MS`, `VESC_STALE_R2_MS`, `CAN_EMA_ALPHA`, `CAN_HEALTH_LOG_MS`, `CAN_LOG_RAW_FRAMES` (also logs the poll request and reply frames) |
| Active VESC polling | `VESC_POLL_MS` (0 = never transmit), `CAN_OWN_ID` (1..254, not a VESC id), `VESC_POLL_TIMEOUT_MS`, `VESC_EXT_STALE_MS`, `CAN_TX_QUEUE_LEN` |
| VESC over UART (`deadman_uart` env, section 3b) | `VESC_LINK_UART` (set by `[env:deadman_uart]`), `PIN_VESC_UART_TX`, `PIN_VESC_UART_RX`, `VESC_UART_BAUD`, `VESC_UART_RX_BUFFER`, `VESC_UART_EMA_ALPHA`, `VESC_UART_RX_WAIT_MS`; `VESC_POLL_MS` / `VESC_POLL_TIMEOUT_MS` / `VESC_POLL_BACKOFF_MS` default to 100 / 50 / 1000 ms there |
| GNSS | `PIN_GNSS_RX`, `PIN_GNSS_TX`, `GNSS_BAUDS`, `GNSS_TARGET_BAUD` (0 = keep the detected baud), `GNSS_RX_BUFFER`, `GNSS_RATE_MS` (200 = 5 Hz, 100 = 10 Hz), `GNSS_DYNMODEL_SEA`, `GNSS_STALE_MS`, `GNSS_REDETECT_MS`, `GNSS_ACK_TIMEOUT_MS`, `GNSS_MAX_SACC_MM_S`, `SPEED_MIN_SHOW`, `SPEED_UNIT_KNOTS` |
| Trip / efficiency | `TRIP_PERIOD_MS`, `EFF_WINDOW_S` (2..120 s), `EFF_MIN_SPEED_MM_S`, `EFF_MIN_DIST_M`, `EFF_UNIT_KM` (1 = Wh/km even with knots), `LOG_TRIP_MS` (0 = off) |
| Dead-man's switch (`deadman` env, section 5) | `DEADMAN_ENABLE`, `DEADMAN_BEACON_ADDR` (mandatory), `DEADMAN_BEACON_ADDR2` ("" = one tag), `DEADMAN_ENROL_MS`, `PIN_DEADMAN_CUT`, `DEADMAN_TIMEOUT_MS`, `DEADMAN_WARN_MS`, `DEADMAN_ARM_REPORTS` / `DEADMAN_ARM_WINDOW_MS` / `DEADMAN_ARM_RSSI`, `DEADMAN_RESET_FRESH_MS`, `DEADMAN_BOOT_CUT`, `DEADMAN_CONFIRM_ENABLE` / `DEADMAN_CONFIRM_MS`, `DEADMAN_SCAN_ITVL_MS` / `DEADMAN_SCAN_WINDOW_MS`, `TASK_PRIO_DEADMAN`, `DEADMAN_TASK_STACK`, `DEADMAN_TICK_MS`, `HB_MAX_DEADMAN_MS`, `LOG_DEADMAN_MS`, `TASK_PRIO_BLE` / `BLE_TASK_STACK` (the `ble` task). Derived: `DEADMAN_UI_ENABLE` |
| Tasks / watchdog / logging | `TASK_PRIO_CAN`, `TASK_PRIO_GNSS`, `TASK_PRIO_DISP`, `WDT_TIMEOUT_MS`, `HB_MAX_CAN_MS`, `HB_MAX_GNSS_MS`, `HB_MAX_DISP_MS`, `LOG_VESC_MS`, `LOG_GNSS_MS`, `LOG_TRIP_MS`, `LOG_SYS_MS`, `SERIAL_BOOT_DELAY_MS`, `PIN_LED`, `FW_VERSION` |
| Advanced (defaults are fine) | `CAN_TASK_STACK`, `CAN_INSTALL_RETRY_MS`, `CAN_RX_TIMEOUT_MS`, `CAN_ERR_LOG_MIN_MS`, `CAN_ERR_WARN_LEVEL`, `CAN_TX_WAIT_MS`, `VESC_POLL_LOG_MIN_MS`, `VESC_POLL_BACKOFF_AFTER`, `VESC_POLL_BACKOFF_MS`, `VESC_GETVALUES_MASK` (0 = plain `COMM_GET_VALUES`), `VESC_RX_BUFFER_SIZE`, `GNSS_TASK_STACK`, `GNSS_AUTOBAUD_LISTEN_MS`, `GNSS_MONVER_TIMEOUT_MS`, `GNSS_AUTOBAUD_RETRY_MS`, `OLED_TASK_STACK`, `OLED_INIT_RETRY_MS`, `OLED_I2C_TIMEOUT_MS`, `OLED_BUTTON_POLL_MS`. Also in `config.h`, with local `#ifndef` fallbacks in the sources so a trimmed copy still builds: `GNSS_BAUD_SWITCH_ATTEMPTS` (2), `GNSS_BAUD_SWITCH_SETTLE_MS` (100), `TASK_PRIO_TRIP` (3), `TRIP_TASK_STACK` (4096), `TRIP_DT_MAX_MS` (5000), `TRIP_COUNTER_RESET_WH` (0.5), `TRIP_EFF_MIN_FILL_S` (3), `TRIP_STALE_MS` (5000). `src/can_vesc.cpp` alone: `CAN_SAMPLE_POINT_PERMILL` (800, the bit sample point) |

Notes: the OLED layout is hard-coded for 128x64 in `OLED_ROTATION` 0 or 2. `OLED_PERIOD_MS` may go down to about 50 (a frame is ~30 ms), but 250 is plenty for EMA-smoothed currents and a 5 Hz speed that is pushed only when a digit changes. `VESC_POLL_MS` can go down to 100..200 if a faster fault indication is wanted (keep `VESC_POLL_TIMEOUT_MS` below it, or accept that a poll is skipped while a reply is outstanding). `WDT_TIMEOUT_MS` (20 s) only has to exceed the longest legitimate task stall (a stuck I2C bus costs `OLED_I2C_TIMEOUT_MS` per transaction). `GNSS_BAUDS` is a brace list and is easiest to change in `config.h` itself; `GNSS_TARGET_BAUD` must stay 0 or one of its members, best the first one (a lost switch verification then costs one autobaud pass). `TRIP_PERIOD_MS` should match `GNSS_RATE_MS`; a longer window (`EFF_WINDOW_S`) steadies `now` at the price of a slower response. The NimBLE settings (observer role only, pool sizes, TX power) are Kconfig, not `config.h`: edit `sdkconfig.defaults.ble`, then delete the dead-man envs' generated `sdkconfig.<env>` so the change takes effect.

## 9. Passivity and safety notes

- With the default `VESC_POLL_MS 1000` the board is no longer silent on the bus: it transmits one 7-byte read request per second under its own id `CAN_OWN_ID` (`candump` on a second adapter shows `0000084A [7] 78 00 32 00 3E C0 3C` for VESC id 74) and the VESC answers with seven frames. The request is the same read-only query VESC Tool uses; nothing in the firmware can write a setting or command the motor (`grep -rn twai_node_transmit src/` finds the single call in `poll_tick()`, under `#if VESC_POLL_MS > 0`). Build with `-DVESC_POLL_MS=0` to get a node that only ACKs and contains no transmit call at all.
- The dead-man's switch (section 5) does **not** change any of this. It cuts the motor through a GPIO wired to the VESC's own kill-switch input, so `grep -rn twai_node_transmit src/` still finds the single call in `poll_tick()` and the bytes on the CAN bus are identical with `DEADMAN_ENABLE` 0 or 1. What does change is that the board now actuates a safety input. It is fail-passive - the pin is open drain, so the firmware can only ever ADD a kill and a dead dashboard leaves the mechanical lanyard in charge - but a hung dashboard silently removes the BLE layer for up to `WDT_TIMEOUT_MS`, and a reboot releases a latched cut. It is not a certified man-overboard device and does not replace the lanyard.
- The UART build (section 3b) transmits exactly one thing: the 6-byte `COMM_GET_VALUES` request every `VESC_POLL_MS`, from `poll_tick()` in `src/uart_vesc.cpp` (the only `uart_write_bytes` call there). It is read-only and does not reset the VESC's control timeout, and the TWAI driver is not linked at all.
- The radio of the dead-man builds (section 5) only listens: NimBLE runs in the observer role with a passive scan, so the board never advertises, never connects and transmits nothing over BLE (a passive scan sends no scan requests). It stores no bonding: after the first controller enable (PHY calibration blob, written before the CAN node exists) nothing is written to flash.
- The polled averages (Iavg, id/iq, vd/vq) are "since the previous read": running VESC Tool's realtime page at the same time shortens their window on both sides. Everything else is unaffected by a second reader.
- With the VESC powered off, each unanswered request costs the controller 8 error points until it goes error-passive (~16 polls); the log then shows `CAN: error passive` and the poll warnings every 10 s and the request rate drops to every 5 s. No bus-off, and it recovers by itself when the VESC is back.
- Displayed and logged power is signed: `v_in x current_in`, negative while regenerating (the water drives the motor); VESC Tool's realtime power stat is an absolute value, so the two can differ in sign, not in magnitude. The cells are EMA-smoothed (`CAN_EMA_ALPHA`) and may be up to `VESC_STALE_R1_MS` old; a fault shows on the VESC 1/3 page only after the next poll (up to `VESC_POLL_MS` later) and only as long as the VESC keeps it or via the latch. With `DEADMAN_ENABLE` 0 (the default) this is a dashboard, not a protection device: rely on the VESC's own limits and a BMS for that. `DEADMAN_ENABLE` 1 adds one GPIO that asserts a kill the VESC already knows how to act on; it still cannot command the motor, change a limit or write a setting.
- OLED modules are specified for roughly -40..70 C (vendor dependent; some 1.54" boards only -20..60 C), but the glass, the FPC and the boost converter must stay dry: condensation on a boat needs an enclosure or conformal coating. In direct sun a ~110 cd/m2 OLED is hard to read; a sunshade helps more than contrast.
- Burn-in is the OLED's wear mechanism: a dashboard that shows the same numbers for hours is the worst case. Keep `OLED_IDLE_DIM_MS` enabled and expect the panel to lose brightness over thousands of hours; the detail screens' inverted title bar is the only filled area, and a long press (or `SCREEN_AUTO_RETURN_MS`, off by default) brings the sparse main screen back.
- Over-the-air update (section 7) is the only time the board transmits over a radio: a WPA2 access point (fresh 8-digit passphrase per session, one client, 13 dBm) and a plain-HTTP upload page, for at most `OTA_IDLE_TIMEOUT_MS` without an upload. It runs in a boot of its own with nothing else of the firmware: no CAN node (the VESC sees no ACK from the board, exactly as with the dashboard switched off), no UART poll, no BLE, and in a dead-man build the cut held for the whole time. Anyone within Wi-Fi range who can read the OLED can flash firmware during that window; nobody else can. It is refused while the motor turns or the boat moves.
- A hung task reboots the board after `WDT_TIMEOUT_MS` (20 s); the next banner reports `reset=TASK_WDT`. The OLED keeps its last image until the firmware pulses RES about 1.5 s into the reboot, then shows the boot frame.
- Apart from the CAN transceiver everything is 3.3 V. Nothing on the header tolerates 5 V, and neither does an OLED module whose pull-ups would then sit at 5 V, nor a button wired to anything but GND or 3V3.

## 10. Future work (out of scope for this version)

- `CAN_PACKET_PING` / `PONG` discovery so polling can start on a VESC that broadcasts no status frames without pinning `VESC_CAN_ID`, and answering pings so the board shows up in VESC Tool's CAN scan.
- Burn-in mitigation beyond dimming: `DISPLAYOFF` (0xAE) after a long idle, or shifting the layout by a pixel every few minutes.
- Deep sleep and the LP-UART for the GNSS receiver.
- Persisting the trip (distance, energy) across reboots and logging a track; today the trip restarts at every boot and nothing is written to flash.
- `CONFIG_TWAI_ISR_IN_IRAM` (with `IRAM_ATTR` on the callbacks in `src/can_vesc.cpp`) if regular flash writes (trip persistence) are ever added, so CAN reception continues while the flash cache is disabled. Today the only flash write in normal operation is the one otadata sector that confirms a new image, once, a minute after an update (a few CAN frames may be dropped then); the upload itself happens in update mode, where no CAN node exists.
