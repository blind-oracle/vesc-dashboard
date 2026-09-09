# boat-motor

ESP32-C6 firmware that listens to a VESC motor controller on CAN, polls it once a second for the few values its broadcasts lack (fault code, per-MOSFET temperatures, id/iq, vd/vq, absolute tachometer), reads ground speed and time from a u-blox GNSS receiver at 5 Hz, shows everything on a 128x64 monochrome OLED (SSD1309 controller, I2C) as seven screens cycled by a push button, and integrates speed and energy into a trip log with the boat's efficiency in Wh per nautical mile (or km). Built for the DFRobot FireBeetle 2 ESP32-C6 (DFR1075) with an MCP2551-class CAN transceiver, any 2.42" or 1.54" SSD1309 module switched to I2C mode and any receiver that speaks UBX (u-blox 6/7, M8, M9, M10 or a compatible clone). On the CAN bus the firmware acknowledges frames and transmits exactly one thing: a 7-byte `COMM_GET_VALUES_SELECTIVE` request every `VESC_POLL_MS` (1 s). It never sends a command or a setting; `-DVESC_POLL_MS=0` makes it strictly passive (ACK only). Optionally (`pio run -e bms`) it also reads a JK (Jikong) BMS over Bluetooth LE with the C6's own radio and adds a BMS page and two CELLS pages (ten screens); the default build contains no BLE code.

## 1. What it does

Five FreeRTOS tasks on the single core (tick 1 ms; the `bms` env adds `bmsTask` and the two NimBLE tasks). Priorities guarantee that CAN reception is never starved by a frame push or by a stuck I2C bus (50 ms timeout per transaction).

| Task | Priority | Stack | Job |
|---|---|---|---|
| `canTask` (`src/can_vesc.cpp`) | `TASK_PRIO_CAN` = 6 | 4096 | take frames from the TWAI ISR queue, keep VESC STATUS 1..6 frames, decode, smooth (EMA), write `g_state.vesc`; every `VESC_POLL_MS` send one `COMM_GET_VALUES_SELECTIVE` request, reassemble and CRC-check the reply into `g_state.vesc_ext`, latch the last fault; bus-off recovery and health counters |
| `gnssTask` (`src/gnss_ubx.cpp`) | `TASK_PRIO_GNSS` = 5 | 4096 | AUTOBAUD -> DETECT (MON-VER) -> BAUD (move the receiver's UART to `GNSS_TARGET_BAUD`) -> CONFIGURE (`GNSS_RATE_MS`, NAV-PVT on, NMEA off) -> RUN; parses NAV-PVT (position, speed, accuracies, heading, UTC date and time) into `g_state.gnss`; back to autobaud after `GNSS_REDETECT_MS` of silence or when the receiver does not answer at the new baud |
| `displayTask` (`src/display_oled.cpp`) | `TASK_PRIO_DISP` = 2 | 6144 (`OLED_TASK_STACK`) | loop every `OLED_BUTTON_POLL_MS` (20 ms): debounce the button, switch screens; every `OLED_PERIOD_MS` (or at once after a screen change) build the frame for the current screen from a snapshot and push it only when something changed; dim after `OLED_IDLE_DIM_MS` idle; re-probe the controller every 5 s and re-initialise a panel that stopped answering |
| `tripTask` (`src/trip.cpp`) | `TASK_PRIO_TRIP` = 3 | 4096 (`TRIP_TASK_STACK`) | every `TRIP_PERIOD_MS` (200 ms): snapshot, integrate GNSS ground speed into distance and the VESC watt-hour counters (or `v_in x current_in`) into energy, keep the `EFF_WINDOW_S` ring, publish `g_state.trip` (section "Efficiency and trip"); a pure consumer that owns no peripheral and is not watched by the watchdog |
| `bmsTask` (`src/bms_ble.cpp`, `bms` env only) | `BMS_TASK_PRIO` = 4 | 4096 (`BMS_TASK_STACK`) | NimBLE central for a JK BMS (section 5): scan for the BMS (name prefix or service 0xFFE0, or a pinned MAC), connect, exchange the MTU, discover 0xFFE0 / 0xFFE1, subscribe, send 0x97 (device info) and 0x96 (cell info); the BLE callbacks only copy notification bytes into a FreeRTOS message buffer (`BMS_MSG_BUF_BYTES`), the task reassembles the 300-byte frames, checks and decodes them (`include/jk_bms.h`), detects the 24S / 32S layout and publishes `g_state.bms`; reconnect back-off and scan presets. The NimBLE host and controller tasks (priorities 21 / 23) run their own short slices |
| supervisor loop in `app_main()` (`src/main.cpp`) | 1 | 8192 (`CONFIG_ESP_MAIN_TASK_STACK_SIZE`) | sole Task-WDT subscriber (`WDT_TIMEOUT_MS`), fed only while the CAN, GNSS and display heartbeats are fresh (the trip task has none; the BMS heartbeat only gates it with `HB_MAX_BMS_MS` > 0); 1 Hz VESC and GNSS log lines, 5 s TRIP and BMS lines (`LOG_TRIP_MS`, `LOG_BMS_MS`), 10 s SYS line with the display, button, poll and BMS link counters; LED |

Data flow:

```
 VESC ---CAN---> transceiver -> TWAI ISR -> RX queue (64 frames) -> canTask --+
      <--------- 1 request frame / s (single shot) <--------------- canTask   +--> g_state (one mutex) --> displayTask -> u8g2 (fonts, frame buffer) -> I2C master -> OLED
 GNSS ---UART--> UART1 ISR -> 2048 B ring -------------------------> gnssTask -+       |    ^             \-> app_main(): logs, watchdog, LED
      <--------- UBX configuration: 115200 baud, 5 Hz NAV-PVT <---- gnssTask   |       +----+-- tripTask (200 ms snapshots): speed x dt -> distance, Wh counters -> energy -> g_state.trip
 JK BMS ~~BLE~~> NimBLE host -> message buffer (2048 B) ----------> bmsTask --+   (bms env only)
        <~~~~~~~ 0x97 device info, 0x96 cell info, CCCD 01 00 <~~~ bmsTask    |
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
| `include/shared_state.h`, `src/shared_state.cpp` | `g_state` (`vesc`, `vesc_ext`, `can`, `gnss`, `trip`, `disp`, plus `bms` with `BMS_UI_ENABLE`), mutex, snapshots, heartbeats, `fresh()` |
| `include/can_vesc.h`, `src/can_vesc.cpp` | TWAI node setup (`esp_twai_onchip.h`), ISR callbacks and RX queue, canTask, poll state machine, bus-off recovery |
| `include/gnss_ubx.h`, `src/gnss_ubx.cpp` | UART1 setup, autobaud, MON-VER detection, baud switch, VALSET/legacy configuration |
| `include/trip.h`, `src/trip.cpp` | header-only trip integrator (distance, energy sources, `EFF_WINDOW_S` window, efficiency; framework-free, unit-tested) and the 200 ms task that feeds it snapshots and publishes `g_state.trip` |
| `include/display.h` | `display_start()`, the display task's one entry point (implemented by `src/display_oled.cpp`) |
| `include/display_strings_oled.h` | pure builders of all OLED screens (seven, or ten with the BMS and CELLS pages of a `bms` build; character budgets, stale -> `--`, UTC -> local clock), unit-tested |
| `src/display_oled.cpp` | I2C bring-up and presence probe, u8g2 rendering, button debounce, screen switching, change-detected refresh, idle dimming |
| `include/u8g2_hal_idf.h`, `src/u8g2_hal_idf.cpp` | u8g2 HAL for ESP-IDF: I2C master byte callback, reset GPIO, delays, address probe |
| `components/u8g2/` | u8g2 library (git submodule, tag 2.37.1): fonts, frame buffer, SSD1309 init sequence; only the referenced fonts are linked |
| `include/jk_bms.h` | header-only JK BMS BLE codec: 20-byte command builder, 300-byte frame assembler (notification chunks -> checksum-verified frames), cell-info and device-info decoders, JK02_24S / JK02_32S layout detection, alarm bit names (framework-free, unit-tested) |
| `include/bms_ble.h`, `src/bms_ble.cpp` | NimBLE central for the JK BMS (the only file that includes NimBLE): scan, connect, subscribe, bmsTask, link state machine, BMS log line; inline no-ops when `BMS_BLE_ENABLE` is 0 |
| `src/main.cpp` | `app_main()`: start-up and the supervisor loop |
| `platformio.ini`, `boards/dfrobot_firebeetle2_esp32c6.json`, `sdkconfig.defaults`, `sdkconfig.defaults.bms`, `partitions.csv`, `CMakeLists.txt`, `src/CMakeLists.txt` | build: pinned platform and envs, board definition, ESP-IDF configuration (the `.bms` fragment adds the NimBLE host for the `bms` and `demo` envs), partition table, IDF project files |
| `test/test_vesc/`, `test/test_vesc_getvalues/`, `test/test_ubx/`, `test/test_trip/`, `test/test_jk_bms/`, `test/test_display_strings_oled/` | Unity tests (one `test_main.cpp` each, 183 test cases; `test/test_jk_bms/jk_frames.h` holds recorded BMS frames), run on the host with `pio test -e native` |

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

### JK BMS: nothing to wire

The optional BMS link (`pio run -e bms`, section 5) uses the ESP32-C6's own Bluetooth LE radio to talk to the BLE module of a JK (Jikong) BMS: no pin, cable or transceiver is involved, and the pin tables above are complete for a `bms` build too. The range is a few metres, so the BMS must sit within reach of the dashboard.

### Pins to leave alone

| GPIO | Why |
|---|---|
| 12, 13 | USB D-/D+: the USB-Serial/JTAG console (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`). `config.h` refuses to build if a peripheral uses them |
| 16, 17 | UART0, the ROM/IDF boot console. GPIO16 is driven by UART0 TX for the whole run |
| 9 | BOOT button, boot-mode strapping. Usable as the screen button (`-DPIN_BUTTON=9`) because the level is only latched at reset |
| 0 | battery ADC, not on the header |
| 8, 15 | strapping pins (boot mode, JTAG select). MCU-driven outputs are fine (the on-board LED is on 15); never a peripheral-driven input |
| 4, 5 | strapping only for the SDIO edge, harmless; used for GNSS |

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

## 5. BMS over Bluetooth LE (JK BMS)

Optional: `pio run -e bms -t upload` builds the dashboard with a Bluetooth LE client for a JK (Jikong) BMS and adds the BMS and CELLS screens (section 6). The default env contains no BLE code at all (its generated `sdkconfig` has no `CONFIG_BT`; the feature cost it 64 bytes of log strings, 292,480 -> 292,544 B). `[env:bms]` in `platformio.ini` adds `-DBMS_BLE_ENABLE=1` and a second ESP-IDF fragment, `board_build.cmake_extra_args = -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.bms"`, which turns on the NimBLE host in the central and observer roles only: no advertising, no GATT server, no security or bonding, one connection, preferred MTU 256, BLE 4.2 feature set, Wi-Fi coexistence off. PlatformIO re-runs CMake only when `sdkconfig.defaults` or `sdkconfig.<env>` changed, so after editing the fragment run `touch sdkconfig.defaults` (or delete `sdkconfig.bms`). `[env:demo]` extends `bms`, so the layout demo shows the BMS pages with synthetic data. To make BLE the default, change `default_envs` in `platformio.ini`. Measured cost (init-only build, 2026-09-09): `firmware.bin` 641,664 B against 292,480 B (+349 KB of flash, in a 1,310,720 B OTA slot) and 21,884 against 18,132 B of static RAM (+3.7 KB); the heap the NimBLE pools take (expected 50..80 KB with the defaults) is logged at boot (`bms: BLE init ok, heap N -> N`) and in every BMS and SYS line, and optional pool trims are commented in `sdkconfig.defaults.bms`. The protocol details come from syssi/esphome-jk-bms (https://github.com/syssi/esphome-jk-bms), the reference implementation `include/jk_bms.h` was verified against.

- No wiring: the ESP32-C6's own radio talks to the BMS's BLE module (GATT service 0xFFE0, characteristic 0xFFE1). The range is a few metres; the BMS must be within reach of the dashboard.
- One central only: a JK BMS accepts exactly one BLE connection. While the JK phone app is connected the board cannot connect (it keeps retrying with back-off and the BMS screen shows `SCAN ...`, after a minute `app open?`), and while the board is connected the app cannot. Force-stop the app, or switch Bluetooth off on the phone, before expecting the board to connect; on many BMS modules a blinking red LED means "no central connected".
- Finding the BMS: with `BMS_BLE_ADDR ""` (default) the board scans actively and connects to the first device whose name starts with `BMS_BLE_NAME_PREFIX` (`JK`) or that advertises 0xFFE0; the log prints the match (`bms: match c8:47:8c:12:34:56 rssi -61 name JK-B2A24S20P`). Pin it for your boat with `-DBMS_BLE_ADDR='"C8:47:8C:12:34:56"'` (a build flag in `[env:bms]` or your own env, or a `#define` in `config.h`): passive scan, no dependence on the name (which is editable in the app), and a phone with the same name prefix cannot be mistaken for the BMS.
- Protocol: a 20-byte command written to 0xFFE1 (`AA 55 90 EB`, register, length, value, padding, sum8 checksum) and 300-byte answers (`55 AA EB 90`, type byte, payload, sum8 at index 299) delivered as notifications of whatever size the negotiated MTU allows (20, 128+128+44 and 150+150 have all been seen); the assembler in `include/jk_bms.h` never trusts notification boundaries, scans for the preamble at any offset and survives the ACK echoes and the 4-byte `AT\r\n` notifications of JK-PB modules. Two cell-info layouts exist and no field says which: JK02_24S (BMS software 6.x .. 10.x) and JK02_32S (11.x and newer, all JK-PB). `BMS_PROTOCOL 0` (default) auto-detects: a guess from the device-info frame (software major >= 11 or model `JK_PB` / `JK-PB` -> 32S, else 24S) that the first cell-info frame must confirm (the sum of the cells within 2 % of the pack voltage) before anything is shown; if neither layout is plausible the screen shows `CONNECTED  layout ?` and the log prints model and software version once. `BMS_PROTOCOL 1` / `2` forces 24S / 32S. JK04 firmware (3.x, float frames; e.g. JK-B2A16S sw 3.3.0 or JK-B5A24S sw 8.0.3M) is not supported.
- Data, from the type 0x02 cell-info frame the BMS streams by itself (about 2 Hz): pack voltage, current, power, SoC, remaining and nominal capacity, cycle count, T1 / T2 and MOSFET temperatures, lowest and highest cell with their index, cell delta, charge and discharge MOSFET state, balance current, the alarm bitmask, and the per-cell voltages. `BMS_CELLS_MAX` (24; 1..32 allowed) cells are kept in `g_state.bms` and shown on ceil(n/12) CELLS pages (`BMS_CELL_PAGES`; 24 -> two pages of 12). Min, max and delta are computed from the non-zero cells; power = pack voltage x current.
- Sign: the JK app shows charging as positive. `BMS_CURRENT_SIGN 1` (default) shows discharge as positive, the same sign as the `BA` / `PW` cells on the main screen, so a running motor reads `Ibat 12.4` and a charger `Ibat -6.0`; `BMS_CURRENT_SIGN 0` keeps the app's convention. The balance current keeps the JK sign.
- Tasks: the NimBLE controller and host tasks (ESP-IDF, priorities 23 / 21) run their own short slices; all BMS logic runs in `bmsTask` (`BMS_TASK_PRIO` 4, between trip and GNSS, `BMS_TASK_STACK` 4 KB). The BLE callbacks only copy notification bytes into a FreeRTOS message buffer (`BMS_MSG_BUF_BYTES`, 2048); the task assembles, decodes and publishes one `BmsState` copy under the shared-state mutex like every other producer. Its heartbeat is logged but by default does not gate the task watchdog (`HB_MAX_BMS_MS 0`): a missing BMS or a stuck link must never reboot the dashboard. `bms_ble_start()` runs before `can_vesc_start()` because the first controller enable may write the PHY calibration blob to NVS and the TWAI ISR is not cache-safe; after that the link does no flash writes (no bonding, `CONFIG_BT_NIMBLE_NVS_PERSIST=n`). The board never asks for connection-parameter changes.
- Written to the BMS, ever: register 0x97 (device info) and 0x96 (start the cell-info stream), plus the CCCD subscription (`01 00`) that enables notifications. No BMS setting can be changed by this firmware (section 9).
- Log (tag `bms`): `cfg BMS : ble=1 addr=(scan) name=JK* proto=0 cells<=24 sign=dis+ stale=10000 ms prio=4` at boot (`cfg BMS : ble=0 (build -e bms to enable)` in the default build), `started: ... bms=1`, `BMS link=STREAM since=123s frames ok=246 bad=0 heap=...` every `LOG_BMS_MS` (5 s; 0 = off), and the SYS line ends in `| bms=<OFF|SCAN|CONN|SETUP|STREAM> frm=<frames> hb=<heartbeat age ms>`. Transitions (match, connecting, connected, mtu, handles, device model / hw / sw, layout, disconnect reason, retry) are logged at INFO.
- Host tests: `test/test_jk_bms` (34 cases: command bytes, checksum, the assembler under every observed chunking including ACK echoes and JK-PB `AT\r\n` noise, decoding of recorded 24S / 32S / 13S frames from the syssi/esphome-jk-bms dumps in `jk_frames.h`, layout detection, device info, alarm names); the display suite gained the BMS and CELLS screen tests.

Link handling (`config.h`):

| Define | Default | Effect |
|---|---|---|
| `BMS_SCAN_FAST_MS` | 30000 | fast scan preset (`BMS_SCAN_FAST_ITVL_MS` / `BMS_SCAN_FAST_WINDOW_MS` = 60 / 30 ms, about 50 % radio duty) after boot and after every disconnect; then the slow preset (`BMS_SCAN_SLOW_ITVL_MS` / `BMS_SCAN_SLOW_WINDOW_MS` = 1000 / 30 ms, about 3 %) while the BMS is absent. With `BMS_BLE_ADDR` set the scan is passive |
| `BMS_CONNECT_TIMEOUT_MS` | 10000 | a connect request without an answer (phone app connected, out of range) is abandoned after this |
| `BMS_SETUP_TIMEOUT_MS` | 8000 | MTU exchange, service / characteristic / descriptor discovery and the CCCD write must finish within this |
| `BMS_FIRST_FRAME_TIMEOUT_MS` | 15000 | the first checksum-valid, plausible cell-info frame must arrive within this after subscribing; the 0x96 request is re-sent every `BMS_POLL_MS` (5000) until the stream starts |
| `BMS_RECONNECT_MS`, `BMS_RECONNECT_MAX_MS` | 2000, 30000 | back-off after a disconnect or a failed attempt, doubling from the first value to the second |
| `BMS_RECONNECT_FAILS` | 3 | direct reconnects to the last known address before the task scans again |
| `BMS_STALE_MS` | 10000 | no decoded cell-info frame for this long: the BMS screens show `--` and `NO DATA`, and the link is recycled |
| `BMS_APP_HINT_S` | 60 | seconds of scanning without ever having seen the BMS before the screen adds `app open?` |
| `HB_MAX_BMS_MS` | 0 | 0 = the bmsTask heartbeat is only logged (`hb=` in the SYS line); > 0 = it gates the task watchdog like the other tasks |

## 6. Display behaviour

128 x 64, white on black, `OLED_ROTATION 0`. Text is drawn by u8g2 (the C library in `components/u8g2`, full frame buffer, SSD1309 `noname0` driver, HAL in `src/u8g2_hal_idf.cpp`): `u8g2_font_logisoso24_tn` for the speed (24 px digits, tabular 15 px advance; the font has digits, `-`, `.` and `:` only), `u8g2_font_6x10_tf` for all other text (6 px monospace, 7 px caps, has the degree sign) and `u8g2_font_6x13B_tf` (bold, same 6 px advance) for the title bars. Only the three referenced fonts are linked (~4.7 KB of flash); the rest of the U8g2 font file costs nothing. A row of small text is 21 characters wide.

Seven screens, cycled by the button (section "Button" below); the detail screens carry their page number `n/7` in the title bar. A `bms` build (section 5) inserts the BMS page and two CELLS pages between VESC 3/3 and GNSS: ten screens, page numbers `n/10`. All value fields are fixed-width cells; `--` means stale or never received.

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

With the BMS pages (`bms` build; a 16S pack discharging, GNSS and SYS become 9/10 and 10/10):

```
 BMS (6/10, bms build)        CELLS 1/2 (7/10)             CELLS 2/2 (8/10)
 +---------------------+      +---------------------+      +---------------------+
 |[BMS 16S        6/10]|      |[CELLS 1/2 16S  7/10]|      |[CELLS 2/2 16S  8/10]|
 |Vbat 53.56 Ibat 12.4 |      | 1 3.348    7 3.348  |      |13 3.348   19 --     |
 |P 664W     SOC 87%   |      | 2 3.348    8 3.348  |      |14 3.352H  20 --     |
 |Ah 269.4/310 cyc 17  |      | 3 3.349    9 3.347L |      |15 3.349   21 --     |
 |T 6/6      MOS 2.5   |      | 4 3.348   10 3.348  |      |16 3.348   22 --     |
 |lo 9:3.347 hi14:3.352|      | 5 3.348   11 3.349  |      |17 --      23 --     |
 |d 5mV CD bal 0.00A   |      | 6 3.348   12 3.348  |      |18 --      24 --     |
 +---------------------+      +---------------------+      +---------------------+
```

- **MAIN.** Zone A (rows 0..23): ground speed right-aligned in the big font (max 4 characters), then a 12-character column at x 56 with three lines: the local clock `HH:MM` (or `--:--`) followed by the fix field, the CAN line, and the speed unit (`kn` / `km/h`). A 1-px rule at row 26, then Zone B: four rows of two 64-px cells, label left, value right-aligned (up to 6 characters). Fix field: `3D 9sv` / `3D12sv` / `2D 7sv` (fix type, satellites used), `NOFIX` (receiver alive, no 2D/3D fix), `NODATA` (no epoch for `GNSS_STALE_MS`), `NOGNSS` (nothing received since boot), `BOOT` on the first frame. CAN line: `VESC 74` (controller running, STATUS_1 or STATUS_5 within `VESC_STALE_R1_MS`), `CAN idle` (running, no fresh frame: VESC off or status messages not enabled), `CAN RECV` (bus-off recovery), `CAN OFF` (STOPPED / BUS_OFF / driver not installed); the boot frame shows `FW_VERSION` here.
- **Cell glossary.** `BV` battery voltage (STATUS_5, one decimal, `48.2v`); `BA` battery current (STATUS_4, signed, one decimal while it fits: `12.4A`, `-12.4A`, `-123A`); `MA` motor current (STATUS_1, whole amps: `35A`, `-1000A`); `PW` electrical power = voltage x battery current, signed, only when both are fresh (`598W`, `-4800W`, `12.3kW` from 10 kW); `TF` MOSFET temperature (STATUS_4, whole degrees, `--` outside -40..200 C); `TM` motor temperature (`n/a` when the VESC reports an implausible value because no motor NTC is wired); `RPM` mechanical rpm = erpm / (`VESC_MOTOR_POLES` / 2) (`2350`, `12.3k` from 10000); `WH` watt-hours drawn since the VESC booted (STATUS_3: one decimal below 1000 `605.7`, whole below 10000 `1235`, then `12.3k`, `123.5k`).
- **EFFICIENCY** (from `g_state.trip`, section "Efficiency and trip"). `now` = net Wh per distance unit over the last `EFF_WINDOW_S` (10 s) and `avg` since boot, one decimal below 100 (`98.5`, `-12.3` while regenerating), whole above (`123`); the unit is `Wh/NM` on a knots build and `Wh/km` on a km/h build or with `EFF_UNIT_KM 1`. `dist` trip distance in that unit (two decimals while they fit: `6.10`, `123.4`, `1235`) and net trip energy (`601Wh`, `-250Wh`, `1.23kWh`, `12.35kWh`, `1.23MWh`); `P` average electrical power over the window (`598W`, `-480W`, `12.3kW`) and `win` the seconds of data in it (0..10); `run` integrator time since boot and `mov` time above `EFF_MIN_SPEED_MM_S` (`12m34s`, `1h23m`, `12d03h`, whole days from 100 d); `mean` speed while moving (distance / moving time, in the speed unit) and `src`: `S3` = energy from the VESC watt-hour counters, `PxI` = the `v_in x current_in` fallback. `--` rules: `now --` until the window holds `TRIP_EFF_MIN_FILL_S` (3) closed seconds and `EFF_MIN_DIST_M` (10 m) of distance with no gap (GNSS unusable, no energy source, counter reset or a stalled tick inside the window); `avg --` below `EFF_MIN_DIST_M` of trip distance; `P --` before the first window second closes; `mean --` while nothing has moved; the whole screen `--` when the integrator has not stamped `trip.t_ms` for `TRIP_STALE_MS` (5 s) or never ran.
- **VESC 1/3.** Row 1 is the fault row: `FAULT <name>` while the live fault byte of a fresh poll reply is non-zero, `LAST <name> <age>` when a fault was latched earlier (`34s`, `12m`, `3h`, `2d`), `FAULT none` (fresh reply, no fault, nothing latched), `FAULT --` (polled values stale, or `VESC_POLL_MS 0`). Then input voltage / battery current (one decimal), motor current / duty %, power / rpm, FET and motor temperature with one decimal (raw, no plausibility filter), Ah / Wh drawn.
- **VESC 2/3** (polled values, `VESC_EXT_STALE_MS` window): `Tmos`, the first MOSFET temperature (the VESC reports three; MOSFET 2 and 3 stay in the poll and the log but left the screen) with `Iin`, the VESC's own averaged input current (`Tmos --    Iin --` when stale), average d/q currents, d/q voltages, tachometer (STATUS_5) / absolute tachometer (polled), status `St OK` / `TIMEOUT` (no control input, timeout brake active) / `KILLSW` / `TO+KILL` and the controller id from the reply, charged Ah / Wh.
- **VESC 3/3**: PPM input / PID position, `ADC` 1/2/3 volts (`ADC --` when STATUS_6 is stale), poll counters `poll` (requests sent) / `ok` (complete replies) / `bad` (CRC or format failures) / `tmo` (reply timeouts) - `poll off   ok --` and `bad --     tmo --` in a `VESC_POLL_MS 0` build -, `frm` accepted status frames / `oth` frames from another controller id, `misc` non-status or odd-DLC frames / `busoff` events.
- **BMS** (`bms` build, `g_state.bms`, section 5; the title carries the pack's cell count once a frame was decoded, `BMS 16S`). Live only while the link is streaming and the last decoded frame is within `BMS_STALE_MS` (10 s): `Vbat` pack voltage (two decimals) / `Ibat` pack current (one decimal, discharge-positive with `BMS_CURRENT_SIGN 1`); `P` pack power (same sign, `kW` from 10 kW) / `SOC` in %; `Ah` remaining / nominal capacity and `cyc` cycle count; `T` T1/T2 in whole degrees / `MOS` MOSFET temperature with one decimal (`--` outside -40..200 C, like the VESC temperatures); `lo` / `hi` lowest and highest cell as index:volts (`lo --` without an index); row 5: `d` cell delta in mV, `C` / `D` for charge / discharge MOSFET on (`-` = off), then `bal` balance current in A or, while any alarm bit is set, `ALM <name>` of the lowest set bit with `+` appended when there are more (`d 5mV CD ALM CELLUV+`; `mV` is dropped if the row would exceed 21 characters). The 32 alarm names, bit 0 first (the 24S layout carries bits 0..15 only): `WIRE_R MOS_OT CELLNO BIT3 FULL PACKOV CHG_OC CHG_SC CHG_OT CHG_UT COPROC CELLUV PACKUV DIS_OC DIS_SC DIS_OT CMOSAB DMOSAB GPS PASSWD DISON BAT_OT TSENS PLMOD SCPREL DOCP2 DOCP3 DIS_UT GPSLCK BIT29 BIT30 BIT31`. While not streaming, row 0 is a status and rows 1..5 keep their labels with `--`: `BLE OFF    no BMS` (BLE not started or NimBLE init failed), `SCAN 34s` / `SCAN 34s   last 2m` (scanning for 34 s; `last` = age of the last decoded frame, omitted before the first one), `SCAN 1m    app open?` (scanning for `BMS_APP_HINT_S` = 60 s without ever seeing the BMS: the phone app probably holds its single connection), `CONNECTING last 2m`, `SETUP      last 2m` (connected: MTU exchange, discovery and subscription pending), `CONNECTED  layout ?` (frames arrive but neither cell layout is plausible), `NO DATA    last 12s` (streaming, but no frame for more than `BMS_STALE_MS`), `WAIT DATA` (subscribed, first frame pending).
- **CELLS 1/2, 2/2** (title `CELLS 1/2 16S`): 12 cells per page in two columns, column-major (left 1..6, right 7..12), each slot `index volts` with `L` after the pack's lowest cell and `H` after its highest, `--` for a cell the pack does not have, one beyond `BMS_CELLS_MAX`, or every cell while not streaming. Page 2 shows 13..24 (`14 3.352H`, `17 --` .. `24 --` for a 16S pack); `BMS_CELLS_MAX` 12 or less gives a single `CELLS` page, 25..32 a third.
- **GNSS**: fix and satellites (`3D 9sv`, `2D 12sv`, `TIME ONLY`, `NO FIX`, `NO DATA`, `NO GNSS`, or the phase `AUTOBAUD` / `DETECT` / `CONFIG` while nothing has been received) with pDOP; latitude/longitude with five decimals (`Pos --` without a usable fix); height above sea level / horizontal accuracy; speed (same rules as the main screen) / speed accuracy in m/s; heading of motion (only while moving) / vertical accuracy; UTC date-time (`UTC --` when unknown).
- **SYS `FW_VERSION`**: uptime (`12m34s`, `1h23m`, `12d03h`) / debounced button presses, free heap / minimum free heap in kB, TWAI state (`UNINST` / `STOP` / `RUN` / `BUSOFF` / `RECOV`) with the TEC and REC error counters, GNSS phase, baud and PROTVER, good/bad UBX frames and redetects, reset reason / `state_lock()` timeouts.
- **Formatting rules.** A number that does not fit its cell drops decimals one by one (`48.2` -> `100`), then switches to thousands (`12.3k`, `12k`), millions (`12M`) and finally `MAX` / `-MAX`; a clipped-but-plausible wrong number is never shown. Counters: `12345`, `1234k`, `123M`, `4G`. `-0.0` is printed as `0.0` so a current hovering around zero does not flicker. Speed: `0.0` below `SPEED_MIN_SHOW`, one decimal below 100, whole above, `--` without a usable fix or with a speed-accuracy estimate above `GNSS_MAX_SACC_MM_S`. Freshness windows: STATUS 1/4/5 `VESC_STALE_R1_MS`, STATUS 2/3/6 `VESC_STALE_R2_MS`, polled values `VESC_EXT_STALE_MS`, GNSS `GNSS_STALE_MS`, trip integrator `TRIP_STALE_MS`, BMS frames `BMS_STALE_MS`; anything older, or never received, is `--`.

### Button

The display task samples `PIN_BUTTON` every `OLED_BUTTON_POLL_MS` (20 ms) and accepts a level once it has been stable for `BUTTON_DEBOUNCE_MS` (30 ms). Releasing before `BUTTON_LONG_PRESS_MS` (1500 ms) is a short press: next screen, wrapping from SYS back to MAIN. Holding for `BUTTON_LONG_PRESS_MS` is a long press: back to MAIN, fired once while the button is still held (the release then does nothing). `SCREEN_AUTO_RETURN_MS` (60000) after the last press on any other screen the display returns to MAIN by itself (0 = stay). Every press counts (`btn` on the SYS screen, `btn=` in the SYS log line), restores the contrast if the panel was dimmed and restarts the idle timer; a screen change is drawn immediately, not at the next tick, and logged as `display: screen 1 (EFF), short press` (screen indices 0..6 = MAIN, EFF, VESC 1/3, VESC 2/3, VESC 3/3, GNSS, SYS, or 0..9 = MAIN, EFF, VESC 1/3, VESC 2/3, VESC 3/3, BMS, CELLS 1/2, CELLS 2/2, GNSS, SYS in a `bms` build; reasons: `short press`, `long press`, `auto-return`). The button works even while no panel answers.

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

The panel is probed with a zero-length I2C transaction before every init, so a missing display is reported instead of drawn into blindly: `display_start()` logs `OLED: no response at 0x3C (check wiring/address jumper): ESP_ERR_NOT_FOUND` and the task retries every 5 s (`OLED: still no response at 0x3C (retry N, ESP_ERR_NOT_FOUND)`); while running it re-probes every 5 s and re-initialises a panel that stopped answering (`OLED: lost contact at 0x3C (ESP_ERR_NOT_FOUND), re-initialising`), as it does at once when a frame transfer fails (`OLED: frame transfer failed (ESP_ERR_INVALID_RESPONSE), re-initialising`), so hot-plugging works in both directions. After every (re)init the panel shows the boot frame (main screen, `--` everywhere, `--:--`, `BOOT`, firmware version) and the live frame on the next tick. `-DDISPLAY_DEMO=1` (the `demo` env, which extends `bms`) feeds synthetic changing values to all ten screens (the EFFICIENCY page runs a synthetic trip at 3 m/s and ~600 W; the BMS and CELLS pages show a 16S pack discharging 12.4 A with cell 9 the lowest, cell 14 the highest and a `CELLUV` alarm for 10 s per cycle) with a 30 s hold every 2 min, a live `OT_FET` fault for 10 s per cycle and the latched `LAST OT_FET <age>` row afterwards, so you can check every layout and the idle dimming with no VESC or GNSS attached (`-DOLED_IDLE_DIM_MS=20000` to see the dimming sooner).

## 7. Building, flashing, testing

Prerequisites: PlatformIO Core (`pio` on the PATH, e.g. `~/.platformio/penv/bin/pio`), git (u8g2 is a submodule) and a USB-C cable to the FireBeetle's USB port. The firmware is native ESP-IDF 6.1, no Arduino core: `platformio.ini` pins the official PlatformIO platform `espressif32 @ 7.1.1` (ESP-IDF 6.1.0, GCC 15.2 toolchain; `7.0.1` = ESP-IDF 6.0.1 is the fallback). The board definition is `boards/dfrobot_firebeetle2_esp32c6.json` (the official platform has none for this board), the ESP-IDF configuration is `sdkconfig.defaults` (the builder derives the gitignored `sdkconfig.<env>` from it; the `bms` and `demo` envs append `sdkconfig.defaults.bms`, section 5) and the partition table is `partitions.csv`. The first build downloads the toolchain and ESP-IDF (about 1 GB), creates the IDF Python environment under `~/.platformio/penv/.espidf-6.1.0` and compiles ESP-IDF from source (a little over a minute on a recent laptop); later builds take seconds.

| Command | What it does |
|---|---|
| `git submodule update --init` | fetch u8g2 into `components/u8g2` (once per checkout) |
| `pio run` | build the default firmware (env `dfrobot_firebeetle2_esp32c6`) |
| `pio run -t upload` | build and flash over USB (`/dev/cu.usbmodem*`); hold BOOT while plugging in only if the port does not enumerate |
| `pio device monitor` | serial console at 115200 with exception decoder and timestamps |
| `pio test -e native` | host unit tests in six directories (`test/test_vesc`, `test/test_vesc_getvalues`, `test/test_ubx`, `test/test_trip`, `test/test_jk_bms`, `test/test_display_strings_oled`; 183 test cases): STATUS decoder, poll codec (request bytes, reassembly, CRC, decode, fault table), UBX parser and the CFG-PRT / CFG-RATE / VALSET frame builders (byte-exact against the u-blox frames), trip integrator (distance gate, both energy sources, counter resets, outages, window and gap rules, 32-bit clock wrap), JK BMS codec (command bytes, checksum, the frame assembler under every observed chunking, recorded 24S / 32S / 13S frames, layout detection, device info, alarm names), all OLED screens including BMS and CELLS, and the clock arithmetic; no hardware |
| `pio run -e demo -t upload` | flash the layout demo (`DISPLAY_DEMO=1` on top of the `bms` env: all ten screens with synthetic data, dimming after 20 s) |
| `pio run -e bms -t upload` | build and flash with the JK BMS BLE client (`BMS_BLE_ENABLE=1`, NimBLE host from `sdkconfig.defaults.bms`; +349 KB of flash, section 5). `pio run -e bms` alone only builds it |
| `pio run -t menuconfig` | ESP-IDF menuconfig on the env's generated `sdkconfig`; put lasting changes into `sdkconfig.defaults` |
| `pio project init --ide vscode` | regenerate IntelliSense after changing `platformio.ini` |

A healthy boot (ESP-IDF log format `L (ms) tag: message`; numbers and the `...` parts vary, the phrases are the ones the code prints) looks like:

```
I (1512) main: boat-motor 0.1.0 (built Sep  7 2026 19:35:02) reset=POWERON
I (1513) main: cfg CAN : tx=3 rx=2 500 kbit/s mode=NORMAL(ack+poll) rxq=64 vesc_id=-1 poles=10 poll=1000 ms own_id=120
I (1514) main: cfg OLED: sda=22 scl=23 rst=14 addr=0x3C i2c=400000 Hz 128x64 rot=0 period=250 ms dim_after=600000 ms
I (1515) main: cfg GNSS: rx=4 tx=5 rate=200 ms unit=km/h rxbuf=2048
I (1516) main: cfg BMS : ble=0 (build -e bms to enable)
I (1530) can: CAN: node enabled: mode=NORMAL (ACK, transmits only GET_VALUES_SELECTIVE polls) bitrate=500k sample_point=800 permill single-shot tx=GPIO3 rx=GPIO2 rx_queue=64 tx_queue=4 poll=1000 ms own_id=120 mask=0x003EC03C
I (1535) gnss: GNSS: UART1 RX=GPIO4 TX=GPIO5 ring=2048 B, first baud 115200, target baud 115200, rate 200 ms
I (1540) oled: OLED: button on GPIO7 (active low, pull-up, debounce 30 ms, long press 1500 ms, auto-return 60000 ms)
I (1575) oled: OLED: init ok (SSD1309 128x64, I2C 0x3C @ 400000 Hz, sda=22 scl=23 rst=14)
I (1611) main: started: can=1 gnss=1 display=1 trip=1 bms=0
I (1612) trip: trip: task started (period 200 ms, window 10 s, unit Wh/km, min speed 500 mm/s)
I (1640) oled: OLED: task started (poll 20 ms, frame 250 ms, 7 screens, button GPIO7, auto-return 60000 ms, demo=0)
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
I (10000) main: SYS up=10s reset=POWERON heap=... minheap=... lockfail=0 | can=RUN tec=0 rec=0 busoff=0 | disp ok=1 refreshes=.. skipped=.. dim=0 scr=0 btn=0 | poll sent=9 ok=9 bad=0 to=0 | hb can=.. gnss=.. disp=.. | bms=OFF frm=0 hb=0
```

An M9/M10 receiver logs `GNSS: PROTVER 34.10 -> VALSET configuration`, `GNSS: switching UART1 38400 -> 115200 baud (VALSET)` and four `GNSS: VALSET ...: ACK` lines (UART1OUTPROT, MSGOUT, RATE, NAVSPG-DYNMODEL) instead of the CFG-MSG/CFG-RATE ones; one that still runs at 115200 from an earlier boot (backup battery) is found within a second and skips the switch. The first `TRIP` line shows `now=-- avg=--` until the boat has covered `EFF_MIN_DIST_M`. Before a VESC is connected the VESC line prints `never` for every age, `can=RUN` with `frames=0` and `polls=0` (no id to poll yet), the cells show `--` and the CAN line `CAN idle`; that is expected. With `-DVESC_POLL_MS=0` the install line ends in `poll=off` and the VESC line carries `| poll=off` instead of the `| ext:` section. The LED blinks at 1 Hz while VESC data is fresh and slowly otherwise. `OLED: refresh N ms (#N, screen N, stack free min N B)` per frame and `VESC poll: reply 42 B from id 74 in N ms: ...` per reply need `-DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG` in `build_flags` (debug lines of this project's own tags only; the IDF components stay at info).

BMS bring-up (`bms` env, section 5): 1) `pio run -e bms -t upload`, then `pio device monitor`; 2) force-stop the JK app on the phone (or switch its Bluetooth off): the BMS takes one central; 3) the boot log shows `cfg BMS : ble=1 addr=(scan) name=JK* proto=0 cells<=24 sign=dis+ stale=10000 ms prio=4`, `bms: BLE init ok, heap N -> N` and `started: can=1 gnss=1 display=1 trip=1 bms=1`, then the link walks through `match <mac> rssi <n> name <name>` -> connecting -> connected -> mtu -> service / characteristic / CCCD handles -> `device <model> hw <x> sw <y>` -> `layout <24S|32S>` -> stream, followed by `BMS link=STREAM since=... frames ok=N bad=0 heap=...` every 5 s while the BMS screen fills in; 4) compare Vbat, the cells, SoC, Ah, cycles and temperatures with the JK app (connected in turn, since only one of them can be); 5) pin `BMS_BLE_ADDR` to the logged MAC, and `BMS_PROTOCOL` / `BMS_CELLS_MAX` if you like; 6) soak with CAN and GNSS live and check that the SYS line keeps `bms=STREAM` with `frm=` climbing and `heap=` steady and that the VESC and GNSS ages are unchanged; then, optionally, make `bms` the default env (`default_envs` in `platformio.ini`).

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
| BMS screen stays `SCAN ...`, after a minute `SCAN 1m    app open?`; log `BMS link=SCAN` | the phone app holds the BMS's single BLE connection (force-stop it); BMS asleep or out of range; its advertised name does not start with `BMS_BLE_NAME_PREFIX` (look for other `match` lines in the log, or pin the MAC with `BMS_BLE_ADDR`) |
| `CONNECTING` for 10 s, then `SCAN` again, repeating | connect timeouts (`BMS_CONNECT_TIMEOUT_MS`): the app holds the link, or the BMS is at the edge of the range; the back-off grows from 2 s to 30 s between attempts |
| `CONNECTED  layout ?` | neither the 24S nor the 32S layout fits the frames: JK04 firmware (3.x, unsupported) or a layout this codec does not know; send the `device <model> hw <x> sw <y>` log line. `-DBMS_PROTOCOL=1` / `2` forces a layout |
| `NO DATA    last 12s` on a link that was streaming | the BMS stopped notifying; the task recycles the link by itself |
| `BLE OFF    no BMS` in a `bms` build, `started: ... bms=0` | NimBLE init failed: see the `bms:` error lines at boot (`nvs_flash_init`, `nimble_port_init`, task create) |
| BMS values differ from the JK app | the sign convention (`BMS_CURRENT_SIGN`: discharge-positive here, charge-positive in the app) or the app's own rounding; cells, SoC, Ah and cycles are the BMS's raw values |
| `firmware.bin` grew by about 350 KB | expected for the `bms` env (NimBLE host and controller); it still fits the 1.25 MB OTA slot |
| `fatal error: u8g2.h: No such file or directory` | the submodule is empty: `git submodule update --init` |
| `#error This firmware targets the ESP32-C6 (ESP-IDF)` | the board or platform in `platformio.ini` was changed, `sdkconfig.defaults` lost `CONFIG_IDF_TARGET`, or the wrong env is selected |
| A change in `sdkconfig.defaults` or `sdkconfig.defaults.bms` has no effect | the generated `sdkconfig.dfrobot_firebeetle2_esp32c6` (or `sdkconfig.demo`, `sdkconfig.bms`) already exists: delete it and rebuild. An edit to the `.bms` fragment alone is never noticed: `touch sdkconfig.defaults` first |
| The build stops inside `components/u8g2` with a warning reported as an error | ESP-IDF 6 turns the default warnings into errors for IDF-built components; add `CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y` to `sdkconfig.defaults` |
| `#error CAN_OWN_ID must be 1..254` / `CAN_OWN_ID must differ from VESC_CAN_ID` / `VESC_POLL_MS > 0 requires CAN_LISTEN_ONLY 0` | fix the offending override; a listen-only build needs `-DVESC_POLL_MS=0` as well |
| `#error BMS_BLE_ENABLE needs the NimBLE host` | `-DBMS_BLE_ENABLE=1` in an env whose sdkconfig lacks `sdkconfig.defaults.bms`: build `-e bms`, or give your env the same `board_build.cmake_extra_args` |

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

Then build with `pio run -e myboat -t upload`. For a boat with a BMS extend `env:bms` (and inherit `${env:bms.build_flags}`) instead, adding `-DBMS_BLE_ADDR='"C8:47:8C:12:34:56"'` to pin your BMS; the NimBLE fragment comes along with the inherited `board_build.cmake_extra_args`, as it does for the `demo` env.

Compile-time checks stop you from using GPIO12/13, from sharing a GPIO between two peripherals (the button pin takes part; `PIN_OLED_RST` is checked separately because it may be -1), from an unsupported CAN bitrate, from a `CAN_OWN_ID` outside 1..254 or equal to `VESC_CAN_ID`, from polling in listen-only mode, from a `GNSS_RATE_MS` outside 50..10000 ms, from an `EFF_WINDOW_S` outside 2..120 s, from a `BMS_CELLS_MAX` outside 1..32 or a `BMS_PROTOCOL` outside 0..2, from a `BMS_TASK_PRIO` at or above `TASK_PRIO_GNSS`, from a BLE scan window longer than its interval and from `BMS_BLE_ENABLE 1` without the NimBLE host in the sdkconfig (`sdkconfig.defaults.bms`). `src/gnss_ubx.cpp` adds `static_assert`s that `GNSS_TARGET_BAUD` is 0 or one of `GNSS_BAUDS` and that `GNSS_BAUD_SWITCH_ATTEMPTS` is 1..255. `src/display_oled.cpp` adds `static_assert`s for 128x64, `OLED_ROTATION` 0 or 2, `PIN_OLED_RST` in -1..30, contrast 0..255, `OLED_I2C_ADDR` a 7-bit address, `OLED_I2C_HZ` in 1..1000000, `PIN_BUTTON` in -1..30, `BUTTON_DEBOUNCE_MS` shorter than `BUTTON_LONG_PRESS_MS` and `OLED_BUTTON_POLL_MS` not above `OLED_PERIOD_MS`.

| Group | Defines |
|---|---|
| Display demo | `DISPLAY_DEMO` (1 = synthetic changing values on every screen, the `demo` env) |
| OLED | `PIN_OLED_SCL`, `PIN_OLED_SDA`, `PIN_OLED_RST` (-1 = not wired), `OLED_I2C_ADDR`, `OLED_I2C_HZ`, `OLED_WIDTH`, `OLED_HEIGHT`, `OLED_ROTATION`, `OLED_PERIOD_MS`, `OLED_CONTRAST`, `OLED_IDLE_DIM_MS`, `OLED_IDLE_CONTRAST` |
| Screens, button, clock | `PIN_BUTTON` (-1 = none, 9 = BOOT button), `BUTTON_ACTIVE_LOW`, `BUTTON_DEBOUNCE_MS`, `BUTTON_LONG_PRESS_MS`, `SCREEN_AUTO_RETURN_MS` (0 = stay), `TIME_UTC_OFFSET_MIN` |
| CAN / VESC | `PIN_CAN_TX`, `PIN_CAN_RX`, `CAN_BITRATE_KBPS` (125/250/500/1000), `CAN_LISTEN_ONLY`, `CAN_RX_QUEUE_LEN`, `VESC_CAN_ID` (-1 = any), `VESC_MOTOR_POLES`, `VESC_STALE_R1_MS`, `VESC_STALE_R2_MS`, `CAN_EMA_ALPHA`, `CAN_HEALTH_LOG_MS`, `CAN_LOG_RAW_FRAMES` (also logs the poll request and reply frames) |
| Active VESC polling | `VESC_POLL_MS` (0 = never transmit), `CAN_OWN_ID` (1..254, not a VESC id), `VESC_POLL_TIMEOUT_MS`, `VESC_EXT_STALE_MS`, `CAN_TX_QUEUE_LEN` |
| GNSS | `PIN_GNSS_RX`, `PIN_GNSS_TX`, `GNSS_BAUDS`, `GNSS_TARGET_BAUD` (0 = keep the detected baud), `GNSS_RX_BUFFER`, `GNSS_RATE_MS` (200 = 5 Hz, 100 = 10 Hz), `GNSS_DYNMODEL_SEA`, `GNSS_STALE_MS`, `GNSS_REDETECT_MS`, `GNSS_ACK_TIMEOUT_MS`, `GNSS_MAX_SACC_MM_S`, `SPEED_MIN_SHOW`, `SPEED_UNIT_KNOTS` |
| Trip / efficiency | `TRIP_PERIOD_MS`, `EFF_WINDOW_S` (2..120 s), `EFF_MIN_SPEED_MM_S`, `EFF_MIN_DIST_M`, `EFF_UNIT_KM` (1 = Wh/km even with knots), `LOG_TRIP_MS` (0 = off) |
| BMS over BLE (`bms` env, section 5) | `BMS_BLE_ENABLE` (set by `[env:bms]`), `BMS_BLE_ADDR` ("" = scan by name prefix / service, a MAC = connect by address), `BMS_BLE_NAME_PREFIX`, `BMS_PROTOCOL` (0 = auto, 1 = JK02_24S, 2 = JK02_32S), `BMS_CELLS_MAX` (1..32), `BMS_CURRENT_SIGN` (1 = discharge-positive), `BMS_STALE_MS`, `BMS_RECONNECT_MS`, `BMS_RECONNECT_MAX_MS`, `BMS_RECONNECT_FAILS`, `HB_MAX_BMS_MS` (0 = never gate the watchdog), `LOG_BMS_MS` (0 = off). Derived, do not edit: `BMS_UI_ENABLE` (`BMS_BLE_ENABLE` or `DISPLAY_DEMO`: the screens and `g_state.bms` exist) and `BMS_CELL_PAGES` (ceil(`BMS_CELLS_MAX` / 12)) |
| Tasks / watchdog / logging | `TASK_PRIO_CAN`, `TASK_PRIO_GNSS`, `TASK_PRIO_DISP`, `WDT_TIMEOUT_MS`, `HB_MAX_CAN_MS`, `HB_MAX_GNSS_MS`, `HB_MAX_DISP_MS`, `LOG_VESC_MS`, `LOG_GNSS_MS`, `LOG_TRIP_MS`, `LOG_SYS_MS`, `SERIAL_BOOT_DELAY_MS`, `PIN_LED`, `FW_VERSION` |
| Advanced (defaults are fine) | `CAN_TASK_STACK`, `CAN_INSTALL_RETRY_MS`, `CAN_RX_TIMEOUT_MS`, `CAN_ERR_LOG_MIN_MS`, `CAN_ERR_WARN_LEVEL`, `CAN_TX_WAIT_MS`, `VESC_POLL_LOG_MIN_MS`, `VESC_POLL_BACKOFF_AFTER`, `VESC_POLL_BACKOFF_MS`, `VESC_GETVALUES_MASK` (0 = plain `COMM_GET_VALUES`), `VESC_RX_BUFFER_SIZE`, `GNSS_TASK_STACK`, `GNSS_AUTOBAUD_LISTEN_MS`, `GNSS_MONVER_TIMEOUT_MS`, `GNSS_AUTOBAUD_RETRY_MS`, `OLED_TASK_STACK`, `OLED_INIT_RETRY_MS`, `OLED_I2C_TIMEOUT_MS`, `OLED_BUTTON_POLL_MS`. Also in `config.h`, with local `#ifndef` fallbacks in the sources so a trimmed copy still builds: `GNSS_BAUD_SWITCH_ATTEMPTS` (2), `GNSS_BAUD_SWITCH_SETTLE_MS` (100), `TASK_PRIO_TRIP` (3), `TRIP_TASK_STACK` (4096), `TRIP_DT_MAX_MS` (5000), `TRIP_COUNTER_RESET_WH` (0.5), `TRIP_EFF_MIN_FILL_S` (3), `TRIP_STALE_MS` (5000). The BMS link timing (section 5): `BMS_TASK_PRIO` (4), `BMS_TASK_STACK` (4096), `BMS_CONNECT_TIMEOUT_MS` (10000), `BMS_SETUP_TIMEOUT_MS` (8000), `BMS_FIRST_FRAME_TIMEOUT_MS` (15000), `BMS_POLL_MS` (5000), `BMS_SCAN_FAST_MS` (30000), `BMS_SCAN_FAST_ITVL_MS` / `BMS_SCAN_FAST_WINDOW_MS` (60 / 30), `BMS_SCAN_SLOW_ITVL_MS` / `BMS_SCAN_SLOW_WINDOW_MS` (1000 / 30), `BMS_MSG_BUF_BYTES` (2048), `BMS_APP_HINT_S` (60). `src/can_vesc.cpp` alone: `CAN_SAMPLE_POINT_PERMILL` (800, the bit sample point) |

Notes: the OLED layout is hard-coded for 128x64 in `OLED_ROTATION` 0 or 2. `OLED_PERIOD_MS` may go down to about 50 (a frame is ~30 ms), but 250 is plenty for EMA-smoothed currents and a 5 Hz speed that is pushed only when a digit changes. `VESC_POLL_MS` can go down to 100..200 if a faster fault indication is wanted (keep `VESC_POLL_TIMEOUT_MS` below it, or accept that a poll is skipped while a reply is outstanding). `WDT_TIMEOUT_MS` (20 s) only has to exceed the longest legitimate task stall (a stuck I2C bus costs `OLED_I2C_TIMEOUT_MS` per transaction). `GNSS_BAUDS` is a brace list and is easiest to change in `config.h` itself; `GNSS_TARGET_BAUD` must stay 0 or one of its members, best the first one (a lost switch verification then costs one autobaud pass). `TRIP_PERIOD_MS` should match `GNSS_RATE_MS`; a longer window (`EFF_WINDOW_S`) steadies `now` at the price of a slower response. The NimBLE settings (roles, connection count, preferred MTU 256, pool sizes, TX power) are Kconfig, not `config.h`: edit `sdkconfig.defaults.bms`, then `touch sdkconfig.defaults` so the builder notices.

## 9. Passivity and safety notes

- With the default `VESC_POLL_MS 1000` the board is no longer silent on the bus: it transmits one 7-byte read request per second under its own id `CAN_OWN_ID` (`candump` on a second adapter shows `0000084A [7] 78 00 32 00 3E C0 3C` for VESC id 74) and the VESC answers with seven frames. The request is the same read-only query VESC Tool uses; nothing in the firmware can write a setting or command the motor (`grep -rn twai_node_transmit src/` finds the single call in `poll_tick()`, under `#if VESC_POLL_MS > 0`). Build with `-DVESC_POLL_MS=0` to get a node that only ACKs and contains no transmit call at all.
- The BLE link of the `bms` env (section 5) is read-only as well: the firmware writes exactly two registers to the BMS, 0x97 (device info) and 0x96 (start the cell-info stream), plus the CCCD subscription (`01 00`) that enables notifications; it never changes a BMS setting (charge / discharge switches, protection limits, balancing, names). It does occupy the BMS's single BLE slot while connected, so the JK app cannot be used at the same time, and it stores no bonding: after the first controller enable (PHY calibration blob, written before the CAN node exists) nothing is written to flash.
- The polled averages (Iavg, id/iq, vd/vq) are "since the previous read": running VESC Tool's realtime page at the same time shortens their window on both sides. Everything else is unaffected by a second reader.
- With the VESC powered off, each unanswered request costs the controller 8 error points until it goes error-passive (~16 polls); the log then shows `CAN: error passive` and the poll warnings every 10 s and the request rate drops to every 5 s. No bus-off, and it recovers by itself when the VESC is back.
- Displayed and logged power is signed: `v_in x current_in`, negative while regenerating (the water drives the motor); VESC Tool's realtime power stat is an absolute value, so the two can differ in sign, not in magnitude. The cells are EMA-smoothed (`CAN_EMA_ALPHA`) and may be up to `VESC_STALE_R1_MS` old; a fault shows on the VESC 1/3 page only after the next poll (up to `VESC_POLL_MS` later) and only as long as the VESC keeps it or via the latch. This is a dashboard, not a protection device: rely on the VESC's own limits and a BMS for that.
- OLED modules are specified for roughly -40..70 C (vendor dependent; some 1.54" boards only -20..60 C), but the glass, the FPC and the boost converter must stay dry: condensation on a boat needs an enclosure or conformal coating. In direct sun a ~110 cd/m2 OLED is hard to read; a sunshade helps more than contrast.
- Burn-in is the OLED's wear mechanism: a dashboard that shows the same numbers for hours is the worst case. Keep `OLED_IDLE_DIM_MS` enabled and expect the panel to lose brightness over thousands of hours; the detail screens' inverted title bar is the only filled area, and the auto-return brings the sparse main screen back after a minute.
- A hung task reboots the board after `WDT_TIMEOUT_MS` (20 s); the next banner reports `reset=TASK_WDT`. The OLED keeps its last image until the firmware pulses RES about 1.5 s into the reboot, then shows the boot frame.
- Apart from the CAN transceiver everything is 3.3 V. Nothing on the header tolerates 5 V, and neither does an OLED module whose pull-ups would then sit at 5 V, nor a button wired to anything but GND or 3V3.

## 10. Future work (out of scope for this version)

- `CAN_PACKET_PING` / `PONG` discovery so polling can start on a VESC that broadcasts no status frames without pinning `VESC_CAN_ID`, and answering pings so the board shows up in VESC Tool's CAN scan.
- Burn-in mitigation beyond dimming: `DISPLAYOFF` (0xAE) after a long idle, or shifting the layout by a pixel every few minutes.
- OTA updates (`partitions.csv` already has two app slots).
- Deep sleep and the LP-UART for the GNSS receiver.
- Persisting the trip (distance, energy) across reboots and logging a track; today the trip restarts at every boot and nothing is written to flash.
- `CONFIG_TWAI_ISR_IN_IRAM` (with `IRAM_ATTR` on the callbacks in `src/can_vesc.cpp`) if flash writes (trip persistence, OTA) are ever added, so CAN reception continues while the flash cache is disabled.
- JK BMS: the JK04 (software 3.x) frame layout; those modules today stop at `CONNECTED  layout ?`.
