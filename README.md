# boat-motor

ESP32-C6 firmware that listens to a VESC motor controller on CAN and shows battery voltage, battery current, motor current, electrical power and the ground speed from a u-blox GNSS receiver on a 128x64 monochrome OLED (SSD1309 controller, I2C). Built for the DFRobot FireBeetle 2 ESP32-C6 (DFR1075) with an MCP2551-class CAN transceiver, any 2.42" or 1.54" SSD1309 module switched to I2C mode and any receiver that speaks UBX (u-blox 6/7, M8, M9, M10 or a compatible clone). On the CAN bus the firmware is protocol-passive: it acknowledges frames but never transmits one, so it cannot disturb the VESC.

The dashboard was first built for a 4.2" Good Display GDEY042T81 e-paper panel; that panel broke. The e-ink code stays in the tree as a legacy option, selected with the `DISPLAY_TYPE` switch in `config.h` and built only by `pio run -e epd`; everything marked "legacy" below refers to it.

## 1. What it does

Four FreeRTOS tasks on the single core (tick 1 ms). Priorities guarantee that CAN reception is never starved by a frame push or by a stuck I2C bus (50 ms timeout per transaction).

| Task | Priority | Stack | Job |
|---|---|---|---|
| `canTask` (`src/can_vesc.cpp`) | `TASK_PRIO_CAN` = 6 | 4096 | `twai_receive()`, keep VESC STATUS 1..6 frames, decode, smooth (EMA), write `g_state.vesc`; bus-off recovery and health counters |
| `gnssTask` (`src/gnss_ubx.cpp`) | `TASK_PRIO_GNSS` = 5 | 4096 | AUTOBAUD -> DETECT (MON-VER) -> CONFIGURE -> RUN; parses NAV-PVT into `g_state.gnss`; back to autobaud after `GNSS_REDETECT_MS` of silence |
| `displayTask` (`src/display_oled.cpp`) | `TASK_PRIO_DISP` = 2 | 6144 (`OLED_TASK_STACK`) | every `OLED_PERIOD_MS` builds the display strings from a snapshot, pushes a frame only when a string changed, dims the panel after `OLED_IDLE_DIM_MS` idle, re-probes the controller every 5 s and re-initialises a panel that stopped answering |
| `loop()` supervisor (`src/main.cpp`) | 1 | Arduino default | sole Task-WDT subscriber (`WDT_TIMEOUT_MS`), fed only while all three task heartbeats are fresh; 1 Hz VESC and GNSS log lines, 10 s SYS line with the display counters; LED |

Data flow:

```
 VESC ---CAN---> transceiver -> TWAI ISR -> RX queue (64 frames) -> canTask --+
                                                                              +--> g_state (one mutex) --> displayTask -> DIYables_OLED_SSD1309 -> I2C -> SSD1309 OLED
 GNSS ---UART--> UART1 ISR -> 2048 B ring -------------------------> gnssTask -+                         \-> loop(): logs, watchdog, LED
```

Every hold of the mutex is a memcpy or a few field writes, never I/O; consumers work on snapshots. Every timestamp is a `millis()` value where 0 means "never received" and all age arithmetic is wrap-safe.

Files:

| Path | Purpose |
|---|---|
| `include/config.h` | every pin and tunable plus the display selection; the only file you normally edit |
| `include/vesc_status.h` | header-only VESC STATUS_1..6 decoder (Arduino-free, unit-tested) |
| `include/ubx_min.h` | header-only UBX frame parser + NAV-PVT / NAV-VELNED decoders (Arduino-free, unit-tested) |
| `include/shared_state.h`, `src/shared_state.cpp` | `g_state`, mutex, snapshots, heartbeats, `fresh()`, `DisplayStats` |
| `include/can_vesc.h`, `src/can_vesc.cpp` | TWAI setup, canTask, alerts and bus-off recovery |
| `include/gnss_ubx.h`, `src/gnss_ubx.cpp` | Serial1 setup, autobaud, MON-VER detection, VALSET/legacy configuration |
| `include/display.h` | `display_start()`, the one entry point both display back-ends implement |
| `include/display_strings_oled.h` | pure OLED string builder (4-character value cells, stale -> `--`), unit-tested |
| `src/display_oled.cpp` | I2C bring-up and presence probe, 128x64 layout, change-detected refresh, idle dimming |
| `include/display_strings.h`, `include/display_epd.h`, `src/display_epd.cpp` | legacy e-ink string builder and GxEPD2 task; compiled only in the `epd` env |
| `src/main.cpp` | `setup()` and the supervisor `loop()` |
| `test/test_vesc/`, `test/test_ubx/`, `test/test_display_strings_oled/`, `test/test_display_strings/` | Unity tests (one `test_main.cpp` each), run on the host with `pio test -e native` |

## 2. Hardware and wiring

Silkscreen labels on the FireBeetle 2 ESP32-C6 equal GPIO numbers.

| Signal | GPIO (`config.h`) | FireBeetle silkscreen | Module pin |
|---|---|---|---|
| CAN TX | 3 (`PIN_CAN_TX`) | 3 / A2 | MCP2551 TXD (pin 1), through 1 kOhm |
| CAN RX | 2 (`PIN_CAN_RX`) | 2 / A1 | MCP2551 RXD (pin 4), through the level-shift divider |
| OLED SCL | 23 (`PIN_OLED_SCL`) | 23 / SCK | OLED SCL (CLK / D0) |
| OLED SDA | 22 (`PIN_OLED_SDA`) | 22 / MOSI | OLED SDA (DIN / D1) |
| OLED RST | 14 (`PIN_OLED_RST`, optional: -1 = not wired) | 14 / RES | OLED RES |
| GNSS RX | 4 (`PIN_GNSS_RX`) | 4 / LP_RX | GNSS module TX |
| GNSS TX | 5 (`PIN_GNSS_TX`) | 5 / LP_TX | GNSS module RX |
| LED | 15 (`PIN_LED`) | 15 | on-board LED, nothing to wire |

The OLED module and the GNSS module go to the board's 3V3 and GND. The CAN transceiver needs more care.

### CAN transceiver: the 5 V problem

The MCP2551 is a 5 V part. ESP32-C6 pads are not 5 V tolerant (absolute maximum VDD + 0.3 V), so its RXD output must never reach GPIO2 directly.

- Preferred: a transceiver with 3.3 V I/O and no divider at all: MCP2562 (VDD 5 V, VIO pin to 3V3), TJA1051T/3 (VIO to 3V3) or SN65HVD230 (3.3 V supply, needs no 5 V).
- If you already have an MCP2551: RXD (pin 4) -> 1 kOhm -> GPIO2, plus 2.2 kOhm from GPIO2 to GND. That gives 5.0 V x 2.2/3.2 = 3.44 V at a nominal supply but 3.61 V at 5.25 V, which is over the absolute maximum. Measure your 5 V rail, then measure the idle level at the divider output before plugging it into the board: 3.2..3.5 V, never above 3.6 V.
- TXD (pin 1) accepts 3.3 V directly (VIH 2.0 V). Put 1 kOhm in series anyway: TXD has an internal 25 kOhm pull-up to 5 V and the resistor limits the current into GPIO3 while the ESP32 is in reset. TXD must be wired even though the firmware never sends data: the controller hardware generates the ACK bit on this line.
- RS (pin 8) to GND (high-speed slope mode).
- 120 Ohm termination at both ends of the bus. Most VESC boards have no on-board terminator; with everything powered off you should measure about 60 Ohm between CANH and CANL once both terminators are in place (120 Ohm means only one).
- Power the transceiver from the VESC's 5 V pin (or another regulated 5 V) and common the grounds. The FireBeetle VIN pin is dead when the board runs from its battery connector, and USB 5 V is only there while you debug.

### OLED: 128x64 SSD1309 over I2C

Any 128x64 SSD1309 module works (Waveshare, DIYables, LCDWIKI, Diymore / ACEIRMC 2.42" or 1.54", Adafruit 2719); the firmware drives it with the DIYables_OLED_SSD1309 library (Adafruit_SSD1306-compatible API on Adafruit GFX). Most 7-pin modules ship in 4-wire SPI mode and must be switched to I2C first.

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
- Why the old SPI pins: the harness that ran to the e-ink already carries GPIO22/23/14, the C6 routes I2C to any GPIO through its GPIO matrix (22 and 23 are unrestricted "P2" pins) and nothing else uses SPI. The former CS/DC/BUSY pins (1, 8, 18) are free. If you prefer the FireBeetle's default I2C pins (19 = SDA, 20 = SCL, also on the GDI connector), build with `-DPIN_OLED_SDA=19 -DPIN_OLED_SCL=20`.
- Burn-in: white SSD1309 glass is rated ~20,000 h to half luminance (yellow ~50,000 h) and static images leave residual ghosts. The layout is black with sparse text, and the contrast drops to `OLED_IDLE_CONTRAST` after `OLED_IDLE_DIM_MS` without a change (section 5). If the panel is still to be bought, a yellow one lasts longer.

<details>
<summary>Legacy e-ink: GDEY042T81 on DESPI-C02 (<code>pio run -e epd</code>)</summary>

| Signal | GPIO (`config.h`) | Silkscreen | DESPI-C02 pin |
|---|---|---|---|
| SCK | 23 (`PIN_EPD_SCK`) | 23 / SCK | SCK |
| MOSI | 22 (`PIN_EPD_MOSI`) | 22 / MOSI | SDI |
| CS | 1 (`PIN_EPD_CS`) | 1 / CS | CS |
| DC | 8 (`PIN_EPD_DC`) | 8 / DC | D/C |
| RST | 14 (`PIN_EPD_RST`) | 14 / RES | RES |
| BUSY | 18 (`PIN_EPD_BUSY`) | 18 / SD_CS | BUSY |

- Set the DESPI-C02 "RESE" DIP switch to "3" (older adapter revision) or "2.2 Ohm" (current revision). Position 0.47 is for UC81xx-based "GDEW" panels and gives a blank or faint image on this SSD1683 panel.
- 3.3 V only, supply and every signal line. BUSY is a panel-driven, active-high input; `config.h` refuses (`#error`) to build the `epd` env with BUSY on a strapping pin (8, 9, 15). DC on GPIO8 is a strapping pin, but an MCU-driven output, so boot is unaffected.
- Operating temperature 0..50 C; below about 10 C set `EPD_FAST_FULL_UPDATE 0`. The pins are the FireBeetle's default SPI (GDI connector) pins, MISO unused.
</details>

### GNSS receiver

Any u-blox receiver or clone with a 3.3 V UART: NEO-6M/7M, NEO-M8N, SAM-M8Q, NEO-M9N, MAX-M10S and the like. Module TX -> GPIO4, module RX <- GPIO5, VCC 3V3, GND. Some breakout boards take 5 V in and regulate it down but keep 3.3 V logic; a few have 5 V logic. Check the datasheet: the C6 pins are not 5 V tolerant. The board has a 499 Ohm series resistor on GPIO5; that is fine for UART.

### Pins to leave alone

| GPIO | Why |
|---|---|
| 12, 13 | USB D-/D+: the `Serial` console (USB-CDC). `config.h` refuses to build if a peripheral uses them |
| 16, 17 | UART0, the ROM/IDF boot console. GPIO16 is driven by UART0 TX for the whole run |
| 9 | BOOT button, boot-mode strapping |
| 0 | battery ADC, not on the header |
| 8, 15 | strapping pins (boot mode, JTAG select). MCU-driven outputs are fine (LED, the legacy e-ink DC); never a panel-driven input |
| 4, 5 | strapping only for the SDIO edge, harmless; used for GNSS |

## 3. VESC configuration (VESC Tool)

A VESC broadcasts nothing on CAN until you enable status messages. Connect VESC Tool to the VESC, then App Settings > General:

| Setting | Value |
|---|---|
| CAN Baud Rate | 500K (must match `CAN_BITRATE_KBPS`) |
| CAN Mode | VESC |
| Can Messages Rate 1 (FW 6.x) | Status 1, Status 4, Status 5 at 10..50 Hz |
| Can Messages Rate 2 (FW 6.x) | Status 2, Status 3 (and Status 6 if you want the ADC/PPM values) at 1..5 Hz |
| Can Status Message Mode (FW <= 5.03) | CAN_STATUS_1_2_3_4_5, status rate 10..50 Hz |
| VESC ID | note it; leave `VESC_CAN_ID -1` to lock onto whatever appears first, or pin that id |

Then "Write App Configuration". Also note Motor Settings > Additional Info > Motor Poles and put it in `VESC_MOTOR_POLES`; it is only used to log mechanical rpm (mech rpm = erpm / (poles / 2)).

### Why the board acknowledges but never transmits

`CAN_LISTEN_ONLY 0` (default) runs the TWAI controller in NORMAL mode with an empty TX queue. The firmware never calls `twai_transmit`, so it never arbitrates for the bus or sends data; it only generates the ACK bit for frames it receives. That ACK is required when the VESC and this board are the only two nodes: the VESC's STM32 bxCAN has no retransmit limit, so with a non-acknowledging listener it goes error-passive and repeats the same stale STATUS frame forever ("lonely VESC"). Set `-DCAN_LISTEN_ONLY=1` (no ACK, TX electrically silent) only when another node acknowledges anyway: a second VESC, a BMS, a permanently attached VESC Tool CAN adapter.

### What each status frame carries

| Frame | packet id | Fields (resolution per LSB) | Rate group | Used for |
|---|---|---|---|---|
| STATUS_1 | 9 | erpm, motor current (0.1 A, signed), duty (0.001) | Rate 1 | `M/A` cell, CAN line freshness, log |
| STATUS_2 | 14 | Ah drawn, Ah charged (0.1 mAh) | Rate 2 | log |
| STATUS_3 | 15 | Wh drawn, Wh charged (0.1 mWh) | Rate 2 | log |
| STATUS_4 | 16 | FET temp, motor temp (0.1 C), battery current (0.1 A, signed), PID position | Rate 1 | `B/A` and `P/W` cells; FET temp in the log (and the legacy e-ink status bar) |
| STATUS_5 | 27 | tachometer, input voltage (0.1 V) | Rate 1 | `B/V` and `P/W` cells, CAN line freshness |
| STATUS_6 | 58 | ADC1..3 (1 mV), PPM (0.001) | Rate 2 | log |

STATUS 1/4/5 older than `VESC_STALE_R1_MS` (2 s) are shown as `--`; STATUS 2/3/6 use `VESC_STALE_R2_MS` (5 s) in the log. Not available passively, because they only appear in replies to `COMM_GET_VALUES`: fault code, id/iq currents, per-MOSFET temperatures (section 9).

## 4. GNSS

The receiver is driven with the binary UBX protocol only; NMEA output is switched off.

- Generations: u-blox 7, M8, M9, M10 and clones via NAV-PVT; u-blox 6 (no NAV-PVT) via NAV-VELNED. The generation is read from MON-VER's PROTVER: >= 27 is configured with VALSET (CFG-UART1OUTPROT, CFG-MSGOUT, CFG-RATE), 14..26 with the legacy CFG-MSG / CFG-RATE messages, below 14 or on a NAK for NAV-PVT the task enables NAV-VELNED instead. No MON-VER reply, or every VALSET refused, also falls back to the legacy messages.
- Autobaud: `GNSS_BAUDS` = 38400, 9600, 115200, 57600, 230400, tried in that order. A rate is accepted when a checksum-valid UBX frame arrives within 1.2 s or the receiver answers a MON-VER poll within 1.5 s. Factory modules are usually NMEA-only, so the MON-VER path is the one that normally succeeds. If no rate works the task pauses 2 s and starts over, forever, without blocking anything else.
- Configuration is written to RAM (VALSET also to BBR, the battery-backed RAM), never to the receiver's flash. The receiver is reconfigured on every boot and after every redetect; a configuration you saved to the receiver's flash with u-center is left untouched and returns after a backup-battery loss.
- `GNSS_RATE_MS` sets the navigation rate (1000 = 1 Hz). `GNSS_DYNMODEL_SEA 1` sets the dynamic model to SEA (5), which suits a boat.
- `SPEED_UNIT_KNOTS 1` = knots, 0 = km/h. Speed shows `--` without a 2D/3D fix, when the receiver's own speed-accuracy estimate is worse than `GNSS_MAX_SACC_MM_S`, or when no NAV-PVT arrived for `GNSS_STALE_MS`. Speeds below `SPEED_MIN_SHOW` are shown as 0.0 to hide drift at the mooring.
- No valid UBX frame for `GNSS_REDETECT_MS` (20 s) while running (module power-cycled, baud lost) sends the task back to autobaud without a reboot.
- The receiver is on HP UART1 (`Serial1`) with a `GNSS_RX_BUFFER` (2048 byte) receive ring, so no NAV-PVT epoch is lost while the display task holds the CPU (an OLED frame is ~26 ms of I2C; the legacy e-ink refresh took seconds). Serial2 is the LP UART on the C6 and is not used.

## 5. Display behaviour

128 x 64, white on black, classic 5x7 GFX font (a glyph cell is 6n x 8n px at text size n), `OLED_ROTATION 0`:

```
 +----------------------------+
 |   1 2 . 3      kn          |   speed, size-3 digits (max 4 chars) | unit
 |                3D  9sv     |                                      | fix line
 |                VESC 74     |                                      | CAN line
 |----------------------------|   rule
 | B     4 8 . 2  B   1 2 . 4 |   B/V battery volts | B/A battery amps (signed)
 | V              A           |   size-2 values, max 4 chars, right-aligned
 | M        3 5   P     5 9 8 |   M/A motor amps    | P/W power in watts (signed)
 | A              W           |
 +----------------------------+
```

- Zone A (y 0..24): ground speed right-aligned in size-3 digits, and a size-1 column at x 80 with the unit (`kn` / `km/h`), the fix line and the CAN line (8 characters each).
  - Fix line: `3D  9sv` / `2D 12sv` (fix type and satellites used), `NO FIX` (live receiver without a fix), `NO DATA` (a receiver was seen but no epoch for `GNSS_STALE_MS`), `NO GNSS` (nothing received since boot), `BOOT` on the first frame.
  - CAN line: `VESC 74` (controller running, STATUS_1 or STATUS_5 within `VESC_STALE_R1_MS`, id locked), `CAN idle` (controller running, no fresh frame: VESC off or status messages not enabled), `CAN RECV` (bus-off recovery), `CAN OFF` (STOPPED / BUS_OFF / driver not installed). The first frame shows the firmware version here.
- Zone B (y 28..62): four values with stacked letter captions. `B/V` = STATUS_5 input voltage, `B/A` = STATUS_4 battery current (signed), `M/A` = STATUS_1 motor current, `P/W` = voltage x battery current (signed, only when both are fresh). Currents and voltage are EMA-smoothed (`CAN_EMA_ALPHA`, 0.15; 1.0 = off).
- Value cells hold 4 characters: one decimal while it fits (`48.2`, `12.4`, `10.0`), else whole units (`100`, `-12`, `117`, `-480`), else thousands (`12k`, `-5k`), else `MAX` / `-MAX`. Battery current keeps its decimal only while positive and below 100 A; motor current is whole amps, `1k` from 1000 A. Speed shows `0.0` below `SPEED_MIN_SHOW` and `--` without a usable fix, with a speed-accuracy estimate above `GNSS_MAX_SACC_MM_S` or without an epoch for `GNSS_STALE_MS`. Anything stale or never received is `--`; `-0.0` is printed as `0.0` so a current hovering around zero does not flicker.

Refresh and idle policy (`config.h`):

| Define | Default | Effect |
|---|---|---|
| `OLED_PERIOD_MS` | 250 | tick (4 Hz); each tick renders a snapshot to strings and compares them with what is on the panel; nothing changed = no I2C traffic. A changed frame pushes the whole 1 KB buffer, ~26 ms at 400 kHz |
| `OLED_CONTRAST` | 255 | normal contrast (0..255) |
| `OLED_IDLE_DIM_MS` | 600000 | no string changed for 10 min -> contrast drops to `OLED_IDLE_CONTRAST` (log `OLED: dimmed after N s idle`); the next change restores it (`OLED: contrast restored (value changed)`). 0 = never dim |
| `OLED_IDLE_CONTRAST` | 8 | contrast while dimmed; segment current falls roughly 15..40x versus 255 |
| `OLED_ROTATION` | 0 | 0 = connector at the top, 2 = upside down. 1 and 3 (portrait) are rejected at compile time: the layout is landscape-only |
| `OLED_I2C_HZ` | 400000 | bus clock; the SSD1309 is specified to 400 kHz. 100000 for long or noisy wiring |
| `OLED_WIDTH`, `OLED_HEIGHT` | 128, 64 | panel geometry; the layout is hard-coded for it (`static_assert`) |

The panel is probed with a zero-length I2C transaction before every init, so a missing display is reported instead of drawn into blindly: `display_start()` logs `OLED: no response at 0x3C (check wiring/address jumper), Wire error N` and the task retries every 5 s (`OLED: still no response at 0x3C (retry N, Wire error N)`); while running it re-probes every 5 s and re-initialises a panel that stopped answering (`OLED: lost contact at 0x3C (Wire error N), re-initialising`), so hot-plugging works in both directions. After every (re)init the panel shows the boot frame (`--` everywhere, `BOOT`, firmware version) and the live frame on the next tick. `-DDISPLAY_DEMO=1` (the `demo` env) feeds synthetic changing values with a 30 s hold every 2 min so you can check the layout and the idle dimming with no VESC or GNSS attached (`-DOLED_IDLE_DIM_MS=20000` to see the dimming sooner).

<details>
<summary>Legacy e-ink refresh policy (<code>epd</code> env)</summary>

Landscape 400 x 300, black on white: speed in large digits with satellite count, fix state and UTC time (zone A), four tiles BATTERY V / BATTERY A / MOTOR A / POWER W (zone B) and a status bar with VESC id, TWAI state, FET temperature, GNSS baud and the partial-refresh counter `#N` (zone C).

| Define | Default | Effect |
|---|---|---|
| `DISPLAY_PERIOD_MS` | 1000 | tick; keep at 1000 or more, a partial cycle takes 0.5..0.7 s |
| `EPD_FULL_EVERY_N_PARTIALS` | 30 | changes are drawn with fast partial updates (~0.4 s, no flash); after this many, a full refresh (flashes) cleans ghosting ... |
| `EPD_FULL_EVERY_MS` | 300000 | ... or at least every 5 min, whichever comes first. Lower N if ghosting is visible |
| `EPD_POWEROFF_IDLE_MS` | 15000 | nothing changed for 15 s -> `powerOff()` (booster off, image stays) |
| `EPD_HIBERNATE_IDLE_MS` | 600000 | ... after 10 min -> `hibernate()` (deep sleep); the next change wakes the panel with a full refresh |
| `EPD_FAST_FULL_UPDATE` | 1 | fast full refresh (~1.1 s, fixed waveform temperature); 0 below ~10 C (2..3 s, temperature-compensated, less ghosting in the cold) |
| `EPD_SPI_HZ`, `EPD_RESET_MS`, `EPD_ROTATION` | 4000000, 10, 0 | SPI clock (the SSD1683 allows 20 MHz), reset pulse, landscape |
| `EPD_DIAG_BAUD` | 0 | 115200 makes GxEPD2 print refresh timings on Serial during bring-up |

The first image after boot costs two full refreshes (~2.5 s). GxEPD2 hard-codes a 10 s BUSY timeout, which is why `WDT_TIMEOUT_MS` is 20 s.
</details>

## 6. Building, flashing, testing

Prerequisites: PlatformIO Core (`pio` on the PATH, e.g. `~/.platformio/penv/bin/pio`) and a USB-C cable to the FireBeetle's USB port. The platform is pinned in `platformio.ini` to a pioarduino release (`.../platform-espressif32/releases/download/55.03.31-1/platform-espressif32.zip`: Arduino core 3.3.1 on ESP-IDF 5.5.1). Do not replace it with bare `espressif32`: the registry version has no ESP32-C6 support. The first build downloads about 475 MB of toolchain and precompiled libraries.

| Command | What it does |
|---|---|
| `pio run` | build the default OLED firmware (env `dfrobot_firebeetle2_esp32c6`) |
| `pio run -t upload` | build and flash over USB (`/dev/cu.usbmodem*`); hold BOOT while plugging in only if the port does not enumerate |
| `pio device monitor` | serial console at 115200 with exception decoder and timestamps |
| `pio test -e native` | host unit tests (`test/test_vesc`, `test/test_ubx`, `test/test_display_strings_oled`, `test/test_display_strings`) for the VESC decoder, the UBX parser and both display-string builders; no hardware |
| `pio run -e demo -t upload` | flash the layout demo (`DISPLAY_DEMO=1`, OLED) |
| `pio run -e epd -t upload` | legacy e-ink build (`DISPLAY_TYPE=DISPLAY_TYPE_EPD_GDEY042T81`, pulls in GxEPD2) |
| `pio project init --ide vscode` | regenerate IntelliSense after changing `platformio.ini` |

A healthy boot (Arduino `log_i` format; numbers and the `...` parts vary, the phrases are the ones the code prints) looks like:

```
[  1512][I][main.cpp:..] setup(): boat-motor 0.1.0 (built Sep  4 2026 19:35:02) reset=POWERON
[  1513][I][main.cpp:..] logConfig(): cfg CAN : tx=3 rx=2 500 kbit/s mode=NORMAL(ack-only) rxq=64 vesc_id=-1 poles=14
[  1514][I][main.cpp:..] logConfig(): cfg OLED: sda=22 scl=23 rst=14 addr=0x3C i2c=400000 Hz 128x64 rot=0 period=250 ms dim_after=600000 ms
[  1515][I][main.cpp:..] logConfig(): cfg GNSS: rx=4 tx=5 rate=1000 ms unit=kn rxbuf=2048
[  1530][I][can_vesc.cpp:..] ...: CAN: driver installed, state RUNNING: mode=NORMAL (ACK-only, never transmits) bitrate=500k tx=GPIO3 rx=GPIO2 ...
[  1535][I][gnss_ubx.cpp:..] ...: GNSS: Serial1 (HP UART1) RX=GPIO4 TX=GPIO5 ring=2048 B, first baud 38400
[  1540][I][esp32-hal-i2c-ng.c:..] i2cInit(): Initializing I2C Master: num=0 sda=22 scl=23 freq=400000
[  1575][I][display_oled.cpp:..] ...: OLED: init ok (SSD1309 128x64, I2C 0x3C @ 400000 Hz, sda=22 scl=23 rst=14)
[  1610][I][display_oled.cpp:..] ...: OLED: task started (period 250 ms, demo=0)
[  1611][I][main.cpp:..] setup(): started: can=1 gnss=1 display=1
[  4102][I][gnss_ubx.cpp:..] ...: GNSS: UBX detected at 9600 baud (MON-VER reply)
[  4310][I][gnss_ubx.cpp:..] ...: GNSS: PROTVER 18.00 -> legacy CFG configuration
[  4520][I][gnss_ubx.cpp:..] ...: GNSS: NMEA off: 8/8 ACKed
[  4530][I][gnss_ubx.cpp:..] ...: GNSS: CFG-RATE meas=1000 nav=1: ACK
[  4540][I][gnss_ubx.cpp:..] ...: GNSS: CFG-MSG NAV-PVT on: ACK
[  5000][I][can_vesc.cpp:..] ...: VESC id=74 frames=212 other=0 | erpm=0 rpm=0 duty=0.000 Im=0.0A(0.0) | Iin=0.2A(0.2) Vin=50.1V(50.1) P=10W | Tfet=41.0C Tmot=n/a | ... | age s1=12ms s2=340ms s3=341ms s4=15ms s5=9ms s6=never | can=RUN tec=0 rec=0 ...
[  5001][I][gnss_ubx.cpp:..] ...: GNSS phase=RUN baud=9600 protver=18.00 cfg=1 fix=3 ok=1 sats=9 spd=0.00kn sAcc=0.40m/s ... age=310ms good=57 bad=0 redetect=0
[ 10000][I][main.cpp:..] loop(): SYS ... (uptime, heap, min heap, reset reason, CAN health, disp ok= refreshes= skipped= dim=)
```

An M9/M10 receiver logs `GNSS: PROTVER 34.10 -> VALSET configuration` and three `GNSS: VALSET ...: ACK` lines instead of the CFG-MSG/CFG-RATE ones. Before a VESC is connected the VESC line prints `never` for every age, `can=RUN` with `frames=0`, the cells show `--` and the CAN line `CAN idle`; that is expected. The LED blinks at 1 Hz while VESC data is fresh and slowly otherwise. `OLED: refresh N ms (#N, stack free min N B)` per frame needs `CORE_DEBUG_LEVEL=4`.

### Troubleshooting

| Symptom | Check |
|---|---|
| Panel dark; boot log `OLED: no response at 0x3C (check wiring/address jumper), Wire error 2`, then `OLED: still no response at 0x3C (retry N, Wire error 2)` every 5 s | error 2 = nothing ACKs at that address: the module's DC/SA0 jumper selects 0x3D (`-DOLED_I2C_ADDR=0x3D`), SDA/SCL swapped, module still in SPI mode (solder resistors), CS not grounded, module unpowered |
| Same lines with `Wire error 5` | timeout: the bus never idles high: no pull-ups, SDA or SCL shorted, module unpowered. `Wire error 4` = the bus itself failed to start (`OLED: Wire.begin(sda=22, scl=23, 400000 Hz) failed`) |
| `OLED: init ok` but the panel stays dark or dies at random | RES floating (wire GPIO14 or tie RES to 3V3); VCC not 3.3 V; a `-DOLED_CONTRAST` override near 0 |
| `OLED: lost contact at 0x3C (Wire error N), re-initialising` | loose connector or a brown-out that reset the controller; the task re-initialises the panel by itself. A few HAL `i2c_master_transmit failed` lines for the frame that was in flight are expected |
| Garbage, an offset or mirrored image, only half the panel lit | an SSD1306 module (different init sequence, 128x32 variants; this build is 128x64 SSD1309 only) or an `OLED_WIDTH` / `OLED_HEIGHT` override. Upside down: `-DOLED_ROTATION=2` |
| Frames missing or flicker at 400 kHz | pull-ups missing or long wiring; `-DOLED_I2C_HZ=100000` |
| Legacy e-ink (`pio run -e epd`) stays blank, or every refresh takes 10 s with `Busy Timeout!` / `display: BUSY timeout during refresh` | DESPI-C02 RESE switch on "3" / "2.2 Ohm"; CS, DC, RST, BUSY wiring (GPIO1, 8, 14, 18); 3.3 V |
| `can=RUN` but `frames=0` and every age `never` in the VESC log line, cells `--`, CAN line `CAN idle` | status messages not enabled in VESC Tool (section 3); bitrate mismatch (`CAN_BITRATE_KBPS` vs VESC Tool); RXD level shifting missing or the divider output too low; TXD not wired (no ACK, the VESC then stalls, see section 3) |
| Repeated `CAN: BUS_OFF`, `CAN: error passive` or `CAN: N bus error alert(s)` | termination (60 Ohm across CANH/CANL), CANH/CANL swapped, grounds not commoned, RS not grounded, bitrate mismatch |
| `CAN: RX queue full` warnings | something floods the bus; raise `CAN_RX_QUEUE_LEN` |
| GNSS stuck in AUTOBAUD, log repeats `GNSS: no UBX reply at any baud [38400:0B/$0 9600:...]` | `0B` at every baud: no bytes at all, so TX/RX swapped (module TX must go to GPIO4), power or ground. Bytes and `$` counts at one baud: an NMEA talker that ignores UBX input on this port (re-enable the UBX input protocol in u-center). Nothing sensible at any rate: a baud not in `GNSS_BAUDS` (add it) or 5 V logic |
| GNSS detected, `NO FIX` for minutes | antenna indoors; a cold first fix can take a few minutes outdoors |
| Link error: duplicate `app_main` | a leftover `src/main.c` from the ESP-IDF template; delete it and `rm -rf .pio` |
| `#error This firmware targets the ESP32-C6 with the Arduino core` | the platform URL in `platformio.ini` was changed or the wrong env is selected |

## 7. Changing pins and tunables

Everything lives in `include/config.h` as `#ifndef NAME / #define NAME value`. Either edit the file or override from `platformio.ini` without touching it, in your own environment that inherits the main one (the same pattern the `demo` and `epd` envs use):

```ini
[env:myboat]
extends = env:dfrobot_firebeetle2_esp32c6
build_flags =
    ${env:dfrobot_firebeetle2_esp32c6.build_flags}
    -DPIN_CAN_TX=7 -DPIN_CAN_RX=6
    -DSPEED_UNIT_KNOTS=0 -DOLED_I2C_ADDR=0x3D
```

Then build with `pio run -e myboat -t upload`.

Compile-time checks stop you from using GPIO12/13, from sharing a GPIO between two peripherals (only the pins of the selected display take part: the OLED deliberately reuses the e-ink's SPI pins), from putting the legacy BUSY on 8/9/15 and from an unsupported CAN bitrate. `src/display_oled.cpp` adds `static_assert`s for 128x64, `OLED_ROTATION` 0 or 2, `PIN_OLED_RST` in -1..127, contrast 0..255 and `OLED_I2C_HZ` in 1..1000000 (the bus clock must equal the configured value exactly, or the library's clock bracketing stops being free).

| Group | Defines |
|---|---|
| Display selection | `DISPLAY_TYPE` (`DISPLAY_TYPE_OLED_SSD1309`, the default, or `DISPLAY_TYPE_EPD_GDEY042T81`), `DISPLAY_DEMO` |
| OLED | `PIN_OLED_SCL`, `PIN_OLED_SDA`, `PIN_OLED_RST` (-1 = not wired), `OLED_I2C_ADDR`, `OLED_I2C_HZ`, `OLED_WIDTH`, `OLED_HEIGHT`, `OLED_ROTATION`, `OLED_PERIOD_MS`, `OLED_CONTRAST`, `OLED_IDLE_DIM_MS`, `OLED_IDLE_CONTRAST` |
| CAN / VESC | `PIN_CAN_TX`, `PIN_CAN_RX`, `CAN_BITRATE_KBPS` (125/250/500/1000), `CAN_LISTEN_ONLY`, `CAN_RX_QUEUE_LEN`, `VESC_CAN_ID` (-1 = any), `VESC_MOTOR_POLES`, `VESC_STALE_R1_MS`, `VESC_STALE_R2_MS`, `CAN_EMA_ALPHA`, `CAN_HEALTH_LOG_MS`, `CAN_LOG_RAW_FRAMES` |
| GNSS | `PIN_GNSS_RX`, `PIN_GNSS_TX`, `GNSS_BAUDS`, `GNSS_RX_BUFFER`, `GNSS_RATE_MS`, `GNSS_DYNMODEL_SEA`, `GNSS_STALE_MS`, `GNSS_REDETECT_MS`, `GNSS_ACK_TIMEOUT_MS`, `GNSS_MAX_SACC_MM_S`, `SPEED_MIN_SHOW`, `SPEED_UNIT_KNOTS` |
| Tasks / watchdog / logging | `TASK_PRIO_CAN`, `TASK_PRIO_GNSS`, `TASK_PRIO_DISP`, `WDT_TIMEOUT_MS`, `HB_MAX_CAN_MS`, `HB_MAX_GNSS_MS`, `HB_MAX_DISP_MS`, `LOG_VESC_MS`, `LOG_GNSS_MS`, `LOG_SYS_MS`, `SERIAL_BOOT_DELAY_MS`, `PIN_LED`, `FW_VERSION` |
| E-paper (legacy, `epd` env only) | `PIN_EPD_SCK`, `PIN_EPD_MOSI`, `PIN_EPD_CS`, `PIN_EPD_DC`, `PIN_EPD_RST`, `PIN_EPD_BUSY`, `EPD_SPI_HZ`, `EPD_RESET_MS`, `EPD_ROTATION`, `EPD_FAST_FULL_UPDATE`, `EPD_DIAG_BAUD`, `DISPLAY_PERIOD_MS`, `EPD_FULL_EVERY_N_PARTIALS`, `EPD_FULL_EVERY_MS`, `EPD_POWEROFF_IDLE_MS`, `EPD_HIBERNATE_IDLE_MS` |
| Advanced (defaults are fine) | `CAN_TASK_STACK`, `CAN_INSTALL_RETRY_MS`, `CAN_RX_TIMEOUT_MS`, `CAN_ERR_LOG_MIN_MS`, `CAN_ERR_WARN_LEVEL`, `GNSS_TASK_STACK`, `GNSS_AUTOBAUD_LISTEN_MS`, `GNSS_MONVER_TIMEOUT_MS`, `GNSS_AUTOBAUD_RETRY_MS`; at the top of `src/display_oled.cpp`: `OLED_TASK_STACK` (6144), `OLED_INIT_RETRY_MS` (5000), `OLED_I2C_TIMEOUT_MS` (50) |

Notes: the OLED layout is hard-coded for 128x64 in `OLED_ROTATION` 0 or 2. `OLED_PERIOD_MS` may go down to about 50 (a frame is ~26 ms), but 250 is plenty for a 1 Hz GNSS and EMA-smoothed currents. `WDT_TIMEOUT_MS` must stay above GxEPD2's 10 s busy timeout in the `epd` env. `GNSS_BAUDS` is a brace list and is easiest to change in `config.h` itself.

## 8. Passivity and safety notes

- The firmware never calls `twai_transmit`: `grep -rn twai_transmit src/` returns nothing, and `candump` on a second CAN adapter shows no frames from the ESP32. It cannot change a VESC setting or command the motor. It does ACK (section 3).
- Displayed and logged power is signed: `v_in x current_in`, negative while regenerating (the water drives the motor). VESC Tool's realtime power stat is an absolute value, so the two can differ in sign, not in magnitude.
- The cells are EMA-smoothed (`CAN_EMA_ALPHA`) and may be up to `VESC_STALE_R1_MS` old. This is a dashboard, not a protection device: rely on the VESC's own limits and a BMS for that.
- OLED modules are specified for roughly -40..70 C (vendor dependent; some 1.54" boards only -20..60 C), far wider than the legacy e-ink's 0..50 C, but the glass, the FPC and the boost converter must stay dry: condensation on a boat needs an enclosure or conformal coating. In direct sun a ~110 cd/m2 OLED is hard to read; a sunshade helps more than contrast.
- Burn-in is the OLED's wear mechanism: a dashboard that shows the same numbers for hours is the worst case. Keep `OLED_IDLE_DIM_MS` enabled, do not add filled boxes or bars to the layout, and expect the panel to lose brightness over thousands of hours.
- A hung task reboots the board after `WDT_TIMEOUT_MS` (20 s); the next banner reports `reset=TASK_WDT`. The OLED keeps its last image until the firmware pulses RES about 1.5 s into the reboot, then shows the boot frame.
- Apart from the CAN transceiver everything is 3.3 V. Nothing on the header tolerates 5 V, and neither does an OLED module whose pull-ups would then sit at 5 V.

## 9. Future work (out of scope for this version)

- Active VESC polling with `COMM_GET_VALUES` over CAN (`PROCESS_SHORT_BUFFER` request, `FILL_RX_BUFFER` / `PROCESS_RX_BUFFER` reassembly, CRC-16/XMODEM) for fault codes, id/iq and per-MOSFET temperatures. It requires transmitting, so it ends the strict passivity.
- A proper large-digit font for the speed (the scaled 5x7 GFX font is legible but blocky) and a second page (FET temperature, Ah/Wh, GNSS details) on a button.
- Burn-in mitigation beyond dimming: `DISPLAYOFF` (0xAE) after a long idle, or shifting the layout by a pixel every few minutes.
- OTA updates (`default.csv` already has two app slots).
- Deep sleep and the LP-UART for the GNSS receiver.
- Persistent logging (Ah/Wh trip counters, track).
- Port from the legacy `driver/twai.h` to `esp_twai_onchip.h` when the Arduino core moves to IDF 6; only `src/can_vesc.cpp` touches the driver.
