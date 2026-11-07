// vesc-dashboard configuration
// ----------------------------------------------------------------------------
// THE one header you edit to match your wiring and preferences. Nothing else in
// the project hard-codes a pin. Every value is wrapped in #ifndef so it can also
// be overridden from platformio.ini:  build_flags = -DPIN_CAN_TX=7
//
// Display: 128x64 SSD1309 OLED over I2C (see the OLED section).
//
// Board: DFRobot FireBeetle 2 ESP32-C6 (DFR1075) by default; BOARD_S3_ZERO 1 selects
// the Waveshare ESP32-S3-Zero (see the Board block below). FireBeetle: silkscreen labels equal GPIO
// numbers. Pins to leave alone: 12/13 (USB), 16/17 (UART0 boot console, GPIO16
// is driven by UART0 TX the whole run; with VESC_LINK_UART, UART0 is the VESC link
// on PIN_VESC_UART_TX/RX and GPIO16 may still mirror its TX), 9 (BOOT button), 0
// (battery ADC, not on the header). Strapping pins: 4/5 (SDIO edge only, harmless),
// 8/9 (boot mode), 15 (JTAG select, on-board LED). Never put a peripheral-driven
// INPUT (a sensor IRQ or busy line, say) on 8, 9 or 15.
// ----------------------------------------------------------------------------
#pragma once

#include <stdint.h>

#ifndef FW_VERSION
#define FW_VERSION "0.1.0"
#endif

// ============================================================================
// Board
// ============================================================================
// 0 = DFRobot FireBeetle 2 ESP32-C6 (every env except s3zero_*): the defaults further down.
// 1 = Waveshare ESP32-S3-Zero (env s3zero_deadman_uart, README section 2b). Same GPIO numbers
//     as the FireBeetle wherever the module has them: the block below only moves the OLED
//     (there is no GPIO22/23) and the status LED (the on-board WS2812), and puts the UART
//     VESC link on UART2; it also runs both cores (TASK_CORE_*). Its pins are #ifndef too, so
//     -D overrides still win.
#ifndef BOARD_S3_ZERO
#define BOARD_S3_ZERO 0
#endif

#if BOARD_S3_ZERO
// Waveshare ESP32-S3-Zero: ESP32-S3FH4R2 (4 MB flash, 2 MB PSRAM left unused), native USB
// only (GPIO19/20), WS2812 on GPIO21, BOOT button on GPIO0 (not on the edge). Front edge:
// 5V / GND / 3V3 / GP1..GP6 on one side, TX(43) / RX(44) / GP13..GP7 on the other, GP14..16
// as pads at the bottom; GP17, 18, 38..42 and 45 are pads on the back. Strapping pins: 0, 3,
// 45, 46. GPIO3 only selects the JTAG source when the STRAP_JTAG_SEL eFuse is burned (it is
// not from the factory), so the VESC TX output there is fine. GPIO26..37 belong to the
// in-package flash / PSRAM and are not broken out.
#define BOARD_NAME "Waveshare ESP32-S3-Zero"
#define BOARD_GPIO_MAX 48
#define BOARD_USB_DM 19
#define BOARD_USB_DP 20
#define BOARD_BOOT_BUTTON 0
#define BOARD_CUT_PIN_FORBIDDEN(p) ((p) == 0 || (p) == 3 || (p) == 45 || (p) == 46) // strapping pins
#define BOARD_PIN_RESERVED(p) ((p) >= 26 && (p) <= 37)                              // flash / PSRAM
#ifndef PIN_OLED_SDA
#define PIN_OLED_SDA 8 // GP8 -> OLED SDA
#endif
#ifndef PIN_OLED_SCL
#define PIN_OLED_SCL 9 // GP9 -> OLED SCL
#endif
#ifndef PIN_OLED_RST
#define PIN_OLED_RST 10 // GP10 -> OLED RES (optional; -1 if the module has no reset pin)
#endif
#ifndef PIN_LED
#define PIN_LED 21 // the on-board WS2812
#endif
#ifndef LED_WS2812
#define LED_WS2812 1
#endif
#ifndef VESC_UART_PORT
#define VESC_UART_PORT 2 // UART2: UART0 and its TX/RX pads (43/44, where the ROM prints its boot log) stay untouched
#endif
// Two cores (sdkconfig.defaults.esp32s3). Core 0 runs the radio: the BLE controller, the NimBLE
// host and the ble task. Core 1 runs app_main and every other task of this firmware, so the
// interrupts they install land there too. The firmware's own tasks therefore share one core with
// exactly the priorities they have on the single-core C6; only the radio moved away.
#define TASK_CORE_APP 1
#define TASK_CORE_RADIO 0
#else
// DFRobot FireBeetle 2 ESP32-C6: the pins to leave alone are in the header comment above.
#define BOARD_NAME "DFRobot FireBeetle 2 ESP32-C6"
#define BOARD_GPIO_MAX 30
#define BOARD_USB_DM 12
#define BOARD_USB_DP 13
#define BOARD_BOOT_BUTTON 9
#define BOARD_CUT_PIN_FORBIDDEN(p) ((p) == 8 || (p) == 9 || (p) == 15) // strapping pins 8/9, JTAG select / LED 15
#define BOARD_PIN_RESERVED(p) 0                                          // nothing beyond the USB pair
#ifndef VESC_UART_PORT
#define VESC_UART_PORT 0 // UART0: UART1 is the GNSS and the C6 has no UART2
#endif
#define TASK_CORE_APP 0   // single core: everything on core 0
#define TASK_CORE_RADIO 0
#endif

// ============================================================================
// VESC link: CAN (default) or UART
// ============================================================================
// 0 = CAN (src/can_vesc.cpp): TWAI + transceiver, STATUS_1..6 broadcasts plus a 1 Hz
//     COMM_GET_VALUES_SELECTIVE poll for the values no broadcast carries.
// 1 = UART (src/uart_vesc.cpp, env 'deadman_uart'): no transceiver; the VESC's COMM-header
//     UART is polled with COMM_GET_VALUES every VESC_POLL_MS. Everything the dashboard
//     shows is in that reply except STATUS_6 (ADC1..3, PPM), which stays "--". The CAN
//     settings below are then unused. See README section 3b.
#ifndef VESC_LINK_UART
#define VESC_LINK_UART 0
#endif

// ============================================================================
// CAN bus (MCP2551 transceiver -> VESC)
// ============================================================================
// ESP32-C6 GPIOs are NOT 5 V tolerant (absolute max VDD + 0.3 V). The MCP2551
// is a 5 V part: its RXD output must be level-shifted (1k/2.2k divider gives
// ~3.4 V at 5.0 V, marginal at 5.25 V) or, better, use a 3.3 V-IO transceiver
// (MCP2562 with VIO=3V3, TJA1051T/3, SN65HVD230). Its TXD input accepts 3.3 V
// directly; add 1 kOhm in series (TXD has a 25 kOhm pull-up to 5 V).
#ifndef PIN_CAN_TX
#define PIN_CAN_TX 3 // GPIO3 (silkscreen "3"/A2) -> transceiver TXD. Must be wired: it carries the ACK bit.
#endif
#ifndef PIN_CAN_RX
#define PIN_CAN_RX 2 // GPIO2 (silkscreen "2"/A1) <- transceiver RXD (level-shifted!)
#endif
#ifndef CAN_BITRATE_KBPS
#define CAN_BITRATE_KBPS 500 // VESC default CAN_BAUD_500K. Supported: 125, 250, 500, 1000. Must match VESC Tool "CAN Baud Rate".
#endif
// 0 = normal mode: the controller acknowledges frames; the only thing this firmware ever transmits is the
//     optional COMM_GET_VALUES_SELECTIVE poll (VESC_POLL_MS below; 0 = strictly passive, never transmits).
//     REQUIRED when the VESC and this board are the only two nodes: the VESC's bxCAN has no automatic-retransmit
//     limit, so with a non-ACKing listener it goes error-passive and repeats one stale frame forever.
// 1 = listen-only (no ACK at all): only if another ACKing node (2nd VESC, BMS, VESC Tool adapter) exists;
//     requires VESC_POLL_MS 0 (a listen-only controller cannot transmit).
#ifndef CAN_LISTEN_ONLY
#define CAN_LISTEN_ONLY 0
#endif
#ifndef CAN_RX_QUEUE_LEN
#define CAN_RX_QUEUE_LEN 64 // RX queue between the TWAI ISR and canTask (frames). 6 status frames arrive as a burst every 20 ms.
#endif
#ifndef VESC_CAN_ID
#define VESC_CAN_ID -1 // -1 = accept any VESC id and lock onto the first one seen; 0..254 = only this id (VESC Tool "VESC ID")
#endif
#ifndef VESC_MOTOR_POLES
#define VESC_MOTOR_POLES 10 // VESC Tool > Motor Settings > Additional Info > Motor Poles. mech rpm = erpm / (poles/2). Logging only.
#endif
#ifndef VESC_STALE_R1_MS
#define VESC_STALE_R1_MS 2000 // STATUS 1/4/5 ("Rate 1", default 50 Hz) older than this are shown as "--"
#endif
#ifndef VESC_STALE_R2_MS
#define VESC_STALE_R2_MS 5000 // STATUS 2/3/6 ("Rate 2", default 5 Hz) older than this are stale (logs)
#endif
#ifndef CAN_EMA_ALPHA
#define CAN_EMA_ALPHA 0.15f // exponential smoothing of displayed I_in / I_motor / V_in (STATUS_4 current_in is unfiltered). 1.0f = off
#endif
#ifndef CAN_HEALTH_LOG_MS
#define CAN_HEALTH_LOG_MS 10000 // period of the twai_node_get_info() health snapshot (TEC/REC/bus errors)
#endif
#ifndef CAN_LOG_RAW_FRAMES
#define CAN_LOG_RAW_FRAMES 0 // 1 = debug-log every accepted raw frame (needs -DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG)
#endif

// ============================================================================
// Active VESC polling (COMM_GET_VALUES_SELECTIVE over CAN)
// ============================================================================
// The status frames carry most data passively. Fault code, per-MOSFET
// temperatures, id/iq, vd/vq and the absolute tachometer are only available by
// asking the VESC (CAN_PACKET_PROCESS_SHORT_BUFFER carrying COMM_GET_VALUES_SELECTIVE,
// answered as FILL_RX_BUFFER/PROCESS_RX_BUFFER frames). This is the ONLY place the
// firmware transmits on the bus. Set VESC_POLL_MS 0 for a strictly passive (ACK-only) node.
#ifndef VESC_POLL_MS
#if VESC_LINK_UART
#define VESC_POLL_MS 100 // UART: the poll is the ONLY data source (79-byte reply, ~7 ms at 115200), so 10 Hz; must be > 0
#else
#define VESC_POLL_MS 1000 // request period in ms; 0 = never transmit anything
#endif
#endif
#ifndef CAN_OWN_ID
#define CAN_OWN_ID 120 // our controller id on the VESC bus: 1..254, must differ from every VESC id (255 = broadcast)
#endif
#ifndef VESC_POLL_TIMEOUT_MS
#if VESC_LINK_UART
#define VESC_POLL_TIMEOUT_MS 50 // UART: no complete reply within this long = timeout (a reply takes ~7 ms on the wire)
#else
#define VESC_POLL_TIMEOUT_MS 500 // an incomplete reply is discarded after this long
#endif
#endif
#ifndef VESC_EXT_STALE_MS
#define VESC_EXT_STALE_MS 3000 // polled values older than this are shown as "--"
#endif
#ifndef CAN_TX_QUEUE_LEN
#define CAN_TX_QUEUE_LEN 4 // TWAI node TX queue depth (>= 1); only the poll requests ever use it
#endif

// ============================================================================
// VESC over UART (VESC_LINK_UART 1 only; src/uart_vesc.cpp, include/vesc_uart.h)
// ============================================================================
// HP UART VESC_UART_PORT (board block: UART0 on the C6, UART2 on the S3-Zero; UART1 is the
// GNSS, the console is USB-Serial/JTAG) on the two pins the CAN transceiver used. 3.3 V logic on both sides, no transceiver, 1 kOhm in series in
// each line. On the VESC: the COMM-header UART, which only runs while App to Use includes
// UART ("ADC and UART", "PPM and UART" or "UART"). The only thing ever sent is the 6-byte
// read-only COMM_GET_VALUES request.
#ifndef PIN_VESC_UART_TX
#define PIN_VESC_UART_TX 3 // GPIO3 -> 1k -> VESC RX (COMM header; PB11 on the 75_300)
#endif
#ifndef PIN_VESC_UART_RX
#define PIN_VESC_UART_RX 2 // GPIO2 <- 1k <- VESC TX (PB10 on the 75_300); internal pull-up, so an unpowered VESC reads idle
#endif
#ifndef VESC_UART_BAUD
#define VESC_UART_BAUD 115200 // must equal VESC Tool > App Settings > UART > Baudrate (default 115200)
#endif
#ifndef VESC_UART_RX_BUFFER
#define VESC_UART_RX_BUFFER 1024 // UART0 driver RX ring (bytes); one reply is 79
#endif
#ifndef VESC_UART_EMA_ALPHA
#define VESC_UART_EMA_ALPHA 0.5f // display smoothing of I_in / I_motor / V_in per reply. The UART currents already are
                                 // averages over the poll period, so lighter than CAN_EMA_ALPHA at 50 Hz. 1.0f = off
#endif

// ============================================================================
// Screens, button, clock
// ============================================================================
#ifndef PIN_BUTTON
#define PIN_BUTTON 7 // silkscreen "7"/INT: momentary button to GND (internal pull-up). -1 = no button.
                     // BOARD_BOOT_BUTTON (9 on the FireBeetle, 0 on the S3-Zero) = use the on-board BOOT button instead
                     // (fine at runtime; holding it during reset enters download mode)
#endif
#ifndef BUTTON_ACTIVE_LOW
#define BUTTON_ACTIVE_LOW 1 // 1 = pressed reads LOW (button to GND, internal pull-up); 0 = pressed reads HIGH (button to 3V3, internal pull-down)
#endif
#ifndef BUTTON_DEBOUNCE_MS
#define BUTTON_DEBOUNCE_MS 30
#endif
#ifndef BUTTON_LONG_PRESS_MS
#define BUTTON_LONG_PRESS_MS 1500 // long press -> back to the main screen
#endif
#ifndef SCREEN_AUTO_RETURN_MS
#define SCREEN_AUTO_RETURN_MS 0 // 0 (default) = stay on the selected screen; > 0 = return to the main screen this long after
                                // the last button event. A long press always returns to the main screen.
#endif
#ifndef TIME_UTC_OFFSET_MIN
#define TIME_UTC_OFFSET_MIN 0 // displayed clock = GNSS UTC + this many minutes (e.g. 120 for UTC+2). No DST logic.
#endif

// ============================================================================
// Over-the-air firmware update (src/ota.cpp, README section 7)
// ============================================================================
// Holding the button for OTA_LONG_PRESS_MS reboots into UPDATE MODE: the dashboard brings up
// its own WPA2 access point (a fresh 8-digit passphrase per session, shown on the OLED), serves
// an upload page at http://192.168.4.1 and writes the posted .bin to the spare app slot. Update
// mode starts nothing but the OLED, the button and the Wi-Fi: no VESC link, no GNSS, no BLE, and
// in a dead-man build the cut is HELD for its whole duration (nothing watches the tags then).
// It is refused while the motor turns, the boat moves or the dead-man alarm is up.
#ifndef OTA_ENABLE
#if PIN_BUTTON >= 0
#define OTA_ENABLE 1 // the hold is the only way in, so no button = no OTA
#else
#define OTA_ENABLE 0
#endif
#endif
#ifndef OTA_LONG_PRESS_MS
#define OTA_LONG_PRESS_MS 5000 // hold time; the long press (BUTTON_LONG_PRESS_MS) of the same hold fires first
#endif
#ifndef OTA_AP_SSID
#define OTA_AP_SSID "VESC-DASH-OTA" // access point name (<= 18 chars: it shares an OLED row with "AP ")
#endif
#ifndef OTA_AP_TX_POWER_QDBM
#define OTA_AP_TX_POWER_QDBM 52 // 0.25 dBm units, 8..84; the driver quantises (52 = 13 dBm). Wi-Fi at full power is the
                                // largest current peak on the board, and a phone at the boat needs little margin
#endif
#ifndef OTA_IDLE_TIMEOUT_MS
#define OTA_IDLE_TIMEOUT_MS 300000 // update mode reboots back to normal after this long without an upload in progress
#endif
#ifndef OTA_CONFIRM_MS
#define OTA_CONFIRM_MS 60000 // a freshly flashed image is on probation: it is marked valid once it has run this long with
                             // every task heartbeat fresh. A panic, a watchdog reset or a power cycle before that boots
                             // the previous image again (bootloader rollback)
#endif
#ifndef OTA_MAX_RPM
#define OTA_MAX_RPM 100 // update mode is refused while fresh VESC data shows more than this (mechanical rpm)
#endif
#ifndef OTA_MAX_SPEED_MM_S
#define OTA_MAX_SPEED_MM_S 500 // ... or a fresh, accurate GNSS fix shows more than this (500 mm/s = ~1 kn)
#endif
#ifndef OTA_REFUSED_SHOW_MS
#define OTA_REFUSED_SHOW_MS 3000 // how long the "UPDATE REFUSED" screen stays up after a refused hold
#endif

// ============================================================================
// Trip / efficiency (src/trip.cpp, EFFICIENCY screen)
// ============================================================================
// Distance = GNSS ground speed integrated while moving; energy = VESC watt-hour
// counters (STATUS_3), falling back to v_in x current_in integration while
// STATUS_3 is not being received. Efficiency = Wh per distance unit.
#ifndef TRIP_PERIOD_MS
#define TRIP_PERIOD_MS 200 // integrator tick (matches the 5 Hz GNSS rate)
#endif
#ifndef EFF_WINDOW_S
#define EFF_WINDOW_S 10 // "now" efficiency = last this many seconds
#endif
#ifndef EFF_MIN_SPEED_MM_S
#define EFF_MIN_SPEED_MM_S 500 // below this ground speed no distance is integrated (GNSS drift at the mooring); 500 mm/s ~ 1 kn
#endif
#ifndef EFF_MIN_DIST_M
#define EFF_MIN_DIST_M 10 // a window / trip shorter than this shows "--" instead of a meaningless ratio
#endif
#ifndef EFF_UNIT_KM
#define EFF_UNIT_KM 0 // 0 = follow the speed unit (knots -> Wh/NM, km/h -> Wh/km); 1 = always Wh/km
#endif

// ============================================================================
// Dead-man's switch (BLE beacon -> the VESC's own kill-switch input; src/deadman.cpp)
// ============================================================================
// A BLE tag worn by the helmsman is watched by the board's own radio (src/ble_scan.cpp). When its
// advertisements stop arriving for DEADMAN_TIMEOUT_MS the firmware pulls PIN_DEADMAN_CUT
// low, which is wired IN PARALLEL with the boat's mechanical kill switch on the VESC's
// ADC2 input (VESC Tool -> App Settings -> General -> Kill Switch Mode = ADC2 Low).
// Fail-passive by construction: the pin is open-drain, so the firmware can only ever ADD
// a kill. A dead, hung or rebooting dashboard leaves the line alone and the mechanical
// switch remains the primary protection - it can never strand the boat by failing.
// The VESC samples that input every 10 ms against a hard 1.65 V threshold (no hysteresis)
// and releases the motor immediately, overriding the local PPM/ADC throttle; it reports
// the state back as VESC_STATUS_KILL_SW, which this firmware already polls and displays,
// so the cut is verified end to end (DEADMAN_CONFIRM_*). Nothing is sent over CAN.
#ifndef DEADMAN_ENABLE
#define DEADMAN_ENABLE 0 // 1 = build the BLE-beacon dead-man's switch and the NimBLE scan it runs on (needs sdkconfig.defaults.ble, env 'deadman'); 0 = no BLE code at all
#endif
#ifndef PIN_DEADMAN_CUT
#define PIN_DEADMAN_CUT 1 // open-drain output wired in PARALLEL with the mechanical kill switch on VESC ADC2.
                          // High-Z = permit (the mechanical switch drives the line), driven low = kill.
                          // GPIO1, 8 and 18 are the free header pins; NEVER a strapping pin (8/9/15): a glitch
                          // low at reset would be a spurious engine cut. Wiring (README section 5), the VESC on
                          // ADC2 Low either way: a switch CLOSED while running sits in the 3.3 V leg with a series
                          // resistor Rs <= Rpd/2; one OPEN while running goes from ADC2 to GND with a pull-up
                          // (1k..2.2k) to the VESC's 3V3. Both keep the permit level clear of the VESC's hard
                          // 1.65 V threshold, which has no hysteresis. -1 = no pin (logic only, for soak tests).
#endif
// Up to two tags. Which of them actually guard the motor is decided ONCE, at the moment
// protection begins: the first tag to arm opens a DEADMAN_ENROL_MS window, any other tag
// that arms inside it joins, and the set is then frozen for the run. A tag that turns up
// later - the one left in the car, still in range at the dock - can never join and
// silently keep the switch alive. With two tags enrolled the cut comes only when BOTH
// have gone quiet; either one aboard keeps the motor running.
#ifndef DEADMAN_BEACON_ADDR
#define DEADMAN_BEACON_ADDR "" // tag 1 MAC, fixed public address. Mandatory: there is no discovery mode
#endif
#ifndef DEADMAN_BEACON_ADDR2
#define DEADMAN_BEACON_ADDR2 "" // tag 2 MAC, or "" for a single-tag boat. A malformed non-empty address is fatal (a typo
                                // must never silently disable a tag); the two addresses must differ
#endif
#ifndef DEADMAN_ENROL_MS
#define DEADMAN_ENROL_MS 5000 // how long the enrolment window stays open after the FIRST tag arms, so a second tag
                              // that is also aboard can join. Longer = more forgiving of a slow tag; too long and a
                              // tag that is merely nearby at the dock gets in
#endif
#ifndef DEADMAN_TIMEOUT_MS
#define DEADMAN_TIMEOUT_MS 5000 // no matching advert for this long while armed -> assert the cut and latch it
#endif
#ifndef DEADMAN_WARN_MS
#define DEADMAN_WARN_MS 2000 // ... after this long the screen counts down (state GRACE); the motor is still permitted
#endif
#ifndef DEADMAN_ARM_REPORTS
#define DEADMAN_ARM_REPORTS 3 // adverts needed to arm: a tag that was never aboard must not be able to trip anything
#endif
#ifndef DEADMAN_ARM_WINDOW_MS
#define DEADMAN_ARM_WINDOW_MS 5000 // ... they must all arrive inside this window
#endif
#ifndef DEADMAN_ARM_RSSI
#define DEADMAN_ARM_RSSI -75 // ... and the last one must be at least this strong (dBm): arming on a tag in the car park means a trip at the dock
#endif
#ifndef DEADMAN_RESET_FRESH_MS
#define DEADMAN_RESET_FRESH_MS 1000 // a reset is refused unless the tag was seen this recently: never restart while the person is still in the water
#endif
#ifndef DEADMAN_BOOT_CUT
#define DEADMAN_BOOT_CUT 0 // 0 = the motor runs until the tag has been seen once (you can leave without the tag, with no protection; the screen says so)
                           // 1 = cut until the tag is seen (you cannot forget the tag, but a dead tag battery immobilises the boat until the dashboard is powered down)
#endif
#ifndef DEADMAN_CONFIRM_ENABLE
#define DEADMAN_CONFIRM_ENABLE 1 // 1 = close the loop: after asserting the cut, the VESC must report VESC_STATUS_KILL_SW back over the poll
#endif
#ifndef DEADMAN_CONFIRM_MS
#define DEADMAN_CONFIRM_MS 3500 // ... within this long, else state FAULT (wiring, series resistor, or the VESC lost its ADC2 kill-switch config)
#endif

// ============================================================================
// Display
// ============================================================================
#ifndef DISPLAY_DEMO
#define DISPLAY_DEMO 0 // 1 = render synthetic changing values (no CAN/GNSS needed) to validate the layout
#endif

// ============================================================================
// OLED: 128x64 SSD1309 over I2C (native u8g2 on driver/i2c_master.h, HAL in src/u8g2_hal_idf.cpp)
// ============================================================================
// Wired to the FireBeetle's SPI header pins (SCK -> SCL, MOSI -> SDA, RES -> RST); the C6
// routes I2C to any GPIO through its matrix.
// Most modules carry their own I2C pull-ups and accept 3.3 V VCC; check yours.
#ifndef PIN_OLED_SCL
#define PIN_OLED_SCL 23 // silkscreen "23"/SCK  -> OLED SCL (any GPIO works on the C6, routed via the GPIO matrix)
#endif
#ifndef PIN_OLED_SDA
#define PIN_OLED_SDA 22 // silkscreen "22"/MOSI -> OLED SDA
#endif
#ifndef PIN_OLED_RST
#define PIN_OLED_RST 14 // silkscreen "14"/RES  -> OLED RES (optional). -1 if your module has no reset pin / RES tied high.
#endif
#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR 0x3C // 7-bit address; 0x3D on modules with the address jumper moved
#endif
#ifndef OLED_I2C_HZ
#define OLED_I2C_HZ 400000 // I2C clock; SSD1309 is specified to 400 kHz. Drop to 100000 for long/noisy wiring.
#endif
#ifndef OLED_WIDTH
#define OLED_WIDTH 128
#endif
#ifndef OLED_HEIGHT
#define OLED_HEIGHT 64
#endif
#ifndef OLED_ROTATION
#define OLED_ROTATION 0 // 0 = normal, 2 = upside down (u8g2 U8G2_R0 / U8G2_R2; layout assumes landscape)
#endif
#ifndef OLED_PERIOD_MS
#define OLED_PERIOD_MS 250 // display task tick (4 Hz). A full 1 KB frame over 400 kHz I2C takes ~25 ms.
#endif
#ifndef OLED_CONTRAST
#define OLED_CONTRAST 255 // normal contrast 0..255
#endif
#ifndef OLED_IDLE_DIM_MS
#define OLED_IDLE_DIM_MS 600000 // no displayed value changed for this long -> dim the panel (burn-in / power). 0 = never dim
#endif
#ifndef OLED_IDLE_CONTRAST
#define OLED_IDLE_CONTRAST 8 // contrast while dimmed
#endif

// ============================================================================
// GNSS (u-blox compatible receiver, UBX protocol, HP UART1, 3.3 V logic)
// ============================================================================
#ifndef PIN_GNSS_RX
#define PIN_GNSS_RX 4 // silkscreen "4"/LP_RX <- GNSS module TX
#endif
#ifndef PIN_GNSS_TX
#define PIN_GNSS_TX 5 // silkscreen "5"/LP_TX -> GNSS module RX (board has a 499 Ohm series resistor, fine)
#endif
#ifndef GNSS_BAUDS
#define GNSS_BAUDS {115200, 38400, 9600, 57600, 230400} // autobaud order: the target baud first (a receiver configured by us keeps it while powered / battery-backed), then the factory defaults 38400 (M9/M10) and 9600 (M8, MAX-M10S)
#endif
#ifndef GNSS_RX_BUFFER
#define GNSS_RX_BUFFER 2048 // UART1 driver RX ring (bytes). 256 would overflow at 10 Hz while a display frame is pushed (NAV-PVT = 100 B).
#endif
#ifndef GNSS_TARGET_BAUD
#define GNSS_TARGET_BAUD 115200 // after detection the receiver's UART1 is switched to this baud (RAM+BBR); 0 = keep the detected baud. Must be in GNSS_BAUDS.
#endif
#ifndef GNSS_RATE_MS
#define GNSS_RATE_MS 200 // navigation rate: 200 = 5 Hz (safe on every M8/M9/M10 with all constellations); 100 = 10 Hz needs a fast module and GNSS_TARGET_BAUD >= 38400
#endif
#ifndef GNSS_DYNMODEL_SEA
#define GNSS_DYNMODEL_SEA 1 // 1 = set the receiver dynamic model to SEA (5); 0 = leave the receiver default
#endif
#ifndef GNSS_STALE_MS
#define GNSS_STALE_MS 3000 // no valid NAV-PVT for this long -> speed shows "--"
#endif
#ifndef GNSS_REDETECT_MS
#define GNSS_REDETECT_MS 20000 // no valid UBX frame for this long while running -> back to autobaud (module power-cycled / baud lost)
#endif
#ifndef GNSS_ACK_TIMEOUT_MS
#define GNSS_ACK_TIMEOUT_MS 1200 // wait for UBX-ACK after each configuration message
#endif
#ifndef GNSS_MAX_SACC_MM_S
#define GNSS_MAX_SACC_MM_S 2000 // speed is shown as "--" when the receiver's speed accuracy estimate is worse than this (2 m/s)
#endif
#ifndef SPEED_MIN_SHOW
#define SPEED_MIN_SHOW 0.3f // speeds below this (in the display unit) are shown as 0.0 to hide GNSS drift at rest
#endif
#ifndef SPEED_UNIT_KNOTS
#define SPEED_UNIT_KNOTS 0 // 1 = knots (mm/s * 0.00194384), 0 = km/h (mm/s * 0.0036)
#endif

// ============================================================================
// Tasks / watchdog / logging
// ============================================================================
#ifndef TASK_PRIO_CAN
#define TASK_PRIO_CAN 6 // highest: CAN reception must never be starved. With VESC_LINK_UART it is the UART link task's priority
#endif
#ifndef TASK_PRIO_GNSS
#define TASK_PRIO_GNSS 5
#endif
#ifndef TASK_PRIO_DISP
#define TASK_PRIO_DISP 2 // above the supervisor (app_main, priority 1), below the data producers
#endif
#ifndef WDT_TIMEOUT_MS
#define WDT_TIMEOUT_MS 20000 // task watchdog; must exceed the longest legitimate task stall (a stuck I2C bus costs 50 ms per transaction)
#endif
#ifndef HB_MAX_CAN_MS
#define HB_MAX_CAN_MS 5000 // supervisor: a task heartbeat older than this stops the watchdog feed -> reboot
#endif
#ifndef HB_MAX_GNSS_MS
#define HB_MAX_GNSS_MS 5000
#endif
#ifndef HB_MAX_DISP_MS
#define HB_MAX_DISP_MS 30000 // the display task touches its heartbeat every OLED_BUTTON_POLL_MS
#endif
#ifndef LOG_VESC_MS
#define LOG_VESC_MS 1000 // period of the all-fields VESC log line (0 = off)
#endif
#ifndef LOG_GNSS_MS
#define LOG_GNSS_MS 1000 // period of the GNSS log line (0 = off)
#endif
#ifndef LOG_SYS_MS
#define LOG_SYS_MS 10000 // uptime / heap / reset reason / CAN health / display counters (0 = off)
#endif
#ifndef LOG_TRIP_MS
#define LOG_TRIP_MS 5000 // period of the trip / efficiency log line (0 = off)
#endif
#ifndef SERIAL_BOOT_DELAY_MS
#define SERIAL_BOOT_DELAY_MS 1500 // give the USB-Serial/JTAG host time to re-enumerate so the boot banner is visible
#endif
#ifndef PIN_LED
#define PIN_LED 15 // on-board green LED (active high): 1 Hz blink = VESC data fresh, slow blink = no VESC data
#endif
#ifndef LED_WS2812
#define LED_WS2812 0 // 1 = PIN_LED drives one WS2812 RGB LED (the S3-Zero's on-board one): green blink = fresh, red = no data
#endif
#ifndef LED_WS2812_LEVEL
#define LED_WS2812_LEVEL 16 // WS2812 brightness 1..255: full brightness is blinding on a dashboard at night
#endif

// ---- Advanced task tunables (defaults are fine; exposed so nothing is hidden in the .cpp files) ----
#ifndef CAN_TASK_STACK
#define CAN_TASK_STACK 4096 // canTask stack, bytes (also the UART link task's, and HB_MAX_CAN_MS / hb_can gate it the same way)
#endif
#ifndef CAN_INSTALL_RETRY_MS
#define CAN_INSTALL_RETRY_MS 5000 // retry period while creating / enabling the TWAI node keeps failing
#endif
#ifndef CAN_RX_TIMEOUT_MS
#define CAN_RX_TIMEOUT_MS 100 // RX queue wait = heartbeat granularity of canTask
#endif
#ifndef CAN_ERR_LOG_MIN_MS
#define CAN_ERR_LOG_MIN_MS 2000 // rate limit for bus-error / queue-full log lines (counts are aggregated)
#endif
#ifndef CAN_ERR_WARN_LEVEL
#define CAN_ERR_WARN_LEVEL 96 // TEC/REC at or above this turns the periodic health line into a warning
#endif
#ifndef GNSS_TASK_STACK
#define GNSS_TASK_STACK 4096 // gnssTask stack, bytes
#endif
#ifndef GNSS_AUTOBAUD_LISTEN_MS
#define GNSS_AUTOBAUD_LISTEN_MS 1200 // passive listen window per candidate baud
#endif
#ifndef GNSS_MONVER_TIMEOUT_MS
#define GNSS_MONVER_TIMEOUT_MS 1500 // wait for the UBX-MON-VER reply during autobaud / detect
#endif
#ifndef GNSS_AUTOBAUD_RETRY_MS
#define GNSS_AUTOBAUD_RETRY_MS 2000 // pause between failed autobaud passes
#endif
#ifndef OLED_TASK_STACK
#define OLED_TASK_STACK 6144 // display task stack, bytes (the debug refresh line prints the high-water mark)
#endif
#ifndef OLED_INIT_RETRY_MS
#define OLED_INIT_RETRY_MS 5000 // re-probe / re-init period while the panel is absent, and presence re-check while running
#endif
#ifndef OLED_I2C_TIMEOUT_MS
#define OLED_I2C_TIMEOUT_MS 50 // I2C transaction timeout (a stuck bus costs this per transaction)
#endif

#ifndef OLED_BUTTON_POLL_MS
#define OLED_BUTTON_POLL_MS 20 // display task loop period = button sampling granularity
#endif
#ifndef CAN_TX_WAIT_MS
#define CAN_TX_WAIT_MS 10 // twai_node_transmit() block time for a poll request
#endif
#ifndef VESC_POLL_LOG_MIN_MS
#define VESC_POLL_LOG_MIN_MS 10000 // rate limit for "no reply" / "bad reply" / "not transmitted" warnings
#endif
#ifndef VESC_POLL_BACKOFF_AFTER
#define VESC_POLL_BACKOFF_AFTER 3 // consecutive polls without a reply before slowing down ...
#endif
#ifndef VESC_POLL_BACKOFF_MS
#if VESC_LINK_UART
#define VESC_POLL_BACKOFF_MS 1000 // ... to this period. UART: no broadcast announces a VESC that is back, so keep probing at 1 Hz
#else
#define VESC_POLL_BACKOFF_MS 5000 // ... to this period (a STATUS frame resumes the normal rate at once)
#endif
#endif
#ifndef VESC_UART_RX_WAIT_MS
#define VESC_UART_RX_WAIT_MS 10 // UART: uart_read_bytes() wait = task loop period; a read that returns nothing means the line was idle
#endif
#ifndef VESC_GETVALUES_MASK
#define VESC_GETVALUES_MASK 0x003EC03Cu // COMM_GET_VALUES_SELECTIVE field mask (fields the STATUS frames lack); 0 = plain COMM_GET_VALUES
#endif
#ifndef VESC_RX_BUFFER_SIZE
#define VESC_RX_BUFFER_SIZE 512 // reassembly buffer for polled replies (VESC maximum packet payload)
#endif

#ifndef GNSS_BAUD_SWITCH_ATTEMPTS
#define GNSS_BAUD_SWITCH_ATTEMPTS 2 // unconfirmed baud switches in a row before the detected baud is kept
#endif
#ifndef GNSS_BAUD_SWITCH_SETTLE_MS
#define GNSS_BAUD_SWITCH_SETTLE_MS 100 // u-blox: "typically 100 ms" between the baud-change message and data at the new rate
#endif
#ifndef TASK_PRIO_TRIP
#define TASK_PRIO_TRIP 3 // trip integrator task: between the display (2) and the data producers (GNSS 5, CAN 6)
#endif
#ifndef TRIP_TASK_STACK
#define TRIP_TASK_STACK 4096 // bytes
#endif
#ifndef TRIP_DT_MAX_MS
#define TRIP_DT_MAX_MS 5000 // longest step integrated after a stall; beyond it time is dropped, not integrated
#endif
#ifndef TRIP_COUNTER_RESET_WH
#define TRIP_COUNTER_RESET_WH 0.5f // a watt-hour counter delta below -this means the VESC rebooted (counters restarted)
#endif
#ifndef TRIP_EFF_MIN_FILL_S
#define TRIP_EFF_MIN_FILL_S 3 // closed one-second window buckets needed before the "now" efficiency is shown
#endif
#ifndef TRIP_STALE_MS
#define TRIP_STALE_MS 5000 // TripState older than this -> the EFFICIENCY screen shows "--"
#endif

#ifndef TASK_PRIO_DEADMAN
#define TASK_PRIO_DEADMAN 7 // ABOVE TASK_PRIO_CAN on purpose (see the #error below): a few comparisons and one
                            // GPIO write per DEADMAN_TICK_MS must never wait behind a burst of CAN frames
#endif
#ifndef DEADMAN_TASK_STACK
#define DEADMAN_TASK_STACK 2560 // bytes; the task never logs and never formats (all of that is in the supervisor)
#endif
#ifndef DEADMAN_TICK_MS
#define DEADMAN_TICK_MS 20 // deadmanTask loop period = actuation granularity
#endif
#ifndef DEADMAN_SCAN_ITVL_MS
#define DEADMAN_SCAN_ITVL_MS 100 // watchdog scan interval ...
#endif
#ifndef DEADMAN_SCAN_WINDOW_MS
#define DEADMAN_SCAN_WINDOW_MS 100 // ... and window. Equal = continuous: below ~80 % duty a 300 ms tag is missed too often
#endif
#ifndef HB_MAX_DEADMAN_MS
#define HB_MAX_DEADMAN_MS 1000 // supervisor: a deadman heartbeat older than this stops the WDT feed -> reboot. Never 0:
                               // the task is safety-relevant and a stall must be caught
#endif
#ifndef LOG_DEADMAN_MS
#define LOG_DEADMAN_MS 5000 // period of the DM log line (0 = off)
#endif
#ifndef TASK_PRIO_BLE
#define TASK_PRIO_BLE 4 // ble task (src/ble_scan.cpp, dead-man builds): keeps the watchdog scan up. Between tripTask (3) and
                        // gnssTask (5); the NimBLE host / controller tasks run at 21 / 23 regardless (on TASK_CORE_RADIO)
#endif
#ifndef BLE_TASK_STACK
#define BLE_TASK_STACK 4096 // bytes
#endif

// ============================================================================
// Derived values (do not edit)
// ============================================================================
#if SPEED_UNIT_KNOTS
#define SPEED_FACTOR 0.00194384f // mm/s -> knots
#define SPEED_UNIT_STR "kn"
#else
#define SPEED_FACTOR 0.0036f // mm/s -> km/h
#define SPEED_UNIT_STR "km/h"
#endif
#if SPEED_UNIT_KNOTS && !EFF_UNIT_KM
#define EFF_DIST_UNIT_M 1852.0f // one nautical mile
#define EFF_DIST_UNIT_STR "NM"
#define EFF_UNIT_STR "Wh/NM"
#else
#define EFF_DIST_UNIT_M 1000.0f
#define EFF_DIST_UNIT_STR "km"
#define EFF_UNIT_STR "Wh/km"
#endif
// Dead-man's switch: the state struct and its screen exist when the feature is built or in the demo.
#define DEADMAN_UI_ENABLE (DEADMAN_ENABLE || DISPLAY_DEMO)
// Tag slots. Fixed at 2 so there is ONE code path: an empty or unparsable DEADMAN_BEACON_ADDR2
// leaves slot 2 inert (ble_addr_parse zeroes its output, and an all-zero address never matches a
// real advert), so a single-tag boat behaves exactly as it did before the slot existed.
#define DEADMAN_TAGS 2

// ============================================================================
// Compile-time sanity checks
// ============================================================================
#if CAN_BITRATE_KBPS != 125 && CAN_BITRATE_KBPS != 250 && CAN_BITRATE_KBPS != 500 && CAN_BITRATE_KBPS != 1000
#error "CAN_BITRATE_KBPS must be 125, 250, 500 or 1000"
#endif
#if PIN_CAN_TX == PIN_CAN_RX
#error "PIN_CAN_TX and PIN_CAN_RX must differ"
#endif
#if CAN_LISTEN_ONLY && VESC_POLL_MS > 0
#error "VESC_POLL_MS > 0 requires CAN_LISTEN_ONLY 0: a listen-only controller cannot transmit (set VESC_POLL_MS 0 for a passive node)"
#endif
#if GNSS_RATE_MS < 50 || GNSS_RATE_MS > 10000
#error "GNSS_RATE_MS must be 50..10000 ms (u-blox M8 minimum 50-100 ms)"
#endif
#if EFF_WINDOW_S < 2 || EFF_WINDOW_S > 120
#error "EFF_WINDOW_S must be 2..120 seconds"
#endif
#if VESC_LINK_UART && VESC_POLL_MS <= 0
#error "VESC_LINK_UART needs VESC_POLL_MS > 0: over UART nothing is broadcast, the poll is the only data source"
#endif
#if VESC_LINK_UART && PIN_VESC_UART_TX == PIN_VESC_UART_RX
#error "PIN_VESC_UART_TX and PIN_VESC_UART_RX must differ"
#endif
#if VESC_LINK_UART && VESC_UART_PORT == 1
#error "VESC_UART_PORT must not be 1: UART1 is the GNSS"
#endif
#if CAN_OWN_ID < 1 || CAN_OWN_ID > 254
#error "CAN_OWN_ID must be 1..254 (255 is the CAN broadcast id)"
#endif
#if VESC_CAN_ID >= 0 && VESC_CAN_ID == CAN_OWN_ID
#error "CAN_OWN_ID must differ from VESC_CAN_ID"
#endif

#if DEADMAN_ENABLE && PIN_BUTTON < 0
#error "DEADMAN_ENABLE needs a button: a tripped dead-man's switch latches and can only be cleared by hand (-DPIN_BUTTON=BOARD_BOOT_BUTTON uses the BOOT button)"
#endif
#if DEADMAN_ENABLE && DEADMAN_WARN_MS >= DEADMAN_TIMEOUT_MS
#error "DEADMAN_WARN_MS must be shorter than DEADMAN_TIMEOUT_MS (it is the on-screen countdown before the cut)"
#endif
#if DEADMAN_ENABLE && DEADMAN_RESET_FRESH_MS >= DEADMAN_TIMEOUT_MS
#error "DEADMAN_RESET_FRESH_MS must be shorter than DEADMAN_TIMEOUT_MS, else a reset can succeed with a tag that is already timing out"
#endif
#if DEADMAN_ENABLE && DEADMAN_CONFIRM_ENABLE && (VESC_POLL_MS == 0 || CAN_LISTEN_ONLY)
#error "DEADMAN_CONFIRM_ENABLE needs the VESC poll: the kill-switch status bit only arrives in a poll reply (set DEADMAN_CONFIRM_ENABLE 0)"
#endif
#if DEADMAN_ENABLE && DEADMAN_CONFIRM_ENABLE && (DEADMAN_CONFIRM_MS < 3 * VESC_POLL_MS)
#error "DEADMAN_CONFIRM_MS must cover three poll periods, else one lost single-shot request raises a false FAULT"
#endif
#if DEADMAN_ENABLE && (DEADMAN_ARM_RSSI < -110 || DEADMAN_ARM_RSSI > 0)
#error "DEADMAN_ARM_RSSI is a dBm value in -110..0"
#endif
#if DEADMAN_ENABLE && TASK_PRIO_DEADMAN <= TASK_PRIO_CAN
#error "TASK_PRIO_DEADMAN must stay ABOVE TASK_PRIO_CAN: the cut must never wait behind a CAN burst. This is the deliberate exception to the rule that TASK_PRIO_BLE follows - do not 'fix' it"
#endif
#if DEADMAN_ENABLE && BOARD_CUT_PIN_FORBIDDEN(PIN_DEADMAN_CUT)
#error "PIN_DEADMAN_CUT must not be a strapping pin of this board (FireBeetle C6: 8/9 and the LED/JTAG pin 15; S3-Zero: 0/3/45/46): a glitch low at reset would be a spurious engine cut"
#endif
#if DEADMAN_ENABLE && (DEADMAN_SCAN_WINDOW_MS > DEADMAN_SCAN_ITVL_MS)
#error "BLE scan window must not exceed the scan interval"
#endif
#if DEADMAN_ENABLE && (DEADMAN_SCAN_WINDOW_MS * 10 < DEADMAN_SCAN_ITVL_MS * 8)
#error "the watchdog scan duty cycle must be >= 80 %: below that a 300 ms beacon is missed too often to trust the timeout"
#endif
#if DEADMAN_ENABLE && HB_MAX_DEADMAN_MS == 0
#error "HB_MAX_DEADMAN_MS must be > 0: the dead-man task is safety-relevant, so a stall has to reboot the board"
#endif
#if DEADMAN_ENABLE && TASK_PRIO_BLE >= TASK_PRIO_GNSS
#error "TASK_PRIO_BLE must stay below the data producers (TASK_PRIO_GNSS / TASK_PRIO_CAN)"
#endif
#if OTA_ENABLE && PIN_BUTTON < 0
#error "OTA_ENABLE needs a button: holding it is the only way into update mode"
#endif
#if OTA_ENABLE && OTA_LONG_PRESS_MS <= BUTTON_LONG_PRESS_MS
#error "OTA_LONG_PRESS_MS must be longer than BUTTON_LONG_PRESS_MS (the same hold passes the long press first)"
#endif
#if OTA_ENABLE && (OTA_AP_TX_POWER_QDBM < 8 || OTA_AP_TX_POWER_QDBM > 84)
#error "OTA_AP_TX_POWER_QDBM must be 8..84 (2..20 dBm in 0.25 dBm units)"
#endif
#if OTA_ENABLE && OTA_IDLE_TIMEOUT_MS < 30000
#error "OTA_IDLE_TIMEOUT_MS must be >= 30 s: time to join the access point and pick a file"
#endif
// hbOk() counts a heartbeat that was never touched as fresh for its first max_age ms, so a
// confirmation earlier than every HB_MAX_* would vouch for tasks that may never have started.
#if OTA_ENABLE && (OTA_CONFIRM_MS <= HB_MAX_CAN_MS || OTA_CONFIRM_MS <= HB_MAX_GNSS_MS || OTA_CONFIRM_MS <= HB_MAX_DISP_MS || \
                   OTA_CONFIRM_MS <= WDT_TIMEOUT_MS)
#error "OTA_CONFIRM_MS must exceed every HB_MAX_*_MS and WDT_TIMEOUT_MS, else a broken image can be confirmed"
#endif

#ifndef UNIT_TEST // native host tests only; every firmware build must pass this check
#include "sdkconfig.h"
#if BOARD_S3_ZERO && !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "BOARD_S3_ZERO needs an ESP32-S3 target: use board = waveshare_esp32s3_zero (see env s3zero_deadman_uart in platformio.ini)"
#endif
#if !BOARD_S3_ZERO && !defined(CONFIG_IDF_TARGET_ESP32C6)
#error "This firmware targets the FireBeetle 2 ESP32-C6 unless BOARD_S3_ZERO is set. Check platform / board in platformio.ini."
#endif
#if VESC_LINK_UART && defined(CONFIG_ESP_CONSOLE_UART) && CONFIG_ESP_CONSOLE_UART_NUM == VESC_UART_PORT
#error "VESC_LINK_UART drives VESC_UART_PORT, which must not be the console: keep CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG (sdkconfig.defaults)"
#endif
#if DEADMAN_ENABLE && !defined(CONFIG_BT_NIMBLE_ENABLED)
#error "DEADMAN_ENABLE needs the NimBLE host: build an env with sdkconfig.defaults.ble (env 'deadman'), see platformio.ini"
#endif
#if BOARD_S3_ZERO && (defined(CONFIG_FREERTOS_UNICORE) || CONFIG_ESP_MAIN_TASK_AFFINITY != TASK_CORE_APP)
#error "The S3-Zero runs dual-core with app_main on TASK_CORE_APP: sdkconfig.defaults.esp32s3 sets that (delete the stale sdkconfig.<env>)"
#endif
#if BOARD_S3_ZERO && defined(CONFIG_BT_NIMBLE_ENABLED) && \
    (CONFIG_BT_NIMBLE_PINNED_TO_CORE != TASK_CORE_RADIO || CONFIG_BT_CTRL_PINNED_TO_CORE != TASK_CORE_RADIO)
#error "The BLE controller and the NimBLE host must run on TASK_CORE_RADIO (sdkconfig.defaults.esp32s3)"
#endif
#endif

#ifdef __cplusplus
namespace cfg_check
{
  // The two pins of the VESC link: the CAN transceiver's, or the UART's.
#if VESC_LINK_UART
  constexpr int kLinkTx = PIN_VESC_UART_TX;
  constexpr int kLinkRx = PIN_VESC_UART_RX;
#else
  constexpr int kLinkTx = PIN_CAN_TX;
  constexpr int kLinkRx = PIN_CAN_RX;
#endif
  // Every GPIO in use must be unique, off the USB pair and off the board's reserved pins.
  constexpr int kPins[] = {
      kLinkTx,
      kLinkRx,
      PIN_GNSS_RX,
      PIN_GNSS_TX,
      PIN_LED,
      PIN_OLED_SCL,
      PIN_OLED_SDA,
#if PIN_BUTTON >= 0
      PIN_BUTTON,
#endif
#if DEADMAN_ENABLE && PIN_DEADMAN_CUT >= 0
      PIN_DEADMAN_CUT,
#endif
  };
// PIN_OLED_RST is optional (-1); guard it separately.
#if PIN_OLED_RST >= 0
  static_assert(PIN_OLED_RST != PIN_OLED_SCL && PIN_OLED_RST != PIN_OLED_SDA && PIN_OLED_RST != kLinkTx &&
                    PIN_OLED_RST != kLinkRx && PIN_OLED_RST != PIN_GNSS_RX && PIN_OLED_RST != PIN_GNSS_TX &&
                    PIN_OLED_RST != PIN_LED && PIN_OLED_RST != BOARD_USB_DM && PIN_OLED_RST != BOARD_USB_DP &&
                    !BOARD_PIN_RESERVED(PIN_OLED_RST),
                "PIN_OLED_RST collides with another pin");
#endif
  constexpr bool allDistinct()
  {
    for (unsigned i = 0; i < sizeof(kPins) / sizeof(kPins[0]); ++i)
      for (unsigned j = i + 1; j < sizeof(kPins) / sizeof(kPins[0]); ++j)
        if (kPins[i] == kPins[j])
          return false;
    return true;
  }
  constexpr bool noUsbPins()
  {
    for (unsigned i = 0; i < sizeof(kPins) / sizeof(kPins[0]); ++i)
      if (kPins[i] == BOARD_USB_DM || kPins[i] == BOARD_USB_DP)
        return false;
    return true;
  }
  constexpr bool noReservedPins()
  {
    for (unsigned i = 0; i < sizeof(kPins) / sizeof(kPins[0]); ++i)
      if (BOARD_PIN_RESERVED(kPins[i]))
        return false;
    return true;
  }
  static_assert(allDistinct(), "Two peripherals share a GPIO in config.h");
  static_assert(noUsbPins(), "BOARD_USB_DM/DP (FireBeetle: GPIO12/13, S3-Zero: GPIO19/20) are the USB pins (Serial console); do not use them");
  static_assert(noReservedPins(), "a pin in use belongs to the board's flash / PSRAM (S3-Zero: GPIO26..37)");
#if OTA_ENABLE
  static_assert(sizeof(OTA_AP_SSID) >= 2 && sizeof(OTA_AP_SSID) <= 19, "OTA_AP_SSID must be 1..18 characters");
#endif
} // namespace cfg_check
#endif
