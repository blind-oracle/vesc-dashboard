// boat-motor configuration
// ----------------------------------------------------------------------------
// THE one header you edit to match your wiring and preferences. Nothing else in
// the project hard-codes a pin. Every value is wrapped in #ifndef so it can also
// be overridden from platformio.ini:  build_flags = -DPIN_CAN_TX=7
//
// Display: 128x64 SSD1309 OLED over I2C (see the OLED section).
//
// Board: DFRobot FireBeetle 2 ESP32-C6 (DFR1075). Silkscreen labels equal GPIO
// numbers. Pins to leave alone: 12/13 (USB), 16/17 (UART0 boot console, GPIO16
// is driven by UART0 TX the whole run), 9 (BOOT button), 0 (battery ADC, not on
// the header). Strapping pins: 4/5 (SDIO edge only, harmless), 8/9 (boot mode),
// 15 (JTAG select, on-board LED). Never put a peripheral-driven INPUT (a sensor
// IRQ or busy line, say) on 8, 9 or 15.
// ----------------------------------------------------------------------------
#pragma once

#include <stdint.h>

#ifndef FW_VERSION
#define FW_VERSION "0.1.0"
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
#define VESC_POLL_MS 1000 // request period in ms; 0 = never transmit anything
#endif
#ifndef CAN_OWN_ID
#define CAN_OWN_ID 120 // our controller id on the VESC bus: 1..254, must differ from every VESC id (255 = broadcast)
#endif
#ifndef VESC_POLL_TIMEOUT_MS
#define VESC_POLL_TIMEOUT_MS 500 // an incomplete reply is discarded after this long
#endif
#ifndef VESC_EXT_STALE_MS
#define VESC_EXT_STALE_MS 3000 // polled values older than this are shown as "--"
#endif
#ifndef CAN_TX_QUEUE_LEN
#define CAN_TX_QUEUE_LEN 4 // TWAI node TX queue depth (>= 1); only the poll requests ever use it
#endif

// ============================================================================
// Screens, button, clock
// ============================================================================
#ifndef PIN_BUTTON
#define PIN_BUTTON 7 // silkscreen "7"/INT: momentary button to GND (internal pull-up). -1 = no button.
                     // 9 = use the on-board BOOT button instead (fine at runtime; holding it during reset enters download mode)
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
#define SCREEN_AUTO_RETURN_MS 60000 // after this long on another screen, return to the main screen; 0 = stay
#endif
#ifndef TIME_UTC_OFFSET_MIN
#define TIME_UTC_OFFSET_MIN 0 // displayed clock = GNSS UTC + this many minutes (e.g. 120 for UTC+2). No DST logic.
#endif

// ============================================================================
// Trip / efficiency (src/trip.cpp, EFFICIENCY screen)
// ============================================================================
// Distance = GNSS ground speed integrated while moving; energy = VESC watt-hour
// counters (STATUS_3), falling back to v_in x current_in integration while
// STATUS_3 is not being received. Efficiency = Wh per distance unit.
#ifndef TRIP_PERIOD_MS
#define TRIP_PERIOD_MS 200      // integrator tick (matches the 5 Hz GNSS rate)
#endif
#ifndef EFF_WINDOW_S
#define EFF_WINDOW_S 10         // "now" efficiency = last this many seconds
#endif
#ifndef EFF_MIN_SPEED_MM_S
#define EFF_MIN_SPEED_MM_S 500  // below this ground speed no distance is integrated (GNSS drift at the mooring); 500 mm/s ~ 1 kn
#endif
#ifndef EFF_MIN_DIST_M
#define EFF_MIN_DIST_M 10       // a window / trip shorter than this shows "--" instead of a meaningless ratio
#endif
#ifndef EFF_UNIT_KM
#define EFF_UNIT_KM 0           // 0 = follow the speed unit (knots -> Wh/NM, km/h -> Wh/km); 1 = always Wh/km
#endif

// ============================================================================
// BMS over BLE (JK BMS, NimBLE central; src/bms_ble.cpp, include/jk_bms.h)
// ============================================================================
// Reads a Jikong (JK) BMS over Bluetooth LE: service 0xFFE0 / characteristic 0xFFE1,
// commands 0x97 (device info) and 0x96 (cell info stream). Only built in the 'bms'
// environment (sdkconfig.defaults.bms enables the NimBLE host). A JK BMS accepts ONE
// BLE central: the phone app cannot be connected at the same time.
#ifndef BMS_BLE_ENABLE
#define BMS_BLE_ENABLE 0          // 1 = build the NimBLE client + bmsTask (needs 'pio run -e bms'); 0 = no BLE code at all
#endif
#ifndef BMS_BLE_ADDR
#define BMS_BLE_ADDR ""           // BMS Bluetooth MAC "C8:47:8C:12:34:56" = connect by address (passive scan); "" = match by name prefix / 0xFFE0 (active scan)
#endif
#ifndef BMS_BLE_NAME_PREFIX
#define BMS_BLE_NAME_PREFIX "JK"  // advertised name prefix ("JK-..." / "JK_..."); the name is user-editable in the JK app
#endif
#ifndef BMS_PROTOCOL
#define BMS_PROTOCOL 0            // 0 = auto (device-info sw >= 11 or JK-PB -> 32S layout, else 24S; confirmed by a cell-sum check), 1 = force JK02_24S, 2 = force JK02_32S
#endif
#ifndef BMS_CELLS_MAX
#define BMS_CELLS_MAX 24          // 1..32 cells kept in the shared state and shown on ceil(n/12) CELLS pages
#endif
#ifndef BMS_CURRENT_SIGN
#define BMS_CURRENT_SIGN 1        // 1 = discharge-positive on screen (same sign as the BA/PW cells); 0 = JK app convention (charge-positive)
#endif
#ifndef BMS_STALE_MS
#define BMS_STALE_MS 10000        // no decoded cell-info frame for this long -> every BMS value shows "--" (frames arrive every ~0.5 s)
#endif
#ifndef BMS_RECONNECT_MS
#define BMS_RECONNECT_MS 2000     // first back-off after a disconnect / failed connect ...
#endif
#ifndef BMS_RECONNECT_MAX_MS
#define BMS_RECONNECT_MAX_MS 30000 // ... doubling up to this
#endif
#ifndef BMS_RECONNECT_FAILS
#define BMS_RECONNECT_FAILS 3     // direct reconnects to the last known address before falling back to scanning
#endif
#ifndef HB_MAX_BMS_MS
#define HB_MAX_BMS_MS 0           // 0 = never gate the task watchdog on the BMS task (its heartbeat age is only logged); > 0 = gate like the other tasks
#endif
#ifndef LOG_BMS_MS
#define LOG_BMS_MS 5000           // period of the BMS log line (0 = off)
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
#define GNSS_RATE_MS 200    // navigation rate: 200 = 5 Hz (safe on every M8/M9/M10 with all constellations); 100 = 10 Hz needs a fast module and GNSS_TARGET_BAUD >= 38400
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
#define SPEED_UNIT_KNOTS 0  // 1 = knots (mm/s * 0.00194384), 0 = km/h (mm/s * 0.0036)
#endif

// ============================================================================
// Tasks / watchdog / logging
// ============================================================================
#ifndef TASK_PRIO_CAN
#define TASK_PRIO_CAN 6 // highest: CAN reception must never be starved
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
#define LOG_TRIP_MS 5000    // period of the trip / efficiency log line (0 = off)
#endif
#ifndef SERIAL_BOOT_DELAY_MS
#define SERIAL_BOOT_DELAY_MS 1500 // give the USB-Serial/JTAG host time to re-enumerate so the boot banner is visible
#endif
#ifndef PIN_LED
#define PIN_LED 15 // on-board green LED (active high): 1 Hz blink = VESC data fresh, slow blink = no VESC data
#endif

// ---- Advanced task tunables (defaults are fine; exposed so nothing is hidden in the .cpp files) ----
#ifndef CAN_TASK_STACK
#define CAN_TASK_STACK 4096 // canTask stack, bytes
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
#define VESC_POLL_BACKOFF_MS 5000 // ... to this period (a STATUS frame resumes the normal rate at once)
#endif
#ifndef VESC_GETVALUES_MASK
#define VESC_GETVALUES_MASK 0x003EC03Cu // COMM_GET_VALUES_SELECTIVE field mask (fields the STATUS frames lack); 0 = plain COMM_GET_VALUES
#endif
#ifndef VESC_RX_BUFFER_SIZE
#define VESC_RX_BUFFER_SIZE 512 // reassembly buffer for polled replies (VESC maximum packet payload)
#endif

#ifndef GNSS_BAUD_SWITCH_ATTEMPTS
#define GNSS_BAUD_SWITCH_ATTEMPTS 2    // unconfirmed baud switches in a row before the detected baud is kept
#endif
#ifndef GNSS_BAUD_SWITCH_SETTLE_MS
#define GNSS_BAUD_SWITCH_SETTLE_MS 100 // u-blox: "typically 100 ms" between the baud-change message and data at the new rate
#endif
#ifndef TASK_PRIO_TRIP
#define TASK_PRIO_TRIP 3          // trip integrator task: between the display (2) and the data producers (GNSS 5, CAN 6)
#endif
#ifndef TRIP_TASK_STACK
#define TRIP_TASK_STACK 4096      // bytes
#endif
#ifndef TRIP_DT_MAX_MS
#define TRIP_DT_MAX_MS 5000       // longest step integrated after a stall; beyond it time is dropped, not integrated
#endif
#ifndef TRIP_COUNTER_RESET_WH
#define TRIP_COUNTER_RESET_WH 0.5f // a watt-hour counter delta below -this means the VESC rebooted (counters restarted)
#endif
#ifndef TRIP_EFF_MIN_FILL_S
#define TRIP_EFF_MIN_FILL_S 3     // closed one-second window buckets needed before the "now" efficiency is shown
#endif
#ifndef TRIP_STALE_MS
#define TRIP_STALE_MS 5000        // TripState older than this -> the EFFICIENCY screen shows "--"
#endif

#ifndef BMS_TASK_PRIO
#define BMS_TASK_PRIO 4           // bmsTask: between tripTask (3) and gnssTask (5); the NimBLE host/controller tasks run at 21/23 regardless
#endif
#ifndef BMS_TASK_STACK
#define BMS_TASK_STACK 4096       // bytes
#endif
#ifndef BMS_CONNECT_TIMEOUT_MS
#define BMS_CONNECT_TIMEOUT_MS 10000 // ble_gap_connect() duration; a peer that does not answer (phone app connected) costs this per attempt
#endif
#ifndef BMS_SETUP_TIMEOUT_MS
#define BMS_SETUP_TIMEOUT_MS 8000 // MTU exchange + service/characteristic/descriptor discovery + CCCD write must finish within this
#endif
#ifndef BMS_FIRST_FRAME_TIMEOUT_MS
#define BMS_FIRST_FRAME_TIMEOUT_MS 15000 // first CRC-valid, plausible cell-info frame after subscribing, else terminate + back-off
#endif
#ifndef BMS_POLL_MS
#define BMS_POLL_MS 5000          // 0x96 re-send period until the unsolicited cell-info stream starts
#endif
#ifndef BMS_SCAN_FAST_MS
#define BMS_SCAN_FAST_MS 30000    // fast scan preset duration after boot / after a disconnect ...
#endif
#ifndef BMS_SCAN_FAST_ITVL_MS
#define BMS_SCAN_FAST_ITVL_MS 60  // ... interval / window of the fast preset (~50 % RX duty)
#endif
#ifndef BMS_SCAN_FAST_WINDOW_MS
#define BMS_SCAN_FAST_WINDOW_MS 30
#endif
#ifndef BMS_SCAN_SLOW_ITVL_MS
#define BMS_SCAN_SLOW_ITVL_MS 1000 // slow preset while the BMS is absent (~3 % RX duty)
#endif
#ifndef BMS_SCAN_SLOW_WINDOW_MS
#define BMS_SCAN_SLOW_WINDOW_MS 30
#endif
#ifndef BMS_MSG_BUF_BYTES
#define BMS_MSG_BUF_BYTES 2048    // NimBLE host -> bmsTask message buffer (a 300-byte frame arrives as 2..15 notifications)
#endif
#ifndef BMS_APP_HINT_S
#define BMS_APP_HINT_S 60         // after this long without a link the BMS screen adds the "app open?" hint
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
// BMS: the shared state / screens exist when the BLE client is built or in the display demo; CELLS pages hold 12 cells each.
#define BMS_UI_ENABLE (BMS_BLE_ENABLE || DISPLAY_DEMO)
#define BMS_CELL_PAGES ((BMS_CELLS_MAX + 11) / 12)

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
#if CAN_OWN_ID < 1 || CAN_OWN_ID > 254
#error "CAN_OWN_ID must be 1..254 (255 is the CAN broadcast id)"
#endif
#if VESC_CAN_ID >= 0 && VESC_CAN_ID == CAN_OWN_ID
#error "CAN_OWN_ID must differ from VESC_CAN_ID"
#endif

#if BMS_CELLS_MAX < 1 || BMS_CELLS_MAX > 32
#error "BMS_CELLS_MAX must be 1..32"
#endif
#if BMS_PROTOCOL < 0 || BMS_PROTOCOL > 2
#error "BMS_PROTOCOL must be 0 (auto), 1 (JK02_24S) or 2 (JK02_32S)"
#endif
#if BMS_BLE_ENABLE && BMS_TASK_PRIO >= TASK_PRIO_GNSS
#error "BMS_TASK_PRIO must stay below the data producers (TASK_PRIO_GNSS / TASK_PRIO_CAN)"
#endif
#if BMS_SCAN_FAST_WINDOW_MS > BMS_SCAN_FAST_ITVL_MS || BMS_SCAN_SLOW_WINDOW_MS > BMS_SCAN_SLOW_ITVL_MS
#error "BLE scan window must not exceed the scan interval"
#endif

#ifndef UNIT_TEST // native host tests only; every firmware build must pass this check
#include "sdkconfig.h"
#if !defined(CONFIG_IDF_TARGET_ESP32C6)
#error "This firmware targets the ESP32-C6 (ESP-IDF). Check platform / board in platformio.ini and sdkconfig.defaults."
#endif
#if BMS_BLE_ENABLE && !defined(CONFIG_BT_NIMBLE_ENABLED)
#error "BMS_BLE_ENABLE needs the NimBLE host: build the 'bms' env (sdkconfig.defaults.bms), see platformio.ini"
#endif
#endif

#ifdef __cplusplus
namespace cfg_check
{
  // Every GPIO in use must be unique and off the USB pair (12/13).
  constexpr int kPins[] = {PIN_CAN_TX, PIN_CAN_RX, PIN_GNSS_RX, PIN_GNSS_TX, PIN_LED, PIN_OLED_SCL, PIN_OLED_SDA,
#if PIN_BUTTON >= 0
                           PIN_BUTTON,
#endif
  };
// PIN_OLED_RST is optional (-1); guard it separately.
#if PIN_OLED_RST >= 0
  static_assert(PIN_OLED_RST != PIN_OLED_SCL && PIN_OLED_RST != PIN_OLED_SDA && PIN_OLED_RST != PIN_CAN_TX &&
                    PIN_OLED_RST != PIN_CAN_RX && PIN_OLED_RST != PIN_GNSS_RX && PIN_OLED_RST != PIN_GNSS_TX &&
                    PIN_OLED_RST != PIN_LED && PIN_OLED_RST != 12 && PIN_OLED_RST != 13,
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
      if (kPins[i] == 12 || kPins[i] == 13)
        return false;
    return true;
  }
  static_assert(allDistinct(), "Two peripherals share a GPIO in config.h");
  static_assert(noUsbPins(), "GPIO12/13 are the USB D-/D+ pins (Serial console); do not use them");
} // namespace cfg_check
#endif
