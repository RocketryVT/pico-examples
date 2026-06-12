// main.cpp — 915 MHz LoRa receive-only sanity test for the ground-station Pico.
//
// Puts the SX1276 (LoRa0, SPI0) into continuous receive and prints every
// packet plus its RSSI / SNR over USB serial. Between packets it samples the
// channel RSSI (the noise floor) once a second so you can watch the receiver
// "hear" energy even when nothing decodes.
//
// Why this exists: we are feeding the radio through a DIY antenna with a
// terrible (~4 kOhm) impedance. We *expect* it not to work — this is a quick
// second opinion to cross-check a VNA reading we don't trust. If the noise
// floor moves when you key a nearby transmitter, or any packet/CRC-error shows
// up, the RF front end is alive regardless of what the VNA says.
//
// Pins, SPI bus and radio config are copied from the ground-station firmware's
// LoRa0 path (projects/ground_station/pico/src/shared.hpp + lora_task.cpp).

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/uart.h"

#include "PicoHal.h"        // Pico SDK implementation of RadioLibHal (lora/hal/)
#include <RadioLib.h>

#include "rf_csv.h"         // common CSV output schema (../common/)
#include "rf_payload.h"     // binary TX telemetry payload (../common/)
#include "rf_log.h"         // mirror the CSV stream into on-chip flash (../common/)
#include "gps_task.h"       // UART0 u-blox GPS — UTC + position stamping (../common/)
#include "rf_console.h"     // USB console task + log_print() (../common/)
#include "rtos.h"           // FreeRTOS task helpers (../common/)

#include "FreeRTOS.h"
#include "task.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

// -- LoRa0 / 915 MHz radio wiring (SPI0) — from shared.hpp Pins -----------------
static constexpr uint PIN_EN   = 16;  // GPIO 16, phys 21 — power enable (active high)
static constexpr uint PIN_DIO0 = 17;  // GPIO 17, phys 22 — G0 / IRQ (RxDone)
static constexpr uint PIN_SCK  = 18;  // GPIO 18, phys 24
static constexpr uint PIN_MOSI = 19;  // GPIO 19, phys 25
static constexpr uint PIN_MISO = 20;  // GPIO 20, phys 26
static constexpr uint PIN_NSS  = 21;  // GPIO 21, phys 27 — CS
static constexpr uint PIN_RST  = 22;  // GPIO 22, phys 29 — reset

// -- LoRa0 air config — from LoRa0Cfg in shared.hpp ----------------------------
static constexpr float   FREQ_MHZ  = 915.0f;
static constexpr float   BW_KHZ    = 125.0f;
static constexpr uint8_t SF        = 7;
static constexpr uint8_t CR        = 5;
static constexpr uint8_t SYNC_WORD = 0x12;
static constexpr int8_t  TX_DBM    = 20;   // unused in RX, kept for config parity
static constexpr uint16_t PREAMBLE = 8;

// CSV identity for this tool.
static constexpr const char* ROLE = "rx";
static constexpr const char* MOD  = "lora";

// GPS wiring for this 915 MHz RX board.
static constexpr uint8_t  RADIO_GPS_TX_PIN = 13;      // Pico TX -> GPS RX
static constexpr uint8_t  RADIO_GPS_RX_PIN = 12;      // Pico RX <- GPS TX
static constexpr uint32_t RADIO_GPS_BAUD   = 230400;  // u-center2 verified data rate

// Temporary u-center bridge mode. This bypasses the radio test, does not send
// any GPS configuration commands, and just relays USB CDC <-> GPS UART0.
static constexpr bool     UCENTER_UART_RELAY = false;
static constexpr uint32_t UCENTER_UART_BAUD  = 230400; // current module setting

// HAL + radio. Declared static/global because RadioLib keeps internal pointers
// into the HAL and Module objects.
static PicoHal hal( spi0, static_cast<uint8_t>( PIN_SCK ),
                          static_cast<uint8_t>( PIN_MOSI ),
                          static_cast<uint8_t>( PIN_MISO ) );
static Module  module_( &hal, PIN_NSS, PIN_DIO0, PIN_RST, RADIOLIB_NC );
static SX1276  radio( &module_ );

// FreeRTOS task priorities (console runs at tskIDLE_PRIORITY+1 internally).
// GPS above radio so the UART ring is drained promptly.
static constexpr UBaseType_t GPS_TASK_PRIORITY   = 3;
static constexpr UBaseType_t RADIO_TASK_PRIORITY = 2;

static StaticTask_t s_radio_tcb;
static StackType_t  s_radio_stack[4096];

void run_ucenter_uart_relay()
{
    stdio_init_all();

    gpio_init( PIN_EN );
    gpio_set_dir( PIN_EN, GPIO_OUT );
    gpio_put( PIN_EN, 1 );

    uart_init( uart0, UCENTER_UART_BAUD );
    uart_set_hw_flow( uart0, false, false );
    uart_set_format( uart0, 8, 1, UART_PARITY_NONE );
    uart_set_fifo_enabled( uart0, true );
    gpio_set_function( RADIO_GPS_TX_PIN, UART_FUNCSEL_NUM( uart0, RADIO_GPS_TX_PIN ) );
    gpio_set_function( RADIO_GPS_RX_PIN, UART_FUNCSEL_NUM( uart0, RADIO_GPS_RX_PIN ) );

    for ( ;; ) {
        while ( uart_is_readable( uart0 ) ) {
            putchar_raw( uart_getc( uart0 ) );
        }

        int c;
        while ( ( c = getchar_timeout_us( 0 ) ) != PICO_ERROR_TIMEOUT ) {
            if ( uart_is_writable( uart0 ) ) {
                uart_putc_raw( uart0, static_cast<char>( c ) );
            }
        }

        tight_loop_contents();
    }
}

// -- Radio task: SX1276 init + continuous-receive loop (core 1) ---------------
static void radio_task( void* )
{
    // The LoRa0 radio is gated behind a power-enable MOSFET — turn it on and
    // let the supply settle before talking to the chip.
    gpio_init( PIN_EN );
    gpio_set_dir( PIN_EN, GPIO_OUT );
    gpio_put( PIN_EN, 1 );
    log_print( "# [rx] power enabled (GPIO%u), settling 1 s...\n", PIN_EN );
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );

    log_print( "# [rx] init SX1276 @ %.1f MHz SF%u BW%.0f kHz CR4/%u sync=0x%02X\n",
               (double)FREQ_MHZ, (unsigned)SF, (double)BW_KHZ,
               (unsigned)CR, (unsigned)SYNC_WORD );

    ConfigLoRa_t cfg;
    cfg.frequency       = FREQ_MHZ;
    cfg.bandwidth       = BW_KHZ;
    cfg.spreadingFactor = SF;
    cfg.codingRate      = CR;
    cfg.syncWord        = SYNC_WORD;
    cfg.power           = TX_DBM;
    cfg.preambleLength  = PREAMBLE;

    int state = radio.begin( cfg );
    if ( state != RADIOLIB_ERR_NONE ) {
        log_print( "# [rx] begin() failed, code %d — check wiring/power.\n", state );
        for ( ;; ) { vTaskDelay( pdMS_TO_TICKS( 1000 ) ); }
    }
    log_print( "# [rx] SX1276 ready. Listening...\n" );
    rf_csv_header();

    // Continuous receive; DIO0 goes high on RxDone in LoRa mode.
    radio.startReceive();

    uint8_t  buf[256];
    uint32_t last_floor_ms = 0;

    // -- Link statistics -------------------------------------------------------
    // Loss is derived from the "#N" sequence number the TX embeds: any gap in
    // N between two good packets is counted as lost. CRC errors are counted
    // separately (we can't trust their payload, so they don't carry a seq).
    bool     have_seq   = false;   // seen at least one parseable seq yet
    uint32_t last_seq   = 0;       // last good seq seen
    uint32_t good_count = 0;       // good packets received
    uint32_t lost_count = 0;       // missing seq numbers (gaps)
    uint32_t crc_count  = 0;       // CRC failures
    bool     warned_legacy_payload = false;

    for ( ;; ) {
        const uint32_t now = to_ms_since_boot( get_absolute_time() );

        // Refresh the GPS fix stamped onto this iteration's rows (the GPS task
        // keeps the shared fix current; this just snapshots it).
        rf_csv_set_gps( gps_task_fix() );

        // -- Packet arrived? ---------------------------------------------------
        if ( gpio_get( PIN_DIO0 ) ) {
            const size_t len = static_cast<size_t>( radio.getPacketLength() );
            const int    err = radio.readData( buf, sizeof(buf) - 1 );

            const float rssi = radio.getRSSI();
            const float snr  = radio.getSNR();
            const float ferr = radio.getFrequencyError();

            if ( err == RADIOLIB_ERR_NONE ) {
                RfTxTelemetry tx_meta;

                // Recover the packet sequence number for loss tracking. Prefer
                // the current binary telemetry payload, but keep the old "#N"
                // text fallback so historical/simple TX firmware still works.
                bool          seq_ok = false;
                unsigned long seq    = 0;
                if ( rf_payload_parse( buf, len, &tx_meta ) && tx_meta.has_seq ) {
                    seq = tx_meta.seq;
                    seq_ok = true;
                } else {
                    if ( !warned_legacy_payload ) {
                        warned_legacy_payload = true;
                        log_print( "# [rx] non-RFT2 payload len=%lu; flash updated lora915_tx_test.uf2 for TX GPS fields\n",
                                   (unsigned long)len );
                    }
                    buf[( len < sizeof(buf) ) ? len : sizeof(buf) - 1] = '\0';
                    if ( const char* h = strchr( reinterpret_cast<char*>( buf ), '#' ) ) {
                        char* end = nullptr;
                        seq = strtoul( h + 1, &end, 10 );
                        seq_ok = ( end != h + 1 );   // at least one digit parsed
                    }
                }

                if ( seq_ok ) {
                    if ( !have_seq || seq < last_seq ) {
                        // First packet, or the TX restarted (seq went
                        // backwards) — rebase the counters here.
                        have_seq   = true;
                        last_seq   = seq;
                        good_count = 1;
                        lost_count = 0;
                        crc_count  = 0;
                    } else {
                        lost_count += ( seq - last_seq - 1 );  // gap = lost
                        last_seq    = seq;
                        ++good_count;
                    }
                } else {
                    ++good_count;   // good frame, just no parseable seq
                }

                const uint32_t expected = good_count + lost_count;
                const float per = expected ? ( 100.0f * lost_count / expected ) : 0.0f;
                rf_csv_row_tx( now, ROLE, FREQ_MHZ, MOD, "packet",
                               seq_ok ? (long)seq : -1, (long)len,
                               rssi, snr, ferr,
                               (long)good_count, (long)lost_count, (long)crc_count,
                               per, -1, tx_meta.valid ? &tx_meta : nullptr );
            } else if ( err == RADIOLIB_ERR_CRC_MISMATCH ) {
                // Energy received but corrupt — still proof the front end hears
                // something. Very useful with a marginal antenna.
                ++crc_count;
                const uint32_t expected = good_count + lost_count;
                const float per = expected ? ( 100.0f * lost_count / expected ) : 0.0f;
                rf_csv_row( now, ROLE, FREQ_MHZ, MOD, "crc_error",
                            -1, (long)len, rssi, snr, ferr,
                            (long)good_count, (long)lost_count, (long)crc_count,
                            per, -1 );
            } else {
                rf_csv_row( now, ROLE, FREQ_MHZ, MOD, "read_error",
                            -1, -1, rssi, NAN, NAN,
                            (long)good_count, (long)lost_count, (long)crc_count,
                            NAN, -1 );
            }

            // Re-arm for the next packet.
            radio.startReceive();
        }

        // -- Idle: report the channel noise floor about once a second ----------
        if ( now - last_floor_ms >= 1000 ) {
            last_floor_ms = now;
            // packet=false -> instantaneous channel RSSI; skipReceive=true ->
            // don't disturb the continuous-RX mode we're already in.
            const float floor = radio.getRSSI( false, true );
            rf_csv_row( now, ROLE, FREQ_MHZ, MOD, "noise_floor",
                        -1, -1, floor, NAN, NAN, -1, -1, -1, NAN, -1 );

            // Flush buffered log bytes to flash about once a second.
            rf_log_sync();
        }

        vTaskDelay( pdMS_TO_TICKS( 2 ) );
    }
}

int main()
{
    if constexpr ( UCENTER_UART_RELAY ) {
        run_ucenter_uart_relay();   // never returns
    }

    stdio_init_all();

    // Give a USB host a brief window to attach so we don't lose the early log
    // lines — but never *require* it. These are queued and flushed once the
    // console task runs.
    for ( int i = 0; i < 20 && !stdio_usb_connected(); ++i ) {
        sleep_ms( 100 );
    }
    sleep_ms( 200 );

    // Console task first (owns USB; creates the log queue so early log_print()
    // calls below are captured and flushed once the scheduler runs).
    rf_console_start();

    // Mount the flash filesystem and mirror every CSV row into a fresh file.
    if ( rf_log_init( ROLE ) ) {
        rf_csv_set_sink( rf_log_write );
    }

    // Bring up UART0 GPS in listen-only mode. The GM10/M10050 stream verified
    // in u-center already emits valid NMEA fixes; do not send UBX config here.
    gps_task_init_autobaud_listen_only( RADIO_GPS_TX_PIN, RADIO_GPS_RX_PIN, RADIO_GPS_BAUD );
    gps_task_set_nav_pvt_debug( true );

    rf_csv_set_stdout_enabled( false );
    log_print( "# console: 'list' / 'export <n>' / 'live on' (or 'help') over USB\n" );
    log_print( "# [rx] live CSV rows suppressed; use 'export <n>' or 'live on'\n" );

    // Spawn the GPS task (core 1) and the radio task (core 1), then run.
    gps_task_start( GPS_TASK_PRIORITY );
    TaskHandle_t h = rtos_task_create( radio_task, "radio", 4096, nullptr,
                                       RADIO_TASK_PRIORITY, s_radio_stack, &s_radio_tcb );
    vTaskCoreAffinitySet( h, 1u << 1 );  // core 1

    vTaskStartScheduler();
    for ( ;; ) {}
}
