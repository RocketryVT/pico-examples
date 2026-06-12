// main.cpp — 915 MHz LoRa transmit-only sanity test for the ground-station Pico.
//
// Puts the SX1276 (LoRa0, SPI0) into transmit and sends a short numbered test
// packet once a second, printing the result + on-air time over USB serial.
//
// Pair this with lora915_rx_test on a second board (COTS antenna here, since
// this is the transmit end) to check the RF link end-to-end. Pins, SPI bus and
// radio config match the ground-station LoRa0 path so the two ends agree.

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#include "PicoHal.h"        // Pico SDK implementation of RadioLibHal (lora/hal/)
#include <RadioLib.h>

#include "rf_csv.h"         // common CSV output schema (../common/)
#include "rf_payload.h"     // binary TX telemetry payload (../common/)
#include "rf_log.h"         // mirror the CSV stream into on-chip flash (../common/)
#include "gps_task.h"       // UART0 u-blox GPS — UTC + position stamping (../common/)
#include "rf_console.h"     // USB "list" / "export" command console (../common/)

#include <cstdio>
#include <cstring>
#include <cmath>
#include <math.h>

// -- LoRa0 / 915 MHz radio wiring (SPI0) — from shared.hpp Pins -----------------
static constexpr uint PIN_EN   = 16;  // GPIO 16, phys 21 — power enable (active high)
static constexpr uint PIN_DIO0 = 17;  // GPIO 17, phys 22 — G0 / IRQ (TxDone)
static constexpr uint PIN_SCK  = 18;  // GPIO 18, phys 24
static constexpr uint PIN_MOSI = 19;  // GPIO 19, phys 25
static constexpr uint PIN_MISO = 20;  // GPIO 20, phys 26
static constexpr uint PIN_NSS  = 21;  // GPIO 21, phys 27 — CS
static constexpr uint PIN_RST  = 22;  // GPIO 22, phys 29 — reset

// -- LoRa0 air config — from LoRa0Cfg in shared.hpp ----------------------------
static constexpr float    FREQ_MHZ  = 915.0f;
static constexpr float    BW_KHZ    = 125.0f;
static constexpr uint8_t  SF        = 7;
static constexpr uint8_t  CR        = 5;
static constexpr uint8_t  SYNC_WORD = 0x12;
static constexpr int8_t   TX_DBM    = 20;   // +20 dBm with PA boost (COTS antenna)
static constexpr uint16_t PREAMBLE  = 8;

// CSV identity for this tool.
static constexpr const char* ROLE = "tx";
static constexpr const char* MOD  = "lora";

// GPS wiring for this board.
static constexpr uint8_t  GPS_TX_PIN = 0;       // Pico TX -> GPS RX
static constexpr uint8_t  GPS_RX_PIN = 1;       // Pico RX <- GPS TX
static constexpr uint32_t GPS_BAUD   = 230400;  // preferred listen-only GPS UART

// HAL + radio. Declared static/global because RadioLib keeps internal pointers
// into the HAL and Module objects.
static PicoHal hal( spi0, static_cast<uint8_t>( PIN_SCK ),
                          static_cast<uint8_t>( PIN_MOSI ),
                          static_cast<uint8_t>( PIN_MISO ) );
static Module  module_( &hal, PIN_NSS, PIN_DIO0, PIN_RST, RADIOLIB_NC );
static SX1276  radio( &module_ );

int main()
{
    stdio_init_all();
    sleep_ms( 500 );

    // The LoRa0 radio is gated behind a power-enable MOSFET — turn it on and
    // let the supply settle before talking to the chip.
    gpio_init( PIN_EN );
    gpio_set_dir( PIN_EN, GPIO_OUT );
    gpio_put( PIN_EN, 1 );
    printf( "# [tx] power enabled (GPIO%u), settling 1 s...\n", PIN_EN );
    sleep_ms( 1000 );

    printf( "# [tx] init SX1276 @ %.1f MHz SF%u BW%.0f kHz CR4/%u sync=0x%02X pwr=%d dBm\n",
            (double)FREQ_MHZ, (unsigned)SF, (double)BW_KHZ,
            (unsigned)CR, (unsigned)SYNC_WORD, (int)TX_DBM );

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
        printf( "# [tx] begin() failed, code %d — check wiring/power. Halting.\n", state );
        while ( true ) { sleep_ms( 1000 ); }
    }
    printf( "# [tx] SX1276 ready. Transmitting...\n" );

    // Mount the flash filesystem and mirror every CSV row into a fresh file.
    if ( rf_log_init( ROLE ) ) {
        rf_csv_set_sink( rf_log_write );
    }

    // Bring up UART0 GPS in listen-only mode. The GM10/M10050 stream verified
    // in u-center already emits valid NMEA fixes; do not send UBX config here.
    gps_task_init_autobaud_listen_only( GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD );
    gps_task_set_nav_pvt_debug( true );

    printf( "# console: type 'list' or 'export <n>' (or 'help') over USB serial\n" );
    printf( "# [tx] live CSV rows suppressed; use 'export <n>' for CSV logs\n" );
    rf_csv_set_stdout_enabled( false );
    rf_csv_header();
    printf( "# [tx] RF payload RFT2 binary, %u bytes\n", (unsigned)sizeof( RfPacketV2 ) );

    uint8_t  msg[sizeof( RfPacketV2 )];
    uint32_t count = 0;

    for ( ;; ) {
        // Stamp the freshest GPS fix into the RF packet itself.
        gps_task_poll();
        const RfGps tx_gps = gps_task_fix();
        rf_csv_set_gps( tx_gps );

        const uint32_t tx_ms = to_ms_since_boot( get_absolute_time() );
        const size_t len = rf_payload_build( msg, sizeof msg, count, tx_ms, tx_gps );

        const uint32_t t0  = to_ms_since_boot( get_absolute_time() );
        state = radio.transmit( msg, len );
        const uint32_t t1  = to_ms_since_boot( get_absolute_time() );
        const uint32_t air = t1 - t0;

        rf_csv_row( t1, ROLE, FREQ_MHZ, MOD,
                    ( state == RADIOLIB_ERR_NONE ) ? "tx_ok" : "tx_fail",
                    (long)count, (long)len,
                    NAN, NAN, NAN, -1, -1, -1, NAN, (long)air );

        rf_log_sync();   // flush this row to flash before the next beacon.

        ++count;
        // Wait ~1 s before the next beacon, servicing the GPS and USB console
        // throughout so the GPS UART FIFO doesn't overflow and commands stay
        // responsive between transmits.
        for ( int i = 0; i < 100; ++i ) {
            gps_task_poll();
            rf_console_poll();
            sleep_ms( 10 );
        }
    }

    return 0;
}
