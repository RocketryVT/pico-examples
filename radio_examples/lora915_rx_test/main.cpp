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

#include "PicoHal.h"        // Pico SDK implementation of RadioLibHal (lora/hal/)
#include <RadioLib.h>

#include "rf_csv.h"         // common CSV output schema (../common/)
#include "rf_log.h"         // mirror the CSV stream into on-chip flash (../common/)
#include "gps_task.h"       // UART0 u-blox GPS — UTC + position stamping (../common/)
#include "rf_console.h"     // USB "list" / "export" command console (../common/)

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

    // Wait for the USB CDC host so we don't lose the early log lines.
    while ( !stdio_usb_connected() ) {
        sleep_ms( 100 );
    }
    sleep_ms( 500 );

    // The LoRa0 radio is gated behind a power-enable MOSFET — turn it on and
    // let the supply settle before talking to the chip.
    gpio_init( PIN_EN );
    gpio_set_dir( PIN_EN, GPIO_OUT );
    gpio_put( PIN_EN, 1 );
    printf( "# [rx] power enabled (GPIO%u), settling 1 s...\n", PIN_EN );
    sleep_ms( 1000 );

    printf( "# [rx] init SX1276 @ %.1f MHz SF%u BW%.0f kHz CR4/%u sync=0x%02X\n",
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
        printf( "# [rx] begin() failed, code %d — check wiring/power. Halting.\n", state );
        while ( true ) { sleep_ms( 1000 ); }
    }
    printf( "# [rx] SX1276 ready. Listening...\n" );

    // Mount the flash filesystem and mirror every CSV row into a fresh file.
    if ( rf_log_init( ROLE ) ) {
        rf_csv_set_sink( rf_log_write );
    }

    // Bring up the UART0 GPS and auto-configure UBX NAV-PVT (stamps utc/gps_*).
    gps_task_init();

    printf( "# console: type 'list' or 'export <n>' (or 'help') over USB serial\n" );
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

    for ( ;; ) {
        const uint32_t now = to_ms_since_boot( get_absolute_time() );

        // Service the GPS and refresh the fix stamped onto this iteration's rows.
        gps_task_poll();
        rf_csv_set_gps( gps_task_fix() );

        // Handle any USB console command (list / export).
        rf_console_poll();

        // -- Packet arrived? ---------------------------------------------------
        if ( gpio_get( PIN_DIO0 ) ) {
            const size_t len = static_cast<size_t>( radio.getPacketLength() );
            const int    err = radio.readData( buf, sizeof(buf) - 1 );

            const float rssi = radio.getRSSI();
            const float snr  = radio.getSNR();
            const float ferr = radio.getFrequencyError();

            if ( err == RADIOLIB_ERR_NONE ) {
                buf[len] = '\0';

                // Recover the "#N" sequence number for loss tracking.
                bool          seq_ok = false;
                unsigned long seq    = 0;
                if ( const char* h = strchr( reinterpret_cast<char*>( buf ), '#' ) ) {
                    char* end = nullptr;
                    seq = strtoul( h + 1, &end, 10 );
                    seq_ok = ( end != h + 1 );   // at least one digit parsed
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
                rf_csv_row( now, ROLE, FREQ_MHZ, MOD, "packet",
                            seq_ok ? (long)seq : -1, (long)len,
                            rssi, snr, ferr,
                            (long)good_count, (long)lost_count, (long)crc_count,
                            per, -1 );
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

        sleep_ms( 5 );
    }

    return 0;
}
