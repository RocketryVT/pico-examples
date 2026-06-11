// main.cpp — 433 MHz GFSK receive-only sanity test for the ground-station Pico.
//
// Puts the RFM69HCW (RF69, LoRa1, SPI1) into GFSK receive and prints every
// packet plus its RSSI over USB serial, samples the channel noise floor once a
// second, and tracks packet-loss (PER) + rolling RSSI from the TX sequence
// number.
//
// FSK has no SNR (that's a spread-spectrum concept), so link quality here is
// judged by RSSI margin above the noise floor and by PER — not SNR.
//
// Pins, SPI bus and radio config match the ground-station LoRa1 / RF69 path
// (projects/ground_station/pico/src/shared.hpp + lora1_task.cpp). GFSK uses
// Gaussian shaping (BT=0.5); the transmitter must use the same shaping.

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#include "PicoHal.h"        // Pico SDK implementation of RadioLibHal (lora/hal/)
#include <RadioLib.h>

#include "rf_csv.h"         // common CSV output schema (../common/)

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

// -- LoRa1 / 433 MHz RFM69HCW wiring (SPI1) — from shared.hpp Pins --------------
static constexpr uint PIN_MISO = 8;   // GPIO  8, phys 11
static constexpr uint PIN_NSS  = 9;   // GPIO  9, phys 12 — CS
static constexpr uint PIN_SCK  = 10;  // GPIO 10, phys 14
static constexpr uint PIN_MOSI = 11;  // GPIO 11, phys 15
static constexpr uint PIN_RST  = 26;  // GPIO 26, phys 31 — reset
static constexpr uint PIN_DIO0 = 27;  // GPIO 27, phys 32 — G0 / IRQ (PayloadReady)
static constexpr uint PIN_EN   = 28;  // GPIO 28, phys 34 — power enable (active high)

// -- LoRa1 / RF69 air config — from LoRa1Cfg in shared.hpp ----------------------
static constexpr float    FREQ_MHZ  = 433.0f;
static constexpr float    BR_KBPS   = 4.8f;    // bit rate
static constexpr float    FDEV_KHZ  = 5.0f;    // frequency deviation
static constexpr float    RXBW_KHZ  = 125.0f;  // RX channel filter bandwidth
static constexpr int8_t   RX_DBM    = 13;      // RX-only; PA power is irrelevant
static constexpr uint16_t PREAMBLE  = 16;      // preamble length in bits

// CSV identity for this tool.
static constexpr const char* ROLE = "rx";
static constexpr const char* MOD  = "gfsk";

// HAL + radio. Declared static/global because RadioLib keeps internal pointers
// into the HAL and Module objects.
static PicoHal hal( spi1, static_cast<uint8_t>( PIN_SCK ),
                          static_cast<uint8_t>( PIN_MOSI ),
                          static_cast<uint8_t>( PIN_MISO ) );
static Module  module_( &hal, PIN_NSS, PIN_DIO0, PIN_RST, RADIOLIB_NC );
static RF69    radio( &module_ );

// Diagnostic init. begin() returned -25 (UNSUPPORTED) and nothing in RadioLib's
// RF69 begin sequence should return that — so probe explicitly: read the chip
// version register, then run each config setter WITHOUT bailing, printing every
// status code. This pinpoints the offending call and the chip revision in one
// flash. (RadioLib RF69 expects version 0x24 = SX1231 rev 2D.)
static int radio_init()
{
    int err = radio.begin();   // also initializes the SPI/HAL
    printf( "# [rx] begin() = %d\n", err );

    const int16_t ver = radio.getChipVersion();
    printf( "# [rx] chip version reg = 0x%02X (raw %d; RadioLib wants 0x24)\n",
            (unsigned)( ver & 0xFF ), ver );

    int16_t e;
    e = radio.standby();                       printf( "# [rx] standby = %d\n", e );
    e = radio.setFrequency( FREQ_MHZ );        printf( "# [rx] setFrequency = %d\n", e );
    e = radio.setBitRate( BR_KBPS );           printf( "# [rx] setBitRate = %d\n", e );
    e = radio.setRxBandwidth( RXBW_KHZ );      printf( "# [rx] setRxBandwidth = %d\n", e );
    e = radio.setFrequencyDeviation( FDEV_KHZ );printf( "# [rx] setFrequencyDeviation = %d\n", e );
    e = radio.setOutputPower( RX_DBM );        printf( "# [rx] setOutputPower = %d\n", e );
    e = radio.setPreambleLength( PREAMBLE );   printf( "# [rx] setPreambleLength = %d\n", e );
    e = radio.setDataShaping( RADIOLIB_SHAPING_0_5 ); printf( "# [rx] setDataShaping = %d\n", e );

    return err;
}

int main()
{
    stdio_init_all();

    // Wait for the USB CDC host so we don't lose the early log lines.
    while ( !stdio_usb_connected() ) {
        sleep_ms( 100 );
    }
    sleep_ms( 500 );

    // The RFM69 is gated behind a power-enable MOSFET — turn it on and let the
    // supply settle before talking to the chip.
    gpio_init( PIN_EN );
    gpio_set_dir( PIN_EN, GPIO_OUT );
    gpio_put( PIN_EN, 1 );
    printf( "# [rx] power enabled (GPIO%u), settling 1 s...\n", PIN_EN );
    sleep_ms( 1000 );

    printf( "# [rx] init RFM69HCW @ %.1f MHz GFSK %.1f kbps fdev=%.1f kHz "
            "rxbw=%.0f kHz\n",
            (double)FREQ_MHZ, (double)BR_KBPS, (double)FDEV_KHZ, (double)RXBW_KHZ );

    int state = radio_init();
    if ( state != RADIOLIB_ERR_NONE ) {
        printf( "# [rx] init failed, code %d — check wiring/power. Halting.\n", state );
        while ( true ) { sleep_ms( 1000 ); }
    }
    printf( "# [rx] RFM69HCW ready. Listening...\n" );
    rf_csv_header();

    // Continuous receive; DIO0 maps to PayloadReady in RX packet mode.
    radio.startReceive();

    uint8_t  buf[64];   // RF69 FIFO depth
    uint32_t last_floor_ms = 0;

    // -- Link statistics -------------------------------------------------------
    // Loss is derived from the "#N" sequence number the TX embeds: any gap in
    // N between two good packets is counted as lost. CRC errors are counted
    // separately (we can't trust their payload, so they don't carry a seq).
    // FSK has no SNR/ferr, so those CSV columns are left blank for this tool.
    bool     have_seq   = false;   // seen at least one parseable seq yet
    uint32_t last_seq   = 0;       // last good seq seen
    uint32_t good_count = 0;       // good packets received
    uint32_t lost_count = 0;       // missing seq numbers (gaps)
    uint32_t crc_count  = 0;       // CRC failures

    for ( ;; ) {
        const uint32_t now = to_ms_since_boot( get_absolute_time() );

        // -- Packet arrived? ---------------------------------------------------
        if ( gpio_get( PIN_DIO0 ) ) {
            size_t len = static_cast<size_t>( radio.getPacketLength() );
            if ( len > sizeof(buf) - 1 ) len = sizeof(buf) - 1;
            const int err = radio.readData( buf, len );

            // RF69 reads the live RSSI register, so grab it right after the
            // packet (still reflects the just-received signal).
            const float rssi = radio.getRSSI();

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
                            rssi, NAN, NAN,
                            (long)good_count, (long)lost_count, (long)crc_count,
                            per, -1 );
            } else if ( err == RADIOLIB_ERR_CRC_MISMATCH ) {
                // Energy received but corrupt — still proof the front end hears
                // something.
                ++crc_count;
                const uint32_t expected = good_count + lost_count;
                const float per = expected ? ( 100.0f * lost_count / expected ) : 0.0f;
                rf_csv_row( now, ROLE, FREQ_MHZ, MOD, "crc_error",
                            -1, (long)len, rssi, NAN, NAN,
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
            // RF69 getRSSI() reads the live RSSI register; in continuous RX this
            // is the current channel level (the noise floor when idle).
            const float floor = radio.getRSSI();
            rf_csv_row( now, ROLE, FREQ_MHZ, MOD, "noise_floor",
                        -1, -1, floor, NAN, NAN, -1, -1, -1, NAN, -1 );
        }

        sleep_ms( 5 );
    }

    return 0;
}
