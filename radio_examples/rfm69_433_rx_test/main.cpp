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
#include "rf_log.h"         // mirror the CSV stream into on-chip flash (../common/)
#include "gps_task.h"       // UART0 u-blox GPS — UTC + position stamping (../common/)
#include "rf_console.h"     // USB console task + log_print() (../common/)
#include "rtos.h"           // FreeRTOS task helpers (../common/)

#include "FreeRTOS.h"
#include "task.h"

#include "boards/board.hpp"   // HAS_* flags + Pins:: + Board:: (board + project profile)

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

// This tool needs a 433 MHz RFM69 and a GPS plugged into the carrier. Assert on
// the device matrix from board_profile.hpp (not just the coarse category gate),
// so pointing it at a board/profile that lacks the actual chip fails loudly.
static_assert(HAS_RADIO, "rfm69_433_rx_test requires a radio (set APP_HAS_RADIO in board_profile.hpp)");
static_assert(HAS_GPS,   "rfm69_433_rx_test requires a GPS (set APP_HAS_GPS in board_profile.hpp)");
static_assert(Board::RadioCount > 0, "rfm69_433_rx_test requires at least one Board::Radios entry");
static_assert(Board::GpsCount > 0, "rfm69_433_rx_test requires at least one Board::Gpses entry");
static_assert(Board::has_model(Board::Radios, Board::RadioModel::RFM69HCW),
              "rfm69_433_rx_test requires an RFM69HCW in Board::Radios");

// The RFM69 instance this tool drives (first 433-capable radio in the profile).
static constexpr Board::RadioInstance RADIO = Board::Radios[0];
static constexpr Board::GpsInstance GPS = Board::Gpses[0];

static_assert(RADIO.model == Board::RadioModel::RFM69HCW,
              "rfm69_433_rx_test requires Board::Radios[0] to be an RFM69HCW");
static_assert(RADIO.bus == Board::Bus::SPI1,
              "rfm69_433_rx_test currently supports the RFM69 on SPI1 only");
static_assert(GPS.bus == Board::Bus::UART0,
              "rfm69_433_rx_test currently supports GPS on UART0 only");
static_assert(GPS.nav_hz > 0, "GPS nav_hz must be non-zero");
static_assert(GPS.nav_hz <= Board::spec_of(GPS.model).max_nav_hz,
              "GPS nav_hz exceeds the selected receiver's device spec");

// -- RFM69 wiring (its SPI bus connector) — from the active board's Pins:: map --
// (Radios[0].cs_pin carries the CS; the rest of the SPI1 connector is Pins::LORA1_*.)
static constexpr uint PIN_MISO = Pins::LORA1_MISO;
static constexpr uint PIN_NSS  = RADIO.cs_pin;      // CS
static constexpr uint PIN_SCK  = Pins::LORA1_SCK;
static constexpr uint PIN_MOSI = Pins::LORA1_MOSI;
static constexpr uint PIN_RST  = Pins::LORA1_RST;   // reset
static constexpr uint PIN_DIO0 = Pins::LORA1_DIO0;  // G0 / IRQ (PayloadReady)
static constexpr uint PIN_EN   = Pins::LORA1_EN;    // power enable (active high)

// -- RFM69 config: operating freq from the instance; link params from the profile.
static constexpr float    FREQ_MHZ  = RADIO.freq_mhz;
static constexpr float    BR_KBPS   = Board::Rfm433::BR_KBPS;
static constexpr float    FDEV_KHZ  = Board::Rfm433::FDEV_KHZ;
static constexpr float    RXBW_KHZ  = Board::Rfm433::RXBW_KHZ;
static constexpr int8_t   RX_DBM    = Board::Rfm433::RX_DBM;
static constexpr uint16_t PREAMBLE  = Board::Rfm433::PREAMBLE;
static constexpr uint8_t  PACKET_LEN = 32;
// Operating freq must be within the chip's range (spec_of from devices.hpp).
static_assert(FREQ_MHZ >= Board::spec_of(RADIO.model).freq_min_mhz &&
              FREQ_MHZ <= Board::spec_of(RADIO.model).freq_max_mhz,
              "RFM69 operating frequency outside the device's supported band");

// CSV identity for this tool.
static constexpr const char* ROLE = "rx";
static constexpr const char* MOD  = "gfsk";

// GPS wiring (UART0 connector) from Pins::; baud from the GPS instance.
static constexpr uint8_t  GPS_TX_PIN = Pins::GPS_TX;
static constexpr uint8_t  GPS_RX_PIN = Pins::GPS_RX;
static constexpr uint32_t GPS_BAUD   = GPS.baud;

// HAL + radio. Declared static/global because RadioLib keeps internal pointers
// into the HAL and Module objects.
static PicoHal hal( spi1, static_cast<uint8_t>( PIN_SCK ),
                          static_cast<uint8_t>( PIN_MOSI ),
                          static_cast<uint8_t>( PIN_MISO ) );
static Module  module_( &hal, PIN_NSS, PIN_DIO0, PIN_RST, RADIOLIB_NC );
static RF69    radio( &module_ );

// FreeRTOS task priorities (console runs at tskIDLE_PRIORITY+1 internally).
static constexpr UBaseType_t GPS_TASK_PRIORITY   = 3;
static constexpr UBaseType_t RADIO_TASK_PRIORITY = 2;

static StaticTask_t s_radio_tcb;
static StackType_t  s_radio_stack[4096];

static int log_init_step( const char* label, int state )
{
    log_print( "# [rx] %s = %d\n", label, state );
    return state;
}

static int radio_init()
{
    ConfigFSK_t cfg;
    cfg.frequency          = FREQ_MHZ;
    cfg.bitRate            = BR_KBPS;
    cfg.frequencyDeviation = FDEV_KHZ;
    cfg.receiverBandwidth  = RXBW_KHZ;
    cfg.power              = RX_DBM;
    cfg.preambleLength     = PREAMBLE;

    int err = log_init_step( "begin", radio.begin( cfg ) );
    if ( err != RADIOLIB_ERR_NONE ) return err;

    // Match the transmitter's fixed-length test packet. This avoids the RF69
    // variable-length FIFO byte so the sanity test exercises only RF payload IO.
    err = log_init_step( "fixedPacketLengthMode", radio.fixedPacketLengthMode( PACKET_LEN ) );
    if ( err != RADIOLIB_ERR_NONE ) return err;

    // GFSK: Gaussian shaping, BT = 0.5. Must match the transmitter.
    err = log_init_step( "setDataShaping", radio.setDataShaping( RADIOLIB_SHAPING_0_5 ) );
    if ( err != RADIOLIB_ERR_NONE ) return err;

    return RADIOLIB_ERR_NONE;
}

// -- Radio task: RFM69 init + continuous-receive loop (core 1) ----------------
static void radio_task( void* )
{
    // The RFM69 is gated behind a power-enable MOSFET — turn it on and let the
    // supply settle before talking to the chip.
    gpio_init( PIN_EN );
    gpio_set_dir( PIN_EN, GPIO_OUT );
    gpio_put( PIN_EN, 1 );
    log_print( "# [rx] power enabled (GPIO%u), settling 1 s...\n", PIN_EN );
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );

    log_print( "# [rx] init RFM69HCW @ %.1f MHz GFSK %.1f kbps fdev=%.1f kHz "
               "rxbw=%.0f kHz\n",
               (double)FREQ_MHZ, (double)BR_KBPS, (double)FDEV_KHZ, (double)RXBW_KHZ );

    int state = radio_init();
    if ( state != RADIOLIB_ERR_NONE ) {
        log_print( "# [rx] init failed, code %d — check wiring/power.\n", state );
        for ( ;; ) { vTaskDelay( pdMS_TO_TICKS( 1000 ) ); }
    }
    log_print( "# [rx] RFM69HCW ready. Listening...\n" );
    rf_csv_header();

    // Continuous receive; DIO0 maps to PayloadReady in RX packet mode.
    radio.startReceive();

    uint8_t  buf[PACKET_LEN + 1];
    uint32_t last_floor_ms = 0;

    // -- Link statistics -------------------------------------------------------
    bool     have_seq   = false;   // seen at least one parseable seq yet
    uint32_t last_seq   = 0;       // last good seq seen
    uint32_t good_count = 0;       // good packets received
    uint32_t lost_count = 0;       // missing seq numbers (gaps)
    uint32_t crc_count  = 0;       // CRC failures

    for ( ;; ) {
        const uint32_t now = to_ms_since_boot( get_absolute_time() );

        // Refresh the GPS fix stamped onto this iteration's rows.
        rf_csv_set_gps( gps_task_fix() );

        // -- Packet arrived? ---------------------------------------------------
        const uint8_t irq2 = module_.SPIreadRegister( RADIOLIB_RF69_REG_IRQ_FLAGS_2 );
        if ( irq2 & RADIOLIB_RF69_IRQ_PAYLOAD_READY ) {
            size_t len = PACKET_LEN;
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

            // Flush buffered log bytes to flash about once a second.
            rf_log_sync();
        }

        vTaskDelay( pdMS_TO_TICKS( 2 ) );
    }
}

int main()
{
    stdio_init_all();

    // Brief, non-blocking window for a USB host to attach (queued log lines are
    // flushed once the console task runs).
    for ( int i = 0; i < 20 && !stdio_usb_connected(); ++i ) {
        sleep_ms( 100 );
    }
    sleep_ms( 200 );

    rf_console_start();

    if ( rf_log_init( ROLE ) ) {
        rf_csv_set_sink( rf_log_write );
    }

    // Bring up the profile GPS and auto-configure UBX NAV-PVT (stamps utc/gps_*).
    gps_task_init_nav_hz( GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD, GPS.nav_hz );

    rf_csv_set_stdout_enabled( false );
    log_print( "# console: 'list' / 'export <n>' / 'live on' (or 'help') over USB\n" );

    gps_task_start( GPS_TASK_PRIORITY );
    TaskHandle_t h = rtos_task_create( radio_task, "radio", 4096, nullptr,
                                       RADIO_TASK_PRIORITY, s_radio_stack, &s_radio_tcb );
    vTaskCoreAffinitySet( h, 1u << 1 );  // core 1

    vTaskStartScheduler();
    for ( ;; ) {}
}
