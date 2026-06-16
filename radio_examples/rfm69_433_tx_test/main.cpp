// main.cpp — 433 MHz GFSK transmit-only sanity test for the ground-station Pico.
//
// Puts the RFM69HCW (RF69, LoRa1, SPI1) into GFSK transmit and sends a short
// numbered test packet once a second, printing the result + on-air time over
// USB serial.
//
// Sibling of rfm69_433_rx_test. Pins, SPI bus and radio config match the
// ground-station LoRa1 / RF69 path (projects/ground_station/pico/src/shared.hpp
// + lora1_task.cpp) so the two ends agree. GFSK = FSK with Gaussian shaping
// (BT=0.5); both ends must use the same shaping.

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

#include "boards/board.hpp"   // HAS_* flags + Pins:: + Board:: device profile

#include <cstdio>
#include <cstring>
#include <cmath>

static_assert(HAS_RADIO, "rfm69_433_tx_test requires a radio (set APP_HAS_RADIO in board_profile.hpp)");
static_assert(HAS_GPS,   "rfm69_433_tx_test requires a GPS (set APP_HAS_GPS in board_profile.hpp)");
static_assert(Board::RadioCount > 0, "rfm69_433_tx_test requires at least one Board::Radios entry");
static_assert(Board::GpsCount > 0, "rfm69_433_tx_test requires at least one Board::Gpses entry");

static constexpr Board::RadioInstance RADIO = Board::Radios[0];
static constexpr Board::GpsInstance GPS = Board::Gpses[0];

static_assert(RADIO.model == Board::RadioModel::RFM69HCW,
              "rfm69_433_tx_test requires Board::Radios[0] to be an RFM69HCW");
static_assert(RADIO.bus == Board::Bus::SPI1,
              "rfm69_433_tx_test currently supports the RFM69 on SPI1 only");
static_assert(GPS.bus == Board::Bus::UART0,
              "rfm69_433_tx_test currently supports GPS on UART0 only");
static_assert(GPS.nav_hz > 0, "GPS nav_hz must be non-zero");
static_assert(GPS.nav_hz <= Board::spec_of(GPS.model).max_nav_hz,
              "GPS nav_hz exceeds the selected receiver's device spec");
static_assert(RADIO.freq_mhz >= Board::spec_of(RADIO.model).freq_min_mhz &&
              RADIO.freq_mhz <= Board::spec_of(RADIO.model).freq_max_mhz,
              "RFM69 operating frequency outside the device's supported band");

// -- LoRa1 / 433 MHz RFM69HCW wiring (SPI1) — from the active board profile ----
static constexpr uint PIN_MISO = Pins::LORA1_MISO;
static constexpr uint PIN_NSS  = RADIO.cs_pin;      // CS
static constexpr uint PIN_SCK  = Pins::LORA1_SCK;
static constexpr uint PIN_MOSI = Pins::LORA1_MOSI;
static constexpr uint PIN_RST  = Pins::LORA1_RST;   // reset
static constexpr uint PIN_DIO0 = Pins::LORA1_DIO0;  // G0 / IRQ (TxDone)
static constexpr uint PIN_EN   = Pins::LORA1_EN;    // power enable (active high)

// -- RF69 air config from this target's board_profile.hpp ----------------------
static constexpr float    FREQ_MHZ   = RADIO.freq_mhz;
static constexpr float    BR_KBPS    = Board::Rfm433::BR_KBPS;
static constexpr float    FDEV_KHZ   = Board::Rfm433::FDEV_KHZ;
static constexpr float    RXBW_KHZ   = Board::Rfm433::RXBW_KHZ;
static constexpr int8_t   TX_DBM     = Board::Rfm433::TX_DBM;
static constexpr uint16_t PREAMBLE   = Board::Rfm433::PREAMBLE;
static constexpr bool     HIGH_POWER = Board::Rfm433::HIGH_POWER;
static constexpr uint8_t  PACKET_LEN = 32;

// CSV identity for this tool.
static constexpr const char* ROLE = "tx";
static constexpr const char* MOD  = "gfsk";

// GPS wiring comes from the active board pin map; baud/rate come from profile.
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
    log_print( "# [tx] %s = %d\n", label, state );
    return state;
}

// RF69 GFSK + HCW high-power init. begin() can't set >13 dBm with the PA-boost
// flag, so cap power at 13 for begin() then re-apply the real power with
// high_power=true.
static int radio_init()
{
    const int8_t init_power = ( HIGH_POWER && TX_DBM > 13 ) ? 13 : TX_DBM;

    ConfigFSK_t cfg;
    cfg.frequency          = FREQ_MHZ;
    cfg.bitRate            = BR_KBPS;
    cfg.frequencyDeviation = FDEV_KHZ;
    cfg.receiverBandwidth  = RXBW_KHZ;
    cfg.power              = init_power;
    cfg.preambleLength     = PREAMBLE;

    int err = log_init_step( "begin", radio.begin( cfg ) );
    if ( err != RADIOLIB_ERR_NONE ) return err;

    // Use fixed-length packet mode for this sanity test. It avoids depending on
    // the RF69 variable-length FIFO byte, which is what we are trying to prove.
    err = log_init_step( "fixedPacketLengthMode", radio.fixedPacketLengthMode( PACKET_LEN ) );
    if ( err != RADIOLIB_ERR_NONE ) return err;

    // GFSK: Gaussian shaping, BT = 0.5. Must match the receiver.
    err = log_init_step( "setDataShaping", radio.setDataShaping( RADIOLIB_SHAPING_0_5 ) );
    if ( err != RADIOLIB_ERR_NONE ) return err;

    // Re-apply output power with the PA-boost flag for the HCW variant.
    if ( HIGH_POWER ) {
        err = log_init_step( "setOutputPowerHigh", radio.setOutputPower( TX_DBM, true ) );
        if ( err != RADIOLIB_ERR_NONE ) return err;
    }
    return RADIOLIB_ERR_NONE;
}

static int transmit_packet( const uint8_t* data, size_t len )
{
    int state = radio.startTransmit( data, len );
    if ( state != RADIOLIB_ERR_NONE ) return state;

    const uint32_t timeout_ms =
        10u + static_cast<uint32_t>( ( static_cast<float>( len * 8u ) / BR_KBPS ) * 5.0f );
    const uint32_t start_ms = to_ms_since_boot( get_absolute_time() );

    for ( ;; ) {
        const uint8_t irq2 = module_.SPIreadRegister( RADIOLIB_RF69_REG_IRQ_FLAGS_2 );
        if ( gpio_get( PIN_DIO0 ) || ( irq2 & RADIOLIB_RF69_IRQ_PACKET_SENT ) ) {
            return radio.finishTransmit();
        }

        if ( to_ms_since_boot( get_absolute_time() ) - start_ms > timeout_ms ) {
            radio.finishTransmit();
            return RADIOLIB_ERR_TX_TIMEOUT;
        }

        vTaskDelay( pdMS_TO_TICKS( 1 ) );
    }
}

// -- Radio task: RFM69 init + 1 Hz beacon loop (core 1) -----------------------
static void radio_task( void* )
{
    // The RFM69 is gated behind a power-enable MOSFET — turn it on and let the
    // supply settle before talking to the chip.
    gpio_init( PIN_EN );
    gpio_set_dir( PIN_EN, GPIO_OUT );
    gpio_put( PIN_EN, 1 );
    log_print( "# [tx] power enabled (GPIO%u), settling 1 s...\n", PIN_EN );
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );

    log_print( "# [tx] init RFM69HCW @ %.1f MHz GFSK %.1f kbps fdev=%.1f kHz "
               "rxbw=%.0f kHz pwr=%d dBm\n",
               (double)FREQ_MHZ, (double)BR_KBPS, (double)FDEV_KHZ,
               (double)RXBW_KHZ, (int)TX_DBM );

    int state = radio_init();
    if ( state != RADIOLIB_ERR_NONE ) {
        log_print( "# [tx] init failed, code %d — check wiring/power.\n", state );
        for ( ;; ) { vTaskDelay( pdMS_TO_TICKS( 1000 ) ); }
    }
    log_print( "# [tx] RFM69HCW ready. Transmitting...\n" );
    rf_csv_header();

    uint8_t  msg[PACKET_LEN];
    uint32_t count = 0;

    for ( ;; ) {
        memset( msg, 0, sizeof(msg) );
        snprintf( reinterpret_cast<char*>( msg ), sizeof(msg),
                  "rfm69_433_tx_test #%08lu", (unsigned long)count );

        const uint32_t t0  = to_ms_since_boot( get_absolute_time() );
        state = transmit_packet( msg, PACKET_LEN );
        const uint32_t t1  = to_ms_since_boot( get_absolute_time() );
        const uint32_t air = t1 - t0;

        // Stamp the freshest GPS fix onto this beacon's row.
        rf_csv_set_gps( gps_task_fix() );

        rf_csv_row( t1, ROLE, FREQ_MHZ, MOD,
                    ( state == RADIOLIB_ERR_NONE ) ? "tx_ok" : "tx_fail",
                    (long)count, (long)PACKET_LEN,
                    NAN, NAN, NAN, -1, -1, -1, NAN, (long)air );

        rf_log_sync();   // flush this row to flash before the next beacon.

        ++count;
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );   // 1 Hz beacon
    }
}

int main()
{
    stdio_init_all();

    // Brief, non-blocking window for a USB host to attach.
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
