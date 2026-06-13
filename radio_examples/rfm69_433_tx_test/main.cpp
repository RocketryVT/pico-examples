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

#include "boards/board_pins.hpp"   // Pins:: for the active PICO_BOARD (board-neutral)

#include <cstdio>
#include <cstring>
#include <cmath>

// -- LoRa1 / 433 MHz RFM69HCW wiring (SPI1) — from gs_pcb_v1 board pin map ------
static constexpr uint PIN_MISO = Pins::LORA1_MISO;
static constexpr uint PIN_NSS  = Pins::LORA1_NSS;   // CS
static constexpr uint PIN_SCK  = Pins::LORA1_SCK;
static constexpr uint PIN_MOSI = Pins::LORA1_MOSI;
static constexpr uint PIN_RST  = Pins::LORA1_RST;   // reset
static constexpr uint PIN_DIO0 = Pins::LORA1_DIO0;  // G0 / IRQ (TxDone)
static constexpr uint PIN_EN   = Pins::LORA1_EN;    // power enable (active high)

// -- LoRa1 / RF69 air config — from LoRa1Cfg in shared.hpp ----------------------
static constexpr float    FREQ_MHZ  = 433.0f;
static constexpr float    BR_KBPS   = 4.8f;    // bit rate
static constexpr float    FDEV_KHZ  = 5.0f;    // frequency deviation
static constexpr float    RXBW_KHZ  = 125.0f;  // RX channel filter bandwidth
static constexpr int8_t   TX_DBM    = 20;      // +20 dBm with PA boost (HCW)
static constexpr uint16_t PREAMBLE  = 16;      // preamble length in bits
static constexpr bool     HIGH_POWER = true;   // RFM69HCW PA-boost variant

// CSV identity for this tool.
static constexpr const char* ROLE = "tx";
static constexpr const char* MOD  = "gfsk";

// GPS wiring — from gs_pcb_v1 board pin map (UART0 TX=12, RX=13).
static constexpr uint8_t  GPS_TX_PIN = Pins::GPS_TX;
static constexpr uint8_t  GPS_RX_PIN = Pins::GPS_RX;
static constexpr uint32_t GPS_BAUD   = 115200;  // Flywoo GM10 Nano V3.1 default

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

// RF69 GFSK + HCW high-power init. Mirrors the ground-station RF69 wrapper:
// begin() can't set >13 dBm with the PA-boost flag, so cap power at 13 for
// begin() then re-apply the real power with high_power=true.
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

    int err = radio.begin( cfg );
    if ( err != RADIOLIB_ERR_NONE ) return err;

    // GFSK: Gaussian shaping, BT = 0.5. Must match the receiver.
    err = radio.setDataShaping( RADIOLIB_SHAPING_0_5 );
    if ( err != RADIOLIB_ERR_NONE ) return err;

    // Re-apply output power with the PA-boost flag for the HCW variant.
    if ( HIGH_POWER ) {
        err = radio.setOutputPower( TX_DBM, true );
        if ( err != RADIOLIB_ERR_NONE ) return err;
    }
    return RADIOLIB_ERR_NONE;
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

    char     msg[64];
    uint32_t count = 0;

    for ( ;; ) {
        snprintf( msg, sizeof(msg), "rfm69_433_tx_test #%lu", (unsigned long)count );
        const size_t len = strlen( msg );

        const uint32_t t0  = to_ms_since_boot( get_absolute_time() );
        state = radio.transmit( reinterpret_cast<uint8_t*>( msg ),
                                static_cast<size_t>( len ) );
        const uint32_t t1  = to_ms_since_boot( get_absolute_time() );
        const uint32_t air = t1 - t0;

        // Stamp the freshest GPS fix onto this beacon's row.
        rf_csv_set_gps( gps_task_fix() );

        rf_csv_row( t1, ROLE, FREQ_MHZ, MOD,
                    ( state == RADIOLIB_ERR_NONE ) ? "tx_ok" : "tx_fail",
                    (long)count, (long)len,
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

    // Bring up the UART0 GPS and auto-configure UBX NAV-PVT (stamps utc/gps_*).
    gps_task_init( GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD );

    rf_csv_set_stdout_enabled( false );
    log_print( "# console: 'list' / 'export <n>' / 'live on' (or 'help') over USB\n" );

    gps_task_start( GPS_TASK_PRIORITY );
    TaskHandle_t h = rtos_task_create( radio_task, "radio", 4096, nullptr,
                                       RADIO_TASK_PRIORITY, s_radio_stack, &s_radio_tcb );
    vTaskCoreAffinitySet( h, 1u << 1 );  // core 1

    vTaskStartScheduler();
    for ( ;; ) {}
}
