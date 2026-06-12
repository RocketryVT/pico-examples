// gps_task.cpp — see gps_task.h.

#include "gps_task.h"

// Pico headers first: gps_driver.hpp only compiles its UART/I2C transport
// wrappers when PICO_SDK_VERSION_MAJOR is already defined (pico/version.h,
// pulled in by pico/stdlib.h).
#include "pico/stdlib.h"
#include "pico/version.h"

#include "gps/gps_driver.hpp"

#include <cstdio>
#include <optional>

namespace {

// -- UART0 link config ---------------------------------------------------------
constexpr uint16_t GPS_RATE_MS = 1000; // 1 Hz navigation solution
constexpr uint32_t GPS_DIAG_MS = 5000; // low-rate "why are fields blank?" hint

struct GpsTaskConfig {
    uint8_t  tx_pin = 0;      // Pico UART0 TX -> GPS RX
    uint8_t  rx_pin = 1;      // Pico UART0 RX <- GPS TX
    uint32_t baud   = 115200;
};

// The driver owns the transport by reference, so both must outlive every poll.
// Constructed in gps_task_init() (not at static-init time) because UartTransport
// touches hardware that isn't ready until clocks/stdio are up.
std::optional<gps::UartTransport>                 g_uart;
std::optional<gps::GpsDriver<gps::UartTransport>> g_driver;
GpsTaskConfig g_config;
bool g_ready = false;

void open_uart( const GpsTaskConfig& config )
{
    g_ready = false;
    g_driver.reset();
    g_uart.reset();
    uart_deinit( uart0 );

    g_config = config;
    g_uart.emplace( uart0, g_config.tx_pin, g_config.rx_pin, g_config.baud );
    g_driver.emplace( *g_uart );
}

void configure_nav_pvt()
{
    // Auto-configure M10-style receivers the same way as the ground-station GPS
    // task. The older CFG-PRT/CFG-MSG sequence is not reliable on these modules.
    g_driver->send_ubx( gps::Ubx::valset_uart1_inprot_ubx( true ) );
    g_driver->send_ubx( gps::Ubx::valset_uart1_inprot_nmea( true ) );
    g_driver->send_ubx( gps::Ubx::valset_uart1_outprot_ubx( true ) );
    g_driver->send_ubx( gps::Ubx::valset_uart1_outprot_nmea( false ) );
    g_driver->send_ubx( gps::Ubx::valset_nav_pvt_uart1( 1 ) );
    g_driver->send_ubx( gps::Ubx::valset_rate_meas( GPS_RATE_MS ) );
    g_driver->send_ubx( gps::Ubx::valset_dyn_model( 2 ) );  // stationary bench test
    sleep_ms( 200 );  // let the module apply the config before we poll.
}

void feed_raw( const uint8_t* buf, size_t n )
{
    for ( size_t i = 0; i < n; ++i ) {
        g_driver->feed( buf[i] );
    }
}

bool detect_valid_stream( uint32_t wait_ms )
{
    const absolute_time_t end = make_timeout_time_ms( wait_ms );
    uint8_t buf[gps::GpsDriver<gps::UartTransport>::READ_CHUNK];

    while ( !time_reached( end ) ) {
        const size_t n = g_driver->read_raw( buf, sizeof buf );
        feed_raw( buf, n );

        const auto& d = g_driver->diagnostics();
        if ( d.nmea_good > 0 || d.ubx_frames > 0 ) {
            return true;
        }

        sleep_ms( 1 );
    }

    return false;
}

void print_raw_sample( const uint8_t* sample, size_t count )
{
    printf( "# [gps] raw hex:" );
    for ( size_t i = 0; i < count; ++i ) {
        printf( " %02X", sample[i] );
    }
    printf( "\n# [gps] raw ascii: " );
    for ( size_t i = 0; i < count; ++i ) {
        const uint8_t c = sample[i];
        putchar( ( c >= 32 && c <= 126 ) ? c : '.' );
    }
    putchar( '\n' );
}

// Build the CSV snapshot from the driver's current Coordinate.
RfGps snapshot()
{
    RfGps out;
    if ( !g_ready ) {
        return out;
    }

    const gps::Coordinate& c = g_driver->coordinate();
    out.valid = c.valid;

    if ( c.valid ) {
        out.has_pos = true;
        out.lat     = c.latitude;
        out.lon     = c.longitude;
        out.alt_m   = c.altitude;
    }

    // utc_year == 0 means the time hasn't been resolved yet.
    if ( c.utc_year != 0 ) {
        const uint32_t ms  = c.utc_ms;
        const unsigned hh  = ( ms / 3600000u ) % 24u;
        const unsigned mm  = ( ms / 60000u ) % 60u;
        const unsigned ss  = ( ms / 1000u ) % 60u;
        snprintf( out.utc, sizeof out.utc, "%04u-%02u-%02uT%02u:%02u:%02uZ",
                  (unsigned)c.utc_year, (unsigned)c.utc_month, (unsigned)c.utc_day,
                  hh, mm, ss );
        out.has_time = true;
    }

    return out;
}

}  // namespace

bool gps_task_init( uint8_t tx_pin, uint8_t rx_pin, uint32_t baud )
{
    open_uart( GpsTaskConfig{ .tx_pin = tx_pin, .rx_pin = rx_pin, .baud = baud } );
    configure_nav_pvt();

    g_ready = true;
    printf( "# [gps] UART0 @ %lu baud on GPIO%u/%u — UBX NAV-PVT @ %u ms\n",
            (unsigned long)g_config.baud,
            (unsigned)g_config.tx_pin,
            (unsigned)g_config.rx_pin,
            (unsigned)GPS_RATE_MS );
    return true;
}

bool gps_task_init_autobaud( uint8_t tx_pin, uint8_t rx_pin, uint32_t target_baud )
{
    static constexpr uint32_t BAUDS[] = {
        230400, 115200, 57600, 38400, 19200, 9600, 4800, 460800, 921600,
    };

    uint32_t current_baud = 0;
    for ( uint32_t baud : BAUDS ) {
        open_uart( GpsTaskConfig{ .tx_pin = tx_pin, .rx_pin = rx_pin, .baud = baud } );
        if ( detect_valid_stream( 1200 ) ) {
            current_baud = baud;
            const auto& d = g_driver->diagnostics();
            printf( "# [gps] detected UART0 @ %lu baud (ubx=%lu nmea=%lu)\n",
                    (unsigned long)current_baud,
                    (unsigned long)d.ubx_frames,
                    (unsigned long)d.nmea_good );
            break;
        }
    }

    if ( current_baud == 0 ) {
        printf( "# [gps] autobaud failed; trying target baud %lu directly\n",
                (unsigned long)target_baud );
        return gps_task_init( tx_pin, rx_pin, target_baud );
    }

    if ( current_baud != target_baud ) {
        printf( "# [gps] switching UART1 baud %lu -> %lu\n",
                (unsigned long)current_baud,
                (unsigned long)target_baud );
        g_driver->send_ubx( gps::Ubx::valset_uart1_baud( target_baud ) );
        sleep_ms( 200 );
        open_uart( GpsTaskConfig{ .tx_pin = tx_pin, .rx_pin = rx_pin, .baud = target_baud } );
    }

    configure_nav_pvt();

    g_ready = true;
    printf( "# [gps] UART0 @ %lu baud on GPIO%u/%u — UBX NAV-PVT @ %u ms\n",
            (unsigned long)g_config.baud,
            (unsigned)g_config.tx_pin,
            (unsigned)g_config.rx_pin,
            (unsigned)GPS_RATE_MS );
    return true;
}

void gps_task_poll( void )
{
    if ( g_ready ) {
        // Module is configured UBX-out only, but tolerate stray NMEA at startup.
        uint8_t buf[gps::GpsDriver<gps::UartTransport>::READ_CHUNK];
        const size_t n = g_driver->read_raw( buf, sizeof buf );
        feed_raw( buf, n );

        static uint32_t diag_bytes = 0;
        static uint8_t  diag_sample[48];
        static size_t   diag_sample_count = 0;

        diag_bytes += static_cast<uint32_t>( n );
        for ( size_t i = 0; i < n && diag_sample_count < sizeof diag_sample; ++i ) {
            diag_sample[diag_sample_count++] = buf[i];
        }

        static uint32_t last_diag_ms = 0;
        const uint32_t now = to_ms_since_boot( get_absolute_time() );
        if ( now - last_diag_ms >= GPS_DIAG_MS ) {
            last_diag_ms = now;
            const auto& d = g_driver->diagnostics();
            if ( !g_driver->has_fix() ) {
                printf( "# [gps] no fix yet: bytes=%lu ubx=%lu ubx_pvt=%lu nmea=%lu bad_nmea=%lu fix=%.*s\n",
                        (unsigned long)diag_bytes,
                        (unsigned long)d.ubx_frames,
                        (unsigned long)d.ubx_pvt,
                        (unsigned long)d.nmea_good,
                        (unsigned long)d.nmea_bad_cksum,
                        (int)g_driver->fix_label().size(),
                        g_driver->fix_label().data() );
                if ( diag_sample_count > 0 && d.ubx_pvt == 0 && d.nmea_good == 0 ) {
                    print_raw_sample( diag_sample, diag_sample_count );
                }
            }
            diag_bytes = 0;
            diag_sample_count = 0;
        }
    }
}

RfGps gps_task_fix( void )
{
    return snapshot();
}

bool gps_task_has_fix( void )
{
    return g_ready && g_driver->has_fix();
}
