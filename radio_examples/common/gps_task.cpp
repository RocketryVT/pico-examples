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

// -- UART0 wiring + link config -----------------------------------------------
constexpr uint     GPS_TX_PIN = 0;     // GPIO 0 — UART0 TX -> GPS RX
constexpr uint     GPS_RX_PIN = 1;     // GPIO 1 — UART0 RX <- GPS TX
constexpr uint32_t GPS_BAUD   = 9600;  // u-blox factory default
constexpr uint16_t GPS_RATE_MS = 1000; // 1 Hz navigation solution

// The driver owns the transport by reference, so both must outlive every poll.
// Constructed in gps_task_init() (not at static-init time) because UartTransport
// touches hardware that isn't ready until clocks/stdio are up.
std::optional<gps::UartTransport>                 g_uart;
std::optional<gps::GpsDriver<gps::UartTransport>> g_driver;
bool g_ready = false;

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

bool gps_task_init( void )
{
    g_uart.emplace( uart0, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD );
    g_driver.emplace( *g_uart );

    // Auto-configure: UBX in/out, NMEA silenced, NAV-PVT enabled at 1 Hz.
    // Keep the module's UART1 at GPS_BAUD so it matches the Pico UART we opened
    // (the convenience configure() overload would default it to 38400 and break
    // the link). CFG-PRT is sent at the current baud, which the module already
    // uses at power-up, so it is received correctly before any baud change.
    g_driver->configure( gps::GpsDriver<gps::UartTransport>::ConfigOptions{
        .port         = gps::Port::UART1,
        .baud         = GPS_BAUD,
        .in_proto     = gps::InProto::UBX | gps::InProto::NMEA,
        .out_proto    = gps::OutProto::UBX,
        .meas_rate_ms = GPS_RATE_MS,
    } );
    sleep_ms( 100 );  // let the module apply the config before we poll.

    g_ready = true;
    printf( "# [gps] UART0 @ %lu baud on GPIO%u/%u — UBX NAV-PVT @ %u ms\n",
            (unsigned long)GPS_BAUD, GPS_TX_PIN, GPS_RX_PIN,
            (unsigned)GPS_RATE_MS );
    return true;
}

void gps_task_poll( void )
{
    if ( g_ready ) {
        // Module is configured UBX-out only, but tolerate stray NMEA at startup.
        g_driver->poll();
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
