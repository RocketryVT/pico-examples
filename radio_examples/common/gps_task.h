// gps_task.h — UART0 u-blox GPS support for the RF bench-test tools.
//
// A cooperative "task": rather than a FreeRTOS thread or a second core, decoded
// GPS state is serviced from each tool's existing super-loop. UART RX bytes are
// first captured into a small RAM ring from the UART0 IRQ so 230400-baud mixed
// NMEA/UBX bursts survive radio/logging/USB work between gps_task_poll() calls.
// The IRQ handler is kept out of flash; flash writes still briefly disable
// interrupts, so the debug output reports ring overflows if bytes are lost.
//
// Wiring is supplied by each board/example:
//   tx_pin = Pico UART0 TX -> GPS RX   (carries the UBX config)
//   rx_pin = Pico UART0 RX <- GPS TX   (carries NAV-PVT)
//   plus GND and 3V3/5V to the module.
//
// On gps_task_init() the module is auto-configured to emit UBX NAV-PVT at 1 Hz.
// NMEA is left enabled as a fallback for M10-compatible modules that report
// useful fixes there before/while UBX NAV-PVT settles — see gps_task.cpp.
//
// Usage (see any main.cpp):
//   gps_task_init( tx_pin, rx_pin, baud ); // UART0 + configure NAV-PVT
//   ...
//   for (;;) {
//       gps_task_poll();                   // drain UART, decode fixes
//       rf_csv_set_gps( gps_task_fix() );  // stamp gps_*/utc onto next rows
//       ...
//   }
#pragma once

#include "rf_gps.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bring up UART0 on the supplied board pins/baud and push the UBX NAV-PVT
// configuration to the module. Prints a "# [gps]" status comment. Returns true
// on success (always true unless the UART can't be brought up; absence of a
// module just means no fixes arrive).
bool gps_task_init( uint8_t tx_pin, uint8_t rx_pin, uint32_t baud );

// Detect the module's current UART baud from valid NMEA/UBX traffic, switch it
// to target_baud with UBX CFG-UART1-BAUDRATE when needed, then configure NAV-PVT.
// Falls back to opening target_baud directly if no valid stream is detected.
bool gps_task_init_autobaud( uint8_t tx_pin, uint8_t rx_pin, uint32_t target_baud );

// Detect the module's current UART baud from valid NMEA/UBX traffic, but do not
// send any GPS configuration commands. Use this for M10050/GM10 modules after
// u-center has verified the output stream.
bool gps_task_init_autobaud_listen_only( uint8_t tx_pin, uint8_t rx_pin,
                                         uint32_t preferred_baud );

// Drain the UART ring and decode any complete NMEA/UBX frames. Call every loop
// iteration.
void gps_task_poll( void );

// Print each valid UBX NAV-PVT frame as one '#'-prefixed raw hex line. This is
// for GPS debugging on the USB console; decoded CSV logging is unchanged.
void gps_task_set_nav_pvt_debug( bool enabled );

// Latest fix as a CSV-ready snapshot (valid flags inside). Cheap; copies a POD.
RfGps gps_task_fix( void );

// True once the module has reported a usable position fix.
bool gps_task_has_fix( void );

#ifdef __cplusplus
}
#endif
