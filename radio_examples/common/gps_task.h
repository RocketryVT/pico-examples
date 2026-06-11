// gps_task.h — UART0 u-blox GPS support for the RF bench-test tools.
//
// A cooperative "task": rather than a FreeRTOS thread or a second core, the GPS
// is serviced from each tool's existing super-loop. This keeps the flash logger
// in rf_log.cpp valid — that code erases/programs flash with interrupts simply
// disabled, which is only safe because nothing else (no other core, no ISR) is
// executing from flash at the same time. Polling here costs microseconds.
//
// Wiring (Pico 2 W UART0, free in every example — radios use SPI0/SPI1):
//   GPIO 0 (phys 1)  = UART0 TX -> GPS RX   (carries the UBX config)
//   GPIO 1 (phys 2)  = UART0 RX <- GPS TX   (carries NAV-PVT)
//   plus GND and 3V3 to the module.
//
// On gps_task_init() the module is auto-configured to emit UBX NAV-PVT only
// (NMEA silenced) at 1 Hz — see gps_task.cpp.
//
// Usage (see any main.cpp):
//   gps_task_init();                       // UART0 + configure NAV-PVT
//   ...
//   for (;;) {
//       gps_task_poll();                   // drain UART, decode fixes
//       rf_csv_set_gps( gps_task_fix() );  // stamp gps_*/utc onto next rows
//       ...
//   }
#pragma once

#include "rf_gps.h"

#ifdef __cplusplus
extern "C" {
#endif

// Bring up UART0 and push the UBX NAV-PVT configuration to the module. Prints a
// "# [gps]" status comment. Returns true on success (always true unless the
// UART can't be brought up; absence of a module just means no fixes arrive).
bool gps_task_init( void );

// Drain the UART and decode any complete UBX frames. Call every loop iteration.
void gps_task_poll( void );

// Latest fix as a CSV-ready snapshot (valid flags inside). Cheap; copies a POD.
RfGps gps_task_fix( void );

// True once the module has reported a usable position fix.
bool gps_task_has_fix( void );

#ifdef __cplusplus
}
#endif
