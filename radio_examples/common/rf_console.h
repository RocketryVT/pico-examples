// rf_console.h — tiny USB-serial command console for the RF bench-test tools.
//
// Type a line into the USB serial terminal and press Enter:
//   list            list the stored log files with index + size
//   export <n>      dump the whole CSV of log <n> (index from `list`)
//   help            show the command list
//
// Cooperative, like the GPS task: rf_console_poll() drains whatever input bytes
// are queued (non-blocking) and only acts on a completed line. An `export`
// runs to completion inside that call, so the loop is briefly blocked and no
// live CSV rows interleave with the dump.
//
// Usage (see any main.cpp):
//   for (;;) { ...; rf_console_poll(); ... }
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Drain queued USB input and execute any completed command line. Call every
// loop iteration. No-op when no input is waiting.
void rf_console_poll( void );

#ifdef __cplusplus
}
#endif
