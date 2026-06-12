// rf_console.h — USB console task + logging for the RF bench-test tools.
//
// Under FreeRTOS a single console task (pinned to core 0, where the TinyUSB IRQ
// lives) owns ALL USB serial I/O. No other task calls printf/getchar directly.
// Instead they enqueue output with log_print()/log_puts(); the console task
// drains the queue and prints it, and reads typed commands.
//
// Commands (type a line + Enter in the serial terminal):
//   list            list the stored log files with index + size
//   export <n>      dump the whole CSV of log <n> (index from `list`)
//   format yes      ERASE all logs and start a fresh file
//   live [on|off]   echo live CSV rows to USB (default off; flash always logs)
//   help            show the command list
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Create the log queue and start the console task pinned to core 0. Call this
// BEFORE any log_print()/log_puts() and before creating other tasks, so early
// startup messages are queued (and flushed once the scheduler runs).
void rf_console_start( void );

// printf-style log — formats into a queued message that the console task prints.
// Safe from any FreeRTOS task; never call from an ISR. Drops the message if the
// queue is full. Messages logged before the scheduler starts are queued and
// flushed when the console task first runs.
void log_print( const char* fmt, ... ) __attribute__( ( format( printf, 1, 2 ) ) );

// Enqueue a pre-formatted line verbatim (used by rf_csv for CSV rows). Same
// task/ISR rules as log_print().
void log_puts( const char* s );

#ifdef __cplusplus
}
#endif
