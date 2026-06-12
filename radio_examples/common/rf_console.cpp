// rf_console.cpp — see rf_console.h.

#include "rf_console.h"
#include "rf_log.h"
#include "rf_csv.h"
#include "rtos.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "pico/stdlib.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdarg>

namespace {

// -- Log queue ----------------------------------------------------------------
// One ordered stream of text from every task to the console task. 384-byte
// messages fit the widest CSV row and the GPS diagnostic line.
struct LogMessage {
    char buf[384];
};

constexpr int LOG_QUEUE_DEPTH = 48;

QueueHandle_t  g_log_queue = nullptr;
StaticQueue_t  g_log_queue_buf;
uint8_t        g_log_queue_storage[ LOG_QUEUE_DEPTH * sizeof( LogMessage ) ];

StaticTask_t   g_console_tcb;
StackType_t    g_console_stack[ 2048 ];

// -- Command handling ---------------------------------------------------------

void print_help()
{
    printf( "# commands:\n"
            "#   list          list stored log files (index + size)\n"
            "#   export <n>    dump the whole CSV of log <n>\n"
            "#   format yes    ERASE all logs and start a fresh file\n"
            "#   live [on|off] echo live CSV rows to USB (default off)\n"
            "#   help          this message\n" );
}

void dispatch( char* line )
{
    while ( *line == ' ' || *line == '\t' ) {
        ++line;
    }
    if ( *line == '\0' ) {
        return;
    }

    if ( strcmp( line, "list" ) == 0 || strcmp( line, "ls" ) == 0 ) {
        rf_log_list();
    } else if ( strncmp( line, "export", 6 ) == 0 || strncmp( line, "cat", 3 ) == 0 ) {
        const char* sp  = strchr( line, ' ' );
        char*       end = nullptr;
        const long  idx = sp ? strtol( sp, &end, 10 ) : 0;
        if ( !sp || end == sp ) {
            printf( "# usage: export <n>   (see 'list')\n" );
        } else {
            rf_log_export( (int)idx );
        }
    } else if ( strncmp( line, "format", 6 ) == 0 ) {
        const char* arg = line + 6;
        while ( *arg == ' ' || *arg == '\t' ) {
            ++arg;
        }
        if ( strcmp( arg, "yes" ) == 0 ) {
            rf_log_format();
        } else {
            printf( "# format ERASES all logs. Type 'format yes' to confirm.\n" );
        }
    } else if ( strncmp( line, "live", 4 ) == 0 ) {
        const char* arg = line + 4;
        while ( *arg == ' ' || *arg == '\t' ) {
            ++arg;
        }
        bool on;
        if      ( strcmp( arg, "on" )  == 0 ) on = true;
        else if ( strcmp( arg, "off" ) == 0 ) on = false;
        else                                  on = true;   // bare "live" -> on
        rf_csv_set_stdout_enabled( on );
        printf( "# live CSV echo: %s\n", on ? "on" : "off" );
    } else if ( strcmp( line, "help" ) == 0 || strcmp( line, "?" ) == 0 ) {
        print_help();
    } else {
        printf( "# unknown command '%s' (try 'help')\n", line );
    }
}

// -- Console task -------------------------------------------------------------
// Pinned to core 0 (TinyUSB IRQ core). Sole owner of printf()/getchar().
void console_task( void* )
{
    static char line[96];
    size_t      len = 0;

    for ( ;; ) {
        // 1. Drain queued output from other tasks.
        LogMessage msg;
        bool printed = false;
        while ( xQueueReceive( g_log_queue, &msg, 0 ) == pdTRUE ) {
            fputs( msg.buf, stdout );
            printed = true;
        }
        if ( printed ) {
            stdio_flush();
        }

        // 2. Read any typed command bytes (non-blocking).
        int c;
        while ( ( c = getchar_timeout_us( 0 ) ) != PICO_ERROR_TIMEOUT && c >= 0 ) {
            if ( c == '\r' || c == '\n' ) {
                if ( len > 0 ) {
                    line[len] = '\0';
                    dispatch( line );
                    len = 0;
                }
            } else if ( c == 0x08 || c == 0x7F ) {
                if ( len > 0 ) --len;
            } else if ( len < sizeof( line ) - 1 ) {
                line[len++] = (char)c;
            } else {
                len = 0;
            }
        }

        // Poll cadence: fast enough that an export drains promptly, slow enough
        // to leave core 0 mostly idle between bursts.
        vTaskDelay( pdMS_TO_TICKS( 5 ) );
    }
}

}  // namespace

void rf_console_start( void )
{
    g_log_queue = xQueueCreateStatic( LOG_QUEUE_DEPTH, sizeof( LogMessage ),
                                      g_log_queue_storage, &g_log_queue_buf );

    TaskHandle_t h = rtos_task_create( console_task, "console", 2048, nullptr,
                                       tskIDLE_PRIORITY + 1,
                                       g_console_stack, &g_console_tcb );
    vTaskCoreAffinitySet( h, 1u << 0 );  // core 0 — TinyUSB IRQ core
}

void log_print( const char* fmt, ... )
{
    if ( !g_log_queue ) {
        return;
    }
    LogMessage msg;
    va_list args;
    va_start( args, fmt );
    vsnprintf( msg.buf, sizeof( msg.buf ), fmt, args );
    va_end( args );
    xQueueSend( g_log_queue, &msg, 0 );
}

void log_puts( const char* s )
{
    if ( !g_log_queue ) {
        return;
    }
    LogMessage msg;
    snprintf( msg.buf, sizeof( msg.buf ), "%s", s );
    xQueueSend( g_log_queue, &msg, 0 );
}
