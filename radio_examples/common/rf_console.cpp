// rf_console.cpp — see rf_console.h.

#include "rf_console.h"
#include "rf_log.h"

#include "pico/stdlib.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace {

char   g_line[64];
size_t g_len = 0;

void print_help()
{
    printf( "# commands:\n"
            "#   list          list stored log files (index + size)\n"
            "#   export <n>    dump the whole CSV of log <n>\n"
            "#   format yes    ERASE all logs and start a fresh file\n"
            "#   help          this message\n" );
}

void dispatch( char* line )
{
    while ( *line == ' ' || *line == '\t' ) {
        ++line;  // skip leading whitespace
    }
    if ( *line == '\0' ) {
        return;
    }

    if ( strcmp( line, "list" ) == 0 || strcmp( line, "ls" ) == 0 ) {
        rf_log_list();
    } else if ( strncmp( line, "export", 6 ) == 0 || strncmp( line, "cat", 3 ) == 0 ) {
        const char* sp = strchr( line, ' ' );
        char*       end = nullptr;
        const long  idx = sp ? strtol( sp, &end, 10 ) : 0;
        if ( !sp || end == sp ) {
            printf( "# usage: export <n>   (see 'list')\n" );
        } else {
            rf_log_export( (int)idx );
        }
    } else if ( strncmp( line, "format", 6 ) == 0 ) {
        // Destructive — require an explicit "format yes" to confirm.
        const char* arg = line + 6;
        while ( *arg == ' ' || *arg == '\t' ) {
            ++arg;
        }
        if ( strcmp( arg, "yes" ) == 0 ) {
            rf_log_format();
        } else {
            printf( "# format ERASES all logs. Type 'format yes' to confirm.\n" );
        }
    } else if ( strcmp( line, "help" ) == 0 || strcmp( line, "?" ) == 0 ) {
        print_help();
    } else {
        printf( "# unknown command '%s' (try 'help')\n", line );
    }
}

}  // namespace

void rf_console_poll( void )
{
    int c;
    while ( ( c = getchar_timeout_us( 0 ) ) != PICO_ERROR_TIMEOUT ) {
        if ( c == '\r' || c == '\n' ) {
            if ( g_len > 0 ) {
                g_line[g_len] = '\0';
                dispatch( g_line );
                g_len = 0;
            }
        } else if ( c == 0x08 || c == 0x7F ) {   // backspace / delete
            if ( g_len > 0 ) {
                --g_len;
            }
        } else if ( g_len < sizeof( g_line ) - 1 ) {
            g_line[g_len++] = (char)c;
        } else {
            g_len = 0;   // overrun — drop the oversized line
        }
    }
}
