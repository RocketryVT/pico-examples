// rf_log.cpp — see rf_log.h.

#include "rf_log.h"

#include "lfs.h"
#include "lfs_pico_flash.h"

#include "hardware/flash.h"   // FLASH_PAGE_SIZE (per-file cache buffer size)

#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace {

lfs_t             g_lfs;
struct lfs_config g_cfg;
lfs_file_t        g_file;
struct lfs_file_config g_file_cfg;

// Per-file cache buffer so lfs_file_opencfg() never touches the heap. Sized to
// the block device cache (one program page).
uint8_t g_file_buf[FLASH_PAGE_SIZE];

bool g_ready      = false; // FS mounted and file open
bool g_full       = false; // hit ENOSPC — stop trying to write
char g_name[24]   = { 0 }; // "/rx_0007.csv"
char g_prefix[12] = { 0 }; // "rx"/"tx" — remembered so format can reopen

// Scan the FS root for files named "<prefix>_NNNN.csv" and return one past the
// highest NNNN seen (0 if none). Lets each boot start a fresh, ordered file.
int next_index_for( const char* prefix )
{
    lfs_dir_t dir;
    if ( lfs_dir_open( &g_lfs, &dir, "/" ) < 0 ) {
        return 0;
    }

    const size_t plen   = strlen( prefix );
    int          max_ix = -1;
    struct lfs_info info;

    while ( lfs_dir_read( &g_lfs, &dir, &info ) > 0 ) {
        if ( info.type != LFS_TYPE_REG ) {
            continue;
        }
        // Expect "<prefix>_" ... ".csv".
        if ( strncmp( info.name, prefix, plen ) != 0 || info.name[plen] != '_' ) {
            continue;
        }
        const char* num = info.name + plen + 1;
        char*       end = nullptr;
        const long  ix  = strtol( num, &end, 10 );
        if ( end != num && strcmp( end, ".csv" ) == 0 && ix > max_ix ) {
            max_ix = (int)ix;
        }
    }

    lfs_dir_close( &g_lfs, &dir );
    return max_ix + 1;
}

// -- Directory listing support (for the USB "list"/"export" commands) ---------
constexpr int MAX_LISTED = 64;   // plenty for bench captures

struct FileEntry {
    char     name[32];
    uint32_t size;
};

// Static (not stack) so list/export don't blow the modest main-loop stack.
FileEntry g_entries[MAX_LISTED];

// Snapshot the regular files in the FS root into g_entries; returns the count
// (capped at MAX_LISTED) or -1 on error.
int collect_files()
{
    lfs_dir_t dir;
    if ( lfs_dir_open( &g_lfs, &dir, "/" ) < 0 ) {
        return -1;
    }

    int             n = 0;
    struct lfs_info info;
    while ( n < MAX_LISTED && lfs_dir_read( &g_lfs, &dir, &info ) > 0 ) {
        if ( info.type != LFS_TYPE_REG ) {
            continue;
        }
        strncpy( g_entries[n].name, info.name, sizeof( g_entries[n].name ) - 1 );
        g_entries[n].name[sizeof( g_entries[n].name ) - 1] = '\0';
        g_entries[n].size = info.size;
        ++n;
    }

    lfs_dir_close( &g_lfs, &dir );
    return n;
}

// Open a fresh "/<g_prefix>_NNNN.csv" (one past the highest existing index) on
// the already-mounted FS. Shared by rf_log_init() and rf_log_format(). Leaves
// g_name set and the file open on success.
bool open_new_log()
{
    const int ix = next_index_for( g_prefix );
    snprintf( g_name, sizeof( g_name ), "/%s_%04d.csv", g_prefix, ix );

    memset( &g_file_cfg, 0, sizeof( g_file_cfg ) );
    g_file_cfg.buffer = g_file_buf;

    // RDWR (not WRONLY) so rf_log_export() can read the active file back out
    // through this same handle (littlefs forbids a second handle to an open
    // file). Writes still append sequentially from the start.
    const int err = lfs_file_opencfg( &g_lfs, &g_file, g_name,
                                      LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC,
                                      &g_file_cfg );
    if ( err ) {
        printf( "# [log] open '%s' failed (lfs %d)\n", g_name, err );
        return false;
    }
    return true;
}

// True if entry name (no leading slash) is the currently open log file.
bool is_active( const char* name )
{
    return g_name[0] != '\0' && strcmp( name, g_name + 1 ) == 0;
}

// Stream a closed file straight from flash to stdout (own read-only handle +
// its own cache buffer — safe because it's a different file from g_file).
void dump_closed( const char* name )
{
    static uint8_t          rbuf[FLASH_PAGE_SIZE];
    lfs_file_t              f;
    struct lfs_file_config  fc;
    memset( &fc, 0, sizeof( fc ) );
    fc.buffer = rbuf;

    char path[40];
    snprintf( path, sizeof( path ), "/%s", name );

    if ( lfs_file_opencfg( &g_lfs, &f, path, LFS_O_RDONLY, &fc ) < 0 ) {
        printf( "# export: open '%s' failed\n", path );
        return;
    }

    uint8_t     buf[256];
    lfs_ssize_t n;
    while ( ( n = lfs_file_read( &g_lfs, &f, buf, sizeof( buf ) ) ) > 0 ) {
        fwrite( buf, 1, (size_t)n, stdout );
    }
    lfs_file_close( &g_lfs, &f );
}

// Dump the currently open log via its existing handle: flush, rewind, read to
// EOF, then seek back to the end so subsequent rows keep appending.
void dump_active()
{
    lfs_file_sync( &g_lfs, &g_file );
    const lfs_soff_t end = lfs_file_size( &g_lfs, &g_file );
    if ( end < 0 ) {
        printf( "# export: size query failed\n" );
        return;
    }

    lfs_file_seek( &g_lfs, &g_file, 0, LFS_SEEK_SET );
    uint8_t     buf[256];
    lfs_ssize_t n;
    while ( ( n = lfs_file_read( &g_lfs, &g_file, buf, sizeof( buf ) ) ) > 0 ) {
        fwrite( buf, 1, (size_t)n, stdout );
    }
    lfs_file_seek( &g_lfs, &g_file, end, LFS_SEEK_SET );  // resume appending
}

}  // namespace

bool rf_log_init( const char* prefix )
{
    // Remember the prefix so rf_log_format() can reopen a fresh file later.
    strncpy( g_prefix, prefix, sizeof( g_prefix ) - 1 );
    g_prefix[sizeof( g_prefix ) - 1] = '\0';

    lfs_pico_flash_config( &g_cfg );

    const int err = lfs_pico_flash_mount( &g_lfs, &g_cfg );
    if ( err ) {
        printf( "# [log] mount/format failed (lfs %d) — USB-only logging\n", err );
        return false;
    }

    if ( !open_new_log() ) {
        printf( "# [log] USB-only logging\n" );
        return false;
    }

    g_ready = true;
    g_full  = false;
    printf( "# [log] writing to %s  (%lu bytes free)\n",
            g_name, rf_log_bytes_free() );
    return true;
}

void rf_log_write( const char* line )
{
    if ( !g_ready || g_full ) {
        return;
    }

    const lfs_size_t len = (lfs_size_t)strlen( line );
    const lfs_ssize_t n  = lfs_file_write( &g_lfs, &g_file, line, len );
    if ( n < 0 ) {
        // ENOSPC (or any write error): stop here so we don't spam the FS.
        g_full = true;
        printf( "# [log] %s write failed (lfs %d) — file logging stopped\n",
                g_name, (int)n );
    }
}

void rf_log_sync( void )
{
    if ( g_ready && !g_full ) {
        lfs_file_sync( &g_lfs, &g_file );
    }
}

unsigned long rf_log_bytes_free( void )
{
    const lfs_ssize_t used = lfs_fs_size( &g_lfs );  // blocks in use
    if ( used < 0 ) {
        return 0;
    }
    const lfs_size_t free_blocks =
        ( g_cfg.block_count > (lfs_size_t)used )
            ? g_cfg.block_count - (lfs_size_t)used
            : 0;
    return (unsigned long)free_blocks * g_cfg.block_size;
}

void rf_log_list( void )
{
    if ( !g_ready ) {
        printf( "# [log] no filesystem mounted\n" );
        return;
    }

    const int n = collect_files();
    if ( n < 0 ) {
        printf( "# [log] list failed\n" );
        return;
    }

    printf( "# logs (%d, %lu bytes free):\n", n, rf_log_bytes_free() );
    for ( int i = 0; i < n; ++i ) {
        printf( "#   %d  %-16s %8lu bytes%s\n",
                i, g_entries[i].name, (unsigned long)g_entries[i].size,
                is_active( g_entries[i].name ) ? "  (active)" : "" );
    }
    if ( n == 0 ) {
        printf( "#   (none)\n" );
    }
    printf( "# use 'export <n>' to dump one\n" );
}

bool rf_log_export( int idx )
{
    if ( !g_ready ) {
        printf( "# [log] no filesystem mounted\n" );
        return false;
    }

    const int n = collect_files();
    if ( n <= 0 ) {
        printf( "# export: no logs (run 'list')\n" );
        return false;
    }
    if ( idx < 0 || idx >= n ) {
        printf( "# export: index %d out of range (0..%d)\n", idx, n - 1 );
        return false;
    }

    const char* name   = g_entries[idx].name;
    const bool  active = is_active( name );

    // Markers are '#'-prefixed so a serial capture's CSV parser skips them; the
    // raw file bytes (its own header + rows) sit between. Nothing else is
    // printed during the dump — the cooperative loop is blocked here — so no
    // live rows interleave.
    printf( "# ---- begin export %s (%lu bytes)%s ----\n",
            name, (unsigned long)g_entries[idx].size, active ? " active" : "" );

    if ( active ) {
        dump_active();
    } else {
        dump_closed( name );
    }

    printf( "\n# ---- end export %s ----\n", name );
    return true;
}

bool rf_log_format( void )
{
    // Close the active log and unmount before formatting (lfs_format needs an
    // unmounted FS).
    if ( g_ready ) {
        lfs_file_close( &g_lfs, &g_file );
        lfs_unmount( &g_lfs );
        g_ready = false;
    }

    // Ensure the block-device config is populated even if init never mounted.
    lfs_pico_flash_config( &g_cfg );

    int err = lfs_format( &g_lfs, &g_cfg );
    if ( err ) {
        printf( "# [log] format failed (lfs %d)\n", err );
        return false;
    }
    err = lfs_mount( &g_lfs, &g_cfg );
    if ( err ) {
        printf( "# [log] remount after format failed (lfs %d)\n", err );
        return false;
    }

    // Fall back to a default prefix if format is somehow run before init.
    if ( g_prefix[0] == '\0' ) {
        strncpy( g_prefix, "log", sizeof( g_prefix ) - 1 );
    }

    if ( !open_new_log() ) {
        return false;
    }

    g_ready = true;
    g_full  = false;
    printf( "# [log] formatted — now writing to %s  (%lu bytes free)\n",
            g_name, rf_log_bytes_free() );
    return true;
}
