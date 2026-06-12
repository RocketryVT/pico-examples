// lfs_pico_flash.c — see lfs_pico_flash.h.

#include "lfs_pico_flash.h"

#include "hardware/flash.h"   // flash_range_program / flash_range_erase, FLASH_*
#include "hardware/sync.h"    // save_and_disable_interrupts / restore_interrupts
#include "hardware/regs/addressmap.h"  // XIP_BASE
#include "pico/flash.h"       // flash_safe_execute (SMP-safe under FreeRTOS)

#include "FreeRTOS.h"         // xTaskGetSchedulerState
#include "task.h"

#include <string.h>

// littlefs block geometry, tied to the flash device.
//   read/prog granularity = one 256-byte program page
//   erase granularity      = one 4096-byte sector
#define FS_READ_SIZE  FLASH_PAGE_SIZE     // 256
#define FS_PROG_SIZE  FLASH_PAGE_SIZE     // 256
#define FS_BLOCK_SIZE FLASH_SECTOR_SIZE   // 4096
#define FS_BLOCK_COUNT ( LFS_PICO_FS_SIZE / FS_BLOCK_SIZE )

// Static caches so littlefs never touches the heap for the filesystem itself.
static uint8_t s_read_buf[FS_READ_SIZE];
static uint8_t s_prog_buf[FS_PROG_SIZE];
static uint8_t s_lookahead_buf[32] __attribute__( ( aligned( 8 ) ) );

// Byte offset of (block, off) from the start of flash (offset 0 == XIP_BASE).
static inline uint32_t fs_flash_offset( lfs_block_t block, lfs_off_t off )
{
    return LFS_PICO_FS_OFFSET + (uint32_t)block * FS_BLOCK_SIZE + (uint32_t)off;
}

static int pico_read( const struct lfs_config* c, lfs_block_t block,
                      lfs_off_t off, void* buffer, lfs_size_t size )
{
    (void)c;
    const uint8_t* src =
        (const uint8_t*)( XIP_BASE + fs_flash_offset( block, off ) );
    memcpy( buffer, src, size );
    return LFS_ERR_OK;
}

// One flash erase/program request, marshalled to flash_safe_execute().
struct flash_op {
    uint32_t       target;
    const uint8_t* buffer;  // program only
    uint32_t       size;    // program only
    bool           erase;
};

// Runs with both cores guaranteed not to be executing from flash (either via
// flash_safe_execute's lockout, or because the scheduler isn't started yet so
// only this core is live).
static void __not_in_flash_func( do_flash_op )( void* param )
{
    const struct flash_op* op = (const struct flash_op*)param;
    if ( op->erase ) {
        flash_range_erase( op->target, FS_BLOCK_SIZE );
    } else {
        flash_range_program( op->target, op->buffer, op->size );
    }
}

// Perform an erase/program safely. Under the running SMP scheduler this uses
// flash_safe_execute() so the *other* core is parked (and not running from XIP)
// during the operation. Before the scheduler starts (first-boot format/mount)
// only this core is live, so a plain interrupt-disable is sufficient.
static void run_flash_op( struct flash_op* op )
{
    if ( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING ) {
        if ( flash_safe_execute( do_flash_op, op, 2000 ) == PICO_OK ) {
            return;
        }
        // Fallback: should not happen on a 2-core SMP build, but never leave the
        // write undone — disable interrupts locally and proceed.
    }
    const uint32_t ints = save_and_disable_interrupts();
    do_flash_op( op );
    restore_interrupts( ints );
}

static int pico_prog( const struct lfs_config* c, lfs_block_t block,
                      lfs_off_t off, const void* buffer, lfs_size_t size )
{
    (void)c;
    struct flash_op op = {
        .target = fs_flash_offset( block, off ),
        .buffer = (const uint8_t*)buffer,
        .size   = size,
        .erase  = false,
    };
    run_flash_op( &op );
    return LFS_ERR_OK;
}

static int pico_erase( const struct lfs_config* c, lfs_block_t block )
{
    (void)c;
    struct flash_op op = {
        .target = fs_flash_offset( block, 0 ),
        .buffer = NULL,
        .size   = 0,
        .erase  = true,
    };
    run_flash_op( &op );
    return LFS_ERR_OK;
}

static int pico_sync( const struct lfs_config* c )
{
    (void)c;
    return LFS_ERR_OK;  // writes hit flash synchronously; nothing to flush.
}

void lfs_pico_flash_config( struct lfs_config* cfg )
{
    memset( cfg, 0, sizeof( *cfg ) );

    cfg->read  = pico_read;
    cfg->prog  = pico_prog;
    cfg->erase = pico_erase;
    cfg->sync  = pico_sync;

    cfg->read_size      = FS_READ_SIZE;
    cfg->prog_size      = FS_PROG_SIZE;
    cfg->block_size     = FS_BLOCK_SIZE;
    cfg->block_count    = FS_BLOCK_COUNT;
    cfg->cache_size     = FS_PROG_SIZE;
    cfg->lookahead_size = sizeof( s_lookahead_buf );
    cfg->block_cycles   = 500;  // wear-level the metadata blocks periodically.

    cfg->read_buffer      = s_read_buf;
    cfg->prog_buffer      = s_prog_buf;
    cfg->lookahead_buffer = s_lookahead_buf;
}

int lfs_pico_flash_mount( lfs_t* lfs, struct lfs_config* cfg )
{
    int err = lfs_mount( lfs, cfg );
    if ( err ) {
        // No (or corrupt) filesystem yet — format then mount.
        err = lfs_format( lfs, cfg );
        if ( err ) {
            return err;
        }
        err = lfs_mount( lfs, cfg );
    }
    return err;
}
