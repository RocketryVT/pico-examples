// lfs_pico_flash.c — see lfs_pico_flash.h.

#include "lfs_pico_flash.h"

#include "hardware/flash.h"   // flash_range_program / flash_range_erase, FLASH_*
#include "hardware/sync.h"    // save_and_disable_interrupts / restore_interrupts
#include "hardware/regs/addressmap.h"  // XIP_BASE

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

static int pico_prog( const struct lfs_config* c, lfs_block_t block,
                      lfs_off_t off, const void* buffer, lfs_size_t size )
{
    (void)c;
    const uint32_t target = fs_flash_offset( block, off );
    const uint32_t ints   = save_and_disable_interrupts();
    flash_range_program( target, (const uint8_t*)buffer, size );
    restore_interrupts( ints );
    return LFS_ERR_OK;
}

static int pico_erase( const struct lfs_config* c, lfs_block_t block )
{
    (void)c;
    const uint32_t target = fs_flash_offset( block, 0 );
    const uint32_t ints   = save_and_disable_interrupts();
    flash_range_erase( target, FS_BLOCK_SIZE );
    restore_interrupts( ints );
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
