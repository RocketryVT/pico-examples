// lfs_pico_flash.h — littlefs block device backed by the Pico 2 W on-chip flash.
//
// Flash layout (Pico 2 W has 4 MB of QSPI flash):
//
//   0x000000 ┌──────────────────────────┐
//            │ program (.uf2 image)      │  1 MB  — must stay below FS_OFFSET
//   0x100000 ├──────────────────────────┤
//            │ littlefs file region      │  3 MB  — CSV logs live here
//   0x400000 └──────────────────────────┘
//
// Reads are served straight from the memory-mapped XIP window; programs and
// erases go through the SDK flash API with interrupts disabled (these are
// single-core super-loop tools, so no multicore lockout is needed).
//
// Usage:
//   lfs_t lfs;
//   struct lfs_config cfg;
//   lfs_pico_flash_config( &cfg );
//   lfs_pico_flash_mount( &lfs, &cfg );   // formats automatically on first boot
#pragma once

#include "lfs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Offset and size (bytes) of the littlefs region within the on-chip flash.
// 1 MB reserved for the program image, the remaining 3 MB for files.
#define LFS_PICO_FS_OFFSET ( 1u * 1024u * 1024u )
#define LFS_PICO_FS_SIZE   ( 3u * 1024u * 1024u )

// Populate a lfs_config with the on-chip-flash block device and static buffers.
// The config refers to internal static storage, so only one filesystem may be
// mounted at a time (true for every tool in this directory).
void lfs_pico_flash_config( struct lfs_config* cfg );

// Mount the filesystem, formatting it once if the mount fails (first boot or a
// corrupted region). Returns an lfs error code (0 == success).
int lfs_pico_flash_mount( lfs_t* lfs, struct lfs_config* cfg );

#ifdef __cplusplus
}
#endif
