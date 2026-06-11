// rf_log.h — persist the RF bench-test CSV stream to on-chip flash via littlefs.
//
// Each of the four radio tools already prints its CSV to USB serial. This adds
// a durable copy on the Pico's flash (the 3 MB file region defined in
// lfs_pico_flash.h) so a log survives an unplugged laptop or a field test with
// no host attached.
//
// Wiring it up (see any main.cpp):
//   rf_log_init( "rx" );                 // mount FS, open /rx_NNNN.csv
//   rf_csv_set_sink( rf_log_write );     // mirror every CSV row into the file
//   ... rf_csv_header(); rf_csv_row(...) as before ...
//   rf_log_sync();                       // call periodically; flushes to flash
//
// Reading logs back off the board: each tool prints "# [log] dump ..." support
// is out of scope here — pull the file region with picotool, or extend this
// with a dump-over-serial command.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Mount the filesystem and open a fresh log file named "/<prefix>_NNNN.csv",
// where NNNN is one past the highest existing index for that prefix (so every
// power-up starts a new file without clobbering older runs). Prints a "# [log]"
// status comment to stdout. Returns true on success; on failure the other
// rf_log_* calls become no-ops and the tool keeps running USB-only.
bool rf_log_init( const char* prefix );

// Append a CSV line (as produced by rf_csv.h) to the open log file. Safe to
// pass to rf_csv_set_sink(). No-op if rf_log_init() failed or the FS is full.
void rf_log_write( const char* line );

// Flush buffered bytes to flash. Cheap to call often; the tools call it on a
// timer so at most a second or two of data is at risk on a power cut.
void rf_log_sync( void );

// Bytes still free in the file region (approximate; for the "# [log]" banner).
unsigned long rf_log_bytes_free( void );

// Print a numbered list of stored log files to stdout (the "list" command).
// Each line is '#'-prefixed so a serial CSV capture skips it.
void rf_log_list( void );

// Dump the whole CSV of the log at display-index idx (as shown by rf_log_list)
// to stdout, wrapped in "# ---- begin/end export ----" markers. Handles the
// currently-active log too. Returns false if idx is out of range / no FS.
bool rf_log_export( int idx );

// Erase every stored log (reformat the file region) and reopen a fresh log file
// for the current run. Destructive — all previous captures are lost. Returns
// true on success.
bool rf_log_format( void );

#ifdef __cplusplus
}
#endif
