// rf_csv.h — common CSV output schema for the RF bench-test tools.
//
// Shared by lora915_rx_test / lora915_tx_test / rfm69_433_rx_test /
// rfm69_433_tx_test so every log is the same shape and the analysis scripts in
// scripts/ can parse them uniformly.
//
// Under FreeRTOS the row builders run in the radio task. Emitted rows always go
// to the flash sink (rf_log_write); they are echoed to USB (via the console
// log queue) only when live echo is enabled (`live on`). State lives in
// rf_csv.cpp so the console task can toggle the echo flag from another task.
//
// One CSV row per event. Status / boot lines are printed separately as "# ..."
// comments so the parser can skip them.
//
// Columns (24): timestamp_ms, role, freq_mhz, modulation, event, seq,
//   len_bytes, rssi_dbm, snr_db, ferr_hz, good, lost, crc, per_pct, air_ms,
//   gps_lat, gps_lon, gps_alt_m, utc, tx_ms, tx_gps_lat, tx_gps_lon,
//   tx_gps_alt_m, tx_utc
//
// The gps_*/utc columns are stamped from the latest fix supplied via
// rf_csv_set_gps(); the tx_* columns from the RF payload (rf_payload.h).
#pragma once

#include "rf_gps.h"
#include "rf_payload.h"

// Output sink for the durable copy (rf_log_write mirrors rows into flash).
typedef void ( *rf_csv_sink_t )( const char* line );
void rf_csv_set_sink( rf_csv_sink_t sink );

// Toggle echoing live CSV rows to USB (default off — flash always logs). Safe
// to call from the console task; the flag is shared global state.
void rf_csv_set_stdout_enabled( bool enabled );

// Latest GPS fix to stamp onto subsequent rows.
void rf_csv_set_gps( const RfGps& fix );

// Emit the CSV header row.
void rf_csv_header( void );

// Emit one CSV row. Blank a field by passing a negative integer (seq/len/good/
// lost/crc/air) or NAN (rssi/snr/ferr/per).
void rf_csv_row( unsigned long ts_ms,
                 const char* role, float freq_mhz, const char* mod,
                 const char* event,
                 long seq, long len,
                 float rssi, float snr, float ferr,
                 long good, long lost, long crc, float per,
                 long air );

// As rf_csv_row, but also fills the tx_* columns from a decoded RF payload.
void rf_csv_row_tx( unsigned long ts_ms,
                    const char* role, float freq_mhz, const char* mod,
                    const char* event,
                    long seq, long len,
                    float rssi, float snr, float ferr,
                    long good, long lost, long crc, float per,
                    long air,
                    const RfTxTelemetry* tx );
