// rf_csv.h — common CSV output schema for the RF bench-test tools.
//
// Shared by lora915_rx_test / lora915_tx_test / rfm69_433_rx_test /
// rfm69_433_tx_test so every log is the same shape and the analysis scripts in
// scripts/ can parse them uniformly.
//
// One CSV row per event. Status / boot lines are printed separately as "# ..."
// comments so the parser can skip them.
//
// Columns (24):
//   timestamp_ms  - Pico boot time in ms (to_ms_since_boot)
//   role          - "rx" or "tx"
//   freq_mhz      - carrier frequency (e.g. 915.0, 433.0)
//   modulation    - "lora" or "gfsk"
//   event         - packet | crc_error | read_error | noise_floor | tx_ok | tx_fail
//   seq           - "#N" sequence number from the payload (blank if N/A)
//   len_bytes     - payload length
//   rssi_dbm      - received signal strength (blank if N/A)
//   snr_db        - LoRa SNR (blank for FSK)
//   ferr_hz       - LoRa frequency error (blank for FSK)
//   good          - cumulative good-packet count (rx)
//   lost          - cumulative lost-packet count from seq gaps (rx)
//   crc           - cumulative CRC-error count (rx)
//   per_pct       - cumulative packet-error rate % (rx)
//   air_ms        - on-air time for a transmit (tx)
//   gps_lat       - latitude from the UART0 GPS, deg (blank until first fix)
//   gps_lon       - longitude from the UART0 GPS, deg (blank until first fix)
//   gps_alt_m     - MSL altitude from the UART0 GPS, m (blank until first fix)
//   utc           - UTC timestamp "YYYY-MM-DDTHH:MM:SSZ" from the GPS (blank
//                   until the time is resolved)
//   tx_ms         - transmitter boot timestamp embedded in the RF payload
//   tx_gps_lat    - transmitter latitude embedded in the RF payload
//   tx_gps_lon    - transmitter longitude embedded in the RF payload
//   tx_gps_alt_m  - transmitter altitude embedded in the RF payload
//   tx_utc        - transmitter GPS UTC embedded in the RF payload
//
// The gps_* / utc columns are stamped from the latest fix supplied via
// rf_csv_set_gps() (see rf_gps.h / gps_task.h). They stay blank on tools/boots
// with no GPS attached.
#pragma once

#include "rf_gps.h"
#include "rf_payload.h"

#include <cstdio>
#include <cmath>
#include <cstring>

// Optional output sink. Every emitted line is always written to stdout (USB
// serial); if a sink is registered it also receives the same line. rf_log.h
// provides rf_log_write() for mirroring the stream into a flash file.
typedef void ( *rf_csv_sink_t )( const char* line );

static rf_csv_sink_t s_rf_csv_sink = nullptr;
static bool s_rf_csv_stdout_enabled = true;

static inline void rf_csv_set_sink( rf_csv_sink_t sink )
{
    s_rf_csv_sink = sink;
}

static inline void rf_csv_set_stdout_enabled( bool enabled )
{
    s_rf_csv_stdout_enabled = enabled;
}

// Latest GPS fix to stamp onto subsequent rows. The main loop refreshes this
// from the GPS task once per iteration; rows emitted before any fix carry blank
// gps_*/utc columns.
static RfGps s_rf_gps = {};

static inline void rf_csv_set_gps( const RfGps& fix )
{
    s_rf_gps = fix;
}

// Emit one fully-formed line (newline included) to stdout and the sink.
static inline void rf_csv_emit( const char* line )
{
    if ( s_rf_csv_stdout_enabled ) {
        fputs( line, stdout );
    }
    if ( s_rf_csv_sink ) {
        s_rf_csv_sink( line );
    }
}

static inline void rf_csv_header( void )
{
    rf_csv_emit( "timestamp_ms,role,freq_mhz,modulation,event,seq,len_bytes,"
                 "rssi_dbm,snr_db,ferr_hz,good,lost,crc,per_pct,air_ms,"
                 "gps_lat,gps_lon,gps_alt_m,utc,"
                 "tx_ms,tx_gps_lat,tx_gps_lon,tx_gps_alt_m,tx_utc\n" );
}

// Emit one CSV row. Blank a field by passing a negative integer (seq/len/good/
// lost/crc/air) or NAN (rssi/snr/ferr/per). The gps_*/utc columns are filled
// from the snapshot set via rf_csv_set_gps() (blank until the first fix).
static inline void rf_csv_row( unsigned long ts_ms,
                               const char* role, float freq_mhz, const char* mod,
                               const char* event,
                               long seq, long len,
                               float rssi, float snr, float ferr,
                               long good, long lost, long crc, float per,
                               long air )
;

static inline void rf_csv_row_tx( unsigned long ts_ms,
                                  const char* role, float freq_mhz, const char* mod,
                                  const char* event,
                                  long seq, long len,
                                  float rssi, float snr, float ferr,
                                  long good, long lost, long crc, float per,
                                  long air,
                                  const RfTxTelemetry* tx )
{
    char b_seq[12], b_len[12], b_rssi[16], b_snr[16], b_ferr[16];
    char b_good[12], b_lost[12], b_crc[12], b_per[12], b_air[12];

    if ( seq  < 0 ) b_seq[0]  = '\0'; else snprintf( b_seq,  sizeof b_seq,  "%ld", seq  );
    if ( len  < 0 ) b_len[0]  = '\0'; else snprintf( b_len,  sizeof b_len,  "%ld", len  );
    if ( good < 0 ) b_good[0] = '\0'; else snprintf( b_good, sizeof b_good, "%ld", good );
    if ( lost < 0 ) b_lost[0] = '\0'; else snprintf( b_lost, sizeof b_lost, "%ld", lost );
    if ( crc  < 0 ) b_crc[0]  = '\0'; else snprintf( b_crc,  sizeof b_crc,  "%ld", crc  );
    if ( air  < 0 ) b_air[0]  = '\0'; else snprintf( b_air,  sizeof b_air,  "%ld", air  );

    if ( std::isnan( rssi ) ) b_rssi[0] = '\0'; else snprintf( b_rssi, sizeof b_rssi, "%.1f", (double)rssi );
    if ( std::isnan( snr )  ) b_snr[0]  = '\0'; else snprintf( b_snr,  sizeof b_snr,  "%.1f", (double)snr );
    if ( std::isnan( ferr ) ) b_ferr[0] = '\0'; else snprintf( b_ferr, sizeof b_ferr, "%.0f", (double)ferr );
    if ( std::isnan( per )  ) b_per[0]  = '\0'; else snprintf( b_per,  sizeof b_per,  "%.1f", (double)per );

    // GPS columns from the latest fix snapshot — blank when unavailable.
    char b_lat[20], b_lon[20], b_alt[16];
    if ( s_rf_gps.has_pos ) {
        snprintf( b_lat, sizeof b_lat, "%.7f", s_rf_gps.lat );
        snprintf( b_lon, sizeof b_lon, "%.7f", s_rf_gps.lon );
        snprintf( b_alt, sizeof b_alt, "%.1f", (double)s_rf_gps.alt_m );
    } else {
        b_lat[0] = b_lon[0] = b_alt[0] = '\0';
    }
    const char* utc = s_rf_gps.has_time ? s_rf_gps.utc : "";

    char b_tx_ms[12], b_tx_lat[20], b_tx_lon[20], b_tx_alt[16];
    const char* tx_utc = "";
    if ( tx && tx->valid ) {
        if ( tx->has_ms ) snprintf( b_tx_ms, sizeof b_tx_ms, "%lu", (unsigned long)tx->tx_ms );
        else b_tx_ms[0] = '\0';

        if ( tx->has_pos ) {
            snprintf( b_tx_lat, sizeof b_tx_lat, "%.7f", tx->lat );
            snprintf( b_tx_lon, sizeof b_tx_lon, "%.7f", tx->lon );
            snprintf( b_tx_alt, sizeof b_tx_alt, "%.1f", (double)tx->alt_m );
        } else {
            b_tx_lat[0] = b_tx_lon[0] = b_tx_alt[0] = '\0';
        }

        tx_utc = tx->has_time ? tx->utc : "";
    } else {
        b_tx_ms[0] = b_tx_lat[0] = b_tx_lon[0] = b_tx_alt[0] = '\0';
    }

    // 15 value columns + local GPS/time + transmitter GPS/time.
    char line[384];
    snprintf( line, sizeof line,
              "%lu,%s,%.1f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
              ts_ms, role, (double)freq_mhz, mod, event,
              b_seq, b_len, b_rssi, b_snr, b_ferr,
              b_good, b_lost, b_crc, b_per, b_air,
              b_lat, b_lon, b_alt, utc,
              b_tx_ms, b_tx_lat, b_tx_lon, b_tx_alt, tx_utc );
    rf_csv_emit( line );
}

static inline void rf_csv_row( unsigned long ts_ms,
                               const char* role, float freq_mhz, const char* mod,
                               const char* event,
                               long seq, long len,
                               float rssi, float snr, float ferr,
                               long good, long lost, long crc, float per,
                               long air )
{
    rf_csv_row_tx( ts_ms, role, freq_mhz, mod, event, seq, len,
                   rssi, snr, ferr, good, lost, crc, per, air, nullptr );
}
