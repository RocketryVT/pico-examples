// rf_csv.h — common CSV output schema for the RF bench-test tools.
//
// Shared by lora915_rx_test / lora915_tx_test / rfm69_433_rx_test /
// rfm69_433_tx_test so every log is the same shape and the analysis scripts in
// scripts/ can parse them uniformly.
//
// One CSV row per event. Status / boot lines are printed separately as "# ..."
// comments so the parser can skip them.
//
// Columns (18):
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
//   gps_lat       - reserved for future GPS logging (blank)
//   gps_lon       - reserved for future GPS logging (blank)
//   gps_alt_m     - reserved for future GPS logging (blank)
#pragma once

#include <cstdio>
#include <cmath>

static inline void rf_csv_header( void )
{
    printf( "timestamp_ms,role,freq_mhz,modulation,event,seq,len_bytes,"
            "rssi_dbm,snr_db,ferr_hz,good,lost,crc,per_pct,air_ms,"
            "gps_lat,gps_lon,gps_alt_m\n" );
}

// Emit one CSV row. Blank a field by passing a negative integer (seq/len/good/
// lost/crc/air) or NAN (rssi/snr/ferr/per). The three gps_* columns are always
// left blank for now.
static inline void rf_csv_row( unsigned long ts_ms,
                               const char* role, float freq_mhz, const char* mod,
                               const char* event,
                               long seq, long len,
                               float rssi, float snr, float ferr,
                               long good, long lost, long crc, float per,
                               long air )
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

    // 15 value columns + 3 blank gps columns (trailing ",,,").
    printf( "%lu,%s,%.1f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,,,\n",
            ts_ms, role, (double)freq_mhz, mod, event,
            b_seq, b_len, b_rssi, b_snr, b_ferr,
            b_good, b_lost, b_crc, b_per, b_air );
}
