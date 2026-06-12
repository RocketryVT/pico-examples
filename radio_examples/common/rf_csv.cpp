// rf_csv.cpp — see rf_csv.h.

#include "rf_csv.h"
#include "rf_console.h"   // log_puts() — route rows to the console task

#include <cstdio>
#include <cmath>
#include <cstring>

namespace {

rf_csv_sink_t s_sink           = nullptr;  // durable copy (flash)
bool          s_stdout_enabled = false;    // live USB echo (off by default)
RfGps         s_gps            = {};       // latest fix stamped onto rows

// Emit one fully-formed line (newline included): always to the flash sink,
// and to USB (the console log queue) only when live echo is enabled.
void emit( const char* line )
{
    if ( s_stdout_enabled ) {
        log_puts( line );
    }
    if ( s_sink ) {
        s_sink( line );
    }
}

}  // namespace

void rf_csv_set_sink( rf_csv_sink_t sink )      { s_sink = sink; }
void rf_csv_set_stdout_enabled( bool enabled )  { s_stdout_enabled = enabled; }
void rf_csv_set_gps( const RfGps& fix )         { s_gps = fix; }

void rf_csv_header( void )
{
    emit( "timestamp_ms,role,freq_mhz,modulation,event,seq,len_bytes,"
          "rssi_dbm,snr_db,ferr_hz,good,lost,crc,per_pct,air_ms,"
          "gps_lat,gps_lon,gps_alt_m,utc,"
          "tx_ms,tx_gps_lat,tx_gps_lon,tx_gps_alt_m,tx_utc\n" );
}

void rf_csv_row_tx( unsigned long ts_ms,
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

    // Local GPS columns from the latest fix snapshot — blank when unavailable.
    char b_lat[20], b_lon[20], b_alt[16];
    if ( s_gps.has_pos ) {
        snprintf( b_lat, sizeof b_lat, "%.7f", s_gps.lat );
        snprintf( b_lon, sizeof b_lon, "%.7f", s_gps.lon );
        snprintf( b_alt, sizeof b_alt, "%.1f", (double)s_gps.alt_m );
    } else {
        b_lat[0] = b_lon[0] = b_alt[0] = '\0';
    }
    const char* utc = s_gps.has_time ? s_gps.utc : "";

    // Transmitter telemetry columns from the decoded RF payload.
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

    char line[384];
    snprintf( line, sizeof line,
              "%lu,%s,%.1f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
              ts_ms, role, (double)freq_mhz, mod, event,
              b_seq, b_len, b_rssi, b_snr, b_ferr,
              b_good, b_lost, b_crc, b_per, b_air,
              b_lat, b_lon, b_alt, utc,
              b_tx_ms, b_tx_lat, b_tx_lon, b_tx_alt, tx_utc );
    emit( line );
}

void rf_csv_row( unsigned long ts_ms,
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
