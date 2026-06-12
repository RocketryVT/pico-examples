// rf_payload.h — binary LoRa bench-test payload for TX/RX correlation.
//
// This is intentionally specific to the current 915 MHz LoRa range-test flow:
// RX logs local receive metrics and GPS, while every TX packet carries the
// transmitter's sequence number, TX boot timestamp, GPS position and GPS UTC.
#pragma once

#include "rf_gps.h"

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstring>

static constexpr uint8_t RF_PAYLOAD_FLAG_POS  = 1u << 0;
static constexpr uint8_t RF_PAYLOAD_FLAG_TIME = 1u << 1;

struct RfTxTelemetry {
    bool     valid    = false;
    bool     has_seq  = false;
    bool     has_ms   = false;
    bool     has_pos  = false;
    bool     has_time = false;
    uint32_t seq      = 0;
    uint32_t tx_ms    = 0;
    double   lat      = 0.0;
    double   lon      = 0.0;
    float    alt_m    = 0.0f;
    char     utc[24]  = { 0 };
};

struct [[gnu::packed]] RfPacketV2 {
    char     magic[4];     // "RFT2"
    uint8_t  version;      // 2
    uint8_t  flags;        // RF_PAYLOAD_FLAG_*
    uint16_t len;          // sizeof(RfPacketV2), little-endian on RP2350
    uint32_t seq;
    uint32_t tx_ms;        // transmitter boot ms
    int32_t  lat_e7;       // degrees * 1e7
    int32_t  lon_e7;       // degrees * 1e7
    int32_t  alt_cm;       // metres * 100
    char     utc[20];      // "YYYY-MM-DDTHH:MM:SSZ", no trailing NUL
};

static_assert( sizeof( RfPacketV2 ) == 48 );

static inline size_t rf_payload_build( uint8_t* out, size_t out_len,
                                       uint32_t seq, uint32_t tx_ms,
                                       const RfGps& gps )
{
    if ( !out || out_len < sizeof( RfPacketV2 ) ) {
        return 0;
    }

    RfPacketV2 pkt = {};
    pkt.magic[0] = 'R';
    pkt.magic[1] = 'F';
    pkt.magic[2] = 'T';
    pkt.magic[3] = '2';
    pkt.version = 2;
    pkt.len = sizeof( RfPacketV2 );
    pkt.seq = seq;
    pkt.tx_ms = tx_ms;

    if ( gps.has_pos ) {
        pkt.flags |= RF_PAYLOAD_FLAG_POS;
        pkt.lat_e7 = static_cast<int32_t>( std::llround( gps.lat * 10000000.0 ) );
        pkt.lon_e7 = static_cast<int32_t>( std::llround( gps.lon * 10000000.0 ) );
        pkt.alt_cm = static_cast<int32_t>( std::llround( (double)gps.alt_m * 100.0 ) );
    }

    if ( gps.has_time ) {
        pkt.flags |= RF_PAYLOAD_FLAG_TIME;
        memcpy( pkt.utc, gps.utc, 20 );
    }

    memcpy( out, &pkt, sizeof pkt );
    return sizeof pkt;
}

static inline bool rf_payload_parse( const uint8_t* data, size_t len,
                                     RfTxTelemetry* out )
{
    if ( !data || !out || len < sizeof( RfPacketV2 ) ) {
        return false;
    }

    RfPacketV2 pkt;
    memcpy( &pkt, data, sizeof pkt );

    if ( memcmp( pkt.magic, "RFT2", 4 ) != 0 ||
         pkt.version != 2 ||
         pkt.len != sizeof( RfPacketV2 ) ) {
        return false;
    }

    RfTxTelemetry parsed;
    parsed.valid = true;
    parsed.has_seq = true;
    parsed.has_ms = true;
    parsed.seq = pkt.seq;
    parsed.tx_ms = pkt.tx_ms;

    if ( pkt.flags & RF_PAYLOAD_FLAG_POS ) {
        parsed.has_pos = true;
        parsed.lat = (double)pkt.lat_e7 / 10000000.0;
        parsed.lon = (double)pkt.lon_e7 / 10000000.0;
        parsed.alt_m = (float)pkt.alt_cm / 100.0f;
    }

    if ( pkt.flags & RF_PAYLOAD_FLAG_TIME ) {
        parsed.has_time = true;
        memcpy( parsed.utc, pkt.utc, 20 );
        parsed.utc[20] = '\0';
    }

    *out = parsed;
    return true;
}
