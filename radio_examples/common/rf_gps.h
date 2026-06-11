// rf_gps.h — the small GPS snapshot the CSV layer stamps onto each row.
//
// Kept in its own header (no statics, no Pico/GPS deps) so both rf_csv.h (which
// formats the columns) and gps_task.h (which produces the snapshot) can share
// the type without pulling each other in.
#pragma once

struct RfGps {
    bool   valid    = false;   // a usable fix is available
    bool   has_time = false;   // utc[] holds a resolved UTC timestamp
    bool   has_pos  = false;   // lat/lon/alt_m are populated
    char   utc[24]  = { 0 };   // "YYYY-MM-DDTHH:MM:SSZ" (empty if !has_time)
    double lat      = 0.0;     // degrees WGS-84
    double lon      = 0.0;
    float  alt_m    = 0.0f;    // metres MSL
};
