#pragma once

// board_profile.hpp — per-project peripheral profile for lora915_tx_test.
//
// This target currently runs as a GPS-only parser/autobaud test. The gs_pcb_v1
// carrier supplies connector GPIOs through Pins::; this profile declares the
// actual GPS module connected to that carrier using boards/devices.hpp types.

#define APP_HAS_GPS 1

namespace Board {

inline constexpr GpsInstance Gpses[] = {
    { GpsModel::Generic, Bus::UART0, /*baud*/115200, /*nav_hz*/10, "primary" },
};
inline constexpr int GpsCount = static_cast<int>(std::size(Gpses));

} // namespace Board
