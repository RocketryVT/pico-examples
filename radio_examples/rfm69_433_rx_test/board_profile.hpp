#pragma once

// board_profile.hpp — per-project peripheral profile for rfm69_433_rx_test.
//
// The board (gs_pcb_v1) is a carrier: it only routes connectors to GPIOs. THIS
// file declares what this firmware actually plugs in (as Board::*Instance arrays
// — model + bus + wiring + rate) and any link/modulation settings. boards/
// board.hpp pulls this in (via __has_include), merges APP_HAS_* with the board's
// BOARD_HAS_* into HAS_*, and exposes Board:: + devices.hpp's spec_of().
//
// NOTE: this file is only ever included *through* boards/board.hpp, which has
// already included devices.hpp (for the model enums / *Instance types) and
// board_pins.hpp (for Pins::). It is not self-contained — editors parsing it in
// isolation will flag those names as undefined; that is expected.
//
// Coarse category gates (compile whole subsystems in/out):
#define APP_HAS_RADIO 1   // a packet radio is present
#define APP_HAS_GPS   1   // a GPS is present

namespace Board {

// Devices plugged into the carrier's connectors. devices.hpp + Pins:: come from
// board.hpp (included before this file), so we can name models and wire to Pins.
inline constexpr RadioInstance Radios[] = {
    { RadioModel::RFM69HCW, Bus::SPI1, Pins::LORA1_NSS, 433.0f, "433-gfsk" },
};
inline constexpr int RadioCount = static_cast<int>(std::size(Radios));

inline constexpr GpsInstance Gpses[] = {
    { GpsModel::Generic, Bus::UART0, /*baud*/115200, /*nav_hz*/10, "primary" },
};
inline constexpr int GpsCount = static_cast<int>(std::size(Gpses));

// RFM69 GFSK link parameters (must match the transmitter). These are link/
// deployment settings, not device-intrinsic, so they live with the instance
// rather than in devices.hpp's spec_of().
namespace Rfm433 {
    inline constexpr float    BR_KBPS  = 4.8f;    // bit rate
    inline constexpr float    FDEV_KHZ = 5.0f;    // frequency deviation
    inline constexpr float    RXBW_KHZ = 125.0f;  // RX channel filter bandwidth
    inline constexpr int8_t   RX_DBM   = 13;      // RX-only; PA power irrelevant
    inline constexpr uint16_t PREAMBLE = 16;      // preamble length in bits
}

} // namespace Board
