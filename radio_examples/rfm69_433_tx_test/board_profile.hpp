#pragma once

// board_profile.hpp — per-project peripheral profile for rfm69_433_tx_test.
//
// The gs_pcb_v1 carrier routes connectors to GPIOs; this file declares the
// actual radio and GPS module used by this firmware plus the 433 MHz GFSK link
// settings. It is included through boards/board.hpp.

#define APP_HAS_RADIO 1
#define APP_HAS_GPS   1

namespace Board {

inline constexpr RadioInstance Radios[] = {
    { RadioModel::RFM69HCW, Bus::SPI1, Pins::LORA1_NSS, 424.500f, "433-gfsk" },
};
inline constexpr int RadioCount = static_cast<int>(std::size(Radios));

inline constexpr GpsInstance Gpses[] = {
    { GpsModel::Generic, Bus::UART0, /*baud*/115200, /*nav_hz*/10, "primary" },
};
inline constexpr int GpsCount = static_cast<int>(std::size(Gpses));

namespace Rfm433 {
    inline constexpr float    BR_KBPS    = 4.8f;
    inline constexpr float    FDEV_KHZ   = 5.0f;
    inline constexpr float    RXBW_KHZ   = 125.0f;
    inline constexpr int8_t   TX_DBM     = 20;
    inline constexpr uint16_t PREAMBLE   = 16;
    inline constexpr bool     HIGH_POWER = true;
}

} // namespace Board
