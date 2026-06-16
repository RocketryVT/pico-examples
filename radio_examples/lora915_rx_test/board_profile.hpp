#pragma once

// board_profile.hpp — per-project peripheral profile for lora915_rx_test.
//
// The gs_pcb_v1 carrier routes connectors to GPIOs; this profile declares the
// SX1276 radio and GPS module used by this firmware plus the 915 MHz LoRa link
// settings. It is included through boards/board.hpp.

#define APP_HAS_RADIO 1
#define APP_HAS_SX1276 1
#define APP_HAS_GPS   1

namespace Board {

inline constexpr RadioInstance Radios[] = {
    { RadioModel::SX1276, Bus::SPI0, Pins::LORA0_NSS, 915.0f, "915-lora" },
};
inline constexpr int RadioCount = static_cast<int>(std::size(Radios));

inline constexpr GpsInstance Gpses[] = {
    { GpsModel::Generic, Bus::UART0, /*baud*/230400, /*nav_hz*/10, "primary" },
};
inline constexpr int GpsCount = static_cast<int>(std::size(Gpses));

namespace Lora915 {
    inline constexpr float   BW_KHZ    = 125.0f;
    inline constexpr uint8_t SF        = 7;
    inline constexpr uint8_t CR        = 5;
    inline constexpr uint8_t SYNC_WORD = 0x12;
    inline constexpr int8_t  TX_DBM    = 20;
    inline constexpr uint16_t PREAMBLE = 8;
}

} // namespace Board
