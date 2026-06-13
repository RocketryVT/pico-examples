// main.cpp — GPS-only UART parser test.
//
// LoRa/radio code is intentionally disabled in this example while testing the
// GPS driver. The loop prints every complete NMEA sentence and UBX frame seen
// on UART0, then feeds the same chunks into the GPS driver.

#include "gps/ubx_commands.hpp"
#include "pico/stdlib.h"
#include "pico/version.h"
#include "hardware/uart.h"

#include "gps/gps_driver.hpp"
#include "boards/board.hpp"   // HAS_* flags + Pins:: + Board:: device profile

#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <new>

static_assert(HAS_GPS, "lora915_tx_test requires a GPS (set APP_HAS_GPS in board_profile.hpp)");
static_assert(Board::GpsCount > 0, "lora915_tx_test requires at least one Board::Gpses entry");
static_assert(Board::Gpses[0].bus == Board::Bus::UART0,
              "lora915_tx_test currently supports GPS on UART0 only");
static_assert(Board::Gpses[0].nav_hz > 0, "GPS nav_hz must be non-zero");
static_assert(Board::Gpses[0].nav_hz <= Board::spec_of(Board::Gpses[0].model).max_nav_hz,
              "GPS nav_hz exceeds the selected receiver's device spec");

static constexpr Board::GpsInstance GPS = Board::Gpses[0];

// GPS wiring comes from the active board pin map.
static constexpr uint8_t  GPS_TX_PIN = Pins::GPS_TX;
static constexpr uint8_t  GPS_RX_PIN = Pins::GPS_RX;
static constexpr uint32_t GPS_BAUD   = GPS.baud;  // target baud after autobaud
static constexpr uint16_t GPS_RATE_MS = static_cast<uint16_t>(1000u / GPS.nav_hz);

using Driver = gps::PicoGpsDriver;

alignas(Driver) static uint8_t s_driver_buf[sizeof(Driver)];

class PacketPrinter {
public:
    void feed(const uint8_t* data, std::size_t len)
    {
        for (std::size_t i = 0; i < len; ++i)
            feed_byte(data[i]);
    }

private:
    enum class State : uint8_t {
        Idle,
        Nmea,
        Ubx,
    };

    static constexpr std::size_t NMEA_BUF = 128;
    static constexpr std::size_t UBX_BUF = 1024;

    State state_ = State::Idle;
    uint8_t nmea_[NMEA_BUF]{};
    std::size_t nmea_len_ = 0;
    uint8_t ubx_[UBX_BUF]{};
    std::size_t ubx_len_ = 0;
    std::size_t ubx_expected_ = 0;

    void feed_byte(uint8_t b)
    {
        switch (state_) {
        case State::Idle:
            if (b == '$') start_nmea(b);
            else if (b == 0xB5u) start_ubx(b);
            break;
        case State::Nmea:
            append_nmea(b);
            break;
        case State::Ubx:
            append_ubx(b);
            break;
        }
    }

    void reset()
    {
        state_ = State::Idle;
        nmea_len_ = 0;
        ubx_len_ = 0;
        ubx_expected_ = 0;
    }

    void start_nmea(uint8_t b)
    {
        state_ = State::Nmea;
        nmea_len_ = 0;
        nmea_[nmea_len_++] = b;
    }

    void append_nmea(uint8_t b)
    {
        if (b == '$') {
            start_nmea(b);
            return;
        }
        if (nmea_len_ >= sizeof(nmea_)) {
            reset();
            return;
        }
        nmea_[nmea_len_++] = b;
        if (b == '\n' || b == '\r') {
            print_nmea();
            reset();
        }
    }

    void start_ubx(uint8_t b)
    {
        state_ = State::Ubx;
        ubx_len_ = 0;
        ubx_expected_ = 0;
        ubx_[ubx_len_++] = b;
    }

    void append_ubx(uint8_t b)
    {
        if (ubx_len_ >= sizeof(ubx_)) {
            reset();
            return;
        }

        ubx_[ubx_len_++] = b;

        if (ubx_len_ == 2u) {
            if (b == 0x62u) return;
            if (b == 0xB5u) {
                start_ubx(b);
                return;
            }
            reset();
            return;
        }

        if (ubx_len_ == 6u) {
            const uint16_t payload_len =
                static_cast<uint16_t>(ubx_[4]) |
                (static_cast<uint16_t>(ubx_[5]) << 8);
            ubx_expected_ = static_cast<std::size_t>(payload_len) + 8u;
            if (ubx_expected_ > sizeof(ubx_)) {
                reset();
                return;
            }
        }

        if (ubx_expected_ != 0u && ubx_len_ >= ubx_expected_) {
            print_ubx();
            reset();
        }
    }

    void print_nmea() const
    {
        printf("[nmea] ");
        for (std::size_t i = 0; i < nmea_len_; ++i) {
            const char c = static_cast<char>(nmea_[i]);
            if (c != '\r' && c != '\n')
                putchar(c);
        }
        printf("\n");
    }

    bool ubx_checksum_ok() const
    {
        if (ubx_len_ < 8u) return false;
        uint8_t ck_a = 0;
        uint8_t ck_b = 0;
        for (std::size_t i = 2; i + 2 < ubx_len_; ++i) {
            ck_a = static_cast<uint8_t>(ck_a + ubx_[i]);
            ck_b = static_cast<uint8_t>(ck_b + ck_a);
        }
        return ck_a == ubx_[ubx_len_ - 2] && ck_b == ubx_[ubx_len_ - 1];
    }

    void print_ubx() const
    {
        const uint16_t payload_len =
            static_cast<uint16_t>(ubx_[4]) |
            (static_cast<uint16_t>(ubx_[5]) << 8);

        printf("[ubx] cls=0x%02X id=0x%02X payload=%u ck=%s hex:",
               ubx_[2], ubx_[3], payload_len,
               ubx_checksum_ok() ? "ok" : "bad");

        const std::size_t cap = ubx_len_ < 48u ? ubx_len_ : 48u;
        for (std::size_t i = 0; i < cap; ++i)
            printf(" %02X", ubx_[i]);
        if (cap < ubx_len_)
            printf(" ...");
        printf("\n");
    }
};

static const char* ned_source_name(gps::NedVelocitySource source)
{
    switch (source) {
    case gps::NedVelocitySource::UbxNavPvt: return "ubx-nav-pvt";
    case gps::NedVelocitySource::NmeaPositionDelta: return "nmea-delta";
    default: return "none";
    }
}

static void send_runtime_test_config(Driver& gps_driver)
{
    auto send = [&](const gps::UbxFrame& frame) {
        gps_driver.send_ubx(frame);
        sleep_ms(50);
    };

    // RAM-only config: enable both NMEA and UBX output for parser testing.
    send(gps::Ubx::valset_uart1_inprot_ubx(true, gps::ValLayer::RAM));
    send(gps::Ubx::valset_uart1_inprot_nmea(true, gps::ValLayer::RAM));
    send(gps::Ubx::valset_uart1_outprot_ubx(true, gps::ValLayer::RAM));
    send(gps::Ubx::valset_uart1_outprot_nmea(false, gps::ValLayer::RAM));
    send(gps::Ubx::valset_nav_pvt_uart1(1, gps::ValLayer::RAM));
    send(gps::Ubx::valset_nav_dop_uart1(1, gps::ValLayer::RAM));
    send(gps::Ubx::valset_rate_meas(GPS_RATE_MS, gps::ValLayer::RAM));
    send(gps::Ubx::valset_dyn_model(gps::Ubx::DynModel::Airborne4g, gps::ValLayer::RAM));
}

int main()
{
    stdio_init_all();

    for (int i = 0; i < 30 && !stdio_usb_connected(); ++i)
        sleep_ms(100);
    sleep_ms(300);

    printf("\n# gps-only parser test; LoRa radio code disabled\n");
    printf("# GPS %s (%s) UART0 TX=GPIO%u RX=GPIO%u target=%lu nav=%u Hz DMA=4K\n",
           Board::spec_of(GPS.model).name,
           GPS.role,
           GPS_TX_PIN, GPS_RX_PIN,
           static_cast<unsigned long>(GPS_BAUD),
           static_cast<unsigned>(GPS.nav_hz));

    Driver* gps_driver = new (s_driver_buf) Driver(gps::PicoGpsConfig{
        .uart = uart0,
        .tx_pin = GPS_TX_PIN,
        .rx_pin = GPS_RX_PIN,
        .desired_baud = GPS_BAUD,
        .rx_mode = gps::UartRxMode::DmaRing,
        .dma_ring_size = gps::DmaRingBufferSize::Bytes4K,
    });

    printf("# autobaud initialized=%s detected=%lu current=%lu baud_change=%s dma_ch=%d ring=%luB\n",
           gps_driver->initialized() ? "yes" : "no",
           static_cast<unsigned long>(gps_driver->detected_baud()),
           static_cast<unsigned long>(gps_driver->current_baud()),
           gps_driver->baud_change_ok() ? "ok" : "failed",
           gps_driver->dma_channel(),
           static_cast<unsigned long>(gps_driver->dma_ring_size_bytes()));

    send_runtime_test_config(*gps_driver);
    printf("# requested RAM-only GPS output config: NMEA on, UBX NAV-PVT/NAV-DOP on, %u Hz\n",
           static_cast<unsigned>(GPS.nav_hz));

    PacketPrinter printer;
    gps::Diagnostics last_diag = gps_driver->diagnostics();
    uint32_t last_fix_print_ms = 0;

    for (;;) {
        uint8_t rx[512];
        const std::size_t n = gps_driver->read_raw(rx, sizeof(rx));
        if (n == 0u) {
            sleep_ms(5);
            continue;
        }

        printer.feed(rx, n);
        gps_driver->feed(rx, n);

        const gps::Diagnostics& d = gps_driver->diagnostics();
        if (d.nmea_good != last_diag.nmea_good ||
            d.nmea_bad_cksum != last_diag.nmea_bad_cksum ||
            d.ubx_frames != last_diag.ubx_frames ||
            d.ubx_pvt != last_diag.ubx_pvt ||
            d.ubx_dop != last_diag.ubx_dop ||
            d.ubx_ack != last_diag.ubx_ack ||
            d.ubx_nak != last_diag.ubx_nak) {
            printf("[parsed] nmea=%lu bad_nmea=%lu ubx=%lu pvt=%lu dop=%lu ack=%lu nak=%lu\n",
                   static_cast<unsigned long>(d.nmea_good),
                   static_cast<unsigned long>(d.nmea_bad_cksum),
                   static_cast<unsigned long>(d.ubx_frames),
                   static_cast<unsigned long>(d.ubx_pvt),
                   static_cast<unsigned long>(d.ubx_dop),
                   static_cast<unsigned long>(d.ubx_ack),
                   static_cast<unsigned long>(d.ubx_nak));
            last_diag = d;
        }

        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_fix_print_ms >= 1000u) {
            const gps::Coordinate& c = gps_driver->coordinate();
            printf("[fix] valid=%d lat=%.7f lon=%.7f alt=%.1f sats=%d speed=%.2f course=%.1f "
                   "vel_ned=(%ld,%ld,%ld) source=%s\n",
                   c.valid ? 1 : 0,
                   c.latitude,
                   c.longitude,
                   static_cast<double>(c.altitude),
                   c.satellites,
                   static_cast<double>(c.speed_mps),
                   static_cast<double>(c.course_deg),
                   static_cast<long>(c.vel_north_mms),
                   static_cast<long>(c.vel_east_mms),
                   static_cast<long>(c.vel_down_mms),
                   ned_source_name(c.ned_velocity_source));
            last_fix_print_ms = now_ms;
        }
    }
}
