// gps_task.cpp — see gps_task.h.

#include "gps_task.h"
#include "rf_console.h"   // log_print() — all USB output goes via the console task
#include "rtos.h"

// Pico headers first: gps_driver.hpp only compiles its UART/I2C transport
// wrappers when PICO_SDK_VERSION_MAJOR is already defined (pico/version.h,
// pulled in by pico/stdlib.h).
#include "pico/stdlib.h"
#include "pico/version.h"
#include "hardware/irq.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "gps/gps_driver.hpp"

#include <cstdio>
#include <optional>

namespace {

// Guards the parser/coordinate state shared between the GPS task (feed/parse)
// and any task reading a fix snapshot (gps_task_fix). Created in gps_task_start.
SemaphoreHandle_t g_gps_mutex = nullptr;
StaticSemaphore_t g_gps_mutex_buf;

bool gps_sched_running()
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

void gps_lock()
{
    if ( g_gps_mutex && gps_sched_running() ) {
        xSemaphoreTake( g_gps_mutex, portMAX_DELAY );
    }
}

void gps_unlock()
{
    if ( g_gps_mutex && gps_sched_running() ) {
        xSemaphoreGive( g_gps_mutex );
    }
}

// GPS task storage.
StaticTask_t g_gps_tcb;
StackType_t  g_gps_stack[2048];

// -- UART0 link config ---------------------------------------------------------
constexpr uint16_t GPS_RATE_MS = 1000; // 1 Hz navigation solution
constexpr uint32_t GPS_DIAG_MS = 5000; // low-rate "why are fields blank?" hint
constexpr uint32_t GPS_STALE_MS = 3000; // blank cached GPS fields after this

struct GpsTaskConfig {
    uint8_t  tx_pin = 0;      // Pico UART0 TX -> GPS RX
    uint8_t  rx_pin = 1;      // Pico UART0 RX <- GPS TX
    uint32_t baud   = 115200;
};

// The driver owns the transport by reference, so both must outlive every poll.
// Constructed in gps_task_init() (not at static-init time) because UartTransport
// touches hardware that isn't ready until clocks/stdio are up.
std::optional<gps::UartTransport>                 g_uart;
std::optional<gps::GpsDriver<gps::UartTransport>> g_driver;
GpsTaskConfig g_config;
bool g_ready = false;
bool g_nav_pvt_debug = false;
bool g_configure_output = true;
bool g_rx_irq_installed = false;
bool g_rx_irq_enabled = false;
uint32_t g_tap_ubx_frames = 0;
uint32_t g_tap_nav_pvt_frames = 0;
uint32_t g_tap_other_prints = 0;
uint32_t g_raw_chunk_count = 0;
uint32_t g_rx_ring_overflows = 0;
uint32_t g_last_reported_overflows = 0;
uint32_t g_last_frame_ms = 0;
uint32_t g_last_time_update_ms = 0;
uint32_t g_last_frame_total = 0;
uint16_t g_last_utc_year = 0;
uint8_t  g_last_utc_month = 0;
uint8_t  g_last_utc_day = 0;
uint32_t g_last_utc_ms = 0;

// UART0's hardware FIFO is tiny compared with a 230400-baud mixed NMEA+UBX
// burst. Capture bytes in an IRQ ring first, then parse them cooperatively from
// gps_task_poll() after radio/logging/USB work yields.
constexpr uint16_t RX_RING_SIZE = 8192;
constexpr uint16_t RX_RING_MASK = RX_RING_SIZE - 1;
static_assert( ( RX_RING_SIZE & RX_RING_MASK ) == 0, "RX ring must be power-of-two" );

uint8_t           g_rx_ring[RX_RING_SIZE];
volatile uint16_t g_rx_head = 0;
volatile uint16_t g_rx_tail = 0;
volatile uint32_t g_rx_overflow_isr = 0;

void reset_rx_ring()
{
    g_rx_head = 0;
    g_rx_tail = 0;
    g_rx_overflow_isr = 0;
    g_rx_ring_overflows = 0;
    g_last_reported_overflows = 0;
}

void __not_in_flash_func(gps_uart0_irq_handler)()
{
    while ( uart_is_readable( uart0 ) ) {
        const uint8_t b = static_cast<uint8_t>( uart_getc( uart0 ) );
        const uint16_t head = g_rx_head;
        const uint16_t next = static_cast<uint16_t>( ( head + 1u ) & RX_RING_MASK );
        if ( next == g_rx_tail ) {
            g_rx_overflow_isr = g_rx_overflow_isr + 1u;
        } else {
            g_rx_ring[head] = b;
            g_rx_head = next;
        }
    }
}

void disable_uart_rx_irq()
{
    if ( g_rx_irq_enabled ) {
        uart_set_irq_enables( uart0, false, false );
        irq_set_enabled( UART0_IRQ, false );
        g_rx_irq_enabled = false;
    }
}

void enable_uart_rx_irq()
{
    reset_rx_ring();

    if ( !g_rx_irq_installed ) {
        irq_set_exclusive_handler( UART0_IRQ, gps_uart0_irq_handler );
        g_rx_irq_installed = true;
    }

    while ( uart_is_readable( uart0 ) ) {
        (void)uart_getc( uart0 );
    }
    uart_set_irq_enables( uart0, true, false );
    irq_set_enabled( UART0_IRQ, true );
    g_rx_irq_enabled = true;
}

size_t read_rx_ring( uint8_t* buf, size_t len )
{
    size_t n = 0;
    while ( n < len && g_rx_tail != g_rx_head ) {
        const uint16_t tail = g_rx_tail;
        buf[n++] = g_rx_ring[tail];
        g_rx_tail = static_cast<uint16_t>( ( tail + 1u ) & RX_RING_MASK );
    }
    g_rx_ring_overflows = g_rx_overflow_isr;
    return n;
}

void open_uart( const GpsTaskConfig& config )
{
    g_ready = false;
    disable_uart_rx_irq();
    g_driver.reset();
    g_uart.reset();
    uart_deinit( uart0 );
    g_tap_ubx_frames = 0;
    g_tap_nav_pvt_frames = 0;
    g_tap_other_prints = 0;
    g_raw_chunk_count = 0;
    reset_rx_ring();
    g_last_frame_ms = 0;
    g_last_time_update_ms = 0;
    g_last_frame_total = 0;
    g_last_utc_year = 0;
    g_last_utc_month = 0;
    g_last_utc_day = 0;
    g_last_utc_ms = 0;

    g_config = config;
    g_uart.emplace( uart0, g_config.tx_pin, g_config.rx_pin, g_config.baud );
    g_driver.emplace( *g_uart );
}

void configure_nav_pvt()
{
    // Auto-configure M10-style receivers the same way as the ground-station GPS
    // task. The older CFG-PRT/CFG-MSG sequence is not reliable on these modules.
    g_driver->send_ubx( gps::Ubx::valset_uart1_inprot_ubx( true ) );
    g_driver->send_ubx( gps::Ubx::valset_uart1_inprot_nmea( true ) );
    g_driver->send_ubx( gps::Ubx::valset_uart1_outprot_ubx( true ) );
    // Keep NMEA enabled as a fallback for GPS modules that emit useful
    // GGA/RMC fixes while their UBX-compatible NAV-PVT fix flags lag or differ.
    g_driver->send_ubx( gps::Ubx::valset_uart1_outprot_nmea( true ) );
    g_driver->send_ubx( gps::Ubx::valset_nav_pvt_uart1( 1 ) );
    g_driver->send_ubx( gps::Ubx::valset_rate_meas( GPS_RATE_MS ) );
    g_driver->send_ubx( gps::Ubx::valset_dyn_model( gps::Ubx::DynModel::Stationary ) );  // bench test
    sleep_ms( 200 );  // let the module apply the config before we poll.
}

void feed_raw( const uint8_t* buf, size_t n )
{
    g_driver->feed( buf, n );
}

bool detect_valid_stream( uint32_t wait_ms )
{
    const absolute_time_t end = make_timeout_time_ms( wait_ms );
    uint8_t buf[gps::GpsDriver<gps::UartTransport>::READ_CHUNK];

    while ( !time_reached( end ) ) {
        size_t n = 0;
        do {
            n = g_driver->read_raw( buf, sizeof buf );
            feed_raw( buf, n );
        } while ( n == sizeof buf );

        const auto& d = g_driver->diagnostics();
        if ( d.nmea_good > 0 || d.ubx_frames > 0 ) {
            return true;
        }

        sleep_ms( 1 );
    }

    return false;
}

bool detect_listen_only_stream( uint32_t wait_ms, bool* saw_nmea )
{
    const absolute_time_t end = make_timeout_time_ms( wait_ms );
    uint8_t buf[gps::GpsDriver<gps::UartTransport>::READ_CHUNK];
    bool saw_ubx = false;
    if ( saw_nmea ) {
        *saw_nmea = false;
    }

    while ( !time_reached( end ) ) {
        size_t n = 0;
        do {
            n = g_driver->read_raw( buf, sizeof buf );
            feed_raw( buf, n );
        } while ( n == sizeof buf );

        const auto& d = g_driver->diagnostics();
        if ( d.nmea_good > 0 ) {
            if ( saw_nmea ) {
                *saw_nmea = true;
            }
            return true;
        }
        if ( d.ubx_frames > 0 ) {
            saw_ubx = true;
        }

        sleep_ms( 1 );
    }

    return saw_ubx;
}

// Append " %02X" hex for up to `cap` bytes into a caller buffer.
size_t append_hex( char* dst, size_t dst_len, size_t pos,
                   const uint8_t* b, size_t count, size_t cap )
{
    for ( size_t i = 0; i < count && i < cap && pos + 4 < dst_len; ++i ) {
        pos += (size_t)snprintf( dst + pos, dst_len - pos, " %02X", b[i] );
    }
    return pos;
}

void print_raw_sample( const uint8_t* sample, size_t count )
{
    char line[384];
    size_t pos = (size_t)snprintf( line, sizeof line, "# [gps] raw hex:" );
    pos = append_hex( line, sizeof line, pos, sample, count, 48 );
    log_print( "%s", line );

    char ascii[80];
    size_t ap = (size_t)snprintf( ascii, sizeof ascii, "# [gps] raw ascii: " );
    for ( size_t i = 0; i < count && i < 48 && ap + 1 < sizeof ascii; ++i ) {
        const uint8_t c = sample[i];
        ascii[ap++] = ( c >= 32 && c <= 126 ) ? (char)c : '.';
    }
    ascii[ap] = '\0';
    log_print( "%s", ascii );
}

void print_raw_uart_chunk( const uint8_t* sample, size_t count )
{
    if ( !g_nav_pvt_debug || count == 0 ) {
        return;
    }

    char line[384];
    size_t pos = (size_t)snprintf( line, sizeof line, "# [gps] raw-uart n=%lu len=%u hex:",
                                   (unsigned long)g_raw_chunk_count++, (unsigned)count );
    pos = append_hex( line, sizeof line, pos, sample, count, 100 );
    log_print( "%s", line );
}

void print_nav_pvt_frame( const uint8_t* frame, size_t count )
{
    const gps::Coordinate& c = g_driver->coordinate();
    char line[384];
    size_t pos = (size_t)snprintf(
        line, sizeof line,
        "# [gps] nav-pvt raw n=%lu len=%u fix=%.*s sats=%d hAcc=%.1fm hex:",
        (unsigned long)g_tap_nav_pvt_frames, (unsigned)count,
        (int)g_driver->fix_label().size(), g_driver->fix_label().data(),
        c.satellites, (double)c.h_acc_mm / 1000.0 );
    pos = append_hex( line, sizeof line, pos, frame, count, 100 );
    log_print( "%s", line );
}

void print_ubx_frame( const uint8_t* frame, size_t count, uint8_t cls, uint8_t id,
                      uint16_t length )
{
    if ( cls == 0x01u && id == 0x07u ) {
        ++g_tap_nav_pvt_frames;
        print_nav_pvt_frame( frame, count );
        return;
    }

    // Keep this bounded. ACK/NAK or unexpected NAV messages are useful clues,
    // but repeated non-PVT UBX output can swamp the USB console.
    if ( g_tap_other_prints >= 12 ) {
        return;
    }
    ++g_tap_other_prints;

    char line[384];
    size_t pos = (size_t)snprintf(
        line, sizeof line,
        "# [gps] ubx raw n=%lu cls=0x%02X id=0x%02X payload_len=%u hex:",
        (unsigned long)g_tap_ubx_frames, (unsigned)cls, (unsigned)id,
        (unsigned)length );
    pos = append_hex( line, sizeof line, pos, frame, count, 100 );
    log_print( "%s", line );
}

void tap_nav_pvt_byte( uint8_t b )
{
    enum class State : uint8_t {
        Idle,
        Sync2,
        Class,
        Id,
        Len1,
        Len2,
        Payload,
        CkA,
        CkB,
    };

    static State state = State::Idle;
    static uint8_t frame[128];
    static uint16_t idx = 0;
    static uint16_t length = 0;
    static uint16_t payload_idx = 0;
    static uint8_t cls = 0;
    static uint8_t id = 0;
    static uint8_t ck_a = 0;
    static uint8_t ck_b = 0;

    auto reset = []() {
        state = State::Idle;
        idx = 0;
        length = 0;
        payload_idx = 0;
        cls = 0;
        id = 0;
        ck_a = 0;
        ck_b = 0;
    };

    auto push = [&]( uint8_t v ) {
        if ( idx < sizeof frame ) {
            frame[idx++] = v;
        } else {
            reset();
        }
    };

    switch ( state ) {
    case State::Idle:
        if ( b == 0xB5u ) {
            reset();
            push( b );
            state = State::Sync2;
        }
        break;

    case State::Sync2:
        if ( b == 0x62u ) {
            push( b );
            state = State::Class;
        } else if ( b == 0xB5u ) {
            reset();
            push( b );
            state = State::Sync2;
        } else {
            reset();
        }
        break;

    case State::Class:
        if ( b == 0xB5u ) {
            reset();
            push( b );
            state = State::Sync2;
            break;
        }
        cls = b;
        ck_a = b;
        ck_b = ck_a;
        push( b );
        state = State::Id;
        break;

    case State::Id:
        id = b;
        ck_a += b;
        ck_b += ck_a;
        push( b );
        state = State::Len1;
        break;

    case State::Len1:
        length = b;
        ck_a += b;
        ck_b += ck_a;
        push( b );
        state = State::Len2;
        break;

    case State::Len2:
        length |= static_cast<uint16_t>( b ) << 8;
        ck_a += b;
        ck_b += ck_a;
        push( b );
        payload_idx = 0;
        if ( length > 120u ) {
            reset();
        } else {
            state = ( length > 0 ) ? State::Payload : State::CkA;
        }
        break;

    case State::Payload:
        ck_a += b;
        ck_b += ck_a;
        push( b );
        if ( ++payload_idx >= length ) {
            state = State::CkA;
        }
        break;

    case State::CkA:
        push( b );
        state = ( b == ck_a ) ? State::CkB : State::Idle;
        break;

    case State::CkB:
        push( b );
        if ( b == ck_b ) {
            ++g_tap_ubx_frames;
            print_ubx_frame( frame, idx, cls, id, length );
        }
        reset();
        break;
    }
}

void tap_nav_pvt( const uint8_t* buf, size_t n )
{
    if ( !g_nav_pvt_debug ) {
        return;
    }
    for ( size_t i = 0; i < n; ++i ) {
        tap_nav_pvt_byte( buf[i] );
    }
}

// Build the CSV snapshot from the driver's current Coordinate.
RfGps snapshot()
{
    RfGps out;
    if ( !g_ready ) {
        return out;
    }

    gps_lock();
    const uint32_t now = to_ms_since_boot( get_absolute_time() );
    const bool fresh_frames = g_last_frame_ms != 0 &&
                              ( now - g_last_frame_ms ) <= GPS_STALE_MS;
    const bool fresh_time = g_last_time_update_ms != 0 &&
                            ( now - g_last_time_update_ms ) <= GPS_STALE_MS;

    const gps::Coordinate& c = g_driver->coordinate();
    out.valid = c.valid && fresh_frames;

    if ( c.valid && fresh_frames ) {
        out.has_pos = true;
        out.lat     = c.latitude;
        out.lon     = c.longitude;
        out.alt_m   = c.altitude;
    }

    // utc_year == 0 means the time hasn't been resolved yet.
    if ( c.utc_year != 0 && fresh_time ) {
        const uint32_t ms  = c.utc_ms;
        const unsigned hh  = ( ms / 3600000u ) % 24u;
        const unsigned mm  = ( ms / 60000u ) % 60u;
        const unsigned ss  = ( ms / 1000u ) % 60u;
        snprintf( out.utc, sizeof out.utc, "%04u-%02u-%02uT%02u:%02u:%02uZ",
                  (unsigned)c.utc_year, (unsigned)c.utc_month, (unsigned)c.utc_day,
                  hh, mm, ss );
        out.has_time = true;
    }

    gps_unlock();
    return out;
}

}  // namespace

bool gps_task_init( uint8_t tx_pin, uint8_t rx_pin, uint32_t baud )
{
    g_configure_output = true;
    open_uart( GpsTaskConfig{ .tx_pin = tx_pin, .rx_pin = rx_pin, .baud = baud } );
    configure_nav_pvt();

    g_ready = true;
    enable_uart_rx_irq();
    log_print( "# [gps] UART0 @ %lu baud on GPIO%u/%u — UBX NAV-PVT @ %u ms\n",
            (unsigned long)g_config.baud,
            (unsigned)g_config.tx_pin,
            (unsigned)g_config.rx_pin,
            (unsigned)GPS_RATE_MS );
    return true;
}

bool gps_task_init_autobaud( uint8_t tx_pin, uint8_t rx_pin, uint32_t target_baud )
{
    g_configure_output = true;
    static constexpr uint32_t BAUDS[] = {
        230400, 115200, 57600, 38400, 19200, 9600, 4800, 460800, 921600,
    };

    uint32_t current_baud = 0;
    for ( uint32_t baud : BAUDS ) {
        open_uart( GpsTaskConfig{ .tx_pin = tx_pin, .rx_pin = rx_pin, .baud = baud } );
        if ( detect_valid_stream( 1200 ) ) {
            current_baud = baud;
            const auto& d = g_driver->diagnostics();
            log_print( "# [gps] detected UART0 @ %lu baud (ubx=%lu nmea=%lu)\n",
                    (unsigned long)current_baud,
                    (unsigned long)d.ubx_frames,
                    (unsigned long)d.nmea_good );
            break;
        }
    }

    if ( current_baud == 0 ) {
        log_print( "# [gps] autobaud failed; trying target baud %lu directly\n",
                (unsigned long)target_baud );
        return gps_task_init( tx_pin, rx_pin, target_baud );
    }

    if ( current_baud != target_baud ) {
        log_print( "# [gps] switching UART1 baud %lu -> %lu\n",
                (unsigned long)current_baud,
                (unsigned long)target_baud );
        g_driver->send_ubx( gps::Ubx::valset_uart1_baud( target_baud ) );
        sleep_ms( 200 );
        open_uart( GpsTaskConfig{ .tx_pin = tx_pin, .rx_pin = rx_pin, .baud = target_baud } );
    }

    configure_nav_pvt();

    g_ready = true;
    enable_uart_rx_irq();
    log_print( "# [gps] UART0 @ %lu baud on GPIO%u/%u — UBX NAV-PVT @ %u ms\n",
            (unsigned long)g_config.baud,
            (unsigned)g_config.tx_pin,
            (unsigned)g_config.rx_pin,
            (unsigned)GPS_RATE_MS );
    return true;
}

bool gps_task_init_autobaud_listen_only( uint8_t tx_pin, uint8_t rx_pin,
                                         uint32_t preferred_baud )
{
    g_configure_output = false;
    const uint32_t fallback_baud = ( preferred_baud == 0 ) ? 230400 : preferred_baud;
    const uint32_t bauds[] = {
        fallback_baud, 230400, 115200, 57600, 38400, 19200, 9600, 4800, 460800, 921600,
    };

    uint32_t current_baud = 0;
    for ( uint32_t baud : bauds ) {
        if ( baud == 0 ) {
            continue;
        }
        open_uart( GpsTaskConfig{ .tx_pin = tx_pin, .rx_pin = rx_pin, .baud = baud } );
        bool saw_nmea = false;
        if ( detect_listen_only_stream( 1600, &saw_nmea ) ) {
            current_baud = baud;
            const auto& d = g_driver->diagnostics();
            log_print( "# [gps] listen-only UART0 @ %lu baud on GPIO%u/%u (ubx=%lu nmea=%lu source=%s)\n",
                    (unsigned long)current_baud,
                    (unsigned)g_config.tx_pin,
                    (unsigned)g_config.rx_pin,
                    (unsigned long)d.ubx_frames,
                    (unsigned long)d.nmea_good,
                    saw_nmea ? "nmea" : "ubx" );
            break;
        }
    }

    if ( current_baud == 0 ) {
        open_uart( GpsTaskConfig{ .tx_pin = tx_pin, .rx_pin = rx_pin, .baud = fallback_baud } );
        log_print( "# [gps] listen-only autobaud failed; UART0 @ %lu baud on GPIO%u/%u\n",
                (unsigned long)g_config.baud,
                (unsigned)g_config.tx_pin,
                (unsigned)g_config.rx_pin );
    }

    g_ready = true;
    enable_uart_rx_irq();
    return true;
}

void gps_task_poll( void )
{
    if ( g_ready ) {
        gps_lock();
        // Drain all currently buffered UART bytes. The GM10 can emit a mixed
        // UBX+NMEA burst larger than one read chunk per navigation epoch.
        const uint32_t now = to_ms_since_boot( get_absolute_time() );
        uint8_t buf[gps::GpsDriver<gps::UartTransport>::READ_CHUNK];

        static uint32_t diag_bytes = 0;
        static uint8_t  diag_sample[48];
        static size_t   diag_sample_count = 0;
        static uint32_t last_ubx_frames = 0;
        static uint32_t last_ubx_pvt = 0;
        static uint32_t last_nmea_good = 0;
        static uint32_t last_tap_ubx_frames = 0;
        static uint32_t last_tap_nav_pvt_frames = 0;
        static uint8_t  stale_windows = 0;

        size_t n = 0;
        do {
            n = g_rx_irq_enabled ? read_rx_ring( buf, sizeof buf )
                                 : g_driver->read_raw( buf, sizeof buf );
            print_raw_uart_chunk( buf, n );
            feed_raw( buf, n );
            tap_nav_pvt( buf, n );
            diag_bytes += static_cast<uint32_t>( n );
            for ( size_t i = 0; i < n && diag_sample_count < sizeof diag_sample; ++i ) {
                diag_sample[diag_sample_count++] = buf[i];
            }
        } while ( n == sizeof buf );

        const auto& d_now = g_driver->diagnostics();
        const uint32_t frame_total = d_now.ubx_frames + d_now.nmea_good;
        if ( frame_total != g_last_frame_total ) {
            g_last_frame_total = frame_total;
            g_last_frame_ms = now;
        }

        const gps::Coordinate& c_now = g_driver->coordinate();
        if ( c_now.utc_year != 0 &&
             ( c_now.utc_year != g_last_utc_year ||
               c_now.utc_month != g_last_utc_month ||
               c_now.utc_day != g_last_utc_day ||
               c_now.utc_ms != g_last_utc_ms ) ) {
            g_last_utc_year = c_now.utc_year;
            g_last_utc_month = c_now.utc_month;
            g_last_utc_day = c_now.utc_day;
            g_last_utc_ms = c_now.utc_ms;
            g_last_time_update_ms = now;
        }

        static uint32_t last_diag_ms = 0;
        if ( now - last_diag_ms >= GPS_DIAG_MS ) {
            last_diag_ms = now;
            const auto& d = g_driver->diagnostics();
            const uint32_t ubx_delta = d.ubx_frames - last_ubx_frames;
            const uint32_t pvt_delta = d.ubx_pvt - last_ubx_pvt;
            const uint32_t nmea_delta = d.nmea_good - last_nmea_good;
            const uint32_t accepted_delta = ubx_delta + nmea_delta;
            const uint32_t tap_ubx_delta = g_tap_ubx_frames - last_tap_ubx_frames;
            const uint32_t tap_pvt_delta = g_tap_nav_pvt_frames - last_tap_nav_pvt_frames;
            const uint32_t overflow_delta = g_rx_ring_overflows - g_last_reported_overflows;
            last_ubx_frames = d.ubx_frames;
            last_ubx_pvt = d.ubx_pvt;
            last_nmea_good = d.nmea_good;
            last_tap_ubx_frames = g_tap_ubx_frames;
            last_tap_nav_pvt_frames = g_tap_nav_pvt_frames;
            g_last_reported_overflows = g_rx_ring_overflows;

            const gps::Coordinate& c = g_driver->coordinate();
            if ( g_nav_pvt_debug || !g_driver->has_fix() ) {
                log_print( "# [gps] %s bytes=%lu +raw_ubx=%lu +raw_pvt=%lu +ubx=%lu +pvt=%lu +nmea=%lu +ovf=%lu totals raw_ubx=%lu raw_pvt=%lu ubx=%lu pvt=%lu nmea=%lu bad_nmea=%lu ovf=%lu fix=%.*s sats=%d hAcc=%.1fm\n",
                        g_driver->has_fix() ? "debug" : "no fix yet:",
                        (unsigned long)diag_bytes,
                        (unsigned long)tap_ubx_delta,
                        (unsigned long)tap_pvt_delta,
                        (unsigned long)ubx_delta,
                        (unsigned long)pvt_delta,
                        (unsigned long)nmea_delta,
                        (unsigned long)overflow_delta,
                        (unsigned long)g_tap_ubx_frames,
                        (unsigned long)g_tap_nav_pvt_frames,
                        (unsigned long)d.ubx_frames,
                        (unsigned long)d.ubx_pvt,
                        (unsigned long)d.nmea_good,
                        (unsigned long)d.nmea_bad_cksum,
                        (unsigned long)g_rx_ring_overflows,
                        (int)g_driver->fix_label().size(),
                        g_driver->fix_label().data(),
                        c.satellites,
                        (double)c.h_acc_mm / 1000.0 );
                if ( diag_sample_count > 0 && tap_ubx_delta == 0 && accepted_delta == 0 ) {
                    print_raw_sample( diag_sample, diag_sample_count );
                }
            }

            if ( g_configure_output && diag_bytes > 0 && accepted_delta == 0 ) {
                ++stale_windows;
                if ( stale_windows >= 2 ) {
                    log_print( "# [gps] bytes are arriving but no frames parsed; re-sending GPS output config\n" );
                    configure_nav_pvt();
                    stale_windows = 0;
                }
            } else {
                stale_windows = 0;
            }

            diag_bytes = 0;
            diag_sample_count = 0;
        }
        gps_unlock();
    }
}

// FreeRTOS GPS task: continuously drain the UART IRQ ring and parse. Runs on
// core 1 at a priority above the radio task so it is serviced promptly; the
// 2 ms tick keeps the 8 KB ring well ahead of the byte rate even at 230400 baud.
static void gps_task( void* )
{
    for ( ;; ) {
        gps_task_poll();
        vTaskDelay( pdMS_TO_TICKS( 2 ) );
    }
}

RfGps gps_task_fix( void )
{
    return snapshot();
}

void gps_task_set_nav_pvt_debug( bool enabled )
{
    g_nav_pvt_debug = enabled;
}

bool gps_task_has_fix( void )
{
    if ( !g_ready ) {
        return false;
    }
    gps_lock();
    const bool fix = g_driver->has_fix();
    gps_unlock();
    return fix;
}

void gps_task_start( unsigned priority )
{
    if ( !g_gps_mutex ) {
        g_gps_mutex = xSemaphoreCreateMutexStatic( &g_gps_mutex_buf );
    }
    TaskHandle_t h = rtos_task_create( gps_task, "gps", 2048, nullptr,
                                       (UBaseType_t)priority,
                                       g_gps_stack, &g_gps_tcb );
    vTaskCoreAffinitySet( h, 1u << 1 );  // core 1 (alongside the radio task)
}
