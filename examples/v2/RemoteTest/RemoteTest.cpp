// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — RemoteTest
//
// Automated test flow for the mux protocol over serial. Connects to a
// device running RemoteServer firmware, exercises hello/ping, I2C bus
// scan, and SPI full-duplex data channel echo.
//
// Build & run:
//   pio run -e RemoteTest_host
//   .pio/build/RemoteTest_host/program [port] [baud]
//     port:  serial device path (default: auto-discover)
//     baud:  baud rate (default: 3000000 for espidf device)
// =============================================================================

#if !defined(ARDUINO)

#include <M5HAL_v2.hpp>
#include <m5_hal/variants/frameworks/posix/hal/remote/wire_dump.hpp>

#include <new>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace m5hal        = m5::hal::v2;
namespace posix_uart   = m5::variants::frameworks::posix::hal::v2::uart;
namespace posix_tcp    = m5::variants::frameworks::posix::hal::v2::tcp;
namespace posix_remote = m5::variants::frameworks::posix::hal::v2::remote;

// ---------------------------------------------------------------------------
// Serial port connection (mux hello instead of v2 frame hello)
// ---------------------------------------------------------------------------
namespace {

struct MuxHostEndpoint {
    m5hal::uart::Bus_posix bus;
    m5hal::uart::AccessConfig uart_cfg;
    m5hal::uart::RxAccessor rx;
    m5hal::uart::TxAccessor tx;
    // TCP transport (used when the port string starts with "tcp:").
    posix_tcp::TcpStream tcp_stream;
    bool is_tcp = false;

    uint8_t rx_scratch[4096];
    uint8_t tx_scratch[4096];
    // HelloResp caps: [proto_ver][flags][bus_count].
    uint8_t proto_ver  = 0;
    uint8_t caps_flags = 0;
    uint8_t bus_count  = 0;

    // M5HAL_WIRE_DUMP capture taps, one pair per transport. Pass-through when
    // the env var is unset.
    posix_remote::WireDump* wire_dump = posix_remote::WireDump::fromEnv();
    posix_remote::WireDumpWriter dump_uart_rx{wire_dump, "uart", '<'};
    posix_remote::WireDumpWriter dump_uart_tx{wire_dump, "uart", '>'};
    posix_remote::WireDumpWriter dump_tcp_rx{wire_dump, "tcp", '<'};
    posix_remote::WireDumpWriter dump_tcp_tx{wire_dump, "tcp", '>'};
    m5hal::data::TapReader tap_rx{rx, &dump_uart_rx};
    m5hal::data::TapWriter tap_tx{tx, &dump_uart_tx};
    m5hal::data::TapReader tap_tcp_rx{tcp_stream, &dump_tcp_rx};
    m5hal::data::TapWriter tap_tcp_tx{tcp_stream, &dump_tcp_tx};

    // Wire source/sink are reconstructed in connect() to bind to whichever
    // transport (serial accessors or the TCP stream) is active.
    m5hal::data::StreamSource wire_src{tap_rx, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    m5hal::data::StreamSink wire_snk{tap_tx, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
    m5hal::data::MuxFrameEncoder enc{m5hal::memory::defaultAllocator()};
    m5hal::data::MuxFrameDecoder dec{m5hal::memory::defaultAllocator()};
    m5hal::remote::RemoteSession sess{enc, dec, wire_src, wire_snk};
    m5hal::remote::RemoteSession* session = &sess;

    explicit MuxHostEndpoint(uint32_t baud) : uart_cfg(makeConfig(baud)), rx{bus, uart_cfg}, tx{bus, uart_cfg}
    {
    }

    static m5hal::uart::AccessConfig makeConfig(uint32_t baud)
    {
        m5hal::uart::AccessConfig cfg;
        cfg.baud_rate             = baud;
        cfg.first_byte_timeout_ms = 2;
        cfg.inter_byte_timeout_ms = 2;
        cfg.write_timeout_ms      = 1000;
        return cfg;
    }

    bool connect(const char* port)
    {
        if (port != nullptr && ::strncmp(port, "tcp:", 4) == 0) {
            return connectTcp(port + 4);
        }
        return connectSerial(port);
    }

    // Connect over a serial device. `port` is a tty path.
    bool connectSerial(const char* port)
    {
        is_tcp = false;
        m5hal::uart::BusConfig_posix bus_cfg;
        bus_cfg.tx_coalesce_bytes = 4096;
        (void)bus.init(bus_cfg);

        auto err = bus.open(port, uart_cfg.baud_rate);
        if (err != m5hal::error::error_t::OK) {
            ::fprintf(stderr, "open %s failed: %d\n", port, static_cast<int>(err));
            return false;
        }

        ::usleep(500 * 1000);
        (void)::tcflush(bus.nativeHandle(), TCIOFLUSH);

        bindEndpoint(tap_rx, tap_tx);
        return true;
    }

    // Connect over TCP. `hostport` is "<host>:<port>".
    bool connectTcp(const char* hostport)
    {
        is_tcp = true;
        char host[256];
        ::snprintf(host, sizeof(host), "%s", hostport);
        char* colon = ::strrchr(host, ':');
        if (colon == nullptr) {
            ::fprintf(stderr, "bad tcp endpoint %s (want host:port)\n", hostport);
            return false;
        }
        *colon         = '\0';
        uint16_t tport = static_cast<uint16_t>(::strtoul(colon + 1, nullptr, 10));

        auto err = tcp_stream.connect(host, tport);
        // Session pump paces itself; long first-byte waits just add latency.
        tcp_stream.read_timeout_ms = 1;
        if (err != m5hal::error::error_t::OK) {
            ::fprintf(stderr, "tcp connect %s:%u failed: %d\n", host, static_cast<unsigned>(tport),
                      static_cast<int>(err));
            return false;
        }

        bindEndpoint(tap_tcp_rx, tap_tcp_tx);
        return true;
    }

    void bindEndpoint(m5hal::data::StreamReader& reader, m5hal::data::StreamWriter& writer)
    {
        wire_src = m5hal::data::StreamSource{reader, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
        wire_snk = m5hal::data::StreamSink{writer, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};

        enc.releaseAll();
        dec.releaseAll();
        // RemoteSession registers itself as the decoder's frame handler in
        // its constructor, so it must be reconstructed in place (assignment
        // from a temporary would leave the decoder pointing at the dead
        // temporary and every response would be dropped — hello timeout).
        sess.~RemoteSession();
        new (&sess) m5hal::remote::RemoteSession{enc, dec, wire_src, wire_snk};
        session = &sess;
    }

    void release()
    {
        if (is_tcp) {
            (void)tcp_stream.close();
        } else {
            (void)bus.release();
        }
    }
};

// ---------------------------------------------------------------------------
// Auto-discover serial port (walk candidates, try mux hello)
// ---------------------------------------------------------------------------
bool autoDiscover(MuxHostEndpoint& host, char* found_path, size_t cap)
{
    posix_uart::SerialPortInfo ports[8];
    size_t n = posix_uart::listSerialPorts(ports, sizeof(ports) / sizeof(ports[0]));
    for (size_t i = 0; i < n; ++i) {
        if (ports[i].rank > 1) {
            continue;
        }
        ::printf("trying %s ...\n", ports[i].path);
        if (!host.connect(ports[i].path)) {
            continue;
        }
        auto r = host.session->hello();
        if (r.has_value()) {
            ::snprintf(found_path, cap, "%s", ports[i].path);
            return true;
        }
        (void)host.bus.release();
    }
    return false;
}

// ---------------------------------------------------------------------------
// Test: GPIO read via bytecode
// ---------------------------------------------------------------------------
void testPortRead(MuxHostEndpoint& host, int slot, int port_index)
{
    ::printf("\n--- Port Read (slot %d, port %d) ---\n", slot, port_index);
    uint8_t script_buf[32];
    m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
    m5hal::bytecode::BytecodeEncoder enc{script_sink};
    (void)enc.gpioPortRead(0, static_cast<m5hal::types::gpio_slot_t>(slot), static_cast<uint8_t>(port_index));
    (void)enc.end();

    auto r = host.session->request({script_buf, script_sink.written()});
    if (!r.has_value()) {
        ::printf("  request failed: %d\n", static_cast<int>(r.error()));
        return;
    }
    auto resp = host.session->lastResponse();
    if (resp.size == 0) {
        ::printf("  empty response\n");
        return;
    }
    m5hal::bytecode::BytecodeRunner runner{m5hal::memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto run_r = runner.run(resp);
    if (!run_r.has_value() || !runner.statusReported()) {
        ::printf("  decode failed\n");
        return;
    }
    if (runner.reportedStatus() != m5hal::error::error_t::OK) {
        ::printf("  error: %d\n", static_cast<int>(runner.reportedStatus()));
        return;
    }
    auto stored = runner.storedData(0);
    if (stored.size >= 4) {
        uint32_t val = static_cast<uint32_t>(stored.data[0]) | (static_cast<uint32_t>(stored.data[1]) << 8) |
                       (static_cast<uint32_t>(stored.data[2]) << 16) | (static_cast<uint32_t>(stored.data[3]) << 24);
        ::printf("  Port[%d][%d] = 0x%08X  (", slot, port_index, val);
        for (int b = 31; b >= 0; --b) {
            ::putchar((val & (1u << b)) ? '1' : '0');
            if (b > 0 && (b % 8) == 0) {
                ::putchar('_');
            }
        }
        ::printf(")\n");
    } else {
        ::printf("  no data in store\n");
    }
}

void testPortWrite(MuxHostEndpoint& host, int slot, int port_index, uint32_t set_mask, uint32_t clear_mask)
{
    ::printf("  Port[%d][%d] set=0x%08X clear=0x%08X\n", slot, port_index, set_mask, clear_mask);
    uint8_t script_buf[32];
    m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
    m5hal::bytecode::BytecodeEncoder enc{script_sink};
    (void)enc.gpioPortWrite(static_cast<m5hal::types::gpio_slot_t>(slot), static_cast<uint8_t>(port_index), set_mask,
                            clear_mask);
    (void)enc.end();

    auto r = host.session->request({script_buf, script_sink.written()});
    if (!r.has_value()) {
        ::printf("    request failed: %d\n", static_cast<int>(r.error()));
    }
}

void testGpioRead(MuxHostEndpoint& host, int pin)
{
    ::printf("\n--- GPIO Read (pin %d) ---\n", pin);
    uint8_t script_buf[32];
    m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
    m5hal::bytecode::BytecodeEncoder enc{script_sink};
    m5hal::types::gpio_number_t gnum = static_cast<m5hal::types::gpio_number_t>(pin);
    (void)enc.gpioRead(0, &gnum, 1);
    (void)enc.end();

    auto r = host.session->request({script_buf, script_sink.written()});
    if (!r.has_value()) {
        ::printf("  request failed: %d\n", static_cast<int>(r.error()));
        return;
    }
    auto resp = host.session->lastResponse();
    if (resp.size == 0) {
        ::printf("  empty response\n");
        return;
    }
    m5hal::bytecode::BytecodeRunner runner{m5hal::memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto run_r = runner.run(resp);
    if (!run_r.has_value() || !runner.statusReported()) {
        ::printf("  decode failed\n");
        return;
    }
    if (runner.reportedStatus() != m5hal::error::error_t::OK) {
        ::printf("  error: %d\n", static_cast<int>(runner.reportedStatus()));
        return;
    }
    auto stored = runner.storedData(0);
    if (stored.size > 0) {
        bool level = (stored.data[0] & 1) != 0;
        ::printf("  GPIO %d = %s\n", pin, level ? "HIGH" : "LOW");
    } else {
        ::printf("  no data in store\n");
    }
}

// Returns 0 on success, otherwise the (negative) remote error code.
int testGpioWrite(MuxHostEndpoint& host, int pin, bool high)
{
    ::printf("  GPIO %d -> %s\n", pin, high ? "HIGH" : "LOW");
    uint8_t script_buf[32];
    m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
    m5hal::bytecode::BytecodeEncoder enc{script_sink};
    m5hal::types::gpio_number_t gnum = static_cast<m5hal::types::gpio_number_t>(pin);
    if (high) {
        (void)enc.gpioWriteHigh(&gnum, 1);
    } else {
        (void)enc.gpioWriteLow(&gnum, 1);
    }
    (void)enc.end();

    auto r = host.session->request({script_buf, script_sink.written()});
    if (!r.has_value()) {
        return static_cast<int>(r.error());
    }
    auto resp = host.session->lastResponse();
    m5hal::bytecode::BytecodeRunner runner{m5hal::memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    (void)runner.run(resp);
    if (runner.statusReported() && runner.reportedStatus() != m5hal::error::error_t::OK) {
        return static_cast<int>(runner.reportedStatus());
    }
    return 0;
}

void testGpioSetMode(MuxHostEndpoint& host, int pin, m5hal::types::gpio_mode_t mode)
{
    uint8_t script_buf[32];
    m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
    m5hal::bytecode::BytecodeEncoder enc{script_sink};
    m5hal::types::gpio_number_t gnum = static_cast<m5hal::types::gpio_number_t>(pin);
    (void)enc.gpioSetMode(mode, &gnum, 1);
    (void)enc.end();
    (void)host.session->request({script_buf, script_sink.written()});
}

// ---------------------------------------------------------------------------
// Test: I2C bus scan via bytecode
// ---------------------------------------------------------------------------
void testI2cScan(MuxHostEndpoint& host, int bus_id)
{
    ::printf("\n--- I2C Bus Scan (bus_id=%d) ---\n", bus_id);
    int found = 0;

    for (uint16_t addr = 0x08; addr <= 0x77; ++addr) {
        uint8_t script_buf[64];
        m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
        m5hal::bytecode::BytecodeEncoder enc{script_sink};
        m5hal::i2c::MasterAccessConfig cfg;
        cfg.i2c_addr        = addr;
        cfg.wire_timeout_ms = 100;
        (void)enc.configure(static_cast<uint8_t>(bus_id), cfg);
        m5hal::i2c::TransferDesc desc{};
        (void)enc.transfer(static_cast<uint8_t>(bus_id), desc, {nullptr, 0}, 0, 0);
        (void)enc.end();

        auto r = host.session->request({script_buf, script_sink.written()});
        if (r.has_value()) {
            auto resp = host.session->lastResponse();
            if (resp.size > 0) {
                m5hal::bytecode::BytecodeRunner runner{m5hal::memory::defaultAllocator()};
                runner.setReceiveOnly(true);
                auto run_r = runner.run(resp);
                if (run_r.has_value() && runner.statusReported() &&
                    runner.reportedStatus() == m5hal::error::error_t::OK) {
                    ::printf("  0x%02X: ACK\n", addr);
                    ++found;
                }
            }
        }
    }
    ::printf("  scan done: %d device(s)\n", found);
}

// ---------------------------------------------------------------------------
// Test: I2C register read (write reg addr, then read `len` bytes)
// ---------------------------------------------------------------------------
void testRegWrite(MuxHostEndpoint& host, int i2c_addr, int reg_addr, int value, int bus_id)
{
    ::printf("\n--- I2C Register Write (bus_id=%d) ---\n", bus_id);
    ::printf("  0x%02X reg 0x%02X <= 0x%02X:", i2c_addr, reg_addr, value);

    uint8_t script_buf[64];
    m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
    m5hal::bytecode::BytecodeEncoder enc{script_sink};
    m5hal::i2c::MasterAccessConfig cfg;
    cfg.i2c_addr        = static_cast<uint16_t>(i2c_addr);
    cfg.wire_timeout_ms = 100;
    (void)enc.configure(static_cast<uint8_t>(bus_id), cfg);
    m5hal::i2c::TransferDesc desc{static_cast<uint8_t>(reg_addr)};
    uint8_t v = static_cast<uint8_t>(value);
    (void)enc.transfer(static_cast<uint8_t>(bus_id), desc, {&v, 1}, 0, 0);
    (void)enc.end();

    auto r = host.session->request({script_buf, script_sink.written()});
    if (!r.has_value()) {
        ::printf(" ERR(%d)\n", static_cast<int>(r.error()));
        return;
    }
    auto chk = host.session->checkResponse();
    if (!chk.has_value()) {
        ::printf(" ERR(%d)\n", static_cast<int>(chk.error()));
        return;
    }
    ::printf(" OK\n");
}

void testRegRead(MuxHostEndpoint& host, int i2c_addr, int reg_addr, int len, int bus_id)
{
    ::printf("\n--- I2C Register Read (bus_id=%d) ---\n", bus_id);

    // Print the header first so callers (and smoke scripts) always see the
    // "0x.. reg 0x.. [N byte(s)]:" line; the byte values (or ERR markers)
    // follow on the same line.
    ::printf("  0x%02X reg 0x%02X [%d byte(s)]:", i2c_addr, reg_addr, len);

    uint8_t script_buf[64];
    m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
    m5hal::bytecode::BytecodeEncoder enc{script_sink};
    m5hal::i2c::MasterAccessConfig cfg;
    cfg.i2c_addr        = static_cast<uint16_t>(i2c_addr);
    cfg.wire_timeout_ms = 100;
    (void)enc.configure(static_cast<uint8_t>(bus_id), cfg);
    m5hal::i2c::TransferDesc desc{static_cast<uint8_t>(reg_addr)};
    (void)enc.transfer(static_cast<uint8_t>(bus_id), desc, {nullptr, 0}, static_cast<size_t>(len), 0);
    (void)enc.end();

    int err = 0;
    auto r  = host.session->request({script_buf, script_sink.written()});
    if (!r.has_value()) {
        err = static_cast<int>(r.error());
    } else {
        auto resp = host.session->lastResponse();
        m5hal::bytecode::BytecodeRunner runner{m5hal::memory::defaultAllocator()};
        runner.setReceiveOnly(true);
        auto run_r = runner.run(resp);
        if (resp.size == 0 || !run_r.has_value() || !runner.statusReported()) {
            err = static_cast<int>(m5hal::error::error_t::REMOTE_FAULT);
        } else if (runner.reportedStatus() != m5hal::error::error_t::OK) {
            err = static_cast<int>(runner.reportedStatus());
        } else {
            auto stored = runner.storedData(0);
            for (size_t i = 0; i < stored.size; ++i) {
                ::printf(" 0x%02X", stored.data[i]);
            }
        }
    }
    if (err != 0) {
        ::printf(" ERR(%d)", err);
    }
    ::printf("\n");
}

// RemoteServer statically registers the SPI bus at this id (see RemoteServer.cpp BUS_ID_SPI).
constexpr uint8_t kSpiBusId = 2;

// ---------------------------------------------------------------------------
// Test: SPI large data transfer via data channel
// Simulates 320x240 16bpp framebuffer (153600 bytes).
// Throughput-only: validates byte count, not payload content.
// ---------------------------------------------------------------------------
void testSpiTransfer(MuxHostEndpoint& host, size_t frame_size)
{
    ::printf("\n--- SPI Large Transfer (%zu bytes) ---\n", frame_size);
    static constexpr size_t kMaxFrame = 320 * 240 * 2;  // 153600
    if (frame_size > kMaxFrame) {
        frame_size = kMaxFrame;
    }
    static uint8_t tx_buf[kMaxFrame];
    static uint8_t rx_buf[kMaxFrame];
    for (size_t i = 0; i < frame_size; ++i) {
        tx_buf[i] = static_cast<uint8_t>(i);
    }

    m5hal::spi::Bus_remote bus{*host.session, kSpiBusId};
    m5hal::spi::MasterAccessConfig cfg;
    m5hal::spi::TransferDesc desc;
    m5hal::data::MemorySource src{tx_buf, frame_size};
    m5hal::data::MemorySink dst{rx_buf, frame_size};

    auto t0      = m5::utility::millis();
    auto r       = bus.transfer(nullptr, cfg, desc, &src, frame_size, &dst, frame_size);
    auto elapsed = static_cast<uint32_t>(m5::utility::millis()) - static_cast<uint32_t>(t0);
    if (!r.has_value()) {
        ::printf("  transfer failed: %d\n", static_cast<int>(r.error()));
        return;
    }
    double kbps = elapsed > 0 ? (static_cast<double>(frame_size) * 8.0 / elapsed) : 0.0;
    ::printf("  transferred %zu bytes in %u ms (%.1f kbps)\n", frame_size, elapsed, kbps);
}

// ---------------------------------------------------------------------------
// Test: dynamic I2C bus create + quick 0x34 probe
// ---------------------------------------------------------------------------
void testI2cCreate(MuxHostEndpoint& host, int scl, int sda, int bus_id)
{
    ::printf("\n--- BusCreate I2C (scl=%d sda=%d bus_id=%d) ---\n", scl, sda, bus_id);
    uint8_t pin[4];
    pin[0] = static_cast<uint8_t>(scl);
    pin[1] = static_cast<uint8_t>(scl >> 8);
    pin[2] = static_cast<uint8_t>(sda);
    pin[3] = static_cast<uint8_t>(sda >> 8);
    uint8_t sb[32];
    m5hal::data::MemorySink ss(sb, sizeof(sb));
    m5hal::bytecode::BytecodeEncoder enc{ss};
    (void)enc.busCreate(m5hal::types::bus_kind_t::I2C, static_cast<uint8_t>(bus_id), 0xFF, {pin, 4});
    (void)enc.end();
    auto r = host.session->request({sb, ss.written()});
    if (!r.has_value()) {
        ::printf("  request failed: %d\n", static_cast<int>(r.error()));
    } else {
        auto resp = host.session->lastResponse();
        m5hal::bytecode::BytecodeRunner runner{m5hal::memory::defaultAllocator()};
        runner.setReceiveOnly(true);
        (void)runner.run(resp);
        if (runner.statusReported() && runner.reportedStatus() == m5hal::error::error_t::OK) {
            ::printf("  OK\n");
        } else if (runner.statusReported()) {
            ::printf("  FAILED (status=%d)\n", static_cast<int>(runner.reportedStatus()));
        } else {
            ::printf("  FAILED (no status)\n");
        }
    }
    // Quick probe 0x34 on the newly created bus
    m5hal::data::MemorySink ps(sb, sizeof(sb));
    m5hal::bytecode::BytecodeEncoder pe{ps};
    m5hal::i2c::MasterAccessConfig pcfg;
    pcfg.i2c_addr        = 0x34;
    pcfg.wire_timeout_ms = 200;
    (void)pe.configure(static_cast<uint8_t>(bus_id), pcfg);
    m5hal::i2c::TransferDesc pdesc{};
    (void)pe.transfer(static_cast<uint8_t>(bus_id), pdesc, {nullptr, 0}, 0, 0);
    (void)pe.end();
    auto pr = host.session->request({sb, ps.written()});
    if (!pr.has_value()) {
        ::printf("  probe 0x34: no response\n");
    } else {
        auto probe_resp = host.session->lastResponse();
        m5hal::bytecode::BytecodeRunner probe_runner{m5hal::memory::defaultAllocator()};
        probe_runner.setReceiveOnly(true);
        (void)probe_runner.run(probe_resp);
        if (probe_runner.statusReported() && probe_runner.reportedStatus() == m5hal::error::error_t::OK) {
            ::printf("  probe 0x34: ACK\n");
        } else {
            ::printf("  probe 0x34: NACK\n");
        }
    }
}

// ---------------------------------------------------------------------------
// Helper: run a one-shot bus-create request and print OK/FAILED.
// ---------------------------------------------------------------------------
void runBusCreate(MuxHostEndpoint& host, m5hal::types::bus_kind_t kind, int bus_id, const uint8_t* pin, size_t pin_len)
{
    uint8_t sb[64];
    m5hal::data::MemorySink ss(sb, sizeof(sb));
    m5hal::bytecode::BytecodeEncoder enc{ss};
    (void)enc.busCreate(kind, static_cast<uint8_t>(bus_id), 0xFF, {pin, pin_len});
    (void)enc.end();
    auto r = host.session->request({sb, ss.written()});
    if (!r.has_value()) {
        ::printf("  request failed: %d\n", static_cast<int>(r.error()));
        return;
    }
    auto resp = host.session->lastResponse();
    m5hal::bytecode::BytecodeRunner runner{m5hal::memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    (void)runner.run(resp);
    ::printf("  %s\n",
             (runner.statusReported() && runner.reportedStatus() == m5hal::error::error_t::OK) ? "OK" : "FAILED");
}

// ---------------------------------------------------------------------------
// Test: dynamic SPI bus create
// pin_config: [clk:i16LE][mosi:i16LE][miso:i16LE]
// ---------------------------------------------------------------------------
void testSpiCreate(MuxHostEndpoint& host, int clk, int mosi, int miso, int bus_id)
{
    ::printf("\n--- BusCreate SPI (clk=%d mosi=%d miso=%d bus_id=%d) ---\n", clk, mosi, miso, bus_id);
    uint8_t pin[6];
    pin[0] = static_cast<uint8_t>(clk);
    pin[1] = static_cast<uint8_t>(clk >> 8);
    pin[2] = static_cast<uint8_t>(mosi);
    pin[3] = static_cast<uint8_t>(mosi >> 8);
    pin[4] = static_cast<uint8_t>(miso);
    pin[5] = static_cast<uint8_t>(miso >> 8);
    runBusCreate(host, m5hal::types::bus_kind_t::SPI, bus_id, pin, sizeof(pin));
}

// ---------------------------------------------------------------------------
// Test: dynamic UART bus create
// pin_config: [tx:i16LE][rx:i16LE][port_num:u8][rx_buf_unit:u8][tx_buf_unit:u8]
// Buffer units are page counts (one page = 256 bytes on the device side).
// ---------------------------------------------------------------------------
void testUartCreate(MuxHostEndpoint& host, int tx, int rx, int bus_id, int rx_buf_pages, int tx_buf_pages)
{
    ::printf("\n--- BusCreate UART (tx=%d rx=%d bus_id=%d rx_pages=%d tx_pages=%d) ---\n", tx, rx, bus_id, rx_buf_pages,
             tx_buf_pages);
    uint8_t pin[7];
    pin[0] = static_cast<uint8_t>(tx);
    pin[1] = static_cast<uint8_t>(tx >> 8);
    pin[2] = static_cast<uint8_t>(rx);
    pin[3] = static_cast<uint8_t>(rx >> 8);
    pin[4] = 0;  // port_num
    pin[5] = static_cast<uint8_t>(rx_buf_pages);
    pin[6] = static_cast<uint8_t>(tx_buf_pages);
    runBusCreate(host, m5hal::types::bus_kind_t::UART, bus_id, pin, sizeof(pin));
}

// ---------------------------------------------------------------------------
// Test: dynamic I2S bus create
// pin_config: [bclk:i16LE][ws:i16LE][dout:i16LE][din:i16LE][role:u8][tx_buf_kB:u8][rx_buf_kB:u8]
// role: 0 = Master, 1 = Slave.
// ---------------------------------------------------------------------------
void testI2sCreate(MuxHostEndpoint& host, int bclk, int ws, int dout, int din, int bus_id, int role, int tx_kB,
                   int rx_kB)
{
    ::printf("\n--- BusCreate I2S (bclk=%d ws=%d dout=%d din=%d bus_id=%d role=%d tx=%dkB rx=%dkB) ---\n", bclk, ws,
             dout, din, bus_id, role, tx_kB, rx_kB);
    uint8_t pin[11];
    pin[0]  = static_cast<uint8_t>(bclk);
    pin[1]  = static_cast<uint8_t>(bclk >> 8);
    pin[2]  = static_cast<uint8_t>(ws);
    pin[3]  = static_cast<uint8_t>(ws >> 8);
    pin[4]  = static_cast<uint8_t>(dout);
    pin[5]  = static_cast<uint8_t>(dout >> 8);
    pin[6]  = static_cast<uint8_t>(din);
    pin[7]  = static_cast<uint8_t>(din >> 8);
    pin[8]  = static_cast<uint8_t>(role);
    pin[9]  = static_cast<uint8_t>(tx_kB);
    pin[10] = static_cast<uint8_t>(rx_kB);
    runBusCreate(host, m5hal::types::bus_kind_t::I2S, bus_id, pin, sizeof(pin));
}

// ---------------------------------------------------------------------------
// Test: UART transfer (write text, then read back rx_len bytes)
// ---------------------------------------------------------------------------
void testUartXfer(MuxHostEndpoint& host, int bus_id, const char* text, size_t rx_len)
{
    size_t tx_len = ::strlen(text);
    if (rx_len == 0) {
        rx_len = tx_len;
    }
    ::printf("\n--- UART Transfer (bus_id=%d tx=%zu rx=%zu) ---\n", bus_id, tx_len, rx_len);

    uint8_t script_buf[128];
    m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
    m5hal::bytecode::BytecodeEncoder enc{script_sink};
    m5hal::uart::AccessConfig cfg;
    cfg.baud_rate             = 115200;
    cfg.first_byte_timeout_ms = 500;
    cfg.inter_byte_timeout_ms = 20;
    cfg.write_timeout_ms      = 500;
    (void)enc.configure(static_cast<uint8_t>(bus_id), cfg);
    (void)enc.uartTransfer(static_cast<uint8_t>(bus_id), {reinterpret_cast<const uint8_t*>(text), tx_len}, rx_len, 0);
    (void)enc.end();

    auto r = host.session->request({script_buf, script_sink.written()});
    if (!r.has_value()) {
        ::printf("  request failed: %d\n", static_cast<int>(r.error()));
        return;
    }
    auto resp = host.session->lastResponse();
    m5hal::bytecode::BytecodeRunner runner{m5hal::memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto run_r = runner.run(resp);
    if (resp.size == 0 || !run_r.has_value() || !runner.statusReported()) {
        ::printf("  decode failed\n");
        return;
    }
    if (runner.reportedStatus() != m5hal::error::error_t::OK) {
        ::printf("  error: %d\n", static_cast<int>(runner.reportedStatus()));
        return;
    }
    auto stored = runner.storedData(0);
    ::printf("  rx [%zu byte(s)]:", stored.size);
    for (size_t i = 0; i < stored.size; ++i) {
        ::printf(" 0x%02X", stored.data[i]);
    }
    ::printf("\n");
}

// ---------------------------------------------------------------------------
// Test: SPI bytecode transfer (configure + small ramp transfer)
// ---------------------------------------------------------------------------
void testSpiBytecode(MuxHostEndpoint& host, int bus_id, int mode, int freq, int cs, size_t tx_len)
{
    ::printf("\n--- SPI Bytecode Transfer (bus_id=%d mode=%d freq=%d cs=%d tx=%zu) ---\n", bus_id, mode, freq, cs,
             tx_len);
    if (tx_len > 64) {
        tx_len = 64;  // keep within the script buffer
    }

    uint8_t script_buf[128];
    m5hal::data::MemorySink script_sink(script_buf, sizeof(script_buf));
    m5hal::bytecode::BytecodeEncoder enc{script_sink};
    m5hal::spi::MasterAccessConfig cfg;
    cfg.pin_cs   = static_cast<m5hal::types::gpio_number_t>(cs);
    cfg.freq     = static_cast<uint32_t>(freq);
    cfg.spi_mode = static_cast<uint8_t>(mode) & 0x03;
    (void)enc.configure(static_cast<uint8_t>(bus_id), cfg);

    uint8_t tx_data[64];
    for (size_t i = 0; i < tx_len; ++i) {
        tx_data[i] = static_cast<uint8_t>(i);
    }
    m5hal::spi::TransferDesc desc{};
    (void)enc.transfer(static_cast<uint8_t>(bus_id), desc, {tx_data, tx_len}, tx_len, 0);
    (void)enc.end();

    auto r = host.session->request({script_buf, script_sink.written()});
    if (!r.has_value()) {
        ::printf("  request failed: %d\n", static_cast<int>(r.error()));
        return;
    }
    auto resp = host.session->lastResponse();
    m5hal::bytecode::BytecodeRunner runner{m5hal::memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto run_r = runner.run(resp);
    if (resp.size == 0 || !run_r.has_value() || !runner.statusReported()) {
        ::printf("  decode failed\n");
        return;
    }
    if (runner.reportedStatus() != m5hal::error::error_t::OK) {
        ::printf("  error: %d\n", static_cast<int>(runner.reportedStatus()));
        return;
    }
    auto stored = runner.storedData(0);
    ::printf("  rx [%zu byte(s)]:", stored.size);
    for (size_t i = 0; i < stored.size; ++i) {
        ::printf(" 0x%02X", stored.data[i]);
    }
    ::printf("\n");
}

// ---------------------------------------------------------------------------
// Test: device reset (optionally wait for reconnect on serial transports)
// ---------------------------------------------------------------------------
bool testReset(MuxHostEndpoint& host, const char* port_buf, bool wait)
{
    ::printf("\n--- Sending Reset ---\n");
    auto r = host.session->reset();
    ::printf("  Reset %s\n", r.has_value() ? "sent" : "FAILED");
    if (!wait) {
        ::usleep(100 * 1000);
        return false;  // caller should exit
    }
    ::printf("  Waiting for reboot...\n");
    host.release();
    bool reconnected = false;
    for (int attempt = 0; attempt < 30; ++attempt) {
        ::usleep(500 * 1000);
        if (!host.is_tcp && ::access(port_buf, F_OK) != 0) {
            continue;
        }
        ::usleep(300 * 1000);
        if (!host.connect(port_buf)) {
            continue;
        }
        if (host.session->hello().has_value()) {
            reconnected = true;
            break;
        }
        host.release();
    }
    ::printf("  Reconnect Hello %s\n", reconnected ? "OK" : "FAILED");
    return reconnected;
}

// ---------------------------------------------------------------------------
// Test: ping N times, report success ratio
// ---------------------------------------------------------------------------
void testPing(MuxHostEndpoint& host, int count)
{
    if (count <= 0) {
        count = 10;
    }
    int ok  = 0;
    auto t0 = m5::utility::millis();
    for (int i = 0; i < count; ++i) {
        if (host.session->ping().has_value()) {
            ++ok;
        }
    }
    auto elapsed = static_cast<uint32_t>(m5::utility::millis()) - static_cast<uint32_t>(t0);
    double rtt   = count > 0 ? static_cast<double>(elapsed) / count : 0.0;
    ::printf("\n--- Ping ---\n");
    ::printf("  ping %d/%d  (avg RTT %.2f ms)\n", ok, count, rtt);
}

// ---------------------------------------------------------------------------
// SPI repeat test (64-byte rounds) — exercises the data channel.
// ---------------------------------------------------------------------------
void testSpiRepeat(MuxHostEndpoint& host, size_t xfer_size, size_t max_rounds)
{
    ::printf("\n--- SPI repeat test: %zu bytes x %zu rounds ---\n", xfer_size, max_rounds);
    static constexpr size_t kMaxXfer = sizeof(host.tx_scratch);
    if (xfer_size > kMaxXfer) {
        xfer_size = kMaxXfer;
    }
    static uint8_t tx_buf[kMaxXfer];
    static uint8_t rx_buf[kMaxXfer];
    for (size_t i = 0; i < xfer_size; ++i) {
        tx_buf[i] = static_cast<uint8_t>(i);
    }

    m5hal::spi::Bus_remote bus{*host.session, kSpiBusId};
    m5hal::spi::MasterAccessConfig cfg;
    m5hal::spi::TransferDesc desc;

    size_t ok = 0;
    auto t0   = m5::utility::millis();
    for (size_t round = 0; round < max_rounds; ++round) {
        m5hal::data::MemorySource src{tx_buf, xfer_size};
        m5hal::data::MemorySink dst{rx_buf, xfer_size};
        auto r = bus.transfer(nullptr, cfg, desc, &src, xfer_size, &dst, xfer_size);
        if (r.has_value()) {
            ++ok;
        }
    }
    auto elapsed = static_cast<uint32_t>(m5::utility::millis()) - static_cast<uint32_t>(t0);
    ::printf("  %zu/%zu rounds OK (%u ms total)\n", ok, max_rounds, elapsed);
}

void printCaps(MuxHostEndpoint& host)
{
    ::printf("\n--- Capabilities ---\n");
    ::printf("  remote proto v%u\n", static_cast<unsigned>(host.proto_ver));
    ::printf("  flags: 0x%02X (%s%s%s)\n", static_cast<unsigned>(host.caps_flags),
             (host.caps_flags & 0x01) ? "has_gpio" : "", ((host.caps_flags & 0x03) == 0x03) ? " | " : "",
             (host.caps_flags & 0x02) ? "supports_bus_create" : "");
    ::printf("  bus_count: %u\n", static_cast<unsigned>(host.bus_count));
}

void printUsage(const char* prog)
{
    ::printf("Usage: %s [port] [baud] [command [args...]]\n", prog);
    ::printf("\n");
    ::printf("  port            Serial device path or tcp:<host>:<port> (default: auto-discover)\n");
    ::printf("  baud            Baud rate (default: 3000000)\n");
    ::printf("\n");
    ::printf("With a command, runs that single command and exits. With no command,\n");
    ::printf("reads commands line-by-line from stdin (REPL mode) until EOF or quit.\n");
    ::printf("\nCommands:\n");
    ::printf("  caps                      Show device capabilities from the hello exchange\n");
    ::printf("  ping                      Ping the device 10 times, report success ratio\n");
    ::printf("  reset                     Send reset and exit\n");
    ::printf("  reset-wait                Send reset, wait for re-enumeration, reconnect\n");
    ::printf("  i2c-create <scl> <sda> [bus_id]  Create a dynamic I2C bus\n");
    ::printf("  i2c-scan [bus_id]         I2C bus scan (alias: scan)\n");
    ::printf("  reg <addr> <reg> <len> [bus_id]  Read `len` bytes from an I2C register\n");
    ::printf("  regw <addr> <reg> <val> [bus_id]  Write one byte to an I2C register\n");
    ::printf("  gpio-read <pin>           Read a single GPIO pin\n");
    ::printf("  gpio-write <pin> <0|1>    Set pin mode to output, write, then read back\n");
    ::printf("  gpio-scan                 Read all GPIO pins 0-39\n");
    ::printf("  port-read [slot] [port]   Read a 32-bit GPIO port (default: slot=0 port=0)\n");
    ::printf("  port-write <slot> <port> <set> <clear>  Write port with W1TS/W1TC masks\n");
    ::printf("  spi-create <clk> <mosi> <miso> [bus_id]  Create a dynamic SPI bus\n");
    ::printf("  uart-create <tx> <rx> [bus_id] [rx_pages] [tx_pages]  Create a dynamic UART bus\n");
    ::printf("  uart-xfer <bus_id> <text> [rx_len]  UART send/receive test\n");
    ::printf("  i2s-create <bclk> <ws> <dout> <din> [bus_id] [role] [tx_kB] [rx_kB]  Create a dynamic I2S bus\n");
    ::printf("  spi-bytecode <bus_id> [mode] [freq] [cs] [tx_len]  SPI bytecode transfer test\n");
    ::printf("  large [size]              SPI large transfer test (default: 153600 bytes)\n");
    ::printf("  spi-repeat [bytes] [rounds]  SPI repeat test (default: 64 bytes x 200 rounds)\n");
    ::printf("  help                      Show this help\n");
    ::printf("  quit / exit               Leave REPL mode\n");
}

// Tokenize `line` (in place) into argv-style tokens split on whitespace.
// Returns the token count; tokens[] point into `line`.
size_t tokenize(char* line, char** tokens, size_t max_tokens)
{
    size_t n = 0;
    char* p  = line;
    while (*p != '\0' && n < max_tokens) {
        while (*p == ' ' || *p == '\t') {
            ++p;
        }
        if (*p == '\0') {
            break;
        }
        tokens[n++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') {
            ++p;
        }
        if (*p != '\0') {
            *p++ = '\0';
        }
    }
    return n;
}

// Dispatch a single command line. Returns false to request quit, true to
// continue. `port_buf` is needed for reset-wait reconnect; `prog` for help.
bool dispatchCommand(MuxHostEndpoint& host, const char* prog, const char* port_buf, const char* line)
{
    char buf[1024];
    ::snprintf(buf, sizeof(buf), "%s", line);
    char* tok[16];
    size_t ntok = tokenize(buf, tok, sizeof(tok) / sizeof(tok[0]));
    if (ntok == 0) {
        return true;
    }
    const char* cmd = tok[0];

    auto argInt = [&](size_t idx, int def) -> int {
        return idx < ntok ? static_cast<int>(::strtol(tok[idx], nullptr, 0)) : def;
    };
    auto argSize = [&](size_t idx, size_t def) -> size_t {
        return idx < ntok ? static_cast<size_t>(::strtoul(tok[idx], nullptr, 0)) : def;
    };

    if (::strcmp(cmd, "quit") == 0 || ::strcmp(cmd, "exit") == 0) {
        return false;
    }
    if (::strcmp(cmd, "help") == 0) {
        printUsage(prog);
    } else if (::strcmp(cmd, "caps") == 0) {
        printCaps(host);
    } else if (::strcmp(cmd, "ping") == 0) {
        testPing(host, argInt(1, 10));
    } else if (::strcmp(cmd, "reset") == 0) {
        return testReset(host, port_buf, false);
    } else if (::strcmp(cmd, "reset-wait") == 0) {
        return testReset(host, port_buf, true);
    } else if (::strcmp(cmd, "i2c-create") == 0) {
        testI2cCreate(host, argInt(1, 0), argInt(2, 0), argInt(3, 0));
    } else if (::strcmp(cmd, "i2c-scan") == 0 || ::strcmp(cmd, "scan") == 0) {
        testI2cScan(host, argInt(1, 0));
    } else if (::strcmp(cmd, "regw") == 0) {
        testRegWrite(host, argInt(1, 0), argInt(2, 0), argInt(3, 0), argInt(4, 0));
    } else if (::strcmp(cmd, "reg") == 0) {
        testRegRead(host, argInt(1, 0), argInt(2, 0), argInt(3, 1), argInt(4, 0));
    } else if (::strcmp(cmd, "gpio-read") == 0) {
        testGpioRead(host, argInt(1, 0));
    } else if (::strcmp(cmd, "gpio-write") == 0) {
        int pin  = argInt(1, 0);
        bool val = argInt(2, 0) != 0;
        ::printf("\n--- GPIO Write ---\n");
        testGpioSetMode(host, pin, m5hal::types::gpio_mode_t::Output);
        int werr = testGpioWrite(host, pin, val);
        if (werr != 0) {
            // A denied pin (outside the device allowlist) surfaces here.
            ::printf("  gpio-write pin %d = %d error %d (denied?)\n", pin, val ? 1 : 0, werr);
        } else {
            testGpioRead(host, pin);
        }
    } else if (::strcmp(cmd, "gpio-scan") == 0) {
        ::printf("\n--- GPIO Scan (read all non-denied pins) ---\n");
        for (int p = 0; p < 40; ++p) {
            testGpioRead(host, p);
        }
    } else if (::strcmp(cmd, "port-read") == 0) {
        testPortRead(host, argInt(1, 0), argInt(2, 0));
    } else if (::strcmp(cmd, "port-write") == 0) {
        testPortWrite(host, argInt(1, 0), argInt(2, 0), static_cast<uint32_t>(argSize(3, 0)),
                      static_cast<uint32_t>(argSize(4, 0)));
        testPortRead(host, argInt(1, 0), argInt(2, 0));
    } else if (::strcmp(cmd, "spi-create") == 0) {
        testSpiCreate(host, argInt(1, 0), argInt(2, 0), argInt(3, 0), argInt(4, 3));
    } else if (::strcmp(cmd, "uart-create") == 0) {
        testUartCreate(host, argInt(1, 0), argInt(2, 0), argInt(3, 0), argInt(4, 8), argInt(5, 2));
    } else if (::strcmp(cmd, "uart-xfer") == 0) {
        testUartXfer(host, argInt(1, 0), ntok > 2 ? tok[2] : "HELLO", argSize(3, 0));
    } else if (::strcmp(cmd, "i2s-create") == 0) {
        testI2sCreate(host, argInt(1, -1), argInt(2, -1), argInt(3, -1), argInt(4, -1), argInt(5, 0), argInt(6, 0),
                      argInt(7, 8), argInt(8, 8));
    } else if (::strcmp(cmd, "spi-bytecode") == 0) {
        testSpiBytecode(host, argInt(1, 0), argInt(2, 0), argInt(3, 1000000), argInt(4, -1), argSize(5, 8));
    } else if (::strcmp(cmd, "large") == 0) {
        testSpiTransfer(host, argSize(1, 153600));
    } else if (::strcmp(cmd, "spi-repeat") == 0) {
        testSpiRepeat(host, argSize(1, 64), argSize(2, 200));
    } else {
        ::printf("unknown command: %s (try 'help')\n", cmd);
    }
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ::setvbuf(stdout, nullptr, _IOLBF, 0);

    if (argc > 1 &&
        (::strcmp(argv[1], "help") == 0 || ::strcmp(argv[1], "--help") == 0 || ::strcmp(argv[1], "-h") == 0)) {
        printUsage(argv[0]);
        return 0;
    }

    const char* port    = argc > 1 ? argv[1] : nullptr;
    const uint32_t baud = argc > 2 ? static_cast<uint32_t>(::strtoul(argv[2], nullptr, 10)) : 3000000;
    char port_buf[128]  = {};

    MuxHostEndpoint host{baud};
    bool hello_done = false;

    if (port != nullptr) {
        ::snprintf(port_buf, sizeof(port_buf), "%s", port);
        if (!host.connect(port_buf)) {
            return 1;
        }
        ::printf("using %s @ %u baud\n", port_buf, static_cast<unsigned>(baud));
    } else {
        if (!autoDiscover(host, port_buf, sizeof(port_buf))) {
            ::fprintf(stderr, "no mux device found\n");
            return 1;
        }
        hello_done = true;  // autoDiscover already completed a hello exchange
        ::printf("using %s @ %u baud\n", port_buf, static_cast<unsigned>(baud));
    }

    // Hello (capture caps for the `caps` command and the proto banner).
    if (!hello_done) {
        auto hello_r = host.session->hello();
        if (!hello_r.has_value()) {
            ::fprintf(stderr, "hello failed: %d\n", static_cast<int>(hello_r.error()));
            return 1;
        }
    }
    {
        auto caps = host.session->lastResponse();
        if (caps.size >= 1) {
            host.proto_ver = caps.data[0];
        }
        if (caps.size >= 2) {
            host.caps_flags = caps.data[1];
        }
        if (caps.size >= 3) {
            host.bus_count = caps.data[2];
        }
    }
    ::printf("remote proto v%u, flags 0x%02X, %u bus(es)\n", static_cast<unsigned>(host.proto_ver),
             static_cast<unsigned>(host.caps_flags), static_cast<unsigned>(host.bus_count));

    // argv mode: argv[3..] joined into a single command line, run once.
    if (argc > 3) {
        char cmdline[1024] = {};
        size_t off         = 0;
        for (int i = 3; i < argc; ++i) {
            int w = ::snprintf(cmdline + off, sizeof(cmdline) - off, (i == 3) ? "%s" : " %s", argv[i]);
            if (w < 0 || static_cast<size_t>(w) >= sizeof(cmdline) - off) {
                break;
            }
            off += static_cast<size_t>(w);
        }
        if (!dispatchCommand(host, argv[0], port_buf, cmdline)) {
            return 1;
        }
        return 0;
    }

    // REPL mode: read commands line-by-line from stdin.
    char line[1024];
    while (::fgets(line, sizeof(line), stdin) != nullptr) {
        size_t len = ::strlen(line);
        if (len > 0 && line[len - 1] == '\n') {
            line[len - 1] = '\0';
        }
        if (line[0] == '\0') {
            continue;
        }
        if (!dispatchCommand(host, argv[0], port_buf, line)) {
            break;
        }
    }
    return 0;
}

#endif  // !ARDUINO
