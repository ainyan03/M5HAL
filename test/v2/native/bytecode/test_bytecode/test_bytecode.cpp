// SPDX-License-Identifier: MIT
// Native gtest for the bytecode encoder + runner (hal/v2/bytecode/).
//
// Mechanically verifies the wire contract of spec/design/bytecode.md:
// LenVar boundaries, encode -> run roundtrips for every core opcode
// against capture buses and a recording GPIO, store-slot semantics,
// forward compatibility (skippable unknown opcodes, critical-bit
// abort), truncation diagnostics, the symmetric response pipeline
// (writeResponse -> run on the host side), and streaming execution
// through a StreamSource with chunked arrival.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

using ::m5::hal::v2::result_t;

namespace {

namespace bytecode = ::m5::hal::v2::bytecode;
namespace bus      = ::m5::hal::v2::bus;
namespace data     = ::m5::hal::v2::data;
namespace error    = ::m5::hal::v2::error;
namespace gpio     = ::m5::hal::v2::gpio;
namespace i2c      = ::m5::hal::v2::i2c;
namespace spi      = ::m5::hal::v2::spi;
namespace types    = ::m5::hal::v2::types;
namespace uart     = ::m5::hal::v2::uart;
using error_t      = error::error_t;

// ---- capture fakes ----------------------------------------------------------

// Drain helper shared by the capture buses: pull everything from `tx`,
// then feed `rx_script` into `rx` until it stops accepting.
template <typename Vec>
result_t<size_t> drainAndFeed(data::Source* tx, data::Sink* rx, Vec& tx_log, const std::vector<uint8_t>& rx_script)
{
    size_t done = 0;
    while (tx != nullptr && !tx->eof()) {
        auto span = tx->peek(64);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span->size == 0) {
            break;
        }
        tx_log.insert(tx_log.end(), span->data, span->data + span->size);
        auto advanced = tx->advance(span->size);
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        done += span->size;
    }
    size_t fed = 0;
    while (rx != nullptr && !rx->closed() && fed < rx_script.size()) {
        auto span = rx->reserve(rx_script.size() - fed);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span->size == 0) {
            break;
        }
        std::memcpy(span->data, rx_script.data() + fed, span->size);
        auto committed = rx->commit(span->size);
        if (!committed.has_value()) {
            return m5::stl::make_unexpected(committed.error());
        }
        fed += span->size;
        done += span->size;
    }
    return done;
}

class CaptureI2cBus : public i2c::IBus {
public:
    result_t<void> init(const i2c::IBusConfig&)
    {
        return {};
    }
    result_t<void> release(void) override
    {
        return {};
    }
    result_t<void> transfer(bus::IAccessor*, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc,
                            data::Source* tx, size_t, data::Sink* rx, size_t) override
    {
        ++transfer_count;
        last_addr = cfg.i2c_addr;
        last_freq = cfg.freq;
        prefix.assign(desc.prefix, desc.prefix + desc.prefix_len);
        auto tx_before = tx_bytes.size();
        auto drained   = drainAndFeed(tx, rx, tx_bytes, rx_script);
        if (!drained.has_value()) {
            return m5::stl::make_unexpected(drained.error());
        }
        last_totals.tx = tx_bytes.size() - tx_before;
        last_totals.rx = *drained - last_totals.tx;
        return {};
    }
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor*, const i2c::MasterAccessConfig&) override
    {
        auto totals = last_totals;
        last_totals.clear();
        if (rx_report_override != 0) {
            totals.rx = rx_report_override;  // model a backend over-reporting rx
        }
        return totals;
    }

    size_t transfer_count     = 0;
    uint16_t last_addr        = 0;
    uint32_t last_freq        = 0;
    size_t rx_report_override = 0;
    std::vector<uint8_t> prefix;
    std::vector<uint8_t> tx_bytes;
    std::vector<uint8_t> rx_script;
    bus::TransferTotals last_totals{};
};

class CaptureSpiBus : public spi::IBus {
public:
    result_t<void> init(const spi::IBusConfig&)
    {
        return {};
    }
    result_t<void> release(void) override
    {
        return {};
    }
    result_t<void> transfer(bus::IAccessor*, const spi::MasterAccessConfig&, const spi::TransferDesc& desc,
                            data::Source* tx, size_t, data::Sink* rx, size_t) override
    {
        ++transfer_count;
        last_desc      = desc;
        auto tx_before = tx_bytes.size();
        auto drained   = drainAndFeed(tx, rx, tx_bytes, rx_script);
        if (!drained.has_value()) {
            return m5::stl::make_unexpected(drained.error());
        }
        last_totals.tx = tx_bytes.size() - tx_before;
        last_totals.rx = *drained - last_totals.tx;
        return {};
    }
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor*, const spi::MasterAccessConfig&) override
    {
        auto totals = last_totals;
        last_totals.clear();
        return totals;
    }

    size_t transfer_count = 0;
    spi::TransferDesc last_desc{};
    std::vector<uint8_t> tx_bytes;
    std::vector<uint8_t> rx_script;
    bus::TransferTotals last_totals{};
};

class CaptureUartBus : public uart::IBus {
public:
    result_t<size_t> write(bus::IAccessor*, const uart::AccessConfig&, data::Source* tx, size_t len) override
    {
        const size_t limit = std::min(len, max_write_per_call);
        size_t done        = 0;
        while (done < limit && tx != nullptr && !tx->eof()) {
            auto span = tx->peek(limit - done);
            if (!span.has_value()) {
                return m5::stl::make_unexpected(span.error());
            }
            if (span->size == 0) {
                break;
            }
            const size_t n = std::min(span->size, limit - done);
            tx_bytes.insert(tx_bytes.end(), span->data, span->data + n);
            auto advanced = tx->advance(n);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
            done += n;
        }
        return done;
    }
    result_t<size_t> read(bus::IAccessor*, const uart::AccessConfig&, data::Sink* rx, size_t len) override
    {
        std::vector<uint8_t> ignored;
        std::vector<uint8_t> chunk{rx_script.begin(),
                                   rx_script.begin() + static_cast<ptrdiff_t>(std::min(len, rx_script.size()))};
        return drainAndFeed(nullptr, rx, ignored, chunk);
    }
    result_t<size_t> readableBytes(bus::IAccessor*, const uart::AccessConfig&) override
    {
        return rx_script.size();
    }

    std::vector<uint8_t> tx_bytes;
    std::vector<uint8_t> rx_script;
    size_t max_write_per_call = static_cast<size_t>(-1);
};

// Stream-bus fake: accepts at most `accept_limit` bytes per write (the
// rest is "dropped" so the runner sees a short accept), logs config, and
// reports a programmable writableBytes.
;

// Recording GPIO port: scripted read levels, logged writes / modes.
class RecordingPort : public gpio::IPort {
public:
    std::array<bool, 8> levels{};
    std::vector<std::pair<uint8_t, bool>> writes;
    std::vector<std::pair<uint8_t, uint8_t>> modes;

protected:
    void _writePinEncoded(uint32_t num, bool v) override
    {
        writes.push_back({static_cast<uint8_t>(num), v});
    }
    bool _readPinEncoded(uint32_t num) override
    {
        return levels[num % levels.size()];
    }
    void _setPinModeEncoded(uint32_t num, types::gpio_mode_t mode) override
    {
        modes.push_back({static_cast<uint8_t>(num), static_cast<uint8_t>(mode)});
    }
    types::gpio_local_pin_t _toLocalPin(uint32_t num) const override
    {
        return static_cast<types::gpio_local_pin_t>(num);
    }
    uint32_t _fromLocalPin(types::gpio_local_pin_t pin) const override
    {
        return pin;
    }
};

struct RecordingGPIO : public gpio::IGPIO {
    gpio::IPort* portForPin(types::gpio_local_pin_t) const override
    {
        return &port;
    }
    gpio::IPort* getPort(uint8_t) const override
    {
        return &port;
    }
    uint16_t getPinCount() const override
    {
        return 8;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }
    mutable RecordingPort port;
};

// Chunked stream feeder (same shape as test_stream_adapter, plus a
// per-read cap so the runner's grow-the-peek loop is exercised).
class FakeStreamReader : public data::StreamReader {
public:
    void feed(const uint8_t* bytes, size_t len)
    {
        _pending.insert(_pending.end(), bytes, bytes + len);
    }
    result_t<size_t> read(data::DataSpan dst) override
    {
        ++read_calls;
        const size_t n = std::min({dst.size, _pending.size(), max_chunk});
        std::memcpy(dst.data, _pending.data(), n);
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<ptrdiff_t>(n));
        return n;
    }
    result_t<size_t> readableBytes(void) override
    {
        return std::min(_pending.size(), max_chunk);
    }

    size_t max_chunk  = static_cast<size_t>(-1);
    size_t read_calls = 0;  // each call models one (possibly timeout-long) blocking read

private:
    std::vector<uint8_t> _pending;
};

uint32_t g_delay_total = 0;
void countDelay(uint32_t ms)
{
    g_delay_total += ms;
}

// ============================================================================
// LenVar
// ============================================================================

TEST(BytecodeLenVar, RoundtripAcrossBoundaries)
{
    const size_t values[] = {0, 1, 0xFC, 0xFD, 0x1234, 0xFFFF, 0x10000, 0xABCDEF};
    for (size_t v : values) {
        uint8_t buf[5] = {};
        const size_t n = bytecode::encodeLenVar(buf, v);
        EXPECT_EQ(n, bytecode::lenVarSize(v)) << v;
        auto decoded = bytecode::decodeLenVar({buf, n});
        EXPECT_TRUE(decoded.valid) << v;
        EXPECT_EQ(decoded.consumed, n) << v;
        EXPECT_EQ(decoded.value, v) << v;
    }
}

TEST(BytecodeLenVar, ShortInputAndReservedMarker)
{
    uint8_t wide[5] = {};
    (void)bytecode::encodeLenVar(wide, 0x1234);  // 3-byte form
    auto partial = bytecode::decodeLenVar({wide, 2});
    EXPECT_TRUE(partial.valid);
    EXPECT_EQ(partial.consumed, 0u);  // needs more bytes

    const uint8_t reserved[] = {0xFF};
    auto bad                 = bytecode::decodeLenVar({reserved, 1});
    EXPECT_FALSE(bad.valid);
}

// ============================================================================
// Encoder + Runner roundtrips
// ============================================================================

struct Rig {
    CaptureI2cBus i2c_bus;
    CaptureSpiBus spi_bus;
    CaptureUartBus uart_bus;
    RecordingGPIO gpio_dev;
    gpio::GPIOGroup gpio_group{&gpio_dev};

    i2c::MasterAccessConfig i2c_cfg{};
    spi::MasterAccessConfig spi_cfg{};
    uart::AccessConfig uart_cfg{};
    i2c::MasterAccessor i2c_acc{i2c_bus, i2c_cfg};
    spi::MasterAccessor spi_acc{spi_bus, spi_cfg};
    uart::Accessor uart_acc{uart_bus, uart_cfg};

    bytecode::BytecodeRunner runner;

    Rig()
    {
        EXPECT_TRUE(runner.registerI2C(0, i2c_acc).has_value());
        EXPECT_TRUE(runner.registerSPI(0, spi_acc).has_value());
        EXPECT_TRUE(runner.registerUART(0, uart_acc).has_value());
        runner.setGPIOGroup(gpio_group);
        runner.setDelayFn(&countDelay);
    }
};

TEST(BytecodeRunner, DelayDispatchesToInjectedFn)
{
    Rig rig;
    uint8_t buf[32] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.delayMs(12).has_value());
    ASSERT_TRUE(enc.delayMs(30).has_value());
    ASSERT_TRUE(enc.end().has_value());

    g_delay_total = 0;
    auto consumed = rig.runner.run(data::ConstDataSpan{buf, sizeof(buf)});
    ASSERT_TRUE(consumed.has_value());
    EXPECT_EQ(g_delay_total, 42u);
}

TEST(BytecodeRunner, ConfigureReachesAccessor)
{
    Rig rig;
    i2c::MasterAccessConfig cfg;
    cfg.freq             = 400000;
    cfg.i2c_addr         = 0x42;
    cfg.address_is_10bit = true;
    cfg.use_restart      = false;

    uint8_t buf[64] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.configure(0, cfg).has_value());
    ASSERT_TRUE(enc.end().has_value());

    ASSERT_TRUE(rig.runner.run(data::ConstDataSpan{buf, sizeof(buf)}).has_value());
    const auto& applied = static_cast<const i2c::MasterAccessConfig&>(rig.i2c_acc.getConfig());
    EXPECT_EQ(applied.freq, 400000u);
    EXPECT_EQ(applied.i2c_addr, 0x42);
    EXPECT_TRUE(applied.address_is_10bit);
    EXPECT_FALSE(applied.use_restart);
}

TEST(BytecodeRunner, I2CTransferRoundtrip)
{
    Rig rig;
    rig.i2c_bus.rx_script = {0xC0, 0xC1, 0xC2};

    i2c::TransferDesc desc{uint8_t{0xD0}};  // 1-byte register prefix
    const uint8_t tx[] = {0x10, 0x20};

    uint8_t buf[64] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.transfer(0, desc, {tx, sizeof(tx)}, 3, 7).has_value());
    ASSERT_TRUE(enc.end().has_value());

    ASSERT_TRUE(rig.runner.run(data::ConstDataSpan{buf, sizeof(buf)}).has_value());
    EXPECT_EQ(rig.i2c_bus.transfer_count, 1u);
    ASSERT_EQ(rig.i2c_bus.prefix.size(), 1u);
    EXPECT_EQ(rig.i2c_bus.prefix[0], 0xD0);
    ASSERT_EQ(rig.i2c_bus.tx_bytes.size(), sizeof(tx));
    EXPECT_EQ(std::memcmp(rig.i2c_bus.tx_bytes.data(), tx, sizeof(tx)), 0);

    auto stored = rig.runner.storedData(7);
    ASSERT_EQ(stored.size, 3u);
    EXPECT_EQ(stored.data[0], 0xC0);
    EXPECT_EQ(stored.data[2], 0xC2);
}

TEST(BytecodeRunner, TransferClampsOverReportedRxToAllocatedStore)
{
    // Regression anchor: opBusTransfer recorded the backend's rx claim
    // verbatim as the store-slot length; an over-reporting backend made a
    // later StoreData read past the slot's allocation.
    Rig rig;
    rig.i2c_bus.rx_script          = {0xC0, 0xC1, 0xC2};
    rig.i2c_bus.rx_report_override = 100;  // way past the 3-byte allocation

    i2c::TransferDesc desc{uint8_t{0xD0}};
    uint8_t buf[64] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.transfer(0, desc, data::ConstDataSpan{}, 3, 7).has_value());
    ASSERT_TRUE(enc.end().has_value());

    ASSERT_TRUE(rig.runner.run(data::ConstDataSpan{buf, sizeof(buf)}).has_value());
    EXPECT_EQ(rig.runner.storedData(7).size, 3u);  // clamped to the allocation
}

TEST(BytecodeRunner, SPITransferCarriesDescriptor)
{
    Rig rig;
    rig.spi_bus.rx_script = {0xAA};

    spi::TransferDesc desc;
    desc.command        = 0x0B;
    desc.command_bytes  = 1;
    desc.address        = 0x123456;
    desc.address_bytes  = 3;
    desc.dummy_cycles   = 8;
    desc.dc_level_valid = true;
    desc.dc_level       = false;

    uint8_t buf[64] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.transfer(0, desc, data::ConstDataSpan{}, 1, 2).has_value());
    ASSERT_TRUE(enc.end().has_value());

    ASSERT_TRUE(rig.runner.run(data::ConstDataSpan{buf, sizeof(buf)}).has_value());
    EXPECT_EQ(rig.spi_bus.transfer_count, 1u);
    EXPECT_EQ(rig.spi_bus.last_desc.command, 0x0Bu);
    EXPECT_EQ(rig.spi_bus.last_desc.command_bytes, 1);
    EXPECT_EQ(rig.spi_bus.last_desc.address, 0x123456u);
    EXPECT_EQ(rig.spi_bus.last_desc.address_bytes, 3);
    EXPECT_EQ(rig.spi_bus.last_desc.dummy_cycles, 8);
    EXPECT_TRUE(rig.spi_bus.last_desc.dc_level_valid);
    EXPECT_FALSE(rig.spi_bus.last_desc.dc_level);
    EXPECT_EQ(rig.runner.storedData(2).size, 1u);
}

TEST(BytecodeRunner, UARTTransferTracksActualReadCount)
{
    Rig rig;
    rig.uart_bus.rx_script = {0x01, 0x02};  // fewer than requested

    const uint8_t tx[] = {0x55, 0x66, 0x77};
    uint8_t buf[64]    = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.uartTransfer(0, {tx, sizeof(tx)}, 8, 4).has_value());
    ASSERT_TRUE(enc.end().has_value());

    ASSERT_TRUE(rig.runner.run(data::ConstDataSpan{buf, sizeof(buf)}).has_value());
    ASSERT_EQ(rig.uart_bus.tx_bytes.size(), sizeof(tx));
    EXPECT_EQ(std::memcmp(rig.uart_bus.tx_bytes.data(), tx, sizeof(tx)), 0);
    EXPECT_EQ(rig.runner.storedData(4).size, 2u);  // short read keeps the actual count
}

TEST(BytecodeRunner, UARTStreamTransferChunkReportsShortWriteForCarry)
{
    Rig rig;
    rig.uart_bus.max_write_per_call = 3;

    const uint8_t tx[] = {0x10, 0x11, 0x12, 0x13, 0x14};
    size_t actual_tx   = 0;
    size_t actual_rx   = 0;
    auto first = rig.runner.streamTransferChunk(types::bus_kind_t::UART, 0, data::ConstDataSpan{}, {tx, sizeof(tx)},
                                                data::DataSpan{}, actual_tx, actual_rx);
    ASSERT_TRUE(first.has_value()) << "err=" << error::toString(first.error());
    EXPECT_EQ(actual_tx, 3u);
    EXPECT_EQ(actual_rx, 0u);
    ASSERT_EQ(rig.uart_bus.tx_bytes.size(), 3u);
    EXPECT_EQ(std::memcmp(rig.uart_bus.tx_bytes.data(), tx, 3), 0);

    auto second = rig.runner.streamTransferChunk(types::bus_kind_t::UART, 0, data::ConstDataSpan{}, {tx + actual_tx, 2},
                                                 data::DataSpan{}, actual_tx, actual_rx);
    ASSERT_TRUE(second.has_value()) << "err=" << error::toString(second.error());
    EXPECT_EQ(actual_tx, 2u);
    EXPECT_EQ(actual_rx, 0u);
    ASSERT_EQ(rig.uart_bus.tx_bytes.size(), sizeof(tx));
    EXPECT_EQ(std::memcmp(rig.uart_bus.tx_bytes.data(), tx, sizeof(tx)), 0);
}

// ============================================================================
// Stream credit (I2S)
// ============================================================================

TEST(BytecodeRunner, GpioOpsReachThePins)
{
    Rig rig;
    rig.gpio_dev.port.levels = {true, false, true, false, false, false, false, false};

    const types::gpio_number_t write_pins[] = {1, 3};
    const types::gpio_number_t mode_pins[]  = {2};
    const types::gpio_number_t read_pins[]  = {0, 1, 2};

    uint8_t buf[64] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.gpioSetMode(types::gpio_mode_t::Output, mode_pins, 1).has_value());
    ASSERT_TRUE(enc.gpioWriteHigh(write_pins, 2).has_value());
    ASSERT_TRUE(enc.gpioWriteLow(write_pins, 1).has_value());
    ASSERT_TRUE(enc.gpioRead(5, read_pins, 3).has_value());
    ASSERT_TRUE(enc.end().has_value());

    ASSERT_TRUE(rig.runner.run(data::ConstDataSpan{buf, sizeof(buf)}).has_value());

    ASSERT_EQ(rig.gpio_dev.port.modes.size(), 1u);
    EXPECT_EQ(rig.gpio_dev.port.modes[0].first, 2);
    ASSERT_EQ(rig.gpio_dev.port.writes.size(), 3u);
    EXPECT_EQ(rig.gpio_dev.port.writes[0], (std::pair<uint8_t, bool>{1, true}));
    EXPECT_EQ(rig.gpio_dev.port.writes[1], (std::pair<uint8_t, bool>{3, true}));
    EXPECT_EQ(rig.gpio_dev.port.writes[2], (std::pair<uint8_t, bool>{1, false}));

    auto stored = rig.runner.storedData(5);
    ASSERT_EQ(stored.size, 1u);
    EXPECT_EQ(stored.data[0], 0b101);  // pins 0,2 high, pin 1 low (LSB first)
}

TEST(BytecodeRunner, GpioOpsWithoutGroupIsInvalidState)
{
    bytecode::BytecodeRunner runner;
    const types::gpio_number_t pin = 0;

    uint8_t buf[32] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.gpioWriteHigh(&pin, 1).has_value());
    ASSERT_TRUE(enc.end().has_value());

    auto r = runner.run(data::ConstDataSpan{buf, sink.written()});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::INVALID_STATE);
}

// ============================================================================
// Store slots
// ============================================================================

TEST(BytecodeRunner, StoreSlotsOverwriteAndExhaust)
{
    Rig rig;
    uint8_t buf[256] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};
    const uint8_t a[] = {0x01};
    const uint8_t b[] = {0x02, 0x03};
    ASSERT_TRUE(enc.storeData(9, {a, sizeof(a)}).has_value());
    ASSERT_TRUE(enc.storeData(9, {b, sizeof(b)}).has_value());  // same id: overwrite
    ASSERT_TRUE(enc.end().has_value());

    ASSERT_TRUE(rig.runner.run(data::ConstDataSpan{buf, sizeof(buf)}).has_value());
    EXPECT_EQ(rig.runner.storedCount(), 1u);
    auto stored = rig.runner.storedData(9);
    ASSERT_EQ(stored.size, 2u);
    EXPECT_EQ(stored.data[1], 0x03);

    // 9 distinct ids exceed the 8 slots.
    data::MemorySink sink2{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc2{sink2};
    for (uint8_t id = 0; id < 9; ++id) {
        ASSERT_TRUE(enc2.storeData(id, {a, sizeof(a)}).has_value());
    }
    ASSERT_TRUE(enc2.end().has_value());
    auto result = rig.runner.run(data::ConstDataSpan{buf, sizeof(buf)});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error_t::OUT_OF_RESOURCE);
}

// ============================================================================
// Forward compatibility / diagnostics
// ============================================================================

TEST(BytecodeRunner, UnknownOpcodeSkipsOrAborts)
{
    Rig rig;
    g_delay_total = 0;
    // [size 3][opcode 0x30 unknown, non-critical][2 payload bytes],
    // then a known delay instruction.
    const uint8_t script[] = {3, 0x30, 0xAA, 0xBB, 5, 0x01, 7, 0, 0, 0, 0};
    auto consumed          = rig.runner.run(data::ConstDataSpan{script, sizeof(script)});
    ASSERT_TRUE(consumed.has_value());
    EXPECT_EQ(rig.runner.unknownSkipped(), 1u);
    EXPECT_EQ(g_delay_total, 7u);

    // Critical bit set -> abort with PROTOCOL_ERROR at that offset.
    const uint8_t critical[] = {5, 0x01, 1, 0, 0, 0, 3, 0x90, 0xAA, 0xBB};
    auto aborted             = rig.runner.run(data::ConstDataSpan{critical, sizeof(critical)});
    ASSERT_FALSE(aborted.has_value());
    EXPECT_EQ(aborted.error(), error_t::PROTOCOL_ERROR);
    EXPECT_EQ(rig.runner.lastOffset(), 6u);
}

TEST(BytecodeRunner, HostileLenVarDoesNotWrap)
{
    Rig rig;
    // u32 LenVar claiming a 0xFFFFFFFF-byte instruction. On 32-bit
    // targets `consumed + value` would wrap without the overflow guard
    // (PROTOCOL_ERROR); on 64-bit hosts the size simply can never be
    // satisfied (BUFFER_UNDERFLOW). Either way: an error, no crash, no
    // out-of-bounds read.
    const uint8_t hostile[] = {0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xAA, 0xBB};
    auto result             = rig.runner.run(data::ConstDataSpan{hostile, sizeof(hostile)});
    ASSERT_FALSE(result.has_value());
    EXPECT_TRUE(result.error() == error_t::PROTOCOL_ERROR || result.error() == error_t::BUFFER_UNDERFLOW);
    EXPECT_EQ(rig.runner.lastOffset(), 0u);

    // Reserved LenVar marker (0xFF) is rejected outright.
    const uint8_t reserved[] = {0xFF, 0x01};
    auto rejected            = rig.runner.run(data::ConstDataSpan{reserved, sizeof(reserved)});
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error(), error_t::PROTOCOL_ERROR);
}

TEST(BytecodeEncoder, RejectsPayloadSizeOverflowBeforeReserve)
{
    uint8_t buf[32] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};

    uint8_t byte = 0;
    auto uart_overflow =
        enc.uartTransfer(0, data::ConstDataSpan{&byte, static_cast<size_t>(-1)}, 0, bytecode::kDiscardStoreId);
    ASSERT_FALSE(uart_overflow.has_value());
    EXPECT_EQ(uart_overflow.error(), error_t::INVALID_ARGUMENT);

    auto spi_overflow = enc.transfer(0, spi::TransferDesc{}, data::ConstDataSpan{&byte, static_cast<size_t>(-1)}, 0,
                                     bytecode::kDiscardStoreId);
    ASSERT_FALSE(spi_overflow.has_value());
    EXPECT_EQ(spi_overflow.error(), error_t::INVALID_ARGUMENT);

    auto i2c_overflow = enc.transfer(0, i2c::TransferDesc{}, data::ConstDataSpan{&byte, static_cast<size_t>(-1)}, 0,
                                     bytecode::kDiscardStoreId);
    ASSERT_FALSE(i2c_overflow.has_value());
    EXPECT_EQ(i2c_overflow.error(), error_t::INVALID_ARGUMENT);

    const types::gpio_number_t pin = 0;
    const size_t too_many_pins     = (static_cast<size_t>(-1) / 2) + 1;
    auto gpio_overflow             = enc.gpioWriteHigh(&pin, too_many_pins);
    ASSERT_FALSE(gpio_overflow.has_value());
    EXPECT_EQ(gpio_overflow.error(), error_t::INVALID_ARGUMENT);
}

// streamTransfer writes meta.size into a single u8 wire field. A meta span
// larger than 255 (or an incoherent null-with-size span) must be rejected up
// front rather than silently truncated into a malformed instruction.
TEST(BytecodeEncoder, StreamTransferRejectsOversizedOrIncoherentMeta)
{
    uint8_t buf[512] = {};
    data::MemorySink sink{data::DataSpan{buf, sizeof(buf)}};
    bytecode::BytecodeEncoder enc{sink};

    std::array<uint8_t, 256> meta{};
    auto oversized =
        enc.streamTransfer(types::bus_kind_t::I2C, 0, 0, 0, 0, data::ConstDataSpan{meta.data(), meta.size()});
    ASSERT_FALSE(oversized.has_value());
    EXPECT_EQ(oversized.error(), error_t::INVALID_ARGUMENT);

    auto null_with_size = enc.streamTransfer(types::bus_kind_t::I2C, 0, 0, 0, 0, data::ConstDataSpan{nullptr, 4});
    ASSERT_FALSE(null_with_size.has_value());
    EXPECT_EQ(null_with_size.error(), error_t::INVALID_ARGUMENT);

    // The 255-byte boundary is still accepted.
    auto ok = enc.streamTransfer(types::bus_kind_t::I2C, 0, 0, 0, 0, data::ConstDataSpan{meta.data(), 255});
    ASSERT_TRUE(ok.has_value()) << "err=" << error::toString(ok.error());
}

TEST(BytecodeRunner, TruncationAndTerminator)
{
    Rig rig;
    // Instruction claims 5 bytes of body but the script ends early.
    const uint8_t truncated[] = {5, 0x01, 1, 0};
    auto result               = rig.runner.run(data::ConstDataSpan{truncated, sizeof(truncated)});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error_t::BUFFER_UNDERFLOW);

    // Terminator stops execution before trailing garbage.
    g_delay_total          = 0;
    const uint8_t script[] = {5, 0x01, 3, 0, 0, 0, 0x00, 0xFF, 0xFF};
    auto consumed          = rig.runner.run(data::ConstDataSpan{script, sizeof(script)});
    ASSERT_TRUE(consumed.has_value());
    EXPECT_EQ(consumed.value(), 7u);  // delay (6) + terminator (1)
    EXPECT_EQ(g_delay_total, 3u);

    // An empty script is a clean no-op.
    auto empty = rig.runner.run(data::ConstDataSpan{});
    ASSERT_TRUE(empty.has_value());
    EXPECT_EQ(empty.value(), 0u);
}

// ============================================================================
// Symmetric response pipeline
// ============================================================================

TEST(BytecodeRunner, ResponseRoundtripDeviceToHost)
{
    Rig device;
    device.i2c_bus.rx_script = {0x11, 0x22};

    // Device executes a command script with one read.
    uint8_t cmd[64] = {};
    data::MemorySink cmd_sink{data::DataSpan{cmd, sizeof(cmd)}};
    bytecode::BytecodeEncoder enc{cmd_sink};
    ASSERT_TRUE(enc.transfer(0, i2c::TransferDesc{uint8_t{0x04}}, data::ConstDataSpan{}, 2, 42).has_value());
    ASSERT_TRUE(enc.end().has_value());
    ASSERT_TRUE(device.runner.run(data::ConstDataSpan{cmd, sizeof(cmd)}).has_value());

    // Device encodes its slots as a response script.
    uint8_t resp[64] = {};
    data::MemorySink resp_sink{data::DataSpan{resp, sizeof(resp)}};
    ASSERT_TRUE(device.runner.writeResponse(resp_sink, error_t::OK).has_value());

    // Host executes the response: slots fill, status is recorded.
    bytecode::BytecodeRunner host;
    ASSERT_TRUE(host.run(data::ConstDataSpan{resp, sizeof(resp)}).has_value());
    auto stored = host.storedData(42);
    ASSERT_EQ(stored.size, 2u);
    EXPECT_EQ(stored.data[0], 0x11);
    EXPECT_EQ(stored.data[1], 0x22);
    EXPECT_TRUE(host.statusReported());
    EXPECT_EQ(host.reportedStatus(), error_t::OK);
}

TEST(BytecodeRunner, ReportErrorCarriesOffset)
{
    uint8_t resp[32] = {};
    data::MemorySink sink{data::DataSpan{resp, sizeof(resp)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.reportError(error_t::I2C_NO_ACK, 0x1234).has_value());
    ASSERT_TRUE(enc.end().has_value());

    bytecode::BytecodeRunner host;
    ASSERT_TRUE(host.run(data::ConstDataSpan{resp, sizeof(resp)}).has_value());
    EXPECT_TRUE(host.statusReported());
    EXPECT_EQ(host.reportedStatus(), error_t::I2C_NO_ACK);
    EXPECT_EQ(host.reportedOffset(), 0x1234u);
}

// ============================================================================
// Streaming execution (StreamSource)
// ============================================================================

TEST(BytecodeRunner, BufferedScriptRunsWithoutExtraBlockingReads)
{
    // Latency regression guard: a fully buffered script must execute
    // without further blocking reads (each read models a potential
    // full-timeout wait on a real stream). The 1-byte LenVar peek keeps
    // already-arrived instructions executing immediately.
    Rig rig;
    uint8_t script[32] = {};
    data::MemorySink sink{data::DataSpan{script, sizeof(script)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.delayMs(1).has_value());
    ASSERT_TRUE(enc.delayMs(2).has_value());
    ASSERT_TRUE(enc.end().has_value());
    const size_t script_len = 6 + 6 + 1;

    FakeStreamReader stream;
    stream.feed(script, script_len);
    uint8_t scratch[64] = {};
    data::StreamSource source{stream, data::DataSpan{scratch, sizeof(scratch)}};

    g_delay_total = 0;
    auto consumed = rig.runner.run(source);
    ASSERT_TRUE(consumed.has_value());
    EXPECT_EQ(consumed.value(), script_len);
    EXPECT_EQ(g_delay_total, 3u);
    EXPECT_EQ(stream.read_calls, 1u);  // one read filled the buffer; the rest ran from it
}

TEST(BytecodeRunner, RunsFromChunkedStream)
{
    Rig rig;
    rig.i2c_bus.rx_script = {0x77};

    uint8_t script[64] = {};
    data::MemorySink sink{data::DataSpan{script, sizeof(script)}};
    bytecode::BytecodeEncoder enc{sink};
    const uint8_t tx[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    ASSERT_TRUE(enc.delayMs(1).has_value());
    ASSERT_TRUE(enc.transfer(0, i2c::TransferDesc{}, {tx, sizeof(tx)}, 1, 3).has_value());
    ASSERT_TRUE(enc.end().has_value());
    // Recover the encoded length: find it via a MemorySink-independent
    // counter - encode again into a sized probe is overkill; instead use
    // the fact that the script is terminator-delimited and feed it all.

    FakeStreamReader stream;
    stream.max_chunk = 3;  // force the grow-the-peek loop
    stream.feed(script, sizeof(script));

    uint8_t scratch[64] = {};
    data::StreamSource source{stream, data::DataSpan{scratch, sizeof(scratch)}};

    g_delay_total = 0;
    auto consumed = rig.runner.run(source);
    ASSERT_TRUE(consumed.has_value());
    EXPECT_EQ(g_delay_total, 1u);
    EXPECT_EQ(rig.i2c_bus.transfer_count, 1u);
    ASSERT_EQ(rig.i2c_bus.tx_bytes.size(), sizeof(tx));
    EXPECT_EQ(std::memcmp(rig.i2c_bus.tx_bytes.data(), tx, sizeof(tx)), 0);
    EXPECT_EQ(rig.runner.storedData(3).size, 1u);
}

// ---- Symmetric trust boundary -----------------------------------------

TEST(BytecodeRunner, ReceiveOnlyRejectsExecutableOpcodes)
{
    // A receive-only runner (the host session's) must reject every
    // executable opcode: a peer's script fills data and reports status,
    // it never drives local buses, pins, or the clock.
    Rig rig;
    rig.runner.setReceiveOnly(true);

    uint8_t script[32] = {};
    data::MemorySink sink{data::DataSpan{script, sizeof(script)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.delayMs(0xFFFFFFFFu).has_value());  // 49-day stall if executed
    ASSERT_TRUE(enc.end().has_value());

    g_delay_total = 0;
    auto r        = rig.runner.run(data::ConstDataSpan{script, sink.written()});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::PROTOCOL_ERROR);
    EXPECT_EQ(g_delay_total, 0u);  // the delay handler never ran

    // Receive-side opcodes still execute.
    uint8_t resp[32] = {};
    data::MemorySink resp_sink{data::DataSpan{resp, sizeof(resp)}};
    bytecode::BytecodeEncoder resp_enc{resp_sink};
    const uint8_t payload[] = {0xAA};
    ASSERT_TRUE(resp_enc.storeData(7, {payload, sizeof(payload)}).has_value());
    ASSERT_TRUE(resp_enc.reportComplete(error_t::OK).has_value());
    ASSERT_TRUE(resp_enc.end().has_value());
    ASSERT_TRUE(rig.runner.run(data::ConstDataSpan{resp, resp_sink.written()}).has_value());
    EXPECT_EQ(rig.runner.storedData(7).size, 1u);
    EXPECT_TRUE(rig.runner.statusReported());
}

TEST(BytecodeRunner, RunEventPreservesRequestState)
{
    // An event script executed via runEvent must not clobber the stored
    // slots or report state of the previous response — even when it
    // (out of contract) carries store_data / report_* instructions.
    bytecode::BytecodeRunner host;

    uint8_t resp[32] = {};
    data::MemorySink resp_sink{data::DataSpan{resp, sizeof(resp)}};
    bytecode::BytecodeEncoder resp_enc{resp_sink};
    const uint8_t payload[] = {0x11, 0x22};
    ASSERT_TRUE(resp_enc.storeData(5, {payload, sizeof(payload)}).has_value());
    ASSERT_TRUE(resp_enc.reportComplete(error_t::OK).has_value());
    ASSERT_TRUE(resp_enc.end().has_value());
    ASSERT_TRUE(host.run(data::ConstDataSpan{resp, resp_sink.written()}).has_value());
    ASSERT_EQ(host.storedData(5).size, 2u);
    ASSERT_TRUE(host.statusReported());

    uint8_t evt[32] = {};
    data::MemorySink evt_sink{data::DataSpan{evt, sizeof(evt)}};
    bytecode::BytecodeEncoder evt_enc{evt_sink};
    const uint8_t junk[] = {0xEE};
    ASSERT_TRUE(evt_enc.storeData(5, {junk, sizeof(junk)}).has_value());
    ASSERT_TRUE(evt_enc.reportError(error_t::UNKNOWN_ERROR, 9).has_value());
    ASSERT_TRUE(evt_enc.end().has_value());
    ASSERT_TRUE(host.runEvent(data::ConstDataSpan{evt, evt_sink.written()}).has_value());

    // The request state survives untouched.
    auto stored = host.storedData(5);
    ASSERT_EQ(stored.size, 2u);
    EXPECT_EQ(stored.data[0], 0x11);
    EXPECT_TRUE(host.statusReported());
    EXPECT_EQ(host.reportedStatus(), error_t::OK);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
