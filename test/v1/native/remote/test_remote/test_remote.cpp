// SPDX-License-Identifier: MIT
//
// Remote bus mechanism (spec/design/remote.md) unit tests: message layer,
// Server policy, RemoteSession, and the RemoteI2CBus proxy, all over an
// in-memory loopback wire (no UART involved). The posix pty end-to-end
// version lives in test_posix_uart.cpp.

#include <M5HAL_v1.hpp>
#include <gtest/gtest.h>

#include <cstring>
#include <vector>

namespace {

namespace bus      = ::m5::hal::v1::bus;
namespace bytecode = ::m5::hal::v1::bytecode;
namespace data     = ::m5::hal::v1::data;
namespace error    = ::m5::hal::v1::error;
namespace frame    = ::m5::hal::v1::frame;
namespace i2c      = ::m5::hal::v1::i2c;
namespace remote   = ::m5::hal::v1::remote;
namespace service  = ::m5::hal::v1::service;
namespace types    = ::m5::hal::v1::types;

using error_t = error::error_t;

// ---- in-memory wire ---------------------------------------------------------

// Sink that appends committed bytes to a byte vector.
class VecSink : public data::Sink {
public:
    explicit VecSink(std::vector<uint8_t>& out) : _out{&out}
    {
    }
    m5::stl::expected<data::DataSpan, error_t> reserve(size_t max_len) override
    {
        return data::DataSpan{_stage, max_len < sizeof(_stage) ? max_len : sizeof(_stage)};
    }
    m5::stl::expected<void, error_t> commit(size_t N) override
    {
        _out->insert(_out->end(), _stage, _stage + N);
        return {};
    }
    bool closed() const override
    {
        return false;
    }

private:
    std::vector<uint8_t>* _out = nullptr;
    uint8_t _stage[frame::kMaxWireSize]{};
};

// Source over a growing byte vector. An empty backlog reports
// TIMEOUT_ERROR (a live but idle line), never EOF. The optional pump
// callback runs once at the start of each peek so the single-threaded
// tests can let "the other endpoint" make progress.
class VecSource : public data::Source {
public:
    using pump_fn_t = void (*)(void* ctx);

    explicit VecSource(std::vector<uint8_t>& in) : _in{&in}
    {
    }
    void setPump(pump_fn_t fn, void* ctx)
    {
        _pump     = fn;
        _pump_ctx = ctx;
    }

    m5::stl::expected<data::ConstDataSpan, error_t> peek(size_t max_len) override
    {
        if (_pump != nullptr) {
            _pump(_pump_ctx);
        }
        const size_t avail = _in->size() - _pos;
        if (avail == 0) {
            return m5::stl::make_unexpected(error_t::TIMEOUT_ERROR);
        }
        return data::ConstDataSpan{_in->data() + _pos, avail < max_len ? avail : max_len};
    }
    m5::stl::expected<void, error_t> advance(size_t N) override
    {
        const size_t avail = _in->size() - _pos;
        _pos += N < avail ? N : avail;
        return {};
    }
    bool eof() const override
    {
        return false;
    }

private:
    std::vector<uint8_t>* _in = nullptr;
    size_t _pos               = 0;
    pump_fn_t _pump           = nullptr;
    void* _pump_ctx           = nullptr;
};

// ---- device-side stub I2C bus -------------------------------------------------

// Captures the transfer (config, prefix, tx) and answers reads from a
// pattern, standing in for real hardware behind the server.
struct StubI2CBus : public i2c::I2CBus {
    i2c::I2CMasterAccessConfig last_cfg{};
    uint8_t last_prefix[i2c::TransferDesc::PREFIX_CAPACITY]{};
    size_t last_prefix_len = 0;
    std::vector<uint8_t> last_tx;
    size_t transfer_count = 0;
    uint8_t rx_pattern    = 0xA0;          // rx byte i = rx_pattern + i
    error_t result        = error_t::OK;   // forced outcome

    m5::stl::expected<void, error_t> init(const bus::BusConfig& config) override
    {
        if (config.getBusKind() != types::bus_kind_t::I2C) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
        _config = static_cast<const i2c::I2CBusConfig&>(config);
        return {};
    }

    m5::stl::expected<size_t, error_t> transfer(bus::Accessor*, const i2c::I2CMasterAccessConfig& cfg,
                                                const i2c::TransferDesc& desc, data::Source* tx,
                                                data::Sink* rx) override
    {
        ++transfer_count;
        last_cfg        = cfg;
        last_prefix_len = desc.prefix_len;
        ::memcpy(last_prefix, desc.prefix, sizeof(last_prefix));
        last_tx.clear();
        if (error::isError(result)) {
            return m5::stl::make_unexpected(result);
        }
        size_t total = desc.prefix_len;
        if (tx != nullptr) {
            while (!tx->eof()) {
                auto p = tx->peek(64);
                if (!p.has_value() || p.value().size == 0) {
                    break;
                }
                last_tx.insert(last_tx.end(), p.value().data, p.value().data + p.value().size);
                (void)tx->advance(p.value().size);
            }
            total += last_tx.size();
        }
        if (rx != nullptr) {
            auto rsv = rx->reserve(SIZE_MAX);
            if (!rsv.has_value()) {
                return m5::stl::make_unexpected(rsv.error());
            }
            for (size_t i = 0; i < rsv.value().size; ++i) {
                rsv.value().data[i] = static_cast<uint8_t>(rx_pattern + i);
            }
            (void)rx->commit(rsv.value().size);
            total += rsv.value().size;
        }
        return total;
    }
};

// ---- loopback fixture ----------------------------------------------------------

struct Loopback : public ::testing::Test {
    std::vector<uint8_t> to_server;
    std::vector<uint8_t> to_host;

    VecSink host_tx{to_server};
    VecSource host_rx{to_host};
    VecSink server_tx{to_host};
    VecSource server_rx{to_server};

    StubI2CBus stub_bus;
    i2c::I2CMasterAccessConfig stub_acc_cfg{};
    i2c::I2CMasterAccessor stub_acc{stub_bus, stub_acc_cfg};

    uint8_t server_scratch[remote::kMaxMessageSize]{};
    remote::Server server{data::DataSpan{server_scratch, sizeof(server_scratch)}};
    remote::RemoteServerService server_service{server, server_rx, server_tx};

    remote::RemoteSession session{host_rx, host_tx};

    bool pump_enabled = true;

    static void pumpThunk(void* ctx)
    {
        auto* self = static_cast<Loopback*>(ctx);
        if (self->pump_enabled) {
            self->server_service.service(service::ServiceContext{});
        }
    }

    void SetUp() override
    {
        ASSERT_TRUE(server.registerI2C(0, stub_acc).has_value());
        host_rx.setPump(&pumpThunk, this);
    }
};

// ---- tests ----------------------------------------------------------------------

TEST(RemoteErrorMapping, UnknownCodesFoldIntoRemoteFault)
{
    EXPECT_EQ(remote::mapRemoteError(static_cast<int8_t>(error_t::I2C_NO_ACK)), error_t::I2C_NO_ACK);
    EXPECT_EQ(remote::mapRemoteError(0), error_t::OK);
    EXPECT_EQ(remote::mapRemoteError(-100), error_t::REMOTE_FAULT);
    EXPECT_EQ(remote::mapRemoteError(42), error_t::REMOTE_FAULT);
}

TEST_F(Loopback, HelloExchangesCapabilities)
{
    auto caps = session.hello();
    ASSERT_TRUE(caps.has_value());
    EXPECT_EQ(caps.value().proto_ver, remote::kProtocolVersion);
    EXPECT_FALSE(caps.value().has_gpio);
    ASSERT_EQ(caps.value().bus_count, 1u);
    EXPECT_EQ(caps.value().buses[0].kind, types::bus_kind_t::I2C);
    EXPECT_EQ(caps.value().buses[0].bus_id, 0);
}

TEST_F(Loopback, PingPong)
{
    EXPECT_TRUE(session.ping().has_value());
}

TEST_F(Loopback, ProxyRegisterReadRoundtrip)
{
    remote::RemoteI2CBus proxy{session, 0};
    i2c::I2CMasterAccessConfig cfg;
    cfg.i2c_addr = 0x68;
    i2c::I2CMasterAccessor acc{proxy, cfg};

    uint8_t rx[4] = {};
    auto r        = acc.readRegister(uint8_t{0x75}, rx, sizeof(rx));
    ASSERT_TRUE(r.has_value());

    // The device-side stub saw the marshalled config and prefix...
    EXPECT_EQ(stub_bus.last_cfg.i2c_addr, 0x68);
    ASSERT_EQ(stub_bus.last_prefix_len, 1u);
    EXPECT_EQ(stub_bus.last_prefix[0], 0x75);
    // ...and its pattern came back through the response script.
    for (size_t i = 0; i < sizeof(rx); ++i) {
        EXPECT_EQ(rx[i], static_cast<uint8_t>(stub_bus.rx_pattern + i));
    }
}

TEST_F(Loopback, ProxyWriteCarriesTxBytes)
{
    remote::RemoteI2CBus proxy{session, 0};
    i2c::I2CMasterAccessConfig cfg;
    cfg.i2c_addr = 0x10;
    i2c::I2CMasterAccessor acc{proxy, cfg};

    const uint8_t tx[] = {0x01, 0x02, 0x03};
    auto r             = acc.write(tx, sizeof(tx));
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(stub_bus.last_tx.size(), sizeof(tx));
    EXPECT_EQ(::memcmp(stub_bus.last_tx.data(), tx, sizeof(tx)), 0);
}

TEST_F(Loopback, RemoteBusErrorFoldsIntoCallResult)
{
    stub_bus.result = error_t::I2C_NO_ACK;

    remote::RemoteI2CBus proxy{session, 0};
    i2c::I2CMasterAccessConfig cfg;
    i2c::I2CMasterAccessor acc{proxy, cfg};

    auto r = acc.probe();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::I2C_NO_ACK);
}

TEST_F(Loopback, UnregisteredBusReportsInvalidArgument)
{
    // bus 2 is a valid binding slot with nothing registered (a slot
    // outside kMaxBusBindings would be PROTOCOL_ERROR instead).
    remote::RemoteI2CBus proxy{session, 2};
    i2c::I2CMasterAccessConfig cfg;
    i2c::I2CMasterAccessor acc{proxy, cfg};

    auto r = acc.probe();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::INVALID_ARGUMENT);
}

TEST_F(Loopback, DelayBudgetIsRejectedBeforeExecution)
{
    uint8_t script[16] = {};
    data::MemorySink sink{data::DataSpan{script, sizeof(script)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.delayMs(60000).has_value());  // way over Config::max_delay_ms
    ASSERT_TRUE(enc.end().has_value());

    auto r = session.request(data::ConstDataSpan{script, sink.written()});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::INVALID_ARGUMENT);
    EXPECT_EQ(stub_bus.transfer_count, 0u);  // nothing executed
}

TEST_F(Loopback, NorespFailureArrivesAsPendingError)
{
    // A NORESP script addressing an unregistered bus fails server-side;
    // no immediate reply, but the next synchronous exchange delivers it.
    uint8_t script[32] = {};
    data::MemorySink sink{data::DataSpan{script, sizeof(script)}};
    bytecode::BytecodeEncoder enc{sink};
    i2c::I2CMasterAccessConfig cfg;
    ASSERT_TRUE(enc.configure(2, cfg).has_value());  // valid slot, nothing registered
    ASSERT_TRUE(enc.end().has_value());

    ASSERT_TRUE(session.requestNoResponse(data::ConstDataSpan{script, sink.written()}).has_value());
    EXPECT_EQ(session.lastRemoteError(), error_t::OK);  // not delivered yet

    ASSERT_TRUE(session.ping().has_value());
    EXPECT_EQ(session.lastRemoteError(), error_t::INVALID_ARGUMENT);
    EXPECT_FALSE(server.hasPendingError());
}

TEST_F(Loopback, OversizedReceiveIsRejectedLocally)
{
    remote::RemoteI2CBus proxy{session, 0};
    i2c::I2CMasterAccessConfig cfg;
    i2c::I2CMasterAccessor acc{proxy, cfg};

    uint8_t rx[remote::kMaxTransferRx + 1] = {};
    const size_t sent_before               = to_server.size();
    auto r                                 = acc.read(rx, sizeof(rx));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::INVALID_ARGUMENT);
    EXPECT_EQ(to_server.size(), sent_before);  // rejected before anything was sent
}

TEST_F(Loopback, MaxReceiveSizeRoundtrips)
{
    remote::RemoteI2CBus proxy{session, 0};
    i2c::I2CMasterAccessConfig cfg;
    i2c::I2CMasterAccessor acc{proxy, cfg};

    uint8_t rx[remote::kMaxTransferRx] = {};
    auto r                             = acc.read(rx, sizeof(rx));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(rx[0], stub_bus.rx_pattern);
    EXPECT_EQ(rx[sizeof(rx) - 1], static_cast<uint8_t>(stub_bus.rx_pattern + sizeof(rx) - 1));
}

TEST_F(Loopback, UnknownTypeAndReservedBitsAreDropped)
{
    // Hand-craft messages the server must ignore: reserved type, MORE bit.
    frame::FrameWriter host_writer{host_tx};
    const uint8_t unknown_kind[] = {0x0F, 0x00};
    const uint8_t more_bit[]     = {static_cast<uint8_t>(remote::MessageType::request) | remote::kTypeMoreBit, 0x01};
    ASSERT_TRUE(host_writer.writeData(remote::kDefaultStreamId, {unknown_kind, sizeof(unknown_kind)}).has_value());
    ASSERT_TRUE(host_writer.writeData(remote::kDefaultStreamId, {more_bit, sizeof(more_bit)}).has_value());

    server_service.service(service::ServiceContext{});
    EXPECT_EQ(server.droppedCount(), 2u);
    EXPECT_TRUE(to_host.empty());  // no replies were generated
}

TEST_F(Loopback, ForeignStreamIdIsIgnored)
{
    frame::FrameWriter host_writer{host_tx};
    const uint8_t msg[] = {static_cast<uint8_t>(remote::MessageType::ping), 0x09};
    ASSERT_TRUE(host_writer.writeData(0x42, {msg, sizeof(msg)}).has_value());

    server_service.service(service::ServiceContext{});
    EXPECT_EQ(server.droppedCount(), 0u);
    EXPECT_TRUE(to_host.empty());
}

// ---- Stage B: SPI / UART / GPIO proxies ------------------------------------------

namespace gpio  = ::m5::hal::v1::gpio;
namespace spi   = ::m5::hal::v1::spi;
namespace uart_ = ::m5::hal::v1::uart;

struct StubSPIBus : public spi::SPIBus {
    spi::SPIMasterAccessConfig last_cfg{};
    spi::TransferDesc last_desc{};
    std::vector<uint8_t> last_tx;
    uint8_t rx_pattern = 0xB0;

    m5::stl::expected<void, error_t> init(const bus::BusConfig& config) override
    {
        if (config.getBusKind() != types::bus_kind_t::SPI) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
        _config = static_cast<const spi::SPIBusConfig&>(config);
        return {};
    }
    m5::stl::expected<size_t, error_t> transfer(bus::Accessor*, const spi::SPIMasterAccessConfig& cfg,
                                                const spi::TransferDesc& desc, data::Source* tx,
                                                data::Sink* rx) override
    {
        last_cfg  = cfg;
        last_desc = desc;
        last_tx.clear();
        size_t total = 0;
        if (tx != nullptr) {
            while (!tx->eof()) {
                auto p = tx->peek(64);
                if (!p.has_value() || p.value().size == 0) {
                    break;
                }
                last_tx.insert(last_tx.end(), p.value().data, p.value().data + p.value().size);
                (void)tx->advance(p.value().size);
            }
            total += last_tx.size();
        }
        if (rx != nullptr) {
            auto rsv = rx->reserve(SIZE_MAX);
            if (!rsv.has_value()) {
                return m5::stl::make_unexpected(rsv.error());
            }
            for (size_t i = 0; i < rsv.value().size; ++i) {
                rsv.value().data[i] = static_cast<uint8_t>(rx_pattern + i);
            }
            (void)rx->commit(rsv.value().size);
            total += rsv.value().size;
        }
        return total;
    }
};

struct StubUARTBus : public uart_::UARTBus {
    std::vector<uint8_t> written;
    std::vector<uint8_t> rx_data;  // bytes "already received" remotely
    uart_::UARTAccessConfig last_cfg{};

    m5::stl::expected<void, error_t> init(const bus::BusConfig& config) override
    {
        if (config.getBusKind() != types::bus_kind_t::UART) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
        _config = static_cast<const uart_::UARTBusConfig&>(config);
        return {};
    }
    m5::stl::expected<size_t, error_t> write(bus::Accessor*, const uart_::UARTAccessConfig& cfg, data::Source* tx,
                                             size_t len) override
    {
        last_cfg     = cfg;
        size_t total = 0;
        while (tx != nullptr && total < len && !tx->eof()) {
            auto p = tx->peek(len - total);
            if (!p.has_value() || p.value().size == 0) {
                break;
            }
            written.insert(written.end(), p.value().data, p.value().data + p.value().size);
            total += p.value().size;
            (void)tx->advance(p.value().size);
        }
        return total;
    }
    m5::stl::expected<size_t, error_t> read(bus::Accessor*, const uart_::UARTAccessConfig& cfg, data::Sink* rx,
                                            size_t len) override
    {
        last_cfg = cfg;
        if (rx == nullptr) {
            return size_t{0};
        }
        auto rsv = rx->reserve(len);
        if (!rsv.has_value()) {
            return m5::stl::make_unexpected(rsv.error());
        }
        const size_t n = rx_data.size() < rsv.value().size ? rx_data.size() : rsv.value().size;
        ::memcpy(rsv.value().data, rx_data.data(), n);
        rx_data.erase(rx_data.begin(), rx_data.begin() + static_cast<long>(n));
        (void)rx->commit(n);
        return n;  // short read when fewer bytes were "available"
    }
    m5::stl::expected<size_t, error_t> readableBytes(bus::Accessor*, const uart_::UARTAccessConfig&) override
    {
        return rx_data.size();
    }
};

// Records pin operations; read values are programmable per pin.
class RecordingGPIO : public gpio::IGPIO {
public:
    static constexpr uint16_t kPins = 64;
    bool level[kPins]               = {};
    bool written[kPins]             = {};
    uint8_t mode[kPins]             = {};
    bool read_value[kPins]          = {};

    gpio::IPort* portForPin(types::gpio_local_pin_t pin_index) const override
    {
        return pin_index < kPins ? &_port : nullptr;
    }
    gpio::IPort* getPort(uint8_t n) const override
    {
        return n == 0 ? &_port : nullptr;
    }
    uint16_t getPinCount() const override
    {
        return kPins;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }

private:
    class Port : public gpio::IPort {
    public:
        explicit Port(RecordingGPIO& owner) : _owner{&owner}
        {
        }

    protected:
        void _writePinEncoded(uint32_t n, bool v) override
        {
            _owner->level[n]   = v;
            _owner->written[n] = true;
        }
        bool _readPinEncoded(uint32_t n) override
        {
            return _owner->read_value[n];
        }
        void _setPinModeEncoded(uint32_t n, types::gpio_mode_t m) override
        {
            _owner->mode[n] = static_cast<uint8_t>(m);
        }
        types::gpio_local_pin_t _toLocalPin(uint32_t n) const override
        {
            return static_cast<types::gpio_local_pin_t>(n);
        }
        uint32_t _fromLocalPin(types::gpio_local_pin_t p) const override
        {
            return p;
        }

    private:
        RecordingGPIO* _owner;
    };

    mutable Port _port{*const_cast<RecordingGPIO*>(this)};
};

// Full server: I2C + SPI + UART + GPIO published, host proxies on top.
struct StageBLoopback : public ::testing::Test {
    std::vector<uint8_t> to_server;
    std::vector<uint8_t> to_host;

    VecSink host_tx{to_server};
    VecSource host_rx{to_host};
    VecSink server_tx{to_host};
    VecSource server_rx{to_server};

    StubSPIBus stub_spi;
    spi::SPIMasterAccessConfig spi_acc_cfg{};
    spi::SPIMasterAccessor spi_acc{stub_spi, spi_acc_cfg};

    StubUARTBus stub_uart;
    uart_::UARTAccessConfig uart_acc_cfg{};
    uart_::UARTAccessor uart_acc{stub_uart, uart_acc_cfg};

    RecordingGPIO rec_gpio;
    gpio::GPIOGroup server_group;

    uint8_t server_scratch[remote::kMaxMessageSize]{};
    remote::Server server{data::DataSpan{server_scratch, sizeof(server_scratch)}};
    remote::RemoteServerService server_service{server, server_rx, server_tx};

    remote::RemoteSession session{host_rx, host_tx};

    static void pumpThunk(void* ctx)
    {
        auto* self = static_cast<StageBLoopback*>(ctx);
        self->server_service.service(service::ServiceContext{});
    }

    void SetUp() override
    {
        ASSERT_TRUE(server.registerSPI(0, spi_acc).has_value());
        ASSERT_TRUE(server.registerUART(0, uart_acc).has_value());
        ASSERT_TRUE(server_group.addGPIO(&rec_gpio, 0).has_value());
        server.setGPIOGroup(server_group);
        host_rx.setPump(&pumpThunk, this);
    }
};

TEST_F(StageBLoopback, HelloListsAllPublishedBuses)
{
    auto caps = session.hello();
    ASSERT_TRUE(caps.has_value());
    EXPECT_TRUE(caps.value().has_gpio);
    ASSERT_EQ(caps.value().bus_count, 2u);
}

TEST_F(StageBLoopback, SpiTransferCarriesDescAndData)
{
    remote::RemoteSPIBus proxy{session, 0};
    spi::SPIMasterAccessConfig cfg;
    cfg.freq = 1000000;

    // Direct transfer with a command/address-bearing desc: verifies the
    // SPI meta marshalling through the bytecode.
    spi::TransferDesc desc;
    desc.command       = 0x9F;
    desc.command_bytes = 1;
    desc.dummy_cycles  = 8;
    const uint8_t tx[] = {0x12, 0x34};
    data::MemorySource tx_src{data::ConstDataSpan{tx, sizeof(tx)}};
    uint8_t rx[4] = {};
    data::MemorySink rx_snk{data::DataSpan{rx, sizeof(rx)}};

    auto r = proxy.transfer(nullptr, cfg, desc, &tx_src, &rx_snk);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(stub_spi.last_cfg.freq, 1000000u);
    EXPECT_EQ(stub_spi.last_desc.command, 0x9Fu);
    EXPECT_EQ(stub_spi.last_desc.command_bytes, 1);
    EXPECT_EQ(stub_spi.last_desc.dummy_cycles, 8);
    ASSERT_EQ(stub_spi.last_tx.size(), sizeof(tx));
    EXPECT_EQ(::memcmp(stub_spi.last_tx.data(), tx, sizeof(tx)), 0);
    for (size_t i = 0; i < sizeof(rx); ++i) {
        EXPECT_EQ(rx[i], static_cast<uint8_t>(stub_spi.rx_pattern + i));
    }
}

TEST_F(StageBLoopback, SpiAccessorSugarWorks)
{
    remote::RemoteSPIBus proxy{session, 0};
    spi::SPIMasterAccessConfig cfg;
    spi::SPIMasterAccessor acc{proxy, cfg};

    uint8_t rx[3] = {};
    auto r        = acc.read(rx, sizeof(rx));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(rx[0], stub_spi.rx_pattern);
}

TEST_F(StageBLoopback, UartWriteAndReadRoundtrip)
{
    remote::RemoteUARTBus proxy{session, 0};
    uart_::UARTAccessConfig cfg;
    uart_::UARTTxAccessor tx{proxy, cfg};
    uart_::UARTRxAccessor rx{proxy, cfg};

    const uint8_t out[] = {0x01, 0x02, 0x03, 0x04};
    auto w              = tx.write(out, sizeof(out));
    ASSERT_TRUE(w.has_value());
    EXPECT_EQ(w.value(), sizeof(out));
    ASSERT_EQ(stub_uart.written.size(), sizeof(out));
    EXPECT_EQ(::memcmp(stub_uart.written.data(), out, sizeof(out)), 0);

    stub_uart.rx_data = {0xAA, 0xBB, 0xCC};
    uint8_t in[8]     = {};
    auto r            = rx.read(in, sizeof(in));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 3u);  // remote short read preserved
    EXPECT_EQ(in[0], 0xAA);
    EXPECT_EQ(in[2], 0xCC);
}

TEST_F(StageBLoopback, UartReadableBytesIsUnsupported)
{
    remote::RemoteUARTBus proxy{session, 0};
    uart_::UARTAccessConfig cfg;
    uart_::UARTRxAccessor rx{proxy, cfg};
    auto r = rx.readableBytes();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::UNSUPPORTED);
}

TEST_F(StageBLoopback, GpioPinHandlesWorkThroughHostGroupSlot)
{
    // Host side: the remote GPIO registers on a host group slot like an
    // I/O expander; Pin handles come from the unified number space.
    remote::RemoteGPIO remote_gpio{session, 100, 0};  // remote slot 0
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());  // host slot 3

    auto pin = host_group.tryGetPin(types::makeGpioNumber(3, 13));
    ASSERT_TRUE(pin.has_value());

    pin.value().setMode(types::gpio_mode_t::Output);
    pin.value().writeHigh();
    ASSERT_TRUE(session.ping().has_value());  // flush the NORESP writes
    EXPECT_TRUE(stub_uart.rx_data.empty());   // (sanity: unrelated state untouched)
    EXPECT_TRUE(rec_gpio.written[13]);
    EXPECT_TRUE(rec_gpio.level[13]);
    EXPECT_EQ(rec_gpio.mode[13], static_cast<uint8_t>(types::gpio_mode_t::Output));
    EXPECT_EQ(session.lastRemoteError(), error_t::OK);

    pin.value().writeLow();
    ASSERT_TRUE(session.ping().has_value());
    EXPECT_FALSE(rec_gpio.level[13]);

    rec_gpio.read_value[13] = true;
    EXPECT_TRUE(pin.value().read());
    rec_gpio.read_value[13] = false;
    EXPECT_FALSE(pin.value().read());
}

TEST_F(StageBLoopback, GpioWriteFailureSurfacesAsPendingError)
{
    // Host-side pin 80 exists (pin_count 100) but the server's recording
    // GPIO only has 64 pins: the NORESP write fails remotely and the
    // error arrives with the next synchronous exchange.
    remote::RemoteGPIO remote_gpio{session, 100, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    auto pin = host_group.tryGetPin(types::makeGpioNumber(3, 80));
    ASSERT_TRUE(pin.has_value());
    pin.value().writeHigh();
    EXPECT_EQ(session.lastRemoteError(), error_t::OK);  // not delivered yet
    ASSERT_TRUE(session.ping().has_value());
    EXPECT_NE(session.lastRemoteError(), error_t::OK);
}

// ---- RemoteLink (StreamReader/Writer seam) --------------------------------------

// Stream-level wire doubles: the same loopback idea as VecSource/VecSink,
// but at the transport seam RemoteLink consumes (StreamReader/Writer).
class VecStreamWriter : public data::StreamWriter {
public:
    explicit VecStreamWriter(std::vector<uint8_t>& out) : _out{&out}
    {
    }
    m5::stl::expected<size_t, error_t> write(data::ConstDataSpan src) override
    {
        _out->insert(_out->end(), src.data, src.data + src.size);
        return src.size;
    }

private:
    std::vector<uint8_t>* _out = nullptr;
};

class VecStreamReader : public data::StreamReader {
public:
    using pump_fn_t = void (*)(void* ctx);

    explicit VecStreamReader(std::vector<uint8_t>& in) : _in{&in}
    {
    }
    void setPump(pump_fn_t fn, void* ctx)
    {
        _pump     = fn;
        _pump_ctx = ctx;
    }
    m5::stl::expected<size_t, error_t> read(data::DataSpan dst) override
    {
        if (_pump != nullptr) {
            _pump(_pump_ctx);
        }
        const size_t avail = _in->size() - _pos;
        const size_t take  = avail < dst.size ? avail : dst.size;
        ::memcpy(dst.data, _in->data() + _pos, take);
        _pos += take;
        return take;  // 0 = nothing arrived in time (StreamReader contract)
    }
    m5::stl::expected<size_t, error_t> readableBytes(void) override
    {
        return _in->size() - _pos;
    }

private:
    std::vector<uint8_t>* _in = nullptr;
    size_t _pos               = 0;
    pump_fn_t _pump           = nullptr;
    void* _pump_ctx           = nullptr;
};

struct LinkLoopback : public ::testing::Test {
    std::vector<uint8_t> to_server;
    std::vector<uint8_t> to_host;

    // host endpoint at the stream seam
    VecStreamWriter host_tx{to_server};
    VecStreamReader host_rx{to_host};
    remote::RemoteLink link{host_rx, host_tx};

    // device endpoint (Source/Sink level, as in the Loopback fixture)
    VecSink server_tx{to_host};
    VecSource server_rx{to_server};

    StubI2CBus stub_bus;
    i2c::I2CMasterAccessConfig stub_acc_cfg{};
    i2c::I2CMasterAccessor stub_acc{stub_bus, stub_acc_cfg};

    uint8_t server_scratch[remote::kMaxMessageSize]{};
    remote::Server server{data::DataSpan{server_scratch, sizeof(server_scratch)}};
    remote::RemoteServerService server_service{server, server_rx, server_tx};

    bool pump_enabled = true;

    static void pumpThunk(void* ctx)
    {
        auto* self = static_cast<LinkLoopback*>(ctx);
        if (self->pump_enabled) {
            self->server_service.service(service::ServiceContext{});
        }
    }

    void SetUp() override
    {
        ASSERT_TRUE(server.registerI2C(0, stub_acc).has_value());
        host_rx.setPump(&pumpThunk, this);
    }
};

TEST_F(LinkLoopback, HelloThroughRemoteLink)
{
    auto caps = link.session().hello();
    ASSERT_TRUE(caps.has_value());
    EXPECT_EQ(caps.value().proto_ver, remote::kProtocolVersion);
    ASSERT_EQ(caps.value().bus_count, 1u);
    EXPECT_EQ(caps.value().buses[0].kind, types::bus_kind_t::I2C);
}

TEST_F(LinkLoopback, ResetStartsAFreshConnection)
{
    auto& session = link.session();
    session.setResponseTimeout(20);

    // Silence the server: ping times out and arms the resync delimiter.
    pump_enabled = false;
    ASSERT_FALSE(session.ping().has_value());

    // reset() begins a new connection: the armed delimiter is dropped,
    // so the next outgoing bytes start with a frame, not 00 55.
    session.reset();
    pump_enabled      = true;
    const size_t mark = to_server.size();
    ASSERT_TRUE(session.ping().has_value());
    ASSERT_GT(to_server.size(), mark);
    EXPECT_NE(to_server[mark + 1], 0x55);  // no delimiter prefix after reset
}

TEST_F(LinkLoopback, ProxyWorksOverRemoteLink)
{
    remote::RemoteI2CBus proxy{link.session(), 0};
    i2c::I2CMasterAccessConfig cfg;
    cfg.i2c_addr = 0x42;
    i2c::I2CMasterAccessor acc{proxy, cfg};

    uint8_t rx[2] = {};
    ASSERT_TRUE(acc.read(rx, sizeof(rx)).has_value());
    EXPECT_EQ(rx[0], stub_bus.rx_pattern);
    EXPECT_EQ(stub_bus.last_cfg.i2c_addr, 0x42);
}

TEST_F(Loopback, TimeoutThenResyncDelimiter)
{
    pump_enabled = false;  // the server goes silent

    remote::RemoteSession::Config cfg;
    cfg.response_timeout_ms = 20;
    remote::RemoteSession fast_session{host_rx, host_tx, cfg};

    auto r = fast_session.ping();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::TIMEOUT_ERROR);

    // The next outgoing message is preceded by a delimiter (00 55).
    pump_enabled            = true;
    const size_t mark       = to_server.size();
    ASSERT_TRUE(fast_session.ping().has_value());
    ASSERT_GE(to_server.size(), mark + 2);
    EXPECT_EQ(to_server[mark], 0x00);
    EXPECT_EQ(to_server[mark + 1], 0x55);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
