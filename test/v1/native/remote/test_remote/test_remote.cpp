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

using ::m5::hal::v1::result_t;

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
    result_t<data::DataSpan> reserve(size_t max_len) override
    {
        return data::DataSpan{_stage, max_len < sizeof(_stage) ? max_len : sizeof(_stage)};
    }
    result_t<void> commit(size_t N) override
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

    result_t<data::ConstDataSpan> peek(size_t max_len) override
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
    result_t<void> advance(size_t N) override
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
struct StubI2cBus : public i2c::IBus {
    i2c::MasterAccessConfig last_cfg{};
    uint8_t last_prefix[i2c::TransferDesc::PREFIX_CAPACITY]{};
    size_t last_prefix_len = 0;
    std::vector<uint8_t> last_tx;
    size_t transfer_count = 0;
    uint8_t rx_pattern    = 0xA0;         // rx byte i = rx_pattern + i
    error_t result        = error_t::OK;  // forced outcome

    result_t<void> init(const i2c::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    result_t<size_t> transfer(bus::IAccessor*, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc,
                              data::Source* tx, data::Sink* rx) override
    {
        ++transfer_count;
        last_cfg        = cfg;
        last_prefix_len = desc.prefix_len;
        ::memcpy(last_prefix, desc.prefix, sizeof(last_prefix));
        last_tx.clear();
        if (error::isError(result)) {
            return m5::stl::make_unexpected(result);
        }
        size_t total = 0;  // data phase only
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

    StubI2cBus stub_bus;
    i2c::MasterAccessConfig stub_acc_cfg{};
    i2c::MasterAccessor stub_acc{stub_bus, stub_acc_cfg};

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
    i2c::MasterAccessConfig cfg;
    cfg.i2c_addr = 0x68;
    i2c::MasterAccessor acc{proxy, cfg};

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
    i2c::MasterAccessConfig cfg;
    cfg.i2c_addr = 0x10;
    i2c::MasterAccessor acc{proxy, cfg};

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
    i2c::MasterAccessConfig cfg;
    i2c::MasterAccessor acc{proxy, cfg};

    auto r = acc.probe();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::I2C_NO_ACK);
}

TEST_F(Loopback, UnregisteredBusReportsInvalidArgument)
{
    // bus 2 is a valid binding slot with nothing registered (a slot
    // outside kMaxBusBindings would be PROTOCOL_ERROR instead).
    remote::RemoteI2CBus proxy{session, 2};
    i2c::MasterAccessConfig cfg;
    i2c::MasterAccessor acc{proxy, cfg};

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
    i2c::MasterAccessConfig cfg;
    ASSERT_TRUE(enc.configure(2, cfg).has_value());  // valid slot, nothing registered
    ASSERT_TRUE(enc.end().has_value());

    ASSERT_TRUE(session.requestNoResponse(data::ConstDataSpan{script, sink.written()}).has_value());
    EXPECT_EQ(session.lastRemoteError(), error_t::OK);  // not delivered yet

    ASSERT_TRUE(session.ping().has_value());
    EXPECT_EQ(session.lastRemoteError(), error_t::INVALID_ARGUMENT);
    EXPECT_FALSE(server.hasPendingError());
}

// Regression pair: the server clears its pending NORESP error as
// soon as the error frame is SENT, so when that delivery crosses a
// host-side timeout, the frame that eventually arrives carries the seq
// of the timed-out exchange. It is the only copy — it must be recorded
// no matter which host path drains it.

TEST_F(Loopback, ErrorCrossingTimeoutIsRecordedBySubsequentRequest)
{
    remote::RemoteSession::Config scfg;
    scfg.response_timeout_ms = 20;
    remote::RemoteSession fast_session{host_rx, host_tx, scfg};

    uint8_t script[32] = {};
    data::MemorySink sink{data::DataSpan{script, sizeof(script)}};
    bytecode::BytecodeEncoder enc{sink};
    i2c::MasterAccessConfig cfg;
    ASSERT_TRUE(enc.configure(2, cfg).has_value());  // valid slot, nothing registered
    ASSERT_TRUE(enc.end().has_value());
    ASSERT_TRUE(fast_session.requestNoResponse(data::ConstDataSpan{script, sink.written()}).has_value());

    // The server goes silent: the ping that would deliver the pending
    // error times out host-side.
    pump_enabled = false;
    auto p1      = fast_session.ping();
    ASSERT_FALSE(p1.has_value());
    EXPECT_EQ(p1.error(), error_t::TIMEOUT_ERROR);
    EXPECT_EQ(fast_session.lastRemoteError(), error_t::OK);  // nothing arrived yet

    // The server catches up late: it processes the NORESP failure and the
    // timed-out ping, emitting error + pong stamped with the OLD seq.
    pump_enabled = true;
    server_service.service(service::ServiceContext{});

    // The next ping awaits a NEW seq; the late error frame read along the
    // way must still land in lastRemoteError().
    ASSERT_TRUE(fast_session.ping().has_value());
    EXPECT_EQ(fast_session.lastRemoteError(), error_t::INVALID_ARGUMENT);
    EXPECT_FALSE(server.hasPendingError());
}

TEST_F(Loopback, ErrorCrossingTimeoutIsRecordedByIdlePoll)
{
    remote::RemoteSession::Config scfg;
    scfg.response_timeout_ms = 20;
    remote::RemoteSession fast_session{host_rx, host_tx, scfg};

    uint8_t script[32] = {};
    data::MemorySink sink{data::DataSpan{script, sizeof(script)}};
    bytecode::BytecodeEncoder enc{sink};
    i2c::MasterAccessConfig cfg;
    ASSERT_TRUE(enc.configure(2, cfg).has_value());
    ASSERT_TRUE(enc.end().has_value());
    ASSERT_TRUE(fast_session.requestNoResponse(data::ConstDataSpan{script, sink.written()}).has_value());

    pump_enabled = false;
    auto p1      = fast_session.ping();
    ASSERT_FALSE(p1.has_value());
    EXPECT_EQ(p1.error(), error_t::TIMEOUT_ERROR);

    pump_enabled = true;
    server_service.service(service::ServiceContext{});

    // An idle poll() drains the late error (and the stale pong): the
    // error is recorded but does not count as an event.
    auto polled = fast_session.poll();
    ASSERT_TRUE(polled.has_value());
    EXPECT_EQ(polled.value(), 0u);
    EXPECT_EQ(fast_session.lastRemoteError(), error_t::INVALID_ARGUMENT);
}

TEST_F(Loopback, OversizedWireRxLenIsRejectedByPrescan)
{
    // The wire LenVar can spell a full u32 rx_len and the runner
    // allocates it up front — the server prescan caps it
    // (Config::max_transfer_rx, default 4096). Sent via a raw
    // request: the host-side proxy limit (kMaxTransferRx) never sees it.
    uint8_t script[32] = {};
    data::MemorySink sink{data::DataSpan{script, sizeof(script)}};
    bytecode::BytecodeEncoder enc{sink};
    ASSERT_TRUE(enc.transfer(0, i2c::TransferDesc{}, data::ConstDataSpan{}, 8192, 1).has_value());
    ASSERT_TRUE(enc.end().has_value());

    auto r = session.request(data::ConstDataSpan{script, sink.written()});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::INVALID_ARGUMENT);
}

TEST_F(Loopback, OversizedReceiveIsRejectedLocally)
{
    remote::RemoteI2CBus proxy{session, 0};
    i2c::MasterAccessConfig cfg;
    i2c::MasterAccessor acc{proxy, cfg};

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
    i2c::MasterAccessConfig cfg;
    i2c::MasterAccessor acc{proxy, cfg};

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
namespace i2s   = ::m5::hal::v1::i2s;
namespace spi   = ::m5::hal::v1::spi;
namespace uart_ = ::m5::hal::v1::uart;

struct StubSpiBus : public spi::IBus {
    spi::MasterAccessConfig last_cfg{};
    spi::TransferDesc last_desc{};
    std::vector<uint8_t> last_tx;
    uint8_t rx_pattern = 0xB0;

    result_t<void> init(const spi::IBusConfig& config)
    {
        _config = config;
        return {};
    }
    result_t<size_t> transfer(bus::IAccessor*, const spi::MasterAccessConfig& cfg, const spi::TransferDesc& desc,
                              data::Source* tx, data::Sink* rx) override
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

struct StubUartBus : public uart_::IBus {
    std::vector<uint8_t> written;
    std::vector<uint8_t> rx_data;  // bytes "already received" remotely
    uart_::AccessConfig last_cfg{};

    result_t<void> init(const uart_::IBusConfig& config)
    {
        _config = config;
        return {};
    }
    result_t<size_t> write(bus::IAccessor*, const uart_::AccessConfig& cfg, data::Source* tx, size_t len) override
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
    result_t<size_t> read(bus::IAccessor*, const uart_::AccessConfig& cfg, data::Sink* rx, size_t len) override
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
    result_t<size_t> readableBytes(bus::IAccessor*, const uart_::AccessConfig&) override
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

    StubSpiBus stub_spi;
    spi::MasterAccessConfig spi_acc_cfg{};
    spi::MasterAccessor spi_acc{stub_spi, spi_acc_cfg};

    StubUartBus stub_uart;
    uart_::AccessConfig uart_acc_cfg{};
    uart_::Accessor uart_acc{stub_uart, uart_acc_cfg};

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
    spi::MasterAccessConfig cfg;
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
    spi::MasterAccessConfig cfg;
    spi::MasterAccessor acc{proxy, cfg};

    uint8_t rx[3] = {};
    auto r        = acc.read(rx, sizeof(rx));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(rx[0], stub_spi.rx_pattern);
}

TEST_F(StageBLoopback, UartWriteAndReadRoundtrip)
{
    remote::RemoteUARTBus proxy{session, 0};
    uart_::AccessConfig cfg;
    uart_::TxAccessor tx{proxy, cfg};
    uart_::RxAccessor rx{proxy, cfg};

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
    uart_::AccessConfig cfg;
    uart_::RxAccessor rx{proxy, cfg};
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
    result_t<size_t> write(data::ConstDataSpan src) override
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
    result_t<size_t> read(data::DataSpan dst) override
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
    result_t<size_t> readableBytes(void) override
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

    StubI2cBus stub_bus;
    i2c::MasterAccessConfig stub_acc_cfg{};
    i2c::MasterAccessor stub_acc{stub_bus, stub_acc_cfg};

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
    i2c::MasterAccessConfig cfg;
    cfg.i2c_addr = 0x42;
    i2c::MasterAccessor acc{proxy, cfg};

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
    pump_enabled      = true;
    const size_t mark = to_server.size();
    ASSERT_TRUE(fast_session.ping().has_value());
    ASSERT_GE(to_server.size(), mark + 2);
    EXPECT_EQ(to_server[mark], 0x00);
    EXPECT_EQ(to_server[mark + 1], 0x55);
}

// ---- Stage C: push event machinery -------------------------------------------

// StageBLoopback has rec_gpio at server slot 0, pin_count 64.
// The host registers RemoteGPIO at host slot 3.

// Helper: pump the server_service once (used in poll() callback)
// poll() on the session drives a single server_service pass which may
// emit an event frame.  We wire this as the VecSource pump.

// Note: StageBLoopback::pumpThunk already drives the server service once
// per host_rx.peek() call. For poll() we need the server to be driven
// too, which happens via the pump.

TEST_F(StageBLoopback, SubscribeAndEventRoundtrip)
{
    // Install gpio event handler on host runner
    types::gpio_number_t captured_pin = -1;
    bool captured_level               = false;
    session.runner().setGpioEventHandler(
        [](void* ctx, types::gpio_number_t pin, bool level) {
            auto* p    = static_cast<std::pair<types::gpio_number_t*, bool*>*>(ctx);
            *p->first  = pin;
            *p->second = level;
        },
        nullptr);
    // Reinitialize with proper context
    std::pair<types::gpio_number_t*, bool*> ctx{&captured_pin, &captured_level};
    session.runner().setGpioEventHandler(
        [](void* c, types::gpio_number_t pin, bool level) {
            auto* p    = static_cast<std::pair<types::gpio_number_t*, bool*>*>(c);
            *p->first  = pin;
            *p->second = level;
        },
        &ctx);

    // Register remote GPIO at host slot 3
    remote::RemoteGPIO remote_gpio{session, 100, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    // subscribe pin 13 (local pin on host slot 3 -> remote makeGpioNumber(0,13))
    rec_gpio.read_value[13] = false;
    ASSERT_TRUE(remote_gpio.subscribe(13).has_value());

    // Change the level: server should detect and send event
    rec_gpio.read_value[13] = true;

    // poll() triggers server_service (via pump) which calls pollSubscriptions
    auto r = session.poll();
    ASSERT_TRUE(r.has_value());
    EXPECT_GE(r.value(), 1u);
    EXPECT_EQ(captured_pin, types::makeGpioNumber(0, 13));
    EXPECT_TRUE(captured_level);

    // Change back
    rec_gpio.read_value[13] = false;
    captured_pin            = -1;
    r                       = session.poll();
    ASSERT_TRUE(r.has_value());
    EXPECT_GE(r.value(), 1u);
    EXPECT_EQ(captured_pin, types::makeGpioNumber(0, 13));
    EXPECT_FALSE(captured_level);
}

TEST_F(StageBLoopback, SubscribeBaselineIsCurrentLevel)
{
    types::gpio_number_t captured_pin = -1;
    bool captured_level               = false;
    std::pair<types::gpio_number_t*, bool*> ctx{&captured_pin, &captured_level};
    session.runner().setGpioEventHandler(
        [](void* c, types::gpio_number_t pin, bool level) {
            auto* p    = static_cast<std::pair<types::gpio_number_t*, bool*>*>(c);
            *p->first  = pin;
            *p->second = level;
        },
        &ctx);

    remote::RemoteGPIO remote_gpio{session, 100, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    // subscribe with pin already high
    rec_gpio.read_value[13] = true;
    ASSERT_TRUE(remote_gpio.subscribe(13).has_value());

    // poll without changing: no event expected
    auto r = session.poll();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 0u);
    EXPECT_EQ(captured_pin, static_cast<types::gpio_number_t>(-1));

    // Now change: event fires
    rec_gpio.read_value[13] = false;
    r                       = session.poll();
    ASSERT_TRUE(r.has_value());
    EXPECT_GE(r.value(), 1u);
    EXPECT_FALSE(captured_level);
}

TEST_F(StageBLoopback, NoChangeNoEvent)
{
    remote::RemoteGPIO remote_gpio{session, 100, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    rec_gpio.read_value[13] = false;
    ASSERT_TRUE(remote_gpio.subscribe(13).has_value());

    // No change: poll returns 0
    auto r = session.poll();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 0u);
}

TEST_F(StageBLoopback, UnsubscribeStopsEvents)
{
    types::gpio_number_t captured_pin = -1;
    std::pair<types::gpio_number_t*, bool*> ctx{&captured_pin, nullptr};
    // Use a simpler handler that just records the pin
    session.runner().setGpioEventHandler(
        [](void* c, types::gpio_number_t pin, bool) { *static_cast<types::gpio_number_t*>(c) = pin; }, &captured_pin);

    remote::RemoteGPIO remote_gpio{session, 100, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    rec_gpio.read_value[13] = false;
    ASSERT_TRUE(remote_gpio.subscribe(13).has_value());
    ASSERT_TRUE(remote_gpio.unsubscribe(13).has_value());

    rec_gpio.read_value[13] = true;
    auto r                  = session.poll();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 0u);
    EXPECT_EQ(captured_pin, static_cast<types::gpio_number_t>(-1));
}

TEST_F(StageBLoopback, SubscriptionTableFull)
{
    remote::RemoteGPIO remote_gpio{session, 100, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    // Subscribe kMaxSubscriptions pins (0..7): all should succeed
    for (uint8_t p = 0; p < remote::Server::kMaxSubscriptions; ++p) {
        ASSERT_TRUE(remote_gpio.subscribe(p).has_value()) << "pin " << (int)p;
    }

    // 9th subscription should fail with OUT_OF_RESOURCE
    auto r = remote_gpio.subscribe(static_cast<types::gpio_local_pin_t>(remote::Server::kMaxSubscriptions));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::OUT_OF_RESOURCE);
}

TEST_F(StageBLoopback, BatchSubscribeIsAllOrNothing)
{
    types::gpio_number_t captured_pin = -1;
    session.runner().setGpioEventHandler(
        [](void* c, types::gpio_number_t pin, bool) { *static_cast<types::gpio_number_t*>(c) = pin; }, &captured_pin);

    remote::RemoteGPIO remote_gpio{session, RecordingGPIO::kPins, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    // Pre-existing subscription that a failed batch must not disturb.
    rec_gpio.read_value[13] = false;
    ASSERT_TRUE(remote_gpio.subscribe(13).has_value());

    // Batch [valid 5, invalid 200]: the whole instruction must fail...
    uint8_t script_buf[32];
    data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{sink};
    const types::gpio_number_t pins[2] = {types::makeGpioNumber(0, 5), types::makeGpioNumber(0, 200)};
    ASSERT_TRUE(enc.gpioSubscribe(pins, 2).has_value());
    ASSERT_TRUE(enc.end().has_value());
    auto rq = session.request(data::ConstDataSpan{script_buf, sink.written()});
    ASSERT_FALSE(rq.has_value());
    EXPECT_EQ(rq.error(), error_t::INVALID_ARGUMENT);

    // ...without subscribing the leading valid pin (no event on change).
    rec_gpio.read_value[5] = true;
    auto p                 = session.poll();
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(p.value(), 0u);
    EXPECT_EQ(captured_pin, static_cast<types::gpio_number_t>(-1));

    // The pre-existing subscription still works.
    rec_gpio.read_value[13] = true;
    p                       = session.poll();
    ASSERT_TRUE(p.has_value());
    EXPECT_GE(p.value(), 1u);
    EXPECT_EQ(captured_pin, types::makeGpioNumber(0, 13));
}

TEST_F(StageBLoopback, BatchSubscribeCapacityCheckedUpfrontWithDedup)
{
    remote::RemoteGPIO remote_gpio{session, RecordingGPIO::kPins, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    // Fill all but one slot (pins 0..6).
    for (uint8_t p = 0; p + 1 < remote::Server::kMaxSubscriptions; ++p) {
        ASSERT_TRUE(remote_gpio.subscribe(p).has_value()) << "pin " << (int)p;
    }

    // Batch of 2 NEW pins with 1 free slot: must fail atomically (the
    // first pin would fit, but no prefix may be applied).
    uint8_t script_buf[32];
    {
        data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
        bytecode::BytecodeEncoder enc{sink};
        const types::gpio_number_t pins[2] = {types::makeGpioNumber(0, 9), types::makeGpioNumber(0, 10)};
        ASSERT_TRUE(enc.gpioSubscribe(pins, 2).has_value());
        ASSERT_TRUE(enc.end().has_value());
        auto rq = session.request(data::ConstDataSpan{script_buf, sink.written()});
        ASSERT_FALSE(rq.has_value());
        EXPECT_EQ(rq.error(), error_t::OUT_OF_RESOURCE);
    }

    // Batch [9, 9, already-subscribed 0] nets ONE new pin: the capacity
    // pre-check must count duplicates only once, so this fits the free
    // slot (which also proves the failed batch above consumed nothing).
    {
        data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
        bytecode::BytecodeEncoder enc{sink};
        const types::gpio_number_t pins[3] = {types::makeGpioNumber(0, 9), types::makeGpioNumber(0, 9),
                                              types::makeGpioNumber(0, 0)};
        ASSERT_TRUE(enc.gpioSubscribe(pins, 3).has_value());
        ASSERT_TRUE(enc.end().has_value());
        ASSERT_TRUE(session.request(data::ConstDataSpan{script_buf, sink.written()}).has_value());
    }
}

TEST_F(StageBLoopback, HelloClearsSubscriptions)
{
    types::gpio_number_t captured_pin = -1;
    session.runner().setGpioEventHandler(
        [](void* c, types::gpio_number_t pin, bool) { *static_cast<types::gpio_number_t*>(c) = pin; }, &captured_pin);

    remote::RemoteGPIO remote_gpio{session, 100, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    rec_gpio.read_value[13] = false;
    ASSERT_TRUE(remote_gpio.subscribe(13).has_value());

    // hello clears server subscriptions
    ASSERT_TRUE(session.hello().has_value());

    // Change level: no event expected (subscription was cleared)
    rec_gpio.read_value[13] = true;
    auto r                  = session.poll();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 0u);
    EXPECT_EQ(captured_pin, static_cast<types::gpio_number_t>(-1));
}

TEST_F(Loopback, UnsupportedWithoutGpioGroup)
{
    // Loopback fixture has no GPIOGroup set on the server
    remote::RemoteGPIO remote_gpio{session, 64, 0};
    gpio::GPIOGroup host_group;
    ASSERT_TRUE(host_group.addGPIO(&remote_gpio, 3).has_value());

    auto r = remote_gpio.subscribe(5);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::UNSUPPORTED);
}

TEST(BytecodeRunnerSubscribe, PlainRunnerRejectsSubscribe)
{
    // A plain BytecodeRunner (no subscribe handler) should return UNSUPPORTED
    bytecode::BytecodeRunner runner;
    uint8_t script_buf[16];
    data::MemorySink sink{data::DataSpan{script_buf, sizeof(script_buf)}};
    bytecode::BytecodeEncoder enc{sink};
    types::gpio_number_t pin = types::makeGpioNumber(0, 5);
    ASSERT_TRUE(enc.gpioSubscribe(&pin, 1).has_value());
    ASSERT_TRUE(enc.end().has_value());

    auto r = runner.run(data::ConstDataSpan{script_buf, sink.written()});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error_t::UNSUPPORTED);
}

// ---- Stage I2S-1: stream credit ----------------------------------------------

// Device-side stream bus stand-in: a fixed-capacity ring of `capacity`
// bytes that DMA drains explicitly (drain()). `submitted` is the cumulative
// accepted byte count (mod 2^32); a write accepts at most the current free
// space (credit violations show up as a short accept -> BUFFER_OVERFLOW).
struct StubI2sBus : public i2s::IBus {
    uint32_t capacity  = 4096;
    uint32_t in_buffer = 0;  // bytes currently pending in the "DMA buffer"
    uint32_t submitted = 0;  // cumulative accepted (mod 2^32)
    size_t write_calls = 0;
    i2s::AccessConfig last_cfg{};

    // Seed the cumulative counters so wrap-boundary behavior can be tested.
    void seed(uint32_t start)
    {
        submitted = start;
    }
    // Simulate DMA consuming `n` bytes (frees buffer space).
    void drain(uint32_t n)
    {
        in_buffer -= (n < in_buffer ? n : in_buffer);
    }
    uint32_t freeBytes() const
    {
        return capacity - in_buffer;
    }

    result_t<void> init(const i2s::IBusConfig& config)
    {
        _config = config;
        return {};
    }
    result_t<size_t> write(bus::IAccessor*, const i2s::AccessConfig& cfg, data::Source* tx, size_t len) override
    {
        ++write_calls;
        last_cfg = cfg;
        // Drain tx fully (what actually arrived on the wire), accept up to
        // the free space.
        uint32_t arrived = 0;
        while (tx != nullptr && !tx->eof() && arrived < len) {
            auto p = tx->peek(len - arrived);
            if (!p.has_value() || p.value().size == 0) {
                break;
            }
            arrived += static_cast<uint32_t>(p.value().size);
            (void)tx->advance(p.value().size);
        }
        const uint32_t space    = freeBytes();
        const uint32_t accepted = arrived < space ? arrived : space;
        in_buffer += accepted;
        submitted += accepted;
        return static_cast<size_t>(accepted);
    }
    result_t<size_t> writableBytes(bus::IAccessor*, const i2s::AccessConfig&) override
    {
        return static_cast<size_t>(freeBytes());
    }
};

struct StageI2SLoopback : public ::testing::Test {
    std::vector<uint8_t> to_server;
    std::vector<uint8_t> to_host;

    VecSink host_tx{to_server};
    VecSource host_rx{to_host};
    VecSink server_tx{to_host};
    VecSource server_rx{to_server};

    StubI2sBus stub_i2s;
    i2s::AccessConfig i2s_acc_cfg{};
    i2s::TxAccessor i2s_acc{stub_i2s, i2s_acc_cfg};

    uint8_t server_scratch[remote::kMaxMessageSize]{};
    remote::Server::Config server_cfg{};
    remote::Server server{data::DataSpan{server_scratch, sizeof(server_scratch)}, server_cfg};
    remote::RemoteServerService server_service{server, server_rx, server_tx};

    remote::RemoteSession session{host_rx, host_tx};

    // Auto-drain applied inside the pump before each service() pass, so DMA
    // consumption is modeled deterministically: every pump frees `drain_per_pump`
    // device-buffer bytes (0 = the device never drains).
    uint32_t drain_per_pump = 0;
    bool pump_enabled       = true;

    static void pumpThunk(void* ctx)
    {
        auto* self = static_cast<StageI2SLoopback*>(ctx);
        if (!self->pump_enabled) {
            return;
        }
        if (self->drain_per_pump != 0) {
            self->stub_i2s.drain(self->drain_per_pump);
        }
        self->server_service.service(service::ServiceContext{});
    }

    // Pump the server until everything the host queued has been processed.
    // RemoteServerService drains up to kMaxFramesPerPoll frames per call, so a
    // bounded number of passes flushes any backlog of NORESP writes.
    void flushServer()
    {
        for (int i = 0; i < 64; ++i) {
            server_service.service(service::ServiceContext{});
        }
    }

    void SetUp() override
    {
        ASSERT_TRUE(server.registerI2S(0, i2s_acc).has_value());
        host_rx.setPump(&pumpThunk, this);
    }
};

TEST_F(StageI2SLoopback, HelloListsIBus)
{
    auto caps = session.hello();
    ASSERT_TRUE(caps.has_value());
    ASSERT_EQ(caps.value().bus_count, 1u);
    EXPECT_EQ(caps.value().buses[0].kind, types::bus_kind_t::I2S);
    EXPECT_EQ(caps.value().buses[0].bus_id, 0);
}

TEST_F(StageI2SLoopback, WriteConfiguresThenStreamsToServer)
{
    remote::RemoteI2SBus proxy{session, 0};
    i2s::AccessConfig cfg;
    cfg.sample_rate_hz   = 44100;
    cfg.bits_per_sample  = 16;
    cfg.channels         = 2;
    cfg.write_timeout_ms = 100;
    i2s::TxAccessor acc{proxy, cfg};

    std::vector<uint8_t> audio(64);
    for (size_t i = 0; i < audio.size(); ++i) {
        audio[i] = static_cast<uint8_t>(i);
    }
    auto w = acc.write(audio.data(), audio.size());
    ASSERT_TRUE(w.has_value());
    EXPECT_EQ(w.value(), audio.size());

    // Pump the trailing NORESP bus_write_stream messages into the server.
    flushServer();

    // The configure carried through (device write_timeout forced to 0).
    EXPECT_EQ(stub_i2s.last_cfg.sample_rate_hz, 44100u);
    EXPECT_EQ(stub_i2s.last_cfg.write_timeout_ms, 0u);
    // All bytes reached the device's DMA buffer.
    EXPECT_EQ(stub_i2s.submitted, audio.size());
    EXPECT_EQ(stub_i2s.in_buffer, audio.size());
}

TEST_F(StageI2SLoopback, LargeWriteSplitsIntoChunks)
{
    stub_i2s.capacity = 100000;  // plenty of credit
    remote::RemoteI2SBus proxy{session, 0};
    i2s::AccessConfig cfg;
    cfg.write_timeout_ms = 500;
    i2s::TxAccessor acc{proxy, cfg};

    std::vector<uint8_t> audio(1000, 0xAB);  // > one message body (238)
    auto w = acc.write(audio.data(), audio.size());
    ASSERT_TRUE(w.has_value());
    EXPECT_EQ(w.value(), audio.size());

    flushServer();
    EXPECT_EQ(stub_i2s.submitted, audio.size());
    EXPECT_GT(stub_i2s.write_calls, 1u);  // chunked across several messages
}

TEST_F(StageI2SLoopback, CreditExhaustionStallsThenResumesOnDrain)
{
    // Small device buffer: a write larger than the buffer cannot complete in
    // one shot, so the host runs out of credit and stalls. The device drains
    // (drain_per_pump) as the host polls; credit recovers — both through the
    // server's pollStreamCredit event and the host's periodic status re-sync —
    // and the stalled write finishes.
    stub_i2s.capacity = 256;

    remote::RemoteI2SBus proxy{session, 0};
    i2s::AccessConfig cfg;
    cfg.write_timeout_ms = 5000;  // generous: rely on the drain, not the timeout
    i2s::TxAccessor acc{proxy, cfg};

    // 384 bytes > the 256 B buffer: the device drains 128 B per pump, so
    // credit returns over several poll cycles and the write completes.
    drain_per_pump = 128;
    std::vector<uint8_t> audio(384, 0x22);
    auto w = acc.write(audio.data(), audio.size());
    ASSERT_TRUE(w.has_value());
    EXPECT_EQ(w.value(), audio.size());  // all bytes sent once credit recovered
    flushServer();
    EXPECT_EQ(stub_i2s.submitted, audio.size());
}

TEST_F(StageI2SLoopback, CreditViolationSurfacesAsPendingError)
{
    // Force the device to accept less than the host sends by shrinking the
    // device buffer behind the host's back (the host's credit estimate
    // becomes stale). The short accept is a BUFFER_OVERFLOW pending error,
    // delivered on the next synchronous exchange (status re-sync / ping).
    stub_i2s.capacity = 1000000;  // host believes there is huge credit
    remote::RemoteI2SBus proxy{session, 0};
    i2s::AccessConfig cfg;
    cfg.write_timeout_ms = 50;
    i2s::TxAccessor acc{proxy, cfg};

    // Prime credit (config + status) with the large capacity.
    std::vector<uint8_t> warm(4, 0x00);
    ASSERT_TRUE(acc.write(warm.data(), warm.size()).has_value());
    flushServer();

    // Now shrink the device buffer so the next NORESP write overflows it.
    stub_i2s.capacity = stub_i2s.in_buffer;  // zero real free space
    std::vector<uint8_t> big(200, 0x33);
    (void)acc.write(big.data(), big.size());  // some of this overflows server-side
    flushServer();

    // The pending BUFFER_OVERFLOW is delivered on the next synchronous call.
    ASSERT_TRUE(session.ping().has_value());
    EXPECT_EQ(session.lastRemoteError(), error_t::BUFFER_OVERFLOW);
}

TEST_F(StageI2SLoopback, CreditMathWrapsAroundU32Boundary)
{
    // Seed the device's cumulative submitted near the u32 boundary so the
    // host's credit = free - (sent - submitted) exercises wrap arithmetic.
    stub_i2s.capacity = 100000;
    stub_i2s.seed(0xFFFFFF00u);

    remote::RemoteI2SBus proxy{session, 0};
    i2s::AccessConfig cfg;
    cfg.write_timeout_ms = 200;
    i2s::TxAccessor acc{proxy, cfg};

    // Write enough to roll `submitted` (and the host's `_sent`) past 2^32.
    std::vector<uint8_t> audio(1024, 0x5A);
    auto w = acc.write(audio.data(), audio.size());
    ASSERT_TRUE(w.has_value());
    EXPECT_EQ(w.value(), audio.size());
    flushServer();
    // submitted wrapped: 0xFFFFFF00 + 1024 = 0x000003FC (mod 2^32).
    EXPECT_EQ(stub_i2s.submitted, static_cast<uint32_t>(0xFFFFFF00u + 1024u));
    EXPECT_EQ(stub_i2s.in_buffer, 1024u);

    // writableBytes (host estimate) stays sane after the wrap.
    auto wb = acc.writableBytes();
    ASSERT_TRUE(wb.has_value());
    EXPECT_GT(wb.value(), 0u);
}

TEST_F(StageI2SLoopback, CreditPinnedAtZeroAfterFrameLossResolvesOnResync)
{
    // Focused regression for the credit-pinned-at-zero bug.
    //
    // Setup: capacity exactly fills with one write so the second write stalls
    // on credit=0.  The first write's frames are discarded (lost) before the
    // server processes them.  After kCreditResyncMs syncStatus() re-baselines
    // _sent = _submitted and credit is restored.
    //
    // Timing: write_timeout_ms = 500 ms >> kCreditResyncMs (50 ms) so the
    // resync fires well before the timeout, avoiding flakiness.

    const size_t kBuf = 128;
    stub_i2s.capacity = kBuf;  // exactly one write-chunk fills it

    remote::RemoteI2SBus proxy{session, 0};
    i2s::AccessConfig cfg;
    cfg.write_timeout_ms = 500;  // long enough for 50 ms resync + margin
    i2s::TxAccessor acc{proxy, cfg};

    // --- Prime: syncConfig (synchronous, pump active) sets _free=kBuf, _sent=0 ---
    // Trigger syncConfig by doing a zero-effect write of 0 bytes:
    // acc.write with size=0 does not actually send a NORESP, just configures.
    // Actually size=0 would return immediately with done=0; we need to call
    // syncConfig indirectly.  The easiest trigger is a dummy 1-byte write that
    // also pumps normally.
    std::vector<uint8_t> dummy(1, 0xDD);
    ASSERT_TRUE(acc.write(dummy.data(), dummy.size()).has_value());
    flushServer();
    // After dummy: device submitted=1, in_buffer=1, free=kBuf-1.
    // Proxy: _free=kBuf-1, _sent=1, _submitted=1, credit=kBuf-1.
    ASSERT_EQ(stub_i2s.submitted, 1u);

    // --- Write exactly (kBuf-1) bytes to fill the credit to zero ---
    // in_flight before = 0, credit = kBuf-1.  Write kBuf-1 bytes:
    // _sent += kBuf-1 → _sent = kBuf, in_flight = kBuf-1, credit = 0.
    const size_t kFill = kBuf - 1;
    const size_t mark  = to_server.size();
    std::vector<uint8_t> fill_buf(kFill, 0xCC);
    ASSERT_TRUE(acc.write(fill_buf.data(), fill_buf.size()).has_value());

    // Discard the fill NORESP frame — server never receives it.
    to_server.resize(mark);
    ASSERT_EQ(stub_i2s.submitted, 1u);  // server unchanged

    // --- Now credit = 0; the next write must stall and then self-heal ---
    // write_timeout_ms=500 gives plenty of time for syncStatus at t=50 ms.
    std::vector<uint8_t> payload(kFill, 0xEE);
    auto w = acc.write(payload.data(), payload.size());
    ASSERT_TRUE(w.has_value()) << "write timed out — syncStatus did not restore credit";
    EXPECT_EQ(w.value(), kFill);
    flushServer();

    // Re-baseline means _sent = _submitted = 1 after syncStatus.
    // Then payload (kFill bytes) is sent fresh and arrives at device.
    // device submitted = 1 (dummy) + kFill (payload) = kBuf.
    EXPECT_EQ(stub_i2s.submitted, 1u + kFill);

    // The discarded fill_buf (0xCC) never arrived — only dummy (0xDD) and
    // payload (0xEE) are in the device buffer.
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
