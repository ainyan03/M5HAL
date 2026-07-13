// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/remote/server_handler.hpp>
#include <m5_hal/variants/frameworks/posix/hal/remote/tcp_connection.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

namespace {

using namespace m5::hal::v2;
namespace posix_remote = m5::variants::frameworks::posix::hal::v2::remote;

// Endpoint parsing rejects out-of-range ports before any socket work.
// Regression anchor: the port accumulator only checked 65535 after the
// digit loop, so a long digit string could wrap unsigned long back into
// range and "connect" to an unintended port.
TEST(PosixTcpEndpointParse, RejectsOutOfRangeAndOverflowingPorts)
{
    remote::DeviceConfig cfg;
    cfg.response_timeout_ms = 50;

    const char* bad_endpoints[] = {
        "127.0.0.1:0",
        "127.0.0.1:65536",
        "127.0.0.1:4294967297",            // wraps 32-bit unsigned long to 1
        "127.0.0.1:18446744073709551617",  // wraps 64-bit unsigned long to 1
    };
    for (const char* ep : bad_endpoints) {
        auto r = posix_remote::PosixTcpConnection::create(ep, cfg);
        ASSERT_FALSE(r.has_value()) << ep;
        EXPECT_EQ(r.error(), error::error_t::INVALID_ARGUMENT) << ep;
    }
}

#define ASSERT_OK_RESULT(expr)                                                                    \
    do {                                                                                          \
        auto m5hal_result = (expr);                                                               \
        ASSERT_TRUE(m5hal_result.has_value()) << "err=" << error::toString(m5hal_result.error()); \
    } while (false)

#define EXPECT_OK_RESULT(expr)                                                                    \
    do {                                                                                          \
        auto m5hal_result = (expr);                                                               \
        EXPECT_TRUE(m5hal_result.has_value()) << "err=" << error::toString(m5hal_result.error()); \
    } while (false)

class RemoteTestHal : public Hal {
public:
    explicit RemoteTestHal(bus::IHalBackend* backend) : Hal{backend}
    {
    }
};

struct ConnectOutcome {
    std::unique_ptr<posix_remote::PosixTcpConnection> conn;
    error::error_t err = error::error_t::UNKNOWN_ERROR;
};

void peerPollServer(void* ctx)
{
    auto* server = static_cast<remote::BsdTcpRemoteServer*>(ctx);
    (void)server->service();
}

ConnectOutcome connectClient(remote::BsdTcpRemoteServer& server)
{
    char endpoint[64];
    std::snprintf(endpoint, sizeof(endpoint), "127.0.0.1:%u", static_cast<unsigned>(server.boundPort()));

    remote::DeviceConfig cfg;
    cfg.response_timeout_ms = 200;

    std::atomic<bool> done{false};
    posix_remote::PosixTcpConnection* raw = nullptr;
    error::error_t err                    = error::error_t::UNKNOWN_ERROR;

    std::thread worker([&]() {
        auto r = posix_remote::PosixTcpConnection::create(endpoint, cfg);
        if (r.has_value()) {
            raw = r.value();
            err = error::error_t::OK;
        } else {
            err = r.error();
        }
        done.store(true, std::memory_order_release);
    });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!done.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        auto serviced = server.service();
        if (!serviced.has_value()) {
            err = serviced.error();
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    worker.join();

    ConnectOutcome out;
    out.err = err;
    if (raw != nullptr) {
        out.conn.reset(raw);
        out.conn->session().setPeerPoll(&peerPollServer, &server);
    }
    return out;
}

size_t usedI2CSlots(const remote::ServerPhysicalBusPool& phys)
{
    size_t count = 0;
    for (size_t i = 0; i < remote::kServerBusPoolSlots; ++i) {
        if (phys.i2c[i].used) {
            ++count;
        }
    }
    return count;
}

const remote::PhysI2CSlot* firstUsedI2CSlot(const remote::ServerPhysicalBusPool& phys)
{
    for (size_t i = 0; i < remote::kServerBusPoolSlots; ++i) {
        if (phys.i2c[i].used) {
            return &phys.i2c[i];
        }
    }
    return nullptr;
}

const remote::PhysI2CSlot* findI2CSlotByRefcount(const remote::ServerPhysicalBusPool& phys, uint8_t refcount)
{
    for (size_t i = 0; i < remote::kServerBusPoolSlots; ++i) {
        if (phys.i2c[i].used && phys.i2c[i].refcount == refcount) {
            return &phys.i2c[i];
        }
    }
    return nullptr;
}

bool serviceUntil(remote::BsdTcpRemoteServer& server, size_t want_count)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        auto r = server.service();
        if (!r.has_value()) {
            return false;
        }
        if (r.value() == want_count) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

struct SessionPair {
    memory::Allocator& alloc = memory::defaultAllocator();
    uint8_t wire_ab_buf[4096], wire_ba_buf[4096];
    data::RingFIFO wire_ab, wire_ba;
    data::MuxFrameEncoder enc_a{alloc}, enc_b{alloc};
    data::MuxFrameDecoder dec_a{alloc}, dec_b{alloc};

    SessionPair()
    {
        wire_ab.setBuf(wire_ab_buf, sizeof(wire_ab_buf));
        wire_ba.setBuf(wire_ba_buf, sizeof(wire_ba_buf));
    }
};

class RecordingUARTBus : public uart::IBus {
public:
    explicit RecordingUARTBus(size_t max_write) : max_write_per_call{max_write}
    {
    }

    result_t<size_t> write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src, size_t len) override
    {
        (void)owner;
        last_cfg = cfg;
        ++write_calls;
        const size_t limit = len < max_write_per_call ? len : max_write_per_call;
        size_t done        = 0;
        while (done < limit && src != nullptr && !src->eof()) {
            auto p = src->peek(limit - done);
            if (!p.has_value()) {
                return m5::stl::make_unexpected(p.error());
            }
            if (p.value().size == 0) {
                break;
            }
            bytes.insert(bytes.end(), p.value().data, p.value().data + p.value().size);
            auto adv = src->advance(p.value().size);
            if (!adv.has_value()) {
                return m5::stl::make_unexpected(adv.error());
            }
            done += p.value().size;
        }
        return done;
    }

    size_t max_write_per_call = 0;
    size_t write_calls        = 0;
    uart::AccessConfig last_cfg{};
    std::vector<uint8_t> bytes;
};

class TcpRemoteServerE2E : public ::testing::Test {
protected:
    void SetUp() override
    {
        ASSERT_OK_RESULT(M5_Hal.init());
        ASSERT_OK_RESULT(server.begin(0, "127.0.0.1"));
        ASSERT_NE(server.boundPort(), 0u);
    }

    void TearDown() override
    {
        server.end();
    }

    ConnectOutcome connectOk()
    {
        auto out = connectClient(server);
        EXPECT_EQ(out.err, error::error_t::OK) << "err=" << error::toString(out.err);
        EXPECT_NE(out.conn, nullptr);
        return out;
    }

    remote::BsdTcpRemoteServer server;
};

class ServerPump {
public:
    explicit ServerPump(remote::BsdTcpRemoteServer& server) : _server{server}, _worker{[this]() { run(); }}
    {
    }

    ~ServerPump()
    {
        _running.store(false, std::memory_order_release);
        _worker.join();
    }

private:
    void run()
    {
        while (_running.load(std::memory_order_acquire)) {
            (void)_server.service();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    remote::BsdTcpRemoteServer& _server;
    std::atomic<bool> _running{true};
    std::thread _worker;
};

TEST_F(TcpRemoteServerE2E, TwoClientsConnectAndReceiveCapabilities)
{
    auto a = connectOk();
    auto b = connectOk();

    ASSERT_NE(a.conn, nullptr);
    ASSERT_NE(b.conn, nullptr);
    EXPECT_EQ(a.conn->capabilities().proto_ver, remote::kProtocolVersion);
    EXPECT_EQ(b.conn->capabilities().proto_ver, remote::kProtocolVersion);
    EXPECT_TRUE(a.conn->capabilities().supports_bus_create);
    EXPECT_TRUE(b.conn->capabilities().supports_bus_create);
    EXPECT_EQ(server.connectionCount(), 2u);
}

TEST_F(TcpRemoteServerE2E, I2CBusCreatesSharePhysicalSlotsAcrossClients)
{
    auto a = connectOk();
    auto b = connectOk();
    ASSERT_NE(a.conn, nullptr);
    ASSERT_NE(b.conn, nullptr);

    RemoteTestHal hal_a{&a.conn->backend()};
    RemoteTestHal hal_b{&b.conn->backend()};

    auto a_bus = hal_a.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}});
    ASSERT_TRUE(a_bus.has_value()) << "err=" << error::toString(a_bus.error());
    auto b_bus = hal_b.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}});
    ASSERT_TRUE(b_bus.has_value()) << "err=" << error::toString(b_bus.error());

    EXPECT_EQ(usedI2CSlots(server.physicalPool()), 1u);
    ASSERT_NE(firstUsedI2CSlot(server.physicalPool()), nullptr);
    EXPECT_EQ(firstUsedI2CSlot(server.physicalPool())->refcount, 2u);

    auto b_other = hal_b.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{19}, i2c::Sda{18}});
    ASSERT_TRUE(b_other.has_value()) << "err=" << error::toString(b_other.error());
    EXPECT_EQ(usedI2CSlots(server.physicalPool()), 2u);
    EXPECT_NE(findI2CSlotByRefcount(server.physicalPool(), 2), nullptr);
    EXPECT_NE(findI2CSlotByRefcount(server.physicalPool(), 1), nullptr);
}

TEST_F(TcpRemoteServerE2E, TransportDisconnectReleasesLeasesAndKeepsOtherClientAlive)
{
    auto a = connectOk();
    auto b = connectOk();
    ASSERT_NE(a.conn, nullptr);
    ASSERT_NE(b.conn, nullptr);

    RemoteTestHal hal_a{&a.conn->backend()};
    RemoteTestHal hal_b{&b.conn->backend()};
    auto a_bus = hal_a.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}});
    ASSERT_TRUE(a_bus.has_value()) << "err=" << error::toString(a_bus.error());
    auto b_bus = hal_b.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}});
    ASSERT_TRUE(b_bus.has_value()) << "err=" << error::toString(b_bus.error());
    ASSERT_EQ(firstUsedI2CSlot(server.physicalPool())->refcount, 2u);

    // A transport close must drop only that connection's bus_id leases —
    // the surviving client keeps its shared physical bus.
    a.conn.reset();
    ASSERT_TRUE(serviceUntil(server, 1));
    ASSERT_NE(firstUsedI2CSlot(server.physicalPool()), nullptr);
    EXPECT_EQ(firstUsedI2CSlot(server.physicalPool())->refcount, 1u);

    EXPECT_OK_RESULT(b.conn->session().ping());
    EXPECT_EQ(server.connectionCount(), 1u);
}

TEST_F(TcpRemoteServerE2E, ThirdClientIsRejectedWhenSlotsAreFull)
{
    auto a = connectOk();
    auto b = connectOk();
    ASSERT_NE(a.conn, nullptr);
    ASSERT_NE(b.conn, nullptr);
    ASSERT_EQ(server.connectionCount(), remote::BsdTcpRemoteServer::kMaxConnections);

    auto c = connectClient(server);
    EXPECT_EQ(c.conn, nullptr);
    EXPECT_NE(c.err, error::error_t::OK) << "err=" << error::toString(c.err);
    EXPECT_EQ(server.connectionCount(), remote::BsdTcpRemoteServer::kMaxConnections);
}

TEST_F(TcpRemoteServerE2E, ConnectionExposesRemoteGpioAfterSubscribeRoundTrip)
{
    // GPIO_remote construction on the client side only survives if HelloResp
    // reports has_gpio=true AND the follow-up GpioSubscribe round trip
    // succeeds (constructGPIO() deletes and nulls the object on either
    // failure). Both require the server's ConnectionWiring to hand the
    // handler a non-null gpio/gpio_group and to register
    // setGpioSubscribeHandler on the runner, so this single check exercises
    // that whole per-connection wiring path over the wire.
    server.setConnectionSetupHandler(
        [](void*, remote::Server& srv) -> result_t<void> {
            srv.setGPIOGroup(M5_Hal.Gpio);
            return {};
        },
        nullptr);

    auto a = connectOk();
    ASSERT_NE(a.conn, nullptr);

    EXPECT_TRUE(a.conn->capabilities().has_gpio);
    EXPECT_GT(a.conn->capabilities().gpio_port_count, 0);
    ASSERT_NE(a.conn->gpio(), nullptr);
    EXPECT_EQ(a.conn->gpio()->getPortCount(), 1);
}

TEST_F(TcpRemoteServerE2E, HalReconnectKeepsOldGpioPinStorageClosedAndDistinct)
{
    server.setConnectionSetupHandler(
        [](void*, remote::Server& srv) -> result_t<void> {
            srv.setGPIOGroup(M5_Hal.Gpio);
            return {};
        },
        nullptr);

    char endpoint[80];
    std::snprintf(endpoint, sizeof(endpoint), "127.0.0.1:%u", static_cast<unsigned>(server.boundPort()));
    char remote_endpoint[96];
    std::snprintf(remote_endpoint, sizeof(remote_endpoint), "tcp:%s", endpoint);

    ServerPump pump{server};
    Hal hal;
    remote::DeviceConfig cfg;
    cfg.response_timeout_ms = 500;
    ASSERT_OK_RESULT(hal.connect(remote_endpoint, cfg));
    ASSERT_TRUE(hal.hasRemoteGpio());

    const auto old_number = types::makeGpioNumber(hal.remoteGpioSlot(), 0);
    auto old_result       = hal.Gpio.tryGetPin(old_number);
    ASSERT_TRUE(old_result.has_value());
    auto old_pin                = old_result.value();
    auto* old_port              = old_pin.getPort();
    const bool old_cached_level = old_pin.read();
    auto old_port_result        = hal.Gpio.getPort(hal.remoteGpioSlot(), 0);
    ASSERT_TRUE(old_port_result.has_value());
    auto old_port_access          = old_port_result.value();
    const uint32_t old_port_cache = old_port_access.port->readPort();

    ASSERT_OK_RESULT(hal.connect(remote_endpoint, cfg));
    ASSERT_TRUE(hal.hasRemoteGpio());
    auto new_result = hal.Gpio.tryGetPin(types::makeGpioNumber(hal.remoteGpioSlot(), 0));
    ASSERT_TRUE(new_result.has_value());
    EXPECT_NE(new_result.value().getPort(), old_port);
    auto new_port_result = hal.Gpio.getPort(hal.remoteGpioSlot(), 0);
    ASSERT_TRUE(new_port_result.has_value());
    EXPECT_NE(new_port_result.value().port, old_port_access.port);

    // The old pin is not silently rebound to the new device/session.  Its
    // connection is closed, so void operations are no-ops and reads retain
    // the final cache without dereferencing freed connection storage.
    old_pin.write(!old_cached_level);
    old_pin.setMode(types::GpioMode::Output);
    EXPECT_EQ(old_pin.read(), old_cached_level);
    old_port_access.port->writePort(~old_port_cache, old_port_cache);
    EXPECT_EQ(old_port_access.port->readPort(), old_port_cache);
}

TEST(TcpRemoteStreamSoak, UartWriteCompletesBeyondCreditDriftWindow)
{
    SessionPair pair;
    remote::RemoteSession session{pair.enc_a, pair.dec_a, pair.wire_ba.source(), pair.wire_ab.sink()};
    remote::RemoteServerAdapter adapter{pair.enc_b, pair.dec_b, pair.wire_ab.source(), pair.wire_ba.sink()};
    remote::RemoteServerHandler handler;
    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server{data::DataSpan{scratch, sizeof(scratch)}};

    uart::AccessConfig server_cfg;
    server_cfg.write_timeout_ms = 1000;
    RecordingUARTBus sink{101};
    uart::Accessor server_uart{sink, server_cfg};
    auto reg = server.registerUART(0, server_uart);
    ASSERT_TRUE(reg.has_value()) << "err=" << error::toString(reg.error());

    handler.server = &server;
    adapter.setHandler(&remote::RemoteServerHandler::handler, &handler);
    adapter.setPollHandler(&remote::RemoteServerHandler::poll, &handler);
    session.setPeerPoll(
        [](void* ctx) {
            auto* a = static_cast<remote::RemoteServerAdapter*>(ctx);
            (void)a->service();
        },
        &adapter);

    constexpr size_t kTransferSize = 520 * 1024;
    std::vector<uint8_t> tx_data(kTransferSize);
    for (size_t i = 0; i < tx_data.size(); ++i) {
        tx_data[i] = static_cast<uint8_t>(i & 0xFF);
    }

    data::MemorySource tx_src{tx_data.data(), tx_data.size()};
    uart::IBusConfig bus_cfg;
    uart::Bus_remote bus{session, 0, bus_cfg};
    uart::AccessConfig access_cfg;
    access_cfg.write_timeout_ms = 1000;

    // 520 KiB crosses the historical ~230 KiB credit-drift window while
    // exercising the real Server pending-stream executor, not a hand-rolled
    // drain harness.
    auto written = bus.write(nullptr, access_cfg, &tx_src, tx_data.size());

    ASSERT_TRUE(written.has_value()) << "err=" << error::toString(written.error()) << " consumed=" << sink.bytes.size()
                                     << " remaining_src=" << (tx_src.eof() ? 0 : 1);
    EXPECT_EQ(written.value(), tx_data.size());
    EXPECT_TRUE(tx_src.eof());
    EXPECT_GT(sink.write_calls, 1u);
    ASSERT_EQ(sink.bytes.size(), tx_data.size());
    EXPECT_EQ(0, ::memcmp(sink.bytes.data(), tx_data.data(), tx_data.size()));
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
