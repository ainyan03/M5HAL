// SPDX-License-Identifier: MIT
//
// Host TCP transport tests (posix variant, hal/tcp/). TCP is a remote-bus
// TRANSPORT: TcpStream implements the stream seam directly, so everything
// from the frame layer up is the code already covered by test_remote — what
// needs proving here is the seam itself (timeouts, hang-up reporting) and
// the establishment utility (connectRemoteTcp), end to end over a localhost
// loopback with a real Server on the accepted side.
//
// Every listener binds 127.0.0.1 port 0 (ephemeral) and reads the port back,
// so parallel test runs never collide.

#include <M5HAL_v1.hpp>
#include <gtest/gtest.h>

#include <csignal>

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <atomic>
#include <cstdio>
#include <cstring>
#include <thread>

namespace {

namespace bus     = ::m5::hal::v1::bus;
namespace data    = ::m5::hal::v1::data;
namespace error   = ::m5::hal::v1::error;
namespace frame   = ::m5::hal::v1::frame;
namespace i2c     = ::m5::hal::v1::i2c;
namespace remote  = ::m5::hal::v1::remote;
namespace service = ::m5::hal::v1::service;
namespace tcp     = ::m5::variants::frameworks::posix::hal::v1::tcp;
namespace types   = ::m5::hal::v1::types;

// Connected localhost pair: `client` dialed in, `server` is the accepted end.
struct TcpPair {
    tcp::TcpListener listener;
    tcp::TcpStream client;
    tcp::TcpStream server;

    bool open()
    {
        if (listener.listen(0, "127.0.0.1") != error::error_t::OK) {
            return false;
        }
        if (client.connect("127.0.0.1", listener.boundPort(), 2000) != error::error_t::OK) {
            return false;
        }
        const int fd = listener.accept(2000);
        return fd >= 0 && server.attach(fd) == error::error_t::OK;
    }
};

TEST(PosixTCP, ListenerBindsEphemeralPortOnLoopback)
{
    tcp::TcpListener listener;
    ASSERT_EQ(listener.listen(0, "127.0.0.1"), error::error_t::OK);
    EXPECT_TRUE(listener.isOpen());
    EXPECT_NE(listener.boundPort(), 0);

    // Nobody dials in: accept paces by its own timeout and gives up.
    EXPECT_LT(listener.accept(10), 0);

    listener.close();
    EXPECT_FALSE(listener.isOpen());
    EXPECT_EQ(listener.boundPort(), 0);
}

TEST(PosixTCP, StreamRoundTripBothDirections)
{
    TcpPair pair;
    ASSERT_TRUE(pair.open());

    const uint8_t ping[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    auto wrote           = pair.client.write(data::ConstDataSpan{ping, sizeof(ping)});
    ASSERT_TRUE(wrote.has_value());
    EXPECT_EQ(wrote.value(), sizeof(ping));

    uint8_t rx[16] = {};
    auto got       = pair.server.read(data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(got.has_value());
    ASSERT_EQ(got.value(), sizeof(ping));
    EXPECT_EQ(0, ::memcmp(rx, ping, sizeof(ping)));

    const uint8_t pong[] = {0xA0, 0xA1};
    ASSERT_TRUE(pair.server.write(data::ConstDataSpan{pong, sizeof(pong)}).has_value());
    auto back = pair.client.read(data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(back.has_value());
    ASSERT_EQ(back.value(), sizeof(pong));
    EXPECT_EQ(0, ::memcmp(rx, pong, sizeof(pong)));
}

TEST(PosixTCP, ReadableBytesReportsPending)
{
    TcpPair pair;
    ASSERT_TRUE(pair.open());

    auto idle = pair.server.readableBytes();
    ASSERT_TRUE(idle.has_value());
    EXPECT_EQ(idle.value(), 0u);

    const uint8_t payload[] = {0x11, 0x22, 0x33};
    ASSERT_TRUE(pair.client.write(data::ConstDataSpan{payload, sizeof(payload)}).has_value());
    // Loopback delivery is asynchronous by a few microseconds; a paced
    // read both waits for and consumes the bytes, then the count is 0.
    uint8_t rx[8] = {};
    auto got      = pair.server.read(data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got.value(), sizeof(payload));
    auto drained = pair.server.readableBytes();
    ASSERT_TRUE(drained.has_value());
    EXPECT_EQ(drained.value(), 0u);
}

TEST(PosixTCP, ReadTimesOutWhenIdle)
{
    TcpPair pair;
    ASSERT_TRUE(pair.open());
    pair.server.read_timeout_ms = 20;

    uint8_t rx[8] = {};
    auto got      = pair.server.read(data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(got.has_value());  // a timeout is NOT an error...
    EXPECT_EQ(got.value(), 0u);    // ...just zero bytes (StreamReader contract)
}

TEST(PosixTCP, ReadReportsClosedOnPeerHangup)
{
    TcpPair pair;
    ASSERT_TRUE(pair.open());

    // In-flight bytes survive the close (orderly shutdown delivers the
    // queue first)...
    const uint8_t last_words[] = {0x42};
    ASSERT_TRUE(pair.client.write(data::ConstDataSpan{last_words, sizeof(last_words)}).has_value());
    pair.client.close();

    uint8_t rx[8] = {};
    auto got      = pair.server.read(data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got.value(), sizeof(last_words));

    // ...then the hang-up is reported as CLOSED — the distinction a
    // serial line cannot make — and it latches.
    auto hangup = pair.server.read(data::DataSpan{rx, sizeof(rx)});
    ASSERT_FALSE(hangup.has_value());
    EXPECT_EQ(hangup.error(), error::error_t::CLOSED);
    EXPECT_FALSE(pair.server.isOpen());
    auto again = pair.server.read(data::DataSpan{rx, sizeof(rx)});
    ASSERT_FALSE(again.has_value());
    EXPECT_EQ(again.error(), error::error_t::CLOSED);
}

TEST(PosixTCP, ConnectRefusedReportsIoError)
{
    // Bind an ephemeral port, then close it: connecting to it afterwards
    // is a deterministic ECONNREFUSED on loopback.
    uint16_t dead_port = 0;
    {
        tcp::TcpListener listener;
        ASSERT_EQ(listener.listen(0, "127.0.0.1"), error::error_t::OK);
        dead_port = listener.boundPort();
    }
    tcp::TcpStream stream;
    EXPECT_EQ(stream.connect("127.0.0.1", dead_port, 2000), error::error_t::IO_ERROR);
    EXPECT_FALSE(stream.isOpen());
}

// Device-side I2C hardware stand-in: answers reads with a pattern and
// records the marshalled target address (same stub as the pty E2E).
struct StubI2CBus : public i2c::I2CBus {
    uint16_t last_addr    = 0;
    error::error_t result = error::error_t::OK;

    m5::stl::expected<void, error::error_t> init(const bus::BusConfig&) override
    {
        return {};
    }
    m5::stl::expected<size_t, error::error_t> transfer(bus::Accessor*, const i2c::I2CMasterAccessConfig& cfg,
                                                       const i2c::TransferDesc& desc, data::Source* tx,
                                                       data::Sink* rx) override
    {
        last_addr = cfg.i2c_addr;
        if (error::isError(result)) {
            return m5::stl::make_unexpected(result);
        }
        size_t total = 0;  // data phase only (S16 D4)
        if (tx != nullptr) {
            while (!tx->eof()) {
                auto p = tx->peek(64);
                if (!p.has_value() || p.value().size == 0) {
                    break;
                }
                total += p.value().size;
                (void)tx->advance(p.value().size);
            }
        }
        if (rx != nullptr) {
            auto rsv = rx->reserve(SIZE_MAX);
            if (!rsv.has_value()) {
                return m5::stl::make_unexpected(rsv.error());
            }
            for (size_t i = 0; i < rsv.value().size; ++i) {
                rsv.value().data[i] = static_cast<uint8_t>(0x60 + i);
            }
            (void)rx->commit(rsv.value().size);
            total += rsv.value().size;
        }
        return total;
    }
};

// connectRemoteTcp end to end over a localhost loopback: a real remote
// Server (I2C proxy registered) serves the accepted end of the socket in a
// background thread, the host establishes with the utility and then runs an
// RPC through the proxy — hello and request/response over actual TCP.
TEST(PosixTCP, ConnectRemoteTcpEndToEndOverLoopback)
{
    tcp::TcpListener listener;
    ASSERT_EQ(listener.listen(0, "127.0.0.1"), error::error_t::OK);

    // Device endpoint. The stream starts detached; the serve thread
    // attaches the accepted fd into it (the adapters hold references).
    tcp::TcpStream dev_stream;
    // Tiny first-byte timeout: the server poll must not stall when idle
    // (spec/design/remote.md, server execution model).
    dev_stream.read_timeout_ms                  = 2;
    uint8_t dev_rx_scratch[frame::kMaxWireSize] = {};
    uint8_t dev_tx_scratch[frame::kMaxWireSize] = {};
    data::StreamSource dev_src{dev_stream, data::DataSpan{dev_rx_scratch, sizeof(dev_rx_scratch)}};
    data::StreamSink dev_snk{dev_stream, data::DataSpan{dev_tx_scratch, sizeof(dev_tx_scratch)}};

    StubI2CBus stub_bus;
    i2c::I2CMasterAccessConfig stub_acc_cfg;
    i2c::I2CMasterAccessor stub_acc{stub_bus, stub_acc_cfg};
    uint8_t server_scratch[remote::kMaxMessageSize] = {};
    remote::Server server{data::DataSpan{server_scratch, sizeof(server_scratch)}};
    ASSERT_TRUE(server.registerI2C(0, stub_acc).has_value());
    remote::RemoteServerService server_service{server, dev_src, dev_snk};

    std::atomic<bool> stop{false};
    std::thread dev_thread([&] {
        const int fd = listener.accept(5000);
        if (fd < 0 || dev_stream.attach(fd) != error::error_t::OK) {
            return;
        }
        while (!stop.load()) {
            server_service.service(service::ServiceContext{});
        }
    });

    char endpoint[32];
    ::snprintf(endpoint, sizeof(endpoint), "127.0.0.1:%u", static_cast<unsigned>(listener.boundPort()));
    tcp::TcpRemoteEndpoint ep;
    tcp::TcpConnectOptions opt;
    opt.hello_timeout_ms = 300;
    auto caps            = tcp::connectRemoteTcp(ep, endpoint, opt);

    ASSERT_TRUE(caps.has_value()) << "hello over TCP failed";
    EXPECT_STREQ(ep.peer(), endpoint);
    EXPECT_EQ(caps.value().proto_ver, remote::kProtocolVersion);
    ASSERT_EQ(caps.value().bus_count, 1u);
    EXPECT_EQ(caps.value().buses[0].kind, types::bus_kind_t::I2C);
    // The probe timeout was restored to the configured default.
    EXPECT_EQ(ep.link.session().responseTimeoutMs(), 1000u);

    // The proxy behaves like a local bus: register read over the wire.
    remote::RemoteI2CBus proxy{ep.link.session(), 0};
    i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr = 0x68;
    i2c::I2CMasterAccessor acc{proxy, acc_cfg};
    uint8_t rx[4] = {};
    auto r        = acc.readRegister(uint8_t{0x75}, rx, sizeof(rx));

    stop.store(true);
    dev_thread.join();

    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(stub_bus.last_addr, 0x68);
    for (size_t i = 0; i < sizeof(rx); ++i) {
        EXPECT_EQ(rx[i], static_cast<uint8_t>(0x60 + i));
    }
}

TEST(PosixTCP, ConnectRemoteTcpSilentPeerReportsTimeout)
{
    // The listener never serves (the connect itself completes against the
    // listen backlog): reached-but-mute is TIMEOUT_ERROR, distinct from
    // unreachable (IO_ERROR), and the socket is closed again.
    tcp::TcpListener listener;
    ASSERT_EQ(listener.listen(0, "127.0.0.1"), error::error_t::OK);

    char endpoint[32];
    ::snprintf(endpoint, sizeof(endpoint), "127.0.0.1:%u", static_cast<unsigned>(listener.boundPort()));
    tcp::TcpRemoteEndpoint ep;
    tcp::TcpConnectOptions opt;
    opt.hello_timeout_ms = 50;
    opt.attempts         = 1;
    auto r               = tcp::connectRemoteTcp(ep, endpoint, opt);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error::error_t::TIMEOUT_ERROR);
    EXPECT_FALSE(ep.stream.isOpen());
    EXPECT_STREQ(ep.peer(), "");
}

TEST(PosixTCP, ConnectRemoteTcpUnreachableReportsIoError)
{
    uint16_t dead_port = 0;
    {
        tcp::TcpListener listener;
        ASSERT_EQ(listener.listen(0, "127.0.0.1"), error::error_t::OK);
        dead_port = listener.boundPort();
    }
    char endpoint[32];
    ::snprintf(endpoint, sizeof(endpoint), "127.0.0.1:%u", static_cast<unsigned>(dead_port));
    tcp::TcpRemoteEndpoint ep;
    auto r = tcp::connectRemoteTcp(ep, endpoint);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error::error_t::IO_ERROR);
    EXPECT_STREQ(ep.peer(), "");
}

TEST(PosixTCP, ConnectRemoteTcpRejectsMalformedEndpoints)
{
    tcp::TcpRemoteEndpoint ep;
    const char* malformed[] = {
        nullptr,         // no endpoint at all
        "localhost",     // no port
        "localhost:",    // empty port
        ":5555",         // empty host
        "host:0",        // port 0 is not connectable
        "host:65536",    // port out of range
        "host:12ab",     // trailing junk after the port
        "[::1]",         // bracket form without a port
        "[::1]5555",     // bracket form missing the separator
    };
    for (const char* endpoint : malformed) {
        auto r = tcp::connectRemoteTcp(ep, endpoint);
        ASSERT_FALSE(r.has_value()) << (endpoint ? endpoint : "(null)");
        EXPECT_EQ(r.error(), error::error_t::INVALID_ARGUMENT) << (endpoint ? endpoint : "(null)");
    }

    // A well-formed bracketed IPv6 literal passes the parse: the failure
    // (nothing listens on [::1]:1) is reported as unreachable instead.
    auto v6 = tcp::connectRemoteTcp(ep, "[::1]:1", tcp::TcpConnectOptions{});
    ASSERT_FALSE(v6.has_value());
    EXPECT_EQ(v6.error(), error::error_t::IO_ERROR);
}

}  // namespace

#else  // M5HAL_FRAMEWORK_HAS_POSIX

TEST(PosixTCP, SkippedWhenDisabled)
{
    SUCCEED() << "posix variant disabled (M5HAL_CONFIG_POSIX_UART=0 or non-POSIX host)";
}

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

int main(int argc, char** argv)
{
    // Belt and braces: writes to a hung-up socket are already masked per
    // send / per socket, but a stray SIGPIPE must not kill the test run.
    ::signal(SIGPIPE, SIG_IGN);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
