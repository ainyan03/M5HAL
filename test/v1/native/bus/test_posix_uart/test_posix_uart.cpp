// SPDX-License-Identifier: MIT
//
// Host POSIX UART variant round-trip test. Uses a pseudo-terminal (pty) as a
// loopback "serial cable": the test owns the master end, the M5HAL posix UART
// Bus is attached to the slave end, and bytes are exchanged both ways.
//
// A pty (not socketpair) is required because the variant configures the line
// via termios (tcsetattr), which only succeeds on a tty. We open the pty with
// the portable POSIX primitives (posix_openpt/grantpt/unlockpt/ptsname) rather
// than openpty(3) so the test links without -lutil on both macOS and Linux.

#include <M5HAL_v1.hpp>
#include <gtest/gtest.h>

#include <csignal>

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <fcntl.h>
#include <stdlib.h>
#include <sys/select.h>
#include <termios.h>  // for the B<rate> baud constants checked in AcceptsHighBaudRates
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <thread>
#include <vector>

using ::m5::hal::v1::result_t;

namespace {

namespace uart  = ::m5::hal::v1::uart;
namespace data  = ::m5::hal::v1::data;
namespace error = ::m5::hal::v1::error;

// Block until fd is readable for up to timeout_ms. Returns true if readable.
bool waitReadable(int fd, uint32_t timeout_ms)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv;
    tv.tv_sec  = static_cast<time_t>(timeout_ms / 1000);
    tv.tv_usec = static_cast<suseconds_t>((timeout_ms % 1000) * 1000);
    return ::select(fd + 1, &fds, nullptr, nullptr, &tv) > 0;
}

// RAII pty pair: master kept by the test, slave handed to the Bus.
struct PtyPair {
    int master = -1;
    int slave  = -1;

    bool open()
    {
        master = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master < 0) {
            return false;
        }
        if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
            return false;
        }
        const char* name = ::ptsname(master);
        if (name == nullptr) {
            return false;
        }
        slave = ::open(name, O_RDWR | O_NOCTTY);
        return slave >= 0;
    }

    ~PtyPair()
    {
        if (slave >= 0) {
            ::close(slave);
        }
        if (master >= 0) {
            ::close(master);
        }
    }
};

uart::AccessConfig makeConfig()
{
    uart::AccessConfig cfg;
    cfg.baud_rate = 115200;
    // Short timeouts: a pty delivers in microseconds, and StreamSource's
    // peek blocks up to first_byte/inter_byte when a request cannot be
    // fully satisfied - long values only slow the suite down.
    cfg.first_byte_timeout_ms = 200;
    cfg.inter_byte_timeout_ms = 20;
    cfg.write_timeout_ms      = 1000;
    cfg.data_bits             = 8;
    cfg.stop_bits             = 1;
    cfg.parity                = uart::parity_t::none;
    return cfg;
}

TEST(PosixUART, AttachAndConfigure)
{
    PtyPair pty;
    ASSERT_TRUE(pty.open()) << "failed to open pty pair";

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    EXPECT_EQ(bus.nativeHandle(), pty.slave);

    auto cfg = makeConfig();
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());
}

TEST(PosixUART, WriteReachesMaster)
{
    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    const uint8_t tx[] = {0xDE, 0xAD, 0xBE, 0xEF};
    auto written       = dev.write(data::ConstDataSpan{tx, sizeof(tx)});
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(written.value(), sizeof(tx));

    ASSERT_TRUE(waitReadable(pty.master, 1000)) << "master never became readable";
    uint8_t got[16] = {};
    ssize_t n       = ::read(pty.master, got, sizeof(got));
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof(tx)));
    EXPECT_EQ(::memcmp(got, tx, sizeof(tx)), 0);
}

TEST(PosixUART, ReadReceivesFromMaster)
{
    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    const uint8_t pat[] = {0x01, 0x02, 0x03};
    ASSERT_EQ(::write(pty.master, pat, sizeof(pat)), static_cast<ssize_t>(sizeof(pat)));

    uint8_t rx[16] = {};
    // Request more than will arrive: the read returns the 3 bytes once the
    // inter-byte gap times out.
    auto got = dev.read(data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got.value(), sizeof(pat));
    EXPECT_EQ(::memcmp(rx, pat, sizeof(pat)), 0);
}

TEST(PosixUART, ReadableBytesReportsPending)
{
    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    const uint8_t pat[] = {0x55, 0x66};
    ASSERT_EQ(::write(pty.master, pat, sizeof(pat)), static_cast<ssize_t>(sizeof(pat)));
    ASSERT_TRUE(waitReadable(bus.nativeHandle(), 1000)) << "slave never saw the bytes";

    auto avail = dev.readableBytes();
    ASSERT_TRUE(avail.has_value());
    EXPECT_GE(avail.value(), static_cast<size_t>(1));
}

TEST(PosixUART, ReadTimesOutWhenIdle)
{
    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg                  = makeConfig();
    cfg.first_byte_timeout_ms = 50;  // keep the test quick
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    uint8_t rx[8] = {};
    auto got      = dev.read(data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got.value(), static_cast<size_t>(0));
}

// End-to-end Stream adapter checks: the RX accessor consumed as a
// `Source` (StreamSource) and the TX accessor fed as a `Sink`
// (StreamSink), over a real posix UART Bus on a pty.
TEST(PosixUART, StreamSourcePullsFromRxAccessor)
{
    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    const uint8_t pat[] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
    ASSERT_EQ(::write(pty.master, pat, sizeof(pat)), static_cast<ssize_t>(sizeof(pat)));

    uint8_t scratch[16] = {};
    data::StreamSource src{dev.rx(), data::DataSpan{scratch, sizeof(scratch)}};
    EXPECT_FALSE(src.eof());

    auto peeked = src.peek(sizeof(pat));
    ASSERT_TRUE(peeked.has_value());
    ASSERT_EQ(peeked->size, sizeof(pat));
    EXPECT_EQ(::memcmp(peeked->data, pat, sizeof(pat)), 0);

    // Consume a prefix; the rest stays peekable.
    ASSERT_TRUE(src.advance(4).has_value());
    auto rest = src.peek(8);
    ASSERT_TRUE(rest.has_value());
    ASSERT_EQ(rest->size, static_cast<size_t>(2));
    EXPECT_EQ(rest->data[0], 0xA4);
    EXPECT_EQ(rest->data[1], 0xA5);
}

TEST(PosixUART, StreamSourceTimesOutWhenIdle)
{
    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg                  = makeConfig();
    cfg.first_byte_timeout_ms = 50;  // keep the test quick
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    uint8_t scratch[16] = {};
    data::StreamSource src{dev.rx(), data::DataSpan{scratch, sizeof(scratch)}};

    auto peeked = src.peek(8);
    ASSERT_FALSE(peeked.has_value());
    EXPECT_EQ(peeked.error(), error::error_t::TIMEOUT_ERROR);
    EXPECT_FALSE(src.eof());  // a timeout is recoverable, not end-of-stream
}

TEST(PosixUART, StreamSinkPushesToTxAccessor)
{
    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    uint8_t scratch[16] = {};
    data::StreamSink snk{dev.tx(), data::DataSpan{scratch, sizeof(scratch)}};
    EXPECT_FALSE(snk.closed());

    const uint8_t pat[] = {0x5A, 0x5B, 0x5C, 0x5D};
    auto reserved       = snk.reserve(sizeof(pat));
    ASSERT_TRUE(reserved.has_value());
    ASSERT_GE(reserved->size, sizeof(pat));
    ::memcpy(reserved->data, pat, sizeof(pat));
    ASSERT_TRUE(snk.commit(sizeof(pat)).has_value());

    ASSERT_TRUE(waitReadable(pty.master, 1000)) << "master never became readable";
    uint8_t got[16] = {};
    ssize_t n       = ::read(pty.master, got, sizeof(got));
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof(pat)));
    EXPECT_EQ(::memcmp(got, pat, sizeof(pat)), 0);
}

// End-to-end frame codec over a real posix UART: raw frames written to
// the pty master are pulled out as decoded frames through
// StreamSource + FrameReader, and FrameWriter + StreamSink transmits
// frames the master can decode.
TEST(PosixUART, FrameReaderExtractsFramesFromUART)
{
    namespace frame = ::m5::hal::v1::frame;

    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    // One delimiter + one data frame, written raw by the peer.
    uint8_t wire[64]        = {};
    size_t used             = 0;
    const uint8_t payload[] = {0xAB, 0xCD, 0xEF};
    used += frame::encodeDelimiter({wire, sizeof(wire)}).value();
    used += frame::encodeData({wire + used, sizeof(wire) - used}, 0x11, {payload, sizeof(payload)}).value();
    ASSERT_EQ(::write(pty.master, wire, used), static_cast<ssize_t>(used));

    uint8_t scratch[frame::kMaxWireSize] = {};
    data::StreamSource src{dev.rx(), data::DataSpan{scratch, sizeof(scratch)}};
    frame::FrameReader reader{src};

    frame::View view;
    auto result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::delimiter);

    result = reader.next(view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::data);
    ASSERT_EQ(view.kind_body.size, 1 + sizeof(payload));
    EXPECT_EQ(view.kind_body.data[0], 0x11);
    EXPECT_EQ(::memcmp(view.kind_body.data + 1, payload, sizeof(payload)), 0);
}

TEST(PosixUART, FrameWriterTransmitsFramesOverUART)
{
    namespace frame = ::m5::hal::v1::frame;

    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    uint8_t scratch[frame::kMaxWireSize] = {};
    data::StreamSink snk{dev.tx(), data::DataSpan{scratch, sizeof(scratch)}};
    frame::FrameWriter writer{snk};

    const uint8_t payload[] = {0x12, 0x34};
    auto written            = writer.writeData(0x22, {payload, sizeof(payload)});
    ASSERT_TRUE(written.has_value());

    ASSERT_TRUE(waitReadable(pty.master, 1000)) << "master never became readable";
    uint8_t got[32] = {};
    ssize_t n       = ::read(pty.master, got, sizeof(got));
    ASSERT_EQ(n, static_cast<ssize_t>(written.value()));

    frame::View view;
    auto result = frame::decode({got, static_cast<size_t>(n)}, view);
    ASSERT_EQ(result.status, frame::DecodeStatus::ok);
    EXPECT_EQ(view.kind, frame::Kind::data);
    ASSERT_EQ(view.kind_body.size, 1 + sizeof(payload));
    EXPECT_EQ(view.kind_body.data[0], 0x22);
    EXPECT_EQ(::memcmp(view.kind_body.data + 1, payload, sizeof(payload)), 0);
}

// Mini remote roundtrip: a bytecode script travels host -> device in a
// frame over the pty, the device runner executes it, and the response
// script comes back framed the same way. Exercises the whole lower-
// layer composition (Stream adapters x frame codec x bytecode) on a
// real posix UART, without any mux/transport layer.
TEST(PosixUART, BytecodeRoundtripOverFramedUART)
{
    namespace frame    = ::m5::hal::v1::frame;
    namespace bytecode = ::m5::hal::v1::bytecode;

    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::Bus_posix bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::Accessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());

    // Host side: a command script (store_data keeps the test HW-free),
    // framed as kind=data and written raw to the pty master.
    uint8_t script[32] = {};
    data::MemorySink script_sink{data::DataSpan{script, sizeof(script)}};
    bytecode::BytecodeEncoder enc{script_sink};
    const uint8_t payload[] = {0xAB, 0xCD};
    ASSERT_TRUE(enc.storeData(3, {payload, sizeof(payload)}).has_value());
    ASSERT_TRUE(enc.end().has_value());
    const size_t script_len = 1 + (1 + 1 + sizeof(payload)) + 1;  // lenvar+opcode+id+data, terminator

    uint8_t wire[frame::kMaxWireSize] = {};
    auto framed                       = frame::encodeData({wire, sizeof(wire)}, 0x01, {script, script_len});
    ASSERT_TRUE(framed.has_value());
    ASSERT_EQ(::write(pty.master, wire, framed.value()), static_cast<ssize_t>(framed.value()));

    // Device side: frame in, bytecode run, response framed back out.
    uint8_t rx_scratch[frame::kMaxWireSize] = {};
    data::StreamSource rx_src{dev.rx(), data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    frame::FrameReader reader{rx_src};
    frame::View view;
    auto received = reader.next(view);
    ASSERT_TRUE(received.has_value());
    ASSERT_EQ(received.value().status, frame::DecodeStatus::ok);
    ASSERT_EQ(view.kind, frame::Kind::data);
    ASSERT_GE(view.kind_body.size, 1u);
    const uint8_t stream_id = view.kind_body.data[0];

    bytecode::BytecodeRunner device_runner;
    auto executed = device_runner.run(data::ConstDataSpan{view.kind_body.data + 1, view.kind_body.size - 1});
    ASSERT_TRUE(executed.has_value());

    uint8_t resp_script[64] = {};
    data::MemorySink resp_sink{data::DataSpan{resp_script, sizeof(resp_script)}};
    ASSERT_TRUE(device_runner.writeResponse(resp_sink, error::error_t::OK).has_value());
    // Trim to the encoded length: scan to the terminator the response ends with.
    size_t resp_len = 0;
    {
        bytecode::BytecodeRunner probe;  // length probe via a dry parse
        auto parsed = probe.run(data::ConstDataSpan{resp_script, sizeof(resp_script)});
        ASSERT_TRUE(parsed.has_value());
        resp_len = parsed.value();
    }

    uint8_t tx_scratch[frame::kMaxWireSize] = {};
    data::StreamSink tx_snk{dev.tx(), data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
    frame::FrameWriter writer{tx_snk};
    ASSERT_TRUE(writer.writeData(stream_id, {resp_script, resp_len}).has_value());

    // Host side: decode the response frame and execute the response script.
    ASSERT_TRUE(waitReadable(pty.master, 1000)) << "no framed response arrived";
    uint8_t got[frame::kMaxWireSize] = {};
    ssize_t n                        = ::read(pty.master, got, sizeof(got));
    ASSERT_GT(n, 0);
    frame::View resp_view;
    auto decoded = frame::decode({got, static_cast<size_t>(n)}, resp_view);
    ASSERT_EQ(decoded.status, frame::DecodeStatus::ok);
    ASSERT_EQ(resp_view.kind, frame::Kind::data);
    EXPECT_EQ(resp_view.kind_body.data[0], stream_id);

    bytecode::BytecodeRunner host_runner;
    auto host_run = host_runner.run(data::ConstDataSpan{resp_view.kind_body.data + 1, resp_view.kind_body.size - 1});
    ASSERT_TRUE(host_run.has_value());
    auto stored = host_runner.storedData(3);
    ASSERT_EQ(stored.size, sizeof(payload));
    EXPECT_EQ(::memcmp(stored.data, payload, sizeof(payload)), 0);
    EXPECT_TRUE(host_runner.statusReported());
    EXPECT_EQ(host_runner.reportedStatus(), error::error_t::OK);
}

// Full remote-bus end-to-end over a real posix UART: RemoteSession +
// RemoteI2CBus proxy on the host (pty master, raw fd), Server +
// RemoteServerService behind the posix UART variant on the device (pty
// slave). Single-threaded: the host-side reader pumps the device
// service once before each blocking read, the same interleave the
// in-memory loopback tests use.
TEST(PosixUART, RemoteSessionEndToEndOverPty)
{
    namespace bus     = ::m5::hal::v1::bus;
    namespace frame   = ::m5::hal::v1::frame;
    namespace i2c     = ::m5::hal::v1::i2c;
    namespace remote  = ::m5::hal::v1::remote;
    namespace service = ::m5::hal::v1::service;
    namespace types   = ::m5::hal::v1::types;

    // Minimal device-side I2C hardware stand-in: answers reads with a
    // pattern and records the marshalled target address.
    struct StubIBus : public i2c::IBus {
        uint16_t last_addr    = 0;
        error::error_t result = error::error_t::OK;

        result_t<void> init(const i2c::IBusConfig&)
        {
            return {};
        }
        result_t<size_t> transfer(bus::IAccessor*, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc,
                                  data::Source* tx, data::Sink* rx) override
        {
            last_addr = cfg.i2c_addr;
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

    PtyPair pty;
    ASSERT_TRUE(pty.open());

    // --- device endpoint (pty slave via the posix UART variant) ---
    uart::Bus_posix dev_bus;
    ASSERT_EQ(dev_bus.attach(pty.slave), error::error_t::OK);
    auto dev_cfg = makeConfig();
    // Tiny first-byte timeout: the server poll must not stall when idle
    // (spec/design/remote.md, server execution model).
    dev_cfg.first_byte_timeout_ms = 2;
    dev_cfg.inter_byte_timeout_ms = 2;
    uart::Accessor dev{dev_bus, dev_cfg};
    ASSERT_TRUE(dev.setConfig(dev_cfg).has_value());

    uint8_t dev_rx_scratch[frame::kMaxWireSize] = {};
    uint8_t dev_tx_scratch[frame::kMaxWireSize] = {};
    data::StreamSource dev_src{dev.rx(), data::DataSpan{dev_rx_scratch, sizeof(dev_rx_scratch)}};
    data::StreamSink dev_snk{dev.tx(), data::DataSpan{dev_tx_scratch, sizeof(dev_tx_scratch)}};

    StubIBus stub_bus;
    i2c::MasterAccessConfig stub_acc_cfg;
    i2c::MasterAccessor stub_acc{stub_bus, stub_acc_cfg};

    uint8_t server_scratch[remote::kMaxMessageSize] = {};
    remote::Server server{data::DataSpan{server_scratch, sizeof(server_scratch)}};
    ASSERT_TRUE(server.registerI2C(0, stub_acc).has_value());
    remote::RemoteServerService server_service{server, dev_src, dev_snk};

    // --- host endpoint (pty master, raw fd + device pump) ---
    struct MasterReader : public data::StreamReader {
        int fd                              = -1;
        remote::RemoteServerService* device = nullptr;

        result_t<size_t> read(data::DataSpan dst) override
        {
            device->service(service::ServiceContext{});  // let the peer make progress
            if (!waitReadable(fd, 100)) {
                return size_t{0};
            }
            ssize_t n = ::read(fd, dst.data, dst.size);
            return n < 0 ? result_t<size_t>{m5::stl::make_unexpected(error::error_t::IO_ERROR)}
                         : result_t<size_t>{static_cast<size_t>(n)};
        }
        result_t<size_t> readableBytes(void) override
        {
            return waitReadable(fd, 0) ? size_t{1} : size_t{0};
        }
    };
    struct MasterWriter : public data::StreamWriter {
        int fd = -1;
        result_t<size_t> write(data::ConstDataSpan src) override
        {
            ssize_t n = ::write(fd, src.data, src.size);
            return n < 0 ? result_t<size_t>{m5::stl::make_unexpected(error::error_t::IO_ERROR)}
                         : result_t<size_t>{static_cast<size_t>(n)};
        }
    };

    MasterReader host_reader;
    host_reader.fd     = pty.master;
    host_reader.device = &server_service;
    MasterWriter host_writer;
    host_writer.fd = pty.master;

    uint8_t host_rx_scratch[frame::kMaxWireSize] = {};
    uint8_t host_tx_scratch[frame::kMaxWireSize] = {};
    data::StreamSource host_src{host_reader, data::DataSpan{host_rx_scratch, sizeof(host_rx_scratch)}};
    data::StreamSink host_snk{host_writer, data::DataSpan{host_tx_scratch, sizeof(host_tx_scratch)}};

    remote::RemoteSession session{host_src, host_snk};

    // hello: the capability list names the registered I2C bus.
    auto caps = session.hello();
    ASSERT_TRUE(caps.has_value());
    EXPECT_EQ(caps.value().proto_ver, remote::kProtocolVersion);
    ASSERT_EQ(caps.value().bus_count, 1u);
    EXPECT_EQ(caps.value().buses[0].kind, types::bus_kind_t::I2C);
    EXPECT_EQ(caps.value().buses[0].bus_id, 0);

    // The proxy behaves like a local bus: register read over the wire.
    remote::RemoteI2CBus proxy{session, 0};
    i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr = 0x68;
    i2c::MasterAccessor acc{proxy, acc_cfg};

    uint8_t rx[4] = {};
    auto r        = acc.readRegister(uint8_t{0x75}, rx, sizeof(rx));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(stub_bus.last_addr, 0x68);
    for (size_t i = 0; i < sizeof(rx); ++i) {
        EXPECT_EQ(rx[i], static_cast<uint8_t>(0x60 + i));
    }

    // A remote bus error folds into the local expected error path.
    stub_bus.result = error::error_t::I2C_NO_ACK;
    auto p          = acc.probe();
    ASSERT_FALSE(p.has_value());
    EXPECT_EQ(p.error(), error::error_t::I2C_NO_ACK);
    stub_bus.result = error::error_t::OK;
    EXPECT_TRUE(acc.probe().has_value());
}

// connectRemoteSerial (remote_connect.hpp), explicit-path route over a
// pty: the host endpoint opens the pty slave BY PATH (exactly what the
// utility does with a real adapter), while a background thread serves a
// remote Server on the master end. The auto-discovery route is
// deliberately NOT exercised here - it would open the test machine's
// real serial ports and DTR-reset whatever boards are attached; its
// building blocks (listSerialPorts, open, hello) are covered separately.
TEST(PosixUART, ConnectRemoteSerialExplicitPathOverPty)
{
    namespace bus        = ::m5::hal::v1::bus;
    namespace frame      = ::m5::hal::v1::frame;
    namespace i2c        = ::m5::hal::v1::i2c;
    namespace remote     = ::m5::hal::v1::remote;
    namespace service    = ::m5::hal::v1::service;
    namespace posix_uart = ::m5::variants::frameworks::posix::hal::v1::uart;

    PtyPair pty;
    ASSERT_TRUE(pty.open());
    char slave_path[128] = {};
    {
        const char* name = ::ptsname(pty.master);
        ASSERT_NE(name, nullptr);
        ::snprintf(slave_path, sizeof(slave_path), "%s", name);
    }

    // Device endpoint on the master fd (raw, no termios).
    struct FdReader : public data::StreamReader {
        int fd = -1;
        result_t<size_t> read(data::DataSpan dst) override
        {
            if (!waitReadable(fd, 5)) {
                return size_t{0};
            }
            ssize_t n = ::read(fd, dst.data, dst.size);
            return n < 0 ? result_t<size_t>{m5::stl::make_unexpected(error::error_t::IO_ERROR)}
                         : result_t<size_t>{static_cast<size_t>(n)};
        }
        result_t<size_t> readableBytes(void) override
        {
            return waitReadable(fd, 0) ? size_t{1} : size_t{0};
        }
    };
    struct FdWriter : public data::StreamWriter {
        int fd = -1;
        result_t<size_t> write(data::ConstDataSpan src) override
        {
            ssize_t n = ::write(fd, src.data, src.size);
            return n < 0 ? result_t<size_t>{m5::stl::make_unexpected(error::error_t::IO_ERROR)}
                         : result_t<size_t>{static_cast<size_t>(n)};
        }
    };
    struct StubIBus : public i2c::IBus {
        result_t<void> init(const i2c::IBusConfig&)
        {
            return {};
        }
        result_t<size_t> transfer(bus::IAccessor*, const i2c::MasterAccessConfig&, const i2c::TransferDesc&,
                                  data::Source*, data::Sink*) override
        {
            return size_t{0};  // data phase only
        }
    };

    FdReader dev_reader;
    dev_reader.fd = pty.master;
    FdWriter dev_writer;
    dev_writer.fd                               = pty.master;
    uint8_t dev_rx_scratch[frame::kMaxWireSize] = {};
    uint8_t dev_tx_scratch[frame::kMaxWireSize] = {};
    data::StreamSource dev_src{dev_reader, data::DataSpan{dev_rx_scratch, sizeof(dev_rx_scratch)}};
    data::StreamSink dev_snk{dev_writer, data::DataSpan{dev_tx_scratch, sizeof(dev_tx_scratch)}};

    StubIBus stub_bus;
    i2c::MasterAccessConfig stub_acc_cfg;
    i2c::MasterAccessor stub_acc{stub_bus, stub_acc_cfg};
    uint8_t server_scratch[remote::kMaxMessageSize] = {};
    remote::Server server{data::DataSpan{server_scratch, sizeof(server_scratch)}};
    ASSERT_TRUE(server.registerI2C(0, stub_acc).has_value());
    remote::RemoteServerService server_service{server, dev_src, dev_snk};

    std::atomic<bool> stop{false};
    std::thread dev_thread([&] {
        // Boot noise first: a freshly reset ESP32 streams its ROM log
        // before the sketch serves. The connect utility must flush this
        // (per-attempt) instead of letting the frame reader resync
        // through it and swallow the real hello_resp — the field
        // failure this reproduces.
        const char boot_noise[] =
            "ets Jul 29 2019 12:21:46\r\n\r\nrst:0x1 (POWERON_RESET),boot:0x17 "
            "(SPI_FAST_FLASH_BOOT)\r\nload:0x3fff0030,len:1184\r\nentry 0x400805e4\r\n";
        (void)::write(pty.master, boot_noise, sizeof(boot_noise) - 1);
        while (!stop.load()) {
            server_service.service(service::ServiceContext{});
        }
    });

    posix_uart::SerialRemoteEndpoint ep{115200};
    posix_uart::ConnectOptions opt;
    opt.path             = slave_path;
    opt.hello_timeout_ms = 300;
    auto caps            = posix_uart::connectRemoteSerial(ep, opt);

    stop.store(true);
    dev_thread.join();

    ASSERT_TRUE(caps.has_value());
    EXPECT_STREQ(ep.devicePath(), slave_path);
    EXPECT_EQ(caps.value().proto_ver, remote::kProtocolVersion);
    ASSERT_EQ(caps.value().bus_count, 1u);
    // The probe timeout was restored to the configured default.
    EXPECT_EQ(ep.link.session().responseTimeoutMs(), 1000u);
}

TEST(PosixUART, ConnectRemoteSerialMissingPathReportsIoError)
{
    namespace posix_uart = ::m5::variants::frameworks::posix::hal::v1::uart;
    posix_uart::SerialRemoteEndpoint ep;
    posix_uart::ConnectOptions opt;
    opt.path            = "/dev/m5hal-no-such-port";
    opt.strong_attempts = 1;
    auto r              = posix_uart::connectRemoteSerial(ep, opt);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error::error_t::IO_ERROR);
    EXPECT_STREQ(ep.devicePath(), "");
}

// Serial-port enumeration (ports.hpp): the pure ranking heuristic per
// platform, and a smoke run of the real /dev scan (its content is
// host-dependent, so only the invariants are asserted).
TEST(PosixUART, SerialPortNameRanking)
{
    namespace posix_uart = ::m5::variants::frameworks::posix::hal::v1::uart;
#if defined(__APPLE__)
    EXPECT_EQ(posix_uart::rankSerialPortName("cu.usbserial-1410"), 0);
    EXPECT_EQ(posix_uart::rankSerialPortName("cu.usbmodem401"), 0);
    EXPECT_EQ(posix_uart::rankSerialPortName("cu.SLAB_USBtoUART"), 1);
    EXPECT_EQ(posix_uart::rankSerialPortName("cu.Bluetooth-Incoming-Port"), 2);
    EXPECT_EQ(posix_uart::rankSerialPortName("cu.debug-console"), 2);
    EXPECT_EQ(posix_uart::rankSerialPortName("tty.usbserial-1410"), -1);  // dial-in side: not a candidate
    EXPECT_EQ(posix_uart::rankSerialPortName("ttyS0"), -1);
#else
    EXPECT_EQ(posix_uart::rankSerialPortName("ttyUSB0"), 0);
    EXPECT_EQ(posix_uart::rankSerialPortName("ttyACM3"), 0);
    EXPECT_EQ(posix_uart::rankSerialPortName("ttyS0"), -1);
    EXPECT_EQ(posix_uart::rankSerialPortName("cu.usbserial-1410"), -1);
#endif
    EXPECT_EQ(posix_uart::rankSerialPortName(nullptr), -1);
}

TEST(PosixUART, ListSerialPortsSmoke)
{
    namespace posix_uart = ::m5::variants::frameworks::posix::hal::v1::uart;
    posix_uart::SerialPortInfo ports[8];
    const size_t n = posix_uart::listSerialPorts(ports, sizeof(ports) / sizeof(ports[0]));
    ASSERT_LE(n, sizeof(ports) / sizeof(ports[0]));
    for (size_t i = 0; i < n; ++i) {
        EXPECT_NE(ports[i].path[0], '\0');
        if (i > 0) {
            EXPECT_LE(ports[i - 1].rank, ports[i].rank);  // best candidates first
        }
    }
    EXPECT_EQ(posix_uart::listSerialPorts(nullptr, 4), 0u);
    EXPECT_EQ(posix_uart::listSerialPorts(ports, 0), 0u);
}

TEST(PosixUART, OpenMissingDeviceReportsIoError)
{
    uart::Bus_posix bus;
    EXPECT_EQ(bus.open("/dev/m5hal-this-device-should-not-exist", 115200), error::error_t::IO_ERROR);
}

TEST(PosixUART, AttachNonTtyReportsIoError)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(::pipe(fds), 0);

    uart::Bus_posix bus;
    EXPECT_EQ(bus.attach(fds[0]), error::error_t::IO_ERROR);

    ::close(fds[0]);
    ::close(fds[1]);
}

// High baud rates (>1 Mbaud) where the libc provides the B* constant — Linux
// glibc/musl do, macOS does not. A write forces applyConfig(baud), which runs
// baudToSpeed()+tcsetattr; on a pty the baud is accepted but otherwise ignored.
TEST(PosixUART, AcceptsHighBaudRates)
{
    std::vector<uint32_t> bauds;
#ifdef B1000000
    bauds.push_back(1000000);
#endif
#ifdef B1500000
    bauds.push_back(1500000);
#endif
#ifdef B2000000
    bauds.push_back(2000000);
#endif
#ifdef B3000000
    bauds.push_back(3000000);
#endif
    if (bauds.empty()) {
        GTEST_SKIP() << "this libc defines no >1 Mbaud B* constants (e.g. macOS)";
    }

    const uint8_t tx[] = {0x11, 0x22, 0x33};
    for (uint32_t b : bauds) {
        PtyPair pty;
        ASSERT_TRUE(pty.open());
        uart::Bus_posix bus;
        ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
        auto cfg      = makeConfig();
        cfg.baud_rate = b;
        uart::Accessor dev{bus, cfg};
        // write() triggers applyConfig(b) -> baudToSpeed(b) must succeed.
        EXPECT_TRUE(dev.write(data::ConstDataSpan{tx, sizeof(tx)}).has_value()) << "baud " << b;
    }
}

// White-box check of the baud -> termios B* table (Bus::baudToSpeed). A pty
// ignores the line speed, so the behavioral tests above cannot catch a wrong
// constant (e.g. mapping 1.5M to B2000000); this does. Each high rate is
// #ifdef-guarded by whether this libc defines its constant (Linux yes, macOS
// no — there IOSSIOSPEED handles it), so the test is correct on both.
TEST(PosixUART, BaudTableMapsKnownConstants)
{
    using Bus  = uart::Bus_posix;
    uint32_t s = 0;

    EXPECT_TRUE(Bus::baudToSpeed(9600, s));
    EXPECT_EQ(s, static_cast<uint32_t>(B9600));
    EXPECT_TRUE(Bus::baudToSpeed(115200, s));
    EXPECT_EQ(s, static_cast<uint32_t>(B115200));
    EXPECT_TRUE(Bus::baudToSpeed(230400, s));
    EXPECT_EQ(s, static_cast<uint32_t>(B230400));

#ifdef B921600
    EXPECT_TRUE(Bus::baudToSpeed(921600, s));
    EXPECT_EQ(s, static_cast<uint32_t>(B921600));
#else
    EXPECT_FALSE(Bus::baudToSpeed(921600, s));
#endif
#ifdef B1500000
    EXPECT_TRUE(Bus::baudToSpeed(1500000, s));
    EXPECT_EQ(s, static_cast<uint32_t>(B1500000));
#else
    EXPECT_FALSE(Bus::baudToSpeed(1500000, s));
#endif
#ifdef B3000000
    EXPECT_TRUE(Bus::baudToSpeed(3000000, s));
    EXPECT_EQ(s, static_cast<uint32_t>(B3000000));
#else
    EXPECT_FALSE(Bus::baudToSpeed(3000000, s));
#endif

    // A rate no libc has a B* constant for is never mapped here (macOS would set
    // it via IOSSIOSPEED instead).
    EXPECT_FALSE(Bus::baudToSpeed(12345, s));
}

}  // namespace

#else  // M5HAL_FRAMEWORK_HAS_POSIX

TEST(PosixUART, SkippedWhenDisabled)
{
    SUCCEED() << "POSIX UART variant disabled (M5HAL_CONFIG_POSIX_UART=0 or non-POSIX host)";
}

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

int main(int argc, char** argv)
{
    // Closing a pty end can raise SIGHUP on the process; ignore it so the
    // test run is not killed during teardown.
    ::signal(SIGHUP, SIG_IGN);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
