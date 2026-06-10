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

#include <cstring>
#include <vector>

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

uart::UARTAccessConfig makeConfig()
{
    uart::UARTAccessConfig cfg;
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    EXPECT_EQ(bus.nativeHandle(), pty.slave);

    auto cfg = makeConfig();
    uart::UARTAccessor dev{bus, cfg};
    ASSERT_TRUE(dev.setConfig(cfg).has_value());
}

TEST(PosixUART, WriteReachesMaster)
{
    PtyPair pty;
    ASSERT_TRUE(pty.open());

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::UARTAccessor dev{bus, cfg};
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::UARTAccessor dev{bus, cfg};
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::UARTAccessor dev{bus, cfg};
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg                  = makeConfig();
    cfg.first_byte_timeout_ms = 50;  // keep the test quick
    uart::UARTAccessor dev{bus, cfg};
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::UARTAccessor dev{bus, cfg};
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg                  = makeConfig();
    cfg.first_byte_timeout_ms = 50;  // keep the test quick
    uart::UARTAccessor dev{bus, cfg};
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::UARTAccessor dev{bus, cfg};
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::UARTAccessor dev{bus, cfg};
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::UARTAccessor dev{bus, cfg};
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

    uart::variant::posix::Bus bus;
    ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
    auto cfg = makeConfig();
    uart::UARTAccessor dev{bus, cfg};
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
    auto framed = frame::encodeData({wire, sizeof(wire)}, 0x01, {script, script_len});
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
    auto executed =
        device_runner.run(data::ConstDataSpan{view.kind_body.data + 1, view.kind_body.size - 1});
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
    auto host_run =
        host_runner.run(data::ConstDataSpan{resp_view.kind_body.data + 1, resp_view.kind_body.size - 1});
    ASSERT_TRUE(host_run.has_value());
    auto stored = host_runner.storedData(3);
    ASSERT_EQ(stored.size, sizeof(payload));
    EXPECT_EQ(::memcmp(stored.data, payload, sizeof(payload)), 0);
    EXPECT_TRUE(host_runner.statusReported());
    EXPECT_EQ(host_runner.reportedStatus(), error::error_t::OK);
}

TEST(PosixUART, OpenMissingDeviceReportsIoError)
{
    uart::variant::posix::Bus bus;
    EXPECT_EQ(bus.open("/dev/m5hal-this-device-should-not-exist", 115200), error::error_t::IO_ERROR);
}

TEST(PosixUART, AttachNonTtyReportsIoError)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(::pipe(fds), 0);

    uart::variant::posix::Bus bus;
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
        uart::variant::posix::Bus bus;
        ASSERT_EQ(bus.attach(pty.slave), error::error_t::OK);
        auto cfg      = makeConfig();
        cfg.baud_rate = b;
        uart::UARTAccessor dev{bus, cfg};
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
    using Bus  = uart::variant::posix::Bus;
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
