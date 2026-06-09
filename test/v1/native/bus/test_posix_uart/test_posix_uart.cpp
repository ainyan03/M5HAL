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
    cfg.baud_rate             = 115200;
    cfg.first_byte_timeout_ms = 1000;
    cfg.inter_byte_timeout_ms = 100;
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
    SUCCEED() << "POSIX UART variant disabled (M5HAL_DISABLE_POSIX or non-POSIX host)";
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
