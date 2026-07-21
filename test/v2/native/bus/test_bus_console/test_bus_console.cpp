// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "m5_hal/hal/v2/uart/bus_console.hpp"
#include "support/gtest_watchdog.hpp"

#if M5HAL_FRAMEWORK_HAS_POSIX && !defined(_WIN32)

#include <unistd.h>

#include <array>
#include <cstdio>

namespace {

namespace uart = ::m5::hal::v2::uart;

struct ConsoleFiles {
    FILE* input  = nullptr;
    FILE* output = nullptr;
    int writer   = -1;

    bool open()
    {
        int fds[2] = {-1, -1};
        if (::pipe(fds) != 0) {
            return false;
        }
        input = ::fdopen(fds[0], "r");
        if (input == nullptr) {
            ::close(fds[0]);
            ::close(fds[1]);
            return false;
        }
        writer = fds[1];
        output = ::tmpfile();
        return output != nullptr;
    }

    ~ConsoleFiles()
    {
        if (input != nullptr) {
            ::fclose(input);
        }
        if (writer >= 0) {
            ::close(writer);
        }
        if (output != nullptr) {
            ::fclose(output);
        }
    }
};

TEST(BusConsolePosix, InjectedOutputReceivesAccessorWrites)
{
    ConsoleFiles files;
    ASSERT_TRUE(files.open());

    uart::Bus_console bus;
    ASSERT_TRUE(bus.init(files.input, files.output).has_value());
    const auto caps = bus.capabilities();
    EXPECT_EQ(bus.backendKind(), ::m5::hal::v2::types::backend_kind_t::Hardware);
    EXPECT_TRUE(caps.supports(::m5::hal::v2::bus::BusFeature::HardwareBackend));
    EXPECT_TRUE(caps.supports(::m5::hal::v2::bus::BusFeature::Transmit));
    EXPECT_TRUE(caps.supports(::m5::hal::v2::bus::BusFeature::Receive));
    EXPECT_TRUE(caps.supports(::m5::hal::v2::bus::BusFeature::FullDuplex));
    uart::TxAccessor tx{bus, uart::AccessConfig{}};

    constexpr std::array<uint8_t, 5> payload = {0x4D, 0x35, 0x48, 0x41, 0x4C};
    auto written                             = tx.write(payload.data(), payload.size());
    ASSERT_TRUE(written.has_value());
    ASSERT_EQ(written.value(), payload.size());

    ASSERT_EQ(::fseek(files.output, 0, SEEK_SET), 0);
    std::array<uint8_t, payload.size()> actual{};
    ASSERT_EQ(::fread(actual.data(), 1, actual.size(), files.output), actual.size());
    EXPECT_EQ(actual, payload);
}

TEST(BusConsolePosix, InjectedPipeSupportsReadableBytesAndAccessorReads)
{
    ConsoleFiles files;
    ASSERT_TRUE(files.open());

    uart::Bus_console bus;
    ASSERT_TRUE(bus.init(files.input, files.output).has_value());
    uart::AccessConfig config;
    config.first_byte_timeout_ms = 10;
    config.inter_byte_timeout_ms = 0;
    uart::RxAccessor rx{bus, config};

    constexpr std::array<uint8_t, 4> payload = {0x10, 0x20, 0x30, 0x40};
    ASSERT_EQ(::write(files.writer, payload.data(), payload.size()), static_cast<ssize_t>(payload.size()));

    auto readable = rx.readableBytes();
    ASSERT_TRUE(readable.has_value());
    EXPECT_EQ(readable.value(), payload.size());

    std::array<uint8_t, payload.size()> actual{};
    auto read = rx.read(actual.data(), actual.size());
    ASSERT_TRUE(read.has_value());
    ASSERT_EQ(read.value(), actual.size());
    EXPECT_EQ(actual, payload);

    readable = rx.readableBytes();
    ASSERT_TRUE(readable.has_value());
    EXPECT_EQ(readable.value(), 0u);
}

TEST(BusConsolePosix, InitRejectsNullStreamsWithoutBecomingUsable)
{
    ConsoleFiles files;
    ASSERT_TRUE(files.open());

    uart::Bus_console bus;
    auto invalid = bus.init(nullptr, files.output);
    ASSERT_FALSE(invalid.has_value());
    EXPECT_EQ(invalid.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    uart::TxAccessor tx{bus, uart::AccessConfig{}};
    const uint8_t byte = 0x55;
    auto written       = tx.write(&byte, 1);
    ASSERT_FALSE(written.has_value());
    EXPECT_EQ(written.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

}  // namespace

#endif

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
