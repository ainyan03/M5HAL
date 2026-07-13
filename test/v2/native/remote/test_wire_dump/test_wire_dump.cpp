// SPDX-License-Identifier: MIT
//
// Native gtest for the host wire dump
// (variants/frameworks/posix/hal/remote/wire_dump.hpp): record format,
// WireDump::ok(), and WireDumpWriter's dump == nullptr pass-through.
//
// Scope: WireDump / WireDumpWriter only. The generic mirror decorators
// (data::TapReader / data::TapWriter) that WireDumpWriter plugs into are
// covered on their own in test_stream_tap.cpp. WireDump::fromEnv() is a
// process-wide singleton keyed off getenv() at first use, which makes it
// order-dependent across the whole test binary; it is exercised manually
// (M5HAL_WIRE_DUMP=<path> against a real connection), not in gtest.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include <m5_hal/variants/frameworks/posix/hal/remote/wire_dump.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

namespace data         = ::m5::hal::v2::data;
namespace posix_remote = ::m5::variants::frameworks::posix::hal::v2::remote;
using posix_remote::WireDump;
using posix_remote::WireDumpWriter;

// Reads back every line of `path` (each writeRecord call is one line).
std::vector<std::string> readLines(const std::string& path)
{
    std::vector<std::string> lines;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        lines.push_back(line);
    }
    return lines;
}

// Strips the leading "<monotonic-sec> " field so assertions do not depend
// on wall-clock timing.
std::string dropTimestamp(const std::string& line)
{
    auto pos = line.find(' ');
    return pos == std::string::npos ? line : line.substr(pos + 1);
}

class WireDumpFile : public ::testing::Test {
protected:
    void SetUp() override
    {
        char tmpl[] = "/tmp/m5hal_wire_dump_test_XXXXXX";
        int fd      = ::mkstemp(tmpl);
        ASSERT_GE(fd, 0);
        ::close(fd);
        path = tmpl;
    }
    void TearDown() override
    {
        std::remove(path.c_str());
    }

    std::string path;
};

TEST_F(WireDumpFile, OpensAndReportsOk)
{
    WireDump dump(path.c_str());
    EXPECT_TRUE(dump.ok());
}

TEST_F(WireDumpFile, FailedOpenIsNotOk)
{
    WireDump dump("/nonexistent-dir/m5hal-wire-dump-test/does-not-exist.log");
    EXPECT_FALSE(dump.ok());
}

TEST_F(WireDumpFile, WriterFormatsRecordWithTagAndDirection)
{
    WireDump dump(path.c_str());
    ASSERT_TRUE(dump.ok());
    WireDumpWriter writer(&dump, "uart", '<');

    const uint8_t payload[] = {0xDE, 0xAD};
    auto r                  = writer.write(data::ConstDataSpan{payload, sizeof payload});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 2u);

    auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(dropTimestamp(lines[0]), "uart < 2 de ad");
}

TEST_F(WireDumpFile, WriterUsesTheOtherDirectionMarker)
{
    WireDump dump(path.c_str());
    ASSERT_TRUE(dump.ok());
    WireDumpWriter writer(&dump, "tcp", '>');

    const uint8_t payload[] = {0xBE, 0xEF, 0x01};
    auto r                  = writer.write(data::ConstDataSpan{payload, sizeof payload});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 3u);

    auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(dropTimestamp(lines[0]), "tcp > 3 be ef 01");
}

TEST_F(WireDumpFile, WriterAppendsOneRecordPerCall)
{
    WireDump dump(path.c_str());
    ASSERT_TRUE(dump.ok());
    WireDumpWriter writer(&dump, "uart", '<');

    const uint8_t first[]  = {0x01};
    const uint8_t second[] = {0x02, 0x03};
    ASSERT_TRUE(writer.write(data::ConstDataSpan{first, sizeof first}).has_value());
    ASSERT_TRUE(writer.write(data::ConstDataSpan{second, sizeof second}).has_value());

    auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(dropTimestamp(lines[0]), "uart < 1 01");
    EXPECT_EQ(dropTimestamp(lines[1]), "uart < 2 02 03");
}

TEST(WireDumpWriterNullDump, WriteIsNoOpAndReportsFullSize)
{
    WireDumpWriter writer(nullptr, "uart", '<');

    const uint8_t payload[] = {1, 2, 3, 4};
    auto r                  = writer.write(data::ConstDataSpan{payload, sizeof payload});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 4u);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
