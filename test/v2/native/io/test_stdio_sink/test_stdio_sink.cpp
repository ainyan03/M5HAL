// SPDX-License-Identifier: MIT
// Native gtest for StdioSink::commit (data/stdio.hpp).
//
// Regression anchor: commit() ignored the fwrite() return value, so a
// failing or short write was reported as full success and the bytes
// silently vanished. commit() now reports IO_ERROR and exposes the
// accepted prefix via partialCommitAccepted() (acceptance boundary =
// the FILE* stream). Spec: spec/design/data_io.md §Sink (partial commit).

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/data/stdio.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

using m5::hal::v2::data::StdioSink;
using error_t = m5::hal::v2::error::error_t;

TEST(StdioSink, CommitWritesThroughToFile)
{
    FILE* f = std::tmpfile();
    ASSERT_NE(f, nullptr);
    StdioSink sink{f};

    const uint8_t payload[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    auto span               = sink.reserve(sizeof(payload));
    ASSERT_TRUE(span.has_value());
    ASSERT_GE(span->size, sizeof(payload));
    std::memcpy(span->data, payload, sizeof(payload));
    ASSERT_TRUE(sink.commit(sizeof(payload)).has_value());

    std::rewind(f);
    uint8_t back[sizeof(payload)] = {};
    ASSERT_EQ(std::fread(back, 1, sizeof(back), f), sizeof(back));
    EXPECT_EQ(0, std::memcmp(back, payload, sizeof(payload)));
    std::fclose(f);
}

TEST(StdioSink, CommitToReadOnlyFileReportsIoError)
{
    FILE* f = std::fopen("/dev/null", "r");
    ASSERT_NE(f, nullptr);
    StdioSink sink{f};

    auto span = sink.reserve(4);
    ASSERT_TRUE(span.has_value());
    ASSERT_GE(span->size, 4u);
    std::memset(span->data, 0xA5, 4);

    // Pre-fix this reported success while every byte was dropped.
    auto committed = sink.commit(4);
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), error_t::IO_ERROR);
    EXPECT_EQ(sink.partialCommitAccepted(), 0u);
    std::fclose(f);
}

TEST(StdioSink, NullFileCommitIsNoOpAndClosed)
{
    StdioSink sink{nullptr};
    EXPECT_TRUE(sink.closed());
    EXPECT_TRUE(sink.commit(3).has_value());
    EXPECT_EQ(sink.partialCommitAccepted(), 0u);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
