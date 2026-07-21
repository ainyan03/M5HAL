// SPDX-License-Identifier: MIT
// Native gtest for ConstDataSpan / DataSpan constexpr helpers (U15).
//
// Covers: empty(), begin()/end(), first(n), subspan(offset, count) on both
// ConstDataSpan (read-only) and DataSpan (mutable). All helpers are constexpr;
// a static_assert per struct confirms compile-time usability. DataSpan::first()
// and DataSpan::subspan() must return a mutable DataSpan, verified by writing
// through the result.
//
// No main() — the entry point lives in test_memory_source_sink.cpp.

#include <gtest/gtest.h>
#include <M5HAL_v2.hpp>

#include <array>
#include <cstdint>

namespace {

using m5::hal::v2::data::ConstDataSpan;
using m5::hal::v2::data::DataSpan;

// ============================================================================
// ConstDataSpan helpers
// ============================================================================

// constexpr smoke: verify the helpers are usable in constant expressions.
static_assert(ConstDataSpan{}.empty(), "ConstDataSpan::empty() must be constexpr");
static_assert(ConstDataSpan{}.first(5).size == 0u, "ConstDataSpan::first() must be constexpr");
static_assert(ConstDataSpan{}.subspan(0, 5).size == 0u, "ConstDataSpan::subspan() must be constexpr");

TEST(ConstDataSpan, EmptyTrueOnZeroSize)
{
    ConstDataSpan empty_span;
    EXPECT_TRUE(empty_span.empty());

    const uint8_t byte = 0;
    ConstDataSpan non_empty{&byte, 1};
    EXPECT_FALSE(non_empty.empty());
}

TEST(ConstDataSpan, DefaultEmptyPreservesNullRange)
{
    ConstDataSpan empty_span;
    EXPECT_EQ(empty_span.begin(), nullptr);
    EXPECT_EQ(empty_span.end(), nullptr);
    EXPECT_EQ(empty_span.first(1).data, nullptr);
    EXPECT_EQ(empty_span.subspan(0, 1).data, nullptr);

    size_t count = 0;
    for (uint8_t byte : empty_span) {
        (void)byte;
        ++count;
    }
    EXPECT_EQ(count, 0u);
}

TEST(ConstDataSpan, NonNullEmptyPreservesOriginalPointer)
{
    const uint8_t byte = 0;
    ConstDataSpan empty_span{&byte, 0};
    EXPECT_EQ(empty_span.begin(), &byte);
    EXPECT_EQ(empty_span.end(), &byte);
    EXPECT_EQ(empty_span.subspan(0, 1).data, &byte);
}

TEST(ConstDataSpan, BeginEndCoverWholeBuffer)
{
    const std::array<uint8_t, 4> bytes{0x10, 0x20, 0x30, 0x40};
    ConstDataSpan s{bytes.data(), bytes.size()};

    EXPECT_EQ(static_cast<size_t>(s.end() - s.begin()), s.size);

    size_t idx = 0;
    for (uint8_t b : s) {
        EXPECT_EQ(b, bytes[idx]);
        ++idx;
    }
    EXPECT_EQ(idx, 4u);
}

TEST(ConstDataSpan, FirstExactWhenNWithinSize)
{
    const std::array<uint8_t, 5> bytes{1, 2, 3, 4, 5};
    ConstDataSpan s{bytes.data(), bytes.size()};

    auto head = s.first(3);
    EXPECT_EQ(head.size, 3u);
    EXPECT_EQ(head.data[0], 1);
    EXPECT_EQ(head.data[2], 3);
}

TEST(ConstDataSpan, FirstClampsWhenNExceedsSize)
{
    const std::array<uint8_t, 3> bytes{1, 2, 3};
    ConstDataSpan s{bytes.data(), bytes.size()};

    auto all = s.first(100);
    EXPECT_EQ(all.size, s.size);
    EXPECT_EQ(all.data[2], 3);
}

TEST(ConstDataSpan, SubspanInRange)
{
    const std::array<uint8_t, 6> bytes{10, 20, 30, 40, 50, 60};
    ConstDataSpan s{bytes.data(), bytes.size()};

    auto sub = s.subspan(2, 3);
    EXPECT_EQ(sub.size, 3u);
    EXPECT_EQ(sub.data[0], 30);
    EXPECT_EQ(sub.data[2], 50);
}

TEST(ConstDataSpan, SubspanOffsetPastEndIsEmpty)
{
    const std::array<uint8_t, 3> bytes{1, 2, 3};
    ConstDataSpan s{bytes.data(), bytes.size()};

    auto sub = s.subspan(10, 2);
    EXPECT_EQ(sub.size, 0u);
}

TEST(ConstDataSpan, SubspanCountClamped)
{
    const std::array<uint8_t, 4> bytes{0xAA, 0xBB, 0xCC, 0xDD};
    ConstDataSpan s{bytes.data(), bytes.size()};

    // Only 2 bytes remain after offset=2; count=100 must be clamped.
    auto sub = s.subspan(2, 100);
    EXPECT_EQ(sub.size, 2u);
    EXPECT_EQ(sub.data[0], 0xCC);
    EXPECT_EQ(sub.data[1], 0xDD);
}

// ============================================================================
// DataSpan helpers (mutable)
// ============================================================================

// constexpr smoke on zero-size DataSpan.
static_assert(DataSpan{}.empty(), "DataSpan::empty() must be constexpr");
static_assert(DataSpan{}.first(5).size == 0u, "DataSpan::first() must be constexpr");
static_assert(DataSpan{}.subspan(0, 5).size == 0u, "DataSpan::subspan() must be constexpr");

TEST(DataSpan, EmptyTrueOnZeroSize)
{
    DataSpan empty_span;
    EXPECT_TRUE(empty_span.empty());

    uint8_t byte = 0;
    DataSpan non_empty{&byte, 1};
    EXPECT_FALSE(non_empty.empty());
}

TEST(DataSpan, DefaultEmptyPreservesNullRange)
{
    DataSpan empty_span;
    EXPECT_EQ(empty_span.begin(), nullptr);
    EXPECT_EQ(empty_span.end(), nullptr);
    EXPECT_EQ(empty_span.first(1).data, nullptr);
    EXPECT_EQ(empty_span.subspan(0, 1).data, nullptr);

    size_t count = 0;
    for (uint8_t byte : empty_span) {
        (void)byte;
        ++count;
    }
    EXPECT_EQ(count, 0u);
}

TEST(DataSpan, NonNullEmptyPreservesOriginalPointer)
{
    uint8_t byte = 0;
    DataSpan empty_span{&byte, 0};
    EXPECT_EQ(empty_span.begin(), &byte);
    EXPECT_EQ(empty_span.end(), &byte);
    EXPECT_EQ(empty_span.subspan(0, 1).data, &byte);
}

TEST(DataSpan, BeginEndCoverWholeBuffer)
{
    std::array<uint8_t, 3> bytes{0x01, 0x02, 0x03};
    DataSpan s{bytes.data(), bytes.size()};

    EXPECT_EQ(static_cast<size_t>(s.end() - s.begin()), s.size);

    size_t idx = 0;
    for (uint8_t b : s) {
        EXPECT_EQ(b, bytes[idx]);
        ++idx;
    }
    EXPECT_EQ(idx, 3u);
}

TEST(DataSpan, FirstReturnsMutableSpan)
{
    std::array<uint8_t, 5> bytes{1, 2, 3, 4, 5};
    DataSpan s{bytes.data(), bytes.size()};

    // Return type must be DataSpan (mutable): verified by writing through it.
    DataSpan head = s.first(3);
    EXPECT_EQ(head.size, 3u);
    head.data[0] = 0xFF;
    EXPECT_EQ(bytes[0], 0xFF);  // mutation is reflected in the backing array

    // Clamping: n > size returns the whole span.
    DataSpan all = s.first(100);
    EXPECT_EQ(all.size, s.size);
}

TEST(DataSpan, SubspanReturnsMutableSpan)
{
    std::array<uint8_t, 6> bytes{10, 20, 30, 40, 50, 60};
    DataSpan s{bytes.data(), bytes.size()};

    // Return type must be DataSpan (mutable): verified by writing through it.
    DataSpan sub = s.subspan(2, 3);
    EXPECT_EQ(sub.size, 3u);
    sub.data[0] = 0xFF;
    EXPECT_EQ(bytes[2], 0xFF);  // mutation is reflected in the backing array
}

TEST(DataSpan, SubspanOffsetPastEndIsEmpty)
{
    std::array<uint8_t, 3> bytes{1, 2, 3};
    DataSpan s{bytes.data(), bytes.size()};

    DataSpan sub = s.subspan(10, 2);
    EXPECT_EQ(sub.size, 0u);
}

TEST(DataSpan, SubspanCountClamped)
{
    std::array<uint8_t, 4> bytes{0xAA, 0xBB, 0xCC, 0xDD};
    DataSpan s{bytes.data(), bytes.size()};

    // Only 2 bytes remain after offset=2; count=100 must be clamped.
    DataSpan sub = s.subspan(2, 100);
    EXPECT_EQ(sub.size, 2u);
    EXPECT_EQ(sub.data[1], 0xDD);
}

}  // namespace
