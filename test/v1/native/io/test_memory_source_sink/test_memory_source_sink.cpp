// Native gtest for MemorySource / MemorySink.
//
// Mechanically verifies the behavioural contract of the memory-backed
// Source / Sink: `peek` is idempotent and monotonically non-decreasing,
// an empty span on `peek` means end-of-stream, an empty `DataSpan`
// on `reserve` means closed, sequential `advance` discards bytes,
// `advance` past the end drops the excess, and `reserve` mirrors the
// idempotency of `peek`. Spec: spec/design/data_io.md.

#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

#include <array>
#include <cstdint>

namespace {

using m5::hal::v1::data::ConstDataSpan;
using m5::hal::v1::data::DataSpan;
using m5::hal::v1::data::MemorySink;
using m5::hal::v1::data::MemorySource;
using m5::hal::v1::data::Sink;
using m5::hal::v1::data::Source;

// ============================================================================
// MemorySource
// ============================================================================

TEST(MemorySource, EmptySpanIsEofImmediately)
{
    MemorySource src;
    EXPECT_TRUE(src.eof());

    auto peeked = src.peek(64);
    ASSERT_TRUE(peeked);
    EXPECT_EQ(peeked->size, 0u);
    EXPECT_EQ(peeked->data, nullptr);
}

TEST(MemorySource, PeekReturnsClampedSize)
{
    const std::array<uint8_t, 8> bytes{0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    MemorySource src{ConstDataSpan{bytes.data(), bytes.size()}};
    EXPECT_FALSE(src.eof());

    auto small = src.peek(3);
    ASSERT_TRUE(small);
    EXPECT_EQ(small->size, 3u);
    EXPECT_EQ(small->data[0], 0x10);
    EXPECT_EQ(small->data[2], 0x12);

    auto big = src.peek(100);
    ASSERT_TRUE(big);
    EXPECT_EQ(big->size, bytes.size());
    EXPECT_EQ(big->data[7], 0x17);
}

TEST(MemorySource, PeekIsMonotonicIdempotent)
{
    // Successive peek calls without an intervening advance keep
    // the previously returned prefix bytes intact and never shrink
    // the size. The memory-backed source happens to return the same
    // pointer too, but the contract only guarantees the content.
    const std::array<uint8_t, 4> bytes{0xA0, 0xA1, 0xA2, 0xA3};
    MemorySource src{ConstDataSpan{bytes.data(), bytes.size()}};

    auto first  = src.peek(2);
    auto second = src.peek(2);
    auto larger = src.peek(10);

    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    ASSERT_TRUE(larger);
    EXPECT_EQ(first->size, 2u);
    EXPECT_EQ(second->size, 2u);
    EXPECT_EQ(larger->size, bytes.size());  // Growing the requested size is allowed.
    EXPECT_EQ(first->data[0], second->data[0]);
    EXPECT_EQ(first->data[1], second->data[1]);
    EXPECT_EQ(larger->data[0], 0xA0);
}

TEST(MemorySource, AdvancePartialThenPeek)
{
    const std::array<uint8_t, 5> bytes{1, 2, 3, 4, 5};
    MemorySource src{ConstDataSpan{bytes.data(), bytes.size()}};

    auto advance_result = src.advance(2);
    ASSERT_TRUE(advance_result);
    EXPECT_FALSE(src.eof());

    auto rest = src.peek(100);
    ASSERT_TRUE(rest);
    EXPECT_EQ(rest->size, 3u);
    EXPECT_EQ(rest->data[0], 3);
    EXPECT_EQ(rest->data[2], 5);
}

TEST(MemorySource, AdvanceWithoutPeekIsLegal)
{
    // Discarding bytes through repeated advance during sequential reads is a supported usage.
    const std::array<uint8_t, 6> bytes{10, 20, 30, 40, 50, 60};
    MemorySource src{ConstDataSpan{bytes.data(), bytes.size()}};

    auto skipped = src.advance(4);
    ASSERT_TRUE(skipped);
    EXPECT_FALSE(src.eof());

    auto rest = src.peek(10);
    ASSERT_TRUE(rest);
    EXPECT_EQ(rest->size, 2u);
    EXPECT_EQ(rest->data[0], 50);
    EXPECT_EQ(rest->data[1], 60);
}

TEST(MemorySource, AdvanceBeyondEndDiscardsAndReachesEof)
{
    // The memory-backed Source drops the excess on an out-of-range advance and moves to end-of-stream.
    const std::array<uint8_t, 3> bytes{0x5A, 0x5B, 0x5C};
    MemorySource src{ConstDataSpan{bytes.data(), bytes.size()}};

    auto overflow = src.advance(100);
    ASSERT_TRUE(overflow);
    EXPECT_TRUE(src.eof());

    auto post_eof = src.peek(64);
    ASSERT_TRUE(post_eof);
    EXPECT_EQ(post_eof->size, 0u);
}

TEST(MemorySource, ExactConsumeReachesEof)
{
    const std::array<uint8_t, 2> bytes{0xDE, 0xAD};
    MemorySource src{ConstDataSpan{bytes.data(), bytes.size()}};

    auto consume = src.advance(bytes.size());
    ASSERT_TRUE(consume);
    EXPECT_TRUE(src.eof());

    auto peek = src.peek(64);
    ASSERT_TRUE(peek);
    EXPECT_EQ(peek->size, 0u);
}

// ============================================================================
// MemorySink
// ============================================================================

TEST(MemorySink, EmptyBufferIsClosedImmediately)
{
    MemorySink sink;
    EXPECT_TRUE(sink.closed());

    auto reserved = sink.reserve(64);
    ASSERT_TRUE(reserved);
    EXPECT_EQ(reserved->size, 0u);
    EXPECT_EQ(reserved->data, nullptr);
}

TEST(MemorySink, ReserveReturnsClampedSize)
{
    std::array<uint8_t, 8> buf{};
    MemorySink sink{DataSpan{buf.data(), buf.size()}};
    EXPECT_FALSE(sink.closed());

    auto small = sink.reserve(3);
    ASSERT_TRUE(small);
    EXPECT_EQ(small->size, 3u);
    EXPECT_EQ(small->data, buf.data());

    auto big = sink.reserve(100);
    ASSERT_TRUE(big);
    EXPECT_EQ(big->size, buf.size());
}

TEST(MemorySink, ReserveIsMonotonicIdempotent)
{
    std::array<uint8_t, 4> buf{};
    MemorySink sink{DataSpan{buf.data(), buf.size()}};

    auto first  = sink.reserve(2);
    auto second = sink.reserve(2);
    auto larger = sink.reserve(10);

    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    ASSERT_TRUE(larger);
    EXPECT_EQ(first->size, 2u);
    EXPECT_EQ(second->size, 2u);
    EXPECT_EQ(larger->size, buf.size());
    EXPECT_EQ(first->data, second->data);  // Memory-backed sinks happen to return the same pointer too.
}

TEST(MemorySink, ReserveCommitSequenceAdvancesCursor)
{
    std::array<uint8_t, 6> buf{};
    MemorySink sink{DataSpan{buf.data(), buf.size()}};

    {
        auto r = sink.reserve(3);
        ASSERT_TRUE(r);
        r->data[0] = 0xAA;
        r->data[1] = 0xBB;
        r->data[2] = 0xCC;
        auto c     = sink.commit(3);
        ASSERT_TRUE(c);
    }
    EXPECT_FALSE(sink.closed());

    {
        auto r = sink.reserve(100);
        ASSERT_TRUE(r);
        EXPECT_EQ(r->size, 3u);  // 3 bytes remain.
        r->data[0] = 0xDD;
        r->data[1] = 0xEE;
        r->data[2] = 0xFF;
        auto c     = sink.commit(3);
        ASSERT_TRUE(c);
    }
    EXPECT_TRUE(sink.closed());

    EXPECT_EQ(buf[0], 0xAA);
    EXPECT_EQ(buf[1], 0xBB);
    EXPECT_EQ(buf[2], 0xCC);
    EXPECT_EQ(buf[3], 0xDD);
    EXPECT_EQ(buf[4], 0xEE);
    EXPECT_EQ(buf[5], 0xFF);
}

TEST(MemorySink, PostClosedReserveReturnsEmpty)
{
    std::array<uint8_t, 2> buf{};
    MemorySink sink{DataSpan{buf.data(), buf.size()}};

    auto fill = sink.commit(buf.size());
    ASSERT_TRUE(fill);
    EXPECT_TRUE(sink.closed());

    auto post = sink.reserve(64);
    ASSERT_TRUE(post);
    EXPECT_EQ(post->size, 0u);
}

// ============================================================================
// LimitedSource / LimitedSink
// Cap-based decorators. Covers both termination paths: the limit
// triggers before the base exhausts, and the base exhausts before
// the limit triggers.
// ============================================================================

using m5::hal::v1::data::LimitedSink;
using m5::hal::v1::data::LimitedSource;

TEST(LimitedSource, LimitBeforeBaseExhausts)
{
    // 8 bytes in the base, limit = 3 -> EOF after 3 bytes.
    const std::array<uint8_t, 8> bytes{0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    MemorySource base{ConstDataSpan{bytes.data(), bytes.size()}};
    LimitedSource src{base, 3};

    EXPECT_FALSE(src.eof());
    EXPECT_EQ(src.remaining(), 3u);

    auto peeked = src.peek(64);
    ASSERT_TRUE(peeked);
    EXPECT_EQ(peeked->size, 3u);  // Clamped to 3, not 8.
    EXPECT_EQ(peeked->data[0], 0x10);
    EXPECT_EQ(peeked->data[2], 0x12);

    ASSERT_TRUE(src.advance(3));
    EXPECT_TRUE(src.eof());
    EXPECT_EQ(src.remaining(), 0u);

    // peek after EOF returns an empty span.
    auto after = src.peek(64);
    ASSERT_TRUE(after);
    EXPECT_EQ(after->size, 0u);
}

TEST(LimitedSource, BaseExhaustsBeforeLimit)
{
    // 4 bytes in the base, limit = 10 -> the base reaches EOF first.
    const std::array<uint8_t, 4> bytes{0xAA, 0xBB, 0xCC, 0xDD};
    MemorySource base{ConstDataSpan{bytes.data(), bytes.size()}};
    LimitedSource src{base, 10};

    EXPECT_FALSE(src.eof());
    auto peeked = src.peek(64);
    ASSERT_TRUE(peeked);
    EXPECT_EQ(peeked->size, 4u);  // Capped by what the base still has.

    ASSERT_TRUE(src.advance(4));
    EXPECT_TRUE(src.eof());  // base.eof() === true
    // The limit budget is still available but EOF is already true.
    EXPECT_EQ(src.remaining(), 6u);
}

TEST(LimitedSource, ZeroLimitImmediatelyEof)
{
    const std::array<uint8_t, 4> bytes{0x00, 0x01, 0x02, 0x03};
    MemorySource base{ConstDataSpan{bytes.data(), bytes.size()}};
    LimitedSource src{base, 0};

    EXPECT_TRUE(src.eof());
    auto peeked = src.peek(64);
    ASSERT_TRUE(peeked);
    EXPECT_EQ(peeked->size, 0u);
}

TEST(LimitedSource, NullBaseIsEmpty)
{
    LimitedSource src{static_cast<Source*>(nullptr), 8};

    EXPECT_TRUE(src.eof());
    EXPECT_EQ(src.remaining(), 8u);

    auto peeked = src.peek(64);
    ASSERT_TRUE(peeked);
    EXPECT_EQ(peeked->size, 0u);

    ASSERT_TRUE(src.advance(3));
    EXPECT_EQ(src.remaining(), 5u);
    EXPECT_TRUE(src.eof());
}

TEST(LimitedSource, PeekDoesNotConsumeLimit)
{
    // peek is idempotent; the limit counter only decreases on advance.
    const std::array<uint8_t, 8> bytes{0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    MemorySource base{ConstDataSpan{bytes.data(), bytes.size()}};
    LimitedSource src{base, 5};

    auto p1 = src.peek(64);
    ASSERT_TRUE(p1);
    EXPECT_EQ(p1->size, 5u);
    EXPECT_EQ(src.remaining(), 5u);  // peek doesn't shrink the budget.

    auto p2 = src.peek(64);
    ASSERT_TRUE(p2);
    EXPECT_EQ(p2->size, 5u);  // Same size (monotonic + idempotent).

    ASSERT_TRUE(src.advance(2));
    EXPECT_EQ(src.remaining(), 3u);

    auto p3 = src.peek(64);
    ASSERT_TRUE(p3);
    EXPECT_EQ(p3->size, 3u);
    EXPECT_EQ(p3->data[0], 0x12);
}

TEST(LimitedSink, LimitBeforeBaseExhausts)
{
    // 16-byte base buffer, limit = 4 -> closed after 4 bytes.
    std::array<uint8_t, 16> buf{};
    MemorySink base{DataSpan{buf.data(), buf.size()}};
    LimitedSink sink{base, 4};

    EXPECT_FALSE(sink.closed());
    EXPECT_EQ(sink.remaining(), 4u);

    auto rsv = sink.reserve(64);
    ASSERT_TRUE(rsv);
    EXPECT_EQ(rsv->size, 4u);  // Clamped to 4, not 16.

    rsv->data[0] = 0xAA;
    rsv->data[3] = 0xBB;
    ASSERT_TRUE(sink.commit(4));
    EXPECT_TRUE(sink.closed());
    EXPECT_EQ(sink.remaining(), 0u);
    EXPECT_EQ(buf[0], 0xAA);
    EXPECT_EQ(buf[3], 0xBB);

    auto after = sink.reserve(64);
    ASSERT_TRUE(after);
    EXPECT_EQ(after->size, 0u);  // Reserve after close returns empty.
}

TEST(LimitedSink, BaseExhaustsBeforeLimit)
{
    // 3-byte base buffer, limit = 10 -> the base closes first.
    std::array<uint8_t, 3> buf{};
    MemorySink base{DataSpan{buf.data(), buf.size()}};
    LimitedSink sink{base, 10};

    EXPECT_FALSE(sink.closed());
    auto rsv = sink.reserve(64);
    ASSERT_TRUE(rsv);
    EXPECT_EQ(rsv->size, 3u);  // Capped by what the base still has.

    ASSERT_TRUE(sink.commit(3));
    EXPECT_TRUE(sink.closed());       // base.closed() === true
    EXPECT_EQ(sink.remaining(), 7u);  // Limit budget is still available.
}

TEST(LimitedSink, ZeroLimitImmediatelyClosed)
{
    std::array<uint8_t, 4> buf{};
    MemorySink base{DataSpan{buf.data(), buf.size()}};
    LimitedSink sink{base, 0};

    EXPECT_TRUE(sink.closed());
    auto rsv = sink.reserve(64);
    ASSERT_TRUE(rsv);
    EXPECT_EQ(rsv->size, 0u);
}

TEST(LimitedSink, NullBaseIsClosed)
{
    LimitedSink sink{static_cast<Sink*>(nullptr), 8};

    EXPECT_TRUE(sink.closed());
    EXPECT_EQ(sink.remaining(), 8u);

    auto rsv = sink.reserve(64);
    ASSERT_TRUE(rsv);
    EXPECT_EQ(rsv->size, 0u);

    ASSERT_TRUE(sink.commit(3));
    EXPECT_EQ(sink.remaining(), 5u);
    EXPECT_TRUE(sink.closed());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
