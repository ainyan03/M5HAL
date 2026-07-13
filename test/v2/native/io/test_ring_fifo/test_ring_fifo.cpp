// SPDX-License-Identifier: MIT
// Native gtest for RingFIFO (data/ring.hpp).
//
// Regression anchor: advance(0) / commit(0) on an unbound ring
// (capacity 0) used to reach `(cursor + 0) % capacity` — a division by
// zero. Both are now early-returning no-ops. The remaining cases pin
// the SPSC contract faces the spec relies on: wrap-around byte order,
// contiguity clamping at the buffer boundary, overflow rejection, and
// reset()/setBuf() invalidating an in-flight reserve.
// Spec: spec/design/data_io.md.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/data/ring.hpp>

#include <cstdint>
#include <cstring>

namespace {

using m5::hal::v2::data::RingFIFO;
using error_t = m5::hal::v2::error::error_t;

// Push up to `len` bytes through reserve/commit; returns bytes accepted.
size_t push(RingFIFO& ring, const uint8_t* data, size_t len)
{
    size_t done = 0;
    while (done < len) {
        auto span = ring.sink().reserve(len - done);
        if (!span.has_value() || span->size == 0) {
            break;
        }
        std::memcpy(span->data, data + done, span->size);
        if (!ring.sink().commit(span->size).has_value()) {
            break;
        }
        done += span->size;
    }
    return done;
}

// Pop up to `len` bytes through peek/advance into `out`; returns bytes read.
size_t pop(RingFIFO& ring, uint8_t* out, size_t len)
{
    size_t done = 0;
    while (done < len) {
        auto span = ring.source().peek(len - done);
        if (!span.has_value() || span->size == 0) {
            break;
        }
        std::memcpy(out + done, span->data, span->size);
        if (!ring.source().advance(span->size).has_value()) {
            break;
        }
        done += span->size;
    }
    return done;
}

TEST(RingFIFO, UnboundZeroOpsAreSafeNoOps)
{
    RingFIFO ring;  // no buffer bound: capacity 0

    // Pre-fix these reached `% 0`; reaching the asserts is the check.
    EXPECT_TRUE(ring.source().advance(0).has_value());
    EXPECT_TRUE(ring.sink().commit(0).has_value());

    auto peeked = ring.source().peek(8);
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked->size, 0u);
    auto reserved = ring.sink().reserve(8);
    ASSERT_TRUE(reserved.has_value());
    EXPECT_EQ(reserved->size, 0u);

    EXPECT_TRUE(ring.source().eof());
    EXPECT_TRUE(ring.sink().closed());
}

TEST(RingFIFO, UnboundNonZeroOpsAreRejected)
{
    RingFIFO ring;

    auto advanced = ring.source().advance(1);
    ASSERT_FALSE(advanced.has_value());
    EXPECT_EQ(advanced.error(), error_t::INVALID_ARGUMENT);

    auto committed = ring.sink().commit(1);
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), error_t::BUFFER_OVERFLOW);
}

TEST(RingFIFO, WrapAroundPreservesByteOrder)
{
    uint8_t storage[8] = {};
    RingFIFO ring{storage, sizeof(storage)};

    const uint8_t first[] = {1, 2, 3, 4, 5, 6};
    ASSERT_EQ(push(ring, first, sizeof(first)), sizeof(first));

    uint8_t out[16] = {};
    ASSERT_EQ(pop(ring, out, 4), 4u);
    EXPECT_EQ(0, std::memcmp(out, first, 4));

    // 2 bytes remain at offsets 4..5; this write wraps past offset 7.
    const uint8_t second[] = {7, 8, 9, 10, 11};
    ASSERT_EQ(push(ring, second, sizeof(second)), sizeof(second));
    EXPECT_EQ(ring.buffered(), 7u);

    const uint8_t expect[] = {5, 6, 7, 8, 9, 10, 11};
    ASSERT_EQ(pop(ring, out, sizeof(expect)), sizeof(expect));
    EXPECT_EQ(0, std::memcmp(out, expect, sizeof(expect)));
    EXPECT_EQ(ring.buffered(), 0u);
}

TEST(RingFIFO, PeekAndReserveClampToContiguousRun)
{
    uint8_t storage[8] = {};
    RingFIFO ring{storage, sizeof(storage)};

    // Move the cursors to offset 6, then fill across the boundary.
    const uint8_t pad[] = {0, 0, 0, 0, 0, 0};
    ASSERT_EQ(push(ring, pad, sizeof(pad)), sizeof(pad));
    uint8_t out[8] = {};
    ASSERT_EQ(pop(ring, out, sizeof(pad)), sizeof(pad));

    const uint8_t data[] = {1, 2, 3, 4};
    ASSERT_EQ(push(ring, data, sizeof(data)), sizeof(data));

    // Buffered = 4, but only offsets 6..7 are contiguous from the tail.
    auto span = ring.source().peek(sizeof(data));
    ASSERT_TRUE(span.has_value());
    EXPECT_EQ(span->size, 2u);
    EXPECT_EQ(span->data[0], 1);
    EXPECT_EQ(span->data[1], 2);
}

TEST(RingFIFO, OverflowingAdvanceAndCommitAreRejected)
{
    uint8_t storage[8] = {};
    RingFIFO ring{storage, sizeof(storage)};

    const uint8_t data[] = {1, 2, 3};
    ASSERT_EQ(push(ring, data, sizeof(data)), sizeof(data));

    auto advanced = ring.source().advance(sizeof(data) + 1);
    ASSERT_FALSE(advanced.has_value());
    EXPECT_EQ(advanced.error(), error_t::INVALID_ARGUMENT);

    auto span = ring.sink().reserve(4);
    ASSERT_TRUE(span.has_value());
    ASSERT_EQ(span->size, 4u);
    auto committed = ring.sink().commit(span->size + 1);
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), error_t::BUFFER_OVERFLOW);
}

TEST(RingFIFO, ResetInvalidatesInFlightReserve)
{
    uint8_t storage[8] = {};
    RingFIFO ring{storage, sizeof(storage)};

    auto span = ring.sink().reserve(4);
    ASSERT_TRUE(span.has_value());
    ASSERT_EQ(span->size, 4u);

    ring.reset();

    // A stale commit after reset must not hand the consumer bytes that
    // were never written into the post-reset window.
    auto committed = ring.sink().commit(4);
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), error_t::BUFFER_OVERFLOW);
    EXPECT_EQ(ring.buffered(), 0u);
}

TEST(RingFIFO, SetBufRebindsAndResetsState)
{
    uint8_t storage_a[8] = {};
    RingFIFO ring{storage_a, sizeof(storage_a)};

    const uint8_t data[] = {1, 2, 3};
    ASSERT_EQ(push(ring, data, sizeof(data)), sizeof(data));

    uint8_t storage_b[4] = {};
    ring.setBuf(storage_b, sizeof(storage_b));
    EXPECT_EQ(ring.buffered(), 0u);
    EXPECT_EQ(ring.capacity(), sizeof(storage_b));

    const uint8_t fresh[] = {9, 8};
    ASSERT_EQ(push(ring, fresh, sizeof(fresh)), sizeof(fresh));
    uint8_t out[4] = {};
    ASSERT_EQ(pop(ring, out, sizeof(fresh)), sizeof(fresh));
    EXPECT_EQ(0, std::memcmp(out, fresh, sizeof(fresh)));
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
