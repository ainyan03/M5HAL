// SPDX-License-Identifier: MIT
#include <m5_hal/hal/v2/i2c/slave_frame.hpp>

#include <gtest/gtest.h>

#include "support/gtest_watchdog.hpp"

#include <array>
#include <cstdint>
#include <type_traits>

namespace {

using namespace m5::hal::v2;
using namespace m5::hal::v2::i2c;
using error::error_t;

#define ASSERT_OK(expression)                                                               \
    do {                                                                                    \
        auto segment_result = (expression);                                                 \
        ASSERT_TRUE(segment_result.has_value()) << error::toString(segment_result.error()); \
    } while (false)

TEST(I2cSlaveFrame, ExactSegmentTypeAndFlagsRemainPlain)
{
    static_assert(std::is_standard_layout<I2cFrameSegment>::value, "plain shared segment");
    I2cFrameSegment segment;
    segment.offset    = 7;
    segment.length    = 3;
    segment.direction = I2cFrameDirection::Read;
    segment.flags |= I2cSegmentFlags::RepeatedStart;
    EXPECT_EQ(segment.offset, 7u);
    EXPECT_EQ(segment.length, 3u);
    EXPECT_EQ(segment.direction, I2cFrameDirection::Read);
    EXPECT_TRUE(any(segment.flags & I2cSegmentFlags::RepeatedStart));
}

TEST(I2cSlaveFrame, ReservationIsInvisibleUntilCommitAndZeroDetailIsValid)
{
    StaticI2cSegmentStorage<4> storage;
    I2cSegmentQueue queue;
    ASSERT_OK(queue.bind(storage.storage(), 11));

    auto token = queue.beginFrame();
    ASSERT_TRUE(token.has_value());
    EXPECT_EQ(queue.readable(), 0u);
    auto count = queue.commitFrame(*token);
    ASSERT_TRUE(count.has_value());
    EXPECT_EQ(count.value(), 0u);
    auto empty = queue.peekSegments(0);
    ASSERT_TRUE(empty.has_value());
    EXPECT_EQ(empty->first.size + empty->second.size, 0u);
    ASSERT_OK(queue.popSegments(0));
}

TEST(I2cSlaveFrame, RepeatedStartSegmentsWrapAsTwoSpans)
{
    StaticI2cSegmentStorage<3> storage;
    I2cSegmentQueue queue;
    ASSERT_OK(queue.bind(storage.storage(), 5));

    auto first = queue.beginFrame();
    ASSERT_TRUE(first.has_value());
    ASSERT_OK(queue.appendSegment(*first, {0, 1, I2cFrameDirection::Write, I2cSegmentFlags::None, 0}));
    ASSERT_OK(queue.appendSegment(*first, {1, 2, I2cFrameDirection::Read, I2cSegmentFlags::RepeatedStart, 0}));
    auto first_count = queue.commitFrame(*first);
    ASSERT_TRUE(first_count.has_value());
    EXPECT_EQ(first_count.value(), 2u);

    ASSERT_OK(queue.popSegments(1));
    auto second = queue.beginFrame();
    ASSERT_TRUE(second.has_value());
    ASSERT_OK(queue.appendSegment(*second, {0, 4, I2cFrameDirection::Write, I2cSegmentFlags::None, 0}));
    ASSERT_OK(queue.appendSegment(*second, {4, 1, I2cFrameDirection::Read, I2cSegmentFlags::RepeatedStart, 0}));
    auto second_count = queue.commitFrame(*second);
    ASSERT_TRUE(second_count.has_value());
    EXPECT_EQ(second_count.value(), 2u);

    ASSERT_OK(queue.popSegments(1));
    auto wrapped = queue.peekSegments(second_count.value());
    ASSERT_TRUE(wrapped.has_value());
    EXPECT_EQ(wrapped->first.size, 1u);
    EXPECT_EQ(wrapped->second.size, 1u);
    EXPECT_EQ(wrapped->first.data[0].direction, I2cFrameDirection::Write);
    EXPECT_EQ(wrapped->second.data[0].direction, I2cFrameDirection::Read);
    EXPECT_TRUE(any(wrapped->second.data[0].flags & I2cSegmentFlags::RepeatedStart));
}

TEST(I2cSlaveFrame, CapacityAndTokenFailuresNeverPublishPartialSegments)
{
    std::array<I2cFrameSegment, 1> segments{};
    I2cSegmentQueue queue;
    ASSERT_OK(queue.bind({segments.data(), segments.size()}, 3));

    auto token = queue.beginFrame();
    ASSERT_TRUE(token.has_value());
    ASSERT_OK(queue.appendSegment(*token, {}));
    auto full = queue.appendSegment(*token, {});
    ASSERT_FALSE(full.has_value());
    EXPECT_EQ(full.error(), error_t::WOULD_BLOCK);
    EXPECT_EQ(queue.readable(), 0u);
    ASSERT_OK(queue.cancelFrame(*token));
    EXPECT_EQ(queue.readable(), 0u);
    EXPECT_EQ(queue.appendSegment(*token, {}).error(), error_t::INVALID_STATE);
}

TEST(I2cSlaveFrame, GenerationChangePreservesPublishedSegmentsAndResetInvalidatesReservation)
{
    StaticI2cSegmentStorage<2> storage;
    I2cSegmentQueue queue;
    ASSERT_OK(queue.bind(storage.storage(), 1));

    auto published = queue.beginFrame();
    ASSERT_TRUE(published.has_value());
    ASSERT_OK(queue.appendSegment(*published, {0, 2, I2cFrameDirection::Write, I2cSegmentFlags::None, 0}));
    ASSERT_TRUE(queue.commitFrame(*published).has_value());
    ASSERT_OK(queue.setGeneration(2));
    EXPECT_EQ(queue.readable(), 1u);

    auto pending = queue.beginFrame();
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(queue.setGeneration(3).error(), error_t::INVALID_STATE);
    queue.reset(3);
    EXPECT_EQ(queue.readable(), 0u);
    EXPECT_EQ(queue.appendSegment(*pending, {}).error(), error_t::INVALID_STATE);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
