// SPDX-License-Identifier: MIT
#include <m5_hal/hal/v2/slave/queue.hpp>
#include <m5_hal/hal/v2/slave/event.hpp>

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <thread>
#include <type_traits>

namespace {

using namespace m5::hal::v2;
using data::ConstDataSpan;
using data::DataSpan;
using error::error_t;
using namespace slave;

#define ASSERT_OK(expression)                                \
    do {                                                     \
        auto queue_result = (expression);                    \
        if (!queue_result.has_value()) {                     \
            FAIL() << error::toString(queue_result.error()); \
        }                                                    \
    } while (false)

template <size_t Bytes, size_t Descriptors>
struct FixtureStorage {
    std::array<uint8_t, Bytes> bytes{};
    std::array<FrameDescriptor, Descriptors> descriptors{};

    QueueStorage<> view()
    {
        return {{bytes.data(), bytes.size()}, descriptors.data(), descriptors.size()};
    }
};

FrameMetadata metadata(uint32_t id, uint32_t size)
{
    FrameMetadata value;
    value.frame_id     = id;
    value.wire_bytes   = size;
    value.stored_bytes = size;
    value.flags        = FrameFlags::Begin | FrameFlags::End;
    return value;
}

struct EventObservation {
    SlaveEventEndpoint* endpoint = nullptr;
    SlaveEventInfo last{};
    uint32_t calls        = 0;
    error_t reentry_error = error_t::OK;
    SlaveEventInfo level{};
};

void observeEvent(void* user, const SlaveEventInfo& info)
{
    auto& observation = *static_cast<EventObservation*>(user);
    observation.last  = info;
    ++observation.calls;
    if (observation.endpoint != nullptr) {
        auto reentered            = observation.endpoint->dispatchEvents();
        observation.reentry_error = reentered.has_value() ? error_t::OK : reentered.error();
    }
}

SlaveEventInfo probeEventLevel(void* user, uint32_t generation)
{
    auto info       = static_cast<EventObservation*>(user)->level;
    info.generation = generation;
    return info;
}

TEST(SlaveQueueContract, ByteQueueWrapsAndReturnsShortSuccess)
{
    FixtureStorage<5, 0> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Byte));

    const std::array<uint8_t, 4> first{{1, 2, 3, 4}};
    auto written = queue.write({first.data(), first.size()});
    ASSERT_TRUE(written);
    EXPECT_EQ(*written, 4u);

    std::array<uint8_t, 3> prefix{};
    auto read = queue.read({prefix.data(), prefix.size()});
    ASSERT_TRUE(read);
    EXPECT_EQ(*read, 3u);
    EXPECT_EQ(prefix, (std::array<uint8_t, 3>{{1, 2, 3}}));

    const std::array<uint8_t, 5> second{{5, 6, 7, 8, 9}};
    written = queue.write({second.data(), second.size()});
    ASSERT_TRUE(written);
    EXPECT_EQ(*written, 4u);
    EXPECT_EQ(queue.readable(), 5u);
    EXPECT_EQ(queue.writable(), 0u);

    std::array<uint8_t, 5> all{};
    read = queue.read({all.data(), all.size()});
    ASSERT_TRUE(read);
    EXPECT_EQ(*read, 5u);
    EXPECT_EQ(all, (std::array<uint8_t, 5>{{4, 5, 6, 7, 8}}));
}

TEST(SlaveQueueContract, EmptyAndFullAreWouldBlockButZeroLengthSucceeds)
{
    FixtureStorage<2, 0> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Byte));

    uint8_t byte = 0;
    auto empty   = queue.read({&byte, 1});
    ASSERT_FALSE(empty);
    EXPECT_EQ(empty.error(), error_t::WOULD_BLOCK);
    auto zero_read = queue.read({nullptr, 0});
    ASSERT_TRUE(zero_read);
    EXPECT_EQ(*zero_read, 0u);

    const std::array<uint8_t, 2> input{{1, 2}};
    ASSERT_OK(queue.write({input.data(), input.size()}));
    auto full = queue.write({input.data(), 1});
    ASSERT_FALSE(full);
    EXPECT_EQ(full.error(), error_t::WOULD_BLOCK);
    auto zero_write = queue.write({nullptr, 0});
    ASSERT_TRUE(zero_write);
    EXPECT_EQ(*zero_write, 0u);
}

TEST(SlaveQueueContract, BackendBytePeekDoesNotConsumeUntilExplicitPop)
{
    FixtureStorage<5, 0> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Byte));
    const std::array<uint8_t, 4> first{{1, 2, 3, 4}};
    ASSERT_OK(queue.write({first.data(), first.size()}));
    uint8_t discarded[3]{};
    ASSERT_OK(queue.read({discarded, sizeof(discarded)}));
    const std::array<uint8_t, 4> second{{5, 6, 7, 8}};
    ASSERT_OK(queue.write({second.data(), second.size()}));

    auto peeked = queue.peekBytes(4);
    ASSERT_TRUE(peeked) << error::toString(peeked.error());
    EXPECT_EQ(peeked->first.size, 2u);
    EXPECT_EQ(peeked->second.size, 2u);
    EXPECT_EQ(peeked->first.data[0], 4u);
    EXPECT_EQ(peeked->second.data[1], 7u);
    EXPECT_EQ(queue.readable(), 5u);
    ASSERT_OK(queue.popBytes(3));
    EXPECT_EQ(queue.readable(), 2u);
    EXPECT_EQ(queue.popBytes(3).error(), error_t::INVALID_ARGUMENT);
}

TEST(SlaveQueueContract, FrameReservationWrapsAndPublishesAllOrNothing)
{
    FixtureStorage<6, 3> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Frame, 7));

    const std::array<uint8_t, 4> first{{1, 2, 3, 4}};
    ASSERT_OK(queue.writeFrame({first.data(), first.size()}, metadata(1, first.size())));
    auto first_view = queue.peekFrame();
    ASSERT_TRUE(first_view);
    EXPECT_EQ(first_view->first.size + first_view->second.size, first.size());
    ASSERT_OK(queue.popFrame());

    auto reservation = queue.reserveFrame(4);
    ASSERT_TRUE(reservation);
    EXPECT_EQ(reservation->first.size, 2u);
    EXPECT_EQ(reservation->second.size, 2u);
    reservation->first.data[0]  = 5;
    reservation->first.data[1]  = 6;
    reservation->second.data[0] = 7;
    reservation->second.data[1] = 8;
    EXPECT_EQ(queue.readableFrames(), 0u);
    ASSERT_OK(queue.commitFrame(*reservation, metadata(2, 4)));

    auto view = queue.peekFrame();
    ASSERT_TRUE(view);
    ASSERT_EQ(view->metadata.frame_id, 2u);
    ASSERT_EQ(view->first.size, 2u);
    ASSERT_EQ(view->second.size, 2u);
    EXPECT_EQ(view->first.data[0], 5u);
    EXPECT_EQ(view->first.data[1], 6u);
    EXPECT_EQ(view->second.data[0], 7u);
    EXPECT_EQ(view->second.data[1], 8u);
}

TEST(SlaveQueueContract, ZeroLengthFrameUsesDescriptorAndPopsUnambiguously)
{
    FixtureStorage<0, 1> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Frame));

    auto reservation = queue.reserveFrame(0);
    ASSERT_TRUE(reservation);
    EXPECT_EQ(reservation->first.size, 0u);
    EXPECT_EQ(reservation->second.size, 0u);
    ASSERT_OK(queue.commitFrame(*reservation, metadata(4, 0)));
    EXPECT_EQ(queue.readableFrames(), 1u);

    auto view = queue.peekFrame();
    ASSERT_TRUE(view);
    EXPECT_TRUE(view->first.empty());
    EXPECT_TRUE(view->second.empty());
    EXPECT_EQ(view->metadata.frame_id, 4u);
    ASSERT_OK(queue.popFrame());
    EXPECT_EQ(queue.readableFrames(), 0u);
}

TEST(SlaveQueueContract, CapacityFailureNeverOverwritesPublishedFrame)
{
    FixtureStorage<4, 1> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Frame));
    const std::array<uint8_t, 4> original{{11, 12, 13, 14}};
    ASSERT_OK(queue.writeFrame({original.data(), original.size()}, metadata(10, original.size())));

    auto no_bytes = queue.reserveFrame(1);
    ASSERT_FALSE(no_bytes);
    EXPECT_EQ(no_bytes.error(), error_t::WOULD_BLOCK);
    auto view = queue.peekFrame();
    ASSERT_TRUE(view);
    EXPECT_EQ(view->metadata.frame_id, 10u);
    EXPECT_EQ(view->first.data[0], 11u);
}

TEST(SlaveQueueContract, DescriptorExhaustionIsObservableWithoutDescriptor)
{
    FixtureStorage<8, 1> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Frame));
    const uint8_t byte = 1;
    ASSERT_OK(queue.writeFrame({&byte, 1}, metadata(1, 1)));
    auto no_descriptor = queue.reserveFrame(0);
    ASSERT_FALSE(no_descriptor);
    EXPECT_EQ(no_descriptor.error(), error_t::WOULD_BLOCK);

    queue.recordDroppedFrame(17);
    const QueueStatus status = queue.status();
    EXPECT_EQ(status.dropped_bytes, 17u);
    EXPECT_EQ(status.dropped_frames, 1u);
    EXPECT_TRUE(any(status.sticky_events & QueueEventFlags::Overflow));
    EXPECT_TRUE(any(status.sticky_events & QueueEventFlags::Truncated));
    EXPECT_EQ(queue.readableFrames(), 1u);
}

TEST(SlaveQueueContract, ObservedRxFrameKeepsPrefixAndLossMetadata)
{
    FixtureStorage<3, 1> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Frame));
    const std::array<uint8_t, 5> payload{{1, 2, 3, 4, 5}};
    FrameMetadata observed = metadata(8, payload.size());
    ASSERT_OK(queue.writeObservedFrame({payload.data(), payload.size()}, observed));

    auto view = queue.peekFrame();
    ASSERT_TRUE(view) << error::toString(view.error());
    EXPECT_EQ(view->metadata.wire_bytes, 5u);
    EXPECT_EQ(view->metadata.stored_bytes, 3u);
    EXPECT_EQ(view->metadata.dropped_bytes, 2u);
    EXPECT_TRUE(any(view->metadata.flags & FrameFlags::Overflow));
    EXPECT_TRUE(any(view->metadata.flags & FrameFlags::Truncated));
    EXPECT_EQ(view->first.data[0], 1u);
    EXPECT_EQ(view->first.data[2], 3u);
    EXPECT_EQ(queue.status().dropped_bytes, 2u);
    EXPECT_EQ(queue.status().dropped_frames, 0u);
}

TEST(SlaveQueueContract, IncrementalObservedFramePublishesOnlyAtCommit)
{
    uint8_t bytes[5]{};
    slave::FrameDescriptor descriptors[2]{};
    slave::SlaveQueue queue;
    ASSERT_OK(queue.bind({{bytes, sizeof(bytes)}, descriptors, 2}, slave::QueueMode::Frame, 41));

    auto reservation = queue.beginObservedFrame();
    ASSERT_TRUE(reservation.has_value());
    EXPECT_EQ(queue.readableFrames(), 0u);

    const uint8_t first[] = {1, 2, 3};
    auto appended         = queue.appendObservedFrame(*reservation, {first, sizeof(first)});
    ASSERT_TRUE(appended.has_value());
    EXPECT_EQ(appended.value(), 3u);
    EXPECT_EQ(queue.readableFrames(), 0u);

    const uint8_t second[] = {4, 5, 6};
    appended               = queue.appendObservedFrame(*reservation, {second, sizeof(second)});
    ASSERT_TRUE(appended.has_value());
    EXPECT_EQ(appended.value(), 2u);
    auto full = queue.appendObservedFrame(*reservation, {second + 2, 1});
    ASSERT_FALSE(full.has_value());
    EXPECT_EQ(full.error(), error_t::WOULD_BLOCK);

    slave::FrameMetadata metadata;
    metadata.frame_id      = 9;
    metadata.wire_bytes    = 6;
    metadata.stored_bytes  = 5;
    metadata.dropped_bytes = 1;
    metadata.flags =
        slave::FrameFlags::Begin | slave::FrameFlags::End | slave::FrameFlags::Overflow | slave::FrameFlags::Truncated;
    ASSERT_OK(queue.commitFrame(*reservation, metadata));

    auto frame = queue.peekFrame();
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->metadata.frame_id, 9u);
    EXPECT_EQ(frame->first.size + frame->second.size, 5u);
    std::array<uint8_t, 5> observed{};
    std::copy(frame->first.begin(), frame->first.end(), observed.begin());
    std::copy(frame->second.begin(), frame->second.end(), observed.begin() + frame->first.size);
    EXPECT_EQ(observed, (std::array<uint8_t, 5>{1, 2, 3, 4, 5}));
}

TEST(SlaveQueueContract, IncrementalObservedFrameTokenRejectsStaleAndCancelPublishesNothing)
{
    uint8_t bytes[4]{};
    slave::FrameDescriptor descriptors[1]{};
    slave::SlaveQueue queue;
    ASSERT_OK(queue.bind({{bytes, sizeof(bytes)}, descriptors, 1}, slave::QueueMode::Frame, 7));

    auto reservation = queue.beginObservedFrame();
    ASSERT_TRUE(reservation.has_value());
    const uint8_t payload[] = {0xA5, 0x5A};
    ASSERT_TRUE(queue.appendObservedFrame(*reservation, {payload, sizeof(payload)}).has_value());
    ASSERT_OK(queue.cancelFrame(*reservation));
    EXPECT_EQ(queue.readableFrames(), 0u);
    EXPECT_EQ(queue.writable(), sizeof(bytes));
    EXPECT_EQ(queue.appendObservedFrame(*reservation, {payload, 1}).error(), error_t::INVALID_STATE);

    auto next = queue.beginObservedFrame();
    ASSERT_TRUE(next.has_value());
    queue.reset(8);
    EXPECT_EQ(queue.appendObservedFrame(*next, {payload, 1}).error(), error_t::INVALID_STATE);
}

TEST(SlaveQueueContract, StaticDuplexStorageBindsBothDirectionsWithoutHeap)
{
    StaticSlaveQueueStorage<7, 9, 2, 3> storage;
    SlaveQueue tx;
    SlaveQueue rx;
    ASSERT_OK(tx.bind(storage.tx(), QueueMode::Frame));
    ASSERT_OK(rx.bind(storage.rx(), QueueMode::Byte));
    EXPECT_EQ(tx.writable(), 7u);
    EXPECT_EQ(tx.writableFrames(), 2u);
    EXPECT_EQ(rx.writable(), 9u);
}

TEST(SlaveQueueContract, ModeIsExplicitAndCanSwitchOnlyWhenEmpty)
{
    FixtureStorage<4, 1> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Byte));
    EXPECT_EQ(queue.mode(), QueueMode::Byte);
    EXPECT_EQ(queue.frameSource().peekFrame().error(), error_t::INVALID_STATE);

    const uint8_t value = 5;
    ASSERT_OK(queue.write({&value, 1}));
    EXPECT_EQ(queue.setMode(QueueMode::Frame).error(), error_t::INVALID_STATE);
    uint8_t output = 0;
    ASSERT_OK(queue.read({&output, 1}));
    ASSERT_OK(queue.setMode(QueueMode::Frame));
    EXPECT_EQ(queue.bytes().write({&value, 1}).error(), error_t::INVALID_STATE);

    auto reservation = queue.reserveFrame(1);
    ASSERT_TRUE(reservation);
    EXPECT_EQ(queue.setMode(QueueMode::Byte).error(), error_t::INVALID_STATE);
    ASSERT_OK(queue.cancelFrame(*reservation));
    ASSERT_OK(queue.setMode(QueueMode::Byte));
}

TEST(SlaveQueueContract, TokensRejectDoubleStaleGenerationAndWrongOwner)
{
    FixtureStorage<4, 2> first_storage;
    FixtureStorage<4, 2> second_storage;
    SlaveQueue first;
    SlaveQueue second;
    ASSERT_OK(first.bind(first_storage.view(), QueueMode::Frame, 1));
    ASSERT_OK(second.bind(second_storage.view(), QueueMode::Frame, 1));

    auto committed = first.reserveFrame(1);
    ASSERT_TRUE(committed);
    ASSERT_OK(first.commitFrame(*committed, metadata(1, 1)));
    EXPECT_EQ(first.commitFrame(*committed, metadata(1, 1)).error(), error_t::INVALID_STATE);
    EXPECT_EQ(first.cancelFrame(*committed).error(), error_t::INVALID_STATE);

    ASSERT_OK(first.popFrame());
    auto stale = first.reserveFrame(1);
    ASSERT_TRUE(stale);
    first.reset(2);
    EXPECT_EQ(first.commitFrame(*stale, metadata(2, 1)).error(), error_t::INVALID_STATE);

    auto wrong_owner = second.reserveFrame(1);
    ASSERT_TRUE(wrong_owner);
    EXPECT_EQ(first.commitFrame(*wrong_owner, metadata(3, 1)).error(), error_t::INVALID_STATE);
    ASSERT_OK(second.cancelFrame(*wrong_owner));
}

TEST(SlaveQueueContract, ResetInvalidatesFramesAndReservationButKeepsMode)
{
    FixtureStorage<4, 2> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Frame, 9));
    const uint8_t value = 8;
    ASSERT_OK(queue.writeFrame({&value, 1}, metadata(1, 1)));
    queue.reset(10);
    EXPECT_EQ(queue.generation(), 10u);
    EXPECT_EQ(queue.mode(), QueueMode::Frame);
    EXPECT_EQ(queue.readable(), 0u);
    EXPECT_EQ(queue.readableFrames(), 0u);
    EXPECT_EQ(queue.peekFrame().error(), error_t::WOULD_BLOCK);
}

TEST(SlaveQueueContract, AccessGenerationChangePreservesPublishedData)
{
    FixtureStorage<4, 2> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Frame, 1));
    const uint8_t value = 8;
    ASSERT_OK(queue.writeFrame({&value, 1}, metadata(1, 1)));
    ASSERT_OK(queue.setGeneration(2));
    EXPECT_EQ(queue.generation(), 2u);
    EXPECT_EQ(queue.readableFrames(), 1u);
    auto view = queue.peekFrame();
    ASSERT_TRUE(view) << error::toString(view.error());
    EXPECT_EQ(view->metadata.frame_id, 1u);

    auto reserved = queue.reserveFrame(1);
    ASSERT_TRUE(reserved) << error::toString(reserved.error());
    EXPECT_EQ(queue.setGeneration(3).error(), error_t::INVALID_STATE);
    ASSERT_OK(queue.cancelFrame(*reserved));
    ASSERT_OK(queue.popFrame());
}

TEST(SlaveQueueContract, RebindRequiresPublishedQueueToBeEmpty)
{
    FixtureStorage<2, 0> first;
    FixtureStorage<3, 0> second;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(first.view(), QueueMode::Byte));
    const uint8_t value = 1;
    ASSERT_OK(queue.write({&value, 1}));
    EXPECT_EQ(queue.bind(second.view(), QueueMode::Byte).error(), error_t::INVALID_STATE);
    uint8_t output = 0;
    ASSERT_OK(queue.read({&output, 1}));
    ASSERT_OK(queue.bind(second.view(), QueueMode::Byte));
    EXPECT_EQ(queue.writable(), 3u);
}

TEST(SlaveQueueContract, BindRejectsUnrepresentableOrMalformedStorage)
{
    SlaveQueue queue;
    QueueStorage<> malformed{{nullptr, 1}, nullptr, 0};
    EXPECT_EQ(queue.bind(malformed, QueueMode::Byte).error(), error_t::INVALID_ARGUMENT);

#if SIZE_MAX > UINT32_MAX
    uint8_t byte = 0;
    QueueStorage<> oversized{{&byte, static_cast<size_t>(std::numeric_limits<uint32_t>::max()) + 1u}, nullptr, 0};
    EXPECT_EQ(queue.bind(oversized, QueueMode::Byte).error(), error_t::INVALID_ARGUMENT);
    FrameDescriptor descriptor;
    QueueStorage<> too_many_descriptors{
        {&byte, 1}, &descriptor, static_cast<size_t>(std::numeric_limits<uint32_t>::max()) + 1u};
    EXPECT_EQ(queue.bind(too_many_descriptors, QueueMode::Frame).error(), error_t::INVALID_ARGUMENT);
#endif

    EXPECT_EQ(sizeof(FrameDescriptor::offset), sizeof(uint32_t));
    EXPECT_EQ(sizeof(FrameDescriptor::length), sizeof(uint32_t));
}

TEST(SlaveQueueContract, ThinViewsAndTokenHaveNoPolymorphicOrCopySurface)
{
    EXPECT_EQ(sizeof(ByteQueueView), sizeof(void*));
    EXPECT_EQ(sizeof(FrameSourceView), sizeof(void*));
    EXPECT_EQ(sizeof(FrameSinkView), sizeof(void*));
    EXPECT_FALSE(std::is_polymorphic<ByteQueueView>::value);
    EXPECT_FALSE(std::is_polymorphic<FrameSourceView>::value);
    EXPECT_FALSE(std::is_polymorphic<FrameSinkView>::value);
    EXPECT_FALSE(std::is_copy_constructible<FrameToken>::value);
    EXPECT_FALSE(std::is_copy_assignable<FrameToken>::value);
    EXPECT_TRUE(std::is_move_constructible<FrameToken>::value);
}

TEST(SlaveQueueContract, StatusCountersSaturateAndClearReturnsSnapshot)
{
    FixtureStorage<1, 1> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Frame));
    queue.recordDroppedFrame(std::numeric_limits<uint32_t>::max());
    queue.recordDroppedFrames(std::numeric_limits<uint32_t>::max(), 99);
    queue.recordDroppedFrame(1);
    queue.recordUnderrun(std::numeric_limits<uint32_t>::max());
    queue.recordUnderrun(1);

    QueueStatus snapshot = queue.status();
    EXPECT_EQ(snapshot.dropped_bytes, std::numeric_limits<uint32_t>::max());
    EXPECT_EQ(snapshot.dropped_frames, std::numeric_limits<uint32_t>::max());
    EXPECT_EQ(snapshot.underrun_count, std::numeric_limits<uint32_t>::max());
    EXPECT_TRUE(any(snapshot.sticky_events & QueueEventFlags::Underrun));

    snapshot = queue.clearStatus();
    EXPECT_EQ(snapshot.dropped_frames, std::numeric_limits<uint32_t>::max());
    EXPECT_EQ(queue.status().dropped_frames, 0u);
    EXPECT_FALSE(any(queue.status().sticky_events));
}

TEST(SlaveQueueContract, DeterministicSpscModelPublishesPayloadBeforeDescriptor)
{
    constexpr uint32_t iterations = 1000;
    FixtureStorage<8, 1> storage;
    SlaveQueue queue;
    ASSERT_OK(queue.bind(storage.view(), QueueMode::Frame));
    std::atomic<uint32_t> producer_stage{0};
    std::atomic<uint32_t> consumer_stage{0};
    std::atomic<uint32_t> mismatch{0};
    constexpr uint32_t failed = std::numeric_limits<uint32_t>::max();

    std::thread producer([&] {
        for (uint32_t id = 1; id <= iterations; ++id) {
            while (consumer_stage.load(std::memory_order_relaxed) != id - 1) {
                if (consumer_stage.load(std::memory_order_relaxed) == failed) {
                    return;
                }
                std::this_thread::yield();
            }
            const std::array<uint8_t, 4> payload{
                {static_cast<uint8_t>(id), static_cast<uint8_t>(id >> 8), static_cast<uint8_t>(~id), 0xA5}};
            auto result = queue.writeFrame({payload.data(), payload.size()}, metadata(id, payload.size()));
            if (!result) {
                mismatch.fetch_add(1, std::memory_order_relaxed);
                producer_stage.store(failed, std::memory_order_relaxed);
                return;
            }
            // Deliberately relaxed: the queue's descriptor-tail release/acquire,
            // not this deterministic turn variable, must publish the payload.
            producer_stage.store(id, std::memory_order_relaxed);
        }
    });

    std::thread consumer([&] {
        for (uint32_t id = 1; id <= iterations; ++id) {
            while (producer_stage.load(std::memory_order_relaxed) != id) {
                if (producer_stage.load(std::memory_order_relaxed) == failed) {
                    return;
                }
                std::this_thread::yield();
            }
            auto view = queue.peekFrame();
            if (!view || view->metadata.frame_id != id || view->first.size != 4 ||
                view->first.data[0] != static_cast<uint8_t>(id) ||
                view->first.data[1] != static_cast<uint8_t>(id >> 8) ||
                view->first.data[2] != static_cast<uint8_t>(~id) || view->first.data[3] != 0xA5) {
                mismatch.fetch_add(1, std::memory_order_relaxed);
                consumer_stage.store(failed, std::memory_order_relaxed);
                return;
            }
            if (!queue.popFrame()) {
                mismatch.fetch_add(1, std::memory_order_relaxed);
                consumer_stage.store(failed, std::memory_order_relaxed);
                return;
            }
            // The producer must acquire the queue heads; this turn variable is
            // only a deterministic scheduler and contributes no ordering.
            consumer_stage.store(id, std::memory_order_relaxed);
        }
    });

    producer.join();
    consumer.join();
    EXPECT_EQ(mismatch.load(), 0u);
    EXPECT_EQ(queue.readableFrames(), 0u);
}

TEST(SlaveQueueContract, FrameFlagsProvideTypedBitOperations)
{
    FrameFlags flags = FrameFlags::Begin | FrameFlags::Overflow;
    EXPECT_TRUE(any(flags & FrameFlags::Begin));
    EXPECT_FALSE(any(flags & FrameFlags::End));
    flags |= FrameFlags::End;
    flags &= ~FrameFlags::Overflow;
    EXPECT_TRUE(any(flags & FrameFlags::End));
    EXPECT_FALSE(any(flags & FrameFlags::Overflow));
}

TEST(SlaveEventContract, CoalescesUntilAcknowledgedAndRejectsCallbackReentry)
{
    SlaveEventEndpoint endpoint;
    EventObservation observation;
    observation.endpoint = &endpoint;
    ASSERT_OK(endpoint.setEventCallback(&observeEvent, &observation));
    ASSERT_OK(endpoint.begin(7));

    endpoint.publish(SlaveEvent::RxAvailable, 3, 5, 7);
    endpoint.publish(SlaveEvent::RxAvailable | SlaveEvent::FrameCompleted, 4, 5, 7);
    ASSERT_OK(endpoint.dispatchEvents());
    EXPECT_EQ(observation.calls, 1u);
    EXPECT_TRUE(any(observation.last.events & SlaveEvent::RxAvailable));
    EXPECT_TRUE(any(observation.last.events & SlaveEvent::FrameCompleted));
    EXPECT_EQ(observation.last.rx_available, 4u);
    EXPECT_EQ(observation.reentry_error, error_t::BUSY);

    endpoint.publish(SlaveEvent::RxAvailable, 6, 5, 7);
    ASSERT_OK(endpoint.dispatchEvents());
    EXPECT_EQ(observation.calls, 1u);
    ASSERT_OK(endpoint.acknowledgeEvents(SlaveEvent::RxAvailable));
    ASSERT_OK(endpoint.dispatchEvents());
    EXPECT_EQ(observation.calls, 2u);
    EXPECT_EQ(observation.last.rx_available, 6u);
    endpoint.end(7);
}

TEST(SlaveEventContract, AcknowledgeRechecksLevelAndOldGenerationIsIgnored)
{
    SlaveEventEndpoint endpoint;
    EventObservation observation;
    endpoint.setLevelProbe(&probeEventLevel, &observation);
    ASSERT_OK(endpoint.setEventCallback(&observeEvent, &observation));
    ASSERT_OK(endpoint.begin(11));

    endpoint.publish(SlaveEvent::TxSpace, 0, 8, 10);
    ASSERT_OK(endpoint.dispatchEvents());
    EXPECT_EQ(observation.calls, 0u);

    endpoint.publish(SlaveEvent::TxSpace, 0, 8, 11);
    ASSERT_OK(endpoint.dispatchEvents());
    ASSERT_EQ(observation.calls, 1u);
    observation.level = {SlaveEvent::TxSpace, 0, 4, 11};
    ASSERT_OK(endpoint.acknowledgeEvents(SlaveEvent::TxSpace));
    ASSERT_OK(endpoint.dispatchEvents());
    EXPECT_EQ(observation.calls, 2u);
    EXPECT_EQ(observation.last.tx_available, 4u);

    observation.level = {};
    ASSERT_OK(endpoint.acknowledgeEvents(SlaveEvent::TxSpace));
    ASSERT_OK(endpoint.dispatchEvents());
    EXPECT_EQ(observation.calls, 2u);
    EXPECT_EQ(endpoint.setEventCallback(nullptr, nullptr).error(), error_t::INVALID_STATE);
    endpoint.end(11);
    ASSERT_OK(endpoint.setEventCallback(nullptr, nullptr));
    EXPECT_EQ(endpoint.dispatchEvents().error(), error_t::INVALID_STATE);
}

TEST(SlaveEventContract, EmbeddedServiceDispatchesInCallerContext)
{
    SlaveEventEndpoint endpoint;
    EventObservation observation;
    ASSERT_OK(endpoint.setEventCallback(&observeEvent, &observation));
    ASSERT_OK(endpoint.begin(3));
    endpoint.publish(SlaveEvent::Overflow, 9, 2, 3);

    auto poll = service::ServiceRunner::run(endpoint.eventService(), service::ServiceContext{});
    EXPECT_EQ(poll.result, service::ServiceResult::Progress);
    EXPECT_EQ(observation.calls, 1u);
    EXPECT_TRUE(any(observation.last.events & SlaveEvent::Overflow));

    poll = service::ServiceRunner::run(endpoint.eventService(), service::ServiceContext{});
    EXPECT_EQ(poll.result, service::ServiceResult::Idle);
    endpoint.end(3);
    poll = service::ServiceRunner::run(endpoint.eventService(), service::ServiceContext{});
    EXPECT_EQ(poll.result, service::ServiceResult::Idle);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
