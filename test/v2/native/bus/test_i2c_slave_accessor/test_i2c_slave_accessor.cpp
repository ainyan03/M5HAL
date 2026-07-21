// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>

#include <gtest/gtest.h>

#include "support/gtest_watchdog.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace {

using namespace m5::hal::v2;

#define ASSERT_OK(expression)                          \
    do {                                               \
        auto result = (expression);                    \
        if (!result.has_value()) {                     \
            FAIL() << error::toString(result.error()); \
        }                                              \
    } while (false)

class FakeSlaveBus : public i2c::ISlaveBus {
public:
    using Context = bus::OperationContext<i2c::SlaveAccessConfig>;

    result_t<void> init(const i2c::SlaveBusConfig& config) override
    {
        _config = config;
        return {};
    }

    result_t<void> lockFor(bus::IAccessor& accessor)
    {
        return acquireAccessLock(accessor, 0);
    }

    result_t<void> unlockFor(bus::IAccessor& accessor)
    {
        return releaseAccessLock(accessor);
    }

    Context& capturedContext()
    {
        return *captured_context;
    }

protected:
    bus::CloseOutcome closeBackend(void) override
    {
        active = false;
        return bus::CloseOutcome::success();
    }
    result_t<void> releaseAccessLock(bus::IAccessor& accessor) override
    {
        if (fail_unlock) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        return i2c::ISlaveBus::releaseAccessLock(accessor);
    }

public:
    result_t<void> tryOpenWireFrame(bus::IAccessor*) override
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    result_t<void> closeWireFrame(bus::IAccessor*) override
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    result_t<size_t> read(bus::IAccessor*, data::DataSpan) override
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    result_t<size_t> write(bus::IAccessor*, data::ConstDataSpan) override
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    result_t<size_t> readableBytes(bus::IAccessor*) override
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    result_t<bool> wireFrameComplete(bus::IAccessor*) override
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    service::IService* service() override
    {
        return nullptr;
    }

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<i2c::SlaveAccessConfig>& context) override
    {
        ++begin_backend_calls;
        captured_context = &context;
        if (fail_begin) {
            return m5::stl::make_unexpected(error::error_t::IO_ERROR);
        }
        auto& accessor = operationOwner(context);
        active         = true;
        owner          = &accessor;
        generation     = context.runtime.generation;
        operation_log.push_back(1);
        return {};
    }

    result_t<void> endOperationBackend(bus::OperationContext<i2c::SlaveAccessConfig>& context) override
    {
        ++end_backend_calls;
        ended_generation = context.runtime.generation;
        ended_mode       = context.runtime.mode;
        auto& accessor   = operationOwner(context);
        if (!active || owner != &accessor) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        active = false;
        operation_log.push_back(2);
        if (fail_end) {
            return m5::stl::make_unexpected(error::error_t::IO_ERROR);
        }
        return {};
    }

public:
    result_t<void> simulateFrame(data::ConstDataSpan received, const std::vector<i2c::I2cFrameSegment>& segments,
                                 uint32_t wire_bytes, bool exact = true)
    {
        if (!active || owner == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        auto& accessor = static_cast<i2c::SlaveAccessor&>(*owner);
        auto writer    = accessor.backendRxFrames();
        auto frame     = writer.beginFrame();
        if (!frame.has_value()) {
            return m5::stl::make_unexpected(frame.error());
        }

        size_t stored = 0;
        auto appended = writer.appendReceived(*frame, received);
        if (appended.has_value()) {
            stored = *appended;
        } else if (appended.error() != error::error_t::WOULD_BLOCK) {
            (void)writer.cancelFrame(*frame);
            return m5::stl::make_unexpected(appended.error());
        }
        if (exact) {
            for (const auto& segment : segments) {
                auto added = writer.appendSegment(*frame, segment);
                if (!added.has_value()) {
                    if (added.error() == error::error_t::WOULD_BLOCK ||
                        added.error() == error::error_t::OUT_OF_RESOURCE) {
                        break;
                    }
                    (void)writer.cancelFrame(*frame);
                    return added;
                }
            }
        }

        slave::FrameMetadata metadata;
        metadata.frame_id   = ++frame_id;
        metadata.wire_bytes = wire_bytes;
        metadata.flags      = slave::FrameFlags::Begin | slave::FrameFlags::End;
        auto committed      = writer.commitFrame(*frame, metadata);
        if (!committed.has_value()) {
            return committed;
        }
        slave::SlaveEvent event = slave::SlaveEvent::FrameCompleted;
        if (stored != 0) {
            event |= slave::SlaveEvent::RxAvailable;
        }
        if (stored != received.size) {
            event |= slave::SlaveEvent::Overflow;
        }
        accessor.backendEvents().publish(event, accessor.readable(), accessor.writable(), generation);
        return {};
    }

    result_t<void> leaveObservedFrameOpen()
    {
        if (!active || owner == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        auto& accessor = static_cast<i2c::SlaveAccessor&>(*owner);
        leaked_writer  = accessor.backendRxFrames();
        auto frame     = leaked_writer.beginFrame();
        if (!frame.has_value()) {
            return m5::stl::make_unexpected(frame.error());
        }
        leaked_frame = std::move(*frame);
        return {};
    }

    bool active                   = false;
    bool fail_begin               = false;
    bool fail_end                 = false;
    bool fail_unlock              = false;
    bus::IAccessor* owner         = nullptr;
    uint32_t generation           = 0;
    uint32_t ended_generation     = 0;
    uint32_t frame_id             = 0;
    size_t begin_backend_calls    = 0;
    size_t end_backend_calls      = 0;
    bus::OperationMode ended_mode = bus::OperationMode::Control;
    Context* captured_context     = nullptr;
    std::vector<int> operation_log;
    i2c::I2cObservedFrameWriter leaked_writer;
    i2c::I2cObservedFrameReservation leaked_frame;
};

class I2cSlaveContextProbeAccessor : public bus::IAccessor {
public:
    explicit I2cSlaveContextProbeAccessor(FakeSlaveBus& bus)
        : bus::IAccessor{bus}, context{makeOperationContext(config)}
    {
    }

    const bus::IAccessConfig& getConfig() const override
    {
        return config;
    }

    i2c::SlaveAccessConfig config;
    bus::OperationContext<i2c::SlaveAccessConfig> context;
};

template <size_t TxBytes, size_t RxBytes, size_t TxFrames, size_t RxFrames, size_t Segments>
struct AccessorFixture {
    slave::StaticSlaveQueueStorage<TxBytes, RxBytes, TxFrames, RxFrames> queue_storage;
    i2c::StaticI2cSegmentStorage<Segments> segment_storage;
    i2c::SlaveAccessor accessor;

    AccessorFixture(FakeSlaveBus& bus, const i2c::SlaveAccessConfig& config = {})
        : accessor{bus, queue_storage.tx(), queue_storage.rx(), segment_storage.storage(), config}
    {
    }
};

i2c::SlaveAccessConfig framedRxConfig()
{
    i2c::SlaveAccessConfig config;
    config.rx_mode = slave::QueueMode::Frame;
    return config;
}

struct EventObservation {
    uint32_t calls = 0;
    slave::SlaveEventInfo last{};
};

void observeEvent(void* user, const slave::SlaveEventInfo& info)
{
    auto& observation = *static_cast<EventObservation*>(user);
    ++observation.calls;
    observation.last = info;
}

}  // namespace

TEST(I2cSlaveAccessor, DefaultsUsePortableByteModes)
{
    i2c::SlaveAccessConfig config;
    EXPECT_EQ(config.getBusKind(), types::bus_kind_t::I2C);
    EXPECT_EQ(config.tx_mode, slave::QueueMode::Byte);
    EXPECT_EQ(config.rx_mode, slave::QueueMode::Byte);
}

TEST(I2cSlaveAccessor, AccessLifecycleIsNonNestedAndBeginFailureUnlocks)
{
    FakeSlaveBus bus;
    AccessorFixture<8, 8, 2, 2, 4> first{bus};
    AccessorFixture<8, 8, 2, 2, 4> second{bus};

    ASSERT_OK(first.accessor.beginAccess(0));
    EXPECT_TRUE(first.accessor.inAccess());
    EXPECT_EQ(first.accessor.beginAccess(0).error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(second.accessor.beginAccess(0).error(), error::error_t::TIMEOUT_ERROR);
    ASSERT_OK(first.accessor.endAccess(0));
    EXPECT_FALSE(first.accessor.inAccess());

    bus.fail_begin = true;
    EXPECT_EQ(first.accessor.beginAccess(0).error(), error::error_t::IO_ERROR);
    bus.fail_begin = false;
    ASSERT_OK(second.accessor.beginAccess(0));
    ASSERT_OK(second.accessor.endAccess(0));

    bus.fail_end = true;
    ASSERT_OK(first.accessor.beginAccess(0));
    EXPECT_EQ(first.accessor.endAccess(0).error(), error::error_t::IO_ERROR);
    bus.fail_end = false;
    ASSERT_OK(second.accessor.beginAccess(0));
    ASSERT_OK(second.accessor.endAccess(0));
}

TEST(I2cSlaveCheckedFacade, RejectsEndedWrongBusAndWrongAccessorContexts)
{
    FakeSlaveBus first_bus;
    FakeSlaveBus second_bus;
    I2cSlaveContextProbeAccessor never_started{first_bus};
    AccessorFixture<4, 4, 0, 1, 1> first{first_bus};
    AccessorFixture<4, 4, 0, 1, 1> second{first_bus};

    auto inactive_begin = first_bus.beginOperation(never_started.context);
    ASSERT_FALSE(inactive_begin.has_value());
    EXPECT_EQ(inactive_begin.error(), error::error_t::INVALID_STATE);
    auto inactive_end = first_bus.endOperation(never_started.context);
    ASSERT_FALSE(inactive_end.has_value());
    EXPECT_EQ(inactive_end.error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(first_bus.begin_backend_calls, 0u);
    EXPECT_EQ(first_bus.end_backend_calls, 0u);

    ASSERT_OK(first.accessor.beginAccess(0));
    auto& context  = first_bus.capturedContext();
    auto wrong_bus = second_bus.endOperation(context);
    ASSERT_FALSE(wrong_bus.has_value());
    EXPECT_EQ(wrong_bus.error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(second_bus.end_backend_calls, 0u);
    ASSERT_OK(first.accessor.endAccess(0));

    const size_t end_calls = first_bus.end_backend_calls;
    auto ended             = first_bus.endOperation(context);
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(first_bus.end_backend_calls, end_calls);

    context.runtime.begin(0, 0, bus::OperationMode::Slave);
    ASSERT_OK(first_bus.lockFor(second.accessor));
    const size_t begin_calls = first_bus.begin_backend_calls;
    auto wrong_accessor      = first_bus.beginOperation(context);
    ASSERT_FALSE(wrong_accessor.has_value());
    EXPECT_EQ(wrong_accessor.error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(first_bus.begin_backend_calls, begin_calls);
    ASSERT_OK(first_bus.unlockFor(second.accessor));
}

TEST(I2cSlaveCheckedFacade, CorruptRuntimeIsRestoredForBackendCleanup)
{
    FakeSlaveBus bus;
    AccessorFixture<4, 4, 0, 1, 1> first{bus};
    AccessorFixture<4, 4, 0, 1, 1> second{bus};

    ASSERT_OK(first.accessor.beginAccess(0));
    auto& context                        = bus.capturedContext();
    const uint32_t registered_generation = context.runtime.generation;
    const size_t end_calls               = bus.end_backend_calls;
    ++context.runtime.generation;
    context.runtime.mode = bus::OperationMode::Tx;

    auto ended = first.accessor.endAccess(0);
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(bus.end_backend_calls, end_calls + 1);
    EXPECT_EQ(bus.ended_generation, registered_generation);
    EXPECT_EQ(bus.ended_mode, bus::OperationMode::Slave);
    EXPECT_FALSE(bus.active);
    EXPECT_FALSE(first.accessor.inAccess());

    ASSERT_OK(second.accessor.beginAccess(0));
    ASSERT_OK(second.accessor.endAccess(0));
}

TEST(I2cSlaveCheckedFacade, FailedRuntimeRestoreSkipsBackendAndRecoversSlotAndLock)
{
    FakeSlaveBus bus;
    AccessorFixture<4, 4, 0, 1, 1> first{bus};
    AccessorFixture<4, 4, 0, 1, 1> second{bus};

    ASSERT_OK(first.accessor.beginAccess(0));
    auto& context          = bus.capturedContext();
    const size_t end_calls = bus.end_backend_calls;
    ++context.runtime.generation;

    ASSERT_OK(bus.unlockFor(first.accessor));
    ASSERT_OK(bus.lockFor(second.accessor));
    auto skipped = bus.endOperation(context);
    ASSERT_FALSE(skipped.has_value());
    EXPECT_EQ(skipped.error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(bus.end_backend_calls, end_calls);
    EXPECT_TRUE(bus.active);
    ASSERT_OK(bus.unlockFor(second.accessor));
    ASSERT_OK(bus.lockFor(first.accessor));

    auto ended = first.accessor.endAccess(0);
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(bus.end_backend_calls, end_calls);
    EXPECT_FALSE(first.accessor.inAccess());

    ASSERT_OK(second.accessor.beginAccess(0));
    ASSERT_OK(second.accessor.endAccess(0));
}

TEST(I2cSlaveAccessor, BeginRollbackUnlockFailureMarksBusBroken)
{
    FakeSlaveBus bus;
    AccessorFixture<2, 2, 0, 1, 1> first{bus};
    AccessorFixture<2, 2, 0, 1, 1> second{bus};
    bus.fail_begin  = true;
    bus.fail_unlock = true;
    EXPECT_EQ(first.accessor.beginAccess(0).error(), error::error_t::IO_ERROR);
    bus.fail_begin  = false;
    bus.fail_unlock = false;
    EXPECT_EQ(second.accessor.beginAccess(0).error(), error::error_t::INVALID_STATE);
}

TEST(I2cSlaveAccessor, EndUnlockFailureMarksBusBrokenAfterCleanup)
{
    FakeSlaveBus bus;
    AccessorFixture<2, 2, 0, 1, 1> first{bus};
    AccessorFixture<2, 2, 0, 1, 1> second{bus};
    ASSERT_OK(first.accessor.beginAccess(0));
    bus.fail_unlock = true;
    EXPECT_EQ(first.accessor.endAccess(0).error(), error::error_t::INVALID_ARGUMENT);
    EXPECT_FALSE(first.accessor.inAccess());
    bus.fail_unlock = false;
    EXPECT_EQ(second.accessor.beginAccess(0).error(), error::error_t::INVALID_STATE);
}

TEST(I2cSlaveAccessor, ExactRepeatedStartDetailsAndPayloadPopTogether)
{
    FakeSlaveBus bus;
    AccessorFixture<8, 8, 0, 2, 4> fixture{bus, framedRxConfig()};
    ASSERT_OK(fixture.accessor.beginAccess(0));

    const uint8_t received[] = {0x20, 0xAB};
    const std::vector<i2c::I2cFrameSegment> segments{
        {0, 2, i2c::I2cFrameDirection::Write, i2c::I2cSegmentFlags::None, 0},
        {2, 3, i2c::I2cFrameDirection::Read, i2c::I2cSegmentFlags::RepeatedStart, 0},
    };
    ASSERT_OK(bus.simulateFrame({received, sizeof(received)}, segments, 99));
    ASSERT_OK(fixture.accessor.endAccess(0));

    auto frame = fixture.accessor.rxFrames().peekFrame();
    ASSERT_TRUE(frame.has_value()) << error::toString(frame.error());
    EXPECT_EQ(frame->frame.metadata.wire_bytes, 5u);
    EXPECT_EQ(frame->frame.metadata.stored_bytes, 2u);
    EXPECT_EQ(frame->frame.metadata.segment_count, 2u);
    EXPECT_EQ(frame->segments.first.size + frame->segments.second.size, 2u);
    EXPECT_EQ(frame->segments.first.data[0].direction, i2c::I2cFrameDirection::Write);
    EXPECT_EQ(frame->segments.first.data[1].direction, i2c::I2cFrameDirection::Read);
    EXPECT_TRUE(i2c::any(frame->segments.first.data[1].flags & i2c::I2cSegmentFlags::RepeatedStart));
    ASSERT_OK(fixture.accessor.rxFrames().popFrame());
    EXPECT_EQ(fixture.accessor.rxFrames().readableFrames(), 0u);
    EXPECT_EQ(fixture.accessor.rxFrames().peekFrame().error(), error::error_t::WOULD_BLOCK);
}

TEST(I2cSlaveAccessor, ExactWriterRejectsNonContiguousOrMisflaggedSegments)
{
    FakeSlaveBus bus;
    AccessorFixture<2, 2, 0, 1, 2> fixture{bus, framedRxConfig()};
    ASSERT_OK(fixture.accessor.beginAccess(0));
    auto writer = fixture.accessor.backendRxFrames();
    auto frame  = writer.beginFrame();
    ASSERT_TRUE(frame.has_value()) << error::toString(frame.error());
    EXPECT_EQ(
        writer.appendSegment(*frame, {0, 1, i2c::I2cFrameDirection::Write, i2c::I2cSegmentFlags::RepeatedStart, 0})
            .error(),
        error::error_t::INVALID_ARGUMENT);
    ASSERT_OK(writer.appendSegment(*frame, {0, 1, i2c::I2cFrameDirection::Write, i2c::I2cSegmentFlags::None, 0}));
    EXPECT_EQ(writer.appendSegment(*frame, {2, 1, i2c::I2cFrameDirection::Read, i2c::I2cSegmentFlags::RepeatedStart, 0})
                  .error(),
              error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(writer.appendSegment(*frame, {1, 1, i2c::I2cFrameDirection::Read, i2c::I2cSegmentFlags::None, 0}).error(),
              error::error_t::INVALID_ARGUMENT);
    ASSERT_OK(writer.cancelFrame(*frame));
    ASSERT_OK(fixture.accessor.endAccess(0));
}

TEST(I2cSlaveAccessor, SegmentDetailsMayBeUnavailableWithoutFabrication)
{
    FakeSlaveBus bus;
    AccessorFixture<4, 4, 0, 1, 0> fixture{bus, framedRxConfig()};
    ASSERT_OK(fixture.accessor.beginAccess(0));
    const uint8_t received = 7;
    ASSERT_OK(bus.simulateFrame({&received, 1}, {}, 1, false));
    ASSERT_OK(fixture.accessor.endAccess(0));
    auto frame = fixture.accessor.rxFrames().peekFrame();
    ASSERT_TRUE(frame.has_value()) << error::toString(frame.error());
    EXPECT_EQ(frame->frame.metadata.segment_count, 0u);
    EXPECT_EQ(frame->segments.first.size + frame->segments.second.size, 0u);
}

TEST(I2cSlaveAccessor, SegmentExhaustionDropsOnlyDetailAndPreservesCommonFrame)
{
    FakeSlaveBus bus;
    AccessorFixture<4, 4, 0, 1, 1> fixture{bus, framedRxConfig()};
    ASSERT_OK(fixture.accessor.beginAccess(0));
    const uint8_t received[] = {0x20, 0xAB};
    const std::vector<i2c::I2cFrameSegment> segments{
        {0, 2, i2c::I2cFrameDirection::Write, i2c::I2cSegmentFlags::None, 0},
        {2, 3, i2c::I2cFrameDirection::Read, i2c::I2cSegmentFlags::RepeatedStart, 0},
    };
    ASSERT_OK(bus.simulateFrame({received, sizeof(received)}, segments, 5));
    ASSERT_OK(fixture.accessor.endAccess(0));
    auto frame = fixture.accessor.rxFrames().peekFrame();
    ASSERT_TRUE(frame.has_value()) << error::toString(frame.error());
    EXPECT_EQ(frame->frame.metadata.wire_bytes, 5u);
    EXPECT_EQ(frame->frame.metadata.stored_bytes, 2u);
    EXPECT_EQ(frame->frame.metadata.segment_count, 0u);
    EXPECT_EQ(frame->frame.first.size + frame->frame.second.size, 2u);
    EXPECT_EQ(frame->segments.first.size + frame->segments.second.size, 0u);
}

TEST(I2cSlaveAccessor, PreloadedTxAndCompletedRxPersistAcrossAccessBoundary)
{
    FakeSlaveBus bus;
    AccessorFixture<4, 4, 0, 1, 1> fixture{bus, framedRxConfig()};
    const uint8_t response[] = {0x11, 0x22};
    ASSERT_OK(fixture.accessor.write({response, sizeof(response)}));
    ASSERT_OK(fixture.accessor.beginAccess(0));
    const uint8_t received[] = {1, 2};
    ASSERT_OK(bus.simulateFrame({received, sizeof(received)}, {}, 2, false));
    ASSERT_OK(fixture.accessor.endAccess(0));
    EXPECT_EQ(fixture.accessor.writable(), 2u);
    auto frame = fixture.accessor.rxFrames().peekFrame();
    ASSERT_TRUE(frame.has_value()) << error::toString(frame.error());
    EXPECT_EQ(frame->frame.metadata.segment_count, 0u);
    EXPECT_EQ(frame->segments.first.size + frame->segments.second.size, 0u);
}

TEST(I2cSlaveAccessor, ReservationsCannotCrossAccessBoundary)
{
    FakeSlaveBus bus;
    i2c::SlaveAccessConfig config;
    config.tx_mode = slave::QueueMode::Frame;
    AccessorFixture<4, 4, 1, 1, 1> fixture{bus, config};
    auto pending = fixture.accessor.txFrames().reserveFrame(1);
    ASSERT_TRUE(pending.has_value()) << error::toString(pending.error());
    EXPECT_EQ(fixture.accessor.beginAccess(0).error(), error::error_t::INVALID_STATE);
    ASSERT_OK(fixture.accessor.txFrames().cancelFrame(*pending));
    ASSERT_OK(fixture.accessor.beginAccess(0));

    auto leaked = fixture.accessor.txFrames().reserveFrame(1);
    ASSERT_TRUE(leaked.has_value()) << error::toString(leaked.error());
    EXPECT_EQ(fixture.accessor.endAccess(0).error(), error::error_t::INVALID_STATE);
    EXPECT_FALSE(fixture.accessor.inAccess());
    EXPECT_EQ(fixture.accessor.txFrames().commitFrame(*leaked, {}).error(), error::error_t::INVALID_STATE);
}

TEST(I2cSlaveAccessor, BackendReservationLeakIsCancelledDuringEnd)
{
    FakeSlaveBus bus;
    AccessorFixture<4, 4, 0, 1, 1> fixture{bus, framedRxConfig()};
    ASSERT_OK(fixture.accessor.beginAccess(0));
    ASSERT_OK(bus.leaveObservedFrameOpen());
    EXPECT_EQ(fixture.accessor.endAccess(0).error(), error::error_t::INVALID_STATE);
    EXPECT_FALSE(fixture.accessor.inAccess());
    ASSERT_OK(fixture.accessor.beginAccess(0));
    ASSERT_OK(fixture.accessor.endAccess(0));
}

TEST(I2cSlaveAccessor, FrameAndOverflowEventsRemainLevelObservable)
{
    FakeSlaveBus bus;
    AccessorFixture<2, 1, 0, 1, 0> fixture{bus, framedRxConfig()};
    EventObservation observation;
    ASSERT_OK(fixture.accessor.setEventCallback(&observeEvent, &observation));
    ASSERT_OK(fixture.accessor.beginAccess(0));
    const uint8_t received[] = {1, 2};
    ASSERT_OK(bus.simulateFrame({received, sizeof(received)}, {}, 2, false));
    ASSERT_OK(fixture.accessor.dispatchEvents());
    EXPECT_EQ(observation.calls, 1u);
    EXPECT_TRUE(slave::any(observation.last.events & slave::SlaveEvent::FrameCompleted));
    EXPECT_TRUE(slave::any(observation.last.events & slave::SlaveEvent::Overflow));
    EXPECT_EQ(fixture.accessor.rxStatus().dropped_bytes, 1u);
    ASSERT_OK(fixture.accessor.acknowledgeEvents(observation.last.events));
    ASSERT_OK(fixture.accessor.endAccess(0));
}

TEST(I2cSlaveAccessor, ExplicitClearIsInactiveOnlyAndKeepsRingsAligned)
{
    FakeSlaveBus bus;
    AccessorFixture<2, 2, 0, 1, 1> fixture{bus, framedRxConfig()};
    ASSERT_OK(fixture.accessor.beginAccess(0));
    const uint8_t received = 3;
    ASSERT_OK(
        bus.simulateFrame({&received, 1}, {{0, 1, i2c::I2cFrameDirection::Write, i2c::I2cSegmentFlags::None, 0}}, 1));
    EXPECT_EQ(fixture.accessor.clearRx().error(), error::error_t::INVALID_STATE);
    ASSERT_OK(fixture.accessor.endAccess(0));
    ASSERT_OK(fixture.accessor.clearRx());
    EXPECT_EQ(fixture.accessor.rxFrames().readableFrames(), 0u);
    ASSERT_OK(fixture.accessor.beginAccess(0));
    ASSERT_OK(fixture.accessor.endAccess(0));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
