// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <m5_hal/variants/frameworks/espidf/hal/spi/spi.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <algorithm>
#include <cstring>
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

class FakeSlaveBus : public spi::ISlaveBus {
public:
    using Context = bus::OperationContext<spi::SlaveAccessConfig>;

    result_t<void> init(const spi::SlaveBusConfig& cfg) override
    {
        _config = cfg;
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

    result_t<void> beginOperationBackend(bus::OperationContext<spi::SlaveAccessConfig>& context) override
    {
        ++begin_backend_calls;
        captured_context = &context;
        if (fail_begin) {
            return m5::stl::make_unexpected(error::error_t::IO_ERROR);
        }
        auto& owner = operationOwner(context);
        active      = true;
        last_owner  = &owner;
        generation  = context.runtime.generation;
        operation_log.push_back(1);
        return {};
    }

    result_t<void> endOperationBackend(bus::OperationContext<spi::SlaveAccessConfig>& context) override
    {
        ++end_backend_calls;
        ended_generation = context.runtime.generation;
        ended_mode       = context.runtime.mode;
        auto& owner      = operationOwner(context);
        if (!active || &owner != last_owner) {
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
    result_t<void> simulateFrame(data::ConstDataSpan master_tx)
    {
        if (!active || last_owner == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        auto& accessor = static_cast<spi::SpiSlaveAccessor&>(*last_owner);
        auto& tx       = accessor.backendTxQueue();
        auto& rx       = accessor.backendRxQueue();
        served.assign(master_tx.size, _config.tx_fill_byte);

        size_t available_tx = 0;
        const bool tx_frame = tx.mode() == slave::QueueMode::Frame;
        if (tx_frame) {
            auto view = tx.peekFrame();
            if (view.has_value()) {
                available_tx       = view->first.size + view->second.size;
                const size_t first = std::min(view->first.size, master_tx.size);
                std::memcpy(served.data(), view->first.data, first);
                const size_t second = std::min(view->second.size, master_tx.size - first);
                std::memcpy(served.data() + first, view->second.data, second);
                auto popped = tx.popFrame();
                if (!popped.has_value()) {
                    return popped;
                }
            }
        } else {
            auto view = tx.peekBytes(master_tx.size);
            if (view.has_value()) {
                available_tx = view->first.size + view->second.size;
                std::memcpy(served.data(), view->first.data, view->first.size);
                std::memcpy(served.data() + view->first.size, view->second.data, view->second.size);
                auto popped = tx.popBytes(available_tx);
                if (!popped.has_value()) {
                    return popped;
                }
            }
        }

        slave::FrameFlags flags = slave::FrameFlags::Begin | slave::FrameFlags::End;
        if (available_tx < master_tx.size) {
            tx.recordUnderrun();
            flags |= slave::FrameFlags::Underrun;
        } else if (tx_frame && available_tx > master_tx.size) {
            flags |= slave::FrameFlags::Truncated;
        }

        if (rx.mode() == slave::QueueMode::Frame) {
            slave::FrameMetadata metadata;
            metadata.frame_id = ++frame_id;
            metadata.flags    = flags;
            auto stored       = rx.writeObservedFrame(master_tx, metadata);
            if (!stored.has_value() && stored.error() != error::error_t::WOULD_BLOCK) {
                return stored;
            }
        } else {
            auto stored = rx.write(master_tx);
            if (!stored.has_value()) {
                if (stored.error() != error::error_t::WOULD_BLOCK) {
                    return m5::stl::make_unexpected(stored.error());
                }
                rx.recordDroppedFrame(static_cast<uint32_t>(master_tx.size));
            } else if (*stored < master_tx.size) {
                rx.recordDroppedFrame(static_cast<uint32_t>(master_tx.size - *stored));
            }
        }

        slave::SlaveEvent event = slave::SlaveEvent::FrameCompleted;
        if (rx.readable() != 0) {
            event |= slave::SlaveEvent::RxAvailable;
        }
        if (available_tx < master_tx.size) {
            event |= slave::SlaveEvent::Underrun;
        }
        if (slave::any(rx.status().sticky_events & slave::QueueEventFlags::Overflow)) {
            event |= slave::SlaveEvent::Overflow;
        }
        accessor.backendEvents().publish(event, rx.readable(), tx.writable(), generation);
        return {};
    }

    bool active     = false;
    bool fail_begin = false;
    bool fail_end   = false;
    std::vector<int> operation_log;
    std::vector<uint8_t> served;
    bus::IAccessor* last_owner    = nullptr;
    uint32_t generation           = 0;
    uint32_t ended_generation     = 0;
    uint32_t frame_id             = 0;
    size_t begin_backend_calls    = 0;
    size_t end_backend_calls      = 0;
    bus::OperationMode ended_mode = bus::OperationMode::Control;
    Context* captured_context     = nullptr;
};

class SpiSlaveContextProbeAccessor : public bus::IAccessor {
public:
    explicit SpiSlaveContextProbeAccessor(FakeSlaveBus& bus)
        : bus::IAccessor{bus}, context{makeOperationContext(config)}
    {
    }

    const bus::IAccessConfig& getConfig() const override
    {
        return config;
    }

    spi::SlaveAccessConfig config;
    bus::OperationContext<spi::SlaveAccessConfig> context;
};

template <size_t TxBytes, size_t RxBytes, size_t TxFrames, size_t RxFrames>
struct AccessorFixture {
    slave::StaticSlaveQueueStorage<TxBytes, RxBytes, TxFrames, RxFrames> storage;
    spi::SpiSlaveAccessor accessor;

    AccessorFixture(FakeSlaveBus& bus, const spi::SlaveAccessConfig& config = {})
        : accessor{bus, storage.tx(), storage.rx(), config}
    {
    }
};

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

TEST(SpiSlaveBusConfig, DefaultsSeparateBusAndAccessConfiguration)
{
    spi::SlaveBusConfig bus_config;
    EXPECT_EQ(bus_config.getBusKind(), types::bus_kind_t::SPI);
    EXPECT_EQ(bus_config.controller, -1);
    EXPECT_EQ(bus_config.tx_fill_byte, 0u);

    spi::SlaveAccessConfig access_config;
    EXPECT_EQ(access_config.getBusKind(), types::bus_kind_t::SPI);
    EXPECT_EQ(access_config.transaction_bytes, 4096u);
    EXPECT_EQ(access_config.tx_mode, slave::QueueMode::Byte);
    EXPECT_EQ(access_config.rx_mode, slave::QueueMode::Frame);
}

TEST(EspidfSpiControllerMap, ControllerAndHostOrdinalsRoundTrip)
{
    namespace detail         = spi::detail_espidf_spi;
    constexpr uint8_t count  = 2;
    constexpr int first_host = 1;
    int host                 = -1;
    EXPECT_TRUE(detail::hostOrdinalForController(0, count, first_host, host));
    EXPECT_EQ(host, 1);
    EXPECT_TRUE(detail::hostOrdinalForController(1, count, first_host, host));
    EXPECT_EQ(host, 2);
    EXPECT_FALSE(detail::hostOrdinalForController(2, count, first_host, host));
    int8_t controller = -1;
    EXPECT_TRUE(detail::controllerForHostOrdinal(2, count, first_host, controller));
    EXPECT_EQ(controller, 1);
}

TEST(EspidfSpiControllerMap, SlaveDefaultAndInvalidControllersAreDistinct)
{
    namespace detail         = spi::detail_espidf_spi;
    constexpr uint8_t count  = 2;
    constexpr int first_host = 1;
    int host                 = -1;
    EXPECT_TRUE(detail::slaveHostOrdinal(-1, count, first_host, host));
    EXPECT_EQ(host, 1);
    EXPECT_TRUE(detail::slaveHostOrdinal(1, count, first_host, host));
    EXPECT_EQ(host, 2);
    EXPECT_FALSE(detail::slaveHostOrdinal(2, count, first_host, host));
}

TEST(EspidfSpiCloseOrder, FailurePreservesResourcesAndSuccessStopsLast)
{
    namespace detail = spi::detail_espidf_spi;
    std::vector<int> calls;
    auto failure = detail::releaseDriverBeforeWorker(
        true, false,
        [&] {
            calls.push_back(1);
            return error::error_t::IO_ERROR;
        },
        [&] {
            calls.push_back(2);
            return error::error_t::OK;
        },
        [&] { calls.push_back(3); });
    EXPECT_EQ(failure.error, error::error_t::IO_ERROR);
    EXPECT_EQ(calls, (std::vector<int>{1}));

    calls.clear();
    auto success = detail::releaseDriverBeforeWorker(
        true, false,
        [&] {
            calls.push_back(1);
            return error::error_t::OK;
        },
        [&] {
            calls.push_back(2);
            return error::error_t::OK;
        },
        [&] { calls.push_back(3); });
    EXPECT_EQ(success.error, error::error_t::OK);
    EXPECT_EQ(calls, (std::vector<int>{1, 2, 3}));
}

TEST(SpiSlaveAccessor, AccessLifecycleIsNonNestedAndBeginFailureUnlocks)
{
    FakeSlaveBus bus;
    AccessorFixture<8, 8, 2, 2> fixture{bus};
    auto& accessor = fixture.accessor;
    ASSERT_OK(accessor.beginAccess(0));
    EXPECT_TRUE(accessor.inAccess());
    EXPECT_EQ(accessor.beginAccess(0).error(), error::error_t::INVALID_STATE);

    AccessorFixture<4, 4, 1, 1> contender{bus};
    EXPECT_EQ(contender.accessor.beginAccess(0).error(), error::error_t::TIMEOUT_ERROR);
    ASSERT_OK(accessor.endAccess(0));
    EXPECT_FALSE(accessor.inAccess());
    EXPECT_EQ(accessor.endAccess(0).error(), error::error_t::INVALID_STATE);

    bus.fail_begin = true;
    EXPECT_EQ(accessor.beginAccess(0).error(), error::error_t::IO_ERROR);
    bus.fail_begin = false;
    ASSERT_OK(contender.accessor.beginAccess(0));
    ASSERT_OK(contender.accessor.endAccess(0));

    bus.fail_end = true;
    ASSERT_OK(accessor.beginAccess(0));
    EXPECT_EQ(accessor.endAccess(0).error(), error::error_t::IO_ERROR);
    bus.fail_end = false;
    ASSERT_OK(contender.accessor.beginAccess(0));
    ASSERT_OK(contender.accessor.endAccess(0));
}

TEST(SpiSlaveCheckedFacade, RejectsEndedWrongBusAndWrongAccessorContexts)
{
    FakeSlaveBus first_bus;
    FakeSlaveBus second_bus;
    SpiSlaveContextProbeAccessor never_started{first_bus};
    AccessorFixture<4, 4, 0, 1> first{first_bus};
    AccessorFixture<4, 4, 0, 1> second{first_bus};

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

TEST(SpiSlaveCheckedFacade, CorruptRuntimeIsRestoredForBackendCleanup)
{
    FakeSlaveBus bus;
    AccessorFixture<4, 4, 0, 1> first{bus};
    AccessorFixture<4, 4, 0, 1> second{bus};

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

TEST(SpiSlaveCheckedFacade, FailedRuntimeRestoreSkipsBackendAndRecoversSlotAndLock)
{
    FakeSlaveBus bus;
    AccessorFixture<4, 4, 0, 1> first{bus};
    AccessorFixture<4, 4, 0, 1> second{bus};

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

TEST(SpiSlaveAccessor, PreloadedTxAndCompletedRxPersistAcrossAccessBoundary)
{
    FakeSlaveBus bus;
    spi::SlaveBusConfig bus_config;
    bus_config.tx_fill_byte = 0xEE;
    ASSERT_OK(bus.init(bus_config));
    AccessorFixture<8, 8, 0, 2> fixture{bus};
    const uint8_t response[] = {0x10, 0x20, 0x30};
    ASSERT_OK(fixture.accessor.write({response, sizeof(response)}));
    ASSERT_OK(fixture.accessor.beginAccess(0));

    const uint8_t request[] = {1, 2, 3, 4};
    ASSERT_OK(bus.simulateFrame({request, sizeof(request)}));
    EXPECT_EQ(bus.served, (std::vector<uint8_t>{0x10, 0x20, 0x30, 0xEE}));
    ASSERT_OK(fixture.accessor.endAccess(0));

    auto frame = fixture.accessor.rxFrames().peekFrame();
    ASSERT_TRUE(frame.has_value()) << error::toString(frame.error());
    EXPECT_EQ(frame->metadata.wire_bytes, 4u);
    EXPECT_EQ(frame->metadata.stored_bytes, 4u);
    EXPECT_TRUE(slave::any(frame->metadata.flags & slave::FrameFlags::Underrun));
    EXPECT_EQ(frame->first.data[0], 1u);
    ASSERT_OK(fixture.accessor.rxFrames().popFrame());
}

TEST(SpiSlaveAccessor, FrameReservationCannotCrossAccessBoundary)
{
    FakeSlaveBus bus;
    spi::SlaveAccessConfig config;
    config.tx_mode = slave::QueueMode::Frame;
    AccessorFixture<8, 8, 2, 2> fixture{bus, config};
    auto reservation = fixture.accessor.txFrames().reserveFrame(2);
    ASSERT_TRUE(reservation.has_value()) << error::toString(reservation.error());
    EXPECT_EQ(fixture.accessor.beginAccess(0).error(), error::error_t::INVALID_STATE);
    ASSERT_OK(fixture.accessor.txFrames().cancelFrame(*reservation));
    ASSERT_OK(fixture.accessor.beginAccess(0));

    auto leaked = fixture.accessor.txFrames().reserveFrame(2);
    ASSERT_TRUE(leaked.has_value()) << error::toString(leaked.error());
    EXPECT_EQ(fixture.accessor.endAccess(0).error(), error::error_t::INVALID_STATE);
    EXPECT_FALSE(fixture.accessor.inAccess());
    EXPECT_EQ(fixture.accessor.txFrames().commitFrame(*leaked, {}).error(), error::error_t::INVALID_STATE);
}

TEST(SpiSlaveAccessor, OverflowAndLevelEventsRemainObservable)
{
    FakeSlaveBus bus;
    AccessorFixture<2, 2, 0, 1> fixture{bus};
    EventObservation observation;
    ASSERT_OK(fixture.accessor.setEventCallback(&observeEvent, &observation));
    ASSERT_OK(fixture.accessor.beginAccess(0));
    const uint8_t request[] = {1, 2, 3, 4};
    ASSERT_OK(bus.simulateFrame({request, sizeof(request)}));
    ASSERT_OK(fixture.accessor.dispatchEvents());
    EXPECT_EQ(observation.calls, 1u);
    EXPECT_TRUE(slave::any(observation.last.events & slave::SlaveEvent::Overflow));
    EXPECT_TRUE(slave::any(observation.last.events & slave::SlaveEvent::Underrun));

    auto frame = fixture.accessor.rxFrames().peekFrame();
    ASSERT_TRUE(frame.has_value()) << error::toString(frame.error());
    EXPECT_EQ(frame->metadata.stored_bytes, 2u);
    EXPECT_EQ(frame->metadata.dropped_bytes, 2u);
    EXPECT_EQ(fixture.accessor.rxStatus().dropped_bytes, 2u);
    ASSERT_OK(fixture.accessor.rxFrames().popFrame());
    ASSERT_OK(fixture.accessor.acknowledgeEvents(observation.last.events));
    ASSERT_OK(fixture.accessor.endAccess(0));
}

TEST(SpiSlaveAccessor, ExplicitClearIsInactiveOnly)
{
    FakeSlaveBus bus;
    AccessorFixture<4, 4, 0, 1> fixture{bus};
    const uint8_t value = 7;
    ASSERT_OK(fixture.accessor.write({&value, 1}));
    ASSERT_OK(fixture.accessor.clearTx());
    EXPECT_EQ(fixture.accessor.writable(), 4u);
    ASSERT_OK(fixture.accessor.beginAccess(0));
    EXPECT_EQ(fixture.accessor.clearTx().error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(fixture.accessor.clearRx().error(), error::error_t::INVALID_STATE);
    ASSERT_OK(fixture.accessor.endAccess(0));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
