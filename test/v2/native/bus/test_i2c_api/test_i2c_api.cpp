// SPDX-License-Identifier: MIT
// Native gtest for the non-nested I2C access lifecycle and per-I/O status.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include "m5_hal/variants/frameworks/arduino/hal/i2c/begin_result.hpp"

#include <cstdint>
#include <atomic>
#include <cstdlib>
#include <memory>
#include <new>
#include <type_traits>

namespace {
std::atomic<bool> g_track_allocations{false};
std::atomic<size_t> g_allocation_count{0};
}  // namespace

void* operator new(std::size_t size)
{
    if (g_track_allocations.load(std::memory_order_relaxed)) {
        g_allocation_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (void* ptr = std::malloc(size)) {
        return ptr;
    }
    throw std::bad_alloc{};
}

void operator delete(void* ptr) noexcept
{
    std::free(ptr);
}

void* operator new[](std::size_t size)
{
    return ::operator new(size);
}

void operator delete[](void* ptr) noexcept
{
    ::operator delete(ptr);
}

namespace {

TEST(ArduinoI2CBeginResult, VoidReturnHasNoFailureSignal)
{
    bool called = false;
    EXPECT_TRUE(m5::hal::v2::i2c::wire_begin_detail::invokeWireBegin([&] { called = true; }));
    EXPECT_TRUE(called);
}

TEST(ArduinoI2CBeginResult, BoolReturnPropagatesSuccessAndFailure)
{
    EXPECT_TRUE(m5::hal::v2::i2c::wire_begin_detail::invokeWireBegin([] { return true; }));
    EXPECT_FALSE(m5::hal::v2::i2c::wire_begin_detail::invokeWireBegin([] { return false; }));
}

namespace bus   = m5::hal::v2::bus;
namespace data  = m5::hal::v2::data;
namespace error = m5::hal::v2::error;
namespace i2c   = m5::hal::v2::i2c;
using m5::hal::v2::result_t;

class FakeI2cBus : public i2c::IBus {
public:
    result_t<void> lockFor(bus::IAccessor& owner)
    {
        return acquireAccessLock(owner, 0);
    }
    result_t<void> unlockFor(bus::IAccessor& owner)
    {
        return releaseAccessLock(owner);
    }

protected:
    bus::CloseOutcome closeBackend(void) override
    {
        return bus::CloseOutcome::success();
    }

    result_t<void> transferBackend(bus::OperationContext<i2c::MasterAccessConfig>&, const i2c::TransferDesc&,
                                   data::Source*, size_t, data::Sink*, size_t) override
    {
        if (fail_transfer) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        ++transfer_calls;
        return {};
    }
    result_t<bus::TransferTotals> waitTransferBackend(bus::OperationContext<i2c::MasterAccessConfig>&) override
    {
        if (fail_wait) {
            return m5::stl::make_unexpected(error::error_t::I2C_NO_ACK);
        }
        return totals;
    }
    bool transferBusyBackend(bus::OperationContext<i2c::MasterAccessConfig>&) override
    {
        return busy;
    }
    result_t<void> beginOperationBackend(bus::OperationContext<i2c::MasterAccessConfig>& context) override
    {
        ++begin_calls;
        observed_generation = context.runtime.generation;
        if (fail_begin) {
            return m5::stl::make_unexpected(error::error_t::IO_ERROR);
        }
        return {};
    }
    result_t<void> endOperationBackend(bus::OperationContext<i2c::MasterAccessConfig>&) override
    {
        ++end_calls;
        if (fail_end) {
            return m5::stl::make_unexpected(error::error_t::IO_ERROR);
        }
        return {};
    }

public:
    const bus::ResourceKey* registryResourceKey(void) const override
    {
        ++registry_key_queries;
        return nullptr;
    }

    bool fail_transfer                  = false;
    bool fail_wait                      = false;
    bool fail_begin                     = false;
    bool fail_end                       = false;
    bool busy                           = false;
    size_t transfer_calls               = 0;
    size_t begin_calls                  = 0;
    size_t end_calls                    = 0;
    uint32_t observed_generation        = 0;
    mutable size_t registry_key_queries = 0;
    bus::TransferTotals totals{1, 2};
};

class InspectableI2cAccessor : public i2c::MasterAccessor {
public:
    using i2c::MasterAccessor::MasterAccessor;

    bus::OperationContext<i2c::MasterAccessConfig>& context()
    {
        return _context;
    }
};

using I2cContext = bus::OperationContext<i2c::MasterAccessConfig>;
static_assert(!std::is_default_constructible_v<I2cContext>);
static_assert(!std::is_constructible_v<I2cContext, const i2c::MasterAccessConfig&>);
static_assert(!std::is_copy_constructible_v<I2cContext>);
static_assert(!std::is_copy_assignable_v<I2cContext>);
static_assert(!std::is_move_constructible_v<I2cContext>);
static_assert(!std::is_move_assignable_v<I2cContext>);
static_assert(sizeof(I2cContext) <= sizeof(i2c::MasterAccessConfig) + sizeof(bus::OperationRuntime) +
                                        3 * sizeof(void*) + 2 * sizeof(uint32_t));

TEST(I2CCheckedFacade, RejectsInactiveEndedWrongBusAndWrongAccessorContexts)
{
    FakeI2cBus first_bus;
    FakeI2cBus second_bus;
    InspectableI2cAccessor first{first_bus, i2c::MasterAccessConfig{}};
    InspectableI2cAccessor second{first_bus, i2c::MasterAccessConfig{}};

    auto inactive = first_bus.transfer(first.context(), i2c::TransferDesc{}, nullptr, 0, nullptr, 0);
    ASSERT_FALSE(inactive.has_value());
    EXPECT_EQ(inactive.error(), error::error_t::INVALID_STATE);

    // A Context permanently belongs to its constructing Accessor. Even with
    // a current epoch, it cannot register while another Accessor owns the lock.
    first.context().runtime.begin(0, 0, bus::OperationMode::TxRx);
    ASSERT_TRUE(first_bus.lockFor(second).has_value());
    auto wrong_accessor = first_bus.beginOperation(first.context());
    ASSERT_FALSE(wrong_accessor.has_value());
    EXPECT_EQ(wrong_accessor.error(), error::error_t::INVALID_STATE);
    ASSERT_TRUE(first_bus.unlockFor(second).has_value());

    ASSERT_TRUE(first.beginAccess().has_value());
    auto wrong_bus = second_bus.transfer(first.context(), i2c::TransferDesc{}, nullptr, 0, nullptr, 0);
    ASSERT_FALSE(wrong_bus.has_value());
    EXPECT_EQ(wrong_bus.error(), error::error_t::INVALID_STATE);
    ASSERT_TRUE(first.endAccess().has_value());

    auto ended = first_bus.transfer(first.context(), i2c::TransferDesc{}, nullptr, 0, nullptr, 0);
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::INVALID_STATE);
}

TEST(I2CCheckedFacade, RejectsOldGenerationAndStillRecoversTheSlotAndLock)
{
    FakeI2cBus bus;
    InspectableI2cAccessor accessor{bus, i2c::MasterAccessConfig{}};

    ASSERT_TRUE(accessor.beginAccess().has_value());
    const auto registered_generation = accessor.context().runtime.generation;
    ++accessor.context().runtime.generation;
    auto stale = bus.transfer(accessor.context(), i2c::TransferDesc{}, nullptr, 0, nullptr, 0);
    ASSERT_FALSE(stale.has_value());
    EXPECT_EQ(stale.error(), error::error_t::INVALID_STATE);
    auto ended = accessor.endAccess();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::INVALID_STATE);
    EXPECT_FALSE(accessor.inAccess());

    ASSERT_TRUE(accessor.beginAccess().has_value());
    EXPECT_GT(accessor.context().runtime.generation, registered_generation);
    EXPECT_TRUE(accessor.endAccess().has_value());
}

TEST(I2CCheckedFacade, BeginAndEndFailuresDoNotLeaveReusableFalseAuthority)
{
    FakeI2cBus bus;
    InspectableI2cAccessor accessor{bus, i2c::MasterAccessConfig{}};

    bus.fail_begin    = true;
    auto failed_begin = accessor.beginAccess();
    ASSERT_FALSE(failed_begin.has_value());
    EXPECT_EQ(failed_begin.error(), error::error_t::IO_ERROR);
    EXPECT_FALSE(accessor.inAccess());

    bus.fail_begin = false;
    ASSERT_TRUE(accessor.beginAccess().has_value());
    bus.fail_end    = true;
    auto failed_end = accessor.endAccess();
    ASSERT_FALSE(failed_end.has_value());
    EXPECT_EQ(failed_end.error(), error::error_t::IO_ERROR);
    EXPECT_FALSE(accessor.inAccess());

    bus.fail_end = false;
    EXPECT_TRUE(accessor.beginAccess().has_value());
    EXPECT_TRUE(accessor.endAccess().has_value());
}

TEST(I2CMasterAccessor, TransferHotPathDoesNotAllocateQueryIdentityOrCopySharedOwner)
{
    auto bus_owner = std::make_shared<FakeI2cBus>();
    i2c::MasterAccessor accessor{bus_owner, i2c::MasterAccessConfig{}};
    const long owner_count = bus_owner.use_count();
    const uint8_t byte[]   = {0x12};
    const data::ConstDataSpan tx{byte, sizeof(byte)};
    ASSERT_TRUE(accessor.beginAccess().has_value());

    // Warm any host-runtime one-time initialization before measurement.
    ASSERT_TRUE(accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{}).has_value());
    bus_owner->registry_key_queries = 0;
    g_allocation_count.store(0, std::memory_order_relaxed);
    g_track_allocations.store(true, std::memory_order_relaxed);
    bool all_ok = true;
    for (size_t i = 0; i < 32; ++i) {
        all_ok = all_ok && accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{}).has_value();
    }
    g_track_allocations.store(false, std::memory_order_relaxed);

    EXPECT_TRUE(all_ok);
    EXPECT_EQ(g_allocation_count.load(std::memory_order_relaxed), 0u);
    EXPECT_EQ(bus_owner->registry_key_queries, 0u);
    EXPECT_EQ(bus_owner.use_count(), owner_count);
    EXPECT_TRUE(accessor.endAccess().has_value());
}

TEST(I2CBusCapabilities, RepeatedSnapshotQueriesDoNotAllocate)
{
    i2c::Bus bus;
    ASSERT_TRUE(bus.swapBackend(std::unique_ptr<i2c::IBus>{new FakeI2cBus()}).has_value());
    uint32_t observed = 0;

    // Warm test/runtime infrastructure before observing global operator new.
    (void)bus.capabilities();
    g_allocation_count.store(0, std::memory_order_relaxed);
    g_track_allocations.store(true, std::memory_order_relaxed);
    for (size_t i = 0; i < 1000; ++i) {
        const auto snapshot = bus.capabilities();
        observed ^= snapshot.generation();
        observed ^= snapshot.supports(bus::BusFeature::MasterTransfer) ? 1u : 0u;
    }
    g_track_allocations.store(false, std::memory_order_relaxed);

    EXPECT_EQ(observed, 0u);  // identical values XORed an even number of times
    EXPECT_EQ(g_allocation_count.load(std::memory_order_relaxed), 0u);
}

TEST(I2CMasterAccessor, WaitFailureIsPerIoAndTheSameAccessCanContinue)
{
    FakeI2cBus bus;
    i2c::MasterAccessor accessor{bus, i2c::MasterAccessConfig{}};

    const uint8_t byte[] = {0x12};
    const data::ConstDataSpan tx{byte, sizeof(byte)};

    ASSERT_TRUE(accessor.beginAccess().has_value());
    EXPECT_EQ(bus.begin_calls, 1u);
    EXPECT_NE(bus.observed_generation, 0u);
    auto nested = accessor.beginAccess();
    ASSERT_FALSE(nested.has_value());
    EXPECT_EQ(nested.error(), error::error_t::INVALID_STATE);

    bus.fail_wait = true;
    auto first    = accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), error::error_t::I2C_NO_ACK);
    EXPECT_EQ(bus.transfer_calls, 1u);
    auto failed_status = accessor.getLastTransferStatus();
    ASSERT_TRUE(failed_status.has_value());
    EXPECT_EQ(failed_status->error, error::error_t::I2C_NO_ACK);
    EXPECT_EQ(failed_status->completion, bus::CompletionLevel::Aborted);

    bus.fail_wait = false;
    auto second   = accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->tx, 1u);
    EXPECT_EQ(second->rx, 2u);
    EXPECT_EQ(bus.transfer_calls, 2u);
    auto success_status = accessor.getLastTransferStatus();
    ASSERT_TRUE(success_status.has_value());
    EXPECT_EQ(success_status->completion, bus::CompletionLevel::Complete);
    EXPECT_GT(success_status->transfer_id, failed_status->transfer_id);

    EXPECT_TRUE(accessor.endAccess().has_value());
    EXPECT_EQ(bus.end_calls, 1u);
}

TEST(I2CMasterAccessor, PreflightRejectionDoesNotPoisonTheAccess)
{
    FakeI2cBus bus;
    i2c::MasterAccessor accessor{bus, i2c::MasterAccessConfig{}};

    const uint8_t byte[] = {0x12};
    const data::ConstDataSpan tx{byte, sizeof(byte)};

    ASSERT_TRUE(accessor.beginAccess().has_value());
    bus.fail_transfer = true;
    auto first        = accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(bus.transfer_calls, 0u);

    bus.fail_transfer = false;
    auto second       = accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(bus.transfer_calls, 1u);
    EXPECT_TRUE(accessor.endAccess().has_value());
}

TEST(I2CMasterAccessor, InactiveSugarOpensAndClosesOneOperation)
{
    FakeI2cBus bus;
    i2c::MasterAccessor accessor{bus, i2c::MasterAccessConfig{}};
    const uint8_t byte[] = {0x12};

    auto written = accessor.write(data::ConstDataSpan{byte, sizeof(byte)});
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(written.value(), 1u);
    EXPECT_EQ(bus.begin_calls, 1u);
    EXPECT_EQ(bus.end_calls, 1u);
    EXPECT_FALSE(accessor.inAccess());
}

TEST(I2CMasterAccessor, ScopedAccessFinishReportsCleanupAndSugarBorrows)
{
    FakeI2cBus bus;
    i2c::MasterAccessor accessor{bus, i2c::MasterAccessConfig{}};
    const uint8_t byte[] = {0x12};

    bus::ScopedAccess scope{accessor};
    ASSERT_TRUE(scope.ok());
    auto written = accessor.write(data::ConstDataSpan{byte, sizeof(byte)});
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(bus.begin_calls, 1u);
    EXPECT_EQ(bus.end_calls, 0u);
    EXPECT_TRUE(scope.finish(7).has_value());
    EXPECT_EQ(bus.end_calls, 1u);
    EXPECT_TRUE(scope.ok());
    auto second_finish = scope.finish();
    ASSERT_FALSE(second_finish.has_value());
    EXPECT_EQ(second_finish.error(), error::error_t::INVALID_STATE);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
