// SPDX-License-Identifier: MIT
// Native gtest for i2c::MasterAccessor transaction error semantics.
//
// Counterpart of the MasterAccessor sticky-error tests in test_spi_api:
// the accessor transaction logic is currently a per-kind copy, so the
// contract is pinned for each kind. Contract: spec/design/i2c.md
// §transaction 中のエラー — ANY failed segment (synchronous pre-flight
// rejection or wire failure surfacing via waitTransfer) latches and
// invalidates the rest of the transaction.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>
#include "m5_hal/variants/frameworks/arduino/hal/i2c/begin_result.hpp"

#include <cstdint>

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
    result_t<void> release(void) override
    {
        return {};
    }
    result_t<void> transfer(bus::IAccessor*, const i2c::MasterAccessConfig&, const i2c::TransferDesc&, data::Source*,
                            size_t, data::Sink*, size_t) override
    {
        if (fail_transfer) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        ++transfer_calls;
        return {};
    }
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor*, const i2c::MasterAccessConfig&) override
    {
        if (fail_wait) {
            return m5::stl::make_unexpected(error::error_t::I2C_NO_ACK);
        }
        return bus::TransferTotals{};
    }
    bool transferBusy(bus::IAccessor*) override
    {
        return busy;
    }

    bool fail_transfer    = false;
    bool fail_wait        = false;
    bool busy             = false;
    size_t transfer_calls = 0;
};

TEST(I2CMasterAccessor, AsyncWaitFailurePoisonsTransaction)
{
    // Sticky-error contract, wire side: a segment failure surfacing via
    // waitTransfer() latches into the transaction — every later transfer
    // is rejected with the same error and endTransaction() reports it.
    FakeI2cBus bus;
    i2c::MasterAccessor accessor{bus, i2c::MasterAccessConfig{}};

    const uint8_t byte[] = {0x12};
    const data::ConstDataSpan tx{byte, sizeof(byte)};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    bus.busy = true;  // segment stays pending; its failure surfaces later
    ASSERT_TRUE(accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{}).has_value());
    ASSERT_EQ(bus.transfer_calls, 1u);

    bus.fail_wait = true;  // the pending segment failed on the wire (NACK)
    auto second   = accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(second.has_value());
    EXPECT_EQ(second.error(), error::error_t::I2C_NO_ACK);
    EXPECT_EQ(bus.transfer_calls, 1u);

    // The latch, not the bus, must reject from now on: were the third
    // attempt to reach the (now healthy) bus, transfer_calls would grow.
    bus.fail_wait = false;
    auto third    = accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(third.has_value());
    EXPECT_EQ(third.error(), error::error_t::I2C_NO_ACK);
    EXPECT_EQ(bus.transfer_calls, 1u);

    auto ended = accessor.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::I2C_NO_ACK);

    // A fresh transaction starts clean.
    bus.busy = false;
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    EXPECT_TRUE(accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{}).has_value());
    EXPECT_TRUE(accessor.endTransaction().has_value());
    EXPECT_EQ(bus.transfer_calls, 2u);
}

TEST(I2CMasterAccessor, SyncPreflightRejectionAlsoPoisonsTransaction)
{
    // Sticky-error contract, pre-flight side: transaction segments form
    // one logical operation, so even a synchronous rejection that never
    // touched the wire invalidates the rest of the transaction — later
    // transfers are rejected and endTransaction() reports the error.
    FakeI2cBus bus;
    i2c::MasterAccessor accessor{bus, i2c::MasterAccessConfig{}};

    const uint8_t byte[] = {0x12};
    const data::ConstDataSpan tx{byte, sizeof(byte)};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    bus.fail_transfer = true;
    auto first        = accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(bus.transfer_calls, 0u);

    // Even though the bus would now succeed, the transaction is poisoned.
    bus.fail_transfer = false;
    auto second       = accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(second.has_value());
    EXPECT_EQ(second.error(), error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(bus.transfer_calls, 0u);  // the second segment never reached the bus

    auto ended = accessor.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::INVALID_ARGUMENT);

    // A fresh transaction starts clean.
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    EXPECT_TRUE(accessor.transfer(i2c::TransferDesc{}, tx, data::DataSpan{}).has_value());
    EXPECT_EQ(bus.transfer_calls, 1u);
    EXPECT_TRUE(accessor.endTransaction().has_value());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
