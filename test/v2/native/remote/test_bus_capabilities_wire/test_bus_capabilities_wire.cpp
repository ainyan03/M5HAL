// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <m5_hal/hal/v2/remote/bus_capabilities_wire.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace {

namespace bus    = m5::hal::v2::bus;
namespace data   = m5::hal::v2::data;
namespace error  = m5::hal::v2::error;
namespace remote = m5::hal::v2::remote;

using remote::detail::decodeBusCapabilitiesWire;
using remote::detail::encodeBusCapabilitiesWire;

bus::BusCapabilities makeAllKnownCapabilities(uint32_t generation)
{
    bus::detail::BusCapabilitiesBuilder builder;
    builder.setGeneration(generation);
    for (const auto feature : remote::detail::kKnownBusFeatures) {
        builder.enable(feature);
    }
    uint32_t value = 100;
    for (const auto limit : remote::detail::kKnownBusLimits) {
        builder.setLimit(limit, value);
        value += 100;
    }
    return builder.build();
}

TEST(BusCapabilitiesWire, EmptySnapshotRoundTripsAndEmptyInputIsRejected)
{
    const bus::BusCapabilities empty;
    std::array<uint8_t, remote::detail::kBusCapabilitiesWireMaxKnownRecordSize> storage{};
    auto encoded = encodeBusCapabilitiesWire(empty, {storage.data(), storage.size()});
    ASSERT_TRUE(encoded.has_value()) << "err=" << error::toString(encoded.error());
    EXPECT_EQ(encoded.value(), remote::detail::kBusCapabilitiesWireHeaderSize);

    auto decoded = decodeBusCapabilitiesWire({storage.data(), encoded.value()});
    ASSERT_TRUE(decoded.has_value()) << "err=" << error::toString(decoded.error());
    EXPECT_EQ(decoded->generation(), 0u);
    for (const auto feature : remote::detail::kKnownBusFeatures) {
        EXPECT_FALSE(decoded->supports(feature));
    }
    for (const auto limit : remote::detail::kKnownBusLimits) {
        auto value = decoded->limit(limit);
        ASSERT_FALSE(value.has_value());
        EXPECT_EQ(value.error(), error::error_t::UNSUPPORTED);
    }

    auto absent = decodeBusCapabilitiesWire({});
    ASSERT_FALSE(absent.has_value());
    EXPECT_EQ(absent.error(), error::error_t::PROTOCOL_ERROR);
}

TEST(BusCapabilitiesWire, AllKnownValuesRoundTripAtMaximumKnownSize)
{
    const auto input = makeAllKnownCapabilities(0x78563412u);
    std::array<uint8_t, remote::detail::kBusCapabilitiesWireMaxKnownRecordSize> storage{};
    auto encoded = encodeBusCapabilitiesWire(input, {storage.data(), storage.size()});
    ASSERT_TRUE(encoded.has_value()) << "err=" << error::toString(encoded.error());
    ASSERT_EQ(encoded.value(), remote::detail::kBusCapabilitiesWireMaxKnownRecordSize);

    auto decoded = decodeBusCapabilitiesWire({storage.data(), encoded.value()});
    ASSERT_TRUE(decoded.has_value()) << "err=" << error::toString(decoded.error());
    EXPECT_EQ(decoded->generation(), 0x78563412u);
    for (const auto feature : remote::detail::kKnownBusFeatures) {
        EXPECT_TRUE(decoded->supports(feature));
    }
    uint32_t expected = 100;
    for (const auto limit : remote::detail::kKnownBusLimits) {
        auto value = decoded->limit(limit);
        ASSERT_TRUE(value.has_value()) << "err=" << error::toString(value.error());
        EXPECT_EQ(value.value(), expected);
        expected += 100;
    }
}

TEST(BusCapabilitiesWire, UnknownFeatureAndLimitIdsAreSkipped)
{
    const uint8_t record[] = {
        1,
        0x44,
        0x33,
        0x22,
        0x11,
        3,
        static_cast<uint8_t>(bus::BusFeature::Transmit),
        0xFE,
        static_cast<uint8_t>(bus::BusFeature::Receive),
        2,
        0xFD,
        1,
        2,
        3,
        4,
        static_cast<uint8_t>(bus::BusLimit::MaxAtomicRxBytes),
        0x34,
        0x12,
        0,
        0,
    };

    auto decoded = decodeBusCapabilitiesWire({record, sizeof(record)});
    ASSERT_TRUE(decoded.has_value()) << "err=" << error::toString(decoded.error());
    EXPECT_EQ(decoded->generation(), 0x11223344u);
    EXPECT_TRUE(decoded->supports(bus::BusFeature::Transmit));
    EXPECT_TRUE(decoded->supports(bus::BusFeature::Receive));
    EXPECT_FALSE(decoded->supports(bus::BusFeature::FullDuplex));
    auto rx = decoded->limit(bus::BusLimit::MaxAtomicRxBytes);
    ASSERT_TRUE(rx.has_value()) << "err=" << error::toString(rx.error());
    EXPECT_EQ(rx.value(), 0x1234u);
    auto tx = decoded->limit(bus::BusLimit::MaxAtomicTxBytes);
    ASSERT_FALSE(tx.has_value());
    EXPECT_EQ(tx.error(), error::error_t::UNSUPPORTED);
}

TEST(BusCapabilitiesWire, DuplicateKnownIdsPermitSameMeaningAndRejectConflict)
{
    const uint8_t same[] = {
        1,
        7,
        0,
        0,
        0,
        2,
        static_cast<uint8_t>(bus::BusFeature::Transmit),
        static_cast<uint8_t>(bus::BusFeature::Transmit),
        2,
        static_cast<uint8_t>(bus::BusLimit::MaxFrequencyHz),
        0x80,
        0x84,
        0x1E,
        0,
        static_cast<uint8_t>(bus::BusLimit::MaxFrequencyHz),
        0x80,
        0x84,
        0x1E,
        0,
    };
    auto accepted = decodeBusCapabilitiesWire({same, sizeof(same)});
    ASSERT_TRUE(accepted.has_value()) << "err=" << error::toString(accepted.error());
    EXPECT_TRUE(accepted->supports(bus::BusFeature::Transmit));
    auto frequency = accepted->limit(bus::BusLimit::MaxFrequencyHz);
    ASSERT_TRUE(frequency.has_value()) << "err=" << error::toString(frequency.error());
    EXPECT_EQ(frequency.value(), 2000000u);

    std::array<uint8_t, sizeof(same)> conflict{};
    for (size_t i = 0; i < conflict.size(); ++i) {
        conflict[i] = same[i];
    }
    conflict.back() = 1;
    auto rejected   = decodeBusCapabilitiesWire({conflict.data(), conflict.size()});
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error(), error::error_t::PROTOCOL_ERROR);
}

TEST(BusCapabilitiesWire, EveryTruncatedPrefixAndTrailingGarbageIsRejected)
{
    const auto input = makeAllKnownCapabilities(9);
    std::array<uint8_t, remote::detail::kBusCapabilitiesWireMaxKnownRecordSize> storage{};
    auto encoded = encodeBusCapabilitiesWire(input, {storage.data(), storage.size()});
    ASSERT_TRUE(encoded.has_value()) << "err=" << error::toString(encoded.error());
    for (size_t size = 0; size < encoded.value(); ++size) {
        auto decoded = decodeBusCapabilitiesWire({storage.data(), size});
        ASSERT_FALSE(decoded.has_value()) << "truncated size=" << size;
        EXPECT_EQ(decoded.error(), error::error_t::PROTOCOL_ERROR) << "truncated size=" << size;
    }

    std::array<uint8_t, remote::detail::kBusCapabilitiesWireMaxKnownRecordSize + 1> with_trailing{};
    for (size_t i = 0; i < encoded.value(); ++i) {
        with_trailing[i] = storage[i];
    }
    with_trailing[encoded.value()] = 0xA5;
    auto trailing                  = decodeBusCapabilitiesWire({with_trailing.data(), encoded.value() + 1});
    ASSERT_FALSE(trailing.has_value());
    EXPECT_EQ(trailing.error(), error::error_t::PROTOCOL_ERROR);
}

TEST(BusCapabilitiesWire, CountOverflowAndUnknownSchemaAreRejectedPrecisely)
{
    const uint8_t feature_overflow[] = {1, 0, 0, 0, 0, 0xFF, 0};
    auto features                    = decodeBusCapabilitiesWire({feature_overflow, sizeof(feature_overflow)});
    ASSERT_FALSE(features.has_value());
    EXPECT_EQ(features.error(), error::error_t::PROTOCOL_ERROR);

    const uint8_t limit_overflow[] = {1, 0, 0, 0, 0, 0, 0xFF};
    auto limits                    = decodeBusCapabilitiesWire({limit_overflow, sizeof(limit_overflow)});
    ASSERT_FALSE(limits.has_value());
    EXPECT_EQ(limits.error(), error::error_t::PROTOCOL_ERROR);

    uint8_t schema_zero[] = {0, 0, 0, 0, 0, 0, 0};
    auto zero             = decodeBusCapabilitiesWire({schema_zero, sizeof(schema_zero)});
    ASSERT_FALSE(zero.has_value());
    EXPECT_EQ(zero.error(), error::error_t::UNSUPPORTED);

    schema_zero[0] = 2;
    auto future    = decodeBusCapabilitiesWire({schema_zero, sizeof(schema_zero)});
    ASSERT_FALSE(future.has_value());
    EXPECT_EQ(future.error(), error::error_t::UNSUPPORTED);

    schema_zero[0] = 0xFF;
    auto unknown   = decodeBusCapabilitiesWire({schema_zero, sizeof(schema_zero)});
    ASSERT_FALSE(unknown.has_value());
    EXPECT_EQ(unknown.error(), error::error_t::UNSUPPORTED);
}

TEST(BusCapabilitiesWire, EncoderReportsCapacityAndNullStorageErrorsWithoutWriting)
{
    const auto input = makeAllKnownCapabilities(3);
    std::array<uint8_t, remote::detail::kBusCapabilitiesWireMaxKnownRecordSize> storage{};
    storage.fill(0xCC);
    auto short_buffer = encodeBusCapabilitiesWire(input, {storage.data(), storage.size() - 1});
    ASSERT_FALSE(short_buffer.has_value());
    EXPECT_EQ(short_buffer.error(), error::error_t::BUFFER_OVERFLOW);
    for (const auto byte : storage) {
        EXPECT_EQ(byte, 0xCC);
    }

    auto null_storage = encodeBusCapabilitiesWire(input, {nullptr, storage.size()});
    ASSERT_FALSE(null_storage.has_value());
    EXPECT_EQ(null_storage.error(), error::error_t::INVALID_ARGUMENT);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
