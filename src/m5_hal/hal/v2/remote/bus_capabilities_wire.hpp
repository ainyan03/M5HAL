// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_REMOTE_BUS_CAPABILITIES_WIRE_HPP_
#define M5_HAL_HAL_V2_REMOTE_BUS_CAPABILITIES_WIRE_HPP_

#include "../bus/capabilities.hpp"
#include "../data.hpp"

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v2::remote::detail {

constexpr uint8_t kBusCapabilitiesWireSchemaVersion = 1;

constexpr bus::BusFeature kKnownBusFeatures[] = {
    bus::BusFeature::ManagedAllocation,    bus::BusFeature::HardwareBackend, bus::BusFeature::LowPowerBackend,
    bus::BusFeature::MasterTransfer,       bus::BusFeature::Transmit,        bus::BusFeature::Receive,
    bus::BusFeature::FullDuplex,           bus::BusFeature::MosiSharedRx,    bus::BusFeature::SlaveByteTx,
    bus::BusFeature::SlaveByteRx,          bus::BusFeature::SlaveFrameTx,    bus::BusFeature::SlaveFrameRx,
    bus::BusFeature::SlaveLegacyWireFrame, bus::BusFeature::ClockStretch,    bus::BusFeature::IsrRegMap,
};
static_assert(sizeof(kKnownBusFeatures) / sizeof(kKnownBusFeatures[0]) ==
                  static_cast<size_t>(bus::BusFeature::IsrRegMap) + 1,
              "Every public BusFeature ID must be represented by the v1 wire encoder");

constexpr bus::BusLimit kKnownBusLimits[] = {
    bus::BusLimit::MaxFrequencyHz,
    bus::BusLimit::MaxAtomicTxBytes,
    bus::BusLimit::MaxAtomicRxBytes,
    bus::BusLimit::MaxSlaveTransactionBytes,
};
static_assert(sizeof(kKnownBusLimits) / sizeof(kKnownBusLimits[0]) ==
                  static_cast<size_t>(bus::BusLimit::MaxSlaveTransactionBytes) + 1,
              "Every public BusLimit ID must be represented by the v1 wire encoder");

constexpr size_t kBusCapabilitiesWireHeaderSize         = 7;
constexpr size_t kBusCapabilitiesWireMaxKnownRecordSize = kBusCapabilitiesWireHeaderSize +
                                                          sizeof(kKnownBusFeatures) / sizeof(kKnownBusFeatures[0]) +
                                                          5 * sizeof(kKnownBusLimits) / sizeof(kKnownBusLimits[0]);
static_assert(kBusCapabilitiesWireMaxKnownRecordSize == 42,
              "Bus capability wire size must be updated when known IDs change");

namespace bus_capabilities_wire {

constexpr bool isKnownFeature(uint8_t id)
{
    for (const auto feature : kKnownBusFeatures) {
        if (static_cast<uint8_t>(feature) == id) {
            return true;
        }
    }
    return false;
}

constexpr bool isKnownLimit(uint8_t id)
{
    for (const auto limit : kKnownBusLimits) {
        if (static_cast<uint8_t>(limit) == id) {
            return true;
        }
    }
    return false;
}

constexpr void putU32LE(uint8_t* dst, uint32_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xFFu);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    dst[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    dst[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

constexpr uint32_t getU32LE(const uint8_t* src)
{
    return static_cast<uint32_t>(src[0]) | (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) | (static_cast<uint32_t>(src[3]) << 24);
}

}  // namespace bus_capabilities_wire

/*! @brief Encode one allocation-free Bus capability record into caller storage. */
inline result_t<size_t> encodeBusCapabilitiesWire(const bus::BusCapabilities& capabilities, data::DataSpan output)
{
    uint8_t feature_count = 0;
    for (const auto feature : kKnownBusFeatures) {
        if (capabilities.supports(feature)) {
            ++feature_count;
        }
    }

    uint8_t limit_count = 0;
    for (const auto limit : kKnownBusLimits) {
        if (capabilities.limit(limit).has_value()) {
            ++limit_count;
        }
    }

    const size_t required = kBusCapabilitiesWireHeaderSize + feature_count + static_cast<size_t>(limit_count) * 5;
    if (output.size < required) {
        return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
    }
    if (output.data == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    size_t offset         = 0;
    output.data[offset++] = kBusCapabilitiesWireSchemaVersion;
    bus_capabilities_wire::putU32LE(output.data + offset, capabilities.generation());
    offset += 4;
    output.data[offset++] = feature_count;
    for (const auto feature : kKnownBusFeatures) {
        if (capabilities.supports(feature)) {
            output.data[offset++] = static_cast<uint8_t>(feature);
        }
    }
    output.data[offset++] = limit_count;
    for (const auto limit : kKnownBusLimits) {
        auto value = capabilities.limit(limit);
        if (!value.has_value()) {
            continue;
        }
        output.data[offset++] = static_cast<uint8_t>(limit);
        bus_capabilities_wire::putU32LE(output.data + offset, value.value());
        offset += 4;
    }
    return offset;
}

/*! @brief Decode one exact Bus capability record without allocating. */
inline result_t<bus::BusCapabilities> decodeBusCapabilitiesWire(data::ConstDataSpan record)
{
    if (record.data == nullptr || record.size < kBusCapabilitiesWireHeaderSize) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    const uint8_t schema = record.data[0];
    if (schema != kBusCapabilitiesWireSchemaVersion) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }

    bus::detail::BusCapabilitiesBuilder builder;
    builder.setGeneration(bus_capabilities_wire::getU32LE(record.data + 1));
    size_t offset               = 5;
    const uint8_t feature_count = record.data[offset++];
    if (static_cast<size_t>(feature_count) > record.size - offset) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    for (size_t i = 0; i < feature_count; ++i) {
        const uint8_t id = record.data[offset++];
        if (bus_capabilities_wire::isKnownFeature(id)) {
            builder.enable(static_cast<bus::BusFeature>(id));
        }
    }

    if (offset >= record.size) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    const uint8_t limit_count = record.data[offset++];
    if (static_cast<size_t>(limit_count) > (record.size - offset) / 5) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }

    uint32_t known_limit_mask = 0;
    uint32_t known_limit_values[sizeof(kKnownBusLimits) / sizeof(kKnownBusLimits[0])]{};
    for (size_t i = 0; i < limit_count; ++i) {
        const uint8_t id     = record.data[offset++];
        const uint32_t value = bus_capabilities_wire::getU32LE(record.data + offset);
        offset += 4;
        if (!bus_capabilities_wire::isKnownLimit(id)) {
            continue;
        }
        const uint32_t mask = uint32_t{1} << id;
        if ((known_limit_mask & mask) != 0 && known_limit_values[id] != value) {
            return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
        }
        known_limit_mask |= mask;
        known_limit_values[id] = value;
        builder.setLimit(static_cast<bus::BusLimit>(id), value);
    }
    if (offset != record.size) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    return builder.build();
}

}  // namespace m5::hal::v2::remote::detail

#endif  // M5_HAL_HAL_V2_REMOTE_BUS_CAPABILITIES_WIRE_HPP_
