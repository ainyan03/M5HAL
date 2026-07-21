// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_BUS_CAPABILITIES_HPP_
#define M5_HAL_HAL_V2_BUS_CAPABILITIES_HPP_

#include "../error.hpp"

#include <stddef.h>
#include <stdint.h>
#include <type_traits>

namespace m5::hal::v2::bus {

/*!
  @brief Boolean capabilities of one concrete Bus instance.

  Numeric values are part of the remote capability schema. Add new values at
  the end; never renumber an existing value.
 */
enum class BusFeature : uint8_t {
    // The facade type participates in managed allocation/hot-swap. This is a
    // structural capability, not the current acquire path or allocation state.
    ManagedAllocation    = 0,
    HardwareBackend      = 1,
    LowPowerBackend      = 2,
    MasterTransfer       = 3,
    Transmit             = 4,
    Receive              = 5,
    FullDuplex           = 6,
    MosiSharedRx         = 7,
    SlaveByteTx          = 8,
    SlaveByteRx          = 9,
    SlaveFrameTx         = 10,
    SlaveFrameRx         = 11,
    SlaveLegacyWireFrame = 12,
    ClockStretch         = 13,
    IsrRegMap            = 14,
};

/*!
  @brief Numeric limits of one concrete Bus instance.

  `MaxAtomicTxBytes` counts all bytes in the wire write phase, including an
  I2C prefix. A caller therefore subtracts the prefix length before sizing its
  Source payload. Numeric values are part of the remote capability schema.
 */
enum class BusLimit : uint8_t {
    MaxFrequencyHz           = 0,
    MaxAtomicTxBytes         = 1,
    MaxAtomicRxBytes         = 2,
    MaxSlaveTransactionBytes = 3,
};

namespace detail {
class BusCapabilitiesBuilder;
}

/*!
  @brief Immutable, allocation-free capability snapshot for one Bus generation.

  The object owns every value it exposes: it contains no pointer or shared
  state, remains valid after the Bus closes, and never changes when a facade
  hot-swaps its backend. Unsupported/unknown limits are errors rather than an
  ambiguous zero value.
 */
class BusCapabilities {
public:
    constexpr BusCapabilities() = default;

    constexpr bool supports(BusFeature feature) const
    {
        const auto bit = static_cast<uint8_t>(feature);
        return bit < 32 && (_features & (uint32_t{1} << bit)) != 0;
    }

    result_t<uint32_t> limit(BusLimit limit) const
    {
        const auto index = static_cast<uint8_t>(limit);
        if (index >= LIMIT_COUNT || (_present_limits & (uint32_t{1} << index)) == 0) {
            return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
        }
        return _limits[index];
    }

    constexpr uint32_t generation() const
    {
        return _generation;
    }

private:
    static constexpr uint8_t LIMIT_COUNT = 4;

    uint32_t _features       = 0;
    uint32_t _present_limits = 0;
    uint32_t _limits[LIMIT_COUNT]{};
    uint32_t _generation = 0;

    friend class detail::BusCapabilitiesBuilder;
};

namespace detail {

/*! @brief Internal construction seam shared by providers and wire decoders. */
class BusCapabilitiesBuilder {
public:
    constexpr BusCapabilitiesBuilder() = default;
    explicit constexpr BusCapabilitiesBuilder(const BusCapabilities& base) : _value{base}
    {
    }

    constexpr BusCapabilitiesBuilder& enable(BusFeature feature, bool enabled = true)
    {
        const auto bit = static_cast<uint8_t>(feature);
        if (bit < 32) {
            const auto mask  = uint32_t{1} << bit;
            _value._features = enabled ? (_value._features | mask) : (_value._features & ~mask);
        }
        return *this;
    }

    constexpr BusCapabilitiesBuilder& setLimit(BusLimit limit, uint32_t value)
    {
        const auto index = static_cast<uint8_t>(limit);
        if (index < BusCapabilities::LIMIT_COUNT) {
            _value._limits[index] = value;
            _value._present_limits |= uint32_t{1} << index;
        }
        return *this;
    }

    constexpr BusCapabilitiesBuilder& clearLimit(BusLimit limit)
    {
        const auto index = static_cast<uint8_t>(limit);
        if (index < BusCapabilities::LIMIT_COUNT) {
            _value._limits[index] = 0;
            _value._present_limits &= ~(uint32_t{1} << index);
        }
        return *this;
    }

    constexpr BusCapabilitiesBuilder& setGeneration(uint32_t generation)
    {
        _value._generation = generation;
        return *this;
    }

    constexpr BusCapabilities build() const
    {
        return _value;
    }

    static constexpr uint32_t featureMask(const BusCapabilities& value)
    {
        return value._features;
    }

    static constexpr uint32_t limitPresentMask(const BusCapabilities& value)
    {
        return value._present_limits;
    }

    static constexpr uint32_t limitValue(const BusCapabilities& value, uint8_t index)
    {
        return index < BusCapabilities::LIMIT_COUNT ? value._limits[index] : 0;
    }

    constexpr BusCapabilitiesBuilder& setRawFeatureMask(uint32_t mask)
    {
        _value._features = mask;
        return *this;
    }

    constexpr BusCapabilitiesBuilder& setRawLimit(uint8_t index, bool present, uint32_t value)
    {
        if (index < BusCapabilities::LIMIT_COUNT) {
            _value._limits[index]  = value;
            const auto mask        = uint32_t{1} << index;
            _value._present_limits = present ? (_value._present_limits | mask) : (_value._present_limits & ~mask);
        }
        return *this;
    }

private:
    BusCapabilities _value{};
};

}  // namespace detail

static_assert(sizeof(BusCapabilities) == 28, "BusCapabilities wire-independent value layout changed");
static_assert(sizeof(BusCapabilities) <= 32, "BusCapabilities must remain a small bounded value");
static_assert(std::is_trivially_copyable<BusCapabilities>::value,
              "BusCapabilities must remain an allocation-free value snapshot");

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_HAL_V2_BUS_CAPABILITIES_HPP_
