// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_RESOURCE_KEY_HPP_
#define M5_HAL_BUS_RESOURCE_KEY_HPP_

#include "../error.hpp"
#include "../types.hpp"

#include <M5Utility.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <type_traits>

namespace m5::hal::v2::bus {

enum class ResourceTag : uint8_t { Pins, Native, Path, Remote };

struct SlotGeneration {
    uint16_t slot       = std::numeric_limits<uint16_t>::max();
    uint16_t reserved   = 0;
    uint32_t generation = 0;

    bool valid(void) const
    {
        return slot != std::numeric_limits<uint16_t>::max() && generation != 0;
    }

    friend bool operator==(const SlotGeneration& lhs, const SlotGeneration& rhs)
    {
        return lhs.slot == rhs.slot && lhs.generation == rhs.generation;
    }
};

using RegistryEntryToken = SlotGeneration;
using PathToken          = SlotGeneration;
using NativeToken        = SlotGeneration;

/*! @brief Fixed-size, exact physical-resource identity within one domain. */
struct ResourceKey {
    static constexpr size_t kMaxValues = 6;

    types::bus_kind_t kind = types::bus_kind_t::Unknown;
    ResourceTag tag        = ResourceTag::Pins;
    uint8_t value_count    = 0;
    uint8_t reserved       = 0;

    union Payload {
        struct Pins {
            types::gpio_number_t values[kMaxValues];
        } pins;
        struct Token {
            SlotGeneration value;
            uint32_t type;
        } token;
        struct Remote {
            uint32_t session_generation_low;
            uint32_t session_generation_high;
            types::gpio_number_t target[kMaxValues];
            uint16_t target_kind;
            uint16_t reserved;
        } remote;
        uint8_t bytes[28];

        constexpr Payload() : bytes{}
        {
        }
    } payload;

    bool isValid(void) const
    {
        if (kind == types::bus_kind_t::Unknown || reserved != 0) {
            return false;
        }
        switch (tag) {
            case ResourceTag::Pins:
                return value_count != 0 && value_count <= kMaxValues;
            case ResourceTag::Native:
            case ResourceTag::Path:
                return value_count == 0 && payload.token.value.valid() && payload.token.value.reserved == 0;
            case ResourceTag::Remote:
                return value_count != 0 && value_count <= kMaxValues && remoteSessionGeneration() != 0 &&
                       payload.remote.reserved == 0;
        }
        return false;
    }

    static result_t<ResourceKey> makePins(types::bus_kind_t kind, const types::gpio_number_t* values, size_t count)
    {
        if (kind == types::bus_kind_t::Unknown || values == nullptr || count == 0 || count > kMaxValues) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        ResourceKey out;
        out.kind        = kind;
        out.tag         = ResourceTag::Pins;
        out.value_count = static_cast<uint8_t>(count);
        for (size_t i = 0; i < count; ++i) {
            out.payload.pins.values[i] = values[i];
        }
        return out;
    }

    static ResourceKey fromPins(types::bus_kind_t kind, std::initializer_list<types::gpio_number_t> values)
    {
        auto made = makePins(kind, values.begin(), values.size());
        return made.has_value() ? made.value() : ResourceKey{};
    }

    static result_t<ResourceKey> makeToken(types::bus_kind_t kind, ResourceTag tag, SlotGeneration value, uint32_t type)
    {
        if (kind == types::bus_kind_t::Unknown || (tag != ResourceTag::Native && tag != ResourceTag::Path) ||
            !value.valid() || value.reserved != 0) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        ResourceKey out;
        out.kind                = kind;
        out.tag                 = tag;
        out.payload.token.value = value;
        out.payload.token.type  = type;
        return out;
    }

    static result_t<ResourceKey> makeRemote(types::bus_kind_t kind, uint64_t session_generation,
                                            const types::gpio_number_t* target, size_t count, uint16_t target_kind)
    {
        if (kind == types::bus_kind_t::Unknown || session_generation == 0 || target == nullptr || count == 0 ||
            count > kMaxValues) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        ResourceKey out;
        out.kind                                   = kind;
        out.tag                                    = ResourceTag::Remote;
        out.value_count                            = static_cast<uint8_t>(count);
        out.payload.remote.session_generation_low  = static_cast<uint32_t>(session_generation);
        out.payload.remote.session_generation_high = static_cast<uint32_t>(session_generation >> 32u);
        out.payload.remote.target_kind             = target_kind;
        for (size_t i = 0; i < count; ++i) {
            out.payload.remote.target[i] = target[i];
        }
        return out;
    }

    friend bool operator==(const ResourceKey& lhs, const ResourceKey& rhs)
    {
        if (!lhs.isValid() || !rhs.isValid()) {
            return false;
        }
        if (lhs.kind != rhs.kind || lhs.tag != rhs.tag || lhs.value_count != rhs.value_count) {
            return false;
        }
        switch (lhs.tag) {
            case ResourceTag::Pins:
                return std::memcmp(lhs.payload.pins.values, rhs.payload.pins.values,
                                   lhs.value_count * sizeof(types::gpio_number_t)) == 0;
            case ResourceTag::Native:
            case ResourceTag::Path:
                return lhs.payload.token.value == rhs.payload.token.value &&
                       lhs.payload.token.type == rhs.payload.token.type;
            case ResourceTag::Remote:
                return lhs.remoteSessionGeneration() == rhs.remoteSessionGeneration() &&
                       lhs.payload.remote.target_kind == rhs.payload.remote.target_kind &&
                       std::memcmp(lhs.payload.remote.target, rhs.payload.remote.target,
                                   lhs.value_count * sizeof(types::gpio_number_t)) == 0;
        }
        return false;
    }

private:
    uint64_t remoteSessionGeneration(void) const
    {
        return static_cast<uint64_t>(payload.remote.session_generation_low) |
               (static_cast<uint64_t>(payload.remote.session_generation_high) << 32u);
    }
};

static_assert(std::is_trivially_copyable<ResourceKey>::value, "ResourceKey must remain a fixed-size value");
static_assert(sizeof(ResourceKey) == 32u, "ResourceKey production ABI must remain 32 bytes on every target");
static_assert(alignof(ResourceKey) <= alignof(uint32_t), "ResourceKey must not require over-aligned allocation");

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_RESOURCE_KEY_HPP_
