// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_NATIVE_BINDING_HPP_
#define M5_HAL_BUS_NATIVE_BINDING_HPP_

#include "./resource_interner.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <type_traits>

namespace m5::hal::v2::bus {

/*! @brief How a Bus relates to the provider resource named by a binding. */
enum class Ownership : uint8_t { None, Borrowed, Managed };

/*! @brief Shape of the provider resource identity carried by a binding. */
enum class NativeBindingKind : uint8_t { None, Native, Path, NativeAndPath };

/*! @brief Strong discriminator for the values in a NativeIdentity. */
enum class NativeIdentityKind : uint32_t {
    None = 0,
    ObjectAddress,
    PlatformResource,
    PosixDevice,
};

/*! @brief Fixed-size exact provider-resource identity interned by a ResourceDomain.

  Three 64-bit value words cover the largest currently required identity
  (`st_dev`, `st_ino`, `st_rdev` for a POSIX device). Object addresses and
  platform resource numbers use fewer words. Construction is acquisition-time
  work; no identity lookup or allocation is performed per transfer.
 */
struct NativeIdentity {
    static constexpr size_t kMaxValues = 3;

    NativeIdentityKind kind     = NativeIdentityKind::None;
    uint8_t value_count         = 0;
    uint8_t reserved[3]         = {};
    uint64_t values[kMaxValues] = {};

    bool isValid(void) const
    {
        if (kind == NativeIdentityKind::None || value_count == 0 || value_count > kMaxValues || reserved[0] != 0 ||
            reserved[1] != 0 || reserved[2] != 0) {
            return false;
        }
        for (size_t i = value_count; i < kMaxValues; ++i) {
            if (values[i] != 0) {
                return false;
            }
        }
        return true;
    }

    static result_t<NativeIdentity> make(NativeIdentityKind kind, const uint64_t* values, size_t count)
    {
        if (kind == NativeIdentityKind::None || values == nullptr || count == 0 || count > kMaxValues) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        NativeIdentity out;
        out.kind        = kind;
        out.value_count = static_cast<uint8_t>(count);
        std::memcpy(out.values, values, count * sizeof(uint64_t));
        return out;
    }

    static result_t<NativeIdentity> make(NativeIdentityKind kind, std::initializer_list<uint64_t> values)
    {
        return make(kind, values.begin(), values.size());
    }

    friend bool operator==(const NativeIdentity& lhs, const NativeIdentity& rhs)
    {
        return lhs.kind == rhs.kind && lhs.value_count == rhs.value_count &&
               std::memcmp(lhs.reserved, rhs.reserved, sizeof(lhs.reserved)) == 0 &&
               std::memcmp(lhs.values, rhs.values, sizeof(lhs.values)) == 0;
    }
};

/*! @brief Exact acquisition binding used to validate a registry hit.

  This is deliberately a value object. Provider, ownership, identity tokens,
  portable bus-level configuration and provider creation options all
  participate in equality; a hash or a subset must never decide compatibility.
 */
struct BindingDescriptor {
    uint16_t provider             = 0;
    Ownership ownership           = Ownership::None;
    NativeBindingKind native_kind = NativeBindingKind::None;
    NativeToken native{};
    PathToken path{};
    uint32_t config_primary   = 0;
    uint32_t config_secondary = 0;
    uint32_t native_options   = 0;

    friend bool operator==(const BindingDescriptor& lhs, const BindingDescriptor& rhs)
    {
        return lhs.provider == rhs.provider && lhs.ownership == rhs.ownership && lhs.native_kind == rhs.native_kind &&
               exactToken(lhs.native, rhs.native) && exactToken(lhs.path, rhs.path) &&
               lhs.config_primary == rhs.config_primary && lhs.config_secondary == rhs.config_secondary &&
               lhs.native_options == rhs.native_options;
    }

private:
    static bool exactToken(const SlotGeneration& lhs, const SlotGeneration& rhs)
    {
        return lhs.slot == rhs.slot && lhs.reserved == rhs.reserved && lhs.generation == rhs.generation;
    }
};

static_assert(std::is_trivially_copyable<NativeIdentity>::value, "native identity must remain value-like");
static_assert(sizeof(NativeIdentity) == 32u, "native identity must remain 32 bytes on 32/64-bit targets");
static_assert(std::is_trivially_copyable<BindingDescriptor>::value, "binding descriptor must remain value-like");
static_assert(sizeof(BindingDescriptor) == 32u, "binding descriptor must remain 32 bytes on 32/64-bit targets");

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_NATIVE_BINDING_HPP_
