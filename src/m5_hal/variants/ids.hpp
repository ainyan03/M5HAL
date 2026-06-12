// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_IDS_HPP
#define M5_HAL_VARIANTS_IDS_HPP

// The single registry of variant identity numbers (v1).
//
// One number space for everything the variant machinery identifies:
// platform detection (M5HAL_V1_TARGET_PLATFORM_VARIANT_ID,
// platforms/_checker.hpp) and per-kind selection markers
// (M5HAL_V1_SELECTED_VARIANT_<KIND>, _macro/offer_all.inl) both carry
// values from THIS list, so any two of them compare meaningfully.
// Plain integer object-like macros: usable in both #if and
// static_assert.
//
// Wire format is fixed at TWO BYTES (u16): today's values stay below
// 250 for readability, 250-65535 is reserved (e.g. a future
// distributed/hashed id range for out-of-tree variants).
//
// Ranges (decimal, grouped by axis; append-only within each range):
//   0         NONE (nothing detected / nothing offered)
//   1-49      framework variants
//   50-99     platform variants: host OS
//   100-249   platform variants: MCU
//   250-65535 reserved
//
// Registry rules (spec/design/variants.md §選択 variant の診断):
//   - Values released in a v0.2 publication are FROZEN forever (no
//     renumbering; v0's M5HAL_PLATFORM_NUMBER_* values are a separate
//     frozen list and are intentionally NOT carried over).
//   - New entries append at the end of their range (+1) — never
//     reordered, never reusing a retired number (retire = leave a gap
//     with a one-line comment).
//   - Consumers must reference entries BY NAME; raw numeric literals
//     are outside the contract.
//   - An entry may exist for detection only (no variant implementation
//     yet) — implementation status is an attribute, not a separate
//     number space.

#define M5HAL_V1_VARIANT_ID_NONE 0

// --- framework variants (1-49) ---
#define M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO  1
#define M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF   2
#define M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX    3
#define M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE 4
#define M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB     5

// --- platform variants: host OS (50-99); detection-only so far ---
#define M5HAL_V1_VARIANT_ID_PLATFORM_WINDOWS 50
#define M5HAL_V1_VARIANT_ID_PLATFORM_MACOS   51
#define M5HAL_V1_VARIANT_ID_PLATFORM_LINUX   52

// --- platform variants: MCU (100-249) ---
// Implemented: ESP32 (variants/platforms/espressif/esp32, the whole
// ESP32 family). The rest are detection-only entries today.
#define M5HAL_V1_VARIANT_ID_PLATFORM_AVR       100
#define M5HAL_V1_VARIANT_ID_PLATFORM_ESP8266   101
#define M5HAL_V1_VARIANT_ID_PLATFORM_ESP32     102
#define M5HAL_V1_VARIANT_ID_PLATFORM_RP2040    103
#define M5HAL_V1_VARIANT_ID_PLATFORM_SAMD21    104
#define M5HAL_V1_VARIANT_ID_PLATFORM_SAMD51    105
#define M5HAL_V1_VARIANT_ID_PLATFORM_SPRESENSE 106
#define M5HAL_V1_VARIANT_ID_PLATFORM_STM32     107

// --- X-macro mirror of the registry above (same names, registry order) ---
//
// PP constraint ([026]): #define / #if directives cannot be generated
// from a macro expansion, so the #define list above stays hand-written
// (it must work in #if) and this list mirrors it. The C++ derivations
// below keep the two in lockstep: an entry added here but not above
// fails to compile (unknown M5HAL_V1_VARIANT_ID_<NAME>), and the
// ascending-order static_assert catches duplicate or out-of-order
// values — the registry ranges are ordered and append-only, so the
// value sequence is strictly increasing by construction.
#define M5HAL_V1_VARIANT_ID_LIST_(X) \
    X(NONE)                          \
    X(FRAMEWORK_ARDUINO)             \
    X(FRAMEWORK_ESPIDF)              \
    X(FRAMEWORK_POSIX)               \
    X(FRAMEWORK_SOFTWARE)            \
    X(FRAMEWORK_STUB)                \
    X(PLATFORM_WINDOWS)              \
    X(PLATFORM_MACOS)                \
    X(PLATFORM_LINUX)                \
    X(PLATFORM_AVR)                  \
    X(PLATFORM_ESP8266)              \
    X(PLATFORM_ESP32)                \
    X(PLATFORM_RP2040)               \
    X(PLATFORM_SAMD21)               \
    X(PLATFORM_SAMD51)               \
    X(PLATFORM_SPRESENSE)            \
    X(PLATFORM_STM32)

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v1 {

/*!
  @brief The variant identity registry as a typed enum (u16 = the wire
         width; values are the M5HAL_V1_VARIANT_ID_* macros).

  For runtime / typed use; the PP macros remain the #if-capable face of
  the same registry.
 */
enum class variant_id_t : uint16_t {
#define M5HAL_V1_VARIANT_ID_ENUM_ENTRY_(NAME) NAME = M5HAL_V1_VARIANT_ID_##NAME,
    M5HAL_V1_VARIANT_ID_LIST_(M5HAL_V1_VARIANT_ID_ENUM_ENTRY_)
#undef M5HAL_V1_VARIANT_ID_ENUM_ENTRY_
};

/*! @brief Registry name of `id` ("PLATFORM_ESP32", ...), or "UNKNOWN". */
constexpr const char* variantIdName(variant_id_t id)
{
    switch (id) {
#define M5HAL_V1_VARIANT_ID_NAME_CASE_(NAME) \
    case variant_id_t::NAME:                 \
        return #NAME;
        M5HAL_V1_VARIANT_ID_LIST_(M5HAL_V1_VARIANT_ID_NAME_CASE_)
#undef M5HAL_V1_VARIANT_ID_NAME_CASE_
    }
    return "UNKNOWN";
}

/*! @brief Overload for raw registry values (e.g. the
    M5HAL_V1_SELECTED_VARIANT_<KIND> / M5HAL_V1_TARGET_PLATFORM_VARIANT_ID
    integer markers). */
constexpr const char* variantIdName(uint16_t id)
{
    return variantIdName(static_cast<variant_id_t>(id));
}

namespace detail {

constexpr uint16_t kVariantIdValues[] = {
#define M5HAL_V1_VARIANT_ID_VALUE_(NAME) M5HAL_V1_VARIANT_ID_##NAME,
    M5HAL_V1_VARIANT_ID_LIST_(M5HAL_V1_VARIANT_ID_VALUE_)
#undef M5HAL_V1_VARIANT_ID_VALUE_
};

constexpr bool variantIdsStrictlyIncrease()
{
    for (size_t i = 1; i < sizeof(kVariantIdValues) / sizeof(kVariantIdValues[0]); ++i) {
        if (kVariantIdValues[i - 1] >= kVariantIdValues[i]) {
            return false;
        }
    }
    return true;
}

static_assert(variantIdsStrictlyIncrease(),
              "variants/ids.hpp: registry values must be unique and listed in ascending registry order");

}  // namespace detail

}  // namespace m5::hal::v1

#endif  // M5_HAL_VARIANTS_IDS_HPP
