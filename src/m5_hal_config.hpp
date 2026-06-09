// SPDX-License-Identifier: MIT
//
// Inline-namespace switch macros for the M5HAL v0/v1 coexistence strategy.
//
// The switch controls whether `m5::hal::vN` is an `inline namespace`. Making a
// version inline resolves `m5::hal::Foo` to `vN::Foo` (existing code can use
// that version without any changes). Only one version may be inline at a time
// (making both inline makes `m5::hal::Foo` ambiguous, i.e. a build error).
//
//   `M5HAL_V0_INLINE` (default 1): make v0 inline (current no-touch default)
//   `M5HAL_V1_INLINE` (default 0): make v1 inline (for the future deprecation flip)
//
// Overridable via build flags, e.g. `-DM5HAL_V0_INLINE=0 -DM5HAL_V1_INLINE=1`.
// The switch affects ABI compatibility, so flip it all at once at a major
// release boundary.

#ifndef M5_HAL_CONFIG_HPP
#define M5_HAL_CONFIG_HPP

#ifndef M5HAL_V0_INLINE
#define M5HAL_V0_INLINE 1
#endif

#ifndef M5HAL_V1_INLINE
#define M5HAL_V1_INLINE 0
#endif

#if M5HAL_V0_INLINE && M5HAL_V1_INLINE
#error "M5HAL_V0_INLINE and M5HAL_V1_INLINE are mutually exclusive (m5::hal lookup becomes ambiguous)"
#endif

#if M5HAL_V0_INLINE
#define M5HAL_INLINE_V0 inline
#else
#define M5HAL_INLINE_V0
#endif

#if M5HAL_V1_INLINE
#define M5HAL_INLINE_V1 inline
#else
#define M5HAL_INLINE_V1
#endif

#ifndef M5HAL_MEMORY_TEMP_BLOCK_SIZE
#define M5HAL_MEMORY_TEMP_BLOCK_SIZE 256
#endif

#ifndef M5HAL_MEMORY_TEMP_BLOCK_COUNT
#define M5HAL_MEMORY_TEMP_BLOCK_COUNT 32
#endif

#endif  // M5_HAL_CONFIG_HPP
