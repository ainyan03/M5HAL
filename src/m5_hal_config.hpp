// SPDX-License-Identifier: MIT
//
// Compile-time configuration for M5HAL.
//
// Macro families across the tree — a name tells you which it is:
//
//   1. Version/ABI inline-namespace switches (M5HAL_V0_INLINE / M5HAL_V1_INLINE).
//      A grandfathered category of their own, NOT covered by the conventions
//      below: they pick which `m5::hal::vN` is the `inline namespace` (so
//      `m5::hal::Foo` resolves to `vN::Foo`). Only one may be inline at a time
//      (both inline makes `m5::hal::Foo` ambiguous = a build error). The switch
//      affects ABI, so flip it all at once at a major release boundary.
//      e.g. `-DM5HAL_V0_INLINE=0 -DM5HAL_V1_INLINE=1`.
//
//   2. Supported behavior knobs, prefixed `M5HAL_CONFIG_<area>_<knob>`. The rule:
//        - The M5HAL_CONFIG_ prefix marks a user-overridable INPUT; internal
//          derived macros (M5HAL_FRAMEWORK_HAS_*, M5HAL_VARIANT_*, ...) never
//          use it, so a name tells input from library-computed output.
//        - Value-based, never definedness-based: every knob is `#ifndef`+default
//          and read by VALUE (0/1 for flags), so `-D...=0` always disables.
//          (A `#if defined(X)` knob silently ignores `=0` — that footgun is out.)
//        - Positive nouns, no polarity verbs (no USE_/DISABLE_/NO_ in names);
//          the documented default carries the polarity.
//
//   3. Diagnostic toggles, prefixed `M5HAL_DEBUG_<area>_<knob>`. Same value-based
//      mechanism as (2), but these are UNSUPPORTED probes for debugging the
//      library (default off, not in the public catalog). The distinct prefix
//      keeps them out of the supported M5HAL_CONFIG_ surface; a debug toggle may
//      name the diagnostic action directly (e.g. ..._NO_WAIT).
//
//   Defaults for (2)/(3) are co-located with the subsystem that reads them (so
//   code and default never drift); the cross-cutting ones live in this file. The
//   supported-knob catalog is in spec/design/configuration.md.

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

// Cross-cutting behavior knobs (see the convention note above). Subsystem-local
// knobs live next to their code: M5HAL_CONFIG_IDF_I2C_LEGACY (espidf variant),
// M5HAL_CONFIG_POSIX_UART (frameworks/_checker.hpp), M5HAL_CONFIG_SOFTWARE_I2C_*
// (software i2c variant).
#ifndef M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE
#define M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE 256
#endif

#ifndef M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT
#define M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT 32
#endif

#endif  // M5_HAL_CONFIG_HPP
