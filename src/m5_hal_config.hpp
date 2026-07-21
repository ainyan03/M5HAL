// SPDX-License-Identifier: MIT
//
// Compile-time configuration for M5HAL.
//
// Macro taxonomy across the tree — a name tells you who owns it:
//
//   1. Version/ABI inline-namespace switches (M5HAL_V0_INLINE / M5HAL_V2_INLINE).
//      A grandfathered category of their own, NOT covered by the conventions
//      below: they pick which `m5::hal::vN` is the `inline namespace` (so
//      `m5::hal::Foo` resolves to `vN::Foo`). Only one may be inline at a time
//      (both inline makes `m5::hal::Foo` ambiguous = a build error). The switch
//      affects ABI, so flip it all at once at a major release boundary.
//      e.g. `-DM5HAL_V0_INLINE=0 -DM5HAL_V2_INLINE=1`.
//
//   2. Supported behavior inputs use `M5HAL_CONFIG_<area>_<knob>`. The rule:
//        - The M5HAL_CONFIG_ prefix marks a user-overridable INPUT; internal
//          derived macros (M5HAL_FRAMEWORK_HAS_*, M5HAL_VARIANT_*, ...) never
//          use it, so a name tells input from library-computed output.
//        - Value-based, never definedness-based: every knob is `#ifndef`+default
//          and read by VALUE (0/1 for flags), so `-D...=0` always disables.
//          (A `#if defined(X)` knob silently ignores `=0` — that footgun is out.)
//        - A boolean value of 1 reads as an affirmative proposition; CONFIG
//          names do not use USE_/DISABLE_/NO_.
//
//   3. Unsupported diagnostic inputs use `M5HAL_DEBUG_<area>_<knob>` and are
//      cataloged in configuration.md. They include value-based boolean switches
//      and dependent parameters such as marker pins. `NO_` is allowed here for
//      explicit A/B fault injection.
//
//   4. Library-computed public macros are read-only outputs. Implementation-only
//      helpers use `M5HAL_DETAIL_*_`; local harness controls use `M5HAL_TEST_*`,
//      `M5HAL_EXAMPLE_*`, or `M5HAL_HIL_*` and are not library configuration.
//
//   Defaults for input families are co-located with the subsystem that reads
//   them; cross-cutting defaults live here. The catalogs and ownership rules are
//   in spec/design/configuration.md.

#ifndef M5_HAL_CONFIG_HPP
#define M5_HAL_CONFIG_HPP

#ifndef M5HAL_V0_INLINE
#define M5HAL_V0_INLINE 1
#endif

#ifndef M5HAL_V2_INLINE
#define M5HAL_V2_INLINE 0
#endif

#if M5HAL_V0_INLINE && M5HAL_V2_INLINE
#error "M5HAL_V0_INLINE and M5HAL_V2_INLINE are mutually exclusive (m5::hal lookup becomes ambiguous)"
#endif

#if M5HAL_V0_INLINE
#define M5HAL_INLINE_V0 inline
#else
#define M5HAL_INLINE_V0
#endif

#if M5HAL_V2_INLINE
#define M5HAL_INLINE_V2 inline
#else
#define M5HAL_INLINE_V2
#endif

// Cross-cutting behavior knobs (see the convention note above). Subsystem-local
// knobs live next to the code that consumes them; the public catalog lists their
// defaults and definition locations.
#ifndef M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE_BYTES
#define M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE_BYTES 256
#endif

#ifndef M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT
#define M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT 32
#endif

// Per-kind variant selection. NONE means "no override": retain the ordered
// first-hit default. The selected provider remains available through the
// read-only M5HAL_V2_SELECTED_VARIANT_<KIND> output macros.
#ifndef M5HAL_CONFIG_VARIANT_RUNTIME
#define M5HAL_CONFIG_VARIANT_RUNTIME M5HAL_V2_VARIANT_ID_NONE
#endif

#ifndef M5HAL_CONFIG_VARIANT_RUNTIME_MUTEX
#define M5HAL_CONFIG_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_NONE
#endif

#ifndef M5HAL_CONFIG_VARIANT_RUNTIME_TASK
#define M5HAL_CONFIG_VARIANT_RUNTIME_TASK M5HAL_V2_VARIANT_ID_NONE
#endif

#ifndef M5HAL_CONFIG_VARIANT_RUNTIME_EVENT
#define M5HAL_CONFIG_VARIANT_RUNTIME_EVENT M5HAL_V2_VARIANT_ID_NONE
#endif

#ifndef M5HAL_CONFIG_VARIANT_GPIO
#define M5HAL_CONFIG_VARIANT_GPIO M5HAL_V2_VARIANT_ID_NONE
#endif

#ifndef M5HAL_CONFIG_VARIANT_I2C
#define M5HAL_CONFIG_VARIANT_I2C M5HAL_V2_VARIANT_ID_NONE
#endif

#ifndef M5HAL_CONFIG_VARIANT_SPI
#define M5HAL_CONFIG_VARIANT_SPI M5HAL_V2_VARIANT_ID_NONE
#endif

#ifndef M5HAL_CONFIG_VARIANT_I2S
#define M5HAL_CONFIG_VARIANT_I2S M5HAL_V2_VARIANT_ID_NONE
#endif

#ifndef M5HAL_CONFIG_VARIANT_PDM
#define M5HAL_CONFIG_VARIANT_PDM M5HAL_V2_VARIANT_ID_NONE
#endif

#ifndef M5HAL_CONFIG_VARIANT_UART
#define M5HAL_CONFIG_VARIANT_UART M5HAL_V2_VARIANT_ID_NONE
#endif

#endif  // M5_HAL_CONFIG_HPP
