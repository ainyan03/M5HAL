// SPDX-License-Identifier: MIT

#ifndef M5_HAL_TYPES_HPP
#define M5_HAL_TYPES_HPP

#include <stdint.h>
#include <stddef.h>

/*!
  @namespace m5::hal::v2::types
  @brief Shared value types: GPIO numbering, GPIO mode, bus kind.
 */
namespace m5::hal::v2::types {

/*!
  @brief Opaque global GPIO number (slot + local-pin model).

  Layout (`int16_t`):
  - bit 15:   invalid sentinel (`1` = invalid, any negative value is invalid)
  - bit 14-8: slot (0..127, identifies one `IGPIO` inside the `GPIOGroup`)
  - bit 7-0:  local pin (0..255, position inside that `IGPIO`)

  Slot 0 is reserved for the MCU GPIO. Because slot 0's high bits are zero,
  existing literals such as `gpio_number_t{21}` keep pointing at MCU local
  pin 21 unchanged.

  Callers MUST NOT compose or decompose a `gpio_number_t` with raw bit
  operations; only `GPIOGroup` interprets the layout. Construct constants
  with `makeGpioNumber(slot, local_pin)` instead. Literal construction
  (`gpio_number_t{0x0123}`) is discouraged. Negative values are invalid;
  the default sentinel is `-1`.

  See spec/design/gpio.md for the full design.
 */
typedef int16_t gpio_number_t;

/*!
  @brief Slot index inside a `GPIOGroup` (one slot = one `IGPIO`).

  The underlying type is `uint8_t`, but valid values are 0..127 because
  `gpio_number_t`'s sign bit doubles as the invalid sentinel and only the
  upper 7 bits of slot are addressable. Slot 0 is reserved for the MCU GPIO.
 */
typedef uint8_t gpio_slot_t;

/*!
  @brief Local pin index inside a single `IGPIO` (0..255).
 */
typedef uint8_t gpio_local_pin_t;

/*!
  @brief Compose a `gpio_number_t` from slot and local pin.

  Intended for board profiles and variant boot code:
  @code
  static constexpr gpio_slot_t   EXPANDER_SLOT = 1;
  static constexpr gpio_number_t PIN_EXP_LED   = makeGpioNumber(EXPANDER_SLOT, 5);
  @endcode

  Passing slot >= 128 sets the invalid bit (bit 15); callers are
  responsible for keeping `slot < 128`. Contract violations are caught
  on resolution paths such as `GPIOGroup::isValid()` /
  `GPIOGroup::tryGetPin()`.
 */
constexpr gpio_number_t makeGpioNumber(gpio_slot_t slot, gpio_local_pin_t local_pin)
{
    return static_cast<gpio_number_t>((static_cast<int16_t>(slot) << 8) | static_cast<int16_t>(local_pin));
}

/*!
  @brief Extract the slot part (bit 14-8) from a `gpio_number_t`.

  Undefined for invalid (negative) inputs. Normally only `GPIOGroup`
  uses this on its dispatch path.
 */
constexpr gpio_slot_t extractSlot(gpio_number_t gpio_num)
{
    return static_cast<gpio_slot_t>((static_cast<int16_t>(gpio_num) >> 8) & 0x7F);
}

/*!
  @brief Extract the local pin part (bit 7-0) from a `gpio_number_t`.

  Undefined for invalid (negative) inputs. Normally only `GPIOGroup`
  uses this on its dispatch path.
 */
constexpr gpio_local_pin_t extractLocalPin(gpio_number_t gpio_num)
{
    return static_cast<gpio_local_pin_t>(static_cast<int16_t>(gpio_num) & 0xFF);
}

/*!
  @brief GPIO pin mode (bitfield encoding).

  The public API takes a single enum value via `setMode`, but each value
  encodes four orthogonal axes:
  - bit 0: output     (1 = output, 0 = input)
  - bit 1: open_drain (meaningful only when output, 1 = open-drain, 0 = push-pull)
  - bit 2: pull_up    (effective for input / open-drain output)
  - bit 3: pull_down  (effective for input / open-drain output;
                       not normally used with push-pull output)

  The enumerators below are named shortcuts for common combinations.
  Callers may OR axes together with
  `static_cast<gpio_mode_t>(static_cast<uint8_t>(A) | static_cast<uint8_t>(B))`,
  but whether the combination is honored depends on the variant
  implementation (e.g. simultaneous pull-up + pull-down may be ignored
  or warned about by hardware that cannot represent it).
 */
enum class GpioMode : uint8_t {
    Input                 = 0b0000,
    Output                = 0b0001,
    OutputOpenDrain       = 0b0011,
    InputPullup           = 0b0100,
    InputPulldown         = 0b1000,
    OutputOpenDrainPullup = 0b0111,
};
using gpio_mode_t = GpioMode;

namespace gpio_mode_bits {
constexpr uint8_t output     = 0b0001;
constexpr uint8_t open_drain = 0b0010;
constexpr uint8_t pull_up    = 0b0100;
constexpr uint8_t pull_down  = 0b1000;
}  // namespace gpio_mode_bits

/*!
  @brief Bus kind tag carried by `BusConfig` / `AccessConfig`.

  Used as a runtime discriminator when downcasting a base reference to
  the concrete kind. The reverse is also true: every `BusConfig` /
  `AccessConfig` derivation sets its kind from its constructor.
 */
// SAMD51's CMSIS device header (framework-cmsis-atmel) #defines DAC as the
// peripheral's register base address, and STM32's CMSIS device headers
// (framework-arduinoststm32) do the same for ADC (a legacy alias of
// ADC1_COMMON), which textually corrupts not only the plain `DAC,`/`ADC,`
// enumerators below but every later `BusKind::DAC`/`BusKind::ADC` spelling
// in this translation unit too (preprocessing has no concept of enum-class
// scoping, so restoring the macro after this definition would just move the
// corruption to the next call site instead of fixing it). Pull in the core
// header first so the macros exist here regardless of what the consumer TU
// included before us, then #undef them permanently for the rest of the TU.
// Any SAMD51/STM32 code that genuinely needs the raw CMSIS register macros
// must grab them before pulling in M5HAL headers.
//
// The early include is scoped to non-ESP32 Arduino cores (the only place a
// CMSIS DAC macro exists): on arduino-esp32 2.x, Arduino.h ahead of <cmath>
// corrupts the bundled GCC 8.4 libstdc++ (::acos undeclared and friends) —
// the mirror image of the abs()/round() <chrono> landmine on the GCC 9
// cores (see _checker.hpp), just with the include order reversed.
#if defined(ARDUINO) && !defined(ESP_PLATFORM) && __has_include(<Arduino.h>)
#include <Arduino.h>
#endif
#undef DAC
#undef ADC
enum class BusKind : uint8_t {
    Unknown = 0,
    I2C,
    SPI,
    I2S,
    UART,

    PWM,
    GPIO,
    ADC,
    DAC,
};
using bus_kind_t = BusKind;

/*!
  @brief Whether a bus is driven by a dedicated hardware controller or a
         software (bit-bang) implementation.

  Returned by `IBus::backendKind()`. A backend swap can
  change a logical bus's backend kind at runtime, so any holder that cached
  hardware guarantees should re-query (or watch `backendGeneration()`).
 */
enum class BackendKind : uint8_t {
    Software = 0,  ///< Bit-bang / software implementation; consumes no HW controller.
    Hardware,      ///< Backed by a dedicated peripheral controller.
};
using backend_kind_t = BackendKind;

/*!
  @brief Capability bitmask requested of / offered by a backend.

  An allocation request states which capabilities a backend MUST, SHOULD, or
  MUST NOT have; a backend (a specific hardware controller, or the bit-bang
  software implementation) offers a set of capabilities. Bit meanings are
  LOCAL to a bus kind (i2c, spi, i2s each define their own), EXCEPT bits 0-1
  (`HARDWARE`, `LOW_POWER`) which are reserved across all kinds so the
  kind-agnostic controller resolver can reason about them without knowing the
  kind. The software (bit-bang) backend does not participate in controller
  capability matching (and never sets `HARDWARE`). A kind may guarantee that
  its software fallback implements a declared feature and use the matching
  kind-local bit only to exclude hardware controllers that do not.
 */
using backend_caps_t = uint16_t;

/*!
  @namespace m5::hal::v2::types::backend_caps
  @brief Cross-kind reserved capability bits. Kind-local bits start at bit 2.
 */
namespace backend_caps {
constexpr backend_caps_t HARDWARE = 1u << 0;  ///< Backed by a dedicated peripheral controller.
/*!
  @brief Controller lives in a low-power domain (e.g. LP_I2C).
  Opt-in only: never selected by automatic allocation (see
  `IAllocationKind::optInCaps`); a request must name it explicitly.
 */
constexpr backend_caps_t LOW_POWER = 1u << 1;
}  // namespace backend_caps

/*!
  @brief How strictly a specific controller index is requested.
 */
enum class ControllerMode : uint8_t {
    Any = 0,  ///< No controller preference (`controller_id` is ignored).
    Prefer,   ///< Use `controller_id` if free, otherwise any eligible controller.
    Require,  ///< Must obtain `controller_id`; commit fails otherwise.
};
using controller_mode_t = ControllerMode;

/*!
  @brief A backend-allocation request expressed as capability constraints.

  This is the internal, resolver-facing form. Callers build it through the
  kind helpers (e.g. `i2c::requireHardware()`, `i2c::preferController(1)`)
  rather than writing masks by hand. The commit-time resolver ranks backends:
  a candidate must hold every `require` bit and none of the `forbid` bits;
  among candidates it favours those matching more `prefer` bits. With
  `controller_mode != Any`, `controller_id` names a specific controller
  (preferred or required). `HARDWARE` in `forbid` forces a software backend.
 */
struct AllocationIntent {
    backend_caps_t require         = 0;   ///< Backend must hold ALL of these caps.
    backend_caps_t prefer          = 0;   ///< Rank candidates by these; may go unmet.
    backend_caps_t forbid          = 0;   ///< Backend must hold NONE of these.
    int8_t controller_id           = -1;  ///< Specific controller request (with `controller_mode`).
    ControllerMode controller_mode = ControllerMode::Any;
    uint32_t max_freq              = 0;  ///< Capability hint; 0 = best available. Not part of bus identity.

    /*!
      @brief Invariant: `require`/`forbid` disjoint, and a specific-controller
             request (Require/Prefer) carries a non-negative `controller_id`.

      A negative `controller_id` under Require/Prefer is an impossible request
      (no such controller), not "any controller" -- rejecting it here stops the
      allocator from silently relaxing it to a generic hardware request.
     */
    constexpr bool valid(void) const
    {
        if ((require & forbid) != 0) {
            return false;
        }
        if ((controller_mode == ControllerMode::Require || controller_mode == ControllerMode::Prefer) &&
            controller_id < 0) {
            return false;
        }
        return true;
    }
};

/*!
  @brief Wait forever (lock-acquisition timeout sentinel).

  Passing this as a `timeout_ms` lock-acquisition argument means "block
  until the lock is obtained" (the same convention as FreeRTOS
  `portMAX_DELAY`). It is the default of every lock-timeout parameter:
  prefer passing an explicit budget you can handle on expiry; omitting
  the argument is the sugar for call sites where handling a timeout
  would be more trouble than it is worth. `0` stays an immediate
  try-lock.
 */
constexpr uint32_t TIMEOUT_FOREVER = 0xFFFFFFFFu;

/*!
  @brief runtime::Task::start core-placement sentinels (`core` argument).

  Non-negative values pin the task to that core id. The sentinels:
  TASK_CORE_ANY leaves placement to the scheduler (no affinity);
  TASK_CORE_SAME pins to the CALLING core — the placement for a worker
  that must share the caller's per-core clock domain (the CPU cycle
  counter is per-core and stops in WFI, so time marks written by the
  caller are only comparable on the same core; see service.md);
  TASK_CORE_OPPOSITE pins to the complement of the calling core. On
  single-core targets and backends without core placement (posix host,
  stub) the sentinels degrade to "no effect".
 */
constexpr int TASK_CORE_ANY      = -1;
constexpr int TASK_CORE_OPPOSITE = -2;
constexpr int TASK_CORE_SAME     = -3;

}  // namespace m5::hal::v2::types

#endif
