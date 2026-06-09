
#ifndef M5_HAL_TYPES_HPP
#define M5_HAL_TYPES_HPP

#include <stdint.h>
#include <stddef.h>

/*!
  @namespace m5::hal::v1::types
  @brief Shared value types: GPIO numbering, GPIO mode, bus kind.
 */
namespace m5::hal::v1::types {

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
enum class BusKind : uint8_t {
    UNKNOWN = 0,
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

}  // namespace m5::hal::v1::types

#endif
