// SPDX-License-Identifier: MIT
#ifndef M5_HAL_GPIO_PORT_HPP_
#define M5_HAL_GPIO_PORT_HPP_

// Pin / IPort — value-type facade plus abstract base for a GPIO bank.
// Authoritative spec: spec/design/gpio.md.

#include "../assert.hpp"
#include "../types.hpp"

#include <stdint.h>

/*!
  @namespace m5::hal::v1::gpio
  @brief GPIO abstractions (Pin, IPort, IGPIO, GPIOGroup).
 */
namespace m5::hal::v1::gpio {

class IPort;

/*!
  @brief Value-type handle to a single GPIO pin.

  Holds a back-pointer to the owning `IPort` plus an encoded pin
  number. Constructed via `IPort::getPin`; default-constructed `Pin`
  objects are invalid. Operations on an invalid `Pin` assert via
  `M5HAL_ASSERT`.
 */
class Pin {
public:
    Pin() : _owner{nullptr}, _encoded_num{0}
    {
    }

    void write(bool v) const;
    void writeHigh() const;
    void writeLow() const;
    bool read() const;
    void setMode(::m5::hal::v1::types::gpio_mode_t mode) const;

    /*! @brief Return the local pin index inside the owning `IPort`. */
    ::m5::hal::v1::types::gpio_local_pin_t getLocalPin() const;

    /*! @brief Return the owning `IPort`, or `nullptr` if the pin is invalid. */
    IPort* getPort() const
    {
        return _owner;
    }
    bool isValid() const
    {
        return _owner != nullptr;
    }

private:
    Pin(IPort* owner, uint32_t encoded_num) : _owner{owner}, _encoded_num{encoded_num}
    {
    }

    IPort* _owner;
    uint32_t _encoded_num;

    friend class IPort;
};

/*!
  @brief Abstract base for a GPIO port (a bank of pins).

  Public operations accept a `gpio_local_pin_t`; concrete variants
  implement the `_xxxPinEncoded` hooks. Pin index validity is a
  contract: `_fromLocalPin` asserts on an out-of-range input.
 */
class IPort {
public:
    void write(::m5::hal::v1::types::gpio_local_pin_t pin_index, bool v)
    {
        _writePinEncoded(_fromLocalPin(pin_index), v);
    }
    void writeHigh(::m5::hal::v1::types::gpio_local_pin_t pin_index)
    {
        _writePinEncodedHigh(_fromLocalPin(pin_index));
    }
    void writeLow(::m5::hal::v1::types::gpio_local_pin_t pin_index)
    {
        _writePinEncodedLow(_fromLocalPin(pin_index));
    }
    bool read(::m5::hal::v1::types::gpio_local_pin_t pin_index)
    {
        return _readPinEncoded(_fromLocalPin(pin_index));
    }
    void setMode(::m5::hal::v1::types::gpio_local_pin_t pin_index, ::m5::hal::v1::types::gpio_mode_t mode)
    {
        _setPinModeEncoded(_fromLocalPin(pin_index), mode);
    }

    /*!
      @brief `Pin` factory.

      Invalid `pin_index` triggers an assert inside `_fromLocalPin`.
     */
    Pin getPin(::m5::hal::v1::types::gpio_local_pin_t pin_index)
    {
        return Pin{this, _fromLocalPin(pin_index)};
    }

protected:
    // Encoded hooks (variants MUST override).
    virtual void _writePinEncoded(uint32_t encoded_num, bool v)                                   = 0;
    virtual bool _readPinEncoded(uint32_t encoded_num)                                            = 0;
    virtual void _setPinModeEncoded(uint32_t encoded_num, ::m5::hal::v1::types::gpio_mode_t mode) = 0;

    // Fast-path overrides: if the variant overrides these, the high /
    // low writes go straight to a value-less hardware register (e.g.
    // ESP32 W1TS / W1TC).
    virtual void _writePinEncodedHigh(uint32_t encoded_num)
    {
        _writePinEncoded(encoded_num, true);
    }
    virtual void _writePinEncodedLow(uint32_t encoded_num)
    {
        _writePinEncoded(encoded_num, false);
    }

    // Encoded <-> local-pin conversion (out-of-range inputs assert
    // inside the variant — contract-based).
    virtual ::m5::hal::v1::types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const = 0;
    virtual uint32_t _fromLocalPin(::m5::hal::v1::types::gpio_local_pin_t pin_index) const = 0;

    // Protected non-virtual dtor (forbids polymorphic delete, allows
    // constexpr-eligible derivations).
    ~IPort() = default;

    friend class Pin;
};

// Pin operations live out-of-class because they need the full IPort
// definition to call the encoded hooks (granted via friendship).
inline void Pin::write(bool v) const
{
    M5HAL_ASSERT(_owner != nullptr, "Pin is invalid");
    _owner->_writePinEncoded(_encoded_num, v);
}
inline void Pin::writeHigh() const
{
    M5HAL_ASSERT(_owner != nullptr, "Pin is invalid");
    _owner->_writePinEncodedHigh(_encoded_num);
}
inline void Pin::writeLow() const
{
    M5HAL_ASSERT(_owner != nullptr, "Pin is invalid");
    _owner->_writePinEncodedLow(_encoded_num);
}
inline bool Pin::read() const
{
    M5HAL_ASSERT(_owner != nullptr, "Pin is invalid");
    return _owner->_readPinEncoded(_encoded_num);
}
inline void Pin::setMode(::m5::hal::v1::types::gpio_mode_t mode) const
{
    M5HAL_ASSERT(_owner != nullptr, "Pin is invalid");
    _owner->_setPinModeEncoded(_encoded_num, mode);
}
inline ::m5::hal::v1::types::gpio_local_pin_t Pin::getLocalPin() const
{
    M5HAL_ASSERT(_owner != nullptr, "Pin is invalid");
    return _owner->_toLocalPin(_encoded_num);
}

}  // namespace m5::hal::v1::gpio

#endif
