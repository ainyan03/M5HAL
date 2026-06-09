#ifndef M5_HAL_GPIO_GROUP_HPP_
#define M5_HAL_GPIO_GROUP_HPP_

// GPIOGroup — bundles multiple IGPIO instances and resolves a global
// `gpio_number_t` to a concrete `Pin`. Role split: IGPIO owns the local
// pin space, GPIOGroup is the global resolver.
// Authoritative spec: spec/design/gpio.md §GPIOGroup.

#include "../assert.hpp"
#include "../error.hpp"
#include "../types.hpp"
#include "gpio.hpp"
#include "port.hpp"

#include <M5Utility.hpp>

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v1::gpio {

/*!
  @brief Slot-based registry that resolves a global `gpio_number_t`
         to a `Pin`.

  Registration happens during startup only; afterwards the registry
  is treated as immutable and read-only (no locking).
 */
class GPIOGroup {
public:
    /*! @brief Upper bound on the slot key space (bits 14-8 of
        `gpio_number_t`, values 0..127). */
    static constexpr size_t kSlotCount = 128;
    /*! @brief Backing-storage capacity (maximum number of IGPIOs
        registered at once). Slot keys are sparse (0..127) but the
        storage is dense. */
    static constexpr size_t kMaxEntries = 16;

    constexpr GPIOGroup() noexcept = default;

    /*! @brief Load the MCU GPIO into slot 0 (or build an empty group
        when `mcu_gpio == nullptr`). */
    explicit constexpr GPIOGroup(const IGPIO* mcu_gpio) noexcept
    {
        if (mcu_gpio != nullptr) {
            _entries[0] = Entry{mcu_gpio, 0};
            _count      = 1;
        }
    }

    GPIOGroup(const GPIOGroup&)            = delete;
    GPIOGroup& operator=(const GPIOGroup&) = delete;
    GPIOGroup(GPIOGroup&&)                 = delete;
    GPIOGroup& operator=(GPIOGroup&&)      = delete;

    /*!
      @brief Register an `IGPIO` at `slot` (startup-only operation).

      Rejected (returns `INVALID_ARGUMENT`) when: `gpio == nullptr`,
      `slot >= kSlotCount`, pin count not in `[1, 256]`, the slot is
      already in use, or storage is full. See spec §addGPIO for the
      full rule set.
     */
    [[nodiscard]] m5::stl::expected<void, error::error_t> addGPIO(const IGPIO* gpio, types::gpio_slot_t slot)
    {
        if (gpio == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        if (slot >= kSlotCount) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        const uint16_t pin_count = gpio->getPinCount();
        if (pin_count == 0 || pin_count > 256) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        // Reject duplicate slot (single linear scan).
        for (size_t i = 0; i < _count; ++i) {
            if (_entries[i].slot == slot) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }
        }
        // Storage cap (checked after we confirm there's no duplicate).
        if (_count >= kMaxEntries) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        _entries[_count++] = Entry{gpio, slot};
        return {};
    }

    /*!
      @brief Unregister the IGPIO at `slot` (startup-only operation).

      Rejected when `slot >= kSlotCount` or the slot is not currently
      registered. The hole is filled by moving the last entry over it
      (swap-and-pop; the entry order is not preserved).
     */
    [[nodiscard]] m5::stl::expected<void, error::error_t> removeGPIO(types::gpio_slot_t slot)
    {
        if (slot >= kSlotCount) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        for (size_t i = 0; i < _count; ++i) {
            if (_entries[i].slot == slot) {
                _entries[i]        = _entries[_count - 1];
                _entries[--_count] = Entry{};
                return {};
            }
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    const IGPIO* getGPIO(types::gpio_slot_t slot) const
    {
        const Entry* e = _find(slot);
        return e != nullptr ? e->gpio : nullptr;
    }

    bool hasGPIO(types::gpio_slot_t slot) const
    {
        return _find(slot) != nullptr;
    }

    /*!
      @brief Return whether `gpio_num` is in range (slot registered
             and local pin within the IGPIO's pin count).
     */
    bool isValid(types::gpio_number_t gpio_num) const
    {
        if (gpio_num < 0) {
            return false;
        }
        const Entry* e = _find(types::extractSlot(gpio_num));
        if (e == nullptr) {
            return false;
        }
        return e->gpio->isValid(types::extractLocalPin(gpio_num));
    }

    /*!
      @brief Checked resolution. Invalid input recovers through the
             `expected` error path.
     */
    [[nodiscard]] m5::stl::expected<Pin, error::error_t> tryGetPin(types::gpio_number_t gpio_num) const
    {
        if (!isValid(gpio_num)) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        return getPin(gpio_num);
    }

    /*!
      @brief Unchecked fast-path resolution. Contract violations
             assert / are UB; use `tryGetPin` when the caller needs
             to recover.
     */
    Pin getPin(types::gpio_number_t gpio_num) const
    {
        M5HAL_ASSERT(gpio_num >= 0, "GPIOGroup::getPin: invalid sentinel (negative gpio_number_t)");
        const Entry* e = _find(types::extractSlot(gpio_num));
        M5HAL_ASSERT(e != nullptr, "GPIOGroup::getPin: slot unregistered");
        const types::gpio_local_pin_t local = types::extractLocalPin(gpio_num);
        M5HAL_ASSERT(e->gpio->isValid(local), "GPIOGroup::getPin: local pin out of range");
        return e->gpio->getPin(local);
    }

private:
    // One registered IGPIO (slot key + IGPIO pointer). The pointer
    // leads so the alignment is well-defined; the slot tag rides in
    // the trailing padding.
    struct Entry {
        const IGPIO* gpio       = nullptr;
        types::gpio_slot_t slot = 0;
    };

    // Linear lookup over the dense `[0, _count)` range. Returns
    // `nullptr` when the slot is not registered.
    const Entry* _find(types::gpio_slot_t slot) const
    {
        for (size_t i = 0; i < _count; ++i) {
            if (_entries[i].slot == slot) {
                return &_entries[i];
            }
        }
        return nullptr;
    }

    // Dense backing array (slot keys are sparse, storage is dense).
    Entry _entries[kMaxEntries] = {};
    size_t _count               = 0;
};

}  // namespace m5::hal::v1::gpio

#endif
