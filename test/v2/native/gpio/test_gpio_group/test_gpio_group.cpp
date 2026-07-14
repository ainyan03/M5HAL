// SPDX-License-Identifier: MIT
// GPIOGroup tests.
//
// Verifies the slot-based registration model:
//   - Slot 0 is reserved for the MCU GPIO.
//   - Slots 1..127 are free for callers (board profiles, tests, ...).
//   - `gpio_number_t` is opaque; composition and decomposition go
//     through `makeGpioNumber` / `extractSlot` / `extractLocalPin`.
//   - `getPin` asserts (or is UB in release) on contract violations
//     — this is the fast path.
//   - `tryGetPin` returns `expected<Pin, error_t>` so callers can
//     recover from invalid input.
//   - Chained groups (a Group of Groups) are out of scope for v2.
//
// The contract is defined in spec/design/gpio.md §GPIOGroup.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

namespace {

using ::m5::hal::v2::types::extractLocalPin;
using ::m5::hal::v2::types::extractSlot;
using ::m5::hal::v2::types::gpio_local_pin_t;
using ::m5::hal::v2::types::gpio_mode_t;
using ::m5::hal::v2::types::gpio_number_t;
using ::m5::hal::v2::types::gpio_slot_t;
using ::m5::hal::v2::types::makeGpioNumber;
using StubPort = ::m5::hal::v2::gpio::Port_stub;

// Minimal concrete `IGPIO` for tests: wraps a single `stub::Port`
// and takes its width from the ctor. The internal local pin space
// is fixed at `0..(width-1)` (Port._base = 0). The caller manages
// slot assignment by calling `g.addGPIO(this, slot)`. Native-test
// helper only.
struct TinyGPIO : public ::m5::hal::v2::gpio::IGPIO {
    explicit TinyGPIO(uint8_t width) : _port(width, 0), _width(width)
    {
    }

    ::m5::hal::v2::gpio::IPort* portForPin(gpio_local_pin_t local_pin) const override
    {
        (void)local_pin;
        return &_port;
    }
    ::m5::hal::v2::gpio::IPort* getPort(uint8_t port_index) const override
    {
        (void)port_index;
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return _width;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }

    mutable StubPort _port;
    uint8_t _width;
};

class ReorderedPort : public ::m5::hal::v2::gpio::IPort {
protected:
    void _writePinEncoded(uint32_t encoded_num, bool value) override
    {
        const uint32_t bit = 1u << encoded_num;
        _state             = value ? (_state | bit) : (_state & ~bit);
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        return (_state & (1u << encoded_num)) != 0;
    }
    void _setPinModeEncoded(uint32_t, gpio_mode_t) override
    {
    }
    gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(gpio_local_pin_t pin_index) const override
    {
        return pin_index;
    }
    uint32_t _readPortAll() override
    {
        return _state;
    }
    void _writePortMasked(uint32_t set_mask, uint32_t clear_mask) override
    {
        _state = (_state & ~clear_mask) | set_mask;
    }

private:
    uint32_t _state = 0;
};

// Deliberately maps local pin 0 to port ordinal 1 and local pin 1 to
// ordinal 0.  This catches code that mistakes a 32-pin bank number for
// the IGPIO-defined port ordinal.
struct ReorderedGPIO : public ::m5::hal::v2::gpio::IGPIO {
    ::m5::hal::v2::gpio::IPort* portForPin(gpio_local_pin_t local_pin) const override
    {
        return &_ports[local_pin == 0 ? 1 : 0];
    }
    ::m5::hal::v2::gpio::IPort* getPort(uint8_t port_index) const override
    {
        return &_ports[port_index];
    }
    uint16_t getPinCount() const override
    {
        return 2;
    }
    uint8_t getPortCount() const override
    {
        return 2;
    }

    mutable ReorderedPort _ports[2];
};

// Models framework GPIOs whose pin operations share one stateless IPort
// while deny/watch masks still use two logical 32-bit port ordinals.
struct SharedPortGPIO : public ::m5::hal::v2::gpio::IGPIO {
    ::m5::hal::v2::gpio::IPort* portForPin(gpio_local_pin_t) const override
    {
        return &_port;
    }
    ::m5::hal::v2::gpio::IPort* getPort(uint8_t) const override
    {
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return 40;
    }
    uint8_t getPortCount() const override
    {
        return 2;
    }
    PinLocation locatePin(gpio_local_pin_t local_pin) const override
    {
        return PinLocation{static_cast<uint8_t>(local_pin >> 5), static_cast<uint8_t>(local_pin & 31u)};
    }

    mutable ReorderedPort _port;
};

struct NullPortGPIO : public ::m5::hal::v2::gpio::IGPIO {
    ::m5::hal::v2::gpio::IPort* portForPin(gpio_local_pin_t) const override
    {
        return nullptr;
    }
    ::m5::hal::v2::gpio::IPort* getPort(uint8_t) const override
    {
        return nullptr;
    }
    uint16_t getPinCount() const override
    {
        return 1;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }
};

// ----- makeGpioNumber / extract* helpers -----

TEST(MakeGpioNumber, RoundTripSlot0)
{
    // slot 0, local 21 -> gpio_number_t = 21 (upper bits are zero, so the values match numerically).
    const gpio_number_t num = makeGpioNumber(0, 21);
    EXPECT_EQ(num, 21);
    EXPECT_EQ(extractSlot(num), 0);
    EXPECT_EQ(extractLocalPin(num), 21);
}

TEST(MakeGpioNumber, RoundTripExpanderSlot)
{
    // slot 1, local 5 -> bit 14-8 = 1, bit 7-0 = 5 -> 0x0105 (= 261).
    const gpio_number_t num = makeGpioNumber(1, 5);
    EXPECT_EQ(num, 0x0105);
    EXPECT_EQ(extractSlot(num), 1);
    EXPECT_EQ(extractLocalPin(num), 5);
}

TEST(MakeGpioNumber, MaxValidSlot127MaxLocal255)
{
    // slot 127 (= 0x7F), local 255 (= 0xFF) -> 0x7FFF (INT16_MAX, just below the sign bit).
    const gpio_number_t num = makeGpioNumber(127, 255);
    EXPECT_EQ(num, 0x7FFF);
    EXPECT_EQ(extractSlot(num), 127);
    EXPECT_EQ(extractLocalPin(num), 255);
}

// ----- GPIOGroup behaviour -----

TEST(GPIOGroup, DefaultCtorIsEmpty)
{
    ::m5::hal::v2::gpio::GPIOGroup g{};
    for (gpio_slot_t s = 0; s < 128; ++s) {
        EXPECT_FALSE(g.hasGPIO(s)) << "slot " << static_cast<int>(s) << " should be empty";
    }
    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 0)));
    EXPECT_FALSE(g.isValid(-1));
}

TEST(GPIOGroup, CtorWithMcuLoadsSlot0)
{
    TinyGPIO mcu{32};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};
    EXPECT_TRUE(g.hasGPIO(0));
    EXPECT_EQ(g.getGPIO(0), &mcu);
    // MCU local pins 0-31 are visible through `isValid`.
    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 0)));
    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 31)));
    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 32)));  // Out of the MCU local range.
}

TEST(GPIOGroup, AddGPIOAtNonZeroSlot)
{
    TinyGPIO mcu{32};
    TinyGPIO expander_a{8};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};

    auto r = g.addGPIO(&expander_a, 1);
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(g.hasGPIO(1));
    EXPECT_EQ(g.getGPIO(1), &expander_a);

    // Slot 1 local pins 0-7 are valid.
    EXPECT_TRUE(g.isValid(makeGpioNumber(1, 0)));
    EXPECT_TRUE(g.isValid(makeGpioNumber(1, 7)));
    EXPECT_FALSE(g.isValid(makeGpioNumber(1, 8)));  // Out of the expander's local range.
    EXPECT_FALSE(g.isValid(makeGpioNumber(2, 0)));  // Slot 2 is not registered.
}

TEST(GPIOGroup, AddGPIORejectsNullptr)
{
    ::m5::hal::v2::gpio::GPIOGroup g{};
    auto r = g.addGPIO(nullptr, 0);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, AddGPIORejectsSlotOutOfRange)
{
    TinyGPIO expander{8};
    ::m5::hal::v2::gpio::GPIOGroup g{};
    // slot 128 is out of the valid range (0-127).
    auto r = g.addGPIO(&expander, 128);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, AddGPIORejectsZeroPinCount)
{
    // An `IGPIO` with `pin_count == 0` is meaningless (no pin can be
    // represented), so registration is rejected with INVALID_ARGUMENT.
    struct EmptyGPIO : public ::m5::hal::v2::gpio::IGPIO {
        ::m5::hal::v2::gpio::IPort* portForPin(gpio_local_pin_t) const override
        {
            return nullptr;
        }
        ::m5::hal::v2::gpio::IPort* getPort(uint8_t) const override
        {
            return nullptr;
        }
        uint16_t getPinCount() const override
        {
            return 0;
        }
        uint8_t getPortCount() const override
        {
            return 0;
        }
    } empty;
    ::m5::hal::v2::gpio::GPIOGroup g{};
    auto r = g.addGPIO(&empty, 1);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, AddGPIORejectsTooManyPins)
{
    // pin_count > 256 (beyond the `gpio_local_pin_t = uint8_t`
    // range) cannot be addressed through `makeGpioNumber`, so the
    // group rejects it. Verified with a 257-pin IGPIO.
    struct TooLargeGPIO : public ::m5::hal::v2::gpio::IGPIO {
        ::m5::hal::v2::gpio::IPort* portForPin(gpio_local_pin_t) const override
        {
            return nullptr;
        }
        ::m5::hal::v2::gpio::IPort* getPort(uint8_t) const override
        {
            return nullptr;
        }
        uint16_t getPinCount() const override
        {
            return 257;
        }
        uint8_t getPortCount() const override
        {
            return 0;
        }
    } too_large;
    ::m5::hal::v2::gpio::GPIOGroup g{};
    auto r = g.addGPIO(&too_large, 1);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, AddGPIORejectsOccupiedSlot)
{
    TinyGPIO first{8};
    TinyGPIO second{8};
    ::m5::hal::v2::gpio::GPIOGroup g{};
    ASSERT_TRUE(g.addGPIO(&first, 5).has_value());

    // Trying to add a different IGPIO to a slot that's already taken is rejected.
    auto r = g.addGPIO(&second, 5);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(g.getGPIO(5), &first);  // The existing entry is unchanged.
}

TEST(GPIOGroup, GetPinDispatchesToCorrectSlot)
{
    TinyGPIO mcu{32};
    TinyGPIO expander_a{8};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};
    ASSERT_TRUE(g.addGPIO(&expander_a, 1).has_value());

    // slot 0 (MCU)、 local 5 → mcu._port (local 5)
    {
        auto pin = g.getPin(makeGpioNumber(0, 5));
        EXPECT_TRUE(pin.isValid());
        EXPECT_EQ(pin.getPort(), &mcu._port);
        pin.write(true);
        EXPECT_TRUE(mcu._port.read(5));
        EXPECT_FALSE(expander_a._port.read(1));  // The expander side is untouched.
    }
    // slot 1 (expander)、 local 1 → expander_a._port (local 1)
    {
        auto pin = g.getPin(makeGpioNumber(1, 1));
        EXPECT_TRUE(pin.isValid());
        EXPECT_EQ(pin.getPort(), &expander_a._port);
        pin.write(true);
        EXPECT_TRUE(expander_a._port.read(1));
        EXPECT_FALSE(mcu._port.read(1));  // The MCU side is untouched.
    }
}

TEST(GPIOGroup, RemoveGPIOClearsSlot)
{
    TinyGPIO mcu{32};
    TinyGPIO expander_a{8};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};
    ASSERT_TRUE(g.addGPIO(&expander_a, 1).has_value());
    EXPECT_TRUE(g.hasGPIO(1));

    // Unregister the expander.
    auto r = g.removeGPIO(1);
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(g.hasGPIO(1));
    EXPECT_FALSE(g.isValid(makeGpioNumber(1, 0)));  // Slot 1 is invalid after removal.
    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 0)));   // The MCU stays valid.

    // Removing the same slot twice fails.
    auto r2 = g.removeGPIO(1);
    EXPECT_FALSE(r2.has_value());
    EXPECT_EQ(r2.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, RemoveGPIORejectsSlotOutOfRange)
{
    ::m5::hal::v2::gpio::GPIOGroup g{};
    // slot 128 is out of the valid range.
    auto r = g.removeGPIO(128);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, TryGetPinReturnsExpected)
{
    TinyGPIO mcu{32};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};

    // Valid input -> returns a Pin.
    auto r1 = g.tryGetPin(makeGpioNumber(0, 5));
    ASSERT_TRUE(r1.has_value());
    EXPECT_TRUE(r1.value().isValid());

    // Invalid input (slot not registered) -> INVALID_ARGUMENT.
    auto r2 = g.tryGetPin(makeGpioNumber(1, 0));
    EXPECT_FALSE(r2.has_value());
    EXPECT_EQ(r2.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    // Invalid input (negative value = sentinel) -> INVALID_ARGUMENT.
    auto r3 = g.tryGetPin(-1);
    EXPECT_FALSE(r3.has_value());
    EXPECT_EQ(r3.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    // Invalid input (local pin out of range) -> INVALID_ARGUMENT.
    auto r4 = g.tryGetPin(makeGpioNumber(0, 100));  // The MCU only has 32 pins.
    EXPECT_FALSE(r4.has_value());
    EXPECT_EQ(r4.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, Slot0MCUCompatibility)
{
    // Slot 0 is reserved for the MCU, so an existing-caller literal
    // such as `gpio_number_t{21}` should still resolve to MCU local
    // pin 21 (backward-compatibility check).
    TinyGPIO mcu{32};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};

    // gpio_number_t{21} = makeGpioNumber(0, 21)
    EXPECT_EQ(static_cast<gpio_number_t>(21), makeGpioNumber(0, 21));
    EXPECT_TRUE(g.isValid(21));

    auto pin = g.getPin(21);
    EXPECT_TRUE(pin.isValid());
    pin.write(true);
    EXPECT_TRUE(mcu._port.read(21));
}

// ----- Dense-storage model (sparse keys, dense backing array) -----

TEST(GPIOGroup, SparseSlotNumbersDispatchCorrectly)
{
    // Slot keys may be used sparsely across 0..127, but the backing
    // storage is dense. Even with gaps, every registered slot must
    // resolve to the right IGPIO (sparse key / dense storage).
    TinyGPIO mcu{32};
    TinyGPIO a{8};
    TinyGPIO b{16};
    TinyGPIO c{4};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};  // slot 0
    ASSERT_TRUE(g.addGPIO(&a, 50).has_value());
    ASSERT_TRUE(g.addGPIO(&b, 99).has_value());
    ASSERT_TRUE(g.addGPIO(&c, 127).has_value());

    EXPECT_EQ(g.getGPIO(0), &mcu);
    EXPECT_EQ(g.getGPIO(50), &a);
    EXPECT_EQ(g.getGPIO(99), &b);
    EXPECT_EQ(g.getGPIO(127), &c);
    // Unregistered intermediate slots are not resolvable.
    EXPECT_FALSE(g.hasGPIO(1));
    EXPECT_FALSE(g.hasGPIO(49));
    EXPECT_FALSE(g.hasGPIO(98));

    // `isValid` / `getPin` dispatch correctly across the sparse slot map too.
    EXPECT_TRUE(g.isValid(makeGpioNumber(50, 7)));  // a has 8 pins.
    EXPECT_FALSE(g.isValid(makeGpioNumber(50, 8)));
    EXPECT_TRUE(g.isValid(makeGpioNumber(127, 3)));  // c has 4 pins.
    EXPECT_FALSE(g.isValid(makeGpioNumber(127, 4)));

    auto pin = g.getPin(makeGpioNumber(99, 2));
    EXPECT_TRUE(pin.isValid());
    EXPECT_EQ(pin.getPort(), &b._port);
}

TEST(GPIOGroup, AddGPIORejectsWhenFull)
{
    // Registrations succeed up to the `kMaxEntries` cap, and any
    // further `addGPIO` is rejected for being over capacity. The
    // focus here is the capacity counter, so we register the same
    // IGPIO under distinct slots (the spec allows that as long as
    // the slots differ).
    constexpr size_t kMax = ::m5::hal::v2::gpio::GPIOGroup::kMaxEntries;
    TinyGPIO gpio{8};
    ::m5::hal::v2::gpio::GPIOGroup g{};
    for (gpio_slot_t s = 0; s < kMax; ++s) {
        EXPECT_TRUE(g.addGPIO(&gpio, s).has_value()) << "slot " << static_cast<int>(s);
    }
    // The first `kMax` entries succeeded; the next one (slot = kMax,
    // in range but past the storage cap) must be rejected.
    auto r = g.addGPIO(&gpio, static_cast<gpio_slot_t>(kMax));
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, RemoveRestoresCapacity)
{
    // Removing one entry from a full group frees the slot back up
    // and a subsequent `addGPIO` succeeds again (`_count` must
    // shrink correctly via compaction).
    constexpr size_t kMax = ::m5::hal::v2::gpio::GPIOGroup::kMaxEntries;
    TinyGPIO gpio{8};
    TinyGPIO extra{8};
    ::m5::hal::v2::gpio::GPIOGroup g{};
    for (gpio_slot_t s = 0; s < kMax; ++s) {
        ASSERT_TRUE(g.addGPIO(&gpio, s).has_value());
    }
    EXPECT_FALSE(g.addGPIO(&extra, static_cast<gpio_slot_t>(kMax)).has_value());  // Full.

    ASSERT_TRUE(g.removeGPIO(7).has_value());
    EXPECT_FALSE(g.hasGPIO(7));

    // Capacity has been freed, so `addGPIO` succeeds again.
    ASSERT_TRUE(g.addGPIO(&extra, static_cast<gpio_slot_t>(kMax)).has_value());
    EXPECT_EQ(g.getGPIO(static_cast<gpio_slot_t>(kMax)), &extra);
}

TEST(GPIOGroup, RemoveMiddleKeepsOthersResolvable)
{
    // Removing a middle slot triggers compaction: the last entry
    // moves into the hole (swap-and-pop; order is not preserved).
    // Even after the physical order shifts, every remaining slot
    // must still resolve through slot-key lookup.
    TinyGPIO mcu{32};
    TinyGPIO a{8};
    TinyGPIO b{8};
    TinyGPIO c{8};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};
    ASSERT_TRUE(g.addGPIO(&a, 10).has_value());
    ASSERT_TRUE(g.addGPIO(&b, 20).has_value());
    ASSERT_TRUE(g.addGPIO(&c, 30).has_value());

    // Removing the middle entry (slot 20 = b) moves `c` (the last entry) into the hole.
    ASSERT_TRUE(g.removeGPIO(20).has_value());
    EXPECT_FALSE(g.hasGPIO(20));

    // The remaining entries still resolve correctly.
    EXPECT_EQ(g.getGPIO(0), &mcu);
    EXPECT_EQ(g.getGPIO(10), &a);
    EXPECT_EQ(g.getGPIO(30), &c);
    EXPECT_TRUE(g.isValid(makeGpioNumber(30, 0)));
    auto pin = g.getPin(makeGpioNumber(30, 1));
    EXPECT_TRUE(pin.isValid());
    EXPECT_EQ(pin.getPort(), &c._port);
}

// ----- Death tests: assert behaviour on contract violations (debug builds) -----
//
// `EXPECT_DEATH` is only meaningful in debug builds (NDEBUG undefined =
// asserts active). In release builds the asserts become no-ops and a
// contract violation becomes UB. These tests pin down the debug-build
// behaviour so the contract stays observable in CI.
#if !defined(NDEBUG)

TEST(GPIOGroupDeathTest, GetPinAssertOnNegativeSentinel)
{
    ::m5::hal::v2::gpio::GPIOGroup g{};
    EXPECT_DEATH({ (void)g.getPin(-1); }, "invalid sentinel");
}

TEST(GPIOGroupDeathTest, GetPinAssertOnUnregisteredSlot)
{
    ::m5::hal::v2::gpio::GPIOGroup g{};
    EXPECT_DEATH({ (void)g.getPin(makeGpioNumber(1, 0)); }, "slot unregistered");
}

TEST(GPIOGroupDeathTest, GetPinAssertOnLocalPinOutOfRange)
{
    TinyGPIO mcu{32};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};
    // Slot 0 (MCU) is registered, but the MCU only has 32 pins, so local 100 is out of range.
    EXPECT_DEATH({ (void)g.getPin(makeGpioNumber(0, 100)); }, "local pin out of range");
}

#endif  // !defined(NDEBUG)

// ----- DenyMask -----

TEST(GPIOGroup, DenyMaskBlocksPin)
{
    TinyGPIO mcu{32};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};

    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 6)));
    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 11)));

    ASSERT_TRUE(g.setDenyMask(0, 0, 0x00000FC0u).has_value());

    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 6)));
    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 7)));
    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 11)));
    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 5)));
    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 12)));
    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 0)));

    auto denied = g.tryGetPin(makeGpioNumber(0, 6));
    EXPECT_FALSE(denied.has_value());
    EXPECT_EQ(denied.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, DenyMaskRejectsInvalidSlot)
{
    ::m5::hal::v2::gpio::GPIOGroup g{};
    auto r = g.setDenyMask(0, 0, 0xFFFFFFFF);
    EXPECT_FALSE(r.has_value());
}

TEST(GPIOGroup, DenyMaskRejectsInvalidPort)
{
    TinyGPIO mcu{32};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};
    EXPECT_FALSE(g.setDenyMask(0, 1, 0xFFFFFFFF).has_value());
    auto r = g.setDenyMask(0, 2, 0xFFFFFFFF);
    EXPECT_FALSE(r.has_value());
}

TEST(GPIOGroup, DenyMaskUsesIGPIOPortOrdinal)
{
    ReorderedGPIO mcu;
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};

    const auto pin0_location = mcu.locatePin(0);
    EXPECT_EQ(pin0_location.port_index, 1);
    EXPECT_EQ(pin0_location.bit_index, 0);

    auto deny_pin0 = g.setDenyMask(0, 1, 1u << 0);
    ASSERT_TRUE(deny_pin0.has_value()) << "err=" << ::m5::hal::v2::error::toString(deny_pin0.error());
    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 0)));
    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 1)));

    auto deny_pin1 = g.setDenyMask(0, 0, 1u << 1);
    ASSERT_TRUE(deny_pin1.has_value()) << "err=" << ::m5::hal::v2::error::toString(deny_pin1.error());
    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 1)));
}

TEST(GPIOGroup, DenyMaskUsesLogicalPortAbovePin31)
{
    SharedPortGPIO mcu;
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};

    auto deny = g.setDenyMask(0, 1, 1u << 8);
    ASSERT_TRUE(deny.has_value()) << "err=" << ::m5::hal::v2::error::toString(deny.error());
    EXPECT_TRUE(g.isValid(makeGpioNumber(0, 8)));
    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 40)));
}

TEST(GPIOGroup, CheckedMappingRejectsNullPort)
{
    NullPortGPIO mcu;
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};

    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 0)));
    EXPECT_FALSE(g.tryGetPin(makeGpioNumber(0, 0)).has_value());
    EXPECT_FALSE(g.setDenyMask(0, 0, 1u).has_value());
}

// ----- Port-level operations -----

TEST(GPIOGroup, PortReadWriteRoundTrip)
{
    TinyGPIO mcu{32};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};

    auto pa = g.getPort(0, 0);
    ASSERT_TRUE(pa.has_value());
    EXPECT_EQ(pa.value().port, &mcu._port);
    EXPECT_EQ(pa.value().deny_mask, 0u);

    pa.value().port->writePort(0x0000000Fu, 0);
    EXPECT_EQ(pa.value().port->readPort(), 0x0000000Fu);

    pa.value().port->writePort(0, 0x00000003u);
    EXPECT_EQ(pa.value().port->readPort(), 0x0000000Cu);
}

TEST(GPIOGroup, PortReadMasksDeniedBits)
{
    TinyGPIO mcu{32};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};

    mcu._port.writePort(0xFFFFFFFFu, 0);
    ASSERT_TRUE(g.setDenyMask(0, 0, 0x00000FC0u).has_value());

    auto pa = g.getPort(0, 0);
    ASSERT_TRUE(pa.has_value());
    uint32_t val = pa.value().port->readPort() & ~pa.value().deny_mask;
    EXPECT_EQ(val & 0x00000FC0u, 0u);
    EXPECT_EQ(val & 0x0000003Fu, 0x0000003Fu);
}

TEST(GPIOGroup, GetPortRejectsInvalidSlot)
{
    ::m5::hal::v2::gpio::GPIOGroup g{};
    auto pa = g.getPort(0, 0);
    EXPECT_FALSE(pa.has_value());
}

TEST(GPIOGroup, GetPortRejectsInvalidPortIndex)
{
    TinyGPIO mcu{32};
    ::m5::hal::v2::gpio::GPIOGroup g{&mcu};
    auto pa = g.getPort(0, 5);
    EXPECT_FALSE(pa.has_value());
}

// ----- Watch API (port-mask watcher + single sink) -----
//
// Every watch()/unwatch()/setWatchSink()/notifyPinStateChanged() call
// below is [[nodiscard]]-checked (ASSERT_TRUE/EXPECT_FALSE on the
// result), matching the API contract.

using GPIOGroup     = ::m5::hal::v2::gpio::GPIOGroup;
using ServiceRunner = ::m5::hal::v2::service::ServiceRunner;
// Explicit-context pass: {elapsed, local_tick}. The group has no
// intra-call spins, so a purely virtual local_tick (0) is fine.
using GroupCtx = ::m5::hal::v2::service::ServiceContext;

// IGPIO fake reporting hasPushEvents() == true (models remote GPIO's
// push-fed pin state): GPIOGroup's poll pass must skip it entirely and
// state can only reach the sink through notifyPinStateChanged().
struct PushGPIO : public ::m5::hal::v2::gpio::IGPIO {
    explicit PushGPIO(uint8_t width) : _port(width, 0), _width(width)
    {
    }
    ::m5::hal::v2::gpio::IPort* portForPin(gpio_local_pin_t) const override
    {
        return &_port;
    }
    ::m5::hal::v2::gpio::IPort* getPort(uint8_t) const override
    {
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return _width;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }
    bool hasPushEvents() const override
    {
        return true;
    }

    mutable StubPort _port;
    uint8_t _width;
};

// StubPort variant that counts readPort() calls, to pin down the
// poll-interval gating (no port read before the next due tick).
struct CountingStubPort : public StubPort {
    using StubPort::StubPort;
    uint32_t read_calls = 0;

protected:
    uint32_t _readPortAll() override
    {
        ++read_calls;
        return StubPort::_readPortAll();
    }
};

struct CountingGPIO : public ::m5::hal::v2::gpio::IGPIO {
    explicit CountingGPIO(uint8_t width) : _port(width, 0), _width(width)
    {
    }
    ::m5::hal::v2::gpio::IPort* portForPin(gpio_local_pin_t) const override
    {
        return &_port;
    }
    ::m5::hal::v2::gpio::IPort* getPort(uint8_t) const override
    {
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return _width;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }

    mutable CountingStubPort _port;
    uint8_t _width;
};

struct WatchCapture {
    gpio_number_t pin    = -1;
    bool level           = false;
    GPIOGroup::Edge edge = GPIOGroup::Edge::Rising;
    int count            = 0;
};

void captureWatchEvent(void* raw, gpio_number_t pin, bool level, GPIOGroup::Edge edge)
{
    auto* c  = static_cast<WatchCapture*>(raw);
    c->pin   = pin;
    c->level = level;
    c->edge  = edge;
    ++c->count;
}

// 1. sink + watch -> port value change -> runOnce -> Rising/Falling
//    delivered exactly once each, with the right pin/level/edge.
TEST(GPIOGroupWatch, PollDispatchesRisingThenFallingWithCorrectArgs)
{
    TinyGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    WatchCapture cap;
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    const auto pin = makeGpioNumber(0, 3);
    ASSERT_TRUE(g.watch(pin).has_value());

    mcu._port.writePort(1u << 3, 0);
    EXPECT_TRUE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 1);
    EXPECT_EQ(cap.pin, pin);
    EXPECT_TRUE(cap.level);
    EXPECT_EQ(cap.edge, GPIOGroup::Edge::Rising);

    mcu._port.writePort(0, 1u << 3);
    EXPECT_TRUE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 2);
    EXPECT_FALSE(cap.level);
    EXPECT_EQ(cap.edge, GPIOGroup::Edge::Falling);

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

TEST(GPIOGroupWatch, PollMapsPortOrdinalBackToLocalPin)
{
    ReorderedGPIO mcu;
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    WatchCapture cap;
    auto sink = g.setWatchSink(&captureWatchEvent, &cap, 1000);
    ASSERT_TRUE(sink.has_value()) << "err=" << ::m5::hal::v2::error::toString(sink.error());
    const auto pin = makeGpioNumber(0, 0);  // local 0 belongs to port ordinal 1
    auto watched   = g.watch(pin);
    ASSERT_TRUE(watched.has_value()) << "err=" << ::m5::hal::v2::error::toString(watched.error());

    mcu._ports[1].writePort(1u << 0, 0);
    EXPECT_TRUE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 1);
    EXPECT_EQ(cap.pin, pin);
    EXPECT_TRUE(cap.level);

    auto clear = g.setWatchSink(nullptr, nullptr);
    ASSERT_TRUE(clear.has_value()) << "err=" << ::m5::hal::v2::error::toString(clear.error());
}

// 2. watch() seeds the current level so an unchanged poll reports no
//    (spurious) event.
TEST(GPIOGroupWatch, WatchSeedsLevelSoUnchangedPollReportsNoEvent)
{
    TinyGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    mcu._port.writePort(1u << 2, 0);  // pin 2 already high before watch() seeds it

    WatchCapture cap;
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    ASSERT_TRUE(g.watch(makeGpioNumber(0, 2)).has_value());

    EXPECT_FALSE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 0);

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

// 3. An unwatched pin's change produces no event (mask filter).
TEST(GPIOGroupWatch, UnwatchedPinChangeProducesNoEvent)
{
    TinyGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    WatchCapture cap;
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    ASSERT_TRUE(g.watch(makeGpioNumber(0, 1)).has_value());  // watch pin 1, not pin 5

    mcu._port.writePort(1u << 5, 0);  // pin 5 changes, unwatched
    EXPECT_FALSE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 0);

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

// 4. unwatch() stops further events for that pin.
TEST(GPIOGroupWatch, UnwatchStopsFurtherEvents)
{
    TinyGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    WatchCapture cap;
    const auto pin = makeGpioNumber(0, 4);
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    ASSERT_TRUE(g.watch(pin).has_value());

    mcu._port.writePort(1u << 4, 0);
    EXPECT_TRUE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 1);

    ASSERT_TRUE(g.unwatch(pin).has_value());
    mcu._port.writePort(0, 1u << 4);
    EXPECT_FALSE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 1);  // no new event after unwatch

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

// IGPIO fake with a pin count past port 0/1 (local pin >= 64) but no
// backing storage: only used to probe watch()'s bounds check, which
// rejects on port index before ever touching a Port (unlike TinyGPIO,
// whose single Port_stub caps out at 32 pins).
struct WideGPIO : public ::m5::hal::v2::gpio::IGPIO {
    ::m5::hal::v2::gpio::IPort* portForPin(gpio_local_pin_t pin) const override
    {
        return getPort(pin >> 5);
    }
    ::m5::hal::v2::gpio::IPort* getPort(uint8_t port) const override
    {
        return port == 0 ? &_port0 : (port == 1 ? &_port1 : &_port2);
    }
    uint16_t getPinCount() const override
    {
        return 96;
    }
    uint8_t getPortCount() const override
    {
        return 3;
    }

    mutable StubPort _port0{32, 0};
    mutable StubPort _port1{32, 32};
    mutable StubPort _port2{32, 64};
};

// 5. watch() rejects a deny-masked pin, a pin past port 0/1 (local >=
//    64) even when the IGPIO itself has that many pins, and a pin on
//    an unregistered slot.
TEST(GPIOGroupWatch, WatchRejectsDeniedPortOutOfRangeAndUnregisteredSlot)
{
    TinyGPIO mcu{32};
    GPIOGroup g{&mcu};

    ASSERT_TRUE(g.setDenyMask(0, 0, 1u << 6).has_value());
    auto denied = g.watch(makeGpioNumber(0, 6));
    EXPECT_FALSE(denied.has_value());
    EXPECT_EQ(denied.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    WideGPIO wide{};
    ASSERT_TRUE(g.addGPIO(&wide, 1).has_value());
    auto out_of_port = g.watch(makeGpioNumber(1, 64));
    EXPECT_FALSE(out_of_port.has_value());
    EXPECT_EQ(out_of_port.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    auto unregistered = g.watch(makeGpioNumber(5, 0));
    EXPECT_FALSE(unregistered.has_value());
    EXPECT_EQ(unregistered.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

// 6. notifyPinStateChanged(): immediate dispatch for a watched pin
//    with no ServiceRunner involved at all; a duplicate notification
//    for the same level does not dispatch; a different (unwatched)
//    pin on the same entry does not dispatch either.
TEST(GPIOGroupWatch, NotifyDispatchesImmediatelyWithoutRunner)
{
    PushGPIO push{8};
    GPIOGroup g{};
    ASSERT_TRUE(g.addGPIO(&push, 2).has_value());
    // Deliberately no bindServiceRunner(): notify must not require one.

    WatchCapture cap;
    const auto pin = makeGpioNumber(2, 3);
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    ASSERT_TRUE(g.watch(pin).has_value());

    ASSERT_TRUE(g.notifyPinStateChanged(pin, true).has_value());
    EXPECT_EQ(cap.count, 1);
    EXPECT_TRUE(cap.level);
    EXPECT_EQ(cap.edge, GPIOGroup::Edge::Rising);

    ASSERT_TRUE(g.notifyPinStateChanged(pin, true).has_value());  // same level: no transition
    EXPECT_EQ(cap.count, 1);

    ASSERT_TRUE(g.notifyPinStateChanged(makeGpioNumber(2, 4), true).has_value());  // unwatched pin
    EXPECT_EQ(cap.count, 1);

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

// 7. A push-fed (hasPushEvents() == true) entry is never polled (a
//    direct port mutation produces zero events); it is reachable only
//    through notifyPinStateChanged(), and the very next poll pass does
//    not re-deliver the same edge.
TEST(GPIOGroupWatch, PushFedEntrySkipsPollAndDoesNotDoubleDeliverAfterNotify)
{
    PushGPIO push{8};
    GPIOGroup g{};
    ASSERT_TRUE(g.addGPIO(&push, 3).has_value());
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    WatchCapture cap;
    const auto pin = makeGpioNumber(3, 2);
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    ASSERT_TRUE(g.watch(pin).has_value());

    push._port.writePort(1u << 2, 0);  // bypasses notify -- the poll pass must ignore it
    EXPECT_FALSE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 0);

    ASSERT_TRUE(g.notifyPinStateChanged(pin, true).has_value());
    EXPECT_EQ(cap.count, 1);

    EXPECT_FALSE(runner.runOnce(GroupCtx{2000, 0}));  // push-fed entries are never read by the poll loop
    EXPECT_EQ(cap.count, 1);

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

// 8. setWatchSink(nullptr) suppresses further notify delivery;
//    re-registering the sink resumes it (the watch mask itself is
//    untouched by sink registration).
TEST(GPIOGroupWatch, SetWatchSinkNullSuppressesNotifyDeliveryReRegisterResumes)
{
    PushGPIO push{8};
    GPIOGroup g{};
    ASSERT_TRUE(g.addGPIO(&push, 4).has_value());

    WatchCapture cap;
    const auto pin = makeGpioNumber(4, 1);
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    ASSERT_TRUE(g.watch(pin).has_value());

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
    ASSERT_TRUE(g.notifyPinStateChanged(pin, true).has_value());  // mask still set, but no sink
    EXPECT_EQ(cap.count, 0);

    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    ASSERT_TRUE(g.notifyPinStateChanged(pin, false).has_value());
    EXPECT_EQ(cap.count, 1);
    EXPECT_FALSE(cap.level);

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

// 9. clearWatchers() drops the mask (not just the sink): registering
//    just the sink again is not enough to resume delivery, watch()-ing
//    again is required.
TEST(GPIOGroupWatch, ClearWatchersDropsMaskReWatchNeededAfter)
{
    TinyGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    WatchCapture cap;
    const auto pin = makeGpioNumber(0, 6);
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    ASSERT_TRUE(g.watch(pin).has_value());

    g.clearWatchers();
    EXPECT_EQ(runner.size(), 0u);  // the sink's service registration is torn down too

    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    mcu._port.writePort(1u << 6, 0);
    EXPECT_FALSE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 0);  // mask was cleared: re-registering the sink alone isn't enough

    ASSERT_TRUE(g.watch(pin).has_value());
    mcu._port.writePort(0, 1u << 6);
    EXPECT_TRUE(runner.runOnce(GroupCtx{2000, 0}));
    EXPECT_EQ(cap.count, 1);  // watch()-ing again resumes delivery

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

// 10. Poll interval gating: a second runOnce() inside the interval
//     window does not read the port again (native tests run at 1
//     tick == 1 microsecond, so 1000us == 1000 ticks).
TEST(GPIOGroupWatch, PollIntervalGatesPortReads)
{
    CountingGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    WatchCapture cap;
    const auto pin = makeGpioNumber(0, 0);
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());
    ASSERT_TRUE(g.watch(pin).has_value());

    EXPECT_FALSE(runner.runOnce(GroupCtx{0, 0}));  // first pass: due immediately (next_tick starts at 0)
    EXPECT_EQ(mcu._port.read_calls, 1u);

    EXPECT_FALSE(runner.runOnce(GroupCtx{500, 0}));  // still inside the 1000-tick window: no read
    EXPECT_EQ(mcu._port.read_calls, 1u);

    EXPECT_FALSE(runner.runOnce(GroupCtx{1500, 0}));  // past the due tick: reads again
    EXPECT_EQ(mcu._port.read_calls, 2u);

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

// 11. setWatchSink() registers the sink's service with the bound
//     ServiceRunner on success, and unregisters it on sink teardown.
TEST(GPIOGroupWatch, SetWatchSinkRegistersAndUnregistersService)
{
    TinyGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    EXPECT_EQ(runner.size(), 0u);
    ASSERT_TRUE(g.setWatchSink([](void*, gpio_number_t, bool, GPIOGroup::Edge) {}, nullptr, 1000).has_value());
    EXPECT_EQ(runner.size(), 1u);
    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
    EXPECT_EQ(runner.size(), 0u);
}

struct SelfUnregisterCtx {
    GPIOGroup* g = nullptr;
    int count    = 0;
};

void selfUnregisterSink(void* raw, gpio_number_t, bool, GPIOGroup::Edge)
{
    auto* c = static_cast<SelfUnregisterCtx*>(raw);
    ++c->count;
    if (c->count == 1) {
        (void)c->g->setWatchSink(nullptr, nullptr);  // self-unregister from inside the sink
    }
}

// 12. A sink that unregisters itself on its first event must not
//     receive the remaining changed bits of the same poll pass
//     (per-callback enter/exit; regression guard).
TEST(GPIOGroupWatch, SelfUnregisterInSinkStopsSamePassDelivery)
{
    TinyGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    SelfUnregisterCtx ctx;
    ctx.g = &g;
    ASSERT_TRUE(g.setWatchSink(&selfUnregisterSink, &ctx, 1000).has_value());
    ASSERT_TRUE(g.watch(makeGpioNumber(0, 0)).has_value());
    ASSERT_TRUE(g.watch(makeGpioNumber(0, 1)).has_value());

    mcu._port.writePort(0b11, 0);  // both watched pins rise in the same pass
    (void)runner.runOnce(GroupCtx{2000, 0});

    EXPECT_EQ(ctx.count, 1);       // second changed bit must NOT reach the old sink
    EXPECT_EQ(runner.size(), 0u);  // self-unregister also removed the poll service
}

// 13. Re-registering the sink with a shorter interval takes effect
//     immediately: the stale next-due state (both the group's own gate
//     and the ServiceRunner table entry) must not suppress the service
//     until the OLD interval elapses (regression guard).
TEST(GPIOGroupWatch, ReRegisterWithShorterIntervalTakesEffectImmediately)
{
    CountingGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    WatchCapture cap;
    const auto pin = makeGpioNumber(0, 0);
    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000000).has_value());  // 1s interval
    ASSERT_TRUE(g.watch(pin).has_value());

    EXPECT_FALSE(runner.runOnce(GroupCtx{1000, 0}));  // first pass: publishes a far-future due (~1s)
    EXPECT_EQ(mcu._port.read_calls, 1u);

    mcu._port.writePort(1u << 0, 0);                  // edge while the long interval is pending
    EXPECT_FALSE(runner.runOnce(GroupCtx{1000, 0}));  // still gated by the 1s due: not observed
    EXPECT_EQ(cap.count, 0);

    ASSERT_TRUE(g.setWatchSink(&captureWatchEvent, &cap, 1000).has_value());  // shorten to 1ms
    EXPECT_TRUE(runner.runOnce(GroupCtx{1000, 0}));  // must poll right away, not at the old 1s due
    EXPECT_EQ(cap.count, 1);
    EXPECT_EQ(cap.pin, pin);
    EXPECT_TRUE(cap.level);

    ASSERT_TRUE(g.setWatchSink(nullptr, nullptr).has_value());
}

struct NoOpService : public ::m5::hal::v2::service::IService {
protected:
    ::m5::hal::v2::service::ServicePoll serviceImpl(const ::m5::hal::v2::service::ServiceContext&) override
    {
        return {::m5::hal::v2::service::ServiceResult::Idle};
    }
};

// 14. setWatchSink() failure semantics: with the runner table full it
//     returns OUT_OF_RESOURCE and leaves the deterministic documented
//     state — no sink (notify delivers nothing) and no poll service.
TEST(GPIOGroupWatch, SetWatchSinkFailureLeavesNoSinkNoService)
{
    TinyGPIO mcu{8};
    GPIOGroup g{&mcu};
    ServiceRunner runner;
    g.bindServiceRunner(&runner);

    NoOpService fillers[ServiceRunner::kMaxServices];
    for (auto& f : fillers) {
        ASSERT_TRUE(runner.add(f));
    }
    ASSERT_EQ(runner.size(), ServiceRunner::kMaxServices);

    WatchCapture cap;
    ASSERT_TRUE(g.watch(makeGpioNumber(0, 0)).has_value());
    auto r = g.setWatchSink(&captureWatchEvent, &cap, 1000);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(runner.size(), ServiceRunner::kMaxServices);  // no poll service registered

    // No sink: a push notification for the watched pin goes nowhere.
    ASSERT_TRUE(g.notifyPinStateChanged(makeGpioNumber(0, 0), true).has_value());
    EXPECT_EQ(cap.count, 0);

    runner.clear();
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
