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
//   - Chained groups (a Group of Groups) are out of scope for v1.
//
// The contract is defined in spec/design/gpio.md §GPIOGroup.

#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

namespace {

using ::m5::hal::v1::types::extractLocalPin;
using ::m5::hal::v1::types::extractSlot;
using ::m5::hal::v1::types::gpio_local_pin_t;
using ::m5::hal::v1::types::gpio_mode_t;
using ::m5::hal::v1::types::gpio_number_t;
using ::m5::hal::v1::types::gpio_slot_t;
using ::m5::hal::v1::types::makeGpioNumber;
using StubPort = ::m5::variants::frameworks::stub::hal::v1::gpio::Port;

// Minimal concrete `IGPIO` for tests: wraps a single `stub::Port`
// and takes its width from the ctor. The internal local pin space
// is fixed at `0..(width-1)` (Port._base = 0). The caller manages
// slot assignment by calling `g.addGPIO(this, slot)`. Native-test
// helper only.
struct TinyGPIO : public ::m5::hal::v1::gpio::IGPIO {
    explicit TinyGPIO(uint8_t width) : _port(width, 0), _width(width)
    {
    }

    ::m5::hal::v1::gpio::IPort* portForPin(gpio_local_pin_t local_pin) const override
    {
        (void)local_pin;
        return &_port;
    }
    ::m5::hal::v1::gpio::IPort* getPort(uint8_t port_index) const override
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
    ::m5::hal::v1::gpio::GPIOGroup g{};
    for (gpio_slot_t s = 0; s < 128; ++s) {
        EXPECT_FALSE(g.hasGPIO(s)) << "slot " << static_cast<int>(s) << " should be empty";
    }
    EXPECT_FALSE(g.isValid(makeGpioNumber(0, 0)));
    EXPECT_FALSE(g.isValid(-1));
}

TEST(GPIOGroup, CtorWithMcuLoadsSlot0)
{
    TinyGPIO mcu{32};
    ::m5::hal::v1::gpio::GPIOGroup g{&mcu};
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
    ::m5::hal::v1::gpio::GPIOGroup g{&mcu};

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
    ::m5::hal::v1::gpio::GPIOGroup g{};
    auto r = g.addGPIO(nullptr, 0);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, AddGPIORejectsSlotOutOfRange)
{
    TinyGPIO expander{8};
    ::m5::hal::v1::gpio::GPIOGroup g{};
    // slot 128 is out of the valid range (0-127).
    auto r = g.addGPIO(&expander, 128);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, AddGPIORejectsZeroPinCount)
{
    // An `IGPIO` with `pin_count == 0` is meaningless (no pin can be
    // represented), so registration is rejected with INVALID_ARGUMENT.
    struct EmptyGPIO : public ::m5::hal::v1::gpio::IGPIO {
        ::m5::hal::v1::gpio::IPort* portForPin(gpio_local_pin_t) const override
        {
            return nullptr;
        }
        ::m5::hal::v1::gpio::IPort* getPort(uint8_t) const override
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
    ::m5::hal::v1::gpio::GPIOGroup g{};
    auto r = g.addGPIO(&empty, 1);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, AddGPIORejectsTooManyPins)
{
    // pin_count > 256 (beyond the `gpio_local_pin_t = uint8_t`
    // range) cannot be addressed through `makeGpioNumber`, so the
    // group rejects it. Verified with a 257-pin IGPIO.
    struct TooLargeGPIO : public ::m5::hal::v1::gpio::IGPIO {
        ::m5::hal::v1::gpio::IPort* portForPin(gpio_local_pin_t) const override
        {
            return nullptr;
        }
        ::m5::hal::v1::gpio::IPort* getPort(uint8_t) const override
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
    ::m5::hal::v1::gpio::GPIOGroup g{};
    auto r = g.addGPIO(&too_large, 1);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, AddGPIORejectsOccupiedSlot)
{
    TinyGPIO first{8};
    TinyGPIO second{8};
    ::m5::hal::v1::gpio::GPIOGroup g{};
    ASSERT_TRUE(g.addGPIO(&first, 5).has_value());

    // Trying to add a different IGPIO to a slot that's already taken is rejected.
    auto r = g.addGPIO(&second, 5);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(g.getGPIO(5), &first);  // The existing entry is unchanged.
}

TEST(GPIOGroup, GetPinDispatchesToCorrectSlot)
{
    TinyGPIO mcu{32};
    TinyGPIO expander_a{8};
    ::m5::hal::v1::gpio::GPIOGroup g{&mcu};
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
    ::m5::hal::v1::gpio::GPIOGroup g{&mcu};
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
    EXPECT_EQ(r2.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, RemoveGPIORejectsSlotOutOfRange)
{
    ::m5::hal::v1::gpio::GPIOGroup g{};
    // slot 128 is out of the valid range.
    auto r = g.removeGPIO(128);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, TryGetPinReturnsExpected)
{
    TinyGPIO mcu{32};
    ::m5::hal::v1::gpio::GPIOGroup g{&mcu};

    // Valid input -> returns a Pin.
    auto r1 = g.tryGetPin(makeGpioNumber(0, 5));
    ASSERT_TRUE(r1.has_value());
    EXPECT_TRUE(r1.value().isValid());

    // Invalid input (slot not registered) -> INVALID_ARGUMENT.
    auto r2 = g.tryGetPin(makeGpioNumber(1, 0));
    EXPECT_FALSE(r2.has_value());
    EXPECT_EQ(r2.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);

    // Invalid input (negative value = sentinel) -> INVALID_ARGUMENT.
    auto r3 = g.tryGetPin(-1);
    EXPECT_FALSE(r3.has_value());
    EXPECT_EQ(r3.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);

    // Invalid input (local pin out of range) -> INVALID_ARGUMENT.
    auto r4 = g.tryGetPin(makeGpioNumber(0, 100));  // The MCU only has 32 pins.
    EXPECT_FALSE(r4.has_value());
    EXPECT_EQ(r4.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, Slot0MCUCompatibility)
{
    // Slot 0 is reserved for the MCU, so an existing-caller literal
    // such as `gpio_number_t{21}` should still resolve to MCU local
    // pin 21 (backward-compatibility check).
    TinyGPIO mcu{32};
    ::m5::hal::v1::gpio::GPIOGroup g{&mcu};

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
    ::m5::hal::v1::gpio::GPIOGroup g{&mcu};  // slot 0
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
    constexpr size_t kMax = ::m5::hal::v1::gpio::GPIOGroup::kMaxEntries;
    TinyGPIO gpio{8};
    ::m5::hal::v1::gpio::GPIOGroup g{};
    for (gpio_slot_t s = 0; s < kMax; ++s) {
        EXPECT_TRUE(g.addGPIO(&gpio, s).has_value()) << "slot " << static_cast<int>(s);
    }
    // The first `kMax` entries succeeded; the next one (slot = kMax,
    // in range but past the storage cap) must be rejected.
    auto r = g.addGPIO(&gpio, static_cast<gpio_slot_t>(kMax));
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), ::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(GPIOGroup, RemoveRestoresCapacity)
{
    // Removing one entry from a full group frees the slot back up
    // and a subsequent `addGPIO` succeeds again (`_count` must
    // shrink correctly via compaction).
    constexpr size_t kMax = ::m5::hal::v1::gpio::GPIOGroup::kMaxEntries;
    TinyGPIO gpio{8};
    TinyGPIO extra{8};
    ::m5::hal::v1::gpio::GPIOGroup g{};
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
    ::m5::hal::v1::gpio::GPIOGroup g{&mcu};
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
    ::m5::hal::v1::gpio::GPIOGroup g{};
    EXPECT_DEATH({ (void)g.getPin(-1); }, "invalid sentinel");
}

TEST(GPIOGroupDeathTest, GetPinAssertOnUnregisteredSlot)
{
    ::m5::hal::v1::gpio::GPIOGroup g{};
    EXPECT_DEATH({ (void)g.getPin(makeGpioNumber(1, 0)); }, "slot unregistered");
}

TEST(GPIOGroupDeathTest, GetPinAssertOnLocalPinOutOfRange)
{
    TinyGPIO mcu{32};
    ::m5::hal::v1::gpio::GPIOGroup g{&mcu};
    // Slot 0 (MCU) is registered, but the MCU only has 32 pins, so local 100 is out of range.
    EXPECT_DEATH({ (void)g.getPin(makeGpioNumber(0, 100)); }, "local pin out of range");
}

#endif  // !defined(NDEBUG)

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
