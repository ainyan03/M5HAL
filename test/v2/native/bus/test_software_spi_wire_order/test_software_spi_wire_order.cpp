// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <array>
#include <cstdint>
#include <new>
#include <vector>

// software SPI wire-order invariant: MOSI is valid on BOTH edges of its bit
// cell, i.e. the MOSI update is written before the launch edge, not after it.
//
// Why this matters (measured on real hardware): a classic ESP32 slave in
// CPHA=1 samples MOSI within tens of ns of the launch edge. A master that
// writes CLK first and MOSI second delays the data by one GPIO write gap,
// which such a slave reads as the PREVIOUS bit — the whole stream arrives
// shifted 1 bit late. Canonical doc = spec/design/spi.md wire-timing
// invariants.
//
// The rig records every pin write in program order through a recording
// gpio::IPort registered into the M5_Hal.Gpio group (same slot mechanism as
// the virtual I2C harness), then replays CLK transitions inside the CS-low
// window and asserts the MOSI level at EVERY transition of a bit cell equals
// that cell's TX bit. The old CLK-then-MOSI order fails this at each launch
// edge; the MOSI-then-CLK order passes.

namespace {
namespace v2 = m5::hal::v2;

struct PinEvent {
    uint32_t pin = 0;
    bool level   = false;
};

// Single port serving all recorded pins; writes append to the shared log,
// reads return the last written level (MISO floats low: never written).
class RecordingPort : public v2::gpio::IPort {
public:
    explicit RecordingPort(std::vector<PinEvent>& log) : _log(log)
    {
    }

protected:
    void _writePinEncoded(uint32_t encoded_num, bool value) override
    {
        _log.push_back({encoded_num, value});
        if (encoded_num < 32) {
            _levels = value ? (_levels | (1u << encoded_num)) : (_levels & ~(1u << encoded_num));
        }
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        return encoded_num < 32 && ((_levels >> encoded_num) & 1u) != 0;
    }
    void _setPinModeEncoded(uint32_t, v2::types::gpio_mode_t) override
    {
    }
    v2::types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<v2::types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(v2::types::gpio_local_pin_t pin_index) const override
    {
        return static_cast<uint32_t>(pin_index);
    }

private:
    std::vector<PinEvent>& _log;
    uint32_t _levels = 0;
};

class RecordingGPIO : public v2::gpio::IGPIO {
public:
    explicit RecordingGPIO(RecordingPort& port) : _port(port)
    {
    }
    v2::gpio::IPort* portForPin(v2::types::gpio_local_pin_t) const override
    {
        return &_port;
    }
    v2::gpio::IPort* getPort(uint8_t) const override
    {
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return 5;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }

private:
    RecordingPort& _port;
};

constexpr v2::types::gpio_slot_t kSlot = 2;
constexpr uint32_t kPinClk             = 0;
constexpr uint32_t kPinMosi            = 1;
constexpr uint32_t kPinMiso            = 2;
constexpr uint32_t kPinCs              = 3;
constexpr uint32_t kPinDc              = 4;

bool bitAt(const uint8_t* bytes, size_t bit_index)
{
    return (bytes[bit_index >> 3] & (0x80u >> (bit_index & 7u))) != 0;
}

size_t dcWriteCount(const std::vector<PinEvent>& log)
{
    size_t count = 0;
    for (const auto& event : log) {
        if (event.pin == kPinDc) {
            ++count;
        }
    }
    return count;
}

// Walks the recorded write log and asserts that within the CS-low window,
// the MOSI level observed at every CLK transition matches the TX bit of the
// cell the transition belongs to. The bit index advances after each sample
// transition (CPHA=0: 1st, 3rd, ... transition; CPHA=1: 2nd, 4th, ...).
// Transitions after the last bit (the idle-park edge) are not asserted.
void assertMosiValidOnEveryEdge(const std::vector<PinEvent>& log, uint8_t spi_mode, const uint8_t* tx, size_t tx_len)
{
    const bool cpol = (spi_mode & 0x02) != 0;
    const bool cpha = (spi_mode & 0x01) != 0;

    bool clk               = cpol;
    bool mosi              = false;
    bool cs                = true;
    size_t transition      = 0;  // CLK transitions seen inside the CS window
    size_t bit_index       = 0;
    const size_t bit_count = tx_len * 8;

    for (const auto& ev : log) {
        switch (ev.pin) {
            case kPinMosi:
                mosi = ev.level;
                break;
            case kPinCs:
                cs = ev.level;
                break;
            case kPinClk: {
                const bool transitioned = ev.level != clk;
                clk                     = ev.level;
                if (!transitioned || cs) {
                    break;
                }
                ++transition;
                if (bit_index >= bit_count) {
                    break;  // trailing idle-park edge after the data
                }
                EXPECT_EQ(bitAt(tx, bit_index), mosi) << "MOSI not valid at CLK transition " << transition << " (mode "
                                                      << unsigned(spi_mode) << ", bit " << bit_index << ")";
                // CPHA=0 samples on odd transitions, CPHA=1 on even ones;
                // the cell is consumed once its sample transition passed.
                const bool sample_transition = cpha ? (transition % 2 == 0) : (transition % 2 == 1);
                if (sample_transition) {
                    ++bit_index;
                }
                break;
            }
            default:
                break;
        }
    }
    EXPECT_EQ(bit_count, bit_index) << "not all bits were clocked out (mode " << unsigned(spi_mode) << ")";
}

TEST(SoftwareSpiWireOrder, MosiValidOnLaunchAndSampleEdgesAllModes)
{
    std::vector<PinEvent> log;
    RecordingPort port{log};
    RecordingGPIO gpio{port};
    auto added = v2::M5_Hal.Gpio.addGPIO(&gpio, kSlot);
    ASSERT_TRUE(added.has_value()) << "err=" << v2::error::toString(added.error());

    const uint8_t tx[2] = {0xA5, 0x3C};

    for (uint8_t mode = 0; mode < 4; ++mode) {
        v2::spi::Bus_software bus;
        v2::spi::BusConfig cfg;
        cfg.pin_clk  = v2::types::makeGpioNumber(kSlot, kPinClk);
        cfg.pin_mosi = v2::types::makeGpioNumber(kSlot, kPinMosi);
        cfg.pin_miso = v2::types::makeGpioNumber(kSlot, kPinMiso);
        ASSERT_TRUE(bus.init(cfg).has_value()) << "mode " << unsigned(mode);

        v2::spi::MasterAccessConfig mcfg;
        mcfg.pin_cs   = v2::types::makeGpioNumber(kSlot, kPinCs);
        mcfg.freq     = 1000000;
        mcfg.spi_mode = mode;
        v2::spi::MasterAccessor dev{bus, mcfg};

        log.clear();
        ASSERT_TRUE(dev.beginAccess(0).has_value()) << "mode " << unsigned(mode);
        uint8_t rx[2] = {0, 0};
        auto start    = dev.transfer(v2::spi::TransferDesc{}, m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)},
                                     m5::hal::v2::data::DataSpan{rx, sizeof(rx)});
        ASSERT_TRUE(start.has_value()) << "mode " << unsigned(mode);
        ASSERT_TRUE(dev.endAccess().has_value()) << "mode " << unsigned(mode);

        assertMosiValidOnEveryEdge(log, mode, tx, sizeof(tx));
        ASSERT_TRUE(bus.close().has_value());
    }

    ASSERT_TRUE(v2::M5_Hal.Gpio.removeGPIO(kSlot).has_value());
}

TEST(SoftwareSpiWireOrder, PhaseSpecificDcOverridesLegacyLevel)
{
    std::vector<PinEvent> log;
    RecordingPort port{log};
    RecordingGPIO gpio{port};
    ASSERT_TRUE(v2::M5_Hal.Gpio.addGPIO(&gpio, kSlot).has_value());

    v2::spi::Bus_software bus;
    v2::spi::BusConfig cfg;
    cfg.pin_clk      = v2::types::makeGpioNumber(kSlot, kPinClk);
    cfg.pin_mosi     = v2::types::makeGpioNumber(kSlot, kPinMosi);
    cfg.pin_miso     = v2::types::makeGpioNumber(kSlot, kPinMiso);
    cfg.pin_dc       = v2::types::makeGpioNumber(kSlot, kPinDc);
    auto initialized = bus.init(cfg);
    ASSERT_TRUE(initialized.has_value()) << "err=" << v2::error::toString(initialized.error());

    v2::spi::MasterAccessConfig mcfg;
    mcfg.pin_cs = v2::types::makeGpioNumber(kSlot, kPinCs);
    mcfg.freq   = 1000000;
    v2::spi::MasterAccessor dev{bus, mcfg};

    v2::spi::TransferDesc desc;
    desc.command          = 0x2A;
    desc.command_bytes    = 1;
    desc.command_dc_level = 0;
    desc.data_dc_level    = -1;
    desc.dc_level_valid   = true;
    desc.dc_level         = false;  // Legacy value must not override a phase-specific directive.

    const uint8_t tx = 0xA5;
    log.clear();
    auto begun = dev.beginAccess(0);
    ASSERT_TRUE(begun.has_value()) << "err=" << v2::error::toString(begun.error());
    auto result = dev.transfer(desc, v2::data::ConstDataSpan{&tx, 1}, v2::data::DataSpan{});
    ASSERT_TRUE(result.has_value()) << "err=" << v2::error::toString(result.error());
    auto ended = dev.endAccess();
    ASSERT_TRUE(ended.has_value()) << "err=" << v2::error::toString(ended.error());

    std::vector<bool> dc_levels;
    for (const auto& event : log) {
        if (event.pin == kPinDc) {
            dc_levels.push_back(event.level);
        }
    }
    ASSERT_GE(dc_levels.size(), 2u) << "recorded D/C writes=" << dc_levels.size();
    EXPECT_FALSE(dc_levels.front());  // command phase (idle High was already cached by init)
    EXPECT_TRUE(dc_levels.back());    // unspecified data phase falls back to High

    auto closed = bus.close();
    ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    auto removed = v2::M5_Hal.Gpio.removeGPIO(kSlot);
    ASSERT_TRUE(removed.has_value()) << "err=" << v2::error::toString(removed.error());
}

TEST(SoftwareSpiWireOrder, DcLevelStateIsPerBusWithoutCapacityLimit)
{
    std::vector<PinEvent> log;
    RecordingPort port{log};
    RecordingGPIO gpio{port};
    auto added = v2::M5_Hal.Gpio.addGPIO(&gpio, kSlot);
    ASSERT_TRUE(added.has_value()) << "err=" << v2::error::toString(added.error());

    v2::spi::BusConfig cfg;
    cfg.pin_clk  = v2::types::makeGpioNumber(kSlot, kPinClk);
    cfg.pin_mosi = v2::types::makeGpioNumber(kSlot, kPinMosi);
    cfg.pin_miso = v2::types::makeGpioNumber(kSlot, kPinMiso);
    cfg.pin_dc   = v2::types::makeGpioNumber(kSlot, kPinDc);
    std::array<v2::spi::Bus_software, 10> buses;
    for (auto& bus : buses) {
        auto initialized = bus.init(cfg);
        ASSERT_TRUE(initialized.has_value()) << "err=" << v2::error::toString(initialized.error());
    }

    v2::spi::MasterAccessConfig mcfg;
    mcfg.pin_cs = v2::types::makeGpioNumber(kSlot, kPinCs);
    mcfg.freq   = 1000000;
    for (auto& bus : buses) {
        v2::spi::MasterAccessor accessor{bus, mcfg};
        log.clear();
        auto begun = accessor.beginAccess(0);
        ASSERT_TRUE(begun.has_value()) << "err=" << v2::error::toString(begun.error());
        auto ended = accessor.endAccess(0);
        ASSERT_TRUE(ended.has_value()) << "err=" << v2::error::toString(ended.error());
        EXPECT_EQ(dcWriteCount(log), 0u) << "init already established D/C High for every bus instance";
    }

    v2::spi::MasterAccessor first{buses[0], mcfg};
    v2::spi::TransferDesc low;
    low.dc_level_valid = true;
    low.dc_level       = false;
    const uint8_t tx   = 0x5A;
    auto first_opened  = first.beginAccess(0);
    ASSERT_TRUE(first_opened.has_value()) << "err=" << v2::error::toString(first_opened.error());
    auto sent = first.transfer(low, v2::data::ConstDataSpan{&tx, 1}, v2::data::DataSpan{});
    ASSERT_TRUE(sent.has_value()) << "err=" << v2::error::toString(sent.error());
    auto first_closed = first.endAccess();
    ASSERT_TRUE(first_closed.has_value()) << "err=" << v2::error::toString(first_closed.error());

    log.clear();
    v2::spi::MasterAccessor second{buses[1], mcfg};
    auto second_begun = second.beginAccess(0);
    ASSERT_TRUE(second_begun.has_value()) << "err=" << v2::error::toString(second_begun.error());
    EXPECT_EQ(dcWriteCount(log), 0u) << "one bus's Low state must not change another bus's High state";
    auto second_ended = second.endAccess(0);
    ASSERT_TRUE(second_ended.has_value()) << "err=" << v2::error::toString(second_ended.error());

    log.clear();
    auto first_begun = first.beginAccess(0);
    ASSERT_TRUE(first_begun.has_value()) << "err=" << v2::error::toString(first_begun.error());
    EXPECT_EQ(dcWriteCount(log), 1u) << "the bus that ended Low must restore D/C High";
    auto first_ended = first.endAccess(0);
    ASSERT_TRUE(first_ended.has_value()) << "err=" << v2::error::toString(first_ended.error());

    for (auto& bus : buses) {
        auto closed = bus.close();
        ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    }
    auto reinitialized = buses[0].init(cfg);
    ASSERT_TRUE(reinitialized.has_value()) << "err=" << v2::error::toString(reinitialized.error());
    log.clear();
    v2::spi::MasterAccessor reinit_accessor{buses[0], mcfg};
    auto reinit_begun = reinit_accessor.beginAccess(0);
    ASSERT_TRUE(reinit_begun.has_value()) << "err=" << v2::error::toString(reinit_begun.error());
    EXPECT_EQ(dcWriteCount(log), 0u) << "reinit must establish a fresh D/C High state";
    auto reinit_ended = reinit_accessor.endAccess(0);
    ASSERT_TRUE(reinit_ended.has_value()) << "err=" << v2::error::toString(reinit_ended.error());
    auto reclosed = buses[0].close();
    ASSERT_TRUE(reclosed.has_value()) << "err=" << v2::error::toString(reclosed.error());

    auto removed = v2::M5_Hal.Gpio.removeGPIO(kSlot);
    ASSERT_TRUE(removed.has_value()) << "err=" << v2::error::toString(removed.error());
}

TEST(SoftwareSpiWireOrder, ReusedBusAddressStartsWithFreshDcState)
{
    std::vector<PinEvent> log;
    RecordingPort port{log};
    RecordingGPIO gpio{port};
    auto added = v2::M5_Hal.Gpio.addGPIO(&gpio, kSlot);
    ASSERT_TRUE(added.has_value()) << "err=" << v2::error::toString(added.error());

    v2::spi::BusConfig cfg;
    cfg.pin_clk = v2::types::makeGpioNumber(kSlot, kPinClk);
    cfg.pin_dc  = v2::types::makeGpioNumber(kSlot, kPinDc);
    v2::spi::MasterAccessConfig mcfg;
    mcfg.pin_cs = v2::types::makeGpioNumber(kSlot, kPinCs);
    mcfg.freq   = 1000000;

    using SoftwareBus = v2::spi::Bus_software;
    alignas(SoftwareBus) unsigned char storage[sizeof(SoftwareBus)];
    auto* first      = new (storage) SoftwareBus();
    auto initialized = first->init(cfg);
    ASSERT_TRUE(initialized.has_value()) << "err=" << v2::error::toString(initialized.error());
    v2::spi::TransferDesc low;
    low.dc_level_valid = true;
    low.dc_level       = false;
    v2::spi::MasterAccessor first_device{*first, mcfg};
    auto first_opened = first_device.beginAccess(0);
    ASSERT_TRUE(first_opened.has_value()) << "err=" << v2::error::toString(first_opened.error());
    auto lowered = first_device.transfer(low, v2::data::ConstDataSpan{}, v2::data::DataSpan{});
    ASSERT_TRUE(lowered.has_value()) << "err=" << v2::error::toString(lowered.error());
    auto first_closed = first_device.endAccess();
    ASSERT_TRUE(first_closed.has_value()) << "err=" << v2::error::toString(first_closed.error());
    first->~SoftwareBus();

    auto* second = new (storage) SoftwareBus();
    initialized  = second->init(cfg);
    ASSERT_TRUE(initialized.has_value()) << "err=" << v2::error::toString(initialized.error());
    log.clear();
    v2::spi::MasterAccessor second_device{*second, mcfg};
    auto begun = second_device.beginAccess(0);
    ASSERT_TRUE(begun.has_value()) << "err=" << v2::error::toString(begun.error());
    EXPECT_EQ(dcWriteCount(log), 0u) << "new instance must use its initialized High state, not address history";
    auto ended = second_device.endAccess(0);
    ASSERT_TRUE(ended.has_value()) << "err=" << v2::error::toString(ended.error());
    second->~SoftwareBus();

    auto removed = v2::M5_Hal.Gpio.removeGPIO(kSlot);
    ASSERT_TRUE(removed.has_value()) << "err=" << v2::error::toString(removed.error());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
