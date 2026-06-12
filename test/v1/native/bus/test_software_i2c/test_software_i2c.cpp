// SPDX-License-Identifier: MIT
// Scaffolding for the software (bit-bang) I2C bus native gtest.
// `RecordingPort` (a minimal `gpio::IPort` that records caller events
// in order) plus tests that confirm `software::Bus::init` and the
// Source / Sink based transfer path. Protocol-level checks live in
// the latter half of this file and run `I2CSlaveService` over a
// virtual open-drain bus.
//
// Observation hooks live at the `IPort` layer, not at the `Pin`
// layer, because `Pin` is a POD-ish value type that cannot be
// subclassed.

#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

#include "i2c_virtual_bus.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

class RecordingPort : public m5::hal::v1::gpio::IPort {
public:
    enum class EventKind : uint8_t { Write, Read, SetMode };
    struct Event {
        m5::hal::v1::types::gpio_number_t gpio_num;
        EventKind kind;
        bool value;                            // Carried by Write / Read events.
        m5::hal::v1::types::gpio_mode_t mode;  // Carried by SetMode events.
    };

    void setReadValue(bool value)
    {
        _read_value = value;
    }
    const std::vector<Event>& events(void) const
    {
        return _events;
    }

protected:
    void _writePinEncoded(uint32_t encoded_num, bool v) override
    {
        _events.push_back({static_cast<m5::hal::v1::types::gpio_number_t>(encoded_num), EventKind::Write, v,
                           m5::hal::v1::types::gpio_mode_t::Input});
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        _events.push_back({static_cast<m5::hal::v1::types::gpio_number_t>(encoded_num), EventKind::Read, _read_value,
                           m5::hal::v1::types::gpio_mode_t::Input});
        return _read_value;
    }
    void _setPinModeEncoded(uint32_t encoded_num, m5::hal::v1::types::gpio_mode_t mode) override
    {
        _events.push_back(
            {static_cast<m5::hal::v1::types::gpio_number_t>(encoded_num), EventKind::SetMode, false, mode});
    }
    m5::hal::v1::types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<m5::hal::v1::types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        return static_cast<uint32_t>(pin_index);
    }

private:
    // I2C idle is SDA/SCL high (open-drain pulled up). Default to true so
    // the bit-bang state machine sees a quiet bus rather than a hung slave.
    bool _read_value = true;
    std::vector<Event> _events;
};

// Helper IGPIO that bundles two RecordingPorts (SCL, SDA). When
// registered to `GPIOGroup` at a chosen slot, this restores the
// "observe the bit-bang activity through RecordingPort" path even
// after `BusConfig` collapsed to the single `gpio_number_t` entry
// (Pin value-type direct assignment is no longer supported); it is
// the escape hatch tests use for that observation.
class RecordingGPIO : public m5::hal::v1::gpio::IGPIO {
public:
    RecordingGPIO(RecordingPort& scl, RecordingPort& sda) : _scl(scl), _sda(sda)
    {
    }

    m5::hal::v1::gpio::IPort* portForPin(m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        return (pin_index == 0) ? &_scl : &_sda;
    }
    m5::hal::v1::gpio::IPort* getPort(uint8_t port_index) const override
    {
        return (port_index == 0) ? &_scl : &_sda;
    }
    uint16_t getPinCount() const override
    {
        return 2;
    }
    uint8_t getPortCount() const override
    {
        return 2;
    }

private:
    RecordingPort& _scl;
    RecordingPort& _sda;
};

// RAII helper that registers / unregisters a `RecordingGPIO` at
// slot 1 of `M5_Hal.Gpio` (the singleton `GPIOGroup`). Slot 0 is
// reserved for the MCU GPIO, so the test-side expander analogue
// pins itself to slot 1. `scl()` / `sda()` build the global
// `gpio_number_t` for local pin 0 / 1 inside slot 1 via
// `makeGpioNumber`.
//
// The singleton ownership raises the cost of a missed fixture
// teardown, so this wrapper asserts `r.has_value()` on every
// add / remove for fail-fast diagnostics — the older
// `(void)r` style is intentionally avoided.
struct ScopedRecordingGPIO {
    static constexpr m5::hal::v1::types::gpio_slot_t kSlot = 1;

    ScopedRecordingGPIO(RecordingPort& scl, RecordingPort& sda) : _gpio(scl, sda)
    {
        auto r = m5::hal::v1::M5_Hal.Gpio.addGPIO(&_gpio, kSlot);
        assert(r.has_value() && "ScopedRecordingGPIO: slot 1 add failed (singleton slot pollution?)");
        (void)r;  // Silences the [[nodiscard]] warning when asserts are disabled in release.
    }
    ~ScopedRecordingGPIO()
    {
        auto r = m5::hal::v1::M5_Hal.Gpio.removeGPIO(kSlot);
        assert(r.has_value() && "ScopedRecordingGPIO: slot 1 remove failed (fixture teardown drift)");
        (void)r;
    }

    m5::hal::v1::types::gpio_number_t scl() const
    {
        return m5::hal::v1::types::makeGpioNumber(kSlot, 0);
    }
    m5::hal::v1::types::gpio_number_t sda() const
    {
        return m5::hal::v1::types::makeGpioNumber(kSlot, 1);
    }

private:
    RecordingGPIO _gpio;
};

class FakeMasterLineDriver : public m5::variants::frameworks::software::hal::v1::i2c::detail::MasterLineDriver {
public:
    enum class EventKind : uint8_t { SCL, SDA };
    struct Event {
        EventKind kind;
        bool high;
    };

    void writeSclHigh() override
    {
        scl_high = true;
        events.push_back({EventKind::SCL, true});
    }
    void writeSclLow() override
    {
        scl_high = false;
        events.push_back({EventKind::SCL, false});
    }
    void writeSda(bool high) override
    {
        sda_high = high;
        events.push_back({EventKind::SDA, high});
    }
    bool readScl() const override
    {
        return scl_read_high;
    }
    bool readSda() const override
    {
        return sda_read_high;
    }

    bool scl_high      = true;
    bool sda_high      = true;
    bool scl_read_high = true;
    bool sda_read_high = true;
    std::vector<Event> events;
};

TEST(SoftwareI2CMasterTiming, ConvertsFreqToNsecHalfPeriod)
{
    using m5::variants::frameworks::software::hal::v1::i2c::detail::MasterTiming;

    m5::hal::v1::i2c::I2CMasterAccessConfig cfg;

    cfg.freq      = 100000;
    auto standard = MasterTiming::fromConfig(cfg);
    ASSERT_TRUE(standard.has_value());
    EXPECT_EQ(standard->half_period, 5000u);

    cfg.freq  = 400000;
    auto fast = MasterTiming::fromConfig(cfg);
    ASSERT_TRUE(fast.has_value());
    EXPECT_EQ(fast->half_period, 1250u);

    cfg.freq       = 1000000;
    auto fast_plus = MasterTiming::fromConfig(cfg);
    ASSERT_TRUE(fast_plus.has_value());
    EXPECT_EQ(fast_plus->half_period, 500u);
}

TEST(SoftwareI2CMasterTiming, RejectsInvalidFreqAndOversizedTimeout)
{
    using m5::variants::frameworks::software::hal::v1::i2c::detail::MasterTiming;

    m5::hal::v1::i2c::I2CMasterAccessConfig cfg;

    cfg.freq          = 0;
    auto invalid_freq = MasterTiming::fromConfig(cfg);
    ASSERT_FALSE(invalid_freq.has_value());
    EXPECT_EQ(invalid_freq.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);

    cfg.freq             = 100000;
    cfg.timeout_ms       = 3000;
    auto invalid_timeout = MasterTiming::fromConfig(cfg);
    ASSERT_FALSE(invalid_timeout.has_value());
    EXPECT_EQ(invalid_timeout.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(SoftwareI2CMasterStartCondition, AdvancesByNsecTicks)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 100;

    StartConditionService start;
    start.begin(lines, timing, 1000);

    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{1000}), m5::hal::v1::service::ServiceResult::Progress);
    ASSERT_EQ(lines.events.size(), size_t{1});
    EXPECT_EQ(lines.events[0].kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_TRUE(lines.events[0].high);
    EXPECT_EQ(start.dueTick(), 1100u);

    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{1099}), m5::hal::v1::service::ServiceResult::Idle);
    EXPECT_EQ(lines.events.size(), size_t{1});

    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{1100}), m5::hal::v1::service::ServiceResult::Progress);
    ASSERT_EQ(lines.events.size(), size_t{2});
    EXPECT_EQ(lines.events[1].kind, FakeMasterLineDriver::EventKind::SDA);
    EXPECT_FALSE(lines.events[1].high);
    EXPECT_EQ(start.dueTick(), 1200u);

    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{1200}), m5::hal::v1::service::ServiceResult::Done);
    ASSERT_EQ(lines.events.size(), size_t{3});
    EXPECT_EQ(lines.events[2].kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_FALSE(lines.events[2].high);
    EXPECT_TRUE(start.done());
}

TEST(SoftwareI2CMasterStartCondition, WaitsForClockStretchRelease)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 100;
    timing.timeout     = 1000;

    StartConditionService start;
    start.begin(lines, timing, 2000);

    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{2000}), m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(start.state(), StartConditionService::State::WaitClockHigh);
    ASSERT_EQ(lines.events.size(), size_t{1});
    EXPECT_EQ(lines.events.back().kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_TRUE(lines.events.back().high);

    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{2500}), m5::hal::v1::service::ServiceResult::Idle);
    EXPECT_EQ(lines.events.size(), size_t{1});

    lines.scl_read_high = true;
    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{2600}), m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(start.state(), StartConditionService::State::PullSdaLow);
    EXPECT_EQ(start.dueTick(), 2700u);
}

TEST(SoftwareI2CMasterStartCondition, ReportsTimeoutWhenSclStaysLow)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 30;

    StartConditionService start;
    start.begin(lines, timing, 3000);

    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{3000}), m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(start.state(), StartConditionService::State::WaitClockHigh);

    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{3029}), m5::hal::v1::service::ServiceResult::Idle);
    EXPECT_EQ(start.service(m5::hal::v1::service::ServiceContext{3030}), m5::hal::v1::service::ServiceResult::Error);
    EXPECT_EQ(start.state(), StartConditionService::State::Timeout);
    EXPECT_EQ(start.error(), m5::hal::v1::error::error_t::TIMEOUT_ERROR);
    EXPECT_FALSE(start.done());
}

TEST(SoftwareI2CMasterWriteByte, SendsBitsMsbFirstAndSamplesAck)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = false;  // slave ACK
    MasterTiming timing;
    timing.half_period = 10;

    WriteByteService writer;
    writer.begin(lines, timing, 0xA5, 1000);

    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{999}), m5::hal::v1::service::ServiceResult::Idle);
    ASSERT_EQ(lines.events.size(), size_t{1});
    EXPECT_EQ(lines.events[0].kind, FakeMasterLineDriver::EventKind::SDA);
    EXPECT_TRUE(lines.events[0].high);

    m5::hal::v1::service::ServiceResult result = m5::hal::v1::service::ServiceResult::Idle;
    for (size_t i = 0; i < 32 && result != m5::hal::v1::service::ServiceResult::Done; ++i) {
        result = writer.service(m5::hal::v1::service::ServiceContext{writer.dueTick()});
    }

    EXPECT_EQ(result, m5::hal::v1::service::ServiceResult::Done);
    EXPECT_TRUE(writer.done());
    EXPECT_TRUE(writer.acked());
    EXPECT_EQ(writer.bitIndex(), 7u);

    std::vector<bool> sda_bits;
    size_t scl_rise_count = 0;
    size_t scl_fall_count = 0;
    for (const auto& event : lines.events) {
        if (event.kind == FakeMasterLineDriver::EventKind::SDA && sda_bits.size() < 8) {
            sda_bits.push_back(event.high);
        }
        if (event.kind == FakeMasterLineDriver::EventKind::SCL) {
            if (event.high) {
                ++scl_rise_count;
            } else {
                ++scl_fall_count;
            }
        }
    }

    const std::vector<bool> expected_bits{true, false, true, false, false, true, false, true};
    EXPECT_EQ(sda_bits, expected_bits);
    EXPECT_EQ(scl_rise_count, size_t{9});
    EXPECT_EQ(scl_fall_count, size_t{9});
    ASSERT_FALSE(lines.events.empty());
    auto last_sda = lines.events.end();
    do {
        --last_sda;
    } while (last_sda != lines.events.begin() && last_sda->kind != FakeMasterLineDriver::EventKind::SDA);
    ASSERT_EQ(last_sda->kind, FakeMasterLineDriver::EventKind::SDA);
    EXPECT_TRUE(last_sda->high);
    EXPECT_EQ(lines.events.back().kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_FALSE(lines.events.back().high);
}

TEST(SoftwareI2CMasterWriteByte, KeepsClockPhaseWhenServiceRunsLate)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    WriteByteService writer;
    writer.begin(lines, timing, 0x80, 1000);

    EXPECT_EQ(writer.dueTick(), 1010u);
    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{1013}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::LowerClock);
    EXPECT_EQ(writer.dueTick(), 1020u);

    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{1024}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::RaiseClock);
    EXPECT_EQ(writer.dueTick(), 1030u);
}

TEST(SoftwareI2CMasterWriteByte, ResyncsClockPhaseWhenServiceRunsTooLate)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    WriteByteService writer;
    writer.begin(lines, timing, 0x80, 1000);

    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{1055}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::LowerClock);
    EXPECT_EQ(writer.dueTick(), 1065u);
}

TEST(SoftwareI2CMasterWriteByte, ReportsNackAfterReturningClockLow)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = true;  // slave NACK
    MasterTiming timing;
    timing.half_period = 10;

    WriteByteService writer;
    writer.begin(lines, timing, 0x00, 2000);

    m5::hal::v1::service::ServiceResult result = m5::hal::v1::service::ServiceResult::Idle;
    for (size_t i = 0; i < 32 && result != m5::hal::v1::service::ServiceResult::Error; ++i) {
        result = writer.service(m5::hal::v1::service::ServiceContext{writer.dueTick()});
    }

    EXPECT_EQ(result, m5::hal::v1::service::ServiceResult::Error);
    EXPECT_FALSE(writer.done());
    EXPECT_FALSE(writer.acked());
    EXPECT_EQ(writer.state(), WriteByteService::State::Nack);
    EXPECT_EQ(writer.error(), m5::hal::v1::error::error_t::I2C_NO_ACK);
    ASSERT_FALSE(lines.events.empty());
    EXPECT_EQ(lines.events.back().kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_FALSE(lines.events.back().high);
}

TEST(SoftwareI2CMasterWriteByte, WaitsForClockStretchRelease)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    lines.sda_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 100;

    WriteByteService writer;
    writer.begin(lines, timing, 0x80, 1000);

    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{1000}), m5::hal::v1::service::ServiceResult::Idle);
    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{1010}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::WaitClockHigh);
    ASSERT_EQ(lines.events.size(), size_t{2});
    EXPECT_EQ(lines.events.back().kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_TRUE(lines.events.back().high);

    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{1050}), m5::hal::v1::service::ServiceResult::Idle);
    EXPECT_EQ(lines.events.size(), size_t{2});

    lines.scl_read_high = true;
    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{1060}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::LowerClock);
    EXPECT_EQ(writer.dueTick(), 1070u);
}

TEST(SoftwareI2CMasterWriteByte, ReportsTimeoutWhenClockStretchDoesNotRelease)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 30;

    WriteByteService writer;
    writer.begin(lines, timing, 0x80, 2000);

    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{2000}), m5::hal::v1::service::ServiceResult::Idle);
    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{2010}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::WaitClockHigh);

    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{2039}), m5::hal::v1::service::ServiceResult::Idle);
    EXPECT_EQ(writer.service(m5::hal::v1::service::ServiceContext{2040}), m5::hal::v1::service::ServiceResult::Error);
    EXPECT_EQ(writer.state(), WriteByteService::State::Timeout);
    EXPECT_EQ(writer.error(), m5::hal::v1::error::error_t::TIMEOUT_ERROR);
    EXPECT_FALSE(writer.done());
}

TEST(SoftwareI2CMasterStopCondition, ReleasesSdaWhileSclHigh)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = true;
    MasterTiming timing;
    timing.half_period = 10;

    StopConditionService stop;
    stop.begin(lines, timing, 1000);

    EXPECT_EQ(stop.service(m5::hal::v1::service::ServiceContext{1000}), m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(stop.service(m5::hal::v1::service::ServiceContext{1010}), m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(stop.service(m5::hal::v1::service::ServiceContext{1020}), m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(stop.service(m5::hal::v1::service::ServiceContext{1030}), m5::hal::v1::service::ServiceResult::Done);

    ASSERT_EQ(lines.events.size(), size_t{3});
    EXPECT_EQ(lines.events[0].kind, FakeMasterLineDriver::EventKind::SDA);
    EXPECT_FALSE(lines.events[0].high);
    EXPECT_EQ(lines.events[1].kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_TRUE(lines.events[1].high);
    EXPECT_EQ(lines.events[2].kind, FakeMasterLineDriver::EventKind::SDA);
    EXPECT_TRUE(lines.events[2].high);
    EXPECT_TRUE(stop.done());
}

TEST(SoftwareI2CMasterStopCondition, ReportsBusErrorWhenSdaStaysLow)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;

    StopConditionService stop;
    stop.begin(lines, timing, 2000);

    m5::hal::v1::service::ServiceResult result = m5::hal::v1::service::ServiceResult::Idle;
    for (size_t i = 0; i < 8 && result != m5::hal::v1::service::ServiceResult::Error; ++i) {
        result = stop.service(m5::hal::v1::service::ServiceContext{stop.dueTick()});
    }

    EXPECT_EQ(result, m5::hal::v1::service::ServiceResult::Error);
    EXPECT_EQ(stop.state(), StopConditionService::State::BusError);
    EXPECT_EQ(stop.error(), m5::hal::v1::error::error_t::I2C_BUS_ERROR);
}

TEST(SoftwareI2CMasterStopCondition, ReportsTimeoutWhenSclStaysLow)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 30;

    StopConditionService stop;
    stop.begin(lines, timing, 3000);

    EXPECT_EQ(stop.service(m5::hal::v1::service::ServiceContext{3000}), m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(stop.service(m5::hal::v1::service::ServiceContext{3010}), m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(stop.state(), StopConditionService::State::WaitClockHigh);

    EXPECT_EQ(stop.service(m5::hal::v1::service::ServiceContext{3039}), m5::hal::v1::service::ServiceResult::Idle);
    EXPECT_EQ(stop.service(m5::hal::v1::service::ServiceContext{3040}), m5::hal::v1::service::ServiceResult::Error);
    EXPECT_EQ(stop.state(), StopConditionService::State::Timeout);
    EXPECT_EQ(stop.error(), m5::hal::v1::error::error_t::TIMEOUT_ERROR);
}

TEST(SoftwareI2CMasterReadByte, SamplesBitsMsbFirstAndSendsAck)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    ReadByteService reader;
    reader.begin(lines, timing, true, 1000);

    const bool bits[]                          = {true, false, true, false, false, true, false, true};
    m5::hal::v1::service::ServiceResult result = m5::hal::v1::service::ServiceResult::Idle;
    size_t sampled_bits                        = 0;
    for (size_t i = 0; i < 64 && result != m5::hal::v1::service::ServiceResult::Done; ++i) {
        if (reader.state() == ReadByteService::State::SampleBit && sampled_bits < sizeof(bits)) {
            lines.sda_read_high = bits[sampled_bits++];
        }
        result = reader.service(m5::hal::v1::service::ServiceContext{reader.dueTick()});
    }

    EXPECT_EQ(result, m5::hal::v1::service::ServiceResult::Done);
    EXPECT_TRUE(reader.done());
    EXPECT_EQ(reader.byte(), 0xA5);
    EXPECT_EQ(sampled_bits, size_t{8});
    ASSERT_FALSE(lines.events.empty());
    EXPECT_EQ(lines.events.back().kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_FALSE(lines.events.back().high);

    auto last_sda = lines.events.end();
    do {
        --last_sda;
    } while (last_sda != lines.events.begin() && last_sda->kind != FakeMasterLineDriver::EventKind::SDA);
    ASSERT_EQ(last_sda->kind, FakeMasterLineDriver::EventKind::SDA);
    EXPECT_FALSE(last_sda->high);
}

TEST(SoftwareI2CMasterReadByte, SendsNackAfterFinalByte)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = true;
    MasterTiming timing;
    timing.half_period = 10;

    ReadByteService reader;
    reader.begin(lines, timing, false, 2000);

    m5::hal::v1::service::ServiceResult result = m5::hal::v1::service::ServiceResult::Idle;
    for (size_t i = 0; i < 64 && result != m5::hal::v1::service::ServiceResult::Done; ++i) {
        result = reader.service(m5::hal::v1::service::ServiceContext{reader.dueTick()});
    }

    EXPECT_EQ(result, m5::hal::v1::service::ServiceResult::Done);
    EXPECT_EQ(reader.byte(), 0xFF);

    auto last_sda = lines.events.end();
    do {
        --last_sda;
    } while (last_sda != lines.events.begin() && last_sda->kind != FakeMasterLineDriver::EventKind::SDA);
    ASSERT_EQ(last_sda->kind, FakeMasterLineDriver::EventKind::SDA);
    EXPECT_TRUE(last_sda->high);
}

TEST(SoftwareI2CMasterReadByte, KeepsClockPhaseWhenServiceRunsLate)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    ReadByteService reader;
    reader.begin(lines, timing, true, 1000);

    EXPECT_EQ(reader.dueTick(), 1000u);
    EXPECT_EQ(reader.service(m5::hal::v1::service::ServiceContext{1003}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::RaiseClock);
    EXPECT_EQ(reader.dueTick(), 1013u);

    EXPECT_EQ(reader.service(m5::hal::v1::service::ServiceContext{1016}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::SampleBit);
    EXPECT_EQ(reader.dueTick(), 1023u);

    EXPECT_EQ(reader.service(m5::hal::v1::service::ServiceContext{1028}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::RaiseClock);
    EXPECT_EQ(reader.dueTick(), 1033u);
}

TEST(SoftwareI2CMasterReadByte, ResyncsClockPhaseWhenServiceRunsTooLate)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    ReadByteService reader;
    reader.begin(lines, timing, true, 1000);

    EXPECT_EQ(reader.service(m5::hal::v1::service::ServiceContext{1000}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::RaiseClock);
    EXPECT_EQ(reader.dueTick(), 1010u);

    EXPECT_EQ(reader.service(m5::hal::v1::service::ServiceContext{1055}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::SampleBit);
    EXPECT_EQ(reader.dueTick(), 1065u);
}

TEST(SoftwareI2CMasterReadByte, ReportsTimeoutWhenClockStretchDoesNotRelease)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 30;

    ReadByteService reader;
    reader.begin(lines, timing, true, 3000);

    EXPECT_EQ(reader.service(m5::hal::v1::service::ServiceContext{3000}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(reader.service(m5::hal::v1::service::ServiceContext{3010}),
              m5::hal::v1::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::WaitClockHigh);
    EXPECT_EQ(reader.service(m5::hal::v1::service::ServiceContext{3039}), m5::hal::v1::service::ServiceResult::Idle);
    EXPECT_EQ(reader.service(m5::hal::v1::service::ServiceContext{3040}), m5::hal::v1::service::ServiceResult::Error);
    EXPECT_EQ(reader.state(), ReadByteService::State::Timeout);
    EXPECT_EQ(reader.error(), m5::hal::v1::error::error_t::TIMEOUT_ERROR);
}

TEST(SoftwareI2CMasterTransaction, RunsStartWriteReadAndStopPrimitives)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period  = 10;
    lines.sda_read_high = false;  // ACK for write byte

    MasterTransactionService transaction;
    transaction.beginStart(lines, timing, 1000);

    auto run_until_done = [&](uint32_t now_tick) {
        m5::hal::v1::service::ServiceResult result = m5::hal::v1::service::ServiceResult::Idle;
        for (size_t i = 0; i < 64 && result != m5::hal::v1::service::ServiceResult::Done; ++i) {
            result = transaction.service(m5::hal::v1::service::ServiceContext{now_tick});
            now_tick += 10;
        }
        return result;
    };

    EXPECT_EQ(run_until_done(1000), m5::hal::v1::service::ServiceResult::Done);
    EXPECT_EQ(transaction.operation(), MasterTransactionService::Operation::Idle);

    transaction.beginWriteByte(lines, timing, 0xA5, 2000);
    EXPECT_EQ(run_until_done(2000), m5::hal::v1::service::ServiceResult::Done);
    EXPECT_TRUE(transaction.acked());

    lines.sda_read_high = true;
    transaction.beginReadByte(lines, timing, false, 3000);
    m5::hal::v1::service::ServiceResult result = m5::hal::v1::service::ServiceResult::Idle;
    for (uint32_t now_tick = 3000; now_tick < 4000 && result != m5::hal::v1::service::ServiceResult::Done;
         now_tick += 10) {
        result = transaction.service(m5::hal::v1::service::ServiceContext{now_tick});
    }
    EXPECT_EQ(result, m5::hal::v1::service::ServiceResult::Done);
    EXPECT_EQ(transaction.byte(), 0xFF);

    lines.sda_read_high = true;
    transaction.beginStop(lines, timing, 4000);
    EXPECT_EQ(run_until_done(4000), m5::hal::v1::service::ServiceResult::Done);
    EXPECT_EQ(transaction.operation(), MasterTransactionService::Operation::Idle);
}

TEST(SoftwareI2CMasterTransaction, PropagatesPrimitiveErrors)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = true;  // NACK for write byte
    MasterTiming timing;
    timing.half_period = 10;

    MasterTransactionService transaction;
    transaction.beginWriteByte(lines, timing, 0x00, 1000);

    m5::hal::v1::service::ServiceResult result = m5::hal::v1::service::ServiceResult::Idle;
    for (size_t i = 0; i < 64 && result != m5::hal::v1::service::ServiceResult::Error; ++i) {
        result = transaction.service(m5::hal::v1::service::ServiceContext{1000 + static_cast<uint32_t>(i * 10)});
    }

    EXPECT_EQ(result, m5::hal::v1::service::ServiceResult::Error);
    EXPECT_EQ(transaction.operation(), MasterTransactionService::Operation::Idle);
    EXPECT_EQ(transaction.error(), m5::hal::v1::error::error_t::I2C_NO_ACK);
}

TEST(SoftwareI2CMasterTransaction, RunsAddressAndBufferSequences)
{
    using namespace m5::variants::frameworks::software::hal::v1::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = false;  // ACK for address/data bytes
    MasterTiming timing;
    timing.half_period = 10;

    auto run_until_done = [](MasterTransactionService& transaction, uint32_t now_tick) {
        m5::hal::v1::service::ServiceResult result = m5::hal::v1::service::ServiceResult::Idle;
        for (size_t i = 0; i < 128 && result != m5::hal::v1::service::ServiceResult::Done; ++i) {
            result = transaction.service(m5::hal::v1::service::ServiceContext{now_tick});
            now_tick += 10;
        }
        return result;
    };

    MasterTransactionService transaction;
    transaction.beginAddress(lines, timing, 0x68 << 1, 1000);
    EXPECT_EQ(run_until_done(transaction, 1000), m5::hal::v1::service::ServiceResult::Done);
    EXPECT_EQ(transaction.operation(), MasterTransactionService::Operation::Idle);
    EXPECT_EQ(transaction.transferred(), size_t{0});

    const uint8_t tx[] = {0x12, 0x34, 0x56};
    transaction.beginWriteBuffer(lines, timing, tx, sizeof(tx), 3000);
    EXPECT_EQ(run_until_done(transaction, 3000), m5::hal::v1::service::ServiceResult::Done);
    EXPECT_EQ(transaction.transferred(), sizeof(tx));

    uint8_t rx[2]       = {};
    lines.sda_read_high = true;
    transaction.beginReadBuffer(lines, timing, rx, sizeof(rx), true, 6000);
    EXPECT_EQ(run_until_done(transaction, 6000), m5::hal::v1::service::ServiceResult::Done);
    EXPECT_EQ(transaction.transferred(), sizeof(rx));
    EXPECT_EQ(rx[0], 0xFF);
    EXPECT_EQ(rx[1], 0xFF);
}

TEST(SoftwareI2CBus, TransferRecordsPinEventsViaPrefix)
{
    RecordingPort scl_port;
    RecordingPort sda_port;
    ScopedRecordingGPIO rec{scl_port, sda_port};

    m5::hal::v1::i2c::I2CBusConfig bus_cfg{rec.scl(), rec.sda()};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    auto init_result = bus.init(bus_cfg);
    EXPECT_TRUE(init_result.has_value());

    // init() drives both pins to set up the idle state, so events must exist.
    EXPECT_FALSE(scl_port.events().empty());
    EXPECT_FALSE(sda_port.events().empty());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x68;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;

    // The prefix bytes go directly into the `TransferDesc` inline buffer.
    m5::hal::v1::i2c::TransferDesc desc;
    desc.prefix[0]  = 0xAA;
    desc.prefix_len = 1;

    const size_t baseline_scl = scl_port.events().size();
    const size_t baseline_sda = sda_port.events().size();

    // We don't care whether transfer succeeds — the simulated slave (a
    // fixed-value RecordingPin) cannot acknowledge correctly. All this
    // skeleton asserts is that transfer drove SCL/SDA, i.e. the bit-bang
    // state machine actually ran. Protocol-level checks below use
    // I2CSlaveService on VirtualOpenDrainBus.
    //
    // `owner` is reserved for future lock semantics and accepts nullptr here.
    (void)bus.transfer(nullptr, acc_cfg, desc, nullptr, nullptr);
    EXPECT_GT(scl_port.events().size(), baseline_scl);
    EXPECT_GT(sda_port.events().size(), baseline_sda);
}

TEST(SoftwareI2CBus, TransferRejectsInvalidFrequency)
{
    RecordingPort scl_port;
    RecordingPort sda_port;
    ScopedRecordingGPIO rec{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{rec.scl(), rec.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr = 0x68;
    acc_cfg.freq     = 0;

    auto r = bus.transfer(nullptr, acc_cfg, m5::hal::v1::i2c::TransferDesc{}, nullptr, nullptr);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(SoftwareI2CBus, TransferRecordsPinEventsViaSource)
{
    RecordingPort scl_port;
    RecordingPort sda_port;
    ScopedRecordingGPIO rec{scl_port, sda_port};

    m5::hal::v1::i2c::I2CBusConfig bus_cfg{rec.scl(), rec.sda()};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    auto init_result = bus.init(bus_cfg);
    EXPECT_TRUE(init_result.has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x68;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;

    // Push tx bytes through a MemorySource to exercise the Source path.
    const uint8_t tx_bytes[] = {0xBB, 0xCC};
    m5::hal::v1::data::MemorySource tx_src{m5::hal::v1::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    const size_t baseline_scl = scl_port.events().size();
    const size_t baseline_sda = sda_port.events().size();

    m5::hal::v1::i2c::TransferDesc desc;  // No prefix.
    (void)bus.transfer(nullptr, acc_cfg, desc, &tx_src, nullptr);
    EXPECT_GT(scl_port.events().size(), baseline_scl);
    EXPECT_GT(sda_port.events().size(), baseline_sda);
}

// `probe` (an all-empty transfer) must travel through the variant's
// "send address+W + check ACK" path and actually drive SCL / SDA
// waveforms (an earlier implementation just returned success without
// doing anything). This recording-pin test only confirms that
// waveforms appear on the wire. ACK and read / write semantics are
// pinned down later by the I2CSlaveService + VirtualOpenDrainBus tests.
TEST(SoftwareI2CBus, ProbeProducesWireActivity)
{
    RecordingPort scl_port;
    RecordingPort sda_port;
    ScopedRecordingGPIO rec{scl_port, sda_port};

    m5::hal::v1::i2c::I2CBusConfig bus_cfg{rec.scl(), rec.sda()};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    auto init_result = bus.init(bus_cfg);
    EXPECT_TRUE(init_result.has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x42;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;

    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    const size_t baseline_scl = scl_port.events().size();
    const size_t baseline_sda = sda_port.events().size();

    // `probe()` internally issues an empty transfer (prefix / tx / rx
    // all empty). The bit-bang variant has to emit address+W
    // (one byte = 8 SCL clocks + 1 ACK clock). RecordingPin's SDA
    // read defaults to `true` (NACK), so `probe` will surface an
    // error; the point of this test is just "did a waveform happen".
    (void)accessor.probe();
    EXPECT_GT(scl_port.events().size(), baseline_scl);
    EXPECT_GT(sda_port.events().size(), baseline_sda);
}

TEST(SoftwareI2CBus, VirtualSlaveServiceAcksProbe)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    I2CSlaveService slave{lines, 0x42};
    runner.add(slave);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x42;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};
    auto r = accessor.probe();
    EXPECT_TRUE(r.has_value());
}

TEST(I2CSlaveService, IdleBeforeInit)
{
    m5::hal::v1::i2c::I2CSlaveService slave;
    EXPECT_EQ(slave.service(m5::hal::v1::service::ServiceContext{0}), m5::hal::v1::service::ServiceResult::Idle);
}

TEST(I2CSlaveService, Rejects10BitAddressUntilImplemented)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    m5::hal::v1::i2c::I2CSlaveService slave;
    m5::hal::v1::i2c::I2CSlaveConfig cfg;
    cfg.address          = 0x120;
    cfg.address_is_10bit = true;

    auto r = slave.init(lines, cfg);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(I2CSlaveService, MaxAckedWriteBytesSurvivesInit)
{
    // Configuration set before init() must survive it: resetProtocol()
    // resets protocol state, not configuration (S16 D6).
    service_proto::VirtualOpenDrainBus lines;
    m5::hal::v1::i2c::I2CSlaveService slave;
    slave.setMaxAckedWriteBytes(3);

    m5::hal::v1::i2c::I2CSlaveConfig cfg;
    cfg.address = 0x42;
    ASSERT_TRUE(slave.init(lines, cfg).has_value());
    EXPECT_EQ(slave.maxAckedWriteBytes(), 3u);
}

TEST(I2CSlaveDriverRegistration, RegistersAndRemovesDriverService)
{
    class CountingService : public m5::hal::v1::service::IService {
    public:
        m5::hal::v1::service::ServiceResult service(const m5::hal::v1::service::ServiceContext&) override
        {
            ++calls;
            return m5::hal::v1::service::ServiceResult::Progress;
        }
        size_t calls = 0;
    };
    class StubSlaveDriver : public m5::hal::v1::i2c::I2CSlaveDriver {
    public:
        explicit StubSlaveDriver(m5::hal::v1::service::IService* svc) : svc{svc}
        {
        }
        m5::hal::v1::result_t<void> init(const m5::hal::v1::i2c::I2CSlaveConfig&) override
        {
            return {};
        }
        m5::hal::v1::result_t<void> release() override
        {
            return {};
        }
        m5::hal::v1::service::IService* service() override
        {
            return svc;
        }
        m5::hal::v1::service::IService* svc = nullptr;
    };

    CountingService service;
    StubSlaveDriver driver{&service};
    m5::hal::v1::service::ServiceRunner runner;

    {
        m5::hal::v1::i2c::ScopedI2CSlaveServiceRegistration registration;
        auto r = registration.registerTo(runner, driver);
        ASSERT_TRUE(r.has_value());
        EXPECT_TRUE(registration.registered());
        EXPECT_EQ(runner.size(), size_t{1});
        EXPECT_TRUE(runner.runOnce(0));
        EXPECT_EQ(service.calls, size_t{1});
    }

    EXPECT_EQ(runner.size(), size_t{0});
    EXPECT_FALSE(runner.runOnce(0));
    EXPECT_EQ(service.calls, size_t{1});
}

TEST(I2CSlaveDriverRegistration, RejectsNullOrDuplicateService)
{
    class IdleService : public m5::hal::v1::service::IService {
    public:
        m5::hal::v1::service::ServiceResult service(const m5::hal::v1::service::ServiceContext&) override
        {
            return m5::hal::v1::service::ServiceResult::Idle;
        }
    };
    class StubSlaveDriver : public m5::hal::v1::i2c::I2CSlaveDriver {
    public:
        explicit StubSlaveDriver(m5::hal::v1::service::IService* svc) : svc{svc}
        {
        }
        m5::hal::v1::result_t<void> init(const m5::hal::v1::i2c::I2CSlaveConfig&) override
        {
            return {};
        }
        m5::hal::v1::result_t<void> release() override
        {
            return {};
        }
        m5::hal::v1::service::IService* service() override
        {
            return svc;
        }
        m5::hal::v1::service::IService* svc = nullptr;
    };

    IdleService service;
    StubSlaveDriver null_driver{nullptr};
    StubSlaveDriver driver{&service};
    m5::hal::v1::service::ServiceRunner runner;
    m5::hal::v1::i2c::ScopedI2CSlaveServiceRegistration registration;

    auto null_result = registration.registerTo(runner, null_driver);
    ASSERT_FALSE(null_result.has_value());
    EXPECT_EQ(null_result.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);

    ASSERT_TRUE(runner.add(service));
    auto duplicate_result = registration.registerTo(runner, driver);
    ASSERT_FALSE(duplicate_result.has_value());
    EXPECT_EQ(duplicate_result.error(), m5::hal::v1::error::error_t::BUSY);
    EXPECT_FALSE(registration.registered());
    EXPECT_EQ(runner.size(), size_t{1});
}

TEST(SoftwareI2CBus, VirtualSlaveServiceReceivesWriteBytes)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    I2CSlaveService slave{lines, 0x42};
    runner.add(slave);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x42;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;

    const uint8_t tx_bytes[] = {0x12, 0x34};
    m5::hal::v1::data::MemorySource tx_src{m5::hal::v1::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    m5::hal::v1::i2c::TransferDesc desc{uint8_t{0xAB}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, &tx_src, nullptr);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{2});  // data phase only (prefix not counted, S16 D4)

    const std::vector<uint8_t> expected{0xAB, 0x12, 0x34};
    EXPECT_EQ(slave.received(), expected);
}

TEST(SoftwareI2CBus, VirtualSlaveServiceSupportsWriteThenReadWithRestart)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    I2CSlaveService slave{lines, 0x42, {0xDE, 0xAD}};
    runner.add(slave);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr    = 0x42;
    acc_cfg.freq        = 100000;
    acc_cfg.timeout_ms  = 100;
    acc_cfg.use_restart = true;

    uint8_t rx_bytes[2] = {};
    m5::hal::v1::data::MemorySink rx_sink{m5::hal::v1::data::DataSpan{rx_bytes, sizeof(rx_bytes)}};

    m5::hal::v1::i2c::TransferDesc desc{uint8_t{0x10}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, nullptr, &rx_sink);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{2});  // data phase only (prefix not counted, S16 D4)

    const std::vector<uint8_t> expected_written{0x10};
    EXPECT_EQ(slave.received(), expected_written);
    EXPECT_EQ(rx_bytes[0], 0xDE);
    EXPECT_EQ(rx_bytes[1], 0xAD);
}

TEST(SoftwareI2CBus, VirtualSlaveServiceSupportsReadOnly)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    I2CSlaveService slave{lines, 0x42, {0xA5, 0x5A}};
    runner.add(slave);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x42;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;

    uint8_t rx_bytes[2] = {};
    m5::hal::v1::data::MemorySink rx_sink{m5::hal::v1::data::DataSpan{rx_bytes, sizeof(rx_bytes)}};

    m5::hal::v1::i2c::TransferDesc desc;
    auto r = bus.transfer(nullptr, acc_cfg, desc, nullptr, &rx_sink);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{2});

    EXPECT_TRUE(slave.received().empty());
    EXPECT_EQ(rx_bytes[0], 0xA5);
    EXPECT_EQ(rx_bytes[1], 0x5A);

    const std::vector<bool> expected_master_acks{true, false};
    EXPECT_EQ(slave.masterAcks(), expected_master_acks);
}

TEST(SoftwareI2CBus, VirtualSlaveServiceSupportsWriteThenReadWithoutRestart)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    I2CSlaveService slave{lines, 0x42, {0xBE, 0xEF}};
    runner.add(slave);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr    = 0x42;
    acc_cfg.freq        = 100000;
    acc_cfg.timeout_ms  = 100;
    acc_cfg.use_restart = false;

    uint8_t rx_bytes[2] = {};
    m5::hal::v1::data::MemorySink rx_sink{m5::hal::v1::data::DataSpan{rx_bytes, sizeof(rx_bytes)}};

    m5::hal::v1::i2c::TransferDesc desc{uint8_t{0x20}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, nullptr, &rx_sink);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{2});  // data phase only (prefix not counted, S16 D4)

    const std::vector<uint8_t> expected_written{0x20};
    EXPECT_EQ(slave.received(), expected_written);
    EXPECT_EQ(rx_bytes[0], 0xBE);
    EXPECT_EQ(rx_bytes[1], 0xEF);
}

TEST(SoftwareI2CBus, VirtualSlaveServiceNacksAddressMismatch)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    I2CSlaveService slave{lines, 0x42};
    runner.add(slave);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x43;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.probe();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::I2C_NO_ACK);
    EXPECT_TRUE(slave.received().empty());
}

TEST(SoftwareI2CBus, VirtualSlaveServicesShareBusByAddress)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    VirtualOpenDrainSlaveLineDriver slave42_lines{lines, 0};
    VirtualOpenDrainSlaveLineDriver slave43_lines{lines, 1};
    I2CSlaveService slave42{slave42_lines, 0x42};
    I2CSlaveService slave43{slave43_lines, 0x43};
    runner.add(slave42);
    runner.add(slave43);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x43;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;

    const uint8_t tx_bytes[] = {0x33, 0x44};
    m5::hal::v1::data::MemorySource tx_src{m5::hal::v1::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    m5::hal::v1::i2c::TransferDesc desc{uint8_t{0x22}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, &tx_src, nullptr);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{2});  // data phase only (prefix not counted, S16 D4)

    EXPECT_TRUE(slave42.received().empty());
    const std::vector<uint8_t> expected43{0x22, 0x33, 0x44};
    EXPECT_EQ(slave43.received(), expected43);
    EXPECT_GE(slave42.stopCount(), size_t{1});
    EXPECT_GE(slave43.stopCount(), size_t{1});
}

TEST(SoftwareI2CBus, VirtualSlaveServiceNacksWriteData)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    I2CSlaveService slave{lines, 0x42};
    slave.setMaxAckedWriteBytes(1);
    runner.add(slave);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x42;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;

    const uint8_t tx_bytes[] = {0x12, 0x34};
    m5::hal::v1::data::MemorySource tx_src{m5::hal::v1::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    m5::hal::v1::i2c::TransferDesc desc{uint8_t{0xAB}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, &tx_src, nullptr);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::I2C_NO_ACK);

    const std::vector<uint8_t> expected_received{0xAB, 0x12};
    EXPECT_EQ(slave.received(), expected_received);
    EXPECT_GE(slave.stopCount(), size_t{1});
}

TEST(SoftwareI2CBus, VirtualSlaveServiceObservesFinalReadNack)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    I2CSlaveService slave{lines, 0x42, {0x11, 0x22}};
    runner.add(slave);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x42;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;

    uint8_t rx_bytes[2] = {};
    m5::hal::v1::data::MemorySink rx_sink{m5::hal::v1::data::DataSpan{rx_bytes, sizeof(rx_bytes)}};

    m5::hal::v1::i2c::TransferDesc desc;
    auto r = bus.transfer(nullptr, acc_cfg, desc, nullptr, &rx_sink);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{2});

    const std::vector<bool> expected_master_acks{true, false};
    EXPECT_EQ(slave.masterAcks(), expected_master_acks);
    EXPECT_EQ(rx_bytes[0], 0x11);
    EXPECT_EQ(rx_bytes[1], 0x22);
}

TEST(SoftwareI2CBus, VirtualOpenDrainBusReportsTimeoutWhenSclHeldLow)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    lines.slavePullSclLow(true);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x42;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 1;
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.probe();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::TIMEOUT_ERROR);
}

TEST(SoftwareI2CBus, VirtualOpenDrainBusReportsBusErrorWhenSdaHeldLowAtStop)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    lines.slavePullSdaLow(true);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    ASSERT_TRUE(bus.init(m5::hal::v1::i2c::I2CBusConfig{gpio.scl(), gpio.sda()}).has_value());

    m5::hal::v1::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = 0x42;
    acc_cfg.freq       = 100000;
    acc_cfg.timeout_ms = 100;
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.probe();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::I2C_BUS_ERROR);
}

// ---------------------------------------------------------------------------
// Behaviour of `Bus::lock` / `unlock`,
// `Accessor::beginAccess` / `endAccess`, and the RAII helpers
// `ScopedAccess` / `ScopedLock`.
// ---------------------------------------------------------------------------

namespace stage2 {

// `StubBus` is a minimal `I2CBus`. `transfer` records the arguments
// so tests can observe them, and writes one byte (`fake_rx_byte`)
// into the rx sink for `readRegister`-style tests. The lock owner
// is exposed through a getter so the test can observe lock / unlock
// directly.
class StubBus : public m5::hal::v1::i2c::I2CBus {
public:
    const m5::hal::v1::i2c::I2CBusConfig& getConfig(void) const override
    {
        return _config;
    }
    m5::hal::v1::result_t<size_t> transfer(m5::hal::v1::bus::Accessor*,
                                           const m5::hal::v1::i2c::I2CMasterAccessConfig& cfg,
                                           const m5::hal::v1::i2c::TransferDesc& desc, m5::hal::v1::data::Source* tx,
                                           m5::hal::v1::data::Sink* rx) override
    {
        ++transfer_count;
        last_cfg        = cfg;
        last_desc       = desc;
        last_tx_was_set = (tx != nullptr);
        last_rx_was_set = (rx != nullptr);
        size_t total    = 0;  // data phase only (S16 D4)
        if (tx) {
            // Drain the tx source and stash the bytes into the observation buffer.
            while (!tx->eof()) {
                auto peeked = tx->peek(SIZE_MAX);
                if (!peeked.has_value() || peeked.value().size == 0) break;
                for (size_t i = 0; i < peeked.value().size && tx_recorded.size() < 32; ++i) {
                    tx_recorded.push_back(peeked.value().data[i]);
                }
                auto adv = tx->advance(peeked.value().size);
                if (!adv.has_value()) break;
                total += peeked.value().size;
            }
        }
        if (rx) {
            auto rsv = rx->reserve(SIZE_MAX);
            if (rsv.has_value() && rsv.value().size > 0) {
                // Fill every byte with `fake_rx_byte`.
                for (size_t i = 0; i < rsv.value().size; ++i) {
                    rsv.value().data[i] = fake_rx_byte;
                }
                (void)rx->commit(rsv.value().size);
                total += rsv.value().size;
            }
        }
        return total;
    }

    const m5::hal::v1::bus::Accessor* lockOwner(void) const
    {
        return _lock_owner;
    }
    size_t transfer_count = 0;
    m5::hal::v1::i2c::I2CMasterAccessConfig last_cfg;
    m5::hal::v1::i2c::TransferDesc last_desc;
    bool last_tx_was_set = false;
    bool last_rx_was_set = false;
    std::vector<uint8_t> tx_recorded;
    uint8_t fake_rx_byte = 0;
};

m5::hal::v1::i2c::I2CMasterAccessConfig makeAcc(uint16_t addr)
{
    m5::hal::v1::i2c::I2CMasterAccessConfig acc;
    acc.i2c_addr   = addr;
    acc.freq       = 100000;
    acc.timeout_ms = 100;
    return acc;
}

TEST(BusLock, LockSetsOwnerUnlockClears)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    EXPECT_EQ(bus.lockOwner(), nullptr);
    auto lk = bus.lock(&accessor);
    EXPECT_TRUE(lk.has_value());
    EXPECT_EQ(bus.lockOwner(), &accessor);

    auto ul = bus.unlock(&accessor);
    EXPECT_TRUE(ul.has_value());
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(BusLock, LockRejectsNullOwner)
{
    StubBus bus;
    auto r = bus.lock(nullptr);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(BusLock, SecondLockReturnsBusy)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor a1{bus, acc_cfg};
    m5::hal::v1::i2c::I2CMasterAccessor a2{bus, acc_cfg};

    ASSERT_TRUE(bus.lock(&a1).has_value());

    auto r = bus.lock(&a2);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::BUSY);
    EXPECT_EQ(bus.lockOwner(), &a1);  // The owner is unchanged.

    (void)bus.unlock(&a1);
}

TEST(BusLock, UnlockByWrongOwnerFails)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor a1{bus, acc_cfg};
    m5::hal::v1::i2c::I2CMasterAccessor a2{bus, acc_cfg};

    ASSERT_TRUE(bus.lock(&a1).has_value());

    auto r = bus.unlock(&a2);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(bus.lockOwner(), &a1);  // The owner is unchanged.

    (void)bus.unlock(&a1);
}

TEST(AccessorAccess, BeginEndManageBusLock)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    EXPECT_FALSE(accessor.inAccess());
    ASSERT_TRUE(accessor.beginAccess().has_value());
    EXPECT_TRUE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), &accessor);

    ASSERT_TRUE(accessor.endAccess().has_value());
    EXPECT_FALSE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(AccessorAccess, NestedBeginEndOnlyLocksOnce)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    ASSERT_TRUE(accessor.beginAccess().has_value());
    ASSERT_TRUE(accessor.beginAccess().has_value());  // Inner call (analogous to a sugar method).
    EXPECT_TRUE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), &accessor);

    // Inner `endAccess` — depth is still > 0, so the bus stays locked.
    ASSERT_TRUE(accessor.endAccess().has_value());
    EXPECT_TRUE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), &accessor);

    // Outer `endAccess` — depth hits zero, the bus unlocks here.
    ASSERT_TRUE(accessor.endAccess().has_value());
    EXPECT_FALSE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(AccessorAccess, EndAccessWithoutBeginFails)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.endAccess();
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(ScopedAccess, AcquiresAndReleasesAtScopeExit)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    {
        m5::hal::v1::bus::ScopedAccess scope{accessor};
        EXPECT_FALSE(scope.has_error());
        EXPECT_EQ(bus.lockOwner(), &accessor);
        EXPECT_TRUE(accessor.inAccess());
    }
    EXPECT_EQ(bus.lockOwner(), nullptr);
    EXPECT_FALSE(accessor.inAccess());
}

TEST(ScopedAccess, FailureLeavesBusUnlocked)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor blocker{bus, acc_cfg};
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    ASSERT_TRUE(bus.lock(&blocker).has_value());

    {
        m5::hal::v1::bus::ScopedAccess scope{accessor};
        EXPECT_TRUE(scope.has_error());
        EXPECT_EQ(scope.error(), m5::hal::v1::error::error_t::BUSY);
        EXPECT_FALSE(accessor.inAccess());
    }
    // The blocker's lock must not be unlocked accidentally by the failed scope's dtor.
    EXPECT_EQ(bus.lockOwner(), &blocker);

    (void)bus.unlock(&blocker);
}

TEST(ScopedLock, AcquiresAndReleasesAtScopeExit)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    {
        m5::hal::v1::bus::ScopedLock scope{bus, &accessor};
        EXPECT_FALSE(scope.has_error());
        EXPECT_EQ(bus.lockOwner(), &accessor);
    }
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(ScopedLock, ContendedAcquireSurfacesBusy)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor a1{bus, acc_cfg};
    m5::hal::v1::i2c::I2CMasterAccessor a2{bus, acc_cfg};

    ASSERT_TRUE(bus.lock(&a1).has_value());

    {
        m5::hal::v1::bus::ScopedLock scope{bus, &a2};
        EXPECT_TRUE(scope.has_error());
        EXPECT_EQ(scope.error(), m5::hal::v1::error::error_t::BUSY);
    }
    EXPECT_EQ(bus.lockOwner(), &a1);

    (void)bus.unlock(&a1);
}

TEST(AccessorSugar, TransferLocksDuringCall)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    EXPECT_EQ(bus.lockOwner(), nullptr);
    auto r = accessor.transfer(m5::hal::v1::i2c::TransferDesc{}, m5::hal::v1::data::ConstDataSpan{},
                               m5::hal::v1::data::DataSpan{});
    EXPECT_TRUE(r.has_value());
    EXPECT_EQ(bus.transfer_count, 1u);
    EXPECT_EQ(bus.lockOwner(), nullptr);  // The sugar method unlocked on return.
}

TEST(AccessorSugar, TransferInsideScopedAccessOnlyLocksOnce)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    {
        m5::hal::v1::bus::ScopedAccess scope{accessor};
        ASSERT_FALSE(scope.has_error());
        EXPECT_EQ(bus.lockOwner(), &accessor);

        // The sugar's internal beginAccess folds into the depth counter.
        auto r1 = accessor.transfer(m5::hal::v1::i2c::TransferDesc{}, m5::hal::v1::data::ConstDataSpan{},
                                    m5::hal::v1::data::DataSpan{});
        auto r2 = accessor.transfer(m5::hal::v1::i2c::TransferDesc{}, m5::hal::v1::data::ConstDataSpan{},
                                    m5::hal::v1::data::DataSpan{});
        EXPECT_TRUE(r1.has_value());
        EXPECT_TRUE(r2.has_value());
        EXPECT_EQ(bus.transfer_count, 2u);
        EXPECT_EQ(bus.lockOwner(), &accessor);  // Stays locked until the scope exits.
    }
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

}  // namespace stage2

// ---------------------------------------------------------------------------
// Behaviour of the `TransferDesc` convenience ctors,
// `writeRegister` / `readRegister`, and the `probe` sugar. We reuse
// `StubBus` from above, so the helpers live in their own namespace
// to avoid name collisions.
// ---------------------------------------------------------------------------

namespace stage3alpha {

using stage2::makeAcc;
using stage2::StubBus;

TEST(TransferDesc, CtorOneByteFromUint8)
{
    // The 1-argument ctor is template + SFINAE. `uint8_t` (sizeof = 1)
    // expands to a 1-byte prefix. Bare-literal construction
    // (`TransferDesc d{0xD0}`) is rejected by SFINAE because the
    // literal type is `int`.
    m5::hal::v1::i2c::TransferDesc d{uint8_t{0xD0}};
    EXPECT_EQ(d.prefix_len, 1);
    EXPECT_EQ(d.prefix[0], 0xD0);
}

TEST(TransferDesc, CtorTwoByteFromUint16IsBigEndian)
{
    // `uint16_t` (sizeof = 2) expands to a 2-byte big-endian (MSB first) prefix.
    m5::hal::v1::i2c::TransferDesc d{uint16_t{0x1234}};
    EXPECT_EQ(d.prefix_len, 2);
    EXPECT_EQ(d.prefix[0], 0x12);
    EXPECT_EQ(d.prefix[1], 0x34);
}

TEST(TransferDesc, CtorFourByteFromUint32IsBigEndian)
{
    // `uint32_t` (sizeof = 4) expands to a 4-byte big-endian (MSB first) prefix.
    m5::hal::v1::i2c::TransferDesc d{uint32_t{0xDEADBEEF}};
    EXPECT_EQ(d.prefix_len, 4);
    EXPECT_EQ(d.prefix[0], 0xDE);
    EXPECT_EQ(d.prefix[1], 0xAD);
    EXPECT_EQ(d.prefix[2], 0xBE);
    EXPECT_EQ(d.prefix[3], 0xEF);
}

TEST(TransferDesc, CtorTwoByteFromExplicitBytes)
{
    // The 2-argument ctor takes two `uint8_t`s for byte-by-byte
    // construction (for example, when the caller wants
    // little-endian on the wire). This is a separate use case from
    // the template 1-argument ctor.
    m5::hal::v1::i2c::TransferDesc d{uint8_t{0xAA}, uint8_t{0xBB}};
    EXPECT_EQ(d.prefix_len, 2);
    EXPECT_EQ(d.prefix[0], 0xAA);
    EXPECT_EQ(d.prefix[1], 0xBB);
}

TEST(TransferDesc, CtorFourByteFromExplicitBytes)
{
    m5::hal::v1::i2c::TransferDesc d{uint8_t{0x01}, uint8_t{0x02}, uint8_t{0x03}, uint8_t{0x04}};
    EXPECT_EQ(d.prefix_len, 4);
    EXPECT_EQ(d.prefix[0], 0x01);
    EXPECT_EQ(d.prefix[3], 0x04);
}

TEST(TransferDesc, DefaultCtorIsEmpty)
{
    m5::hal::v1::i2c::TransferDesc d;
    EXPECT_EQ(d.prefix_len, 0);
}

TEST(AccessorRegister, WriteRegisterSendsPrefixAndValue)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    // Recommended style: declare register addresses as typed
    // constants. `uint8_t` / `uint16_t` spell the wire width out.
    static constexpr uint8_t REG_CTRL = 0xF4;
    static constexpr uint8_t VAL_MODE = 0x27;

    auto r = accessor.writeRegister(REG_CTRL, VAL_MODE);
    EXPECT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 1);
    EXPECT_EQ(bus.last_desc.prefix[0], REG_CTRL);
    EXPECT_TRUE(bus.last_tx_was_set);
    EXPECT_FALSE(bus.last_rx_was_set);
    ASSERT_EQ(bus.tx_recorded.size(), 1u);
    EXPECT_EQ(bus.tx_recorded[0], VAL_MODE);
}

TEST(AccessorRegister, ReadRegisterReturnsSinkContents)
{
    StubBus bus;
    bus.fake_rx_byte = 0x60;  // BME280 chip id
    auto acc_cfg     = makeAcc(0x76);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    static constexpr uint8_t REG_ID = 0xD0;
    auto r                          = accessor.readRegister(REG_ID);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 0x60);
    EXPECT_EQ(bus.last_desc.prefix_len, 1);
    EXPECT_EQ(bus.last_desc.prefix[0], REG_ID);
    EXPECT_FALSE(bus.last_tx_was_set);
    EXPECT_TRUE(bus.last_rx_was_set);
}

TEST(AccessorRegister, ReadRegisterSpanFillsSink)
{
    StubBus bus;
    bus.fake_rx_byte = 0xAB;
    auto acc_cfg     = makeAcc(0x76);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    static constexpr uint8_t REG_DATA = 0xF7;
    uint8_t buf[4]                    = {0, 0, 0, 0};
    auto r                            = accessor.readRegister(REG_DATA, m5::hal::v1::data::DataSpan{buf, sizeof(buf)});
    ASSERT_TRUE(r.has_value());
    for (auto b : buf) {
        EXPECT_EQ(b, 0xAB);
    }
}

TEST(AccessorRegister, LiteralReadRegisterUsesOneByteByDefault)
{
    StubBus bus;
    bus.fake_rx_byte = 0x60;
    auto acc_cfg     = makeAcc(0x76);
    // The default register_address_bytes value is 0, which falls back
    // to the historical 1-byte register address behavior.
    EXPECT_EQ(acc_cfg.register_address_bytes, 0);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.readRegister(0xD0);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 0x60);
    EXPECT_EQ(bus.last_desc.prefix_len, 1);
    EXPECT_EQ(bus.last_desc.prefix[0], 0xD0);
}

TEST(AccessorRegister, LiteralWriteRegisterUsesConfiguredTwoByteAddress)
{
    StubBus bus;
    auto acc_cfg                       = makeAcc(0x50);
    acc_cfg.register_address_bytes     = 2;
    static constexpr uint8_t VAL_WRITE = 0xA5;
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.writeRegister(0x1234, VAL_WRITE);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 2);
    EXPECT_EQ(bus.last_desc.prefix[0], 0x12);
    EXPECT_EQ(bus.last_desc.prefix[1], 0x34);
    ASSERT_EQ(bus.tx_recorded.size(), 1u);
    EXPECT_EQ(bus.tx_recorded[0], VAL_WRITE);
}

TEST(AccessorRegister, LiteralReadRegisterRejectsValueTooLargeForConfiguredWidth)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x76);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.readRegister(0x1234);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

// 2-byte register addresses (think large-capacity devices such as
// EEPROMs). Verifies that the wire convention stays big-endian
// (MSB first).
TEST(AccessorRegister, WriteRegister16IsBigEndian)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x50);  // Imaginary EEPROM target at 0x50.
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    static constexpr uint16_t REG_PAGE = 0x1234;
    static constexpr uint8_t VAL       = 0xAB;

    auto r = accessor.writeRegister(REG_PAGE, VAL);
    EXPECT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 2);
    EXPECT_EQ(bus.last_desc.prefix[0], 0x12);  // MSB first
    EXPECT_EQ(bus.last_desc.prefix[1], 0x34);
    ASSERT_EQ(bus.tx_recorded.size(), 1u);
    EXPECT_EQ(bus.tx_recorded[0], VAL);
}

TEST(AccessorRegister, ReadRegister16IsBigEndian)
{
    StubBus bus;
    bus.fake_rx_byte = 0x5A;
    auto acc_cfg     = makeAcc(0x50);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    static constexpr uint16_t REG_PAGE = 0xABCD;
    auto r                             = accessor.readRegister(REG_PAGE);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 0x5A);
    EXPECT_EQ(bus.last_desc.prefix_len, 2);
    EXPECT_EQ(bus.last_desc.prefix[0], 0xAB);  // MSB first
    EXPECT_EQ(bus.last_desc.prefix[1], 0xCD);
}

TEST(AccessorRegister, ReadRegister16SpanFillsSink)
{
    StubBus bus;
    bus.fake_rx_byte = 0xEE;
    auto acc_cfg     = makeAcc(0x50);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    static constexpr uint16_t REG_PAGE = 0x0100;
    uint8_t buf[3]                     = {};
    auto r                             = accessor.readRegister(REG_PAGE, m5::hal::v1::data::DataSpan{buf, sizeof(buf)});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 2);
    EXPECT_EQ(bus.last_desc.prefix[0], 0x01);
    EXPECT_EQ(bus.last_desc.prefix[1], 0x00);
    for (auto b : buf) {
        EXPECT_EQ(b, 0xEE);
    }
}

TEST(AccessorProbe, ProbeSendsEmptyTransfer)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x76);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.probe();
    EXPECT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 0);
    EXPECT_FALSE(bus.last_tx_was_set);
    EXPECT_FALSE(bus.last_rx_was_set);
    EXPECT_EQ(bus.transfer_count, 1u);
}

}  // namespace stage3alpha

// ---------------------------------------------------------------------------
// Raw `uint8_t* + size_t` overloads. They sit alongside the
// existing span overloads and forward straight to them; the wire
// behaviour is identical because both routes feed the same
// `MemorySource` / `MemorySink`.
// ---------------------------------------------------------------------------

namespace spec_polish_a3 {

using stage2::makeAcc;
using stage2::StubBus;

TEST(AccessorRaw, WriteRawSendsTxBytes)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    auto r                  = accessor.write(payload, sizeof(payload));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 0);
    EXPECT_TRUE(bus.last_tx_was_set);
    EXPECT_FALSE(bus.last_rx_was_set);
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    for (size_t i = 0; i < sizeof(payload); ++i) {
        EXPECT_EQ(bus.tx_recorded[i], payload[i]);
    }
}

TEST(AccessorRaw, ReadRawFillsBuffer)
{
    StubBus bus;
    bus.fake_rx_byte = 0x5A;
    auto acc_cfg     = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    uint8_t buf[5] = {0, 0, 0, 0, 0};
    auto r         = accessor.read(buf, sizeof(buf));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 0);
    EXPECT_FALSE(bus.last_tx_was_set);
    EXPECT_TRUE(bus.last_rx_was_set);
    for (auto b : buf) {
        EXPECT_EQ(b, 0x5A);
    }
}

TEST(AccessorRaw, WriteRegisterRawSendsPrefixAndValues)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x76);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    static constexpr uint8_t REG_DATA = 0xF7;
    const uint8_t payload[]           = {0x10, 0x20, 0x30};
    auto r                            = accessor.writeRegister(REG_DATA, payload, sizeof(payload));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 1);
    EXPECT_EQ(bus.last_desc.prefix[0], REG_DATA);
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    for (size_t i = 0; i < sizeof(payload); ++i) {
        EXPECT_EQ(bus.tx_recorded[i], payload[i]);
    }
}

TEST(AccessorRaw, ReadRegisterRawFillsBuffer)
{
    StubBus bus;
    bus.fake_rx_byte = 0xC3;
    auto acc_cfg     = makeAcc(0x76);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    static constexpr uint8_t REG_DATA = 0xF7;
    uint8_t buf[6]                    = {};
    auto r                            = accessor.readRegister(REG_DATA, buf, sizeof(buf));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 1);
    EXPECT_EQ(bus.last_desc.prefix[0], REG_DATA);
    for (auto b : buf) {
        EXPECT_EQ(b, 0xC3);
    }
}

// Confirm the raw overload preserves the big-endian wire convention for 2-byte register addresses too.
TEST(AccessorRaw, WriteRegister16RawIsBigEndian)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x50);  // Imaginary EEPROM target.
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    static constexpr uint16_t REG_PAGE = 0xABCD;
    const uint8_t payload[]            = {0x11, 0x22};
    auto r                             = accessor.writeRegister(REG_PAGE, payload, sizeof(payload));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 2);
    EXPECT_EQ(bus.last_desc.prefix[0], 0xAB);  // MSB first
    EXPECT_EQ(bus.last_desc.prefix[1], 0xCD);
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    EXPECT_EQ(bus.tx_recorded[0], 0x11);
    EXPECT_EQ(bus.tx_recorded[1], 0x22);
}

TEST(AccessorRaw, ReadRegister16RawIsBigEndian)
{
    StubBus bus;
    bus.fake_rx_byte = 0x77;
    auto acc_cfg     = makeAcc(0x50);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, acc_cfg};

    static constexpr uint16_t REG_PAGE = 0x0100;
    uint8_t buf[3]                     = {};
    auto r                             = accessor.readRegister(REG_PAGE, buf, sizeof(buf));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(bus.last_desc.prefix_len, 2);
    EXPECT_EQ(bus.last_desc.prefix[0], 0x01);
    EXPECT_EQ(bus.last_desc.prefix[1], 0x00);
    for (auto b : buf) {
        EXPECT_EQ(b, 0x77);
    }
}

// Side-by-side check that the raw and span overloads agree on the
// wire (regression guard: both routes must reach the same internal
// transfer target).
TEST(AccessorRaw, RawAndSpanOverloadsAreEquivalentForWriteRegister)
{
    static constexpr uint8_t REG = 0xF4;
    const uint8_t payload[]      = {0x55, 0xAA};

    // raw overload
    StubBus bus_raw;
    {
        auto acc_cfg = makeAcc(0x10);
        m5::hal::v1::i2c::I2CMasterAccessor accessor{bus_raw, acc_cfg};
        ASSERT_TRUE(accessor.writeRegister(REG, payload, sizeof(payload)).has_value());
    }

    // Span overload
    StubBus bus_span;
    {
        auto acc_cfg = makeAcc(0x10);
        m5::hal::v1::i2c::I2CMasterAccessor accessor{bus_span, acc_cfg};
        ASSERT_TRUE(
            accessor.writeRegister(REG, m5::hal::v1::data::ConstDataSpan{payload, sizeof(payload)}).has_value());
    }

    EXPECT_EQ(bus_raw.last_desc.prefix_len, bus_span.last_desc.prefix_len);
    EXPECT_EQ(bus_raw.last_desc.prefix[0], bus_span.last_desc.prefix[0]);
    EXPECT_EQ(bus_raw.tx_recorded, bus_span.tx_recorded);
}

}  // namespace spec_polish_a3

// ---------------------------------------------------------------------------
// `I2CBusConfig` exposes a single gpio_number_t path: `pin_scl` /
// `pin_sda` default to `-1`. Callers fill in gpio_number values via
// the 2-argument ctor or field assignment, and `init()` resolves a
// `Pin` from `m5::hal::v1::M5_Hal.Gpio.getPin(num)` through the
// singleton `GPIOGroup`. Coverage axes:
//   1. Both pins default to -1.
//   2. Calling `init` while pins are -1 returns INVALID_ARGUMENT.
//   3. The 2-argument ctor stores the gpio_numbers into
//      `pin_scl` / `pin_sda`.
//   4. Registering a `RecordingGPIO` into `M5_Hal.Gpio` at a chosen
//      slot makes expander-style targets share the same global
//      gpio_number_t path through the singleton `GPIOGroup`.
// ---------------------------------------------------------------------------

namespace spec_polish_a1 {

TEST(I2CBusConfig, DefaultCtorLeavesPinsInvalid)
{
    m5::hal::v1::i2c::I2CBusConfig cfg;
    EXPECT_LT(cfg.pin_scl, 0);
    EXPECT_LT(cfg.pin_sda, 0);
}

TEST(I2CBusConfig, SoftwareVariantRejectsInvalidPins)
{
    m5::hal::v1::i2c::I2CBusConfig cfg;
    // `pin_scl` / `pin_sda` keep their default value (-1).

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    auto err = bus.init(cfg);
    ASSERT_FALSE(err.has_value());
    EXPECT_EQ(err.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
}

TEST(I2CBusConfig, CtorFromGpioNumbersStoresValuesAndResolvesViaGPIOGroup)
{
    // The 2-argument ctor stores the gpio_numbers into
    // `pin_scl` / `pin_sda`. `init()` calls
    // `m5::hal::v1::M5_Hal.Gpio.getPin(num)`, which resolves to a
    // `stub::Port` Pin in the native build via `M5_Hal.Gpio` ->
    // `stub::GPIO`.
    m5::hal::v1::i2c::I2CBusConfig cfg{/*scl=*/21, /*sda=*/22};
    EXPECT_EQ(cfg.pin_scl, 21);
    EXPECT_EQ(cfg.pin_sda, 22);

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    auto err = bus.init(cfg);
    EXPECT_TRUE(err.has_value());
    (void)bus.release();
}

TEST(I2CBusConfig, SoftwareVariantAcceptsRecordingGPIOViaGPIOGroup)
{
    // The expander-style path (driving SCL / SDA from a Port that
    // lives outside the variant) is reached by registering a
    // `RecordingGPIO` into the singleton `M5_Hal.Gpio` at a chosen
    // slot. Resolution then funnels through the same single
    // gpio_number path — this is the supported escape hatch after
    // Pin value-type direct assignment was retired.
    RecordingPort scl_port;
    RecordingPort sda_port;
    ScopedRecordingGPIO rec{scl_port, sda_port};

    m5::hal::v1::i2c::I2CBusConfig cfg{rec.scl(), rec.sda()};

    m5::variants::frameworks::software::hal::v1::i2c::Bus bus;
    auto err = bus.init(cfg);
    EXPECT_TRUE(err.has_value());
    (void)bus.release();
}

}  // namespace spec_polish_a1

// ---------------------------------------------------------------------------
// `setConfig` (replace an accessor's cfg) and `I2CBus::probe(addr)`
// (the accessor-less probe sugar). Reuses `StubBus` from earlier.
// ---------------------------------------------------------------------------

namespace spec_polish_a2 {

using stage2::makeAcc;
using stage2::StubBus;

TEST(AccessorSetConfig, ReplacesConfigOutsideAccess)
{
    StubBus bus;
    auto cfg1 = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, cfg1};

    auto cfg2 = makeAcc(0x20);
    auto r    = accessor.setConfig(cfg2);
    EXPECT_TRUE(r.has_value());

    // The next transfer must observe the new cfg (`probe` internally
    // follows `accessor.transfer` -> `bus.transfer`).
    (void)accessor.probe();
    EXPECT_EQ(bus.last_cfg.i2c_addr, 0x20);
}

TEST(AccessorSetConfig, RejectsWhenInAccess)
{
    StubBus bus;
    auto cfg1 = makeAcc(0x10);
    m5::hal::v1::i2c::I2CMasterAccessor accessor{bus, cfg1};

    auto ba = accessor.beginAccess();
    ASSERT_TRUE(ba.has_value());

    auto cfg2 = makeAcc(0x20);
    auto r    = accessor.setConfig(cfg2);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);

    // After `endAccess`, the transfer should still see the old cfg
    // (the rejected `setConfig` must leave the config untouched).
    (void)accessor.endAccess();
    (void)accessor.probe();
    EXPECT_EQ(bus.last_cfg.i2c_addr, 0x10);
}

TEST(BusProbe, SendsEmptyTransferWithGivenAddress)
{
    StubBus bus;
    auto r = bus.probe(0x42);
    EXPECT_TRUE(r.has_value());
    EXPECT_EQ(bus.transfer_count, 1u);
    EXPECT_EQ(bus.last_desc.prefix_len, 0);
    EXPECT_FALSE(bus.last_tx_was_set);
    EXPECT_FALSE(bus.last_rx_was_set);
    EXPECT_EQ(bus.last_cfg.i2c_addr, 0x42);
}

TEST(BusProbe, UsesDefaultFreqAndShortTimeout)
{
    StubBus bus;
    (void)bus.probe(0x42);
    // `Bus::probe(addr)`'s scan-oriented defaults: `freq = 100000` Hz,
    // `timeout = 50` ms. The short timeout is deliberately different
    // from `I2CMasterAccessConfig`'s default of 1000 ms — scan loops
    // shouldn't pay one second per NACK.
    EXPECT_EQ(bus.last_cfg.freq, 100000u);
    EXPECT_EQ(bus.last_cfg.timeout_ms, 50u);
}

TEST(BusProbe, AcceptsCustomFreqAndTimeout)
{
    StubBus bus;
    (void)bus.probe(0x42, 400000, 200);
    EXPECT_EQ(bus.last_cfg.freq, 400000u);
    EXPECT_EQ(bus.last_cfg.timeout_ms, 200u);
}

TEST(BusProbe, ReleasesLockAfterCall)
{
    // `probe(addr)` internally builds a stack-local sentinel
    // accessor and walks `beginAccess` -> `transfer` -> `endAccess`.
    // By the time `probe` returns, `bus.lockOwner()` must be
    // `nullptr` (the sentinel's dtor has released the lock); if this
    // breaks, every subsequent caller hits BUSY.
    StubBus bus;
    (void)bus.probe(0x42);
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

}  // namespace spec_polish_a2

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
