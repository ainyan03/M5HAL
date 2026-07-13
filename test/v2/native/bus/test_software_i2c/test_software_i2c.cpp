// SPDX-License-Identifier: MIT
// Scaffolding for the software (bit-bang) I2C bus native gtest.
// `RecordingPort` (a minimal `gpio::IPort` that records caller events
// in order) plus tests that confirm `software::Bus::init` and the
// Source / Sink based transfer path. Protocol-level checks live in
// the latter half of this file and run `SlaveBus_software` over a
// virtual open-drain bus.
//
// Observation hooks live at the `IPort` layer, not at the `Pin`
// layer, because `Pin` is a POD-ish value type that cannot be
// subclassed.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include "i2c_virtual_bus.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>

// tsan's instrumentation slowdown (plus a fully loaded host during a parallel
// full-CI run) can push a long virtual-wire transfer past a 100 ms wire
// timeout even though nothing is wedged (observed on the 200-byte
// continuation read). Give the success-path wire timeout the same kind of
// tsan headroom the per-test watchdog takes (gtest_watchdog.hpp, 30 s ->
// 120 s). Timeout-EXPECTING tests scale with it harmlessly: the timeout
// still fires, just later, and nothing asserts on the elapsed time.
#if defined(__SANITIZE_THREAD__)
#define M5HAL_TEST_WIRE_TIMEOUT_MS 1000
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define M5HAL_TEST_WIRE_TIMEOUT_MS 1000
#else
#define M5HAL_TEST_WIRE_TIMEOUT_MS 100
#endif
#else
#define M5HAL_TEST_WIRE_TIMEOUT_MS 100
#endif

namespace {

// Field-assignment helper: the positional pin ctor is gone (SCL/SDA
// share one integer type, so swapped arguments would compile).
m5::hal::v2::i2c::BusConfig_software makeSoftwareBusConfig(m5::hal::v2::types::gpio_number_t scl,
                                                           m5::hal::v2::types::gpio_number_t sda)
{
    m5::hal::v2::i2c::BusConfig_software cfg;
    cfg.pin_scl = scl;
    cfg.pin_sda = sda;
    return cfg;
}

class ScopedServiceRunnerClear {
public:
    ScopedServiceRunnerClear()
    {
        m5::hal::v2::M5_Hal.Services.clear();
    }
    ~ScopedServiceRunnerClear()
    {
        m5::hal::v2::M5_Hal.Services.clear();
    }
};

class IdleService : public m5::hal::v2::service::IService {
public:
    m5::hal::v2::service::ServicePoll serviceImpl(const m5::hal::v2::service::ServiceContext&) override
    {
        return m5::hal::v2::service::ServiceResult::Idle;
    }
};

std::vector<std::unique_ptr<IdleService>> fillGlobalServiceRunner()
{
    std::vector<std::unique_ptr<IdleService>> services;
    services.reserve(m5::hal::v2::service::ServiceRunner::kMaxServices);
    for (size_t i = 0; i < m5::hal::v2::service::ServiceRunner::kMaxServices; ++i) {
        services.emplace_back(new IdleService());
        EXPECT_TRUE(m5::hal::v2::M5_Hal.Services.add(*services.back())) << i;
    }
    return services;
}

class RecordingPort : public m5::hal::v2::gpio::IPort {
public:
    enum class EventKind : uint8_t { Write, Read, SetMode };
    struct Event {
        m5::hal::v2::types::gpio_number_t gpio_num;
        EventKind kind;
        bool value;                            // Carried by Write / Read events.
        m5::hal::v2::types::gpio_mode_t mode;  // Carried by SetMode events.
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
        _events.push_back({static_cast<m5::hal::v2::types::gpio_number_t>(encoded_num), EventKind::Write, v,
                           m5::hal::v2::types::gpio_mode_t::Input});
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        _events.push_back({static_cast<m5::hal::v2::types::gpio_number_t>(encoded_num), EventKind::Read, _read_value,
                           m5::hal::v2::types::gpio_mode_t::Input});
        return _read_value;
    }
    void _setPinModeEncoded(uint32_t encoded_num, m5::hal::v2::types::gpio_mode_t mode) override
    {
        _events.push_back(
            {static_cast<m5::hal::v2::types::gpio_number_t>(encoded_num), EventKind::SetMode, false, mode});
    }
    m5::hal::v2::types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<m5::hal::v2::types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(m5::hal::v2::types::gpio_local_pin_t pin_index) const override
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
class RecordingGPIO : public m5::hal::v2::gpio::IGPIO {
public:
    RecordingGPIO(RecordingPort& scl, RecordingPort& sda) : _scl(scl), _sda(sda)
    {
    }

    m5::hal::v2::gpio::IPort* portForPin(m5::hal::v2::types::gpio_local_pin_t pin_index) const override
    {
        return (pin_index == 0) ? &_scl : &_sda;
    }
    m5::hal::v2::gpio::IPort* getPort(uint8_t port_index) const override
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
    static constexpr m5::hal::v2::types::gpio_slot_t kSlot = 1;

    ScopedRecordingGPIO(RecordingPort& scl, RecordingPort& sda) : _gpio(scl, sda)
    {
        auto r = m5::hal::v2::M5_Hal.Gpio.addGPIO(&_gpio, kSlot);
        assert(r.has_value() && "ScopedRecordingGPIO: slot 1 add failed (singleton slot pollution?)");
        (void)r;  // Silences the [[nodiscard]] warning when asserts are disabled in release.
    }
    ~ScopedRecordingGPIO()
    {
        auto r = m5::hal::v2::M5_Hal.Gpio.removeGPIO(kSlot);
        assert(r.has_value() && "ScopedRecordingGPIO: slot 1 remove failed (fixture teardown drift)");
        (void)r;
    }

    m5::hal::v2::types::gpio_number_t scl() const
    {
        return m5::hal::v2::types::makeGpioNumber(kSlot, 0);
    }
    m5::hal::v2::types::gpio_number_t sda() const
    {
        return m5::hal::v2::types::makeGpioNumber(kSlot, 1);
    }

private:
    RecordingGPIO _gpio;
};

class FakeMasterLineDriver : public m5::variants::frameworks::software::hal::v2::i2c::detail::MasterLineDriver {
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
    using m5::variants::frameworks::software::hal::v2::i2c::detail::MasterTiming;

    m5::hal::v2::i2c::MasterAccessConfig cfg;

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

TEST(SoftwareI2CMasterTiming, RejectsInvalidFreqAndStoresTimeoutUsec)
{
    using m5::variants::frameworks::software::hal::v2::i2c::detail::MasterTiming;

    m5::hal::v2::i2c::MasterAccessConfig cfg;

    cfg.freq          = 0;
    auto invalid_freq = MasterTiming::fromConfig(cfg);
    ASSERT_FALSE(invalid_freq.has_value());
    EXPECT_EQ(invalid_freq.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    cfg.freq            = 100000;
    cfg.wire_timeout_ms = 5000;
    auto long_timeout   = MasterTiming::fromConfig(cfg);
    ASSERT_TRUE(long_timeout.has_value());
    EXPECT_EQ(long_timeout->timeout, 5000000u);

    cfg.wire_timeout_ms = 0xFFFFFFFFu;
    auto max_timeout    = MasterTiming::fromConfig(cfg);
    ASSERT_TRUE(max_timeout.has_value());
    EXPECT_EQ(max_timeout->timeout, MasterTiming::kMaxUsec);
}

TEST(SoftwareI2CMasterStartCondition, AdvancesByNsecTicks)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 100;

    StartConditionService start;
    start.begin(lines, timing, 1000);

    EXPECT_EQ(start.service(1000), m5::hal::v2::service::ServiceResult::Progress);
    ASSERT_EQ(lines.events.size(), size_t{1});
    EXPECT_EQ(lines.events[0].kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_TRUE(lines.events[0].high);
    EXPECT_EQ(start.dueTick(), 1100u);

    EXPECT_EQ(start.service(1099), m5::hal::v2::service::ServiceResult::Idle);
    EXPECT_EQ(lines.events.size(), size_t{1});

    EXPECT_EQ(start.service(1100), m5::hal::v2::service::ServiceResult::Progress);
    ASSERT_EQ(lines.events.size(), size_t{2});
    EXPECT_EQ(lines.events[1].kind, FakeMasterLineDriver::EventKind::SDA);
    EXPECT_FALSE(lines.events[1].high);
    EXPECT_EQ(start.dueTick(), 1200u);

    EXPECT_EQ(start.service(1200), m5::hal::v2::service::ServiceResult::Done);
    ASSERT_EQ(lines.events.size(), size_t{3});
    EXPECT_EQ(lines.events[2].kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_FALSE(lines.events[2].high);
    EXPECT_TRUE(start.done());
}

TEST(SoftwareI2CMasterStartCondition, WaitsForClockStretchRelease)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 100;
    timing.timeout     = 1000;

    StartConditionService start;
    start.begin(lines, timing, 2000);

    EXPECT_EQ(start.service(2000), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(start.state(), StartConditionService::State::WaitClockHigh);
    ASSERT_EQ(lines.events.size(), size_t{1});
    EXPECT_EQ(lines.events.back().kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_TRUE(lines.events.back().high);

    EXPECT_EQ(start.service(2500), m5::hal::v2::service::ServiceResult::Idle);
    EXPECT_EQ(lines.events.size(), size_t{1});

    lines.scl_read_high = true;
    EXPECT_EQ(start.service(2600), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(start.state(), StartConditionService::State::PullSdaLow);
    EXPECT_EQ(start.dueTick(), 2700u);
}

TEST(SoftwareI2CMasterStartCondition, ReportsTimeoutWhenSclStaysLow)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 30;

    StartConditionService start;
    start.begin(lines, timing, 3000);

    EXPECT_EQ(start.service(3000), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(start.state(), StartConditionService::State::WaitClockHigh);

    EXPECT_EQ(start.service(3029), m5::hal::v2::service::ServiceResult::Idle);
    EXPECT_EQ(start.service(3030), m5::hal::v2::service::ServiceResult::Error);
    EXPECT_EQ(start.state(), StartConditionService::State::Timeout);
    EXPECT_EQ(start.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_FALSE(start.done());
}

TEST(SoftwareI2CMasterWriteByte, SendsBitsMsbFirstAndSamplesAck)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = false;  // slave ACK
    MasterTiming timing;
    timing.half_period = 10;

    WriteByteService writer;
    writer.begin(lines, timing, 0xA5, 1000);

    EXPECT_EQ(writer.service(999), m5::hal::v2::service::ServiceResult::Idle);
    ASSERT_EQ(lines.events.size(), size_t{1});
    EXPECT_EQ(lines.events[0].kind, FakeMasterLineDriver::EventKind::SDA);
    EXPECT_TRUE(lines.events[0].high);

    m5::hal::v2::service::ServiceResult result = m5::hal::v2::service::ServiceResult::Idle;
    for (size_t i = 0; i < 32 && result != m5::hal::v2::service::ServiceResult::Done; ++i) {
        result = writer.service(writer.dueTick());
    }

    EXPECT_EQ(result, m5::hal::v2::service::ServiceResult::Done);
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
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    WriteByteService writer;
    writer.begin(lines, timing, 0x80, 1000);

    EXPECT_EQ(writer.dueTick(), 1010u);
    EXPECT_EQ(writer.service(1013), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::LowerClock);
    EXPECT_EQ(writer.dueTick(), 1020u);

    EXPECT_EQ(writer.service(1024), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::RaiseClock);
    EXPECT_EQ(writer.dueTick(), 1030u);
}

TEST(SoftwareI2CMasterWriteByte, ResyncsClockPhaseWhenServiceRunsTooLate)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    WriteByteService writer;
    writer.begin(lines, timing, 0x80, 1000);

    EXPECT_EQ(writer.service(1055), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::LowerClock);
    EXPECT_EQ(writer.dueTick(), 1065u);
}

TEST(SoftwareI2CMasterWriteByte, ReportsNackAfterReturningClockLow)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = true;  // slave NACK
    MasterTiming timing;
    timing.half_period = 10;

    WriteByteService writer;
    writer.begin(lines, timing, 0x00, 2000);

    m5::hal::v2::service::ServiceResult result = m5::hal::v2::service::ServiceResult::Idle;
    for (size_t i = 0; i < 32 && result != m5::hal::v2::service::ServiceResult::Error; ++i) {
        result = writer.service(writer.dueTick());
    }

    EXPECT_EQ(result, m5::hal::v2::service::ServiceResult::Error);
    EXPECT_FALSE(writer.done());
    EXPECT_FALSE(writer.acked());
    EXPECT_EQ(writer.state(), WriteByteService::State::Nack);
    EXPECT_EQ(writer.error(), m5::hal::v2::error::error_t::I2C_NO_ACK);
    ASSERT_FALSE(lines.events.empty());
    EXPECT_EQ(lines.events.back().kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_FALSE(lines.events.back().high);
}

TEST(SoftwareI2CMasterWriteByte, WaitsForClockStretchRelease)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    lines.sda_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 100;

    WriteByteService writer;
    writer.begin(lines, timing, 0x80, 1000);

    EXPECT_EQ(writer.service(1000), m5::hal::v2::service::ServiceResult::Idle);
    EXPECT_EQ(writer.service(1010), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::WaitClockHigh);
    ASSERT_EQ(lines.events.size(), size_t{2});
    EXPECT_EQ(lines.events.back().kind, FakeMasterLineDriver::EventKind::SCL);
    EXPECT_TRUE(lines.events.back().high);

    EXPECT_EQ(writer.service(1050), m5::hal::v2::service::ServiceResult::Idle);
    EXPECT_EQ(lines.events.size(), size_t{2});

    lines.scl_read_high = true;
    EXPECT_EQ(writer.service(1060), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::LowerClock);
    EXPECT_EQ(writer.dueTick(), 1070u);
}

TEST(SoftwareI2CMasterWriteByte, ReportsTimeoutWhenClockStretchDoesNotRelease)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 30;

    WriteByteService writer;
    writer.begin(lines, timing, 0x80, 2000);

    EXPECT_EQ(writer.service(2000), m5::hal::v2::service::ServiceResult::Idle);
    EXPECT_EQ(writer.service(2010), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(writer.state(), WriteByteService::State::WaitClockHigh);

    EXPECT_EQ(writer.service(2039), m5::hal::v2::service::ServiceResult::Idle);
    EXPECT_EQ(writer.service(2040), m5::hal::v2::service::ServiceResult::Error);
    EXPECT_EQ(writer.state(), WriteByteService::State::Timeout);
    EXPECT_EQ(writer.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_FALSE(writer.done());
}

TEST(SoftwareI2CMasterStopCondition, ReleasesSdaWhileSclHigh)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = true;
    MasterTiming timing;
    timing.half_period = 10;

    StopConditionService stop;
    stop.begin(lines, timing, 1000);

    EXPECT_EQ(stop.service(1000), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(stop.service(1010), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(stop.service(1020), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(stop.service(1030), m5::hal::v2::service::ServiceResult::Done);

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
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;

    StopConditionService stop;
    stop.begin(lines, timing, 2000);

    m5::hal::v2::service::ServiceResult result = m5::hal::v2::service::ServiceResult::Idle;
    for (size_t i = 0; i < 8 && result != m5::hal::v2::service::ServiceResult::Error; ++i) {
        result = stop.service(stop.dueTick());
    }

    EXPECT_EQ(result, m5::hal::v2::service::ServiceResult::Error);
    EXPECT_EQ(stop.state(), StopConditionService::State::BusError);
    EXPECT_EQ(stop.error(), m5::hal::v2::error::error_t::I2C_BUS_ERROR);
}

TEST(SoftwareI2CMasterStopCondition, ReportsTimeoutWhenSclStaysLow)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 30;

    StopConditionService stop;
    stop.begin(lines, timing, 3000);

    EXPECT_EQ(stop.service(3000), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(stop.service(3010), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(stop.state(), StopConditionService::State::WaitClockHigh);

    EXPECT_EQ(stop.service(3039), m5::hal::v2::service::ServiceResult::Idle);
    EXPECT_EQ(stop.service(3040), m5::hal::v2::service::ServiceResult::Error);
    EXPECT_EQ(stop.state(), StopConditionService::State::Timeout);
    EXPECT_EQ(stop.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
}

TEST(SoftwareI2CMasterReadByte, SamplesBitsMsbFirstAndSendsAck)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    ReadByteService reader;
    reader.begin(lines, timing, true, 1000);

    const bool bits[]                          = {true, false, true, false, false, true, false, true};
    m5::hal::v2::service::ServiceResult result = m5::hal::v2::service::ServiceResult::Idle;
    size_t sampled_bits                        = 0;
    for (size_t i = 0; i < 64 && result != m5::hal::v2::service::ServiceResult::Done; ++i) {
        if (reader.state() == ReadByteService::State::SampleBit && sampled_bits < sizeof(bits)) {
            lines.sda_read_high = bits[sampled_bits++];
        }
        result = reader.service(reader.dueTick());
    }

    EXPECT_EQ(result, m5::hal::v2::service::ServiceResult::Done);
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
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = true;
    MasterTiming timing;
    timing.half_period = 10;

    ReadByteService reader;
    reader.begin(lines, timing, false, 2000);

    m5::hal::v2::service::ServiceResult result = m5::hal::v2::service::ServiceResult::Idle;
    for (size_t i = 0; i < 64 && result != m5::hal::v2::service::ServiceResult::Done; ++i) {
        result = reader.service(reader.dueTick());
    }

    EXPECT_EQ(result, m5::hal::v2::service::ServiceResult::Done);
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
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    ReadByteService reader;
    reader.begin(lines, timing, true, 1000);

    EXPECT_EQ(reader.dueTick(), 1000u);
    EXPECT_EQ(reader.service(1003), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::RaiseClock);
    EXPECT_EQ(reader.dueTick(), 1013u);

    EXPECT_EQ(reader.service(1016), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::SampleBit);
    EXPECT_EQ(reader.dueTick(), 1023u);

    EXPECT_EQ(reader.service(1028), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::RaiseClock);
    EXPECT_EQ(reader.dueTick(), 1033u);
}

TEST(SoftwareI2CMasterReadByte, ResyncsClockPhaseWhenServiceRunsTooLate)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period = 10;

    ReadByteService reader;
    reader.begin(lines, timing, true, 1000);

    EXPECT_EQ(reader.service(1000), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::RaiseClock);
    EXPECT_EQ(reader.dueTick(), 1010u);

    EXPECT_EQ(reader.service(1055), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::SampleBit);
    EXPECT_EQ(reader.dueTick(), 1065u);
}

TEST(SoftwareI2CMasterReadByte, ReportsTimeoutWhenClockStretchDoesNotRelease)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.scl_read_high = false;
    MasterTiming timing;
    timing.half_period = 10;
    timing.timeout     = 30;

    ReadByteService reader;
    reader.begin(lines, timing, true, 3000);

    EXPECT_EQ(reader.service(3000), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(reader.service(3010), m5::hal::v2::service::ServiceResult::Progress);
    EXPECT_EQ(reader.state(), ReadByteService::State::WaitClockHigh);
    EXPECT_EQ(reader.service(3039), m5::hal::v2::service::ServiceResult::Idle);
    EXPECT_EQ(reader.service(3040), m5::hal::v2::service::ServiceResult::Error);
    EXPECT_EQ(reader.state(), ReadByteService::State::Timeout);
    EXPECT_EQ(reader.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
}

TEST(SoftwareI2CMasterTransaction, RunsStartWriteReadAndStopPrimitives)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    MasterTiming timing;
    timing.half_period  = 10;
    lines.sda_read_high = false;  // ACK for write byte

    MasterTransactionService transaction;
    transaction.beginStart(lines, timing, 1000);

    auto run_until_done = [&](uint32_t now_tick) {
        m5::hal::v2::service::ServiceResult result = m5::hal::v2::service::ServiceResult::Idle;
        for (size_t i = 0; i < 64 && result != m5::hal::v2::service::ServiceResult::Done; ++i) {
            result = transaction.service(now_tick);
            now_tick += 10;
        }
        return result;
    };

    EXPECT_EQ(run_until_done(1000), m5::hal::v2::service::ServiceResult::Done);
    EXPECT_EQ(transaction.operation(), MasterTransactionService::Operation::Idle);

    transaction.beginWriteByte(lines, timing, 0xA5, 2000);
    EXPECT_EQ(run_until_done(2000), m5::hal::v2::service::ServiceResult::Done);
    EXPECT_TRUE(transaction.acked());

    lines.sda_read_high = true;
    transaction.beginReadByte(lines, timing, false, 3000);
    m5::hal::v2::service::ServiceResult result = m5::hal::v2::service::ServiceResult::Idle;
    for (uint32_t now_tick = 3000; now_tick < 4000 && result != m5::hal::v2::service::ServiceResult::Done;
         now_tick += 10) {
        result = transaction.service(now_tick);
    }
    EXPECT_EQ(result, m5::hal::v2::service::ServiceResult::Done);
    EXPECT_EQ(transaction.byte(), 0xFF);

    lines.sda_read_high = true;
    transaction.beginStop(lines, timing, 4000);
    EXPECT_EQ(run_until_done(4000), m5::hal::v2::service::ServiceResult::Done);
    EXPECT_EQ(transaction.operation(), MasterTransactionService::Operation::Idle);
}

TEST(SoftwareI2CMasterTransaction, PropagatesPrimitiveErrors)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = true;  // NACK for write byte
    MasterTiming timing;
    timing.half_period = 10;

    MasterTransactionService transaction;
    transaction.beginWriteByte(lines, timing, 0x00, 1000);

    m5::hal::v2::service::ServiceResult result = m5::hal::v2::service::ServiceResult::Idle;
    for (size_t i = 0; i < 64 && result != m5::hal::v2::service::ServiceResult::Error; ++i) {
        result = transaction.service(1000 + static_cast<uint32_t>(i * 10));
    }

    EXPECT_EQ(result, m5::hal::v2::service::ServiceResult::Error);
    EXPECT_EQ(transaction.operation(), MasterTransactionService::Operation::Idle);
    EXPECT_EQ(transaction.error(), m5::hal::v2::error::error_t::I2C_NO_ACK);
}

TEST(SoftwareI2CMasterTransaction, RunsAddressAndBufferSequences)
{
    using namespace m5::variants::frameworks::software::hal::v2::i2c::detail;

    FakeMasterLineDriver lines;
    lines.sda_read_high = false;  // ACK for address/data bytes
    MasterTiming timing;
    timing.half_period = 10;

    auto run_until_done = [](MasterTransactionService& transaction, uint32_t now_tick) {
        m5::hal::v2::service::ServiceResult result = m5::hal::v2::service::ServiceResult::Idle;
        for (size_t i = 0; i < 128 && result != m5::hal::v2::service::ServiceResult::Done; ++i) {
            result = transaction.service(now_tick);
            now_tick += 10;
        }
        return result;
    };

    MasterTransactionService transaction;
    transaction.beginAddress(lines, timing, 0x68 << 1, 1000);
    EXPECT_EQ(run_until_done(transaction, 1000), m5::hal::v2::service::ServiceResult::Done);
    EXPECT_EQ(transaction.operation(), MasterTransactionService::Operation::Idle);
    EXPECT_EQ(transaction.transferred(), size_t{0});

    const uint8_t tx[] = {0x12, 0x34, 0x56};
    transaction.beginWriteBuffer(lines, timing, tx, sizeof(tx), 3000);
    EXPECT_EQ(run_until_done(transaction, 3000), m5::hal::v2::service::ServiceResult::Done);
    EXPECT_EQ(transaction.transferred(), sizeof(tx));

    uint8_t rx[2]       = {};
    lines.sda_read_high = true;
    transaction.beginReadBuffer(lines, timing, rx, sizeof(rx), true, 6000);
    EXPECT_EQ(run_until_done(transaction, 6000), m5::hal::v2::service::ServiceResult::Done);
    EXPECT_EQ(transaction.transferred(), sizeof(rx));
    EXPECT_EQ(rx[0], 0xFF);
    EXPECT_EQ(rx[1], 0xFF);
}

TEST(SoftwareIBus, TransferRecordsPinEventsViaPrefix)
{
    RecordingPort scl_port;
    RecordingPort sda_port;
    ScopedRecordingGPIO rec{scl_port, sda_port};

    auto bus_cfg = makeSoftwareBusConfig(rec.scl(), rec.sda());

    m5::hal::v2::i2c::Bus_software bus;
    auto init_result = bus.init(bus_cfg);
    EXPECT_TRUE(init_result.has_value());

    // init() drives both pins to set up the idle state, so events must exist.
    EXPECT_FALSE(scl_port.events().empty());
    EXPECT_FALSE(sda_port.events().empty());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x68;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    // The prefix bytes go directly into the `TransferDesc` inline buffer.
    m5::hal::v2::i2c::TransferDesc desc;
    desc.prefix[0]  = 0xAA;
    desc.prefix_len = 1;

    const size_t baseline_scl = scl_port.events().size();
    const size_t baseline_sda = sda_port.events().size();

    // We don't care whether transfer succeeds — the simulated slave (a
    // fixed-value RecordingPin) cannot acknowledge correctly. All this
    // skeleton asserts is that transfer drove SCL/SDA, i.e. the bit-bang
    // state machine actually ran. Protocol-level checks below use
    // SlaveBus_software on VirtualOpenDrainBus.
    //
    // `owner` is reserved for future lock semantics and accepts nullptr here.
    (void)bus.transfer(nullptr, acc_cfg, desc, nullptr, 0, nullptr, 0);
    EXPECT_GT(scl_port.events().size(), baseline_scl);
    EXPECT_GT(sda_port.events().size(), baseline_sda);
}

TEST(SoftwareIBus, TransferRejectsInvalidFrequency)
{
    RecordingPort scl_port;
    RecordingPort sda_port;
    ScopedRecordingGPIO rec{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(rec.scl(), rec.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr = 0x68;
    acc_cfg.freq     = 0;

    auto r = bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{}, nullptr, 0, nullptr, 0);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(SoftwareIBus, TransferRecordsPinEventsViaSource)
{
    RecordingPort scl_port;
    RecordingPort sda_port;
    ScopedRecordingGPIO rec{scl_port, sda_port};

    auto bus_cfg = makeSoftwareBusConfig(rec.scl(), rec.sda());

    m5::hal::v2::i2c::Bus_software bus;
    auto init_result = bus.init(bus_cfg);
    EXPECT_TRUE(init_result.has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x68;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    // Push tx bytes through a MemorySource to exercise the Source path.
    const uint8_t tx_bytes[] = {0xBB, 0xCC};
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    const size_t baseline_scl = scl_port.events().size();
    const size_t baseline_sda = sda_port.events().size();

    m5::hal::v2::i2c::TransferDesc desc;  // No prefix.
    (void)bus.transfer(nullptr, acc_cfg, desc, &tx_src, SIZE_MAX, nullptr, 0);
    EXPECT_GT(scl_port.events().size(), baseline_scl);
    EXPECT_GT(sda_port.events().size(), baseline_sda);
}

// `probe` (an all-empty transfer) must travel through the variant's
// "send address+W + check ACK" path and actually drive SCL / SDA
// waveforms (an earlier implementation just returned success without
// doing anything). This recording-pin test only confirms that
// waveforms appear on the wire. ACK and read / write semantics are
// pinned down later by the SlaveBusSoftware + VirtualOpenDrainBus tests.
TEST(SoftwareIBus, ProbeProducesWireActivity)
{
    RecordingPort scl_port;
    RecordingPort sda_port;
    ScopedRecordingGPIO rec{scl_port, sda_port};

    auto bus_cfg = makeSoftwareBusConfig(rec.scl(), rec.sda());

    m5::hal::v2::i2c::Bus_software bus;
    auto init_result = bus.init(bus_cfg);
    EXPECT_TRUE(init_result.has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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

TEST(SoftwareIBus, VirtualSlaveBusSoftwareAcksProbe)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};
    auto r = accessor.probe();
    EXPECT_TRUE(r.has_value());
}

TEST(SoftwareIBus, AccessorTransferRunsInServiceRunnerUntilEndTransaction)
{
    using namespace service_proto;

    ScopedServiceRunnerClear clear_services;
    VirtualOpenDrainBus lines;
    ServiceRunner slave_runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {}, 0, true};
    slave_runner.add(slave.service());
    slave_runner.add(responder);
    lines.setRunner(&slave_runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    const uint8_t tx[] = {0x10, 0x11, 0x12, 0x13};
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    auto started = accessor.transfer(m5::hal::v2::i2c::TransferDesc{}, m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)},
                                     m5::hal::v2::data::DataSpan{});
    ASSERT_TRUE(started.has_value());
    EXPECT_TRUE(accessor.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 1u);

    for (size_t i = 0; i < 400 && m5::hal::v2::M5_Hal.Services.size() != 0; ++i) {
        lines.advance(1000);
        (void)m5::hal::v2::M5_Hal.Services.runOnce(
            m5::hal::v2::service::ServiceContext{1000, m5::hal::v2::service::fastTick()});
    }
    EXPECT_FALSE(accessor.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 0u);

    auto totals = accessor.endTransaction();
    ASSERT_TRUE(totals.has_value());
    EXPECT_EQ(totals->tx, sizeof(tx));
    EXPECT_EQ(totals->rx, size_t{0});
    EXPECT_EQ(responder.received(), std::vector<uint8_t>({0x10, 0x11, 0x12, 0x13}));
}

TEST(SoftwareIBus, AccessorTransferFailsWhenServiceRunnerIsFull)
{
    using namespace service_proto;

    ScopedServiceRunnerClear clear_services;
    auto fillers = fillGlobalServiceRunner();
    VirtualOpenDrainBus lines;
    ServiceRunner slave_runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {}, 0, true};
    slave_runner.add(slave.service());
    slave_runner.add(responder);
    lines.setRunner(&slave_runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    const uint8_t tx[] = {0x10, 0x11, 0x12, 0x13};
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    auto started = accessor.transfer(m5::hal::v2::i2c::TransferDesc{}, m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)},
                                     m5::hal::v2::data::DataSpan{});
    ASSERT_FALSE(started.has_value());
    EXPECT_EQ(started.error(), m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_FALSE(accessor.transferBusy());

    // The failed segment poisons the transaction (unified latch contract,
    // spec/design/i2c.md §transaction 中のエラー).
    auto ended = accessor.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), m5::hal::v2::service::ServiceRunner::kMaxServices);
    (void)fillers;
}

TEST(SoftwareIBus, NextAccessorTransferDrainsPreviousBeforeStarting)
{
    using namespace service_proto;

    ScopedServiceRunnerClear clear_services;
    VirtualOpenDrainBus lines;
    ServiceRunner slave_runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {}, 0, true};
    slave_runner.add(slave.service());
    slave_runner.add(responder);
    lines.setRunner(&slave_runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    const uint8_t first[]  = {0x20, 0x21};
    const uint8_t second[] = {0x30};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    ASSERT_TRUE(accessor
                    .transfer(m5::hal::v2::i2c::TransferDesc{}, m5::hal::v2::data::ConstDataSpan{first, sizeof(first)},
                              m5::hal::v2::data::DataSpan{})
                    .has_value());
    EXPECT_TRUE(accessor.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 1u);

    ASSERT_TRUE(accessor
                    .transfer(m5::hal::v2::i2c::TransferDesc{},
                              m5::hal::v2::data::ConstDataSpan{second, sizeof(second)}, m5::hal::v2::data::DataSpan{})
                    .has_value());
    EXPECT_TRUE(accessor.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 1u);

    auto totals = accessor.endTransaction();
    ASSERT_TRUE(totals.has_value());
    EXPECT_EQ(totals->tx, sizeof(first) + sizeof(second));
    EXPECT_EQ(totals->rx, size_t{0});
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 0u);
    EXPECT_EQ(responder.received(), std::vector<uint8_t>({0x20, 0x21, 0x30}));
}

TEST(SlaveBusSoftware, IdleBeforeInit)
{
    m5::hal::v2::i2c::SlaveBus_software slave;
    auto& svc = static_cast<m5::hal::v2::service::IService&>(slave);
    EXPECT_EQ(m5::hal::v2::service::ServiceRunner::run(svc, m5::hal::v2::service::ServiceContext{0, 0}),
              m5::hal::v2::service::ServiceResult::Idle);
}

TEST(SlaveBusSoftware, Rejects10BitAddressUntilImplemented)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    m5::hal::v2::i2c::SlaveBus_software slave;
    m5::hal::v2::i2c::SlaveBusConfig cfg;
    cfg.address          = 0x120;
    cfg.address_is_10bit = true;

    auto r = slave.init(lines, cfg);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(SlaveBusSoftware, MaxAckedWriteBytesSurvivesInit)
{
    // Configuration set before init() must survive it: resetProtocol()
    // resets protocol state, not configuration.
    service_proto::VirtualOpenDrainBus lines;
    m5::hal::v2::i2c::SlaveBus_software slave;
    slave.setMaxAckedWriteBytes(3);

    m5::hal::v2::i2c::SlaveBusConfig cfg;
    cfg.address = 0x42;
    ASSERT_TRUE(slave.init(lines, cfg).has_value());
    EXPECT_EQ(slave.maxAckedWriteBytes(), 3u);
}

TEST(SlaveBusSoftwareRegistration, RegistersAndRemovesBusService)
{
    class CountingService : public m5::hal::v2::service::IService {
    public:
        m5::hal::v2::service::ServicePoll serviceImpl(const m5::hal::v2::service::ServiceContext&) override
        {
            ++calls;
            return m5::hal::v2::service::ServiceResult::Progress;
        }
        size_t calls = 0;
    };
    class StubSlaveBus : public m5::hal::v2::i2c::ISlaveBus {
    public:
        explicit StubSlaveBus(m5::hal::v2::service::IService* svc) : svc{svc}
        {
        }
        m5::hal::v2::result_t<void> init(const m5::hal::v2::i2c::SlaveBusConfig&) override
        {
            return {};
        }
        m5::hal::v2::result_t<void> release() override
        {
            return {};
        }
        m5::hal::v2::service::IService* service() override
        {
            return svc;
        }
        m5::hal::v2::result_t<void> beginTransaction(m5::hal::v2::bus::IAccessor*, uint32_t) override
        {
            return {};
        }
        m5::hal::v2::result_t<void> endTransaction(m5::hal::v2::bus::IAccessor*) override
        {
            return {};
        }
        m5::hal::v2::result_t<size_t> read(m5::hal::v2::bus::IAccessor*, m5::hal::v2::data::DataSpan) override
        {
            return size_t{0};
        }
        m5::hal::v2::result_t<size_t> write(m5::hal::v2::bus::IAccessor*, m5::hal::v2::data::ConstDataSpan) override
        {
            return size_t{0};
        }
        m5::hal::v2::result_t<size_t> readableBytes(m5::hal::v2::bus::IAccessor*) override
        {
            return size_t{0};
        }
        m5::hal::v2::result_t<bool> transactionComplete(m5::hal::v2::bus::IAccessor*) override
        {
            return true;
        }
        m5::hal::v2::service::IService* svc = nullptr;
    };

    CountingService service;
    StubSlaveBus driver{&service};
    m5::hal::v2::service::ServiceRunner runner;

    {
        m5::hal::v2::i2c::ScopedSlaveServiceRegistration registration;
        auto r = registration.registerTo(runner, driver);
        ASSERT_TRUE(r.has_value());
        EXPECT_TRUE(registration.registered());
        EXPECT_EQ(runner.size(), size_t{1});
        EXPECT_TRUE(runner.runOnce(m5::hal::v2::service::ServiceContext{0, 0}));
        EXPECT_EQ(service.calls, size_t{1});
    }

    EXPECT_EQ(runner.size(), size_t{0});
    EXPECT_FALSE(runner.runOnce(m5::hal::v2::service::ServiceContext{0, 0}));
    EXPECT_EQ(service.calls, size_t{1});
}

TEST(SlaveBusSoftwareRegistration, RejectsNullOrDuplicateService)
{
    class IdleService : public m5::hal::v2::service::IService {
    public:
        m5::hal::v2::service::ServicePoll serviceImpl(const m5::hal::v2::service::ServiceContext&) override
        {
            return m5::hal::v2::service::ServiceResult::Idle;
        }
    };
    class StubSlaveBus : public m5::hal::v2::i2c::ISlaveBus {
    public:
        explicit StubSlaveBus(m5::hal::v2::service::IService* svc) : svc{svc}
        {
        }
        m5::hal::v2::result_t<void> init(const m5::hal::v2::i2c::SlaveBusConfig&) override
        {
            return {};
        }
        m5::hal::v2::result_t<void> release() override
        {
            return {};
        }
        m5::hal::v2::service::IService* service() override
        {
            return svc;
        }
        m5::hal::v2::result_t<void> beginTransaction(m5::hal::v2::bus::IAccessor*, uint32_t) override
        {
            return {};
        }
        m5::hal::v2::result_t<void> endTransaction(m5::hal::v2::bus::IAccessor*) override
        {
            return {};
        }
        m5::hal::v2::result_t<size_t> read(m5::hal::v2::bus::IAccessor*, m5::hal::v2::data::DataSpan) override
        {
            return size_t{0};
        }
        m5::hal::v2::result_t<size_t> write(m5::hal::v2::bus::IAccessor*, m5::hal::v2::data::ConstDataSpan) override
        {
            return size_t{0};
        }
        m5::hal::v2::result_t<size_t> readableBytes(m5::hal::v2::bus::IAccessor*) override
        {
            return size_t{0};
        }
        m5::hal::v2::result_t<bool> transactionComplete(m5::hal::v2::bus::IAccessor*) override
        {
            return true;
        }
        m5::hal::v2::service::IService* svc = nullptr;
    };

    IdleService service;
    StubSlaveBus null_driver{nullptr};
    StubSlaveBus driver{&service};
    m5::hal::v2::service::ServiceRunner runner;
    m5::hal::v2::i2c::ScopedSlaveServiceRegistration registration;

    auto null_result = registration.registerTo(runner, null_driver);
    ASSERT_FALSE(null_result.has_value());
    EXPECT_EQ(null_result.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    ASSERT_TRUE(runner.add(service));
    auto duplicate_result = registration.registerTo(runner, driver);
    ASSERT_FALSE(duplicate_result.has_value());
    EXPECT_EQ(duplicate_result.error(), m5::hal::v2::error::error_t::BUSY);
    EXPECT_FALSE(registration.registered());
    EXPECT_EQ(runner.size(), size_t{1});
}

TEST(SoftwareIBus, VirtualSlaveBusSoftwareReceivesWriteBytes)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    const uint8_t tx_bytes[] = {0x12, 0x34};
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    m5::hal::v2::i2c::TransferDesc desc{uint8_t{0xAB}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, &tx_src, SIZE_MAX, nullptr, 0);
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(tx_src.eof());

    const std::vector<uint8_t> expected{0xAB, 0x12, 0x34};
    EXPECT_EQ(slave.received(), expected);
}

TEST(SoftwareIBus, VirtualSlaveBusSoftwareSupportsWriteThenReadWithRestart)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {0xDE, 0xAD}, 1};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;

    uint8_t rx_bytes[2] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx_bytes, sizeof(rx_bytes)}};

    m5::hal::v2::i2c::TransferDesc desc{uint8_t{0x10}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, nullptr, 0, &rx_sink, SIZE_MAX);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(rx_sink.written(), size_t{2});

    const std::vector<uint8_t> expected_written{0x10};
    EXPECT_EQ(responder.received(), expected_written);
    EXPECT_EQ(rx_bytes[0], 0xDE);
    EXPECT_EQ(rx_bytes[1], 0xAD);
}

TEST(SoftwareIBus, VirtualSlaveBusSoftwareSupportsReadOnly)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {0xA5, 0x5A}};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    uint8_t rx_bytes[2] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx_bytes, sizeof(rx_bytes)}};

    m5::hal::v2::i2c::TransferDesc desc;
    auto r = bus.transfer(nullptr, acc_cfg, desc, nullptr, 0, &rx_sink, SIZE_MAX);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(rx_sink.written(), size_t{2});

    EXPECT_TRUE(responder.received().empty());
    EXPECT_EQ(rx_bytes[0], 0xA5);
    EXPECT_EQ(rx_bytes[1], 0x5A);

    const std::vector<bool> expected_master_acks{true, false};
    EXPECT_EQ(slave.masterAcks(), expected_master_acks);
}

TEST(SoftwareIBus, VirtualSlaveBusSoftwareSupportsWriteThenReadWithoutRestart)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {0xBE, 0xEF}, 0, true};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = false;

    uint8_t rx_bytes[2] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx_bytes, sizeof(rx_bytes)}};

    m5::hal::v2::i2c::TransferDesc desc{uint8_t{0x20}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, nullptr, 0, &rx_sink, SIZE_MAX);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(rx_sink.written(), size_t{2});

    const std::vector<uint8_t> expected_written{0x20};
    EXPECT_EQ(responder.received(), expected_written);
    EXPECT_EQ(rx_bytes[0], 0xBE);
    EXPECT_EQ(rx_bytes[1], 0xEF);
}

TEST(SoftwareIBus, VirtualSlaveBusSoftwareNacksAddressMismatch)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x43;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.probe();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::I2C_NO_ACK);
    EXPECT_TRUE(slave.received().empty());
}

TEST(SoftwareIBus, VirtualSlaveBusesShareBusByAddress)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    VirtualOpenDrainSlaveLineDriver slave42_lines{lines, 0};
    VirtualOpenDrainSlaveLineDriver slave43_lines{lines, 1};
    SlaveEndpoint slave42{slave42_lines, 0x42};
    SlaveEndpoint slave43{slave43_lines, 0x43};
    runner.add(slave42.service());
    runner.add(slave43.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x43;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    const uint8_t tx_bytes[] = {0x33, 0x44};
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    m5::hal::v2::i2c::TransferDesc desc{uint8_t{0x22}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, &tx_src, SIZE_MAX, nullptr, 0);
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(tx_src.eof());

    EXPECT_TRUE(slave42.received().empty());
    const std::vector<uint8_t> expected43{0x22, 0x33, 0x44};
    EXPECT_EQ(slave43.received(), expected43);
    EXPECT_GE(slave42.stopCount(), size_t{1});
    EXPECT_GE(slave43.stopCount(), size_t{1});
}

TEST(SoftwareIBus, VirtualSlaveBusSoftwareNacksWriteData)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    slave.setMaxAckedWriteBytes(1);
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    const uint8_t tx_bytes[] = {0x12, 0x34};
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    m5::hal::v2::i2c::TransferDesc desc{uint8_t{0xAB}};
    auto r = bus.transfer(nullptr, acc_cfg, desc, &tx_src, SIZE_MAX, nullptr, 0);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::I2C_NO_ACK);

    const std::vector<uint8_t> expected_received{0xAB, 0x12};
    EXPECT_EQ(slave.received(), expected_received);
    EXPECT_GE(slave.stopCount(), size_t{1});
}

TEST(SoftwareIBus, VirtualSlaveBusSoftwareObservesFinalReadNack)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {0x11, 0x22}};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    uint8_t rx_bytes[2] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx_bytes, sizeof(rx_bytes)}};

    m5::hal::v2::i2c::TransferDesc desc;
    auto r = bus.transfer(nullptr, acc_cfg, desc, nullptr, 0, &rx_sink, SIZE_MAX);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(rx_sink.written(), size_t{2});

    const std::vector<bool> expected_master_acks{true, false};
    EXPECT_EQ(slave.masterAcks(), expected_master_acks);
    EXPECT_EQ(rx_bytes[0], 0x11);
    EXPECT_EQ(rx_bytes[1], 0x22);
}

TEST(SlaveStreamAccessorWindow, CompletedMasterTransferOpensReadableWindow)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    const uint8_t tx_bytes[] = {0x12, 0x34};
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0xAB}}, &tx_src, SIZE_MAX, nullptr, 0)
            .has_value());

    ASSERT_TRUE(slave.accessor().beginTransaction(0).has_value());
    uint8_t rx[3] = {};
    auto read     = slave.accessor().read(m5::hal::v2::data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(read.has_value());
    EXPECT_EQ(read.value(), sizeof(rx));
    EXPECT_EQ(rx[0], 0xAB);
    EXPECT_EQ(rx[1], 0x12);
    EXPECT_EQ(rx[2], 0x34);
    EXPECT_TRUE(slave.accessor().endTransaction().has_value());
}

TEST(SlaveStreamAccessorWindow, ConsecutiveMasterTransfersOpenSeparateWindows)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    ASSERT_TRUE(bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x10}}, nullptr, 0, nullptr, 0)
                    .has_value());
    ASSERT_TRUE(bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x20}}, nullptr, 0, nullptr, 0)
                    .has_value());

    ASSERT_TRUE(slave.accessor().beginTransaction(0).has_value());
    uint8_t first[2] = {};
    auto first_read  = slave.accessor().read(m5::hal::v2::data::DataSpan{first, sizeof(first)});
    ASSERT_TRUE(first_read.has_value());
    EXPECT_EQ(first_read.value(), size_t{1});
    EXPECT_EQ(first[0], 0x10);
    EXPECT_EQ(first[1], 0x00);
    EXPECT_TRUE(slave.accessor().endTransaction().has_value());

    ASSERT_TRUE(slave.accessor().beginTransaction(0).has_value());
    uint8_t second[1] = {};
    auto second_read  = slave.accessor().read(m5::hal::v2::data::DataSpan{second, sizeof(second)});
    ASSERT_TRUE(second_read.has_value());
    EXPECT_EQ(second_read.value(), size_t{1});
    EXPECT_EQ(second[0], 0x20);
    EXPECT_TRUE(slave.accessor().endTransaction().has_value());
}

TEST(SlaveStreamAccessorWindow, TxQueuedInWindowExpiresAtStop)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {0x01, 0x02, 0x03}};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    uint8_t rx_first[2] = {};
    m5::hal::v2::data::MemorySink first_sink{m5::hal::v2::data::DataSpan{rx_first, sizeof(rx_first)}};
    ASSERT_TRUE(bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{}, nullptr, 0, &first_sink, SIZE_MAX)
                    .has_value());
    EXPECT_EQ(rx_first[0], 0x01);
    EXPECT_EQ(rx_first[1], 0x02);

    uint8_t rx_second[2] = {};
    m5::hal::v2::data::MemorySink second_sink{m5::hal::v2::data::DataSpan{rx_second, sizeof(rx_second)}};
    ASSERT_TRUE(bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{}, nullptr, 0, &second_sink, SIZE_MAX)
                    .has_value());
    EXPECT_EQ(rx_second[0], 0xFF);
    EXPECT_EQ(rx_second[1], 0xFF);
}

TEST(SlaveStreamAccessorWindow, UnderrunReturnsConfiguredFillByte)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    uint8_t rx[2] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{}, nullptr, 0, &rx_sink, SIZE_MAX).has_value());
    EXPECT_EQ(rx[0], 0xFF);
    EXPECT_EQ(rx[1], 0xFF);
}

TEST(SlaveStreamAccessorWindow, WriteThenReadIsHandledInsideOneWindow)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {0xCA, 0xFE}, 1};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;

    uint8_t rx[2] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x7A}}, nullptr, 0, &rx_sink, SIZE_MAX)
            .has_value());

    const std::vector<uint8_t> expected_written{0x7A};
    EXPECT_EQ(responder.received(), expected_written);
    EXPECT_EQ(rx[0], 0xCA);
    EXPECT_EQ(rx[1], 0xFE);
}

// The native software-I2C master drives its bit-bang timing and whole-transfer
// deadline from wall-clock (fastTick() = micros() off-target). A long transfer
// (hundreds-to-1024 bytes at 100 kHz) is tens of ms on the wire, so under heavy
// parallel CI load OS scheduling delay can push it past a tight 100 ms deadline
// and trip a spurious TIMEOUT mid-stream -- the historical "high-load native
// software-I2C flaky" failure. These streaming / ring tests assert a CORRECTNESS
// property (byte counts round-trip, no overflow), not timing, so they use a
// generous deadline; the short tests keep the tight 100 ms timeout. Keep this
// value unchanged: it is the verified long-stream margin, independent of the
// timeout representation used by MasterTiming.
constexpr uint32_t kStreamWireTimeoutMs = 2000;

TEST(SlaveStreamAccessorWindow, RxOverflowSurfacesTruncatedLongWrite)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = kStreamWireTimeoutMs;

    EXPECT_EQ(slave.bus().rxOverflowCount(), size_t{0});

    // No responder is registered, so read() is never called and the RX ring is
    // never drained. A single transaction of a register byte + 80 data bytes
    // therefore fills the ring's unread backlog to its depth (kRxCapacity = 64);
    // the remaining bytes would overwrite the oldest unread one, so they are
    // dropped and surfaced via rxOverflowCount(). (With a draining consumer the
    // same long write lands in full -- see LongWriteBeyondRxWindowFillsRegisterFileViaRing.)
    uint8_t payload[80];
    for (size_t i = 0; i < sizeof(payload); ++i) {
        payload[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};
    (void)bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x00}}, &tx_src, SIZE_MAX, nullptr, 0);

    EXPECT_GT(slave.bus().rxOverflowCount(), size_t{0});
}

TEST(SlaveStreamAccessorWindow, LongMasterWriteRoundTripsThroughStreamRing)
{
    using namespace service_proto;

    // Foundational ring check WITHOUT the RegMap layer: the master streams a
    // 256-byte sequence in a single transaction and the pure stream responder,
    // draining read() every tick, must accumulate the EXACT same bytes. This
    // isolates the RX ring + stream accessor (occupancy guard + wrap-aware copy)
    // from RegMap pointer / compose logic -- if it passes, the base layer carries
    // far more than kRxCapacity (64) bytes per transaction losslessly as long as
    // the consumer drains, which is the entire point of the ring.
    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {}};  // empty reply: pure RX capture
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = kStreamWireTimeoutMs;

    // 256 distinct-ish bytes (value != index so a misordered/wrapped store is
    // caught positionally). Sent as: first byte via TransferDesc, the rest via the
    // source -- the slave receives all 256 as one raw write stream.
    std::vector<uint8_t> payload(256);
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i * 7 + 1);
    }
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{payload.data() + 1, payload.size() - 1}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{payload[0]}, &tx_src, SIZE_MAX, nullptr, 0)
            .has_value());

    EXPECT_EQ(responder.received(), payload);
    EXPECT_EQ(slave.bus().rxOverflowCount(), size_t{0});
}

TEST(SlaveStreamAccessorWindow, VeryLongMasterWriteHasNoPerTransactionLimit)
{
    using namespace service_proto;

    // The RX ring imposes NO per-transaction byte limit: it bounds only the unread
    // backlog (rx_size - rx_read) while rx_size/rx_read are monotonic size_t, so the
    // 64-byte backing is reused circularly without bound. This streams 1024 bytes
    // (16x kRxCapacity, well past the 256-byte 8-bit-pointer coincidence) through the
    // pure stream accessor and requires every byte back, lossless, with the consumer
    // draining each tick. Lengthen kStreamLen to stress further -- the only real
    // limit is the consumer keeping up (a lagging one surfaces via rxOverflowCount).
    constexpr size_t kStreamLen = 1024;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {}};  // empty reply: pure RX capture
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = kStreamWireTimeoutMs;

    std::vector<uint8_t> payload(kStreamLen);
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i * 31 + 7);  // value != index, period > 256
    }
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{payload.data() + 1, payload.size() - 1}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{payload[0]}, &tx_src, SIZE_MAX, nullptr, 0)
            .has_value());

    EXPECT_EQ(responder.received().size(), kStreamLen);
    EXPECT_EQ(responder.received(), payload);
    EXPECT_EQ(slave.bus().rxOverflowCount(), size_t{0});
}

TEST(SlaveStreamAccessorWindow, LongMasterReadStreamsReplyThroughTxRing)
{
    using namespace service_proto;

    // TX counterpart of the RX round-trip: the master reads 256 bytes in one
    // transaction and the responder STREAMS a 256-byte reply, writing more as the
    // master consumes (the tx ring frees up). With tx_underrun=stretch the master is
    // held whenever the ring momentarily empties, so a reply far larger than the
    // 64-byte tx ring is delivered intact. Proves the TX ring + read-serve streaming
    // contract carry >kTxCapacity per transaction -- the read-direction counterpart
    // of removing the per-transaction write cap.
    constexpr size_t kReplyLen = 256;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42, m5::hal::v2::i2c::TxUnderrun::Stretch};

    std::vector<uint8_t> reply(kReplyLen);
    for (size_t i = 0; i < reply.size(); ++i) {
        reply[i] = static_cast<uint8_t>(i * 11 + 3);  // value != index, period > 256
    }
    SlaveStreamReplyResponder responder{slave, reply};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = kStreamWireTimeoutMs;

    std::vector<uint8_t> rx(kReplyLen);
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx.data(), rx.size()}};
    m5::hal::v2::i2c::TransferDesc desc;  // pure read, no register prefix
    auto r = bus.transfer(nullptr, acc_cfg, desc, nullptr, 0, &rx_sink, SIZE_MAX);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(rx_sink.written(), kReplyLen);
    EXPECT_EQ(rx, reply);
}

// U10: waitForActivity returns result_t<bool> so a custom serve loop can tell an
// activity wake (true) from a timeout (false). The poll fallback (base default, no
// ISR) cannot observe a wake reason and always reports false; an ISR backend reports
// its semaphore-take result. Both paths are modeled here with stub backends.
TEST(SlaveStreamAccessorWaitForActivity, ReportsActivityVersusTimeout)
{
    using m5::hal::v2::result_t;
    namespace v2 = m5::hal::v2;

    // Minimal ISlaveBus that leaves waitForActivity on the base 1 ms poll (false).
    class PollStubBus : public v2::i2c::ISlaveBus {
    public:
        result_t<void> init(const v2::i2c::SlaveBusConfig&) override
        {
            return {};
        }
        result_t<void> release() override
        {
            return {};
        }
        v2::service::IService* service() override
        {
            return nullptr;
        }
        result_t<void> beginTransaction(v2::bus::IAccessor*, uint32_t) override
        {
            return {};
        }
        result_t<void> endTransaction(v2::bus::IAccessor*) override
        {
            return {};
        }
        result_t<size_t> read(v2::bus::IAccessor*, v2::data::DataSpan) override
        {
            return size_t{0};
        }
        result_t<size_t> write(v2::bus::IAccessor*, v2::data::ConstDataSpan) override
        {
            return size_t{0};
        }
        result_t<size_t> readableBytes(v2::bus::IAccessor*) override
        {
            return size_t{0};
        }
        result_t<bool> transactionComplete(v2::bus::IAccessor*) override
        {
            return false;
        }
    };
    // An ISR-style backend that confirms an activity wake and records the timeout.
    class ActivityStubBus : public PollStubBus {
    public:
        result_t<bool> waitForActivity(v2::bus::IAccessor*, uint32_t timeout_ms) override
        {
            last_timeout = timeout_ms;
            return true;
        }
        uint32_t last_timeout = 0xFFFFFFFFu;
    };

    ActivityStubBus active;
    v2::i2c::SlaveStreamAccessor active_acc{active};
    auto woke = active_acc.waitForActivity(50);
    ASSERT_TRUE(woke.has_value());
    EXPECT_TRUE(woke.value());            // activity confirmed
    EXPECT_EQ(active.last_timeout, 50u);  // timeout forwarded to the backend

    PollStubBus poll;
    v2::i2c::SlaveStreamAccessor poll_acc{poll};
    auto timed = poll_acc.waitForActivity(0);
    ASSERT_TRUE(timed.has_value());
    EXPECT_FALSE(timed.value());  // poll fallback never confirms a wake
}

// ===========================================================================
// MasterAccessor Source/Sink overloads
//
// Behavioral tests for write(Source&, len), read(Sink&, len) and the
// Source/Sink register overloads.  All tests run through the full
// virtual-bus stack (VirtualOpenDrainBus + SlaveBus_software) so the
// assertions cover actual on-wire behavior, not just stub dispatch.
// ===========================================================================

TEST(SoftwareI2CMasterSourceSink, WriteSourceSendsBytes)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    const uint8_t tx_bytes[] = {0x12, 0x34};
    m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    auto r = accessor.write(src, sizeof(tx_bytes));
    ASSERT_TRUE(r.has_value());  // result_t<size_t>: this test only needs success.

    const std::vector<uint8_t> expected{0x12, 0x34};
    EXPECT_EQ(slave.received(), expected);
}

TEST(SoftwareI2CMasterSourceSink, ReadSinkReceivesBytes)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {0xDE, 0xAD}};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    uint8_t rx_buf[2] = {};
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx_buf, sizeof(rx_buf)}};

    auto r = accessor.read(sink, sizeof(rx_buf));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{2});
    EXPECT_EQ(rx_buf[0], 0xDE);
    EXPECT_EQ(rx_buf[1], 0xAD);
}

TEST(SoftwareI2CMasterSourceSink, WriteRegisterTypedSourceSendsPrefix)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    static constexpr uint8_t REG = 0xF4;
    const uint8_t payload[]      = {0xAA, 0xBB};
    m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};

    auto r = accessor.writeRegister(REG, src, sizeof(payload));
    ASSERT_TRUE(r.has_value());  // result_t<size_t>: this test only needs success.

    // Wire carries: register prefix then payload bytes.
    const std::vector<uint8_t> expected{REG, 0xAA, 0xBB};
    EXPECT_EQ(slave.received(), expected);
}

TEST(SoftwareI2CMasterSourceSink, WriteRegisterLiteralSourceSendsPrefix)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    const uint8_t payload[] = {0xCC, 0xDD};
    m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};

    // Signed-literal overload: address width from register_address_bytes (default 0 → 1 byte).
    auto r = accessor.writeRegister(0xA0, src, sizeof(payload));
    ASSERT_TRUE(r.has_value());

    const std::vector<uint8_t> expected{0xA0, 0xCC, 0xDD};
    EXPECT_EQ(slave.received(), expected);
}

TEST(SoftwareI2CMasterSourceSink, ReadRegisterTypedSinkReceivesData)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    // min_rx_bytes_before_write=1: wait for the register prefix byte before replying.
    SlaveTransactionResponder responder{slave, {0x5A, 0x3C}, 1};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    static constexpr uint8_t REG = 0x10;
    uint8_t rx_buf[2]            = {};
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx_buf, sizeof(rx_buf)}};

    auto r = accessor.readRegister(REG, sink, sizeof(rx_buf));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{2});

    // Responder observes the register address byte in the write phase.
    const std::vector<uint8_t> expected_written{REG};
    EXPECT_EQ(responder.received(), expected_written);
    EXPECT_EQ(rx_buf[0], 0x5A);
    EXPECT_EQ(rx_buf[1], 0x3C);
}

TEST(SoftwareI2CMasterSourceSink, ReadRegisterLiteralSinkReceivesData)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    SlaveTransactionResponder responder{slave, {0xBE, 0xEF}, 1};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    uint8_t rx_buf[2] = {};
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx_buf, sizeof(rx_buf)}};

    // Signed-literal overload: address width from register_address_bytes (default 0 → 1 byte).
    auto r = accessor.readRegister(0x20, sink, sizeof(rx_buf));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{2});

    const std::vector<uint8_t> expected_written{0x20};
    EXPECT_EQ(responder.received(), expected_written);
    EXPECT_EQ(rx_buf[0], 0xBE);
    EXPECT_EQ(rx_buf[1], 0xEF);
}

TEST(SoftwareI2CMasterSourceSink, WriteCapsBytesAtLen)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    // Source has 8 bytes; the len cap must limit the wire to exactly 4.
    const uint8_t tx_bytes[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)}};

    auto r = accessor.write(src, 4);
    ASSERT_TRUE(r.has_value());

    // Only the first 4 bytes reach the wire; the remaining 4 are not sent.
    const std::vector<uint8_t> expected{0x00, 0x01, 0x02, 0x03};
    EXPECT_EQ(slave.received(), expected);
}

TEST(SoftwareI2CMasterSourceSink, ReadCapsBytesAtLen)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    // Responder offers 8 bytes; the master's capped Sink must accept only 4.
    SlaveTransactionResponder responder{slave, {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7}};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    // Sink has 8-byte backing; the len cap must stop the master after 4 bytes.
    uint8_t rx_buf[8] = {};
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx_buf, sizeof(rx_buf)}};

    auto r = accessor.read(sink, 4);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, size_t{4});

    // Exactly 4 bytes committed into the backing buffer.
    EXPECT_EQ(sink.written(), size_t{4});
    EXPECT_EQ(rx_buf[0], 0xA0);
    EXPECT_EQ(rx_buf[1], 0xA1);
    EXPECT_EQ(rx_buf[2], 0xA2);
    EXPECT_EQ(rx_buf[3], 0xA3);
    // Bytes 4-7 must remain zero (not written by the capped read).
    EXPECT_EQ(rx_buf[4], 0x00);
    EXPECT_EQ(rx_buf[5], 0x00);
    EXPECT_EQ(rx_buf[6], 0x00);
    EXPECT_EQ(rx_buf[7], 0x00);
}

TEST(SoftwareI2CMasterSourceSink, ReadWithShortReserveSinkReceivesAllBytesAcrossChunks)
{
    // Regression for a two-part chunking bug in TransferState::beginReadChunk:
    // (1) it used to overwrite `_rx_remaining` with the (possibly short)
    // reserved span size, so the "more to read" loop in advanceAfterDone
    // could never trigger; (2) `last_nack` was hardcoded true, so the
    // master NACKed at the end of every chunk, not just the final one.
    // A MemorySink always reserves the full remainder contiguously and
    // never exercises either bug. A RingFIFO sink does: reserve() only
    // returns the contiguous run up to its backing buffer's physical end,
    // which is shorter than the request whenever the write cursor sits
    // mid-buffer -- exactly the "short reserve" Sink shape called out in
    // the task spec.
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    const std::vector<uint8_t> tx_bytes{0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
    SlaveTransactionResponder responder{slave, tx_bytes};
    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    // 8-byte backing buffer for a RingFIFO sink. Pre-position the write
    // cursor (`head`) at offset 5 by committing and then fully draining 5
    // bytes, so the buffer holds no data but its *physical* layout has
    // only 3 contiguous bytes left before wrapping to the start.
    uint8_t ring_buf[8] = {};
    m5::hal::v2::data::RingFIFO ring{ring_buf, sizeof(ring_buf)};
    {
        auto primed = ring.sink().reserve(5);
        ASSERT_TRUE(primed.has_value());
        ASSERT_EQ(primed.value().size, size_t{5});
        ASSERT_TRUE(ring.sink().commit(5).has_value());
        ASSERT_TRUE(ring.source().advance(5).has_value());
    }
    ASSERT_EQ(ring.buffered(), size_t{0});
    ASSERT_EQ(ring.free(), size_t{8});

    // Request 6 bytes: the first reserve() call can only return the 3
    // contiguous bytes remaining before the ring wraps (a "short
    // reserve"), forcing TransferState into a second chunk.
    auto r = accessor.read(ring.sink(), tx_bytes.size());
    ASSERT_TRUE(r.has_value()) << "err=" << m5::hal::v2::error::toString(r.error());
    EXPECT_EQ(*r, tx_bytes.size());

    // Drain the ring and confirm every requested byte arrived, in order,
    // across the (necessarily two) chunks.
    std::vector<uint8_t> received;
    for (;;) {
        auto peeked = ring.source().peek(64);
        ASSERT_TRUE(peeked.has_value());
        if (peeked.value().size == 0) {
            break;
        }
        received.insert(received.end(), peeked.value().data, peeked.value().data + peeked.value().size);
        ASSERT_TRUE(ring.source().advance(peeked.value().size).has_value());
    }
    EXPECT_EQ(received, tx_bytes);
}

// ===========================================================================
// SlaveRegMapAccessor
//
// Two layers of coverage, matching the execution-model split:
//   * SlaveRegMapAccessorLogic -- the pure register-map step API (beginExchange
//     / ingest / composeReply / get/setRegister / hooks) exercised directly,
//     with no bus. Fully deterministic; this is where pointer / auto-increment
//     / wrap / hook semantics are pinned.
//   * SlaveRegMapAccessorWire -- the register map serviced through the real
//     SlaveBus_software backend by a software master, driven by a tick loop
//     (serve()'s blocking poll cannot be used single-threaded). Proves the
//     compose / ingest steps drive a real transaction correctly. SPLIT's pure
//     read and the full clock matrix are covered on hardware (HIL).
// ===========================================================================

namespace {

uint8_t regMapOnReadInvert(uint8_t reg, void* ctx)
{
    (void)ctx;
    return static_cast<uint8_t>(~reg);
}

struct OnWriteLog {
    uint8_t regs[16] = {};
    uint8_t vals[16] = {};
    size_t count     = 0;
};

void regMapOnWriteLog(uint8_t reg, uint8_t val, void* ctx)
{
    auto* log = static_cast<OnWriteLog*>(ctx);
    if (log->count < 16) {
        log->regs[log->count] = reg;
        log->vals[log->count] = val;
        ++log->count;
    }
}

struct ReadCounter {
    uint8_t next = 0;
};

struct ComposeCallCounter {
    size_t calls = 0;
};

uint8_t regMapOnReadCountCalls(uint8_t reg, void* ctx)
{
    auto* c = static_cast<ComposeCallCounter*>(ctx);
    ++c->calls;
    return reg;
}

uint8_t regMapOnReadCounter(uint8_t reg, void* ctx)
{
    (void)reg;
    auto* c = static_cast<ReadCounter*>(ctx);
    return c->next++;
}

// Tick-driven driver for SlaveRegMapAccessor over the software backend. Mirrors
// SlaveTransactionResponder, but composes the reply from the register map. The
// reply is queued once `min_rx_before_compose` bytes have arrived: write-then-
// read wants 1 (compose from the just-set pointer), a pure read wants 0 (the
// pointer persisted from a previous transaction).
class RegMapTickResponder : public m5::hal::v2::service::IService {
public:
    RegMapTickResponder(m5::hal::v2::i2c::SlaveRegMapAccessor& regmap, size_t min_rx_before_compose = 0,
                        bool repeat = true)
        : _regmap(regmap), _min_rx_before_compose(min_rx_before_compose), _repeat(repeat)
    {
    }

    m5::hal::v2::service::ServicePoll serviceImpl(const m5::hal::v2::service::ServiceContext&) override
    {
        auto& acc = _regmap.stream();
        if (_ended && !_repeat) {
            return m5::hal::v2::service::ServiceResult::Idle;
        }
        if (!_opened) {
            if (!acc.beginTransaction(0).has_value()) {
                return m5::hal::v2::service::ServiceResult::Idle;
            }
            _regmap.beginExchange();
            _opened           = true;
            _composed         = false;
            _pointer_ingested = false;
            _ended            = false;
            _tx_have          = 0;
            _tx_sent          = 0;
            _resp_off         = 0;
        }

        uint8_t buffer[64] = {};
        size_t n           = 0;
        auto readable      = acc.readableBytes();
        if (readable.has_value() && readable.value() > 0) {
            auto read = acc.read(m5::hal::v2::data::DataSpan{buffer, sizeof(buffer)});
            if (read.has_value()) {
                n = read.value();
            }
        }

        if (!_composed) {
            // Mirror serve()'s ordering: ingest ONLY the first byte (the register
            // pointer) before composing, so the reply window reflects the
            // pre-write register state; data bytes are applied AFTER the reply is
            // queued. (The current responder composes within the same tick it
            // ingests the pointer, so the data is held only momentarily.)
            size_t off = 0;
            if (!_pointer_ingested && n >= 1) {
                _regmap.ingest(m5::hal::v2::data::ConstDataSpan{buffer, 1});
                _pointer_ingested = true;
                off               = 1;
            }
            const size_t seen = _pointer_ingested ? 1 : 0;  // a pure read sees 0
            if (seen >= _min_rx_before_compose) {
                pumpReply(acc);
                _composed = true;
            }
            // Apply data bytes that arrived alongside the pointer, AFTER compose.
            if (_composed && n > off) {
                _regmap.ingest(m5::hal::v2::data::ConstDataSpan{buffer + off, n - off});
            }
        } else if (n > 0) {
            // Post-compose: every further byte is write data; apply incrementally.
            _regmap.ingest(m5::hal::v2::data::ConstDataSpan{buffer, n});
        }
        auto complete = acc.transactionComplete();
        if (complete.has_value() && complete.value()) {
            (void)acc.endTransaction();
            _ended = true;
            if (_repeat) {
                _opened = false;
                return m5::hal::v2::service::ServiceResult::Progress;
            }
            return m5::hal::v2::service::ServiceResult::Done;
        }
        if (_composed) {
            // Keep the reply streaming past the first chunk (mirrors serve()'s
            // pump): the ring frees as the master clocks bytes out; the next
            // chunk is composed from the advancing offset (8-bit wrap).
            // Ordered AFTER the completion check, like serve(): pumping a
            // just-completed transaction would see the queue the STOP freed
            // and compose extra chunks that no one will read.
            pumpReply(acc);
        }
        return m5::hal::v2::service::ServiceResult::Progress;
    }

private:
    // Compose-and-push loop shared by the first reply and its continuation:
    // compose a chunk only once the previous one was fully accepted, retry the
    // unaccepted tail verbatim (never re-compose -> onRead fires once per
    // streamed byte), stand down while the ring is full.
    void pumpReply(m5::hal::v2::i2c::SlaveStreamAccessor& acc)
    {
        for (;;) {
            if (_tx_sent == _tx_have) {
                _tx_have = _regmap.composeReply(m5::hal::v2::data::DataSpan{_tx, sizeof(_tx)}, _resp_off);
                _tx_sent = 0;
                _resp_off += _tx_have;
            }
            auto write = acc.write(m5::hal::v2::data::ConstDataSpan{_tx + _tx_sent, _tx_have - _tx_sent});
            if (!write.has_value()) {
                return;
            }
            _tx_sent += write.value();
            if (_tx_sent < _tx_have) {
                return;  // ring full for now; retry on a later tick
            }
        }
    }

    m5::hal::v2::i2c::SlaveRegMapAccessor& _regmap;
    size_t _min_rx_before_compose = 0;
    bool _repeat                  = false;
    bool _opened                  = false;
    bool _composed                = false;
    bool _pointer_ingested        = false;
    bool _ended                   = false;
    uint8_t _tx[64]               = {};
    size_t _tx_have               = 0;
    size_t _tx_sent               = 0;
    size_t _resp_off              = 0;
};

}  // namespace

TEST(SlaveRegMapAccessorLogic, FirstByteOfExchangeSetsPointer)
{
    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveBus_software bus;  // never initialised: the step API touches no bus.
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    const uint8_t ptr_frame[] = {0x10};
    rm.beginExchange();
    rm.ingest(m5::hal::v2::data::ConstDataSpan{ptr_frame, sizeof(ptr_frame)});
    EXPECT_EQ(rm.pointer(), 0x10);
}

TEST(SlaveRegMapAccessorLogic, WriteBytesAutoIncrementFromPointer)
{
    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveBus_software bus;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    const uint8_t frame[] = {0x10, 0xAA, 0xBB, 0xCC};
    rm.beginExchange();
    rm.ingest(m5::hal::v2::data::ConstDataSpan{frame, sizeof(frame)});

    EXPECT_EQ(rm.pointer(), 0x10);
    EXPECT_EQ(rm.getRegister(0x10), 0xAA);
    EXPECT_EQ(rm.getRegister(0x11), 0xBB);
    EXPECT_EQ(rm.getRegister(0x12), 0xCC);
    EXPECT_EQ(rm.getRegister(0x13), 0x00);
}

TEST(SlaveRegMapAccessorLogic, ReadWindowAutoIncrementsFromPointer)
{
    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveBus_software bus;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    const uint8_t ptr_frame[] = {0x10};
    rm.beginExchange();
    rm.ingest(m5::hal::v2::data::ConstDataSpan{ptr_frame, sizeof(ptr_frame)});

    uint8_t tx[4] = {};
    EXPECT_EQ(rm.composeReply(m5::hal::v2::data::DataSpan{tx, sizeof(tx)}), size_t{4});
    EXPECT_EQ(tx[0], 0x10);
    EXPECT_EQ(tx[1], 0x11);
    EXPECT_EQ(tx[2], 0x12);
    EXPECT_EQ(tx[3], 0x13);
}

TEST(SlaveRegMapAccessorLogic, PointerPersistsAcrossExchanges)
{
    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveBus_software bus;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    // Exchange 1: a pointer-only write (SPLIT's first transaction).
    const uint8_t ptr_frame[] = {0x20};
    rm.beginExchange();
    rm.ingest(m5::hal::v2::data::ConstDataSpan{ptr_frame, sizeof(ptr_frame)});

    // Exchange 2: a pure read (no bytes received) resolves against the pointer.
    rm.beginExchange();
    EXPECT_EQ(rm.pointer(), 0x20);
    uint8_t tx[3] = {};
    rm.composeReply(m5::hal::v2::data::DataSpan{tx, sizeof(tx)});
    EXPECT_EQ(tx[0], 0x20);
    EXPECT_EQ(tx[1], 0x21);
    EXPECT_EQ(tx[2], 0x22);
}

TEST(SlaveRegMapAccessorLogic, ReadWindowWrapsAtEightBits)
{
    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveBus_software bus;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    const uint8_t ptr_frame[] = {0xFE};
    rm.beginExchange();
    rm.ingest(m5::hal::v2::data::ConstDataSpan{ptr_frame, sizeof(ptr_frame)});

    uint8_t tx[4] = {};
    rm.composeReply(m5::hal::v2::data::DataSpan{tx, sizeof(tx)});
    EXPECT_EQ(tx[0], 0xFE);
    EXPECT_EQ(tx[1], 0xFF);
    EXPECT_EQ(tx[2], 0x00);
    EXPECT_EQ(tx[3], 0x01);
}

TEST(SlaveRegMapAccessorLogic, ComposeReplyOffsetContinuesWindowWithoutMovingPointer)
{
    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveBus_software bus;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    const uint8_t ptr_frame[] = {0xF0};
    rm.beginExchange();
    rm.ingest(m5::hal::v2::data::ConstDataSpan{ptr_frame, sizeof(ptr_frame)});

    // Chunked continuation: offset picks up exactly where the previous chunk
    // ended, wrapping at 8 bits (0xF0 + 16 -> 0x00).
    uint8_t a[8] = {}, b[8] = {}, c[8] = {};
    EXPECT_EQ(rm.composeReply(m5::hal::v2::data::DataSpan{a, sizeof(a)}), size_t{8});
    EXPECT_EQ(rm.composeReply(m5::hal::v2::data::DataSpan{b, sizeof(b)}, 8), size_t{8});
    EXPECT_EQ(rm.composeReply(m5::hal::v2::data::DataSpan{c, sizeof(c)}, 16), size_t{8});
    EXPECT_EQ(a[0], 0xF0);
    EXPECT_EQ(a[7], 0xF7);
    EXPECT_EQ(b[0], 0xF8);
    EXPECT_EQ(b[7], 0xFF);
    EXPECT_EQ(c[0], 0x00);  // continuation wrapped past 0xFF
    EXPECT_EQ(c[7], 0x07);

    // Offset composes never advance the pointer: a fresh offset-0 compose
    // re-serves the same base (repeat-read semantics).
    EXPECT_EQ(rm.pointer(), 0xF0);
    uint8_t again[2] = {};
    rm.composeReply(m5::hal::v2::data::DataSpan{again, sizeof(again)});
    EXPECT_EQ(again[0], 0xF0);
}

TEST(SlaveRegMapAccessorLogic, GetSetRegisterTouchBackingStore)
{
    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveBus_software bus;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    rm.setRegister(0x05, 0x99);
    EXPECT_EQ(rm.getRegister(0x05), 0x99);
    EXPECT_EQ(reg_file[0x05], 0x99);
}

TEST(SlaveRegMapAccessorLogic, OnReadOverridesWindowWithoutTouchingStore)
{
    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveBus_software bus;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    rm.setOnRead(&regMapOnReadInvert, nullptr);

    const uint8_t ptr_frame[] = {0x10};
    rm.beginExchange();
    rm.ingest(m5::hal::v2::data::ConstDataSpan{ptr_frame, sizeof(ptr_frame)});

    uint8_t tx[2] = {};
    rm.composeReply(m5::hal::v2::data::DataSpan{tx, sizeof(tx)});
    EXPECT_EQ(tx[0], static_cast<uint8_t>(~0x10));
    EXPECT_EQ(tx[1], static_cast<uint8_t>(~0x11));
    // The backing store is untouched: onRead is a live view, not a cache.
    EXPECT_EQ(rm.getRegister(0x10), 0x10);
}

TEST(SlaveRegMapAccessorLogic, OnWriteFiresWithStoredBytes)
{
    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveBus_software bus;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    OnWriteLog log;
    rm.setOnWrite(&regMapOnWriteLog, &log);

    const uint8_t frame[] = {0x30, 0x01, 0x02};
    rm.beginExchange();
    rm.ingest(m5::hal::v2::data::ConstDataSpan{frame, sizeof(frame)});

    ASSERT_EQ(log.count, size_t{2});
    EXPECT_EQ(log.regs[0], 0x30);
    EXPECT_EQ(log.vals[0], 0x01);
    EXPECT_EQ(log.regs[1], 0x31);
    EXPECT_EQ(log.vals[1], 0x02);
    // The pointer byte (0x30) is not a write and must not fire onWrite.
    EXPECT_EQ(rm.getRegister(0x30), 0x01);
    EXPECT_EQ(rm.getRegister(0x31), 0x02);
}

TEST(SlaveRegMapAccessorLogic, OutOfRangeRegisterFileIsGuarded)
{
    uint8_t reg_file[4] = {0, 1, 2, 3};
    m5::hal::v2::i2c::SlaveBus_software bus;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    // Pointer past the backing store: writes are dropped, reads return 0, no UB.
    const uint8_t frame[] = {0x80, 0xAB};
    rm.beginExchange();
    rm.ingest(m5::hal::v2::data::ConstDataSpan{frame, sizeof(frame)});
    EXPECT_EQ(rm.getRegister(0x80), 0x00);

    uint8_t tx[2] = {0x55, 0x55};
    rm.composeReply(m5::hal::v2::data::DataSpan{tx, sizeof(tx)});
    EXPECT_EQ(tx[0], 0x00);
    EXPECT_EQ(tx[1], 0x00);
}

TEST(SlaveRegMapAccessorWire, WriteThenReadReturnsRegisterWindow)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};

    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveRegMapAccessor regmap{slave.bus(), m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    RegMapTickResponder responder{regmap, /*min_rx_before_compose=*/1};

    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;

    uint8_t rx[4] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x10}}, nullptr, 0, &rx_sink, SIZE_MAX)
            .has_value());

    EXPECT_EQ(rx[0], 0x10);
    EXPECT_EQ(rx[1], 0x11);
    EXPECT_EQ(rx[2], 0x12);
    EXPECT_EQ(rx[3], 0x13);
}

TEST(SlaveRegMapAccessorWire, SplitWriteThenSeparateReadUsesPersistedPointer)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};

    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveRegMapAccessor regmap{slave.bus(), m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    // min_rx=0: composing early in the write-only first transaction is harmless
    // (the master never reads it) and the second transaction is a pure read.
    RegMapTickResponder responder{regmap, /*min_rx_before_compose=*/0};

    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    // Transaction 1: write the register pointer only, STOP.
    const uint8_t ptr_byte[] = {0x40};
    m5::hal::v2::data::MemorySource ptr_src{m5::hal::v2::data::ConstDataSpan{ptr_byte, sizeof(ptr_byte)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{}, &ptr_src, SIZE_MAX, nullptr, 0).has_value());

    // Transaction 2: a pure read resolves against the pointer set above.
    uint8_t rx[3] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{}, nullptr, 0, &rx_sink, SIZE_MAX).has_value());

    EXPECT_EQ(rx[0], 0x40);
    EXPECT_EQ(rx[1], 0x41);
    EXPECT_EQ(rx[2], 0x42);
}

TEST(SlaveRegMapAccessorWire, MultiByteWriteStoresIntoRegisterFile)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};

    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveRegMapAccessor regmap{slave.bus(), m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    RegMapTickResponder responder{regmap, /*min_rx_before_compose=*/0};

    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    // Write [0x50, 0xDE, 0xAD, 0xBE]: pointer 0x50, then three data bytes.
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE};
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x50}}, &tx_src, SIZE_MAX, nullptr, 0)
            .has_value());

    EXPECT_EQ(reg_file[0x50], 0xDE);
    EXPECT_EQ(reg_file[0x51], 0xAD);
    EXPECT_EQ(reg_file[0x52], 0xBE);
}

TEST(SlaveRegMapAccessorWire, LongWriteBeyondRxWindowFillsRegisterFileViaRing)
{
    using namespace service_proto;

    // A single transaction writing far more than the backend RX ring depth
    // (kRxCapacity = 64): pointer 0x00 + 200 data bytes. Because the tick
    // responder drains read() on every pass, the ring's UNREAD backlog never
    // reaches its depth, so every byte lands in the register file and nothing
    // overflows -- the per-transaction 64B cap is gone. The drain also crosses
    // the ring boundary repeatedly (at 64/128/192), exercising the wrap-aware
    // read copy; any wrap bug would misplace the bytes around those offsets.
    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};

    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveRegMapAccessor regmap{slave.bus(), m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    RegMapTickResponder responder{regmap, /*min_rx_before_compose=*/0};

    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = kStreamWireTimeoutMs;

    // Value differs from the register index (i + 1) so a "stored at the wrong
    // offset" wrap bug is caught positionally, not masked by value == index.
    uint8_t payload[200];
    for (size_t i = 0; i < sizeof(payload); ++i) {
        payload[i] = static_cast<uint8_t>(i + 1);
    }
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x00}}, &tx_src, SIZE_MAX, nullptr, 0)
            .has_value());

    for (size_t i = 0; i < sizeof(payload); ++i) {
        EXPECT_EQ(reg_file[i], payload[i]) << "register file mismatch at offset " << i;
    }
    EXPECT_EQ(slave.bus().rxOverflowCount(), size_t{0});
}

TEST(SlaveRegMapAccessorWire, OnReadSuppliesLiveValuesOverTheWire)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};

    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveRegMapAccessor regmap{slave.bus(), m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    ReadCounter counter;
    regmap.setOnRead(&regMapOnReadCounter, &counter);
    RegMapTickResponder responder{regmap, /*min_rx_before_compose=*/1};

    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;

    // onRead ignores the register and returns an incrementing counter, so the
    // window is a live sequence rather than the (zeroed) backing store.
    uint8_t rx[3] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x00}}, nullptr, 0, &rx_sink, SIZE_MAX)
            .has_value());

    EXPECT_EQ(rx[0], 0x00);
    EXPECT_EQ(rx[1], 0x01);
    EXPECT_EQ(rx[2], 0x02);
}

TEST(SlaveRegMapAccessorWire, SameTransactionWriteThenReadSeesPreWriteValue)
{
    using namespace service_proto;

    // The "write data + read-back in ONE transaction" pattern: the master writes
    // [reg, data] then repeated-START reads. serve() composes the reply from the
    // register pointer BEFORE applying the data byte, so the read returns the
    // PRE-write value while the store still lands. This pins the ordering and
    // guards the tick responder against regressing to "ingest all, then compose"
    // (which would leak the post-write value into the read).
    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};

    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveRegMapAccessor regmap{slave.bus(), m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    RegMapTickResponder responder{regmap, /*min_rx_before_compose=*/1};

    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;

    // Write [0x10, 0xAA] then repeated-START read 1 byte.
    const uint8_t payload[] = {0xAA};
    m5::hal::v2::data::MemorySource tx_src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};
    uint8_t rx[1] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    ASSERT_TRUE(bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x10}}, &tx_src, SIZE_MAX,
                             &rx_sink, SIZE_MAX)
                    .has_value());

    EXPECT_EQ(rx[0], 0x10);           // pre-write reg_file[0x10], composed before the store
    EXPECT_EQ(reg_file[0x10], 0xAA);  // the data byte still lands in the backing store
}

TEST(SlaveRegMapAccessorWire, LongReadStreamsBeyondOneReplyChunk)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};

    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveRegMapAccessor regmap{slave.bus(), m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    RegMapTickResponder responder{regmap, /*min_rx_before_compose=*/1};

    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;

    // 96 bytes in ONE transaction: past the 64-byte compose chunk, so the
    // responder must stream a continuation chunk while the master keeps
    // clocking (the reply-pump streaming fix; previously a read was capped at one window).
    uint8_t rx[96] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x00}}, nullptr, 0, &rx_sink, SIZE_MAX)
            .has_value());

    for (int i = 0; i < 96; ++i) {
        ASSERT_EQ(rx[i], static_cast<uint8_t>(i)) << "at offset " << i;
    }
}

TEST(SlaveRegMapAccessorWire, OneByteReadComposeReadAheadIsBounded)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};

    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveRegMapAccessor regmap{slave.bus(), m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    ComposeCallCounter counter;
    regmap.setOnRead(&regMapOnReadCountCalls, &counter);
    RegMapTickResponder responder{regmap, /*min_rx_before_compose=*/1};

    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;

    // Documented read-ahead bound: onRead fires ahead of the wire, but by no
    // more than the TX queue depth plus one staged chunk -- two 64-byte chunks
    // here. A 1-byte read must compose at least the first window and at most
    // two of them (regression pin for the public contract's caveat).
    uint8_t rx[1] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0x20}}, nullptr, 0, &rx_sink, SIZE_MAX)
            .has_value());

    EXPECT_EQ(rx[0], 0x20);
    EXPECT_GE(counter.calls, size_t{64});
    EXPECT_LE(counter.calls, size_t{128});
}

TEST(SlaveRegMapAccessorWire, LongReadContinuationWrapsPastRegisterFileEnd)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};

    uint8_t reg_file[256];
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    m5::hal::v2::i2c::SlaveRegMapAccessor regmap{slave.bus(), m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    RegMapTickResponder responder{regmap, /*min_rx_before_compose=*/1};

    runner.add(slave.service());
    runner.add(responder);
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    acc_cfg.use_restart     = true;

    // 200 bytes from 0xC0: crosses several continuation chunks AND the 8-bit
    // register-address wrap (0xC0..0xFF then 0x00..0x87) -- the classic
    // auto-increment regmap behavior, unbounded by the chunk size.
    uint8_t rx[200] = {};
    m5::hal::v2::data::MemorySink rx_sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    ASSERT_TRUE(
        bus.transfer(nullptr, acc_cfg, m5::hal::v2::i2c::TransferDesc{uint8_t{0xC0}}, nullptr, 0, &rx_sink, SIZE_MAX)
            .has_value());

    for (int i = 0; i < 200; ++i) {
        ASSERT_EQ(rx[i], static_cast<uint8_t>(0xC0 + i)) << "at offset " << i;
    }
}

TEST(SoftwareIBus, VirtualOpenDrainBusReportsTimeoutWhenSclHeldLow)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    lines.slavePullSclLow(true);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = 1;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.probe();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
}

TEST(SoftwareIBus, VirtualOpenDrainBusReportsBusErrorWhenSdaHeldLowAtStop)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    lines.slavePullSdaLow(true);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus_software bus;
    ASSERT_TRUE(bus.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.probe();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::I2C_BUS_ERROR);
}

// ---------------------------------------------------------------------------
// Behaviour of `Bus::lock` / `unlock`,
// `Accessor::beginAccess` / `endAccess`, and the RAII helpers
// `ScopedAccess` / `ScopedLock`.
// ---------------------------------------------------------------------------

namespace stage2 {

// `StubBus` is a minimal `IBus`. `transfer` records the arguments
// so tests can observe them, and writes one byte (`fake_rx_byte`)
// into the rx sink for `readRegister`-style tests. The lock owner
// is exposed through a getter so the test can observe lock / unlock
// directly.
class StubBus : public m5::hal::v2::i2c::IBus {
public:
    const m5::hal::v2::i2c::IBusConfig& getConfig(void) const override
    {
        return _config;
    }
    m5::hal::v2::result_t<void> transfer(m5::hal::v2::bus::IAccessor* owner,
                                         const m5::hal::v2::i2c::MasterAccessConfig& cfg,
                                         const m5::hal::v2::i2c::TransferDesc& desc, m5::hal::v2::data::Source* tx,
                                         size_t tx_len, m5::hal::v2::data::Sink* rx, size_t rx_len) override
    {
        ++transfer_count;
        last_cfg                        = cfg;
        last_desc                       = desc;
        last_tx_was_set                 = (tx != nullptr);
        last_rx_was_set                 = (rx != nullptr);
        last_owner_in_transaction       = false;
        last_owner_in_transaction_known = (owner != nullptr);
        if (owner != nullptr) {
            last_owner_in_transaction = static_cast<const m5::hal::v2::i2c::MasterAccessor*>(owner)->inTransaction();
        }
        last_totals.clear();
        if (tx) {
            // Drain the tx source and stash the bytes into the observation buffer.
            size_t remaining = tx_len;
            while (!tx->eof() && remaining > 0) {
                auto peeked = tx->peek(remaining);
                if (!peeked.has_value() || peeked.value().size == 0) break;
                auto span = peeked.value().first(remaining);
                for (size_t i = 0; i < span.size && tx_recorded.size() < 32; ++i) {
                    tx_recorded.push_back(span.data[i]);
                }
                auto adv = tx->advance(span.size);
                if (!adv.has_value()) break;
                remaining -= span.size;
                last_totals.tx += span.size;
            }
        }
        if (rx && rx_len > 0) {
            auto rsv = rx->reserve(rx_len);
            if (rsv.has_value() && rsv.value().size > 0) {
                auto span = rsv.value().first(rx_len);
                // Fill every byte with `fake_rx_byte`.
                for (size_t i = 0; i < span.size; ++i) {
                    span.data[i] = fake_rx_byte;
                }
                (void)rx->commit(span.size);
                last_totals.rx += span.size;
            }
        }
        return {};
    }
    m5::hal::v2::result_t<m5::hal::v2::bus::TransferTotals> waitTransfer(
        m5::hal::v2::bus::IAccessor* owner, const m5::hal::v2::i2c::MasterAccessConfig& cfg) override
    {
        (void)owner;
        (void)cfg;
        auto totals = last_totals;
        last_totals.clear();
        return totals;
    }

    const m5::hal::v2::bus::IAccessor* lockOwner(void) const
    {
        return _lock_owner;
    }
    size_t transfer_count = 0;
    m5::hal::v2::i2c::MasterAccessConfig last_cfg;
    m5::hal::v2::i2c::TransferDesc last_desc;
    bool last_tx_was_set                 = false;
    bool last_rx_was_set                 = false;
    bool last_owner_in_transaction       = false;
    bool last_owner_in_transaction_known = false;
    m5::hal::v2::bus::TransferTotals last_totals;
    std::vector<uint8_t> tx_recorded;
    uint8_t fake_rx_byte = 0;
};

m5::hal::v2::i2c::MasterAccessConfig makeAcc(uint16_t addr)
{
    m5::hal::v2::i2c::MasterAccessConfig acc;
    acc.i2c_addr        = addr;
    acc.freq            = 100000;
    acc.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;
    return acc;
}

TEST(BusLock, LockSetsOwnerUnlockClears)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(BusLock, SecondLockTimesOut)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor a1{bus, acc_cfg};
    m5::hal::v2::i2c::MasterAccessor a2{bus, acc_cfg};

    ASSERT_TRUE(bus.lock(&a1).has_value());

    // The default timeout now waits forever; pass a small finite budget
    // so the contention check stays deterministic.
    auto r = bus.lock(&a2, 10);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_EQ(bus.lockOwner(), &a1);  // The owner is unchanged.

    (void)bus.unlock(&a1);
}

TEST(BusLock, UnlockByWrongOwnerFails)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor a1{bus, acc_cfg};
    m5::hal::v2::i2c::MasterAccessor a2{bus, acc_cfg};

    ASSERT_TRUE(bus.lock(&a1).has_value());

    auto r = bus.unlock(&a2);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(bus.lockOwner(), &a1);  // The owner is unchanged.

    (void)bus.unlock(&a1);
}

TEST(AccessorAccess, BeginEndManageBusLock)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.endAccess();
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_STATE);
}

TEST(AccessorTransaction, BeginEndManageBusLockAndReturnTotals)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    EXPECT_FALSE(accessor.inTransaction());
    EXPECT_FALSE(accessor.inAccess());
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    EXPECT_TRUE(accessor.inTransaction());
    EXPECT_TRUE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), &accessor);

    auto totals = accessor.endTransaction();
    ASSERT_TRUE(totals.has_value());
    EXPECT_EQ(totals->tx, size_t{0});
    EXPECT_EQ(totals->rx, size_t{0});
    EXPECT_FALSE(accessor.inTransaction());
    EXPECT_FALSE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(AccessorTransaction, NestedBeginEndOnlyLocksOnce)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    EXPECT_TRUE(accessor.inTransaction());
    EXPECT_TRUE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), &accessor);

    auto inner = accessor.endTransaction();
    ASSERT_TRUE(inner.has_value());
    EXPECT_TRUE(accessor.inTransaction());
    EXPECT_TRUE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), &accessor);

    auto outer = accessor.endTransaction();
    ASSERT_TRUE(outer.has_value());
    EXPECT_FALSE(accessor.inTransaction());
    EXPECT_FALSE(accessor.inAccess());
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(AccessorTransaction, EndTransactionWithoutBeginFails)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.endTransaction();
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_STATE);
}

TEST(AccessorTransaction, TransferInsideTransactionKeepsLockUntilEnd)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    const uint8_t tx[] = {0x12, 0x34};
    uint8_t rx[3]      = {};
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    EXPECT_EQ(bus.lockOwner(), &accessor);

    auto w = accessor.transfer(m5::hal::v2::i2c::TransferDesc{}, m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)},
                               m5::hal::v2::data::DataSpan{});
    auto r = accessor.transfer(m5::hal::v2::i2c::TransferDesc{}, m5::hal::v2::data::ConstDataSpan{},
                               m5::hal::v2::data::DataSpan{rx, sizeof(rx)});
    EXPECT_TRUE(w.has_value());
    EXPECT_TRUE(r.has_value());
    EXPECT_EQ(bus.transfer_count, 2u);
    EXPECT_EQ(bus.lockOwner(), &accessor);
    EXPECT_TRUE(accessor.inTransaction());
    EXPECT_TRUE(accessor.inAccess());

    auto totals = accessor.endTransaction();
    ASSERT_TRUE(totals.has_value());
    EXPECT_EQ(totals->tx, sizeof(tx));
    EXPECT_EQ(totals->rx, sizeof(rx));
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

// ---- Unbound construction + typed bind --------------------------------

TEST(AccessorBind, UnboundThenBindRunsNormally)
{
    StubBus bus;
    m5::hal::v2::i2c::MasterAccessor accessor{makeAcc(0x10)};  // unbound

    EXPECT_FALSE(accessor.isBound());
    ASSERT_TRUE(accessor.bind(bus).has_value());
    EXPECT_TRUE(accessor.isBound());

    ASSERT_TRUE(accessor.beginAccess(10).has_value());
    EXPECT_EQ(bus.lockOwner(), &accessor);
    ASSERT_TRUE(accessor.endAccess().has_value());
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(AccessorBind, RebindMovesToTheNewBus)
{
    StubBus bus_a;
    StubBus bus_b;
    m5::hal::v2::i2c::MasterAccessor accessor{makeAcc(0x10)};

    ASSERT_TRUE(accessor.bind(bus_a).has_value());
    ASSERT_TRUE(accessor.bind(bus_b).has_value());  // rebind outside a window is fine

    ASSERT_TRUE(accessor.beginAccess(10).has_value());
    EXPECT_EQ(bus_b.lockOwner(), &accessor);
    EXPECT_EQ(bus_a.lockOwner(), nullptr);
    (void)accessor.endAccess();
}

TEST(AccessorBind, BindIsRejectedWhileAWindowIsOpen)
{
    StubBus bus_a;
    StubBus bus_b;
    m5::hal::v2::i2c::MasterAccessor accessor{bus_a, makeAcc(0x10)};

    ASSERT_TRUE(accessor.beginAccess(10).has_value());
    auto r = accessor.bind(bus_b);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    EXPECT_EQ(bus_a.lockOwner(), &accessor);  // still on the original bus
    (void)accessor.endAccess();
}

// `EXPECT_DEATH` is only meaningful in debug builds (NDEBUG undefined),
// which is what the native test env runs; release builds return
// INVALID_ARGUMENT from the same gate instead.
TEST(AccessorBindDeathTest, UnboundWindowOpenAssertsInDebug)
{
    m5::hal::v2::i2c::MasterAccessor accessor{makeAcc(0x10)};  // never bound
    EXPECT_DEATH({ (void)accessor.beginAccess(10); }, "not bound to a bus");
}

TEST(ScopedAccess, AcquiresAndReleasesAtScopeExit)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    {
        m5::hal::v2::bus::ScopedAccess scope{accessor};
        EXPECT_FALSE(scope.has_error());
        EXPECT_TRUE(scope.ok());  // positive view == !has_error()
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
    m5::hal::v2::i2c::MasterAccessor blocker{bus, acc_cfg};
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    ASSERT_TRUE(bus.lock(&blocker).has_value());

    {
        // Finite budget: the default would wait forever on the blocker.
        m5::hal::v2::bus::ScopedAccess scope{accessor, 10};
        EXPECT_TRUE(scope.has_error());
        EXPECT_FALSE(scope.ok());  // positive view == !has_error()
        EXPECT_EQ(scope.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    {
        m5::hal::v2::bus::ScopedLock scope{bus, &accessor};
        EXPECT_FALSE(scope.has_error());
        EXPECT_TRUE(scope.ok());  // positive view == !has_error()
        EXPECT_EQ(bus.lockOwner(), &accessor);
    }
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(ScopedLock, ContendedAcquireSurfacesTimeout)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor a1{bus, acc_cfg};
    m5::hal::v2::i2c::MasterAccessor a2{bus, acc_cfg};

    ASSERT_TRUE(bus.lock(&a1).has_value());

    {
        // Finite budget: the default would wait forever on the holder.
        m5::hal::v2::bus::ScopedLock scope{bus, &a2, 10};
        EXPECT_TRUE(scope.has_error());
        EXPECT_FALSE(scope.ok());  // positive view == !has_error()
        EXPECT_EQ(scope.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    }
    EXPECT_EQ(bus.lockOwner(), &a1);

    (void)bus.unlock(&a1);
}

TEST(AccessorCoreTransfer, RequiresOpenTransaction)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    EXPECT_EQ(bus.lockOwner(), nullptr);
    auto r = accessor.transfer(m5::hal::v2::i2c::TransferDesc{}, m5::hal::v2::data::ConstDataSpan{},
                               m5::hal::v2::data::DataSpan{});
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    EXPECT_EQ(bus.transfer_count, 0u);
    EXPECT_FALSE(accessor.inTransaction());
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(AccessorSugar, WriteLocksDuringCallAndAccumulatesTx)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    const uint8_t tx[] = {0x12, 0x34};
    EXPECT_EQ(bus.lockOwner(), nullptr);
    auto r = accessor.write(m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)});
    EXPECT_TRUE(r.has_value());
    EXPECT_EQ(bus.transfer_count, 1u);
    EXPECT_TRUE(bus.last_owner_in_transaction_known);
    EXPECT_TRUE(bus.last_owner_in_transaction);
    EXPECT_FALSE(accessor.inTransaction());
    EXPECT_EQ(bus.tx_recorded, (std::vector<uint8_t>{0x12, 0x34}));
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

TEST(AccessorSugar, ReadReturnsRxCountFromEndTransaction)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    uint8_t rx[3] = {};
    auto r        = accessor.read(m5::hal::v2::data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), sizeof(rx));
    EXPECT_EQ(bus.transfer_count, 1u);
    EXPECT_TRUE(bus.last_owner_in_transaction_known);
    EXPECT_TRUE(bus.last_owner_in_transaction);
    EXPECT_FALSE(accessor.inTransaction());
}

TEST(AccessorSugar, WriteInsideScopedAccessOnlyLocksOnce)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    {
        m5::hal::v2::bus::ScopedAccess scope{accessor};
        ASSERT_FALSE(scope.has_error());
        EXPECT_EQ(bus.lockOwner(), &accessor);

        // The sugar runs through a transaction while the outer access window
        // keeps the bus lock alive after each call returns.
        const uint8_t tx1[] = {0x12};
        const uint8_t tx2[] = {0x34};
        auto r1             = accessor.write(m5::hal::v2::data::ConstDataSpan{tx1, sizeof(tx1)});
        auto r2             = accessor.write(m5::hal::v2::data::ConstDataSpan{tx2, sizeof(tx2)});
        EXPECT_TRUE(r1.has_value());
        EXPECT_TRUE(r2.has_value());
        EXPECT_EQ(bus.transfer_count, 2u);
        EXPECT_TRUE(bus.last_owner_in_transaction_known);
        EXPECT_TRUE(bus.last_owner_in_transaction);
        EXPECT_FALSE(accessor.inTransaction());
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
    m5::hal::v2::i2c::TransferDesc d{uint8_t{0xD0}};
    EXPECT_EQ(d.prefix_len, 1);
    EXPECT_EQ(d.prefix[0], 0xD0);
}

TEST(TransferDesc, CtorTwoByteFromUint16IsBigEndian)
{
    // `uint16_t` (sizeof = 2) expands to a 2-byte big-endian (MSB first) prefix.
    m5::hal::v2::i2c::TransferDesc d{uint16_t{0x1234}};
    EXPECT_EQ(d.prefix_len, 2);
    EXPECT_EQ(d.prefix[0], 0x12);
    EXPECT_EQ(d.prefix[1], 0x34);
}

TEST(TransferDesc, CtorFourByteFromUint32IsBigEndian)
{
    // `uint32_t` (sizeof = 4) expands to a 4-byte big-endian (MSB first) prefix.
    m5::hal::v2::i2c::TransferDesc d{uint32_t{0xDEADBEEF}};
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
    m5::hal::v2::i2c::TransferDesc d{uint8_t{0xAA}, uint8_t{0xBB}};
    EXPECT_EQ(d.prefix_len, 2);
    EXPECT_EQ(d.prefix[0], 0xAA);
    EXPECT_EQ(d.prefix[1], 0xBB);
}

TEST(TransferDesc, CtorFourByteFromExplicitBytes)
{
    m5::hal::v2::i2c::TransferDesc d{uint8_t{0x01}, uint8_t{0x02}, uint8_t{0x03}, uint8_t{0x04}};
    EXPECT_EQ(d.prefix_len, 4);
    EXPECT_EQ(d.prefix[0], 0x01);
    EXPECT_EQ(d.prefix[3], 0x04);
}

TEST(TransferDesc, DefaultCtorIsEmpty)
{
    m5::hal::v2::i2c::TransferDesc d;
    EXPECT_EQ(d.prefix_len, 0);
}

TEST(AccessorRegister, WriteRegisterSendsPrefixAndValue)
{
    StubBus bus;
    auto acc_cfg = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    static constexpr uint8_t REG_DATA = 0xF7;
    uint8_t buf[4]                    = {0, 0, 0, 0};
    auto r                            = accessor.readRegister(REG_DATA, m5::hal::v2::data::DataSpan{buf, sizeof(buf)});
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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    auto r = accessor.readRegister(0x1234);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

// 2-byte register addresses (think large-capacity devices such as
// EEPROMs). Verifies that the wire convention stays big-endian
// (MSB first).
TEST(AccessorRegister, WriteRegister16IsBigEndian)
{
    StubBus bus;
    auto acc_cfg                   = makeAcc(0x50);  // Imaginary EEPROM target at 0x50.
    acc_cfg.register_address_bytes = 2;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    bus.fake_rx_byte               = 0x5A;
    auto acc_cfg                   = makeAcc(0x50);
    acc_cfg.register_address_bytes = 2;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    bus.fake_rx_byte               = 0xEE;
    auto acc_cfg                   = makeAcc(0x50);
    acc_cfg.register_address_bytes = 2;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

    static constexpr uint16_t REG_PAGE = 0x0100;
    uint8_t buf[3]                     = {};
    auto r                             = accessor.readRegister(REG_PAGE, m5::hal::v2::data::DataSpan{buf, sizeof(buf)});
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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    auto acc_cfg                   = makeAcc(0x50);  // Imaginary EEPROM target.
    acc_cfg.register_address_bytes = 2;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
    bus.fake_rx_byte               = 0x77;
    auto acc_cfg                   = makeAcc(0x50);
    acc_cfg.register_address_bytes = 2;
    m5::hal::v2::i2c::MasterAccessor accessor{bus, acc_cfg};

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
        m5::hal::v2::i2c::MasterAccessor accessor{bus_raw, acc_cfg};
        ASSERT_TRUE(accessor.writeRegister(REG, payload, sizeof(payload)).has_value());
    }

    // Span overload
    StubBus bus_span;
    {
        auto acc_cfg = makeAcc(0x10);
        m5::hal::v2::i2c::MasterAccessor accessor{bus_span, acc_cfg};
        ASSERT_TRUE(
            accessor.writeRegister(REG, m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}).has_value());
    }

    EXPECT_EQ(bus_raw.last_desc.prefix_len, bus_span.last_desc.prefix_len);
    EXPECT_EQ(bus_raw.last_desc.prefix[0], bus_span.last_desc.prefix[0]);
    EXPECT_EQ(bus_raw.tx_recorded, bus_span.tx_recorded);
}

}  // namespace spec_polish_a3

// ---------------------------------------------------------------------------
// `IBusConfig` exposes a single gpio_number_t path: `pin_scl` /
// `pin_sda` default to `-1`. Callers fill in gpio_number values via
// the 2-argument ctor or field assignment, and `init()` resolves a
// `Pin` from `m5::hal::v2::M5_Hal.Gpio.getPin(num)` through the
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

TEST(IBusConfig, DefaultCtorLeavesPinsInvalid)
{
    m5::hal::v2::i2c::BusConfig_software cfg;
    EXPECT_LT(cfg.pin_scl, 0);
    EXPECT_LT(cfg.pin_sda, 0);
}

TEST(IBusConfig, SoftwareVariantRejectsInvalidPins)
{
    m5::hal::v2::i2c::BusConfig_software cfg;
    // `pin_scl` / `pin_sda` keep their default value (-1).

    m5::hal::v2::i2c::Bus_software bus;
    auto err = bus.init(cfg);
    ASSERT_FALSE(err.has_value());
    EXPECT_EQ(err.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(IBusConfig, FieldAssignedGpioNumbersResolveViaGPIOGroup)
{
    // Field-assigned gpio_numbers land in `pin_scl` / `pin_sda`.
    // `init()` calls `m5::hal::v2::M5_Hal.Gpio.getPin(num)`, which
    // resolves to a `stub::Port` Pin in the native build via
    // `M5_Hal.Gpio` -> `stub::GPIO`.
    auto cfg = makeSoftwareBusConfig(/*scl=*/21, /*sda=*/22);
    EXPECT_EQ(cfg.pin_scl, 21);
    EXPECT_EQ(cfg.pin_sda, 22);

    m5::hal::v2::i2c::Bus_software bus;
    auto err = bus.init(cfg);
    EXPECT_TRUE(err.has_value());
    (void)bus.release();
}

TEST(IBusConfig, SoftwareVariantAcceptsRecordingGPIOViaGPIOGroup)
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

    auto cfg = makeSoftwareBusConfig(rec.scl(), rec.sda());

    m5::hal::v2::i2c::Bus_software bus;
    auto err = bus.init(cfg);
    EXPECT_TRUE(err.has_value());
    (void)bus.release();
}

}  // namespace spec_polish_a1

// ---------------------------------------------------------------------------
// `setConfig` (replace an accessor's cfg) and `IBus::probe(addr)`
// (the accessor-less probe sugar). Reuses `StubBus` from earlier.
// ---------------------------------------------------------------------------

namespace spec_polish_a2 {

using stage2::makeAcc;
using stage2::StubBus;

TEST(AccessorSetConfig, ReplacesConfigOutsideAccess)
{
    StubBus bus;
    auto cfg1 = makeAcc(0x10);
    m5::hal::v2::i2c::MasterAccessor accessor{bus, cfg1};

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
    m5::hal::v2::i2c::MasterAccessor accessor{bus, cfg1};

    auto ba = accessor.beginAccess();
    ASSERT_TRUE(ba.has_value());

    auto cfg2 = makeAcc(0x20);
    auto r    = accessor.setConfig(cfg2);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_STATE);

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
    // from `MasterAccessConfig`'s default of 1000 ms — scan loops
    // shouldn't pay one second per NACK.
    EXPECT_EQ(bus.last_cfg.freq, 100000u);
    EXPECT_EQ(bus.last_cfg.wire_timeout_ms, 50u);
}

TEST(BusProbe, AcceptsCustomFreqAndTimeout)
{
    StubBus bus;
    (void)bus.probe(0x42, 400000, 200);
    EXPECT_EQ(bus.last_cfg.freq, 400000u);
    EXPECT_EQ(bus.last_cfg.wire_timeout_ms, 200u);
}

TEST(BusProbe, ReleasesLockAfterCall)
{
    // `probe(addr)` internally builds a stack-local sentinel
    // accessor and walks `beginAccess` -> `transfer` -> `endAccess`.
    // By the time `probe` returns, `bus.lockOwner()` must be
    // `nullptr` (the sentinel's dtor has released the lock); if this
    // breaks, every subsequent caller times out.
    StubBus bus;
    (void)bus.probe(0x42);
    EXPECT_EQ(bus.lockOwner(), nullptr);
}

}  // namespace spec_polish_a2

// ---------------------------------------------------------------------------
// Tag-pin constructors. One-line construction with strong-typed
// pin tags (`Scl` / `Sda`) — either argument order lands on the right
// field, an untagged positional call stays a compile error, and the
// variant configs expose the constructors through ctor inheritance.
// ---------------------------------------------------------------------------

namespace s20_tag_pin_ctor {

namespace i2c   = m5::hal::v2::i2c;
namespace types = m5::hal::v2::types;

// The construction contract is compile-time; pin the load-bearing
// parts with static_asserts so a regression fails the build, not a run.
static_assert(std::is_constructible<i2c::IBusConfig, i2c::Scl, i2c::Sda>::value, "tag ctor (Scl, Sda)");
static_assert(std::is_constructible<i2c::IBusConfig, i2c::Sda, i2c::Scl>::value, "tag ctor (Sda, Scl)");
static_assert(!std::is_constructible<i2c::IBusConfig, int, int>::value, "no untagged positional ctor");
static_assert(!std::is_convertible<types::gpio_number_t, i2c::Scl>::value, "tags take no implicit integer");
static_assert(std::is_constructible<i2c::BusConfig_software, i2c::Scl, i2c::Sda>::value,
              "variant config inherits the tag ctors");
static_assert(!std::is_constructible<i2c::BusConfig_software, i2c::Sda, i2c::Sda>::value, "no duplicate-tag ctor");

TEST(IBusConfig, TagCtorEitherOrderLandsOnTheRightField)
{
    constexpr i2c::IBusConfig a{i2c::Scl{22}, i2c::Sda{21}};
    constexpr i2c::IBusConfig b{i2c::Sda{21}, i2c::Scl{22}};
    EXPECT_EQ(a.pin_scl, 22);
    EXPECT_EQ(a.pin_sda, 21);
    EXPECT_EQ(b.pin_scl, 22);
    EXPECT_EQ(b.pin_sda, 21);
}

TEST(IBusConfig, TagCtorKeepsTheBusKind)
{
    constexpr i2c::IBusConfig cfg{i2c::Scl{22}, i2c::Sda{21}};
    EXPECT_EQ(cfg.getBusKind(), types::bus_kind_t::I2C);
}

TEST(IBusConfig, TagConstructedVariantConfigInitsTheBus)
{
    // Equivalent to the field-assignment path of
    // `FieldAssignedGpioNumbersResolveViaGPIOGroup`, through the
    // inherited tag ctor on the variant config.
    i2c::BusConfig_software cfg{i2c::Scl{21}, i2c::Sda{22}};
    EXPECT_EQ(cfg.pin_scl, 21);
    EXPECT_EQ(cfg.pin_sda, 22);

    i2c::Bus_software bus;
    auto err = bus.init(cfg);
    EXPECT_TRUE(err.has_value());
    (void)bus.release();
}

}  // namespace s20_tag_pin_ctor

// ===========================================================================
// serve(Source/Sink) consumer loop + RegMap serve() over a SCRIPTED ISlaveBus.
//
// The blocking serve() / RegMap serve() drive the bus through ISlaveBus
// (readableBytes / read / write / transactionComplete). A real software bus needs
// a concurrent master to feed those, which a single-threaded test cannot run
// alongside a blocking serve(). ScriptedSlaveBus replaces the backend: it hands
// serve() a fixed "received" payload and captures the reply, and reports the
// transaction complete once every received byte has been read. That
// deterministically exercises the consumer logic -- the reserve/commit/peek/advance
// loop AND the stall-escape -- with no master and no threads. Real-backend
// window/ring behaviour and the RegMap wire matrix stay covered by the
// SlaveStreamAccessorWindow / SlaveRegMapAccessorWire tests above (and HIL).
// ===========================================================================
namespace serve_scripted {

class ScriptedSlaveBus : public m5::hal::v2::i2c::ISlaveBus {
public:
    std::vector<uint8_t> rx_script;   // bytes the master wrote (serve() reads these)
    std::vector<uint8_t> tx_capture;  // bytes serve() replied (the master would read)
    size_t rx_pos = 0;

    m5::hal::v2::result_t<void> init(const m5::hal::v2::i2c::SlaveBusConfig&) override
    {
        return {};
    }
    m5::hal::v2::result_t<void> beginTransaction(m5::hal::v2::bus::IAccessor*, uint32_t) override
    {
        return {};
    }
    m5::hal::v2::result_t<void> endTransaction(m5::hal::v2::bus::IAccessor*) override
    {
        return {};
    }
    m5::hal::v2::result_t<size_t> readableBytes(m5::hal::v2::bus::IAccessor*) override
    {
        return rx_script.size() - rx_pos;
    }
    m5::hal::v2::result_t<size_t> read(m5::hal::v2::bus::IAccessor*, m5::hal::v2::data::DataSpan dst) override
    {
        const size_t n = std::min(dst.size, rx_script.size() - rx_pos);
        for (size_t i = 0; i < n && dst.data != nullptr; ++i) {
            static_cast<uint8_t*>(dst.data)[i] = rx_script[rx_pos + i];
        }
        rx_pos += n;
        return n;
    }
    m5::hal::v2::result_t<size_t> write(m5::hal::v2::bus::IAccessor*, m5::hal::v2::data::ConstDataSpan src) override
    {
        const auto* p = static_cast<const uint8_t*>(src.data);
        tx_capture.insert(tx_capture.end(), p, p + src.size);
        return src.size;
    }
    m5::hal::v2::result_t<bool> transactionComplete(m5::hal::v2::bus::IAccessor*) override
    {
        return rx_pos >= rx_script.size();
    }
    m5::hal::v2::service::IService* service() override
    {
        return nullptr;
    }
};

// A master that goes inactive mid-transaction: the scripted bytes arrive but the
// STOP never does (transactionComplete stays false), modeling a master that died
// or aborted with no visible STOP. The tx side is bounded like a real backend's
// tx ring, so the reply pump cannot register endless fake progress. This is the
// case a finite serve(timeout) must escape from instead of waiting forever.
class StalledSlaveBus : public ScriptedSlaveBus {
public:
    size_t tx_capacity = 64;

    m5::hal::v2::result_t<size_t> write(m5::hal::v2::bus::IAccessor*, m5::hal::v2::data::ConstDataSpan src) override
    {
        const size_t room = tx_capacity > tx_capture.size() ? tx_capacity - tx_capture.size() : 0;
        const size_t n    = std::min(room, src.size);
        const auto* p     = static_cast<const uint8_t*>(src.data);
        tx_capture.insert(tx_capture.end(), p, p + n);
        return n;
    }
    m5::hal::v2::result_t<bool> transactionComplete(m5::hal::v2::bus::IAccessor*) override
    {
        return false;
    }
};

// A master that pauses mid-transaction and resumes later: the scripted bytes
// become visible only after `reveal_after_ms`. Exercises the escape-drain
// deadline boundary -- a serve() that escaped during the pause must still be
// draining (fresh second deadline) when the bytes finally arrive.
class LateTailSlaveBus : public ScriptedSlaveBus {
public:
    uint32_t reveal_after_ms = 0;
    uint32_t t0              = m5::hal::v2::runtime::millis();

    bool revealed() const
    {
        return m5::hal::v2::runtime::millis() - t0 >= reveal_after_ms;
    }
    m5::hal::v2::result_t<size_t> readableBytes(m5::hal::v2::bus::IAccessor* o) override
    {
        return revealed() ? ScriptedSlaveBus::readableBytes(o) : m5::hal::v2::result_t<size_t>{size_t{0}};
    }
    m5::hal::v2::result_t<size_t> read(m5::hal::v2::bus::IAccessor* o, m5::hal::v2::data::DataSpan dst) override
    {
        return revealed() ? ScriptedSlaveBus::read(o, dst) : m5::hal::v2::result_t<size_t>{size_t{0}};
    }
};

TEST(SlaveStreamServe, WriteTransactionFillsSink)
{
    ScriptedSlaveBus bus;
    bus.rx_script = {0x11, 0x22, 0x33, 0x44, 0x55};
    m5::hal::v2::i2c::SlaveStreamAccessor acc{bus};

    uint8_t rx[8] = {};
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    auto n = acc.serve(nullptr, &sink);  // default TIMEOUT_FOREVER
    ASSERT_TRUE(n.has_value());
    EXPECT_EQ(n.value(), size_t{5});
    EXPECT_EQ(sink.written(), size_t{5});
    EXPECT_EQ(rx[0], 0x11);
    EXPECT_EQ(rx[4], 0x55);
}

TEST(SlaveStreamServe, ReadTransactionDrainsSource)
{
    ScriptedSlaveBus bus;  // rx_script empty -> pure read transaction
    m5::hal::v2::i2c::SlaveStreamAccessor acc{bus};

    const uint8_t reply[] = {0xA1, 0xB2, 0xC3};
    m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{reply, sizeof(reply)}};
    auto n = acc.serve(&src, nullptr);
    ASSERT_TRUE(n.has_value());
    // serve() returns the RECEIVED amount (bytes the master wrote into the Sink).
    // This is a pure read transaction: the master received the 3-byte reply but
    // wrote nothing, so the received count is 0 (the reply is verified via tx_capture).
    EXPECT_EQ(n.value(), size_t{0});
    ASSERT_EQ(bus.tx_capture.size(), size_t{3});
    EXPECT_EQ(bus.tx_capture[0], 0xA1);
    EXPECT_EQ(bus.tx_capture[2], 0xC3);
}

TEST(SlaveStreamServe, FiniteTimeoutWithFittingSinkCompletesNormally)
{
    ScriptedSlaveBus bus;
    bus.rx_script = {0x7A, 0x7B, 0x7C};
    m5::hal::v2::i2c::SlaveStreamAccessor acc{bus};

    uint8_t rx[8] = {};
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    auto n = acc.serve(nullptr, &sink, 50);  // finite, but the Sink fits -> no escape
    ASSERT_TRUE(n.has_value());
    EXPECT_EQ(n.value(), size_t{3});
}

TEST(SlaveStreamServe, BoundedSinkStallEscapesWithFiniteTimeout)
{
    ScriptedSlaveBus bus;
    bus.rx_script.assign(100, 0xEE);  // master writes 100 bytes
    bus.rx_script[0] = 0x01;
    bus.rx_script[1] = 0x02;
    m5::hal::v2::i2c::SlaveStreamAccessor acc{bus};

    uint8_t rx[2] = {};  // Sink too small for the write -> stalls, then escapes
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    auto n = acc.serve(nullptr, &sink, 20);  // finite stall deadline
    ASSERT_FALSE(n.has_value());
    EXPECT_EQ(n.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    // The bytes that fit landed in the Sink; the rest were discarded to release the
    // hold (un-wedge the bus) and the write drained to its STOP.
    EXPECT_EQ(sink.written(), size_t{2});
    EXPECT_EQ(rx[0], 0x01);
    EXPECT_EQ(rx[1], 0x02);
    EXPECT_EQ(bus.rx_pos, size_t{100});
}

TEST(SlaveStreamServe, ClosedSinkEscapesUnderForeverTimeout)
{
    // A fixed Sink that fills mid-write reports closed(). serve() must escape
    // immediately on closed() -- even under the default TIMEOUT_FOREVER (no finite
    // deadline) -- so a bounded Sink can never wedge the bus waiting for room that
    // will never come. (Pre-P2 a FOREVER serve with a too-small Sink hung forever.)
    ScriptedSlaveBus bus;
    bus.rx_script.assign(50, 0xCD);  // master writes 50 bytes
    bus.rx_script[0] = 0x09;
    bus.rx_script[1] = 0x08;
    m5::hal::v2::i2c::SlaveStreamAccessor acc{bus};

    uint8_t rx[2] = {};  // 2-byte Sink: closed() once full, while 48 bytes remain
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    auto n = acc.serve(nullptr, &sink);  // TIMEOUT_FOREVER -- escapes via closed(), not a deadline
    ASSERT_FALSE(n.has_value());
    EXPECT_EQ(n.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_EQ(sink.written(), size_t{2});
    EXPECT_EQ(rx[0], 0x09);
    EXPECT_EQ(rx[1], 0x08);
    EXPECT_EQ(bus.rx_pos, size_t{50});  // the rest drained to discard (bus released)
}

TEST(SlaveStreamServe, MasterInactiveMidTransactionEscapesWithFiniteTimeout)
{
    // The master wrote 3 bytes, then went silent without a STOP. The Sink has
    // room (no local stall), yet no progress is possible -- a finite timeout
    // must abandon the transaction instead of waiting for a STOP that will
    // never come. (Pre-fix the escape drain still waited on
    // transactionComplete() forever even after the deadline expired.)
    StalledSlaveBus bus;
    bus.rx_script = {0x31, 0x32, 0x33};
    m5::hal::v2::i2c::SlaveStreamAccessor acc{bus};

    uint8_t rx[8] = {};
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    auto n = acc.serve(nullptr, &sink, 20);  // finite no-progress deadline
    ASSERT_FALSE(n.has_value());
    EXPECT_EQ(n.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    // The bytes that did arrive were delivered before the abandon.
    EXPECT_EQ(sink.written(), size_t{3});
    EXPECT_EQ(rx[0], 0x31);
    EXPECT_EQ(rx[2], 0x33);
}

TEST(SlaveStreamServe, EscapeDrainGetsAFreshSecondDeadline)
{
    // The master pauses long enough to trip the first no-progress deadline (so
    // serve() escapes), then finishes its write INSIDE the second deadline
    // window. The escape drain must survive to the STOP -- entering escape
    // grants one more full deadline -- instead of abandoning almost immediately
    // on the stale pre-escape timer (the boundary a stale timer breaks).
    LateTailSlaveBus bus;
    bus.rx_script.assign(10, 0x5A);
    bus.reveal_after_ms = 70;  // past the 1st deadline (50 ms), inside the 2nd (100 ms)
    m5::hal::v2::i2c::SlaveStreamAccessor acc{bus};

    uint8_t rx[16] = {};
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rx, sizeof(rx)}};
    auto n = acc.serve(nullptr, &sink, 50);
    ASSERT_FALSE(n.has_value());
    EXPECT_EQ(n.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);  // it DID escape...
    EXPECT_EQ(bus.rx_pos, size_t{10});                                 // ...but drained the late tail to the STOP
    EXPECT_EQ(sink.written(), size_t{0});                              // the tail went to discard, not the Sink
}

TEST(SlaveRegMapServe, RegisterWriteUpdatesFileAndFiresOnWrite)
{
    ScriptedSlaveBus bus;
    bus.rx_script         = {0x10, 0xAA, 0xBB};  // pointer 0x10, then two data bytes
    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    OnWriteLog log;
    rm.setOnWrite(regMapOnWriteLog, &log);

    ASSERT_TRUE(rm.serve().has_value());
    EXPECT_EQ(reg_file[0x10], 0xAA);
    EXPECT_EQ(reg_file[0x11], 0xBB);  // auto-increment
    ASSERT_EQ(log.count, size_t{2});
    EXPECT_EQ(log.regs[0], 0x10);
    EXPECT_EQ(log.vals[0], 0xAA);
    EXPECT_EQ(log.regs[1], 0x11);
    EXPECT_EQ(log.vals[1], 0xBB);
}

TEST(SlaveRegMapServe, SplitReadResolvesPersistedPointerWithAutoIncrement)
{
    ScriptedSlaveBus bus;
    uint8_t reg_file[256] = {};
    reg_file[0x20]        = 0xDE;
    reg_file[0x21]        = 0xAD;
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    // Transaction 1: write the pointer only -> sets pointer to 0x20 (persists).
    bus.rx_script = {0x20};
    bus.rx_pos    = 0;
    bus.tx_capture.clear();
    ASSERT_TRUE(rm.serve().has_value());

    // Transaction 2: pure read -> reply resolves against the persisted pointer.
    bus.rx_script.clear();
    bus.rx_pos = 0;
    bus.tx_capture.clear();
    ASSERT_TRUE(rm.serve().has_value());
    ASSERT_GE(bus.tx_capture.size(), size_t{2});
    EXPECT_EQ(bus.tx_capture[0], 0xDE);  // reg_file[0x20]
    EXPECT_EQ(bus.tx_capture[1], 0xAD);  // reg_file[0x21] (auto-increment)
}

TEST(SlaveRegMapServe, OnReadOverridesReplyByte)
{
    ScriptedSlaveBus bus;
    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};
    rm.setOnRead(regMapOnReadInvert, nullptr);  // returns ~reg

    bus.rx_script = {0x05};  // set pointer to 0x05
    ASSERT_TRUE(rm.serve().has_value());
    bus.rx_script.clear();
    bus.rx_pos = 0;
    bus.tx_capture.clear();
    ASSERT_TRUE(rm.serve().has_value());
    ASSERT_GE(bus.tx_capture.size(), size_t{2});
    EXPECT_EQ(bus.tx_capture[0], static_cast<uint8_t>(~0x05));  // onRead(0x05)
    EXPECT_EQ(bus.tx_capture[1], static_cast<uint8_t>(~0x06));  // onRead(0x06), auto-increment
}

TEST(SlaveRegMapServe, LongWriteBeyondReplyWindowFillsRegisterFile)
{
    ScriptedSlaveBus bus;
    bus.rx_script.push_back(0x00);  // pointer 0x00
    for (int i = 0; i < 80; ++i) {
        bus.rx_script.push_back(static_cast<uint8_t>(i));  // 80 data bytes (> 64 reply window)
    }
    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    ASSERT_TRUE(rm.serve().has_value());
    for (int i = 0; i < 80; ++i) {
        EXPECT_EQ(reg_file[i], static_cast<uint8_t>(i)) << "reg " << i;
    }
}

TEST(SlaveRegMapServe, MasterInactiveMidTransactionEscapesWithFiniteTimeout)
{
    // Pointer + one data byte arrive, then the master goes silent without a
    // STOP. serve(finite) must abandon the exchange and return TIMEOUT_ERROR
    // (pre-fix the timeout only bounded the transaction start, diverging from
    // the documented contract, and this hung forever). The bytes ingested
    // before the stall stay applied, like a real register device cut off
    // mid-write.
    StalledSlaveBus bus;
    bus.rx_script         = {0x10, 0xAA};
    uint8_t reg_file[256] = {};
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    auto r = rm.serve(20);  // finite no-progress deadline
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_EQ(reg_file[0x10], 0xAA);
}

// A backend that reports a bound ISR regmap fast path (SlaveRegMapAccessor::
// serve() then takes its fast-path branch) and confirms activity on every
// waitForActivity() call, each of which sleeps past a short finite deadline.
// transactionComplete() only flips true once `complete_after_activity` such
// calls have happened.
class FastPathActiveSlaveBus : public m5::hal::v2::i2c::ISlaveBus {
public:
    int activity_calls          = 0;
    int complete_after_activity = 4;

    m5::hal::v2::result_t<void> init(const m5::hal::v2::i2c::SlaveBusConfig&) override
    {
        return {};
    }
    m5::hal::v2::result_t<void> beginTransaction(m5::hal::v2::bus::IAccessor*, uint32_t) override
    {
        return {};
    }
    m5::hal::v2::result_t<void> endTransaction(m5::hal::v2::bus::IAccessor*) override
    {
        return {};
    }
    m5::hal::v2::result_t<size_t> read(m5::hal::v2::bus::IAccessor*, m5::hal::v2::data::DataSpan) override
    {
        return size_t{0};
    }
    m5::hal::v2::result_t<size_t> write(m5::hal::v2::bus::IAccessor*, m5::hal::v2::data::ConstDataSpan) override
    {
        return size_t{0};
    }
    m5::hal::v2::result_t<size_t> readableBytes(m5::hal::v2::bus::IAccessor*) override
    {
        return size_t{0};
    }
    m5::hal::v2::result_t<bool> transactionComplete(m5::hal::v2::bus::IAccessor*) override
    {
        return activity_calls >= complete_after_activity;
    }
    m5::hal::v2::service::IService* service() override
    {
        return nullptr;
    }
    m5::hal::v2::result_t<bool> waitForActivity(m5::hal::v2::bus::IAccessor*, uint32_t) override
    {
        ++activity_calls;
        // Real elapsed time across all calls exceeds the finite deadline used
        // below well before complete_after_activity calls accumulate, so a
        // deadline that does NOT refresh on confirmed activity (the F1 bug)
        // would time out first.
        m5::hal::v2::runtime::delayMs(8);
        return true;
    }
    bool bindIsrRegMap(m5::hal::v2::i2c::IsrRegMapBinding* binding) override
    {
        (void)binding;
        return true;
    }
    void unbindIsrRegMap(m5::hal::v2::i2c::IsrRegMapBinding*) override
    {
    }
};

// F1 regression: SlaveRegMapAccessor::serve()'s ISR fast-path branch must
// treat timeout_ms as a NO-PROGRESS stall deadline (refreshed by every
// CONFIRMED waitForActivity() wake), not a wall-clock total -- see the
// fast-path branch's doc comment. Each waitForActivity() call here sleeps
// 8ms and confirms activity; with a 20ms deadline that never refreshes, this
// would return TIMEOUT_ERROR well before the 4th call. With the no-progress
// fix it must complete OK once transactionComplete() flips true.
TEST(SlaveRegMapServe, FastPathNoProgressDeadlineRefreshesOnConfirmedActivity)
{
    FastPathActiveSlaveBus bus;
    uint8_t reg_file[16] = {};
    m5::hal::v2::i2c::SlaveRegMapAccessor rm{bus, m5::hal::v2::data::DataSpan{reg_file, sizeof(reg_file)}};

    auto r = rm.serve(20);  // finite deadline, shorter than the total real elapsed time
    ASSERT_TRUE(r.has_value()) << "err=" << m5::hal::v2::error::toString(r.error());
    EXPECT_GE(bus.activity_calls, 4);
}

}  // namespace serve_scripted

// ===========================================================================
// i2c::Bus runtime facade
//
// `i2c::Bus` is now a runtime facade that owns the lock + accessor binding and
// delegates transfer to a backend (`Bus_<variant>`) created by `init()` via the
// config->backend trait. Driving the facade through the normal accessor path
// over the virtual open-drain bus proves: init() created the software backend
// (from the explicit BusConfig_software), the accessor locked the FACADE, and
// transfer was delegated to the backend's wire.
// ===========================================================================

TEST(I2cBusFacade, DelegatesAccessorTransferToBackend)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus facade;  // the runtime facade, not Bus_software directly
    ASSERT_TRUE(facade.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());
    // getConfig is served from the facade's own cached config.
    EXPECT_EQ(facade.getConfig().pin_scl, gpio.scl());
    EXPECT_EQ(facade.getConfig().pin_sda, gpio.sda());

    m5::hal::v2::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr        = 0x42;
    acc_cfg.freq            = 100000;
    acc_cfg.wire_timeout_ms = M5HAL_TEST_WIRE_TIMEOUT_MS;

    // Accessor binds to the FACADE; the transaction locks the facade and
    // transfer delegates to the backend.
    m5::hal::v2::i2c::MasterAccessor dev{facade, acc_cfg};
    const uint8_t tx_bytes[] = {0x12, 0x34};
    ASSERT_TRUE(dev.beginTransaction().has_value());
    ASSERT_TRUE(dev.transfer(m5::hal::v2::i2c::TransferDesc{uint8_t{0xAB}},
                             m5::hal::v2::data::ConstDataSpan{tx_bytes, sizeof(tx_bytes)},
                             m5::hal::v2::data::DataSpan{})
                    .has_value());
    auto end = dev.endTransaction();
    ASSERT_TRUE(end.has_value());
    EXPECT_EQ(end->tx, sizeof(tx_bytes));
    EXPECT_EQ(end->rx, 0u);

    // The slave received prefix + payload -> the wire was driven through the
    // delegated backend.
    ASSERT_TRUE(slave.accessor().beginTransaction(0).has_value());
    uint8_t rx[3] = {};
    auto read     = slave.accessor().read(m5::hal::v2::data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(read.has_value());
    EXPECT_EQ(read.value(), sizeof(rx));
    EXPECT_EQ(rx[0], 0xAB);
    EXPECT_EQ(rx[1], 0x12);
    EXPECT_EQ(rx[2], 0x34);
    EXPECT_TRUE(slave.accessor().endTransaction().has_value());
}

TEST(I2cBusFacade, ProbeRoutesThroughFacade)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus facade;
    ASSERT_TRUE(facade.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());

    // probe() is inherited from i2c::IBus: it builds a stack accessor bound to
    // the facade, so it goes through the facade's lock + delegated transfer.
    EXPECT_TRUE(facade.probe(0x42).has_value());   // slave ACKs
    EXPECT_FALSE(facade.probe(0x21).has_value());  // no device -> NACK
}

TEST(I2cBusFacade, ReleaseIsIdempotent)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    m5::hal::v2::i2c::Bus facade;
    ASSERT_TRUE(facade.init(makeSoftwareBusConfig(gpio.scl(), gpio.sda())).has_value());
    EXPECT_TRUE(facade.release().has_value());
    EXPECT_TRUE(facade.release().has_value());  // backend already gone -> OK, no crash
    // dtor runs here with a null backend -> safe.
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
