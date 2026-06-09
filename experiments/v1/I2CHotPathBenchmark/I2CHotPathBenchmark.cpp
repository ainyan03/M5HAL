// =============================================================================
// M5HAL - I2CHotPathBenchmark
//
// Measures software I2C hot-path costs without requiring an external I2C slave.
// This sketch separates timer conversion, virtual line operations, and the
// software I2C write/read byte state machines so real-device measurements can be
// compared against synthetic upper bounds.
//
// The write/read-buffer "est kHz" values are internal estimates on a synthetic line
// driver. Real SCL/SDA waveforms also depend on pull-up strength, wiring, bus
// capacitance, and analyzer thresholds. If wire speed plateaus while this
// benchmark still has headroom, inspect SCL/SDA rise time before assuming the
// state machine is the bottleneck.
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v1.hpp>
#include <M5Utility.hpp>
#include <m5_hal/variants/frameworks/software/hal/i2c/i2c.hpp>

#if defined(ESP_PLATFORM)
#include <esp_cpu.h>
#endif

namespace {

namespace service = ::m5::hal::v1::service;
namespace sw_i2c  = ::m5::variants::frameworks::software::hal::v1::i2c::detail;

constexpr uint32_t kLoopIterations = 100000;
constexpr size_t kPayloadBytes     = 64;
constexpr uint32_t kTargetFreqHz   = 2000000;

volatile uint32_t g_sink32 = 0;

uint32_t readCycles()
{
#if defined(ESP_PLATFORM)
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
    return static_cast<uint32_t>(esp_cpu_get_cycle_count());
#else
    return static_cast<uint32_t>(esp_cpu_get_ccount());
#endif
#else
    return static_cast<uint32_t>(m5::utility::micros());
#endif
}

uint32_t cpuMhz()
{
#if defined(ESP_PLATFORM)
    return static_cast<uint32_t>(ESP.getCpuFreqMHz());
#else
    return 0;
#endif
}

struct BenchResult {
    uint32_t cycles     = 0;
    uint32_t usec       = 0;
    uint32_t calls      = 0;
    uint32_t idle_calls = 0;
    uint32_t scl_rises  = 0;
    uint32_t writes     = 0;
    uint32_t reads      = 0;
};

class SyntheticLineDriver : public sw_i2c::MasterLineDriver {
public:
    void reset()
    {
        scl        = true;
        sda        = true;
        writes     = 0;
        reads      = 0;
        scl_rises  = 0;
        sda_writes = 0;
    }

    void writeSclHigh() override
    {
        if (!scl) {
            ++scl_rises;
        }
        scl = true;
        ++writes;
    }

    void writeSclLow() override
    {
        scl = false;
        ++writes;
    }

    void writeSda(bool high) override
    {
        sda = high;
        ++writes;
        ++sda_writes;
    }

    bool readScl() const override
    {
        ++reads;
        return true;
    }

    bool readSda() const override
    {
        ++reads;
        return false;
    }

    bool scl               = true;
    bool sda               = true;
    mutable uint32_t reads = 0;
    uint32_t writes        = 0;
    uint32_t scl_rises     = 0;
    uint32_t sda_writes    = 0;
};

sw_i2c::MasterTiming makeTiming(uint32_t freq_hz)
{
    sw_i2c::MasterTiming timing;
    const uint64_t denom    = static_cast<uint64_t>(freq_hz) * 2ull;
    timing.half_period_nsec = static_cast<service::tick_nsec_t>((1000000000ull + (denom / 2ull)) / denom);
    timing.timeout_nsec     = 1000000000u;
    return timing;
}

sw_i2c::MasterTiming makeFastTickTiming(uint32_t freq_hz)
{
    const auto timing      = makeTiming(freq_hz);
    const auto fasttick_hz = service::fastTickFrequencyHz();
    sw_i2c::MasterTiming result;
    result.half_period_nsec = service::nsecToFastTickCeil(timing.half_period_nsec, fasttick_hz);
    result.timeout_nsec     = service::nsecToFastTickCeil(timing.timeout_nsec, fasttick_hz);
    return result;
}

void fillPayload(uint8_t* payload, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        payload[i] = static_cast<uint8_t>(0xA5u + (i * 17u));
    }
}

BenchResult measureVirtualLineOps()
{
    SyntheticLineDriver concrete;
    sw_i2c::MasterLineDriver* lines = &concrete;
    uint32_t acc                    = 0;

    const uint32_t start_cycles = readCycles();
    const uint32_t start_usec   = static_cast<uint32_t>(m5::utility::micros());
    for (uint32_t i = 0; i < kLoopIterations; ++i) {
        if (i & 1u) {
            lines->writeSclHigh();
        } else {
            lines->writeSclLow();
        }
        lines->writeSda((i & 2u) != 0);
        acc += lines->readScl() ? 1u : 0u;
        acc += lines->readSda() ? 2u : 0u;
    }
    const uint32_t end_usec   = static_cast<uint32_t>(m5::utility::micros());
    const uint32_t end_cycles = readCycles();

    g_sink32 += acc + concrete.writes + concrete.reads;
    return BenchResult{end_cycles - start_cycles, end_usec - start_usec, kLoopIterations, 0,
                       concrete.scl_rises,        concrete.writes,       concrete.reads};
}

BenchResult measureFastTickNsecLoop()
{
    uint32_t acc = 0;

    const uint32_t start_cycles = readCycles();
    const uint32_t start_usec   = static_cast<uint32_t>(m5::utility::micros());
    for (uint32_t i = 0; i < kLoopIterations; ++i) {
        acc += service::fastTickNsec();
    }
    const uint32_t end_usec   = static_cast<uint32_t>(m5::utility::micros());
    const uint32_t end_cycles = readCycles();

    g_sink32 += acc;
    return BenchResult{end_cycles - start_cycles, end_usec - start_usec, kLoopIterations, 0, 0, 0, 0};
}

BenchResult measureFastTickLoop()
{
    uint32_t acc = 0;

    const uint32_t start_cycles = readCycles();
    const uint32_t start_usec   = static_cast<uint32_t>(m5::utility::micros());
    for (uint32_t i = 0; i < kLoopIterations; ++i) {
        acc += service::fastTick();
    }
    const uint32_t end_usec   = static_cast<uint32_t>(m5::utility::micros());
    const uint32_t end_cycles = readCycles();

    g_sink32 += acc;
    return BenchResult{end_cycles - start_cycles, end_usec - start_usec, kLoopIterations, 0, 0, 0, 0};
}

BenchResult measureForcedDueWriteBuffer()
{
    uint8_t payload[kPayloadBytes];
    fillPayload(payload, sizeof(payload));

    SyntheticLineDriver lines;
    sw_i2c::MasterTransactionService transaction;
    sw_i2c::MasterTiming timing;
    timing.half_period_nsec = 0;
    timing.timeout_nsec     = 1000000000u;

    service::tick_nsec_t now = 0;
    transaction.beginWriteBuffer(lines, timing, payload, sizeof(payload), now);

    BenchResult result;
    const uint32_t start_cycles = readCycles();
    const uint32_t start_usec   = static_cast<uint32_t>(m5::utility::micros());
    for (;;) {
        const auto r = transaction.service(service::ServiceContext{transaction.dueNsec()});
        ++result.calls;
        if (r == service::ServiceResult::Idle) {
            ++result.idle_calls;
        } else if (r == service::ServiceResult::Done) {
            break;
        } else if (r == service::ServiceResult::Error) {
            break;
        }
    }
    const uint32_t end_usec   = static_cast<uint32_t>(m5::utility::micros());
    const uint32_t end_cycles = readCycles();

    result.cycles    = end_cycles - start_cycles;
    result.usec      = end_usec - start_usec;
    result.scl_rises = lines.scl_rises;
    result.writes    = lines.writes;
    result.reads     = lines.reads;
    g_sink32 += result.calls + result.scl_rises + transaction.transferred();
    return result;
}

BenchResult measureTimedWriteBuffer()
{
    uint8_t payload[kPayloadBytes];
    fillPayload(payload, sizeof(payload));

    SyntheticLineDriver lines;
    sw_i2c::MasterTransactionService transaction;
    const auto timing = makeFastTickTiming(kTargetFreqHz);

    transaction.beginWriteBuffer(lines, timing, payload, sizeof(payload), service::fastTick());

    BenchResult result;
    const uint32_t start_cycles = readCycles();
    const uint32_t start_usec   = static_cast<uint32_t>(m5::utility::micros());
    for (;;) {
        const auto now = service::fastTick();
        const auto r   = transaction.service(service::ServiceContext{now});
        ++result.calls;
        if (r == service::ServiceResult::Idle) {
            ++result.idle_calls;
        } else if (r == service::ServiceResult::Done) {
            break;
        } else if (r == service::ServiceResult::Error) {
            break;
        }
    }
    const uint32_t end_usec   = static_cast<uint32_t>(m5::utility::micros());
    const uint32_t end_cycles = readCycles();

    result.cycles    = end_cycles - start_cycles;
    result.usec      = end_usec - start_usec;
    result.scl_rises = lines.scl_rises;
    result.writes    = lines.writes;
    result.reads     = lines.reads;
    g_sink32 += result.calls + result.scl_rises + transaction.transferred();
    return result;
}

BenchResult measureForcedDueReadBuffer()
{
    uint8_t payload[kPayloadBytes] = {};

    SyntheticLineDriver lines;
    sw_i2c::MasterTransactionService transaction;
    sw_i2c::MasterTiming timing;
    timing.half_period_nsec = 0;
    timing.timeout_nsec     = 1000000000u;

    service::tick_nsec_t now = 0;
    transaction.beginReadBuffer(lines, timing, payload, sizeof(payload), true, now);

    BenchResult result;
    const uint32_t start_cycles = readCycles();
    const uint32_t start_usec   = static_cast<uint32_t>(m5::utility::micros());
    for (;;) {
        const auto r = transaction.service(service::ServiceContext{transaction.dueNsec()});
        ++result.calls;
        if (r == service::ServiceResult::Idle) {
            ++result.idle_calls;
        } else if (r == service::ServiceResult::Done) {
            break;
        } else if (r == service::ServiceResult::Error) {
            break;
        }
    }
    const uint32_t end_usec   = static_cast<uint32_t>(m5::utility::micros());
    const uint32_t end_cycles = readCycles();

    result.cycles    = end_cycles - start_cycles;
    result.usec      = end_usec - start_usec;
    result.scl_rises = lines.scl_rises;
    result.writes    = lines.writes;
    result.reads     = lines.reads;
    g_sink32 += result.calls + result.scl_rises + transaction.transferred() + payload[0];
    return result;
}

BenchResult measureTimedReadBuffer()
{
    uint8_t payload[kPayloadBytes] = {};

    SyntheticLineDriver lines;
    sw_i2c::MasterTransactionService transaction;
    const auto timing = makeFastTickTiming(kTargetFreqHz);

    transaction.beginReadBuffer(lines, timing, payload, sizeof(payload), true, service::fastTick());

    BenchResult result;
    const uint32_t start_cycles = readCycles();
    const uint32_t start_usec   = static_cast<uint32_t>(m5::utility::micros());
    for (;;) {
        const auto now = service::fastTick();
        const auto r   = transaction.service(service::ServiceContext{now});
        ++result.calls;
        if (r == service::ServiceResult::Idle) {
            ++result.idle_calls;
        } else if (r == service::ServiceResult::Done) {
            break;
        } else if (r == service::ServiceResult::Error) {
            break;
        }
    }
    const uint32_t end_usec   = static_cast<uint32_t>(m5::utility::micros());
    const uint32_t end_cycles = readCycles();

    result.cycles    = end_cycles - start_cycles;
    result.usec      = end_usec - start_usec;
    result.scl_rises = lines.scl_rises;
    result.writes    = lines.writes;
    result.reads     = lines.reads;
    g_sink32 += result.calls + result.scl_rises + transaction.transferred() + payload[0];
    return result;
}

void printResult(const char* name, const BenchResult& result)
{
    const uint32_t cycles_per_call = result.calls ? (result.cycles / result.calls) : 0;
    const uint32_t ns_per_call =
        result.calls ? static_cast<uint32_t>((static_cast<uint64_t>(result.usec) * 1000ull) / result.calls) : 0;
    const uint32_t khz = (result.usec && result.scl_rises)
                             ? static_cast<uint32_t>((static_cast<uint64_t>(result.scl_rises) * 1000ull) / result.usec)
                             : 0;

    Serial.printf(
        "%-28s total=%lu us calls=%lu idle=%lu cycles/call=%lu ns/call=%lu scl=%lu est=%lu kHz io(w/r)="
        "%lu/%lu\n",
        name, static_cast<unsigned long>(result.usec), static_cast<unsigned long>(result.calls),
        static_cast<unsigned long>(result.idle_calls), static_cast<unsigned long>(cycles_per_call),
        static_cast<unsigned long>(ns_per_call), static_cast<unsigned long>(result.scl_rises),
        static_cast<unsigned long>(khz), static_cast<unsigned long>(result.writes),
        static_cast<unsigned long>(result.reads));
}

void runBenchmark()
{
    Serial.println();
    Serial.println("I2CHotPathBenchmark");
    Serial.printf("CPU MHz: %lu\n", static_cast<unsigned long>(cpuMhz()));
    Serial.printf("M5HAL fastTick Hz: %lu\n", static_cast<unsigned long>(service::fastTickFrequencyHz()));
    Serial.printf("target software I2C freq: %lu Hz\n", static_cast<unsigned long>(kTargetFreqHz));
    Serial.printf("payload bytes: %lu\n", static_cast<unsigned long>(kPayloadBytes));

    printResult("fastTickNsec loop", measureFastTickNsecLoop());
    printResult("fastTick loop", measureFastTickLoop());
    printResult("virtual line ops", measureVirtualLineOps());
    printResult("write buffer forced due", measureForcedDueWriteBuffer());
    printResult("write buffer timed", measureTimedWriteBuffer());
    printResult("read buffer forced due", measureForcedDueReadBuffer());
    printResult("read buffer timed", measureTimedReadBuffer());

    Serial.printf("sink=%lu\n", static_cast<unsigned long>(g_sink32));
    Serial.println("I2CHotPathBenchmark done.");
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    delay(1000);
    runBenchmark();
}

void loop()
{
    delay(1000);
}
