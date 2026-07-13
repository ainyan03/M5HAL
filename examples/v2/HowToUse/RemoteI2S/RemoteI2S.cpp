// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — HowToUseRemoteI2S
//
// POSIX PC host-side example: connect to an ESP32 running RemoteServerTCP
// or RemoteServer firmware and stream 16-bit sine-wave audio to the remote
// I2S speaker path through the Hal facade.
//
// Build & run (PlatformIO):
//   pio run -e HowToUse_RemoteI2S_host
//   .pio/build/HowToUse_RemoteI2S_host/program <endpoint> [seconds] [tone_hz] [rate] [channels] [bclk ws dout]
//   [chunk_kib]
//
// Usage:
//   program <endpoint> [seconds] [tone_hz] [rate] [channels] [bclk ws dout] [chunk_kib]
//     endpoint : tcp:<host>:<port> | uart:<path>
//     seconds  : playback duration (default 10)
//     tone_hz  : sine frequency; 0 = silent soak (default 440)
//     rate     : sample rate (default 44100)
//     channels : 1|2 (default 2)
//     bclk ws dout : I2S pins (default 12 0 2 = Core2 speaker)
//     chunk_kib: bytes per write() in KiB, 1..64 (default 16)
//
// The device side does not need a dedicated RemoteI2S sketch. Flash the
// generic RemoteServerTCP or RemoteServer firmware; this host program powers
// the Core2 V1.1 speaker amplifier via remote I2C before acquiring remote I2S.
//
// This example is POSIX-only (macOS / Linux). It is not an Arduino sketch.
// =============================================================================

#if !defined(ARDUINO)

#include <M5HAL_v2.hpp>

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <limits>
#include <memory>

namespace m5hal = m5::hal::v2;

namespace {

constexpr uint32_t DEFAULT_SECONDS  = 10;
constexpr uint32_t DEFAULT_TONE_HZ  = 440;
constexpr uint32_t DEFAULT_RATE     = 44100;
constexpr uint32_t DEFAULT_CHANNELS = 2;

constexpr uint8_t AXP2101_ADDR = 0x34;
constexpr size_t TX_DMA_BYTES  = 24576;
// Each write() is one remote transaction (request + stream + response), so
// the chunk size sets the round-trip amortization: small chunks pay the
// per-transaction overhead more often. Sweeping [chunk_kib] and comparing
// the per-write latency stats separates that fixed cost from the streaming
// rate the wire itself sustains.
constexpr size_t DEFAULT_CHUNK_BYTES = 16384;
constexpr size_t MAX_CHUNK_BYTES     = 65536;
constexpr int16_t AMPLITUDE          = 8000;
constexpr double PI                  = 3.14159265358979323846;

void printUsage(const char* program)
{
    ::fprintf(stderr, "usage: %s <endpoint> [seconds] [tone_hz] [rate] [channels] [bclk ws dout] [chunk_kib]\n",
              program);
    ::fprintf(stderr, "  endpoint : tcp:<host>:<port> | uart:<path>\n");
    ::fprintf(stderr, "  seconds  : playback duration (default 10)\n");
    ::fprintf(stderr, "  tone_hz  : sine frequency; 0 = silent soak (default 440)\n");
    ::fprintf(stderr, "  rate     : sample rate (default 44100)\n");
    ::fprintf(stderr, "  channels : 1|2 (default 2)\n");
    ::fprintf(stderr, "  bclk ws dout : I2S pins (default 12 0 2 = Core2 speaker)\n");
    ::fprintf(stderr, "  chunk_kib: bytes per write() in KiB, 1..64 (default 16)\n");
}

bool parseU32(const char* text, uint32_t min_value, uint32_t max_value, uint32_t* out)
{
    if (text == nullptr || *text == '\0' || out == nullptr) {
        return false;
    }
    errno      = 0;
    char* end  = nullptr;
    auto value = ::strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value > std::numeric_limits<uint32_t>::max()) {
        return false;
    }
    if (value < min_value || value > max_value) {
        return false;
    }
    *out = static_cast<uint32_t>(value);
    return true;
}

double monotonicSeconds(void)
{
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0.0;
    }
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) / 1000000000.0;
}

void fillAudio(uint8_t* buf, size_t frames, uint32_t rate, uint32_t channels, uint32_t tone_hz, uint32_t* phase)
{
    auto* out = reinterpret_cast<int16_t*>(buf);
    for (size_t i = 0; i < frames; ++i) {
        int16_t sample = 0;
        if (tone_hz != 0) {
            const double angle =
                2.0 * PI * static_cast<double>(tone_hz) * static_cast<double>(*phase) / static_cast<double>(rate);
            sample = static_cast<int16_t>(static_cast<double>(AMPLITUDE) * ::sin(angle));
        }
        for (uint32_t ch = 0; ch < channels; ++ch) {
            out[i * channels + ch] = sample;
        }
        ++(*phase);
        if (*phase >= rate) {
            *phase = 0;
        }
    }
}

void warnResult(const char* label, m5hal::error::error_t err)
{
    ::fprintf(stderr, "warning: %s failed: %s (%d)\n", label, m5hal::error::toString(err), static_cast<int>(err));
}

void initCore2V11Amplifier(m5hal::Hal& hal)
{
    auto cfg = hal.I2C.createBusConfig(m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21});
    auto bus = hal.I2C.acquire(cfg);
    if (!bus.has_value()) {
        warnResult("I2C acquire for amplifier init", bus.error());
        return;
    }

    bool ok   = true;
    uint8_t v = 0;
    {
        m5hal::i2c::MasterAccessConfig acc_cfg;
        acc_cfg.i2c_addr = AXP2101_ADDR;
        acc_cfg.freq     = 400000;
        m5hal::i2c::MasterAccessor axp{bus.value(), acc_cfg};

        auto w94 = axp.writeRegister(0x94, 0x1C);
        if (!w94.has_value()) {
            warnResult("AXP2101 reg 0x94 write", w94.error());
            ok = false;
        }
        auto r90 = axp.readRegister(0x90, &v, 1);
        if (!r90.has_value()) {
            warnResult("AXP2101 reg 0x90 read", r90.error());
            ok = false;
        }
        auto w90 = axp.writeRegister(0x90, static_cast<uint8_t>(v | 0x04));
        if (!w90.has_value()) {
            warnResult("AXP2101 reg 0x90 write", w90.error());
            ok = false;
        }
    }
    if (ok) {
        ::printf("Core2 V1.1 amplifier power enabled (AXP2101 ALDO3)\n");
    }

    auto rel = hal.I2C.release(bus.value());
    if (!rel.has_value()) {
        warnResult("I2C release after amplifier init", rel.error());
    }
}

int streamTone(const std::shared_ptr<m5hal::i2s::IBus>& bus, uint32_t seconds, uint32_t tone_hz, uint32_t rate,
               uint32_t channels, size_t chunk_bytes)
{
    m5hal::i2s::AccessConfig acc_cfg;
    acc_cfg.sample_rate_hz  = rate;
    acc_cfg.bits_per_sample = 16;
    acc_cfg.channels        = static_cast<uint8_t>(channels);
    m5hal::i2s::TxAccessor tx{bus, acc_cfg};

    const size_t frame_bytes = static_cast<size_t>(channels) * sizeof(int16_t);
    const size_t frames      = chunk_bytes / frame_bytes;
    const size_t bytes       = frames * frame_bytes;
    std::unique_ptr<uint8_t[]> chunk{new uint8_t[bytes]};
    size_t offset       = bytes;
    uint32_t phase      = 0;
    uint64_t total_sent = 0;
    uint64_t last_sent  = 0;

    // Per-write latency stats for the 1 s stat window: separates the fixed
    // per-transaction cost (visible at small chunks) from wire streaming
    // throughput (dominant at large chunks).
    uint32_t w_count = 0;
    double w_sum_ms  = 0.0;
    double w_max_ms  = 0.0;

    const uint64_t nominal = static_cast<uint64_t>(rate) * static_cast<uint64_t>(channels) * sizeof(int16_t);
    const double start     = monotonicSeconds();
    double next_stat       = 1.0;
    uint64_t verdict_sent  = 0;

    ::printf("streaming %u Hz, %u channel(s), %u Hz sample rate for %u second(s), %u B/write\n",
             static_cast<unsigned>(tone_hz), static_cast<unsigned>(channels), static_cast<unsigned>(rate),
             static_cast<unsigned>(seconds), static_cast<unsigned>(bytes));

    for (;;) {
        const double elapsed = monotonicSeconds() - start;
        if (elapsed >= static_cast<double>(seconds)) {
            break;
        }
        while (elapsed >= next_stat && next_stat <= static_cast<double>(seconds)) {
            const uint64_t inst = total_sent - last_sent;
            const uint64_t avg =
                next_stat > 0.0 ? static_cast<uint64_t>(static_cast<double>(total_sent) / next_stat) : uint64_t{0};
            const double w_avg_ms = w_count > 0 ? w_sum_ms / static_cast<double>(w_count) : 0.0;
            ::printf("t=%us sent=%lluB avg=%lluB/s inst=%lluB/s nominal=%lluB/s writes=%u wavg=%.1fms wmax=%.1fms\n",
                     static_cast<unsigned>(next_stat), static_cast<unsigned long long>(total_sent),
                     static_cast<unsigned long long>(avg), static_cast<unsigned long long>(inst),
                     static_cast<unsigned long long>(nominal), static_cast<unsigned>(w_count), w_avg_ms, w_max_ms);
            if (seconds <= 2 || next_stat > 2.0) {
                verdict_sent += inst;
            }
            last_sent = total_sent;
            next_stat += 1.0;
            w_count  = 0;
            w_sum_ms = 0.0;
            w_max_ms = 0.0;
        }

        // Pace the sender to the playback rate (plus a DMA prefill worth of
        // lead). The device accepts data faster than the I2S clock drains it;
        // an unpaced sender overruns the device DMA and the excess is lost.
        const uint64_t budget =
            static_cast<uint64_t>(elapsed * static_cast<double>(nominal)) + static_cast<uint64_t>(TX_DMA_BYTES);
        if (total_sent >= budget) {
            ::usleep(5000);
            continue;
        }

        if (offset >= bytes) {
            fillAudio(chunk.get(), frames, rate, channels, tone_hz, &phase);
            offset = 0;
        }

        const double w_begin = monotonicSeconds();
        auto r               = tx.write(chunk.get() + offset, bytes - offset);
        const double w_ms    = (monotonicSeconds() - w_begin) * 1000.0;
        ++w_count;
        w_sum_ms += w_ms;
        if (w_ms > w_max_ms) {
            w_max_ms = w_ms;
        }
        if (!r.has_value()) {
            ::fprintf(stderr, "I2S write failed: %s (%d)\n", m5hal::error::toString(r.error()),
                      static_cast<int>(r.error()));
            return 2;
        }
        if (r.value() == 0) {
            ::usleep(1000);
            continue;
        }
        offset += r.value();
        total_sent += r.value();
    }

    while (next_stat <= static_cast<double>(seconds)) {
        const uint64_t inst = total_sent - last_sent;
        const uint64_t avg =
            next_stat > 0.0 ? static_cast<uint64_t>(static_cast<double>(total_sent) / next_stat) : uint64_t{0};
        const double w_avg_ms = w_count > 0 ? w_sum_ms / static_cast<double>(w_count) : 0.0;
        ::printf("t=%us sent=%lluB avg=%lluB/s inst=%lluB/s nominal=%lluB/s writes=%u wavg=%.1fms wmax=%.1fms\n",
                 static_cast<unsigned>(next_stat), static_cast<unsigned long long>(total_sent),
                 static_cast<unsigned long long>(avg), static_cast<unsigned long long>(inst),
                 static_cast<unsigned long long>(nominal), static_cast<unsigned>(w_count), w_avg_ms, w_max_ms);
        if (seconds <= 2 || next_stat > 2.0) {
            verdict_sent += inst;
        }
        last_sent = total_sent;
        next_stat += 1.0;
        w_count  = 0;
        w_sum_ms = 0.0;
        w_max_ms = 0.0;
    }

    const uint32_t verdict_seconds = seconds > 2 ? seconds - 2 : seconds;
    const uint64_t verdict_avg     = verdict_seconds > 0 ? verdict_sent / verdict_seconds : uint64_t{0};
    const uint64_t low             = nominal * 985 / 1000;
    const uint64_t high            = nominal * 1015 / 1000;
    if (verdict_avg >= low && verdict_avg <= high) {
        ::printf("RESULT: CONVERGED\n");
        return 0;
    }
    ::printf("RESULT: STARVED (avg=%llu nominal=%llu)\n", static_cast<unsigned long long>(verdict_avg),
             static_cast<unsigned long long>(nominal));
    return 3;
}

}  // namespace

int main(int argc, char** argv)
{
    ::setvbuf(stdout, nullptr, _IOLBF, 0);

    if (argc < 2 || argc > 10) {
        printUsage(argv[0]);
        return 1;
    }

    const char* endpoint = argv[1];
    if (::strncmp(endpoint, "tcp:", 4) != 0 && ::strncmp(endpoint, "uart:", 5) != 0) {
        ::fprintf(stderr, "endpoint must start with tcp: or uart:\n");
        printUsage(argv[0]);
        return 1;
    }

    uint32_t seconds   = DEFAULT_SECONDS;
    uint32_t tone_hz   = DEFAULT_TONE_HZ;
    uint32_t rate      = DEFAULT_RATE;
    uint32_t channels  = DEFAULT_CHANNELS;
    uint32_t bclk      = 12;  // Core2 V1.1 speaker path (NS4168)
    uint32_t ws        = 0;
    uint32_t dout      = 2;
    uint32_t chunk_kib = static_cast<uint32_t>(DEFAULT_CHUNK_BYTES / 1024);
    if ((argc > 2 && !parseU32(argv[2], 1, 3600, &seconds)) || (argc > 3 && !parseU32(argv[3], 0, 20000, &tone_hz)) ||
        (argc > 4 && !parseU32(argv[4], 1, 384000, &rate)) || (argc > 5 && !parseU32(argv[5], 1, 2, &channels)) ||
        (argc > 6 && !parseU32(argv[6], 0, 48, &bclk)) || (argc > 7 && !parseU32(argv[7], 0, 48, &ws)) ||
        (argc > 8 && !parseU32(argv[8], 0, 48, &dout)) ||
        (argc > 9 && !parseU32(argv[9], 1, static_cast<uint32_t>(MAX_CHUNK_BYTES / 1024), &chunk_kib))) {
        printUsage(argv[0]);
        return 1;
    }

    m5hal::Hal remote;
    auto connected = remote.connect(endpoint);
    if (!connected.has_value()) {
        ::fprintf(stderr, "connect(%s) failed: %s (%d)\n", endpoint, m5hal::error::toString(connected.error()),
                  static_cast<int>(connected.error()));
        return 1;
    }

    const auto* caps = remote.capabilities();
    if (caps != nullptr) {
        ::printf("Connected to %s  (GPIO: %s, buses: %u)\n", endpoint, caps->has_gpio ? "yes" : "no",
                 static_cast<unsigned>(caps->bus_count));
    } else {
        ::printf("Connected to %s\n", endpoint);
    }

    initCore2V11Amplifier(remote);

    m5hal::i2s::BusConfig_remote i2s_cfg;
    i2s_cfg.pin_bclk       = static_cast<m5hal::types::gpio_number_t>(bclk);
    i2s_cfg.pin_ws         = static_cast<m5hal::types::gpio_number_t>(ws);
    i2s_cfg.pin_dout       = static_cast<m5hal::types::gpio_number_t>(dout);
    i2s_cfg.tx_buffer_size = TX_DMA_BYTES;
    auto i2s_bus           = remote.I2S.acquire(i2s_cfg);
    if (!i2s_bus.has_value()) {
        ::fprintf(stderr, "I2S acquire failed: %s (%d)\n", m5hal::error::toString(i2s_bus.error()),
                  static_cast<int>(i2s_bus.error()));
        return 1;
    }
    ::printf("I2S acquired (BCLK=%u, WS=%u, DOUT=%u, TX DMA=%u bytes)\n", static_cast<unsigned>(bclk),
             static_cast<unsigned>(ws), static_cast<unsigned>(dout), static_cast<unsigned>(TX_DMA_BYTES));

    int rc = streamTone(i2s_bus.value(), seconds, tone_hz, rate, channels, static_cast<size_t>(chunk_kib) * 1024);

    auto rel = remote.I2S.release(i2s_bus.value());
    if (!rel.has_value()) {
        warnResult("I2S release", rel.error());
    }
    return rc;
}

#endif  // !ARDUINO
