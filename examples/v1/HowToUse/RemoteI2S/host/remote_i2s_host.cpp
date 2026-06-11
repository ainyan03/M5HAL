// SPDX-License-Identifier: MIT
//
// M5HAL — HowToUseRemoteI2S (host side, PC)
//
// Streams a 440 Hz sine wave (16-bit, 24 kHz mono by default) to the
// speaker of a board running the RemoteI2S.ino sketch, over the USB serial
// cable, as if the I2S bus were local. Connection setup is two calls
// (spec/design/remote.md §接続ユーティリティ):
//
//   SerialRemoteEndpoint ep{baud};       // posix Bus + accessors + RemoteLink
//   connectRemoteSerial(ep, opt);        // enumerate ports x hello until a peer answers
//
// The remote I2S bus is a normal i2s::I2SBus: the RemoteI2SBus proxy paces
// NORESP bus_write_stream bursts by stream credit (spec §stream credit), so a
// continuous `write` keeps the speaker fed without per-chunk round-trips.
//
// Build & run (from the M5HAL repository root):
//   pio run -e HowToUse_RemoteI2SHost_native
//   .pio/build/HowToUse_RemoteI2SHost_native/program          # auto-discover the port
//   .pio/build/HowToUse_RemoteI2SHost_native/program <port> [baud] [rate] [channels] [window]
//                       # defaults: 3000000 baud, 24000 Hz, 1 ch, library window
//
// The per-second stat line doubles as a measurement tool: the closed credit
// loop paces writes at the device's true consumption, so `avg` converges on
// the effective I2S byte rate (rate x channels x 2 when everything is
// healthy) without needing ears or a scope.
//
// (This file lives in a subfolder of the sketch, so the Arduino IDE does not
//  try to compile it for the device.)

#if !defined(ARDUINO)

#include <M5HAL_v1.hpp>

#include <cmath>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

namespace m5hal      = m5::hal::v1;
namespace posix_uart = m5hal::uart::variant::posix;  // the posix UART variant alias

namespace {

// Defaults: 24 kHz / 16-bit / mono = 48 KB/s on the wire (M5Stack speakers
// are mono, so a mono wire buys quality, not loses it; the device side
// duplicates each sample into both I2S slots). Measured envelope on Core2
// V1.1 (CH9102 bridge, 3 Mbaud, direct USB port, drain-rate verified):
// every (rate, channels) combination drains at its nominal byte rate; the
// full request/acknowledge stack sustains ~75-120 KB/s depending on USB
// state (a USB hub costs another ~30 %). 24k mono leaves real margin even
// behind a hub; 32k mono (64 KB/s) showed occasional dips on a direct port
// (mono halves the device buffer in audio time, so it rides ack clumps
// with less slack); 44.1k mono (88.2 KB/s) and 32k stereo (128 KB/s) need
// a quieter link than the current stack sustains. Rate note: prefer rates
// whose PLL_160M divider is clean on the classic ESP32 (16k/24k/32k
// families; 22.05 kHz dithers audibly).
uint32_t SAMPLE_RATE            = 24000;
uint32_t CHANNELS               = 1;
constexpr uint32_t SINE_FREQ_HZ = 440;
constexpr size_t CHUNK_SAMPLES  = 2048;  // samples per channel per chunk
size_t CHUNK_BYTES              = 0;     // set in main: CHUNK_SAMPLES * CHANNELS * 2

uint32_t g_phase = 0;

// Fill `buf` (CHUNK_BYTES) with 16-bit 440 Hz sine at -6 dBFS.
void fillSineChunk(uint8_t* buf)
{
    int16_t* p = reinterpret_cast<int16_t*>(buf);
    for (size_t i = 0; i < CHUNK_SAMPLES; ++i) {
        const float angle = 2.0f * static_cast<float>(M_PI) * static_cast<float>(SINE_FREQ_HZ) *
                            static_cast<float>(g_phase) / static_cast<float>(SAMPLE_RATE);
        const int16_t sample = static_cast<int16_t>(16383.0f * sinf(angle));
        for (uint32_t ch = 0; ch < CHANNELS; ++ch) {
            p[i * CHANNELS + ch] = sample;
        }
        if (++g_phase >= SAMPLE_RATE) {
            g_phase = 0;
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    // Keep progress visible even when stdout is a pipe.
    ::setvbuf(stdout, nullptr, _IOLBF, 0);

    const char* port    = argc > 1 ? argv[1] : ::getenv("M5HAL_POSIX_UART_PORT");
    const uint32_t baud = argc > 2 ? static_cast<uint32_t>(::strtoul(argv[2], nullptr, 10)) : 3000000;
    if (argc > 3) {
        SAMPLE_RATE = static_cast<uint32_t>(::strtoul(argv[3], nullptr, 10));
    }
    if (argc > 4) {
        CHANNELS = static_cast<uint32_t>(::strtoul(argv[4], nullptr, 10));
    }
    // argv[5]: in-flight window override in bytes (0/absent = library default).
    const uint32_t window = argc > 5 ? static_cast<uint32_t>(::strtoul(argv[5], nullptr, 10)) : 0;
    CHUNK_BYTES           = CHUNK_SAMPLES * CHANNELS * sizeof(int16_t);

    // Connect: explicit port when given, auto-discovery otherwise.
    posix_uart::SerialRemoteEndpoint ep{baud};
    posix_uart::ConnectOptions opt;
    opt.path       = port;  // nullptr -> walk the listSerialPorts candidates
    opt.on_attempt = [](void*, const char* p) { ::printf("trying %s ...\n", p); };
    auto caps      = posix_uart::connectRemoteSerial(ep, opt);
    if (!caps.has_value()) {
        ::fprintf(stderr, "no RemoteI2S device found: %d (is RemoteI2S.ino running?)\n",
                  static_cast<int>(caps.error()));
        return 1;
    }
    ::printf("using %s @ %u baud\n", ep.devicePath(), static_cast<unsigned>(baud));
    ::printf("remote proto v%u, %u bus(es)\n", caps.value().proto_ver, static_cast<unsigned>(caps.value().bus_count));

    // Find the published I2S bus in the capability list.
    int i2s_bus_id = -1;
    for (size_t i = 0; i < caps.value().bus_count; ++i) {
        if (caps.value().buses[i].kind == m5hal::types::bus_kind_t::I2S) {
            i2s_bus_id = caps.value().buses[i].bus_id;
            break;
        }
    }
    if (i2s_bus_id < 0) {
        ::fprintf(stderr, "remote device publishes no I2S bus\n");
        return 1;
    }
    ::printf("streaming 440 Hz sine to remote I2S bus %d (Ctrl+C to stop)\n", i2s_bus_id);

    // The proxy is a normal i2s::I2SBus; the accessor's write() just works.
    m5hal::remote::RemoteI2SBus remote_i2s{ep.link.session(), static_cast<uint8_t>(i2s_bus_id)};
    if (window > 0) {
        remote_i2s.setStreamWindow(window);
        ::printf("stream window override: %u bytes\n", static_cast<unsigned>(window));
    }
    m5hal::i2s::I2SAccessConfig acc_cfg;
    acc_cfg.sample_rate_hz   = SAMPLE_RATE;
    acc_cfg.bits_per_sample  = 16;
    acc_cfg.channels         = static_cast<uint8_t>(CHANNELS);
    acc_cfg.write_timeout_ms = 100;
    m5hal::i2s::I2STxAccessor dev{remote_i2s, acc_cfg};

    uint8_t chunk[CHUNK_SAMPLES * 2 * sizeof(int16_t)];  // max channels = 2
    unsigned long total_sent = 0;

    // Pre-roll: fill the device buffer with silence before the tone starts.
    // The first writes play back while the pipeline is still shallow, so any
    // start-up micro-underruns land on silence instead of audible audio.
    ::memset(chunk, 0, CHUNK_BYTES);
    for (size_t fill = 0; fill < 32768; fill += CHUNK_BYTES) {
        (void)dev.write(chunk, CHUNK_BYTES);
    }

    // Stats baseline AFTER the pre-roll: from here on the closed credit loop
    // paces writes at the device's true drain rate, so the cumulative average
    // converges on the real consumption (bytes/s) — an ear-free measure of
    // the effective sample clock.
    const uint32_t t0  = static_cast<uint32_t>(m5::utility::millis());
    uint32_t last_stat = t0;
    unsigned long last_sent = 0;

    // The write loop is regulated by the credit/in-flight window itself: the
    // window is sized close to the device's DMA capacity, so "send whenever
    // the window allows" keeps the remote buffer pinned near full — a closed
    // feedback loop on actual buffer level. (An open-loop, exactly-1x paced
    // sender was tried first; it has zero slack, so any acknowledgement clump
    // turns into a permanent buffer deficit and audible dropouts.)
    for (;;) {
        // Drain pending acks (credit events) BEFORE staging more data: this
        // keeps the in-flight window sliding instead of bursting a full
        // window and then stalling for the whole acknowledgement round-trip.
        // poll(1) per pass: only consume frames that are already buffered —
        // an unbounded poll() blocks for the NEXT frame too, and with a
        // steady event cadence that captures this loop (see session.poll docs).
        while (true) {
            auto pending = ep.rx.readableBytes();
            if (!pending.has_value() || pending.value() == 0) {
                break;
            }
            (void)ep.link.session().poll(1);
        }

        fillSineChunk(chunk);
        auto w = dev.write(chunk, CHUNK_BYTES);
        if (!w.has_value()) {
            // Transient remote errors (e.g. a timed-out status re-sync) are
            // worth retrying: the session resyncs itself on the next exchange.
            ::fprintf(stderr, "remote I2S write failed: %d (retrying)\n", static_cast<int>(w.error()));
            ::usleep(200 * 1000);
            continue;
        }
        total_sent += w.value();

        const uint32_t now = static_cast<uint32_t>(m5::utility::millis());
        if (now - last_stat >= 1000) {
            auto credit       = dev.writableBytes();
            const long cval   = credit.has_value() ? static_cast<long>(credit.value()) : -1;
            const uint32_t el = now - t0;
            const unsigned long avg  = (el > 0) ? (total_sent * 1000UL) / el : 0;
            const unsigned long inst = ((total_sent - last_sent) * 1000UL) / (now - last_stat);
            ::printf("t=%u.%03us sent=%lu  avg=%luB/s inst=%luB/s  credit=%ld  lastRemoteError=%d\n", el / 1000,
                     el % 1000, total_sent, avg, inst, cval, static_cast<int>(ep.link.session().lastRemoteError()));
            last_stat = now;
            last_sent = total_sent;
        }
    }

    return 0;
}

#endif  // !ARDUINO
