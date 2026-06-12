// SPDX-License-Identifier: MIT
//
// M5HAL — RemoteMenu (host side, PC)
//
// Integrated stdin menu CLI for the remote bus acceptance harness.
// Combines the former RemoteBus / RemoteI2S example host programs into one
// TU (their device sketches are folded into ../device/remote_menu_device.cpp).
//
// Build & run (from the M5HAL repository root):
//   pio run -e v1_exp_remote_host
//   .pio/build/v1_exp_remote_host/program                       # serial auto-discovery (115200)
//   .pio/build/v1_exp_remote_host/program <port> 3000000        # espidf device
//   .pio/build/v1_exp_remote_host/program tcp:<board-ip>:5555   # TCP device
//
// (This file lives in a subfolder of the sketch, so the Arduino IDE does
//  not try to compile it for the device.)

#if !defined(ARDUINO)

#include <M5HAL_v1.hpp>

#include <cmath>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

namespace m5hal      = m5::hal::v1;
namespace posix_uart = ::m5::variants::frameworks::posix::hal::v1::uart;
// The TCP transport is not a bus kind, so it has no `variant::` alias —
// name the posix variant namespace directly.
namespace posix_tcp = m5::variants::frameworks::posix::hal::v1::tcp;

// ---------------------------------------------------------------------------
// Global connection state (set once in main, read by command handlers)
// ---------------------------------------------------------------------------
namespace {

m5hal::remote::RemoteSession* g_session = nullptr;
m5hal::data::StreamReader* g_link_rx    = nullptr;
bool g_serial_link                      = false;
m5hal::remote::Capabilities g_caps;

// ---------------------------------------------------------------------------
// Sine-wave state (reset at the start of each tone/soak call)
// ---------------------------------------------------------------------------
static uint32_t s_sample_rate  = 44100;
static uint32_t s_channels     = 2;
static uint32_t s_sine_freq_hz = 440;
constexpr size_t kChunkSamples = 2048;
static size_t s_chunk_bytes    = 0;  // set before each tone run

static uint32_t s_phase = 0;

// Fill `buf` with `samples` frames of 16-bit sine at -6 dBFS (per channel).
void fillSine(uint8_t* buf, size_t samples)
{
    int16_t* p = reinterpret_cast<int16_t*>(buf);
    for (size_t i = 0; i < samples; ++i) {
        const float angle = 2.0f * static_cast<float>(M_PI) * static_cast<float>(s_sine_freq_hz) *
                            static_cast<float>(s_phase) / static_cast<float>(s_sample_rate);
        const int16_t sample = static_cast<int16_t>(16383.0f * sinf(angle));
        for (uint32_t ch = 0; ch < s_channels; ++ch) {
            p[i * s_channels + ch] = sample;
        }
        if (++s_phase >= s_sample_rate) {
            s_phase = 0;
        }
    }
}

// ---------------------------------------------------------------------------
// stdin liveness probe: return true when Enter (or any byte) is pending
// ---------------------------------------------------------------------------
bool stdinHasData()
{
    struct pollfd pfd;
    pfd.fd      = STDIN_FILENO;
    pfd.events  = POLLIN;
    pfd.revents = 0;
    return ::poll(&pfd, 1, 0) > 0;
}

// ---------------------------------------------------------------------------
// Helper: parse up to two uint32 arguments from a space-separated string.
// Returns false and leaves *out unchanged if the token is absent.
// ---------------------------------------------------------------------------
static bool parseU32(const char* str, uint32_t* out)
{
    if (str == nullptr || *str == '\0') {
        return false;
    }
    char* end  = nullptr;
    uint32_t v = static_cast<uint32_t>(::strtoul(str, &end, 0));
    if (end == str) {
        return false;
    }
    *out = v;
    return true;
}

// Advance past the first whitespace-delimited token; return pointer to the
// start of the next token (or end-of-string).
static const char* nextToken(const char* p)
{
    while (*p != '\0' && *p != ' ' && *p != '\t') {
        ++p;
    }
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    return p;
}

// ---------------------------------------------------------------------------
// tone / soak shared implementation
// ---------------------------------------------------------------------------

// Run the streaming loop.
//   run_sec == 0  →  infinite (until Enter)
//   mute == true  →  write silence (for soak)
// Returns the number of bytes sent.
static unsigned long runToneLoop(uint32_t run_sec, bool mute, uint32_t rate, uint32_t ch, uint32_t window_override,
                                 uint32_t hz,
                                 // soak credit tracking: pre-allocated by caller
                                 uint32_t* credit_log, size_t credit_log_cap, size_t* credit_log_len_out,
                                 uint32_t* elapsed_ms_out = nullptr)
{
    // Find the I2S bus in the capability list.
    int i2s_bus_id = -1;
    for (size_t i = 0; i < g_caps.bus_count; ++i) {
        if (g_caps.buses[i].kind == m5hal::types::bus_kind_t::I2S) {
            i2s_bus_id = g_caps.buses[i].bus_id;
            break;
        }
    }
    if (i2s_bus_id < 0) {
        ::printf("device publishes no I2S bus (Core BASIC means no speaker I2S)\n");
        return 0;
    }
    // The chunk buffer below is sized for 2 channels — more would overflow it.
    if (ch < 1 || ch > 2) {
        ::printf("channels must be 1 or 2\n");
        return 0;
    }

    // lastRemoteError is sticky (a denied gpio-write would otherwise bleed
    // into this run's verdict) — start the stream with a clean slate.
    g_session->clearRemoteError();

    // Set up globals consumed by fillSine.
    s_sample_rate  = rate;
    s_channels     = ch;
    s_sine_freq_hz = mute ? 0 : hz;
    s_phase        = 0;
    s_chunk_bytes  = kChunkSamples * s_channels * sizeof(int16_t);

    // Build the remote proxy and accessor.
    m5hal::remote::RemoteI2SBus remote_i2s{*g_session, static_cast<uint8_t>(i2s_bus_id)};
    if (window_override > 0) {
        remote_i2s.setStreamWindow(window_override);
        ::printf("stream window override: %u bytes\n", static_cast<unsigned>(window_override));
    }
    m5hal::i2s::AccessConfig acc_cfg;
    acc_cfg.sample_rate_hz   = s_sample_rate;
    acc_cfg.bits_per_sample  = 16;
    acc_cfg.channels         = static_cast<uint8_t>(s_channels);
    acc_cfg.write_timeout_ms = 100;
    m5hal::i2s::TxAccessor dev{remote_i2s, acc_cfg};

    uint8_t chunk[kChunkSamples * 2 * sizeof(int16_t)];  // max channels = 2

    // Shape the stream for the USB-CDC bridge (serial only): the macOS CH9102
    // driver falls into a burst + dead-time delivery mode as soon as the tty
    // output queue saturates — ~57 % duty (170 KB/s of the 300 KB/s line rate
    // at 3 Mbaud) with 100-150 ms delivery gaps that outrun the device's DMA
    // backlog and underrun it. The same wire sustains 256 KB/s with zero gaps
    // when fed small writes below the smooth ceiling, so pace the sender with
    // a token bucket: 2 KiB slices at 210 KB/s payload — above every supported
    // rate's nominal demand, below the driver's collapse threshold.
    const size_t slice_bytes          = g_serial_link ? 2048u : s_chunk_bytes;
    const size_t slice_samples        = slice_bytes / (s_channels * sizeof(int16_t));
    constexpr uint32_t kPaceBytesPerS = 210000;
    constexpr uint64_t kPaceBurst     = 4096;
    uint64_t pace_budget              = kPaceBurst;
    uint32_t pace_last_ms             = static_cast<uint32_t>(m5::utility::millis());

    // Pre-roll: fill the device buffer with silence before the tone starts.
    // Paced like the main loop — an unpaced 32 KiB burst would drop the
    // bridge into its dead-time mode right at stream start.
    ::memset(chunk, 0, s_chunk_bytes);
    for (size_t fill = 0; fill < 32768; fill += slice_bytes) {
        if (g_serial_link) {
            ::usleep(static_cast<useconds_t>(slice_bytes * 1000000ULL / kPaceBytesPerS));
        }
        (void)dev.write(chunk, slice_bytes);
    }

    const uint32_t t0        = static_cast<uint32_t>(m5::utility::millis());
    uint32_t last_stat       = t0;
    unsigned long total_sent = 0;
    unsigned long last_sent  = 0;
    size_t credit_log_len    = 0;

    if (run_sec > 0) {
        ::printf("streaming to remote I2S bus %d for %us (stdin ignored — deterministic run)\n", i2s_bus_id,
                 static_cast<unsigned>(run_sec));
    } else {
        ::printf("streaming to remote I2S bus %d — press Enter to stop\n", i2s_bus_id);
    }

    for (;;) {
        // Drain pending acks (credit events) BEFORE staging more data: this
        // keeps the in-flight window sliding instead of bursting a full
        // window and then stalling for the whole acknowledgement round-trip.
        // poll(1) per pass: only consume frames that are already buffered —
        // an unbounded poll() blocks for the NEXT frame too, and with a
        // steady event cadence that captures this loop.
        while (true) {
            auto pending = g_link_rx->readableBytes();
            if (!pending.has_value() || pending.value() == 0) {
                break;
            }
            (void)g_session->poll(1);
        }

        // Token-bucket gate (serial only; see the pacing note above).
        if (g_serial_link) {
            const uint32_t now_ms = static_cast<uint32_t>(m5::utility::millis());
            pace_budget += static_cast<uint64_t>(now_ms - pace_last_ms) * kPaceBytesPerS / 1000;
            pace_last_ms = now_ms;
            if (pace_budget > kPaceBurst) {
                pace_budget = kPaceBurst;
            }
            if (pace_budget < slice_bytes) {
                (void)g_session->poll(1);
                // Check for Enter between pacing ticks — interactive
                // (open-ended) runs only: a timed run ignores stdin so
                // scripted/piped invocations are deterministic (queued
                // bytes such as a trailing 'quit' must not cut it short).
                if (run_sec == 0 && stdinHasData()) {
                    (void)::getchar();
                    break;
                }
                continue;
            }
            pace_budget -= slice_bytes;
        }

        if (mute) {
            ::memset(chunk, 0, slice_bytes);
        } else {
            fillSine(chunk, slice_samples);
        }
        auto w = dev.write(chunk, slice_bytes);
        if (!w.has_value()) {
            ::fprintf(stderr, "remote I2S write failed: %d (retrying)\n", static_cast<int>(w.error()));
            ::usleep(200 * 1000);
            continue;
        }
        total_sent += w.value();

        const uint32_t now = static_cast<uint32_t>(m5::utility::millis());
        if (now - last_stat >= 1000) {
            auto credit              = dev.writableBytes();
            const long cval          = credit.has_value() ? static_cast<long>(credit.value()) : -1;
            const uint32_t el        = now - t0;
            const unsigned long avg  = (el > 0) ? (total_sent * 1000UL) / el : 0;
            const unsigned long inst = ((total_sent - last_sent) * 1000UL) / (now - last_stat);
            ::printf("t=%u.%03us sent=%lu  avg=%luB/s inst=%luB/s  credit=%ld  lastRemoteError=%d\n", el / 1000,
                     el % 1000, total_sent, avg, inst, cval, static_cast<int>(g_session->lastRemoteError()));
            last_stat = now;
            last_sent = total_sent;

            // Record per-second credit for soak bucket analysis.
            if (credit_log != nullptr && credit_log_len < credit_log_cap && cval >= 0) {
                credit_log[credit_log_len++] = static_cast<uint32_t>(cval);
            }

            if (run_sec > 0 && el / 1000 >= run_sec) {
                break;
            }
        }

        // Check for Enter (non-blocking) to abort — interactive runs
        // only (see the pacing-tick check above for the rationale).
        if (run_sec == 0 && !g_serial_link && stdinHasData()) {
            (void)::getchar();
            break;
        }
    }

    if (credit_log_len_out != nullptr) {
        *credit_log_len_out = credit_log_len;
    }
    if (elapsed_ms_out != nullptr) {
        *elapsed_ms_out = static_cast<uint32_t>(m5::utility::millis()) - t0;
    }
    return total_sent;
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

void cmdCaps(char* /*args*/)
{
    ::printf("proto_ver=%u  bus_count=%u  has_gpio=%s\n", g_caps.proto_ver, static_cast<unsigned>(g_caps.bus_count),
             g_caps.has_gpio ? "yes" : "no");
    for (size_t i = 0; i < g_caps.bus_count; ++i) {
        ::printf("  bus %u: kind=%u\n", g_caps.buses[i].bus_id, static_cast<unsigned>(g_caps.buses[i].kind));
    }
}

void cmdPing(char* args)
{
    uint32_t n = 10;
    (void)parseU32(args, &n);
    if (n == 0) {
        n = 10;
    }

    unsigned long rtt_min = ULONG_MAX;
    unsigned long rtt_max = 0;
    unsigned long rtt_sum = 0;
    uint32_t ok           = 0;

    for (uint32_t i = 0; i < n; ++i) {
        const unsigned long t0 = static_cast<unsigned long>(m5::utility::micros());
        auto r                 = g_session->ping();
        const unsigned long t1 = static_cast<unsigned long>(m5::utility::micros());
        if (!r.has_value()) {
            ::printf("  ping %u: FAILED (%d)\n", i + 1, static_cast<int>(r.error()));
            continue;
        }
        const unsigned long rtt = t1 - t0;
        if (rtt < rtt_min) {
            rtt_min = rtt;
        }
        if (rtt > rtt_max) {
            rtt_max = rtt;
        }
        rtt_sum += rtt;
        ++ok;
    }
    if (ok > 0) {
        ::printf("ping %u/%u  min=%luus avg=%luus max=%luus\n", ok, n, rtt_min, rtt_sum / ok, rtt_max);
    } else {
        ::printf("all pings failed\n");
    }
}

void cmdScan(char* /*args*/)
{
    m5hal::remote::RemoteI2CBus remote_bus{*g_session, 0};
    m5hal::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.wire_timeout_ms = 100;
    m5hal::i2c::MasterAccessor dev{remote_bus, acc_cfg};

    ::printf("scanning remote I2C bus...\n");
    size_t found = 0;
    for (uint16_t addr = 0x08; addr <= 0x77; ++addr) {
        acc_cfg.i2c_addr = addr;
        if (!dev.setConfig(acc_cfg).has_value()) {
            continue;
        }
        if (dev.probe().has_value()) {
            ::printf("  0x%02X\n", addr);
            ++found;
        }
    }
    ::printf("scan done: %zu device(s)\n", found);
}

void cmdReg(char* args)
{
    // <addr> <reg> [n=1]
    if (args == nullptr || *args == '\0') {
        ::printf("usage: reg <addr> <reg> [n=1]\n");
        return;
    }

    char* end      = nullptr;
    uint32_t i2c_a = static_cast<uint32_t>(::strtoul(args, &end, 0));
    const char* p  = end;
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    if (*p == '\0') {
        ::printf("usage: reg <addr> <reg> [n=1]\n");
        return;
    }
    uint32_t reg_a = static_cast<uint32_t>(::strtoul(p, &end, 0));
    p              = end;
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    uint32_t nbytes = 1;
    if (*p != '\0') {
        nbytes = static_cast<uint32_t>(::strtoul(p, nullptr, 0));
    }
    if (nbytes == 0) {
        nbytes = 1;
    }
    if (nbytes > 32) {
        nbytes = 32;
    }

    m5hal::remote::RemoteI2CBus remote_bus{*g_session, 0};
    m5hal::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.wire_timeout_ms = 100;
    acc_cfg.i2c_addr   = static_cast<uint16_t>(i2c_a);
    m5hal::i2c::MasterAccessor dev{remote_bus, acc_cfg};

    // Read each byte via single-byte readRegister calls.
    ::printf("I2C 0x%02X reg 0x%02X [%u byte(s)]:", static_cast<unsigned>(i2c_a), static_cast<unsigned>(reg_a),
             static_cast<unsigned>(nbytes));
    bool any_err = false;
    for (uint32_t i = 0; i < nbytes; ++i) {
        auto r = dev.readRegister(static_cast<uint8_t>(reg_a + i));
        if (r.has_value()) {
            ::printf(" 0x%02X", static_cast<unsigned>(r.value()));
        } else {
            ::printf(" ERR(%d)", static_cast<int>(r.error()));
            any_err = true;
        }
    }
    ::printf("\n");
    if (any_err) {
        ::printf("  (some bytes failed — device present at 0x%02X?)\n", static_cast<unsigned>(i2c_a));
    }
}

void cmdGpioRead(char* /*args*/)
{
    if (!g_caps.has_gpio) {
        ::printf("device has no GPIO capability\n");
        return;
    }
    m5hal::remote::RemoteGPIO remote_gpio{*g_session, 40, 0};
    m5hal::gpio::GPIOGroup remote_pins;
    if (!remote_pins.addGPIO(&remote_gpio, 0).has_value()) {
        ::printf("addGPIO failed\n");
        return;
    }

    const uint8_t buttons[] = {39, 38, 37};
    const char* names[]     = {"A(39)", "B(38)", "C(37)"};
    ::printf("GPIO snapshot:");
    for (size_t i = 0; i < 3; ++i) {
        auto pin = remote_pins.tryGetPin(m5hal::types::makeGpioNumber(0, buttons[i]));
        if (!pin.has_value()) {
            ::printf("  %s=UNAVAIL", names[i]);
            continue;
        }
        pin.value().setMode(m5hal::types::gpio_mode_t::Input);
        ::printf("  %s=%d", names[i], pin.value().read() ? 1 : 0);
    }
    ::printf("\n");
}

void cmdGpioWrite(char* args)
{
    if (!g_caps.has_gpio) {
        ::printf("device has no GPIO capability\n");
        return;
    }
    // <pin> <0|1>
    if (args == nullptr || *args == '\0') {
        ::printf("usage: gpio-write <pin> <0|1>\n");
        return;
    }

    char* end      = nullptr;
    uint32_t pin_n = static_cast<uint32_t>(::strtoul(args, &end, 0));
    const char* p  = end;
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    if (*p == '\0') {
        ::printf("usage: gpio-write <pin> <0|1>\n");
        return;
    }
    const bool level = (::strtoul(p, nullptr, 0) != 0);

    m5hal::remote::RemoteGPIO remote_gpio{*g_session, 40, 0};
    m5hal::gpio::GPIOGroup remote_pins;
    if (!remote_pins.addGPIO(&remote_gpio, 0).has_value()) {
        ::printf("addGPIO failed\n");
        return;
    }

    auto pin =
        remote_pins.tryGetPin(m5hal::types::makeGpioNumber(0, static_cast<m5hal::types::gpio_local_pin_t>(pin_n)));
    if (!pin.has_value()) {
        ::printf("pin %u unavailable (not in device allowlist?)\n", static_cast<unsigned>(pin_n));
        return;
    }
    g_session->clearRemoteError();
    pin.value().setMode(m5hal::types::gpio_mode_t::Output);
    pin.value().write(level);

    // write/setMode go out as NORESP scripts: a server-side failure only
    // surfaces as lastRemoteError after a later exchange. Complete one RPC
    // round-trip so the verdict below is about THIS write.
    (void)g_session->ping();
    auto err = g_session->lastRemoteError();
    if (err != m5hal::error::error_t::OK) {
        ::printf("gpio-write pin %u = %d → error %d (allowlist outside {39,38,37}?)\n", static_cast<unsigned>(pin_n),
                 level ? 1 : 0, static_cast<int>(err));
    } else {
        ::printf("gpio-write pin %u = %d OK\n", static_cast<unsigned>(pin_n), level ? 1 : 0);
    }
}

void cmdWatch(char* /*args*/)
{
    if (!g_caps.has_gpio) {
        ::printf("device has no GPIO capability\n");
        return;
    }
    m5hal::remote::RemoteGPIO remote_gpio{*g_session, 40, 0};
    m5hal::gpio::GPIOGroup remote_pins;
    if (!remote_pins.addGPIO(&remote_gpio, 0).has_value()) {
        ::printf("addGPIO failed\n");
        return;
    }

    const uint8_t buttons[] = {39, 38, 37};
    const char* names[]     = {"A", "B", "C"};

    // Initial snapshot.
    for (size_t i = 0; i < 3; ++i) {
        auto pin = remote_pins.tryGetPin(m5hal::types::makeGpioNumber(0, buttons[i]));
        if (!pin.has_value()) {
            ::fprintf(stderr, "button %s unavailable\n", names[i]);
            return;
        }
        pin.value().setMode(m5hal::types::gpio_mode_t::Input);
        ::printf("%s%s=%d", i == 0 ? "initial (pressed=0): " : " ", names[i], pin.value().read() ? 1 : 0);
    }
    ::printf("\n");

    struct Ctx {
        const uint8_t* buttons;
        const char* const* names;
    } ctx{buttons, names};
    g_session->runner().setGpioEventHandler(
        [](void* c, m5hal::types::gpio_number_t pin, bool level) {
            auto* x             = static_cast<Ctx*>(c);
            const uint8_t local = static_cast<uint8_t>(m5hal::types::extractLocalPin(pin));
            for (size_t i = 0; i < 3; ++i) {
                if (x->buttons[i] == local) {
                    ::printf("button %s %s\n", x->names[i], level ? "released" : "pressed");
                    return;
                }
            }
        },
        &ctx);

    for (size_t i = 0; i < 3; ++i) {
        auto s = remote_gpio.subscribe(buttons[i]);
        if (!s.has_value()) {
            ::fprintf(stderr, "subscribe %s failed: %d\n", names[i], static_cast<int>(s.error()));
            // Unsubscribe already-subscribed pins before returning.
            for (size_t j = 0; j < i; ++j) {
                (void)remote_gpio.unsubscribe(buttons[j]);
            }
            return;
        }
    }

    ::printf("watching buttons A/B/C via push events — press Enter to stop\n");

    // poll() blocks for at most the rx stream's own timeout when idle, so this
    // loop self-paces; ping ~ every 2 s guards the link (events are best-effort
    // and carry no liveness signal).
    for (unsigned cycle = 0;; ++cycle) {
        (void)g_session->poll(1);
        if (cycle % 20 == 19 && !g_session->ping().has_value()) {
            ::fprintf(stderr, "connection lost\n");
            break;
        }
        if (stdinHasData()) {
            (void)::getchar();
            break;
        }
    }

    // Unsubscribe cleanly before returning to the menu.
    for (size_t i = 0; i < 3; ++i) {
        (void)remote_gpio.unsubscribe(buttons[i]);
    }
    g_session->runner().setGpioEventHandler(nullptr, nullptr);
    ::printf("unsubscribed\n");
}

void cmdTone(char* args)
{
    uint32_t rate   = 44100;
    uint32_t ch     = 2;
    uint32_t window = 0;
    uint32_t hz     = 440;
    uint32_t sec    = 0;  // 0 = run until Enter; >0 = run exactly that long

    const char* p = (args != nullptr) ? args : "";
    if (parseU32(p, &rate)) {
        p = nextToken(p);
        if (parseU32(p, &ch)) {
            p = nextToken(p);
            if (parseU32(p, &window)) {
                p = nextToken(p);
                if (parseU32(p, &hz)) {
                    p = nextToken(p);
                    (void)parseU32(p, &sec);
                }
            }
        }
    }

    (void)runToneLoop(sec, false, rate, ch, window, hz, nullptr, 0, nullptr);
}

void cmdSoak(char* args)
{
    uint32_t run_sec = 0;
    uint32_t rate    = 44100;
    uint32_t ch      = 2;
    uint32_t window  = 0;

    const char* p = (args != nullptr) ? args : "";
    if (!parseU32(p, &run_sec) || run_sec == 0) {
        ::printf("usage: soak <sec> [rate=44100] [ch=2] [window=0]\n");
        return;
    }
    p = nextToken(p);
    if (parseU32(p, &rate)) {
        p = nextToken(p);
        if (parseU32(p, &ch)) {
            p = nextToken(p);
            (void)parseU32(p, &window);
        }
    }

    // Allocate per-second credit log (cap at 3600 s).
    const size_t log_cap = run_sec < 3600 ? run_sec + 4 : 3600;
    auto* credit_log     = static_cast<uint32_t*>(::malloc(log_cap * sizeof(uint32_t)));
    if (credit_log == nullptr) {
        ::printf("out of memory for credit log\n");
        return;
    }

    size_t log_len      = 0;
    uint32_t elapsed_ms = 0;
    unsigned long total = runToneLoop(run_sec, true, rate, ch, window, 0, credit_log, log_cap, &log_len, &elapsed_ms);

    // Judgment summary. Divide by the measured wall time, not run_sec: the
    // loop overshoots the deadline by up to one stat interval and a fixed
    // divisor would skew the deviation by ~+0.5 %.
    const unsigned long nominal = static_cast<unsigned long>(rate) * ch * 2;
    double avg_rate = (elapsed_ms > 0 && total > 0)
                          ? static_cast<double>(total) * 1000.0 / static_cast<double>(elapsed_ms)
                          : 0.0;
    double pct_dev =
        (nominal > 0) ? 100.0 * (avg_rate - static_cast<double>(nominal)) / static_cast<double>(nominal) : 0.0;

    ::printf("\n--- soak summary ---\n");
    ::printf("total_sent=%lu  run_sec=%u  avg=%.1fB/s  nominal=%luB/s  dev=%.2f%%\n", total,
             static_cast<unsigned>(run_sec), avg_rate, nominal, pct_dev);
    ::printf("lastRemoteError=%d\n", static_cast<int>(g_session->lastRemoteError()));

    // Per-60s bucket average credit comparison.
    if (log_len >= 2) {
        constexpr size_t kBucket = 60;
        // First half vs second half of the 60s-bucket series.
        const size_t n_buckets = (log_len + kBucket - 1) / kBucket;
        if (n_buckets >= 2) {
            const size_t half   = n_buckets / 2;
            unsigned long sum_a = 0;
            size_t cnt_a        = 0;
            unsigned long sum_b = 0;
            size_t cnt_b        = 0;
            for (size_t bi = 0; bi < n_buckets; ++bi) {
                const size_t bstart = bi * kBucket;
                const size_t bend   = (bstart + kBucket < log_len) ? bstart + kBucket : log_len;
                unsigned long bsum  = 0;
                for (size_t k = bstart; k < bend; ++k) {
                    bsum += credit_log[k];
                }
                const size_t bcount = bend - bstart;
                if (bi < half) {
                    sum_a += bsum;
                    cnt_a += bcount;
                } else {
                    sum_b += bsum;
                    cnt_b += bcount;
                }
            }
            const double avg_a = cnt_a > 0 ? static_cast<double>(sum_a) / static_cast<double>(cnt_a) : 0.0;
            const double avg_b = cnt_b > 0 ? static_cast<double>(sum_b) / static_cast<double>(cnt_b) : 0.0;
            ::printf("credit 60s-bucket avg: first=%.1f  last=%.1f  %s\n", avg_a, avg_b,
                     (avg_b < avg_a * 0.95) ? "DEGRADED (monotone drop → frame loss?)" : "OK");
        } else {
            // Short soak: simple min/max.
            uint32_t cmin = credit_log[0];
            uint32_t cmax = credit_log[0];
            for (size_t i = 1; i < log_len; ++i) {
                if (credit_log[i] < cmin) {
                    cmin = credit_log[i];
                }
                if (credit_log[i] > cmax) {
                    cmax = credit_log[i];
                }
            }
            ::printf("credit range: min=%u max=%u (run too short for 60s buckets)\n", static_cast<unsigned>(cmin),
                     static_cast<unsigned>(cmax));
        }
    }

    ::free(credit_log);
}

// ---------------------------------------------------------------------------
// Command table
// ---------------------------------------------------------------------------

struct Command {
    const char* name;
    const char* args_help;
    const char* help;
    void (*run)(char* args);
};

static const Command kCommands[] = {
    {"caps", "", "show capabilities from hello exchange", cmdCaps},
    {"ping", "[n=10]", "RTT liveness probe (µs min/avg/max)", cmdPing},
    {"scan", "", "I2C scan 0x08-0x77", cmdScan},
    {"reg", "<addr> <reg> [n=1]", "I2C register read (hex/dec)", cmdReg},
    {"gpio-read", "", "snapshot of pins 39/38/37", cmdGpioRead},
    {"gpio-write", "<pin> <0|1>", "write to any pin (allowlist outside 39/38/37 is denied)", cmdGpioWrite},
    {"watch", "", "subscribe pin 39/38/37 push events, Enter to stop", cmdWatch},
    {"tone", "[rate] [ch] [window] [hz] [sec]", "sine stream (44100/2/lib/440); sec>0 = timed, else Enter stops", cmdTone},
    {"soak", "<sec> [rate] [ch] [window]", "mute tone for <sec>s (stdin ignored), credit/rate judgment", cmdSoak},
    {"quit", "", "exit", nullptr},
};
constexpr size_t kCommandCount = sizeof(kCommands) / sizeof(kCommands[0]);

static void printMenu()
{
    ::printf("\n--- RemoteMenu ---\n");
    for (size_t i = 0; i < kCommandCount; ++i) {
        ::printf("  %2zu  %-12s  %-30s  %s\n", i, kCommands[i].name, kCommands[i].args_help, kCommands[i].help);
    }
    ::printf("> ");
}

}  // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    // Keep progress visible even when stdout is a pipe (e.g. running
    // through `pio run -t upload`, which block-buffers it otherwise:
    // nothing would appear until the program exits).
    ::setvbuf(stdout, nullptr, _IOLBF, 0);

    // Arguments: [port|tcp:<host>:<port>] [baud]
    //   port absent → serial auto-discovery (env M5HAL_POSIX_UART_PORT as fallback)
    //   baud default 115200
    const char* port    = argc > 1 ? argv[1] : ::getenv("M5HAL_POSIX_UART_PORT");
    const uint32_t baud = argc > 2 ? static_cast<uint32_t>(::strtoul(argv[2], nullptr, 10)) : 115200;

    // Connect: "tcp:<host>:<port>" selects the TCP transport, anything else
    // is a serial port (explicit when given, auto-discovery otherwise). The
    // serial path hardware-resets each candidate into run mode before the
    // hello probes (ConnectOptions::hardware_reset, default on). Past this
    // block both transports hand back the same session and the rest of the
    // program cannot tell them apart.
    //
    // Streaming wants a TINY first-byte read timeout: every drain/credit
    // poll(1) may block for it when the line goes momentarily quiet, and with
    // the discovery-friendly default (100 ms) that block freezes the send
    // path, the device runs out of data, stops emitting credit events, and
    // the two ends sit in mutual silence for the full timeout — a repeating
    // ~110 ms stall that caps the link far below the wire rate. 2 ms mirrors
    // the TCP endpoint's read timeout; RPC waits are unaffected (they loop on
    // their own response deadline).
    auto serial_cfg                  = posix_uart::SerialRemoteEndpoint::makeHostConfig(baud);
    serial_cfg.first_byte_timeout_ms = 2;
    // Short inter-byte too: the USB bridge delivers device->host bytes in
    // clumps, so a frame regularly straddles deliveries; FrameReader resumes
    // a partial frame across calls (need_more), so a long continuation wait
    // buys nothing — it just turns each straddle into a 20 ms poll stall
    // (observed chained to 40-74 ms).
    serial_cfg.inter_byte_timeout_ms = 2;
    posix_uart::SerialRemoteEndpoint serial_ep{serial_cfg};
    posix_tcp::TcpRemoteEndpoint tcp_ep;

    if (port != nullptr && ::strncmp(port, "tcp:", 4) == 0) {
        auto connected = posix_tcp::connectRemoteTcp(tcp_ep, port + 4);
        if (!connected.has_value()) {
            ::fprintf(stderr,
                      "connect %s failed: %d\n"
                      "  hint: for espidf device (v1_exp_remote_idf5) pass baud 3000000\n",
                      port, static_cast<int>(connected.error()));
            return 1;
        }
        g_caps        = connected.value();
        g_session     = &tcp_ep.link.session();
        g_link_rx     = &tcp_ep.stream;
        g_serial_link = false;
        ::printf("using %s (tcp)\n", tcp_ep.peer());
    } else {
        posix_uart::ConnectOptions opt;
        opt.path       = port;  // nullptr -> walk the listSerialPorts candidates
        opt.on_attempt = [](void*, const char* p) { ::printf("trying %s ...\n", p); };
        auto connected = posix_uart::connectRemoteSerial(serial_ep, opt);
        if (!connected.has_value()) {
            ::fprintf(stderr,
                      "no RemoteMenu device found: %d\n"
                      "  hint: for espidf device (v1_exp_remote_idf5) pass baud 3000000\n",
                      static_cast<int>(connected.error()));
            return 1;
        }
        g_caps        = connected.value();
        g_session     = &serial_ep.link.session();
        g_link_rx     = &serial_ep.rx;
        g_serial_link = true;
        ::printf("using %s @ %u baud\n", serial_ep.devicePath(), static_cast<unsigned>(baud));
    }

    ::printf("remote proto v%u, %u bus(es)%s\n", g_caps.proto_ver, static_cast<unsigned>(g_caps.bus_count),
             g_caps.has_gpio ? ", gpio" : "");

    // stdin menu loop.
    char line[256];
    printMenu();
    while (::fgets(line, static_cast<int>(sizeof(line)), stdin) != nullptr) {
        // Strip trailing newline.
        size_t len = ::strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        // Skip blank lines.
        if (len == 0) {
            printMenu();
            continue;
        }

        // First token: numeric index or command name.
        char* p    = line;
        char* rest = line;
        // Find end of first token.
        while (*rest != '\0' && *rest != ' ' && *rest != '\t') {
            ++rest;
        }
        // Null-terminate it; advance rest past whitespace.
        if (*rest != '\0') {
            *rest++ = '\0';
            while (*rest == ' ' || *rest == '\t') {
                ++rest;
            }
        }

        // Resolve to a command index.
        size_t cmd_idx = kCommandCount;  // sentinel = not found
        // Try numeric.
        char* num_end    = nullptr;
        unsigned long ni = ::strtoul(p, &num_end, 10);
        if (num_end != p && *num_end == '\0' && ni < kCommandCount) {
            cmd_idx = static_cast<size_t>(ni);
        } else {
            // Try name match.
            for (size_t i = 0; i < kCommandCount; ++i) {
                if (::strcmp(p, kCommands[i].name) == 0) {
                    cmd_idx = i;
                    break;
                }
            }
            // Accept "help" as alias for printing the menu.
            if (::strcmp(p, "help") == 0) {
                printMenu();
                continue;
            }
        }

        if (cmd_idx >= kCommandCount) {
            ::printf("unknown command '%s' — type 'help' or a number\n", p);
            printMenu();
            continue;
        }

        // quit command: run == nullptr by convention.
        if (kCommands[cmd_idx].run == nullptr) {
            ::printf("bye\n");
            return 0;
        }

        kCommands[cmd_idx].run(rest);
        printMenu();
    }

    return 0;
}

#endif  // !ARDUINO
