// SPDX-License-Identifier: MIT
#ifndef M5HAL_TEST_SPI_WIRE_CAPTURE_HPP
#define M5HAL_TEST_SPI_WIRE_CAPTURE_HPP

// Shared GPIO self-capture rig for the SPI wire self-tests
// (test_software_spi_wire / test_espidf_spi_wire). A FreeRTOS task on
// the other core polls the CLK/MOSI/DC/CS pins through the GPIO input
// registers and records every state change with a µs timestamp; the
// asserts then check protocol semantics on the recorded edge stream.
//
// Resolution limit: one poll iteration is a few µs, so transitions
// closer than that merge into one sample — the rig is for LOW-SPEED
// configurations only (see spec/verification.md). Implementations keep
// phase-boundary events half-period spaced for exactly this reason.
//
// Usage: call `wire_capture::setPins(...)` once in setup(), wrap each
// transfer in start()/stop(), then run the collect*/assert helpers.

#include <Arduino.h>
#include <unity.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace wire_capture {

constexpr uint8_t kClkBit  = 0x01;
constexpr uint8_t kMosiBit = 0x02;
constexpr uint8_t kDcBit   = 0x04;
constexpr uint8_t kCsBit   = 0x08;

struct Sample {
    uint32_t usec = 0;
    uint8_t state = 0;
};

constexpr size_t kMaxSamples = 2048;
inline Sample samples[kMaxSamples];
inline volatile size_t sample_count   = 0;
inline volatile bool capture_running  = false;
inline volatile bool capture_ready    = false;
inline volatile bool capture_done     = false;
inline volatile bool capture_overflow = false;

inline uint8_t pin_clk  = 0;
inline uint8_t pin_mosi = 0;
inline uint8_t pin_dc   = 0;
inline uint8_t pin_cs   = 0;

inline void setPins(uint8_t clk, uint8_t mosi, uint8_t dc, uint8_t cs)
{
    pin_clk  = clk;
    pin_mosi = mosi;
    pin_dc   = dc;
    pin_cs   = cs;
}

inline uint8_t readState()
{
    uint8_t state = 0;
    state |= digitalRead(pin_clk) ? kClkBit : 0;
    state |= digitalRead(pin_mosi) ? kMosiBit : 0;
    state |= digitalRead(pin_dc) ? kDcBit : 0;
    state |= digitalRead(pin_cs) ? kCsBit : 0;
    return state;
}

inline void captureTask(void*)
{
    uint8_t last     = readState();
    sample_count     = 0;
    capture_overflow = false;
    if (sample_count < kMaxSamples) {
        samples[sample_count++] = {micros(), last};
    }
    capture_ready = true;

    while (capture_running) {
        const uint8_t state = readState();
        if (state != last) {
            const size_t index = sample_count;
            if (index < kMaxSamples) {
                samples[index] = {micros(), state};
                sample_count   = index + 1;
            } else {
                capture_overflow = true;
            }
            last = state;
        }
    }

    capture_done = true;
    vTaskDelete(nullptr);
}

inline void start()
{
    capture_running  = true;
    capture_ready    = false;
    capture_done     = false;
    capture_overflow = false;
    sample_count     = 0;
    xTaskCreatePinnedToCore(captureTask, "spi-capture", 4096, nullptr, 3, nullptr, 0);
    while (!capture_ready) {
        delay(1);
    }
}

inline void stop()
{
    delay(2);
    capture_running = false;
    while (!capture_done) {
        delay(1);
    }
}

inline size_t collectActiveRisingEdges(uint8_t* states, size_t capacity)
{
    size_t count          = 0;
    const size_t captured = sample_count;
    for (size_t i = 1; i < captured; ++i) {
        const uint8_t prev = samples[i - 1].state;
        const uint8_t cur  = samples[i].state;
        const bool rising  = ((prev & kClkBit) == 0) && ((cur & kClkBit) != 0);
        const bool active  = (cur & kCsBit) == 0;
        if (rising && active) {
            if (count < capacity) {
                states[count] = cur;
            }
            ++count;
        }
    }
    return count;
}

inline size_t collectActiveClockEdges(uint8_t* states, bool* rising_edges, size_t capacity)
{
    size_t count          = 0;
    const size_t captured = sample_count;
    for (size_t i = 1; i < captured; ++i) {
        const uint8_t prev = samples[i - 1].state;
        const uint8_t cur  = samples[i].state;
        const bool changed = ((prev ^ cur) & kClkBit) != 0;
        const bool active  = (cur & kCsBit) == 0;
        if (changed && active) {
            if (count < capacity) {
                states[count]       = cur;
                rising_edges[count] = ((prev & kClkBit) == 0) && ((cur & kClkBit) != 0);
            }
            ++count;
        }
    }
    return count;
}

inline bool bitAt(const uint8_t* bytes, size_t bit_index, bool lsb_first = false)
{
    const uint8_t byte = bytes[bit_index >> 3];
    const uint8_t bit  = static_cast<uint8_t>(bit_index & 7u);
    const uint8_t mask = lsb_first ? static_cast<uint8_t>(0x01u << bit) : static_cast<uint8_t>(0x80u >> bit);
    return (byte & mask) != 0;
}

inline void assertBits(const uint8_t* states, size_t start_bit, const uint8_t* bytes, size_t bit_count,
                       bool lsb_first = false)
{
    for (size_t i = 0; i < bit_count; ++i) {
        const bool expected = bitAt(bytes, i, lsb_first);
        const bool actual   = (states[start_bit + i] & kMosiBit) != 0;
        char msg[48];
        snprintf(msg, sizeof(msg), "MOSI bit mismatch at edge %u", static_cast<unsigned>(start_bit + i));
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected ? 1 : 0, actual ? 1 : 0, msg);
    }
}

inline void assertDcRange(const uint8_t* states, size_t start_bit, size_t bit_count, bool expected_high)
{
    for (size_t i = 0; i < bit_count; ++i) {
        const bool actual = (states[start_bit + i] & kDcBit) != 0;
        char msg[48];
        snprintf(msg, sizeof(msg), "DC phase mismatch at edge %u", static_cast<unsigned>(start_bit + i));
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected_high ? 1 : 0, actual ? 1 : 0, msg);
    }
}

/*
  Pacing guard: the clock must never run faster than configured
  (software SPI policy, spec/design/spi.md §段階移行 — and for hardware
  backends this doubles as a clock-divider sanity check). Ideal-phase
  keeping allows a single late edge to catch up inside its own
  half-period window, so two ADJACENT edges may legitimately come
  closer than a half period - but across any THREE consecutive edges at
  least one full half period must have elapsed (t[i+2] - t[i] >= half).
  A catch-up burst (edges back-to-back at CPU speed) violates this by
  orders of magnitude. The 20% margin absorbs the capture task's
  sampling jitter.
 */
inline void assertClockNeverFasterThanConfigured(uint32_t freq_hz)
{
    const uint32_t half_period_us = 500000u / freq_hz;
    const uint32_t floor_us       = half_period_us - (half_period_us / 5);
    uint32_t prev2                = 0;
    uint32_t prev1                = 0;
    size_t toggles                = 0;
    const size_t captured         = sample_count;
    for (size_t i = 1; i < captured; ++i) {
        const uint8_t prev = samples[i - 1].state;
        const uint8_t cur  = samples[i].state;
        if ((((prev ^ cur) & kClkBit) == 0) || ((cur & kCsBit) != 0)) {
            continue;
        }
        const uint32_t now = samples[i].usec;
        if (toggles >= 2) {
            TEST_ASSERT_GREATER_OR_EQUAL_UINT32_MESSAGE(floor_us, now - prev2,
                                                        "three consecutive clock edges faster than configured (burst)");
        }
        prev2 = prev1;
        prev1 = now;
        ++toggles;
    }
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32_MESSAGE(3, toggles, "not enough clock edges captured for pacing guard");
}

}  // namespace wire_capture

#endif  // M5HAL_TEST_SPI_WIRE_CAPTURE_HPP
