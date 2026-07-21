// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — HowToUseI2SAudio
//
// Plays a 440 Hz sine wave (16-bit / 44.1 kHz) continuously through the
// built-in speaker. Only M5HAL v2 is used (no M5Unified dependency).
//
// Supported boards:
//   - M5Stack Core2 V1.1 (ESP32): AXP2101 PMIC speaker enable over external I2C.
//                                  Verified on hardware.
//                                  NOTE: Core2 V1.0 (AXP192) is NOT supported here.
//                                  (AXP192 uses a different register map.)
//   - M5Stack CoreS3  (ESP32-S3): AW9523 + AW88298 amplifier over internal I2C.
//                                  NOTE: currently UNVERIFIED on hardware (no sound
//                                  in the first trial; the amplifier prerequisites
//                                  may be incomplete).
//
// Pin / register sources:
//   M5Unified's public board definitions and speaker bring-up implementations.
//   - CoreS3: AW9523 / AW88298 initialization; internal I2C uses SCL=GPIO11,
//             SDA=GPIO12; I2S uses BCK=GPIO34, WS=GPIO33, DOUT=GPIO13.
//   - Core2 V1.1: AXP2101 ALDO3 speaker power; external I2C uses SCL=GPIO22,
//             SDA=GPIO21; I2S uses BCK=GPIO12, WS=GPIO0, DOUT=GPIO2.
//
// Audio configuration:
//   - 44100 Hz sample rate, 16-bit stereo, Philips standard
//   - 440 Hz sine wave, chunk size = 512 samples per channel = 1024 bytes
//
// Framework compatibility:
//   - Arduino (Arduino IDE / PlatformIO arduino framework)
//   - ESP-IDF (PlatformIO espidf framework, build-check only)
// =============================================================================

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#endif

#include <M5HAL_v2.hpp>

#include <cmath>
#include <memory>

#ifndef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#endif

namespace m5hal = m5::hal::v2;

// ---------------------------------------------------------------------------
// Logging helpers (Arduino: Serial, espidf: printf)
// ---------------------------------------------------------------------------

#ifdef ARDUINO
#define LOG_PRINTLN(s)       Serial.println(s)
#define LOG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#define SLEEP_MS(ms)         delay(ms)
#else
#define LOG_PRINTLN(s)       printf("%s\n", s)
#define LOG_PRINTF(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define SLEEP_MS(ms)         vTaskDelay(pdMS_TO_TICKS(ms))
#endif

// ---------------------------------------------------------------------------
// Board detection / pin constants
// ---------------------------------------------------------------------------

#if defined(CONFIG_IDF_TARGET_ESP32S3)
// ---- CoreS3 ----
static constexpr int PIN_I2C_SCL      = 11;  // internal I2C (AW9523, AW88298)
static constexpr int PIN_I2C_SDA      = 12;
static constexpr int PIN_I2S_BCK      = 34;
static constexpr int PIN_I2S_WS       = 33;
static constexpr int PIN_I2S_DOUT     = 13;
static constexpr uint8_t AW9523_ADDR  = 0x58;
static constexpr uint8_t AW88298_ADDR = 0x36;
#define BOARD_NAME "CoreS3"

#elif defined(CONFIG_IDF_TARGET_ESP32)
// ---- Core2 V1.1 (AXP2101) ----
// Core2 V1.0 (AXP192) is NOT supported here (see file header).
static constexpr int PIN_I2C_SCL      = 22;  // external I2C shared with PMIC
static constexpr int PIN_I2C_SDA      = 21;
static constexpr int PIN_I2S_BCK      = 12;
static constexpr int PIN_I2S_WS       = 0;
static constexpr int PIN_I2S_DOUT     = 2;
static constexpr uint8_t AXP2101_ADDR = 0x34;
#define BOARD_NAME "Core2 V1.1"

#else
#error "I2SAudio: unsupported target. Build for esp32 (Core2 V1.1) or esp32s3 (CoreS3)."
#endif

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

// I2S bus: acquired as a shared owner (espidf variant via IDF gen5 driver). The
// handle pins the bus for the lifetime of playback. (I2C is needed only to
// bring up the speaker amplifier; it stays local to initAmplifier below.)
static std::shared_ptr<m5hal::i2s::IBus> i2s_bus;
static m5hal::i2s::BusConfig i2s_bus_cfg;
static m5hal::i2s::AccessConfig i2s_acc_cfg;

static bool audio_ready = false;

// ---------------------------------------------------------------------------
// Error helper
// ---------------------------------------------------------------------------

static void printError(const char* label, m5hal::error::error_t error)
{
    LOG_PRINTF("%s failed: %s (%d)\n", label, m5hal::error::toString(error), static_cast<int>(error));
}

// ---------------------------------------------------------------------------
// Amplifier init (CoreS3: AW9523 port + AW88298 / Core2: AXP2101 ALDO3)
//
// I2C lives here only: one MasterAccessor per device (reused across that
// device's register writes) and the writeRegister / readRegister sugar keep
// the amplifier bring-up out of the audio path's way.
// ---------------------------------------------------------------------------

#if defined(CONFIG_IDF_TARGET_ESP32S3)

// This register sequence follows M5Unified's CoreS3 AW9523/AW88298
// speaker bring-up.
// sample_rate=44100 → rate index 7 in rate_tbl {4,5,6,8,10,11,15,20,22,44}
// → reg 0x06 = 7 | 0x14C0 (BCK mode 16*2) = 0x14C7.
static void initAmplifier(const std::shared_ptr<m5hal::i2c::IBus>& i2c)
{
    m5hal::i2c::MasterAccessConfig cfg;
    cfg.freq = 400000;

    // AW9523: enable speaker power (port-1 output reg 0x02, bit 2).
    cfg.i2c_addr = AW9523_ADDR;
    m5hal::i2c::MasterAccessor aw9523{i2c, cfg};
    bool ok       = true;
    uint8_t port1 = 0;
    ok &= aw9523.readRegister(0x02, &port1, 1).has_value();
    ok &= aw9523.writeRegister(0x02, static_cast<uint8_t>(port1 | 0b00000100)).has_value();
    if (!ok) {
        LOG_PRINTF("amp init (AW9523) had I2C errors\n");
    }

    // AW88298: 44.1 kHz / 16-bit BCK config. Registers are 16-bit, MSB first.
    cfg.i2c_addr = AW88298_ADDR;
    m5hal::i2c::MasterAccessor aw88298{i2c, cfg};
    ok        = true;
    auto wr16 = [&](uint8_t reg, uint16_t val) -> bool {
        const uint8_t be[2] = {static_cast<uint8_t>(val >> 8), static_cast<uint8_t>(val)};
        return aw88298.writeRegister(reg, m5hal::data::ConstDataSpan{be, sizeof(be)}).has_value();
    };
    ok &= wr16(0x61, 0x0673);  // boost disabled
    ok &= wr16(0x04, 0x4040);  // I2SEN=1, AMPPD=0, PWDN=0
    ok &= wr16(0x05, 0x0008);  // unmute
    ok &= wr16(0x06, 0x14C7);  // BCK=16*2, rate index 7 (44100 Hz)
    ok &= wr16(0x0C, 0x0064);  // full volume
    if (!ok) {
        LOG_PRINTF("amp init (AW88298) had I2C errors\n");
    }
}

#elif defined(CONFIG_IDF_TARGET_ESP32)

// Core2 V1.1 amplifier init: AXP2101 ALDO3 → 3300 mV to power the speaker.
// The voltage encoding follows M5Unified's AXP2101_Class::_set_LDO(2, 3300): reg 0x94 =
// (3300 - 500) / 100 = 0x1C, reg 0x90 bit 2 = ALDO3 enable.
static void initAmplifier(const std::shared_ptr<m5hal::i2c::IBus>& i2c)
{
    m5hal::i2c::MasterAccessConfig cfg;
    cfg.i2c_addr = AXP2101_ADDR;
    cfg.freq     = 400000;
    m5hal::i2c::MasterAccessor axp{i2c, cfg};

    bool ok   = true;
    uint8_t v = 0;
    ok &= axp.writeRegister(0x94, 0x1C).has_value();  // ALDO3 = 3300 mV (0.5 V base + 0x1C * 100 mV)
    ok &= axp.readRegister(0x90, &v, 1).has_value();
    ok &= axp.writeRegister(0x90, static_cast<uint8_t>(v | 0x04)).has_value();  // ALDO3 enable
    if (!ok) {
        LOG_PRINTF("amp init (AXP2101) had I2C errors\n");
    }
}

#endif

// ---------------------------------------------------------------------------
// Sine-wave audio generation
// ---------------------------------------------------------------------------

static constexpr uint32_t SAMPLE_RATE  = 44100;
static constexpr uint32_t SINE_FREQ_HZ = 440;
static constexpr size_t CHUNK_SAMPLES  = 512;                                  // samples per channel per chunk
static constexpr size_t CHUNK_BYTES    = CHUNK_SAMPLES * 2 * sizeof(int16_t);  // stereo 16-bit

static uint32_t s_sine_phase = 0;

// Fill `buf` (CHUNK_BYTES bytes) with stereo 16-bit 440 Hz sine at -6 dBFS.
static void fillSineChunk(uint8_t* buf)
{
    int16_t* p = reinterpret_cast<int16_t*>(buf);
    for (size_t i = 0; i < CHUNK_SAMPLES; ++i) {
        const float angle = 2.0f * static_cast<float>(M_PI) * static_cast<float>(SINE_FREQ_HZ) *
                            static_cast<float>(s_sine_phase) / static_cast<float>(SAMPLE_RATE);
        const int16_t sample = static_cast<int16_t>(16383.0f * sinf(angle));  // -6 dBFS
        p[i * 2 + 0]         = sample;                                        // left
        p[i * 2 + 1]         = sample;                                        // right
        ++s_sine_phase;
        if (s_sine_phase >= SAMPLE_RATE) {
            s_sine_phase = 0;
        }
    }
}

// ---------------------------------------------------------------------------
// Core init/loop logic (shared between Arduino and ESP-IDF entry points)
// ---------------------------------------------------------------------------

static void i2sAudioInit(void)
{
#ifdef ARDUINO
    Serial.begin(115200);
    SLEEP_MS(300);
    LOG_PRINTLN("");
#endif
    LOG_PRINTLN("M5HAL HowToUseI2SAudio — board: " BOARD_NAME);

    // ---- Amplifier init (acquire I2C just for bring-up, then let go) ----
    m5hal::i2c::BusConfig i2c_bus_cfg{m5hal::i2c::Scl{PIN_I2C_SCL}, m5hal::i2c::Sda{PIN_I2C_SDA}};
    auto acquired = m5hal::M5_Hal.I2C.acquire(i2c_bus_cfg);
    if (!acquired.has_value()) {
        printError("I2C acquire", acquired.error());
        return;
    }
    initAmplifier(acquired.value());
    LOG_PRINTLN("Amplifier init OK");
    // `acquired` (and its bus) is destroyed at the end of init: playback uses I2S only.

    // ---- I2S bus init ----
    i2s_bus_cfg.pin_bclk       = PIN_I2S_BCK;
    i2s_bus_cfg.pin_ws         = PIN_I2S_WS;
    i2s_bus_cfg.pin_dout       = PIN_I2S_DOUT;
    i2s_bus_cfg.tx_buffer_size = 8192;

    auto i2s_acquired = m5hal::M5_Hal.I2S.acquire(i2s_bus_cfg);
    if (!i2s_acquired.has_value()) {
        printError("I2S bus acquire", i2s_acquired.error());
        return;
    }
    i2s_bus = i2s_acquired.value();
    LOG_PRINTLN("I2S bus init OK");

    // ---- I2S access config ----
    i2s_acc_cfg.sample_rate_hz   = SAMPLE_RATE;
    i2s_acc_cfg.bits_per_sample  = 16;
    i2s_acc_cfg.channels         = 2;
    i2s_acc_cfg.write_timeout_ms = 200;

    audio_ready = true;
    LOG_PRINTLN("Playing 440 Hz sine wave...");
}

static void i2sAudioLoop(void)
{
    static uint8_t chunk[CHUNK_BYTES];
    fillSineChunk(chunk);

    // Intentional per-chunk accessor: it is a lightweight bus handle plus config/depth state.
    m5hal::i2s::TxAccessor acc{i2s_bus, i2s_acc_cfg};
    auto r = acc.write(chunk, sizeof(chunk));
    if (!r.has_value()) {
        printError("I2S write", r.error());
        SLEEP_MS(100);
    }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

#ifdef ARDUINO

void setup(void)
{
    i2sAudioInit();
}

void loop(void)
{
    if (!audio_ready) {
        delay(1000);
        return;
    }
    i2sAudioLoop();
}

#else  // ESP-IDF

// ---------------------------------------------------------------------------
// ESP-IDF entry point
// ---------------------------------------------------------------------------

extern "C" void app_main(void)
{
    i2sAudioInit();
    while (true) {
        if (!audio_ready) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        i2sAudioLoop();
    }
}

#endif  // ARDUINO
