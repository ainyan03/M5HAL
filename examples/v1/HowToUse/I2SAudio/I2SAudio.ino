// =============================================================================
// M5HAL — HowToUseI2SAudio
//
// Plays a 440 Hz sine wave (16-bit / 44.1 kHz) continuously through the
// built-in speaker. Only M5HAL v1 is used (no M5Unified dependency).
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
//   M5Unified.cpp (M5Unified_sandbox)
//   - CoreS3: line 2211-2221 (spk_cfg), lines 415-488 (aw88298 / aw9523 init)
//             internal I2C: line 78 SCL=GPIO11, SDA=GPIO12
//             I2S: BCK=GPIO34, WS=GPIO33, DOUT=GPIO13
//   - Core2 V1.1: line 2483-2490 (spk_cfg BCK=GPIO12, WS=GPIO0, DOUT=GPIO2)
//             speaker enable via AXP2101 ALDO3 = 3300 mV (line 447-448)
//             external I2C: line 125 SCL=GPIO22, SDA=GPIO21
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

#include <M5HAL_v1.hpp>

#include <cmath>

#ifndef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#endif

namespace m5hal = m5::hal::v1;

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

// I2C bus: arduino variant when using Arduino framework, espidf variant
// (flat-injected default) when using ESP-IDF framework.
#ifdef ARDUINO
static m5hal::i2c::variant::arduino::Bus i2c_bus;
static m5hal::i2c::variant::arduino::BusConfig i2c_bus_cfg;
#else
static m5hal::i2c::Bus i2c_bus;
static m5hal::i2c::BusConfig i2c_bus_cfg;
#endif

// I2S: flat-injected default bus (espidf variant via IDF gen5 driver).
static m5hal::i2s::Bus i2s_bus;
static m5hal::i2s::I2SBusConfig i2s_bus_cfg;
static m5hal::i2s::I2SAccessConfig i2s_access_cfg;

static bool audio_ready = false;

// ---------------------------------------------------------------------------
// Helpers: raw I2C register write via M5HAL
// ---------------------------------------------------------------------------

static bool i2cWriteReg8(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    m5hal::i2c::I2CMasterAccessConfig cfg;
    cfg.i2c_addr = dev_addr;
    cfg.freq     = 400000;
    m5hal::i2c::I2CMasterAccessor acc{i2c_bus, cfg};
    uint8_t buf[2] = {reg, value};
    auto r         = acc.write(buf, sizeof(buf));
    return r.has_value() && r.value() == sizeof(buf);
}

static bool i2cWriteReg16BE(uint8_t dev_addr, uint8_t reg, uint16_t value)
{
    // AW88298 expects MSB first (big-endian)
    uint8_t buf[3] = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    m5hal::i2c::I2CMasterAccessConfig cfg;
    cfg.i2c_addr = dev_addr;
    cfg.freq     = 400000;
    m5hal::i2c::I2CMasterAccessor acc{i2c_bus, cfg};
    auto r = acc.write(buf, sizeof(buf));
    return r.has_value() && r.value() == sizeof(buf);
}

static bool i2cReadReg8(uint8_t dev_addr, uint8_t reg, uint8_t& out)
{
    m5hal::i2c::I2CMasterAccessConfig cfg;
    cfg.i2c_addr = dev_addr;
    cfg.freq     = 400000;
    m5hal::i2c::I2CMasterAccessor acc{i2c_bus, cfg};
    auto r = acc.readRegister(static_cast<int>(reg), &out, 1);
    return r.has_value();
}

static bool i2cBitOn(uint8_t dev_addr, uint8_t reg, uint8_t mask)
{
    uint8_t v = 0;
    if (!i2cReadReg8(dev_addr, reg, v)) {
        return false;
    }
    return i2cWriteReg8(dev_addr, reg, static_cast<uint8_t>(v | mask));
}

// ---------------------------------------------------------------------------
// Amplifier init (CoreS3: AW9523 port + AW88298 / Core2: AXP2101 ALDO3)
// ---------------------------------------------------------------------------

#if defined(CONFIG_IDF_TARGET_ESP32S3)

// CoreS3 amplifier init sequence derived from M5Unified.cpp lines 458-488:
//   AW9523 reg 0x02 bit 2 = speaker power enable (port 1 output)
//   AW88298 registers:
//     0x61 = 0x0673 (boost mode disabled)
//     0x04 = 0x4040 (I2SEN=1, AMPPD=0, PWDN=0)
//     0x05 = 0x0008 (RMSE=0, HAGCE=0, HDCCE=0, HMUTE=0)
//     0x06 = rate-dependent value | 0x14C0 (BCK mode 16*2)
//     0x0C = 0x0064 (full volume)
// sample_rate=44100 → rate=(44100+1102)/2205=20 → rate_tbl {4,5,6,8,10,11,15,20,22,44}
// の走査は rate_tbl[7]=20 で止まる → index=7 → reg0x06 = 7|0x14C0 = 0x14C7
static void initAmplifier(void)
{
    // AW9523: enable speaker power (bit 2 of port-1 output register 0x02)
    i2cBitOn(AW9523_ADDR, 0x02, 0b00000100);

    // AW88298: configure for 44.1 kHz, 16-bit BCK mode
    i2cWriteReg16BE(AW88298_ADDR, 0x61, 0x0673);  // boost disabled
    i2cWriteReg16BE(AW88298_ADDR, 0x04, 0x4040);  // I2SEN=1, AMPPD=0, PWDN=0
    i2cWriteReg16BE(AW88298_ADDR, 0x05, 0x0008);  // unmute
    i2cWriteReg16BE(AW88298_ADDR, 0x06, 0x14C7);  // BCK=16*2, rate index 7 (44100 Hz)
    i2cWriteReg16BE(AW88298_ADDR, 0x0C, 0x0064);  // full volume
}

#elif defined(CONFIG_IDF_TARGET_ESP32)

// Core2 V1.1 amplifier init: AXP2101 ALDO3 → 3300 mV to power the speaker.
// Source: M5Unified.cpp (_speaker_enabled_cb_core2 / AXP2101 branch) →
// AXP2101_Class::_set_LDO(2, 3300): reg 0x94 = (3300 - 500) / 100 = 0x1C,
// reg 0x90 bit 2 = ALDO3 enable.
static void initAmplifier(void)
{
    i2cWriteReg8(AXP2101_ADDR, 0x94, 0x1C);  // ALDO3 = 3300 mV (0.5 V base + 0x1C * 100 mV)
    i2cBitOn(AXP2101_ADDR, 0x90, 0x04);      // ALDO3 enable
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

    // ---- I2C init ----
#ifdef ARDUINO
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    i2c_bus_cfg.wire    = &Wire;
    i2c_bus_cfg.pin_scl = PIN_I2C_SCL;
    i2c_bus_cfg.pin_sda = PIN_I2C_SDA;
#else
    i2c_bus_cfg = m5hal::i2c::BusConfig{PIN_I2C_SCL, PIN_I2C_SDA};
#endif
    auto r_i2c = i2c_bus.init(i2c_bus_cfg);
    if (!r_i2c.has_value()) {
        LOG_PRINTF("I2C init failed: %d\n", static_cast<int>(r_i2c.error()));
        return;
    }
    LOG_PRINTLN("I2C init OK");

    // ---- Amplifier init ----
    initAmplifier();
    LOG_PRINTLN("Amplifier init OK");

    // ---- I2S bus init ----
    i2s_bus_cfg.pin_bclk       = PIN_I2S_BCK;
    i2s_bus_cfg.pin_ws         = PIN_I2S_WS;
    i2s_bus_cfg.pin_dout       = PIN_I2S_DOUT;
    i2s_bus_cfg.tx_buffer_size = 8192;

    auto r_i2s = i2s_bus.init(i2s_bus_cfg);
    if (!r_i2s.has_value()) {
        LOG_PRINTF("I2S bus init failed: %d\n", static_cast<int>(r_i2s.error()));
        return;
    }
    LOG_PRINTLN("I2S bus init OK");

    // ---- I2S access config ----
    i2s_access_cfg.sample_rate_hz   = SAMPLE_RATE;
    i2s_access_cfg.bits_per_sample  = 16;
    i2s_access_cfg.channels         = 2;
    i2s_access_cfg.write_timeout_ms = 200;

    audio_ready = true;
    LOG_PRINTLN("Playing 440 Hz sine wave...");
}

static void i2sAudioLoop(void)
{
    static uint8_t chunk[CHUNK_BYTES];
    fillSineChunk(chunk);

    m5hal::i2s::I2STxAccessor acc{i2s_bus, i2s_access_cfg};
    auto r = acc.write(chunk, sizeof(chunk));
    if (!r.has_value()) {
        LOG_PRINTF("I2S write failed: %d\n", static_cast<int>(r.error()));
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
