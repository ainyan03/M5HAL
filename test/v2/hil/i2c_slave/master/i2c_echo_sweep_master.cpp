// SPDX-License-Identifier: MIT
//
// HIL MASTER firmware -- independent-transport stream echo sweep against the
// M5HAL I2C stream-echo slave.
//
// Companion to ../device/i2c_echo.cpp (SlaveBus_espidf + SlaveStreamAccessor,
// addr 0x42, pure-stream echo -- no register addressing). Unlike
// i2c_echo_master.cpp (which drives m5::hal::v2::i2c::Bus_espidf through
// MasterAccessor::transfer()), this master does NOT go through the M5HAL
// master API at all: it drives the vendor transport directly (Arduino Wire,
// or the ESP-IDF `i2c_master` driver on chips without Arduino support).
//
// Why this exists alongside i2c_echo_master.cpp: if M5HAL's own master and
// slave implementations shared a bug born from the same misreading of the
// I2C spec, a M5HAL-master-driven test could not catch it -- both sides
// would agree with each other on the wrong thing. This master proves the
// slave is correct against an implementation that shares no code with
// M5HAL's master, which is exactly the scenario that matters for M5HAL's
// coexistence-with-non-M5HAL-Arduino-libraries use case: some other library
// on the same bus, using Wire (or the raw IDF driver) directly, must be able
// to talk to an M5HAL-served slave with no M5HAL-specific quirks required on
// its side.
//
// Two transports, one test body: Arduino builds (M5Stack, Wire) and ESP-IDF
// builds (the `driver/i2c_master.h` generation-5 driver) share every round
// through a small rawWrite/rawRead seam.
//
// Protocol per round (matches i2c_echo_master.cpp's SPLIT contract): WRITE N
// payload bytes (pure stream, no register byte), STOP, then READ N bytes
// back in a SEPARATE transaction, STOP. Verifies rx == tx byte for byte.
// Sweeps N over the echo device's ring/FIFO boundaries (kEchoLens, matching
// i2c_echo_master.cpp) x 5 data patterns (zero/ones/count/rand/bound) x
// 100/400/800 kHz -- one deterministic pass per speed, unlike
// i2c_echo_master.cpp's infinite loop, so this prints one RESULT line and
// halts (matching i2c_regmap_sweep_master.cpp's convention).
//
// Wiring (2-board HIL, matches ../README.md): Core2 (this master) SDA=GPIO32
// / SCL=GPIO33 <-> CoreS3SE (device/i2c_echo.cpp) SDA=GPIO2 / SCL=GPIO1;
// common GND. Slave address 0x42. An external ~2.2-4.7k pull-up is
// recommended for the 800 kHz pass's signal margin.
//
// Build / flash:
//   export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
//   pio run -e v2_hil_i2c_echo_sweep_master_arduino_esp32 -t upload   # Wire
//   pio run -e v2_hil_i2c_echo_sweep_master_idf_esp32 -t upload       # i2c_master
//
// RESOLVED ISSUE (found as ok=159 bad=6 on the Arduino/Wire
// transport at 400 kHz only, len=65/100; fixed same day): the root cause was a
// TOCTOU race in the M5HAL slave's serve() loops (hal/v2/i2c/slave.inl, both
// SlaveStreamAccessor::serve and SlaveRegMapAccessor::serve) -- the STOP ISR
// could deliver a write's final FIFO bytes into the rx ring between a pass's
// readableBytes()==0 and its transactionComplete() check, so serve() returned
// with the tail unread (a 65-byte write surfacing as 64, no overflow count).
// The following READ was then served one byte short, the slave held the master
// under a TX_EMPTY stretch for a reply byte that never came, and the Wire
// master aborted with "requestFrom(): i2cRead returned Error 263": the legacy
// esp32-hal-i2c.c driver fails fast on its HW SCL timeout (~13-20 ms,
// I2C_LL_MAX_TIMEOUT) regardless of Wire.setTimeOut(), and its abort +
// bit-banged bus-clear against a still-stretching slave desynced the slave's
// transaction accounting for several rounds (the observed 6-round failure
// cascade with 79 ms/write dribble). The ESP-IDF gen-5 transport was clean on
// the same rig NOT because the slave was correct but because its write pacing
// never landed the STOP inside the serve() race window (its HW SCL timeout is
// 2 ms -- SHORTER than Wire's -- so a genuine slave stall would have failed it
// first). Lesson recorded: "independent transport B passes" does not acquit
// the slave; it only means B's timing does not hit the window. Post-fix this
// sweep passes ok=165 bad=0 (Wire) / ok=240 bad=0 (IDF) on the S3 rig, all
// three speeds.
//
// Vendor limits this sweep still respects (real constraints for any non-M5HAL
// Wire consumer, unchanged by the fix): the classic-ESP32 legacy driver fails
// reads fast on its HW SCL-stretch timeout cap (~13-20 ms observed end to
// end) no matter the software timeout, so an M5HAL slave application that
// blocks its serve() loop for tens of ms while a Wire master is mid-read WILL
// time that master out -- keep the serve loop event-driven (it is, by
// default) and off slow work between transactions.

#if defined(ARDUINO)
#include <Arduino.h>
#include <Wire.h>
#define LOGF(...) Serial.printf(__VA_ARGS__)
#else
#include <cstdio>
#include <cstring>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#define LOGF(...) printf(__VA_ARGS__)
static void delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
#endif

namespace {

constexpr uint8_t SLAVE_ADDR = 0x42;  // matches device/i2c_echo.cpp
// Master-side pins default to Core2 Port A; other masters override via
// build_flags (e.g. an M5Stack BASIC Port A rig: -DMASTER_PIN_SDA=21
// -DMASTER_PIN_SCL=22).
#ifndef MASTER_PIN_SDA
#define MASTER_PIN_SDA 32
#endif
#ifndef MASTER_PIN_SCL
#define MASTER_PIN_SCL 33
#endif
constexpr int PIN_SDA = MASTER_PIN_SDA;
constexpr int PIN_SCL = MASTER_PIN_SCL;

// Lengths straddling the echo device's RX/TX ring and HW FIFO boundaries (32),
// matching i2c_echo_master.cpp's kEchoLens for the same coverage.
constexpr size_t kMaxLen = 1024;
#if defined(ARDUINO)
// Classic-ESP32 Arduino Wire caps a single requestFrom() read at 255 bytes:
// the legacy esp32-hal-i2c.c driver this core's TwoWire wraps tracks the
// received byte count in what silently overflows mod 256 above that --
// requestFrom(256) returns 0, requestFrom(384) returns 128, requestFrom(1000)
// returns 232 (measured on this rig; textbook uint8_t wraparound inside the
// vendor driver, independent of the type used at this file's call site or of
// M5HAL). This is a real constraint of the vendor library any non-M5HAL
// Arduino Wire consumer on a classic ESP32 would also hit, so the Arduino
// transport's sweep stays within it; the ESP-IDF i2c_master driver below has
// no such cap and covers the full 1 KB range, matching i2c_echo_master.cpp
// (M5HAL's own master, which is also ESP-IDF-driver-based).
// 33/40/47/48 (new) plus the pre-existing 63/64 cover the STOP-tail band of a
// DELAYED consumer (device built with M5HAL_TEST_ECHO_SERVE_DELAY_MS): unread
// ring == kRxCapacity(32) with a tail still in the HW FIFO at STOP, where
// back-pressure cannot save it. Measured (5 ms delay, Wire): the
// whole 33..64 band failed -- at high SCL the RX_FULL stretch asserts too late
// to stop a master already clocking its final bytes, so the tail can reach the
// full FIFO depth (32), not just the water-mark residue.
constexpr uint16_t kEchoLens[] = {1, 2, 16, 32, 33, 40, 47, 48, 63, 64, 65, 100, 128, 200, 255};
#else
constexpr uint16_t kEchoLens[] = {1,  2,  16,  32,  33,  40,  47,  48,   63,   64,  65,
                                  100, 128, 200, 256, 384, 512, 768, 1000, 1024};
#endif
constexpr int kNumPatterns      = 5;
const char *const kPatternNames[kNumPatterns] = {"zero", "ones", "count", "rand", "bound"};

uint8_t tx_buf[kMaxLen];
uint8_t rx_buf[kMaxLen];
uint32_t ok_count  = 0;
uint32_t bad_count = 0;
char result_line[80] = "RESULT: not finished";

// Fill tx_buf with one of 5 patterns; pass_index perturbs "rand" and "count"
// so a stale reply from a previous speed pass surfaces as a mismatch instead
// of silently matching (same invariant as i2c_regmap_sweep_master.cpp).
void fillPattern(size_t n, int pat, int pass_index)
{
    uint32_t lcg = 0x12345678u ^ (uint32_t)(n * 2654435761u) ^ (uint32_t)(pass_index * 0x9E3779B9u);
    for (size_t i = 0; i < n; ++i) {
        switch (pat) {
            case 0:  tx_buf[i] = 0x00; break;
            case 1:  tx_buf[i] = 0xFF; break;
            case 2:  tx_buf[i] = (uint8_t)(i + pass_index); break;
            case 3:  lcg = lcg * 1664525u + 1013904223u; tx_buf[i] = (uint8_t)(lcg >> 24); break;
            default: tx_buf[i] = (uint8_t)((i & 1) ? 0xFF : 0x00); break;
        }
    }
}

//----------------------------------------------------------------------------
// Transport (per framework)
//----------------------------------------------------------------------------

#if defined(ARDUINO)

void busBegin(void)
{
    Wire.setBufferSize((size_t)kMaxLen);
    Wire.begin(PIN_SDA, PIN_SCL, 100000);
    Wire.setTimeOut(200);
}

void busSetClock(uint32_t freq)
{
    Wire.setClock(freq);
}

bool rawWrite(const uint8_t *data, size_t len)
{
    Wire.beginTransmission(SLAVE_ADDR);
    if (len) {
        Wire.write(data, len);
    }
    return Wire.endTransmission(true) == 0;
}

bool rawRead(uint8_t *dst, size_t len)
{
    // No call-site types lift the >255-byte cap documented at kEchoLens's
    // definition above -- the wrap is inside the vendor driver, not this call
    // site: measured with explicit (uint16_t, size_t) argument casts (since
    // removed as cosmetic; a bare (uint8_t, size_t) call resolves fine on this
    // core), requestFrom(256/384/1000) still returned 0/128/232.
    size_t n = Wire.requestFrom(SLAVE_ADDR, len);
    if (n != len) {
        return false;
    }
    for (size_t i = 0; i < len; ++i) {
        dst[i] = Wire.read();
    }
    return true;
}

#else  // ESP-IDF (driver/i2c_master.h)

constexpr int XFER_TIMEOUT_MS = 200;
i2c_master_bus_handle_t s_bus = nullptr;
i2c_master_dev_handle_t s_dev = nullptr;

void busBegin(void)
{
    i2c_master_bus_config_t cfg      = {};
    cfg.i2c_port                     = 0;
    cfg.sda_io_num                   = (gpio_num_t)PIN_SDA;
    cfg.scl_io_num                   = (gpio_num_t)PIN_SCL;
    cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt            = 7;
    cfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &s_bus));
}

void busSetClock(uint32_t freq)
{
    if (s_dev != nullptr) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = nullptr;
    }
    i2c_device_config_t dcfg = {};
    dcfg.dev_addr_length     = I2C_ADDR_BIT_LEN_7;
    dcfg.device_address      = SLAVE_ADDR;
    dcfg.scl_speed_hz        = freq;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dcfg, &s_dev));
}

bool rawWrite(const uint8_t *data, size_t len)
{
    return i2c_master_transmit(s_dev, data, len, XFER_TIMEOUT_MS) == ESP_OK;
}

bool rawRead(uint8_t *dst, size_t len)
{
    return i2c_master_receive(s_dev, dst, len, XFER_TIMEOUT_MS) == ESP_OK;
}

#endif

//----------------------------------------------------------------------------
// Test body
//----------------------------------------------------------------------------

void runSuite(uint32_t freq, int pass_index)
{
    LOGF("[%lukHz] echo sweep start\n", (unsigned long)(freq / 1000));
    busSetClock(freq);

    uint32_t fail = 0;
    int round     = 0;
    for (uint16_t n : kEchoLens) {
        for (int pat = 0; pat < kNumPatterns; ++pat) {
            fillPattern(n, pat, pass_index);
            memset(rx_buf, 0, n);

            bool wr_ok = rawWrite(tx_buf, n);      // WRITE N bytes, own transaction (STOP)
            bool rd_ok = wr_ok && rawRead(rx_buf, n);  // READ N bytes, separate transaction (STOP)

            int first_bad = -1;
            for (size_t i = 0; i < n; ++i) {
                if (rx_buf[i] != tx_buf[i]) {
                    first_bad = (int)i;
                    break;
                }
            }
            bool ok = wr_ok && rd_ok && first_bad < 0;
            LOGF("#%d ECHO scl=%lukHz len=%4u pat=%-5s first_bad=%d [%s]\n", round++,
                 (unsigned long)(freq / 1000), (unsigned)n, kPatternNames[pat], first_bad, ok ? "OK" : "BAD");
            if (ok) {
                ++ok_count;
            } else {
                ++bad_count;
                ++fail;
            }
            delay(2);  // brief gap between rounds
        }
    }
    LOGF("[%lukHz] echo sweep end (fail=%lu/%d)\n", (unsigned long)(freq / 1000), (unsigned long)fail, round);
}

}  // namespace

//----------------------------------------------------------------------------

void setup(void)
{
#if defined(ARDUINO)
    Serial.begin(115200);
#endif
    delay(500);
    LOGF("\n[i2c_echo_sweep_master] slave=0x%02X SDA=%d SCL=%d\n", SLAVE_ADDR, PIN_SDA, PIN_SCL);

    busBegin();

    runSuite(100000, 0);
    runSuite(400000, 1);
    runSuite(800000, 2);

    snprintf(result_line, sizeof(result_line), "RESULT: ok=%lu bad=%lu (%s)", (unsigned long)ok_count,
             (unsigned long)bad_count, (bad_count == 0) ? "PASS" : "FAIL");
    LOGF("%s\n", result_line);
}

void loop(void)
{
    delay(5000);
    LOGF("%s\n", result_line);
}

#if !defined(ARDUINO)

extern "C" void app_main(void)
{
    setup();
    for (;;) {
        loop();
    }
}

#endif
