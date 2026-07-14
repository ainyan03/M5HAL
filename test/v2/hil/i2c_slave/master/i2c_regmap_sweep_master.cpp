// SPDX-License-Identifier: MIT
//
// HIL MASTER firmware -- full register-map sweep against the M5HAL I2C slave.
//
// Companion to ../device/i2c_slave.cpp (SlaveBus_espidf + SlaveRegMapAccessor,
// addr 0x42, a 256-byte register file with reg_file[i] == i and an
// auto-incrementing pointer). Unlike i2c_echo_master.cpp, this master does NOT
// go through the M5HAL master API: it drives the vendor transport directly
// (Arduino Wire, or the ESP-IDF `i2c_master` driver on chips without Arduino
// support) so the slave is validated by an implementation independent of
// M5HAL's own master backend -- a bug shared by both sides of the wire would
// not show up if the master were also M5HAL.
//
// Two transports, one test body: Arduino builds (M5Stack, Wire) and ESP-IDF
// builds (the new `driver/i2c_master.h` generation-5 driver) share every test
// below through a small rawWrite/rawRead/rawPureRead/rawProbe seam.
//
// Data pattern design (keep this invariant if the suite is ever extended):
// every test pattern is generated from an odd stride mod 256 (a bijection on
// byte values) or a running counter, so no two bytes inside a verified window
// share a value. A byte shift, duplication, drop, or stale carry-over then
// shows up as a full-window mismatch instead of silently reading as "the same
// value, just moved" -- a same-valued pattern (e.g. all 0x00) would let those
// exact failure modes slip through undetected. `pass_index` (the 100/400/
// 800 kHz sweep index) additionally offsets every pattern so a stale reply
// left over from the previous speed pass is also caught as a mismatch.
//
// COUNTER_REG (0xFF) deviation from the imported test's flat register file:
// the M5HAL device fixture makes reg 0xFF a live, ever-incrementing value on
// every read (device/i2c_slave.cpp's onReadRegister), to prove the
// accessor's onRead hook runs at read time rather than serving a static
// backing store. That is incompatible with this suite's write-then-verify
// design for that one register, so verify() excludes 0xFF from the
// byte-for-byte comparison; every other register is checked exactly as in
// the source suite. T14 separately asserts the live-increment property
// itself (two single-byte reads, second == first + 1) -- the only HW-level
// proof anywhere in this repo that SlaveBus_espidf's onRead path runs at
// read time rather than serving a stale/cached value.
//
// Wiring (2-board HIL, matches ../README.md): Core2 (this master)
// SDA=GPIO32 / SCL=GPIO33 <-> CoreS3SE (device/i2c_slave.cpp) SDA=GPIO2 /
// SCL=GPIO1; common GND. Slave address 0x42. An external ~2.2-4.7k pull-up is
// recommended for the 800 kHz pass's signal margin.
//
// T1-T10 exercise fixed-size cases (whole-file read, window base, wrap,
// long read/write, single byte, a zero-gap stress); T11-T13 sweep the length
// dimension directly -- WTR read (T11), SPLIT read (T12), and a
// WRITETEST-equivalent write-then-verify (T13) -- each n/k = 1..64 or 1..48;
// T14 is the onRead-live-value smoke (COUNTERTEST equivalent). This subsumes
// the private i2c_slave_hybrid HIL pair's WTR/SPLIT/WTEST/COUNTERTEST modes
// at 100/400/800 kHz (see kSweepRegSeq's comment for the length-sweep
// coverage argument).
//
// Build / flash:
//   export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
//   pio run -e v2_hil_i2c_regmap_sweep_master_arduino_esp32 -t upload   # Wire
//   pio run -e v2_hil_i2c_regmap_sweep_master_idf_esp32 -t upload       # i2c_master

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

// Target slave address defaults to the device fixture's default; a multi-slave
// rig retargets the sweep via build_flags (e.g. -DMASTER_SLAVE_ADDR=0x43).
#ifndef MASTER_SLAVE_ADDR
#define MASTER_SLAVE_ADDR 0x42
#endif
constexpr uint8_t SLAVE_ADDR  = MASTER_SLAVE_ADDR;  // matches device/i2c_slave.cpp
constexpr uint8_t COUNTER_REG = 0xFF;               // live incrementing register; excluded from verify()
// Register cycle shared by T11-T13's length sweeps. Mirrors the private
// i2c_slave_hybrid verification master's regseq[8]; because that master's
// register index (i & 7) and length index (i % 64 or i % 48) are both driven
// by the same loop counter over a register-count period that divides the
// length period, its long-running sweep in fact pairs each length with a
// single fixed register -- reg = regseq[(len - 1) & 7]. Reusing that pairing
// here reproduces its exact (register, length) coverage in one deterministic
// pass instead of a full cross product.
constexpr uint8_t kSweepRegSeq[8] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};
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

uint8_t ref[256];  // write-mirror: expected value of every register (except COUNTER_REG)
uint32_t ok_count     = 0;
uint32_t bad_count    = 0;
uint32_t stress_bad   = 0;
char result_line[120] = "RESULT: not finished";

// Consecutive-transport-failure bailout: on a bus that cannot complete any
// transfer at the current clock (e.g. an electrically unsupported speed on a
// shared multi-device bus with no external pull-up), every one of this
// suite's ~650 per-pass transport calls would otherwise still be attempted
// against the vendor driver back-to-back with zero gap. That call volume,
// not any single call, was the trigger for an observed firmware crash
// (StoreProhibited/EXCVADDR=0, reboot loop) once real hardware exercised a
// pass that fails from its very first transfer -- a failure shape earlier
// suite runs never hit. Once a short streak of raw transport calls fails in
// a row, presume the bus is down for the remainder of this pass and stop
// issuing further transfers instead of continuing to hammer the driver;
// this keeps the failure diagnosis clean (ERR counted, no panic) without
// touching any transport/timeout/frequency configuration.
constexpr uint32_t kConsecutiveFailBailout = 3;
uint32_t consecutive_fail                  = 0;
bool pass_aborted                          = false;

// Feed a raw transport call's result through the bailout counter. Returns
// ok unchanged; callers only need to gate on pass_aborted afterward.
bool noteResult(bool ok)
{
    if (ok) {
        consecutive_fail = 0;
        return true;
    }
    if (!pass_aborted && ++consecutive_fail >= kConsecutiveFailBailout) {
        pass_aborted = true;
        LOGF("  bus unresponsive (%lu consecutive transport failures) -- aborting remainder of this pass\n",
             (unsigned long)consecutive_fail);
    }
    return false;
}

//----------------------------------------------------------------------------
// Transport (per framework)
//----------------------------------------------------------------------------

#if defined(ARDUINO)

void busBegin(void)
{
    Wire.setBufferSize(256);
    Wire.begin(PIN_SDA, PIN_SCL, 100000);
    Wire.setTimeOut(100);
}

void busSetClock(uint32_t freq)
{
    Wire.setClock(freq);
}

// Write [ptr, data...], terminated by STOP.
bool rawWrite(uint8_t ptr, const uint8_t *data, size_t len)
{
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(ptr);
    if (len) {
        Wire.write(data, len);
    }
    return Wire.endTransmission(true) == 0;
}

// Write the pointer, then read len bytes. restart=true uses a repeated START
// (WTR); restart=false inserts a STOP between the two transactions (SPLIT).
bool rawRead(uint8_t ptr, uint8_t *dst, size_t len, bool restart)
{
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(ptr);
    if (Wire.endTransmission(!restart) != 0) {
        return false;
    }
    // Size cap, not an overload issue: classic-ESP32 Arduino Wire caps a
    // single requestFrom() at 255 bytes regardless of the call-site types (see
    // i2c_echo_sweep_master.cpp's kEchoLens comment) -- moot here since this
    // suite never reads more than 96 bytes, but keep any future extension of
    // this sweep past 255 bytes on the ESP-IDF i2c_master transport instead.
    size_t n = Wire.requestFrom(SLAVE_ADDR, len);
    if (n != len) {
        return false;
    }
    for (size_t i = 0; i < len; ++i) {
        dst[i] = Wire.read();
    }
    return true;
}

// Pure read with no preceding pointer write (relies on the pointer persisting
// from the prior transaction).
bool rawPureRead(uint8_t *dst, size_t len)
{
    // 255-byte cap: see rawRead() above.
    size_t n = Wire.requestFrom(SLAVE_ADDR, len);
    if (n != len) {
        return false;
    }
    for (size_t i = 0; i < len; ++i) {
        dst[i] = Wire.read();
    }
    return true;
}

// Address-only transaction (no data): does the slave ACK its address?
bool rawProbe(void)
{
    Wire.beginTransmission(SLAVE_ADDR);
    return Wire.endTransmission(true) == 0;
}

#else  // ESP-IDF (driver/i2c_master.h)

// Abort-discrimination knobs: the M5HAL gen5 master differs from this
// baseline transport only in dev_config.scl_wait_us (10000us vs the driver
// default 2000us) and the per-call transfer timeout (10ms vs 100ms). These
// overrides let this independent master adopt either M5HAL value one at a
// time, to test whether the M5HAL-only sporadic mid-byte abort follows the
// CONFIG (this build reproduces it) or the wrapper code (this build stays
// clean):  -DSWEEP_SCL_WAIT_US=10000  -DSWEEP_XFER_TIMEOUT_MS=10
#ifndef SWEEP_XFER_TIMEOUT_MS
#define SWEEP_XFER_TIMEOUT_MS 100
#endif
constexpr int XFER_TIMEOUT_MS = SWEEP_XFER_TIMEOUT_MS;
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

// scl_speed_hz is a per-device-handle setting, so a clock change re-adds the
// device rather than reconfiguring in place.
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
#ifdef SWEEP_SCL_WAIT_US
    dcfg.scl_wait_us = SWEEP_SCL_WAIT_US;  // discrimination knob (see XFER_TIMEOUT_MS)
#endif
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dcfg, &s_dev));
}

bool rawWrite(uint8_t ptr, const uint8_t *data, size_t len)
{
    uint8_t buf[128];
    buf[0] = ptr;
    if (len) {
        memcpy(buf + 1, data, len);
    }
    return i2c_master_transmit(s_dev, buf, len + 1, XFER_TIMEOUT_MS) == ESP_OK;
}

bool rawRead(uint8_t ptr, uint8_t *dst, size_t len, bool restart)
{
    if (restart) {
        // i2c_master_transmit_receive issues the pointer write and the read
        // as one transaction with a repeated START -- the tightest form of
        // the RESTART path (no software gap between write and read).
        return i2c_master_transmit_receive(s_dev, &ptr, 1, dst, len, XFER_TIMEOUT_MS) == ESP_OK;
    }
    if (i2c_master_transmit(s_dev, &ptr, 1, XFER_TIMEOUT_MS) != ESP_OK) {
        return false;
    }
    return i2c_master_receive(s_dev, dst, len, XFER_TIMEOUT_MS) == ESP_OK;
}

bool rawPureRead(uint8_t *dst, size_t len)
{
    return i2c_master_receive(s_dev, dst, len, XFER_TIMEOUT_MS) == ESP_OK;
}

bool rawProbe(void)
{
    return i2c_master_probe(s_bus, SLAVE_ADDR, XFER_TIMEOUT_MS) == ESP_OK;
}

#endif

//----------------------------------------------------------------------------
// Shared helpers
//----------------------------------------------------------------------------

// Write [ptr, data...]; on success also advance the write-mirror.
bool writeRegs(uint8_t ptr, const uint8_t *data, size_t len)
{
    if (pass_aborted) {
        return false;
    }
    if (!noteResult(rawWrite(ptr, data, len))) {
        return false;
    }
    for (size_t i = 0; i < len; ++i) {
        ref[(uint8_t)(ptr + i)] = data[i];
    }
    return true;
}

bool readRegs(uint8_t ptr, uint8_t *dst, size_t len, bool restart)
{
    if (pass_aborted) {
        return false;
    }
    return noteResult(rawRead(ptr, dst, len, restart));
}

bool pureRead(uint8_t *dst, size_t len)
{
    if (pass_aborted) {
        return false;
    }
    return noteResult(rawPureRead(dst, len));
}

// T9's address-only probe goes through the same bailout gate as the other
// transport helpers so a dead bus does not keep issuing probes either.
bool probeBus(void)
{
    if (pass_aborted) {
        return false;
    }
    return noteResult(rawProbe());
}

// dst[0..len) against ref[ptr..ptr+len) (mod 256), skipping COUNTER_REG.
bool verify(uint8_t ptr, const uint8_t *dst, size_t len, const char *tag)
{
    for (size_t i = 0; i < len; ++i) {
        uint8_t reg = (uint8_t)(ptr + i);
        if (reg == COUNTER_REG) {
            continue;  // live value, not comparable to the write-mirror
        }
        uint8_t e = ref[reg];
        if (dst[i] != e) {
            LOGF("  MISMATCH %s: reg[0x%02X] expect 0x%02X got 0x%02X\n", tag, reg, e, dst[i]);
            return false;
        }
    }
    return true;
}

void report(const char *name, bool pass)
{
    LOGF("  %-12s : %s\n", name, pass ? "OK" : "BAD");
    if (pass) {
        ++ok_count;
    } else {
        ++bad_count;
    }
}

//----------------------------------------------------------------------------
// Test body
//----------------------------------------------------------------------------

void runSuite(uint32_t freq, int pass_index)
{
    LOGF("[%lukHz] suite start\n", (unsigned long)(freq / 1000));
    // Fresh bailout budget per pass: a speed that fails outright should not
    // consume the allowance of a later pass at a different (working) speed.
    consecutive_fail = 0;
    pass_aborted     = false;
    busSetClock(freq);

    uint8_t buf[128];

    // Seed the whole register file with a known pattern (32 bytes x 8 blocks).
    bool setup_ok = true;
    for (int b = 0; b < 8; ++b) {
        uint8_t data[32];
        for (int i = 0; i < 32; ++i) {
            data[i] = (uint8_t)((b * 32 + i) * 7 + pass_index * 13 + 1);
        }
        if (!writeRegs((uint8_t)(b * 32), data, 32)) {
            setup_ok = false;
        }
        delay(1);
    }
    report("setup", setup_ok);

#if !defined(SWEEP_SKIP_T1)
    // T1 SPLIT read (whole file)
    {
        bool pass = true;
        for (int b = 0; b < 8 && pass; ++b) {
            uint8_t ptr = (uint8_t)(b * 32);
            pass        = readRegs(ptr, buf, 32, false) && verify(ptr, buf, 32, "T1");
        }
        report("T1 split", pass);
    }
#endif

#if !defined(SWEEP_SKIP_T2)
    // T2 WTR read (whole file)
    {
        bool pass = true;
        for (int b = 0; b < 8 && pass; ++b) {
            uint8_t ptr = (uint8_t)(b * 32);
            pass        = readRegs(ptr, buf, 32, true) && verify(ptr, buf, 32, "T2");
        }
        report("T2 wtr", pass);
    }
#endif

    // T3 window base: a pure read right after a data write returns what was
    // just written (the pointer must not have advanced from the data write).
    {
        uint8_t data[8];
        for (int i = 0; i < 8; ++i) {
            data[i] = (uint8_t)(0xA0 + i + pass_index);
        }
// -DSWEEP_T3_GAP_MS=N inserts a delay between T3's data write and its pure
// read; diagnostic knob for a slave-side write-STOP -> read race (the failure
// disappearing with a gap proves the zero-gap timing is the trigger).
#if defined(SWEEP_T3_GAP_MS)
        bool pass = writeRegs(0x40, data, 8);
        delay(SWEEP_T3_GAP_MS);
        pass = pass && pureRead(buf, 8) && verify(0x40, buf, 8, "T3");
#else
        bool pass = writeRegs(0x40, data, 8) && pureRead(buf, 8) && verify(0x40, buf, 8, "T3");
#endif
        if (!pass) {
            // Dump the full window: whether the reply is the pre-write register
            // state (a stale composed reply) or noise tells the failure apart.
            LOGF("  T3 got:");
            for (int i = 0; i < 8; ++i) {
                LOGF(" %02X", buf[i]);
            }
            LOGF("  expect: %02X..%02X\n", data[0], data[7]);
        }
        report("T3 winbase", pass);
    }

#if !defined(SWEEP_SKIP_T4)
    // T4 repeat read: reading the same window twice returns the same bytes.
    {
        bool pass = readRegs(0x10, buf, 16, true) && verify(0x10, buf, 16, "T4a");
        uint8_t second[16];
        pass = pass && pureRead(second, 16) && verify(0x10, second, 16, "T4b");
        report("T4 repeat", pass);
    }
#endif

#if !defined(SWEEP_SKIP_T5)
    // T5 wrap: a 16-byte window starting at 0xF8 wraps to 0x00.
    {
        uint8_t data[16];
        for (int i = 0; i < 16; ++i) {
            data[i] = (uint8_t)(0x5A ^ (i + pass_index));
        }
        bool pass = writeRegs(0xF8, data, 16) && readRegs(0xF8, buf, 16, true) && verify(0xF8, buf, 16, "T5a") &&
                    readRegs(0x00, buf, 8, false) && verify(0x00, buf, 8, "T5b");
        report("T5 wrap", pass);
    }
#endif

#if !defined(SWEEP_SKIP_T6)
    // T6 long read: 96 bytes in ONE transaction (3x the 32-byte HW FIFO, and
    // past the 64-byte compose chunk). Exercises the serve() reply pump: the
    // continuation chunk must stream while the master keeps clocking (before
    // the reply-pump streaming fix, a single read was capped at one 64B
    // window and this test timed out).
    {
        bool pass = readRegs(0x00, buf, 96, true) && verify(0x00, buf, 96, "T6");
        report("T6 read96", pass);
    }
#endif

#if !defined(SWEEP_SKIP_T7)
    // T7 long write: 96 bytes exercises sustained mid-transaction RX drain
    // (the slave drains at the 16-byte RX watermark interrupt; the RX_FULL
    // stretch only backstops an ISR- or consumer-lag, it is not on the normal
    // path; writes were never window-capped), then a 96B single-transaction
    // readback through the same streaming path as T6.
    {
        uint8_t data[96];
        for (int i = 0; i < 96; ++i) {
            data[i] = (uint8_t)(i * 3 + pass_index * 7 + 5);
        }
        bool pass = writeRegs(0x80, data, 96) && readRegs(0x80, buf, 96, true) && verify(0x80, buf, 96, "T7");
        report("T7 write96", pass);
    }
#endif

#if !defined(SWEEP_SKIP_T8)
    // T8 single byte
    {
        uint8_t v = (uint8_t)(0xC3 + pass_index);
        bool pass = writeRegs(0x77, &v, 1) && readRegs(0x77, buf, 1, true) && verify(0x77, buf, 1, "T8");
        report("T8 single", pass);
    }
#endif

#if !defined(SWEEP_SKIP_T9)
    // T9 ack probe: an address-only transaction (no data) is ACKed.
    {
        report("T9 probe", probeBus());
    }
#endif

#if !defined(SWEEP_SKIP_T10)
    // T10 stress: 200 back-to-back WTR write+verify rounds with zero gap
    // between them (checks the known-best-effort write-transaction-boundary
    // race under load; tallied separately from the base suite).
    {
        uint32_t fail = 0;
        for (int n = 0; n < 200; ++n) {
            uint8_t ptr = (uint8_t)(n * 37 + 11);
            uint8_t len = (uint8_t)(1 + (n % 16));
            uint8_t data[16];
            for (int i = 0; i < len; ++i) {
                data[i] = (uint8_t)(n + i * 29 + pass_index);
            }
            if (!writeRegs(ptr, data, len) || !readRegs(ptr, buf, len, true) || !verify(ptr, buf, len, "T10")) {
                ++fail;
            }
        }
        stress_bad += fail;
        LOGF("  %-12s : %s (fail=%lu/200)\n", "T10 stress", fail == 0 ? "OK" : "NG", (unsigned long)fail);
    }
#endif

#if !defined(SWEEP_SKIP_T11)
    // T11 WTR length sweep: n=1..64, one repeated-START read per length, using
    // kSweepRegSeq[(n-1)&7] as the register (see its comment for why this one
    // pass reproduces the private master's full sweep coverage).
    {
        bool pass = true;
        for (int n = 1; n <= 64 && pass; ++n) {
            uint8_t reg = kSweepRegSeq[(n - 1) & 7];
            pass        = readRegs(reg, buf, (size_t)n, true) && verify(reg, buf, (size_t)n, "T11");
        }
        report("T11 wtr sweep", pass);
    }
#endif

#if !defined(SWEEP_SKIP_T12)
    // T12 SPLIT length sweep: same (register, length) pairing as T11, but a
    // STOP between the pointer write and the read (separate transactions).
    {
        bool pass = true;
        for (int n = 1; n <= 64 && pass; ++n) {
            uint8_t reg = kSweepRegSeq[(n - 1) & 7];
            pass        = readRegs(reg, buf, (size_t)n, false) && verify(reg, buf, (size_t)n, "T12");
        }
        report("T12 split sweep", pass);
    }
#endif

#if !defined(SWEEP_SKIP_T13)
    // T13 WTEST-equivalent: multi-byte write (k=1..48; k>=32 exercises
    // mid-transaction RX drain -- the slave drains at the 16-byte RX watermark
    // interrupt, with the RX_FULL stretch only as the ISR-/consumer-lag
    // backstop, not the normal path) then a SEPARATE-transaction read-back
    // verify --
    // the read must follow the write's STOP, since the slave preloads its
    // response at the register byte, before the write's data lands.
    {
        bool pass = true;
        for (int k = 1; k <= 48 && pass; ++k) {
            uint8_t reg = kSweepRegSeq[(k - 1) & 7];
            uint8_t data[48];
            for (int j = 0; j < k; ++j) {
                data[j] = (uint8_t)(reg + 0x80 + j + pass_index);
            }
            pass = writeRegs(reg, data, (size_t)k) && readRegs(reg, buf, (size_t)k, false) &&
                   verify(reg, buf, (size_t)k, "T13");
        }
        report("T13 wtest sweep", pass);
    }
#endif

    // T14 onRead live-value smoke: read COUNTER_REG (0xFF) twice, one byte
    // each, and expect the second value to be exactly the first + 1 (mod
    // 256). The device's onRead hook (device/i2c_slave.cpp's onReadRegister)
    // computes this value just-in-time while SlaveBus_espidf holds the
    // master under clock stretch, instead of serving it from the static
    // reg_file backing store -- this is the one behavior that differentiates
    // a stretch-capable slave, and it only proves out on the real HW clock-
    // stretch backend (no other test in this suite, nor any native/QEMU
    // target, exercises SlaveBus_espidf's onRead-at-read-time path).
    // -DSWEEP_SKIP_T14 drops this test from the pass; diagnostic knob for
    // isolating cross-pass state contamination (a later pass's failure that
    // depends on an earlier pass having run this test).
#if !defined(SWEEP_SKIP_T14)
    {
        uint8_t v1 = 0, v2 = 0;
        bool pass =
            readRegs(COUNTER_REG, &v1, 1, true) && readRegs(COUNTER_REG, &v2, 1, true) && ((uint8_t)(v1 + 1) == v2);
        if (!pass) {
            LOGF("  MISMATCH T14: v1=0x%02X v2=0x%02X (expect v2==v1+1)\n", v1, v2);
        }
        report("T14 ctr smoke", pass);
    }
#endif

    LOGF("[%lukHz] suite end\n", (unsigned long)(freq / 1000));
}

}  // namespace

//----------------------------------------------------------------------------

void setup(void)
{
#if defined(ARDUINO)
    Serial.begin(115200);
#endif
// A USB-Serial-JTAG console re-enumerates on reset, so a host capture loses
// the first ~2s of output; a diagnostic run can push the suite past that
// window with e.g. -DSWEEP_STARTUP_DELAY_MS=4000.
#ifndef SWEEP_STARTUP_DELAY_MS
#define SWEEP_STARTUP_DELAY_MS 500
#endif
    delay(SWEEP_STARTUP_DELAY_MS);
    LOGF("\n[i2c_regmap_sweep_master] slave=0x%02X SDA=%d SCL=%d\n", SLAVE_ADDR, PIN_SDA, PIN_SCL);
#if !defined(ARDUINO)
    // Self-documenting capture: which abort-discrimination config this build ran.
    LOGF("[i2c_regmap_sweep_master] xfer_timeout_ms=%d scl_wait_us=%d\n", XFER_TIMEOUT_MS,
#ifdef SWEEP_SCL_WAIT_US
         (int)SWEEP_SCL_WAIT_US
#else
         0  // 0 = driver default (2000us in IDF v5.5)
#endif
    );
#endif

    busBegin();

// The standard acceptance matrix is 100/400/800 kHz. A diagnostic build can
// override the pass list with e.g. -DSWEEP_FREQS=200000,300000,600000 to
// bisect a frequency-dependent failure (each pass still gets its own
// pattern offset via the pass index).
#ifndef SWEEP_FREQS
#define SWEEP_FREQS 100000, 400000, 800000
#endif
    static constexpr uint32_t sweep_freqs[] = {SWEEP_FREQS};
    static constexpr size_t sweep_count     = sizeof(sweep_freqs) / sizeof(sweep_freqs[0]);
    for (size_t i = 0; i < sweep_count; ++i) {
        runSuite(sweep_freqs[i], static_cast<int>(i));
    }

    snprintf(result_line, sizeof(result_line), "RESULT: ok=%lu bad=%lu stress_bad=%lu/%lu (%s)",
             (unsigned long)ok_count, (unsigned long)bad_count, (unsigned long)stress_bad,
             (unsigned long)(sweep_count * 200), (bad_count == 0) ? "PASS" : "FAIL");
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
