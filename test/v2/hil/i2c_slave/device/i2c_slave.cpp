// SPDX-License-Identifier: MIT
//
// HIL device firmware — I2C slave register file via the M5HAL v2 public API.
//
// Runs on a CoreS3SE (ESP32-S3) as an I2C slave at address 0x42 on SDA=2/SCL=1,
// using m5::hal::v2::i2c::SlaveBus_espidf (the LL clock-stretch backend) plus a
// SlaveRegMapAccessor. The accessor owns a 256-byte register file
// (reg_file[i] == i) with an auto-incrementing pointer and composes each read
// reply from the register file under clock stretch -- exactly the workload the
// raw-LL bench recipe validated, but driven entirely through the public M5HAL
// register-map accessor. This is the on-hardware consumer of SlaveBus_espidf +
// SlaveRegMapAccessor, so it doubles as the HIL acceptance fixture, and as the
// minimal "write an I2C slave register device with M5HAL" example: hand the
// accessor a backing array and call serve() in a loop.
//
// ESP-IDF app (NOT Arduino): the LL stretch backend needs IDF 5.x for the
// stretch-cause SoC caps and i2c_ll_* helpers, which the project's Arduino
// platform (espressif32@6.12.0 = IDF 4.4) does not ship. Built framework=espidf,
// like the raw-LL bench firmware whose recipe this backend distilled.
//
// Pair with a verification MASTER: a companion ESP-IDF I2C master (kept as a
// bench firmware in the project's private experiments tree, not published)
// flashed onto a Core2 (classic ESP32, SDA=32/SCL=33). The master sweeps the
// register file in three modes -- WTR (write-then-read, repeated-START, one
// transaction), SPLIT (register write, STOP, then a separate read) and WTEST
// (multi-byte write then read-back) -- at 100/400/800 kHz, n=1..64, and prints
// OK/BAD per transaction. Acceptance = BAD=0 across the whole matrix.
//
// An optional onRead hook turns register 0xFF into a read-counter (incremented
// each time the master reads it) so the master can prove live-value semantics:
// two reads of 0xFF return different values though the backing store never
// changed. The rest of the register file keeps its static reg_file[i] == i.
//
// Wiring (2-board HIL): Core2 SDA(32) <-> CoreS3SE SDA(2); Core2 SCL(33) <->
// CoreS3SE SCL(1); common GND. Pull-ups: the master enables internal pull-ups;
// add ~2.2-4.7k externals for reliable 800 kHz.
//
// Build / flash:
//   export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
//   pio run -e v2_hil_i2c_slave_device_esp32s3 -t upload

#include <M5HAL_v2.hpp>

#include <esp_log.h>

#if defined(M5HAL_TEST_SLAVE_REINIT_DEEP)
#include <soc/soc_caps.h>
#if !defined(SOC_RCC_IS_INDEPENDENT) || !SOC_RCC_IS_INDEPENDENT
#error \
    "M5HAL_TEST_SLAVE_REINIT_DEEP is an RCC-independent-SoC-only diag knob (C6/H2); RCC-shared SoCs need PERIPH_RCC_ATOMIC(), which this knob does not implement"
#endif
#include <hal/i2c_ll.h>
#endif

namespace m5hal = m5::hal::v2;

namespace {

// Pins default to the CoreS3SE wiring; other boards override via build_flags
// (e.g. the esp32c61 env sets SDA=5/SCL=6).
#ifndef M5HAL_HIL_I2C_PIN_SDA
#define M5HAL_HIL_I2C_PIN_SDA 2
#endif
#ifndef M5HAL_HIL_I2C_PIN_SCL
#define M5HAL_HIL_I2C_PIN_SCL 1
#endif
constexpr int PIN_SDA         = M5HAL_HIL_I2C_PIN_SDA;
constexpr int PIN_SCL         = M5HAL_HIL_I2C_PIN_SCL;
// Slave address defaults to 0x42 (the single-slave rigs); a multi-slave shared
// bus gives each board its own address via build_flags
// (e.g. -DM5HAL_HIL_I2C_SLAVE_ADDR=0x43).
#ifndef M5HAL_HIL_I2C_SLAVE_ADDR
#define M5HAL_HIL_I2C_SLAVE_ADDR 0x42
#endif
constexpr uint8_t SLAVE_ADDR  = M5HAL_HIL_I2C_SLAVE_ADDR;  // matches the verification master
constexpr uint8_t COUNTER_REG = 0xFF;  // reads return a live, incrementing value

m5hal::i2c::SlaveBus_espidf slave_bus;

// 256-byte register file: reg_file[i] == i, so reading register R for n bytes
// returns {R, R+1, ..., R+n-1} via the accessor's auto-incrementing pointer.
// The pointer persists across transactions so a SPLIT read (register write,
// STOP, then a pure read) resolves against the pointer set by the preceding
// write. The accessor stores write data straight into this array.
uint8_t reg_file[256];

// Live read-counter for COUNTER_REG: the accessor calls this for every byte of
// the read window; we only special-case the counter register and fall back to
// the backing store for everything else. Returning a fresh value each read is
// the differentiating point of a stretch-capable slave -- the value is produced
// just-in-time while the master is held under clock stretch.
// IRAM_ATTR: on a classic ESP32 (BE flavor) the accessor binds this hook into
// the backend's ISR regmap fast path, so it runs in interrupt context with the
// flash cache possibly disabled -- ISR-callable hooks must live in IRAM (see
// spec/design/i2c_slave.md). Harmless on the stretch (LL) flavors, where the
// hook runs in task context.
uint8_t read_counter = 0;

uint8_t IRAM_ATTR onReadRegister(uint8_t reg, void* ctx)
{
    (void)ctx;
    if (reg == COUNTER_REG) {
        return read_counter++;
    }
    return reg_file[reg];
}

void initRegFile()
{
    for (int i = 0; i < 256; ++i) {
        reg_file[i] = static_cast<uint8_t>(i);
    }
    read_counter = 0;
}

}  // namespace

extern "C" void app_main(void)
{
    // Boot banner: a healthy fixture is otherwise silent, so a rig with several
    // build-flag-addressed slaves has no way to tell which address a given
    // board serves without this line (a repurposed board left an
    // address unserved and the sweep's all-NACK run read as a master bug).
    ESP_LOGI("i2c_slave", "regmap slave fixture: addr=0x%02X SDA=%d SCL=%d", SLAVE_ADDR, PIN_SDA, PIN_SCL);
    initRegFile();

    m5hal::i2c::SlaveBusConfig cfg;
    cfg.pin_sda = PIN_SDA;
    cfg.pin_scl = PIN_SCL;
    cfg.address = SLAVE_ADDR;
// stretch = hold the master while the accessor composes the reply. Chips
// without stretch-cause support (classic ESP32) reject TxUnderrun::Stretch at
// init(), so those fall back to Fill. On those chips the accessor binds its
// register map into the backend's ISR fast path (bindIsrRegMap), which serves
// WTR/SPLIT reads from the ISR itself -- best-effort (no stretch to hold the
// master if the ISR is delayed), but correct data whenever the ISR wins the
// race, which the reference sweeps show it does through 400 kHz.
#if defined(SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE) && SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE
    cfg.tx_underrun = m5hal::i2c::TxUnderrun::Stretch;
#else
    cfg.tx_underrun = m5hal::i2c::TxUnderrun::Fill;
#endif
    cfg.tx_fill_byte       = 0xFF;  // only seen if the app never writes in time
    cfg.stretch_timeout_ms = 100;   // fall back to a fill byte past this budget
    cfg.timeout_ms         = 1000;

    auto init_result = slave_bus.init(cfg);
    if (!init_result.has_value()) {
        // Do not serve on a dead bus: a HIL fixture that silently never answers
        // is hard to diagnose. Surface the init error on the serial console
        // instead (also a reminder for the example's readers to check result_t).
        for (;;) {
            ESP_LOGE("i2c_slave", "SlaveBus_espidf::init failed (err=%d)", static_cast<int>(init_result.error()));
            m5hal::runtime::delayMs(1000);
        }
    }

    // The register-map accessor services each transaction: it reads the
    // register pointer, composes the read reply from reg_file (or onRead) under
    // stretch, and stores write data back into reg_file. serve() blocks until
    // one transaction completes; the backend ISR advances the bus meanwhile.
    m5hal::i2c::SlaveRegMapAccessor acc{slave_bus, m5hal::data::DataSpan{reg_file, sizeof(reg_file)}};
    acc.setOnRead(&onReadRegister, nullptr);

#if defined(M5HAL_TEST_REGMAP_SERVE_STALL_EVERY)
    uint32_t serve_count = 0;
#endif
#if defined(M5HAL_TEST_SLAVE_REINIT_AT_MS)
    bool reinit_done = false;
#endif
    for (;;) {
#ifdef M5HAL_TEST_REGMAP_SERVE_STALL_EVERY
        // Diag knob (opt-in, pair with M5HAL_TEST_REGMAP_SERVE_STALL_MS): stall
        // ONE serve() in every M5HAL_TEST_REGMAP_SERVE_STALL_EVERY calls, long
        // enough to exceed the master's transaction timeout budget, so the
        // master ABORTS that transaction mid-flight -- see i2c_echo.cpp's
        // identical knob for the mechanism. This is a regression trigger for
        // the master's STOP-on-abort contract: an aborted transaction must
        // never leave the bus unterminated, since a slave left mid-transaction
        // can misread the master's own bus-recovery pulses as continued data.
        // The surrounding serves run undelayed: the stalled round may fail,
        // but the register file must stay uncorrupted and the next round must
        // go through clean. Never define this for ordinary acceptance runs.
        if (++serve_count % M5HAL_TEST_REGMAP_SERVE_STALL_EVERY == 0) {
            m5hal::runtime::delayMs(M5HAL_TEST_REGMAP_SERVE_STALL_MS);
        }
#endif
#if defined(M5HAL_TEST_SLAVE_REINIT_AT_MS)
        // Diag knob (opt-in): once uptime crosses M5HAL_TEST_SLAVE_REINIT_AT_MS,
        // release() + init() the slave bus exactly once, without touching
        // reg_file / read_counter / acc. This isolates whether a SW-only
        // reinit of the M5HAL slave backend, on its own, clears a wedged H2
        // (i.e. does the wedge live in backend/driver state that release()+
        // init() resets, or does it survive a SW reinit and need a HW reset).
        // Never define this for ordinary acceptance runs.
        if (!reinit_done && m5hal::runtime::millis() > M5HAL_TEST_SLAVE_REINIT_AT_MS) {
            reinit_done = true;
            ESP_LOGI("i2c_slave", "diag reinit: begin (t=%lu ms)",
                     static_cast<unsigned long>(m5hal::runtime::millis()));
            auto release_result = slave_bus.release();
            if (release_result.has_value()) {
                ESP_LOGI("i2c_slave", "diag reinit: release() ok");
            } else {
                ESP_LOGE("i2c_slave", "diag reinit: release() failed (err=%d)",
                         static_cast<int>(release_result.error()));
            }
#if defined(M5HAL_TEST_SLAVE_REINIT_DEEP)
            // Diag knob (opt-in, requires M5HAL_TEST_SLAVE_REINIT_AT_MS): between
            // release() and init(), cycle the I2C peripheral's clock gates --
            // controller clock off, then bus clock off -- before letting init()
            // turn both back on (init() itself does bus-clock-on + reset_register
            // + controller-clock-on, see slave.inl). This isolates whether a
            // wedge that survives a plain SW reinit (release()+init() alone,
            // i.e. reset_register without a clock-gate cycle) is actually
            // cleared once the clock domain itself is dropped and re-raised --
            // i.e. does the wedged state live in clock-gated retained register
            // state that reset_register alone does not touch.
            {
                ::i2c_dev_t* hw = I2C_LL_GET_HW(I2C_NUM_0);
                // No `::` prefix on the i2c_ll_* calls below: on some SoCs these
                // are function-like macros, and prefixing a macro expansion with
                // `::` breaks (same caveat as slave.inl's init()).
                i2c_ll_enable_controller_clock(hw, false);
                // Bus clock off must come after the controller clock off: once
                // the bus clock is gated, the peripheral's registers (including
                // the controller clock enable bit itself) are no longer
                // accessible, so gating it first would leave the controller
                // clock call moot.
                i2c_ll_enable_bus_clock(I2C_NUM_0, false);
                m5hal::runtime::delayMs(10);
                ESP_LOGI("i2c_slave", "diag reinit: deep clock-gate cycle done");
            }
#endif
            auto reinit_result = slave_bus.init(cfg);
            if (reinit_result.has_value()) {
                ESP_LOGI("i2c_slave", "diag reinit: init() ok");
            } else {
                ESP_LOGE("i2c_slave", "diag reinit: init() failed (err=%d)", static_cast<int>(reinit_result.error()));
            }
            // Magic byte: proof the reinit sequence actually fired, readable by
            // the master at 400 kHz (a speed that still reads cleanly even while
            // wedged) without depending on reg_file's original content. This
            // deliberately corrupts one byte of reg_file, sacrificing the
            // "register file survived reinit intact" evidence (path B) for that
            // one register -- an acceptable trade because confirming the reinit
            // fired outweighs the single-byte content-preservation check.
            reg_file[0xF0] = 0xA5;
            ESP_LOGI("i2c_slave", "diag reinit: done");
        }
        // TIMEOUT_FOREVER would block serve() on a transaction that never
        // completes, so the reinit check above would never run past a wedge.
        // A finite budget returns control to the loop periodically instead;
        // the discarded return value matches the ordinary (non-knob) call.
        (void)acc.serve(500);
#else
        (void)acc.serve();
#endif
    }
}
