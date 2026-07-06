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

namespace m5hal = m5::hal::v2;

namespace {

constexpr int PIN_SDA         = 2;     // CoreS3SE I2C SDA
constexpr int PIN_SCL         = 1;     // CoreS3SE I2C SCL
constexpr uint8_t SLAVE_ADDR  = 0x42;  // matches the verification master
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
uint8_t read_counter = 0;

uint8_t onReadRegister(uint8_t reg, void* ctx)
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
    initRegFile();

    m5hal::i2c::SlaveBusConfig cfg;
    cfg.pin_sda = PIN_SDA;
    cfg.pin_scl = PIN_SCL;
    cfg.address = SLAVE_ADDR;
    // stretch = hold the master while the accessor composes the reply.
    cfg.tx_underrun        = m5hal::i2c::TxUnderrun::Stretch;
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

    for (;;) {
        (void)acc.serve();
    }
}
