// SPDX-License-Identifier: MIT
//
// HIL device firmware — I2C slave STREAM echo via the M5HAL v2 public API.
//
// Companion to i2c_slave.cpp (the register-map device). Where that one composes
// replies from a 256-byte register file through SlaveRegMapAccessor, this one is
// the *pure-stream* fixture: it drives m5::hal::v2::i2c::SlaveBus_espidf (the LL
// clock-stretch backend) through SlaveStreamAccessor DIRECTLY -- no register
// pointer, no addressing. A transaction is just an opaque byte run.
//
// Echo contract (SPLIT-style, two transactions per round):
//   1. The master WRITEs N payload bytes (no register byte). serve() pushes them
//      into a MemorySink over echo_buf; echo_len = the bytes captured.
//   2. The master then READs N bytes in a SEPARATE transaction. serve() pulls the
//      reply from a MemorySource over echo_buf[0..echo_len].
// So whatever the master writes, it reads back verbatim. Each serve() call handles
// ONE transaction with the Source/Sink streaming model (the slave counterpart of the
// master's transfer(Source*, Sink*)). It exercises BOTH per-transaction caps removed
// in S21, now with back-pressure rather than truncation:
//   - the RX ring (a single write far exceeds kRxCapacity; the RX_FULL stretch backs
//     the master off while serve() drains into the Sink -- no byte dropped), and
//   - the TX ring + TX_EMPTY underrun stretch (a single read far exceeds kTxCapacity;
//     the backend holds the master under stretch when the ring empties and resumes
//     as serve() pulls more reply bytes from the Source).
// Target: a 1 KB write/read set round-trips byte-perfect across data patterns at
// 100/400/800 kHz. serve() is event-driven (woken by the backend ISR), so it drains
// promptly enough that the ring is never full at the master's STOP.
//
// Direction is discovered at run time inside serve(), not queried (the bus exposes
// no direction): a write delivers received bytes into the Sink, a read pulls reply
// bytes from the Source; serve() only composes a reply once nothing has been received
// this transaction, so a writing master never triggers wasted reply work.
//
// ESP-IDF app (NOT Arduino), framework=espidf (IDF 5.x): the LL stretch backend
// needs the stretch-cause SoC caps + i2c_ll_* helpers the project's Arduino
// platform (espressif32@6.12.0 = IDF 4.4) does not ship. Runs on a CoreS3SE
// (ESP32-S3) as slave 0x42 on SDA=2/SCL=1.
//
// Pair with a verification MASTER firmware (ROLE_MASTER -DECHO_MODE):
// it sweeps N over {1..1024} x {all-0, all-1, counter, pseudo-random, boundary},
// writing then reading each and printing OK/BAD per round. Acceptance = BAD=0.
//
// Wiring (2-board HIL): Core2 SDA(32) <-> CoreS3SE SDA(2); Core2 SCL(33) <->
// CoreS3SE SCL(1); common GND. Externals ~2.2-4.7k recommended for 800 kHz.
//
// Build / flash:
//   export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
//   pio run -e v2_hil_i2c_slave_echo_device_esp32s3 -t upload

#include <M5HAL_v2.hpp>

#include <esp_log.h>

namespace m5hal = m5::hal::v2;

namespace {

constexpr int PIN_SDA        = 2;     // CoreS3SE I2C SDA
constexpr int PIN_SCL        = 1;     // CoreS3SE I2C SCL
constexpr uint8_t SLAVE_ADDR = 0x42;  // matches the verification master

// Largest single-transaction payload the echo round-trips. The buffer bounds the
// captured write; a master writing more than this fills the Sink, which back-pressures
// the master via the RX ring stretch (no tail drop) -- a no-op for the 1 KB target.
constexpr size_t kEchoCap = 1024;

m5hal::i2c::SlaveBus_espidf slave_bus;

// The captured payload to echo back. echo_len persists across transactions so the
// READ transaction can replay the bytes of the preceding WRITE transaction; it is
// reset to 0 only when a new write delivers its first byte.
uint8_t echo_buf[kEchoCap];
size_t echo_len    = 0;
size_t echo_offset = 0;

}  // namespace

extern "C" void app_main(void)
{
    m5hal::i2c::SlaveBusConfig cfg;
    cfg.pin_sda = PIN_SDA;
    cfg.pin_scl = PIN_SCL;
    cfg.address = SLAVE_ADDR;
    // stretch = hold the master while this loop captures a long write / streams a
    // long reply. tx_fill_byte is only ever seen if a read outruns echo_len past
    // the stretch budget (our master reads exactly what it wrote, so never).
    cfg.tx_underrun        = m5hal::i2c::TxUnderrun::Stretch;
    cfg.tx_fill_byte       = 0xFF;
    cfg.stretch_timeout_ms = 100;
    cfg.timeout_ms         = 1000;

    auto init_result = slave_bus.init(cfg);
    if (!init_result.has_value()) {
        // Do not serve on a dead bus: a HIL fixture that silently never answers is
        // hard to diagnose. Surface the init error on the serial console instead.
        for (;;) {
            ESP_LOGE("i2c_echo", "SlaveBus_espidf::init failed (err=%d)", static_cast<int>(init_result.error()));
            m5hal::runtime::delayMs(1000);
        }
    }

    // Drive the backend with the bare stream accessor and its Source/Sink serve():
    // each call handles ONE transaction -- received bytes flow into the Sink, reply
    // bytes are pulled from the Source. This IS the "write an I2C slave that echoes a
    // byte stream with M5HAL" minimal example (SPI-transfer-style usage).
    m5hal::i2c::SlaveStreamAccessor acc{slave_bus};

    // One buffer reused as both the Sink (capture a write) and the Source (replay it
    // on the following read). echo_len tracks the last write's length so the read
    // transaction replays exactly those bytes -- the SPLIT echo contract. A write
    // overwrites echo_buf from offset 0 (fresh Sink); a read commits nothing, so
    // echo_len and the buffer persist for it. Back-pressure (RX ring stretch, TX
    // underrun stretch) is handled inside serve()/the backend, not here.
    for (;;) {
        m5hal::data::MemorySink sink{m5hal::data::DataSpan{echo_buf + echo_offset, kEchoCap - echo_offset}};
        m5hal::data::MemorySource src{m5hal::data::ConstDataSpan{echo_buf, echo_len}};

        auto n = acc.serve(&src, &sink);
        if (!n.has_value()) {
            continue;
        }
        if (sink.written() > 0) {
            echo_offset += sink.written();
            echo_len = echo_offset;
        } else {
            echo_offset = 0;
            echo_len    = 0;
        }
    }
}
