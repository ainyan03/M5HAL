// SPDX-License-Identifier: MIT
//
// HIL MASTER firmware — I2C echo master via the M5HAL v2 public master API.
//
// Companion to ../device/i2c_echo.cpp (the M5HAL stream-echo SLAVE). This is the
// MASTER half of the "both ends on the M5HAL Source/Sink design" pair: it drives
// m5::hal::v2::i2c::Bus_espidf (the espidf master backend) through MasterAccessor's
// transfer(TransferDesc, data::Source*, data::Sink*) -- the same Source/Sink model
// the slave serves with serve(Source*, Sink*). No raw vendor i2c_master_* calls.
//
// Per round it runs the SPLIT echo contract against the slave at 0x42:
//   1. WRITE N bytes  -> transfer(desc, &MemorySource(tx_buf,N), nullptr)  (STOP)
//   2. READ  N bytes  -> transfer(desc, nullptr, &MemorySink(rx_buf,N))    (STOP)
// then verifies rx_buf == tx_buf and logs "[OK]"/"[BAD]" with first_bad (the index
// of the first mismatch, -1 = clean) so the HIL capture tooling tallies pass/fail.
// Two separate transfers = two separate transactions, matching the slave's SPLIT
// expectation (a combined write-then-read would not trip the slave's reply gate).
//
// It sweeps N over ring/FIFO boundaries up to 1 KB and cycles the SCL clock over
// 100/400/800 kHz internally (a fresh MasterAccessor per speed), so a single flash
// exercises the full matrix; read the master UART to tally OK/BAD.
//
// ESP-IDF app (NOT Arduino), framework=espidf (IDF 5.x): pairs with the espidf
// slave. Runs on a Core2 (ESP32) as the bus master on SDA=32/SCL=33.
//
// Wiring (2-board HIL): Core2 SDA(32) <-> CoreS3SE SDA(2); Core2 SCL(33) <->
// CoreS3SE SCL(1); common GND. Externals ~2.2-4.7k recommended for 800 kHz.
//
// Build / flash:
//   export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
//   pio run -e v2_hil_i2c_echo_master_esp32 -t upload

#include <M5HAL_v2.hpp>
#if defined(ECHO_MASTER_SOFTWARE)
// Software (bit-banged) I2C master variant: -DECHO_MASTER_SOFTWARE swaps Bus_espidf
// (vendor HW driver) for Bus_software, which bit-bangs SCL/SDA via M5_Hal.Gpio (no
// hardware FIFO). Used to isolate the 32-byte "息継ぎ": a HW master pauses every 32 B
// to refill its TX FIFO from the CPU; a bit-banged master has no FIFO, so if the
// write-side gap vanishes here it was the master's FIFO, not the slave. Bit-banging
// caps the achievable SCL well below the HW master -- read the analyzer for the real
// rate and compare the two masters at the same speed band.
#include <m5_hal/variants/frameworks/software/hal/i2c/i2c.hpp>
#endif

#include <esp_log.h>

#include <cstring>

namespace m5hal = m5::hal::v2;

namespace {

constexpr int PIN_SDA        = 32;    // Core2 PortA SDA
constexpr int PIN_SCL        = 33;    // Core2 PortA SCL
constexpr uint8_t SLAVE_ADDR = 0x42;  // matches the echo slave
constexpr char TAG[]         = "i2c_echo_master";

// Largest payload to round-trip (the 1 KB goal) + a sweep of lengths that straddle
// the slave's RX/TX ring (32) and HW FIFO (32) boundaries -- where back-pressure and
// the STOP tail are most stressed.
constexpr size_t kMaxLen       = 1024;
constexpr uint16_t kEchoLens[] = {1, 2, 16, 32, 63, 64, 65, 100, 128, 200, 256, 384, 512, 768, 1000, 1024};
constexpr uint32_t kSpeeds[]   = {100000, 400000, 800000};

uint8_t tx_buf[kMaxLen];
uint8_t rx_buf[kMaxLen];

// Fill tx_buf with one of a few patterns. Verification only compares rx_buf to
// tx_buf, so the exact pattern is immaterial -- variety just exercises byte values.
void fillPattern(size_t n, int pat)
{
    uint32_t lcg = 0x12345678u ^ (uint32_t)(n * 2654435761u);
    for (size_t i = 0; i < n; ++i) {
        switch (pat) {
            case 0:
                tx_buf[i] = 0x00;
                break;
            case 1:
                tx_buf[i] = 0xFF;
                break;
            case 2:
                tx_buf[i] = (uint8_t)i;
                break;  // count
            case 3:
                lcg       = lcg * 1664525u + 1013904223u;
                tx_buf[i] = (uint8_t)(lcg >> 24);
                break;  // pseudo-random
            default:
                tx_buf[i] = (uint8_t)((i & 1) ? 0xFF : 0x00);
                break;  // boundary
        }
    }
}

}  // namespace

extern "C" void app_main(void)
{
    // M5HAL master bus on PortA. Pins via the Scl/Sda tag constructors. The bus type
    // is the only difference between the HW (espidf) and bit-banged (software) masters
    // -- MasterAccessor::transfer() drives either through the same IBus interface.
#if defined(ECHO_MASTER_SOFTWARE)
    m5hal::i2c::BusConfig bus_cfg{m5hal::i2c::Scl{PIN_SCL}, m5hal::i2c::Sda{PIN_SDA}};
    m5hal::i2c::Bus_software bus;
#else
    m5hal::i2c::BusConfig bus_cfg{m5hal::i2c::Scl{PIN_SCL}, m5hal::i2c::Sda{PIN_SDA}};
    m5hal::i2c::Bus_espidf bus;
#endif
    auto bus_init = bus.init(bus_cfg);
    if (!bus_init.has_value()) {
        for (;;) {
            ESP_LOGE(TAG, "Bus_espidf::init failed (err=%d)", static_cast<int>(bus_init.error()));
            m5hal::runtime::delayMs(1000);
        }
    }

    const char* pat_name[] = {"zero", "ones", "count", "rand", "bound"};
    int round              = 0;

    for (;;) {
        for (uint32_t speed : kSpeeds) {
            // A MasterAccessor carries the per-target clock; a fresh one per speed
            // reconfigures the bus to that SCL for its transfers.
            m5hal::i2c::MasterAccessConfig acc_cfg;
            acc_cfg.i2c_addr        = SLAVE_ADDR;
            acc_cfg.freq            = speed;
            acc_cfg.wire_timeout_ms = 1000;
            m5hal::i2c::MasterAccessor dev{bus, acc_cfg};

            for (uint16_t n : kEchoLens) {
                for (int pat = 0; pat < 5; ++pat) {
                    fillPattern(n, pat);
                    ::memset(rx_buf, 0, n);

                    // 1. WRITE N bytes (Source -> bus), its own transaction (STOP).
                    m5hal::data::MemorySource src{m5hal::data::ConstDataSpan{tx_buf, n}};
                    auto wr = dev.transfer(m5hal::i2c::TransferDesc{}, &src, n, nullptr, 0);

                    // 2. READ N bytes back (bus -> Sink), a separate transaction (STOP).
                    m5hal::data::MemorySink sink{m5hal::data::DataSpan{rx_buf, n}};
                    auto rd = dev.transfer(m5hal::i2c::TransferDesc{}, nullptr, 0, &sink, n);

                    int first_bad = -1;
                    for (size_t i = 0; i < n; ++i) {
                        if (rx_buf[i] != tx_buf[i]) {
                            first_bad = (int)i;
                            break;
                        }
                    }
                    const bool ok = wr.has_value() && rd.has_value() && first_bad < 0;
                    ESP_LOGI(TAG, "#%d ECHO scl=%uk len=%4d pat=%-5s first_bad=%d [%s] wr=%d rd=%d", round++,
                             (unsigned)(speed / 1000), (int)n, pat_name[pat], first_bad, ok ? "OK" : "BAD",
                             wr.has_value() ? 0 : -(int)wr.error(), rd.has_value() ? 0 : -(int)rd.error());
                    m5hal::runtime::delayMs(2);  // brief gap between rounds
                }
            }
        }
    }
}
