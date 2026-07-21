// SPDX-License-Identifier: MIT
//
// Two-board HIL device for the queue-driven public I2C slave API.
//
// CoreS3 (this file): SDA=GPIO2, SCL=GPIO1, address 0x42.
// Core2 master:       SDA=GPIO32, SCL=GPIO33.
//
// The companion independent ESP-IDF master performs two phases. In each
// phase this device preloads TX before beginAccess(), accepts several distinct
// master writes, stops acceptance with a finite endAccess(), and validates the
// concatenated RX bytes only after Access has ended. Between phases the bus is
// closed and initialized again while the same caller-owned queues and
// SlaveAccessor are reused.

#include <M5HAL_v2.hpp>

#include <esp_log.h>

#include <cstring>

namespace m5hal = m5::hal::v2;

namespace {

constexpr int kPinSda            = 2;
constexpr int kPinScl            = 1;
constexpr uint8_t kAddress       = 0x42;
constexpr uint8_t kFillByte      = 0xE7;
constexpr uint32_t kEndTimeout   = 1000;
constexpr uint32_t kPhaseTimeout = 120000;

constexpr uint8_t kPhase1Tx[] = {
    0x31, 0xA2, 0x13, 0xC4, 0x55, 0xE6, 0x77, 0x08, 0x99, 0x2A, 0xBB, 0x4C, 0xDD, 0x6E, 0xFF, 0x80, 0x11, 0x92,
};
constexpr uint8_t kPhase1Rx[] = {
    0x41, 0x12, 0xE3, 0xB4, 0x85, 0x56, 0x27, 0xF8, 0xC9, 0x9A, 0x6B, 0x3C, 0x0D, 0xDE, 0xAF,
};

constexpr uint8_t kPhase2Tx[] = {
    0xD1, 0x42, 0xB3, 0x24, 0x95, 0x06, 0xF7, 0x68, 0xC9, 0x3A, 0xAB,
};
constexpr uint8_t kPhase2Rx[] = {
    0x72, 0xE3, 0x54, 0xC5, 0x36, 0xA7, 0x18, 0x89, 0xFA,
};

m5hal::i2c::SlaveBus_espidf slave_bus;
m5hal::slave::StaticSlaveQueueStorage<64, 64, 0, 0> queue_storage;
m5hal::i2c::StaticI2cSegmentStorage<0> segment_storage;
m5hal::i2c::SlaveAccessConfig access_config;
m5hal::i2c::SlaveAccessor slave_accessor{slave_bus, queue_storage.tx(), queue_storage.rx(), segment_storage.storage(),
                                         access_config};

m5hal::i2c::SlaveBusConfig makeBusConfig()
{
    m5hal::i2c::SlaveBusConfig config;
    config.pin_sda      = kPinSda;
    config.pin_scl      = kPinScl;
    config.address      = kAddress;
    config.tx_underrun  = m5hal::i2c::TxUnderrun::Fill;
    config.tx_fill_byte = kFillByte;
    config.timeout_ms   = kEndTimeout;
    return config;
}

bool awaitRx(size_t expected, uint32_t timeout_ms)
{
    const uint32_t started = m5hal::runtime::millis();
    while (slave_accessor.readable() < expected) {
        if (m5hal::runtime::millis() - started >= timeout_ms) {
            ESP_LOGE("i2c_queue", "RX timeout: have=%u expected=%u", static_cast<unsigned>(slave_accessor.readable()),
                     static_cast<unsigned>(expected));
            return false;
        }
        m5hal::runtime::delayMs(1);
    }
    return true;
}

bool drainAndVerify(const uint8_t* expected, size_t size)
{
    uint8_t received[64] = {};
    auto read            = slave_accessor.read({received, size});
    if (!read.has_value() || *read != size) {
        ESP_LOGE("i2c_queue", "post-Access drain failed: read=%d err=%d expected=%u",
                 read.has_value() ? static_cast<int>(*read) : -1, read.has_value() ? 0 : static_cast<int>(read.error()),
                 static_cast<unsigned>(size));
        return false;
    }
    if (std::memcmp(received, expected, size) != 0) {
        for (size_t i = 0; i < size; ++i) {
            if (received[i] != expected[i]) {
                ESP_LOGE("i2c_queue", "RX mismatch at %u: got=0x%02X expected=0x%02X", static_cast<unsigned>(i),
                         received[i], expected[i]);
                break;
            }
        }
        return false;
    }
    if (slave_accessor.readable() != 0) {
        ESP_LOGE("i2c_queue", "unexpected RX suffix: %u bytes", static_cast<unsigned>(slave_accessor.readable()));
        return false;
    }
    return true;
}

bool runPhase(unsigned phase, const uint8_t* tx, size_t tx_size, const uint8_t* expected_rx, size_t rx_size)
{
    auto preloaded = slave_accessor.write({tx, tx_size});
    if (!preloaded.has_value() || *preloaded != tx_size) {
        ESP_LOGE("i2c_queue", "phase %u TX preload failed (err=%d count=%d)", phase,
                 preloaded.has_value() ? 0 : static_cast<int>(preloaded.error()),
                 preloaded.has_value() ? static_cast<int>(*preloaded) : -1);
        return false;
    }

    auto begun = slave_accessor.beginAccess(1000);
    if (!begun.has_value()) {
        ESP_LOGE("i2c_queue", "phase %u beginAccess failed (err=%d)", phase, static_cast<int>(begun.error()));
        return false;
    }
    ESP_LOGI("i2c_queue", "phase %u accepting: tx=%u rx=%u", phase, static_cast<unsigned>(tx_size),
             static_cast<unsigned>(rx_size));

    if (!awaitRx(rx_size, kPhaseTimeout)) {
        (void)slave_accessor.endAccess(kEndTimeout);
        return false;
    }

    // endAccess fences new address matches and waits only up to this explicit
    // finite budget for the final already-accepted write to reach STOP.
    const uint32_t end_started = m5hal::runtime::millis();
    auto ended                 = slave_accessor.endAccess(kEndTimeout);
    const uint32_t end_elapsed = m5hal::runtime::millis() - end_started;
    if (!ended.has_value()) {
        ESP_LOGE("i2c_queue", "phase %u endAccess failed after %lu ms (err=%d)", phase,
                 static_cast<unsigned long>(end_elapsed), static_cast<int>(ended.error()));
        return false;
    }
    if (end_elapsed > kEndTimeout + 50) {
        ESP_LOGE("i2c_queue", "phase %u endAccess exceeded finite budget: %lu ms", phase,
                 static_cast<unsigned long>(end_elapsed));
        return false;
    }
    if (!drainAndVerify(expected_rx, rx_size)) {
        return false;
    }

    ESP_LOGI("i2c_queue", "phase %u PASS: endAccess=%lu ms", phase, static_cast<unsigned long>(end_elapsed));
    return true;
}

[[noreturn]] void finish(bool pass)
{
    for (;;) {
        ESP_LOGI("i2c_queue", "RESULT: %s", pass ? "PASS" : "FAIL");
        m5hal::runtime::delayMs(5000);
    }
}

}  // namespace

extern "C" void app_main(void)
{
    ESP_LOGI("i2c_queue", "queued slave fixture: addr=0x%02X SDA=%d SCL=%d fill=0x%02X", kAddress, kPinSda, kPinScl,
             kFillByte);

    const auto bus_config = makeBusConfig();
    auto initialized      = slave_bus.init(bus_config);
    if (!initialized.has_value()) {
        ESP_LOGE("i2c_queue", "initial init failed (err=%d)", static_cast<int>(initialized.error()));
        finish(false);
    }
    if (!runPhase(1, kPhase1Tx, sizeof(kPhase1Tx), kPhase1Rx, sizeof(kPhase1Rx))) {
        (void)slave_bus.close();
        finish(false);
    }

    auto closed = slave_bus.close();
    if (!closed.has_value()) {
        ESP_LOGE("i2c_queue", "close between phases failed (err=%d)", static_cast<int>(closed.error()));
        finish(false);
    }

    // Make the inactive interval observable to the master. This is also a
    // positive proof that beginAccess does not leave the address accepted.
    m5hal::runtime::delayMs(500);

    initialized = slave_bus.init(bus_config);
    if (!initialized.has_value()) {
        ESP_LOGE("i2c_queue", "reinit failed (err=%d)", static_cast<int>(initialized.error()));
        finish(false);
    }
    if (!runPhase(2, kPhase2Tx, sizeof(kPhase2Tx), kPhase2Rx, sizeof(kPhase2Rx))) {
        (void)slave_bus.close();
        finish(false);
    }

    closed = slave_bus.close();
    if (!closed.has_value()) {
        ESP_LOGE("i2c_queue", "final close failed (err=%d)", static_cast<int>(closed.error()));
        finish(false);
    }
    finish(true);
}
