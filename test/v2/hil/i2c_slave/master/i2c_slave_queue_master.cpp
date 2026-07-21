// SPDX-License-Identifier: MIT
//
// Independent ESP-IDF master for device/i2c_slave_queue.cpp. This file uses
// only ESP-IDF's generation-5 I2C master driver; it does not use M5HAL.

#include <driver/i2c_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>
#include <cstring>

namespace {

constexpr int kPinSda            = 32;
constexpr int kPinScl            = 33;
constexpr uint8_t kAddress       = 0x42;
constexpr uint8_t kFillByte      = 0xE7;
constexpr int kTimeoutMs         = 100;
constexpr uint32_t kStartDelayMs = 1000;

constexpr uint8_t kPhase1Tx[] = {
    0x31, 0xA2, 0x13, 0xC4, 0x55, 0xE6, 0x77, 0x08, 0x99, 0x2A, 0xBB, 0x4C, 0xDD, 0x6E, 0xFF, 0x80, 0x11, 0x92,
};
constexpr size_t kPhase1Reads[]   = {3, 1, 5, 2, 7};
constexpr uint8_t kPhase1Write1[] = {0x41, 0x12, 0xE3, 0xB4};
constexpr uint8_t kPhase1Write2[] = {0x85, 0x56, 0x27, 0xF8, 0xC9, 0x9A};
constexpr uint8_t kPhase1Write3[] = {0x6B, 0x3C, 0x0D, 0xDE, 0xAF};

constexpr uint8_t kPhase2Tx[] = {
    0xD1, 0x42, 0xB3, 0x24, 0x95, 0x06, 0xF7, 0x68, 0xC9, 0x3A, 0xAB,
};
constexpr size_t kPhase2Reads[]   = {2, 4, 5};
constexpr uint8_t kPhase2Write1[] = {0x72, 0xE3, 0x54, 0xC5, 0x36};
constexpr uint8_t kPhase2Write2[] = {0xA7, 0x18, 0x89, 0xFA};

i2c_master_bus_handle_t bus = nullptr;
i2c_master_dev_handle_t dev = nullptr;
uint32_t checks             = 0;
uint32_t failures           = 0;

void delayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void check(bool pass, const char* name)
{
    ++checks;
    if (!pass) {
        ++failures;
    }
    std::printf("  %-30s : %s\n", name, pass ? "OK" : "BAD");
}

bool probe()
{
    return i2c_master_probe(bus, kAddress, 20) == ESP_OK;
}

bool waitForProbe(bool expected, uint32_t timeout_ms)
{
    const TickType_t started = xTaskGetTickCount();
    const TickType_t budget  = pdMS_TO_TICKS(timeout_ms);
    for (;;) {
        if (probe() == expected) {
            return true;
        }
        if (xTaskGetTickCount() - started >= budget) {
            return false;
        }
        delayMs(20);
    }
}

bool receiveExact(const uint8_t* expected, size_t size)
{
    uint8_t buffer[32] = {};
    if (size > sizeof(buffer)) {
        std::printf("      receive size %u exceeds buffer\n", static_cast<unsigned>(size));
        return false;
    }
    const esp_err_t error = i2c_master_receive(dev, buffer, size, kTimeoutMs);
    if (error != ESP_OK) {
        std::printf("      receive failed: %s\n", esp_err_to_name(error));
        return false;
    }
    if (std::memcmp(buffer, expected, size) == 0) {
        return true;
    }
    std::printf("      expected:");
    for (size_t i = 0; i < size; ++i) std::printf(" %02X", expected[i]);
    std::printf("\n      received:");
    for (size_t i = 0; i < size; ++i) std::printf(" %02X", buffer[i]);
    std::printf("\n");
    return false;
}

bool receiveFill(size_t size)
{
    uint8_t buffer[8] = {};
    if (size > sizeof(buffer)) {
        std::printf("      fill receive size %u exceeds buffer\n", static_cast<unsigned>(size));
        return false;
    }
    const esp_err_t error = i2c_master_receive(dev, buffer, size, kTimeoutMs);
    if (error != ESP_OK) {
        std::printf("      fill receive failed: %s\n", esp_err_to_name(error));
        return false;
    }
    for (size_t i = 0; i < size; ++i) {
        if (buffer[i] != kFillByte) {
            std::printf("      expected fill %02X, received:", kFillByte);
            for (size_t j = 0; j < size; ++j) std::printf(" %02X", buffer[j]);
            std::printf("\n");
            return false;
        }
    }
    return true;
}

bool transmit(const uint8_t* data, size_t size)
{
    return i2c_master_transmit(dev, data, size, kTimeoutMs) == ESP_OK;
}

template <size_t N>
bool verifyFragmentedReads(const uint8_t* expected, size_t expected_size, const size_t (&fragments)[N])
{
    size_t offset = 0;
    for (size_t i = 0; i < N; ++i) {
        if (offset + fragments[i] > expected_size || !receiveExact(expected + offset, fragments[i])) {
            std::printf("    fragment %u failed (offset=%u len=%u)\n", static_cast<unsigned>(i),
                        static_cast<unsigned>(offset), static_cast<unsigned>(fragments[i]));
            return false;
        }
        // Every receive call terminates its own wire frame with an early NACK.
        // The next call must therefore resume at the unclocked TX suffix.
        offset += fragments[i];
    }
    return offset == expected_size;
}

bool runPhase1()
{
    bool pass         = true;
    const bool suffix = verifyFragmentedReads(kPhase1Tx, sizeof(kPhase1Tx), kPhase1Reads);
    check(suffix, "phase1 early-NACK suffix");
    pass &= suffix;
    const bool fill = receiveFill(4);
    check(fill, "phase1 Fill underrun");
    pass &= fill;
    const bool write1 = transmit(kPhase1Write1, sizeof(kPhase1Write1));
    const bool write2 = transmit(kPhase1Write2, sizeof(kPhase1Write2));
    const bool write3 = transmit(kPhase1Write3, sizeof(kPhase1Write3));
    check(write1 && write2 && write3, "phase1 multiple writes");
    pass &= write1 && write2 && write3;
    return pass;
}

bool runPhase2()
{
    bool pass         = true;
    const bool suffix = verifyFragmentedReads(kPhase2Tx, sizeof(kPhase2Tx), kPhase2Reads);
    check(suffix, "phase2 early-NACK suffix");
    pass &= suffix;
    const bool fill = receiveFill(2);
    check(fill, "phase2 Fill underrun");
    pass &= fill;
    const bool write1 = transmit(kPhase2Write1, sizeof(kPhase2Write1));
    const bool write2 = transmit(kPhase2Write2, sizeof(kPhase2Write2));
    check(write1 && write2, "phase2 multiple writes");
    pass &= write1 && write2;
    return pass;
}

void initMaster()
{
    i2c_master_bus_config_t config      = {};
    config.i2c_port                     = 0;
    config.sda_io_num                   = static_cast<gpio_num_t>(kPinSda);
    config.scl_io_num                   = static_cast<gpio_num_t>(kPinScl);
    config.clk_source                   = I2C_CLK_SRC_DEFAULT;
    config.glitch_ignore_cnt            = 7;
    config.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&config, &bus));

    i2c_device_config_t device = {};
    device.dev_addr_length     = I2C_ADDR_BIT_LEN_7;
    device.device_address      = kAddress;
    device.scl_speed_hz        = 400000;
    device.scl_wait_us         = 10000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device, &dev));
}

}  // namespace

extern "C" void app_main(void)
{
    // Give the freshly flashed device a short startup margin. The final result
    // is repeated below, so monitors may attach after the wire run completes.
    delayMs(kStartDelayMs);
    std::printf("\n[i2c_slave_queue_master] slave=0x%02X SDA=%d SCL=%d\n", kAddress, kPinSda, kPinScl);
    initMaster();

    bool pass = waitForProbe(true, 5000);
    check(pass, "phase1 Access visible");
    if (pass) {
        pass &= runPhase1();
    }

    // The device advances only after finite endAccess and post-Access RX
    // validation succeed. Its deliberate 500 ms close interval makes both
    // the inactive edge and the following reinit visible to this master.
    const bool inactive1 = waitForProbe(false, 5000);
    check(inactive1, "phase1 finite end/close");
    pass &= inactive1;
    const bool active2 = inactive1 && waitForProbe(true, 5000);
    check(active2, "close/init/reuse visible");
    pass &= active2;
    if (active2) {
        pass &= runPhase2();
    }

    // Final address disappearance happens only after second-phase endAccess, RX
    // drain/validation, and close all succeeded on the device.
    const bool inactive2 = waitForProbe(false, 5000);
    check(inactive2, "phase2 validated and closed");
    pass &= inactive2;

    const bool final_pass = pass && failures == 0;
    for (;;) {
        std::printf("RESULT: checks=%lu failures=%lu (%s)\n", static_cast<unsigned long>(checks),
                    static_cast<unsigned long>(failures), final_pass ? "PASS" : "FAIL");
        delayMs(5000);
    }
}
