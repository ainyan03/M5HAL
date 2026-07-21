// SPDX-License-Identifier: MIT
#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>

#include <condition_variable>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>

enum spi_host_device_t : int { SPI1_HOST = 0, SPI2_HOST = 1, SPI3_HOST = 2 };

constexpr int SPI_DMA_CH_AUTO             = 0;
constexpr uint32_t SPI_SLAVE_BIT_LSBFIRST = 1u;

struct spi_bus_config_t {
    int mosi_io_num;
    int miso_io_num;
    int sclk_io_num;
    int quadwp_io_num;
    int quadhd_io_num;
    int max_transfer_sz;
};

struct spi_slave_transaction_t {
    size_t length         = 0;
    size_t trans_len      = 0;
    const void* tx_buffer = nullptr;
    void* rx_buffer       = nullptr;
    void* user            = nullptr;
};

using spi_slave_cb_t = void (*)(spi_slave_transaction_t*);

struct spi_slave_interface_config_t {
    int spics_io_num;
    int queue_size;
    int mode;
    uint32_t flags;
    spi_slave_cb_t post_trans_cb;
};

namespace m5hal_hostharness {

struct FakeSpiSlave {
    std::mutex mutex;
    std::condition_variable cv;
    bool active                        = false;
    bool enabled                       = false;
    bool fail_initialize               = false;
    bool fail_disable                  = false;
    bool fail_reset                    = false;
    bool fail_free                     = false;
    bool hold_queue                    = false;
    bool queue_entered                 = false;
    spi_slave_transaction_t* queued    = nullptr;
    spi_slave_transaction_t* completed = nullptr;
    spi_slave_transaction_t* retired   = nullptr;
    spi_slave_cb_t callback            = nullptr;
    spi_slave_cb_t retired_callback    = nullptr;
    uint32_t initialize_count          = 0;
    uint32_t disable_count             = 0;
    uint32_t reset_count               = 0;
    uint32_t free_count                = 0;
    uint32_t queue_count               = 0;
    uint32_t result_count              = 0;
};

inline FakeSpiSlave& fakeSpiSlave()
{
    static FakeSpiSlave state;
    return state;
}

inline void resetSpiSlave()
{
    auto& state = fakeSpiSlave();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.active           = false;
    state.enabled          = false;
    state.fail_initialize  = false;
    state.fail_disable     = false;
    state.fail_reset       = false;
    state.fail_free        = false;
    state.hold_queue       = false;
    state.queue_entered    = false;
    state.queued           = nullptr;
    state.completed        = nullptr;
    state.retired          = nullptr;
    state.callback         = nullptr;
    state.retired_callback = nullptr;
    state.initialize_count = 0;
    state.disable_count    = 0;
    state.reset_count      = 0;
    state.free_count       = 0;
    state.queue_count      = 0;
    state.result_count     = 0;
}

inline bool completeSpiTransaction(const uint8_t* rx, size_t bytes)
{
    spi_slave_transaction_t* transaction = nullptr;
    spi_slave_cb_t callback              = nullptr;
    {
        auto& state = fakeSpiSlave();
        std::lock_guard<std::mutex> lock(state.mutex);
        // Calling this helper represents a transaction that the master already
        // started. It may finish after the slave has fenced new acceptance.
        if (!state.active || state.queued == nullptr) {
            return false;
        }
        transaction            = state.queued;
        state.queued           = nullptr;
        transaction->trans_len = bytes * 8;
        if (rx != nullptr && transaction->rx_buffer != nullptr) {
            std::memcpy(transaction->rx_buffer, rx, bytes);
        }
        state.completed = transaction;
        callback        = state.callback;
    }
    if (callback != nullptr) {
        callback(transaction);
    }
    return true;
}

inline void releaseHeldQueue()
{
    auto& state = fakeSpiSlave();
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.hold_queue = false;
    }
    state.cv.notify_all();
}

inline bool waitForQueuedTransaction(uint32_t timeout_ms = 1000)
{
    auto& state = fakeSpiSlave();
    std::unique_lock<std::mutex> lock(state.mutex);
    return state.cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] { return state.queued != nullptr; });
}

inline bool fireRetiredCallback()
{
    spi_slave_transaction_t* transaction = nullptr;
    spi_slave_cb_t callback              = nullptr;
    {
        auto& state = fakeSpiSlave();
        std::lock_guard<std::mutex> lock(state.mutex);
        transaction = state.retired;
        callback    = state.retired_callback;
    }
    if (transaction == nullptr || callback == nullptr) {
        return false;
    }
    callback(transaction);
    return true;
}

}  // namespace m5hal_hostharness

inline esp_err_t spi_slave_initialize(spi_host_device_t, const spi_bus_config_t*,
                                      const spi_slave_interface_config_t* config, int)
{
    auto& state = m5hal_hostharness::fakeSpiSlave();
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.initialize_count;
    if (state.fail_initialize || state.active) {
        return ESP_ERR_INVALID_STATE;
    }
    state.active   = true;
    state.enabled  = true;
    state.callback = config->post_trans_cb;
    return ESP_OK;
}

inline esp_err_t spi_slave_disable(spi_host_device_t)
{
    auto& state = m5hal_hostharness::fakeSpiSlave();
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.disable_count;
    if (!state.active || !state.enabled || state.fail_disable) {
        return ESP_ERR_INVALID_STATE;
    }
    state.enabled = false;
    return ESP_OK;
}

inline esp_err_t spi_slave_enable(spi_host_device_t)
{
    auto& state = m5hal_hostharness::fakeSpiSlave();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.active || state.enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    state.enabled = true;
    return ESP_OK;
}

inline esp_err_t spi_slave_free(spi_host_device_t)
{
    auto& state = m5hal_hostharness::fakeSpiSlave();
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.free_count;
    if (!state.active || state.fail_free) {
        return ESP_ERR_INVALID_STATE;
    }
    // Successful free synchronously removes the ISR source. Retired callbacks
    // remain observable only when free fails and the driver still owns them.
    state.retired          = nullptr;
    state.retired_callback = nullptr;
    state.active           = false;
    state.enabled          = false;
    state.queued           = nullptr;
    state.completed        = nullptr;
    state.callback         = nullptr;
    return ESP_OK;
}

inline esp_err_t spi_slave_queue_trans(spi_host_device_t, spi_slave_transaction_t* transaction, TickType_t)
{
    auto& state = m5hal_hostharness::fakeSpiSlave();
    std::unique_lock<std::mutex> lock(state.mutex);
    ++state.queue_count;
    state.queue_entered = true;
    state.cv.notify_all();
    state.cv.wait(lock, [&] { return !state.hold_queue; });
    if (!state.active || state.queued != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    state.queued = transaction;
    state.cv.notify_all();
    return ESP_OK;
}

inline esp_err_t spi_slave_get_trans_result(spi_host_device_t, spi_slave_transaction_t** out, TickType_t)
{
    auto& state = m5hal_hostharness::fakeSpiSlave();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.completed == nullptr) {
        return ESP_ERR_TIMEOUT;
    }
    *out            = state.completed;
    state.completed = nullptr;
    ++state.result_count;
    return ESP_OK;
}
