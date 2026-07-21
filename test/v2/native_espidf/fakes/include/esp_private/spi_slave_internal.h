// SPDX-License-Identifier: MIT
#pragma once

#include <driver/spi_slave.h>

inline esp_err_t spi_slave_queue_reset(spi_host_device_t)
{
    auto& state = m5hal_hostharness::fakeSpiSlave();
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.reset_count;
    if (!state.active || state.enabled || state.fail_reset) {
        return ESP_ERR_INVALID_STATE;
    }
    state.retired          = state.queued != nullptr ? state.queued : state.completed;
    state.retired_callback = state.callback;
    state.queued           = nullptr;
    state.completed        = nullptr;
    return ESP_OK;
}
