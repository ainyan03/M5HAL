// SPDX-License-Identifier: MIT
#pragma once

// Fake mirror of ESP-IDF's soc/i2c_periph.h. The espidf slave backend reads
// `i2c_periph_signal[port]` only to (a) manually route SDA/SCL through the
// GPIO matrix at init() and (b) pick the interrupt source number for
// esp_intr_alloc(). Neither is load-bearing in this harness (GPIO routing is
// a no-op fake -- see ../driver/gpio.h -- and esp_intr_alloc.h ignores the
// source argument), so the signal numbers below are placeholders; only the
// field names/types and the array's I2C_NUM_0 entry need to exist.

#include <hal/i2c_types.h>

#include <cstdint>

struct i2c_signal_conn_t {
    uint32_t sda_out_sig;
    uint32_t sda_in_sig;
    uint32_t scl_out_sig;
    uint32_t scl_in_sig;
    int irq;
};

inline const i2c_signal_conn_t i2c_periph_signal[1] = {
    {0, 0, 0, 0, 0},  // I2C_NUM_0
};
