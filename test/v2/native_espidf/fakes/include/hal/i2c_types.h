// SPDX-License-Identifier: MIT
#pragma once

// Fake I2C type surface (mirrors ESP-IDF's hal/i2c_types.h enumerators the
// espidf slave backend references). Values are internal to this harness --
// only self-consistency with hal/i2c_ll.h and the test matters, not
// matching the real ESP-IDF numeric encoding.

enum i2c_port_t { I2C_NUM_0 = 0 };

enum i2c_clock_source_t { I2C_CLK_SRC_DEFAULT = 0 };

// i2c_ll_slave_get_read_write_status() return value: which direction the
// current bus transaction's address phase selected.
enum i2c_rw_t { I2C_SLAVE_WRITE_BY_MASTER = 0, I2C_SLAVE_READ_BY_MASTER = 1 };

enum i2c_slave_stretch_cause_t {
    I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH = 0,
    I2C_SLAVE_STRETCH_CAUSE_TX_EMPTY      = 1,
    I2C_SLAVE_STRETCH_CAUSE_RX_FULL       = 2,
};
