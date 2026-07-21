// SPDX-License-Identifier: MIT
#pragma once

// Fake mirror of ESP-IDF's hal/i2c_ll.h -- the host harness's I2C device
// model. Two groups of functions:
//
//   * Init-only / config LL calls (pin timing, FIFO thresholds, ...): no-ops.
//     Clock enables/disables are the exception: they append to a deterministic
//     trace so init/close ownership and ordering can be asserted.
//   * State-machine LL calls (interrupt mask, RX/TX FIFO, slave direction,
//     stretch cause): operate on the `i2c_dev_t` fake device model (see
//     soc/i2c_struct.h). These are the ONLY functions a test's assertions
//     and setup should reason about; a test scripts an ISR event by writing
//     directly into the `i2c_dev_t` fields these functions read, then calls
//     m5hal_hostharness::fireLastIsr() (esp_intr_alloc.h) to run the
//     captured handler synchronously. See ../../README.md.
//
// I2C_LL_GET_HW(port) returns a pointer to a process-wide singleton per
// port, matching real hardware (I2C_NUM_0 is one physical peripheral, so
// every SlaveBus_espidf instance that inits port 0 shares the same
// register file on real silicon too). init() resets the fields the state
// machine relies on (FIFOs, interrupt mask, stretch); a test creating a
// fresh SlaveBus_espidf per TEST case and calling init() first therefore
// gets a clean model, but the diagnostic-only rst counters are NOT reset --
// see soc/i2c_struct.h.

#include <soc/i2c_struct.h>
#include <hal/i2c_types.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

// ---- interrupt-mask bit assignments (harness-internal; only need to be
// self-consistent with the product code's unqualified use of these names) --
//
// Real `#define` macros, NOT `constexpr` -- the BE flavor's slave.inl probes
// for the WM names with `#if defined(I2C_RXFIFO_WM_INT_ENA_M)` (classic-ESP32
// real headers expose FULL/EMPTY names instead; every other slave-capable SoC
// exposes WM). `defined()` only sees preprocessor macros, so a `constexpr`
// declaration here would make that probe always resolve to the FULL/EMPTY
// fallback names, which this fake does not define -- a link/compile break
// only the BE host-harness env would hit (the LL env never takes that #if).
#define I2C_RXFIFO_WM_INT_ENA_M      (1u << 0)
#define I2C_TXFIFO_WM_INT_ENA_M      (1u << 1)
#define I2C_TRANS_COMPLETE_INT_ENA_M (1u << 2)
#define I2C_SLAVE_STRETCH_INT_ENA_M  (1u << 3)
#define I2C_LL_INTR_MASK \
    (I2C_RXFIFO_WM_INT_ENA_M | I2C_TXFIFO_WM_INT_ENA_M | I2C_TRANS_COMPLETE_INT_ENA_M | I2C_SLAVE_STRETCH_INT_ENA_M)
#define I2C_LL_MAX_TIMEOUT 0xFFFFFFFFu

namespace m5hal_hostharness {

enum class I2cClockEvent : uint8_t {
    BusEnable,
    ControllerEnable,
    ControllerDisable,
    BusDisable,
};

inline std::vector<I2cClockEvent>& i2cClockTrace()
{
    static std::vector<I2cClockEvent> trace;
    return trace;
}

inline void resetI2cClockTrace()
{
    i2cClockTrace().clear();
}

inline i2c_dev_t& i2cDeviceFor(i2c_port_t)
{
    static i2c_dev_t device;
    return device;
}

}  // namespace m5hal_hostharness

#define I2C_LL_GET_HW(port) (&::m5hal_hostharness::i2cDeviceFor(port))

// ---- init-only / config --------------------------------------------------
inline void i2c_ll_enable_bus_clock(i2c_port_t, bool enable)
{
    m5hal_hostharness::i2cClockTrace().push_back(enable ? m5hal_hostharness::I2cClockEvent::BusEnable
                                                        : m5hal_hostharness::I2cClockEvent::BusDisable);
}
inline void i2c_ll_reset_register(i2c_port_t)
{
}
inline void i2c_ll_enable_controller_clock(i2c_dev_t*, bool enable)
{
    m5hal_hostharness::i2cClockTrace().push_back(enable ? m5hal_hostharness::I2cClockEvent::ControllerEnable
                                                        : m5hal_hostharness::I2cClockEvent::ControllerDisable);
}
inline void i2c_ll_set_source_clk(i2c_dev_t*, i2c_clock_source_t)
{
}
inline void i2c_ll_master_rx_full_ack_level(i2c_dev_t*, int)
{
}
inline void i2c_ll_slave_enable_auto_start(i2c_dev_t*, bool)
{
}
inline void i2c_ll_set_slave_addr(i2c_dev_t* hw, uint16_t address, bool address_10bit)
{
    hw->pending_slave_address       = address;
    hw->pending_slave_address_10bit = address_10bit;
}
inline void i2c_ll_set_tout(i2c_dev_t*, uint32_t)
{
}
inline void i2c_ll_set_sda_timing(i2c_dev_t*, int, int)
{
}
inline void i2c_ll_master_set_filter(i2c_dev_t*, uint8_t)
{
}
inline void i2c_ll_set_rxfifo_full_thr(i2c_dev_t*, uint8_t)
{
}
inline void i2c_ll_set_txfifo_empty_thr(i2c_dev_t*, uint8_t)
{
}
inline void i2c_ll_enable_fifo_mode(i2c_dev_t*, bool)
{
}
inline void i2c_ll_slave_enable_scl_stretch(i2c_dev_t*, bool)
{
}
inline void i2c_ll_slave_set_stretch_protect_num(i2c_dev_t*, uint32_t)
{
}
inline void i2c_ll_update(i2c_dev_t* hw)
{
    hw->slave_address       = hw->pending_slave_address;
    hw->slave_address_10bit = hw->pending_slave_address_10bit;
    ++hw->update_count;
}

// ---- state-machine LL calls: the fake device model -----------------------

inline void i2c_ll_enable_intr_mask(i2c_dev_t* hw, uint32_t mask)
{
    hw->int_ena |= mask;
}
inline void i2c_ll_disable_intr_mask(i2c_dev_t* hw, uint32_t mask)
{
    hw->int_ena &= ~mask;
}
inline void i2c_ll_clear_intr_mask(i2c_dev_t* hw, uint32_t mask)
{
    hw->int_st &= ~mask;
}
inline void i2c_ll_get_intr_mask(i2c_dev_t* hw, uint32_t* out)
{
    *out = hw->int_st & hw->int_ena;
}

inline void i2c_ll_txfifo_rst(i2c_dev_t* hw)
{
    hw->txfifo_count = 0;
    ++hw->txfifo_rst_count;
}
inline void i2c_ll_rxfifo_rst(i2c_dev_t* hw)
{
    hw->rxfifo_count = 0;
    ++hw->rxfifo_rst_count;
}

inline void i2c_ll_get_rxfifo_cnt(i2c_dev_t* hw, uint32_t* out)
{
    *out = static_cast<uint32_t>(hw->rxfifo_count);
}

// Consumes from the FRONT of the fake RX FIFO (index 0) and shifts the
// remainder down -- matches a real FIFO's read-then-drop semantics. See
// soc/i2c_struct.h's note on how a test must prime a second batch.
inline void i2c_ll_read_rxfifo(i2c_dev_t* hw, uint8_t* dst, uint32_t len)
{
    const size_t n = std::min(static_cast<size_t>(len), hw->rxfifo_count);
    std::memcpy(dst, hw->rxfifo, n);
    if (n < hw->rxfifo_count) {
        std::memmove(hw->rxfifo, hw->rxfifo + n, hw->rxfifo_count - n);
    }
    hw->rxfifo_count -= n;
}

// "Free space" semantics, matching the product code's `freelen` usage
// (i2c_ll_get_txfifo_len feeds directly into "how many bytes can I push
// now").
inline void i2c_ll_get_txfifo_len(i2c_dev_t* hw, uint32_t* out)
{
    *out = static_cast<uint32_t>(i2c_dev_t::kFifoLen - std::min(hw->txfifo_count, i2c_dev_t::kFifoLen));
}
inline void i2c_ll_write_txfifo(i2c_dev_t* hw, const uint8_t* src, uint32_t len)
{
    const size_t space = i2c_dev_t::kFifoLen - std::min(hw->txfifo_count, i2c_dev_t::kFifoLen);
    const size_t n     = std::min(static_cast<size_t>(len), space);
    std::memcpy(hw->txfifo + hw->txfifo_count, src, n);
    hw->txfifo_count += n;
}

inline int i2c_ll_slave_get_read_write_status(i2c_dev_t* hw)
{
    return hw->slave_rw;
}
inline bool i2c_ll_is_bus_busy(i2c_dev_t* hw)
{
    return hw->bus_busy;
}
inline void i2c_ll_slave_get_stretch_cause(i2c_dev_t* hw, i2c_slave_stretch_cause_t* out)
{
    *out = static_cast<i2c_slave_stretch_cause_t>(hw->stretch_cause);
}
inline void i2c_ll_slave_clear_stretch(i2c_dev_t* hw)
{
    hw->stretch_active = false;
}
