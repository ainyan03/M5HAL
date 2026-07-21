// SPDX-License-Identifier: MIT
#pragma once

// Fake mirror of ESP-IDF's soc/i2c_struct.h -- but unlike the real header,
// `i2c_dev_t` here is NOT a register-layout overlay. It is the host
// harness's I2C fake device MODEL: hal/i2c_ll.h's fake LL functions read and
// write these fields, and a test scripts a bus transaction by poking them
// directly before calling m5hal_hostharness::fireLastIsr() (see
// ../../README.md "How the fake device model works").
//
// Two fields are the exception -- `ctr.sda_force_out` / `ctr.scl_force_out`
// and `fifo_conf.fifo_prt_en` / `fifo_conf.fifo_addr_cfg_en` -- because the
// espidf slave backend pokes them directly as bitfields (`hw->ctr.sda_force_out
// = 1;` etc., bypassing the LL helpers for these few portable-but-unhelpered
// fields). They exist here purely so those pokes compile; the fake model
// does not read them back.

#include <hal/i2c_types.h>

#include <cstddef>
#include <cstdint>

struct i2c_dev_t {
    static constexpr size_t kFifoLen = 32;  // must match soc/soc_caps.h's SOC_I2C_FIFO_LEN

    // ---- direct register pokes the product code makes (field names/nesting
    // must match slave.inl's `hw->ctr.*` / `hw->fifo_conf.*` accesses) ----
    struct {
        unsigned sda_force_out : 1;
        unsigned scl_force_out : 1;
    } ctr{};
    struct {
        unsigned fifo_prt_en : 1;
        unsigned fifo_addr_cfg_en : 1;
    } fifo_conf{};

    // ---- fake device model state (hal/i2c_ll.h + tests only) ----

    // RX FIFO: bytes a test primes to represent what the master just clocked
    // in. i2c_ll_read_rxfifo() consumes from the front (index 0) and shifts
    // the remainder down, so a test that primes MORE bytes while some are
    // still unconsumed must APPEND (starting at rxfifo_count), not overwrite
    // index 0.
    uint8_t rxfifo[kFifoLen] = {};
    size_t rxfifo_count      = 0;

    // TX FIFO: bytes the product code has queued via i2c_ll_write_txfifo()
    // for the (simulated) master to read. This fake never drains it to model
    // the master clocking bytes out -- the wave-1 scenarios only need to
    // inspect the final queued content, not a multi-refill stream. A
    // scenario that needs FIFO backpressure mid-read would extend
    // i2c_ll_get_txfifo_len() to shrink free space as the test "drains" this
    // buffer.
    uint8_t txfifo[kFifoLen] = {};
    size_t txfifo_count      = 0;

    // Interrupt state: int_ena mirrors what i2c_ll_enable_intr_mask /
    // i2c_ll_disable_intr_mask have enabled; int_st is what a test primes as
    // "pending" before firing the ISR (i2c_ll_get_intr_mask ANDs the two,
    // matching the real LL semantics the product code relies on).
    uint32_t int_ena = 0;
    uint32_t int_st  = 0;

    // Address-phase direction latch (i2c_ll_slave_get_read_write_status).
    // A test sets this before firing an ADDRESS_MATCH-cause stretch or an
    // RX/TX water-mark event, matching what the real LL reports for the
    // in-flight bus transaction.
    int slave_rw                     = I2C_SLAVE_WRITE_BY_MASTER;
    bool bus_busy                    = false;
    uint16_t slave_address           = 0;
    bool slave_address_10bit         = false;
    uint16_t pending_slave_address   = 0;
    bool pending_slave_address_10bit = false;

    // Clock-stretch state (i2c_ll_slave_get_stretch_cause /
    // i2c_ll_slave_clear_stretch). `stretch_active` is this fake's stand-in
    // for "the real HW is physically holding SCL low" -- the product code
    // never reads it back (real HW has no such readback either), but tests
    // assert on it to verify a stretch was (or was not) released, which is
    // exactly the axis the cc133e89 stale-reply regression turns on.
    int stretch_cause   = I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH;
    bool stretch_active = false;

    // Diagnostic-only counters a test may inspect (not read by the fake LL
    // functions themselves). Not reset by i2c_ll_txfifo_rst /
    // i2c_ll_rxfifo_rst's fifo_count reset -- if a test wants deltas, it
    // must snapshot before/after rather than assume a zero baseline (this
    // model is a process-wide singleton per port; see hal/i2c_ll.h).
    unsigned txfifo_rst_count = 0;
    unsigned rxfifo_rst_count = 0;
    unsigned update_count     = 0;
};
