// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v2/spi/slave.hpp"
#include "spi.hpp"

// SPI slave is full-duplex on every ESP32-family SoC (driver/spi_slave.h), so
// unlike the I2C slave there is no per-SoC backend split: one backend builds
// wherever the SPI slave driver header is present. The half-duplex register-map
// model (spi_slave_hd, HW v2 only) is intentionally out of scope (see the
// interface contract in hal/v2/spi/slave.hpp).
#if defined(ESP_PLATFORM) && __has_include(<driver/spi_slave.h>)
#define M5HAL_ESPIDF_SPI_HAS_SLAVE 1
#else
#define M5HAL_ESPIDF_SPI_HAS_SLAVE 0
#endif

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_SLAVE

#include <driver/spi_slave.h>

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v2::spi {

// ESP-IDF full-duplex SPI slave bus. serve() runs ONE master-clocked
// transaction: the slave must have its TX/RX buffers queued before the master
// starts the clock, so each serve() call queues a fresh transaction and blocks
// in spi_slave_transmit() until the master completes it (or the timeout elapses
// with no transaction). The whole interaction is a single primitive -- there is
// no mid-transaction stall, no addressing, and no clock stretching (the master
// owns SCLK), so this backend stays far simpler than the I2C slave (no ring, no
// two-stage handoff, no stretch state machine).
class SpiSlaveBus_espidf : public spi::ISlaveBus {
public:
    // DMA-capable bounce-buffer capacity (per direction). spi_slave_transmit()
    // requires DMA-able, word-aligned buffers; we own one TX and one RX bounce
    // of this fixed size and clamp serve()'s len to it.
    //
    // 32767, not 32768: the transfer length lives in an 18-bit register field
    // that stores (bit_num - 1) on this chip family (SOC_SPI_MAXIMUM_BUFFER_SIZE
    // covers only the non-DMA FIFO path, so it does not document this). A
    // 32768-byte (262144-bit) transfer needs to store 262143, the field's exact
    // max, and measured on hardware this boundary case corrupts the transfer
    // (received byte count collapses to 0) while 32767 bytes transfers cleanly.
    // Measured on ESP32-S3 (slave role) only; not reverified on classic ESP32,
    // whose 24-bit dlen registers may tolerate a larger bound.
    static constexpr size_t kBounceCapacity = 32767;

    SpiSlaveBus_espidf() = default;
    ~SpiSlaveBus_espidf() override
    {
        (void)release();
    }

    result_t<void> init(const spi::SlaveBusConfig& cfg) override;
    result_t<void> release(void) override;
    result_t<size_t> serve(bus::IAccessor* owner, data::Source* tx, data::Sink* rx, size_t len,
                           uint32_t timeout_ms) override;

private:
    ::spi_host_device_t _host = SPI2_HOST;
    bool _initialized         = false;
    // DMA-capable, word-aligned bounce buffers (heap_caps_malloc MALLOC_CAP_DMA).
    // Owned for the bus lifetime so each serve() reuses them; freed in release().
    uint8_t* _tx_bounce = nullptr;
    uint8_t* _rx_bounce = nullptr;
    // The queued transaction descriptor and whether one is outstanding. The
    // descriptor must outlive the serve() call that queues it: the ESP-IDF slave
    // driver keeps the queued &_trans in its internal queue until the master
    // clocks it, even if get_trans_result() times out first. A stack-local
    // descriptor would dangle in that window and let the ISR write into a dead
    // stack frame, so the descriptor lives for the bus lifetime here and queue /
    // collection are split across serve() calls (see serve()).
    ::spi_slave_transaction_t _trans = {};
    bool _trans_pending              = false;
};

// No BackendFor<SlaveBusConfig> specialization (mirrors the I2C slave backend):
// the SPI slave is not part of the facade's hardware-controller pool. It is
// constructed directly by the application, which drives a resident serve() loop
// around it, so there is no init(SlaveBusConfig) facade dispatch to select.

}  // namespace m5::hal::v2::spi

#endif  // defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_SLAVE

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_HPP
