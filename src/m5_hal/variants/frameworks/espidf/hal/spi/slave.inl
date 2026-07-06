// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_INL

#include "slave.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_SLAVE

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "../../../freertos/hal/runtime/time.hpp"

namespace m5::hal::v2::spi {

namespace {
namespace impl_espidf_slave {

error::error_t mapEspErr(::esp_err_t err)
{
    switch (err) {
        case ESP_OK:
            return error::error_t::OK;
        case ESP_ERR_INVALID_ARG:
        case ESP_ERR_INVALID_STATE:
            return error::error_t::INVALID_ARGUMENT;
        case ESP_ERR_TIMEOUT:
            return error::error_t::TIMEOUT_ERROR;
        case ESP_ERR_NO_MEM:
            return error::error_t::OUT_OF_RESOURCE;
        default:
            return error::error_t::IO_ERROR;
    }
}

::TickType_t ticks(uint32_t timeout_ms)
{
    return ::m5::hal::v2::detail::timeoutMsToTicks(timeout_ms);
}

}  // namespace impl_espidf_slave
}  // namespace

result_t<void> SpiSlaveBus_espidf::init(const spi::SlaveBusConfig& cfg)
{
    // All four wires are required for a full-duplex slave (the master clocks CS +
    // SCLK, drives MOSI, samples MISO); reject a partial wiring instead of letting
    // the driver fail opaquely.
    if (cfg.pin_clk < 0 || cfg.pin_mosi < 0 || cfg.pin_miso < 0 || cfg.pin_cs < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (cfg.spi_mode > 3) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    if (_initialized) {
        (void)release();
    }
    _config = cfg;
    _host   = (cfg.host >= 0) ? static_cast<::spi_host_device_t>(cfg.host) : SPI2_HOST;

    // DMA-capable, word-aligned bounce buffers. spi_slave_transmit requires DMA-able
    // buffers when a DMA channel is selected; heap_caps_malloc(MALLOC_CAP_DMA)
    // returns 4-byte-aligned DMA memory, and the capacity is a multiple of 4 so the
    // RX side meets the "length multiple of 4 bytes" DMA requirement.
    _tx_bounce = static_cast<uint8_t*>(::heap_caps_malloc(kBounceCapacity, MALLOC_CAP_DMA));
    _rx_bounce = static_cast<uint8_t*>(::heap_caps_malloc(kBounceCapacity, MALLOC_CAP_DMA));
    if (_tx_bounce == nullptr || _rx_bounce == nullptr) {
        (void)release();
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    ::spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num        = cfg.pin_mosi;
    buscfg.miso_io_num        = cfg.pin_miso;
    buscfg.sclk_io_num        = cfg.pin_clk;
    buscfg.quadwp_io_num      = -1;
    buscfg.quadhd_io_num      = -1;
    buscfg.max_transfer_sz    = kBounceCapacity;

    ::spi_slave_interface_config_t slvcfg = {};
    slvcfg.spics_io_num                   = cfg.pin_cs;
    slvcfg.queue_size                     = 3;
    slvcfg.mode                           = cfg.spi_mode;
    slvcfg.flags                          = cfg.spi_order ? SPI_SLAVE_BIT_LSBFIRST : 0;

    auto mapped = impl_espidf_slave::mapEspErr(::spi_slave_initialize(_host, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    if (error::isError(mapped)) {
        (void)release();
        return m5::stl::make_unexpected(mapped);
    }

    _initialized = true;
    return {};
}

result_t<void> SpiSlaveBus_espidf::release(void)
{
    if (_initialized) {
        auto mapped = impl_espidf_slave::mapEspErr(::spi_slave_free(_host));
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        _initialized = false;
    }
    if (_tx_bounce != nullptr) {
        ::heap_caps_free(_tx_bounce);
        _tx_bounce = nullptr;
    }
    if (_rx_bounce != nullptr) {
        ::heap_caps_free(_rx_bounce);
        _rx_bounce = nullptr;
    }
    return {};
}

result_t<size_t> SpiSlaveBus_espidf::serve(bus::IAccessor* owner, data::Source* tx, data::Sink* rx, size_t len,
                                           uint32_t timeout_ms)
{
    // owner is the single resident serve()-loop accessor; multi-accessor
    // multiplexing onto one slave bus is out of scope (one CS = one exchange).
    (void)owner;
    if (!_initialized) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    // Clamp the exchange to the bounce-buffer capacity: SPI is full-duplex, so one
    // len bounds BOTH directions (the shared clock-cycle count).
    if (len > kBounceCapacity) {
        len = kBounceCapacity;
    }
    if (len == 0) {
        return static_cast<size_t>(0);
    }

    // Pull up to `len` bytes of MISO data from the tx Source into the TX bounce.
    // Any bytes the Source cannot supply (short Source, or tx == null) are filled
    // with cfg.tx_fill_byte so the master always clocks a defined MISO level.
    size_t filled = 0;
    while (tx != nullptr && !tx->eof() && filled < len) {
        auto span = tx->peek(len - filled);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        ::memcpy(_tx_bounce + filled, span.value().data, span.value().size);
        auto advanced = tx->advance(span.value().size);
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        filled += span.value().size;
    }
    if (filled < len) {
        ::memset(_tx_bounce + filled, _config.tx_fill_byte, len - filled);
    }
    // Clear the RX bounce so a short transaction leaves no stale bytes past
    // trans_len (only the actually-clocked prefix is committed below).
    ::memset(_rx_bounce, 0, len);

    ::spi_slave_transaction_t t = {};
    t.length                    = len * 8;  // bits
    t.tx_buffer                 = _tx_bounce;
    t.rx_buffer                 = _rx_bounce;

    // NOTE (HW spike observation): the very first transaction after init can come
    // back shifted by one bit on some boards/wiring. No correction mechanism is
    // built in here -- the application/bench decides on real hardware whether a
    // priming transaction or a wiring fix is needed. serve() reports trans_len as
    // delivered by the driver.
    esp_err_t e = ::spi_slave_transmit(_host, &t, impl_espidf_slave::ticks(timeout_ms));
    if (e != ESP_OK) {
        // A timeout means the master never clocked a transaction in the window:
        // report zero exchanged rather than an error (matches the serve() contract
        // of returning empty on no transaction).
        if (e == ESP_ERR_TIMEOUT) {
            return static_cast<size_t>(0);
        }
        return m5::stl::make_unexpected(impl_espidf_slave::mapEspErr(e));
    }

    // Bytes actually clocked (the master may clock fewer than len, ending early on
    // CS deassert). trans_len is in bits; round down to whole bytes.
    size_t got = static_cast<size_t>(t.trans_len) / 8;
    if (got > len) {
        got = len;
    }

    // Commit the captured MOSI bytes to the rx Sink (a null Sink discards them).
    size_t committed = 0;
    while (rx != nullptr && !rx->closed() && committed < got) {
        auto reserved = rx->reserve(got - committed);
        if (!reserved.has_value()) {
            return m5::stl::make_unexpected(reserved.error());
        }
        size_t want = reserved.value().size;
        if (want == 0) {
            break;  // sink is full
        }
        if (want > got - committed) {
            want = got - committed;
        }
        ::memcpy(reserved.value().data, _rx_bounce + committed, want);
        auto done = rx->commit(want);
        if (!done.has_value()) {
            return m5::stl::make_unexpected(done.error());
        }
        committed += want;
    }

    return got;
}

}  // namespace m5::hal::v2::spi

#endif  // defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_SLAVE

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SLAVE_INL
