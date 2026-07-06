// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/spi/spi.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_MASTER

#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace m5::hal::v2::spi {

struct BusConfig_espidf : public spi::IBusConfig {
    // Inherit the tag-pin constructors (Clk / Mosi / Miso); the host is set
    // by field assignment afterwards.
    using spi::IBusConfig::IBusConfig;

    ::spi_host_device_t host = SPI2_HOST;

    constexpr BusConfig_espidf(void) : spi::IBusConfig{}
    {
    }
};

// ESP-IDF SPI master bus. CS and D/C are managed by M5HAL so the shared
// MasterAccessor transaction semantics match the Arduino and software
// variants. Driver-generation differences stay behind detail/espidf_version.hpp
// and backend includes.
class Bus_espidf : public spi::IBus {
public:
    ~Bus_espidf() override
    {
        (void)release();
    }

    // Typed init: takes this variant's BusConfig_espidf. Passing the
    // abstract IBusConfig (or a sibling variant's config) is a
    // compile error instead of a silent bad downcast.
    result_t<void> init(const BusConfig_espidf& config);
    result_t<void> release(void) override;

    result_t<void> beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    result_t<void> endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    result_t<void> transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg, const spi::TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    bool transferBusy(bus::IAccessor* owner) override;

    // This backend drives a dedicated ESP-IDF SPI host. The phase-3 controller
    // pool assigns the host through BusConfig_espidf::host; the query API
    // reports it (ADR 034) as a zero-based controller index so the resolver's
    // incumbency check and a holder watching for a downgrade both see the live
    // state. The index axis is the generic-host axis (SPI2_HOST -> 0,
    // SPI3_HOST -> 1), excluding SPI1 (flash), to match the budget reported by
    // hardwareControllerCountForSPI().
    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }
    int8_t controllerId(void) const override
    {
        return static_cast<int8_t>(_host - SPI2_HOST);
    }
    uint32_t maxFrequency(void) const override
    {
        // ESP32-family SPI master peripheral ceiling (80 MHz). Declares the
        // capability so a holder can notice a drop after a downgrade to
        // software (ADR 034), without depending on a SoC-specific macro.
        return 80000000u;
    }

    error::error_t attach(::spi_host_device_t host);
    ::spi_host_device_t nativeHost() const
    {
        return _host;
    }
    ::spi_device_handle_t nativeDevice() const
    {
        return _device;
    }

private:
    static void workerEntry(void* arg);
    void workerLoop(void);
    void waitInFlight(void);
    // true = the device was (re)created (SCK idle level may need settling).
    result_t<bool> ensureDevice(const spi::MasterAccessConfig& cfg);
    result_t<void> removeDevice(void);

    ::spi_host_device_t _host     = SPI2_HOST;
    ::spi_device_handle_t _device = nullptr;
    // Last accessor-level D/C pin switched to output (-1 = none yet);
    // avoids a gpio reconfig per transfer on the override path.
    types::gpio_number_t _last_acc_dc      = -1;
    bool _owns_bus                         = false;
    bool _transaction_active               = false;
    uint32_t _device_freq                  = 0;
    uint8_t _device_mode                   = 0;
    uint8_t _device_order                  = 0;
    spi::spi_data_mode_t _device_data_mode = spi::spi_data_mode_t::FullDuplex;

    // Double-buffer DMA transfer. One buffer is owned by the SPI peripheral
    // while the worker prepares or drains the other.
    static constexpr size_t kMaxDmaChunk = 32768;
    uint8_t* _dma_buf[2]                 = {nullptr, nullptr};
    ::spi_transaction_t _dma_trans[2];
    TaskHandle_t _worker_task              = nullptr;
    volatile bool _in_flight               = false;
    volatile bool _worker_active           = false;
    volatile error::error_t _worker_status = error::error_t::OK;
    data::Source* _worker_src              = nullptr;
    data::Sink* _worker_dst                = nullptr;
    size_t _worker_remaining               = 0;
    bus::TransferTotals _transfer_totals{};
    bus::IAccessor* _transfer_owner = nullptr;
    size_t _worker_tx_remaining     = 0;
    size_t _worker_rx_remaining     = 0;
    size_t _worker_front_tx_len     = 0;
    size_t _worker_front_rx_len     = 0;
};

// Facade backend selection: spi::Bus::init(BusConfig_espidf) -> Bus_espidf.
template <>
struct BackendFor<BusConfig_espidf> {
    using type = Bus_espidf;
};

// Phase-3 hardware backend factory (ADR 034). Builds a Bus_espidf for a logical
// request, binding the leased controller index to an ESP-IDF SPI host. The pool
// hands out a zero-based index; this is the only code that knows it maps onto
// the generic hosts (SPI2_HOST + index), so the kind-generic BusView / pool stay
// variant-agnostic. SPI1_HOST (flash) is deliberately not in the pool, so index
// 0 is SPI2_HOST. M5HALCore wires this into spi::BusView when this variant
// provides hardware SPI (M5HAL_SPI_HAS_HW_BACKEND below).
inline spi::IBus* makeHardwareBackendForSPI(const spi::LogicalBusConfig& logical, int8_t controller)
{
    auto* backend = new (std::nothrow) Bus_espidf();
    if (backend == nullptr) {
        return nullptr;
    }
    BusConfig_espidf cfg;
    cfg.pin_clk  = logical.pin_clk;
    cfg.pin_mosi = logical.pin_mosi;
    cfg.pin_miso = logical.pin_miso;
    cfg.host     = static_cast<::spi_host_device_t>(SPI2_HOST + controller);
    auto r       = backend->init(cfg);
    if (!r.has_value()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

// Silicon budget for general-purpose SPI on this SoC. SOC_SPI_PERIPH_NUM counts
// every SPI peripheral including SPI1 (the flash/PSRAM controller), which is not
// a general-purpose host, so the pool budget excludes it (classic ESP32:
// SOC_SPI_PERIPH_NUM = 3 -> SPI2 + SPI3 = 2). When the headers do not expose the
// macro a conservative 2 is used (every ESP32-family chip has at least SPI2 +
// SPI3, except single-host parts where the subtraction still clamps to >= 1).
inline uint8_t hardwareControllerCountForSPI(void)
{
#if defined(SOC_SPI_PERIPH_NUM)
    return SOC_SPI_PERIPH_NUM > 1 ? static_cast<uint8_t>(SOC_SPI_PERIPH_NUM - 1) : 1;
#else
    return 2;
#endif
}

// Tells M5HALCore that this build has a poolable hardware SPI backend, so the
// SPI BusView is wired with the hardware factory + controller pool.
#define M5HAL_SPI_HAS_HW_BACKEND 1

}  // namespace m5::hal::v2::spi

#endif

#endif
