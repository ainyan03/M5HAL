// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../../../../hal/v2/spi/spi.hpp"

namespace m5::hal::v2::spi::detail_espidf_spi {

// Primitive-only mapping seam kept outside the ESP_PLATFORM guard so native
// tests can fix the controller/host boundary without faking the ESP-IDF driver.
constexpr bool hostOrdinalForController(int8_t controller, uint8_t controller_count, int first_host, int& host)
{
    if (controller < 0 || static_cast<uint8_t>(controller) >= controller_count) {
        return false;
    }
    host = first_host + controller;
    return true;
}

constexpr bool controllerForHostOrdinal(int host, uint8_t controller_count, int first_host, int8_t& controller)
{
    const int index = host - first_host;
    if (index < 0 || index >= static_cast<int>(controller_count)) {
        return false;
    }
    controller = static_cast<int8_t>(index);
    return true;
}

constexpr bool slaveHostOrdinal(int8_t controller, uint8_t controller_count, int first_host, int& host)
{
    if (controller == -1) {
        if (controller_count == 0) {
            return false;
        }
        host = first_host;
        return true;
    }
    return hostOrdinalForController(controller, controller_count, first_host, host);
}

// Driver resources must be detached before the worker that services them is
// stopped. Keeping this small ordering seam outside the ESP_PLATFORM guard
// lets native tests cover failure paths without faking the ESP-IDF driver.
struct ReleaseDriverResult {
    error::error_t error;
    bool bus_released;
    bool worker_stopped;
};

template <typename RemoveDevice, typename FreeBus, typename StopWorker>
ReleaseDriverResult releaseDriverBeforeWorker(bool owns_bus, bool bus_free_releases_on_error,
                                              RemoveDevice remove_device, FreeBus free_bus, StopWorker stop_worker)
{
    auto err = remove_device();
    if (error::isError(err)) {
        return {err, false, false};
    }
    if (owns_bus) {
        err = free_bus();
        if (error::isError(err)) {
            if (!bus_free_releases_on_error) {
                return {err, false, false};
            }
            // Arduino-core 2.x's embedded ESP-IDF 4.4 frees the bus even when
            // a registered destroy callback fails. The worker must not survive
            // that destructive error, and release must report completion so the
            // facade does not retain an already-freed backend.
            stop_worker();
            return {error::error_t::OK, true, true};
        }
    }
    stop_worker();
    return {error::error_t::OK, owns_bus, true};
}

}  // namespace m5::hal::v2::spi::detail_espidf_spi

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_MASTER

#include <atomic>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace m5::hal::v2::spi {

// Silicon budget for general-purpose SPI on this SoC. SOC_SPI_PERIPH_NUM counts
// every SPI peripheral including SPI1 (the flash/PSRAM controller), which is not
// a general-purpose host, so the pool budget excludes it (classic ESP32:
// SOC_SPI_PERIPH_NUM = 3 -> SPI2 + SPI3 = 2). When the headers do not expose the
// macro a conservative 2 is used.
inline uint8_t hardwareControllerCountForSPI(void)
{
#if defined(SOC_SPI_PERIPH_NUM)
    return SOC_SPI_PERIPH_NUM > 1 ? static_cast<uint8_t>(SOC_SPI_PERIPH_NUM - 1) : 1;
#else
    return 2;
#endif
}

namespace detail_espidf_spi {

inline bool hostForController(int8_t controller, ::spi_host_device_t& host)
{
    int ordinal = 0;
    if (!hostOrdinalForController(controller, hardwareControllerCountForSPI(), static_cast<int>(SPI2_HOST), ordinal)) {
        return false;
    }
    host = static_cast<::spi_host_device_t>(ordinal);
    return true;
}

inline bool controllerForHost(::spi_host_device_t host, int8_t& controller)
{
    return controllerForHostOrdinal(static_cast<int>(host), hardwareControllerCountForSPI(),
                                    static_cast<int>(SPI2_HOST), controller);
}

inline bool hostForSlaveController(int8_t controller, ::spi_host_device_t& host)
{
    int ordinal = 0;
    if (!slaveHostOrdinal(controller, hardwareControllerCountForSPI(), static_cast<int>(SPI2_HOST), ordinal)) {
        return false;
    }
    host = static_cast<::spi_host_device_t>(ordinal);
    return true;
}

}  // namespace detail_espidf_spi

// ESP-IDF SPI master bus. CS and D/C are managed by M5HAL so the shared
// MasterAccessor transaction semantics match the Arduino and software
// variants. Driver-generation differences stay behind detail/espidf_version.hpp
// and backend includes.
class Bus_espidf : public spi::IBus {
public:
    ~Bus_espidf() override;

    result_t<void> init(const IBusConfig& config);
    result_t<void> close(void)
    {
        return bus::IBus::close();
    }

    // This backend drives a dedicated ESP-IDF SPI host. The controller
    // pool assigns the host through the provider-private init helper; the
    // query API reports it as a zero-based controller index so the resolver's
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
        int8_t controller = -1;
        return detail_espidf_spi::controllerForHost(_host, controller) ? controller : -1;
    }
    uint32_t maxFrequency(void) const override
    {
        // ESP32-family SPI master peripheral ceiling (80 MHz). Declares the
        // capability so a holder can notice a drop after a downgrade to
        // software, without depending on a SoC-specific macro.
        return 80000000u;
    }
    bus::BusCapabilities capabilities(void) const override
    {
        const bool has_mosi = _config.pin_mosi >= 0;
        const bool has_miso = _config.pin_miso >= 0;
        return bus::detail::BusCapabilitiesBuilder{bus::IBus::capabilities()}
            .enable(bus::BusFeature::MasterTransfer)
            .enable(bus::BusFeature::Transmit, has_mosi)
            .enable(bus::BusFeature::Receive, has_miso || has_mosi)
            .enable(bus::BusFeature::FullDuplex, has_mosi && has_miso)
            .enable(bus::BusFeature::MosiSharedRx, has_mosi && !has_miso)
            .build();
    }

    ::spi_host_device_t nativeHost() const
    {
        return _host;
    }
    ::spi_device_handle_t nativeDevice() const
    {
        return _device;
    }

protected:
    bus::CloseOutcome closeBackend(void) override
    {
        return teardownBackend();
    }
    result_t<void> beginOperationBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    result_t<void> transferBackend(bus::OperationContext<spi::MasterAccessConfig>& context,
                                   const spi::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                   size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransferBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    bool transferBusyBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;

private:
    bus::CloseOutcome teardownBackend(void);
    result_t<void> initBackend(const IBusConfig& config, ::spi_host_device_t host);
    friend spi::IBus* makeHardwareBackendForSPI(const bus::LocalResourceContext&, const spi::LogicalBusConfig&, int8_t);
    result_t<void> resetForInitialization(void);
    static void workerEntry(void* arg);
    void workerLoop(void);
    void waitInFlight(void);
    void stopWorker(void);
    void freeDmaBuffers(void);
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
    TaskHandle_t _worker_task = nullptr;
    std::atomic<bool> _in_flight{false};
    std::atomic<bool> _worker_active{false};
    std::atomic<error::error_t> _worker_status{error::error_t::OK};
    data::Source* _worker_src = nullptr;
    data::Sink* _worker_dst   = nullptr;
    size_t _worker_remaining  = 0;
    bus::TransferTotals _transfer_totals{};
    bus::IAccessor* _transfer_owner = nullptr;
    size_t _worker_tx_remaining     = 0;
    size_t _worker_rx_remaining     = 0;
    size_t _worker_front_tx_len     = 0;
    size_t _worker_front_rx_len     = 0;
    bool _worker_half_duplex        = false;
};

inline result_t<std::unique_ptr<IBus>> makePortableBackend_espidf(const bus::LocalResourceContext& resources,
                                                                  const IBusConfig& config)
{
    return bus::makePortableBackend<IBus, Bus_espidf, IBusConfig>(resources, config);
}

// hardware backend factory. Builds a Bus_espidf for a logical
// request, binding the leased controller index to an ESP-IDF SPI host. The pool
// hands out a zero-based index; this is the only code that knows it maps onto
// the generic hosts (SPI2_HOST + index), so the kind-generic BusView / pool stay
// variant-agnostic. SPI1_HOST (flash) is deliberately not in the pool, so index
// 0 is SPI2_HOST. M5HALCore wires this into spi::BusView when this variant
// provides hardware SPI (M5HAL_DETAIL_SPI_HAS_HARDWARE_BACKEND_ below).
inline spi::IBus* makeHardwareBackendForSPI(const bus::LocalResourceContext& resources,
                                            const spi::LogicalBusConfig& logical, int8_t controller)
{
    ::spi_host_device_t host;
    if (!detail_espidf_spi::hostForController(controller, host)) {
        return nullptr;
    }
    auto* backend = new (std::nothrow) Bus_espidf();
    if (backend == nullptr) {
        return nullptr;
    }
    backend->bindLocalResources(resources);
    IBusConfig cfg;
    cfg.pin_clk  = logical.pin_clk;
    cfg.pin_mosi = logical.pin_mosi;
    cfg.pin_miso = logical.pin_miso;
    auto r       = backend->initBackend(cfg, host);
    if (!r.has_value()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

// Tells M5HALCore that this build has a poolable hardware SPI backend, so the
// SPI BusView is wired with the hardware factory + controller pool.
#define M5HAL_DETAIL_SPI_HAS_HARDWARE_BACKEND_ 1

template <class Policy>
struct NativeProvider_espidf {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, Policy)
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
};

}  // namespace m5::hal::v2::spi

#endif

#endif
