// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_HPP

#include "../../../../../hal/v2/gpio/port.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../../../../hal/v2/m5_hal.hpp"
#include "../../../../../hal/v2/service/completion_gate.hpp"
#include "../../../../../hal/v2/service/service.hpp"
#include "../../../../../hal/v2/spi/spi.hpp"

#include <atomic>

// SPI bit-bang implementation. The bus stores CLK/MOSI/MISO/DC pins resolved
// from IBusConfig via the injected LocalResourceContext GPIOGroup. Per-device CS is resolved from
// MasterAccessConfig for each transaction.
namespace m5::hal::v2::spi {

class Bus_software : public spi::IBus, private service::IService {
public:
    Bus_software();
    ~Bus_software() override;

    result_t<void> close(void)
    {
        return spi::IBus::close();
    }

    result_t<void> init(const IBusConfig& config);

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

private:
    result_t<void> teardown(void);

protected:
    bus::CloseOutcome closeBackend(void) override;
    result_t<void> beginOperationBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    result_t<void> transferBackend(bus::OperationContext<spi::MasterAccessConfig>& context,
                                   const spi::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                   size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransferBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    bool transferBusyBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;

private:
    service::ServicePoll serviceImpl(const service::ServiceContext& ctx) override;
    service::ServicePoll serviceTransfer(const service::ServiceContext& ctx);
    result_t<void> unregisterTransferService(void);
    result_t<void> clearTransferService(void);

    gpio::Pin _pin_clk{};
    gpio::Pin _pin_dc{};
    // Accessor-level D/C override pin, resolved lazily on first use and
    // re-resolved only when the configured number changes.
    gpio::Pin _pin_dc_acc{};
    types::gpio_number_t _acc_dc_num = -1;
    // Current expected level of the bus-level D/C pin. Per-accessor override
    // pins do not affect it.
    bool _dc_level_high = false;
    gpio::Pin _pin_mosi{};
    gpio::Pin _pin_miso{};
    gpio::Pin _transaction_cs{};
    void* _transfer_service         = nullptr;
    bus::IAccessor* _transfer_owner = nullptr;
    std::atomic<bool> _transfer_registered{false};
    service::CompletionGate _transfer_gate;
    error::error_t _transfer_error = error::error_t::OK;
    bus::TransferTotals _transfer_totals{};
};

// Portable provider factory for the software backend.
inline result_t<std::unique_ptr<IBus>> makePortableBackend_software(const bus::LocalResourceContext& resources,
                                                                    const IBusConfig& config)
{
    return bus::makePortableBackend<IBus, Bus_software, IBusConfig>(resources, config);
}

// software backend factory: builds a bit-bang Bus_software from a
// LogicalBusConfig's pins. M5HALCore wires this into spi::BusView (the logical
// acquire path). The software variant is always present, so this is the
// universal software fallback used when a bus is not (or not yet) on hardware.
inline IBus* makeSoftwareBackendForSPI(const bus::LocalResourceContext& resources, const LogicalBusConfig& logical)
{
    auto* backend = new (std::nothrow) Bus_software();
    if (backend == nullptr) {
        return nullptr;
    }
    backend->bindLocalResources(resources);
    IBusConfig cfg;
    cfg.pin_clk  = logical.pin_clk;
    cfg.pin_mosi = logical.pin_mosi;
    cfg.pin_miso = logical.pin_miso;
    auto r       = backend->init(cfg);
    if (!r.has_value()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

template <class Policy>
struct NativeProvider_software {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, Policy)
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
};

}  // namespace m5::hal::v2::spi

#endif
