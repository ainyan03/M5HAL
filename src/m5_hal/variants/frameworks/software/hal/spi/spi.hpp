// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_SPI_SPI_HPP

#include "../../../../../hal/v2/gpio/port.hpp"
#include "../../../../../hal/v2/m5_hal.hpp"
#include "../../../../../hal/v2/service/completion_gate.hpp"
#include "../../../../../hal/v2/service/service.hpp"
#include "../../../../../hal/v2/spi/spi.hpp"

#include <atomic>

// SPI bit-bang implementation. The bus stores CLK/MOSI/MISO/DC pins resolved
// from IBusConfig via M5_Hal.Gpio. Per-device CS is resolved from
// MasterAccessConfig for each transaction.
namespace m5::hal::v2::spi {

// This variant needs no fields beyond the abstract kind config; the
// empty derivation still gives `init` a variant-owned type, so a
// sibling variant's config cannot be passed by accident.
struct BusConfig_software : public spi::IBusConfig {
    using IBusConfig::IBusConfig;
};

class Bus_software : public spi::IBus, private service::IService {
public:
    Bus_software();
    ~Bus_software() override;

    // Typed init: takes this variant's BusConfig_software, so a sibling
    // variant's config is a compile error instead of a bad downcast.
    result_t<void> init(const BusConfig_software& config);
    result_t<void> release(void) override;

    result_t<void> beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    result_t<void> endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    result_t<void> transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg, const spi::TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    bool transferBusy(bus::IAccessor* owner) override;

private:
    service::ServicePoll serviceImpl(const service::ServiceContext& ctx) override;
    service::ServicePoll serviceTransfer(const service::ServiceContext& ctx);
    void unregisterTransferService(void);
    void clearTransferService(void);

    gpio::Pin _pin_clk{};
    gpio::Pin _pin_dc{};
    // Accessor-level D/C override pin, resolved lazily on first use and
    // re-resolved only when the configured number changes.
    gpio::Pin _pin_dc_acc{};
    types::gpio_number_t _acc_dc_num = -1;
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

// Facade backend selection: spi::Bus::init(BusConfig_software) -> Bus_software.
template <>
struct BackendFor<BusConfig_software> {
    using type = Bus_software;
};

// software backend factory: builds a bit-bang Bus_software from a
// LogicalBusConfig's pins. M5HALCore wires this into spi::BusView (the logical
// acquire path). The software variant is always present, so this is the
// universal software fallback used when a bus is not (or not yet) on hardware.
inline IBus* makeSoftwareBackendForSPI(const LogicalBusConfig& logical)
{
    auto* backend = new (std::nothrow) Bus_software();
    if (backend == nullptr) {
        return nullptr;
    }
    BusConfig_software cfg;
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

}  // namespace m5::hal::v2::spi

#endif
