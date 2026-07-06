// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_SPI_SPI_HPP

#include "../../../../../hal/v2/spi/spi.hpp"
#include "../../remote_transfer.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::spi {

using remote::RemoteSession;

struct BusConfig_remote : public spi::IBusConfig {
    using spi::IBusConfig::IBusConfig;

    constexpr BusConfig_remote(void) : spi::IBusConfig{}
    {
    }
};

class Bus_remote : public spi::IBus {
public:
    Bus_remote() = default;

    Bus_remote(RemoteSession& session, uint8_t bus_id) : Bus_remote{session, bus_id, spi::IBusConfig{}}
    {
    }

    Bus_remote(RemoteSession& session, uint8_t bus_id, const spi::IBusConfig& cfg) : _session{&session}, _bus_id{bus_id}
    {
        _config = cfg;
    }

    result_t<void> init(const BusConfig_remote& config);

    uint8_t busId(void) const
    {
        return _bus_id;
    }

    result_t<void> beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    result_t<void> endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    result_t<void> transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg, const spi::TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override;
    bool transferBusy(bus::IAccessor* owner) override;

private:
    static constexpr uint32_t kTransferTimeoutMs = 5000;

    static data::ConstDataSpan encodeSpiMeta(uint8_t* buf, const spi::TransferDesc& desc);

    RemoteSession* _session = nullptr;
    uint8_t _bus_id         = 0;
    bus::TransferTotals _last_totals;
    bool _in_transaction = false;
    remote::RemoteConfigCache _config_cache;
};

template <>
struct BackendFor<BusConfig_remote> {
    using type = Bus_remote;
};

}  // namespace m5::hal::v2::spi

#endif
