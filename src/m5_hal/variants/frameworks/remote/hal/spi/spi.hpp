// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_SPI_SPI_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_SPI_SPI_HPP

#include "../../../../../hal/v2/spi/spi.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../bus_lease.hpp"
#include "../../remote_transfer.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::remote {
class RemoteBackend;
}

namespace m5::hal::v2::spi {

using remote::RemoteSession;

class Bus_remote : public spi::IBus {
public:
    Bus_remote() = default;

    Bus_remote(RemoteSession& session, uint8_t bus_id) : Bus_remote{session, bus_id, spi::IBusConfig{}}
    {
    }

    Bus_remote(RemoteSession& session, uint8_t bus_id, const spi::IBusConfig& cfg)
        : Bus_remote{remote::makeBorrowedSessionHandle(session), bus_id, cfg}
    {
    }

    Bus_remote(std::shared_ptr<remote::RemoteSessionHandle> session, uint8_t bus_id, const spi::IBusConfig& cfg,
               std::shared_ptr<remote::RemoteBusLease> bus_lease = {})
        : _session{std::move(session)},
          _bus_id{bus_id},
          _lifecycle{bus_lease ? bus_lease->lifecycle() : std::make_shared<bus::BusLifecycle>()},
          _bus_lease{std::move(bus_lease)}
    {
        _config = cfg;
    }

    result_t<void> init(const IBusConfig& config);
    bus::BusCapabilities capabilities(void) const override
    {
        return _capabilities;
    }

    uint8_t busId(void) const
    {
        return _bus_id;
    }

    std::shared_ptr<remote::RemoteBusLease> remoteBusLease() const
    {
        return _bus_lease;
    }
    std::shared_ptr<bus::BusLifecycle> lifecycleHandle(void) const override
    {
        return _lifecycle;
    }

private:
    friend class remote::RemoteBackend;

    void setRemoteCapabilities(const bus::BusCapabilities& value)
    {
        _capabilities = value;
    }

    static data::ConstDataSpan encodeSpiMeta(uint8_t* buf, const spi::TransferDesc& desc);

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    result_t<void> transferBackend(bus::OperationContext<spi::MasterAccessConfig>& context,
                                   const spi::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                   size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransferBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;
    bool transferBusyBackend(bus::OperationContext<spi::MasterAccessConfig>& context) override;

private:
    std::shared_ptr<remote::RemoteSessionHandle> _session;
    uint8_t _bus_id                               = 0;
    std::shared_ptr<bus::BusLifecycle> _lifecycle = std::make_shared<bus::BusLifecycle>();
    std::shared_ptr<remote::RemoteBusLease> _bus_lease;
    bus::TransferTotals _last_totals;
    bool _in_transaction = false;
    remote::RemoteConfigCache _config_cache;
    bus::BusCapabilities _capabilities;
};

inline result_t<std::unique_ptr<IBus>> makePortableBackend_remote(const bus::LocalResourceContext&, const IBusConfig&)
{
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

template <class Policy>
struct NativeProvider_remote {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, Policy)
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
};

}  // namespace m5::hal::v2::spi

#endif
