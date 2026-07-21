// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_PDM_PDM_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_PDM_PDM_HPP

#include "../../../../../hal/v2/pdm/pdm.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../bus_lease.hpp"
#include "../../remote_transfer.hpp"

namespace m5::hal::v2::remote {
class RemoteBackend;
}

namespace m5::hal::v2::pdm {

class Bus_remote : public pdm::IBus {
public:
    Bus_remote() = default;
    Bus_remote(std::shared_ptr<remote::RemoteSessionHandle> session, uint8_t bus_id, const pdm::IBusConfig& cfg,
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

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<AccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<AccessConfig>& context) override;

public:
    uint8_t busId() const
    {
        return _bus_id;
    }
    std::shared_ptr<remote::RemoteBusLease> remoteBusLease() const
    {
        return _bus_lease;
    }
    std::shared_ptr<bus::BusLifecycle> lifecycleHandle() const override
    {
        return _lifecycle;
    }

protected:
    result_t<size_t> readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len) override;
    result_t<size_t> readableBytesBackend(bus::OperationContext<AccessConfig>& context) override;

private:
    friend class remote::RemoteBackend;

    void setRemoteCapabilities(const bus::BusCapabilities& value)
    {
        _capabilities = value;
    }
    std::shared_ptr<remote::RemoteSessionHandle> _session;
    uint8_t _bus_id                               = 0;
    std::shared_ptr<bus::BusLifecycle> _lifecycle = std::make_shared<bus::BusLifecycle>();
    std::shared_ptr<remote::RemoteBusLease> _bus_lease;
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

}  // namespace m5::hal::v2::pdm

#endif
