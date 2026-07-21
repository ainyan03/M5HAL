// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_HPP

#include "../../../../../hal/v2/uart/uart.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../bus_lease.hpp"
#include "../../remote_transfer.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::remote {
class RemoteBackend;
}

namespace m5::hal::v2::uart {

using remote::RemoteSession;

class Bus_remote : public uart::IBus {
public:
    Bus_remote() = default;

    Bus_remote(RemoteSession& session, uint8_t bus_id, const uart::IBusConfig& cfg)
        : Bus_remote{remote::makeBorrowedSessionHandle(session), bus_id, cfg}
    {
    }

    Bus_remote(std::shared_ptr<remote::RemoteSessionHandle> session, uint8_t bus_id, const uart::IBusConfig& cfg,
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
    result_t<void> beginOperationBackend(bus::OperationContext<uart::AccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<uart::AccessConfig>& context) override;

public:
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

protected:
    result_t<size_t> writeBackend(bus::OperationContext<uart::AccessConfig>& context, data::Source* src,
                                  size_t len) override;
    result_t<size_t> readBackend(bus::OperationContext<uart::AccessConfig>& context, data::Sink* dst,
                                 size_t len) override;
    result_t<bus::TransferTotals> transferBackend(bus::OperationContext<uart::AccessConfig>& tx_context,
                                                  bus::OperationContext<uart::AccessConfig>& rx_context,
                                                  data::Source* src, size_t tx_len, data::Sink* dst,
                                                  size_t rx_len) override;
    result_t<size_t> readableBytesBackend(bus::OperationContext<uart::AccessConfig>& context) override;

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
    runtime::Mutex _state_mutex;
    bool _configured = false;
    uart::AccessConfig _applied_cfg;
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

}  // namespace m5::hal::v2::uart

#endif
