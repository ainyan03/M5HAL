// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2C_I2C_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2C_I2C_HPP

#include "../../../../../hal/v2/i2c/i2c.hpp"
#include "../../bus_lease.hpp"
#include "../../remote_transfer.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::i2c {

using remote::RemoteSession;

struct BusConfig_remote : public i2c::IBusConfig {
    using i2c::IBusConfig::IBusConfig;

    constexpr BusConfig_remote(void) : i2c::IBusConfig{}
    {
    }
};

class Bus_remote : public i2c::IBus {
public:
    Bus_remote() = default;

    Bus_remote(RemoteSession& session, uint8_t bus_id, const i2c::IBusConfig& cfg)
        : Bus_remote{remote::makeBorrowedSessionHandle(session), bus_id, cfg}
    {
    }

    Bus_remote(std::shared_ptr<remote::RemoteSessionHandle> session, uint8_t bus_id, const i2c::IBusConfig& cfg,
               std::shared_ptr<remote::RemoteBusLease> bus_lease = {})
        : _session{std::move(session)},
          _bus_id{bus_id},
          _lifecycle{bus_lease ? bus_lease->lifecycle() : std::make_shared<bus::BusLifecycle>()},
          _bus_lease{std::move(bus_lease)}
    {
        _config = cfg;
    }

    result_t<void> init(const BusConfig_remote& config);
    result_t<void> transfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg) override;
    bool transferBusy(bus::IAccessor* owner) override;

    /*! @brief Remote bus_id assigned to this proxy on the peer. */
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
    static constexpr uint32_t kTransferTimeoutMs = 5000;

    static size_t encodeI2cMeta(uint8_t* buf, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc);

    std::shared_ptr<remote::RemoteSessionHandle> _session;
    uint8_t _bus_id                               = 0;
    std::shared_ptr<bus::BusLifecycle> _lifecycle = std::make_shared<bus::BusLifecycle>();
    std::shared_ptr<remote::RemoteBusLease> _bus_lease;
    bool _has_totals = false;
    bus::TransferTotals _last_totals;
    remote::RemoteConfigCache _config_cache;
};

template <>
struct BackendFor<BusConfig_remote> {
    using type = Bus_remote;
};

}  // namespace m5::hal::v2::i2c

#endif
