// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2S_I2S_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2S_I2S_HPP

#include "../../../../../hal/v2/i2s/i2s.hpp"
#include "../../remote_transfer.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::i2s {

using remote::RemoteSession;

struct BusConfig_remote : public i2s::IBusConfig {
    using i2s::IBusConfig::IBusConfig;

    constexpr BusConfig_remote(void) : i2s::IBusConfig{}
    {
    }
};

class Bus_remote : public i2s::IBus {
public:
    Bus_remote() = default;

    Bus_remote(RemoteSession& session, uint8_t bus_id) : Bus_remote{session, bus_id, i2s::IBusConfig{}}
    {
    }

    Bus_remote(RemoteSession& session, uint8_t bus_id, const i2s::IBusConfig& cfg) : _session{&session}, _bus_id{bus_id}
    {
        _config = cfg;
    }

    result_t<void> init(const BusConfig_remote& config);

    uint8_t busId(void) const
    {
        return _bus_id;
    }

    result_t<size_t> write(bus::IAccessor* owner, const i2s::AccessConfig& cfg, data::Source* src, size_t len) override;
    result_t<size_t> writableBytes(bus::IAccessor* owner, const i2s::AccessConfig& cfg) override;
    result_t<size_t> read(bus::IAccessor* owner, const i2s::AccessConfig& cfg, data::Sink* dst, size_t len) override;
    result_t<bus::TransferTotals> transfer(bus::IAccessor* owner, const i2s::AccessConfig& cfg, data::Source* src,
                                           size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<size_t> readableBytes(bus::IAccessor* owner, const i2s::AccessConfig& cfg) override;

private:
    static constexpr uint32_t kTransferTimeoutMs = 10000;

    RemoteSession* _session = nullptr;
    uint8_t _bus_id         = 0;
    remote::RemoteConfigCache _config_cache;
};

template <>
struct BackendFor<BusConfig_remote> {
    using type = Bus_remote;
};

}  // namespace m5::hal::v2::i2s

#endif
