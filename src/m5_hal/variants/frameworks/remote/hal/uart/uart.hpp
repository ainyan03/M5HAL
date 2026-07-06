// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_HPP

#include "../../../../../hal/v2/uart/uart.hpp"
#include "../../remote_transfer.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::uart {

using remote::RemoteSession;

struct BusConfig_remote : public uart::IBusConfig {
    using uart::IBusConfig::IBusConfig;

    constexpr BusConfig_remote(void) : uart::IBusConfig{}
    {
    }
};

class Bus_remote : public uart::IBus {
public:
    Bus_remote() = default;

    Bus_remote(RemoteSession& session, uint8_t bus_id, const uart::IBusConfig& cfg)
        : _session{&session}, _bus_id{bus_id}
    {
        _config = cfg;
    }

    result_t<void> init(const BusConfig_remote& config);

    uint8_t busId(void) const
    {
        return _bus_id;
    }

    result_t<size_t> write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src,
                           size_t len) override;
    result_t<size_t> read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len) override;
    result_t<bus::TransferTotals> transfer(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src,
                                           size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<size_t> readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg) override;

private:
    static constexpr uint32_t kTransferTimeoutMs = 5000;

    RemoteSession* _session = nullptr;
    uint8_t _bus_id         = 0;
    remote::RemoteConfigCache _config_cache;
};

template <>
struct BackendFor<BusConfig_remote> {
    using type = Bus_remote;
};

}  // namespace m5::hal::v2::uart

#endif
