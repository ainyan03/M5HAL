// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_INL_

#include "uart.hpp"

#include "../../remote_transfer.hpp"

namespace m5::hal::v2::uart {

result_t<void> Bus_remote::init(const BusConfig_remote& config)
{
    (void)config;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return {};
}

result_t<size_t> Bus_remote::write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src, size_t len)
{
    (void)owner;
    if (_session == nullptr || src == nullptr || len == 0) {
        return static_cast<size_t>(0);
    }

    uint8_t cfg_buf[bytecode::kUARTConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::UART, _bus_id, cfg_bytes, {}, src, len, nullptr, 0,
                                        kTransferTimeoutMs, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return len;
}

result_t<size_t> Bus_remote::read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len)
{
    (void)owner;
    if (_session == nullptr || dst == nullptr || len == 0) {
        return static_cast<size_t>(0);
    }

    uint8_t cfg_buf[bytecode::kUARTConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::UART, _bus_id, cfg_bytes, {}, nullptr, 0, dst, len,
                                        kTransferTimeoutMs, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return len;
}

result_t<bus::TransferTotals> Bus_remote::transfer(bus::IAccessor* owner, const uart::AccessConfig& cfg,
                                                   data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len)
{
    (void)owner;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if ((tx_len != 0 && src == nullptr) || (rx_len != 0 && dst == nullptr)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (tx_len == 0 && rx_len == 0) {
        return bus::TransferTotals{};
    }

    uint8_t cfg_buf[bytecode::kUARTConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::UART, _bus_id, cfg_bytes, {}, src, tx_len, dst,
                                        rx_len, kTransferTimeoutMs, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return bus::TransferTotals{tx_len, rx_len};
}

result_t<size_t> Bus_remote::readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return static_cast<size_t>(0);
}

}  // namespace m5::hal::v2::uart

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_INL_
