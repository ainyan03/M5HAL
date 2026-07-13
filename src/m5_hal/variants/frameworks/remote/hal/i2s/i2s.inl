// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2S_I2S_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2S_I2S_INL_

#include "i2s.hpp"

#include "../../remote_transfer.hpp"

namespace m5::hal::v2::i2s {

result_t<void> Bus_remote::init(const BusConfig_remote& config)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)config;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return {};
}

result_t<size_t> Bus_remote::write(bus::IAccessor* owner, const i2s::AccessConfig& cfg, data::Source* src, size_t len)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)owner;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (len != 0 && src == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (len == 0) {
        return static_cast<size_t>(0);
    }

    uint8_t cfg_buf[bytecode::kI2SConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::I2S, _bus_id, cfg_bytes, {}, src, len, nullptr, 0,
                                        kTransferTimeoutMs, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return len;
}

result_t<size_t> Bus_remote::writableBytes(bus::IAccessor* owner, const i2s::AccessConfig& cfg)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)owner;
    (void)cfg;
    return static_cast<size_t>(0);
}

result_t<size_t> Bus_remote::read(bus::IAccessor* owner, const i2s::AccessConfig& cfg, data::Sink* dst, size_t len)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)owner;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (len != 0 && dst == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (len == 0) {
        return static_cast<size_t>(0);
    }

    uint8_t cfg_buf[bytecode::kI2SConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::I2S, _bus_id, cfg_bytes, {}, nullptr, 0, dst, len,
                                        kTransferTimeoutMs, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return len;
}

result_t<bus::TransferTotals> Bus_remote::transfer(bus::IAccessor* owner, const i2s::AccessConfig& cfg,
                                                   data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
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

    uint8_t cfg_buf[bytecode::kI2SConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::I2S, _bus_id, cfg_bytes, {}, src, tx_len, dst,
                                        rx_len, kTransferTimeoutMs, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return bus::TransferTotals{tx_len, rx_len};
}

result_t<size_t> Bus_remote::readableBytes(bus::IAccessor* owner, const i2s::AccessConfig& cfg)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)owner;
    (void)cfg;
    return static_cast<size_t>(0);
}

}  // namespace m5::hal::v2::i2s

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2S_I2S_INL_
