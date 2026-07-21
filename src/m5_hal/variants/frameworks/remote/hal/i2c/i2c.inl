// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2C_I2C_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2C_I2C_INL_

#include "i2c.hpp"

#include "../../remote_transfer.hpp"

#include <cstring>

namespace m5::hal::v2::i2c {

result_t<void> Bus_remote::init(const IBusConfig& config)
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

result_t<void> Bus_remote::transferBackend(bus::OperationContext<i2c::MasterAccessConfig>& context,
                                           const i2c::TransferDesc& desc, data::Source* src, size_t tx_len,
                                           data::Sink* dst, size_t rx_len)
{
    auto* owner     = &bus::OperationSlot::contextOwner(context);
    const auto& cfg = context.config;
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
    // meta_buf only reserves PREFIX_CAPACITY bytes for the prefix; reject an
    // over-length prefix instead of overflowing the stack buffer in encodeI2cMeta.
    if (desc.prefix_len > i2c::TransferDesc::PREFIX_CAPACITY) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    uint8_t cfg_buf[bytecode::kI2CConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    auto r         = remote::remoteAtomicTransferWire(_session, _bus_id, cfg_bytes, desc, src, tx_len, dst, rx_len,
                                                      kTransferTimeoutMs, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    _last_totals = r.value();
    _has_totals  = true;
    return {};
}

result_t<bus::TransferTotals> Bus_remote::waitTransferBackend(bus::OperationContext<i2c::MasterAccessConfig>& context)
{
    auto* owner     = &bus::OperationSlot::contextOwner(context);
    const auto& cfg = context.config;
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)owner;
    (void)cfg;
    auto out    = _has_totals ? _last_totals : bus::TransferTotals{};
    _has_totals = false;
    _last_totals.clear();
    return out;
}

bool Bus_remote::transferBusyBackend(bus::OperationContext<i2c::MasterAccessConfig>& context)
{
    auto* owner = &bus::OperationSlot::contextOwner(context);
    bus::BusLifecycle::Operation operation{*_lifecycle, 0};
    if (!operation) {
        return false;
    }
    (void)owner;
    return false;
}

size_t Bus_remote::encodeI2cMeta(uint8_t* buf, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc)
{
    buf[0] = desc.prefix_len;
    if (desc.prefix_len > 0) {
        ::memcpy(buf + 1, desc.prefix, desc.prefix_len);
    }
    (void)cfg;
    return 1 + desc.prefix_len;
}

}  // namespace m5::hal::v2::i2c

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2C_I2C_INL_
