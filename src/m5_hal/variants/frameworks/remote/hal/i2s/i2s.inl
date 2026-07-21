// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2S_I2S_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2S_I2S_INL_

#include "i2s.hpp"

#include "../../../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../../../hal/v2/data/memory.hpp"
#include "../../detail_helpers.hpp"
#include "../../remote_transfer.hpp"

#include <cstdlib>

namespace m5::hal::v2::i2s {

namespace {
void putU32(uint8_t* dst, uint32_t value)
{
    dst[0] = static_cast<uint8_t>(value);
    dst[1] = static_cast<uint8_t>(value >> 8);
    dst[2] = static_cast<uint8_t>(value >> 16);
    dst[3] = static_cast<uint8_t>(value >> 24);
}
}  // namespace

result_t<void> Bus_remote::init(const IBusConfig& config)
{
    if (config.pin_bclk < 0 || config.pin_ws < 0 || (config.pin_dout < 0 && config.pin_din < 0)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return {};
}

result_t<void> Bus_remote::beginOperationBackend(bus::OperationContext<i2s::AccessConfig>& context)
{
    auto* accessor = operationOwner(context);
    if (accessor == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    const bool tx = context.runtime.mode == bus::OperationMode::Tx;
    const bool rx = context.runtime.mode == bus::OperationMode::Rx;
    if (!tx && !rx) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto locked = _operation_mutex.lock(bus::remainingTimeout(context.runtime, runtime::millis()));
    if (!locked.has_value()) {
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock unlocker{_operation_mutex};
    const bus::IAccessor* opposite = (tx ? _active_rx_owner : _active_tx_owner).load(std::memory_order_acquire);
    if (opposite != nullptr && _operation_configured &&
        (_active_operation_cfg.sample_rate_hz != context.config.sample_rate_hz ||
         _active_operation_cfg.bits_per_sample != context.config.bits_per_sample ||
         _active_operation_cfg.channels != context.config.channels)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (opposite != nullptr && _operation_configured) {
        // TX and RX share one remote I2S format.  The first side has already
        // configured it; opening the matching opposite side must only acquire
        // that side's authority, not emit a duplicate configure request.
        if (tx) {
            _active_tx_owner.store(accessor, std::memory_order_release);
        } else {
            _active_rx_owner.store(accessor, std::memory_order_release);
        }
        return {};
    }
    uint8_t cfg_buf[bytecode::kI2SConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, context.config);
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder encoder{script};
    auto encoded = encoder.configure(types::bus_kind_t::I2S, _bus_id, cfg_bytes);
    if (encoded.has_value()) {
        encoded = encoder.end();
    }
    if (!encoded.has_value()) {
        return m5::stl::make_unexpected(encoded.error());
    }
    remote::RemoteSessionHandle::Lease lease{*_session};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }
    auto& session  = lease.session();
    auto requested = session.request({script_buf, script.written()});
    if (!requested.has_value()) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(requested.error());
    }
    auto decoded = remote::detail::decodeResponseStatus(session.lastResponse());
    if (!decoded.has_value()) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(decoded.error());
    }
    _config_cache.rememberSent(&session, cfg_bytes);
    if (tx) {
        _active_tx_owner.store(accessor, std::memory_order_release);
    } else {
        _active_rx_owner.store(accessor, std::memory_order_release);
    }
    _active_operation_cfg = context.config;
    _operation_configured = true;
    return {};
}

result_t<void> Bus_remote::endOperationBackend(bus::OperationContext<i2s::AccessConfig>& context)
{
    auto* accessor     = operationOwner(context);
    auto& active       = context.runtime.mode == bus::OperationMode::Tx ? _active_tx_owner : _active_rx_owner;
    auto recover_owner = [&] {
        bus::IAccessor* expected = accessor;
        (void)active.compare_exchange_strong(expected, nullptr, std::memory_order_acq_rel);
    };
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        recover_owner();
        return m5::stl::make_unexpected(operation.error());
    }
    auto locked = _operation_mutex.lock(bus::remainingTimeout(context.runtime, runtime::millis()));
    if (!locked.has_value()) {
        recover_owner();
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock unlocker{_operation_mutex};
    if (active.load(std::memory_order_acquire) != accessor) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    active.store(nullptr, std::memory_order_release);
    if (_active_tx_owner.load(std::memory_order_acquire) == nullptr &&
        _active_rx_owner.load(std::memory_order_acquire) == nullptr) {
        _operation_configured = false;
    }
    return {};
}

result_t<size_t> Bus_remote::writeBackend(bus::OperationContext<i2s::AccessConfig>& context, data::Source* src,
                                          size_t len)
{
    const auto& cfg = context.config;
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
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
    auto cfg_bytes            = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    const uint32_t timeout_ms = remote::detail::remotePcmResponseTimeoutMs(cfg.sample_rate_hz, cfg.bits_per_sample,
                                                                           cfg.channels, cfg.write_timeout_ms, len);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::I2S, _bus_id, cfg_bytes, {}, src, len, nullptr, 0,
                                        timeout_ms, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->tx;
}

result_t<size_t> Bus_remote::writableBytesBackend(bus::OperationContext<i2s::AccessConfig>& context)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)context;
    return static_cast<size_t>(0);
}

result_t<size_t> Bus_remote::readBackend(bus::OperationContext<i2s::AccessConfig>& context, data::Sink* dst, size_t len)
{
    const auto& cfg = context.config;
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
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
    auto cfg_bytes            = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    const uint32_t timeout_ms = remote::detail::remotePcmResponseTimeoutMs(cfg.sample_rate_hz, cfg.bits_per_sample,
                                                                           cfg.channels, cfg.read_timeout_ms, len);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::I2S, _bus_id, cfg_bytes, {}, nullptr, 0, dst, len,
                                        timeout_ms, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->rx;
}

result_t<bus::TransferTotals> Bus_remote::transferBackend(bus::OperationContext<i2s::AccessConfig>& tx_context,
                                                          bus::OperationContext<i2s::AccessConfig>& rx_context,
                                                          data::Source* src, size_t tx_len, data::Sink* dst,
                                                          size_t rx_len)
{
    const auto& tx_cfg = tx_context.config;
    const auto& rx_cfg = rx_context.config;
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if ((tx_len != 0 && src == nullptr) || (rx_len != 0 && dst == nullptr)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (tx_len == 0 && rx_len == 0) {
        return bus::TransferTotals{};
    }

    if (tx_len != 0 && rx_len != 0 &&
        (tx_cfg.sample_rate_hz != rx_cfg.sample_rate_hz || tx_cfg.bits_per_sample != rx_cfg.bits_per_sample ||
         tx_cfg.channels != rx_cfg.channels)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    uint8_t cfg_buf[bytecode::kI2SConfigSize];
    const auto& format_cfg = tx_len != 0 ? tx_cfg : rx_cfg;
    putU32(cfg_buf, format_cfg.sample_rate_hz);
    putU32(cfg_buf + bytecode::kI2SConfigWriteTimeoutOffset, tx_cfg.write_timeout_ms);
    cfg_buf[8] = format_cfg.bits_per_sample;
    cfg_buf[9] = format_cfg.channels;
    putU32(cfg_buf + bytecode::kI2SConfigReadTimeoutOffset, rx_cfg.read_timeout_ms);
    data::ConstDataSpan cfg_bytes{cfg_buf, sizeof(cfg_buf)};
    // Full-duplex: both directions run concurrently at the shared format, so
    // the data term is the longer of the two; each side keeps its own DMA
    // wait budget.
    const uint32_t tx_ms = remote::detail::remotePcmDurationMs(format_cfg.sample_rate_hz, format_cfg.bits_per_sample,
                                                               format_cfg.channels, tx_len);
    const uint32_t rx_ms = remote::detail::remotePcmDurationMs(format_cfg.sample_rate_hz, format_cfg.bits_per_sample,
                                                               format_cfg.channels, rx_len);
    const uint32_t timeout_ms = remote::detail::clampBelowForever(remote::detail::saturatingAddU32(
        remote::detail::saturatingAddU32(
            tx_ms > rx_ms ? tx_ms : rx_ms,
            remote::detail::saturatingAddU32(tx_cfg.write_timeout_ms, rx_cfg.read_timeout_ms)),
        remote::kRemoteTimeoutMarginMs));
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::I2S, _bus_id, cfg_bytes, {}, src, tx_len, dst,
                                        rx_len, timeout_ms, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r.value();
}

result_t<size_t> Bus_remote::readableBytesBackend(bus::OperationContext<i2s::AccessConfig>& context)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)context;
    return static_cast<size_t>(0);
}

}  // namespace m5::hal::v2::i2s

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_I2S_I2S_INL_
