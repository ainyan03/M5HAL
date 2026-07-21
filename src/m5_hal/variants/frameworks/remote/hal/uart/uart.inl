// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_INL_

#include "uart.hpp"

#include "../../../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../../../hal/v2/data/memory.hpp"
#include "../../../../../hal/v2/remote/remote.hpp"
#include "../../detail_helpers.hpp"
#include "../../remote_transfer.hpp"

#include <cstdlib>

namespace m5::hal::v2::uart {

namespace {

bool sameAccessConfig(const uart::AccessConfig& lhs, const uart::AccessConfig& rhs)
{
    return lhs.baud_rate == rhs.baud_rate && lhs.data_bits == rhs.data_bits && lhs.stop_bits == rhs.stop_bits &&
           lhs.parity == rhs.parity && lhs.invert == rhs.invert;
}

bool sameFullAccessConfig(const uart::AccessConfig& lhs, const uart::AccessConfig& rhs)
{
    return sameAccessConfig(lhs, rhs) && lhs.first_byte_timeout_ms == rhs.first_byte_timeout_ms &&
           lhs.inter_byte_timeout_ms == rhs.inter_byte_timeout_ms && lhs.write_timeout_ms == rhs.write_timeout_ms;
}

}  // namespace

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

result_t<void> Bus_remote::beginOperationBackend(bus::OperationContext<uart::AccessConfig>& context)
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
    auto locked = _state_mutex.lock(bus::remainingTimeout(context.runtime, runtime::millis()));
    if (!locked.has_value()) {
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock state_unlock{_state_mutex};

    if (_configured && sameFullAccessConfig(_applied_cfg, context.config)) {
        return {};
    }

    IBus* facade = nullptr;
    QuiescenceGrant grant{};
    const bool requires_reconfigure = _configured && !sameAccessConfig(_applied_cfg, context.config);
    if (requires_reconfigure) {
        facade                = &static_cast<IBus&>(accessor->getBus());
        const Channel entered = context.runtime.mode == bus::OperationMode::Tx ? Channel::Tx : Channel::Rx;
        grant                 = facade->tryAcquireOppositeChannel(accessor, entered);
        if (!grant.granted) {
            return m5::stl::make_unexpected(error::error_t::BUSY);
        }
    }
    auto release_gate = [&] {
        if (facade != nullptr) {
            facade->releaseOppositeChannel(accessor, grant);
        }
    };

    uint8_t cfg_buf[bytecode::kUARTConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, context.config);
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto encoded = enc.configure(types::bus_kind_t::UART, _bus_id, cfg_bytes);
    if (encoded.has_value()) {
        encoded = enc.end();
    }
    if (!encoded.has_value()) {
        release_gate();
        return m5::stl::make_unexpected(encoded.error());
    }
    remote::RemoteSessionHandle::Lease lease{*_session};
    if (!lease) {
        release_gate();
        return m5::stl::make_unexpected(lease.error());
    }
    auto& session = lease.session();
    auto request  = session.request({script_buf, script.written()});
    if (!request.has_value()) {
        _config_cache.invalidate();
        release_gate();
        return m5::stl::make_unexpected(request.error());
    }
    auto decoded = remote::detail::decodeResponseStatus(session.lastResponse());
    if (!decoded.has_value()) {
        _config_cache.invalidate();
        release_gate();
        return m5::stl::make_unexpected(decoded.error());
    }
    _config_cache.rememberSent(&session, cfg_bytes);
    _applied_cfg = context.config;
    _configured  = true;
    release_gate();
    return {};
}

result_t<void> Bus_remote::endOperationBackend(bus::OperationContext<uart::AccessConfig>& context)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)context;
    // UART bytecode currently has no begin/end operation opcodes. The local
    // Access still provides channel exclusion and eager configuration; no
    // distributed close can be emitted until that wire seam is added.
    return {};
}

result_t<size_t> Bus_remote::writeBackend(bus::OperationContext<uart::AccessConfig>& context, data::Source* src,
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

    uint8_t cfg_buf[bytecode::kUARTConfigSize];
    auto cfg_bytes            = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    const uint32_t timeout_ms = remote::detail::remoteUartWriteResponseTimeoutMs(cfg.write_timeout_ms, len);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::UART, _bus_id, cfg_bytes, {}, src, len, nullptr, 0,
                                        timeout_ms, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->tx;
}

result_t<size_t> Bus_remote::readBackend(bus::OperationContext<uart::AccessConfig>& context, data::Sink* dst,
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
    if (len != 0 && dst == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (len == 0) {
        return static_cast<size_t>(0);
    }

    uint8_t cfg_buf[bytecode::kUARTConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    const uint32_t timeout_ms =
        remote::detail::remoteUartReadResponseTimeoutMs(cfg.first_byte_timeout_ms, cfg.inter_byte_timeout_ms, len);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::UART, _bus_id, cfg_bytes, {}, nullptr, 0, dst, len,
                                        timeout_ms, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->rx;
}

result_t<bus::TransferTotals> Bus_remote::transferBackend(bus::OperationContext<uart::AccessConfig>& tx_context,
                                                          bus::OperationContext<uart::AccessConfig>& rx_context,
                                                          data::Source* src, size_t tx_len, data::Sink* dst,
                                                          size_t rx_len)
{
    uart::AccessConfig cfg    = tx_context.config;
    cfg.first_byte_timeout_ms = rx_context.config.first_byte_timeout_ms;
    cfg.inter_byte_timeout_ms = rx_context.config.inter_byte_timeout_ms;
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

    uint8_t cfg_buf[bytecode::kUARTConfigSize];
    auto cfg_bytes            = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    const uint32_t timeout_ms = remote::detail::remoteUartTransferResponseTimeoutMs(
        cfg.write_timeout_ms, cfg.first_byte_timeout_ms, cfg.inter_byte_timeout_ms, tx_len, rx_len);
    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::UART, _bus_id, cfg_bytes, {}, src, tx_len, dst,
                                        rx_len, timeout_ms, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r.value();
}

result_t<size_t> Bus_remote::readableBytesBackend(bus::OperationContext<uart::AccessConfig>& context)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)context;
    return static_cast<size_t>(0);
}

}  // namespace m5::hal::v2::uart

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_UART_UART_INL_
