// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_PDM_PDM_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_PDM_PDM_INL

#include "pdm.hpp"

#include "../../../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../../../hal/v2/data/memory.hpp"
#include "../../detail_helpers.hpp"

namespace m5::hal::v2::pdm {

result_t<void> Bus_remote::init(const IBusConfig& config)
{
    if (config.pin_clk < 0 || config.pin_din < 0 || config.rx_buffer_size == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    return _session ? result_t<void>{} : m5::stl::make_unexpected(error::error_t::INVALID_STATE);
}

result_t<void> Bus_remote::beginOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    uint8_t cfg_buf[bytecode::kPDMConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, context.config);
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder encoder{script};
    auto encoded = encoder.configure(types::bus_kind_t::PDM, _bus_id, cfg_bytes);
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
    return {};
}

result_t<void> Bus_remote::endOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    bus::BusLifecycle::Operation operation{*_lifecycle};
    return operation ? result_t<void>{} : m5::stl::make_unexpected(operation.error());
}

result_t<size_t> Bus_remote::readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len)
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
        return size_t{0};
    }
    uint8_t cfg_buf[bytecode::kPDMConfigSize];
    auto cfg_bytes            = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    const uint32_t timeout_ms = remote::detail::remotePcmResponseTimeoutMs(cfg.sample_rate_hz, cfg.bits_per_sample,
                                                                           cfg.channels, cfg.read_timeout_ms, len);
    auto transferred = remote::remoteTransferWire(_session, types::bus_kind_t::PDM, _bus_id, cfg_bytes, {}, nullptr, 0,
                                                  dst, len, timeout_ms, &_config_cache);
    if (!transferred.has_value()) {
        return m5::stl::make_unexpected(transferred.error());
    }
    return transferred->rx;
}

result_t<size_t> Bus_remote::readableBytesBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    return size_t{0};
}

}  // namespace m5::hal::v2::pdm

#endif
