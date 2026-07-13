// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_SPI_SPI_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_SPI_SPI_INL_

#include "spi.hpp"

#include "../../../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../../../hal/v2/data/memory.hpp"
#include "../../../../../hal/v2/remote/remote.hpp"
#include "../../remote_transfer.hpp"

namespace m5::hal::v2::spi {

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

result_t<void> Bus_remote::beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)owner;
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    uint8_t cfg_buf[bytecode::kSPIConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto r = enc.configure(types::bus_kind_t::SPI, _bus_id, cfg_bytes);
    if (r.has_value()) {
        r = enc.busBeginTransaction(types::bus_kind_t::SPI, _bus_id);
    }
    if (r.has_value()) {
        r = enc.end();
    }
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    remote::RemoteSessionHandle::Lease lease{*_session};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }
    auto& session = lease.session();
    auto req      = session.request({script_buf, script.written()});
    if (!req.has_value()) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(req.error());
    }
    bytecode::BytecodeRunner runner{memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto resp = session.lastResponse();
    auto run  = runner.run(resp);
    if (!run.has_value()) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(run.error());
    }
    if (!runner.statusReported()) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    if (error::isError(runner.reportedStatus())) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(runner.reportedStatus());
    }
    _in_transaction = true;
    _config_cache.rememberSent(&session, cfg_bytes);
    return {};
}

result_t<void> Bus_remote::endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)owner;
    (void)cfg;
    if (_session == nullptr || !_in_transaction) {
        _in_transaction = false;
        return {};
    }
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto r = enc.busEndTransaction(types::bus_kind_t::SPI, _bus_id);
    if (r.has_value()) {
        r = enc.end();
    }
    if (!r.has_value()) {
        _in_transaction = false;
        return m5::stl::make_unexpected(r.error());
    }
    remote::RemoteSessionHandle::Lease lease{*_session};
    if (!lease) {
        _in_transaction = false;
        return m5::stl::make_unexpected(lease.error());
    }
    auto& session   = lease.session();
    auto req        = session.request({script_buf, script.written()});
    _in_transaction = false;
    if (!req.has_value()) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(req.error());
    }
    bytecode::BytecodeRunner runner{memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto resp = session.lastResponse();
    auto run  = runner.run(resp);
    if (!run.has_value()) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(run.error());
    }
    if (!runner.statusReported()) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    if (error::isError(runner.reportedStatus())) {
        _config_cache.invalidate();
        return m5::stl::make_unexpected(runner.reportedStatus());
    }
    return {};
}

result_t<void> Bus_remote::transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg,
                                    const spi::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                    size_t rx_len)
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

    uint8_t cfg_buf[bytecode::kSPIConfigSize];
    auto cfg_bytes = remote::detail::encodeRemoteConfig(cfg_buf, cfg);
    uint8_t meta_buf[15];
    data::ConstDataSpan meta = encodeSpiMeta(meta_buf, desc);

    auto r = remote::remoteTransferWire(_session, types::bus_kind_t::SPI, _bus_id, cfg_bytes, meta, src, tx_len, dst,
                                        rx_len, kTransferTimeoutMs, &_config_cache);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    _last_totals = bus::TransferTotals{tx_len, rx_len};
    return {};
}

result_t<bus::TransferTotals> Bus_remote::waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    bus::BusLifecycle::Operation operation{*_lifecycle};
    if (!operation) {
        return m5::stl::make_unexpected(operation.error());
    }
    (void)owner;
    (void)cfg;
    auto totals  = _last_totals;
    _last_totals = bus::TransferTotals{};
    return totals;
}

bool Bus_remote::transferBusy(bus::IAccessor* owner)
{
    bus::BusLifecycle::Operation operation{*_lifecycle, 0};
    if (!operation) {
        return false;
    }
    (void)owner;
    return false;
}

data::ConstDataSpan Bus_remote::encodeSpiMeta(uint8_t* buf, const spi::TransferDesc& desc)
{
    buf[0]  = static_cast<uint8_t>((desc.dc_level_valid ? 0x01 : 0x00) | (desc.dc_level ? 0x02 : 0x00));
    buf[1]  = static_cast<uint8_t>(desc.command_dc_level);
    buf[2]  = static_cast<uint8_t>(desc.address_dc_level);
    buf[3]  = static_cast<uint8_t>(desc.data_dc_level);
    buf[4]  = static_cast<uint8_t>(desc.command & 0xFF);
    buf[5]  = static_cast<uint8_t>((desc.command >> 8) & 0xFF);
    buf[6]  = static_cast<uint8_t>((desc.command >> 16) & 0xFF);
    buf[7]  = static_cast<uint8_t>((desc.command >> 24) & 0xFF);
    buf[8]  = static_cast<uint8_t>(desc.address & 0xFF);
    buf[9]  = static_cast<uint8_t>((desc.address >> 8) & 0xFF);
    buf[10] = static_cast<uint8_t>((desc.address >> 16) & 0xFF);
    buf[11] = static_cast<uint8_t>((desc.address >> 24) & 0xFF);
    buf[12] = desc.command_bytes;
    buf[13] = desc.address_bytes;
    buf[14] = desc.dummy_cycles;
    return {buf, 15};
}

}  // namespace m5::hal::v2::spi

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_SPI_SPI_INL_
