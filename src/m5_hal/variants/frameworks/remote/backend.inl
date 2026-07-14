// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_BACKEND_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_BACKEND_INL_

#include "backend.hpp"
#include "detail_helpers.hpp"

#include "../../../hal/v2/bytecode/bytecode.hpp"

namespace m5::hal::v2::remote {

result_t<std::shared_ptr<bus::IBus>> RemoteBackend::acquireBusTyped(types::bus_kind_t kind, const bus::IdentityKey& id,
                                                                    const bus::IBusConfig& cfg)
{
    uint8_t pin_buf[16];
    auto pin_r = extractPinConfig(kind, cfg, pin_buf, sizeof(pin_buf));
    if (!pin_r.has_value()) {
        return m5::stl::make_unexpected(pin_r.error());
    }
    data::ConstDataSpan pin_config{pin_buf, pin_r.value()};
    return busRegistry().acquireOrFind(
        kind, id,
        [&](const std::shared_ptr<bus::IBus>& existing) -> result_t<void> {
            if (!sameBaseConfig(kind, existing->getConfig(), cfg)) {
                return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
            }
            return {};
        },
        [&]() -> result_t<std::shared_ptr<bus::IBus>> { return createRemoteBus(kind, pin_config, cfg); });
}

result_t<std::shared_ptr<bus::IBus>> RemoteBackend::acquireBusLogical(types::bus_kind_t kind,
                                                                      const bus::IdentityKey& id,
                                                                      const bus::AllocationRequest& req)
{
    // AllocationIntent is local resolver metadata and is not carried by the
    // BusCreate wire format. Do not silently drop an SPI protocol guarantee:
    // remote feature negotiation must be added before a peer can promise it.
    if (kind == types::bus_kind_t::SPI && (req.intent.require & spi::caps::MOSI_SHARED_RX) != 0) {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
    uint8_t pin_buf[16];
    auto pin_r = extractPinConfigFromLogical(kind, req.config, pin_buf, sizeof(pin_buf));
    if (!pin_r.has_value()) {
        return m5::stl::make_unexpected(pin_r.error());
    }
    data::ConstDataSpan pin_config{pin_buf, pin_r.value()};
    return busRegistry().acquireOrFind(kind, id, [&]() -> result_t<std::shared_ptr<bus::IBus>> {
        return createRemoteBusFromLogical(kind, pin_config);
    });
}

result_t<void> RemoteBackend::commitBuses(types::bus_kind_t kind, uint32_t timeout_ms)
{
    (void)kind;
    (void)timeout_ms;
    return {};
}

result_t<void> RemoteBackend::releaseBus(types::bus_kind_t kind, const bus::IdentityKey& id,
                                         const std::shared_ptr<bus::IBus>& expected)
{
    auto ticket = busRegistry().beginRelease(kind, id, expected);
    if (!ticket.has_value()) {
        return m5::stl::make_unexpected(ticket.error());
    }
    const uint8_t remote_bus_id                  = extractBusId(kind, expected);
    auto lease                                   = extractBusLease(kind, expected);
    std::shared_ptr<bus::BusLifecycle> lifecycle = lease ? lease->lifecycle() : expected->lifecycleHandle();
    if (!lifecycle) {
        (void)busRegistry().cancelRelease(ticket.value());
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    bus::BusLifecycle::Close closing{*lifecycle};
    if (!closing) {
        (void)busRegistry().cancelRelease(ticket.value());
        return m5::stl::make_unexpected(closing.error());
    }
    if (remote_bus_id != 0xFF) {
        auto sent = sendReleaseBus(kind, remote_bus_id);
        if (!sent.has_value()) {
            closing.rollback();
            (void)busRegistry().cancelRelease(ticket.value());
            return m5::stl::make_unexpected(sent.error());
        }
    }
    closing.commit();
    if (lease) {
        lease->disarm();
    } else if (remote_bus_id != 0xFF) {
        freeBusId(kind, remote_bus_id);
    }
    auto committed = busRegistry().commitRelease(ticket.value());
    if (!committed.has_value()) {
        return m5::stl::make_unexpected(committed.error());
    }
    return {};
}

std::shared_ptr<RemoteBusLease> RemoteBackend::extractBusLease(types::bus_kind_t kind,
                                                               const std::shared_ptr<bus::IBus>& bus)
{
    switch (kind) {
        case types::bus_kind_t::I2C:
            return static_cast<i2c::Bus_remote*>(bus.get())->remoteBusLease();
        case types::bus_kind_t::SPI:
            return static_cast<spi::Bus_remote*>(bus.get())->remoteBusLease();
        case types::bus_kind_t::UART:
            return static_cast<uart::Bus_remote*>(bus.get())->remoteBusLease();
        case types::bus_kind_t::I2S:
            return static_cast<i2s::Bus_remote*>(bus.get())->remoteBusLease();
        default:
            return {};
    }
}

uint8_t RemoteBackend::extractBusId(types::bus_kind_t kind, const std::shared_ptr<bus::IBus>& bus)
{
    switch (kind) {
        case types::bus_kind_t::I2C:
            return static_cast<i2c::Bus_remote*>(bus.get())->busId();
        case types::bus_kind_t::SPI:
            return static_cast<spi::Bus_remote*>(bus.get())->busId();
        case types::bus_kind_t::UART:
            return static_cast<uart::Bus_remote*>(bus.get())->busId();
        case types::bus_kind_t::I2S:
            return static_cast<i2s::Bus_remote*>(bus.get())->busId();
        default:
            return 0xFF;
    }
}

bool RemoteBackend::sameBaseConfig(types::bus_kind_t kind, const bus::IBusConfig& current,
                                   const bus::IBusConfig& requested)
{
    switch (kind) {
        case types::bus_kind_t::I2C: {
            const auto& a = static_cast<const i2c::IBusConfig&>(current);
            const auto& b = static_cast<const i2c::IBusConfig&>(requested);
            return a.pin_scl == b.pin_scl && a.pin_sda == b.pin_sda;
        }
        case types::bus_kind_t::SPI: {
            const auto& a = static_cast<const spi::IBusConfig&>(current);
            const auto& b = static_cast<const spi::IBusConfig&>(requested);
            return a.pin_clk == b.pin_clk && a.pin_mosi == b.pin_mosi && a.pin_miso == b.pin_miso;
        }
        case types::bus_kind_t::UART: {
            const auto& a = static_cast<const uart::IBusConfig&>(current);
            const auto& b = static_cast<const uart::IBusConfig&>(requested);
            return a.pin_tx == b.pin_tx && a.pin_rx == b.pin_rx &&
                   detail::sizeToUnit(a.rx_buffer_size, 256) == detail::sizeToUnit(b.rx_buffer_size, 256) &&
                   detail::sizeToUnit(a.tx_buffer_size, 256) == detail::sizeToUnit(b.tx_buffer_size, 256);
        }
        case types::bus_kind_t::I2S: {
            const auto& a = static_cast<const i2s::IBusConfig&>(current);
            const auto& b = static_cast<const i2s::IBusConfig&>(requested);
            return a.pin_bclk == b.pin_bclk && a.pin_ws == b.pin_ws && a.pin_dout == b.pin_dout &&
                   a.pin_din == b.pin_din && a.role == b.role &&
                   detail::sizeToUnit(a.tx_buffer_size, 1024) == detail::sizeToUnit(b.tx_buffer_size, 1024) &&
                   detail::sizeToUnit(a.rx_buffer_size, 1024) == detail::sizeToUnit(b.rx_buffer_size, 1024);
        }
        default:
            return false;
    }
}

result_t<void> RemoteBackend::sendReleaseBus(types::bus_kind_t kind, uint8_t bus_id)
{
    uint8_t script_buf[kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto r = enc.busRelease(kind, bus_id);
    if (r.has_value()) {
        r = enc.end();
    }
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    if (!_session) {
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }
    RemoteSessionHandle::Lease lease{*_session};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }
    auto& session = lease.session();
    auto req      = session.request({script_buf, script.written()});
    if (!req.has_value()) {
        return m5::stl::make_unexpected(req.error());
    }
    bytecode::BytecodeRunner runner{memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto resp = session.lastResponse();
    auto run  = runner.run(resp);
    if (!run.has_value()) {
        return m5::stl::make_unexpected(run.error());
    }
    if (!runner.statusReported()) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    if (error::isError(runner.reportedStatus())) {
        return m5::stl::make_unexpected(runner.reportedStatus());
    }
    return {};
}

result_t<std::shared_ptr<bus::IBus>> RemoteBackend::createRemoteBus(types::bus_kind_t kind,
                                                                    data::ConstDataSpan pin_config,
                                                                    const bus::IBusConfig& cfg)
{
    const uint8_t bus_id = allocateBusId(kind);
    if (bus_id == 0xFF) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    auto bus_lease = std::make_shared<RemoteBusLease>(_session, _bus_ids, kind, bus_id);
    if (!bus_lease) {
        freeBusId(kind, bus_id);
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    auto proxy = makeProxyBus(kind, bus_id, cfg, bus_lease);
    if (!proxy.has_value()) {
        return m5::stl::make_unexpected(proxy.error());
    }
    bus_lease->arm();
    auto r = sendCreateBus(kind, bus_id, pin_config);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return proxy;
}

result_t<std::shared_ptr<bus::IBus>> RemoteBackend::createRemoteBusFromLogical(types::bus_kind_t kind,
                                                                               data::ConstDataSpan pin_config)
{
    const uint8_t bus_id = allocateBusId(kind);
    if (bus_id == 0xFF) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    auto bus_lease = std::make_shared<RemoteBusLease>(_session, _bus_ids, kind, bus_id);
    if (!bus_lease) {
        freeBusId(kind, bus_id);
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    auto proxy = makeProxyBusFromPinConfig(kind, bus_id, pin_config, bus_lease);
    if (!proxy.has_value()) {
        return m5::stl::make_unexpected(proxy.error());
    }
    bus_lease->arm();
    auto r = sendCreateBus(kind, bus_id, pin_config);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return proxy;
}

result_t<void> RemoteBackend::sendCreateBus(types::bus_kind_t kind, uint8_t bus_id, data::ConstDataSpan pin_config)
{
    uint8_t script_buf[kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto r = enc.busCreate(kind, bus_id, bytecode::kDiscardStoreId, pin_config);
    if (r.has_value()) {
        r = enc.end();
    }
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    if (!_session) {
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }
    RemoteSessionHandle::Lease lease{*_session};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }
    auto& session = lease.session();
    auto req      = session.request({script_buf, script.written()});
    if (!req.has_value()) {
        return m5::stl::make_unexpected(req.error());
    }
    bytecode::BytecodeRunner runner{memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    auto resp = session.lastResponse();
    auto run  = runner.run(resp);
    if (!run.has_value()) {
        return m5::stl::make_unexpected(run.error());
    }
    if (!runner.statusReported()) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    if (error::isError(runner.reportedStatus())) {
        return m5::stl::make_unexpected(runner.reportedStatus());
    }
    return {};
}

result_t<std::shared_ptr<bus::IBus>> RemoteBackend::makeProxyBus(types::bus_kind_t kind, uint8_t bus_id,
                                                                 const bus::IBusConfig& cfg,
                                                                 const std::shared_ptr<RemoteBusLease>& bus_lease)
{
    switch (kind) {
        case types::bus_kind_t::I2C: {
            auto p = std::make_shared<i2c::Bus_remote>(_session, bus_id, static_cast<const i2c::IBusConfig&>(cfg),
                                                       bus_lease);
            if (!p) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            return std::shared_ptr<bus::IBus>{p};
        }
        case types::bus_kind_t::SPI: {
            auto p = std::make_shared<spi::Bus_remote>(_session, bus_id, static_cast<const spi::IBusConfig&>(cfg),
                                                       bus_lease);
            if (!p) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            return std::shared_ptr<bus::IBus>{p};
        }
        case types::bus_kind_t::UART: {
            auto p = std::make_shared<uart::Bus_remote>(_session, bus_id, static_cast<const uart::IBusConfig&>(cfg),
                                                        bus_lease);
            if (!p) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            return std::shared_ptr<bus::IBus>{p};
        }
        case types::bus_kind_t::I2S: {
            auto p = std::make_shared<i2s::Bus_remote>(_session, bus_id, static_cast<const i2s::IBusConfig&>(cfg),
                                                       bus_lease);
            if (!p) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            return std::shared_ptr<bus::IBus>{p};
        }
        default:
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
}

result_t<std::shared_ptr<bus::IBus>> RemoteBackend::makeProxyBusFromPinConfig(
    types::bus_kind_t kind, uint8_t bus_id, data::ConstDataSpan pin_config,
    const std::shared_ptr<RemoteBusLease>& bus_lease)
{
    switch (kind) {
        case types::bus_kind_t::I2C: {
            if (pin_config.size < 4) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }
            i2c::IBusConfig cfg;
            cfg.pin_scl = readI16LE(pin_config.data);
            cfg.pin_sda = readI16LE(pin_config.data + 2);
            auto p      = std::make_shared<i2c::Bus_remote>(_session, bus_id, cfg, bus_lease);
            if (!p) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            return std::shared_ptr<bus::IBus>{p};
        }
        case types::bus_kind_t::SPI: {
            if (pin_config.size < 6) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }
            spi::IBusConfig cfg;
            cfg.pin_clk  = readI16LE(pin_config.data);
            cfg.pin_mosi = readI16LE(pin_config.data + 2);
            cfg.pin_miso = readI16LE(pin_config.data + 4);
            auto p       = std::make_shared<spi::Bus_remote>(_session, bus_id, cfg, bus_lease);
            if (!p) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            return std::shared_ptr<bus::IBus>{p};
        }
        case types::bus_kind_t::UART: {
            if (pin_config.size < 7) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }
            uart::IBusConfig cfg;
            cfg.pin_tx         = readI16LE(pin_config.data);
            cfg.pin_rx         = readI16LE(pin_config.data + 2);
            cfg.rx_buffer_size = static_cast<size_t>(pin_config.data[5]) * 256;
            cfg.tx_buffer_size = static_cast<size_t>(pin_config.data[6]) * 256;
            auto p             = std::make_shared<uart::Bus_remote>(_session, bus_id, cfg, bus_lease);
            if (!p) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            return std::shared_ptr<bus::IBus>{p};
        }
        case types::bus_kind_t::I2S: {
            if (pin_config.size < 11) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }
            i2s::IBusConfig cfg;
            cfg.pin_bclk       = readI16LE(pin_config.data);
            cfg.pin_ws         = readI16LE(pin_config.data + 2);
            cfg.pin_dout       = readI16LE(pin_config.data + 4);
            cfg.pin_din        = readI16LE(pin_config.data + 6);
            cfg.role           = pin_config.data[8] != 0 ? i2s::IBusConfig::Role::Slave : i2s::IBusConfig::Role::Master;
            cfg.tx_buffer_size = static_cast<size_t>(pin_config.data[9]) * 1024;
            cfg.rx_buffer_size = static_cast<size_t>(pin_config.data[10]) * 1024;
            auto p             = std::make_shared<i2s::Bus_remote>(_session, bus_id, cfg, bus_lease);
            if (!p) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            return std::shared_ptr<bus::IBus>{p};
        }
        default:
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
}

types::gpio_number_t RemoteBackend::readI16LE(const uint8_t* p)
{
    return static_cast<types::gpio_number_t>(
        static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8)));
}

result_t<size_t> RemoteBackend::extractPinConfig(types::bus_kind_t kind, const bus::IBusConfig& cfg, uint8_t* out,
                                                 size_t out_size)
{
    switch (kind) {
        case types::bus_kind_t::I2C: {
            if (out_size < 4) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            const auto& c = static_cast<const i2c::IBusConfig&>(cfg);
            detail::putI16LE(out, c.pin_scl);
            detail::putI16LE(out + 2, c.pin_sda);
            return size_t{4};
        }
        case types::bus_kind_t::SPI: {
            if (out_size < 6) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            const auto& c = static_cast<const spi::IBusConfig&>(cfg);
            detail::putI16LE(out, c.pin_clk);
            detail::putI16LE(out + 2, c.pin_mosi);
            detail::putI16LE(out + 4, c.pin_miso);
            return size_t{6};
        }
        case types::bus_kind_t::UART: {
            if (out_size < 7) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            const auto& c = static_cast<const uart::IBusConfig&>(cfg);
            detail::putI16LE(out, c.pin_tx);
            detail::putI16LE(out + 2, c.pin_rx);
            out[4] = 0;
            out[5] = detail::sizeToUnit(c.rx_buffer_size, 256);
            out[6] = detail::sizeToUnit(c.tx_buffer_size, 256);
            return size_t{7};
        }
        case types::bus_kind_t::I2S: {
            if (out_size < 11) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            const auto& c = static_cast<const i2s::IBusConfig&>(cfg);
            if (c.pin_bclk < 0 || c.pin_ws < 0 || (c.pin_dout < 0 && c.pin_din < 0)) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }
            detail::putI16LE(out, c.pin_bclk);
            detail::putI16LE(out + 2, c.pin_ws);
            detail::putI16LE(out + 4, c.pin_dout);
            detail::putI16LE(out + 6, c.pin_din);
            out[8]  = c.role == i2s::IBusConfig::Role::Slave ? 1 : 0;
            out[9]  = detail::sizeToUnit(c.tx_buffer_size, 1024);
            out[10] = detail::sizeToUnit(c.rx_buffer_size, 1024);
            return size_t{11};
        }
        default:
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
}

result_t<size_t> RemoteBackend::extractPinConfigFromLogical(types::bus_kind_t kind, const void* config, uint8_t* out,
                                                            size_t out_size)
{
    switch (kind) {
        case types::bus_kind_t::I2C: {
            if (out_size < 4) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            const auto& c = *static_cast<const i2c::LogicalBusConfig*>(config);
            detail::putI16LE(out, c.pin_scl);
            detail::putI16LE(out + 2, c.pin_sda);
            return size_t{4};
        }
        case types::bus_kind_t::SPI: {
            if (out_size < 6) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            const auto& c = *static_cast<const spi::LogicalBusConfig*>(config);
            detail::putI16LE(out, c.pin_clk);
            detail::putI16LE(out + 2, c.pin_mosi);
            detail::putI16LE(out + 4, c.pin_miso);
            return size_t{6};
        }
        case types::bus_kind_t::UART: {
            if (out_size < 7) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            const auto& c = *static_cast<const uart::LogicalBusConfig*>(config);
            detail::putI16LE(out, c.pin_tx);
            detail::putI16LE(out + 2, c.pin_rx);
            out[4] = 0;
            out[5] = 1;
            out[6] = 0;
            return size_t{7};
        }
        case types::bus_kind_t::I2S: {
            if (out_size < 11) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            const auto& c = *static_cast<const i2s::LogicalBusConfig*>(config);
            detail::putI16LE(out, c.pin_bclk);
            detail::putI16LE(out + 2, c.pin_ws);
            detail::putI16LE(out + 4, c.pin_dout);
            detail::putI16LE(out + 6, c.pin_din);
            out[8]  = 0;
            out[9]  = detail::sizeToUnit(8192, 1024);
            out[10] = detail::sizeToUnit(8192, 1024);
            return size_t{11};
        }
        default:
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
}

uint8_t RemoteBackend::allocateBusId(types::bus_kind_t kind)
{
    return _bus_ids ? _bus_ids->reserve(kind) : 0xFF;
}

void RemoteBackend::freeBusId(types::bus_kind_t kind, uint8_t bus_id)
{
    if (_bus_ids) {
        _bus_ids->release(kind, bus_id);
    }
}

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_BACKEND_INL_
