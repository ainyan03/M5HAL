// SPDX-License-Identifier: MIT

#include "spi.hpp"
#include "../data/memory.hpp"
#include "../error.hpp"

namespace m5::hal::v2::spi {

namespace {

void initIdlePin(IBus& bus, types::gpio_number_t pin)
{
    auto* gpio = bus.localResourceContext().gpio;
    if (pin >= 0 && gpio != nullptr) {
        auto p = gpio->getPin(pin);
        if (p.isValid()) {
            p.setMode(types::gpio_mode_t::Output);
            p.writeHigh();
        }
    }
}

uint8_t transferBytesForBits(uint8_t bits)
{
    return (bits == 0) ? 0 : static_cast<uint8_t>((bits + 7u) >> 3);
}

uint32_t composeBigEndian(data::ConstDataSpan bytes)
{
    uint32_t value = 0;
    for (size_t i = 0; i < bytes.size && i < 4; ++i) {
        value = static_cast<uint32_t>((value << 8) | bytes.data[i]);
    }
    return value;
}

m5::hal::v2::result_t<TransferDesc> commandDesc(const MasterAccessConfig& cfg, uint32_t command)
{
    const uint8_t command_bytes = transferBytesForBits(cfg.spi_command_length);
    if (command_bytes == 0 || command_bytes > 4) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }

    TransferDesc desc;
    desc.dc_level_valid   = true;
    desc.dc_level         = true;
    desc.command          = command;
    desc.command_bytes    = command_bytes;
    desc.command_dc_level = 0;
    return desc;
}

m5::hal::v2::result_t<TransferDesc> commandAddressDesc(const MasterAccessConfig& cfg, uint32_t command,
                                                       uint32_t address, uint8_t dummy_cycles)
{
    auto desc = commandDesc(cfg, command);
    if (!desc.has_value()) {
        return desc;
    }

    const uint8_t address_bytes = transferBytesForBits(cfg.spi_address_length);
    if (address_bytes == 0 || address_bytes > 4) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    desc->address          = address;
    desc->address_bytes    = address_bytes;
    desc->address_dc_level = 1;
    desc->dummy_cycles     = dummy_cycles;
    return desc;
}

}  // namespace

MasterAccessor::MasterAccessor(IBus& bus, const MasterAccessConfig& access_config)
    : bus::IAccessor{bus}, _context{makeOperationContext(access_config)}
{
    initIdlePin(bus, access_config.pin_cs);
}

MasterAccessor::MasterAccessor(std::shared_ptr<IBus> bus, const MasterAccessConfig& access_config)
    : bus::IAccessor{std::move(bus)}, _context{makeOperationContext(access_config)}
{
    initIdlePin(getBus(), access_config.pin_cs);
}

IBus& MasterAccessor::getBus(void) const
{
    return static_cast<IBus&>(*_bus);
}

bool MasterAccessor::transferBusy(void)
{
    return getBus().transferBusy(_context);
}

m5::hal::v2::result_t<void> MasterAccessor::setConfig(const MasterAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    if (cfg.pin_cs != _context.config.pin_cs) {
        initIdlePin(getBus(), cfg.pin_cs);
    }
    _context.config = cfg;
    return {};
}

m5::hal::v2::result_t<void> MasterAccessor::beginAccess(uint32_t timeout_ms)
{
    return _beginOperationAccess(_context, timeout_ms, bus::OperationMode::TxRx,
                                 [&](auto& context) { return getBus().beginOperation(context); });
}

m5::hal::v2::result_t<void> MasterAccessor::endAccess(uint32_t timeout_ms)
{
    return _endOperationAccess(_context, timeout_ms, [&](auto& context) {
        error::error_t wait_error = error::error_t::OK;
        if (getBus().transferBusy(context)) {
            auto waited = getBus().waitTransfer(context);
            if (!waited.has_value()) {
                wait_error = waited.error();
            }
        }
        auto ended = getBus().endOperation(context);
        if (error::isError(wait_error)) {
            return result_t<void>{m5::stl::make_unexpected(wait_error)};
        }
        return ended;
    });
}

m5::hal::v2::result_t<bus::TransferStatus> MasterAccessor::getLastTransferStatus(void) const
{
    if (_last_transfer_status.transfer_id == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return _last_transfer_status;
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::transfer(const TransferDesc& desc,
                                                                    data::ConstDataSpan src_bytes,
                                                                    data::DataSpan dst_bytes)
{
    if (!_inOperationAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _span_src = data::MemorySource{src_bytes};
    _span_dst = data::MemorySink{dst_bytes};
    return startTransfer(desc, src_bytes.size ? &_span_src : nullptr, src_bytes.size,
                         dst_bytes.size ? &_span_dst : nullptr, dst_bytes.size);
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::transfer(const TransferDesc& desc, data::Source* src,
                                                                    size_t tx_len, data::Sink* dst, size_t rx_len)
{
    if (!_inOperationAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return startTransfer(desc, src, tx_len, dst, rx_len);
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::startTransfer(const TransferDesc& desc, data::Source* src,
                                                                         size_t tx_len, data::Sink* dst, size_t rx_len)
{
    ++_next_transfer_id;
    if (_next_transfer_id == 0) {
        ++_next_transfer_id;
    }
    _last_transfer_status             = {};
    _last_transfer_status.transfer_id = _next_transfer_id;

    if (getBus().transferBusy(_context)) {
        auto previous = getBus().waitTransfer(_context);
        if (!previous.has_value()) {
            _last_transfer_status.error      = previous.error();
            _last_transfer_status.completion = bus::CompletionLevel::Aborted;
            return m5::stl::make_unexpected(previous.error());
        }
    }

    auto started = getBus().transfer(_context, desc, (src != nullptr && tx_len > 0) ? src : nullptr, tx_len,
                                     (dst != nullptr && rx_len > 0) ? dst : nullptr, rx_len);
    if (!started.has_value()) {
        _last_transfer_status.error = started.error();
        return m5::stl::make_unexpected(started.error());
    }
    _last_transfer_status.completion = bus::CompletionLevel::Accepted;

    auto waited = getBus().waitTransfer(_context);
    if (!waited.has_value()) {
        _last_transfer_status.error      = waited.error();
        _last_transfer_status.completion = bus::CompletionLevel::Aborted;
        return m5::stl::make_unexpected(waited.error());
    }
    _last_transfer_status.totals     = waited.value();
    _last_transfer_status.completion = bus::CompletionLevel::Complete;
    return waited.value();
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::transferSync(const TransferDesc& desc,
                                                                        data::ConstDataSpan src_bytes,
                                                                        data::DataSpan dst_bytes)
{
    data::MemorySource tx_src{src_bytes};
    data::MemorySink rx_dst{dst_bytes};
    return transferSync(desc, (src_bytes.size > 0) ? &tx_src : nullptr, src_bytes.size,
                        (dst_bytes.size > 0) ? &rx_dst : nullptr, dst_bytes.size);
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::transferSync(const TransferDesc& desc, data::Source* src,
                                                                        size_t tx_len, data::Sink* dst, size_t rx_len)
{
    const bool borrowed = _inOperationAccess();
    if (!borrowed) {
        auto begun = beginAccess();
        if (!begun.has_value()) {
            return m5::stl::make_unexpected(begun.error());
        }
    }
    auto transferred = transfer(desc, src, tx_len, dst, rx_len);
    result_t<void> ended{};
    if (!borrowed) {
        ended = endAccess();
    }
    if (!transferred.has_value()) {
        return m5::stl::make_unexpected(transferred.error());
    }
    if (!ended.has_value()) {
        return m5::stl::make_unexpected(ended.error());
    }
    return transferred.value();
}

m5::hal::v2::result_t<size_t> MasterAccessor::write(data::ConstDataSpan src_bytes)
{
    auto r = transferSync(TransferDesc{}, src_bytes, data::DataSpan{});
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::write(data::Source& src, size_t len)
{
    auto r = transferSync(TransferDesc{}, &src, len, nullptr, 0);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::read(data::DataSpan dst_bytes)
{
    auto r = transferSync(TransferDesc{}, data::ConstDataSpan{}, dst_bytes);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::read(data::Sink& dst, size_t len)
{
    auto r = transferSync(TransferDesc{}, nullptr, 0, &dst, len);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::write(const uint8_t* src, size_t len)
{
    return write(data::ConstDataSpan{src, len});
}

m5::hal::v2::result_t<size_t> MasterAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommand(data::ConstDataSpan src_bytes)
{
    TransferDesc desc;
    desc.dc_level_valid = true;
    desc.dc_level       = false;
    desc.data_dc_level  = 0;
    auto r              = transferSync(desc, src_bytes, data::DataSpan{});
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommand(uint32_t command)
{
    auto desc = commandDesc(_context.config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    auto result = transferSync(desc.value(), data::ConstDataSpan{}, data::DataSpan{});
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommandAddress(uint32_t command, uint32_t address)
{
    auto desc = commandAddressDesc(_context.config, command, address, 0);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    auto result = transferSync(desc.value(), data::ConstDataSpan{}, data::DataSpan{});
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommandData(data::ConstDataSpan src_bytes)
{
    const size_t command_bytes = transferBytesForBits(_context.config.spi_command_length);
    // spi_command_length == 0 is an error: writeCommandData always requires a
    // command phase. Callers that want a plain write should call write()
    // directly. Silently falling through to write() here would make
    // spi_command_length optional, contradicting the invariant that the
    // command-sugar family mandates a configured command length.
    if (command_bytes == 0) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    if (command_bytes > 4) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    if (command_bytes >= src_bytes.size) {
        return writeCommand(src_bytes);
    }

    TransferDesc desc;
    desc.dc_level_valid   = true;
    desc.dc_level         = true;
    desc.command          = composeBigEndian(data::ConstDataSpan{src_bytes.data, command_bytes});
    desc.command_bytes    = static_cast<uint8_t>(command_bytes);
    desc.command_dc_level = 0;
    desc.data_dc_level    = 1;
    desc.dummy_cycles     = _context.config.spi_write_dummy_cycle;
    auto result           = transferSync(
        desc, data::ConstDataSpan{src_bytes.data + command_bytes, src_bytes.size - command_bytes}, data::DataSpan{});
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommandData(uint32_t command, data::ConstDataSpan src_bytes)
{
    auto desc = commandDesc(_context.config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _context.config.spi_write_dummy_cycle;
    auto result         = transferSync(desc.value(), src_bytes, data::DataSpan{});
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommandData(uint32_t command, data::Source& src, size_t len)
{
    auto desc = commandDesc(_context.config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _context.config.spi_write_dummy_cycle;
    auto result         = transferSync(desc.value(), &src, len, nullptr, 0);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommandAddressData(uint32_t command, uint32_t address,
                                                                      data::ConstDataSpan src_bytes)
{
    auto desc = commandAddressDesc(_context.config, command, address, _context.config.spi_write_dummy_cycle);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    auto result         = transferSync(desc.value(), src_bytes, data::DataSpan{});
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommandAddressData(uint32_t command, uint32_t address,
                                                                      data::Source& src, size_t len)
{
    auto desc = commandAddressDesc(_context.config, command, address, _context.config.spi_write_dummy_cycle);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    auto result         = transferSync(desc.value(), &src, len, nullptr, 0);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::readCommandData(uint32_t command, data::DataSpan dst_bytes)
{
    auto desc = commandDesc(_context.config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _context.config.spi_read_dummy_cycle;
    auto result         = transferSync(desc.value(), data::ConstDataSpan{}, dst_bytes);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::readCommandData(uint32_t command, data::Sink& dst, size_t len)
{
    auto desc = commandDesc(_context.config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _context.config.spi_read_dummy_cycle;
    auto result         = transferSync(desc.value(), nullptr, 0, &dst, len);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::readCommandAddressData(uint32_t command, uint32_t address,
                                                                     data::DataSpan dst_bytes)
{
    auto desc = commandAddressDesc(_context.config, command, address, _context.config.spi_read_dummy_cycle);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    auto result         = transferSync(desc.value(), data::ConstDataSpan{}, dst_bytes);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::readCommandAddressData(uint32_t command, uint32_t address,
                                                                     data::Sink& dst, size_t len)
{
    auto desc = commandAddressDesc(_context.config, command, address, _context.config.spi_read_dummy_cycle);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    auto result         = transferSync(desc.value(), nullptr, 0, &dst, len);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::sendDummyClock(size_t count)
{
    if (count > 255) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    TransferDesc desc;
    desc.dummy_cycles = static_cast<uint8_t>(count);
    auto r            = transferSync(desc, data::ConstDataSpan{}, data::DataSpan{});
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return r->tx;
}

m5::hal::v2::result_t<void> IBus::beginOperation(bus::OperationContext<MasterAccessConfig>& context)
{
    auto registered = _operation_slot.registerContext(context, this, _lock_owner);
    if (!registered.has_value()) {
        return registered;
    }
    auto begun = beginOperationBackend(context);
    if (!begun.has_value()) {
        _operation_slot.invalidate(context);
    }
    return begun;
}

m5::hal::v2::result_t<void> IBus::endOperation(bus::OperationContext<MasterAccessConfig>& context)
{
    if (!_operation_slot.valid(context, this, _lock_owner)) {
        if (_operation_slot.registered(context, this)) {
            if (_operation_slot.restoreRegisteredRuntime(context, this, _lock_owner)) {
                (void)endOperationBackend(context);
            }
            _operation_slot.invalidate(context);
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto ended = endOperationBackend(context);
    _operation_slot.invalidate(context);
    return ended;
}

m5::hal::v2::result_t<void> IBus::transfer(bus::OperationContext<MasterAccessConfig>& context, const TransferDesc& desc,
                                           data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len)
{
    if (!_operation_slot.valid(context, this, _lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return transferBackend(context, desc, src, tx_len, dst, rx_len);
}

m5::hal::v2::result_t<bus::TransferTotals> IBus::waitTransfer(bus::OperationContext<MasterAccessConfig>& context)
{
    if (!_operation_slot.valid(context, this, _lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return waitTransferBackend(context);
}

bool IBus::transferBusy(bus::OperationContext<MasterAccessConfig>& context)
{
    return _operation_slot.valid(context, this, _lock_owner) && transferBusyBackend(context);
}

m5::hal::v2::result_t<void> IBus::beginOperationBackend(bus::OperationContext<MasterAccessConfig>& context)
{
    (void)context;
    return {};
}

m5::hal::v2::result_t<void> IBus::endOperationBackend(bus::OperationContext<MasterAccessConfig>& context)
{
    (void)context;
    return {};
}

m5::hal::v2::result_t<void> IBus::transferBackend(bus::OperationContext<MasterAccessConfig>& context,
                                                  const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                  data::Sink* dst, size_t rx_len)
{
    (void)context;
    (void)desc;
    (void)src;
    (void)tx_len;
    (void)dst;
    (void)rx_len;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

m5::hal::v2::result_t<bus::TransferTotals> IBus::waitTransferBackend(bus::OperationContext<MasterAccessConfig>& context)
{
    (void)context;
    return bus::TransferTotals{};
}

bool IBus::transferBusyBackend(bus::OperationContext<MasterAccessConfig>& context)
{
    (void)context;
    return false;
}

}  // namespace m5::hal::v2::spi
