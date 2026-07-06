// SPDX-License-Identifier: MIT

#include "spi.hpp"
#include "../data/memory.hpp"
#include "../error.hpp"
#include "../m5_hal.hpp"

namespace m5::hal::v2::spi {

namespace {

void initIdlePin(types::gpio_number_t pin)
{
    if (pin >= 0) {
        auto p = M5_Hal.Gpio.getPin(pin);
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

bus::TransferTotals diffTotals(const bus::TransferTotals& after, const bus::TransferTotals& before)
{
    return bus::TransferTotals{after.tx - before.tx, after.rx - before.rx};
}

}  // namespace

MasterAccessor::MasterAccessor(IBus& bus, const MasterAccessConfig& access_config)
    : bus::IAccessor{bus}, _access_config{access_config}
{
    initIdlePin(access_config.pin_cs);
}

MasterAccessor::MasterAccessor(std::shared_ptr<IBus> bus, const MasterAccessConfig& access_config)
    : bus::IAccessor{std::move(bus)}, _access_config{access_config}
{
    initIdlePin(access_config.pin_cs);
}

IBus& MasterAccessor::getBus(void) const
{
    return static_cast<IBus&>(*_bus);
}

bool MasterAccessor::transferBusy(void)
{
    return getBus().transferBusy(this);
}

m5::hal::v2::result_t<void> MasterAccessor::waitTransfer(void)
{
    auto waited = getBus().waitTransfer(this, _access_config);
    if (!waited.has_value()) {
        const auto err     = waited.error();
        _transaction_error = err;
        return m5::stl::make_unexpected(err);
    }
    _transaction_totals.add(waited.value());
    return {};
}

m5::hal::v2::result_t<void> MasterAccessor::setConfig(const MasterAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    if (cfg.pin_cs != _access_config.pin_cs) {
        initIdlePin(cfg.pin_cs);
    }
    _access_config = cfg;
    return {};
}

m5::hal::v2::result_t<void> MasterAccessor::transfer(const TransferDesc& desc, data::ConstDataSpan src_bytes,
                                                     data::DataSpan dst_bytes)
{
    if (!inTransaction()) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    if (error::isError(_transaction_error)) {
        return m5::stl::make_unexpected(_transaction_error);
    }
    auto waited = waitTransfer();
    if (!waited.has_value()) {
        return m5::stl::make_unexpected(waited.error());
    }
    _span_src = data::MemorySource{src_bytes};
    _span_dst = data::MemorySink{dst_bytes};
    return startTransfer(desc, src_bytes.size ? &_span_src : nullptr, src_bytes.size,
                         dst_bytes.size ? &_span_dst : nullptr, dst_bytes.size);
}

m5::hal::v2::result_t<void> MasterAccessor::transfer(const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                     data::Sink* dst, size_t rx_len)
{
    if (!inTransaction()) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    if (error::isError(_transaction_error)) {
        return m5::stl::make_unexpected(_transaction_error);
    }

    auto waited = waitTransfer();
    if (!waited.has_value()) {
        return m5::stl::make_unexpected(waited.error());
    }

    return startTransfer(desc, src, tx_len, dst, rx_len);
}

m5::hal::v2::result_t<void> MasterAccessor::startTransfer(const TransferDesc& desc, data::Source* src, size_t tx_len,
                                                          data::Sink* dst, size_t rx_len)
{
    auto r = getBus().transfer(this, _access_config, desc, (src != nullptr && tx_len > 0) ? src : nullptr, tx_len,
                               (dst != nullptr && rx_len > 0) ? dst : nullptr, rx_len);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    if (!transferBusy()) {
        auto done = waitTransfer();
        if (!done.has_value()) {
            return m5::stl::make_unexpected(done.error());
        }
    }
    return {};
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
    auto b = beginTransaction();
    if (!b.has_value()) {
        return m5::stl::make_unexpected(b.error());
    }
    const auto before = _transaction_totals;
    auto t            = transfer(desc, src, tx_len, dst, rx_len);
    auto e            = endTransaction();
    if (!t.has_value()) {
        return m5::stl::make_unexpected(t.error());
    }
    if (!e.has_value()) {
        return m5::stl::make_unexpected(e.error());
    }
    return diffTotals(e.value(), before);
}

m5::hal::v2::result_t<void> MasterAccessor::beginTransaction(void)
{
    if (_transaction_depth != 0) {
        ++_transaction_depth;
        return {};
    }

    auto ba = beginAccess();
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }

    auto bt = getBus().beginTransaction(this, _access_config);
    if (!bt.has_value()) {
        (void)endAccess();  // rollback; the primary (bt) error takes priority
        return m5::stl::make_unexpected(bt.error());
    }

    _transaction_totals.clear();
    _transaction_error = error::error_t::OK;
    _transaction_depth = 1;
    return {};
}

m5::hal::v2::result_t<bus::TransferTotals> MasterAccessor::endTransaction(void)
{
    if (_transaction_depth == 0) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }

    auto wait                    = waitTransfer();
    error::error_t primary_error = error::error_t::OK;
    if (!wait.has_value()) {
        primary_error = wait.error();
    } else if (error::isError(_transaction_error)) {
        primary_error = _transaction_error;
    }

    --_transaction_depth;
    if (_transaction_depth != 0) {
        if (error::isError(primary_error)) {
            return m5::stl::make_unexpected(primary_error);
        }
        return _transaction_totals;
    }

    const auto totals = _transaction_totals;
    auto et           = getBus().endTransaction(this, _access_config);
    auto ea           = endAccess();
    _transaction_totals.clear();
    _transaction_error = error::error_t::OK;
    if (error::isError(primary_error)) {
        return m5::stl::make_unexpected(primary_error);
    }
    if (!et.has_value()) {
        return m5::stl::make_unexpected(et.error());
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return totals;
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
    auto desc = commandDesc(_access_config, command);
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
    auto desc = commandAddressDesc(_access_config, command, address, 0);
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
    const size_t command_bytes = transferBytesForBits(_access_config.spi_command_length);
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
    desc.dummy_cycles     = _access_config.spi_write_dummy_cycle;
    auto result           = transferSync(
        desc, data::ConstDataSpan{src_bytes.data + command_bytes, src_bytes.size - command_bytes}, data::DataSpan{});
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommandData(uint32_t command, data::ConstDataSpan src_bytes)
{
    auto desc = commandDesc(_access_config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _access_config.spi_write_dummy_cycle;
    auto result         = transferSync(desc.value(), src_bytes, data::DataSpan{});
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommandData(uint32_t command, data::Source& src, size_t len)
{
    auto desc = commandDesc(_access_config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _access_config.spi_write_dummy_cycle;
    auto result         = transferSync(desc.value(), &src, len, nullptr, 0);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->tx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::writeCommandAddressData(uint32_t command, uint32_t address,
                                                                      data::ConstDataSpan src_bytes)
{
    auto desc = commandAddressDesc(_access_config, command, address, _access_config.spi_write_dummy_cycle);
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
    auto desc = commandAddressDesc(_access_config, command, address, _access_config.spi_write_dummy_cycle);
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
    auto desc = commandDesc(_access_config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _access_config.spi_read_dummy_cycle;
    auto result         = transferSync(desc.value(), data::ConstDataSpan{}, dst_bytes);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::readCommandData(uint32_t command, data::Sink& dst, size_t len)
{
    auto desc = commandDesc(_access_config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _access_config.spi_read_dummy_cycle;
    auto result         = transferSync(desc.value(), nullptr, 0, &dst, len);
    if (!result.has_value()) {
        return m5::stl::make_unexpected(result.error());
    }
    return result->rx;
}

m5::hal::v2::result_t<size_t> MasterAccessor::readCommandAddressData(uint32_t command, uint32_t address,
                                                                     data::DataSpan dst_bytes)
{
    auto desc = commandAddressDesc(_access_config, command, address, _access_config.spi_read_dummy_cycle);
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
    auto desc = commandAddressDesc(_access_config, command, address, _access_config.spi_read_dummy_cycle);
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

m5::hal::v2::result_t<void> IBus::transfer(bus::IAccessor* owner, const MasterAccessConfig& cfg,
                                           const TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                           size_t rx_len)
{
    (void)owner;
    (void)cfg;
    (void)desc;
    (void)src;
    (void)tx_len;
    (void)dst;
    (void)rx_len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

m5::hal::v2::result_t<void> IBus::beginTransaction(bus::IAccessor* owner, const MasterAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return {};
}

m5::hal::v2::result_t<void> IBus::endTransaction(bus::IAccessor* owner, const MasterAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return {};
}

m5::hal::v2::result_t<bus::TransferTotals> IBus::waitTransfer(bus::IAccessor* owner, const MasterAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return bus::TransferTotals{};
}

bool IBus::transferBusy(bus::IAccessor* owner)
{
    (void)owner;
    return false;
}

}  // namespace m5::hal::v2::spi
