
#include "spi.hpp"
#include "../data/limited.hpp"
#include "../data/memory.hpp"
#include "../error.hpp"

namespace m5::hal::v1::spi {

namespace {

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

m5::stl::expected<TransferDesc, m5::hal::v1::error::error_t> commandDesc(const SPIMasterAccessConfig& cfg,
                                                                         uint32_t command)
{
    const uint8_t command_bytes = transferBytesForBits(cfg.spi_command_length);
    if (command_bytes == 0 || command_bytes > 4) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    TransferDesc desc;
    desc.dc_level_valid   = true;
    desc.dc_level         = true;
    desc.command          = command;
    desc.command_bytes    = command_bytes;
    desc.command_dc_level = 0;
    return desc;
}

m5::stl::expected<TransferDesc, m5::hal::v1::error::error_t> commandAddressDesc(const SPIMasterAccessConfig& cfg,
                                                                                uint32_t command, uint32_t address,
                                                                                uint8_t dummy_cycles)
{
    auto desc = commandDesc(cfg, command);
    if (!desc.has_value()) {
        return desc;
    }

    const uint8_t address_bytes = transferBytesForBits(cfg.spi_address_length);
    if (address_bytes == 0 || address_bytes > 4) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    desc->address          = address;
    desc->address_bytes    = address_bytes;
    desc->address_dc_level = 1;
    desc->dummy_cycles     = dummy_cycles;
    return desc;
}

}  // namespace

SPIMasterAccessor::SPIMasterAccessor(SPIBus& bus, const SPIMasterAccessConfig& access_config)
    : bus::Accessor{bus}, _access_config{access_config}
{
}

SPIBus& SPIMasterAccessor::getSPIBus(void) const
{
    return static_cast<SPIBus&>(_bus);
}

m5::stl::expected<void, m5::hal::v1::error::error_t> SPIMasterAccessor::setConfig(const SPIMasterAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::transfer(const TransferDesc& desc,
                                                                                   data::ConstDataSpan tx_bytes,
                                                                                   data::DataSpan rx_bytes)
{
    data::MemorySource tx_src{tx_bytes};
    data::MemorySink rx_dst{rx_bytes};
    const size_t len = (tx_bytes.size > rx_bytes.size) ? tx_bytes.size : rx_bytes.size;
    return transfer(desc, (tx_bytes.size > 0) ? &tx_src : nullptr, (rx_bytes.size > 0) ? &rx_dst : nullptr, len);
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::transfer(const TransferDesc& desc,
                                                                                   data::Source* tx, data::Sink* rx,
                                                                                   size_t len)
{
    auto bt = beginTransaction();
    if (!bt.has_value()) {
        return m5::stl::make_unexpected(bt.error());
    }

    data::LimitedSource limited_tx{tx, len};
    data::LimitedSink limited_rx{rx, len};
    auto result = getSPIBus().transfer(this, _access_config, desc, (tx != nullptr && len > 0) ? &limited_tx : nullptr,
                                       (rx != nullptr && len > 0) ? &limited_rx : nullptr);
    auto et     = endTransaction();
    if (!result.has_value()) {
        return result;
    }
    if (!et.has_value()) {
        return m5::stl::make_unexpected(et.error());
    }
    return result;
}

m5::stl::expected<void, m5::hal::v1::error::error_t> SPIMasterAccessor::beginTransaction(void)
{
    if (_transaction_depth != 0) {
        ++_transaction_depth;
        return {};
    }

    auto ba = beginAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }

    auto bt = getSPIBus().beginTransaction(this, _access_config);
    if (!bt.has_value()) {
        (void)endAccess();  // rollback; the primary (bt) error takes priority
        return m5::stl::make_unexpected(bt.error());
    }

    _transaction_depth = 1;
    return {};
}

m5::stl::expected<void, m5::hal::v1::error::error_t> SPIMasterAccessor::endTransaction(void)
{
    if (_transaction_depth == 0) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    --_transaction_depth;
    if (_transaction_depth != 0) {
        return {};
    }

    auto et = getSPIBus().endTransaction(this, _access_config);
    auto ea = endAccess();
    if (!et.has_value()) {
        return m5::stl::make_unexpected(et.error());
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return {};
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::write(data::ConstDataSpan tx_bytes)
{
    return transfer(TransferDesc{}, tx_bytes, data::DataSpan{});
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::write(data::Source& tx, size_t len)
{
    return transfer(TransferDesc{}, &tx, nullptr, len);
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::read(data::DataSpan rx_bytes)
{
    return transfer(TransferDesc{}, data::ConstDataSpan{}, rx_bytes);
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::read(data::Sink& rx, size_t len)
{
    return transfer(TransferDesc{}, nullptr, &rx, len);
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::write(const uint8_t* tx, size_t len)
{
    return write(data::ConstDataSpan{tx, len});
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::writeCommand(data::ConstDataSpan tx_bytes)
{
    TransferDesc desc;
    desc.dc_level_valid = true;
    desc.dc_level       = false;
    desc.data_dc_level  = 0;
    return transfer(desc, tx_bytes, data::DataSpan{});
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::writeCommand(uint32_t command)
{
    auto desc = commandDesc(_access_config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    auto result = transfer(desc.value(), data::ConstDataSpan{}, data::DataSpan{});
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::writeCommandAddress(uint32_t command,
                                                                                              uint32_t address)
{
    auto desc = commandAddressDesc(_access_config, command, address, 0);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    auto result = transfer(desc.value(), data::ConstDataSpan{}, data::DataSpan{});
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes + desc->address_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::writeCommandData(data::ConstDataSpan tx_bytes)
{
    const size_t command_bytes = transferBytesForBits(_access_config.spi_command_length);
    // spi_command_length == 0 is an error: writeCommandData always requires a
    // command phase. Callers that want a plain write should call write()
    // directly. Silently falling through to write() here would make
    // spi_command_length optional, contradicting the invariant that the
    // command-sugar family mandates a configured command length.
    if (command_bytes == 0) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    if (command_bytes > 4) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    if (command_bytes >= tx_bytes.size) {
        return writeCommand(tx_bytes);
    }

    TransferDesc desc;
    desc.dc_level_valid   = true;
    desc.dc_level         = true;
    desc.command          = composeBigEndian(data::ConstDataSpan{tx_bytes.data, command_bytes});
    desc.command_bytes    = static_cast<uint8_t>(command_bytes);
    desc.command_dc_level = 0;
    desc.data_dc_level    = 1;
    desc.dummy_cycles     = _access_config.spi_write_dummy_cycle;
    auto result = transfer(desc, data::ConstDataSpan{tx_bytes.data + command_bytes, tx_bytes.size - command_bytes},
                           data::DataSpan{});
    if (!result.has_value()) {
        return result;
    }
    return result.value() + command_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::writeCommandData(uint32_t command,
                                                                                           data::ConstDataSpan tx_bytes)
{
    auto desc = commandDesc(_access_config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _access_config.spi_write_dummy_cycle;
    auto result         = transfer(desc.value(), tx_bytes, data::DataSpan{});
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::writeCommandData(uint32_t command,
                                                                                           data::Source& tx, size_t len)
{
    auto desc = commandDesc(_access_config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _access_config.spi_write_dummy_cycle;
    auto result         = transfer(desc.value(), &tx, nullptr, len);
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::writeCommandAddressData(
    uint32_t command, uint32_t address, data::ConstDataSpan tx_bytes)
{
    auto desc = commandAddressDesc(_access_config, command, address, _access_config.spi_write_dummy_cycle);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    auto result         = transfer(desc.value(), tx_bytes, data::DataSpan{});
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes + desc->address_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::writeCommandAddressData(uint32_t command,
                                                                                                  uint32_t address,
                                                                                                  data::Source& tx,
                                                                                                  size_t len)
{
    auto desc = commandAddressDesc(_access_config, command, address, _access_config.spi_write_dummy_cycle);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    auto result         = transfer(desc.value(), &tx, nullptr, len);
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes + desc->address_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::readCommandData(uint32_t command,
                                                                                          data::DataSpan rx_bytes)
{
    auto desc = commandDesc(_access_config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _access_config.spi_read_dummy_cycle;
    auto result         = transfer(desc.value(), data::ConstDataSpan{}, rx_bytes);
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::readCommandData(uint32_t command,
                                                                                          data::Sink& rx, size_t len)
{
    auto desc = commandDesc(_access_config, command);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    desc->dummy_cycles  = _access_config.spi_read_dummy_cycle;
    auto result         = transfer(desc.value(), nullptr, &rx, len);
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::readCommandAddressData(
    uint32_t command, uint32_t address, data::DataSpan rx_bytes)
{
    auto desc = commandAddressDesc(_access_config, command, address, _access_config.spi_read_dummy_cycle);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    auto result         = transfer(desc.value(), data::ConstDataSpan{}, rx_bytes);
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes + desc->address_bytes;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIMasterAccessor::readCommandAddressData(uint32_t command,
                                                                                                 uint32_t address,
                                                                                                 data::Sink& rx,
                                                                                                 size_t len)
{
    auto desc = commandAddressDesc(_access_config, command, address, _access_config.spi_read_dummy_cycle);
    if (!desc.has_value()) {
        return m5::stl::make_unexpected(desc.error());
    }
    desc->data_dc_level = 1;
    auto result         = transfer(desc.value(), nullptr, &rx, len);
    if (!result.has_value()) {
        return result;
    }
    return result.value() + desc->command_bytes + desc->address_bytes;
}

m5::stl::expected<void, m5::hal::v1::error::error_t> SPIMasterAccessor::sendDummyClock(size_t count)
{
    if (count > 255) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    TransferDesc desc;
    desc.dummy_cycles = static_cast<uint8_t>(count);
    auto r            = transfer(desc, data::ConstDataSpan{}, data::DataSpan{});
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return {};
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> SPIBus::transfer(bus::Accessor* owner,
                                                                        const SPIMasterAccessConfig& cfg,
                                                                        const TransferDesc& desc, data::Source* tx,
                                                                        data::Sink* rx)
{
    (void)owner;
    (void)cfg;
    (void)desc;
    (void)tx;
    (void)rx;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

m5::stl::expected<void, m5::hal::v1::error::error_t> SPIBus::beginTransaction(bus::Accessor* owner,
                                                                              const SPIMasterAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return {};
}

m5::stl::expected<void, m5::hal::v1::error::error_t> SPIBus::endTransaction(bus::Accessor* owner,
                                                                            const SPIMasterAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return {};
}

}  // namespace m5::hal::v1::spi
