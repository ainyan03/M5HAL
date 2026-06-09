
#include "i2c.hpp"
#include "../error.hpp"

namespace m5::hal::v1::i2c {

I2CMasterAccessor::I2CMasterAccessor(I2CBus& bus, const I2CMasterAccessConfig& access_config)
    : bus::Accessor{bus}, _access_config{access_config}
{
}

I2CBus& I2CMasterAccessor::getI2CBus(void) const
{
    // `_bus` was upcast from I2CBus& in the ctor, so the static_cast back is safe.
    return static_cast<I2CBus&>(_bus);
}

m5::stl::expected<void, m5::hal::v1::error::error_t> I2CMasterAccessor::setConfig(const I2CMasterAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> I2CMasterAccessor::transfer(const TransferDesc& desc,
                                                                                   data::ConstDataSpan tx_bytes,
                                                                                   data::DataSpan rx_bytes)
{
    // Wrap the body in beginAccess / endAccess for mutual exclusion.
    // When the outer caller already holds a `ScopedAccess` or an
    // explicit `beginAccess`, the depth counter prevents a double lock.
    auto ba = beginAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }
    data::MemorySource tx_src{tx_bytes};
    data::MemorySink rx_dst{rx_bytes};
    auto result = getI2CBus().transfer(this, _access_config, desc, (tx_bytes.size > 0) ? &tx_src : nullptr,
                                       (rx_bytes.size > 0) ? &rx_dst : nullptr);
    (void)endAccess();
    return result;
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> I2CMasterAccessor::write(data::ConstDataSpan tx_bytes)
{
    return transfer(TransferDesc{}, tx_bytes, data::DataSpan{});
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> I2CMasterAccessor::read(data::DataSpan rx_bytes)
{
    return transfer(TransferDesc{}, data::ConstDataSpan{}, rx_bytes);
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> I2CMasterAccessor::write(const uint8_t* tx, size_t len)
{
    return write(data::ConstDataSpan{tx, len});
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> I2CMasterAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

m5::stl::expected<void, m5::hal::v1::error::error_t> I2CMasterAccessor::probe(void)
{
    auto r = transfer(TransferDesc{}, data::ConstDataSpan{}, data::DataSpan{});
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return {};
}

m5::stl::expected<void, m5::hal::v1::error::error_t> I2CBus::probe(uint16_t i2c_addr, uint32_t freq,
                                                                   uint32_t timeout_ms)
{
    I2CMasterAccessConfig cfg;
    cfg.i2c_addr   = i2c_addr;
    cfg.freq       = freq;
    cfg.timeout_ms = timeout_ms;
    I2CMasterAccessor sentinel{*this, cfg};
    return sentinel.probe();
}

m5::stl::expected<size_t, m5::hal::v1::error::error_t> I2CBus::transfer(bus::Accessor* owner,
                                                                        const I2CMasterAccessConfig& cfg,
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

}  // namespace m5::hal::v1::i2c
