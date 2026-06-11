#ifndef M5_HAL_I2S_I2S_INL_
#define M5_HAL_I2S_I2S_INL_

#include "i2s.hpp"

namespace m5::hal::v1::i2s {

I2STxAccessor::I2STxAccessor(I2SBus& bus, const I2SAccessConfig& access_config)
    : bus::Accessor{bus}, _access_config{access_config}
{
}

I2SBus& I2STxAccessor::getI2SBus(void) const
{
    return static_cast<I2SBus&>(getBus());
}

m5::stl::expected<void, error::error_t> I2STxAccessor::setConfig(const I2SAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

m5::stl::expected<size_t, error::error_t> I2STxAccessor::write(data::ConstDataSpan tx_bytes)
{
    data::MemorySource src{tx_bytes};
    return write(src, tx_bytes.size);
}

m5::stl::expected<size_t, error::error_t> I2STxAccessor::write(data::Source& tx, size_t len)
{
    auto ba = beginAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }
    auto result = getI2SBus().write(this, _access_config, &tx, len);
    auto ea     = endAccess();
    if (!result.has_value()) {
        return result;
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return result;
}

m5::stl::expected<size_t, error::error_t> I2STxAccessor::write(const uint8_t* tx, size_t len)
{
    return write(data::ConstDataSpan{tx, len});
}

m5::stl::expected<size_t, error::error_t> I2STxAccessor::writableBytes(void)
{
    auto ba = beginAccess(_access_config.timeout_ms);
    if (!ba.has_value()) {
        return m5::stl::make_unexpected(ba.error());
    }
    auto result = getI2SBus().writableBytes(this, _access_config);
    auto ea     = endAccess();
    if (!result.has_value()) {
        return result;
    }
    if (!ea.has_value()) {
        return m5::stl::make_unexpected(ea.error());
    }
    return result;
}

m5::stl::expected<size_t, error::error_t> I2SBus::write(bus::Accessor* owner, const I2SAccessConfig& cfg,
                                                        data::Source* tx, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)tx;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

m5::stl::expected<size_t, error::error_t> I2SBus::writableBytes(bus::Accessor* owner, const I2SAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

}  // namespace m5::hal::v1::i2s

#endif
