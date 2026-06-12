// SPDX-License-Identifier: MIT
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

result_t<void> I2STxAccessor::setConfig(const I2SAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

result_t<size_t> I2STxAccessor::write(data::ConstDataSpan tx_bytes)
{
    data::MemorySource src{tx_bytes};
    return write(src, tx_bytes.size);
}

result_t<size_t> I2STxAccessor::write(data::Source& tx, size_t len)
{
    // Release-error policy: bus::guarded.
    return bus::guarded([&] { return beginAccess(_access_config.timeout_ms); },
                        [&] { return getI2SBus().write(this, _access_config, &tx, len); }, [&] { return endAccess(); });
}

result_t<size_t> I2STxAccessor::write(const uint8_t* tx, size_t len)
{
    return write(data::ConstDataSpan{tx, len});
}

result_t<size_t> I2STxAccessor::writableBytes(void)
{
    return bus::guarded([&] { return beginAccess(_access_config.timeout_ms); },
                        [&] { return getI2SBus().writableBytes(this, _access_config); }, [&] { return endAccess(); });
}

result_t<size_t> I2SBus::write(bus::Accessor* owner, const I2SAccessConfig& cfg, data::Source* tx, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)tx;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<size_t> I2SBus::writableBytes(bus::Accessor* owner, const I2SAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

}  // namespace m5::hal::v1::i2s

#endif
