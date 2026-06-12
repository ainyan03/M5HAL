// SPDX-License-Identifier: MIT
#ifndef M5_HAL_I2S_I2S_INL_
#define M5_HAL_I2S_I2S_INL_

#include "i2s.hpp"

namespace m5::hal::v1::i2s {

TxAccessor::TxAccessor(IBus& bus, const AccessConfig& access_config)
    : bus::IAccessor{bus}, _access_config{access_config}
{
}

IBus& TxAccessor::getBus(void) const
{
    return static_cast<IBus&>(bus::IAccessor::getBus());
}

result_t<void> TxAccessor::setConfig(const AccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

result_t<size_t> TxAccessor::write(data::ConstDataSpan tx_bytes)
{
    data::MemorySource src{tx_bytes};
    return write(src, tx_bytes.size);
}

result_t<size_t> TxAccessor::write(data::Source& tx, size_t len)
{
    // Release-error policy: bus::guarded.
    return bus::guarded([&] { return beginAccess(); }, [&] { return getBus().write(this, _access_config, &tx, len); },
                        [&] { return endAccess(); });
}

result_t<size_t> TxAccessor::write(const uint8_t* tx, size_t len)
{
    return write(data::ConstDataSpan{tx, len});
}

result_t<size_t> TxAccessor::writableBytes(void)
{
    return bus::guarded([&] { return beginAccess(); }, [&] { return getBus().writableBytes(this, _access_config); },
                        [&] { return endAccess(); });
}

result_t<size_t> IBus::write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* tx, size_t len)
{
    (void)owner;
    (void)cfg;
    (void)tx;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<size_t> IBus::writableBytes(bus::IAccessor* owner, const AccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

}  // namespace m5::hal::v1::i2s

#endif
