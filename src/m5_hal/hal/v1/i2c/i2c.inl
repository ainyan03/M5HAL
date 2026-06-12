// SPDX-License-Identifier: MIT

#include "i2c.hpp"
#include "../error.hpp"

namespace m5::hal::v1::i2c {

MasterAccessor::MasterAccessor(IBus& bus, const MasterAccessConfig& access_config)
    : bus::IAccessor{bus}, _access_config{access_config}
{
}

IBus& MasterAccessor::getBus(void) const
{
    // `_bus` was upcast from IBus& in the ctor, so the static_cast back is safe.
    return static_cast<IBus&>(*_bus);
}

m5::hal::v1::result_t<void> MasterAccessor::setConfig(const MasterAccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    _access_config = cfg;
    return {};
}

m5::hal::v1::result_t<size_t> MasterAccessor::transfer(const TransferDesc& desc, data::ConstDataSpan tx_bytes,
                                                       data::DataSpan rx_bytes)
{
    data::MemorySource tx_src{tx_bytes};
    data::MemorySink rx_dst{rx_bytes};
    return transfer(desc, (tx_bytes.size > 0) ? &tx_src : nullptr, (rx_bytes.size > 0) ? &rx_dst : nullptr);
}

m5::hal::v1::result_t<size_t> MasterAccessor::transfer(const TransferDesc& desc, data::Source* tx, data::Sink* rx)
{
    // Wrap the body in beginAccess / endAccess for mutual exclusion.
    // When the outer caller already holds a `ScopedAccess` or an
    // explicit `beginAccess`, the depth counter prevents a double lock.
    // Release-error policy: bus::guarded.
    return bus::guarded([&] { return beginAccess(); },
                        [&] { return getBus().transfer(this, _access_config, desc, tx, rx); },
                        [&] { return endAccess(); });
}

m5::hal::v1::result_t<size_t> MasterAccessor::write(data::ConstDataSpan tx_bytes)
{
    return transfer(TransferDesc{}, tx_bytes, data::DataSpan{});
}

m5::hal::v1::result_t<size_t> MasterAccessor::read(data::DataSpan rx_bytes)
{
    return transfer(TransferDesc{}, data::ConstDataSpan{}, rx_bytes);
}

m5::hal::v1::result_t<size_t> MasterAccessor::write(const uint8_t* tx, size_t len)
{
    return write(data::ConstDataSpan{tx, len});
}

m5::hal::v1::result_t<size_t> MasterAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

m5::hal::v1::result_t<void> MasterAccessor::probe(void)
{
    auto r = transfer(TransferDesc{}, data::ConstDataSpan{}, data::DataSpan{});
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return {};
}

m5::hal::v1::result_t<void> IBus::probe(uint16_t i2c_addr, uint32_t freq, uint32_t timeout_ms)
{
    MasterAccessConfig cfg;
    cfg.i2c_addr = i2c_addr;
    cfg.freq     = freq;
    // Keep scans fast on stuck buses: the probe budget also bounds the
    // wire-level wait, not just the lock acquisition below.
    cfg.wire_timeout_ms = timeout_ms;
    MasterAccessor sentinel{*this, cfg};
    return bus::guarded([&] { return sentinel.beginAccess(timeout_ms); }, [&] { return sentinel.probe(); },
                        [&] { return sentinel.endAccess(); });
}

m5::hal::v1::result_t<size_t> IBus::transfer(bus::IAccessor* owner, const MasterAccessConfig& cfg,
                                             const TransferDesc& desc, data::Source* tx, data::Sink* rx)
{
    (void)owner;
    (void)cfg;
    (void)desc;
    (void)tx;
    (void)rx;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

}  // namespace m5::hal::v1::i2c
