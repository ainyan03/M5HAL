// SPDX-License-Identifier: MIT
#ifndef M5_HAL_UART_BUS_STREAMING_INL_
#define M5_HAL_UART_BUS_STREAMING_INL_

#include "bus_streaming.hpp"

namespace m5::hal::v2::uart {

result_t<size_t> Bus_streaming::write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* src, size_t len)
{
    (void)owner;
    size_t done = 0;
    while (src != nullptr && !src->eof() && done < len) {
        auto span = src->peek(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        auto w = rawWrite(span.value().data, span.value().size, cfg.write_timeout_ms);
        if (!w.has_value()) {
            return m5::stl::make_unexpected(w.error());
        }
        if (w.value() == 0) {
            break;
        }
        auto advanced = src->advance(w.value());
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        done += w.value();
    }
    return done;
}

result_t<size_t> Bus_streaming::read(bus::IAccessor* owner, const AccessConfig& cfg, data::Sink* dst, size_t len)
{
    (void)owner;
    size_t done = 0;
    while (dst != nullptr && !dst->closed() && done < len) {
        const uint32_t timeout = (done == 0) ? cfg.first_byte_timeout_ms : cfg.inter_byte_timeout_ms;
        auto span              = dst->reserve(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        auto r = rawRead(span.value().data, span.value().size, timeout);
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        if (r.value() == 0) {
            break;
        }
        auto committed = dst->commit(r.value());
        if (!committed.has_value()) {
            return m5::stl::make_unexpected(committed.error());
        }
        done += r.value();
    }
    return done;
}

result_t<size_t> Bus_streaming::readableBytes(bus::IAccessor* owner, const AccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    return rawReadableBytes();
}

}  // namespace m5::hal::v2::uart

#endif  // M5_HAL_UART_BUS_STREAMING_INL_
