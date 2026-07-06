// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_DETAIL_HELPERS_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_DETAIL_HELPERS_HPP_

#include "../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../hal/v2/data/memory.hpp"
#include "../../../hal/v2/remote/remote.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::remote {
namespace detail {

inline void putI16LE(uint8_t* dst, types::gpio_number_t value)
{
    const auto v = static_cast<uint16_t>(value);
    dst[0]       = static_cast<uint8_t>(v & 0xFFu);
    dst[1]       = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

inline uint8_t sizeToUnit(size_t value, size_t unit)
{
    if (value == 0) {
        return 0;
    }
    size_t units = (value + unit - 1) / unit;
    return units > 255 ? 255 : static_cast<uint8_t>(units);
}

inline uint8_t remoteKindIndex(types::bus_kind_t kind)
{
    switch (kind) {
        case types::bus_kind_t::I2C:
            return 0;
        case types::bus_kind_t::SPI:
            return 1;
        case types::bus_kind_t::I2S:
            return 2;
        case types::bus_kind_t::UART:
            return 3;
        default:
            return 0xFF;
    }
}

inline result_t<Capabilities> decodeHelloCaps(data::ConstDataSpan resp)
{
    Capabilities caps;
    if (resp.size == 0) {
        caps.proto_ver           = kProtocolVersion;
        caps.supports_bus_create = true;
        return caps;
    }
    if (resp.size < 3) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    caps.proto_ver = resp.data[0];
    if (caps.proto_ver != kProtocolVersion) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    const uint8_t flags      = resp.data[1];
    caps.has_gpio            = (flags & 0x01u) != 0;
    caps.supports_bus_create = (flags & 0x02u) != 0;
    const size_t n           = resp.data[2];
    if (n > Capabilities::kMaxEntries || resp.size < 3 + n * 2) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    caps.bus_count = n;
    for (size_t i = 0; i < n; ++i) {
        caps.buses[i].kind   = static_cast<types::bus_kind_t>(resp.data[3 + i * 2]);
        caps.buses[i].bus_id = resp.data[4 + i * 2];
    }
    const size_t gpio_off = 3 + n * 2;
    if (caps.has_gpio && resp.size >= gpio_off + 3) {
        caps.gpio_port_count = resp.data[gpio_off];
        caps.gpio_pin_count =
            static_cast<uint16_t>(static_cast<uint16_t>(resp.data[gpio_off + 1]) |
                                  static_cast<uint16_t>(static_cast<uint16_t>(resp.data[gpio_off + 2]) << 8));
    }
    return caps;
}

inline result_t<void> decodeResponseStatus(data::ConstDataSpan resp, bytecode::BytecodeRunner* runner = nullptr)
{
    bytecode::BytecodeRunner local_runner{memory::defaultAllocator()};
    bytecode::BytecodeRunner& r = runner != nullptr ? *runner : local_runner;
    r.setReceiveOnly(true);
    auto run = r.run(resp);
    if (!run.has_value()) {
        return m5::stl::make_unexpected(run.error());
    }
    if (!r.statusReported()) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    if (error::isError(r.reportedStatus())) {
        return m5::stl::make_unexpected(r.reportedStatus());
    }
    return {};
}

}  // namespace detail
}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_DETAIL_HELPERS_HPP_
