// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_DETAIL_HELPERS_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_DETAIL_HELPERS_HPP_

#include "../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../hal/v2/data/memory.hpp"
#include "../../../hal/v2/remote/remote.hpp"
#include "../../../hal/v2/remote/bus_capabilities_wire.hpp"

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
        case types::bus_kind_t::PDM:
            return 4;
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
    const uint8_t flags       = resp.data[1];
    caps.has_gpio             = (flags & kHelloFlagGpio) != 0;
    caps.supports_bus_create  = (flags & kHelloFlagBusCreate) != 0;
    caps.has_bus_capabilities = (flags & kHelloFlagBusCapabilities) != 0;
    const size_t n            = resp.data[2];
    if (n > Capabilities::kMaxEntries || resp.size < 3 + n * 2) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    caps.bus_count = n;
    for (size_t i = 0; i < n; ++i) {
        caps.buses[i].kind   = static_cast<types::bus_kind_t>(resp.data[3 + i * 2]);
        caps.buses[i].bus_id = resp.data[4 + i * 2];
    }
    size_t tail_off = 3 + n * 2;
    if (caps.has_gpio) {
        if (resp.size < tail_off + 3) {
            return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
        }
        caps.gpio_port_count = resp.data[tail_off];
        caps.gpio_pin_count =
            static_cast<uint16_t>(static_cast<uint16_t>(resp.data[tail_off + 1]) |
                                  static_cast<uint16_t>(static_cast<uint16_t>(resp.data[tail_off + 2]) << 8));
        tail_off += 3;
    }
    if (!caps.has_bus_capabilities) {
        return caps;
    }
    if (resp.size <= tail_off) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    const size_t envelope_size = resp.data[tail_off++];
    if (envelope_size > resp.size - tail_off) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    const size_t envelope_end = tail_off + envelope_size;
    bool seen[Capabilities::kMaxEntries]{};
    while (tail_off < envelope_end) {
        if (envelope_end - tail_off < 2) {
            return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
        }
        const uint8_t tag = resp.data[tail_off++];
        const size_t len  = resp.data[tail_off++];
        if (len > envelope_end - tail_off) {
            return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
        }
        const size_t tlv_end = tail_off + len;
        if (tag != kHelloExtensionBusCapabilities) {
            tail_off = tlv_end;
            continue;
        }
        if (tail_off >= tlv_end) {
            return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
        }
        const size_t count = resp.data[tail_off++];
        if (count > Capabilities::kMaxEntries) {
            return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
        }
        for (size_t i = 0; i < count; ++i) {
            if (tlv_end - tail_off < 3) {
                return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
            }
            const auto kind       = static_cast<types::bus_kind_t>(resp.data[tail_off++]);
            const uint8_t bus_id  = resp.data[tail_off++];
            const size_t rec_size = resp.data[tail_off++];
            if (rec_size > tlv_end - tail_off) {
                return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
            }
            size_t entry_index = caps.bus_count;
            for (size_t j = 0; j < caps.bus_count; ++j) {
                if (caps.buses[j].kind == kind && caps.buses[j].bus_id == bus_id) {
                    entry_index = j;
                    break;
                }
            }
            auto decoded = decodeBusCapabilitiesWire({resp.data + tail_off, rec_size});
            if (entry_index < caps.bus_count) {
                if (seen[entry_index]) {
                    return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
                }
                seen[entry_index] = true;
                if (decoded.has_value()) {
                    caps.buses[entry_index].capabilities = decoded.value();
                } else if (decoded.error() != error::error_t::UNSUPPORTED) {
                    return m5::stl::make_unexpected(decoded.error());
                }
            } else if (!decoded.has_value() && decoded.error() != error::error_t::UNSUPPORTED) {
                return m5::stl::make_unexpected(decoded.error());
            }
            tail_off += rec_size;
        }
        if (tail_off != tlv_end) {
            return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
        }
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
