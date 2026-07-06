// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_REMOTE_TRANSFER_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_REMOTE_TRANSFER_HPP_

#include "../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../hal/v2/data.hpp"
#include "./session.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace m5::hal::v2::remote {

constexpr size_t kMaxRemoteConfigCacheSize = bytecode::kUARTConfigSize;
static_assert(bytecode::kI2CConfigSize <= kMaxRemoteConfigCacheSize, "remote config cache must fit i2c config");
static_assert(bytecode::kSPIConfigSize <= kMaxRemoteConfigCacheSize, "remote config cache must fit spi config");
static_assert(bytecode::kI2SConfigSize <= kMaxRemoteConfigCacheSize, "remote config cache must fit i2s config");

class RemoteConfigCache {
public:
    data::ConstDataSpan pending(RemoteSession* session, data::ConstDataSpan cfg) const
    {
        if (cfg.size == 0 || matches(session, cfg)) {
            return {};
        }
        return cfg;
    }

    void rememberSent(RemoteSession* session, data::ConstDataSpan cfg)
    {
        if (session == nullptr || cfg.data == nullptr || cfg.size == 0 || cfg.size > sizeof(_bytes)) {
            _valid = false;
            return;
        }
        ::memcpy(_bytes, cfg.data, cfg.size);
        _size       = cfg.size;
        _generation = session->configGeneration();
        _valid      = true;
    }

private:
    bool matches(RemoteSession* session, data::ConstDataSpan cfg) const
    {
        return session != nullptr && _valid && _generation == session->configGeneration() && _size == cfg.size &&
               cfg.data != nullptr && ::memcmp(_bytes, cfg.data, cfg.size) == 0;
    }

    uint8_t _bytes[kMaxRemoteConfigCacheSize] = {};
    size_t _size                              = 0;
    uint32_t _generation                      = 0;
    bool _valid                               = false;
};

result_t<size_t> remoteTransferWire(RemoteSession* session, types::bus_kind_t kind, uint8_t bus_id,
                                    data::ConstDataSpan cfg_bytes, data::ConstDataSpan meta, data::Source* src,
                                    size_t tx_len, data::Sink* dst, size_t rx_len, uint32_t timeout_ms,
                                    RemoteConfigCache* config_cache = nullptr);

namespace detail {

// One serializer per kind lives in bytecode (encodeConfig, next to its
// decode counterpart); these wrappers only add the span-with-size shape
// the shims want.
inline data::ConstDataSpan encodeRemoteConfig(uint8_t* dst, const i2c::MasterAccessConfig& cfg)
{
    bytecode::detail::encodeConfig(dst, cfg);
    return {dst, bytecode::kI2CConfigSize};
}

inline data::ConstDataSpan encodeRemoteConfig(uint8_t* dst, const spi::MasterAccessConfig& cfg)
{
    bytecode::detail::encodeConfig(dst, cfg);
    return {dst, bytecode::kSPIConfigSize};
}

inline data::ConstDataSpan encodeRemoteConfig(uint8_t* dst, const uart::AccessConfig& cfg)
{
    bytecode::detail::encodeConfig(dst, cfg);
    return {dst, bytecode::kUARTConfigSize};
}

inline data::ConstDataSpan encodeRemoteConfig(uint8_t* dst, const i2s::AccessConfig& cfg)
{
    bytecode::detail::encodeConfig(dst, cfg);
    return {dst, bytecode::kI2SConfigSize};
}

}  // namespace detail
}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_REMOTE_TRANSFER_HPP_
