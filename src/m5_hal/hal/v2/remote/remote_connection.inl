// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_REMOTE_CONNECTION_INL_
#define M5_HAL_REMOTE_REMOTE_CONNECTION_INL_

#include "./remote_connection.hpp"
#include "../diag.hpp"
#include "../../../variants/frameworks/remote/session.hpp"

namespace m5::hal::v2::remote {

result_t<size_t> RemoteConnectionState::pollSession()
{
    auto handle = sessionHandle();
    if (!handle) {
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }
    RemoteSessionHandle::Lease lease{*handle};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }
    return lease.session().poll();
}

result_t<void> RemoteConnectionState::keepalive(const PumpConfig& cfg)
{
    if (!cfg.keepalive) {
        return {};
    }
    auto handle = sessionHandle();
    if (!handle) {
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }
    RemoteSessionHandle::Lease lease{*handle};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }
    const uint32_t now = runtime::millis();
    if (cfg.keepalive_interval_ms != 0 && _last_keepalive_ms != 0 &&
        now - _last_keepalive_ms < cfg.keepalive_interval_ms) {
        return {};
    }
    _last_keepalive_ms = now;
    M5HAL_DIAG("keepalive ping now=%u interval=%u", static_cast<unsigned>(now),
               static_cast<unsigned>(cfg.keepalive_interval_ms));
    return lease.session().ping();
}

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_REMOTE_CONNECTION_INL_
