// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_BUS_LEASE_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_BUS_LEASE_HPP_

#include "../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../hal/v2/bus/bus.hpp"
#include "../../../hal/v2/data/memory.hpp"
#include "../../../hal/v2/remote/remote.hpp"
#include "../../../hal/v2/remote/session_handle.hpp"
#include "detail_helpers.hpp"
#include "session.hpp"

#include <cstdlib>
#include <cstdint>
#include <memory>
#include <utility>

namespace m5::hal::v2::remote {

class RemoteBusIdState {
public:
    uint8_t reserve(types::bus_kind_t kind)
    {
        Guard guard{_mutex};
        if (!guard.locked) {
            return 0xFF;
        }
        const uint8_t idx = detail::remoteKindIndex(kind);
        if (idx >= 5) {
            return 0xFF;
        }
        for (uint8_t i = 0; i < bytecode::kMaxBusBindings; ++i) {
            if ((_used[idx] & (1u << i)) == 0) {
                _used[idx] |= static_cast<uint8_t>(1u << i);
                return i;
            }
        }
        return 0xFF;
    }

    void release(types::bus_kind_t kind, uint8_t bus_id)
    {
        Guard guard{_mutex};
        if (!guard.locked) {
            return;
        }
        const uint8_t idx = detail::remoteKindIndex(kind);
        if (idx < 5 && bus_id < bytecode::kMaxBusBindings) {
            _used[idx] &= static_cast<uint8_t>(~(1u << bus_id));
        }
    }

private:
    struct Guard {
        runtime::Mutex& mutex;
        bool locked;
        explicit Guard(runtime::Mutex& m) : mutex{m}, locked{mutex.lock(types::TIMEOUT_FOREVER).has_value()}
        {
        }
        ~Guard()
        {
            if (locked) {
                if (!mutex.unlock().has_value()) {
                    std::abort();
                }
            }
        }
    };

    runtime::Mutex _mutex;
    uint8_t _used[5] = {};
};

class RemoteBusLease {
public:
    RemoteBusLease(std::shared_ptr<RemoteSessionHandle> handle, std::shared_ptr<RemoteBusIdState> ids,
                   types::bus_kind_t kind, uint8_t bus_id)
        : _handle{std::move(handle)}, _ids{std::move(ids)}, _kind{kind}, _bus_id{bus_id}
    {
    }

    ~RemoteBusLease()
    {
        // Destruction can happen from a RemoteSession callback.  Never wait
        // for either the operation gate or that same non-recursive session
        // mutex in this path; an unsuccessful best-effort release quarantines
        // both the identity tombstone and numeric id instead of risking ABA
        // reuse.
        bus::BusLifecycle::Close closing{*_lifecycle, 0};
        if (!closing) {
            if (_lifecycle->state() != bus::BusLifecycle::State::Closed) {
                _lifecycle->quarantineWithoutLock();
            }
            return;
        }
        if (!_armed) {
            auto committed = closing.commit();
            if (committed.has_value()) {
                releaseId();
            } else {
                _lifecycle->quarantineWithoutLock();
            }
            return;
        }
        bool released = false;
        if (_handle) {
            RemoteSessionHandle::Lease lease{*_handle, 0};
            if (lease) {
                auto result = sendRelease(lease.session());
                if (result.has_value()) {
                    released = true;
                }
            }
        }
        if (released) {
            auto committed = closing.commit();
            if (committed.has_value()) {
                releaseId();
            } else {
                _lifecycle->quarantineWithoutLock();
            }
        } else {
            (void)closing.quarantine();
        }
    }

    RemoteBusLease(const RemoteBusLease&)            = delete;
    RemoteBusLease& operator=(const RemoteBusLease&) = delete;

    void arm()
    {
        _armed = true;
    }

    void disarm()
    {
        _armed = false;
        releaseId();
    }

    std::shared_ptr<bus::BusLifecycle> lifecycle() const
    {
        return _lifecycle;
    }

    using Operation = bus::BusLifecycle::Operation;
    using Close     = bus::BusLifecycle::Close;

private:
    result_t<void> sendRelease(RemoteSession& session)
    {
        uint8_t script_buf[kMaxScriptSize];
        data::MemorySink script{script_buf, sizeof(script_buf)};
        bytecode::BytecodeEncoder enc{script};
        auto r = enc.busRelease(_kind, _bus_id);
        if (r.has_value()) {
            r = enc.end();
        }
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        auto req = session.request({script_buf, script.written()});
        if (!req.has_value()) {
            return m5::stl::make_unexpected(req.error());
        }
        return session.checkResponse();
    }

    void releaseId()
    {
        if (_ids) {
            _ids->release(_kind, _bus_id);
            _ids.reset();
        }
    }

    std::shared_ptr<RemoteSessionHandle> _handle;
    std::shared_ptr<RemoteBusIdState> _ids;
    types::bus_kind_t _kind                       = types::bus_kind_t::Unknown;
    uint8_t _bus_id                               = 0xFF;
    bool _armed                                   = false;
    std::shared_ptr<bus::BusLifecycle> _lifecycle = std::make_shared<bus::BusLifecycle>();
};

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_BUS_LEASE_HPP_
