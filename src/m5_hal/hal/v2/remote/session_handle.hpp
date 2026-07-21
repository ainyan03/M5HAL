// SPDX-License-Identifier: MIT

#ifndef M5_HAL_REMOTE_SESSION_HANDLE_HPP_
#define M5_HAL_REMOTE_SESSION_HANDLE_HPP_

#include "../error.hpp"
#include "../runtime/runtime.hpp"
#include "../types.hpp"

#include <cstdlib>
#include <cstdint>

namespace m5::hal::v2::remote {

class RemoteSession;
uint64_t nextRemoteSessionGeneration(void);

/*!
  @brief Shared lifetime and serialization gate for one remote session.

  A connection binds its session before publishing this handle to a backend.
  Proxies keep the handle alive, not the connection or transport. Closing the
  handle waits for the current lease, then makes every later lease fail with
  `CLOSED`, so a proxy may safely outlive a Hal connection.
 */
class RemoteSessionHandle {
public:
    RemoteSessionHandle() : _generation{nextRemoteSessionGeneration()}
    {
    }
    RemoteSessionHandle(const RemoteSessionHandle&)            = delete;
    RemoteSessionHandle& operator=(const RemoteSessionHandle&) = delete;
    RemoteSessionHandle(RemoteSessionHandle&&)                 = delete;
    RemoteSessionHandle& operator=(RemoteSessionHandle&&)      = delete;

    // Construction-time operation. Call before sharing the handle.
    void bind(RemoteSession& session)
    {
        _session = &session;
    }

    uint64_t generation(void) const
    {
        return _generation;
    }

    bool valid(void) const
    {
        return _generation != 0;
    }

    /*!
      @brief Exclusive access to the bound session.

      A lease covers a complete logical RPC, including stream attachment and
      response parsing. It is deliberately neither copyable nor movable so its
      lock ownership cannot escape the declaring scope accidentally.
     */
    class Lease {
    public:
        explicit Lease(RemoteSessionHandle& handle, uint32_t timeout_ms = types::TIMEOUT_FOREVER) : _handle{&handle}
        {
            const auto locked = handle._mutex.lock(timeout_ms);
            _locked           = locked.has_value();
            _error            = locked.has_value() ? error::error_t::OK : locked.error();
            if (_locked) {
                _session = handle._session;
            }
        }

        ~Lease()
        {
            if (_locked) {
                if (!_handle->_mutex.unlock().has_value()) {
                    std::abort();
                }
            }
        }

        Lease(const Lease&)            = delete;
        Lease& operator=(const Lease&) = delete;
        Lease(Lease&&)                 = delete;
        Lease& operator=(Lease&&)      = delete;

        explicit operator bool() const
        {
            return _locked && _session != nullptr;
        }

        error::error_t error() const
        {
            return _locked ? error::error_t::CLOSED : _error;
        }

        RemoteSession& session() const
        {
            return *_session;
        }

    private:
        RemoteSessionHandle* _handle = nullptr;
        RemoteSession* _session      = nullptr;
        bool _locked                 = false;
        error::error_t _error        = error::error_t::OK;
    };

    /*!
      @brief Invalidate the session after all current work has completed.

      Idempotent. The connection must call this before destroying objects the
      session borrows (encoder, decoder, wire source, and wire sink).
     */
    [[nodiscard]] result_t<void> close()
    {
        auto locked = _mutex.lock(types::TIMEOUT_FOREVER);
        if (!locked.has_value()) {
            return m5::stl::make_unexpected(locked.error());
        }
        _session = nullptr;
        return _mutex.unlock();
    }

private:
    runtime::Mutex _mutex;
    RemoteSession* _session = nullptr;
    uint64_t _generation    = 0;
};

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_SESSION_HANDLE_HPP_
