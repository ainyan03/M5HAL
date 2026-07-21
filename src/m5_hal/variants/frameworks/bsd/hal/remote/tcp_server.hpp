// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_BSD_HAL_REMOTE_TCP_SERVER_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_BSD_HAL_REMOTE_TCP_SERVER_HPP

#include "../tcp/bsd_tcp.hpp"
#include "../../../../../hal/v2/remote/server.hpp"
#include "../../../../../hal/v2/remote/server_adapter.hpp"
#include "../../../../../hal/v2/remote/server_bus_pool.hpp"
#include "../../../../../hal/v2/remote/server_connection_wiring.hpp"
#include "../../../../../hal/v2/remote/server_handler.hpp"

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <new>

#if M5HAL_DETAIL_BSD_TCP_AVAILABLE

namespace m5::hal::v2::remote {

#ifndef M5HAL_CONFIG_REMOTE_TCP_MAX_CONNECTIONS
#define M5HAL_CONFIG_REMOTE_TCP_MAX_CONNECTIONS 2
#endif
static_assert(M5HAL_CONFIG_REMOTE_TCP_MAX_CONNECTIONS >= 1,
              "M5HAL_CONFIG_REMOTE_TCP_MAX_CONNECTIONS must be at least 1");

class BsdTcpRemoteServer {
public:
    static constexpr size_t kMaxConnections = M5HAL_CONFIG_REMOTE_TCP_MAX_CONNECTIONS;

    using connection_setup_fn_t = result_t<void> (*)(void* ctx, Server& server);

    ~BsdTcpRemoteServer();

    result_t<void> begin(uint16_t port, const char* bind_addr = nullptr);
    void end();
    uint16_t boundPort() const;

    /*!
      @brief Register a per-connection setup callback, run right after a
      new `Server` is constructed for an accepted socket (e.g. to call
      `server.setGPIOGroup(...)`).

      `activateSlot()` calls `wireConnection()` after this callback returns,
      which (re-)installs the built-in `ServerBusPool::handler` as the
      dynamic BusCreate handler — a callback that calls
      `server.setBusCreateHandler(...)` itself will have that overwritten.
     */
    void setConnectionSetupHandler(connection_setup_fn_t fn, void* ctx);

    result_t<size_t> service();

    ServerPhysicalBusPool& physicalPool();
    /*! @brief Best-effort snapshot of active connection slots.

      May be queried from a monitoring thread while another thread pumps
      service(). The count is not a barrier for slot payload lifetime; only
      the service thread accesses a slot's stream/server/adapter objects.
     */
    size_t connectionCount() const;

private:
    struct ConnectionSlot {
        ConnectionSlot();
        ~ConnectionSlot();

        ConnectionSlot(const ConnectionSlot&)            = delete;
        ConnectionSlot& operator=(const ConnectionSlot&) = delete;

        Server* server();
        RemoteServerAdapter* adapter();

        ::m5::hal::v2::detail::BsdTcpStream stream;
        uint8_t rx_scratch[4096];
        uint8_t tx_scratch[4096];
        data::StreamSource wire_src;
        data::StreamSink wire_snk;
        data::MuxFrameEncoder enc;
        data::MuxFrameDecoder dec;
        alignas(Server) uint8_t server_storage[sizeof(Server)];
        alignas(RemoteServerAdapter) uint8_t adapter_storage[sizeof(RemoteServerAdapter)];
        uint8_t response_scratch[kMaxScriptSize];
        ServerBusPool pool;
        RemoteServerHandler handler;
        std::atomic<bool> active{false};
        bool server_ok  = false;
        bool adapter_ok = false;
    };

    result_t<void> acceptOne();
    result_t<void> activateSlot(ConnectionSlot& slot, int fd);
    void teardownSlot(ConnectionSlot& slot);
    ConnectionSlot* firstFreeSlot();

    ::m5::hal::v2::detail::BsdTcpListener _listener;
    ServerPhysicalBusPool _phys_pool;
    ConnectionSlot _slots[kMaxConnections];
    connection_setup_fn_t _setup_fn = nullptr;
    void* _setup_ctx                = nullptr;
};

}  // namespace m5::hal::v2::remote

#endif  // M5HAL_DETAIL_BSD_TCP_AVAILABLE

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_BSD_HAL_REMOTE_TCP_SERVER_HPP
