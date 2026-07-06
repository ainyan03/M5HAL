// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_CONNECTION_WIRING_HPP_
#define M5_HAL_REMOTE_SERVER_CONNECTION_WIRING_HPP_

#include "server.hpp"
#include "server_adapter.hpp"
#include "server_bus_pool.hpp"
#include "server_handler.hpp"

namespace m5::hal::v2::remote {

/*!
  @brief Transport-independent per-connection wiring shared by every server
  backend (UART single-connection, TCP per-slot).

  `server`, `pool`, `handler`, and `adapter` cross-reference each other by
  pointer (pool <-> handler <-> adapter), so wiring them piecemeal at each
  call site risks one transport omitting a hookup the others rely on.

  GPIO exposure is deliberately not a `cfg` field: it is read back from
  `server.gpioGroup()`, so the advertised HelloResp capability and the
  group subscribe/poll actually use always match whatever (if anything)
  the caller passed to `Server::setGPIOGroup()` beforehand — a caller
  cannot advertise GPIO without having registered an allowlist group.
 */
struct ConnectionWiring {
    ServerPhysicalBusPool* phys_pool    = nullptr;
    uint32_t hello_flags                = 0;
    spi::MasterAccessor* static_spi_acc = nullptr;
    uint8_t static_spi_bus_id           = 0;
};

// server/pool/handler/adapter are all caller-owned (static allocation for the
// embedded single-connection case, per-slot placement-new for TCP); this
// function never manages their lifetime.
void wireConnection(Server& server, ServerBusPool& pool, RemoteServerHandler& handler, RemoteServerAdapter& adapter,
                    const ConnectionWiring& cfg);

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_SERVER_CONNECTION_WIRING_HPP_
