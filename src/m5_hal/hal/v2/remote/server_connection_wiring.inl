// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_CONNECTION_WIRING_INL_
#define M5_HAL_REMOTE_SERVER_CONNECTION_WIRING_INL_

#include "server_connection_wiring.hpp"

#include "../diag.hpp"

namespace m5::hal::v2::remote {

void wireConnection(Server& server, ServerBusPool& pool, RemoteServerHandler& handler, RemoteServerAdapter& adapter,
                    const ConnectionWiring& cfg)
{
    pool                  = ServerBusPool{};
    pool.server           = &server;
    pool.phys             = cfg.phys_pool;
    pool.pins_claimed_fn  = &RemoteServerHandler::gpioPinsClaimed;
    pool.pins_claimed_ctx = &handler;
    server.setBusCreateHandler(&ServerBusPool::handler, &pool);

    // Read GPIO exposure back from the Server instead of taking it as a cfg
    // field: the advertised HelloResp capability and the group subscribe/poll
    // actually use must always be the group (if any) the caller already
    // registered via setGPIOGroup(), never an independently chosen one.
    auto* registered_group = server.gpioGroup();

    handler                   = RemoteServerHandler{};
    handler.server            = &server;
    handler.pool              = &pool;
    handler.hello_flags       = static_cast<uint8_t>(cfg.hello_flags);
    handler.gpio              = registered_group != nullptr ? const_cast<gpio::IGPIO*>(gpio::getGPIO()) : nullptr;
    handler.gpio_group        = registered_group;
    handler.static_spi_acc    = cfg.static_spi_acc;
    handler.static_spi_bus_id = cfg.static_spi_bus_id;

    server.runner().setGpioSubscribeHandler(&RemoteServerHandler::gpioSubscribe, &handler);
    server.runner().setGpioModeHandler(&RemoteServerHandler::gpioModeSet, &handler);

    adapter.setHandler(&RemoteServerHandler::handler, &handler);
    adapter.setPollHandler(&RemoteServerHandler::poll, &handler);
    M5HAL_DIAG("wire connection gpio=%d hello_flags=0x%02x", registered_group != nullptr,
               static_cast<unsigned>(handler.hello_flags));
}

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_SERVER_CONNECTION_WIRING_INL_
