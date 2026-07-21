// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_BSD_HAL_REMOTE_TCP_SERVER_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_BSD_HAL_REMOTE_TCP_SERVER_INL

#include "tcp_server.hpp"
#include "../../../../../hal/v2/diag.hpp"

#if M5HAL_DETAIL_BSD_TCP_AVAILABLE

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace m5::hal::v2::remote {

namespace {
using error_t = ::m5::hal::v2::error::error_t;

void configureAcceptedSocket(int fd)
{
    int one = 1;
    // Best effort: remote-bus request/reply traffic benefits from low latency,
    // and keepalive lets long-idle broken links eventually free their leases.
    (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    (void)::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
}

void closeFd(int fd)
{
    if (fd >= 0) {
        ::close(fd);
    }
}
}  // namespace

BsdTcpRemoteServer::ConnectionSlot::ConnectionSlot()
    : wire_src{stream, data::DataSpan{rx_scratch, sizeof(rx_scratch)}},
      wire_snk{stream, data::DataSpan{tx_scratch, sizeof(tx_scratch)}}
{
}

BsdTcpRemoteServer::ConnectionSlot::~ConnectionSlot() = default;

Server* BsdTcpRemoteServer::ConnectionSlot::server()
{
    return reinterpret_cast<Server*>(server_storage);
}

RemoteServerAdapter* BsdTcpRemoteServer::ConnectionSlot::adapter()
{
    return reinterpret_cast<RemoteServerAdapter*>(adapter_storage);
}

BsdTcpRemoteServer::~BsdTcpRemoteServer()
{
    end();
}

result_t<void> BsdTcpRemoteServer::begin(uint16_t port, const char* bind_addr)
{
    end();
    auto err = _listener.listen(port, bind_addr, static_cast<int>(kMaxConnections));
    if (err != error_t::OK) {
        return m5::stl::make_unexpected(err);
    }
    return {};
}

void BsdTcpRemoteServer::end()
{
    for (auto& slot : _slots) {
        teardownSlot(slot);
    }
    _listener.close();
}

uint16_t BsdTcpRemoteServer::boundPort() const
{
    return _listener.boundPort();
}

void BsdTcpRemoteServer::setConnectionSetupHandler(connection_setup_fn_t fn, void* ctx)
{
    _setup_fn  = fn;
    _setup_ctx = ctx;
}

result_t<size_t> BsdTcpRemoteServer::service()
{
    auto accepted = acceptOne();
    if (!accepted.has_value()) {
        return m5::stl::make_unexpected(accepted.error());
    }

    for (auto& slot : _slots) {
        if (!slot.active.load(std::memory_order_relaxed)) {
            continue;
        }
        auto r = slot.adapter()->service();
        if (!r.has_value() || !slot.stream.isOpen()) {
            teardownSlot(slot);
        }
    }
    return connectionCount();
}

ServerPhysicalBusPool& BsdTcpRemoteServer::physicalPool()
{
    return _phys_pool;
}

size_t BsdTcpRemoteServer::connectionCount() const
{
    size_t count = 0;
    for (const auto& slot : _slots) {
        if (slot.active.load(std::memory_order_relaxed)) {
            ++count;
        }
    }
    return count;
}

result_t<void> BsdTcpRemoteServer::acceptOne()
{
    if (!_listener.isOpen()) {
        return {};
    }
    int fd = _listener.accept(0);
    if (fd < 0) {
        return {};
    }

    configureAcceptedSocket(fd);

    auto* slot = firstFreeSlot();
    if (slot == nullptr) {
        M5HAL_DIAG("accept reject fd=%d (no free slot)", fd);
        closeFd(fd);
        return {};
    }
    M5HAL_DIAG("accept fd=%d", fd);
    auto activated = activateSlot(*slot, fd);
    if (!activated.has_value()) {
        teardownSlot(*slot);
        return m5::stl::make_unexpected(activated.error());
    }
    return {};
}

result_t<void> BsdTcpRemoteServer::activateSlot(ConnectionSlot& slot, int fd)
{
    teardownSlot(slot);

    auto err = slot.stream.attach(fd, true);
    if (err != error_t::OK) {
        return m5::stl::make_unexpected(err);
    }
    // service() polls every connection in turn; a blocking first-byte wait on
    // one idle socket would stall all the others (measured 100-300 ms per
    // call with the 100 ms default). The hub is poll-driven: never wait.
    slot.stream.read_timeout_ms = 0;
    slot.enc.~MuxFrameEncoder();
    new (&slot.enc) data::MuxFrameEncoder{memory::defaultAllocator()};
    slot.dec.~MuxFrameDecoder();
    new (&slot.dec) data::MuxFrameDecoder{memory::defaultAllocator()};
    slot.wire_src.discardBuffered();

    new (slot.server_storage) Server{data::DataSpan{slot.response_scratch, sizeof(slot.response_scratch)}};
    slot.server_ok = true;

    if (_setup_fn != nullptr) {
        auto setup = _setup_fn(_setup_ctx, *slot.server());
        if (!setup.has_value()) {
            return m5::stl::make_unexpected(setup.error());
        }
    }

    new (slot.adapter_storage) RemoteServerAdapter{slot.enc, slot.dec, slot.wire_src, slot.wire_snk};
    slot.adapter_ok = true;

    // GPIO exposure follows whatever _setup_fn (onConnectionSetup) did or did
    // not register via srv.setGPIOGroup() above — wireConnection() reads it
    // back from the Server, so a setup callback that skips setGPIOGroup()
    // correctly leaves this connection without GPIO capability.
    ConnectionWiring cfg;
    cfg.phys_pool = &_phys_pool;
    wireConnection(*slot.server(), slot.pool, slot.handler, *slot.adapter(), cfg);

    M5HAL_DIAG("slot activate fd=%d", fd);
    slot.active.store(true, std::memory_order_relaxed);
    return {};
}

void BsdTcpRemoteServer::teardownSlot(ConnectionSlot& slot)
{
    const bool was_active = slot.active.load(std::memory_order_relaxed);
    if (slot.server_ok) {
        slot.pool.releaseAll();
    }
    if (slot.adapter_ok) {
        slot.adapter()->~RemoteServerAdapter();
        slot.adapter_ok = false;
    }
    slot.dec.setFrameHandler(nullptr, nullptr);
    slot.enc.releaseAll();
    slot.dec.releaseAll();
    if (slot.server_ok) {
        slot.server()->~Server();
        slot.server_ok = false;
    }
    slot.stream.close();
    slot.pool    = ServerBusPool{};
    slot.handler = RemoteServerHandler{};
    slot.active.store(false, std::memory_order_relaxed);
    if (was_active) {
        M5HAL_DIAG("slot teardown");
    }
}

BsdTcpRemoteServer::ConnectionSlot* BsdTcpRemoteServer::firstFreeSlot()
{
    for (auto& slot : _slots) {
        if (!slot.active.load(std::memory_order_relaxed)) {
            return &slot;
        }
    }
    return nullptr;
}

}  // namespace m5::hal::v2::remote

#endif  // M5HAL_DETAIL_BSD_TCP_AVAILABLE

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_BSD_HAL_REMOTE_TCP_SERVER_INL
