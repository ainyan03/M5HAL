// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DETAIL_BSD_TCP_HPP
#define M5_HAL_DETAIL_BSD_TCP_HPP

// Shared BSD-socket TCP transport pieces for the remote bus.
//
// TCP is a TRANSPORT, not a bus kind (spec/design/remote.md
// §transport): the socket implements the stream seam
// (`data::StreamReader` / `data::StreamWriter`) directly, so the
// frame / message / server / credit layers run unchanged on top. There
// is deliberately no Bus / Accessor / offer declaration — a socket is
// not a peripheral.
//
// Written against the BSD socket API only, so the SAME implementation
// backs every variant that exposes it: the posix host (macOS / Linux /
// BSD libc) and the ESP device (lwIP through the ESP-IDF VFS, which
// also covers arduino-esp32). Each variant's `hal/tcp/tcp.hpp` wraps
// these types under its own namespace — the posix side adds the
// client-only `connect`, the device side stays server-only
// (listen / accept / attach).
//
// Availability self-gate: on platforms without <sys/socket.h> this
// header (and the matching .inl) compile to nothing;
// `M5HAL_DETAIL_BSD_TCP_AVAILABLE` tells the includer which way it
// went. The variant headers additionally keep their own gates.

#if __has_include(<sys/socket.h>) && __has_include(<netinet/tcp.h>)
#define M5HAL_DETAIL_BSD_TCP_AVAILABLE 1
#else
#define M5HAL_DETAIL_BSD_TCP_AVAILABLE 0
#endif

#if M5HAL_DETAIL_BSD_TCP_AVAILABLE

#include "../hal/v1/data/stream.hpp"

namespace m5::hal::v1::detail {

/*!
  @brief Block until `fd` is readable / writable for up to `timeout_ms`.
  @return >0 when ready, 0 on timeout, <0 on error.
 */
int waitFd(int fd, bool for_write, uint32_t timeout_ms);

/*! @brief CLOCK_MONOTONIC in milliseconds (deadline arithmetic). */
uint64_t monotonicMs();

/*!
  @brief One TCP connection as a byte stream (accepted or client side).

  - `read` blocks up to `read_timeout_ms` for the first byte, then
    returns whatever one `recv` delivers. Zero bytes = nothing arrived
    in time (the StreamReader contract); a peer hang-up is reported as
    `CLOSED` instead — unlike a serial line, TCP can tell the
    difference, and the caller should know the link is gone (the device
    serve loop's cue to drop back to `accept`).
  - `write` pushes the whole span, waiting up to `write_timeout_ms`
    when the send buffer is full; a timeout yields a short write.
  - Adopted sockets get the best-effort options of `configureSocket`
    (TCP_NODELAY, SIGPIPE suppression, short keep-alive — see there).
 */
class BsdTcpStream : public data::StreamReader, public data::StreamWriter {
public:
    ~BsdTcpStream() override
    {
        close();
    }

    // Read/write pacing, mirroring the host UART defaults
    // (SerialRemoteEndpoint::makeHostConfig). Plain members: tune freely.
    uint32_t read_timeout_ms  = 100;   ///< max wait for the first byte of a `read`.
    uint32_t write_timeout_ms = 1000;  ///< max wait for send-buffer space per `write`.

    /*!
      @brief Adopt an already-connected socket (e.g. from `BsdTcpListener::accept`).

      Configures it for stream use (`configureSocket`). With
      `take_ownership` (default) the stream closes the fd; pass false to
      keep ownership at the caller.
     */
    ::m5::hal::v1::error::error_t attach(int fd, bool take_ownership = true);

    void close();

    bool isOpen() const
    {
        return _fd >= 0;
    }
    int nativeHandle() const
    {
        return _fd;
    }

    // StreamReader
    ::m5::hal::v1::result_t<size_t> read(data::DataSpan dst) override;
    ::m5::hal::v1::result_t<size_t> readableBytes(void) override;
    // StreamWriter
    ::m5::hal::v1::result_t<size_t> write(data::ConstDataSpan src) override;

protected:
    /*!
      @brief Make `fd` non-blocking and apply the best-effort options.

      - `TCP_NODELAY`: the remote bus exchanges many small frames in
        request/reply patterns that Nagle would serialize into 40 ms+
        stalls.
      - SIGPIPE suppression on write-after-hangup: Linux has the
        per-send flag, macOS/BSD the per-socket option; lwIP has no
        process signals (each applied only where defined).
      - Short keep-alive where the platform provides the knobs: a peer
        that vanished without a FIN/RST (lid shut, cable pulled) is
        detected in ~15 s instead of the TCP retransmission timeout
        (minutes) — on the device that frees the single serve slot, on
        the host it turns idle reads into `CLOSED`.

      A transport that cannot set an option still works, just with
      worse latency / dead-peer detection; only the non-blocking switch
      is mandatory.
     */
    ::m5::hal::v1::error::error_t configureSocket(int fd);

    int _fd       = -1;
    bool _owns_fd = false;
};

/*!
  @brief Minimal IPv4 TCP listener: bind → listen → accept one peer.

  The remote-bus topology is device = server, one connection at a time
  (the Server is single-session; spec/design/remote.md §transport).
  On the host side this backs the localhost E2E tests. `port` 0 binds
  an ephemeral port — read it back with `boundPort()` (how tests avoid
  collisions).
 */
class BsdTcpListener {
public:
    ~BsdTcpListener()
    {
        close();
    }

    /*! @brief Bind and listen. `bind_addr` nullptr = all interfaces
        (tests pass "127.0.0.1" to stay loopback-only). */
    ::m5::hal::v1::error::error_t listen(uint16_t port, const char* bind_addr = nullptr, int backlog = 1);

    /*! @brief Wait up to `timeout_ms` for one connection.
        @return the connected fd (caller owns it; hand it to
        `BsdTcpStream::attach`), or -1 on timeout / error. */
    int accept(uint32_t timeout_ms);

    void close();

    bool isOpen() const
    {
        return _fd >= 0;
    }
    /*! @brief Actual bound port (resolves a `port` 0 request). */
    uint16_t boundPort() const
    {
        return _port;
    }

private:
    int _fd        = -1;
    uint16_t _port = 0;
};

}  // namespace m5::hal::v1::detail

#endif  // M5HAL_DETAIL_BSD_TCP_AVAILABLE

#endif
