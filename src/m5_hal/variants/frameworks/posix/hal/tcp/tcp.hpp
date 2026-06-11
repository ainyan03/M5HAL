#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_TCP_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_TCP_TCP_HPP

// Host TCP transport for the remote bus. TCP is a TRANSPORT, not a bus
// kind (spec/design/remote.md §transport): the socket implements the
// stream seam (`data::StreamReader` / `data::StreamWriter`) directly,
// so the frame / message / server / credit layers run unchanged on top.
// There is deliberately no Bus / Accessor / offer declaration here — a
// socket is not a peripheral.
//
// Gated like the rest of the posix variant: active on POSIX host builds
// only (frameworks/_checker.hpp, M5HAL_FRAMEWORK_HAS_POSIX).

#if M5HAL_FRAMEWORK_HAS_POSIX

#include "../../../../../hal/v1/data/stream.hpp"

namespace m5::variants::frameworks::posix::hal::v1::tcp {

using namespace ::m5::hal::v1;

/*!
  @brief One TCP connection as a byte stream (client or accepted side).

  - `read` blocks up to `read_timeout_ms` for the first byte, then
    returns whatever one `recv` delivers. Zero bytes = nothing arrived
    in time (the StreamReader contract); a peer hang-up is reported as
    `CLOSED` instead — unlike a serial line, TCP can tell the
    difference, and the caller should know the link is gone.
  - `write` pushes the whole span, waiting up to `write_timeout_ms`
    when the send buffer is full; a timeout yields a short write.
  - `TCP_NODELAY` is set on every adopted socket: the remote bus
    exchanges many small frames in request/reply patterns that Nagle
    would serialize into 40 ms+ stalls.
 */
class TcpStream : public data::StreamReader, public data::StreamWriter {
public:
    ~TcpStream() override
    {
        close();
    }

    // Read/write pacing, mirroring the host UART defaults
    // (SerialRemoteEndpoint::makeHostConfig). Plain members: tune freely.
    uint32_t read_timeout_ms  = 100;   ///< max wait for the first byte of a `read`.
    uint32_t write_timeout_ms = 1000;  ///< max wait for send-buffer space per `write`.

    /*!
      @brief Connect to `host:port` (numeric address or DNS name, IPv4/IPv6).
      @param timeout_ms bound on the whole attempt (resolve excluded: getaddrinfo
             has no portable timeout; numeric addresses never block there).
      Replaces any previously held socket.
     */
    ::m5::hal::v1::error::error_t connect(const char* host, uint16_t port, uint32_t timeout_ms = 3000);

    /*!
      @brief Adopt an already-connected socket (e.g. from `TcpListener::accept`).

      Configures it for stream use (non-blocking, TCP_NODELAY, SIGPIPE
      suppression where available). With `take_ownership` (default) the
      stream closes the fd; pass false to keep ownership at the caller.
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
    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> read(data::DataSpan dst) override;
    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> readableBytes(void) override;
    // StreamWriter
    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> write(data::ConstDataSpan src) override;

private:
    ::m5::hal::v1::error::error_t configureSocket(int fd);

    int _fd       = -1;
    bool _owns_fd = false;
};

/*!
  @brief Minimal IPv4 TCP listener: bind → listen → accept one peer.

  Backs the localhost E2E tests and any host-side serving; the device
  (lwIP) server has its own variant. `port` 0 binds an ephemeral port —
  read it back with `boundPort()` (how tests avoid collisions).
 */
class TcpListener {
public:
    ~TcpListener()
    {
        close();
    }

    /*! @brief Bind and listen. `bind_addr` nullptr = all interfaces
        (tests pass "127.0.0.1" to stay loopback-only). */
    ::m5::hal::v1::error::error_t listen(uint16_t port, const char* bind_addr = nullptr, int backlog = 1);

    /*! @brief Wait up to `timeout_ms` for one connection.
        @return the connected fd (caller owns it; hand it to
        `TcpStream::attach`), or -1 on timeout / error. */
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

}  // namespace m5::variants::frameworks::posix::hal::v1::tcp

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
