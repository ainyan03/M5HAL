#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_TCP_TCP_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_TCP_TCP_HPP

// Device-side TCP transport for the remote bus. TCP is a TRANSPORT, not
// a bus kind (spec/design/remote.md §transport): the socket implements
// the stream seam (`data::StreamReader` / `data::StreamWriter`)
// directly, so the frame / message / server / credit layers run
// unchanged on top. No Bus / Accessor / offer declaration — a socket is
// not a peripheral.
//
// Written against the BSD socket API only (lwIP serves it through the
// ESP-IDF VFS), so the SAME implementation backs both the espidf and
// the arduino-esp32 framework — the framework difference is the WiFi
// bring-up, which is example responsibility (decisions/024). The host
// (PC) sibling lives in the posix variant; the device side is the
// server, so this file carries listen/accept/attach and deliberately no
// client connect.
//
// WiFi credentials / network bring-up never enter the HAL.

#if defined(ESP_PLATFORM) && __has_include(<sys/socket.h>)

#include "../../../../../hal/v1/data/stream.hpp"

namespace m5::variants::frameworks::espidf::hal::v1::tcp {

using namespace ::m5::hal::v1;

/*!
  @brief One accepted TCP connection as a byte stream.

  - `read` blocks up to `read_timeout_ms` for the first byte, then
    returns whatever one `recv` delivers. Zero bytes = nothing arrived
    in time (the StreamReader contract); a peer hang-up is reported as
    `CLOSED` — the serve loop's cue to drop back to `accept`.
  - `write` pushes the whole span, waiting up to `write_timeout_ms`
    when the send buffer is full; a timeout yields a short write.
  - Adopted sockets get `TCP_NODELAY` (the remote bus exchanges many
    small frames that Nagle would stall) and, where lwIP provides the
    options, a short keep-alive — a host that vanished without a FIN/RST
    must not hold the single serve slot for the full retransmission
    timeout.
 */
class TcpStream : public data::StreamReader, public data::StreamWriter {
public:
    ~TcpStream() override
    {
        close();
    }

    uint32_t read_timeout_ms  = 100;   ///< max wait for the first byte of a `read`.
    uint32_t write_timeout_ms = 1000;  ///< max wait for send-buffer space per `write`.

    /*!
      @brief Adopt an already-connected socket (from `TcpListener::accept`).

      Configures it for stream use (non-blocking, TCP_NODELAY,
      keep-alive). With `take_ownership` (default) the stream closes the
      fd; pass false to keep ownership at the caller.
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

  The remote-bus topology is device = server, one connection at a time
  (the Server is single-session; spec/design/remote.md §transport): the
  serve loop accepts, serves until the stream reports `CLOSED`, then
  comes back here for the next peer.
 */
class TcpListener {
public:
    ~TcpListener()
    {
        close();
    }

    /*! @brief Bind and listen. `bind_addr` nullptr = all interfaces. */
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

}  // namespace m5::variants::frameworks::espidf::hal::v1::tcp

#endif  // ESP_PLATFORM && <sys/socket.h>

#endif
