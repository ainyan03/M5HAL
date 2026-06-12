// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_REMOTE_CONNECT_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_REMOTE_CONNECT_HPP

// Remote-bus connection establishment over a host serial port: the
// composition of the candidate enumeration (ports.hpp, name heuristics)
// with the protocol-level verification (remote hello). The name
// heuristic only proposes candidates; a port is the peer if and only if
// it answers hello (spec/design/remote.md §接続ユーティリティ).

#if M5HAL_FRAMEWORK_HAS_POSIX

#include "../../../../../hal/v1/remote/remote.hpp"
#include "./ports.hpp"
#include "./uart.hpp"

namespace m5::variants::frameworks::posix::hal::v1::uart {

/*!
  @brief Owning bundle for one serial remote endpoint: the posix Bus, its
         split UART accessors, and the transport-agnostic `RemoteLink`.

  After `connectRemoteSerial` succeeds, `link.session()` is the
  established session and `devicePath()` names the chosen port.
 */
struct SerialRemoteEndpoint {
    Bus bus;
    ::m5::hal::v1::uart::UARTRxAccessor rx;
    ::m5::hal::v1::uart::UARTTxAccessor tx;
    ::m5::hal::v1::remote::RemoteLink link;

    /*! @brief Host-friendly defaults (100 ms first-byte / 20 ms inter-byte read timeouts). */
    explicit SerialRemoteEndpoint(uint32_t baud = 115200) : SerialRemoteEndpoint(makeHostConfig(baud))
    {
    }
    SerialRemoteEndpoint(const ::m5::hal::v1::uart::UARTAccessConfig& uart_cfg,
                         const ::m5::hal::v1::remote::RemoteSession::Config& session_cfg =
                             ::m5::hal::v1::remote::RemoteSession::Config{})
        : rx{bus, uart_cfg}, tx{bus, uart_cfg}, link{rx, tx, session_cfg}
    {
        // Remote hosts burst many small frames and then wait for replies —
        // exactly the pattern that benefits from coalescing writes into a few
        // syscalls/USB transfers (measured ~35 % more sustained throughput on
        // a USB CDC bridge). The read paths auto-flush, so the request/reply
        // exchanges of connect/RPC are unaffected.
        BusConfig bus_cfg;
        bus_cfg.tx_coalesce_bytes = 4096;
        (void)bus.init(bus_cfg);
    }

    /*! @brief Port chosen by the last successful `connectRemoteSerial` ("" before that). */
    const char* devicePath() const
    {
        return _device_path;
    }

    static ::m5::hal::v1::uart::UARTAccessConfig makeHostConfig(uint32_t baud)
    {
        ::m5::hal::v1::uart::UARTAccessConfig cfg;
        cfg.baud_rate             = baud;
        cfg.first_byte_timeout_ms = 100;
        cfg.inter_byte_timeout_ms = 20;
        cfg.write_timeout_ms      = 1000;
        return cfg;
    }

    // Written by connectRemoteSerial.
    char _device_path[kSerialPortPathCapacity] = {};
};

struct ConnectOptions {
    /*! @brief Try only this port when set; nullptr enables auto-discovery. */
    const char* path = nullptr;
    /*! @brief Per-attempt hello timeout while probing (the session's configured timeout is restored after). */
    uint32_t hello_timeout_ms = 500;
    /*! @brief hello attempts on strong candidates (rank 0): the board
        needs ~1 s to boot after the connect-time reset. */
    int strong_attempts = 4;
    int weak_attempts   = 1;  ///< hello attempts on other candidates.
    /*! @brief Auto-discovery considers candidates up to this rank
        (default skips rank 2 = Bluetooth and other known noise). */
    uint8_t max_rank = 1;
    /*!
      @brief Drive a deterministic run-mode reset after opening a port.

      A plain `open()` leaves the DTR/RTS transition timing to the OS,
      and on ESP32-style auto-reset wiring (RTS->EN, DTR->IO0) that race
      occasionally drops the board into its ROM bootloader, where it
      never answers hello. With this enabled (default) the utility
      releases IO0 (DTR clear) and pulses EN (RTS assert -> release)
      after each open: the peer always reboots into the application.
      Best-effort — ports without modem lines (e.g. a pty) ignore it.
      Disable when the peer must not be reset by connecting.
     */
    bool hardware_reset = true;
    /*! @brief Optional progress hook, called once per candidate before probing. */
    using attempt_fn_t      = void (*)(void* ctx, const char* path);
    attempt_fn_t on_attempt = nullptr;
    void* on_attempt_ctx    = nullptr;
};

/*!
  @brief Establish a remote-bus session over a host serial port.

  With `opt.path` set, opens exactly that port; otherwise walks the
  `listSerialPorts` candidates (best first, up to `opt.max_rank`). Each
  candidate is opened (`ep.rx` config's baud_rate is the single source
  of the baud), the session is `reset()`, and hello is attempted; the
  first port that answers is the peer.

  Returns the capabilities on success (also cached in the session).
  Errors: `IO_ERROR` when no port could be opened (or none exists);
  `TIMEOUT_ERROR` when ports opened but nothing answered hello.
 */
::m5::hal::v1::result_t<::m5::hal::v1::remote::Capabilities> connectRemoteSerial(
    SerialRemoteEndpoint& ep, const ConnectOptions& opt = ConnectOptions{});

}  // namespace m5::variants::frameworks::posix::hal::v1::uart

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
