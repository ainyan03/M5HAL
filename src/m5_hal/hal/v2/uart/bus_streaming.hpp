// SPDX-License-Identifier: MIT
#ifndef M5_HAL_UART_BUS_STREAMING_HPP_
#define M5_HAL_UART_BUS_STREAMING_HPP_

#include "uart.hpp"

namespace m5::hal::v2::uart {

/*!
  @brief Base for stream-oriented uart::IBus implementations.

  Provides the Source/Sink loop + first_byte/inter_byte timeout logic
  that every streaming bus (POSIX fd, ESP-IDF UART, USB-JTAG, USB CDC,
  stdio, ...) shares. Subclasses implement only the raw byte I/O:

    rawWrite(data, len, timeout_ms) — push bytes to the transport
    rawRead(buf, len, timeout_ms)   — pull bytes from the transport
    rawReadableBytes()              — bytes available without blocking

  Raw I/O return-value contract (the base loop depends on this):
    - Success: number of bytes transferred (>0).
    - Zero:    no data could be transferred (base loop breaks normally).
    - Error:   make_unexpected(error_t). Timeouts that the caller must
               see MUST be returned as make_unexpected(TIMEOUT_ERROR),
               not as zero (zero means "no data right now", not "timed
               out"). Before any progress the base loop propagates the
               error. After progress it returns the accepted prefix as a
               short success so retry cannot duplicate those bytes.

  Subclasses may override write()/read()/readableBytes() when the
  default loop is insufficient (e.g. applyConfig, post-write drain
  wait, write coalescing).
 */
class Bus_streaming : public IBus {
public:
    result_t<size_t> write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* src, size_t len) override;
    result_t<size_t> read(bus::IAccessor* owner, const AccessConfig& cfg, data::Sink* dst, size_t len) override;
    result_t<size_t> readableBytes(bus::IAccessor* owner, const AccessConfig& cfg) override;

protected:
    // Merge a post-write completion/drain status with the accepted byte
    // count. result_t<size_t> cannot carry both; an accepted prefix wins so
    // callers have an unambiguous retry boundary. With zero accepted bytes,
    // the completion error remains visible.
    static result_t<size_t> completeWrite(result_t<size_t>&& accepted, error::error_t completion_error);

    virtual result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) = 0;
    virtual result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms)         = 0;
    virtual result_t<size_t> rawReadableBytes()                                             = 0;
};

}  // namespace m5::hal::v2::uart

#endif  // M5_HAL_UART_BUS_STREAMING_HPP_
