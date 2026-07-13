// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DATA_STDIO_HPP_
#define M5_HAL_DATA_STDIO_HPP_

#include "../data.hpp"

#include <cstdio>
#include <cstring>

#if defined(ESP_PLATFORM)
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>
#elif defined(_WIN32)
// Windows: TODO if ever needed
#else
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>
#endif

namespace m5::hal::v2::data {

/*!
  @brief Source backed by a C FILE* (default: stdin).

  On ESP32, the VFS maps stdin to the configured console transport
  (UART0, USB-Serial-JTAG, or USB OTG CDC) — so this single Source
  works across all console transports without transport-specific code.

  peek() attempts a non-blocking read to fill the internal buffer,
  returning whatever is available. A zero-length return means no data
  yet (not end-of-stream). Uses select() on the underlying fd for
  non-blocking detection.
 */
class StdioSource : public Source {
public:
    explicit StdioSource(FILE* f = stdin) : _file{f}
    {
    }

    result_t<ConstDataSpan> peek(size_t max_len) override
    {
        compact();
        if (_used < kCapacity) {
            int fd = fileno(_file);
            if (fd >= 0 && readable(fd)) {
                size_t room = kCapacity - _used;
                size_t want = room < max_len ? room : max_len;
                ssize_t n   = ::read(fd, _buf + _used, want);
                if (n > 0) {
                    _used += static_cast<size_t>(n);
                } else if (n == 0) {
                    _eof = true;
                }
            }
        }
        size_t avail = _used < max_len ? _used : max_len;
        return ConstDataSpan{_buf, avail};
    }

    /*!
      @brief Advance the read cursor by up to `N` bytes.

      Clamped to the bytes currently buffered (`_used - _cursor`): asking
      to skip more than that is not a contract violation (spec/design/
      data_io.md §Source documents over-skip as a supported usage), but
      unlike `StreamSource` (which queues the excess as a pending skip to
      consume on arrival) or a memory-backed Source (which would move to
      end-of-stream), the excess here is silently dropped and `eof()`
      does not become true — a not-yet-arrived byte on a live console fd
      may simply not exist yet, so there is nothing to queue against.
     */
    result_t<void> advance(size_t N) override
    {
        size_t remain = _used - _cursor;
        size_t skip   = N < remain ? N : remain;
        _cursor += skip;
        return {};
    }

    bool eof() const override
    {
        return _eof && (_cursor >= _used);
    }

    bool closed() const override
    {
        return _eof;
    }

private:
    static bool readable(int fd)
    {
#if !defined(_WIN32)
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        struct timeval tv = {0, 0};
        return ::select(fd + 1, &fds, nullptr, nullptr, &tv) > 0;
#else
        (void)fd;
        return true;
#endif
    }

    void compact()
    {
        if (_cursor > 0) {
            size_t remain = _used - _cursor;
            if (remain > 0) {
                ::memmove(_buf, _buf + _cursor, remain);
            }
            _used   = remain;
            _cursor = 0;
        }
    }

    static constexpr size_t kCapacity = 512;

    FILE* _file    = nullptr;
    size_t _cursor = 0;
    size_t _used   = 0;
    bool _eof      = false;
    uint8_t _buf[kCapacity];
};

/*!
  @brief Sink backed by a C FILE* (default: stdout).

  On ESP32, stdout is mapped via VFS to the console transport.
  reserve() lends out an internal buffer; commit() flushes the
  committed bytes to the FILE* immediately (write-through).
 */
class StdioSink : public Sink {
public:
    explicit StdioSink(FILE* f = stdout) : _file{f}
    {
    }

    result_t<DataSpan> reserve(size_t max_len) override
    {
        size_t take = max_len < kCapacity ? max_len : kCapacity;
        return DataSpan{_buf, take};
    }

    result_t<void> commit(size_t N) override
    {
        _partial    = 0;
        size_t todo = N < kCapacity ? N : kCapacity;
        if (todo == 0 || _file == nullptr) {
            return {};
        }
        // Acceptance boundary is the FILE* stream: bytes fwrite() took are
        // accepted even if the subsequent flush fails (the FILE abstraction
        // cannot recount bytes lost past its own buffer). A short fwrite()
        // reports the accepted prefix via partialCommitAccepted() so a
        // driving caller advances upstream by exactly what went through.
        size_t written = ::fwrite(_buf, 1, todo, _file);
        int flushed    = ::fflush(_file);
        if (written < todo || flushed != 0) {
            _partial = written;
            return m5::stl::make_unexpected(error::error_t::IO_ERROR);
        }
        return {};
    }

    size_t partialCommitAccepted() const override
    {
        return _partial;
    }

    bool closed() const override
    {
        return _file == nullptr;
    }

private:
    static constexpr size_t kCapacity = 512;

    FILE* _file     = nullptr;
    size_t _partial = 0;
    uint8_t _buf[kCapacity];
};

}  // namespace m5::hal::v2::data

#endif  // M5_HAL_DATA_STDIO_HPP_
