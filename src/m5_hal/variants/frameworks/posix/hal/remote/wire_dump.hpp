// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_WIRE_DUMP_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_WIRE_DUMP_HPP

// Wire byte capture for host-side remote transports. Enabled at runtime by
// M5HAL_WIRE_DUMP=<path> (opened in append mode). Deliberately not build-flag
// gated: posix transports are host-only and the value of the env gate is
// enabling capture on an existing binary without a rebuild.
//
// WireDumpWriter is a StreamWriter that formats bytes into hex-dump records
// and is meant to be plugged into `data::TapReader` / `data::TapWriter`
// (hal/v2/data/tap.hpp) as the mirror target. With WireDump::fromEnv()
// returning nullptr (the common case, no env var set), the whole chain costs
// two extra virtual calls and nothing else - no formatting, no file I/O,
// no locking.

#include "../../../../../hal/v2/data/stream.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <time.h>

#if M5HAL_FRAMEWORK_HAS_POSIX

namespace m5::variants::frameworks::posix::hal::v2::remote {

namespace m5hal = ::m5::hal::v2;

/*!
  @brief Append-only byte-level log for a host-side remote transport.

  One text line per `writeRecord` call:
  `<monotonic-sec> <tag> <dir> <len> <hex bytes...>\n`, where `dir` is
  `<` (device to host) or `>` (host to device). The payload is hex-encoded
  in full - never truncated - because diagnosing a wire issue needs the
  complete record, long lines and all.
 */
class WireDump {
public:
    /*!
      @brief Process-wide instance selected by the M5HAL_WIRE_DUMP
             environment variable, evaluated once on first call.
      @return nullptr when the variable is unset, or when the path could
              not be opened (an open failure warns once on stderr).
     */
    static WireDump* fromEnv()
    {
        static WireDump* instance = createFromEnv();
        return instance;
    }

    //! For tests: open an explicit path (append mode).
    explicit WireDump(const char* path) : _file(std::fopen(path, "a"))
    {
    }

    ~WireDump()
    {
        if (_file != nullptr) {
            std::fclose(_file);
        }
    }

    WireDump(const WireDump&)            = delete;
    WireDump& operator=(const WireDump&) = delete;

    bool ok() const
    {
        return _file != nullptr;
    }

    /*!
      @brief Append one record and flush immediately.
      @param tag  Transport tag ("uart" / "tcp"), fixed per connection.
      @param dir  '<' = device to host, '>' = host to device.
      @param data First `len` bytes are hex-encoded, space-separated.

      Serialized by an internal mutex: multiple connections (hence
      multiple taps) can share one process and one dump file.
     */
    void writeRecord(const char* tag, char dir, const uint8_t* data, size_t len)
    {
        if (_file == nullptr) {
            return;
        }
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        const double t_sec = static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) / 1e9;

        std::lock_guard<std::mutex> lock(_mutex);
        std::fprintf(_file, "%.6f %s %c %zu", t_sec, tag, dir, len);
        for (size_t i = 0; i < len; ++i) {
            std::fprintf(_file, " %02x", data[i]);
        }
        std::fputc('\n', _file);
        std::fflush(_file);
    }

private:
    static WireDump* createFromEnv()
    {
        const char* path = std::getenv("M5HAL_WIRE_DUMP");
        if (path == nullptr || path[0] == '\0') {
            return nullptr;
        }
        auto* dump = new WireDump(path);
        if (!dump->ok()) {
            std::fprintf(stderr, "[WireDump] failed to open '%s' (M5HAL_WIRE_DUMP); wire capture disabled\n", path);
            delete dump;
            return nullptr;
        }
        return dump;
    }

    FILE* _file = nullptr;
    std::mutex _mutex;
};

/*!
  @brief `StreamWriter` that formats bytes into a `WireDump` record.

  Meant as the `mirror` target of a `data::TapReader` / `data::TapWriter`
  (hal/v2/data/tap.hpp). No-op when `dump == nullptr`; otherwise forwards
  the full span to `dump->writeRecord(tag, dir, ...)`. Always reports the
  full input size accepted and never an error: a mirror observes, it does
  not gate the primary stream (tap ignores this return value anyway).
 */
class WireDumpWriter : public m5hal::data::StreamWriter {
public:
    WireDumpWriter(WireDump* dump, const char* tag, char dir) : _dump(dump), _tag(tag), _dir(dir)
    {
    }

    m5hal::result_t<size_t> write(m5hal::data::ConstDataSpan src) override
    {
        if (_dump != nullptr) {
            _dump->writeRecord(_tag, _dir, src.data, src.size);
        }
        return src.size;
    }

private:
    WireDump* _dump;
    const char* _tag;
    char _dir;
};

}  // namespace m5::variants::frameworks::posix::hal::v2::remote

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
