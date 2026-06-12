// SPDX-License-Identifier: MIT
#ifndef M5_HAL_ERROR_HPP
#define M5_HAL_ERROR_HPP

#include <m5_utility/stl/expected.hpp>

#include <stdint.h>

/*!
  @namespace m5::hal::v1::error
  @brief Error codes returned by v1 HAL APIs via `result_t<T>`.
 */
namespace m5::hal::v1::error {

/*!
  @brief Error codes returned by v1 HAL APIs.

  Negative values indicate failures. `OK` (0) and `ASYNC_RUNNING` (1) are
  non-failure states. UPPER_SNAKE_CASE follows the POSIX-style convention
  documented in coding_style.md §Naming.

  The `int8_t` underlying type is load-bearing: the bytecode wire
  format (spec/design/bytecode.md, `report_error` / `report_complete`)
  carries error codes as a single i8 byte. Do not widen the type or
  add values outside -128..127.
 */
enum class ErrorType : int8_t {
    ASYNC_RUNNING    = 1,    ///< Operation accepted and still running asynchronously.
    OK               = 0,    ///< Success.
    UNKNOWN_ERROR    = -1,   ///< Unclassified failure.
    TIMEOUT_ERROR    = -2,   ///< Operation timed out before completion.
    INVALID_ARGUMENT = -3,   ///< A caller-supplied argument violated the API contract.
    NOT_IMPLEMENTED  = -4,   ///< The operation is not implemented by the current variant.
    I2C_BUS_ERROR    = -5,   ///< Low-level I2C bus error (arbitration loss, stretch timeout, etc).
    I2C_NO_ACK       = -6,   ///< Addressed I2C target did not acknowledge.
    BUSY             = -7,   ///< Resource is currently locked by another owner.
    IO_ERROR         = -8,   ///< Generic transport / OS / driver I/O failure.
    CLOSED           = -9,   ///< Resource, stream, or endpoint is closed.
    OUT_OF_RESOURCE  = -10,  ///< Fixed storage, handle slots, or other bounded resources are exhausted.
    BUFFER_OVERFLOW  = -11,  ///< Producer wrote or received more data than the destination can hold.
    BUFFER_UNDERFLOW = -12,  ///< Consumer requested data that is not currently available.
    END_OF_STREAM    = -13,  ///< Stream reached a clean end condition.
    CHECKSUM_ERROR   = -14,  ///< Frame or payload integrity check failed.
    PROTOCOL_ERROR   = -15,  ///< Well-formed bytes violated the protocol state / semantics.
    DISCONNECTED     = -16,  ///< Transport to a remote endpoint was lost.
    REMOTE_FAULT     = -17,  ///< The remote endpoint reported an internal failure (or an unknown code).
    UNSUPPORTED      = -18,  ///< The remote endpoint does not support the requested capability.
    DEVICE_MISMATCH  = -19,  ///< A probed/identified device is not the one the caller expected (e.g. a chip-ID check
                             ///< failed). Reserved for driver layers; the HAL core never returns it.
};
using error_t = ErrorType;

// Internal helpers. Public APIs return `result_t<T>` so callers use
// `if (auto r = ...; !r) ...` uniformly; these are kept for variant
// implementations and test fixtures that handle a raw error_t.
constexpr bool isError(const error_t e)
{
    return e < error_t::OK;
}
constexpr bool isOk(const error_t e)
{
    return e == error_t::OK;
}

// Config knob (spec/design/configuration.md): the code->name table
// below is on by default; build with -DM5HAL_CONFIG_ERROR_STRINGS=0 to
// drop the string literals from builds that cannot spare the bytes.
#ifndef M5HAL_CONFIG_ERROR_STRINGS
#define M5HAL_CONFIG_ERROR_STRINGS 1
#endif

#if M5HAL_CONFIG_ERROR_STRINGS
/*!
  @brief Stable identifier name of an error code (for logs and
         diagnostics).

  Returns the enumerator's spelling (`"I2C_NO_ACK"`); unknown values —
  e.g. a wire-decoded byte from a newer peer — map to `"UNKNOWN"`.
  The strings are identifiers, not prose: they are stable enough to
  grep for, and the troubleshooting hints live in
  spec/design/errors.md (error hint table) rather than in RAM.
  Build with `M5HAL_CONFIG_ERROR_STRINGS=0` to compile the table out
  (the function then stays usable and returns `""` for every code).
 */
constexpr const char* toString(const error_t e)
{
    switch (e) {
        case error_t::ASYNC_RUNNING:
            return "ASYNC_RUNNING";
        case error_t::OK:
            return "OK";
        case error_t::UNKNOWN_ERROR:
            return "UNKNOWN_ERROR";
        case error_t::TIMEOUT_ERROR:
            return "TIMEOUT_ERROR";
        case error_t::INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";
        case error_t::NOT_IMPLEMENTED:
            return "NOT_IMPLEMENTED";
        case error_t::I2C_BUS_ERROR:
            return "I2C_BUS_ERROR";
        case error_t::I2C_NO_ACK:
            return "I2C_NO_ACK";
        case error_t::BUSY:
            return "BUSY";
        case error_t::IO_ERROR:
            return "IO_ERROR";
        case error_t::CLOSED:
            return "CLOSED";
        case error_t::OUT_OF_RESOURCE:
            return "OUT_OF_RESOURCE";
        case error_t::BUFFER_OVERFLOW:
            return "BUFFER_OVERFLOW";
        case error_t::BUFFER_UNDERFLOW:
            return "BUFFER_UNDERFLOW";
        case error_t::END_OF_STREAM:
            return "END_OF_STREAM";
        case error_t::CHECKSUM_ERROR:
            return "CHECKSUM_ERROR";
        case error_t::PROTOCOL_ERROR:
            return "PROTOCOL_ERROR";
        case error_t::DISCONNECTED:
            return "DISCONNECTED";
        case error_t::REMOTE_FAULT:
            return "REMOTE_FAULT";
        case error_t::UNSUPPORTED:
            return "UNSUPPORTED";
        case error_t::DEVICE_MISMATCH:
            return "DEVICE_MISMATCH";
    }
    return "UNKNOWN";
}
#else
constexpr const char* toString(const error_t)
{
    return "";
}
#endif  // M5HAL_CONFIG_ERROR_STRINGS

}  // namespace m5::hal::v1::error

namespace m5::hal::v1 {

/*!
  @brief Standard return type of v1 HAL APIs: a value of `T` or an
         `error::error_t`.

  Pure alias of `m5::stl::expected<T, error::error_t>` — the SAME type,
  not a derived one, so it interconverts freely with code (including
  M5UnitUnified) that spells the underlying type out, and a future move
  to `std::expected` stays a one-line change. The `_t` suffix follows
  the house typedef convention (`error_t`, `gpio_number_t`, ...) and
  avoids shadowing by the ubiquitous `auto result = ...` locals.
 */
template <typename T>
using result_t = m5::stl::expected<T, error::error_t>;

}  // namespace m5::hal::v1

#endif
