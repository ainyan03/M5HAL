// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_REMOTE_HPP_
#define M5_HAL_REMOTE_REMOTE_HPP_

#include "../assert.hpp"
#include "../bytecode/bytecode.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../frame/frame.hpp"
#include "../gpio/gpio.hpp"
#include "../i2c/i2c.hpp"
#include "../i2s/i2s.hpp"
#include "../spi/spi.hpp"
#include "../uart/uart.hpp"

#include <M5Utility.hpp>

#include <stddef.h>
#include <stdint.h>

// =============================================================================
// Remote script execution support shared by the mux remote transport.
// Server executes bytecode request scripts on a BytecodeRunner and writes
// response scripts. What is registered on the server's runner is exactly
// what a remote peer can reach (registration = allowlist).
// =============================================================================

/*!
  @namespace m5::hal::v2::remote
  @brief Remote server endpoint and shared message constants.
 */
namespace m5::hal::v2::remote {

constexpr uint8_t kProtocolVersion = 1;  ///< Hello/HelloResp proto_ver.

constexpr size_t kMaxScriptSize = frame::kMaxPayload;  ///< 252. Max bytecode payload per frame.
/*! @brief Guaranteed per-transfer receive-data limit (response script overhead subtracted).
 *
 *  Derivation (checked by the static_assert below):
 *    A response script carrying one StoreData + ReportComplete + terminator uses:
 *      StoreData      = LenVar(1+1+N) + opcode(1) + store_id(1) + data(N)
 *                     = 1 + 2 + N      (LenVar is 1 byte when 2+N <= 0xFC, i.e. N <= 250)
 *      ReportComplete = LenVar(2) + opcode(1) + status(1) = 3
 *      terminator     = 1
 *    Total overhead   = 3 + 3 + 1 = 7  (8 with the 1-byte safety margin chosen here)
 *    => kMaxTransferRx = kMaxScriptSize - 8 = 252 - 8 = 244
 */
constexpr size_t kMaxTransferRx = 244;
static_assert(kMaxTransferRx + 8 <= kMaxScriptSize,
              "kMaxTransferRx leaves insufficient room for response script overhead "
              "(StoreData header + ReportComplete + terminator = 7 bytes; 8 chosen for margin)");
/*!
  @brief Default store slot for response data.

  Used as the `store_id` argument wherever a proxy bus encodes a single
  receive buffer into the response script. Distinguished from
  `bytecode::kDiscardStoreId` (0xFF), which signals "do not store".
  Every `request()` call resets the runner's store slots before decoding
  the response, so slot 0 is always fresh after each round trip.
 */
constexpr uint8_t kDefaultStoreId = 0;
/*! @brief Round-trip margin added on top of remote-side UART timeouts (spec §UART proxy). */
constexpr uint32_t kRemoteUartTimeoutMarginMs = 250;

namespace detail {

constexpr uint32_t saturatingAddU32(uint32_t lhs, uint32_t rhs)
{
    return (UINT32_MAX - lhs < rhs) ? UINT32_MAX : static_cast<uint32_t>(lhs + rhs);
}

constexpr uint32_t saturatingMulU32(uint32_t lhs, uint32_t rhs)
{
    return (rhs != 0 && lhs > UINT32_MAX / rhs) ? UINT32_MAX : static_cast<uint32_t>(lhs * rhs);
}

constexpr uint32_t clampBelowForever(uint32_t v)
{
    return v == types::TIMEOUT_FOREVER ? types::TIMEOUT_FOREVER - 1 : v;
}

constexpr uint32_t remoteUartWriteResponseTimeoutMs(uint32_t write_timeout_ms)
{
    return clampBelowForever(saturatingAddU32(write_timeout_ms, kRemoteUartTimeoutMarginMs));
}

constexpr uint32_t remoteUartReadResponseTimeoutMs(uint32_t first_byte_timeout_ms, uint32_t inter_byte_timeout_ms,
                                                   size_t rx_len)
{
    const uint32_t gaps = rx_len != 0 ? saturatingMulU32(static_cast<uint32_t>(rx_len - 1), inter_byte_timeout_ms) : 0;
    return clampBelowForever(
        saturatingAddU32(saturatingAddU32(first_byte_timeout_ms, gaps), kRemoteUartTimeoutMarginMs));
}

}  // namespace detail

/*!
  @brief Map a remote-reported i8 error code into the local error_t.

  Codes this build does not know fold into `REMOTE_FAULT`, so a newer
  server never breaks an older host (spec §error mapping).
 */
constexpr m5::hal::v2::error::error_t mapRemoteError(int8_t code)
{
    return (code >= static_cast<int8_t>(m5::hal::v2::error::error_t::NOT_CONNECTED) &&
            code <= static_cast<int8_t>(m5::hal::v2::error::error_t::ASYNC_RUNNING))
               ? static_cast<m5::hal::v2::error::error_t>(code)
               : m5::hal::v2::error::error_t::REMOTE_FAULT;
}

struct DeviceConfig {
    uint32_t baud_rate                      = 3000000;
    uint32_t response_timeout_ms            = 2000;
    void (*on_progress)(void*, const char*) = nullptr;
    void* progress_ctx                      = nullptr;
};

/*! @brief Capability summary carried by `HelloResp`. */
struct Capabilities {
    struct BusEntry {
        types::bus_kind_t kind = types::bus_kind_t::Unknown;
        uint8_t bus_id         = 0;
    };
    static constexpr size_t kMaxEntries = 4 * bytecode::kMaxBusBindings;

    uint8_t proto_ver        = 0;
    bool has_gpio            = false;
    bool supports_bus_create = false;
    size_t bus_count         = 0;
    BusEntry buses[kMaxEntries];
    uint8_t gpio_port_count = 0;
    uint16_t gpio_pin_count = 0;
};

}  // namespace m5::hal::v2::remote

#endif
