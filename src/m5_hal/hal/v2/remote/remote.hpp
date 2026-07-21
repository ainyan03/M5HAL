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
/*! @brief Maximum inline TX bytes in one atomic I2C transfer before its prefix is subtracted. */
constexpr size_t kMaxAtomicI2CTxBase = 243;
/*! @brief Maximum inline TX bytes in one atomic SPI transfer. */
constexpr size_t kMaxAtomicSPITx = 229;
/*!
  @brief Default store slot for response data.

  Used as the `store_id` argument wherever a proxy bus encodes a single
  receive buffer into the response script. Distinguished from
  `bytecode::kDiscardStoreId` (0xFF), which signals "do not store".
  Every `request()` call resets the runner's store slots before decoding
  the response, so slot 0 is always fresh after each round trip.
 */
constexpr uint8_t kDefaultStoreId = 0;
/*! @brief Round-trip margin added on top of every derived remote response timeout (spec §UART proxy). */
constexpr uint32_t kRemoteTimeoutMarginMs = 250;

namespace detail {

constexpr uint32_t saturatingAddU32(uint32_t lhs, uint32_t rhs)
{
    return (UINT32_MAX - lhs < rhs) ? UINT32_MAX : static_cast<uint32_t>(lhs + rhs);
}

constexpr uint32_t saturatingMulU32(uint32_t lhs, uint32_t rhs)
{
    return (rhs != 0 && lhs > UINT32_MAX / rhs) ? UINT32_MAX : static_cast<uint32_t>(lhs * rhs);
}

constexpr uint64_t saturatingAddU64(uint64_t lhs, uint64_t rhs)
{
    return (UINT64_MAX - lhs < rhs) ? UINT64_MAX : static_cast<uint64_t>(lhs + rhs);
}

constexpr uint64_t saturatingMulU64(uint64_t lhs, uint64_t rhs)
{
    return (rhs != 0 && lhs > UINT64_MAX / rhs) ? UINT64_MAX : static_cast<uint64_t>(lhs * rhs);
}

constexpr uint32_t clampBelowForever(uint32_t v)
{
    return v == types::TIMEOUT_FOREVER ? types::TIMEOUT_FOREVER - 1 : v;
}

constexpr uint32_t remoteUartWriteResponseTimeoutMs(uint32_t write_timeout_ms, size_t tx_len)
{
    const uint32_t nominal = saturatingMulU32(static_cast<uint32_t>(tx_len), write_timeout_ms);
    return clampBelowForever(saturatingAddU32(nominal, kRemoteTimeoutMarginMs));
}

constexpr uint32_t remoteUartReadResponseTimeoutMs(uint32_t first_byte_timeout_ms, uint32_t inter_byte_timeout_ms,
                                                   size_t rx_len)
{
    const uint32_t gaps = rx_len != 0 ? saturatingMulU32(static_cast<uint32_t>(rx_len - 1), inter_byte_timeout_ms) : 0;
    return clampBelowForever(saturatingAddU32(saturatingAddU32(first_byte_timeout_ms, gaps), kRemoteTimeoutMarginMs));
}

constexpr uint32_t remoteUartTransferResponseTimeoutMs(uint32_t write_timeout_ms, uint32_t first_byte_timeout_ms,
                                                       uint32_t inter_byte_timeout_ms, size_t tx_len, size_t rx_len)
{
    uint32_t nominal = 0;
    if (tx_len != 0) {
        nominal = saturatingAddU32(nominal, saturatingMulU32(static_cast<uint32_t>(tx_len), write_timeout_ms));
    }
    if (rx_len != 0) {
        const uint32_t gaps = saturatingMulU32(static_cast<uint32_t>(rx_len - 1), inter_byte_timeout_ms);
        nominal             = saturatingAddU32(nominal, saturatingAddU32(first_byte_timeout_ms, gaps));
    }
    return clampBelowForever(saturatingAddU32(nominal, kRemoteTimeoutMarginMs));
}

/*! @brief Expected on-wire duration of `bits` in ms (ceil, saturating). */
constexpr uint32_t remoteWireBitDurationMs(uint64_t bits, uint32_t clock_hz)
{
    if (clock_hz == 0) {
        return 0;
    }
    const uint64_t whole_seconds = bits / clock_hz;
    if (whole_seconds > UINT32_MAX / 1000u) {
        return UINT32_MAX;
    }
    const uint64_t whole_ms   = whole_seconds * 1000u;
    const uint64_t remainder  = bits % clock_hz;
    const uint64_t partial_ms = (remainder * 1000u + clock_hz - 1) / clock_hz;
    return UINT32_MAX - whole_ms < partial_ms ? UINT32_MAX : static_cast<uint32_t>(whole_ms + partial_ms);
}

/*! @brief Expected on-wire duration of `bytes` at `bits_per_byte` bits each, in ms (ceil, saturating). */
constexpr uint32_t remoteWireDurationMs(size_t bytes, uint32_t bits_per_byte, uint32_t clock_hz)
{
    const uint64_t bits = saturatingMulU64(static_cast<uint64_t>(bytes), bits_per_byte);
    return remoteWireBitDurationMs(bits, clock_hz);
}

/*!
  @brief I2C proxy response timeout: expected wire time (9 bits/byte: 8 data
  + ACK) plus the stall allowance the server folds into its transaction
  budget (`wire_timeout_ms`), plus the round-trip margin.
 */
constexpr uint32_t remoteI2cResponseTimeoutMs(uint32_t freq, uint32_t wire_timeout_ms, size_t total_bytes)
{
    const uint32_t wire = remoteWireDurationMs(total_bytes, 9, freq);
    return clampBelowForever(saturatingAddU32(saturatingAddU32(wire, wire_timeout_ms), kRemoteTimeoutMarginMs));
}

/*!
  @brief SPI proxy response timeout: all descriptor phases plus payload wire
  time (worst case: half-duplex tx+rx), then the round-trip margin.
 */
constexpr uint32_t remoteSpiResponseTimeoutMs(uint32_t freq, size_t tx_len, size_t rx_len, uint8_t command_bytes,
                                              uint8_t address_bytes, uint8_t dummy_cycles)
{
    const uint64_t payload_bytes = saturatingAddU64(static_cast<uint64_t>(tx_len), static_cast<uint64_t>(rx_len));
    const uint64_t phase_bytes   = saturatingAddU64(command_bytes, address_bytes);
    uint64_t total_bits          = saturatingMulU64(saturatingAddU64(payload_bytes, phase_bytes), 8u);
    total_bits                   = saturatingAddU64(total_bits, dummy_cycles);
    return clampBelowForever(saturatingAddU32(remoteWireBitDurationMs(total_bits, freq), kRemoteTimeoutMarginMs));
}

/*! @brief PCM playback/capture duration of `bytes` at the configured shape, in ms (ceil, saturating). */
constexpr uint32_t remotePcmDurationMs(uint32_t sample_rate_hz, uint32_t bits_per_sample, uint32_t channels,
                                       size_t bytes)
{
    if (sample_rate_hz == 0) {
        return 0;
    }
    uint32_t frame_bytes = channels * (bits_per_sample / 8u);
    if (frame_bytes == 0) {
        frame_bytes = 1;
    }
    const uint64_t frames = (static_cast<uint64_t>(bytes) + frame_bytes - 1) / frame_bytes;
    const uint64_t ms     = (frames * 1000u + sample_rate_hz - 1) / sample_rate_hz;
    return ms > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(ms);
}

/*!
  @brief I2S/PDM proxy response timeout: real-time duration of the PCM data
  plus the configured DMA wait budget, plus the round-trip margin.

  Assumes the PCM payload rate stays below the session transport rate
  (true for the supported 16-bit shapes up to 48 kHz stereo over the 3 Mbaud
  default); revisit if faster shapes are added.
 */
constexpr uint32_t remotePcmResponseTimeoutMs(uint32_t sample_rate_hz, uint32_t bits_per_sample, uint32_t channels,
                                              uint32_t dma_timeout_ms, size_t bytes)
{
    const uint32_t duration = remotePcmDurationMs(sample_rate_hz, bits_per_sample, channels, bytes);
    return clampBelowForever(saturatingAddU32(saturatingAddU32(duration, dma_timeout_ms), kRemoteTimeoutMarginMs));
}

}  // namespace detail

/*!
  @brief Map a remote-reported i8 error code into the local error_t.

  Codes this build does not know fold into `REMOTE_FAULT`, so a newer
  server never breaks an older host (spec §error mapping).
 */
constexpr m5::hal::v2::error::error_t mapRemoteError(int8_t code)
{
    return (code >= static_cast<int8_t>(m5::hal::v2::error::error_t::WOULD_BLOCK) &&
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

constexpr uint8_t kHelloFlagGpio                 = 0x01u;
constexpr uint8_t kHelloFlagBusCreate            = 0x02u;
constexpr uint8_t kHelloFlagBusCapabilities      = 0x04u;
constexpr uint8_t kHelloExtensionBusCapabilities = 0x01u;

/*! @brief Capability summary carried by `HelloResp`. */
struct Capabilities {
    struct BusEntry {
        types::bus_kind_t kind = types::bus_kind_t::Unknown;
        uint8_t bus_id         = 0;
        bus::BusCapabilities capabilities{};
    };
    static constexpr size_t kMaxEntries = 4 * bytecode::kMaxBusBindings;

    uint8_t proto_ver         = 0;
    bool has_gpio             = false;
    bool supports_bus_create  = false;
    bool has_bus_capabilities = false;
    size_t bus_count          = 0;
    BusEntry buses[kMaxEntries];
    uint8_t gpio_port_count = 0;
    uint16_t gpio_pin_count = 0;
};

}  // namespace m5::hal::v2::remote

#endif
