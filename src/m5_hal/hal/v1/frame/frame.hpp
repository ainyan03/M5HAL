// SPDX-License-Identifier: MIT
#ifndef M5_HAL_FRAME_FRAME_HPP_
#define M5_HAL_FRAME_FRAME_HPP_

#include "../data.hpp"

#include <M5Utility.hpp>

#include <stddef.h>
#include <stdint.h>

// =============================================================================
// Byte-stream framing codec. The authoritative spec lives in
// spec/design/frame.md (wire format, resync semantics, kind reservation).
//
// Wire format ("M5HAL frame v1"):
//   checked frame : [SIZE:1][KIND:1][CHECK16:2 BE][KIND_BODY:SIZE-2]
//   padding       : [0x00]
//   delimiter     : [0x00][0x55]   (no check; resync boundary)
//
// SIZE counts every byte after KIND (check + kind body). CHECK16 is
// CRC16-CCITT (poly 0x1021, init 0xFFFF) over the whole wire frame with
// the two check bytes treated as zero (self-verifying).
//
// The pure codec (encode* / decode) is stateless and span-based: it
// performs no I/O and is the unit-testable core. FrameReader /
// FrameWriter lift it onto the Source / Sink stream model so that, for
// example, a UART RX accessor wrapped in a StreamSource yields decoded
// frames directly.
// =============================================================================

/*!
  @namespace m5::hal::v1::frame
  @brief Byte-stream framing: wire codec plus Source/Sink-driven reader/writer.
 */
namespace m5::hal::v1::frame {

constexpr size_t kPrefixSize     = 2;    ///< SIZE + KIND.
constexpr size_t kCheckSize      = 2;    ///< CHECK16.
constexpr size_t kMaxBodySize    = 255;  ///< Max value of SIZE (check + kind body).
constexpr size_t kMaxWireSize    = kPrefixSize + kMaxBodySize;
constexpr size_t kMaxDataPayload = 240;  ///< Max stream payload in one data frame.

/*!
  @brief Frame kind byte.

  Only `data` (kind body = [stream_id:1][payload]) plus the
  padding / delimiter markers are interpreted by this codec. The
  remaining values are reserved for the (future) mux / transport layer;
  the codec validates and transports their frames without interpreting
  the kind body.
 */
enum class Kind : uint8_t {
    padding      = 0x00,
    data         = 0x01,
    credit       = 0x03,
    checkpoint   = 0x05,
    control      = 0x07,
    negotiate    = 0x09,
    management   = 0x0B,
    credit_delta = 0x0D,
    delimiter    = 0x55,
};

enum class DecodeStatus : uint8_t {
    ok,              ///< One frame decoded; `consumed` bytes used.
    need_more,       ///< Not enough bytes yet; `consumed` is 0.
    padding,         ///< One padding byte skipped.
    invalid_prefix,  ///< Unknown kind; its self-described size was discarded.
    invalid_size,    ///< Known kind with an impossible SIZE; frame discarded.
    invalid_check,   ///< CHECK16 mismatch; frame discarded.
};

/*!
  @brief Borrowed view of one decoded frame.

  All spans point into the caller's input span (decode) or into the
  Source's borrowed peek span (FrameReader) - never owned.
 */
struct View {
    Kind kind = Kind::padding;

    /// Bytes after KIND (includes CHECK16 for checked frames).
    data::ConstDataSpan body{};

    /// Bytes after CHECK16 (the kind-specific payload).
    data::ConstDataSpan kind_body{};

    uint16_t frame_check16 = 0;
    bool has_check         = false;
};

struct DecodeResult {
    DecodeStatus status = DecodeStatus::need_more;
    size_t consumed     = 0;
};

constexpr bool isCheckedKind(Kind kind)
{
    return kind != Kind::padding && kind != Kind::delimiter;
}

constexpr bool isKnownKind(uint8_t kind)
{
    return kind == static_cast<uint8_t>(Kind::data) || kind == static_cast<uint8_t>(Kind::credit) ||
           kind == static_cast<uint8_t>(Kind::checkpoint) || kind == static_cast<uint8_t>(Kind::control) ||
           kind == static_cast<uint8_t>(Kind::negotiate) || kind == static_cast<uint8_t>(Kind::management) ||
           kind == static_cast<uint8_t>(Kind::credit_delta) || kind == static_cast<uint8_t>(Kind::delimiter);
}

constexpr bool isDelimiter(data::ConstDataSpan src)
{
    return src.size >= 2 && src.data[0] == 0x00 && src.data[1] == static_cast<uint8_t>(Kind::delimiter);
}

constexpr size_t checkedFrameWireSize(size_t kind_body_size)
{
    return kPrefixSize + kCheckSize + kind_body_size;
}

/*! @brief One step of CRC16-CCITT (poly 0x1021); seed with 0xFFFF. */
uint16_t crc16CcittUpdate(uint16_t crc, uint8_t data);

/*! @brief CHECK16 over a full wire frame (the two check bytes are
    treated as zero, so the stored value verifies itself). */
uint16_t check16(data::ConstDataSpan wire_frame);

// ---- pure codec -------------------------------------------------------------
//
// Encoders return the number of bytes written into `dst`.
// Errors: INVALID_ARGUMENT (bad kind / oversized or null body),
// BUFFER_OVERFLOW (`dst` cannot hold the frame). No partial output is
// produced on error.

m5::hal::v1::result_t<size_t> encodeDelimiter(data::DataSpan dst);
m5::hal::v1::result_t<size_t> encodeChecked(data::DataSpan dst, Kind kind, data::ConstDataSpan kind_body);
m5::hal::v1::result_t<size_t> encodeData(data::DataSpan dst, uint8_t stream_id, data::ConstDataSpan stream_data);

/*!
  @brief Try to decode one frame from the head of `src`.

  Non-`ok` outcomes are part of the normal stream flow (padding skip,
  resync after corruption), so the result is a status + consumed count
  rather than an error path. `consumed` is how many bytes the caller
  must discard before retrying; it is 0 only for `need_more`.
 */
DecodeResult decode(data::ConstDataSpan src, View& view);

// ---- Source / Sink integration ----------------------------------------------

/*!
  @brief Pulls decoded frames out of a `data::Source`.

  Composes with the Stream adapters: wrap a UART RX accessor in a
  `data::StreamSource` and FrameReader yields its frames without an
  extra copy (the View borrows the Source's peek span).

  Requirements and semantics:
  - Acquisition is two-stage: the reader peeks the 2-byte prefix,
    derives the frame size, and then requests exactly that many bytes.
    An already-buffered frame therefore decodes immediately; a
    StreamSource blocks (up to its timeouts) only when the frame is
    genuinely incomplete.
  - The Source must be able to lend each frame's wire size in one
    `peek` - up to `kMaxWireSize` (257) for maximum-size frames, so a
    `StreamSource` scratch of 257 bytes always suffices. A frame larger
    than what the Source can lend keeps returning `need_more`.
  - On `ok` the consumed bytes are NOT advanced yet: the returned View
    borrows the Source's peek span, which the Source contract keeps
    alive until the next peek/advance. The reader advances them at the
    START of the next `next()` call, so a View stays valid until then
    (and must not be used after).
  - `padding` bytes are skipped internally; `invalid_*` outcomes are
    surfaced (already advanced past) so the caller can count or log
    resync events.
  - Source errors pass through unchanged: `TIMEOUT_ERROR` from a
    StreamSource means "no bytes arrived in time - call again";
    `END_OF_STREAM` is returned when the Source reports EOF (empty
    peek), e.g. a fully drained MemorySource.
  - A short peek on a CLOSED Source (producer done - the bytes can
    never grow into a frame) also returns `END_OF_STREAM` instead of
    `need_more`, so pumping a finite Source with a truncated tail
    terminates instead of livelocking. The partial tail stays
    unconsumed; `source.eof() == false` afterwards tells the caller
    the stream ended mid-frame.
 */
class FrameReader {
public:
    explicit FrameReader(data::Source& source) : _source{&source}
    {
    }

    m5::hal::v1::result_t<DecodeResult> next(View& view);

private:
    data::Source* _source = nullptr;
    size_t _pending       = 0;  // bytes of the handed-out frame, advanced on the next call
};

/*!
  @brief Writes encoded frames into a `data::Sink`.

  Encodes directly into the Sink's reserved span (one copy total).
  Composes with the Stream adapters: wrap a UART TX accessor in a
  `data::StreamSink` (scratch >= the largest frame to be written) and
  each write* call transmits one frame.

  Errors: argument problems are INVALID_ARGUMENT; a Sink that cannot
  lend the frame size yields CLOSED (sink closed) or BUFFER_OVERFLOW;
  Sink reserve/commit errors pass through unchanged.
 */
class FrameWriter {
public:
    explicit FrameWriter(data::Sink& sink) : _sink{&sink}
    {
    }

    m5::hal::v1::result_t<size_t> writeDelimiter(void);
    m5::hal::v1::result_t<size_t> writeChecked(Kind kind, data::ConstDataSpan kind_body);
    m5::hal::v1::result_t<size_t> writeData(uint8_t stream_id, data::ConstDataSpan stream_data);

private:
    m5::hal::v1::result_t<size_t> reserveExact(size_t need, data::DataSpan& out);

    data::Sink* _sink = nullptr;
};

}  // namespace m5::hal::v1::frame

#endif
