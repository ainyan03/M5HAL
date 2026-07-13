// SPDX-License-Identifier: MIT
#ifndef M5_HAL_FRAME_FRAME_HPP_
#define M5_HAL_FRAME_FRAME_HPP_

#include "../data.hpp"

#include <M5Utility.hpp>

#include <stddef.h>
#include <stdint.h>

// =============================================================================
// Byte-stream framing codec.
//
// Wire format:
//   checked frame : [LEN:1][KIND:1][CHECK8:1][B3:1][payload:0..252]
//   padding       : [0x00]
//   delimiter     : [0x00][0x55]
//
// LEN = bytes after KIND = CHECK8 + B3 + payload = 2 + payload_len.
// CHECK8 is CRC-8 ATM (poly 0x07, init 0x00) over (LEN, KIND).
// B3 is KIND-dependent (data -> stream_id, request -> seq, etc.).
// Max frame = 4 + 252 = 256 bytes = one TempBuffer block.
//
// The pure codec (encode* / decode) is stateless and span-based.
// FrameReader / FrameWriter lift it onto the Source / Sink stream model.
// =============================================================================

namespace m5::hal::v2::frame {

constexpr size_t kPrefixSize     = 2;    ///< LEN + KIND.
constexpr size_t kCheckSize      = 1;    ///< CHECK8.
constexpr size_t kHeaderSize     = 4;    ///< LEN + KIND + CHECK8 + B3.
constexpr size_t kPayloadOffset  = 4;    ///< Payload starts at offset 4.
constexpr size_t kB3Offset       = 3;    ///< B3 at offset 3.
constexpr size_t kMaxPayload     = 252;  ///< Max payload bytes.
constexpr size_t kMaxLen         = 254;  ///< Max LEN value (= 2 + 252).
constexpr size_t kMaxFrameSize   = 256;  ///< Max wire frame (= kHeaderSize + kMaxPayload).
constexpr size_t kMaxDataPayload = 252;  ///< Max stream data in one data frame (= kMaxPayload).
constexpr size_t kMinCheckedLen  = 2;    ///< Minimum valid LEN for a checked frame (CHECK8 + B3).

enum class Kind : uint8_t {
    Padding    = 0x00,
    Data       = 0x01,
    Credit     = 0x03,
    Checkpoint = 0x05,
    Control    = 0x07,
    Request    = 0x09,
    Response   = 0x0B,
    HelloReq   = 0x0D,
    HelloResp  = 0x0F,
    Ping       = 0x11,
    Pong       = 0x13,
    Event      = 0x15,
    Delimiter  = 0x55,
};

enum class DecodeStatus : uint8_t {
    Ok,
    NeedMore,
    Padding,
    Delimiter,
    InvalidPrefix,
    InvalidSize,
    InvalidCheck,
};

struct View {
    Kind kind     = Kind::Padding;
    uint8_t b3    = 0;
    uint8_t check = 0;
    data::ConstDataSpan payload{};
    bool has_check = false;
};

struct DecodeResult {
    DecodeStatus status = DecodeStatus::NeedMore;
    size_t consumed     = 0;
};

constexpr bool isCheckedKind(Kind kind)
{
    return kind != Kind::Padding && kind != Kind::Delimiter;
}

constexpr bool isKnownKind(uint8_t kind)
{
    switch (kind) {
        case static_cast<uint8_t>(Kind::Data):
        case static_cast<uint8_t>(Kind::Credit):
        case static_cast<uint8_t>(Kind::Checkpoint):
        case static_cast<uint8_t>(Kind::Control):
        case static_cast<uint8_t>(Kind::Request):
        case static_cast<uint8_t>(Kind::Response):
        case static_cast<uint8_t>(Kind::HelloReq):
        case static_cast<uint8_t>(Kind::HelloResp):
        case static_cast<uint8_t>(Kind::Ping):
        case static_cast<uint8_t>(Kind::Pong):
        case static_cast<uint8_t>(Kind::Event):
        case static_cast<uint8_t>(Kind::Delimiter):
            return true;
        default:
            return false;
    }
}

constexpr bool isDelimiter(data::ConstDataSpan src)
{
    return src.size >= 2 && src.data[0] == 0x00 && src.data[1] == static_cast<uint8_t>(Kind::Delimiter);
}

constexpr size_t checkedFrameWireSize(size_t payload_size)
{
    return kHeaderSize + payload_size;
}

uint8_t crc8AtmUpdate(uint8_t crc, uint8_t byte);
uint8_t check8(uint8_t len, uint8_t kind);

// ---- pure codec -------------------------------------------------------------

m5::hal::v2::result_t<size_t> encodeDelimiter(data::DataSpan dst);
m5::hal::v2::result_t<size_t> encodeChecked(data::DataSpan dst, Kind kind, uint8_t b3, data::ConstDataSpan payload);
m5::hal::v2::result_t<size_t> encodeData(data::DataSpan dst, uint8_t stream_id, data::ConstDataSpan stream_data);

DecodeResult decode(data::ConstDataSpan src, View& view);

// ---- Source-driven frame builder --------------------------------------------

m5::hal::v2::result_t<size_t> buildDataFrame(uint8_t* block, uint8_t stream_id, data::Source& src);

// ---- Source / Sink integration ----------------------------------------------

class FrameReader {
public:
    explicit FrameReader(data::Source& source) : _source{&source}
    {
    }

    m5::hal::v2::result_t<DecodeResult> next(View& view);

private:
    data::Source* _source = nullptr;
    size_t _pending       = 0;
};

class FrameWriter {
public:
    explicit FrameWriter(data::Sink& sink) : _sink{&sink}
    {
    }

    m5::hal::v2::result_t<size_t> writeDelimiter(void);
    m5::hal::v2::result_t<size_t> writeChecked(Kind kind, uint8_t b3, data::ConstDataSpan payload);
    m5::hal::v2::result_t<size_t> writeData(uint8_t stream_id, data::ConstDataSpan stream_data);

private:
    m5::hal::v2::result_t<size_t> reserveExact(size_t need, data::DataSpan& out);

    data::Sink* _sink = nullptr;
};

}  // namespace m5::hal::v2::frame

#endif
