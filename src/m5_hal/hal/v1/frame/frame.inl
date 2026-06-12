// SPDX-License-Identifier: MIT
#ifndef M5_HAL_FRAME_FRAME_INL_
#define M5_HAL_FRAME_FRAME_INL_

#include "frame.hpp"

#include <string.h>

namespace m5::hal::v1::frame {

namespace {
using error_t = m5::hal::v1::error::error_t;

// Offsets within a checked wire frame.
constexpr size_t kCheckOffset = kPrefixSize;               // CHECK16 (big endian)
constexpr size_t kBodyOffset  = kPrefixSize + kCheckSize;  // kind body

// Store CHECK16 over the finished frame image (check bytes already zero).
void finalizeCheck(uint8_t* wire, size_t wire_size)
{
    const uint16_t crc     = check16(data::ConstDataSpan{wire, wire_size});
    wire[kCheckOffset]     = static_cast<uint8_t>(crc >> 8);
    wire[kCheckOffset + 1] = static_cast<uint8_t>(crc & 0xFF);
}
}  // namespace

uint16_t crc16CcittUpdate(uint16_t crc, uint8_t data)
{
    crc ^= static_cast<uint16_t>(data) << 8;
    for (int i = 0; i < 8; ++i) {
        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

uint16_t check16(data::ConstDataSpan wire_frame)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < wire_frame.size; ++i) {
        const uint8_t b = (i == kCheckOffset || i == kCheckOffset + 1) ? uint8_t{0} : wire_frame.data[i];
        crc             = crc16CcittUpdate(crc, b);
    }
    return crc;
}

result_t<size_t> encodeDelimiter(data::DataSpan dst)
{
    if (dst.data == nullptr || dst.size < kPrefixSize) {
        return m5::stl::make_unexpected(error_t::BUFFER_OVERFLOW);
    }
    dst.data[0] = 0x00;
    dst.data[1] = static_cast<uint8_t>(Kind::delimiter);
    return kPrefixSize;
}

result_t<size_t> encodeChecked(data::DataSpan dst, Kind kind, data::ConstDataSpan kind_body)
{
    if (!isCheckedKind(kind) || kind_body.size > kMaxBodySize - kCheckSize ||
        (kind_body.data == nullptr && kind_body.size != 0)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    const size_t need = checkedFrameWireSize(kind_body.size);
    if (dst.data == nullptr || dst.size < need) {
        return m5::stl::make_unexpected(error_t::BUFFER_OVERFLOW);
    }

    dst.data[0]                = static_cast<uint8_t>(kind_body.size + kCheckSize);
    dst.data[1]                = static_cast<uint8_t>(kind);
    dst.data[kCheckOffset]     = 0;
    dst.data[kCheckOffset + 1] = 0;
    if (kind_body.size != 0) {
        ::memcpy(dst.data + kBodyOffset, kind_body.data, kind_body.size);
    }
    finalizeCheck(dst.data, need);
    return need;
}

result_t<size_t> encodeData(data::DataSpan dst, uint8_t stream_id, data::ConstDataSpan stream_data)
{
    if (stream_data.size > kMaxDataPayload || (stream_data.data == nullptr && stream_data.size != 0)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    const size_t kind_body_size = 1 + stream_data.size;  // [stream_id][payload]
    const size_t need           = checkedFrameWireSize(kind_body_size);
    if (dst.data == nullptr || dst.size < need) {
        return m5::stl::make_unexpected(error_t::BUFFER_OVERFLOW);
    }

    // Encoded in place (no staging buffer): header, stream id, payload,
    // then the check over the finished image.
    dst.data[0]                = static_cast<uint8_t>(kind_body_size + kCheckSize);
    dst.data[1]                = static_cast<uint8_t>(Kind::data);
    dst.data[kCheckOffset]     = 0;
    dst.data[kCheckOffset + 1] = 0;
    dst.data[kBodyOffset]      = stream_id;
    if (stream_data.size != 0) {
        ::memcpy(dst.data + kBodyOffset + 1, stream_data.data, stream_data.size);
    }
    finalizeCheck(dst.data, need);
    return need;
}

DecodeResult decode(data::ConstDataSpan src, View& view)
{
    view = {};
    if (src.data == nullptr || src.size < kPrefixSize) {
        return {DecodeStatus::need_more, 0};
    }

    const auto body_size = static_cast<size_t>(src.data[0]);
    const auto kind_u8   = src.data[1];

    if (body_size == 0 && kind_u8 == 0x00) {
        return {DecodeStatus::padding, 1};
    }

    if (!isKnownKind(kind_u8)) {
        if ((kind_u8 & 1) == 0 && body_size == 0) {
            return {DecodeStatus::padding, 1};
        }

        // Trust the self-described size to skip the unknown frame in one
        // step (resync continues at the next candidate prefix).
        const auto full_size = kPrefixSize + body_size;
        if (src.size < full_size) {
            return {DecodeStatus::need_more, 0};
        }
        return {DecodeStatus::invalid_prefix, full_size};
    }

    const auto kind      = static_cast<Kind>(kind_u8);
    const auto full_size = kPrefixSize + body_size;

    if (src.size < full_size) {
        return {DecodeStatus::need_more, 0};
    }

    if (kind == Kind::delimiter) {
        if (body_size != 0) {
            return {DecodeStatus::invalid_size, full_size};
        }
        view.kind = kind;
        return {DecodeStatus::ok, full_size};
    }

    if (body_size < kCheckSize) {
        return {DecodeStatus::invalid_size, full_size};
    }

    const uint16_t stored =
        static_cast<uint16_t>((static_cast<uint16_t>(src.data[kCheckOffset]) << 8) | src.data[kCheckOffset + 1]);
    if (check16({src.data, full_size}) != stored) {
        return {DecodeStatus::invalid_check, full_size};
    }

    view.kind          = kind;
    view.body          = {src.data + kPrefixSize, body_size};
    view.kind_body     = {src.data + kBodyOffset, body_size - kCheckSize};
    view.frame_check16 = stored;
    view.has_check     = true;
    return {DecodeStatus::ok, full_size};
}

result_t<DecodeResult> FrameReader::next(View& view)
{
    view = {};
    // Release the frame handed out by the previous call (the View
    // borrowed the Source's peek span until now).
    if (_pending > 0) {
        const size_t pending = _pending;
        _pending             = 0;
        auto advanced        = _source->advance(pending);
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
    }
    for (;;) {
        // Two-stage acquisition: learn the frame size from the prefix,
        // then request exactly that many bytes. Asking for kMaxWireSize
        // up front would make a StreamSource block a whole timeout for
        // bytes that are not part of an already-complete frame.
        auto head = _source->peek(kPrefixSize);
        if (!head.has_value()) {
            return m5::stl::make_unexpected(head.error());
        }
        if (head.value().size == 0) {
            // Source end-of-stream (empty span is EOF by contract).
            return m5::stl::make_unexpected(error_t::END_OF_STREAM);
        }
        if (head.value().size < kPrefixSize) {
            if (_source->closed()) {
                // The producer is done and the leftover byte can never
                // grow into a frame: surface end-of-stream instead of a
                // need_more the caller would retry forever (the partial
                // tail stays unconsumed).
                return m5::stl::make_unexpected(error_t::END_OF_STREAM);
            }
            return DecodeResult{DecodeStatus::need_more, 0};
        }
        const size_t full_size = kPrefixSize + head.value().data[0];

        auto peeked = _source->peek(full_size);
        if (!peeked.has_value()) {
            return m5::stl::make_unexpected(peeked.error());
        }
        if (peeked.value().size < full_size) {
            if (_source->closed()) {
                // Stream ended mid-frame (finite source with a truncated
                // tail): retrying cannot make progress.
                return m5::stl::make_unexpected(error_t::END_OF_STREAM);
            }
            // Timed out (stream): more bytes may still arrive, so the
            // caller retries. A frame larger than what the Source can
            // lend stays here as well.
            return DecodeResult{DecodeStatus::need_more, 0};
        }

        const auto result = decode(peeked.value(), view);
        switch (result.status) {
            case DecodeStatus::padding: {
                auto advanced = _source->advance(result.consumed);
                if (!advanced.has_value()) {
                    return m5::stl::make_unexpected(advanced.error());
                }
                continue;
            }
            case DecodeStatus::ok:
                _pending = result.consumed;  // advance deferred; View borrows the peek span
                return result;
            case DecodeStatus::need_more:
                return result;
            default: {  // invalid_prefix / invalid_size / invalid_check
                view          = {};
                auto advanced = _source->advance(result.consumed);
                if (!advanced.has_value()) {
                    return m5::stl::make_unexpected(advanced.error());
                }
                return result;
            }
        }
    }
}

result_t<size_t> FrameWriter::reserveExact(size_t need, data::DataSpan& out)
{
    auto reserved = _sink->reserve(need);
    if (!reserved.has_value()) {
        return m5::stl::make_unexpected(reserved.error());
    }
    if (reserved.value().size < need) {
        return m5::stl::make_unexpected(_sink->closed() ? error_t::CLOSED : error_t::BUFFER_OVERFLOW);
    }
    out = reserved.value();
    return need;
}

result_t<size_t> FrameWriter::writeDelimiter(void)
{
    data::DataSpan span{};
    auto reserved = reserveExact(kPrefixSize, span);
    if (!reserved.has_value()) {
        return reserved;
    }
    auto encoded = encodeDelimiter(span);
    if (!encoded.has_value()) {
        return encoded;
    }
    auto committed = _sink->commit(encoded.value());
    if (!committed.has_value()) {
        return m5::stl::make_unexpected(committed.error());
    }
    return encoded;
}

result_t<size_t> FrameWriter::writeChecked(Kind kind, data::ConstDataSpan kind_body)
{
    if (!isCheckedKind(kind) || kind_body.size > kMaxBodySize - kCheckSize ||
        (kind_body.data == nullptr && kind_body.size != 0)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    data::DataSpan span{};
    auto reserved = reserveExact(checkedFrameWireSize(kind_body.size), span);
    if (!reserved.has_value()) {
        return reserved;
    }
    auto encoded = encodeChecked(span, kind, kind_body);
    if (!encoded.has_value()) {
        return encoded;
    }
    auto committed = _sink->commit(encoded.value());
    if (!committed.has_value()) {
        return m5::stl::make_unexpected(committed.error());
    }
    return encoded;
}

result_t<size_t> FrameWriter::writeData(uint8_t stream_id, data::ConstDataSpan stream_data)
{
    if (stream_data.size > kMaxDataPayload || (stream_data.data == nullptr && stream_data.size != 0)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    data::DataSpan span{};
    auto reserved = reserveExact(checkedFrameWireSize(1 + stream_data.size), span);
    if (!reserved.has_value()) {
        return reserved;
    }
    auto encoded = encodeData(span, stream_id, stream_data);
    if (!encoded.has_value()) {
        return encoded;
    }
    auto committed = _sink->commit(encoded.value());
    if (!committed.has_value()) {
        return m5::stl::make_unexpected(committed.error());
    }
    return encoded;
}

}  // namespace m5::hal::v1::frame

#endif
