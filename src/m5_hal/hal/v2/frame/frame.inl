// SPDX-License-Identifier: MIT
#ifndef M5_HAL_FRAME_FRAME_INL_
#define M5_HAL_FRAME_FRAME_INL_

#include "frame.hpp"

#include "../diag.hpp"

#include <string.h>

namespace m5::hal::v2::frame {

namespace {
using error_t = m5::hal::v2::error::error_t;
}  // namespace

uint8_t crc8AtmUpdate(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; ++i) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

uint8_t check8(uint8_t len, uint8_t kind)
{
    uint8_t crc = 0x00;
    crc         = crc8AtmUpdate(crc, len);
    crc         = crc8AtmUpdate(crc, kind);
    return crc;
}

result_t<size_t> encodeDelimiter(data::DataSpan dst)
{
    if (dst.data == nullptr || dst.size < kPrefixSize) {
        return m5::stl::make_unexpected(error_t::BUFFER_OVERFLOW);
    }
    dst.data[0] = 0x00;
    dst.data[1] = static_cast<uint8_t>(Kind::Delimiter);
    return kPrefixSize;
}

result_t<size_t> encodeChecked(data::DataSpan dst, Kind kind, uint8_t b3, data::ConstDataSpan payload)
{
    if (!isCheckedKind(kind) || payload.size > kMaxPayload || (payload.data == nullptr && payload.size != 0)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    const size_t wire_size = checkedFrameWireSize(payload.size);
    if (dst.data == nullptr || dst.size < wire_size) {
        return m5::stl::make_unexpected(error_t::BUFFER_OVERFLOW);
    }

    const auto len = static_cast<uint8_t>(kMinCheckedLen + payload.size);
    dst.data[0]    = len;
    dst.data[1]    = static_cast<uint8_t>(kind);
    dst.data[2]    = check8(len, static_cast<uint8_t>(kind));
    dst.data[3]    = b3;
    if (payload.size != 0) {
        ::memcpy(dst.data + kPayloadOffset, payload.data, payload.size);
    }
    return wire_size;
}

result_t<size_t> encodeData(data::DataSpan dst, uint8_t stream_id, data::ConstDataSpan stream_data)
{
    return encodeChecked(dst, Kind::Data, stream_id, stream_data);
}

DecodeResult decode(data::ConstDataSpan src, View& view)
{
    view = {};
    if (src.data == nullptr || src.size == 0) {
        return {DecodeStatus::NeedMore, 0};
    }

    const auto len_byte = src.data[0];

    if (len_byte == 0) {
        if (src.size >= 2 && src.data[1] == static_cast<uint8_t>(Kind::Delimiter)) {
            view.kind = Kind::Delimiter;
            return {DecodeStatus::Delimiter, 2};
        }
        return {DecodeStatus::Padding, 1};
    }

    if (src.size < kPrefixSize) {
        return {DecodeStatus::NeedMore, 0};
    }

    const auto kind_u8 = src.data[1];

    if (len_byte > kMaxLen) {
        return {DecodeStatus::InvalidSize, 1};
    }

    if (!isKnownKind(kind_u8)) {
        const size_t full_size = kPrefixSize + len_byte;
        if (src.size < full_size) {
            return {DecodeStatus::NeedMore, 0};
        }
        M5HAL_DIAG("frame unknown kind=0x%02x len=%u", static_cast<unsigned>(kind_u8), static_cast<unsigned>(len_byte));
        return {DecodeStatus::InvalidPrefix, full_size};
    }

    if (len_byte < kMinCheckedLen) {
        const size_t full_size = kPrefixSize + len_byte;
        if (src.size < full_size) {
            return {DecodeStatus::NeedMore, 0};
        }
        return {DecodeStatus::InvalidSize, full_size};
    }

    const size_t full_size = kPrefixSize + len_byte;
    if (src.size < full_size) {
        return {DecodeStatus::NeedMore, 0};
    }

    const uint8_t expected_check = check8(len_byte, kind_u8);
    if (src.data[2] != expected_check) {
        M5HAL_DIAG("frame checksum mismatch len=%u kind=%u expected=0x%02x got=0x%02x", static_cast<unsigned>(len_byte),
                   static_cast<unsigned>(kind_u8), static_cast<unsigned>(expected_check),
                   static_cast<unsigned>(src.data[2]));
        return {DecodeStatus::InvalidCheck, full_size};
    }

    const size_t payload_len = static_cast<size_t>(len_byte) - kMinCheckedLen;

    view.kind      = static_cast<Kind>(kind_u8);
    view.b3        = src.data[kB3Offset];
    view.check     = src.data[2];
    view.has_check = true;
    if (payload_len > 0) {
        view.payload = {src.data + kPayloadOffset, payload_len};
    }
    return {DecodeStatus::Ok, full_size};
}

size_t buildDataFrame(uint8_t* block, uint8_t stream_id, data::Source& src)
{
    size_t payload_len = 0;
    uint8_t* payload   = block + kPayloadOffset;

    while (payload_len < kMaxPayload) {
        auto peeked = src.peek(kMaxPayload - payload_len);
        if (!peeked.has_value() || peeked.value().size == 0) {
            break;
        }
        size_t take = peeked.value().size;
        if (take > kMaxPayload - payload_len) {
            take = kMaxPayload - payload_len;
        }
        ::memcpy(payload + payload_len, peeked.value().data, take);
        (void)src.advance(take);
        payload_len += take;
    }

    if (payload_len == 0) {
        return 0;
    }

    const auto len = static_cast<uint8_t>(kMinCheckedLen + payload_len);
    block[0]       = len;
    block[1]       = static_cast<uint8_t>(Kind::Data);
    block[2]       = check8(len, static_cast<uint8_t>(Kind::Data));
    block[3]       = stream_id;
    return kHeaderSize + payload_len;
}

result_t<DecodeResult> FrameReader::next(View& view)
{
    view = {};
    if (_pending > 0) {
        const size_t pending = _pending;
        _pending             = 0;
        auto advanced        = _source->advance(pending);
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
    }
    for (;;) {
        auto head = _source->peek(kPrefixSize);
        if (!head.has_value()) {
            return m5::stl::make_unexpected(head.error());
        }
        if (head.value().size == 0) {
            M5HAL_DIAG("wire closed (end of stream)");
            return m5::stl::make_unexpected(error_t::END_OF_STREAM);
        }
        if (head.value().size < 1) {
            if (_source->closed()) {
                return m5::stl::make_unexpected(error_t::END_OF_STREAM);
            }
            return DecodeResult{DecodeStatus::NeedMore, 0};
        }

        const auto len_byte = head.value().data[0];
        size_t full_size;
        if (len_byte == 0) {
            full_size =
                (head.value().size >= 2 && head.value().data[1] == static_cast<uint8_t>(Kind::Delimiter)) ? 2 : 1;
        } else {
            full_size = kPrefixSize + len_byte;
        }

        auto peeked = _source->peek(full_size);
        if (!peeked.has_value()) {
            return m5::stl::make_unexpected(peeked.error());
        }
        if (peeked.value().size < full_size) {
            if (_source->closed()) {
                return m5::stl::make_unexpected(error_t::END_OF_STREAM);
            }
            return DecodeResult{DecodeStatus::NeedMore, 0};
        }

        const auto result = decode(peeked.value(), view);
        switch (result.status) {
            case DecodeStatus::Padding: {
                auto advanced = _source->advance(result.consumed);
                if (!advanced.has_value()) {
                    return m5::stl::make_unexpected(advanced.error());
                }
                continue;
            }
            case DecodeStatus::Ok:
            case DecodeStatus::Delimiter:
                _pending = result.consumed;
                return result;
            case DecodeStatus::NeedMore:
                return result;
            default: {
                M5HAL_DIAG("frame discarded status=%d consumed=%zu", static_cast<int>(result.status), result.consumed);
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

result_t<size_t> FrameWriter::writeChecked(Kind kind, uint8_t b3, data::ConstDataSpan payload)
{
    if (!isCheckedKind(kind) || payload.size > kMaxPayload || (payload.data == nullptr && payload.size != 0)) {
        return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
    }
    data::DataSpan span{};
    auto reserved = reserveExact(checkedFrameWireSize(payload.size), span);
    if (!reserved.has_value()) {
        return reserved;
    }
    auto encoded = encodeChecked(span, kind, b3, payload);
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
    return writeChecked(Kind::Data, stream_id, stream_data);
}

}  // namespace m5::hal::v2::frame

#endif
