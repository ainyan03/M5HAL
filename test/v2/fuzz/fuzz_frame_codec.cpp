// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>

#include <cstddef>
#include <cstdint>
#include <cstdlib>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    namespace frame = m5::hal::v2::frame;
    frame::View view;
    const auto decoded = frame::decode({data, size}, view);

    if (decoded.consumed > size) {
        std::abort();
    }
    if (decoded.status == frame::DecodeStatus::Ok) {
        if (!view.has_check || decoded.consumed < frame::kHeaderSize || view.payload.size > frame::kMaxPayload) {
            std::abort();
        }
        const auto* begin = data + frame::kPayloadOffset;
        if ((view.payload.size != 0 && view.payload.data != begin) ||
            frame::checkedFrameWireSize(view.payload.size) != decoded.consumed) {
            std::abort();
        }
    }
    return 0;
}
