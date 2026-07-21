// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_CREDIT_NOTIFIER_HPP_
#define M5_HAL_REMOTE_CREDIT_NOTIFIER_HPP_

#include "../data/mux.hpp"
#include "../diag.hpp"

namespace m5::hal::v2::remote::detail {

// ---- CreditNotifier — shared credit advertisement logic --------------------
//
// Advertises the local receive capacity as an absolute Credit frame
// (B3 = grantable frame count) whenever the value changes, or whenever
// temp pool releases happened since the last successful advertisement.
// The grant is the minimum of the temp pool free blocks and the
// remaining queue slots of the most congested block-mode stream,
// saturated at 255. Heap-fallback allocations never count (the pool is
// the hard bound).
//
// Absolute reports are loss-tolerant (a dropped Credit frame is
// corrected by the next one) but can transiently over-grant while
// frames are in flight; the wire-stall backpressure in
// MuxFrameDecoder::pump() remains the hard limit for that window.
struct CreditNotifier {
    void pump(data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec)
    {
        auto* alloc = dec.allocator();
        if (alloc == nullptr) {
            return;
        }
        const size_t used = alloc->usedBlocks();
        size_t free       = used < memory::Allocator::tempBlockCount() ? memory::Allocator::tempBlockCount() - used : 0;
        const size_t slots = dec.blockStreamFreeSlots();
        if (slots < free) {
            free = slots;
        }
        if (free > 255) {
            free = 255;
        }
        const uint8_t credit = static_cast<uint8_t>(free);
        // Progress signal = blocks consumed out of the decoder's own block
        // streams. An allocator-wide free counter would also tick for this
        // side's outgoing frames (including the Credit frame itself), turning
        // the resend condition into a self-triggering endless Credit loop.
        const size_t release_count = dec.blockStreamReleasedTotal();
        if (!_sent || credit != _last || release_count != _last_release_count) {
            if (enc.writeFrame(frame::Kind::Credit, credit, {}).has_value()) {
                M5HAL_DIAG("credit advertise free=%u prev_last=%u release_count=%zu", static_cast<unsigned>(credit),
                           static_cast<unsigned>(_last), release_count);
                _last               = credit;
                _last_release_count = release_count;
                _sent               = true;
            }
        }
    }

private:
    bool _sent                 = false;
    uint8_t _last              = 0;
    size_t _last_release_count = 0;
};

}  // namespace m5::hal::v2::remote::detail

#endif  // M5_HAL_REMOTE_CREDIT_NOTIFIER_HPP_
