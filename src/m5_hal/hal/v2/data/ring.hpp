// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DATA_RING_HPP_
#define M5_HAL_DATA_RING_HPP_

#include "../data.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace m5::hal::v2::data {

class RingFIFO {
public:
    RingFIFO() = default;
    RingFIFO(uint8_t* buf, size_t capacity);

    // SourceView/SinkView hold references to this object; copy/move would
    // leave them dangling. Use setBuf() to re-bind.
    RingFIFO(const RingFIFO&)            = delete;
    RingFIFO& operator=(const RingFIFO&) = delete;
    RingFIFO(RingFIFO&&)                 = delete;
    RingFIFO& operator=(RingFIFO&&)      = delete;

    size_t buffered() const;
    size_t free() const;
    size_t capacity() const;

    void reset();

    /*! @brief Re-bind to a new buffer (also resets head/tail/buffered). */
    void setBuf(uint8_t* buf, size_t cap);

    Source& source();
    Sink& sink();

private:
    uint8_t* _buf    = nullptr;
    size_t _capacity = 0;
    std::atomic<size_t> _head{0};
    std::atomic<size_t> _tail{0};
    std::atomic<size_t> _buffered{0};

    // SPSC-safe: SourceView is the sole consumer (reads _tail, reads _buffered).
    // SinkView is the sole producer (reads _head, reads _buffered).
    // _buffered is the synchronization variable between them.
    class SourceView : public Source {
    public:
        explicit SourceView(RingFIFO& owner);

        m5::hal::v2::result_t<ConstDataSpan> peek(size_t max_len) override;

        m5::hal::v2::result_t<void> advance(size_t N) override;

        bool eof() const override;

    private:
        RingFIFO& _owner;
    };

    class SinkView : public Sink {
    public:
        explicit SinkView(RingFIFO& owner);

        m5::hal::v2::result_t<DataSpan> reserve(size_t max_len) override;

        m5::hal::v2::result_t<void> commit(size_t N) override;

        bool closed() const override;

    private:
        RingFIFO& _owner;
        size_t _reserved = 0;
    };

    SourceView _source{*this};
    SinkView _sink{*this};
};

}  // namespace m5::hal::v2::data

#endif
