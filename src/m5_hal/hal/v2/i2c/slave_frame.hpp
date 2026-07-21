// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_I2C_SLAVE_FRAME_HPP_
#define M5_HAL_HAL_V2_I2C_SLAVE_FRAME_HPP_

#include "../error.hpp"
#include "../slave/queue.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace m5::hal::v2::i2c {

enum class I2cFrameDirection : uint8_t { Write, Read };
enum class I2cSegmentFlags : uint8_t { None = 0, RepeatedStart = 1u << 0 };

constexpr I2cSegmentFlags operator|(I2cSegmentFlags lhs, I2cSegmentFlags rhs);
constexpr I2cSegmentFlags operator&(I2cSegmentFlags lhs, I2cSegmentFlags rhs);
constexpr I2cSegmentFlags& operator|=(I2cSegmentFlags& lhs, I2cSegmentFlags rhs);
constexpr bool any(I2cSegmentFlags value);

struct I2cFrameSegment {
    uint32_t offset             = 0;
    uint32_t length             = 0;
    I2cFrameDirection direction = I2cFrameDirection::Write;
    I2cSegmentFlags flags       = I2cSegmentFlags::None;
    uint16_t reserved           = 0;
};

struct I2cSegmentStorage {
    I2cFrameSegment* segments = nullptr;
    size_t segment_count      = 0;
};

template <size_t SegmentCount>
class StaticI2cSegmentStorage {
public:
    I2cSegmentStorage storage()
    {
        return {segments_.data(), segments_.size()};
    }

private:
    std::array<I2cFrameSegment, SegmentCount> segments_{};
};

struct I2cFrameSegmentSpan {
    const I2cFrameSegment* data = nullptr;
    size_t size                 = 0;

    const I2cFrameSegment* begin() const
    {
        return data;
    }
    const I2cFrameSegment* end() const
    {
        return data == nullptr ? nullptr : data + size;
    }
};

struct I2cFrameSegmentsView {
    I2cFrameSegmentSpan first{};
    I2cFrameSegmentSpan second{};
};

class I2cSegmentToken {
public:
    I2cSegmentToken()                                  = default;
    I2cSegmentToken(const I2cSegmentToken&)            = delete;
    I2cSegmentToken& operator=(const I2cSegmentToken&) = delete;
    I2cSegmentToken(I2cSegmentToken&& other) noexcept;
    I2cSegmentToken& operator=(I2cSegmentToken&& other) noexcept;

private:
    friend class I2cSegmentQueue;
    I2cSegmentToken(uintptr_t owner_cookie, uint32_t generation, uint32_t sequence);
    void invalidate();

    uintptr_t owner_cookie_ = 0;
    uint32_t generation_    = 0;
    uint32_t sequence_      = 0;
};

class I2cSegmentQueue {
public:
    I2cSegmentQueue()                                  = default;
    I2cSegmentQueue(const I2cSegmentQueue&)            = delete;
    I2cSegmentQueue& operator=(const I2cSegmentQueue&) = delete;

    result_t<void> bind(I2cSegmentStorage storage, uint32_t generation = 0);
    result_t<I2cSegmentToken> beginFrame();
    result_t<void> appendSegment(I2cSegmentToken& token, const I2cFrameSegment& segment);
    result_t<uint8_t> commitFrame(I2cSegmentToken& token);
    result_t<void> cancelFrame(I2cSegmentToken& token);
    result_t<I2cFrameSegmentsView> peekSegments(uint8_t count) const;
    result_t<void> popSegments(uint8_t count);
    size_t readable() const;
    size_t writable() const;
    bool hasReservation() const;
    result_t<void> cancelReservation();
    result_t<void> setGeneration(uint32_t generation);
    void reset(uint32_t generation);
    uint32_t generation() const;

private:
    static uintptr_t ownerCookie(const I2cSegmentQueue* queue);
    static uint32_t used(uint32_t producer, uint32_t consumer);
    bool bound() const;
    bool tokenMatches(const I2cSegmentToken& token) const;
    uint32_t nextSequence();
    void clearReservation(I2cSegmentToken* token = nullptr);

    I2cFrameSegment* storage_ = nullptr;
    uint32_t capacity_        = 0;
    bool bound_               = false;
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    uint32_t generation_        = 0;
    uint32_t sequence_          = 0;
    uint32_t reserved_tail_     = 0;
    uint32_t reserved_count_    = 0;
    uint32_t reserved_sequence_ = 0;
};

struct I2cFrameView {
    slave::FrameView frame{};
    I2cFrameSegmentsView segments{};
};

struct I2cObservedFrameReservation {
    I2cObservedFrameReservation()                                                  = default;
    I2cObservedFrameReservation(const I2cObservedFrameReservation&)                = delete;
    I2cObservedFrameReservation& operator=(const I2cObservedFrameReservation&)     = delete;
    I2cObservedFrameReservation(I2cObservedFrameReservation&&) noexcept            = default;
    I2cObservedFrameReservation& operator=(I2cObservedFrameReservation&&) noexcept = default;

private:
    friend class I2cObservedFrameWriter;
    slave::FrameReservation frame_{};
    I2cSegmentToken segments_{};
    uint32_t stored_bytes_          = 0;
    uint32_t dropped_bytes_         = 0;
    uint32_t segment_wire_bytes_    = 0;
    uint8_t segment_count_          = 0;
    bool segment_details_available_ = true;
};

class I2cFrameSourceView {
public:
    I2cFrameSourceView() = default;

    result_t<I2cFrameView> peekFrame() const;
    result_t<void> popFrame();
    size_t readableFrames() const;

private:
    friend class SlaveAccessor;
    I2cFrameSourceView(slave::SlaveQueue* frames, I2cSegmentQueue* segments) : frames_{frames}, segments_{segments}
    {
    }

    slave::SlaveQueue* frames_ = nullptr;
    I2cSegmentQueue* segments_ = nullptr;
};

class I2cObservedFrameWriter {
public:
    I2cObservedFrameWriter() = default;

    result_t<I2cObservedFrameReservation> beginFrame();
    result_t<size_t> appendReceived(I2cObservedFrameReservation& reservation, data::ConstDataSpan payload);
    result_t<void> appendSegment(I2cObservedFrameReservation& reservation, const I2cFrameSegment& segment);
    result_t<void> commitFrame(I2cObservedFrameReservation& reservation, slave::FrameMetadata metadata);
    result_t<void> cancelFrame(I2cObservedFrameReservation& reservation);

private:
    friend class SlaveAccessor;
    I2cObservedFrameWriter(slave::SlaveQueue* frames, I2cSegmentQueue* segments) : frames_{frames}, segments_{segments}
    {
    }

    slave::SlaveQueue* frames_ = nullptr;
    I2cSegmentQueue* segments_ = nullptr;

    result_t<void> discardSegmentDetails(I2cObservedFrameReservation& reservation);
};

static_assert(std::is_standard_layout<I2cFrameSegment>::value, "I2cFrameSegment must remain a plain shared value");
static_assert(sizeof(I2cFrameSourceView) == sizeof(void*) * 2,
              "I2cFrameSourceView must remain a two-pointer thin view");
static_assert(sizeof(I2cObservedFrameWriter) == sizeof(void*) * 2,
              "I2cObservedFrameWriter must remain a two-pointer thin view");

}  // namespace m5::hal::v2::i2c

#include "slave_frame.inl"

#endif  // M5_HAL_HAL_V2_I2C_SLAVE_FRAME_HPP_
