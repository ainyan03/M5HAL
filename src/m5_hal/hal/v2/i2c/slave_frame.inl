// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_I2C_SLAVE_FRAME_INL_
#define M5_HAL_HAL_V2_I2C_SLAVE_FRAME_INL_

#include "slave_frame.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace m5::hal::v2::i2c {

constexpr I2cSegmentFlags operator|(I2cSegmentFlags lhs, I2cSegmentFlags rhs)
{
    return static_cast<I2cSegmentFlags>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}
constexpr I2cSegmentFlags operator&(I2cSegmentFlags lhs, I2cSegmentFlags rhs)
{
    return static_cast<I2cSegmentFlags>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}
constexpr I2cSegmentFlags& operator|=(I2cSegmentFlags& lhs, I2cSegmentFlags rhs)
{
    lhs = lhs | rhs;
    return lhs;
}
constexpr bool any(I2cSegmentFlags value)
{
    return value != I2cSegmentFlags::None;
}

inline I2cSegmentToken::I2cSegmentToken(uintptr_t owner_cookie, uint32_t generation, uint32_t sequence)
    : owner_cookie_{owner_cookie}, generation_{generation}, sequence_{sequence}
{
}
inline I2cSegmentToken::I2cSegmentToken(I2cSegmentToken&& other) noexcept
    : owner_cookie_{other.owner_cookie_}, generation_{other.generation_}, sequence_{other.sequence_}
{
    other.invalidate();
}
inline I2cSegmentToken& I2cSegmentToken::operator=(I2cSegmentToken&& other) noexcept
{
    if (this != &other) {
        owner_cookie_ = other.owner_cookie_;
        generation_   = other.generation_;
        sequence_     = other.sequence_;
        other.invalidate();
    }
    return *this;
}
inline void I2cSegmentToken::invalidate()
{
    owner_cookie_ = 0;
    generation_   = 0;
    sequence_     = 0;
}

inline uintptr_t I2cSegmentQueue::ownerCookie(const I2cSegmentQueue* queue)
{
    const uintptr_t cookie = reinterpret_cast<uintptr_t>(queue);
    return cookie == 0 ? 1 : cookie;
}
inline uint32_t I2cSegmentQueue::used(uint32_t producer, uint32_t consumer)
{
    return producer - consumer;
}
inline bool I2cSegmentQueue::bound() const
{
    return bound_;
}
inline bool I2cSegmentQueue::tokenMatches(const I2cSegmentToken& token) const
{
    return token.owner_cookie_ == ownerCookie(this) && token.generation_ == generation_ && token.sequence_ != 0 &&
           token.sequence_ == reserved_sequence_;
}
inline uint32_t I2cSegmentQueue::nextSequence()
{
    ++sequence_;
    if (sequence_ == 0) {
        ++sequence_;
    }
    return sequence_;
}
inline void I2cSegmentQueue::clearReservation(I2cSegmentToken* token)
{
    reserved_tail_     = 0;
    reserved_count_    = 0;
    reserved_sequence_ = 0;
    if (token != nullptr) {
        token->invalidate();
    }
}

inline result_t<void> I2cSegmentQueue::bind(I2cSegmentStorage storage, uint32_t new_generation)
{
    if (bound() && (readable() != 0 || hasReservation())) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if ((storage.segments == nullptr && storage.segment_count != 0) ||
        storage.segment_count > std::numeric_limits<uint32_t>::max()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    storage_  = storage.segments;
    capacity_ = static_cast<uint32_t>(storage.segment_count);
    bound_    = true;
    reset(new_generation);
    return {};
}

inline result_t<I2cSegmentToken> I2cSegmentQueue::beginFrame()
{
    if (!bound() || hasReservation()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    reserved_tail_     = tail_.load(std::memory_order_relaxed);
    reserved_count_    = 0;
    reserved_sequence_ = nextSequence();
    return I2cSegmentToken{ownerCookie(this), generation_, reserved_sequence_};
}

inline result_t<void> I2cSegmentQueue::appendSegment(I2cSegmentToken& token, const I2cFrameSegment& segment)
{
    if (!tokenMatches(token)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (reserved_count_ >= std::numeric_limits<uint8_t>::max()) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    const uint32_t head = head_.load(std::memory_order_acquire);
    if (reserved_count_ >= capacity_ - used(reserved_tail_, head)) {
        return m5::stl::make_unexpected(error::error_t::WOULD_BLOCK);
    }
    storage_[(reserved_tail_ + reserved_count_) % capacity_] = segment;
    ++reserved_count_;
    return {};
}

inline result_t<uint8_t> I2cSegmentQueue::commitFrame(I2cSegmentToken& token)
{
    if (!tokenMatches(token)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    const uint8_t count = static_cast<uint8_t>(reserved_count_);
    tail_.store(reserved_tail_ + reserved_count_, std::memory_order_release);
    clearReservation(&token);
    return count;
}

inline result_t<void> I2cSegmentQueue::cancelFrame(I2cSegmentToken& token)
{
    if (!tokenMatches(token)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    clearReservation(&token);
    return {};
}

inline result_t<I2cFrameSegmentsView> I2cSegmentQueue::peekSegments(uint8_t count) const
{
    if (!bound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    const uint32_t head      = head_.load(std::memory_order_relaxed);
    const uint32_t tail      = tail_.load(std::memory_order_acquire);
    const uint32_t available = used(tail, head);
    if (count > available) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    I2cFrameSegmentsView view;
    if (count == 0) {
        return view;
    }
    const uint32_t index = head % capacity_;
    const uint32_t first = std::min<uint32_t>(count, capacity_ - index);
    view.first           = {storage_ + index, first};
    view.second          = {storage_, static_cast<size_t>(count - first)};
    return view;
}

inline result_t<void> I2cSegmentQueue::popSegments(uint8_t count)
{
    if (!bound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    const uint32_t head      = head_.load(std::memory_order_relaxed);
    const uint32_t tail      = tail_.load(std::memory_order_acquire);
    const uint32_t available = used(tail, head);
    if (count > available) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    head_.store(head + count, std::memory_order_release);
    return {};
}

inline size_t I2cSegmentQueue::readable() const
{
    const uint32_t head = head_.load(std::memory_order_relaxed);
    const uint32_t tail = tail_.load(std::memory_order_acquire);
    return used(tail, head);
}
inline size_t I2cSegmentQueue::writable() const
{
    const uint32_t head = head_.load(std::memory_order_acquire);
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    return capacity_ - used(tail, head) - reserved_count_;
}
inline bool I2cSegmentQueue::hasReservation() const
{
    return reserved_sequence_ != 0;
}
inline result_t<void> I2cSegmentQueue::cancelReservation()
{
    if (!hasReservation()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    clearReservation();
    return {};
}
inline result_t<void> I2cSegmentQueue::setGeneration(uint32_t new_generation)
{
    if (hasReservation()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    generation_ = new_generation;
    return {};
}
inline void I2cSegmentQueue::reset(uint32_t new_generation)
{
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
    clearReservation();
    generation_ = new_generation;
}
inline uint32_t I2cSegmentQueue::generation() const
{
    return generation_;
}

inline result_t<I2cFrameView> I2cFrameSourceView::peekFrame() const
{
    if (frames_ == nullptr || segments_ == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto frame = frames_->peekFrame();
    if (!frame.has_value()) {
        return m5::stl::make_unexpected(frame.error());
    }
    auto segments = segments_->peekSegments(frame->metadata.segment_count);
    if (!segments.has_value()) {
        return m5::stl::make_unexpected(segments.error());
    }
    return I2cFrameView{*frame, *segments};
}

inline result_t<void> I2cFrameSourceView::popFrame()
{
    if (frames_ == nullptr || segments_ == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto frame = frames_->peekFrame();
    if (!frame.has_value()) {
        return m5::stl::make_unexpected(frame.error());
    }
    auto segments = segments_->peekSegments(frame->metadata.segment_count);
    if (!segments.has_value()) {
        return m5::stl::make_unexpected(segments.error());
    }
    auto popped_segments = segments_->popSegments(frame->metadata.segment_count);
    if (!popped_segments.has_value()) {
        return popped_segments;
    }
    return frames_->popFrame();
}

inline size_t I2cFrameSourceView::readableFrames() const
{
    return frames_ == nullptr ? 0 : frames_->readableFrames();
}

inline result_t<I2cObservedFrameReservation> I2cObservedFrameWriter::beginFrame()
{
    if (frames_ == nullptr || segments_ == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto frame = frames_->beginObservedFrame();
    if (!frame.has_value()) {
        return m5::stl::make_unexpected(frame.error());
    }
    auto segments = segments_->beginFrame();
    if (!segments.has_value()) {
        (void)frames_->cancelFrame(*frame);
        return m5::stl::make_unexpected(segments.error());
    }
    I2cObservedFrameReservation reservation;
    reservation.frame_    = std::move(*frame);
    reservation.segments_ = std::move(*segments);
    return result_t<I2cObservedFrameReservation>{m5::stl::in_place, std::move(reservation)};
}

inline result_t<size_t> I2cObservedFrameWriter::appendReceived(I2cObservedFrameReservation& reservation,
                                                               data::ConstDataSpan payload)
{
    if (frames_ == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto appended         = frames_->appendObservedFrame(reservation.frame_, payload);
    const size_t accepted = appended.has_value() ? *appended : 0;
    if (appended.has_value() || appended.error() == error::error_t::WOULD_BLOCK) {
        const size_t dropped = payload.size - accepted;
        const size_t stored_increment =
            std::min<size_t>(accepted, std::numeric_limits<uint32_t>::max() - reservation.stored_bytes_);
        const size_t dropped_increment =
            std::min<size_t>(dropped, std::numeric_limits<uint32_t>::max() - reservation.dropped_bytes_);
        reservation.stored_bytes_ += static_cast<uint32_t>(stored_increment);
        reservation.dropped_bytes_ += static_cast<uint32_t>(dropped_increment);
    }
    return appended;
}

inline result_t<void> I2cObservedFrameWriter::appendSegment(I2cObservedFrameReservation& reservation,
                                                            const I2cFrameSegment& segment)
{
    if (segments_ == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (!reservation.segment_details_available_) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (segment.offset != reservation.segment_wire_bytes_ ||
        (reservation.segment_count_ == 0 && any(segment.flags & I2cSegmentFlags::RepeatedStart)) ||
        (reservation.segment_count_ != 0 && !any(segment.flags & I2cSegmentFlags::RepeatedStart)) ||
        segment.length > std::numeric_limits<uint32_t>::max() - segment.offset) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto appended = segments_->appendSegment(reservation.segments_, segment);
    if (appended.has_value()) {
        reservation.segment_wire_bytes_ = segment.offset + segment.length;
        ++reservation.segment_count_;
    } else if (appended.error() == error::error_t::WOULD_BLOCK || appended.error() == error::error_t::OUT_OF_RESOURCE) {
        auto discarded = discardSegmentDetails(reservation);
        if (!discarded.has_value()) {
            return discarded;
        }
    }
    return appended;
}

inline result_t<void> I2cObservedFrameWriter::discardSegmentDetails(I2cObservedFrameReservation& reservation)
{
    auto cancelled = segments_->cancelFrame(reservation.segments_);
    if (!cancelled.has_value()) {
        return cancelled;
    }
    auto empty = segments_->beginFrame();
    if (!empty.has_value()) {
        return m5::stl::make_unexpected(empty.error());
    }
    reservation.segments_                  = std::move(*empty);
    reservation.segment_wire_bytes_        = 0;
    reservation.segment_count_             = 0;
    reservation.segment_details_available_ = false;
    return {};
}

inline result_t<void> I2cObservedFrameWriter::commitFrame(I2cObservedFrameReservation& reservation,
                                                          slave::FrameMetadata metadata)
{
    if (frames_ == nullptr || segments_ == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto segment_count = segments_->commitFrame(reservation.segments_);
    if (!segment_count.has_value()) {
        return m5::stl::make_unexpected(segment_count.error());
    }
    metadata.segment_count = *segment_count;
    if (reservation.segment_details_available_ && metadata.segment_count != 0) {
        metadata.wire_bytes = reservation.segment_wire_bytes_;
    }
    metadata.stored_bytes  = reservation.stored_bytes_;
    metadata.dropped_bytes = reservation.dropped_bytes_;
    if (metadata.dropped_bytes != 0) {
        metadata.flags |= slave::FrameFlags::Overflow | slave::FrameFlags::Truncated;
        frames_->recordDroppedFrames(0, metadata.dropped_bytes);
    }
    return frames_->commitFrame(reservation.frame_, metadata);
}

inline result_t<void> I2cObservedFrameWriter::cancelFrame(I2cObservedFrameReservation& reservation)
{
    if (frames_ == nullptr || segments_ == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto cancelled_segments = segments_->cancelFrame(reservation.segments_);
    auto cancelled_frame    = frames_->cancelFrame(reservation.frame_);
    if (!cancelled_segments.has_value()) {
        return cancelled_segments;
    }
    return cancelled_frame;
}

}  // namespace m5::hal::v2::i2c

#endif  // M5_HAL_HAL_V2_I2C_SLAVE_FRAME_INL_
