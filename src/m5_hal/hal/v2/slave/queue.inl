// SPDX-License-Identifier: MIT
#ifndef M5_HAL_SLAVE_QUEUE_INL_
#define M5_HAL_SLAVE_QUEUE_INL_

#include <algorithm>
#include <cstring>
#include <limits>

namespace m5::hal::v2::slave {

namespace detail {
template <class Enum>
constexpr auto enumValue(Enum value)
{
    return static_cast<typename std::underlying_type<Enum>::type>(value);
}

template <class T>
result_t<T> queueError(error::error_t error)
{
    return m5::stl::make_unexpected(error);
}
}  // namespace detail

constexpr FrameFlags operator|(FrameFlags lhs, FrameFlags rhs)
{
    return static_cast<FrameFlags>(detail::enumValue(lhs) | detail::enumValue(rhs));
}

constexpr FrameFlags operator&(FrameFlags lhs, FrameFlags rhs)
{
    return static_cast<FrameFlags>(detail::enumValue(lhs) & detail::enumValue(rhs));
}

constexpr FrameFlags operator~(FrameFlags value)
{
    return static_cast<FrameFlags>(~detail::enumValue(value));
}

constexpr FrameFlags& operator|=(FrameFlags& lhs, FrameFlags rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

constexpr FrameFlags& operator&=(FrameFlags& lhs, FrameFlags rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

constexpr bool any(FrameFlags value)
{
    return detail::enumValue(value) != 0;
}

constexpr QueueEventFlags operator|(QueueEventFlags lhs, QueueEventFlags rhs)
{
    return static_cast<QueueEventFlags>(detail::enumValue(lhs) | detail::enumValue(rhs));
}

constexpr QueueEventFlags operator&(QueueEventFlags lhs, QueueEventFlags rhs)
{
    return static_cast<QueueEventFlags>(detail::enumValue(lhs) & detail::enumValue(rhs));
}

constexpr QueueEventFlags& operator|=(QueueEventFlags& lhs, QueueEventFlags rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

constexpr bool any(QueueEventFlags value)
{
    return detail::enumValue(value) != 0;
}

inline FrameToken::FrameToken(uintptr_t owner_cookie, uint32_t generation, uint32_t sequence)
    : owner_cookie_{owner_cookie}, generation_{generation}, sequence_{sequence}
{
}

inline FrameToken::FrameToken(FrameToken&& other) noexcept
    : owner_cookie_{other.owner_cookie_}, generation_{other.generation_}, sequence_{other.sequence_}
{
    other.invalidate();
}

inline FrameToken& FrameToken::operator=(FrameToken&& other) noexcept
{
    if (this != &other) {
        owner_cookie_ = other.owner_cookie_;
        generation_   = other.generation_;
        sequence_     = other.sequence_;
        other.invalidate();
    }
    return *this;
}

inline void FrameToken::invalidate()
{
    owner_cookie_ = 0;
    generation_   = 0;
    sequence_     = 0;
}

inline uintptr_t SlaveQueue::ownerCookie(const SlaveQueue* queue)
{
    return reinterpret_cast<uintptr_t>(queue);
}

inline uint32_t SlaveQueue::used(uint32_t producer, uint32_t consumer)
{
    return producer - consumer;
}

inline void SlaveQueue::copyInto(uint8_t* storage, uint32_t capacity, uint32_t offset, const uint8_t* src,
                                 uint32_t size)
{
    if (size == 0) {
        return;
    }
    const uint32_t index       = offset % capacity;
    const uint32_t first_count = std::min(size, capacity - index);
    std::memcpy(storage + index, src, first_count);
    std::memcpy(storage, src + first_count, size - first_count);
}

inline void SlaveQueue::copyOut(const uint8_t* storage, uint32_t capacity, uint32_t offset, uint8_t* dst, uint32_t size)
{
    if (size == 0) {
        return;
    }
    const uint32_t index       = offset % capacity;
    const uint32_t first_count = std::min(size, capacity - index);
    std::memcpy(dst, storage + index, first_count);
    std::memcpy(dst + first_count, storage, size - first_count);
}

inline uint32_t SlaveQueue::saturatingAdd(std::atomic<uint32_t>& target, uint32_t increment)
{
    uint32_t current = target.load(std::memory_order_relaxed);
    for (;;) {
        const uint32_t maximum = std::numeric_limits<uint32_t>::max();
        const uint32_t next    = increment > maximum - current ? maximum : current + increment;
        if (target.compare_exchange_weak(current, next, std::memory_order_relaxed, std::memory_order_relaxed)) {
            return next;
        }
    }
}

inline bool SlaveQueue::bound() const
{
    return bound_;
}

inline bool SlaveQueue::empty() const
{
    const uint32_t byte_head = byte_head_.load(std::memory_order_acquire);
    const uint32_t byte_tail = byte_tail_.load(std::memory_order_acquire);
    const uint32_t desc_head = descriptor_head_.load(std::memory_order_acquire);
    const uint32_t desc_tail = descriptor_tail_.load(std::memory_order_acquire);
    return byte_head == byte_tail && desc_head == desc_tail;
}

inline bool SlaveQueue::tokenMatches(const FrameToken& token) const
{
    return token.owner_cookie_ == ownerCookie(this) && token.generation_ == generation_ && token.sequence_ != 0 &&
           token.sequence_ == reserved_sequence_;
}

inline void SlaveQueue::invalidateReservation(FrameReservation& reservation)
{
    reservation.first  = {};
    reservation.second = {};
    reservation.token.invalidate();
    reserved_byte_count_ = 0;
    reserved_byte_tail_  = 0;
    reserved_sequence_   = 0;
}

inline uint32_t SlaveQueue::nextSequence()
{
    ++sequence_;
    if (sequence_ == 0) {
        ++sequence_;
    }
    return sequence_;
}

inline data::DataSpan SlaveQueue::mutableSpan(uint32_t offset, uint32_t size)
{
    if (size == 0 || byte_capacity_ == 0) {
        return {};
    }
    const uint32_t index = offset % byte_capacity_;
    return {byte_storage_ + index, std::min(size, byte_capacity_ - index)};
}

inline data::ConstDataSpan SlaveQueue::constSpan(uint32_t offset, uint32_t size) const
{
    if (size == 0 || byte_capacity_ == 0) {
        return {};
    }
    const uint32_t index = offset % byte_capacity_;
    return {byte_storage_ + index, std::min(size, byte_capacity_ - index)};
}

inline result_t<void> SlaveQueue::bind(QueueStorage<> storage, QueueMode new_mode, uint32_t new_generation)
{
    if (bound() && (!empty() || hasReservation())) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }
    constexpr size_t maximum = std::numeric_limits<uint32_t>::max();
    if ((storage.bytes.data == nullptr && storage.bytes.size != 0) ||
        (storage.descriptors == nullptr && storage.descriptor_count != 0) || storage.bytes.size > maximum ||
        storage.descriptor_count > maximum) {
        return detail::queueError<void>(error::error_t::INVALID_ARGUMENT);
    }

    byte_storage_        = storage.bytes.data;
    descriptor_storage_  = storage.descriptors;
    byte_capacity_       = static_cast<uint32_t>(storage.bytes.size);
    descriptor_capacity_ = static_cast<uint32_t>(storage.descriptor_count);
    bound_               = true;
    mode_.store(static_cast<uint8_t>(new_mode), std::memory_order_relaxed);
    reset(new_generation);
    clearStatus();
    return {};
}

inline result_t<void> SlaveQueue::setMode(QueueMode new_mode)
{
    if (!bound()) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }
    if (mode() == new_mode) {
        return {};
    }
    if (!empty() || hasReservation()) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }
    mode_.store(static_cast<uint8_t>(new_mode), std::memory_order_release);
    return {};
}

inline QueueMode SlaveQueue::mode() const
{
    return static_cast<QueueMode>(mode_.load(std::memory_order_acquire));
}

inline ByteQueueView SlaveQueue::bytes()
{
    return ByteQueueView{this};
}

inline FrameSourceView SlaveQueue::frameSource()
{
    return FrameSourceView{this};
}

inline FrameSinkView SlaveQueue::frameSink()
{
    return FrameSinkView{this};
}

inline result_t<size_t> SlaveQueue::write(data::ConstDataSpan src)
{
    if (!bound() || mode() != QueueMode::Byte || hasReservation()) {
        return detail::queueError<size_t>(error::error_t::INVALID_STATE);
    }
    if (src.data == nullptr && src.size != 0) {
        return detail::queueError<size_t>(error::error_t::INVALID_ARGUMENT);
    }
    if (src.size == 0) {
        return size_t{0};
    }

    const uint32_t head      = byte_head_.load(std::memory_order_acquire);
    const uint32_t tail      = byte_tail_.load(std::memory_order_relaxed);
    const uint32_t available = byte_capacity_ - used(tail, head);
    const uint32_t accepted  = static_cast<uint32_t>(std::min<size_t>(src.size, available));
    if (accepted == 0) {
        return detail::queueError<size_t>(error::error_t::WOULD_BLOCK);
    }
    copyInto(byte_storage_, byte_capacity_, tail, src.data, accepted);
    byte_tail_.store(tail + accepted, std::memory_order_release);
    return static_cast<size_t>(accepted);
}

inline result_t<size_t> SlaveQueue::read(data::DataSpan dst)
{
    if (!bound() || mode() != QueueMode::Byte || hasReservation()) {
        return detail::queueError<size_t>(error::error_t::INVALID_STATE);
    }
    if (dst.data == nullptr && dst.size != 0) {
        return detail::queueError<size_t>(error::error_t::INVALID_ARGUMENT);
    }
    if (dst.size == 0) {
        return size_t{0};
    }

    const uint32_t head      = byte_head_.load(std::memory_order_relaxed);
    const uint32_t tail      = byte_tail_.load(std::memory_order_acquire);
    const uint32_t available = used(tail, head);
    const uint32_t count     = static_cast<uint32_t>(std::min<size_t>(dst.size, available));
    if (count == 0) {
        return detail::queueError<size_t>(error::error_t::WOULD_BLOCK);
    }
    copyOut(byte_storage_, byte_capacity_, head, dst.data, count);
    byte_head_.store(head + count, std::memory_order_release);
    return static_cast<size_t>(count);
}

inline result_t<ByteView> SlaveQueue::peekBytes(size_t maximum) const
{
    if (!bound() || mode() != QueueMode::Byte) {
        return detail::queueError<ByteView>(error::error_t::INVALID_STATE);
    }
    const uint32_t head      = byte_head_.load(std::memory_order_relaxed);
    const uint32_t tail      = byte_tail_.load(std::memory_order_acquire);
    const uint32_t available = used(tail, head);
    uint32_t count           = available;
    if (maximum < count) {
        count = static_cast<uint32_t>(maximum);
    }
    if (count == 0 && maximum != 0) {
        return detail::queueError<ByteView>(error::error_t::WOULD_BLOCK);
    }
    ByteView view;
    view.first = constSpan(head, count);
    if (view.first.size < count) {
        view.second =
            constSpan(head + static_cast<uint32_t>(view.first.size), count - static_cast<uint32_t>(view.first.size));
    }
    return view;
}

inline result_t<void> SlaveQueue::popBytes(size_t count)
{
    if (!bound() || mode() != QueueMode::Byte) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }
    const uint32_t head      = byte_head_.load(std::memory_order_relaxed);
    const uint32_t tail      = byte_tail_.load(std::memory_order_acquire);
    const uint32_t available = used(tail, head);
    if (count > available || count > std::numeric_limits<uint32_t>::max()) {
        return detail::queueError<void>(error::error_t::INVALID_ARGUMENT);
    }
    byte_head_.store(head + static_cast<uint32_t>(count), std::memory_order_release);
    return {};
}

inline size_t SlaveQueue::readable() const
{
    const uint32_t head = byte_head_.load(std::memory_order_relaxed);
    const uint32_t tail = byte_tail_.load(std::memory_order_acquire);
    return used(tail, head);
}

inline size_t SlaveQueue::writable() const
{
    const uint32_t head = byte_head_.load(std::memory_order_acquire);
    const uint32_t tail = byte_tail_.load(std::memory_order_relaxed);
    return byte_capacity_ - used(tail, head) - reserved_byte_count_;
}

inline result_t<FrameReservation> SlaveQueue::reserveFrame(size_t payload_size)
{
    if (!bound() || mode() != QueueMode::Frame || hasReservation()) {
        return detail::queueError<FrameReservation>(error::error_t::INVALID_STATE);
    }
    if (payload_size > byte_capacity_ || payload_size > std::numeric_limits<uint32_t>::max()) {
        return detail::queueError<FrameReservation>(error::error_t::INVALID_ARGUMENT);
    }

    const uint32_t desc_head = descriptor_head_.load(std::memory_order_acquire);
    const uint32_t desc_tail = descriptor_tail_.load(std::memory_order_relaxed);
    const uint32_t byte_head = byte_head_.load(std::memory_order_acquire);
    const uint32_t byte_tail = byte_tail_.load(std::memory_order_relaxed);
    if (used(desc_tail, desc_head) >= descriptor_capacity_ ||
        static_cast<uint32_t>(payload_size) > byte_capacity_ - used(byte_tail, byte_head)) {
        return detail::queueError<FrameReservation>(error::error_t::WOULD_BLOCK);
    }

    reserved_byte_tail_  = byte_tail;
    reserved_byte_count_ = static_cast<uint32_t>(payload_size);
    reserved_sequence_   = nextSequence();

    FrameReservation reservation;
    reservation.first  = mutableSpan(byte_tail, reserved_byte_count_);
    reservation.second = mutableSpan(byte_tail + static_cast<uint32_t>(reservation.first.size),
                                     reserved_byte_count_ - static_cast<uint32_t>(reservation.first.size));
    reservation.token  = FrameToken{ownerCookie(this), generation_, reserved_sequence_};
    return result_t<FrameReservation>{m5::stl::in_place, std::move(reservation)};
}

inline result_t<FrameReservation> SlaveQueue::beginObservedFrame()
{
    return reserveFrame(0);
}

inline result_t<size_t> SlaveQueue::appendObservedFrame(FrameReservation& reservation, data::ConstDataSpan payload)
{
    if (mode() != QueueMode::Frame || !tokenMatches(reservation.token)) {
        return detail::queueError<size_t>(error::error_t::INVALID_STATE);
    }
    if (payload.data == nullptr && payload.size != 0) {
        return detail::queueError<size_t>(error::error_t::INVALID_ARGUMENT);
    }
    if (payload.size == 0) {
        return size_t{0};
    }

    const uint32_t byte_head = byte_head_.load(std::memory_order_acquire);
    const uint32_t available = byte_capacity_ - used(reserved_byte_tail_ + reserved_byte_count_, byte_head);
    const uint32_t accepted  = static_cast<uint32_t>(std::min<size_t>(payload.size, available));
    if (accepted == 0) {
        return detail::queueError<size_t>(error::error_t::WOULD_BLOCK);
    }
    copyInto(byte_storage_, byte_capacity_, reserved_byte_tail_ + reserved_byte_count_, payload.data, accepted);
    reserved_byte_count_ += accepted;
    return static_cast<size_t>(accepted);
}

inline result_t<void> SlaveQueue::commitFrame(FrameReservation& reservation, const FrameMetadata& metadata)
{
    if (mode() != QueueMode::Frame || !tokenMatches(reservation.token)) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }

    const uint32_t desc_tail    = descriptor_tail_.load(std::memory_order_relaxed);
    FrameDescriptor& descriptor = descriptor_storage_[desc_tail % descriptor_capacity_];
    descriptor.offset           = reserved_byte_tail_;
    descriptor.length           = reserved_byte_count_;
    descriptor.metadata         = metadata;

    // Publish payload before the descriptor index. A consumer acquiring the
    // descriptor tail therefore observes both descriptor and payload bytes.
    byte_tail_.store(reserved_byte_tail_ + reserved_byte_count_, std::memory_order_release);
    descriptor_tail_.store(desc_tail + 1, std::memory_order_release);
    invalidateReservation(reservation);
    return {};
}

inline result_t<void> SlaveQueue::cancelFrame(FrameReservation& reservation)
{
    if (mode() != QueueMode::Frame || !tokenMatches(reservation.token)) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }
    invalidateReservation(reservation);
    return {};
}

inline result_t<void> SlaveQueue::writeFrame(data::ConstDataSpan payload, const FrameMetadata& metadata)
{
    if (payload.data == nullptr && payload.size != 0) {
        return detail::queueError<void>(error::error_t::INVALID_ARGUMENT);
    }
    auto reservation = reserveFrame(payload.size);
    if (!reservation) {
        return detail::queueError<void>(reservation.error());
    }
    if (reservation->first.size != 0) {
        std::memcpy(reservation->first.data, payload.data, reservation->first.size);
    }
    if (reservation->second.size != 0) {
        std::memcpy(reservation->second.data, payload.data + reservation->first.size, reservation->second.size);
    }
    return commitFrame(*reservation, metadata);
}

inline result_t<void> SlaveQueue::writeObservedFrame(data::ConstDataSpan payload, FrameMetadata metadata)
{
    if (!bound() || mode() != QueueMode::Frame) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }
    if (payload.data == nullptr && payload.size != 0) {
        return detail::queueError<void>(error::error_t::INVALID_ARGUMENT);
    }
    const size_t stored = payload.size < writable() ? payload.size : writable();
    auto reservation    = reserveFrame(stored);
    if (!reservation.has_value()) {
        if (reservation.error() == error::error_t::WOULD_BLOCK) {
            recordDroppedFrame(static_cast<uint32_t>(payload.size > UINT32_MAX ? UINT32_MAX : payload.size));
        }
        return m5::stl::make_unexpected(reservation.error());
    }
    if (stored != 0) {
        const size_t first_size = reservation->first.size;
        std::memcpy(reservation->first.data, payload.data, first_size);
        if (reservation->second.size != 0) {
            std::memcpy(reservation->second.data, payload.data + first_size, reservation->second.size);
        }
    }
    metadata.wire_bytes   = static_cast<uint32_t>(payload.size > UINT32_MAX ? UINT32_MAX : payload.size);
    metadata.stored_bytes = static_cast<uint32_t>(stored);
    metadata.dropped_bytes =
        static_cast<uint32_t>(payload.size - stored > UINT32_MAX ? UINT32_MAX : payload.size - stored);
    if (stored != payload.size) {
        metadata.flags |= FrameFlags::Overflow | FrameFlags::Truncated;
        recordDroppedFrames(0, metadata.dropped_bytes);
    }
    return commitFrame(*reservation, metadata);
}

inline result_t<FrameView> SlaveQueue::peekFrame() const
{
    if (!bound() || mode() != QueueMode::Frame) {
        return detail::queueError<FrameView>(error::error_t::INVALID_STATE);
    }
    const uint32_t desc_head = descriptor_head_.load(std::memory_order_relaxed);
    const uint32_t desc_tail = descriptor_tail_.load(std::memory_order_acquire);
    if (desc_head == desc_tail) {
        return detail::queueError<FrameView>(error::error_t::WOULD_BLOCK);
    }

    const FrameDescriptor& descriptor = descriptor_storage_[desc_head % descriptor_capacity_];
    FrameView view;
    view.first    = constSpan(descriptor.offset, descriptor.length);
    view.second   = constSpan(descriptor.offset + static_cast<uint32_t>(view.first.size),
                              descriptor.length - static_cast<uint32_t>(view.first.size));
    view.metadata = descriptor.metadata;
    return view;
}

inline result_t<void> SlaveQueue::popFrame()
{
    if (!bound() || mode() != QueueMode::Frame) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }
    const uint32_t desc_head = descriptor_head_.load(std::memory_order_relaxed);
    const uint32_t desc_tail = descriptor_tail_.load(std::memory_order_acquire);
    if (desc_head == desc_tail) {
        return detail::queueError<void>(error::error_t::WOULD_BLOCK);
    }

    const FrameDescriptor& descriptor = descriptor_storage_[desc_head % descriptor_capacity_];
    const uint32_t byte_head          = byte_head_.load(std::memory_order_relaxed);
    byte_head_.store(byte_head + descriptor.length, std::memory_order_release);
    descriptor_head_.store(desc_head + 1, std::memory_order_release);
    return {};
}

inline size_t SlaveQueue::readableFrames() const
{
    const uint32_t head = descriptor_head_.load(std::memory_order_relaxed);
    const uint32_t tail = descriptor_tail_.load(std::memory_order_acquire);
    return used(tail, head);
}

inline size_t SlaveQueue::writableFrames() const
{
    const uint32_t head = descriptor_head_.load(std::memory_order_acquire);
    const uint32_t tail = descriptor_tail_.load(std::memory_order_relaxed);
    return descriptor_capacity_ - used(tail, head) - (hasReservation() ? 1u : 0u);
}

inline bool SlaveQueue::hasReservation() const
{
    return reserved_sequence_ != 0;
}

inline result_t<void> SlaveQueue::cancelReservation()
{
    if (!hasReservation()) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }
    reserved_byte_tail_  = 0;
    reserved_byte_count_ = 0;
    reserved_sequence_   = 0;
    return {};
}

inline result_t<void> SlaveQueue::setGeneration(uint32_t new_generation)
{
    if (!bound() || new_generation == 0 || hasReservation()) {
        return detail::queueError<void>(error::error_t::INVALID_STATE);
    }
    generation_ = new_generation;
    return {};
}

inline void SlaveQueue::reset(uint32_t new_generation)
{
    byte_head_.store(0, std::memory_order_relaxed);
    byte_tail_.store(0, std::memory_order_relaxed);
    descriptor_head_.store(0, std::memory_order_relaxed);
    descriptor_tail_.store(0, std::memory_order_relaxed);
    generation_          = new_generation;
    reserved_byte_tail_  = 0;
    reserved_byte_count_ = 0;
    reserved_sequence_   = 0;
}

inline uint32_t SlaveQueue::generation() const
{
    return generation_;
}

inline QueueStatus SlaveQueue::status() const
{
    return {dropped_bytes_.load(std::memory_order_relaxed), dropped_frames_.load(std::memory_order_relaxed),
            underrun_count_.load(std::memory_order_relaxed),
            static_cast<QueueEventFlags>(sticky_events_.load(std::memory_order_relaxed))};
}

inline QueueStatus SlaveQueue::clearStatus()
{
    return {dropped_bytes_.exchange(0, std::memory_order_relaxed),
            dropped_frames_.exchange(0, std::memory_order_relaxed),
            underrun_count_.exchange(0, std::memory_order_relaxed),
            static_cast<QueueEventFlags>(sticky_events_.exchange(0, std::memory_order_relaxed))};
}

inline void SlaveQueue::recordDroppedFrame(uint32_t dropped_bytes)
{
    recordDroppedFrames(1, dropped_bytes);
}

inline void SlaveQueue::recordDroppedFrames(uint32_t frame_count, uint32_t dropped_bytes)
{
    saturatingAdd(dropped_bytes_, dropped_bytes);
    saturatingAdd(dropped_frames_, frame_count);
    if (frame_count != 0 || dropped_bytes != 0) {
        sticky_events_.fetch_or(detail::enumValue(QueueEventFlags::Overflow | QueueEventFlags::Truncated),
                                std::memory_order_relaxed);
    }
}

inline void SlaveQueue::recordUnderrun(uint32_t count)
{
    saturatingAdd(underrun_count_, count);
    if (count != 0) {
        sticky_events_.fetch_or(detail::enumValue(QueueEventFlags::Underrun), std::memory_order_relaxed);
    }
}

inline result_t<size_t> ByteQueueView::write(data::ConstDataSpan src)
{
    return queue_ == nullptr ? detail::queueError<size_t>(error::error_t::INVALID_STATE) : queue_->write(src);
}

inline result_t<size_t> ByteQueueView::read(data::DataSpan dst)
{
    return queue_ == nullptr ? detail::queueError<size_t>(error::error_t::INVALID_STATE) : queue_->read(dst);
}

inline size_t ByteQueueView::readable() const
{
    return queue_ == nullptr ? 0 : queue_->readable();
}

inline size_t ByteQueueView::writable() const
{
    return queue_ == nullptr ? 0 : queue_->writable();
}

inline result_t<FrameView> FrameSourceView::peekFrame() const
{
    return queue_ == nullptr ? detail::queueError<FrameView>(error::error_t::INVALID_STATE) : queue_->peekFrame();
}

inline result_t<void> FrameSourceView::popFrame()
{
    return queue_ == nullptr ? detail::queueError<void>(error::error_t::INVALID_STATE) : queue_->popFrame();
}

inline size_t FrameSourceView::readableFrames() const
{
    return queue_ == nullptr ? 0 : queue_->readableFrames();
}

inline result_t<FrameReservation> FrameSinkView::reserveFrame(size_t payload_size)
{
    return queue_ == nullptr ? detail::queueError<FrameReservation>(error::error_t::INVALID_STATE)
                             : queue_->reserveFrame(payload_size);
}

inline result_t<void> FrameSinkView::commitFrame(FrameReservation& reservation, const FrameMetadata& metadata)
{
    return queue_ == nullptr ? detail::queueError<void>(error::error_t::INVALID_STATE)
                             : queue_->commitFrame(reservation, metadata);
}

inline result_t<void> FrameSinkView::cancelFrame(FrameReservation& reservation)
{
    return queue_ == nullptr ? detail::queueError<void>(error::error_t::INVALID_STATE)
                             : queue_->cancelFrame(reservation);
}

inline result_t<void> FrameSinkView::writeFrame(data::ConstDataSpan payload, const FrameMetadata& metadata)
{
    return queue_ == nullptr ? detail::queueError<void>(error::error_t::INVALID_STATE)
                             : queue_->writeFrame(payload, metadata);
}

inline size_t FrameSinkView::writableBytes() const
{
    return queue_ == nullptr ? 0 : queue_->writable();
}

inline size_t FrameSinkView::writableFrames() const
{
    return queue_ == nullptr ? 0 : queue_->writableFrames();
}

}  // namespace m5::hal::v2::slave

#endif
