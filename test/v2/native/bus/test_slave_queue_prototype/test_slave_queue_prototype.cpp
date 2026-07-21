// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <type_traits>
#include <utility>

namespace {

// Test-only prototype. Exact public types remain intentionally undecided.

using m5::hal::v2::result_t;
using m5::hal::v2::data::ConstDataSpan;
using m5::hal::v2::data::DataSpan;
using m5::hal::v2::error::error_t;

#define ASSERT_RESULT_OK(expr)                                                                           \
    do {                                                                                                 \
        auto result_ok = (expr);                                                                         \
        ASSERT_TRUE(result_ok.has_value()) << "err=" << m5::hal::v2::error::toString(result_ok.error()); \
    } while (false)

constexpr uint16_t flagBit(unsigned bit)
{
    return static_cast<uint16_t>(1u << bit);
}

enum class Protocol : uint8_t { Generic, I2c, Spi };
enum class SegmentDirection : uint8_t { Write, Read, Duplex };
enum class SegmentFlags : uint8_t { None = 0, RepeatedStart = 1, ChipSelect = 2 };

enum FrameFlags : uint16_t {
    FrameBegin = flagBit(0),
    FrameEnd   = flagBit(1),
    Aborted    = flagBit(2),
    Truncated  = flagBit(3),
    Overflow   = flagBit(4),
    Underrun   = flagBit(5),
};

struct Segment {
    uint16_t offset            = 0;
    uint16_t length            = 0;
    SegmentDirection direction = SegmentDirection::Write;
    SegmentFlags flags         = SegmentFlags::None;
};

struct FrameMetadata {
    uint32_t frame_id      = 0;
    uint16_t wire_bytes    = 0;
    uint16_t stored_bytes  = 0;
    uint16_t dropped_bytes = 0;
    uint16_t flags         = FrameBegin | FrameEnd;
    Protocol protocol      = Protocol::Generic;
    uint8_t segment_count  = 0;
    std::array<Segment, 3> segments{};
};

struct FrameDescriptor {
    uint16_t start  = 0;
    uint16_t length = 0;
    FrameMetadata metadata{};
};

struct QueueStatus {
    uint32_t dropped_bytes  = 0;
    uint32_t dropped_frames = 0;
    uint32_t underrun_count = 0;
    uint16_t sticky_events  = 0;
};

uint32_t saturatingAdd(uint32_t value, size_t increment)
{
    const auto max = std::numeric_limits<uint32_t>::max();
    if (increment >= max || value > max - static_cast<uint32_t>(increment)) {
        return max;
    }
    return value + static_cast<uint32_t>(increment);
}

struct StorageAudit {
    static size_t allocation_attempts;

    static void bind(const void*, size_t)
    {
        // Caller storage is only borrowed; binding is not an allocation.
    }

    static void* allocate(size_t)
    {
        ++allocation_attempts;
        return nullptr;
    }
};

size_t StorageAudit::allocation_attempts = 0;

class ByteRing {
public:
    ByteRing(uint8_t* storage, size_t capacity) : storage_{storage}, capacity_{capacity}
    {
        StorageAudit::bind(storage, capacity);
    }

    size_t readable() const
    {
        return used_;
    }

    size_t writable() const
    {
        return capacity_ - used_ - reserved_;
    }

    size_t head() const
    {
        return head_;
    }

    size_t tail() const
    {
        return capacity_ == 0 ? 0 : (head_ + used_) % capacity_;
    }

    result_t<size_t> write(ConstDataSpan src)
    {
        if (src.data == nullptr && src.size != 0) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
        const size_t accepted = std::min(src.size, writable());
        if (accepted == 0 && src.size != 0) {
            // BUSY is the current closest error to the proposed WOULD_BLOCK result.
            return m5::stl::make_unexpected(error_t::BUSY);
        }
        copyIn(tail(), src.data, accepted);
        used_ += accepted;
        return accepted;
    }

    result_t<size_t> read(DataSpan dst)
    {
        if (dst.data == nullptr && dst.size != 0) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
        const size_t count = std::min(dst.size, readable());
        copyOut(head_, dst.data, count);
        consume(count);
        return count;
    }

    result_t<std::pair<DataSpan, DataSpan>> reserve(size_t size)
    {
        if (reserved_ != 0 || zero_reservation_) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (size > writable()) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        reserved_           = size;
        zero_reservation_   = size == 0;
        const size_t start  = tail();
        const size_t first  = capacity_ == 0 ? 0 : std::min(size, capacity_ - start);
        const size_t second = size - first;
        return std::pair<DataSpan, DataSpan>{{storage_ + start, first}, {storage_, second}};
    }

    void publishReservation()
    {
        used_ += reserved_;
        reserved_         = 0;
        zero_reservation_ = false;
    }

    void cancelReservation()
    {
        reserved_         = 0;
        zero_reservation_ = false;
    }

    std::pair<ConstDataSpan, ConstDataSpan> view(size_t start, size_t size) const
    {
        const size_t first = capacity_ == 0 ? 0 : std::min(size, capacity_ - start);
        return {{storage_ + start, first}, {storage_, size - first}};
    }

    void consume(size_t size)
    {
        const size_t count = std::min(size, used_);
        if (capacity_ != 0) {
            head_ = (head_ + count) % capacity_;
        }
        used_ -= count;
    }

private:
    void copyIn(size_t start, const uint8_t* src, size_t size)
    {
        if (size == 0) {
            return;
        }
        const size_t first = std::min(size, capacity_ - start);
        std::memcpy(storage_ + start, src, first);
        std::memcpy(storage_, src + first, size - first);
    }

    void copyOut(size_t start, uint8_t* dst, size_t size) const
    {
        if (size == 0) {
            return;
        }
        const size_t first = std::min(size, capacity_ - start);
        std::memcpy(dst, storage_ + start, first);
        std::memcpy(dst + first, storage_, size - first);
    }

    uint8_t* storage_      = nullptr;
    size_t capacity_       = 0;
    size_t head_           = 0;
    size_t used_           = 0;
    size_t reserved_       = 0;
    bool zero_reservation_ = false;
};

class DescriptorRing {
public:
    DescriptorRing(FrameDescriptor* storage, size_t capacity) : storage_{storage}, capacity_{capacity}
    {
        StorageAudit::bind(storage, capacity * sizeof(FrameDescriptor));
    }

    bool empty() const
    {
        return used_ == 0;
    }

    bool full() const
    {
        return used_ == capacity_;
    }

    size_t size() const
    {
        return used_;
    }

    FrameDescriptor& front()
    {
        return storage_[head_];
    }

    const FrameDescriptor& front() const
    {
        return storage_[head_];
    }

    bool push(const FrameDescriptor& descriptor)
    {
        if (full()) {
            return false;
        }
        storage_[(head_ + used_) % capacity_] = descriptor;
        ++used_;
        return true;
    }

    void pop()
    {
        if (empty()) {
            return;
        }
        head_ = (head_ + 1) % capacity_;
        --used_;
    }

    void clear()
    {
        head_ = 0;
        used_ = 0;
    }

private:
    FrameDescriptor* storage_ = nullptr;
    size_t capacity_          = 0;
    size_t head_              = 0;
    size_t used_              = 0;
};

class FrameToken {
public:
    FrameToken()                             = default;
    FrameToken(const FrameToken&)            = delete;
    FrameToken& operator=(const FrameToken&) = delete;

    FrameToken(FrameToken&& other) noexcept
        : owner_{other.owner_}, generation_{other.generation_}, sequence_{other.sequence_}, valid_{other.valid_}
    {
        other.valid_ = false;
    }

    FrameToken& operator=(FrameToken&& other) noexcept
    {
        if (this != &other) {
            owner_       = other.owner_;
            generation_  = other.generation_;
            sequence_    = other.sequence_;
            valid_       = other.valid_;
            other.valid_ = false;
        }
        return *this;
    }

private:
    friend class FrameQueue;
    FrameToken(const void* owner, uint32_t generation, uint32_t sequence)
        : owner_{owner}, generation_{generation}, sequence_{sequence}, valid_{true}
    {
    }

    const void* owner_   = nullptr;
    uint32_t generation_ = 0;
    uint32_t sequence_   = 0;
    bool valid_          = false;
};

struct FrameReservation {
    DataSpan first{};
    DataSpan second{};
    FrameToken token{};

    FrameReservation() = default;
    FrameReservation(DataSpan a, DataSpan b, FrameToken&& t) : first{a}, second{b}, token{std::move(t)}
    {
    }
    FrameReservation(const FrameReservation&)                = delete;
    FrameReservation& operator=(const FrameReservation&)     = delete;
    FrameReservation(FrameReservation&&) noexcept            = default;
    FrameReservation& operator=(FrameReservation&&) noexcept = default;
};

static_assert(!std::is_copy_constructible<FrameToken>::value, "FrameToken must be move-only");
static_assert(std::is_move_constructible<FrameToken>::value, "FrameToken must remain movable");

struct FrameView {
    ConstDataSpan first{};
    ConstDataSpan second{};
    FrameMetadata metadata{};
};

class FrameQueue {
public:
    FrameQueue(uint8_t* byte_storage, size_t byte_capacity, FrameDescriptor* descriptor_storage,
               size_t descriptor_capacity)
        : bytes_{byte_storage, byte_capacity}, descriptors_{descriptor_storage, descriptor_capacity}
    {
    }

    size_t readable() const
    {
        return bytes_.readable();
    }

    size_t writable() const
    {
        return bytes_.writable();
    }

    size_t frameCount() const
    {
        return descriptors_.size();
    }

    const QueueStatus& status() const
    {
        return status_;
    }

    result_t<size_t> write(ConstDataSpan src)
    {
        return bytes_.write(src);
    }

    result_t<size_t> read(DataSpan dst)
    {
        return bytes_.read(dst);
    }

    result_t<FrameReservation> reserveFrame(size_t payload_size)
    {
        if (reservation_active_) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (descriptors_.full()) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        const size_t start = bytes_.tail();
        auto spans         = bytes_.reserve(payload_size);
        if (!spans.has_value()) {
            return m5::stl::make_unexpected(spans.error());
        }
        reservation_active_ = true;
        reservation_start_  = start;
        reservation_size_   = payload_size;
        active_sequence_    = ++sequence_;
        return FrameReservation{spans.value().first, spans.value().second,
                                FrameToken{this, generation_, active_sequence_}};
    }

    result_t<void> commitFrame(FrameReservation& reservation, FrameMetadata metadata)
    {
        auto valid = validate(reservation);
        if (!valid.has_value()) {
            return valid;
        }
        metadata.stored_bytes = static_cast<uint16_t>(std::min<size_t>(reservation_size_, UINT16_MAX));
        if (metadata.wire_bytes == 0 && reservation_size_ != 0) {
            metadata.wire_bytes = metadata.stored_bytes;
        }
        FrameDescriptor descriptor{static_cast<uint16_t>(reservation_start_), static_cast<uint16_t>(reservation_size_),
                                   metadata};
        if (!descriptors_.push(descriptor)) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        bytes_.publishReservation();
        invalidate(reservation);
        return {};
    }

    result_t<void> cancelFrame(FrameReservation& reservation)
    {
        auto valid = validate(reservation);
        if (!valid.has_value()) {
            return valid;
        }
        bytes_.cancelReservation();
        invalidate(reservation);
        return {};
    }

    result_t<FrameView> peekFrame() const
    {
        if (descriptors_.empty()) {
            return m5::stl::make_unexpected(error_t::BUFFER_UNDERFLOW);
        }
        const auto& descriptor = descriptors_.front();
        auto spans             = bytes_.view(descriptor.start, descriptor.length);
        return FrameView{spans.first, spans.second, descriptor.metadata};
    }

    result_t<void> popFrame()
    {
        if (descriptors_.empty()) {
            return m5::stl::make_unexpected(error_t::BUFFER_UNDERFLOW);
        }
        const size_t count = descriptors_.front().length;
        bytes_.consume(count);
        descriptors_.pop();
        return {};
    }

    result_t<size_t> ingestFrame(ConstDataSpan src, FrameMetadata metadata)
    {
        if (src.data == nullptr && src.size != 0) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
        if (descriptors_.full()) {
            status_.dropped_frames = saturatingAdd(status_.dropped_frames, 1);
            status_.dropped_bytes  = saturatingAdd(status_.dropped_bytes, src.size);
            status_.sticky_events |= Overflow;
            return m5::stl::make_unexpected(error_t::BUFFER_OVERFLOW);
        }
        const size_t accepted = std::min(src.size, bytes_.writable());
        const size_t start    = bytes_.tail();
        auto written          = bytes_.write({src.data, accepted});
        if (!written.has_value()) {
            return m5::stl::make_unexpected(written.error());
        }
        const size_t dropped   = src.size - accepted;
        metadata.wire_bytes    = static_cast<uint16_t>(std::min<size_t>(src.size, UINT16_MAX));
        metadata.stored_bytes  = static_cast<uint16_t>(std::min<size_t>(accepted, UINT16_MAX));
        metadata.dropped_bytes = static_cast<uint16_t>(std::min<size_t>(dropped, UINT16_MAX));
        if (dropped != 0) {
            metadata.flags |= Overflow | Truncated;
            status_.dropped_bytes = saturatingAdd(status_.dropped_bytes, dropped);
            status_.sticky_events |= Overflow;
        }
        const bool pushed =
            descriptors_.push({static_cast<uint16_t>(start), static_cast<uint16_t>(accepted), metadata});
        if (!pushed) {
            return m5::stl::make_unexpected(error_t::OUT_OF_RESOURCE);
        }
        return accepted;
    }

    result_t<size_t> readShared(DataSpan dst, size_t& metadata_updates)
    {
        if (dst.data == nullptr && dst.size != 0) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
        const size_t target = std::min(dst.size, bytes_.readable());
        auto result         = bytes_.read({dst.data, target});
        if (!result.has_value()) {
            return result;
        }
        size_t remaining = target;
        while (remaining != 0 && !descriptors_.empty()) {
            auto& descriptor = descriptors_.front();
            if (descriptor.length <= remaining) {
                remaining -= descriptor.length;
                descriptors_.pop();
                ++metadata_updates;
            } else {
                descriptor.start  = static_cast<uint16_t>((descriptor.start + remaining) % byteCapacityForIndex());
                descriptor.length = static_cast<uint16_t>(descriptor.length - remaining);
                descriptor.metadata.stored_bytes = descriptor.length;
                remaining                        = 0;
                ++metadata_updates;
            }
        }
        return result;
    }

    void discardFrameMetadata()
    {
        descriptors_.clear();
    }

    void invalidateGenerationForTest()
    {
        ++generation_;
        if (reservation_active_) {
            bytes_.cancelReservation();
            reservation_active_ = false;
        }
    }

private:
    result_t<void> validate(const FrameReservation& reservation) const
    {
        const auto& token = reservation.token;
        if (!token.valid_ || token.owner_ != this || token.generation_ != generation_ || !reservation_active_ ||
            token.sequence_ != active_sequence_) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        return {};
    }

    void invalidate(FrameReservation& reservation)
    {
        reservation.token.valid_ = false;
        reservation_active_      = false;
        reservation_size_        = 0;
    }

    size_t byteCapacityForIndex() const
    {
        return bytes_.readable() + bytes_.writable();
    }

    ByteRing bytes_;
    DescriptorRing descriptors_;
    QueueStatus status_{};
    uint32_t generation_      = 1;
    uint32_t sequence_        = 0;
    uint32_t active_sequence_ = 0;
    size_t reservation_start_ = 0;
    size_t reservation_size_  = 0;
    bool reservation_active_  = false;
};

struct WorkMetrics {
    size_t api_calls        = 0;
    size_t copied_bytes     = 0;
    size_t metadata_updates = 0;
    uint64_t checksum       = 0;
};

class FixedModeConsumer {
public:
    enum class Mode { None, Byte, Frame };

    explicit FixedModeConsumer(FrameQueue& queue) : queue_{queue}
    {
    }

    result_t<size_t> read(DataSpan dst, WorkMetrics& metrics)
    {
        ++metrics.api_calls;
        if (mode_ == Mode::Frame && queue_.readable() != 0) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        mode_       = Mode::Byte;
        auto result = queue_.read(dst);
        if (result.has_value()) {
            metrics.copied_bytes += result.value();
            for (size_t i = 0; i < result.value(); ++i) {
                metrics.checksum += dst.data[i];
            }
            if (queue_.readable() == 0) {
                queue_.discardFrameMetadata();
                mode_ = Mode::None;
            }
        }
        return result;
    }

    result_t<FrameView> peekFrame(WorkMetrics& metrics)
    {
        ++metrics.api_calls;
        if (mode_ == Mode::Byte && queue_.readable() != 0) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        mode_ = Mode::Frame;
        return queue_.peekFrame();
    }

private:
    FrameQueue& queue_;
    Mode mode_ = Mode::None;
};

class SharedCursorConsumer {
public:
    explicit SharedCursorConsumer(FrameQueue& queue) : queue_{queue}
    {
    }

    result_t<size_t> read(DataSpan dst, WorkMetrics& metrics)
    {
        ++metrics.api_calls;
        auto result = queue_.readShared(dst, metrics.metadata_updates);
        if (result.has_value()) {
            metrics.copied_bytes += result.value();
            for (size_t i = 0; i < result.value(); ++i) {
                metrics.checksum += dst.data[i];
            }
        }
        return result;
    }

    result_t<FrameView> peekFrame(WorkMetrics& metrics)
    {
        ++metrics.api_calls;
        auto view = queue_.peekFrame();
        if (view.has_value()) {
            for (const auto byte : view.value().first) {
                metrics.checksum += byte;
            }
            for (const auto byte : view.value().second) {
                metrics.checksum += byte;
            }
        }
        return view;
    }

    result_t<void> popFrame(WorkMetrics& metrics)
    {
        ++metrics.api_calls;
        return queue_.popFrame();
    }

private:
    FrameQueue& queue_;
};

struct TentativeOperationContext {
    uint32_t deadline       = 0;
    uint32_t generation     = 0;
    uint16_t state_flags    = 0;
    uint8_t mode            = 0;
    uint8_t config_revision = 0;
};

struct TentativeAccessorLayout {
    FrameQueue* tx = nullptr;
    FrameQueue* rx = nullptr;
    TentativeOperationContext context{};
    uint32_t transfer_id    = 0;
    uint8_t lifecycle_state = 0;
};

enum class BusKind : uint8_t { I2c, Spi };
enum class OperationMode : uint8_t { Master, Slave };

enum class TraceEvent : uint8_t {
    Lock,
    BeginOperation,
    RollbackBegin,
    Active,
    Body,
    EndOperation,
    StopAccepting,
    GracefulWait,
    AbortFrame,
    ResetPeripheral,
    Unlock,
};

class TraceLog {
public:
    void push(TraceEvent event)
    {
        if (size_ < events_.size()) {
            events_[size_++] = event;
        }
    }

    size_t size() const
    {
        return size_;
    }

    TraceEvent operator[](size_t index) const
    {
        return events_[index];
    }

    size_t count(TraceEvent event) const
    {
        size_t result = 0;
        for (size_t i = 0; i < size_; ++i) {
            result += events_[i] == event;
        }
        return result;
    }

    void clear()
    {
        size_ = 0;
    }

private:
    std::array<TraceEvent, 64> events_{};
    size_t size_ = 0;
};

enum SlaveEvent : uint16_t {
    EventNone             = 0,
    EventRxAvailable      = flagBit(0),
    EventFrameCompleted   = flagBit(1),
    EventTxSpaceAvailable = flagBit(2),
    EventFrameAborted     = flagBit(3),
    EventBusBroken        = flagBit(4),
};

struct SlaveEventInfo {
    uint16_t events     = EventNone;
    size_t rx_available = 0;
    size_t tx_available = 0;
    uint32_t generation = 0;
};

using SlaveEventCallback = void (*)(void*, const SlaveEventInfo&);
using LevelProbe         = uint16_t (*)(void*);
using RaceInjection      = void (*)(void*);

class EventNotifier {
public:
    enum class RacePoint : uint8_t { None, BeforePendingClear, AfterPendingClear };

    result_t<void> setCallback(SlaveEventCallback callback, void* user)
    {
        callback_      = callback;
        callback_user_ = user;
        return {};
    }

    void setLevelProbe(LevelProbe probe, void* user)
    {
        level_probe_ = probe;
        level_user_  = user;
    }

    void beginGeneration(uint32_t generation)
    {
        generation_ = generation;
        pending_    = EventNone;
        dispatched_ = false;
    }

    void invalidateGeneration()
    {
        ++generation_;
        pending_    = EventNone;
        dispatched_ = false;
    }

    bool notify(uint16_t events, uint32_t generation)
    {
        if (generation != generation_) {
            return false;
        }
        pending_ |= events;
        return true;
    }

    bool dispatch(uint32_t generation, size_t rx_available, size_t tx_available)
    {
        if (generation != generation_ || callback_ == nullptr || dispatched_) {
            return false;
        }
        pending_ |= probeLevels();
        const uint16_t observed = pending_;
        if (observed == EventNone) {
            return false;
        }

        in_callback_ = true;
        callback_(callback_user_, {observed, rx_available, tx_available, generation_});
        in_callback_ = false;
        dispatched_  = true;
        return true;
    }

    bool acknowledge(uint32_t generation, RacePoint race_point = RacePoint::None, RaceInjection injection = nullptr,
                     void* injection_user = nullptr)
    {
        if (generation != generation_) {
            return false;
        }
        if (race_point == RacePoint::BeforePendingClear && injection != nullptr) {
            injection(injection_user);
        }
        pending_    = EventNone;
        dispatched_ = false;
        if (race_point == RacePoint::AfterPendingClear && injection != nullptr) {
            injection(injection_user);
        }
        // Rechecking the level after clear recovers a coalesced same-bit update that raced before clear.
        pending_ |= probeLevels();
        return true;
    }

    bool inCallback() const
    {
        return in_callback_;
    }

    uint16_t pending() const
    {
        return pending_;
    }

private:
    uint16_t probeLevels() const
    {
        return level_probe_ == nullptr ? EventNone : level_probe_(level_user_);
    }

    SlaveEventCallback callback_ = nullptr;
    void* callback_user_         = nullptr;
    LevelProbe level_probe_      = nullptr;
    void* level_user_            = nullptr;
    uint32_t generation_         = 0;
    uint16_t pending_            = EventNone;
    bool in_callback_            = false;
    bool dispatched_             = false;
};

class FakeBus {
public:
    explicit FakeBus(BusKind kind) : kind_{kind}
    {
    }

    result_t<void> lock()
    {
        if (locked_) {
            return m5::stl::make_unexpected(error_t::BUSY);
        }
        locked_ = true;
        trace_.push(TraceEvent::Lock);
        return {};
    }

    result_t<void> unlock()
    {
        if (!locked_) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        active_    = false;
        accepting_ = false;
        locked_    = false;
        trace_.push(TraceEvent::Unlock);
        return {};
    }

    result_t<void> beginOperation(TentativeOperationContext& context)
    {
        trace_.push(TraceEvent::BeginOperation);
        if (!locked_ || broken_) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        if (fail_begin_) {
            // A begin hook owns rollback of any partial backend setup it performed.
            trace_.push(TraceEvent::RollbackBegin);
            return m5::stl::make_unexpected(error_t::IO_ERROR);
        }
        active_    = true;
        accepting_ = context.mode == static_cast<uint8_t>(OperationMode::Slave);
        return {};
    }

    result_t<void> endOperation(TentativeOperationContext&, uint32_t, FrameQueue& rx)
    {
        trace_.push(TraceEvent::EndOperation);
        trace_.push(TraceEvent::StopAccepting);
        accepting_ = false;
        if (!frame_in_progress_) {
            active_ = false;
            return {};
        }
        trace_.push(TraceEvent::GracefulWait);
        if (frame_completes_gracefully_) {
            auto result        = publishPartialFrame(rx, FrameBegin | FrameEnd);
            active_            = false;
            frame_in_progress_ = false;
            if (!result.has_value()) {
                return m5::stl::make_unexpected(result.error());
            }
            return {};
        }

        trace_.push(TraceEvent::AbortFrame);
        auto recorded      = publishPartialFrame(rx, FrameBegin | FrameEnd | Aborted | Truncated);
        frame_in_progress_ = false;
        if (!recorded.has_value()) {
            active_ = false;
            return m5::stl::make_unexpected(recorded.error());
        }
        if (!fail_abort_) {
            active_ = false;
            return m5::stl::make_unexpected(error_t::TIMEOUT_ERROR);
        }

        trace_.push(TraceEvent::ResetPeripheral);
        active_ = false;
        if (fail_reset_) {
            broken_ = true;
            return m5::stl::make_unexpected(error_t::IO_ERROR);
        }
        return m5::stl::make_unexpected(error_t::TIMEOUT_ERROR);
    }

    result_t<size_t> masterTransfer(ConstDataSpan tx, DataSpan rx)
    {
        if (!active_ || tx.data == nullptr || tx.size == 0 || (rx.data == nullptr && rx.size != 0)) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        trace_.push(TraceEvent::Body);
        for (size_t i = 0; i < rx.size; ++i) {
            rx.data[i] = static_cast<uint8_t>(0x80u + tx.data[0] + i);
        }
        ++transfer_count_;
        return rx.size;
    }

    result_t<size_t> receiveFrame(FrameQueue& rx, ConstDataSpan payload, FrameMetadata metadata)
    {
        if (!accepting_) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        return rx.ingestFrame(payload, metadata);
    }

    result_t<void> startPartialFrame(ConstDataSpan payload, bool completes_gracefully)
    {
        if (!accepting_ || payload.data == nullptr || payload.size > partial_payload_.size()) {
            return m5::stl::make_unexpected(error_t::INVALID_ARGUMENT);
        }
        std::memcpy(partial_payload_.data(), payload.data, payload.size);
        partial_size_               = payload.size;
        frame_in_progress_          = true;
        frame_completes_gracefully_ = completes_gracefully;
        return {};
    }

    void markActive()
    {
        trace_.push(TraceEvent::Active);
    }

    bool locked() const
    {
        return locked_;
    }

    bool broken() const
    {
        return broken_;
    }

    bool accepting() const
    {
        return accepting_;
    }

    size_t transferCount() const
    {
        return transfer_count_;
    }

    BusKind kind() const
    {
        return kind_;
    }

    TraceLog& trace()
    {
        return trace_;
    }

    void setFailBegin(bool value)
    {
        fail_begin_ = value;
    }

    void setAbortFailure(bool abort_failure, bool reset_failure)
    {
        fail_abort_ = abort_failure;
        fail_reset_ = reset_failure;
    }

private:
    result_t<size_t> publishPartialFrame(FrameQueue& rx, uint16_t flags)
    {
        FrameMetadata metadata{};
        metadata.protocol      = kind_ == BusKind::I2c ? Protocol::I2c : Protocol::Spi;
        metadata.flags         = flags;
        metadata.segment_count = 1;
        metadata.segments[0]   = {0, static_cast<uint16_t>(partial_size_), SegmentDirection::Write,
                                kind_ == BusKind::Spi ? SegmentFlags::ChipSelect : SegmentFlags::None};
        return rx.ingestFrame({partial_payload_.data(), partial_size_}, metadata);
    }

    BusKind kind_;
    TraceLog trace_{};
    std::array<uint8_t, 32> partial_payload_{};
    size_t partial_size_             = 0;
    size_t transfer_count_           = 0;
    bool locked_                     = false;
    bool active_                     = false;
    bool accepting_                  = false;
    bool broken_                     = false;
    bool fail_begin_                 = false;
    bool frame_in_progress_          = false;
    bool frame_completes_gracefully_ = false;
    bool fail_abort_                 = false;
    bool fail_reset_                 = false;
};

class FakeAccessor {
public:
    FakeAccessor(FakeBus& bus, FrameQueue& tx, FrameQueue& rx, OperationMode mode)
        : bus_{bus}, tx_{tx}, rx_{rx}, mode_{mode}
    {
        notifier_.setLevelProbe(&FakeAccessor::probeLevels, this);
    }

    result_t<void> beginAccess(uint32_t timeout_ms)
    {
        if (notifier_.inCallback() || active_) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        auto locked = bus_.lock();
        if (!locked.has_value()) {
            return locked;
        }
        context_.deadline   = timeout_ms;
        context_.generation = ++generation_;
        context_.mode       = static_cast<uint8_t>(mode_);
        auto begun          = bus_.beginOperation(context_);
        if (!begun.has_value()) {
            auto unlocked = bus_.unlock();
            if (!unlocked.has_value()) {
                return unlocked;
            }
            return begun;
        }
        active_ = true;
        bus_.markActive();
        notifier_.beginGeneration(generation_);
        return {};
    }

    result_t<void> endAccess(uint32_t timeout_ms)
    {
        if (notifier_.inCallback() || !active_) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        auto ended = bus_.endOperation(context_, timeout_ms, rx_);
        active_    = false;
        notifier_.invalidateGeneration();
        auto unlocked = bus_.unlock();
        if (!ended.has_value()) {
            return ended;
        }
        return unlocked;
    }

    result_t<size_t> transfer(ConstDataSpan tx, DataSpan rx, uint32_t timeout_ms)
    {
        if (notifier_.inCallback()) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        const bool borrowed = active_;
        if (!borrowed) {
            auto begun = beginAccess(timeout_ms);
            if (!begun.has_value()) {
                return m5::stl::make_unexpected(begun.error());
            }
        }
        auto body = bus_.masterTransfer(tx, rx);
        if (!borrowed) {
            auto ended = endAccess(timeout_ms);
            if (!body.has_value()) {
                return body;
            }
            if (!ended.has_value()) {
                return m5::stl::make_unexpected(ended.error());
            }
        }
        return body;
    }

    result_t<size_t> enqueueTx(ConstDataSpan data)
    {
        if (notifier_.inCallback()) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        return tx_.write(data);
    }

    result_t<size_t> readRx(DataSpan data)
    {
        if (notifier_.inCallback()) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        return rx_.read(data);
    }

    result_t<void> setEventCallback(SlaveEventCallback callback, void* user)
    {
        if (active_) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        return notifier_.setCallback(callback, user);
    }

    result_t<size_t> backendReceive(ConstDataSpan payload, FrameMetadata metadata = {})
    {
        auto received = bus_.receiveFrame(rx_, payload, metadata);
        if (received.has_value()) {
            notifier_.notify(EventRxAvailable | EventFrameCompleted, generation_);
        }
        return received;
    }

    result_t<void> backendStartPartial(ConstDataSpan payload, bool completes_gracefully)
    {
        return bus_.startPartialFrame(payload, completes_gracefully);
    }

    bool dispatchEvents()
    {
        return notifier_.dispatch(generation_, rx_.readable(), tx_.writable());
    }

    bool acknowledgeEvents(EventNotifier::RacePoint race_point = EventNotifier::RacePoint::None,
                           RaceInjection injection = nullptr, void* injection_user = nullptr)
    {
        return notifier_.acknowledge(generation_, race_point, injection, injection_user);
    }

    bool dispatchEventsForGeneration(uint32_t generation)
    {
        return notifier_.dispatch(generation, rx_.readable(), tx_.writable());
    }

    uint16_t pendingEvents() const
    {
        return notifier_.pending();
    }

    result_t<void> popRxFrame()
    {
        if (notifier_.inCallback()) {
            return m5::stl::make_unexpected(error_t::INVALID_STATE);
        }
        return rx_.popFrame();
    }

    bool active() const
    {
        return active_;
    }

    uint32_t generation() const
    {
        return generation_;
    }

private:
    static uint16_t probeLevels(void* user)
    {
        auto& self      = *static_cast<FakeAccessor*>(user);
        uint16_t levels = EventNone;
        if (self.rx_.readable() != 0 || self.rx_.frameCount() != 0) {
            levels |= EventRxAvailable | EventFrameCompleted;
        }
        return levels;
    }

    FakeBus& bus_;
    FrameQueue& tx_;
    FrameQueue& rx_;
    TentativeOperationContext context_{};
    EventNotifier notifier_{};
    OperationMode mode_;
    uint32_t generation_ = 0;
    bool active_         = false;
};

class DispatchApi {
public:
    virtual ~DispatchApi()                                                                    = default;
    virtual result_t<size_t> write(ConstDataSpan src)                                         = 0;
    virtual result_t<size_t> read(DataSpan dst)                                               = 0;
    virtual size_t readable() const                                                           = 0;
    virtual size_t writable() const                                                           = 0;
    virtual result_t<FrameReservation> reserveFrame(size_t size)                              = 0;
    virtual result_t<void> commitFrame(FrameReservation& reservation, FrameMetadata metadata) = 0;
    virtual result_t<FrameView> peekFrame() const                                             = 0;
    virtual result_t<void> popFrame()                                                         = 0;
};

class DispatchEndpoint final : public DispatchApi {
public:
    DispatchEndpoint(uint8_t* bytes, size_t byte_capacity, FrameDescriptor* descriptors, size_t descriptor_capacity)
        : queue_{bytes, byte_capacity, descriptors, descriptor_capacity}
    {
    }

    result_t<size_t> write(ConstDataSpan src) override
    {
        return queue_.write(src);
    }
    result_t<size_t> read(DataSpan dst) override
    {
        return queue_.read(dst);
    }
    size_t readable() const override
    {
        return queue_.readable();
    }
    size_t writable() const override
    {
        return queue_.writable();
    }
    result_t<FrameReservation> reserveFrame(size_t size) override
    {
        return queue_.reserveFrame(size);
    }
    result_t<void> commitFrame(FrameReservation& reservation, FrameMetadata metadata) override
    {
        return queue_.commitFrame(reservation, metadata);
    }
    result_t<FrameView> peekFrame() const override
    {
        return queue_.peekFrame();
    }
    result_t<void> popFrame() override
    {
        return queue_.popFrame();
    }

private:
    FrameQueue queue_;
};

template <class Endpoint>
class StaticDispatchView {
public:
    explicit StaticDispatchView(Endpoint& endpoint) : endpoint_{endpoint}
    {
    }

    result_t<size_t> write(ConstDataSpan src)
    {
        return endpoint_.write(src);
    }
    result_t<size_t> read(DataSpan dst)
    {
        return endpoint_.read(dst);
    }
    size_t readable() const
    {
        return endpoint_.readable();
    }
    size_t writable() const
    {
        return endpoint_.writable();
    }
    result_t<FrameReservation> reserveFrame(size_t size)
    {
        return endpoint_.reserveFrame(size);
    }
    result_t<void> commitFrame(FrameReservation& reservation, FrameMetadata metadata)
    {
        return endpoint_.commitFrame(reservation, metadata);
    }
    result_t<FrameView> peekFrame() const
    {
        return endpoint_.peekFrame();
    }
    result_t<void> popFrame()
    {
        return endpoint_.popFrame();
    }

private:
    Endpoint& endpoint_;
};

template <class Api>
uint64_t dispatchWorkload(Api& api, size_t iterations)
{
    const uint8_t input[] = {1, 3, 5, 7};
    uint8_t output[sizeof(input)]{};
    uint64_t checksum = 0;
    FrameMetadata metadata{};
    for (size_t i = 0; i < iterations; ++i) {
        auto written = api.write({input, sizeof(input)});
        if (!written.has_value()) {
            return checksum ^ static_cast<uint8_t>(written.error());
        }
        checksum += api.readable() + api.writable();
        auto read = api.read({output, sizeof(output)});
        if (!read.has_value()) {
            return checksum ^ static_cast<uint8_t>(read.error());
        }
        auto reservation = api.reserveFrame(sizeof(input));
        if (!reservation.has_value()) {
            return checksum ^ static_cast<uint8_t>(reservation.error());
        }
        std::memcpy(reservation.value().first.data, input, reservation.value().first.size);
        std::memcpy(reservation.value().second.data, input + reservation.value().first.size,
                    reservation.value().second.size);
        auto committed = api.commitFrame(reservation.value(), metadata);
        if (!committed.has_value()) {
            return checksum ^ static_cast<uint8_t>(committed.error());
        }
        auto frame = api.peekFrame();
        if (!frame.has_value()) {
            return checksum ^ static_cast<uint8_t>(frame.error());
        }
        checksum += output[i % sizeof(output)] + frame.value().first.size + frame.value().second.size;
        auto popped = api.popFrame();
        if (!popped.has_value()) {
            return checksum ^ static_cast<uint8_t>(popped.error());
        }
    }
    return checksum;
}

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
uint64_t
virtualDispatchWorkload(DispatchApi& api, size_t iterations)
{
    return dispatchWorkload(api, iterations);
}

template <class Callable>
std::pair<int64_t, uint64_t> bestOf(Callable&& callable, size_t trials)
{
    int64_t best      = std::numeric_limits<int64_t>::max();
    uint64_t checksum = 0;
    for (size_t trial = 0; trial < trials; ++trial) {
        const auto start = std::chrono::steady_clock::now();
        checksum ^= callable();
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
        best = std::min(best, elapsed);
    }
    return {best, checksum};
}

FrameMetadata i2cRegisterReadMetadata()
{
    FrameMetadata metadata{};
    metadata.frame_id      = 17;
    metadata.protocol      = Protocol::I2c;
    metadata.segment_count = 2;
    metadata.segments[0]   = {0, 1, SegmentDirection::Write, SegmentFlags::None};
    metadata.segments[1]   = {1, 2, SegmentDirection::Read, SegmentFlags::RepeatedStart};
    return metadata;
}

FrameMetadata spiDuplexMetadata()
{
    FrameMetadata metadata{};
    metadata.frame_id      = 18;
    metadata.protocol      = Protocol::Spi;
    metadata.segment_count = 1;
    metadata.segments[0]   = {0, 4, SegmentDirection::Duplex, SegmentFlags::ChipSelect};
    return metadata;
}

struct CallbackObservation {
    FakeAccessor* accessor = nullptr;
    size_t calls           = 0;
    uint16_t events        = EventNone;
    uint32_t generation    = 0;
    bool attempt_reentry   = false;
    error_t reentry_error  = error_t::OK;
};

void observeSlaveEvent(void* user, const SlaveEventInfo& event)
{
    auto& observation = *static_cast<CallbackObservation*>(user);
    ++observation.calls;
    observation.events     = event.events;
    observation.generation = event.generation;
    if (observation.attempt_reentry) {
        const uint8_t byte        = 0xEE;
        auto result               = observation.accessor->enqueueTx({&byte, 1});
        observation.reentry_error = result.has_value() ? error_t::OK : result.error();
    }
}

struct RaceFrame {
    FakeAccessor* accessor = nullptr;
    uint8_t payload        = 0;
};

void injectRaceFrame(void* user)
{
    auto& race  = *static_cast<RaceFrame*>(user);
    auto result = race.accessor->backendReceive({&race.payload, 1});
    EXPECT_TRUE(result.has_value()) << "err=" << m5::hal::v2::error::toString(result.error());
}

TEST(SlaveQueuePrototype, LifecycleHookOrderNonNestingAndBeginFailureCleanup)
{
    uint8_t tx_bytes[8]{};
    uint8_t rx_bytes[8]{};
    FrameDescriptor tx_descriptors[2]{};
    FrameDescriptor rx_descriptors[2]{};
    FrameQueue tx{tx_bytes, sizeof(tx_bytes), tx_descriptors, std::size(tx_descriptors)};
    FrameQueue rx{rx_bytes, sizeof(rx_bytes), rx_descriptors, std::size(rx_descriptors)};
    FakeBus bus{BusKind::I2c};
    FakeAccessor accessor{bus, tx, rx, OperationMode::Master};

    ASSERT_RESULT_OK(accessor.beginAccess(5));
    ASSERT_EQ(bus.trace().size(), 3u);
    EXPECT_EQ(bus.trace()[0], TraceEvent::Lock);
    EXPECT_EQ(bus.trace()[1], TraceEvent::BeginOperation);
    EXPECT_EQ(bus.trace()[2], TraceEvent::Active);
    auto nested = accessor.beginAccess(5);
    ASSERT_FALSE(nested.has_value());
    EXPECT_EQ(nested.error(), error_t::INVALID_STATE) << "err=" << m5::hal::v2::error::toString(nested.error());
    ASSERT_RESULT_OK(accessor.endAccess(5));
    ASSERT_EQ(bus.trace().size(), 6u);
    EXPECT_EQ(bus.trace()[3], TraceEvent::EndOperation);
    EXPECT_EQ(bus.trace()[4], TraceEvent::StopAccepting);
    EXPECT_EQ(bus.trace()[5], TraceEvent::Unlock);
    auto inactive_end = accessor.endAccess(5);
    ASSERT_FALSE(inactive_end.has_value());
    EXPECT_EQ(inactive_end.error(), error_t::INVALID_STATE)
        << "err=" << m5::hal::v2::error::toString(inactive_end.error());

    uint8_t failed_tx_bytes[4]{};
    uint8_t failed_rx_bytes[4]{};
    FrameDescriptor failed_tx_descriptors[1]{};
    FrameDescriptor failed_rx_descriptors[1]{};
    FrameQueue failed_tx{failed_tx_bytes, sizeof(failed_tx_bytes), failed_tx_descriptors,
                         std::size(failed_tx_descriptors)};
    FrameQueue failed_rx{failed_rx_bytes, sizeof(failed_rx_bytes), failed_rx_descriptors,
                         std::size(failed_rx_descriptors)};
    FakeBus failed_bus{BusKind::I2c};
    failed_bus.setFailBegin(true);
    FakeAccessor failed_accessor{failed_bus, failed_tx, failed_rx, OperationMode::Master};
    auto failed_begin = failed_accessor.beginAccess(5);
    ASSERT_FALSE(failed_begin.has_value());
    EXPECT_EQ(failed_begin.error(), error_t::IO_ERROR) << "err=" << m5::hal::v2::error::toString(failed_begin.error());
    EXPECT_FALSE(failed_bus.locked());
    ASSERT_EQ(failed_bus.trace().size(), 4u);
    EXPECT_EQ(failed_bus.trace()[0], TraceEvent::Lock);
    EXPECT_EQ(failed_bus.trace()[1], TraceEvent::BeginOperation);
    EXPECT_EQ(failed_bus.trace()[2], TraceEvent::RollbackBegin);
    EXPECT_EQ(failed_bus.trace()[3], TraceEvent::Unlock);
    EXPECT_EQ(failed_bus.trace().count(TraceEvent::EndOperation), 0u);
}

TEST(SlaveQueuePrototype, Consumer1AtomicI2cRegisterReadSugarBorrowsOrOpensAccess)
{
    uint8_t tx_bytes[8]{};
    uint8_t rx_bytes[8]{};
    FrameDescriptor tx_descriptors[1]{};
    FrameDescriptor rx_descriptors[1]{};
    FrameQueue tx{tx_bytes, sizeof(tx_bytes), tx_descriptors, std::size(tx_descriptors)};
    FrameQueue rx{rx_bytes, sizeof(rx_bytes), rx_descriptors, std::size(rx_descriptors)};
    FakeBus bus{BusKind::I2c};
    FakeAccessor accessor{bus, tx, rx, OperationMode::Master};
    const uint8_t register_address = 0x12;
    uint8_t register_value[2]{};

    auto result = accessor.transfer({&register_address, 1}, {register_value, sizeof(register_value)}, 5);
    ASSERT_TRUE(result.has_value()) << "err=" << m5::hal::v2::error::toString(result.error());
    EXPECT_EQ(result.value(), sizeof(register_value));
    EXPECT_EQ(register_value[0], 0x92);
    EXPECT_EQ(register_value[1], 0x93);
    EXPECT_EQ(bus.transferCount(), 1u);
    EXPECT_EQ(bus.trace().count(TraceEvent::Lock), 1u);
    EXPECT_EQ(bus.trace().count(TraceEvent::BeginOperation), 1u);
    EXPECT_EQ(bus.trace().count(TraceEvent::Body), 1u);
    EXPECT_EQ(bus.trace().count(TraceEvent::EndOperation), 1u);
    EXPECT_EQ(bus.trace().count(TraceEvent::Unlock), 1u);

    bus.trace().clear();
    ASSERT_RESULT_OK(accessor.beginAccess(5));
    ASSERT_RESULT_OK(accessor.transfer({&register_address, 1}, {register_value, sizeof(register_value)}, 5));
    EXPECT_TRUE(accessor.active());
    EXPECT_EQ(bus.trace().count(TraceEvent::BeginOperation), 1u);
    EXPECT_EQ(bus.trace().count(TraceEvent::EndOperation), 0u);
    ASSERT_RESULT_OK(accessor.endAccess(5));
    EXPECT_EQ(bus.trace().count(TraceEvent::EndOperation), 1u);
}

TEST(SlaveQueuePrototype, SlaveAccessStartsAfterPreloadedTxAndRetainsRxAfterStop)
{
    uint8_t tx_bytes[8]{};
    uint8_t rx_bytes[8]{};
    FrameDescriptor tx_descriptors[2]{};
    FrameDescriptor rx_descriptors[2]{};
    FrameQueue tx{tx_bytes, sizeof(tx_bytes), tx_descriptors, std::size(tx_descriptors)};
    FrameQueue rx{rx_bytes, sizeof(rx_bytes), rx_descriptors, std::size(rx_descriptors)};
    FakeBus bus{BusKind::I2c};
    FakeAccessor accessor{bus, tx, rx, OperationMode::Slave};
    const uint8_t response[] = {0xA5, 0x5A};
    const uint8_t request[]  = {0x10, 0x20, 0x30};

    auto queued = accessor.enqueueTx({response, sizeof(response)});
    ASSERT_TRUE(queued.has_value()) << "err=" << m5::hal::v2::error::toString(queued.error());
    EXPECT_EQ(tx.readable(), sizeof(response));
    ASSERT_RESULT_OK(accessor.beginAccess(5));
    EXPECT_TRUE(bus.accepting());
    ASSERT_RESULT_OK(accessor.backendReceive({request, sizeof(request)}, i2cRegisterReadMetadata()));
    ASSERT_RESULT_OK(accessor.endAccess(5));
    EXPECT_FALSE(bus.accepting());
    EXPECT_EQ(tx.readable(), sizeof(response));

    uint8_t observed[sizeof(request)]{};
    auto read = accessor.readRx({observed, sizeof(observed)});
    ASSERT_TRUE(read.has_value()) << "err=" << m5::hal::v2::error::toString(read.error());
    EXPECT_EQ(read.value(), sizeof(request));
    EXPECT_EQ(0, std::memcmp(observed, request, sizeof(request)));
}

TEST(SlaveQueuePrototype, Consumer5LevelEventsCoalesceRejectReentryAndAvoidLostWakeups)
{
    uint8_t tx_bytes[16]{};
    uint8_t rx_bytes[16]{};
    FrameDescriptor tx_descriptors[4]{};
    FrameDescriptor rx_descriptors[8]{};
    FrameQueue tx{tx_bytes, sizeof(tx_bytes), tx_descriptors, std::size(tx_descriptors)};
    FrameQueue rx{rx_bytes, sizeof(rx_bytes), rx_descriptors, std::size(rx_descriptors)};
    FakeBus bus{BusKind::I2c};
    FakeAccessor accessor{bus, tx, rx, OperationMode::Slave};
    CallbackObservation observation{&accessor};
    ASSERT_RESULT_OK(accessor.setEventCallback(&observeSlaveEvent, &observation));
    ASSERT_RESULT_OK(accessor.beginAccess(5));
    const uint32_t generation = accessor.generation();
    auto active_registration  = accessor.setEventCallback(&observeSlaveEvent, &observation);
    ASSERT_FALSE(active_registration.has_value());
    EXPECT_EQ(active_registration.error(), error_t::INVALID_STATE)
        << "err=" << m5::hal::v2::error::toString(active_registration.error());
    observation.attempt_reentry = true;
    const uint8_t first         = 1;
    const uint8_t second        = 2;
    ASSERT_RESULT_OK(accessor.backendReceive({&first, 1}));
    ASSERT_RESULT_OK(accessor.backendReceive({&second, 1}));
    EXPECT_TRUE(accessor.dispatchEvents());
    EXPECT_EQ(observation.calls, 1u);
    EXPECT_EQ(observation.generation, generation);
    EXPECT_NE(observation.events & EventRxAvailable, 0);
    EXPECT_EQ(observation.reentry_error, error_t::INVALID_STATE)
        << "err=" << m5::hal::v2::error::toString(observation.reentry_error);
    EXPECT_FALSE(accessor.dispatchEvents());  // one callback until the owner acknowledges the level
    EXPECT_EQ(observation.calls, 1u);
    ASSERT_RESULT_OK(accessor.popRxFrame());
    ASSERT_RESULT_OK(accessor.popRxFrame());
    EXPECT_TRUE(accessor.acknowledgeEvents());
    EXPECT_EQ(accessor.pendingEvents(), EventNone);

    observation.attempt_reentry = false;
    ASSERT_RESULT_OK(accessor.backendReceive({&first, 1}));
    EXPECT_TRUE(accessor.dispatchEvents());
    ASSERT_RESULT_OK(accessor.popRxFrame());
    RaceFrame before_clear{&accessor, 3};
    EXPECT_TRUE(
        accessor.acknowledgeEvents(EventNotifier::RacePoint::BeforePendingClear, &injectRaceFrame, &before_clear));
    EXPECT_NE(accessor.pendingEvents() & EventRxAvailable, 0);
    EXPECT_TRUE(accessor.dispatchEvents());
    ASSERT_RESULT_OK(accessor.popRxFrame());
    EXPECT_TRUE(accessor.acknowledgeEvents());

    ASSERT_RESULT_OK(accessor.backendReceive({&first, 1}));
    EXPECT_TRUE(accessor.dispatchEvents());
    ASSERT_RESULT_OK(accessor.popRxFrame());
    RaceFrame after_clear{&accessor, 4};
    EXPECT_TRUE(
        accessor.acknowledgeEvents(EventNotifier::RacePoint::AfterPendingClear, &injectRaceFrame, &after_clear));
    EXPECT_NE(accessor.pendingEvents() & EventRxAvailable, 0);
    EXPECT_TRUE(accessor.dispatchEvents());
    ASSERT_RESULT_OK(accessor.popRxFrame());
    EXPECT_TRUE(accessor.acknowledgeEvents());
    EXPECT_EQ(observation.calls, 5u);

    ASSERT_RESULT_OK(accessor.backendReceive({&first, 1}));
    ASSERT_RESULT_OK(accessor.endAccess(5));
    EXPECT_FALSE(accessor.dispatchEventsForGeneration(generation));
    EXPECT_EQ(observation.calls, 5u);
}

TEST(SlaveQueuePrototype, GracefulTimeoutAbortsFrameAndResetFailureBreaksButUnlocksBus)
{
    uint8_t tx_bytes[8]{};
    uint8_t rx_bytes[8]{};
    FrameDescriptor tx_descriptors[2]{};
    FrameDescriptor rx_descriptors[3]{};
    FrameQueue tx{tx_bytes, sizeof(tx_bytes), tx_descriptors, std::size(tx_descriptors)};
    FrameQueue rx{rx_bytes, sizeof(rx_bytes), rx_descriptors, std::size(rx_descriptors)};
    FakeBus bus{BusKind::I2c};
    FakeAccessor accessor{bus, tx, rx, OperationMode::Slave};
    const uint8_t partial[] = {0x11, 0x22, 0x33};

    ASSERT_RESULT_OK(accessor.beginAccess(5));
    ASSERT_RESULT_OK(accessor.backendStartPartial({partial, sizeof(partial)}, true));
    ASSERT_RESULT_OK(accessor.endAccess(5));
    auto graceful = rx.peekFrame();
    ASSERT_TRUE(graceful.has_value()) << "err=" << m5::hal::v2::error::toString(graceful.error());
    EXPECT_EQ(graceful.value().metadata.flags & Aborted, 0);
    ASSERT_RESULT_OK(rx.popFrame());
    bus.trace().clear();

    ASSERT_RESULT_OK(accessor.beginAccess(5));
    ASSERT_RESULT_OK(accessor.backendStartPartial({partial, sizeof(partial)}, false));
    auto timed_out = accessor.endAccess(0);
    ASSERT_FALSE(timed_out.has_value());
    EXPECT_EQ(timed_out.error(), error_t::TIMEOUT_ERROR) << "err=" << m5::hal::v2::error::toString(timed_out.error());
    EXPECT_FALSE(bus.locked());
    EXPECT_FALSE(accessor.active());
    EXPECT_EQ(bus.trace().count(TraceEvent::AbortFrame), 1u);
    auto aborted = rx.peekFrame();
    ASSERT_TRUE(aborted.has_value()) << "err=" << m5::hal::v2::error::toString(aborted.error());
    EXPECT_EQ(aborted.value().metadata.stored_bytes, sizeof(partial));
    EXPECT_NE(aborted.value().metadata.flags & Aborted, 0);
    EXPECT_NE(aborted.value().metadata.flags & Truncated, 0);

    uint8_t broken_tx_bytes[8]{};
    uint8_t broken_rx_bytes[8]{};
    FrameDescriptor broken_tx_descriptors[2]{};
    FrameDescriptor broken_rx_descriptors[2]{};
    FrameQueue broken_tx{broken_tx_bytes, sizeof(broken_tx_bytes), broken_tx_descriptors,
                         std::size(broken_tx_descriptors)};
    FrameQueue broken_rx{broken_rx_bytes, sizeof(broken_rx_bytes), broken_rx_descriptors,
                         std::size(broken_rx_descriptors)};
    FakeBus broken_bus{BusKind::Spi};
    broken_bus.setAbortFailure(true, true);
    FakeAccessor broken_accessor{broken_bus, broken_tx, broken_rx, OperationMode::Slave};
    ASSERT_RESULT_OK(broken_accessor.beginAccess(5));
    ASSERT_RESULT_OK(broken_accessor.backendStartPartial({partial, sizeof(partial)}, false));
    auto reset_failed = broken_accessor.endAccess(0);
    ASSERT_FALSE(reset_failed.has_value());
    EXPECT_EQ(reset_failed.error(), error_t::IO_ERROR) << "err=" << m5::hal::v2::error::toString(reset_failed.error());
    EXPECT_TRUE(broken_bus.broken());
    EXPECT_FALSE(broken_bus.locked());
    EXPECT_EQ(broken_bus.trace().count(TraceEvent::ResetPeripheral), 1u);
    auto rejected = broken_accessor.beginAccess(5);
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error(), error_t::INVALID_STATE) << "err=" << m5::hal::v2::error::toString(rejected.error());
    EXPECT_FALSE(broken_bus.locked());
}

TEST(SlaveQueuePrototype, Consumer6I2cAndSpiUseTheSameLifecycleShape)
{
    uint8_t i2c_tx_bytes[4]{};
    uint8_t i2c_rx_bytes[4]{};
    FrameDescriptor i2c_tx_descriptors[1]{};
    FrameDescriptor i2c_rx_descriptors[1]{};
    FrameQueue i2c_tx{i2c_tx_bytes, sizeof(i2c_tx_bytes), i2c_tx_descriptors, std::size(i2c_tx_descriptors)};
    FrameQueue i2c_rx{i2c_rx_bytes, sizeof(i2c_rx_bytes), i2c_rx_descriptors, std::size(i2c_rx_descriptors)};
    FakeBus i2c_bus{BusKind::I2c};
    FakeAccessor i2c{i2c_bus, i2c_tx, i2c_rx, OperationMode::Slave};

    uint8_t spi_tx_bytes[4]{};
    uint8_t spi_rx_bytes[4]{};
    FrameDescriptor spi_tx_descriptors[1]{};
    FrameDescriptor spi_rx_descriptors[1]{};
    FrameQueue spi_tx{spi_tx_bytes, sizeof(spi_tx_bytes), spi_tx_descriptors, std::size(spi_tx_descriptors)};
    FrameQueue spi_rx{spi_rx_bytes, sizeof(spi_rx_bytes), spi_rx_descriptors, std::size(spi_rx_descriptors)};
    FakeBus spi_bus{BusKind::Spi};
    FakeAccessor spi{spi_bus, spi_tx, spi_rx, OperationMode::Slave};

    ASSERT_RESULT_OK(i2c.beginAccess(5));
    ASSERT_RESULT_OK(i2c.endAccess(5));
    ASSERT_RESULT_OK(spi.beginAccess(5));
    ASSERT_RESULT_OK(spi.endAccess(5));
    ASSERT_EQ(i2c_bus.trace().size(), spi_bus.trace().size());
    for (size_t i = 0; i < i2c_bus.trace().size(); ++i) {
        EXPECT_EQ(i2c_bus.trace()[i], spi_bus.trace()[i]);
    }
    EXPECT_EQ(i2c_bus.kind(), BusKind::I2c);
    EXPECT_EQ(spi_bus.kind(), BusKind::Spi);
}

TEST(SlaveQueuePrototype, ByteReadWriteAreNonBlockingAndNeverOverwrite)
{
    uint8_t bytes[4]{};
    FrameDescriptor descriptors[2]{};
    FrameQueue queue{bytes, sizeof(bytes), descriptors, std::size(descriptors)};
    const uint8_t input[] = {1, 2, 3, 4, 5};

    auto written = queue.write({input, sizeof(input)});
    ASSERT_TRUE(written.has_value()) << "err=" << m5::hal::v2::error::toString(written.error());
    EXPECT_EQ(written.value(), 4u);
    EXPECT_EQ(queue.readable(), 4u);
    EXPECT_EQ(queue.writable(), 0u);

    auto full = queue.write({input + 4, 1});
    ASSERT_FALSE(full.has_value());
    EXPECT_EQ(full.error(), error_t::BUSY) << "err=" << m5::hal::v2::error::toString(full.error());

    uint8_t output[4]{};
    auto read = queue.read({output, sizeof(output)});
    ASSERT_TRUE(read.has_value()) << "err=" << m5::hal::v2::error::toString(read.error());
    EXPECT_EQ(read.value(), 4u);
    EXPECT_EQ(0, std::memcmp(input, output, sizeof(output)));
}

TEST(SlaveQueuePrototype, Consumer2I2cRepeatedStartDirectionSegmentsRemainInOneFrame)
{
    uint8_t bytes[8]{};
    FrameDescriptor descriptors[2]{};
    FrameQueue queue{bytes, sizeof(bytes), descriptors, std::size(descriptors)};
    const uint8_t payload[] = {0x42, 0xA5, 0x5A};

    auto accepted = queue.ingestFrame({payload, sizeof(payload)}, i2cRegisterReadMetadata());
    ASSERT_TRUE(accepted.has_value()) << "err=" << m5::hal::v2::error::toString(accepted.error());
    auto view = queue.peekFrame();
    ASSERT_TRUE(view.has_value()) << "err=" << m5::hal::v2::error::toString(view.error());
    EXPECT_EQ(view.value().metadata.protocol, Protocol::I2c);
    ASSERT_EQ(view.value().metadata.segment_count, 2u);
    EXPECT_EQ(view.value().metadata.segments[0].direction, SegmentDirection::Write);
    EXPECT_EQ(view.value().metadata.segments[1].direction, SegmentDirection::Read);
    EXPECT_EQ(view.value().metadata.segments[1].flags, SegmentFlags::RepeatedStart);
    EXPECT_EQ(view.value().metadata.wire_bytes, sizeof(payload));
    EXPECT_EQ(view.value().metadata.stored_bytes, sizeof(payload));
}

TEST(SlaveQueuePrototype, Consumer3SpiDuplexChipSelectFrameCarriesStatusAndAmounts)
{
    uint8_t bytes[8]{};
    FrameDescriptor descriptors[2]{};
    FrameQueue queue{bytes, sizeof(bytes), descriptors, std::size(descriptors)};
    const uint8_t payload[] = {1, 2, 3, 4};
    auto metadata           = spiDuplexMetadata();
    metadata.flags |= Underrun;

    auto accepted = queue.ingestFrame({payload, sizeof(payload)}, metadata);
    ASSERT_TRUE(accepted.has_value()) << "err=" << m5::hal::v2::error::toString(accepted.error());
    auto view = queue.peekFrame();
    ASSERT_TRUE(view.has_value()) << "err=" << m5::hal::v2::error::toString(view.error());
    EXPECT_EQ(view.value().metadata.protocol, Protocol::Spi);
    EXPECT_EQ(view.value().metadata.segments[0].direction, SegmentDirection::Duplex);
    EXPECT_EQ(view.value().metadata.segments[0].flags, SegmentFlags::ChipSelect);
    EXPECT_NE(view.value().metadata.flags & Underrun, 0);
}

TEST(SlaveQueuePrototype, Consumer4WrapZeroLengthReservationMisuseAndOverflowAreObservable)
{
    uint8_t bytes[8]{};
    FrameDescriptor descriptors[2]{};
    FrameQueue queue{bytes, sizeof(bytes), descriptors, std::size(descriptors)};
    const uint8_t prefix[] = {1, 2, 3, 4, 5, 6};
    ASSERT_RESULT_OK(queue.write({prefix, sizeof(prefix)}));
    uint8_t discarded[6]{};
    ASSERT_RESULT_OK(queue.read({discarded, sizeof(discarded)}));

    auto reservation = queue.reserveFrame(5);
    ASSERT_TRUE(reservation.has_value()) << "err=" << m5::hal::v2::error::toString(reservation.error());
    EXPECT_EQ(reservation.value().first.size, 2u);
    EXPECT_EQ(reservation.value().second.size, 3u);
    EXPECT_EQ(queue.readable(), 0u);
    const uint8_t payload[] = {9, 8, 7, 6, 5};
    std::memcpy(reservation.value().first.data, payload, reservation.value().first.size);
    std::memcpy(reservation.value().second.data, payload + reservation.value().first.size,
                reservation.value().second.size);
    auto second_reservation = queue.reserveFrame(1);
    ASSERT_FALSE(second_reservation.has_value());
    EXPECT_EQ(second_reservation.error(), error_t::INVALID_STATE)
        << "err=" << m5::hal::v2::error::toString(second_reservation.error());
    auto unpublished = queue.peekFrame();
    ASSERT_FALSE(unpublished.has_value());
    EXPECT_EQ(unpublished.error(), error_t::BUFFER_UNDERFLOW)
        << "err=" << m5::hal::v2::error::toString(unpublished.error());
    ASSERT_RESULT_OK(queue.commitFrame(reservation.value(), {}));
    EXPECT_EQ(queue.readable(), sizeof(payload));
    auto frame = queue.peekFrame();
    ASSERT_TRUE(frame.has_value()) << "err=" << m5::hal::v2::error::toString(frame.error());
    EXPECT_EQ(frame.value().first.size, 2u);
    EXPECT_EQ(frame.value().second.size, 3u);

    auto double_commit = queue.commitFrame(reservation.value(), {});
    ASSERT_FALSE(double_commit.has_value());
    EXPECT_EQ(double_commit.error(), error_t::INVALID_STATE)
        << "err=" << m5::hal::v2::error::toString(double_commit.error());
    ASSERT_RESULT_OK(queue.popFrame());

    auto zero = queue.reserveFrame(0);
    ASSERT_TRUE(zero.has_value()) << "err=" << m5::hal::v2::error::toString(zero.error());
    EXPECT_TRUE(zero.value().first.empty());
    EXPECT_TRUE(zero.value().second.empty());
    ASSERT_RESULT_OK(queue.commitFrame(zero.value(), {}));
    EXPECT_EQ(queue.frameCount(), 1u);
    ASSERT_RESULT_OK(queue.popFrame());

    auto cancelled = queue.reserveFrame(2);
    ASSERT_TRUE(cancelled.has_value()) << "err=" << m5::hal::v2::error::toString(cancelled.error());
    ASSERT_RESULT_OK(queue.cancelFrame(cancelled.value()));
    auto double_cancel = queue.cancelFrame(cancelled.value());
    ASSERT_FALSE(double_cancel.has_value());
    EXPECT_EQ(double_cancel.error(), error_t::INVALID_STATE)
        << "err=" << m5::hal::v2::error::toString(double_cancel.error());

    auto stale = queue.reserveFrame(1);
    ASSERT_TRUE(stale.has_value()) << "err=" << m5::hal::v2::error::toString(stale.error());
    queue.invalidateGenerationForTest();
    auto stale_commit = queue.commitFrame(stale.value(), {});
    ASSERT_FALSE(stale_commit.has_value());
    EXPECT_EQ(stale_commit.error(), error_t::INVALID_STATE)
        << "err=" << m5::hal::v2::error::toString(stale_commit.error());

    uint8_t small_bytes[3]{};
    FrameDescriptor one_descriptor[1]{};
    FrameQueue overflow{small_bytes, sizeof(small_bytes), one_descriptor, std::size(one_descriptor)};
    const uint8_t too_large[] = {1, 2, 3, 4, 5};
    auto partial              = overflow.ingestFrame({too_large, sizeof(too_large)}, {});
    ASSERT_TRUE(partial.has_value()) << "err=" << m5::hal::v2::error::toString(partial.error());
    EXPECT_EQ(partial.value(), 3u);
    auto partial_view = overflow.peekFrame();
    ASSERT_TRUE(partial_view.has_value()) << "err=" << m5::hal::v2::error::toString(partial_view.error());
    EXPECT_EQ(partial_view.value().metadata.wire_bytes, 5u);
    EXPECT_EQ(partial_view.value().metadata.stored_bytes, 3u);
    EXPECT_EQ(partial_view.value().metadata.dropped_bytes, 2u);
    EXPECT_NE(static_cast<uint32_t>(partial_view.value().metadata.flags & (Overflow | Truncated)), 0u);

    const uint8_t another[] = {6, 7};
    auto descriptor_full    = overflow.ingestFrame({another, sizeof(another)}, {});
    ASSERT_FALSE(descriptor_full.has_value());
    EXPECT_EQ(descriptor_full.error(), error_t::BUFFER_OVERFLOW)
        << "err=" << m5::hal::v2::error::toString(descriptor_full.error());
    EXPECT_EQ(overflow.status().dropped_frames, 1u);
    EXPECT_EQ(overflow.status().dropped_bytes, 4u);
}

TEST(SlaveQueuePrototype, Consumer7FixedModeAndSharedCursorRunEquivalentConsumerIntent)
{
    const uint8_t first[]  = {1, 2, 3, 4};
    const uint8_t second[] = {5, 6, 7, 8};
    uint8_t fixed_bytes[8]{};
    FrameDescriptor fixed_descriptors[2]{};
    FrameQueue fixed_queue{fixed_bytes, sizeof(fixed_bytes), fixed_descriptors, std::size(fixed_descriptors)};
    ASSERT_RESULT_OK(fixed_queue.ingestFrame({first, sizeof(first)}, {}));
    ASSERT_RESULT_OK(fixed_queue.ingestFrame({second, sizeof(second)}, {}));
    FixedModeConsumer fixed{fixed_queue};
    WorkMetrics fixed_metrics{};
    uint8_t prefix[3]{};
    ASSERT_RESULT_OK(fixed.read({prefix, sizeof(prefix)}, fixed_metrics));
    auto rejected_switch = fixed.peekFrame(fixed_metrics);
    ASSERT_FALSE(rejected_switch.has_value());
    EXPECT_EQ(rejected_switch.error(), error_t::INVALID_STATE)
        << "err=" << m5::hal::v2::error::toString(rejected_switch.error());
    uint8_t remainder[5]{};
    ASSERT_RESULT_OK(fixed.read({remainder, sizeof(remainder)}, fixed_metrics));

    uint8_t shared_bytes[8]{};
    FrameDescriptor shared_descriptors[2]{};
    FrameQueue shared_queue{shared_bytes, sizeof(shared_bytes), shared_descriptors, std::size(shared_descriptors)};
    ASSERT_RESULT_OK(shared_queue.ingestFrame({first, sizeof(first)}, {}));
    ASSERT_RESULT_OK(shared_queue.ingestFrame({second, sizeof(second)}, {}));
    SharedCursorConsumer shared{shared_queue};
    WorkMetrics shared_metrics{};
    ASSERT_RESULT_OK(shared.read({prefix, sizeof(prefix)}, shared_metrics));
    auto remaining_first = shared.peekFrame(shared_metrics);
    ASSERT_TRUE(remaining_first.has_value()) << "err=" << m5::hal::v2::error::toString(remaining_first.error());
    EXPECT_EQ(remaining_first.value().first.size + remaining_first.value().second.size, 1u);
    ASSERT_RESULT_OK(shared.popFrame(shared_metrics));
    ASSERT_RESULT_OK(shared.peekFrame(shared_metrics));
    ASSERT_RESULT_OK(shared.popFrame(shared_metrics));

    EXPECT_EQ(fixed_metrics.checksum, shared_metrics.checksum);
    EXPECT_EQ(fixed_metrics.copied_bytes, 8u);
    EXPECT_EQ(shared_metrics.copied_bytes, 3u);
    EXPECT_EQ(fixed_metrics.metadata_updates, 0u);
    EXPECT_EQ(shared_metrics.metadata_updates, 1u);
    std::cout << "S48_METRIC comparison=mode_fixed api_calls=" << fixed_metrics.api_calls
              << " copy_bytes=" << fixed_metrics.copied_bytes
              << " descriptor_updates=" << fixed_metrics.metadata_updates << '\n';
    std::cout << "S48_METRIC comparison=shared_cursor api_calls=" << shared_metrics.api_calls
              << " copy_bytes=" << shared_metrics.copied_bytes
              << " descriptor_updates=" << shared_metrics.metadata_updates << '\n';
}

TEST(SlaveQueuePrototype, VirtualAndStaticDispatchRunIdenticalRepeatedWorkload)
{
    constexpr size_t iterations = 30000;
    constexpr size_t trials     = 5;
    uint8_t virtual_bytes[32]{};
    FrameDescriptor virtual_descriptors[4]{};
    DispatchEndpoint virtual_endpoint{virtual_bytes, sizeof(virtual_bytes), virtual_descriptors,
                                      std::size(virtual_descriptors)};
    DispatchApi& virtual_view = virtual_endpoint;

    uint8_t static_bytes[32]{};
    FrameDescriptor static_descriptors[4]{};
    DispatchEndpoint static_endpoint{static_bytes, sizeof(static_bytes), static_descriptors,
                                     std::size(static_descriptors)};
    StaticDispatchView<DispatchEndpoint> static_view{static_endpoint};

    auto virtual_result = bestOf([&] { return virtualDispatchWorkload(virtual_view, iterations); }, trials);
    auto static_result  = bestOf([&] { return dispatchWorkload(static_view, iterations); }, trials);
    EXPECT_NE(virtual_result.second, 0u);
    EXPECT_NE(static_result.second, 0u);
    EXPECT_EQ(virtual_result.second, static_result.second);
    std::cout << "S48_METRIC dispatch=virtual best_ns=" << virtual_result.first << " iterations=" << iterations
              << " checksum=" << virtual_result.second << '\n';
    std::cout << "S48_METRIC dispatch=static best_ns=" << static_result.first << " iterations=" << iterations
              << " checksum=" << static_result.second << '\n';
}

TEST(SlaveQueuePrototype, SizesAndCallerStorageAllocationAuditAreReported)
{
    // A TU-local allocation gateway is used instead of global operator new, which would also count gtest internals.
    // All prototype storage constructors only bind caller memory. Any future internal allocation path must pass through
    // StorageAudit::allocate, making steady-state allocation mechanically observable without affecting other tests.
    StorageAudit::allocation_attempts = 0;
    uint8_t bytes[16]{};
    FrameDescriptor descriptors[3]{};
    FrameQueue queue{bytes, sizeof(bytes), descriptors, std::size(descriptors)};
    const uint8_t payload[] = {1, 2, 3, 4};
    for (size_t i = 0; i < 1000; ++i) {
        auto reservation = queue.reserveFrame(sizeof(payload));
        ASSERT_TRUE(reservation.has_value()) << "err=" << m5::hal::v2::error::toString(reservation.error());
        std::memcpy(reservation.value().first.data, payload, reservation.value().first.size);
        std::memcpy(reservation.value().second.data, payload + reservation.value().first.size,
                    reservation.value().second.size);
        ASSERT_RESULT_OK(queue.commitFrame(reservation.value(), {}));
        ASSERT_RESULT_OK(queue.popFrame());
    }
    EXPECT_EQ(StorageAudit::allocation_attempts, 0u);
    std::cout << "S48_METRIC sizeof_accessor=" << sizeof(TentativeAccessorLayout)
              << " sizeof_context=" << sizeof(TentativeOperationContext)
              << " sizeof_descriptor=" << sizeof(FrameDescriptor) << " sizeof_reservation=" << sizeof(FrameReservation)
              << " sizeof_token=" << sizeof(FrameToken) << '\n';
    std::cout << "S48_METRIC sizeof_dispatch_endpoint=" << sizeof(DispatchEndpoint)
              << " sizeof_static_dispatch_view=" << sizeof(StaticDispatchView<DispatchEndpoint>)
              << " sizeof_virtual_dispatch_pointer=" << sizeof(DispatchApi*) << '\n';
    std::cout << "S48_METRIC steady_state_heap_allocations=" << StorageAudit::allocation_attempts << " iterations=1000"
              << '\n';
    std::cout << "S48_METRIC baseline_frame_api_calls=4 baseline_frame_copy_bytes=" << sizeof(payload)
              << " frame_model=reserve_commit_peek_pop" << '\n';
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
