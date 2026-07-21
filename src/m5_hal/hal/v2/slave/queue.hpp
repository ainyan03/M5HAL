// SPDX-License-Identifier: MIT
#ifndef M5_HAL_SLAVE_QUEUE_HPP_
#define M5_HAL_SLAVE_QUEUE_HPP_

#include "../data.hpp"
#include "../error.hpp"

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace m5::hal::v2::slave {

enum class FrameFlags : uint16_t {
    None      = 0,
    Begin     = 1u << 0,
    End       = 1u << 1,
    Aborted   = 1u << 2,
    Truncated = 1u << 3,
    Overflow  = 1u << 4,
    Underrun  = 1u << 5,
};

constexpr FrameFlags operator|(FrameFlags lhs, FrameFlags rhs);
constexpr FrameFlags operator&(FrameFlags lhs, FrameFlags rhs);
constexpr FrameFlags operator~(FrameFlags value);
constexpr FrameFlags& operator|=(FrameFlags& lhs, FrameFlags rhs);
constexpr FrameFlags& operator&=(FrameFlags& lhs, FrameFlags rhs);
constexpr bool any(FrameFlags value);

struct FrameMetadata {
    uint32_t frame_id      = 0;
    uint32_t wire_bytes    = 0;
    uint32_t stored_bytes  = 0;
    uint32_t dropped_bytes = 0;
    FrameFlags flags       = FrameFlags::None;
    error::error_t error   = error::error_t::OK;
    uint8_t segment_count  = 0;
};

enum class QueueEventFlags : uint16_t {
    None      = 0,
    Overflow  = 1u << 0,
    Underrun  = 1u << 1,
    Truncated = 1u << 2,
};

constexpr QueueEventFlags operator|(QueueEventFlags lhs, QueueEventFlags rhs);
constexpr QueueEventFlags operator&(QueueEventFlags lhs, QueueEventFlags rhs);
constexpr QueueEventFlags& operator|=(QueueEventFlags& lhs, QueueEventFlags rhs);
constexpr bool any(QueueEventFlags value);

struct QueueStatus {
    uint32_t dropped_bytes        = 0;
    uint32_t dropped_frames       = 0;
    uint32_t underrun_count       = 0;
    QueueEventFlags sticky_events = QueueEventFlags::None;
};

struct FrameDescriptor {
    uint32_t offset = 0;
    uint32_t length = 0;
    FrameMetadata metadata{};
};

template <class Descriptor = FrameDescriptor>
struct QueueStorage {
    data::DataSpan bytes{};
    Descriptor* descriptors = nullptr;
    size_t descriptor_count = 0;
};

template <size_t TxBytes, size_t RxBytes, size_t TxFrames, size_t RxFrames>
class StaticSlaveQueueStorage {
public:
    QueueStorage<> tx()
    {
        return {{tx_bytes_.data(), tx_bytes_.size()}, tx_frames_.data(), tx_frames_.size()};
    }
    QueueStorage<> rx()
    {
        return {{rx_bytes_.data(), rx_bytes_.size()}, rx_frames_.data(), rx_frames_.size()};
    }

private:
    std::array<uint8_t, TxBytes> tx_bytes_{};
    std::array<uint8_t, RxBytes> rx_bytes_{};
    std::array<FrameDescriptor, TxFrames> tx_frames_{};
    std::array<FrameDescriptor, RxFrames> rx_frames_{};
};

enum class QueueMode : uint8_t { Byte, Frame };

class FrameToken {
public:
    FrameToken()                             = default;
    FrameToken(const FrameToken&)            = delete;
    FrameToken& operator=(const FrameToken&) = delete;
    FrameToken(FrameToken&& other) noexcept;
    FrameToken& operator=(FrameToken&& other) noexcept;

private:
    friend class SlaveQueue;

    FrameToken(uintptr_t owner_cookie, uint32_t generation, uint32_t sequence);
    void invalidate();

    uintptr_t owner_cookie_ = 0;
    uint32_t generation_    = 0;
    uint32_t sequence_      = 0;
};

struct FrameView {
    data::ConstDataSpan first{};
    data::ConstDataSpan second{};
    FrameMetadata metadata{};
};

struct ByteView {
    data::ConstDataSpan first{};
    data::ConstDataSpan second{};
};

struct FrameReservation {
    data::DataSpan first{};
    data::DataSpan second{};
    FrameToken token{};

    FrameReservation()                                       = default;
    FrameReservation(const FrameReservation&)                = delete;
    FrameReservation& operator=(const FrameReservation&)     = delete;
    FrameReservation(FrameReservation&&) noexcept            = default;
    FrameReservation& operator=(FrameReservation&&) noexcept = default;
};

class SlaveQueue;

class ByteQueueView {
public:
    ByteQueueView() = default;

    result_t<size_t> write(data::ConstDataSpan src);
    result_t<size_t> read(data::DataSpan dst);
    size_t readable() const;
    size_t writable() const;

private:
    friend class SlaveQueue;
    explicit ByteQueueView(SlaveQueue* queue) : queue_{queue}
    {
    }

    SlaveQueue* queue_ = nullptr;
};

class FrameSourceView {
public:
    FrameSourceView() = default;

    result_t<FrameView> peekFrame() const;
    result_t<void> popFrame();
    size_t readableFrames() const;

private:
    friend class SlaveQueue;
    explicit FrameSourceView(SlaveQueue* queue) : queue_{queue}
    {
    }

    SlaveQueue* queue_ = nullptr;
};

class FrameSinkView {
public:
    FrameSinkView() = default;

    result_t<FrameReservation> reserveFrame(size_t payload_size);
    result_t<void> commitFrame(FrameReservation& reservation, const FrameMetadata& metadata);
    result_t<void> cancelFrame(FrameReservation& reservation);
    result_t<void> writeFrame(data::ConstDataSpan payload, const FrameMetadata& metadata);
    size_t writableBytes() const;
    size_t writableFrames() const;

private:
    friend class SlaveQueue;
    explicit FrameSinkView(SlaveQueue* queue) : queue_{queue}
    {
    }

    SlaveQueue* queue_ = nullptr;
};

/*!
  @brief Caller-storage SPSC byte/frame queue used by slave Accessors.

  Exactly one producer owns `write`, frame reservation/commit/cancel, and
  overflow accounting. Exactly one consumer owns `read`, peek/pop, and status
  snapshots. Binding and mode changes require both sides to be idle; `reset`
  is the lifecycle teardown operation that invalidates queued data and tokens.
  Release publication of the producer indices and acquire observation by the
  consumer make payload and descriptor writes visible without putting atomics
  in the descriptor itself.
 */
class SlaveQueue {
public:
    SlaveQueue()                             = default;
    SlaveQueue(const SlaveQueue&)            = delete;
    SlaveQueue& operator=(const SlaveQueue&) = delete;
    SlaveQueue(SlaveQueue&&)                 = delete;
    SlaveQueue& operator=(SlaveQueue&&)      = delete;

    result_t<void> bind(QueueStorage<> storage, QueueMode mode, uint32_t generation = 0);
    result_t<void> setMode(QueueMode mode);
    QueueMode mode() const;

    ByteQueueView bytes();
    FrameSourceView frameSource();
    FrameSinkView frameSink();

    result_t<size_t> write(data::ConstDataSpan src);
    result_t<size_t> read(data::DataSpan dst);
    result_t<ByteView> peekBytes(size_t maximum) const;
    result_t<void> popBytes(size_t count);
    size_t readable() const;
    size_t writable() const;

    result_t<FrameReservation> reserveFrame(size_t payload_size);
    // Backend producer seam for a wire frame whose final size is unknown until
    // its protocol boundary. Begin reserves one descriptor without publishing
    // it; append grows the private payload prefix; commitFrame publishes both.
    result_t<FrameReservation> beginObservedFrame();
    result_t<size_t> appendObservedFrame(FrameReservation& reservation, data::ConstDataSpan payload);
    result_t<void> commitFrame(FrameReservation& reservation, const FrameMetadata& metadata);
    result_t<void> cancelFrame(FrameReservation& reservation);
    result_t<void> writeFrame(data::ConstDataSpan payload, const FrameMetadata& metadata);
    result_t<void> writeObservedFrame(data::ConstDataSpan payload, FrameMetadata metadata);
    result_t<FrameView> peekFrame() const;
    result_t<void> popFrame();
    size_t readableFrames() const;
    size_t writableFrames() const;

    bool hasReservation() const;
    result_t<void> cancelReservation();
    result_t<void> setGeneration(uint32_t generation);
    void reset(uint32_t generation);
    uint32_t generation() const;

    QueueStatus status() const;
    QueueStatus clearStatus();
    void recordDroppedFrame(uint32_t dropped_bytes);
    void recordDroppedFrames(uint32_t frame_count, uint32_t dropped_bytes);
    void recordUnderrun(uint32_t count = 1);

private:
    static uintptr_t ownerCookie(const SlaveQueue* queue);
    static uint32_t used(uint32_t producer, uint32_t consumer);
    static void copyInto(uint8_t* storage, uint32_t capacity, uint32_t offset, const uint8_t* src, uint32_t size);
    static void copyOut(const uint8_t* storage, uint32_t capacity, uint32_t offset, uint8_t* dst, uint32_t size);
    static uint32_t saturatingAdd(std::atomic<uint32_t>& target, uint32_t increment);

    bool bound() const;
    bool empty() const;
    bool tokenMatches(const FrameToken& token) const;
    void invalidateReservation(FrameReservation& reservation);
    uint32_t nextSequence();
    data::DataSpan mutableSpan(uint32_t offset, uint32_t size);
    data::ConstDataSpan constSpan(uint32_t offset, uint32_t size) const;

    uint8_t* byte_storage_               = nullptr;
    FrameDescriptor* descriptor_storage_ = nullptr;
    uint32_t byte_capacity_              = 0;
    uint32_t descriptor_capacity_        = 0;
    bool bound_                          = false;

    std::atomic<uint32_t> byte_head_{0};
    std::atomic<uint32_t> byte_tail_{0};
    std::atomic<uint32_t> descriptor_head_{0};
    std::atomic<uint32_t> descriptor_tail_{0};
    std::atomic<uint8_t> mode_{static_cast<uint8_t>(QueueMode::Byte)};

    uint32_t generation_          = 0;
    uint32_t sequence_            = 0;
    uint32_t reserved_byte_tail_  = 0;
    uint32_t reserved_byte_count_ = 0;
    uint32_t reserved_sequence_   = 0;

    std::atomic<uint32_t> dropped_bytes_{0};
    std::atomic<uint32_t> dropped_frames_{0};
    std::atomic<uint32_t> underrun_count_{0};
    std::atomic<uint16_t> sticky_events_{0};
};

static_assert(std::is_standard_layout<FrameMetadata>::value, "FrameMetadata must remain a plain shared value");
static_assert(sizeof(ByteQueueView) == sizeof(void*), "ByteQueueView must remain a one-pointer thin view");
static_assert(sizeof(FrameSourceView) == sizeof(void*), "FrameSourceView must remain a one-pointer thin view");
static_assert(sizeof(FrameSinkView) == sizeof(void*), "FrameSinkView must remain a one-pointer thin view");

}  // namespace m5::hal::v2::slave

#include "queue.inl"

#endif
