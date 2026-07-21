// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_BUS_OPERATION_HPP_
#define M5_HAL_HAL_V2_BUS_OPERATION_HPP_

#include "../error.hpp"
#include "../types.hpp"

#include <stddef.h>
#include <stdint.h>
#include <utility>

namespace m5::hal::v2::bus {

struct IAccessor;

enum class OperationMode : uint8_t {
    Control,
    Tx,
    Rx,
    TxRx,
    Slave,
};

enum class OperationStateFlags : uint16_t {
    None                 = 0,
    BackendStarted       = 1u << 0,
    Closing              = 1u << 1,
    Broken               = 1u << 2,
    ReservationCancelled = 1u << 3,
};

constexpr OperationStateFlags operator|(OperationStateFlags lhs, OperationStateFlags rhs)
{
    return static_cast<OperationStateFlags>(static_cast<uint16_t>(lhs) | static_cast<uint16_t>(rhs));
}

constexpr OperationStateFlags operator&(OperationStateFlags lhs, OperationStateFlags rhs)
{
    return static_cast<OperationStateFlags>(static_cast<uint16_t>(lhs) & static_cast<uint16_t>(rhs));
}

constexpr OperationStateFlags& operator|=(OperationStateFlags& lhs, OperationStateFlags rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

struct OperationRuntime {
    uint32_t started_ms       = 0;
    uint32_t timeout_ms       = 0;
    uint32_t generation       = 0;
    OperationStateFlags state = OperationStateFlags::None;
    OperationMode mode        = OperationMode::Control;
    uint8_t backend_slot      = 0xFF;

    void begin(uint32_t now_ms, uint32_t timeout, OperationMode operation_mode)
    {
        started_ms   = now_ms;
        timeout_ms   = timeout;
        mode         = operation_mode;
        state        = OperationStateFlags::None;
        backend_slot = 0xFF;
        ++generation;
        if (generation == 0) {
            ++generation;
        }
    }

    void beginClose(uint32_t now_ms, uint32_t timeout)
    {
        started_ms = now_ms;
        timeout_ms = timeout;
        state |= OperationStateFlags::Closing;
    }
};

constexpr uint32_t remainingTimeout(const OperationRuntime& runtime, uint32_t now_ms)
{
    if (runtime.timeout_ms == types::TIMEOUT_FOREVER) {
        return types::TIMEOUT_FOREVER;
    }
    const uint32_t elapsed = now_ms - runtime.started_ms;
    return elapsed < runtime.timeout_ms ? runtime.timeout_ms - elapsed : 0;
}

template <class Config>
class OperationContext {
public:
    Config config{};
    OperationRuntime runtime{};

    OperationContext()                                   = delete;
    OperationContext(const OperationContext&)            = delete;
    OperationContext& operator=(const OperationContext&) = delete;
    OperationContext(OperationContext&&)                 = delete;
    OperationContext& operator=(OperationContext&&)      = delete;

private:
    friend struct IAccessor;
    friend class OperationSlot;

    explicit OperationContext(IAccessor& owner, const Config& requested) : config{requested}, _owner{&owner}
    {
    }

    IAccessor* _owner      = nullptr;
    const void* _bus       = nullptr;
    uint32_t _generation   = 0;
    OperationMode _mode    = OperationMode::Control;
    bool _authority_active = false;
};

/*!
  @brief Bounded proof that one accessor context owns an active bus operation.

  A bus stores one slot per independently concurrent channel. Registration and
  validation use only pointer and integer comparisons; no ownership or
  allocation operation occurs on the transfer path.
 */
class OperationSlot {
public:
    template <class Config>
    result_t<void> registerContext(OperationContext<Config>& context, const void* bus, IAccessor* active_owner)
    {
        if (_context != nullptr || context._authority_active || context._owner == nullptr || bus == nullptr ||
            context.runtime.generation == 0 || active_owner != context._owner) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        context._bus              = bus;
        context._generation       = context.runtime.generation;
        context._mode             = context.runtime.mode;
        context._authority_active = true;
        _context                  = &context;
        _owner                    = context._owner;
        _generation               = context._generation;
        return {};
    }

    template <class Config>
    bool valid(const OperationContext<Config>& context, const void* bus, const IAccessor* active_owner) const
    {
        return _context == &context && _owner == context._owner && _owner == active_owner && _generation != 0 &&
               _generation == context._generation && _generation == context.runtime.generation &&
               context._mode == context.runtime.mode && context._authority_active && context._bus == bus;
    }

    template <class Config>
    bool registered(const OperationContext<Config>& context, const void* bus) const
    {
        return _context == &context && _owner == context._owner && context._authority_active && context._bus == bus;
    }

    template <class Config>
    bool restoreRegisteredRuntime(OperationContext<Config>& context, const void* bus,
                                  const IAccessor* active_owner) const
    {
        if (!registered(context, bus) || _owner != active_owner || _generation == 0) {
            return false;
        }
        context.runtime.generation = _generation;
        context.runtime.mode       = context._mode;
        return true;
    }

    template <class Config>
    void invalidate(OperationContext<Config>& context)
    {
        if (_context == &context) {
            _context    = nullptr;
            _owner      = nullptr;
            _generation = 0;
        }
        context._authority_active = false;
        context._bus              = nullptr;
        context._mode             = OperationMode::Control;
    }

    template <class Config>
    static IAccessor& contextOwner(const OperationContext<Config>& context)
    {
        return *context._owner;
    }

    template <class Config>
    static OperationMode registeredMode(const OperationContext<Config>& context)
    {
        return context._mode;
    }

private:
    const void* _context = nullptr;
    IAccessor* _owner    = nullptr;
    uint32_t _generation = 0;
};

struct TransferTotals {
    size_t tx = 0;
    size_t rx = 0;

    void clear()
    {
        tx = 0;
        rx = 0;
    }

    void add(const TransferTotals& other)
    {
        tx += other.tx;
        rx += other.rx;
    }
};

enum class CompletionLevel : uint8_t {
    None,
    Accepted,
    Partial,
    Complete,
    Aborted,
};

struct TransferStatus {
    TransferTotals totals{};
    uint32_t transfer_id       = 0;
    error::error_t error       = error::error_t::OK;
    CompletionLevel completion = CompletionLevel::None;
    uint16_t state_flags       = 0;
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_HAL_V2_BUS_OPERATION_HPP_
