// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_SERVICE_COMPLETION_GATE_HPP_
#define M5_HAL_HAL_V2_SERVICE_COMPLETION_GATE_HPP_

#include "../../../../m5_hal_config.hpp"  // M5HAL_INLINE_V2

#include "../runtime/runtime.hpp"

#include <atomic>
#include <stdint.h>

namespace m5 {
namespace hal {
M5HAL_INLINE_V2 namespace v2
{
    namespace service {

    /*!
      @brief One step of a bounded wait: a yield phase with a wall-clock
      budget, then blocking 1 ms sleeps.

      Waiting for another task to make progress must eventually BLOCK, not
      merely yield: FreeRTOS taskYIELD() only yields to READY tasks of the
      SAME priority, so a waiter running above the producer's priority would
      spin forever without the producer ever being scheduled.

      The yield phase is budgeted in wall-clock time, not iterations: on a
      multi-core target a yield with no co-resident READY task returns in
      well under a microsecond, so an iteration count that looks generous
      evaporates in milliseconds (measured on ESP32). Meanwhile
      delayMs(1) rounds up to one FreeRTOS tick (10 ms at the IDF-default
      100 Hz), so entering the sleep phase while the producer is still
      working quantizes completion latency to the tick grid. Budget the
      yield phase to cover the wait's TYPICAL duration; the sleep phase is
      the liveness backstop for the priority-inverted case, not the
      expected path. The default fits a short wait such as a pending-slot
      consumption spin.
     */
    class SpinBackoff {
    public:
        static constexpr uint32_t kDefaultYieldBudgetUs = 200;

        constexpr SpinBackoff() = default;
        explicit constexpr SpinBackoff(uint32_t yield_budget_us) : _yield_budget_us{yield_budget_us}
        {
        }

        void step()
        {
            const auto now = runtime::micros();
            if (!_started) {
                _started  = true;
                _yield_t0 = now;
            }
            if (now - _yield_t0 < _yield_budget_us) {
                runtime::yield();
            } else {
                runtime::delayMs(1);
            }
        }
        void reset()
        {
            _started = false;
        }

    private:
        uint32_t _yield_budget_us = kDefaultYieldBudgetUs;
        uint32_t _yield_t0        = 0;
        bool _started             = false;
    };

    /*!
      @brief Completion gate: the standard primitive for publishing "this
      operation finished" from a producer task (typically a service polled
      by ServiceRunner) to a consumer thread. Plain (non-atomic) shared
      flags and volatile are forbidden for this role; use this gate.

      Contract:
      - The producer writes ALL payload fields (totals, error code, ...)
        BEFORE finish(); the release store publishes them.
      - A consumer may read payload only AFTER state() (acquire load)
        returned Done/Error. Timeout/error exits that skip the acquire
        must not touch payload.
      - The gate publishes VISIBILITY, not LIFETIME: reset() and payload
        destruction are allowed only once no producer can still touch the
        state (e.g. after ServiceRunner::remove() returned).
      - arm() is a consumer-side setup write. Hand the armed state to the
        producer through a synchronizing edge of its own (e.g.
        ServiceRunner::add()); the gate does not order setup writes.
      - No timeout parameter by design: a consumer-side timeout would
        return while the producer may still publish, reintroducing the
        exact lifetime bug this gate closes. Bound the wait inside the
        producer's state machine (deadline -> finish(Error)) instead.
     */
    class CompletionGate {
    public:
        enum class State : uint8_t { Idle, Busy, Done, Error };

        // -- consumer side --------------------------------------------
        void arm()
        {
            _state.store(static_cast<uint8_t>(State::Busy), std::memory_order_relaxed);
        }

        // Only after the producer can no longer publish (see contract).
        void reset()
        {
            _state.store(static_cast<uint8_t>(State::Idle), std::memory_order_relaxed);
        }

        State state() const
        {
            return static_cast<State>(_state.load(std::memory_order_acquire));
        }

        bool busy() const
        {
            return state() == State::Busy;
        }

        // Pure wait until the gate leaves Busy; returns the observed state
        // (Idle if never armed). Waiters that must pump work between polls
        // (e.g. waitTransfer) keep their own loop over state() + SpinBackoff.
        State wait()
        {
            SpinBackoff backoff;
            for (;;) {
                const auto s = state();
                if (s != State::Busy) {
                    return s;
                }
                backoff.step();
            }
        }

        // -- producer side --------------------------------------------
        // terminal must be Done or Error; payload writes must be sequenced
        // before this call.
        void finish(State terminal)
        {
            _state.store(static_cast<uint8_t>(terminal), std::memory_order_release);
        }

    private:
        std::atomic<uint8_t> _state{static_cast<uint8_t>(State::Idle)};
    };

    }  // namespace service
}
}  // namespace hal
}  // namespace m5

#endif  // M5_HAL_HAL_V2_SERVICE_COMPLETION_GATE_HPP_
