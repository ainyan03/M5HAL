// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_SERVICE_SERVICE_INL_
#define M5_HAL_HAL_V2_SERVICE_SERVICE_INL_

#include "service.hpp"

#include "../diag.hpp"

#if defined(ESP_PLATFORM)
#if __has_include(<esp_idf_version.h>)
#include <esp_idf_version.h>
#endif
#if __has_include(<esp_cpu.h>)
#include <esp_cpu.h>
#define M5HAL_SERVICE_INL_HAS_ESP_CPU_H_ 1
#endif
#if __has_include(<esp_private/esp_clk.h>)
#include <esp_private/esp_clk.h>
#define M5HAL_SERVICE_INL_HAS_ESP_CLK_CPU_FREQ_ 1
#endif
#if __has_include(<esp_timer.h>)
#include <esp_timer.h>
#define M5HAL_SERVICE_INL_HAS_ESP_TIMER_H_ 1
#endif
#if !(defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5)
#include <freertos/FreeRTOS.h>  // xPortGetCoreID for the pre-5 core-id read
#endif
#elif defined(ARDUINO)
// Non-ESP32 Arduino cores may ship an old GCC/libstdc++ toolchain where
// Arduino.h's abs()/round() macros corrupt <chrono>'s own internal use of
// those names (see M5Utility's library_log.hpp for the same finding).
// sharedNowUs() below extends micros() into a 64-bit count instead.
#include <Arduino.h>
#else
#include <chrono>
#endif

// Auto-run needs a Task backend that provides REAL concurrency guarantees —
// see "## stub が対象外である理由" below: stub's runtime::Mutex is a
// non-blocking single-owner guard (not real mutual exclusion) and its
// currentTaskId() is a fixed value, so R1/R8 break under actual concurrent
// access. Only freertos and posix qualify. `defined(ARDUINO)` used to be a
// safe proxy for "freertos wins runtime::Task" (arduino always meant
// arduino-esp32), but no longer is now that RP2040/SAMD51 are admitted (see
// _checker.hpp's variant allowlist): there, no freertos/posix is available
// and stub wins runtime::Task instead. Gate on the SELECTED Task variant
// directly instead of the framework macro.
#if (M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS) || \
    (M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX)
#define M5HAL_SERVICE_AUTORUN_TASK_SUPPORTED_ 1
#else
#define M5HAL_SERVICE_AUTORUN_TASK_SUPPORTED_ 0
#endif

namespace m5::hal::v2::service {

fast_tick_t fastTick()
{
#if defined(ESP_PLATFORM) && defined(M5HAL_SERVICE_INL_HAS_ESP_CPU_H_)
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
    return static_cast<fast_tick_t>(esp_cpu_get_cycle_count());
#else
    return static_cast<fast_tick_t>(esp_cpu_get_ccount());
#endif
#else
    return static_cast<fast_tick_t>(m5::utility::micros());
#endif
}

uint32_t fastTickFrequencyHz()
{
    // Branch on the SAME condition as fastTick(): the frequency must
    // describe whatever counter fastTick() actually reads.
#if defined(ESP_PLATFORM) && defined(M5HAL_SERVICE_INL_HAS_ESP_CPU_H_)
    // fastTick() reads the CPU cycle counter, so the frequency is the
    // CPU clock.
#if defined(M5HAL_SERVICE_INL_HAS_ESP_CLK_CPU_FREQ_)
    const auto freq_hz = esp_clk_cpu_freq();
    if (freq_hz > 0) {
        return static_cast<uint32_t>(freq_hz);
    }
#endif
    // F_CPU and CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ are ESP-specific; keeping
    // them inside the ESP_PLATFORM guard prevents non-ESP targets (e.g. AVR)
    // that also define F_CPU from returning a CPU frequency here while
    // fastTick() actually returns micros() (1 MHz) — a guaranteed mismatch.
#if defined(F_CPU) && F_CPU > 0
    return static_cast<uint32_t>(F_CPU);
#elif defined(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) && CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ > 0
    return static_cast<uint32_t>(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) * uint32_t{1000000};
#else
    return uint32_t{240000000};  // last-resort default for ESP without clock query
#endif
#else
    // Everywhere else (non-ESP, or ESP without esp_cpu.h) fastTick()
    // returns micros(), which is always 1 MHz regardless of the CPU
    // clock. F_CPU is intentionally not consulted here.
    return uint32_t{1000000};
#endif
}

fast_tick_t defaultNowTick()
{
    return fastTick();
}

uint32_t fastTickDomain()
{
    // Branch on the SAME condition as fastTick(): the domain must
    // describe whatever counter fastTick() actually reads. The cycle
    // counter is per-core, so the domain is the core id; the micros()
    // fallback is a single shared counter, so one constant domain.
#if M5HAL_CONFIG_SERVICE_ASSUME_PINNED
    return 0;
#elif defined(ESP_PLATFORM) && defined(M5HAL_SERVICE_INL_HAS_ESP_CPU_H_)
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
    return static_cast<uint32_t>(esp_cpu_get_core_id());
#else
    return static_cast<uint32_t>(xPortGetCoreID());
#endif
#else
    return 0;
#endif
}

uint64_t sharedNowUs()
{
#if defined(M5HAL_SERVICE_INL_HAS_ESP_TIMER_H_)
    return static_cast<uint64_t>(::esp_timer_get_time());
#elif defined(ESP_PLATFORM)
    // A widened 32-bit micros() would NOT be wrap-safe for the 64-bit
    // difference comparisons this clock promises (a transfer crossing the
    // ~71-minute wrap would time out instantly), and a lock-free 32->64
    // extension is not worth carrying for a configuration that does not
    // exist on IDF/Arduino. Fail the build instead of miscounting.
#error "M5HAL: ESP build without <esp_timer.h> — no wrap-safe shared clock available"
#elif defined(ARDUINO)
    // No <esp_timer.h> and no portable 64-bit free-running counter across the
    // allowlisted non-ESP Arduino cores (see _checker.hpp), so extend the
    // 32-bit micros() into a monotonic 64-bit count by tracking wraps.
    // Wrap-safe as long as this is called at least once per ~71-minute
    // micros() period; noInterrupts() guards the CURRENT core only, so RP2
    // multicore use remains outside the verified surface.
    noInterrupts();
    static uint32_t s_last_us  = 0;
    static uint64_t s_epoch_us = 0;
    const uint32_t now         = static_cast<uint32_t>(::micros());
    if (now < s_last_us) {
        s_epoch_us += (uint64_t{1} << 32);
    }
    s_last_us             = now;
    const uint64_t result = s_epoch_us + now;
    interrupts();
    return result;
#else
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
#endif
}

TickSample sampleTickWithDomain()
{
    TickSample s;
    const uint32_t d0 = fastTickDomain();
    s.tick            = fastTick();
    // Domain read on both sides of the tick read: if they disagree the
    // task migrated mid-sample and the tick's owner is unknown — mark the
    // sample torn so the stream gap-drops rather than storing core A's
    // tick under core B's domain. On pinned/unicore builds both reads
    // fold to the same constant and the compare vanishes.
    s.domain = (fastTickDomain() == d0) ? d0 : kInvalidTickDomain;
    return s;
}

ServicePoll IService::service(const ServiceContext& ctx)
{
    return serviceImpl(ctx);
}

ServiceRunner::~ServiceRunner()
{
    auto stopped = stopAutoRun();
    if (!stopped.has_value() && _auto_running.load(std::memory_order_acquire)) {
        // Destruction cannot return the control error, but it must not let a
        // runner task retain `this` after the object lifetime ends.
        _auto_stop.store(true, std::memory_order_release);
        _wake.notify();
        _auto_task.join();
        _auto_running.store(false, std::memory_order_release);
    }
}

result_t<void> ServiceRunner::unlockControl()
{
    auto unlocked = _control.unlock();
    if (!unlocked.has_value()) {
        _control_broken.store(true, std::memory_order_release);
    }
    return unlocked;
}

result_t<void> ServiceRunner::add(IService& service)
{
    if (controlBroken()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (isCurrentWriter()) {
        // Self-add from inside a serviceImpl on the table-owning task: the
        // owner is the sole writer (R1), so apply directly. Must not take
        // _control (see isCurrentWriter()).
        return applyAdd(&service);
    }
    auto control_locked = _control.lock(types::TIMEOUT_FOREVER);
    if (!control_locked.has_value()) {
        return m5::stl::make_unexpected(control_locked.error());
    }
    result_t<void> result;
    if (!_auto_running.load(std::memory_order_acquire)) {
        result = applyAdd(&service);
        if (result) {
            // Implicit auto-run start is an EMBEDDED-framework usability
            // affordance only. posix has a working auto-run backend
            // (startAutoRun() works), but must never start it implicitly:
            // native tests drive the table manually via runOnce(), and an
            // implicitly spawned runner task would own the table and turn
            // every manual pass into a try-lock back-off
            // (spec/design/service.md: "posix では add() からの暗黙起動は
            // 行わない"). On bare-Arduino targets with only the stub Task
            // (allowlisted non-ESP Arduino cores), the backend cannot run — skip the attempt
            // instead of ignoring its failure.
#if defined(ESP_PLATFORM) || (defined(ARDUINO) && M5HAL_SERVICE_AUTORUN_TASK_SUPPORTED_) || \
    defined(M5HAL_TEST_SERVICE_IMPLICIT_AUTORUN)
            auto started = startAutoRunLocked();
            if (!started) {
                applyRemove(&service);
                result = m5::stl::make_unexpected(started.error());
            }
#endif
        }
    } else {
        // _control remains held until acknowledgment, so no other external
        // add() can own or reuse this slot. Callback add/remove runs on the
        // sole table writer and may complete/cancel it without taking
        // _control.
        _pending_add_outcome.store(PendingAddOutcome::Pending, std::memory_order_relaxed);
        _pending_add.store(&service, std::memory_order_release);
        _has_pending.store(true, std::memory_order_release);
        _wake.notify();
        uint32_t attempt = 0;
        while (_pending_add.load(std::memory_order_acquire) == &service) {
            if (attempt < 8) {
                ++attempt;
                runtime::yield();
            } else {
                runtime::delayMs(1);
            }
        }
        switch (_pending_add_outcome.load(std::memory_order_relaxed)) {
            case PendingAddOutcome::Success:
                result = result_t<void>{};
                break;
            case PendingAddOutcome::InvalidState:
                result = m5::stl::make_unexpected(error::error_t::INVALID_STATE);
                break;
            case PendingAddOutcome::OutOfResource:
                result = m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
                break;
            case PendingAddOutcome::Pending:
                // Pointer consumption release-publishes the outcome first;
                // observing Pending here would violate the slot protocol, not
                // indicate exhaustion of a public capacity.
                result = m5::stl::make_unexpected(error::error_t::UNKNOWN_ERROR);
                break;
        }
    }
    auto unlocked = unlockControl();
    if (result.has_value() && !unlocked.has_value()) {
        return m5::stl::make_unexpected(unlocked.error());
    }
    return result;
}

result_t<void> ServiceRunner::remove(IService& service)
{
    if (controlBroken()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (isCurrentWriter()) {
        // Self-remove: a service removing itself or a sibling from inside its
        // own serviceImpl() on the table-owning task. Apply directly -- the
        // owner is the sole writer (R1) and applyRemove() compensates the
        // in-pass cursor. Must not take _control or wait (a self-wait would
        // deadlock). Cancel a queued add of the target first so it cannot be
        // resurrected.
        cancelPendingAdd(&service);
        applyRemove(&service);
        return {};
    }
    auto control_locked = _control.lock(types::TIMEOUT_FOREVER);
    if (!control_locked.has_value()) {
        return m5::stl::make_unexpected(control_locked.error());
    }
    if (!_auto_running.load(std::memory_order_acquire)) {
        // Table is still (no runner task): apply directly under _control.
        const bool present = findInTable(&service) != kMaxServices;
        if (present) {
            applyRemove(&service);
        }
        return unlockControl();
    }
    // Auto-run owns the table: cancel a queued add of the target, then queue
    // the removal and wait for the runner task to consume it.
    cancelPendingAdd(&service);
    // Waiting for the runner task to make progress must eventually BLOCK,
    // not merely yield: FreeRTOS taskYIELD() only yields to READY tasks of
    // the SAME priority, so a caller running above the runner's priority
    // (1) would spin forever without the runner ever being scheduled.
    // After a short yield burst (fast path when the runner is about to
    // flush anyway), fall back to a blocking 1 ms sleep.
    uint32_t attempt = 0;
    auto wait_step   = [&attempt]() {
        if (attempt < 8) {
            ++attempt;
            runtime::yield();
        } else {
            runtime::delayMs(1);
        }
    };
    size_t slot = kMaxPendingRemoves;
    for (;;) {
        for (size_t i = 0; i < kMaxPendingRemoves; ++i) {
            IService* expected = nullptr;
            if (_pending_remove[i].compare_exchange_strong(expected, &service, std::memory_order_relaxed,
                                                           std::memory_order_relaxed)) {
                slot = i;
                break;
            }
            if (expected == &service) {
                slot = i;  // already queued by an earlier remove(); wait on it too
                break;
            }
        }
        if (slot != kMaxPendingRemoves) {
            break;
        }
        // All slots belong to other in-flight removals. Returning false here
        // would hand the caller back a service that is still being polled --
        // exactly what the "returned => never polled again" contract forbids
        // (teardown callers delete right after remove()). Instead, release
        // _control so the runner task can flush the queue, give it CPU, and
        // retry.
        auto released = unlockControl();
        if (!released.has_value()) {
            return m5::stl::make_unexpected(released.error());
        }
        wait_step();
        control_locked = _control.lock(types::TIMEOUT_FOREVER);
        if (!control_locked.has_value()) {
            return m5::stl::make_unexpected(control_locked.error());
        }
        if (!_auto_running.load(std::memory_order_acquire)) {
            // The runner stopped while we backed off: direct path.
            const bool present = findInTable(&service) != kMaxServices;
            if (present) {
                applyRemove(&service);
            }
            return unlockControl();
        }
    }
    _has_pending.store(true, std::memory_order_release);
    // Wake the runner so it consumes the queued removal on its next pass
    // instead of sleeping out its idle wait (the caller's own polling below
    // still ticks, but the runner-side latency is gone).
    _wake.notify();
    auto released = unlockControl();
    if (!released.has_value()) {
        return m5::stl::make_unexpected(released.error());
    }

    // Wait until the runner task's flushPending() consumes our slot.
    // flushPending() runs at the HEAD of every pass, before any poll, and
    // exchanges the slot to null (release) BEFORE -- in program order on the
    // runner task -- it applyRemove()s the target and then polls. So once we
    // observe null (acquire), the runner is committed to removing the target
    // before it can poll it again: "returned => never polled again" holds.
    // No generation counter is needed. The alternative failure it would guard
    // (a concurrent add(S) re-registering the target inside the same flush) is
    // excluded two ways: sequential add()->remove() was cancelled above, and a
    // CONCURRENT add()+remove() of the same service is contract-undefined (see
    // remove() docs). Slot consumption is therefore a sufficient signal.
    for (;;) {
        if (_pending_remove[slot].load(std::memory_order_acquire) != &service) {
            return {};  // consumed: removal observed by the runner task
        }
        if (!_auto_running.load(std::memory_order_acquire)) {
            // The runner task appears to have stopped; finish under _control.
            control_locked = _control.lock(types::TIMEOUT_FOREVER);
            if (!control_locked.has_value()) {
                return m5::stl::make_unexpected(control_locked.error());
            }
            if (!_auto_running.load(std::memory_order_acquire)) {
                // Confirmed stopped -- no task can start while we hold _control
                // (startAutoRunLocked needs it) -- so the table is ours: drain
                // our slot if still unconsumed and apply the removal directly.
                IService* cur = _pending_remove[slot].exchange(nullptr, std::memory_order_acq_rel);
                if (cur == &service) {
                    applyRemove(&service);
                }
                return unlockControl();
            }
            auto resumed = unlockControl();  // a new runner task started; resume waiting
            if (!resumed.has_value()) {
                return m5::stl::make_unexpected(resumed.error());
            }
        }
        wait_step();
    }
}

result_t<void> ServiceRunner::clear()
{
    if (controlBroken()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (isCurrentWriter()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto control_locked = _control.lock(types::TIMEOUT_FOREVER);
    if (!control_locked.has_value()) {
        return m5::stl::make_unexpected(control_locked.error());
    }
    if (_auto_running.load(std::memory_order_acquire)) {
        // Stop (join) the runner task so the table is still. It stays stopped;
        // the next add() restarts it. (Draining into pending is not always
        // possible: kMaxPendingRemoves < kMaxServices. clear() is a low-frequency
        // teardown path, so paying the task re-create cost is acceptable.)
        stopAutoRunLocked();
    }
    _pending_add.store(nullptr, std::memory_order_relaxed);
    _pending_add_outcome.store(PendingAddOutcome::Pending, std::memory_order_relaxed);
    for (size_t i = 0; i < kMaxPendingRemoves; ++i) {
        _pending_remove[i].store(nullptr, std::memory_order_relaxed);
    }
    _has_pending.store(false, std::memory_order_relaxed);
    const size_t n = _count.load(std::memory_order_relaxed);
    for (size_t i = 0; i < n; ++i) {
        _services[i]    = nullptr;
        _next_due[i]    = 0;
        _last_polled[i] = 0;
    }
    _count.store(0, std::memory_order_relaxed);
    // Invalidate the default-clock stream so the first pass after a
    // restart gap-drops instead of trusting a reading from before the
    // teardown. _virtual_now deliberately keeps its value: applyAdd
    // re-baselines _last_polled against it, so any origin is valid.
    _stream = TickStream{};
    return unlockControl();
}

ServicePoll ServiceRunner::run(IService& service, const ServiceContext& ctx)
{
    return service.service(ctx);
}

result_t<bool> ServiceRunner::runOnce(const ServiceContext& ctx)
{
    if (controlBroken()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (isCurrentWriter()) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    // Try-lock: a contender (another runOnce, or an add/remove/clear in
    // flight) backs off without polling, implementing the serialization the
    // header promises. Auto-run drives the table on its own task, so a caller
    // must also back off while it is active.
    auto control_locked = _control.lock(0);
    if (!control_locked.has_value()) {
        const auto mapped =
            control_locked.error() == error::error_t::TIMEOUT_ERROR ? error::error_t::BUSY : control_locked.error();
        return m5::stl::make_unexpected(mapped);
    }
    // RAII so a serviceImpl() exception (host builds) cannot leak the lock.
    struct ControlUnlock {
        ServiceRunner* runner;
        bool active = true;
        ~ControlUnlock()
        {
            if (active) {
                (void)runner->unlockControl();
            }
        }
        result_t<void> release()
        {
            active = false;
            return runner->unlockControl();
        }
    } unlock_guard{this};
    if (_auto_running.load(std::memory_order_acquire)) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    // Explicit-context pass: the caller vouches for elapsed. Invalidate
    // the default-clock stream so the next default pass gap-drops
    // (elapsed=0) as the mixing contract promises — its previous reading
    // predates this pass, and counting the real time across it would
    // double-count on top of the explicit elapsed supplied here.
    _stream       = TickStream{};
    auto result   = runOnceInternal(ctx.elapsed, ctx.local_tick, /*refresh_local_each_poll=*/false);
    auto unlocked = unlock_guard.release();
    if (!unlocked.has_value()) {
        return m5::stl::make_unexpected(unlocked.error());
    }
    return result;
}

result_t<bool> ServiceRunner::runOnce()
{
    if (controlBroken()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (isCurrentWriter()) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    auto control_locked = _control.lock(0);
    if (!control_locked.has_value()) {
        const auto mapped =
            control_locked.error() == error::error_t::TIMEOUT_ERROR ? error::error_t::BUSY : control_locked.error();
        return m5::stl::make_unexpected(mapped);
    }
    struct ControlUnlock {
        ServiceRunner* runner;
        bool active = true;
        ~ControlUnlock()
        {
            if (active) {
                (void)runner->unlockControl();
            }
        }
        result_t<void> release()
        {
            active = false;
            return runner->unlockControl();
        }
    } unlock_guard{this};
    if (_auto_running.load(std::memory_order_acquire)) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    // Step the stream only HERE, after the pass is committed (lock held,
    // no auto-run): stepping before the back-off checks would advance
    // `prev` on a pass that never runs and permanently drop that interval,
    // and would race the auto-run task's own step.
    const TickSample s = sampleTickWithDomain();
    auto result        = runOnceInternal(_stream.step(s.tick, s.domain), s.tick, /*refresh_local_each_poll=*/true);
    auto unlocked      = unlock_guard.release();
    if (!unlocked.has_value()) {
        return m5::stl::make_unexpected(unlocked.error());
    }
    return result;
}

size_t ServiceRunner::size() const
{
    return _count.load(std::memory_order_relaxed);
}

size_t ServiceRunner::capacity() const
{
    return kMaxServices;
}

result_t<void> ServiceRunner::startAutoRun()
{
    if (controlBroken()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (isCurrentWriter()) {
        if (_auto_running.load(std::memory_order_acquire)) {
            return {};
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto control_locked = _control.lock(types::TIMEOUT_FOREVER);
    if (!control_locked.has_value()) {
        return m5::stl::make_unexpected(control_locked.error());
    }
    auto result   = startAutoRunLocked();
    auto unlocked = unlockControl();
    if (result.has_value() && !unlocked.has_value()) {
        return m5::stl::make_unexpected(unlocked.error());
    }
    return result;
}

result_t<void> ServiceRunner::startAutoRunLocked()
{
#if defined(ESP_PLATFORM) || M5HAL_SERVICE_AUTORUN_TASK_SUPPORTED_
    if (_auto_running.load(std::memory_order_acquire)) {
        return {};
    }
    _auto_stop.store(false, std::memory_order_release);
    auto started =
        _auto_task.start(&ServiceRunner::autoRunEntry, this, "m5hal-svc", 4096, 1, M5HAL_CONFIG_SERVICE_AUTORUN_CORE);
    if (!started) {
        _auto_stop.store(true, std::memory_order_release);
        return m5::stl::make_unexpected(started.error());
    }
    // Publish AFTER the task exists. Every table-affecting path reads
    // _auto_running under _control (held here), so ordering against them is
    // already serialized; the flag's only lock-free readers are advisory
    // status queries (autoRunActive() and spin-waits built on it). If the
    // flag went up BEFORE the task existed, such a reader could treat a
    // not-yet-created (or failed) runner as alive and wait on it -- and a
    // higher-priority waiter spinning on that belief can starve this thread
    // out of ever reaching the create/revert, wedging both. Start -> flag
    // removes that window. autoRunEntry() waits on _wake until this publish,
    // so even a newly created higher-priority task cannot poll a service while
    // callbacks would still observe the runner as inactive.
    _auto_running.store(true, std::memory_order_release);
    _wake.notify();
    return {};
#else
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
#endif
}

result_t<void> ServiceRunner::stopAutoRun()
{
    if (controlBroken()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (isCurrentWriter()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto control_locked = _control.lock(types::TIMEOUT_FOREVER);
    if (!control_locked.has_value()) {
        return m5::stl::make_unexpected(control_locked.error());
    }
    if (!_auto_running.load(std::memory_order_acquire)) {
        return unlockControl();
    }
    stopAutoRunLocked();
    return unlockControl();
}

void ServiceRunner::stopAutoRunLocked()
{
#if defined(ESP_PLATFORM) || M5HAL_SERVICE_AUTORUN_TASK_SUPPORTED_
    _auto_stop.store(true, std::memory_order_release);
    // Wake the runner so the join below does not wait out an idle sleep.
    // A notify that outruns the loop's exit check simply stays latched;
    // after a restart the first idle wait consumes it and re-loops — the
    // pass (runOnceInternal) runs before the wait, so nothing is missed.
    _wake.notify();
    _auto_task.join();
    _auto_running.store(false, std::memory_order_release);
#endif
}

bool ServiceRunner::autoRunActive() const
{
    return _auto_running.load(std::memory_order_acquire);
}

void ServiceRunner::flushPending()
{
    // Consume only the notification generation that this scan owns. A
    // producer publishing after the exchange leaves true behind for the next
    // pass; a load followed by store(false) could erase that newer publish
    // and strand a synchronous add/remove waiter indefinitely.
    if (!_has_pending.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    for (size_t i = 0; i < kMaxPendingRemoves; ++i) {
        // acq_rel: the release side lets a remove() waiter (spinning on this
        // slot with an acquire load) observe consumption. applyRemove() runs
        // immediately after, before the poll loop below, so the removal is in
        // effect by the time this pass polls -- see remove().
        IService* s = _pending_remove[i].exchange(nullptr, std::memory_order_acq_rel);
        if (s != nullptr) {
            applyRemove(s);
        }
    }
    IService* s = _pending_add.load(std::memory_order_relaxed);
    if (s != nullptr) {
        auto added = applyAdd(s);
        _pending_add_outcome.store(
            added ? PendingAddOutcome::Success
                  : (added.error() == error::error_t::INVALID_STATE ? PendingAddOutcome::InvalidState
                                                                    : PendingAddOutcome::OutOfResource),
            std::memory_order_relaxed);
        _pending_add.store(nullptr, std::memory_order_release);
    }
}

size_t ServiceRunner::findInTable(const IService* service) const
{
    const size_t n = _count.load(std::memory_order_relaxed);
    for (size_t i = 0; i < n; ++i) {
        if (_services[i] == service) {
            return i;
        }
    }
    return kMaxServices;
}

result_t<void> ServiceRunner::applyAdd(IService* service)
{
    const size_t n = _count.load(std::memory_order_relaxed);
    for (size_t i = 0; i < n; ++i) {
        if (_services[i] == service) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
    }
    if (n < kMaxServices) {
        _services[n] = service;
        _next_due[n] = 0;
        // Baseline at the current virtual position: the first poll reports
        // elapsed 0 (gap-drop; the service's pre-add past is not this
        // stream's to vouch for). This also makes clear()'s choice to keep
        // _virtual_now safe -- any starting value works as a baseline.
        _last_polled[n] = _virtual_now;
        _count.store(n + 1, std::memory_order_relaxed);
        return {};
    }
    return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
}

void ServiceRunner::applyRemove(IService* service)
{
    const size_t n = _count.load(std::memory_order_relaxed);
    for (size_t i = 0; i < n; ++i) {
        if (_services[i] == service) {
            // _last_polled shifts IN LOCKSTEP with _services/_next_due: a
            // stale baseline left behind would hand the service that slides
            // into this slot someone else's previous-poll position (huge
            // elapsed / early timeouts).
            for (size_t j = i + 1; j < n; ++j) {
                _services[j - 1]    = _services[j];
                _next_due[j - 1]    = _next_due[j];
                _last_polled[j - 1] = _last_polled[j];
            }
            const size_t m  = n - 1;
            _services[m]    = nullptr;
            _next_due[m]    = 0;
            _last_polled[m] = 0;
            _count.store(m, std::memory_order_relaxed);
            if (_iter_index != kMaxServices && i <= _iter_index) {
                --_iter_index;
            }
            return;
        }
    }
}

void ServiceRunner::autoRunEntry(void* arg)
{
    auto* runner = static_cast<ServiceRunner*>(arg);
    // Task creation may schedule this entry before Task::start() returns to
    // startAutoRunLocked(). Block until the creator publishes _auto_running;
    // otherwise the first callback could see an existing auto-run task as
    // inactive. The predicate loop tolerates a stale latched wake from an
    // earlier stop/restart cycle.
    while (!runner->_auto_running.load(std::memory_order_acquire)) {
        if (runner->_auto_stop.load(std::memory_order_acquire)) {
            return;
        }
        auto waited = runner->_wake.wait(kIdleWakeTimeoutMs);
        if (!waited.has_value() && waited.error() != error::error_t::TIMEOUT_ERROR) {
            ++runner->_wake_errors;
            M5HAL_DIAG("startup wake error #%u: %s", runner->_wake_errors, error::toString(waited.error()));
            runtime::delayMs(1);
        }
    }
    runner->autoRunLoop();
}

bool ServiceRunner::isCurrentWriter() const
{
    void* w = _writer_id.load(std::memory_order_acquire);
    return w != nullptr && w == runtime::currentTaskId();
}

void ServiceRunner::cancelPendingAdd(IService* service)
{
    if (_pending_add.load(std::memory_order_relaxed) == service) {
        IService* expected = service;
        _pending_add_outcome.store(PendingAddOutcome::InvalidState, std::memory_order_relaxed);
        (void)_pending_add.compare_exchange_strong(expected, nullptr, std::memory_order_acq_rel,
                                                   std::memory_order_relaxed);
    }
}

bool ServiceRunner::runOnceInternal(fast_tick_t elapsed_pass, fast_tick_t local_tick, bool refresh_local_each_poll)
{
    // Mark this thread as the table owner for the duration of the pass, so a
    // service re-entering add()/remove() from inside its serviceImpl() applies
    // directly instead of re-taking the (non-recursive) control mutex -- see
    // isCurrentWriter(). Runs on either the auto-run task or a runOnce()
    // caller; never nested (auto-run and runOnce are mutually exclusive, and a
    // re-entrant runOnce try-locks out).
    _writer_id.store(runtime::currentTaskId(), std::memory_order_release);
    // RAII so a serviceImpl() exception (host builds) cannot leave a stale
    // writer id / mid-pass cursor behind.
    struct PassEnd {
        ServiceRunner* r;
        ~PassEnd()
        {
            r->_iter_index = kMaxServices;
            r->_writer_id.store(nullptr, std::memory_order_release);
        }
    } pass_end{this};
    // Advance the virtual timeline BEFORE flushing pending adds: applyAdd
    // baselines _last_polled at _virtual_now, and a service added this
    // pass must start from elapsed 0 (its pre-add past is nobody's to
    // vouch for), not inherit this pass's elapsed.
    _virtual_now += elapsed_pass;
    flushPending();
    bool progressed = false;
    _iter_index     = 0;
    while (_iter_index < _count.load(std::memory_order_relaxed)) {
        IService* s = _services[_iter_index];
        if (s == nullptr || (_next_due[_iter_index] != 0 && !hasReached(_virtual_now, _next_due[_iter_index]))) {
            ++_iter_index;
            continue;
        }
        // Call-local spin anchor: on default-clock passes re-read the live
        // counter for each service (earlier services consumed time, and an
        // unpinned task may have migrated between polls); explicit passes
        // keep the caller's anchor for determinism.
        const fast_tick_t local = refresh_local_each_poll ? fastTick() : local_tick;
        const ServiceContext ctx{static_cast<fast_tick_t>(_virtual_now - _last_polled[_iter_index]), local};
        const auto r          = run(*s, ctx);
        const auto post_index = findInTable(s);
        if (post_index != kMaxServices) {
            _last_polled[post_index] = _virtual_now;
            _next_due[post_index]    = r.next_due_delta ? static_cast<fast_tick_t>(_virtual_now + r.next_due_delta) : 0;
        }
        progressed = progressed || r == ServiceResult::Progress || r == ServiceResult::Done;
        ++_iter_index;
    }
    return progressed;  // pass_end resets _iter_index and _writer_id
}

void ServiceRunner::autoRunLoop()
{
    while (!_auto_stop.load(std::memory_order_acquire)) {
        // This task is the committed pass owner while _auto_running is up,
        // so stepping the stream here cannot race a manual runOnce()
        // (which backs off while auto-run is active).
        const TickSample s = sampleTickWithDomain();
        if (runOnceInternal(_stream.step(s.tick, s.domain), s.tick, /*refresh_local_each_poll=*/true)) {
            continue;
        }
        if (_count.load(std::memory_order_relaxed) == 0) {
            // Idle: block on the wake event instead of a tick-quantized
            // sleep (a FreeRTOS delayMs(1) rounds up to 10-20 ms of real
            // time, which stalled the first poll after an add() past
            // short wire deadlines). add/remove/stop notify _wake; the
            // event is latching, so a notify that lands between the
            // count check above and the wait below is consumed
            // immediately instead of being lost. The timeout is only a
            // liveness backstop — waking through it means a notification
            // was missed, so count those for diagnosis.
            auto waited = _wake.wait(kIdleWakeTimeoutMs);
            if (!waited.has_value()) {
                if (waited.error() == error::error_t::TIMEOUT_ERROR) {
                    ++_idle_wake_timeouts;
                    M5HAL_DIAG("idle wake timeout #%u", _idle_wake_timeouts);
                } else {
                    ++_wake_errors;
                    M5HAL_DIAG("idle wake error #%u: %s", _wake_errors, error::toString(waited.error()));
                    runtime::delayMs(1);
                }
            }
        } else {
            runtime::yield();
        }
    }
    // _auto_running is cleared by stopAutoRunLocked() after join(), under
    // _control, so every _auto_running transition stays within the control
    // plane. The task deliberately does not clear it here: that write would be
    // a transition outside _control, and a restart racing the join could see
    // the flag flap.
}

}  // namespace m5::hal::v2::service

#ifdef M5HAL_SERVICE_INL_HAS_ESP_CPU_H_
#undef M5HAL_SERVICE_INL_HAS_ESP_CPU_H_
#endif
#ifdef M5HAL_SERVICE_INL_HAS_ESP_CLK_CPU_FREQ_
#undef M5HAL_SERVICE_INL_HAS_ESP_CLK_CPU_FREQ_
#endif

#endif  // M5_HAL_HAL_V2_SERVICE_SERVICE_INL_
