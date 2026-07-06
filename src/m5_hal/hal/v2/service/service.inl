// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_SERVICE_SERVICE_INL_
#define M5_HAL_HAL_V2_SERVICE_SERVICE_INL_

#include "service.hpp"

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

ServicePoll IService::service(const ServiceContext& ctx)
{
    return serviceImpl(ctx);
}

ServiceRunner::~ServiceRunner()
{
    stopAutoRun();
}

bool ServiceRunner::add(IService& service)
{
    if (!_auto_running.load(std::memory_order_acquire)) {
        if (findInTable(&service) != kMaxServices || _count >= kMaxServices) {
            return false;
        }
        applyAdd(&service);
        (void)startAutoRun();
        return true;
    }
    for (size_t i = 0; i < kMaxPending; ++i) {
        IService* expected = nullptr;
        if (_pending_add[i].compare_exchange_strong(expected, &service, std::memory_order_relaxed,
                                                    std::memory_order_relaxed)) {
            _has_pending.store(true, std::memory_order_release);
            return true;
        }
        if (expected == &service) {
            return true;
        }
    }
    return false;
}

bool ServiceRunner::remove(IService& service)
{
    if (!_auto_running.load(std::memory_order_acquire)) {
        if (findInTable(&service) == kMaxServices) {
            return false;
        }
        applyRemove(&service);
        return true;
    }
    for (size_t i = 0; i < kMaxPending; ++i) {
        IService* expected = nullptr;
        if (_pending_remove[i].compare_exchange_strong(expected, &service, std::memory_order_relaxed,
                                                       std::memory_order_relaxed)) {
            _has_pending.store(true, std::memory_order_release);
            return true;
        }
        if (expected == &service) {
            return true;
        }
    }
    return false;
}

void ServiceRunner::clear()
{
    for (size_t i = 0; i < kMaxPending; ++i) {
        _pending_add[i].store(nullptr, std::memory_order_relaxed);
        _pending_remove[i].store(nullptr, std::memory_order_relaxed);
    }
    for (size_t i = 0; i < _count; ++i) {
        _services[i] = nullptr;
        _next_due[i] = 0;
    }
    _count = 0;
}

ServicePoll ServiceRunner::run(IService& service, const ServiceContext& ctx)
{
    return service.service(ctx);
}

ServicePoll ServiceRunner::run(IService& service, fast_tick_t now_tick)
{
    return run(service, ServiceContext{now_tick});
}

bool ServiceRunner::runOnce(const ServiceContext& ctx)
{
    if (autoRunActive()) {
        return false;
    }
    return runOnceInternal(ctx);
}

bool ServiceRunner::runOnce(fast_tick_t now_tick)
{
    return runOnce(ServiceContext{now_tick});
}

bool ServiceRunner::runOnce()
{
    return runOnce(defaultNowTick());
}

size_t ServiceRunner::size() const
{
    return _count;
}

size_t ServiceRunner::capacity() const
{
    return kMaxServices;
}

bool ServiceRunner::startAutoRun()
{
#if defined(ESP_PLATFORM) || defined(ARDUINO)
    if (_auto_running.load(std::memory_order_acquire)) {
        return true;
    }
    _auto_stop.store(false, std::memory_order_release);
    if (!_auto_task.start(&ServiceRunner::autoRunEntry, this, "m5hal-svc", 4096, 1)) {
        _auto_stop.store(true, std::memory_order_release);
        return false;
    }
    _auto_running.store(true, std::memory_order_release);
    return true;
#else
    return false;
#endif
}

void ServiceRunner::stopAutoRun()
{
#if defined(ESP_PLATFORM) || defined(ARDUINO)
    _auto_stop.store(true, std::memory_order_release);
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
    if (!_has_pending.load(std::memory_order_acquire)) {
        return;
    }
    _has_pending.store(false, std::memory_order_relaxed);
    for (size_t i = 0; i < kMaxPending; ++i) {
        IService* s = _pending_remove[i].exchange(nullptr, std::memory_order_relaxed);
        if (s != nullptr) {
            applyRemove(s);
        }
    }
    for (size_t i = 0; i < kMaxPending; ++i) {
        IService* s = _pending_add[i].exchange(nullptr, std::memory_order_relaxed);
        if (s != nullptr) {
            applyAdd(s);
        }
    }
}

size_t ServiceRunner::findInTable(const IService* service) const
{
    for (size_t i = 0; i < _count; ++i) {
        if (_services[i] == service) {
            return i;
        }
    }
    return kMaxServices;
}

void ServiceRunner::applyAdd(IService* service)
{
    for (size_t i = 0; i < _count; ++i) {
        if (_services[i] == service) {
            return;
        }
    }
    if (_count < kMaxServices) {
        _services[_count] = service;
        _next_due[_count] = 0;
        ++_count;
    }
}

void ServiceRunner::applyRemove(IService* service)
{
    for (size_t i = 0; i < _count; ++i) {
        if (_services[i] == service) {
            for (size_t j = i + 1; j < _count; ++j) {
                _services[j - 1] = _services[j];
                _next_due[j - 1] = _next_due[j];
            }
            --_count;
            _services[_count] = nullptr;
            _next_due[_count] = 0;
            if (_iter_index != kMaxServices && i <= _iter_index) {
                --_iter_index;
            }
            return;
        }
    }
}

void ServiceRunner::autoRunEntry(void* arg)
{
    static_cast<ServiceRunner*>(arg)->autoRunLoop();
}

bool ServiceRunner::runOnceInternal(const ServiceContext& ctx)
{
    flushPending();
    bool progressed = false;
    _iter_index     = 0;
    while (_iter_index < _count) {
        IService* s = _services[_iter_index];
        if (s == nullptr || (_next_due[_iter_index] != 0 && !hasReached(ctx.now_tick, _next_due[_iter_index]))) {
            ++_iter_index;
            continue;
        }
        const auto r          = run(*s, ctx);
        const auto post_index = findInTable(s);
        if (post_index != kMaxServices) {
            _next_due[post_index] = r.next_due;
        }
        progressed = progressed || r == ServiceResult::Progress || r == ServiceResult::Done;
        ++_iter_index;
    }
    _iter_index = kMaxServices;
    return progressed;
}

void ServiceRunner::autoRunLoop()
{
    while (!_auto_stop.load(std::memory_order_acquire)) {
        if (runOnceInternal(ServiceContext{defaultNowTick()})) {
            continue;
        }
        if (_count == 0) {
            runtime::delayMs(1);
        } else {
            runtime::yield();
        }
    }
    _auto_running.store(false, std::memory_order_release);
}

}  // namespace m5::hal::v2::service

#ifdef M5HAL_SERVICE_INL_HAS_ESP_CPU_H_
#undef M5HAL_SERVICE_INL_HAS_ESP_CPU_H_
#endif
#ifdef M5HAL_SERVICE_INL_HAS_ESP_CLK_CPU_FREQ_
#undef M5HAL_SERVICE_INL_HAS_ESP_CLK_CPU_FREQ_
#endif

#endif  // M5_HAL_HAL_V2_SERVICE_SERVICE_INL_
