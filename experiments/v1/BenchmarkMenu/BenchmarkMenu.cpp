// SPDX-License-Identifier: MIT
// M5HAL v1 benchmark menu for M5Stack BASIC.
//
// This is an experiment harness, not a beginner example. It keeps the GUI
// entry count small while allowing repeated target-side measurements from the
// front buttons.

#include <Arduino.h>
#include <M5HAL_v1.hpp>
#include <M5Utility.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

#if defined(ESP_PLATFORM)
#include <esp_cpu.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#endif

namespace {

namespace m5hal = ::m5::hal::v1;
namespace mem   = ::m5::hal::v1::memory;

constexpr int PIN_BTN_A     = 39;
constexpr int PIN_BTN_B     = 38;
constexpr int PIN_BTN_C     = 37;
constexpr int PIN_BACKLIGHT = 32;

constexpr uint32_t TIMER_ITERATIONS   = 100000;
constexpr uint32_t MEMORY_ITERATIONS  = 20000;
constexpr uint32_t REALLOC_ITERATIONS = 10000;
constexpr size_t MAX_TRACKED_PTRS     = mem::Allocator::tempBlockCount();

volatile uintptr_t g_sink = 0;

struct ButtonState {
    int pin;
    bool pressed = false;
    bool event   = false;

    void begin() const
    {
        pinMode(pin, INPUT);
    }

    void update()
    {
        const bool now_pressed = (digitalRead(pin) == LOW);
        event                  = now_pressed && !pressed;
        pressed                = now_pressed;
    }
};

ButtonState button_a{PIN_BTN_A};
ButtonState button_b{PIN_BTN_B};
ButtonState button_c{PIN_BTN_C};

uint32_t readCycles()
{
#if defined(ESP_PLATFORM)
    return esp_cpu_get_ccount();
#else
    return static_cast<uint32_t>(m5::utility::micros());
#endif
}

uint32_t cpuMhz()
{
#if defined(ESP_PLATFORM)
    return static_cast<uint32_t>(ESP.getCpuFreqMHz());
#else
    return 0;
#endif
}

struct BenchResult {
    uint32_t cycles     = 0;
    uint32_t usec       = 0;
    uint32_t iterations = 0;
};

template <typename Func>
BenchResult measure(uint32_t iterations, Func func)
{
    for (uint32_t i = 0; i < 64; ++i) {
        func(i);
    }

    const uint32_t start_cycles = readCycles();
    const uint32_t start_usec   = static_cast<uint32_t>(m5::utility::micros());
    for (uint32_t i = 0; i < iterations; ++i) {
        func(i);
    }
    const uint32_t end_usec   = static_cast<uint32_t>(m5::utility::micros());
    const uint32_t end_cycles = readCycles();

    return BenchResult{end_cycles - start_cycles, end_usec - start_usec, iterations};
}

BenchResult measureEmpty(uint32_t iterations)
{
    uintptr_t acc = 0;
    auto result   = measure(iterations, [&](uint32_t i) { acc += i; });
    g_sink += acc;
    return result;
}

uint32_t cyclesPerCall(const BenchResult& result)
{
    return result.iterations == 0 ? 0 : result.cycles / result.iterations;
}

uint32_t nsecPerCall(const BenchResult& result)
{
    if (result.iterations == 0) {
        return 0;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(result.usec) * 1000u) / result.iterations);
}

void printBench(const char* label, const BenchResult& result, const BenchResult& empty, uint32_t ops_per_iteration = 1)
{
    const uint32_t raw_cycles   = cyclesPerCall(result);
    const uint32_t empty_cycles = cyclesPerCall(empty);
    const uint32_t net_cycles   = (raw_cycles > empty_cycles) ? (raw_cycles - empty_cycles) : 0;
    const uint32_t net_per_op   = ops_per_iteration == 0 ? net_cycles : net_cycles / ops_per_iteration;
    const uint32_t nsec         = nsecPerCall(result);
    const uint32_t nsec_per_op  = ops_per_iteration == 0 ? nsec : nsec / ops_per_iteration;

    Serial.printf("%-28s total=%lu us cycles/iter=%lu net_cycles/op=%lu ns/op=%lu\n", label,
                  static_cast<unsigned long>(result.usec), static_cast<unsigned long>(raw_cycles),
                  static_cast<unsigned long>(net_per_op), static_cast<unsigned long>(nsec_per_op));
}

void printMemoryState(const char* label)
{
    auto& alloc = mem::defaultAllocator();
    Serial.printf("%s used=%u/%u largest_run=%u block=%uB pool=%uB\n", label, static_cast<unsigned>(alloc.usedBlocks()),
                  static_cast<unsigned>(alloc.tempBlockCount()), static_cast<unsigned>(alloc.largestFreeRun()),
                  static_cast<unsigned>(alloc.tempBlockSize()), static_cast<unsigned>(alloc.tempPoolSize()));
}

void* fallbackMalloc(size_t size, mem::usage_t)
{
    return std::malloc(size);
}

void* fallbackRealloc(void* ptr, size_t, size_t new_size, mem::usage_t)
{
    return std::realloc(ptr, new_size);
}

void fallbackFree(void* ptr)
{
    std::free(ptr);
}

void runTimerBenchmark()
{
    Serial.println();
    Serial.println("[Timer benchmark]");
    Serial.printf("CPU MHz: %lu\n", static_cast<unsigned long>(cpuMhz()));
    Serial.printf("M5HAL fastTick Hz: %lu\n", static_cast<unsigned long>(m5hal::service::fastTickFrequencyHz()));
    Serial.printf("iterations: %lu\n", static_cast<unsigned long>(TIMER_ITERATIONS));

    const auto empty = measureEmpty(TIMER_ITERATIONS);
    printBench("empty loop", empty, empty);

    printBench("Arduino micros()", measure(TIMER_ITERATIONS, [](uint32_t) { g_sink += ::micros(); }), empty);
    printBench("M5Utility micros()", measure(TIMER_ITERATIONS, [](uint32_t) { g_sink += m5::utility::micros(); }),
               empty);
    printBench("M5HAL fastTick()", measure(TIMER_ITERATIONS, [](uint32_t) { g_sink += m5hal::service::fastTick(); }),
               empty);
    printBench("M5HAL fastTickNsec()",
               measure(TIMER_ITERATIONS, [](uint32_t) { g_sink += m5hal::service::fastTickNsec(); }), empty);
    printBench("M5HAL defaultNowNsec()",
               measure(TIMER_ITERATIONS, [](uint32_t) { g_sink += m5hal::service::defaultNowNsec(); }), empty);
    Serial.printf("sink=%lu\n", static_cast<unsigned long>(g_sink));
}

void runLockBenchmark()
{
    Serial.println();
    Serial.println("[Lock benchmark]");
    Serial.printf("CPU MHz: %lu\n", static_cast<unsigned long>(cpuMhz()));
    Serial.printf("iterations: %lu\n", static_cast<unsigned long>(TIMER_ITERATIONS));

    const auto empty = measureEmpty(TIMER_ITERATIONS);
    printBench("empty loop", empty, empty);

    printBench("no-op pair", measure(TIMER_ITERATIONS, [](uint32_t i) { g_sink += i; }), empty, 2);

#if defined(ESP_PLATFORM)
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    printBench("portMUX critical",
               measure(TIMER_ITERATIONS,
                       [](uint32_t i) {
                           taskENTER_CRITICAL(&mux);
                           g_sink += i;
                           taskEXIT_CRITICAL(&mux);
                       }),
               empty, 2);

    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    if (mutex != nullptr) {
        printBench("FreeRTOS mutex",
                   measure(TIMER_ITERATIONS,
                           [&](uint32_t i) {
                               xSemaphoreTake(mutex, portMAX_DELAY);
                               g_sink += i;
                               xSemaphoreGive(mutex);
                           }),
                   empty, 2);
        vSemaphoreDelete(mutex);
    } else {
        Serial.println("FreeRTOS mutex: create failed");
    }

    printBench("scheduler suspend",
               measure(TIMER_ITERATIONS,
                       [](uint32_t i) {
                           vTaskSuspendAll();
                           g_sink += i;
                           xTaskResumeAll();
                       }),
               empty, 2);
#endif

    std::atomic_flag atomic_lock = ATOMIC_FLAG_INIT;
    printBench("atomic_flag spin",
               measure(TIMER_ITERATIONS,
                       [&](uint32_t i) {
                           while (atomic_lock.test_and_set(std::memory_order_acquire)) {
                           }
                           g_sink += i;
                           atomic_lock.clear(std::memory_order_release);
                       }),
               empty, 2);

    Serial.printf("sink=%lu\n", static_cast<unsigned long>(g_sink));
}

void runMemorySizeSweep()
{
    static const size_t sizes[] = {1, 16, 32, 33, 64, 128, 256, 512, 1024, 2048, 4096};
    auto& alloc                 = mem::defaultAllocator();

    Serial.println();
    Serial.println("[Memory pool size sweep]");
    Serial.printf("iterations: %lu\n", static_cast<unsigned long>(MEMORY_ITERATIONS));
    printMemoryState("before");

    const auto empty = measureEmpty(MEMORY_ITERATIONS);
    printBench("empty loop", empty, empty);

    for (size_t size : sizes) {
        char label[32];
        snprintf(label, sizeof(label), "alloc/free %uB", static_cast<unsigned>(size));
        const auto result = measure(MEMORY_ITERATIONS, [&](uint32_t i) {
            void* p = alloc.allocate(size, mem::usage_t::temp);
            g_sink += reinterpret_cast<uintptr_t>(p) + i;
            alloc.deallocate(p);
        });
        printBench(label, result, empty, 2);
    }

    printMemoryState("after");
    Serial.printf("sink=%lu\n", static_cast<unsigned long>(g_sink));
}

void runTempBufferBenchmark()
{
    static const size_t sizes[] = {32, 128, 512, 1024, 2048, 4096};
    auto& alloc                 = mem::defaultAllocator();

    Serial.println();
    Serial.println("[TempBuffer benchmark]");
    Serial.printf("iterations: %lu\n", static_cast<unsigned long>(MEMORY_ITERATIONS));
    printMemoryState("before");

    const auto empty = measureEmpty(MEMORY_ITERATIONS);
    printBench("empty loop", empty, empty);

    for (size_t size : sizes) {
        char label[32];
        snprintf(label, sizeof(label), "TempBuffer %uB", static_cast<unsigned>(size));
        const auto result = measure(MEMORY_ITERATIONS, [&](uint32_t i) {
            mem::TempBuffer buffer{alloc, size};
            g_sink += reinterpret_cast<uintptr_t>(buffer.data()) + buffer.size() + i;
        });
        printBench(label, result, empty, 2);
    }

    printMemoryState("after");
    Serial.printf("sink=%lu\n", static_cast<unsigned long>(g_sink));
}

void runMallocComparison()
{
    static const size_t sizes[] = {32, 128, 512, 1024, 2048, 4096};
    auto& alloc                 = mem::defaultAllocator();

    Serial.println();
    Serial.println("[malloc comparison]");
    Serial.printf("iterations: %lu\n", static_cast<unsigned long>(MEMORY_ITERATIONS));
    printMemoryState("before");

    const auto empty = measureEmpty(MEMORY_ITERATIONS);
    printBench("empty loop", empty, empty);

    for (size_t size : sizes) {
        char label[32];

        snprintf(label, sizeof(label), "pool temp %uB", static_cast<unsigned>(size));
        auto pool_result = measure(MEMORY_ITERATIONS, [&](uint32_t i) {
            void* p = alloc.allocate(size, mem::usage_t::temp);
            g_sink += reinterpret_cast<uintptr_t>(p) + i;
            alloc.deallocate(p);
        });
        printBench(label, pool_result, empty, 2);

        snprintf(label, sizeof(label), "std malloc %uB", static_cast<unsigned>(size));
        auto malloc_result = measure(MEMORY_ITERATIONS, [&](uint32_t i) {
            void* p = std::malloc(size);
            g_sink += reinterpret_cast<uintptr_t>(p) + i;
            std::free(p);
        });
        printBench(label, malloc_result, empty, 2);
    }

    printMemoryState("after");
    Serial.printf("sink=%lu\n", static_cast<unsigned long>(g_sink));
}

void runReallocateBenchmark()
{
    auto& alloc                  = mem::defaultAllocator();
    constexpr size_t block       = mem::Allocator::tempBlockSize();
    constexpr size_t pool_size   = mem::Allocator::tempPoolSize();
    constexpr size_t small_size  = block;
    constexpr size_t medium_size = block * 2u;

    Serial.println();
    Serial.println("[Memory reallocate benchmark]");
    Serial.printf("iterations: %lu\n", static_cast<unsigned long>(REALLOC_ITERATIONS));
    printMemoryState("before");

    const auto empty = measureEmpty(REALLOC_ITERATIONS);
    printBench("empty loop", empty, empty);

    auto inplace_result = measure(REALLOC_ITERATIONS, [&](uint32_t i) {
        void* p = alloc.allocate(small_size, mem::usage_t::temp);
        p       = alloc.reallocate(p, small_size, medium_size, mem::usage_t::temp);
        g_sink += reinterpret_cast<uintptr_t>(p) + i;
        alloc.deallocate(p);
    });
    printBench("pool inplace grow", inplace_result, empty, 3);

    auto move_result = measure(REALLOC_ITERATIONS, [&](uint32_t i) {
        void* p       = alloc.allocate(small_size, mem::usage_t::temp);
        void* tail    = alloc.allocate(small_size, mem::usage_t::temp);
        void* blocker = alloc.allocate(small_size, mem::usage_t::temp);
        p             = alloc.reallocate(p, small_size, medium_size, mem::usage_t::temp);
        g_sink += reinterpret_cast<uintptr_t>(p) + reinterpret_cast<uintptr_t>(tail) +
                  reinterpret_cast<uintptr_t>(blocker) + i;
        alloc.deallocate(p);
        alloc.deallocate(blocker);
        alloc.deallocate(tail);
    });
    printBench("pool move grow", move_result, empty, 7);

    auto drop_result = measure(REALLOC_ITERATIONS, [&](uint32_t i) {
        void* p = alloc.allocate(small_size, mem::usage_t::temp);
        p       = alloc.reallocate(p, 0, medium_size, mem::usage_t::temp);
        g_sink += reinterpret_cast<uintptr_t>(p) + i;
        alloc.deallocate(p);
    });
    printBench("pool grow preserve 0", drop_result, empty, 3);

    auto pool_to_fallback_result = measure(REALLOC_ITERATIONS, [&](uint32_t i) {
        void* p = alloc.allocate(small_size, mem::usage_t::temp);
        p       = alloc.reallocate(p, 0, pool_size + block, mem::usage_t::temp);
        g_sink += reinterpret_cast<uintptr_t>(p) + i;
        alloc.deallocate(p);
    });
    printBench("pool to fallback", pool_to_fallback_result, empty, 3);

    alloc.setFallback(&fallbackMalloc, &fallbackRealloc, &fallbackFree);
    auto fallback_realloc_result = measure(REALLOC_ITERATIONS, [&](uint32_t i) {
        void* p = alloc.allocate(small_size, mem::usage_t::persistent);
        p       = alloc.reallocate(p, small_size, medium_size, mem::usage_t::persistent);
        g_sink += reinterpret_cast<uintptr_t>(p) + i;
        alloc.deallocate(p);
    });
    alloc.setFallback(nullptr, nullptr);
    printBench("fallback realloc hook", fallback_realloc_result, empty, 3);

    auto std_realloc_result = measure(REALLOC_ITERATIONS, [&](uint32_t i) {
        void* p = std::malloc(small_size);
        p       = std::realloc(p, medium_size);
        g_sink += reinterpret_cast<uintptr_t>(p) + i;
        std::free(p);
    });
    printBench("std realloc", std_realloc_result, empty, 3);

    printMemoryState("after");
    Serial.printf("sink=%lu\n", static_cast<unsigned long>(g_sink));
}

void runFragmentationBenchmark()
{
    auto& alloc = mem::defaultAllocator();
    void* ptrs[MAX_TRACKED_PTRS]{};

    Serial.println();
    Serial.println("[Memory fragmentation]");
    printMemoryState("initial");

    size_t allocated = 0;
    for (; allocated < MAX_TRACKED_PTRS; ++allocated) {
        ptrs[allocated] = alloc.allocate(mem::Allocator::tempBlockSize(), mem::usage_t::temp);
        if (ptrs[allocated] == nullptr) {
            break;
        }
    }
    Serial.printf("allocated %u one-block buffers\n", static_cast<unsigned>(allocated));
    printMemoryState("full");

    for (size_t i = 0; i < allocated; i += 2) {
        alloc.deallocate(ptrs[i]);
        ptrs[i] = nullptr;
    }
    printMemoryState("after freeing every other allocation");

    void* large = alloc.allocate(mem::Allocator::tempBlockSize() * 8, mem::usage_t::temp);
    Serial.printf("8-block temp allocation after fragmentation: %s\n", large != nullptr ? "OK" : "fallback/null");
    printMemoryState("after large allocation");
    alloc.deallocate(large);

    for (size_t i = 0; i < allocated; ++i) {
        alloc.deallocate(ptrs[i]);
        ptrs[i] = nullptr;
    }
    printMemoryState("final");
}

void runPoolExhaustionBenchmark()
{
    auto& alloc = mem::defaultAllocator();
    void* ptrs[MAX_TRACKED_PTRS]{};

    Serial.println();
    Serial.println("[Pool exhaustion / fallback]");
    printMemoryState("initial");

    size_t allocated = 0;
    for (; allocated < MAX_TRACKED_PTRS; ++allocated) {
        ptrs[allocated] = alloc.allocate(mem::Allocator::tempBlockSize(), mem::usage_t::temp);
        if (ptrs[allocated] == nullptr) {
            break;
        }
    }
    printMemoryState("pool filled");

    const auto empty = measureEmpty(MEMORY_ITERATIONS);
    printBench("empty loop", empty, empty);
    const auto fallback_result = measure(MEMORY_ITERATIONS, [&](uint32_t i) {
        void* p = alloc.allocate(mem::Allocator::tempBlockSize(), mem::usage_t::temp);
        g_sink += reinterpret_cast<uintptr_t>(p) + i;
        alloc.deallocate(p);
    });
    printBench("temp fallback alloc/free", fallback_result, empty, 2);

    for (size_t i = 0; i < allocated; ++i) {
        alloc.deallocate(ptrs[i]);
        ptrs[i] = nullptr;
    }
    printMemoryState("final");
    Serial.printf("sink=%lu\n", static_cast<unsigned long>(g_sink));
}

void runMemorySnapshot()
{
    Serial.println();
    Serial.println("[Memory snapshot]");
    printMemoryState("M5_Hal.Memory");
    Serial.printf("heap free=%lu min_free=%lu max_alloc=%lu\n", static_cast<unsigned long>(ESP.getFreeHeap()),
                  static_cast<unsigned long>(ESP.getMinFreeHeap()), static_cast<unsigned long>(ESP.getMaxAllocHeap()));
#if defined(BOARD_HAS_PSRAM)
    Serial.printf("psram free=%lu\n", static_cast<unsigned long>(ESP.getFreePsram()));
#endif
}

struct Operation {
    const char* name;
    void (*run)();
};

Operation operations[] = {
    {"Memory snapshot", runMemorySnapshot},
    {"Timer benchmark", runTimerBenchmark},
    {"Lock benchmark", runLockBenchmark},
    {"Memory size sweep", runMemorySizeSweep},
    {"TempBuffer benchmark", runTempBufferBenchmark},
    {"malloc comparison", runMallocComparison},
    {"reallocate benchmark", runReallocateBenchmark},
    {"Fragmentation", runFragmentationBenchmark},
    {"Pool exhaustion", runPoolExhaustionBenchmark},
};

constexpr size_t OPERATION_COUNT = sizeof(operations) / sizeof(operations[0]);
size_t current_operation         = 0;

void printMenu()
{
    Serial.println();
    Serial.println("M5HAL BenchmarkMenu - M5Stack BASIC");
    Serial.println("BtnA: previous  BtnC: next  BtnB: run");
    for (size_t i = 0; i < OPERATION_COUNT; ++i) {
        Serial.printf("  %c %u. %s\n", (i == current_operation) ? '>' : ' ', static_cast<unsigned>(i + 1),
                      operations[i].name);
    }
}

void selectOperation(int delta)
{
    const auto next   = static_cast<int>(current_operation) + delta + static_cast<int>(OPERATION_COUNT);
    current_operation = static_cast<size_t>(next) % OPERATION_COUNT;
    printMenu();
}

void updateButtons()
{
    button_a.update();
    button_b.update();
    button_c.update();
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_BACKLIGHT, OUTPUT);
    digitalWrite(PIN_BACKLIGHT, HIGH);
    button_a.begin();
    button_b.begin();
    button_c.begin();

    printMenu();
}

void loop()
{
    updateButtons();
    if (button_a.event) {
        selectOperation(-1);
    }
    if (button_c.event) {
        selectOperation(1);
    }
    if (button_b.event) {
        Serial.printf("\nRunning: %s\n", operations[current_operation].name);
        operations[current_operation].run();
        Serial.println("Done.");
        printMenu();
    }
    delay(10);
}
