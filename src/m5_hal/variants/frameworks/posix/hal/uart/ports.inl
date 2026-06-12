// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_PORTS_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_PORTS_INL

#include "./ports.hpp"

#if M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

#include <dirent.h>
#include <stdio.h>
#include <string.h>

namespace m5::variants::frameworks::posix::hal::v1::uart {

namespace {

bool hasPrefix(const char* name, const char* prefix)
{
    return ::strncmp(name, prefix, ::strlen(prefix)) == 0;
}

// Insert keeping (rank asc, name asc); drops the weakest tail entry
// when the array is full. `count` is the current fill level.
void insertSorted(SerialPortInfo* out, size_t capacity, size_t& count, const SerialPortInfo& entry)
{
    size_t pos = count;
    while (pos > 0 && (entry.rank < out[pos - 1].rank ||
                       (entry.rank == out[pos - 1].rank && ::strcmp(entry.path, out[pos - 1].path) < 0))) {
        --pos;
    }
    if (pos >= capacity) {
        return;  // weaker than everything kept
    }
    const size_t last = (count < capacity) ? count : capacity - 1;
    for (size_t i = last; i > pos; --i) {
        out[i] = out[i - 1];
    }
    out[pos] = entry;
    if (count < capacity) {
        ++count;
    }
}

// Scan one directory, rank entries via `rank_fn`, and merge them in.
template <typename RankFn>
void scanDirectory(const char* dir_path, SerialPortInfo* out, size_t capacity, size_t& count, RankFn rank_fn)
{
    DIR* dir = ::opendir(dir_path);
    if (dir == nullptr) {
        return;
    }
    while (auto* entry = ::readdir(dir)) {
        const int rank = rank_fn(entry->d_name);
        if (rank < 0) {
            continue;
        }
        SerialPortInfo info;
        const int written = ::snprintf(info.path, sizeof(info.path), "%s/%s", dir_path, entry->d_name);
        if (written <= 0 || static_cast<size_t>(written) >= sizeof(info.path)) {
            continue;  // path does not fit; skip rather than truncate
        }
        info.rank = static_cast<uint8_t>(rank);
        insertSorted(out, capacity, count, info);
    }
    ::closedir(dir);
}

}  // namespace

int rankSerialPortName(const char* dev_name)
{
    if (dev_name == nullptr) {
        return -1;
    }
#if defined(__APPLE__)
    if (hasPrefix(dev_name, "cu.")) {
        if (hasPrefix(dev_name, "cu.usbserial") || hasPrefix(dev_name, "cu.usbmodem")) {
            return 0;
        }
        if (hasPrefix(dev_name, "cu.Bluetooth") || hasPrefix(dev_name, "cu.debug-console") ||
            hasPrefix(dev_name, "cu.wlan-debug")) {
            return 2;
        }
        return 1;
    }
    return -1;
#else
    if (hasPrefix(dev_name, "ttyUSB") || hasPrefix(dev_name, "ttyACM")) {
        return 0;
    }
    return -1;
#endif
}

size_t listSerialPorts(SerialPortInfo* out, size_t capacity)
{
    if (out == nullptr || capacity == 0) {
        return 0;
    }
    size_t count = 0;
#if defined(__APPLE__)
    scanDirectory("/dev", out, capacity, count, &rankSerialPortName);
#else
    // Prefer the stable USB-serial names when the kernel provides them;
    // every entry there IS a USB serial adapter, so rank everything 0.
    scanDirectory("/dev/serial/by-id", out, capacity, count,
                  [](const char* name) { return (name[0] == '.') ? -1 : 0; });
    if (count == 0) {
        scanDirectory("/dev", out, capacity, count, &rankSerialPortName);
    }
#endif
    return count;
}

}  // namespace m5::variants::frameworks::posix::hal::v1::uart

#endif  // M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

#endif
