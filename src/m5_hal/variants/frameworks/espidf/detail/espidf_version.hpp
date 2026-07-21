// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_DETAIL_ESPIDF_VERSION_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_DETAIL_ESPIDF_VERSION_HPP

// Centralized ESP-IDF feature detection for the espidf framework variant.
// Prefer capability probes over raw version checks so backports and future
// driver reshuffles stay local to this file.

#if defined(ESP_PLATFORM)

#include <esp_idf_version.h>

// ESP-IDF I2C master driver detection.
//
// gen<N> is an ERA TAG, not a driver design-generation count. ESP-IDF has had
// only two I2C driver designs:
//   - command-link (driver/i2c.h): the original, dating to 2015; spans v2..v5.1,
//     deprecated in v5.2, EOL as of v6.0. Tagged gen4 for its legacy era.
//   - bus-device (driver/i2c_master.h): introduced in v5.2; current on v5.2 and
//     later (incl. v6.x). Tagged gen5 (= "IDF5 or later", until a newer design ships).
// Detection is by header presence (__has_include), not by version number, so a
// runtime that ships only one of the headers still resolves correctly.

// Capability probes (header presence) — independent of which backend M5HAL
// actually selects below.
//   gen5: bus-device API (driver/i2c_master.h, ESP-IDF v5.2+).
//   gen4: command-link (legacy) API (driver/i2c.h, ESP-IDF v2..v5.x).
#if __has_include(<driver/i2c_master.h>)
#define M5HAL_DETAIL_ESPIDF_I2C_HAS_GEN5_DRIVER_ 1
#else
#define M5HAL_DETAIL_ESPIDF_I2C_HAS_GEN5_DRIVER_ 0
#endif
#if __has_include(<driver/i2c.h>)
#define M5HAL_DETAIL_ESPIDF_I2C_HAS_GEN4_DRIVER_ 1
#else
#define M5HAL_DETAIL_ESPIDF_I2C_HAS_GEN4_DRIVER_ 0
#endif

// Backend selection — M5HAL compiles exactly ONE I2C backend. ESP-IDF aborts at
// runtime if the legacy command-link driver (driver/i2c.h) and the modern
// bus-device driver (driver/i2c_master.h) are linked into the same image, so we
// must never pull in both. Default = modern gen5. The legacy gen4 backend is
// opt-in via M5HAL_CONFIG_ESPIDF_I2C_MASTER_LEGACY_DRIVER=1: a project that already uses the
// legacy driver sets it to force M5HAL onto gen4 and avoid the mixed-link abort.
// Exception: on ESP-IDF older than v5.2 the modern driver does not exist (so no
// conflict is possible), and M5HAL falls back to gen4 automatically there.
#ifndef M5HAL_CONFIG_ESPIDF_I2C_MASTER_LEGACY_DRIVER
#define M5HAL_CONFIG_ESPIDF_I2C_MASTER_LEGACY_DRIVER 0
#endif
#if M5HAL_CONFIG_ESPIDF_I2C_MASTER_LEGACY_DRIVER
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 0
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4 M5HAL_DETAIL_ESPIDF_I2C_HAS_GEN4_DRIVER_
#else
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 M5HAL_DETAIL_ESPIDF_I2C_HAS_GEN5_DRIVER_
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4 \
    (!M5HAL_DETAIL_ESPIDF_I2C_HAS_GEN5_DRIVER_ && M5HAL_DETAIL_ESPIDF_I2C_HAS_GEN4_DRIVER_)
#endif

#define M5HAL_ESPIDF_I2C_HAS_MASTER (M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 || M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4)

// ESP-IDF I2C slave v2 detection. On v5.4/v5.5 driver/i2c_slave.h exposes
// either the legacy v2 or the v2 API from the same header depending on the
// sdkconfig option CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2 (so header
// presence alone cannot distinguish them). On v6.0+ the v2 slave API and the
// Kconfig option were both removed and v2 is the only implementation, so the
// version number is the correct probe there.
#if (defined(CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2) && CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2) || \
    (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0) && __has_include(<driver/i2c_slave.h>))
#define M5HAL_ESPIDF_I2C_HAS_SLAVE_V2 1
#else
#define M5HAL_ESPIDF_I2C_HAS_SLAVE_V2 0
#endif

#if __has_include(<driver/spi_master.h>)
#define M5HAL_ESPIDF_SPI_HAS_MASTER 1
#else
#define M5HAL_ESPIDF_SPI_HAS_MASTER 0
#endif

// ESP-IDF I2S gen5 driver detection (driver/i2s_std.h, ESP-IDF v5.0+).
// Only gen5 is supported; legacy gen4 (driver/i2s.h) is intentionally
// excluded. Header presence alone is NOT sufficient: on SoCs without I2S
// (e.g. esp32c2) the esp_driver_i2s headers are still on the include path
// but do not compile, so SOC_I2S_SUPPORTED must also be checked.
#if __has_include(<soc/soc_caps.h>)
#include <soc/soc_caps.h>
#endif
#if __has_include(<driver/i2s_std.h>) && defined(SOC_I2S_SUPPORTED) && SOC_I2S_SUPPORTED
#define M5HAL_ESPIDF_I2S_HAS_STD 1
#else
#define M5HAL_ESPIDF_I2S_HAS_STD 0
#endif

// PDM is a distinct public bus kind. Its initial backend requires hardware
// PDM RX plus the hardware PDM-to-PCM converter; raw capture is not silently
// substituted on SoCs lacking the converter.
#if __has_include(<driver/i2s_pdm.h>) && defined(SOC_I2S_SUPPORTS_PDM_RX) && SOC_I2S_SUPPORTS_PDM_RX && \
    defined(SOC_I2S_SUPPORTS_PDM2PCM) && SOC_I2S_SUPPORTS_PDM2PCM
#define M5HAL_ESPIDF_PDM_HAS_RX_PCM 1
#else
#define M5HAL_ESPIDF_PDM_HAS_RX_PCM 0
#endif

// LP_I2C exposure gate: the controller pool only reclaims the LP_I2C
// instance (SOC_I2C_NUM's HP+LP combined count) as a poolable, opt-in
// controller when ALL of the following hold:
//   - the modern bus-device driver is selected (gen5) -- gen4 (legacy
//     command-link) always rejects an LP port outright, so it stays HP-only
//     regardless of SOC_LP_I2C_SUPPORTED.
//   - the SoC has an LP_I2C instance at all (SOC_LP_I2C_SUPPORTED).
//   - ESP-IDF >= 5.4 for fixed-pin (IOMUX pad) LP chips, matching
//     arduino-esp32's own NG HAL gate: 5.3.x lacks a verified
//     `lp_source_clk` contract (asserts unconditionally on some builds, see
//     esp-idf#14908).
//   - ESP-IDF >= 5.5 for LP-GPIO-matrix chips: the driver's matrix routing
//     connects SDA/SCL to each other's signals in 5.3.x (i2c_common.c), the
//     fix is source-verified only in 5.5.x, and 5.4.x is UNVERIFIED -- so
//     the matrix path stays HP-only there rather than risking a silently
//     mis-wired bus.
#if M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 && defined(SOC_LP_I2C_SUPPORTED) && SOC_LP_I2C_SUPPORTED && \
    ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
#if defined(SOC_LP_GPIO_MATRIX_SUPPORTED) && SOC_LP_GPIO_MATRIX_SUPPORTED
#define M5HAL_ESPIDF_I2C_LP_POOL (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0))
#else
#define M5HAL_ESPIDF_I2C_LP_POOL 1
#endif
#else
#define M5HAL_ESPIDF_I2C_LP_POOL 0
#endif

#else

#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 0
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4 0
#define M5HAL_ESPIDF_I2C_HAS_MASTER      0
#define M5HAL_ESPIDF_I2C_HAS_SLAVE_V2    0
#define M5HAL_ESPIDF_SPI_HAS_MASTER      0
#define M5HAL_ESPIDF_I2S_HAS_STD         0
#define M5HAL_ESPIDF_PDM_HAS_RX_PCM      0
#define M5HAL_ESPIDF_I2C_LP_POOL         0

#endif

#endif
