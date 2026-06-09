#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_DETAIL_ESPIDF_VERSION_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_DETAIL_ESPIDF_VERSION_HPP

// Centralized ESP-IDF feature detection for the espidf framework variant.
// Prefer capability probes over raw version checks so backports and future
// driver reshuffles stay local to this file.

#if defined(ESP_PLATFORM)

#include <esp_idf_version.h>

// ESP-IDF I2C master driver detection.
//
// gen<N> is an ERA TAG: it names the I2C driver M5HAL uses on ESP-IDF v<N>.x,
// matching the espidf4 / espidf5 / espidf6 build envs. It is NOT a driver
// design-generation count. ESP-IDF has had only two I2C driver designs:
//   - command-link (driver/i2c.h): the original, dating to 2015; spans v1..v5.1,
//     deprecated in v5.2, EOL as of v6.0. Tagged gen4 (M5HAL's IDF4 support floor).
//   - bus-device (driver/i2c_master.h): introduced in v5.2; current on v5.2 and
//     later (incl. v6.x). Tagged gen5 (= "IDF5 or later", until a newer design ships).
// Detection is by header presence (__has_include), not by version number, so a
// runtime that ships only one of the headers still resolves correctly.

// gen5: bus-device I2C master API (driver/i2c_master.h, ESP-IDF v5.2+).
#if __has_include(<driver/i2c_master.h>)
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 1
#else
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 0
#endif

// gen4: command-link (legacy) I2C master API (driver/i2c.h, ESP-IDF v1..v6.0 EOL).
#if __has_include(<driver/i2c.h>)
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4 1
#else
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4 0
#endif

#define M5HAL_ESPIDF_I2C_HAS_MASTER (M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 || M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4)

#if __has_include(<driver/spi_master.h>)
#define M5HAL_ESPIDF_SPI_HAS_MASTER 1
#else
#define M5HAL_ESPIDF_SPI_HAS_MASTER 0
#endif

#else

#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 0
#define M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4 0
#define M5HAL_ESPIDF_I2C_HAS_MASTER      0
#define M5HAL_ESPIDF_SPI_HAS_MASTER      0

#endif

#endif
