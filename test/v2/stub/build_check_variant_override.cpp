// SPDX-License-Identifier: MIT
// Compile-only fixture: each kind can select an exact participating provider.
// The choices deliberately bypass earlier offers where the host matrix has one.
#define M5HAL_CONFIG_REMOTE_VARIANT        1
#define M5HAL_CONFIG_VARIANT_RUNTIME       M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#define M5HAL_CONFIG_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#define M5HAL_CONFIG_VARIANT_RUNTIME_TASK  M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#define M5HAL_CONFIG_VARIANT_RUNTIME_EVENT M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#define M5HAL_CONFIG_VARIANT_GPIO          M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#define M5HAL_CONFIG_VARIANT_I2C           M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#define M5HAL_CONFIG_VARIANT_SPI           M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#define M5HAL_CONFIG_VARIANT_I2S           M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#define M5HAL_CONFIG_VARIANT_PDM           M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#define M5HAL_CONFIG_VARIANT_UART          M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#include <M5HAL_v2.hpp>

#include <type_traits>

static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB);
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB);
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB);
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB);
static_assert(M5HAL_V2_SELECTED_VARIANT_GPIO == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB);
static_assert(M5HAL_V2_SELECTED_VARIANT_I2C == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE);
static_assert(M5HAL_V2_SELECTED_VARIANT_SPI == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE);
static_assert(M5HAL_V2_SELECTED_VARIANT_I2S == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE);
static_assert(M5HAL_V2_SELECTED_VARIANT_PDM == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE);
static_assert(M5HAL_V2_SELECTED_VARIANT_UART == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE);

static_assert(
    std::is_same<::m5::hal::v2::runtime::Mutex, ::m5::variants::frameworks::stub::hal::v2::runtime::Mutex>::value);
static_assert(
    std::is_same<::m5::hal::v2::runtime::Task, ::m5::variants::frameworks::stub::hal::v2::runtime::Task>::value);
static_assert(
    std::is_same<::m5::hal::v2::runtime::Event, ::m5::variants::frameworks::stub::hal::v2::runtime::Event>::value);
static_assert(std::is_same<::m5::hal::v2::gpio::Port, ::m5::hal::v2::gpio::Port_stub>::value);
static_assert(std::is_same<::m5::hal::v2::gpio::GPIO, ::m5::hal::v2::gpio::GPIO_stub>::value);
static_assert(std::is_same<::m5::hal::v2::i2c::BusConfig, ::m5::hal::v2::i2c::IBusConfig>::value);
static_assert(std::is_same<::m5::hal::v2::spi::BusConfig, ::m5::hal::v2::spi::IBusConfig>::value);
static_assert(std::is_same<::m5::hal::v2::i2s::BusConfig, ::m5::hal::v2::i2s::IBusConfig>::value);
static_assert(std::is_same<::m5::hal::v2::pdm::BusConfig, ::m5::hal::v2::pdm::IBusConfig>::value);
static_assert(std::is_same<::m5::hal::v2::uart::BusConfig, ::m5::hal::v2::uart::IBusConfig>::value);
