cmake_minimum_required(VERSION 3.16.0)

# Hybrid manifest for both ESP-IDF component consumers and standalone builds.
#
# - When read by the ESP-IDF Component Registry / idf.py, this file is
#   evaluated with `idf_component_register` already defined, so it registers
#   M5HAL as a component.
# - When PlatformIO's espidf framework builds the repo standalone,
#   `idf_component_register` is undefined, so it acts as a project bootstrap.
#
# Upstream m5stack/M5HAL supports only the former path. We keep the project
# bootstrap path as well, because local v0/v1 development relies heavily on
# build_check + experiments via PlatformIO espidf.

if(COMMAND idf_component_register)
    # ESP-IDF component manifest for M5HAL.
    # Under pure ESP-IDF, v0 GPIO and v1 espidf GPIO/SPI drive pins through
    # driver/gpio.h, and v1 espidf I2C/SPI use the ESP-IDF master drivers.
    # The espidf variant public headers expose driver types for attach/native
    # handles, so the driver component dependencies must be public.
    #
    # The esp_driver_* components (gpio / i2c / spi / uart) were all split out
    # of the monolithic 'driver' component in the IDF v5.3 "driver as
    # components" refactor. On v5.2 and earlier the headers we use
    # (driver/gpio.h, driver/i2c_master.h, driver/spi_master.h, driver/uart.h)
    # all live in 'driver'. Note the new i2c_master API shipped in v5.2 but
    # inside 'driver'; the standalone esp_driver_i2c component only exists from
    # v5.3 — requiring it on v5.2 fails to resolve (verified via the idf-matrix
    # esp32 build). The v5.3 threshold also matches upstream m5stack/M5HAL's
    # esp_driver_gpio fix (commit 67e314f).
    if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
        set(m5hal_gpio_req esp_driver_gpio)
        set(m5hal_spi_req esp_driver_spi)
        set(m5hal_uart_req esp_driver_uart)
        set(m5hal_i2c_req esp_driver_i2c driver)
    else()
        set(m5hal_gpio_req driver)
        set(m5hal_i2c_req driver)
        set(m5hal_spi_req driver)
        set(m5hal_uart_req driver)
    endif()

    # I2C is special on IDF v5.3+: the default backend uses esp_driver_i2c
    # (driver/i2c_master.h), while M5HAL_CONFIG_IDF_I2C_LEGACY=1 switches the
    # C++ headers to driver/i2c.h. That config is usually a compiler definition,
    # not a CMake variable visible while REQUIRES is computed, so the component
    # declares both public dependencies. The C++ feature selection still compiles
    # exactly one backend.
    #
    # v0/v1 coexistence: register two translation units (M5HAL_v0.cpp and
    # M5HAL_v1.cpp) separately. The shared macros (M5HAL_FRAMEWORK_HAS_* /
    # M5HAL_TARGET_PLATFORM_*) collide between v0 and v1, so they cannot be
    # merged into a single TU.
    # See spec/design/v0_v1_coexistence.md for the public design rationale.
    idf_component_register(
        SRCS "src/M5HAL_v0.cpp" "src/M5HAL_v1.cpp"
        INCLUDE_DIRS "src"
        REQUIRES M5Utility ${m5hal_gpio_req} ${m5hal_i2c_req} ${m5hal_spi_req} ${m5hal_uart_req}
    )
else()
    # PlatformIO espidf standalone build path.
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(M5HAL)
endif()
