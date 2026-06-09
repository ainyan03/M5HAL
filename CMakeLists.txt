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
    # esp_driver_* components were split out of the monolithic 'driver'
    # component in IDF v5.2. On v5.0/v5.1 they live in 'driver'.
    if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.2")
        set(m5hal_gpio_req esp_driver_gpio)
        set(m5hal_i2c_req esp_driver_i2c)
        set(m5hal_spi_req esp_driver_spi)
    else()
        set(m5hal_gpio_req driver)
        set(m5hal_i2c_req driver)
        set(m5hal_spi_req driver)
    endif()

    # UART (driver/uart.h, used by the espidf UART variant) was split into its
    # own esp_driver_uart component later, in IDF v5.3; on v5.2 and earlier it
    # still lives in 'driver'.
    if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
        set(m5hal_uart_req esp_driver_uart)
    else()
        set(m5hal_uart_req driver)
    endif()

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
