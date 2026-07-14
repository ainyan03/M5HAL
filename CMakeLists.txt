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
# bootstrap path as well, because local v0/v2 development relies heavily on
# build_check + HIL fixtures via PlatformIO espidf.

if(COMMAND idf_component_register)
    # ESP-IDF component manifest for M5HAL.
    # Under pure ESP-IDF, v0 GPIO and v2 espidf GPIO/SPI drive pins through
    # driver/gpio.h, and v2 espidf I2C/SPI use the ESP-IDF master drivers.
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

    # Arduino as an IDF component: when the arduino-esp32 core is part of the
    # same build, M5HAL must be compiled with ARDUINO visible too. The variant
    # offer headers pick the winner in EVERY including TU, so if user code sees
    # ARDUINO (via its arduino-esp32 dependency) while this component does not,
    # the two disagree on the winning variant and the link fails (undefined
    # reference to Bus_arduino members). Depending on the core component
    # propagates its public ARDUINO defines into M5HAL's own TUs. The core may
    # appear under its registry name (espressif__arduino-esp32) or as a plain
    # local component (arduino / arduino-esp32).
    idf_build_get_property(m5hal_component_targets __COMPONENT_TARGETS)
    if("___idf_espressif__arduino-esp32" IN_LIST m5hal_component_targets)
        set(m5hal_arduino_req espressif__arduino-esp32)
    elseif("___idf_arduino-esp32" IN_LIST m5hal_component_targets)
        set(m5hal_arduino_req arduino-esp32)
    elseif("___idf_arduino" IN_LIST m5hal_component_targets)
        set(m5hal_arduino_req arduino)
    else()
        set(m5hal_arduino_req "")
    endif()

    # I2C is special on IDF v5.3+: the default backend uses esp_driver_i2c
    # (driver/i2c_master.h), while M5HAL_CONFIG_ESPIDF_I2C_MASTER_LEGACY_DRIVER=1 switches the
    # C++ headers to driver/i2c.h. That config is usually a compiler definition,
    # not a CMake variable visible while REQUIRES is computed, so the component
    # declares both public dependencies. The C++ feature selection still compiles
    # exactly one backend.
    #
    # v0/v2 coexistence: each generation keeps its own implementation TU
    # (M5HAL_v0.cpp / M5HAL_v2.cpp), registered separately.
    # See spec/design/v0_v2_coexistence.md for the public design rationale.
    # esp_timer backs the v2 runtime kind (millis/micros, S7) and is included
    # from a public header, so the dependency must be public. It is NOT one of
    # ESP-IDF's implicit common components (unlike esp_rom / freertos, which
    # the runtime kind also uses).
    # USB OTG CDC (TinyUSB): when the esp_tinyusb component is present in the
    # build, add it to REQUIRES so Bus_espidf_usb_cdc can include its headers.
    # Projects without TinyUSB simply compile the class out (header guard).
    if("___idf_espressif__esp_tinyusb" IN_LIST m5hal_component_targets)
        set(m5hal_tinyusb_req espressif__esp_tinyusb)
    elseif("___idf_esp_tinyusb" IN_LIST m5hal_component_targets)
        set(m5hal_tinyusb_req esp_tinyusb)
    else()
        set(m5hal_tinyusb_req "")
    endif()

    # USB Serial/JTAG: split out of 'driver' in IDF v5.3+ as
    # esp_driver_usb_serial_jtag. Only chips with USB-JTAG have this
    # component; detect its presence to avoid REQUIRES failures on
    # chips without it (ESP32, ESP32-S2, ESP32-C2).
    if("___idf_esp_driver_usb_serial_jtag" IN_LIST m5hal_component_targets)
        set(m5hal_usb_jtag_req esp_driver_usb_serial_jtag)
    else()
        set(m5hal_usb_jtag_req "")
    endif()

    idf_component_register(
        SRCS "src/M5HAL_v0.cpp" "src/M5HAL_v2.cpp"
        INCLUDE_DIRS "src"
        REQUIRES M5Utility esp_timer ${m5hal_gpio_req} ${m5hal_i2c_req} ${m5hal_spi_req} ${m5hal_uart_req} ${m5hal_arduino_req} ${m5hal_tinyusb_req} ${m5hal_usb_jtag_req}
    )
else()
    # PlatformIO espidf standalone build path.
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(M5HAL)
endif()
