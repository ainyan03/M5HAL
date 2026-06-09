cmake_minimum_required(VERSION 3.16.0)

# Official ESP-IDF component build check for M5HAL.
#
# Builds M5HAL (and its M5Utility dependency) as ESP-IDF components straight
# from the local source trees, then builds the tiny consumer in main/ that
# exercises the public API surface (test/v1/build_check). Run with the official
# espressif/idf toolchain via `idf.py set-target <chip> build`, locally or in
# CI (.github/workflows/IDFBuild.yml). This is the authoritative full-family
# verification (esp32 / s3 / c3 / c6 / h2 / p4) — official ESP-IDF supports
# every chip M5HAL targets, whereas the PlatformIO `espressif32` platform ships
# no h2/p4 boards.
#
# IDF_COMPONENT_MANAGER=0 is set by the caller so dependencies resolve only
# from the local EXTRA_COMPONENT_DIRS (hermetic, no network), mirroring the
# PlatformIO build check which uses a local M5Utility checkout.
get_filename_component(M5HAL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
get_filename_component(M5UTILITY_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../M5Utility" ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${M5HAL_ROOT}" "${M5UTILITY_ROOT}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(m5hal_idf_build_check)
