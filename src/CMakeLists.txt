# PlatformIO / ESP-IDF standalone-build main component.
#
# The package/component entry point is the repository-root CMakeLists.txt.
# When that root file bootstraps PlatformIO's ESP-IDF project path, ESP-IDF
# still expects a minimal `src/CMakeLists.txt` for the app/main component.
# Keep this file small and do not put M5HAL package metadata here.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources})
