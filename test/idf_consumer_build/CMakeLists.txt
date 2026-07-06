cmake_minimum_required(VERSION 3.16.0)

# Consumer-view dependency-resolution check for M5HAL.
#
# The sibling idf_component_build check is hermetic: it injects M5HAL and a
# local M5Utility checkout via EXTRA_COMPONENT_DIRS with the component manager
# disabled, so it can never notice when the PUBLISHED manifest is broken for
# real consumers (wrong dependency pin, a dependency whose `targets:` list
# misses a chip M5HAL declares, ...). This project is the opposite: a minimal
# downstream application that depends on M5HAL only through main/'s
# idf_component.yml and lets the IDF component manager resolve M5HAL's own
# dependencies exactly as an end user's build would (network access required —
# M5Utility is fetched from its upstream git URL).
#
# Run with the component manager ENABLED (do not set IDF_COMPONENT_MANAGER=0):
#   idf.py set-target <chip> build
# Delete managed_components/ and dependencies.lock first to force a fresh
# resolution; both are generated into this directory and are gitignored.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(m5hal_idf_consumer_check)
