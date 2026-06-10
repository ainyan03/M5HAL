// Per-kind impl hub for the POSIX host framework variant. Included by
// M5HAL_v1.cpp when M5HAL_FRAMEWORK_HAS_POSIX is set; each subfile is
// self-contained for its kind.

#include "hal/uart/uart.inl"
#include "hal/uart/ports.inl"
#include "hal/uart/remote_connect.inl"
