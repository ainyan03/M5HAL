// SPDX-License-Identifier: MIT
// Per-kind impl hub for the ESP-IDF framework variant. Included by
// M5HAL_v1.cpp.
//
// GPIO is header-only. I2C/SPI keep ESP-IDF driver-generation differences
// behind their own backend includes.
#include "hal/i2c/i2c.inl"
#include "hal/spi/spi.inl"
#include "hal/uart/uart.inl"
#include "hal/i2s/i2s.inl"
#include "hal/tcp/tcp.inl"
