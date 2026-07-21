// SPDX-License-Identifier: MIT
// Per-kind impl hub for the ESP-IDF framework variant. Included by
// M5HAL_v2.cpp.
//
// GPIO is header-only. I2C/SPI keep ESP-IDF driver-generation differences
// behind their own backend includes.
#include "hal/i2s/controller_lease.inl"
#include "hal/i2c/i2c.inl"
#include "hal/spi/spi.inl"
#include "hal/spi/slave.inl"
#include "hal/uart/uart.inl"
#include "hal/uart/usb_jtag.inl"
#include "hal/uart/usb_cdc.inl"
#include "hal/i2s/i2s.inl"
#include "hal/pdm/pdm.inl"
#include "hal/tcp/tcp.inl"
