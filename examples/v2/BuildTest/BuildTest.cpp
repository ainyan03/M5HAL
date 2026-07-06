// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — BuildTest
//
// CI build verification: exercises public API types and functions so that
// a successful compile proves the API surface has not broken. This file
// is NOT a runnable example — see HowToUse/ for working tutorials.
//
// Build:
//   pio run -e BuildTest_esp32
//   pio run -e BuildTest_esp32s3
//   pio run -e BuildTest_host       (POSIX native)
// =============================================================================

#include <M5HAL_v2.hpp>

namespace m5hal = m5::hal::v2;

// ---------------------------------------------------------------------------
// Error and result types
// ---------------------------------------------------------------------------
static void checkErrorAndResult()
{
    m5hal::error::error_t ok       = m5hal::error::error_t::OK;
    m5hal::error::error_t timeout  = m5hal::error::error_t::TIMEOUT_ERROR;
    m5hal::error::error_t invalid  = m5hal::error::error_t::INVALID_ARGUMENT;
    m5hal::error::error_t missing  = m5hal::error::error_t::NOT_IMPLEMENTED;
    m5hal::error::error_t busy     = m5hal::error::error_t::BUSY;
    m5hal::error::error_t io       = m5hal::error::error_t::IO_ERROR;
    m5hal::error::error_t closed   = m5hal::error::error_t::CLOSED;
    m5hal::error::error_t resource = m5hal::error::error_t::OUT_OF_RESOURCE;
    m5hal::error::error_t overflow = m5hal::error::error_t::BUFFER_OVERFLOW;

    (void)m5hal::error::isOk(ok);
    (void)m5hal::error::isError(timeout);
    (void)m5hal::error::toString(invalid);
    (void)missing;
    (void)busy;
    (void)io;
    (void)closed;
    (void)resource;
    (void)overflow;

    m5hal::result_t<int> ok_val{42};
    (void)ok_val.has_value();
    (void)ok_val.value();
    (void)static_cast<bool>(ok_val);
    int deref = *ok_val;
    (void)deref;

    m5hal::result_t<int> err_val = m5::stl::make_unexpected(timeout);
    (void)err_val.has_value();
    (void)err_val.error();
    (void)static_cast<bool>(err_val);

    m5hal::result_t<void> ok_void{};
    (void)ok_void.has_value();
    (void)static_cast<bool>(ok_void);

    m5hal::result_t<void> err_void = m5::stl::make_unexpected(invalid);
    (void)err_void.has_value();
    (void)err_void.error();
}

// ---------------------------------------------------------------------------
// Bus base types
// ---------------------------------------------------------------------------
static void checkBusBase()
{
    (void)sizeof(m5hal::bus::IBusConfig);
    (void)sizeof(m5hal::bus::ITransferDesc);
    (void)sizeof(m5hal::bus::IAccessConfig);
    (void)sizeof(m5hal::bus::IAccessor);
    (void)sizeof(m5hal::bus::IBus);
    (void)sizeof(m5hal::bus::ScopedLock);

    m5hal::bus::TransferTotals totals;
    totals.tx = 1;
    totals.rx = 2;
    m5hal::bus::TransferTotals other;
    other.tx = 3;
    other.rx = 4;
    totals.add(other);
    totals.clear();
    (void)totals;

    m5hal::bus::IBus* bus           = nullptr;
    m5hal::bus::IAccessor* accessor = nullptr;
    if (false) {
        (void)bus->getConfig();
        (void)bus->getBusKind();
        (void)bus->release();
        (void)bus->lock(accessor);
        (void)bus->unlock(accessor);
        (void)bus->backendKind();
        (void)bus->controllerId();
        (void)bus->maxFrequency();
        (void)bus->backendGeneration();

        (void)accessor->getConfig();
        (void)accessor->getBusKind();
        (void)accessor->isBound();
        (void)accessor->getBus();
        (void)accessor->getBusConfig();
        (void)accessor->beginAccess();
        (void)accessor->endAccess();
        (void)accessor->inAccess();
    }
}

// ---------------------------------------------------------------------------
// Allocation intent and bus views
// ---------------------------------------------------------------------------
static void checkAllocationAndBusViews()
{
    m5hal::types::AllocationIntent auto_intent   = m5hal::bus::automatic();
    m5hal::types::AllocationIntent require_hw    = m5hal::bus::requireHardware();
    m5hal::types::AllocationIntent prefer_hw     = m5hal::bus::preferHardware();
    m5hal::types::AllocationIntent software_only = m5hal::bus::software();
    m5hal::types::AllocationIntent require_ctl   = m5hal::bus::requireController(0);
    m5hal::types::AllocationIntent prefer_ctl    = m5hal::bus::preferController(1);

    (void)auto_intent.valid();
    (void)require_hw.valid();
    (void)prefer_hw.valid();
    (void)software_only.valid();
    (void)require_ctl.valid();
    (void)prefer_ctl.valid();
    (void)m5hal::types::ControllerMode::Any;
    (void)m5hal::types::ControllerMode::Prefer;
    (void)m5hal::types::ControllerMode::Require;
    (void)m5hal::types::BackendKind::Software;
    (void)m5hal::types::BackendKind::Hardware;

    m5hal::i2c::BusView i2c_view;
    m5hal::spi::BusView spi_view;
    m5hal::uart::BusView uart_view;
    m5hal::i2s::BusView i2s_view;
    (void)i2c_view;
    (void)spi_view;
    (void)uart_view;
    (void)i2s_view;

    m5hal::i2c::LogicalBusConfig i2c_log{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}, require_hw};
    m5hal::spi::LogicalBusConfig spi_log{m5hal::spi::Clk{18}, m5hal::spi::Mosi{23}, m5hal::spi::Miso{19}, prefer_hw};
    m5hal::uart::LogicalBusConfig uart_log{m5hal::uart::Tx{17}, m5hal::uart::Rx{16}, auto_intent};
    m5hal::i2s::LogicalBusConfig i2s_log{m5hal::i2s::Bclk{26}, m5hal::i2s::Ws{25}, m5hal::i2s::Dout{22},
                                         m5hal::i2s::Din{21}, software_only};
    (void)i2c_log.pin_scl;
    (void)spi_log.pin_clk;
    (void)uart_log.pin_tx;
    (void)i2s_log.pin_bclk;

    if (false) {
        auto i2c_bus = i2c_view.acquire(i2c_log);
        (void)i2c_view.release(i2c_bus.value());
        (void)i2c_view.commitBuses();
        (void)i2c_view.hardwareInUse();
        (void)i2c_view.createBusConfig(m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}, require_hw);

        auto spi_bus = spi_view.acquire(spi_log);
        (void)spi_view.release(spi_bus.value());
        (void)spi_view.commitBuses();
        (void)spi_view.hardwareInUse();
        (void)spi_view.createBusConfig(m5hal::spi::Clk{18}, m5hal::spi::Mosi{23}, m5hal::spi::Miso{19}, prefer_hw);

        auto uart_bus = uart_view.acquire(uart_log);
        (void)uart_view.release(uart_bus.value());
        (void)uart_view.commitBuses();
        (void)uart_view.hardwareInUse();
        (void)uart_view.createBusConfig(m5hal::uart::Tx{17}, m5hal::uart::Rx{16}, auto_intent);

        auto i2s_bus = i2s_view.acquire(i2s_log);
        (void)i2s_view.release(i2s_bus.value());
        (void)i2s_view.commitBuses();
        (void)i2s_view.hardwareInUse();
        (void)i2s_view.createBusConfig(m5hal::i2s::Bclk{26}, m5hal::i2s::Ws{25}, m5hal::i2s::Dout{22},
                                       m5hal::i2s::Din{21}, software_only);
    }
}

// ---------------------------------------------------------------------------
// I2C types
// ---------------------------------------------------------------------------
static void checkI2C()
{
    (void)sizeof(m5hal::i2c::IBus);
    (void)sizeof(m5hal::i2c::IBusConfig);
    (void)sizeof(m5hal::i2c::BusConfig);
    (void)sizeof(m5hal::i2c::LogicalBusConfig);
    (void)sizeof(m5hal::i2c::MasterAccessConfig);
    (void)sizeof(m5hal::i2c::MasterAccessor);
    (void)sizeof(m5hal::i2c::TransferDesc);
    (void)sizeof(m5hal::i2c::Scl);
    (void)sizeof(m5hal::i2c::Sda);

    m5hal::i2c::IBusConfig bus_cfg{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}};
    (void)bus_cfg.pin_scl;
    (void)bus_cfg.pin_sda;
    (void)bus_cfg.getBusKind();

    m5hal::i2c::MasterAccessConfig acc_cfg;
    acc_cfg.freq                   = 400000;
    acc_cfg.wire_timeout_ms        = 10;
    acc_cfg.i2c_addr               = 0x3C;
    acc_cfg.address_is_10bit       = false;
    acc_cfg.register_address_bytes = 1;
    acc_cfg.use_restart            = true;
    (void)acc_cfg.getBusKind();

    m5hal::i2c::TransferDesc no_prefix;
    m5hal::i2c::TransferDesc reg8{uint8_t{0x12}};
    m5hal::i2c::TransferDesc reg16{uint16_t{0x1234}};
    m5hal::i2c::TransferDesc bytes2{uint8_t{0x01}, uint8_t{0x02}};
    m5hal::i2c::TransferDesc bytes3{uint8_t{0x01}, uint8_t{0x02}, uint8_t{0x03}};
    m5hal::i2c::TransferDesc bytes4{uint8_t{0x01}, uint8_t{0x02}, uint8_t{0x03}, uint8_t{0x04}};
    (void)no_prefix.prefix_len;
    (void)reg8.prefix[0];
    (void)reg16.prefix_len;
    (void)bytes2.prefix_len;
    (void)bytes3.prefix_len;
    (void)bytes4.prefix_len;

    m5hal::i2c::MasterAccessor accessor;
    m5hal::i2c::MasterAccessor configured{acc_cfg};
    (void)accessor;
    (void)configured;

    m5hal::i2c::IBus* bus = nullptr;
    uint8_t buf[4]        = {};
    m5hal::data::MemorySource src{buf, sizeof(buf)};
    m5hal::data::MemorySink dst{buf, sizeof(buf)};
    if (false) {
        (void)bus->probe(0x3C);
        (void)bus->transfer(&accessor, acc_cfg, no_prefix, &src, sizeof(buf), &dst, sizeof(buf));
        (void)bus->waitTransfer(&accessor, acc_cfg);
        (void)bus->transferBusy(&accessor);

        (void)accessor.bind(*bus);
        (void)accessor.getConfig();
        (void)accessor.getBus();
        (void)accessor.setConfig(acc_cfg);
        (void)accessor.beginTransaction();
        (void)accessor.transfer(no_prefix, m5hal::data::ConstDataSpan{buf, sizeof(buf)},
                                m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)accessor.transfer(no_prefix, &src, sizeof(buf), &dst, sizeof(buf));
        (void)accessor.endTransaction();
        (void)accessor.transferBusy();
        (void)accessor.waitTransfer();
        (void)accessor.write(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)accessor.write(src, sizeof(buf));
        (void)accessor.write(buf, sizeof(buf));
        (void)accessor.read(m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)accessor.read(dst, sizeof(buf));
        (void)accessor.read(buf, sizeof(buf));
        (void)accessor.writeRegister(0x12, m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)accessor.writeRegister(0x12, uint8_t{0x34});
        (void)accessor.writeRegister(0x12, buf, sizeof(buf));
        (void)accessor.writeRegister(0x12, src, sizeof(buf));
        (void)accessor.readRegister(0x12, m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)accessor.readRegister(0x12, buf, sizeof(buf));
        (void)accessor.readRegister(0x12);
        (void)accessor.readRegister(0x12, dst, sizeof(buf));
        (void)accessor.probe();
    }
}

// ---------------------------------------------------------------------------
// SPI types
// ---------------------------------------------------------------------------
static void checkSPI()
{
    (void)sizeof(m5hal::spi::IBus);
    (void)sizeof(m5hal::spi::IBusConfig);
    (void)sizeof(m5hal::spi::BusConfig);
    (void)sizeof(m5hal::spi::LogicalBusConfig);
    (void)sizeof(m5hal::spi::MasterAccessConfig);
    (void)sizeof(m5hal::spi::MasterAccessor);
    (void)sizeof(m5hal::spi::ScopedTransaction);
    (void)sizeof(m5hal::spi::TransferDesc);
    (void)sizeof(m5hal::spi::Clk);
    (void)sizeof(m5hal::spi::Mosi);
    (void)sizeof(m5hal::spi::Miso);

    m5hal::spi::IBusConfig bus_cfg{m5hal::spi::Clk{18}, m5hal::spi::Mosi{23}, m5hal::spi::Miso{19}};
    bus_cfg.pin_dc = 27;
    bus_cfg.pin_d2 = 2;
    bus_cfg.pin_d3 = 4;
    bus_cfg.pin_d4 = 12;
    bus_cfg.pin_d5 = 13;
    bus_cfg.pin_d6 = 14;
    bus_cfg.pin_d7 = 15;
    (void)bus_cfg.pin_clk;
    (void)bus_cfg.pin_mosi;
    (void)bus_cfg.pin_miso;
    (void)bus_cfg.getBusKind();

    m5hal::spi::IBusConfig write_only_cfg{m5hal::spi::Clk{18}, m5hal::spi::Mosi{23}};
    (void)write_only_cfg.pin_miso;

    m5hal::spi::MasterAccessConfig acc_cfg;
    acc_cfg.pin_cs                = 5;
    acc_cfg.pin_dc                = 27;
    acc_cfg.freq                  = 40000000;
    acc_cfg.spi_data_mode         = m5hal::spi::spi_data_mode_t::FullDuplex;
    acc_cfg.spi_mode              = 0;
    acc_cfg.spi_order             = 0;
    acc_cfg.spi_command_length    = 8;
    acc_cfg.spi_address_length    = 24;
    acc_cfg.spi_read_dummy_cycle  = 8;
    acc_cfg.spi_write_dummy_cycle = 0;
    (void)acc_cfg.setupWithDCPin(27);
    (void)acc_cfg.setupWithDCBit();
    (void)acc_cfg.getBusKind();
    (void)m5hal::spi::spi_data_mode_t::HalfDuplex;
    (void)m5hal::spi::spi_data_mode_t::FullDuplex;
    (void)m5hal::spi::spi_data_mode_t::HalfDuplexWithDcPin;
    (void)m5hal::spi::spi_data_mode_t::FullDuplexWithDcPin;
    (void)m5hal::spi::spi_data_mode_t::HalfDuplexWithDcBit;
    (void)m5hal::spi::spi_data_mode_t::FullDuplexWithDcBit;
    (void)m5hal::spi::spi_data_mode_t::DualOutput;
    (void)m5hal::spi::spi_data_mode_t::DualIo;
    (void)m5hal::spi::spi_data_mode_t::QuadOutput;
    (void)m5hal::spi::spi_data_mode_t::QuadIo;
    (void)m5hal::spi::spi_data_mode_t::OctalOutput;
    (void)m5hal::spi::spi_data_mode_t::OctalIo;

    m5hal::spi::TransferDesc desc;
    desc.dc_level_valid   = true;
    desc.dc_level         = false;
    desc.command          = 0x9F;
    desc.address          = 0x123456;
    desc.command_bytes    = 1;
    desc.address_bytes    = 3;
    desc.dummy_cycles     = 8;
    desc.command_dc_level = 0;
    desc.address_dc_level = 1;
    desc.data_dc_level    = 1;
    (void)desc;

    m5hal::spi::MasterAccessor accessor;
    m5hal::spi::MasterAccessor configured{acc_cfg};
    (void)accessor;
    (void)configured;

    m5hal::spi::IBus* bus = nullptr;
    uint8_t buf[4]        = {};
    m5hal::data::MemorySource src{buf, sizeof(buf)};
    m5hal::data::MemorySink dst{buf, sizeof(buf)};
    if (false) {
        (void)bus->beginTransaction(&accessor, acc_cfg);
        (void)bus->endTransaction(&accessor, acc_cfg);
        (void)bus->transfer(&accessor, acc_cfg, desc, &src, sizeof(buf), &dst, sizeof(buf));
        (void)bus->waitTransfer(&accessor, acc_cfg);
        (void)bus->transferBusy(&accessor);

        (void)accessor.bind(*bus);
        (void)accessor.getConfig();
        (void)accessor.getBus();
        (void)accessor.setConfig(acc_cfg);
        (void)accessor.beginTransaction();
        (void)accessor.transfer(desc, m5hal::data::ConstDataSpan{buf, sizeof(buf)},
                                m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)accessor.transfer(desc, &src, sizeof(buf), &dst, sizeof(buf));
        (void)accessor.transfer(desc, &src, &dst, sizeof(buf));
        (void)accessor.endTransaction();
        (void)accessor.transferBusy();
        (void)accessor.waitTransfer();
        (void)accessor.write(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)accessor.write(src, sizeof(buf));
        (void)accessor.write(buf, sizeof(buf));
        (void)accessor.read(m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)accessor.read(dst, sizeof(buf));
        (void)accessor.read(buf, sizeof(buf));
        (void)accessor.writeCommand(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)accessor.writeCommand(uint32_t{0x2A});
        (void)accessor.writeCommandAddress(uint32_t{0x2A}, uint32_t{0x010203});
        (void)accessor.writeCommandData(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)accessor.writeCommandData(uint32_t{0x2C}, m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)accessor.writeCommandData(uint32_t{0x2C}, src, sizeof(buf));
        (void)accessor.writeCommandAddressData(uint32_t{0x2C}, uint32_t{0x010203},
                                               m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)accessor.writeCommandAddressData(uint32_t{0x2C}, uint32_t{0x010203}, src, sizeof(buf));
        (void)accessor.readCommandData(uint32_t{0x0A}, m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)accessor.readCommandData(uint32_t{0x0A}, dst, sizeof(buf));
        (void)accessor.readCommandAddressData(uint32_t{0x0B}, uint32_t{0x010203},
                                              m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)accessor.readCommandAddressData(uint32_t{0x0B}, uint32_t{0x010203}, dst, sizeof(buf));
        (void)accessor.sendDummyClock(8);
        m5hal::spi::ScopedTransaction tx_scope{accessor};
        (void)tx_scope.has_error();
        (void)tx_scope.ok();
        (void)tx_scope.error();
    }
}

// ---------------------------------------------------------------------------
// UART types
// ---------------------------------------------------------------------------
static void checkUART()
{
    (void)sizeof(m5hal::uart::IBus);
    (void)sizeof(m5hal::uart::IBusConfig);
    (void)sizeof(m5hal::uart::BusConfig);
    (void)sizeof(m5hal::uart::LogicalBusConfig);
    (void)sizeof(m5hal::uart::AccessConfig);
    (void)sizeof(m5hal::uart::TxAccessor);
    (void)sizeof(m5hal::uart::RxAccessor);
    (void)sizeof(m5hal::uart::Accessor);
    (void)sizeof(m5hal::uart::Tx);
    (void)sizeof(m5hal::uart::Rx);

    m5hal::uart::IBusConfig bus_cfg{m5hal::uart::Tx{17}, m5hal::uart::Rx{16}};
    bus_cfg.pin_rts        = 4;
    bus_cfg.pin_cts        = 5;
    bus_cfg.rx_buffer_size = 512;
    bus_cfg.tx_buffer_size = 256;
    (void)bus_cfg.pin_tx;
    (void)bus_cfg.pin_rx;
    (void)bus_cfg.getBusKind();

    m5hal::uart::AccessConfig acc_cfg;
    acc_cfg.baud_rate             = 115200;
    acc_cfg.first_byte_timeout_ms = 100;
    acc_cfg.inter_byte_timeout_ms = 20;
    acc_cfg.write_timeout_ms      = 1000;
    acc_cfg.data_bits             = 8;
    acc_cfg.stop_bits             = 1;
    acc_cfg.parity                = m5hal::uart::parity_t::None;
    acc_cfg.invert                = false;
    (void)acc_cfg.getBusKind();
    (void)m5hal::uart::parity_t::Even;
    (void)m5hal::uart::parity_t::Odd;
    (void)m5hal::uart::Channel::None;
    (void)m5hal::uart::Channel::Tx;
    (void)m5hal::uart::Channel::Rx;
    (void)m5hal::uart::Channel::TxRx;
    (void)m5hal::uart::hasChannel(m5hal::uart::Channel::TxRx, m5hal::uart::Channel::Tx);

    m5hal::uart::TxAccessor tx;
    m5hal::uart::RxAccessor rx;
    m5hal::uart::Accessor accessor;
    m5hal::uart::TxAccessor tx_cfg{acc_cfg};
    m5hal::uart::RxAccessor rx_cfg{acc_cfg};
    m5hal::uart::Accessor accessor_cfg{acc_cfg};
    (void)tx;
    (void)rx;
    (void)accessor;
    (void)tx_cfg;
    (void)rx_cfg;
    (void)accessor_cfg;

    m5hal::uart::IBus* bus = nullptr;
    uint8_t buf[4]         = {};
    m5hal::data::MemorySource src{buf, sizeof(buf)};
    m5hal::data::MemorySink dst{buf, sizeof(buf)};
    if (false) {
        (void)bus->write(&tx, acc_cfg, &src, sizeof(buf));
        (void)bus->read(&rx, acc_cfg, &dst, sizeof(buf));
        (void)bus->readableBytes(&rx, acc_cfg);
        (void)bus->lockChannel(&tx, m5hal::uart::Channel::Tx);
        (void)bus->unlockChannel(&tx, m5hal::uart::Channel::Tx);

        (void)tx.bind(*bus);
        (void)tx.getConfig();
        (void)tx.getBus();
        (void)tx.setConfig(acc_cfg);
        (void)tx.beginAccess();
        (void)tx.endAccess();
        (void)tx.inAccess();
        (void)tx.write(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)tx.write(src, sizeof(buf));
        (void)tx.write(buf, sizeof(buf));

        (void)rx.bind(*bus);
        (void)rx.getConfig();
        (void)rx.getBus();
        (void)rx.setConfig(acc_cfg);
        (void)rx.beginAccess();
        (void)rx.endAccess();
        (void)rx.inAccess();
        (void)rx.read(m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)rx.read(dst, sizeof(buf));
        (void)rx.read(buf, sizeof(buf));
        (void)rx.readUntil('\n', buf, sizeof(buf));
        (void)rx.readableBytes();

        (void)accessor.bind(*bus);
        (void)accessor.getConfig();
        (void)accessor.getBus();
        (void)accessor.tx();
        (void)accessor.rx();
        (void)accessor.setConfig(acc_cfg);
        (void)accessor.beginAccess();
        (void)accessor.endAccess();
        (void)accessor.inAccess();
        (void)accessor.write(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)accessor.write(src, sizeof(buf));
        (void)accessor.write(buf, sizeof(buf));
        (void)accessor.read(m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)accessor.read(dst, sizeof(buf));
        (void)accessor.read(buf, sizeof(buf));
        (void)accessor.readUntil('\n', buf, sizeof(buf));
        (void)accessor.readableBytes();
    }
}

// ---------------------------------------------------------------------------
// I2S types
// ---------------------------------------------------------------------------
static void checkI2S()
{
    (void)sizeof(m5hal::i2s::IBus);
    (void)sizeof(m5hal::i2s::IBusConfig);
    (void)sizeof(m5hal::i2s::LogicalBusConfig);
    (void)sizeof(m5hal::i2s::AccessConfig);
    (void)sizeof(m5hal::i2s::TxAccessor);
    (void)sizeof(m5hal::i2s::RxAccessor);
    (void)sizeof(m5hal::i2s::Accessor);
    (void)sizeof(m5hal::i2s::Bclk);
    (void)sizeof(m5hal::i2s::Ws);
    (void)sizeof(m5hal::i2s::Dout);
    (void)sizeof(m5hal::i2s::Din);

    m5hal::i2s::IBusConfig tx_cfg{m5hal::i2s::Bclk{26}, m5hal::i2s::Ws{25}, m5hal::i2s::Dout{22}};
    m5hal::i2s::IBusConfig rx_cfg{m5hal::i2s::Bclk{26}, m5hal::i2s::Ws{25}, m5hal::i2s::Din{21}};
    m5hal::i2s::IBusConfig duplex_cfg{m5hal::i2s::Bclk{26}, m5hal::i2s::Ws{25}, m5hal::i2s::Dout{22},
                                      m5hal::i2s::Din{21}};
    duplex_cfg.pin_mclk       = 0;
    duplex_cfg.tx_buffer_size = 4096;
    duplex_cfg.rx_buffer_size = 4096;
    duplex_cfg.role           = m5hal::i2s::IBusConfig::Role::Master;
    (void)tx_cfg.pin_dout;
    (void)rx_cfg.pin_din;
    (void)duplex_cfg.pin_bclk;
    (void)duplex_cfg.getBusKind();
    (void)m5hal::i2s::IBusConfig::Role::Slave;

    m5hal::i2s::AccessConfig acc_cfg;
    acc_cfg.sample_rate_hz   = 48000;
    acc_cfg.write_timeout_ms = 100;
    acc_cfg.read_timeout_ms  = 100;
    acc_cfg.bits_per_sample  = 16;
    acc_cfg.channels         = 2;
    (void)acc_cfg.getBusKind();
    (void)m5hal::i2s::Channel::None;
    (void)m5hal::i2s::Channel::Tx;
    (void)m5hal::i2s::Channel::Rx;
    (void)m5hal::i2s::Channel::TxRx;
    (void)m5hal::i2s::hasChannel(m5hal::i2s::Channel::TxRx, m5hal::i2s::Channel::Rx);

    m5hal::i2s::TxAccessor tx;
    m5hal::i2s::RxAccessor rx;
    m5hal::i2s::Accessor accessor;
    m5hal::i2s::TxAccessor tx_acc_cfg{acc_cfg};
    m5hal::i2s::RxAccessor rx_acc_cfg{acc_cfg};
    m5hal::i2s::Accessor accessor_cfg{acc_cfg};
    (void)tx;
    (void)rx;
    (void)accessor;
    (void)tx_acc_cfg;
    (void)rx_acc_cfg;
    (void)accessor_cfg;

    m5hal::i2s::IBus* bus = nullptr;
    uint8_t buf[8]        = {};
    m5hal::data::MemorySource src{buf, sizeof(buf)};
    m5hal::data::MemorySink dst{buf, sizeof(buf)};
    if (false) {
        (void)bus->write(&tx, acc_cfg, &src, sizeof(buf));
        (void)bus->writableBytes(&tx, acc_cfg);
        (void)bus->read(&rx, acc_cfg, &dst, sizeof(buf));
        (void)bus->readableBytes(&rx, acc_cfg);
        (void)bus->lockChannel(&tx, m5hal::i2s::Channel::Tx);
        (void)bus->unlockChannel(&tx, m5hal::i2s::Channel::Tx);

        (void)tx.bind(*bus);
        (void)tx.getConfig();
        (void)tx.getBus();
        (void)tx.setConfig(acc_cfg);
        (void)tx.beginAccess();
        (void)tx.endAccess();
        (void)tx.inAccess();
        (void)tx.write(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)tx.write(src, sizeof(buf));
        (void)tx.write(buf, sizeof(buf));
        (void)tx.writableBytes();

        (void)rx.bind(*bus);
        (void)rx.getConfig();
        (void)rx.getBus();
        (void)rx.setConfig(acc_cfg);
        (void)rx.beginAccess();
        (void)rx.endAccess();
        (void)rx.inAccess();
        (void)rx.read(m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)rx.read(dst, sizeof(buf));
        (void)rx.read(buf, sizeof(buf));
        (void)rx.readableBytes();

        (void)accessor.bind(*bus);
        (void)accessor.getConfig();
        (void)accessor.getBus();
        (void)accessor.tx();
        (void)accessor.rx();
        (void)accessor.setConfig(acc_cfg);
        (void)accessor.beginAccess();
        (void)accessor.endAccess();
        (void)accessor.inAccess();
        (void)accessor.write(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)accessor.write(src, sizeof(buf));
        (void)accessor.write(buf, sizeof(buf));
        (void)accessor.writableBytes();
        (void)accessor.read(m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)accessor.read(dst, sizeof(buf));
        (void)accessor.read(buf, sizeof(buf));
        (void)accessor.readableBytes();
    }
}

// ---------------------------------------------------------------------------
// GPIO types
// ---------------------------------------------------------------------------
static void checkGPIO()
{
    (void)sizeof(m5hal::gpio::IGPIO);
    (void)sizeof(m5hal::gpio::IPort);
    (void)sizeof(m5hal::gpio::GPIOGroup);
    (void)sizeof(m5hal::gpio::GPIOGroup::PortAccess);
    (void)sizeof(m5hal::gpio::Pin);

    m5hal::gpio::Pin pin;
    (void)pin.isValid();
    (void)pin.getPort();

    m5hal::types::gpio_number_t gpio_num = m5hal::types::makeGpioNumber(1, 2);
    (void)m5hal::types::extractSlot(gpio_num);
    (void)m5hal::types::extractLocalPin(gpio_num);
    (void)m5hal::types::GpioMode::Input;
    (void)m5hal::types::GpioMode::Output;
    (void)m5hal::types::GpioMode::OutputOpenDrain;
    (void)m5hal::types::GpioMode::InputPullup;
    (void)m5hal::types::GpioMode::InputPulldown;
    (void)m5hal::types::GpioMode::OutputOpenDrainPullup;
    (void)m5hal::types::gpio_mode_bits::output;
    (void)m5hal::types::gpio_mode_bits::open_drain;
    (void)m5hal::types::gpio_mode_bits::pull_up;
    (void)m5hal::types::gpio_mode_bits::pull_down;

    m5hal::gpio::GPIOGroup group;
    m5hal::gpio::IGPIO* gpio = nullptr;
    m5hal::gpio::IPort* port = nullptr;
    m5hal::gpio::GPIOGroup::PortAccess access{port, 0};
    (void)access.port;
    (void)access.deny_mask;

    if (false) {
        (void)group.addGPIO(gpio, 0);
        (void)group.removeGPIO(0);
        (void)group.getGPIO(0);
        (void)group.hasGPIO(0);
        (void)group.setDenyMask(0, 0, 0);
        (void)group.getPort(0, 0);
        (void)group.isValid(gpio_num);
        (void)group.tryGetPin(gpio_num);
        (void)group.getPin(gpio_num);

        (void)gpio->portForPin(0);
        (void)gpio->getPort(0);
        (void)gpio->getPinCount();
        (void)gpio->getPortCount();
        (void)gpio->isValid(0);
        (void)gpio->getPin(0);

        port->write(0, true);
        port->writeHigh(0);
        port->writeLow(0);
        (void)port->read(0);
        port->setMode(0, m5hal::types::gpio_mode_t::Output);
        (void)port->getPin(0);
        (void)port->readPort();
        port->writePort(0x01, 0x02);

        pin.write(true);
        pin.writeHigh();
        pin.writeLow();
        (void)pin.read();
        pin.setMode(m5hal::types::gpio_mode_t::Input);
        (void)pin.getLocalPin();
    }
}

// ---------------------------------------------------------------------------
// Data / memory types
// ---------------------------------------------------------------------------
static void checkData()
{
    (void)sizeof(m5hal::data::Source);
    (void)sizeof(m5hal::data::Sink);
    (void)sizeof(m5hal::data::MemorySource);
    (void)sizeof(m5hal::data::MemorySink);
    (void)sizeof(m5hal::data::LimitedSource);
    (void)sizeof(m5hal::data::LimitedSink);
    (void)sizeof(m5hal::data::RingFIFO);
    (void)sizeof(m5hal::data::ConstDataSpan);
    (void)sizeof(m5hal::data::DataSpan);
    (void)sizeof(m5hal::data::StreamReader);
    (void)sizeof(m5hal::data::StreamWriter);
    (void)sizeof(m5hal::data::StreamSource);
    (void)sizeof(m5hal::data::StreamSink);
    (void)sizeof(m5hal::data::MuxFrameEncoder);
    (void)sizeof(m5hal::data::MuxFrameDecoder);

    uint8_t buf[16] = {};
    m5hal::data::ConstDataSpan const_span{buf, sizeof(buf)};
    m5hal::data::DataSpan span{buf, sizeof(buf)};
    (void)const_span.empty();
    (void)const_span.begin();
    (void)const_span.end();
    (void)const_span.first(4);
    (void)const_span.subspan(1, 4);
    (void)span.empty();
    (void)span.begin();
    (void)span.end();
    (void)span.first(4);
    (void)span.subspan(1, 4);

    m5hal::data::MemorySource mem_src{const_span};
    m5hal::data::MemorySource mem_src_raw{buf, sizeof(buf)};
    m5hal::data::MemorySink mem_sink{span};
    m5hal::data::MemorySink mem_sink_raw{buf, sizeof(buf)};
    m5hal::data::LimitedSource limited_src{mem_src, 4};
    m5hal::data::LimitedSource null_limited_src{static_cast<m5hal::data::Source*>(nullptr), 0};
    m5hal::data::LimitedSink limited_sink{mem_sink, 4};
    m5hal::data::LimitedSink null_limited_sink{static_cast<m5hal::data::Sink*>(nullptr), 0};
    m5hal::data::RingFIFO ring{buf, sizeof(buf)};
    m5hal::data::StreamSource stream_src{static_cast<m5hal::data::StreamReader*>(nullptr), span};
    m5hal::data::StreamSink stream_sink{static_cast<m5hal::data::StreamWriter*>(nullptr), span};
    (void)mem_src_raw;
    (void)mem_sink_raw;
    (void)null_limited_src;
    (void)null_limited_sink;
    (void)stream_src;
    (void)stream_sink;

    (void)mem_src.peek(4);
    (void)mem_src.advance(1);
    (void)mem_src.eof();
    (void)mem_src.closed();
    (void)mem_sink.reserve(4);
    (void)mem_sink.commit(1);
    (void)mem_sink.closed();
    (void)mem_sink.written();
    (void)mem_sink.capacity();
    (void)mem_sink.remaining();
    (void)limited_src.peek(4);
    (void)limited_src.advance(1);
    (void)limited_src.eof();
    (void)limited_src.closed();
    (void)limited_src.remaining();
    (void)limited_sink.reserve(4);
    (void)limited_sink.commit(1);
    (void)limited_sink.closed();
    (void)limited_sink.remaining();
    (void)ring.buffered();
    (void)ring.free();
    (void)ring.capacity();
    ring.reset();
    ring.setBuf(buf, sizeof(buf));
    (void)ring.source();
    (void)ring.sink();

    m5hal::data::MuxFrameEncoder mux_encoder{m5hal::memory::defaultAllocator()};
    m5hal::data::MuxFrameDecoder mux_decoder{m5hal::memory::defaultAllocator()};
    (void)mux_encoder.attach(1, mem_src);
    (void)mux_encoder.stream(1);
    (void)mux_encoder.pump();
    (void)mux_encoder.writeFrame(m5hal::frame::Kind::Ping, 0, {});
    (void)mux_encoder.writeDelimiter();
    (void)mux_encoder.output();
    mux_encoder.detach(1);
    mux_encoder.releaseAll();
    mux_decoder.setFrameHandler(nullptr, nullptr);
    (void)mux_decoder.createStream(1, buf, sizeof(buf));
    (void)mux_decoder.source(1);
    (void)mux_decoder.setSink(2, mem_sink);
    mux_decoder.clearSink(2);
    (void)mux_decoder.pump(mem_src);
    mux_decoder.destroyStream(1);
    mux_decoder.releaseAll();
    (void)m5hal::data::MuxFrameEncoder::kMaxStreams;
    (void)m5hal::data::MuxFrameDecoder::kMaxStreams;
}

// ---------------------------------------------------------------------------
// Bytecode types
// ---------------------------------------------------------------------------
static void checkBytecode()
{
    (void)sizeof(m5hal::bytecode::BytecodeEncoder);
    (void)sizeof(m5hal::bytecode::BytecodeRunner);
    (void)sizeof(m5hal::bytecode::LenVar);
    (void)sizeof(m5hal::bytecode::detail::RunnerBusOps);
    (void)sizeof(m5hal::bytecode::detail::RunnerBusBinding);

    (void)m5hal::bytecode::OpCode::DelayMs;
    (void)m5hal::bytecode::OpCode::BusConfigure;
    (void)m5hal::bytecode::OpCode::BusTransfer;
    (void)m5hal::bytecode::OpCode::GpioSetMode;
    (void)m5hal::bytecode::OpCode::GpioWriteHigh;
    (void)m5hal::bytecode::OpCode::GpioWriteLow;
    (void)m5hal::bytecode::OpCode::GpioRead;
    (void)m5hal::bytecode::OpCode::GpioSubscribe;
    (void)m5hal::bytecode::OpCode::GpioUnsubscribe;
    (void)m5hal::bytecode::OpCode::GpioAllowlist;
    (void)m5hal::bytecode::OpCode::GpioPortRead;
    (void)m5hal::bytecode::OpCode::GpioPortWrite;
    (void)m5hal::bytecode::OpCode::StoreData;
    (void)m5hal::bytecode::OpCode::ReportError;
    (void)m5hal::bytecode::OpCode::ReportComplete;
    (void)m5hal::bytecode::OpCode::EvtGpioState;
    (void)m5hal::bytecode::OpCode::BusCreate;
    (void)m5hal::bytecode::OpCode::BusRelease;
    (void)m5hal::bytecode::OpCode::BusBeginTransaction;
    (void)m5hal::bytecode::OpCode::BusEndTransaction;

    (void)m5hal::bytecode::kCriticalOpcodeBit;
    (void)m5hal::bytecode::kDiscardStoreId;
    (void)m5hal::bytecode::kMaxStoreSlots;
    (void)m5hal::bytecode::kMaxBusBindings;
    (void)m5hal::bytecode::kI2CConfigSize;
    (void)m5hal::bytecode::kSPIConfigSize;
    (void)m5hal::bytecode::kUARTConfigSize;
    (void)m5hal::bytecode::kI2SConfigSize;
    (void)m5hal::bytecode::kI2CConfigWireTimeoutOffset;
    (void)m5hal::bytecode::kUARTConfigFirstByteTimeoutOffset;
    (void)m5hal::bytecode::kUARTConfigInterByteTimeoutOffset;
    (void)m5hal::bytecode::kUARTConfigWriteTimeoutOffset;
    (void)m5hal::bytecode::kI2SConfigWriteTimeoutOffset;
    (void)m5hal::bytecode::kI2SConfigReadTimeoutOffset;

    uint8_t buf[256] = {};
    m5hal::data::MemorySink sink{buf, sizeof(buf)};
    m5hal::bytecode::BytecodeEncoder enc{sink};
    m5hal::bytecode::BytecodeRunner runner;
    m5hal::bytecode::LenVar len = m5hal::bytecode::decodeLenVar(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
    size_t encoded              = m5hal::bytecode::encodeLenVar(buf, 12);
    (void)len.value;
    (void)len.consumed;
    (void)len.valid;
    (void)m5hal::bytecode::lenVarSize(12);
    (void)encoded;

    m5hal::i2c::MasterAccessConfig i2c_cfg;
    m5hal::spi::MasterAccessConfig spi_cfg;
    m5hal::uart::AccessConfig uart_cfg;
    m5hal::i2s::AccessConfig i2s_cfg;
    m5hal::i2c::TransferDesc i2c_desc;
    m5hal::spi::TransferDesc spi_desc;
    m5hal::types::gpio_number_t pins[2] = {1, 2};
    bool levels[2]                      = {false, true};
    if (false) {
        (void)enc.delayMs(1);
        (void)enc.configure(0, i2c_cfg);
        (void)enc.configure(0, spi_cfg);
        (void)enc.configure(0, uart_cfg);
        (void)enc.i2sConfig(0, i2s_cfg);
        (void)enc.transfer(0, i2c_desc, m5hal::data::ConstDataSpan{buf, 1}, 1);
        (void)enc.transfer(0, spi_desc, m5hal::data::ConstDataSpan{buf, 1}, 1);
        (void)enc.uartTransfer(0, m5hal::data::ConstDataSpan{buf, 1}, 1);
        (void)enc.gpioSetMode(m5hal::types::gpio_mode_t::Output, pins, 2);
        (void)enc.gpioWriteHigh(pins, 2);
        (void)enc.gpioWriteLow(pins, 2);
        (void)enc.gpioRead(0, pins, 2);
        (void)enc.gpioPortRead(0, 0, 0);
        (void)enc.gpioPortWrite(0, 0, 1, 2);
        (void)enc.gpioSubscribe(pins, 2);
        (void)enc.gpioUnsubscribe(pins, 2);
        (void)enc.evtGpioState(pins, levels, 2);
        (void)enc.busCreate(m5hal::types::bus_kind_t::I2C, 0, 0, m5hal::data::ConstDataSpan{buf, 2});
        (void)enc.busRelease(m5hal::types::bus_kind_t::I2C, 0);
        (void)enc.busBeginTransaction(m5hal::types::bus_kind_t::SPI, 0);
        (void)enc.busEndTransaction(m5hal::types::bus_kind_t::SPI, 0);
        (void)enc.storeData(0, m5hal::data::ConstDataSpan{buf, 1});
        (void)enc.reportError(m5hal::error::error_t::IO_ERROR, 0);
        (void)enc.reportComplete(m5hal::error::error_t::OK);
        (void)enc.end();

        (void)runner.run(m5hal::data::ConstDataSpan{buf, 0});
        (void)runner.runEvent(m5hal::data::ConstDataSpan{buf, 0});
        (void)runner.receiveOnly();
        runner.setReceiveOnly(true);
        (void)runner.storedData(0);
        (void)runner.storedCount();
        (void)runner.storedIdAt(0);
        runner.clearStored();
        (void)runner.statusReported();
        (void)runner.reportedStatus();
        (void)runner.reportedOffset();
        (void)runner.lastOffset();
        (void)runner.unknownSkipped();
        (void)runner.writeResponse(sink, m5hal::error::error_t::OK);
    }
}

// ---------------------------------------------------------------------------
// Frame types
// ---------------------------------------------------------------------------
static void checkFrame()
{
    (void)sizeof(m5hal::frame::View);
    (void)sizeof(m5hal::frame::DecodeResult);
    (void)sizeof(m5hal::frame::FrameReader);
    (void)sizeof(m5hal::frame::FrameWriter);

    (void)m5hal::frame::Kind::Padding;
    (void)m5hal::frame::Kind::Data;
    (void)m5hal::frame::Kind::Credit;
    (void)m5hal::frame::Kind::Checkpoint;
    (void)m5hal::frame::Kind::Control;
    (void)m5hal::frame::Kind::Request;
    (void)m5hal::frame::Kind::Response;
    (void)m5hal::frame::Kind::HelloReq;
    (void)m5hal::frame::Kind::HelloResp;
    (void)m5hal::frame::Kind::Ping;
    (void)m5hal::frame::Kind::Pong;
    (void)m5hal::frame::Kind::Event;
    (void)m5hal::frame::Kind::Delimiter;
    (void)m5hal::frame::DecodeStatus::Ok;
    (void)m5hal::frame::DecodeStatus::NeedMore;
    (void)m5hal::frame::DecodeStatus::Padding;
    (void)m5hal::frame::DecodeStatus::InvalidPrefix;
    (void)m5hal::frame::DecodeStatus::InvalidSize;
    (void)m5hal::frame::DecodeStatus::InvalidCheck;
    (void)m5hal::frame::kPrefixSize;
    (void)m5hal::frame::kCheckSize;
    (void)m5hal::frame::kHeaderSize;
    (void)m5hal::frame::kPayloadOffset;
    (void)m5hal::frame::kB3Offset;
    (void)m5hal::frame::kMaxPayload;
    (void)m5hal::frame::kMaxLen;
    (void)m5hal::frame::kMaxFrameSize;
    (void)m5hal::frame::kMaxDataPayload;
    (void)m5hal::frame::kMinCheckedLen;
    (void)m5hal::frame::isCheckedKind(m5hal::frame::Kind::Data);
    (void)m5hal::frame::isKnownKind(static_cast<uint8_t>(m5hal::frame::Kind::Data));
    (void)m5hal::frame::checkedFrameWireSize(1);

    uint8_t buf[300] = {};
    m5hal::data::DataSpan dst{buf, sizeof(buf)};
    m5hal::data::ConstDataSpan body{buf, 1};
    m5hal::frame::View view;
    m5hal::frame::DecodeResult decoded;
    decoded.status   = m5hal::frame::DecodeStatus::NeedMore;
    decoded.consumed = 0;
    (void)view.kind;
    (void)view.b3;
    (void)view.check;
    (void)view.payload;
    (void)view.has_check;
    (void)decoded;
    (void)m5hal::frame::isDelimiter(body);
    (void)m5hal::frame::crc8AtmUpdate(0, 0);
    (void)m5hal::frame::check8(2, static_cast<uint8_t>(m5hal::frame::Kind::Data));
    (void)m5hal::frame::decode(body, view);
    (void)m5hal::frame::encodeDelimiter(dst);
    (void)m5hal::frame::encodeChecked(dst, m5hal::frame::Kind::Data, 0, body);
    (void)m5hal::frame::encodeData(dst, 0, body);

    m5hal::data::MemorySource src{body};
    m5hal::data::MemorySink sink{dst};
    m5hal::frame::FrameReader reader{src};
    m5hal::frame::FrameWriter writer{sink};
    if (false) {
        (void)reader.next(view);
        (void)writer.writeDelimiter();
        (void)writer.writeChecked(m5hal::frame::Kind::Data, 0, body);
        (void)writer.writeData(0, body);
    }
}

// ---------------------------------------------------------------------------
// Remote types
// ---------------------------------------------------------------------------
static void checkRemote()
{
    (void)sizeof(m5hal::remote::Server);
    (void)sizeof(m5hal::remote::Server::Config);
    (void)sizeof(m5hal::remote::RemoteSession);
    (void)sizeof(m5hal::remote::RemoteSession::Config);
    (void)sizeof(m5hal::remote::RemoteServerAdapter);
    (void)sizeof(m5hal::remote::DeviceConfig);
    (void)sizeof(m5hal::remote::Capabilities);
    (void)sizeof(m5hal::remote::Capabilities::BusEntry);
    (void)sizeof(m5hal::remote::RemoteWireService);

    (void)m5hal::remote::kProtocolVersion;
    (void)m5hal::remote::kMaxScriptSize;
    (void)m5hal::remote::kMaxTransferRx;
    (void)m5hal::remote::kDefaultStoreId;
    (void)m5hal::remote::kRemoteUartTimeoutMarginMs;
    (void)m5hal::remote::mapRemoteError(0);

    m5hal::remote::DeviceConfig dev_cfg;
    dev_cfg.baud_rate           = 3000000;
    dev_cfg.response_timeout_ms = 2000;
    dev_cfg.on_progress         = nullptr;
    dev_cfg.progress_ctx        = nullptr;
    (void)dev_cfg;

    m5hal::remote::Capabilities caps;
    caps.proto_ver           = m5hal::remote::kProtocolVersion;
    caps.has_gpio            = true;
    caps.supports_bus_create = true;
    caps.bus_count           = 0;
    caps.gpio_port_count     = 1;
    caps.gpio_pin_count      = 40;
    caps.buses[0].kind       = m5hal::types::bus_kind_t::I2C;
    caps.buses[0].bus_id     = 0;
    (void)caps;

    m5hal::remote::Server::Config server_cfg;
    server_cfg.max_delay_ms       = 10;
    server_cfg.max_bus_timeout_ms = 100;
    server_cfg.max_transfer_rx    = m5hal::remote::kMaxTransferRx;
    (void)server_cfg;

    m5hal::remote::RemoteSession::Config session_cfg;
    session_cfg.response_timeout_ms = 1000;
    (void)session_cfg;
}

// ---------------------------------------------------------------------------
// Memory allocator
// ---------------------------------------------------------------------------
static void checkMemory()
{
    (void)sizeof(m5hal::memory::usage_t);
    (void)sizeof(m5hal::memory::Allocator);
    (void)sizeof(m5hal::memory::TempBuffer);
    (void)m5hal::memory::usage_t::Temp;
    (void)m5hal::memory::usage_t::Persistent;
    (void)m5hal::memory::usage_t::PersistentSlow;

    m5hal::memory::Allocator allocator;
    (void)allocator.usedBlocks();
    (void)allocator.largestFreeRun();
    (void)m5hal::memory::Allocator::tempBlockSize();
    (void)m5hal::memory::Allocator::tempBlockCount();
    (void)m5hal::memory::Allocator::tempPoolSize();

    m5hal::memory::TempBuffer empty;
    (void)empty.data();
    (void)empty.size();
    (void)static_cast<bool>(empty);
    (void)empty.release();
    empty.reset();

    if (false) {
        void* ptr = allocator.allocate(16, m5hal::memory::usage_t::Temp);
        (void)allocator.reallocate(ptr, 16, 32, m5hal::memory::usage_t::Persistent);
        allocator.deallocate(ptr);
        allocator.setFallback(nullptr, nullptr);
        allocator.setFallback(nullptr, nullptr, nullptr);

        m5hal::memory::TempBuffer temp{allocator, 16};
        (void)temp.reallocate(32);
        temp.reset();
        (void)temp.release();

        (void)m5hal::memory::defaultAllocator();
        (void)m5hal::memory::alloc_temp(16);
        (void)m5hal::memory::alloc(16);
        (void)m5hal::memory::alloc_psram(16);
        m5hal::memory::free(nullptr);
    }
}

// ---------------------------------------------------------------------------
// Service types
// ---------------------------------------------------------------------------
static void checkService()
{
    (void)sizeof(m5hal::service::ServiceResult);
    (void)sizeof(m5hal::service::ServicePoll);
    (void)sizeof(m5hal::service::ServiceContext);
    (void)sizeof(m5hal::service::IService);
    (void)sizeof(m5hal::service::ServiceRunner);
    (void)sizeof(m5hal::service::tick_nsec_t);
    (void)sizeof(m5hal::service::fast_tick_t);

    (void)m5hal::service::ServiceResult::Idle;
    (void)m5hal::service::ServiceResult::Progress;
    (void)m5hal::service::ServiceResult::Done;
    (void)m5hal::service::ServiceResult::Error;
    (void)m5hal::service::kMaxComparableDelayTicks;
    (void)m5hal::service::elapsedTicks(10, 5);
    (void)m5hal::service::hasReached(10, 5);
    (void)m5hal::service::fastTickToNsec(10, 1000000);
    (void)m5hal::service::nsecToFastTickCeil(1000, 1000000);
    (void)m5hal::service::fastTickFrequencyHz();
    (void)m5hal::service::defaultNowTick();
#if !M5HAL_PC_BUILD
    (void)m5hal::service::fastTick();
#endif

    m5hal::service::ServiceContext ctx;
    ctx.now_tick = 1;
    m5hal::service::ServicePoll idle;
    m5hal::service::ServicePoll progress{m5hal::service::ServiceResult::Progress};
    m5hal::service::ServicePoll due{m5hal::service::ServiceResult::Idle, 100};
    (void)idle.result;
    (void)idle.next_due;
    (void)progress;
    (void)due;
    (void)(idle == m5hal::service::ServiceResult::Idle);
    (void)(idle != m5hal::service::ServiceResult::Done);

    m5hal::service::ServiceRunner runner;
    (void)runner.size();
    (void)runner.capacity();
    (void)runner.autoRunActive();
    if (false) {
        (void)runner.runOnce(ctx);
        (void)runner.runOnce(ctx.now_tick);
        (void)runner.runOnce();
        (void)runner.startAutoRun();
        runner.stopAutoRun();
        runner.clear();
    }
}

// ---------------------------------------------------------------------------
// Slave bus types
// ---------------------------------------------------------------------------
static void checkSlaveTypes()
{
    (void)sizeof(m5hal::i2c::TxUnderrun);
    (void)sizeof(m5hal::i2c::SlaveBusConfig);
    (void)sizeof(m5hal::i2c::SlaveLineDriver);
    (void)sizeof(m5hal::i2c::ISlaveBus);
    (void)sizeof(m5hal::i2c::SlaveStreamAccessor);
    (void)sizeof(m5hal::i2c::SlaveRegMapAccessor);
    (void)sizeof(m5hal::spi::SlaveBusConfig);
    (void)sizeof(m5hal::spi::ISlaveBus);
    (void)sizeof(m5hal::spi::SpiSlaveAccessor);

    m5hal::i2c::SlaveBusConfig i2c_cfg;
    i2c_cfg.pin_scl            = 22;
    i2c_cfg.pin_sda            = 21;
    i2c_cfg.address            = 0x42;
    i2c_cfg.address_is_10bit   = false;
    i2c_cfg.timeout_ms         = 1000;
    i2c_cfg.tx_underrun        = m5hal::i2c::TxUnderrun::Fill;
    i2c_cfg.tx_fill_byte       = 0xFF;
    i2c_cfg.stretch_timeout_ms = 100;
    (void)i2c_cfg.getBusKind();
    (void)m5hal::i2c::TxUnderrun::Stretch;

    m5hal::spi::SlaveBusConfig spi_cfg;
    spi_cfg.pin_clk      = 18;
    spi_cfg.pin_mosi     = 23;
    spi_cfg.pin_miso     = 19;
    spi_cfg.pin_cs       = 5;
    spi_cfg.spi_mode     = 0;
    spi_cfg.spi_order    = 0;
    spi_cfg.host         = -1;
    spi_cfg.tx_fill_byte = 0;
    spi_cfg.timeout_ms   = m5hal::types::TIMEOUT_FOREVER;
    (void)spi_cfg.getBusKind();

    m5hal::i2c::ISlaveBus* i2c_bus = nullptr;
    m5hal::spi::ISlaveBus* spi_bus = nullptr;
    uint8_t buf[8]                 = {};
    m5hal::data::MemorySource src{buf, sizeof(buf)};
    m5hal::data::MemorySink dst{buf, sizeof(buf)};
    if (false) {
        (void)i2c_bus->init(i2c_cfg);
        (void)i2c_bus->beginTransaction(nullptr);
        (void)i2c_bus->endTransaction(nullptr);
        (void)i2c_bus->read(nullptr, m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)i2c_bus->write(nullptr, m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)i2c_bus->readableBytes(nullptr);
        (void)i2c_bus->transactionComplete(nullptr);
        (void)i2c_bus->service();
        (void)i2c_bus->waitForActivity(nullptr, 1);

        m5hal::i2c::SlaveStreamAccessor i2c_acc{*i2c_bus};
        (void)i2c_acc.getConfig();
        (void)i2c_acc.getBus();
        (void)i2c_acc.beginTransaction();
        (void)i2c_acc.endTransaction();
        (void)i2c_acc.read(m5hal::data::DataSpan{buf, sizeof(buf)});
        (void)i2c_acc.write(m5hal::data::ConstDataSpan{buf, sizeof(buf)});
        (void)i2c_acc.readableBytes();
        (void)i2c_acc.transactionComplete();
        (void)i2c_acc.waitForActivity(1);
        (void)i2c_acc.serve(&src, &dst, 1);

        m5hal::i2c::SlaveRegMapAccessor reg_map{*i2c_bus, m5hal::data::DataSpan{buf, sizeof(buf)}};
        (void)reg_map.getRegister(0);
        reg_map.setRegister(0, 1);
        (void)reg_map.pointer();
        reg_map.setOnRead(nullptr, nullptr);
        reg_map.setOnWrite(nullptr, nullptr);
        (void)reg_map.serve(1);

        (void)spi_bus->init(spi_cfg);
        (void)spi_bus->release();
        (void)spi_bus->serve(nullptr, &src, &dst, sizeof(buf), 1);

        m5hal::spi::SpiSlaveAccessor spi_acc{*spi_bus};
        (void)spi_acc.getConfig();
        (void)spi_acc.getBus();
        (void)spi_acc.serve(&src, &dst, sizeof(buf), 1);
        (void)spi_acc.serve(m5hal::data::ConstDataSpan{buf, sizeof(buf)}, m5hal::data::DataSpan{buf, sizeof(buf)}, 1);
    }
}

// ---------------------------------------------------------------------------
// Hal facade
// ---------------------------------------------------------------------------
static void checkHal()
{
    (void)sizeof(m5hal::Hal);
    (void)sizeof(m5hal::M5HALCore);

    m5hal::Hal hal;
    (void)hal.Gpio;
    (void)hal.Services;
    (void)hal.Memory;
    (void)hal.I2C;
    (void)hal.SPI;
    (void)hal.UART;
    (void)hal.I2S;
    (void)hal.backend();
    (void)hal.isConnected();
    (void)hal.remoteGpioSlot();
    (void)hal.hasRemoteGpio();
    (void)m5hal::getM5_Hal();
    (void)m5hal::M5_Hal;

    m5hal::remote::DeviceConfig remote_cfg;
    if (false) {
        (void)hal.connect(nullptr);
        (void)hal.init();
        (void)hal.initUart(nullptr, remote_cfg);
        (void)hal.initTcp(nullptr, remote_cfg);
        (void)hal.session();
        (void)hal.capabilities();
    }
}

// ---------------------------------------------------------------------------
// Entry point (not intended to run — compile check only)
// ---------------------------------------------------------------------------
#if defined(ARDUINO)
void setup()
{
    checkErrorAndResult();
    checkBusBase();
    checkAllocationAndBusViews();
    checkHal();
    checkI2C();
    checkSPI();
    checkUART();
    checkI2S();
    checkGPIO();
    checkData();
    checkBytecode();
    checkFrame();
    checkRemote();
    checkMemory();
    checkService();
    checkSlaveTypes();
}
void loop()
{
}
#elif defined(ESP_PLATFORM)
extern "C" void app_main()
{
    checkErrorAndResult();
    checkBusBase();
    checkAllocationAndBusViews();
    checkHal();
    checkI2C();
    checkSPI();
    checkUART();
    checkI2S();
    checkGPIO();
    checkData();
    checkBytecode();
    checkFrame();
    checkRemote();
    checkMemory();
    checkService();
    checkSlaveTypes();
}
#else
int main()
{
    checkErrorAndResult();
    checkBusBase();
    checkAllocationAndBusViews();
    checkHal();
    checkI2C();
    checkSPI();
    checkUART();
    checkI2S();
    checkGPIO();
    checkData();
    checkBytecode();
    checkFrame();
    checkRemote();
    checkMemory();
    checkService();
    checkSlaveTypes();
    return 0;
}
#endif
