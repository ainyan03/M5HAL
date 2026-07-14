// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

using ::m5::hal::v2::result_t;

namespace {

namespace spi     = m5::hal::v2::spi;
namespace bus     = m5::hal::v2::bus;
namespace data    = m5::hal::v2::data;
namespace error   = m5::hal::v2::error;
namespace gpio    = m5::hal::v2::gpio;
namespace service = m5::hal::v2::service;
namespace types   = m5::hal::v2::types;

class StubIBus : public spi::IBus {
public:
    struct Call {
        const bus::IAccessor* owner = nullptr;
        spi::MasterAccessConfig cfg;
        spi::TransferDesc desc;
        std::vector<uint8_t> tx;
        size_t rx_len = 0;
    };

    // Typed init: the fake adds no fields, so it takes the
    // abstract kind config.
    result_t<void> init(const spi::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    result_t<void> beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override
    {
        if (fail_begin_transaction) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::NOT_IMPLEMENTED);
        }
        transaction_owners.push_back(owner);
        transaction_cfgs.push_back(cfg);
        ++begin_transaction_count;
        return {};
    }

    bool fail_begin_transaction = false;

    result_t<void> endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg) override
    {
        transaction_owners.push_back(owner);
        transaction_cfgs.push_back(cfg);
        ++end_transaction_count;
        return {};
    }

    bool fail_transfer = false;

    result_t<void> transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg, const spi::TransferDesc& desc,
                            data::Source* tx, size_t tx_len, data::Sink* rx, size_t rx_len) override
    {
        if (fail_transfer) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::IO_ERROR);
        }
        Call call;
        call.owner = owner;
        call.cfg   = cfg;
        call.desc  = desc;
        last_totals.clear();

        size_t tx_remaining = tx_len;
        if (tx != nullptr && tx_remaining > 0) {
            while (!tx->eof() && tx_remaining > 0) {
                auto span = tx->peek(std::min<size_t>(16, tx_remaining));
                if (!span.has_value()) {
                    return m5::stl::make_unexpected(span.error());
                }
                if (span->size == 0) {
                    break;
                }
                call.tx.insert(call.tx.end(), span->data, span->data + span->size);
                auto adv = tx->advance(span->size);
                if (!adv.has_value()) {
                    return m5::stl::make_unexpected(adv.error());
                }
                tx_remaining -= span->size;
                last_totals.tx += span->size;
            }
        }

        size_t rx_remaining = rx_len;
        if (rx != nullptr && rx_remaining > 0) {
            uint8_t next = rx_seed;
            while (!rx->closed() && rx_remaining > 0) {
                auto span = rx->reserve(std::min<size_t>(16, rx_remaining));
                if (!span.has_value()) {
                    return m5::stl::make_unexpected(span.error());
                }
                if (span->size == 0) {
                    break;
                }
                for (size_t i = 0; i < span->size; ++i) {
                    span->data[i] = next++;
                }
                call.rx_len += span->size;
                auto commit = rx->commit(span->size);
                if (!commit.has_value()) {
                    return m5::stl::make_unexpected(commit.error());
                }
                rx_remaining -= span->size;
                last_totals.rx += span->size;
            }
        }

        calls.push_back(call);
        has_wait_totals = true;
        return {};
    }

    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor*, const spi::MasterAccessConfig&) override
    {
        if (fail_wait) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::IO_ERROR);
        }
        auto totals = last_totals;
        last_totals.clear();
        has_wait_totals = false;
        return totals;
    }

    bool transferBusy(bus::IAccessor*) override
    {
        return busy;
    }

    bool fail_wait = false;
    bool busy      = false;

    std::vector<Call> calls;
    std::vector<const bus::IAccessor*> transaction_owners;
    std::vector<spi::MasterAccessConfig> transaction_cfgs;
    size_t begin_transaction_count = 0;
    size_t end_transaction_count   = 0;
    uint8_t rx_seed                = 0x40;
    bus::TransferTotals last_totals{};
    bool has_wait_totals = false;
};

class RecordingPort : public gpio::IPort {
public:
    enum class Kind : uint8_t { Write, Read, SetMode };
    struct Event {
        types::gpio_number_t gpio_num = -1;
        Kind kind                     = Kind::Write;
        bool value                    = false;
        types::gpio_mode_t mode       = types::gpio_mode_t::Input;
    };

    void clear()
    {
        events.clear();
    }
    void setReadValue(bool value)
    {
        read_value = value;
    }

    std::vector<Event> events;
    bool read_value = false;

protected:
    void _writePinEncoded(uint32_t encoded_num, bool v) override
    {
        events.push_back({static_cast<types::gpio_number_t>(encoded_num), Kind::Write, v, types::gpio_mode_t::Input});
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        events.push_back(
            {static_cast<types::gpio_number_t>(encoded_num), Kind::Read, read_value, types::gpio_mode_t::Input});
        return read_value;
    }
    void _setPinModeEncoded(uint32_t encoded_num, types::gpio_mode_t mode) override
    {
        events.push_back({static_cast<types::gpio_number_t>(encoded_num), Kind::SetMode, false, mode});
    }
    types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(types::gpio_local_pin_t pin_index) const override
    {
        return pin_index;
    }
};

class RecordingGPIO : public gpio::IGPIO {
public:
    gpio::IPort* portForPin(types::gpio_local_pin_t local_pin) const override
    {
        (void)local_pin;
        return &_port;
    }
    gpio::IPort* getPort(uint8_t port_index) const override
    {
        (void)port_index;
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return 16;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }

    mutable RecordingPort _port;
};

class ScopedServiceRunnerClear {
public:
    ScopedServiceRunnerClear()
    {
        m5::hal::v2::M5_Hal.Services.clear();
    }
    ~ScopedServiceRunnerClear()
    {
        m5::hal::v2::M5_Hal.Services.clear();
    }
};

class IdleService : public service::IService {
public:
    service::ServicePoll serviceImpl(const service::ServiceContext&) override
    {
        return service::ServiceResult::Idle;
    }
};

std::vector<std::unique_ptr<IdleService>> fillGlobalServiceRunner()
{
    std::vector<std::unique_ptr<IdleService>> services;
    services.reserve(service::ServiceRunner::kMaxServices);
    for (size_t i = 0; i < service::ServiceRunner::kMaxServices; ++i) {
        services.emplace_back(new IdleService());
        EXPECT_TRUE(m5::hal::v2::M5_Hal.Services.add(*services.back())) << i;
    }
    return services;
}

RecordingGPIO& softwareSpiGPIO()
{
    static RecordingGPIO gpio;
    static bool registered = [] {
        auto r = m5::hal::v2::M5_Hal.Gpio.addGPIO(&gpio, 42);
        (void)r;
        return true;
    }();
    (void)registered;
    return gpio;
}

types::gpio_number_t softPin(uint8_t local)
{
    return types::makeGpioNumber(42, local);
}

size_t countWrites(const std::vector<RecordingPort::Event>& events, types::gpio_number_t pin, bool value)
{
    const auto local = static_cast<types::gpio_number_t>(types::extractLocalPin(pin));
    return static_cast<size_t>(std::count_if(events.begin(), events.end(), [local, value](const auto& event) {
        return event.kind == RecordingPort::Kind::Write && event.gpio_num == local && event.value == value;
    }));
}

size_t firstWriteIndex(const std::vector<RecordingPort::Event>& events, types::gpio_number_t pin, bool value)
{
    const auto local = static_cast<types::gpio_number_t>(types::extractLocalPin(pin));
    const auto it    = std::find_if(events.begin(), events.end(), [local, value](const auto& event) {
        return event.kind == RecordingPort::Kind::Write && event.gpio_num == local && event.value == value;
    });
    return static_cast<size_t>(std::distance(events.begin(), it));
}

}  // namespace

TEST(MasterAccessConfig, SetupWithDCPinSelectsPinModeAndCommandLength)
{
    spi::AccessConfig cfg;
    cfg.setupWithDCPin(27).pin_cs = 5;

    EXPECT_EQ(cfg.pin_dc, 27);
    EXPECT_EQ(cfg.spi_data_mode, spi::spi_data_mode_t::HalfDuplexWithDcPin);
    EXPECT_EQ(cfg.spi_command_length, 8);
    EXPECT_EQ(cfg.pin_cs, 5);  // the preset chains into further assignments
}

TEST(MasterAccessConfig, SetupWithDCBitSelectsBitModeAndClearsPin)
{
    spi::AccessConfig cfg;
    cfg.pin_dc = 27;  // a previously set pin must be cleared by the bit preset
    cfg.setupWithDCBit();

    EXPECT_EQ(cfg.pin_dc, -1);
    EXPECT_EQ(cfg.spi_data_mode, spi::spi_data_mode_t::HalfDuplexWithDcBit);
    EXPECT_EQ(cfg.spi_command_length, 8);
}

TEST(MasterAccessor, WriteWrapsTransferAndCopiesTx)
{
    StubIBus bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = 18;
    bus_cfg.pin_mosi = 23;
    ASSERT_TRUE(bus.init(bus_cfg).has_value());

    spi::MasterAccessConfig cfg;
    cfg.pin_cs = 5;
    cfg.freq   = 40000000;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x9F, 0x00, 0x01};
    auto result        = accessor.write(tx, sizeof(tx));

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].owner, &accessor);
    EXPECT_EQ(bus.calls[0].cfg.pin_cs, 5);
    EXPECT_EQ(bus.calls[0].cfg.freq, 40000000u);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0x9F, 0x00, 0x01}));
    EXPECT_EQ(bus.calls[0].rx_len, 0u);
}

TEST(MasterAccessor, ReadWrapsTransferAndFillsRx)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    uint8_t rx[4] = {};
    auto result   = accessor.read(rx, sizeof(rx));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(rx));
    EXPECT_EQ((std::vector<uint8_t>{rx, rx + sizeof(rx)}), (std::vector<uint8_t>{0x40, 0x41, 0x42, 0x43}));
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].rx_len, sizeof(rx));
}

TEST(MasterAccessor, WriteSourceLimitsTransferLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    const uint8_t bytes[] = {0x10, 0x11, 0x12, 0x13, 0x14};
    data::MemorySource src{data::ConstDataSpan{bytes, sizeof(bytes)}};

    auto result = accessor.write(src, 3);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0x10, 0x11, 0x12}));

    auto next = src.peek(8);
    ASSERT_TRUE(next.has_value());
    ASSERT_EQ(next->size, 2u);
    EXPECT_EQ(next->data[0], 0x13);
}

TEST(MasterAccessor, ReadSinkLimitsTransferLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    uint8_t rx[5] = {};
    data::MemorySink sink{data::DataSpan{rx, sizeof(rx)}};

    auto result = accessor.read(sink, 2);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 2u);
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].rx_len, 2u);
    EXPECT_EQ((std::vector<uint8_t>{rx, rx + sizeof(rx)}), (std::vector<uint8_t>{0x40, 0x41, 0, 0, 0}));
}

TEST(MasterAccessor, SetConfigRejectsWhileAccessIsHeld)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    ASSERT_TRUE(accessor.beginAccess().has_value());

    spi::MasterAccessConfig next;
    next.freq   = 2000000;
    auto result = accessor.setConfig(next);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error::error_t::INVALID_STATE);
    ASSERT_TRUE(accessor.endAccess().has_value());
    ASSERT_TRUE(accessor.setConfig(next).has_value());
    EXPECT_EQ(accessor.getConfig().freq, 2000000u);
}

TEST(MasterAccessor, TransferWrapsTransaction)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.pin_cs = 5;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x12};
    auto result        = accessor.write(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.begin_transaction_count, 1u);
    EXPECT_EQ(bus.end_transaction_count, 1u);
    ASSERT_EQ(bus.transaction_owners.size(), 2u);
    EXPECT_EQ(bus.transaction_owners[0], &accessor);
    EXPECT_EQ(bus.transaction_owners[1], &accessor);
    EXPECT_EQ(bus.transaction_cfgs[0].pin_cs, 5);
}

TEST(MasterAccessor, CoreTransferRequiresOpenTransaction)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    auto result = accessor.transfer(spi::TransferDesc{}, data::ConstDataSpan{}, data::DataSpan{});

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error::error_t::INVALID_STATE);
    EXPECT_TRUE(bus.calls.empty());
}

TEST(MasterAccessor, ExplicitTransactionSpansNestedTransfers)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    const uint8_t first[]  = {0x12};
    const uint8_t second[] = {0x34};
    EXPECT_TRUE(accessor.write(data::ConstDataSpan{first, sizeof(first)}).has_value());
    EXPECT_TRUE(accessor.write(data::ConstDataSpan{second, sizeof(second)}).has_value());
    auto totals = accessor.endTransaction();
    ASSERT_TRUE(totals.has_value());
    EXPECT_EQ(totals->tx, sizeof(first) + sizeof(second));
    EXPECT_EQ(totals->rx, size_t{0});

    ASSERT_EQ(bus.calls.size(), 2u);
    EXPECT_EQ(bus.begin_transaction_count, 1u);
    EXPECT_EQ(bus.end_transaction_count, 1u);
}

TEST(MasterAccessor, AsyncWaitFailurePoisonsTransaction)
{
    // Sticky-error contract, wire side: a segment failure surfacing via
    // waitTransfer() latches into the transaction — every later transfer
    // is rejected with the same error and endTransaction() reports it.
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    const uint8_t byte[] = {0x12};
    const data::ConstDataSpan tx{byte, sizeof(byte)};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    bus.busy = true;  // segment stays pending; its failure surfaces later
    ASSERT_TRUE(accessor.transfer(spi::TransferDesc{}, tx, data::DataSpan{}).has_value());
    ASSERT_EQ(bus.calls.size(), 1u);

    bus.fail_wait = true;  // the pending segment failed on the wire
    auto second   = accessor.transfer(spi::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(second.has_value());
    EXPECT_EQ(second.error(), error::error_t::IO_ERROR);
    EXPECT_EQ(bus.calls.size(), 1u);

    // The latch, not the bus, must reject from now on: were the third
    // attempt to reach the (now healthy) bus, calls would grow to 2.
    bus.fail_wait = false;
    auto third    = accessor.transfer(spi::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(third.has_value());
    EXPECT_EQ(third.error(), error::error_t::IO_ERROR);
    EXPECT_EQ(bus.calls.size(), 1u);

    auto ended = accessor.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::IO_ERROR);

    // A fresh transaction starts clean.
    bus.busy = false;
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    EXPECT_TRUE(accessor.transfer(spi::TransferDesc{}, tx, data::DataSpan{}).has_value());
    EXPECT_TRUE(accessor.endTransaction().has_value());
    EXPECT_EQ(bus.calls.size(), 2u);
}

TEST(MasterAccessor, SyncPreflightRejectionAlsoPoisonsTransaction)
{
    // Sticky-error contract, pre-flight side: transaction segments form
    // one logical operation, so even a synchronous rejection that never
    // touched the wire invalidates the rest of the transaction — later
    // transfers are rejected and endTransaction() reports the error.
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    const uint8_t byte[] = {0x12};
    const data::ConstDataSpan tx{byte, sizeof(byte)};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    bus.fail_transfer = true;
    auto first        = accessor.transfer(spi::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), error::error_t::IO_ERROR);
    EXPECT_TRUE(bus.calls.empty());

    // Even though the bus would now succeed, the transaction is poisoned.
    bus.fail_transfer = false;
    auto second       = accessor.transfer(spi::TransferDesc{}, tx, data::DataSpan{});
    ASSERT_FALSE(second.has_value());
    EXPECT_EQ(second.error(), error::error_t::IO_ERROR);
    EXPECT_TRUE(bus.calls.empty());  // the second segment never reached the bus

    auto ended = accessor.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::IO_ERROR);

    // A fresh transaction starts clean.
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    EXPECT_TRUE(accessor.transfer(spi::TransferDesc{}, tx, data::DataSpan{}).has_value());
    EXPECT_EQ(bus.calls.size(), 1u);
    EXPECT_TRUE(accessor.endTransaction().has_value());
}

TEST(MasterAccessor, WriteCommandDataSplitsDcLevel)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x2A, 0x00, 0xEF};
    auto result        = accessor.writeCommandData(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_TRUE(bus.calls[0].desc.dc_level_valid);
    EXPECT_TRUE(bus.calls[0].desc.dc_level);
    EXPECT_EQ(bus.calls[0].desc.command, 0x2Au);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].desc.command_dc_level, 0);
    EXPECT_EQ(bus.calls[0].desc.data_dc_level, 1);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0x00, 0xEF}));
}

TEST(MasterAccessor, WriteCommandDataCanTakeSeparateCommand)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x11, 0x22};
    auto result        = accessor.writeCommandData(0x2C, data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x2Cu);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0x11, 0x22}));
}

TEST(MasterAccessor, WriteCommandDataSourceLimitsTransferLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t bytes[] = {0xA0, 0xA1, 0xA2, 0xA3};
    data::MemorySource src{data::ConstDataSpan{bytes, sizeof(bytes)}};
    auto result = accessor.writeCommandData(0x2C, src, 2);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x2Cu);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0xA0, 0xA1}));
}

TEST(MasterAccessor, WriteCommandDataUsesWriteDummyCycle)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_read_dummy_cycle  = 9;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x01};
    auto result        = accessor.writeCommandData(0x2C, data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 4u);
}

TEST(MasterAccessor, ReadCommandDataUsesReadDummyCycle)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_read_dummy_cycle  = 8;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    uint8_t rx[1] = {};
    auto result   = accessor.readCommandData(0x0B, data::DataSpan{rx, sizeof(rx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 8u);
}

TEST(MasterAccessor, WriteCommandAddressDataBuildsSingleTransfer)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_address_length    = 24;
    cfg.spi_read_dummy_cycle  = 9;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0xDE, 0xAD};
    auto result        = accessor.writeCommandAddressData(0x02, 0x00123456, data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x02u);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].desc.address, 0x00123456u);
    EXPECT_EQ(bus.calls[0].desc.address_bytes, 3u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 4u);
    EXPECT_EQ(bus.calls[0].desc.command_dc_level, 0);
    EXPECT_EQ(bus.calls[0].desc.address_dc_level, 1);
    EXPECT_EQ(bus.calls[0].desc.data_dc_level, 1);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0xDE, 0xAD}));
}

TEST(MasterAccessor, WriteCommandAddressDataSourceLimitsTransferLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_address_length    = 24;
    cfg.spi_read_dummy_cycle  = 9;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0xA0, 0xA1, 0xA2};
    data::MemorySource src{data::ConstDataSpan{tx, sizeof(tx)}};
    auto result = accessor.writeCommandAddressData(0x02, 0x00123456, src, 2);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x02u);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].desc.address, 0x00123456u);
    EXPECT_EQ(bus.calls[0].desc.address_bytes, 3u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 4u);
    EXPECT_EQ(bus.calls[0].desc.data_dc_level, 1);
    EXPECT_EQ(bus.calls[0].tx, (std::vector<uint8_t>{0xA0, 0xA1}));
}

TEST(MasterAccessor, ReadCommandAddressDataBuildsSingleTransfer)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    cfg.spi_command_length    = 8;
    cfg.spi_address_length    = 16;
    cfg.spi_read_dummy_cycle  = 8;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};

    uint8_t rx[3] = {};
    auto result   = accessor.readCommandAddressData(0x0B, 0x1234, data::DataSpan{rx, sizeof(rx)});

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 3u);
    EXPECT_EQ((std::vector<uint8_t>{rx, rx + sizeof(rx)}), (std::vector<uint8_t>{0x40, 0x41, 0x42}));
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.command, 0x0Bu);
    EXPECT_EQ(bus.calls[0].desc.command_bytes, 1u);
    EXPECT_EQ(bus.calls[0].desc.address, 0x1234u);
    EXPECT_EQ(bus.calls[0].desc.address_bytes, 2u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 8u);
    EXPECT_EQ(bus.calls[0].rx_len, 3u);
}

TEST(MasterAccessor, SendDummyClockUsesTransferDesc)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessor accessor{bus, spi::MasterAccessConfig{}};
    auto result = accessor.sendDummyClock(12);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(bus.calls.size(), 1u);
    EXPECT_EQ(bus.calls[0].desc.dummy_cycles, 12);
    EXPECT_TRUE(bus.calls[0].tx.empty());
    EXPECT_EQ(bus.calls[0].rx_len, 0u);
}

// Regression: writeCommandData(span) must reject spi_command_length == 0
// rather than silently degrading to write(). Setting a command length is
// mandatory for the command-sugar family; callers that want a plain write
// must call write() directly.
TEST(MasterAccessor, WriteCommandDataSpanRejectsZeroCommandLength)
{
    StubIBus bus;
    ASSERT_TRUE(bus.init(spi::IBusConfig{}).has_value());

    spi::MasterAccessConfig cfg;
    // spi_command_length defaults to 0 — no command configured.
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0xDE, 0xAD};
    auto result        = accessor.writeCommandData(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error::error_t::INVALID_ARGUMENT);
    // Nothing must have been sent to the bus.
    EXPECT_TRUE(bus.calls.empty());
}

TEST(IBus, DefaultTransferReturnsNotImplemented)
{
    spi::IBus bus;
    spi::MasterAccessConfig cfg;
    spi::TransferDesc desc;
    auto result = bus.transfer(nullptr, cfg, desc, nullptr, 0, nullptr, 0);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), error::error_t::NOT_IMPLEMENTED);
}

// ---- ScopedTransaction (CS scope RAII) ---------------------------------

TEST(ScopedTransaction, ClosesTheTransactionOnScopeExit)
{
    StubIBus bus;
    spi::MasterAccessConfig cfg;
    spi::MasterAccessor dev{bus, cfg};

    {
        spi::ScopedTransaction scope{dev};
        ASSERT_FALSE(scope.has_error());
        EXPECT_TRUE(scope.ok());  // positive view == !has_error()
        EXPECT_EQ(scope.error(), m5::hal::v2::error::error_t::OK);
        EXPECT_EQ(bus.begin_transaction_count, 1u);
        EXPECT_EQ(bus.end_transaction_count, 0u);
    }  // scope exit = endTransaction, even on an early return
    EXPECT_EQ(bus.end_transaction_count, 1u);
}

TEST(ScopedTransaction, SurfacesABeginFailureWithoutClosing)
{
    StubIBus bus;
    bus.fail_begin_transaction = true;
    spi::MasterAccessConfig cfg;
    spi::MasterAccessor dev{bus, cfg};

    {
        spi::ScopedTransaction scope{dev};
        EXPECT_TRUE(scope.has_error());
        EXPECT_FALSE(scope.ok());  // positive view == !has_error()
        EXPECT_EQ(scope.error(), m5::hal::v2::error::error_t::NOT_IMPLEMENTED);
    }
    EXPECT_EQ(bus.end_transaction_count, 0u);  // nothing to close
}

TEST(SoftwareIBus, WriteDrivesCsDcClockAndMosi)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_dc   = softPin(1);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());

    spi::MasterAccessConfig cfg;
    cfg.pin_cs             = softPin(3);
    cfg.freq               = 20000000;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};
    port.clear();

    const uint8_t tx[] = {0xA5, 0x5A};
    auto result        = accessor.writeCommandData(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    EXPECT_GE(countWrites(port.events, softPin(0), true), 8u);
    EXPECT_GE(countWrites(port.events, softPin(0), false), 8u);
    EXPECT_GT(countWrites(port.events, softPin(2), true), 0u);
    EXPECT_GT(countWrites(port.events, softPin(2), false), 0u);
    EXPECT_EQ(countWrites(port.events, softPin(1), false), 1u);
    EXPECT_EQ(countWrites(port.events, softPin(1), true), 1u);
    EXPECT_EQ(countWrites(port.events, softPin(3), false), 1u);
    EXPECT_EQ(countWrites(port.events, softPin(3), true), 1u);
}

// Per-device D/C override: a non-negative MasterAccessConfig::pin_dc
// beats the bus-level pin_dc — the bus-level pin must stay untouched.
TEST(SoftwareIBus, AccessorPinDcOverridesBusPinDc)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_dc   = softPin(1);  // bus-level default D/C
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.pin_cs             = softPin(3);
    cfg.pin_dc             = softPin(4);  // device-level override
    cfg.freq               = 20000000;
    cfg.spi_command_length = 8;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0xA5, 0x5A};
    auto result        = accessor.writeCommandData(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    // The override pin carries the command/data D/C swing...
    EXPECT_EQ(countWrites(port.events, softPin(4), false), 1u);
    EXPECT_GE(countWrites(port.events, softPin(4), true), 1u);
    // ...and the bus-level D/C pin stays silent.
    EXPECT_EQ(countWrites(port.events, softPin(1), false), 0u);
    EXPECT_EQ(countWrites(port.events, softPin(1), true), 0u);
}

// Rejection boundary for the single-lane software variant: multi-lane
// modes remain unimplemented, while half-duplex TX and RX are sequential.
TEST(SoftwareIBus, UnimplementedDataModesAreRejected)
{
    auto& gpio = softwareSpiGPIO();
    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_miso = softPin(1);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    gpio._port.clear();

    const uint8_t tx[] = {0x01};
    uint8_t rx[1]      = {};

    // Multi-lane: rejected even for a one-directional write.
    {
        spi::MasterAccessConfig cfg;
        cfg.freq          = 20000000;
        cfg.spi_data_mode = spi::spi_data_mode_t::QuadOutput;
        spi::MasterAccessor accessor{bus, cfg};
        auto result = accessor.write(data::ConstDataSpan{tx, sizeof(tx)});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error(), error::error_t::NOT_IMPLEMENTED);
    }

    // Half-duplex carrying both tx and rx data: TX then RX.
    {
        gpio._port.setReadValue(true);
        spi::MasterAccessConfig cfg;
        cfg.freq          = 20000000;
        cfg.spi_data_mode = spi::spi_data_mode_t::HalfDuplex;
        spi::MasterAccessor accessor{bus, cfg};
        ASSERT_TRUE(accessor.beginTransaction().has_value());
        auto result =
            accessor.transfer(spi::TransferDesc{}, data::ConstDataSpan{tx, sizeof(tx)}, data::DataSpan{rx, sizeof(rx)});
        auto end = accessor.endTransaction();
        ASSERT_TRUE(result.has_value()) << "err=" << error::toString(result.error());
        ASSERT_TRUE(end.has_value()) << "err=" << error::toString(end.error());
        EXPECT_EQ(rx[0], 0xFF);
    }

    // Half-duplex one-directional write: still works.
    {
        spi::MasterAccessConfig cfg;
        cfg.freq          = 20000000;
        cfg.spi_data_mode = spi::spi_data_mode_t::HalfDuplexWithDcPin;
        spi::MasterAccessor accessor{bus, cfg};
        auto result = accessor.write(data::ConstDataSpan{tx, sizeof(tx)});
        EXPECT_TRUE(result.has_value());
    }
}

TEST(SoftwareIBus, MisoLessReadRequiresAndUsesHalfDuplexMosi)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;
    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());

    uint8_t rx[1]      = {};
    const uint8_t tx[] = {0xA5};

    spi::MasterAccessConfig full_cfg;
    full_cfg.freq = 20000000;
    spi::MasterAccessor full{bus, full_cfg};
    ASSERT_TRUE(full.beginTransaction().has_value());
    port.clear();
    auto rejected =
        full.transfer(spi::TransferDesc{}, data::ConstDataSpan{tx, sizeof(tx)}, data::DataSpan{rx, sizeof(rx)});
    EXPECT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error(), error::error_t::INVALID_STATE);
    EXPECT_EQ(countWrites(port.events, softPin(0), true), 0u);
    EXPECT_FALSE(full.endTransaction().has_value());

    spi::MasterAccessConfig half_cfg;
    half_cfg.freq          = 20000000;
    half_cfg.spi_data_mode = spi::spi_data_mode_t::HalfDuplex;
    spi::MasterAccessor half{bus, half_cfg};
    port.setReadValue(true);
    ASSERT_TRUE(half.beginTransaction().has_value());
    port.clear();
    auto transferred =
        half.transfer(spi::TransferDesc{}, data::ConstDataSpan{tx, sizeof(tx)}, data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(transferred.has_value()) << "err=" << error::toString(transferred.error());
    ASSERT_TRUE(half.endTransaction().has_value());
    EXPECT_EQ(rx[0], 0xFF);

    const auto input_mode = std::find_if(port.events.begin(), port.events.end(), [](const auto& event) {
        return event.gpio_num == 2 && event.kind == RecordingPort::Kind::SetMode &&
               event.mode == types::gpio_mode_t::Input;
    });
    ASSERT_NE(input_mode, port.events.end());
    const auto output_mode = std::find_if(input_mode + 1, port.events.end(), [](const auto& event) {
        return event.gpio_num == 2 && event.kind == RecordingPort::Kind::SetMode &&
               event.mode == types::gpio_mode_t::Output;
    });
    ASSERT_NE(output_mode, port.events.end());
    EXPECT_TRUE(std::any_of(port.events.begin(), input_mode, [](const auto& event) {
        return event.gpio_num == 2 && event.kind == RecordingPort::Kind::Write;
    }));
    EXPECT_TRUE(std::any_of(input_mode, output_mode, [](const auto& event) {
        return event.gpio_num == 2 && event.kind == RecordingPort::Kind::Read;
    }));
}

TEST(SoftwareIBus, ReadSamplesMiso)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_miso = softPin(4);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();
    port.setReadValue(true);

    spi::MasterAccessConfig cfg;
    cfg.freq = 20000000;
    spi::MasterAccessor accessor{bus, cfg};

    uint8_t rx[1] = {};
    auto result   = accessor.read(rx, sizeof(rx));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(rx));
    EXPECT_EQ(rx[0], 0xFF);
    EXPECT_EQ(countWrites(port.events, softPin(0), true), 8u);
    EXPECT_GE(countWrites(port.events, softPin(0), false), 8u);
}

TEST(SoftwareIBus, DummyClockCountIsCycleCount)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk = softPin(0);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.freq = 20000000;
    spi::MasterAccessor accessor{bus, cfg};

    auto result = accessor.sendDummyClock(16);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(countWrites(port.events, softPin(0), true), 16u);
    EXPECT_GE(countWrites(port.events, softPin(0), false), 16u);
}

TEST(SoftwareIBus, CommandAddressDataClockCountMatchesWireContract)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_dc   = softPin(1);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());

    spi::MasterAccessConfig cfg;
    cfg.pin_cs                = softPin(3);
    cfg.freq                  = 20000000;
    cfg.spi_command_length    = 8;
    cfg.spi_address_length    = 24;
    cfg.spi_write_dummy_cycle = 4;
    spi::MasterAccessor accessor{bus, cfg};
    port.clear();

    const uint8_t tx[] = {0xDE, 0xAD};
    auto result        = accessor.writeCommandAddressData(0x02, 0x001234, data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(countWrites(port.events, softPin(0), true), 52u);
    EXPECT_GE(countWrites(port.events, softPin(0), false), 52u);
    EXPECT_EQ(countWrites(port.events, softPin(3), false), 1u);
    EXPECT_EQ(countWrites(port.events, softPin(3), true), 1u);
}

TEST(SoftwareIBus, LowLevelTransferHonorsCommandAddressDummyPhases)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_dc   = softPin(1);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());

    spi::MasterAccessConfig cfg;
    cfg.pin_cs = softPin(3);
    cfg.freq   = 20000000;

    spi::MasterAccessor accessor{bus, cfg};
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    port.clear();

    spi::TransferDesc desc;
    desc.command          = 0x02;
    desc.address          = 0x001234;
    desc.command_bytes    = 1;
    desc.address_bytes    = 3;
    desc.dummy_cycles     = 4;
    desc.command_dc_level = 0;
    desc.address_dc_level = 1;
    desc.data_dc_level    = 1;
    const uint8_t tx[]    = {0xDE, 0xAD};

    ASSERT_TRUE(accessor.transfer(desc, data::ConstDataSpan{tx, sizeof(tx)}, data::DataSpan{}).has_value());
    auto totals = accessor.endTransaction();

    ASSERT_TRUE(totals.has_value());
    EXPECT_EQ(totals->tx, sizeof(tx));
    EXPECT_EQ(totals->rx, size_t{0});
    EXPECT_EQ(countWrites(port.events, softPin(0), true), 52u);
    EXPECT_GE(countWrites(port.events, softPin(0), false), 52u);
}

TEST(SoftwareIBus, CpolHighIsAppliedBeforeCsAssert)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.pin_cs   = softPin(3);
    cfg.freq     = 20000000;
    cfg.spi_mode = 2;  // CPOL=1, CPHA=0
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x00};
    auto result        = accessor.write(data::ConstDataSpan{tx, sizeof(tx)});

    ASSERT_TRUE(result.has_value());
    ASSERT_LT(firstWriteIndex(port.events, softPin(0), true), port.events.size());
    ASSERT_LT(firstWriteIndex(port.events, softPin(3), false), port.events.size());
    EXPECT_LT(firstWriteIndex(port.events, softPin(0), true), firstWriteIndex(port.events, softPin(3), false));
}

TEST(SoftwareIBus, ExplicitTransactionKeepsCsAssertedAcrossTransfers)
{
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());

    spi::MasterAccessConfig cfg;
    cfg.pin_cs = softPin(3);
    cfg.freq   = 20000000;
    spi::MasterAccessor accessor{bus, cfg};
    port.clear();

    const uint8_t first[]  = {0x12};
    const uint8_t second[] = {0x34};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    EXPECT_TRUE(accessor.write(data::ConstDataSpan{first, sizeof(first)}).has_value());
    EXPECT_TRUE(accessor.write(data::ConstDataSpan{second, sizeof(second)}).has_value());
    ASSERT_TRUE(accessor.endTransaction().has_value());

    EXPECT_EQ(countWrites(port.events, softPin(3), false), 1u);
    EXPECT_EQ(countWrites(port.events, softPin(3), true), 1u);
}

TEST(SoftwareIBus, CoreTransferRunsInServiceRunnerUntilEndTransaction)
{
    ScopedServiceRunnerClear clear_services;
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.freq = 1000000;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x12, 0x34};
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    auto started = accessor.transfer(spi::TransferDesc{}, data::ConstDataSpan{tx, sizeof(tx)}, data::DataSpan{});
    ASSERT_TRUE(started.has_value());
    EXPECT_TRUE(accessor.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 1u);

    // SPI paces edges by spinning on the live counter, so local_tick must be
    // real-fastTick-derived (see the ServiceContext field notes); the fixed
    // per-pass elapsed advances the service's virtual schedule.
    for (size_t i = 0; i < 200 && m5::hal::v2::M5_Hal.Services.size() != 0; ++i) {
        (void)m5::hal::v2::M5_Hal.Services.runOnce(service::ServiceContext{10, service::fastTick()});
    }
    EXPECT_FALSE(accessor.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 0u);

    auto totals = accessor.endTransaction();
    ASSERT_TRUE(totals.has_value());
    EXPECT_EQ(totals->tx, sizeof(tx));
    EXPECT_EQ(totals->rx, size_t{0});
    EXPECT_GE(countWrites(port.events, softPin(0), true), sizeof(tx) * 8u);
}

TEST(SoftwareIBus, CoreTransferFailsWhenServiceRunnerIsFull)
{
    ScopedServiceRunnerClear clear_services;
    auto fillers = fillGlobalServiceRunner();

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());

    spi::MasterAccessConfig cfg;
    cfg.freq = 1000000;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t tx[] = {0x12, 0x34};
    ASSERT_TRUE(accessor.beginTransaction().has_value());
    auto started = accessor.transfer(spi::TransferDesc{}, data::ConstDataSpan{tx, sizeof(tx)}, data::DataSpan{});
    ASSERT_FALSE(started.has_value());
    EXPECT_EQ(started.error(), error::error_t::OUT_OF_RESOURCE);
    EXPECT_FALSE(accessor.transferBusy());

    // The failed segment poisons the transaction (unified latch contract,
    // spec/design/spi.md §transaction 中のエラー).
    auto ended = accessor.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), service::ServiceRunner::kMaxServices);
    (void)fillers;
}

TEST(SoftwareIBus, NextCoreTransferDrainsPreviousBeforeStarting)
{
    ScopedServiceRunnerClear clear_services;
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus;
    spi::BusConfig_software bus_cfg;
    bus_cfg.pin_clk  = softPin(0);
    bus_cfg.pin_mosi = softPin(2);
    ASSERT_TRUE(bus.init(bus_cfg).has_value());
    port.clear();

    spi::MasterAccessConfig cfg;
    cfg.freq = 1000000;
    spi::MasterAccessor accessor{bus, cfg};

    const uint8_t first[]  = {0x12, 0x34};
    const uint8_t second[] = {0x56};

    ASSERT_TRUE(accessor.beginTransaction().has_value());
    ASSERT_TRUE(accessor.transfer(spi::TransferDesc{}, data::ConstDataSpan{first, sizeof(first)}, data::DataSpan{})
                    .has_value());
    EXPECT_TRUE(accessor.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 1u);

    ASSERT_TRUE(accessor.transfer(spi::TransferDesc{}, data::ConstDataSpan{second, sizeof(second)}, data::DataSpan{})
                    .has_value());
    EXPECT_TRUE(accessor.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 1u);

    auto totals = accessor.endTransaction();
    ASSERT_TRUE(totals.has_value());
    EXPECT_EQ(totals->tx, sizeof(first) + sizeof(second));
    EXPECT_EQ(totals->rx, size_t{0});
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 0u);
}

TEST(SoftwareIBus, MultipleBusesProgressTogetherInServiceRunner)
{
    ScopedServiceRunnerClear clear_services;
    auto& gpio = softwareSpiGPIO();
    auto& port = gpio._port;

    m5::hal::v2::spi::Bus_software bus_a;
    spi::BusConfig_software cfg_a;
    cfg_a.pin_clk  = softPin(0);
    cfg_a.pin_mosi = softPin(2);
    ASSERT_TRUE(bus_a.init(cfg_a).has_value());

    m5::hal::v2::spi::Bus_software bus_b;
    spi::BusConfig_software cfg_b;
    cfg_b.pin_clk  = softPin(4);
    cfg_b.pin_mosi = softPin(5);
    ASSERT_TRUE(bus_b.init(cfg_b).has_value());
    port.clear();

    spi::MasterAccessConfig access_cfg;
    access_cfg.freq = 1000000;
    spi::MasterAccessor accessor_a{bus_a, access_cfg};
    spi::MasterAccessor accessor_b{bus_b, access_cfg};

    const uint8_t tx_a[] = {0x12, 0x34};
    const uint8_t tx_b[] = {0x56, 0x78, 0x9a};

    ASSERT_TRUE(accessor_a.beginTransaction().has_value());
    ASSERT_TRUE(accessor_b.beginTransaction().has_value());
    ASSERT_TRUE(accessor_a.transfer(spi::TransferDesc{}, data::ConstDataSpan{tx_a, sizeof(tx_a)}, data::DataSpan{})
                    .has_value());
    ASSERT_TRUE(accessor_b.transfer(spi::TransferDesc{}, data::ConstDataSpan{tx_b, sizeof(tx_b)}, data::DataSpan{})
                    .has_value());
    EXPECT_TRUE(accessor_a.transferBusy());
    EXPECT_TRUE(accessor_b.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 2u);

    for (size_t i = 0; i < 300 && (accessor_a.transferBusy() || accessor_b.transferBusy()); ++i) {
        (void)m5::hal::v2::M5_Hal.Services.runOnce(service::ServiceContext{10, service::fastTick()});
    }

    EXPECT_FALSE(accessor_a.transferBusy());
    EXPECT_FALSE(accessor_b.transferBusy());
    EXPECT_EQ(m5::hal::v2::M5_Hal.Services.size(), 0u);

    auto totals_a = accessor_a.endTransaction();
    auto totals_b = accessor_b.endTransaction();
    ASSERT_TRUE(totals_a.has_value());
    ASSERT_TRUE(totals_b.has_value());
    EXPECT_EQ(totals_a->tx, sizeof(tx_a));
    EXPECT_EQ(totals_b->tx, sizeof(tx_b));
    EXPECT_GE(countWrites(port.events, softPin(0), true), sizeof(tx_a) * 8u);
    EXPECT_GE(countWrites(port.events, softPin(4), true), sizeof(tx_b) * 8u);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
