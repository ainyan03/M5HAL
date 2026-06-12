#ifndef M5HAL_TEST_V1_NATIVE_BUS_TEST_SOFTWARE_I2C_I2C_VIRTUAL_BUS_HPP
#define M5HAL_TEST_V1_NATIVE_BUS_TEST_SOFTWARE_I2C_I2C_VIRTUAL_BUS_HPP

#include <M5HAL_v1.hpp>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace service_proto {

using IService      = m5::hal::v1::service::IService;
using ServiceResult = m5::hal::v1::service::ServiceResult;
using ServiceRunner = m5::hal::v1::service::ServiceRunner;

class VirtualOpenDrainBus : public m5::hal::v1::i2c::I2CSlaveLineDriver {
public:
    void setRunner(ServiceRunner* runner)
    {
        _runner = runner;
    }
    void setNowTick(m5::hal::v1::service::fast_tick_t now_tick)
    {
        _now_tick = now_tick;
    }

    void masterWriteScl(bool high)
    {
        _master_scl_low = !high;
        pump();
    }
    void masterWriteSda(bool high)
    {
        _master_sda_low = !high;
        pump();
    }
    bool masterReadScl()
    {
        pump();
        return scl();
    }
    bool masterReadSda()
    {
        pump();
        return sda();
    }

    bool scl() const
    {
        return !(_master_scl_low || _external_slave_scl_low || _slave_scl_mask);
    }
    bool sda() const
    {
        return !(_master_sda_low || _external_slave_sda_low || _slave_sda_mask);
    }

    void slavePullSdaLow(bool pull_low)
    {
        _external_slave_sda_low = pull_low;
    }
    void slavePullSclLow(bool pull_low)
    {
        _external_slave_scl_low = pull_low;
    }
    void slavePullSdaLow(size_t index, bool pull_low)
    {
        setMaskBit(_slave_sda_mask, index, pull_low);
    }
    void slavePullSclLow(size_t index, bool pull_low)
    {
        setMaskBit(_slave_scl_mask, index, pull_low);
    }

    bool readScl() const override
    {
        return scl();
    }
    bool readSda() const override
    {
        return sda();
    }
    void pullSdaLow(bool pull_low) override
    {
        slavePullSdaLow(pull_low);
    }
    void pullSclLow(bool pull_low) override
    {
        slavePullSclLow(pull_low);
    }

private:
    static void setMaskBit(uint32_t& mask, size_t index, bool value)
    {
        assert(index < 32 && "VirtualOpenDrainBus supports up to 32 slave line drivers");
        const uint32_t bit = uint32_t{1} << index;
        if (value) {
            mask |= bit;
        } else {
            mask &= ~bit;
        }
    }

    void pump()
    {
        if (_runner != nullptr) {
            (void)_runner->runOnce(_now_tick);
        }
    }

    ServiceRunner* _runner                      = nullptr;
    m5::hal::v1::service::tick_nsec_t _now_tick = 0;
    bool _master_scl_low                        = false;
    bool _master_sda_low                        = false;
    bool _external_slave_scl_low                = false;
    bool _external_slave_sda_low                = false;
    uint32_t _slave_scl_mask                    = 0;
    uint32_t _slave_sda_mask                    = 0;
};

class VirtualOpenDrainSlaveLineDriver : public m5::hal::v1::i2c::I2CSlaveLineDriver {
public:
    VirtualOpenDrainSlaveLineDriver(VirtualOpenDrainBus& bus, size_t index) : _bus(bus), _index(index)
    {
        assert(index < 32 && "VirtualOpenDrainSlaveLineDriver index out of range");
    }

    bool readScl() const override
    {
        return _bus.scl();
    }
    bool readSda() const override
    {
        return _bus.sda();
    }
    void pullSdaLow(bool pull_low) override
    {
        _bus.slavePullSdaLow(_index, pull_low);
    }
    void pullSclLow(bool pull_low) override
    {
        _bus.slavePullSclLow(_index, pull_low);
    }

private:
    VirtualOpenDrainBus& _bus;
    size_t _index;
};

class VirtualI2CPort : public m5::hal::v1::gpio::IPort {
public:
    enum class Line : uint8_t { SCL, SDA };

    VirtualI2CPort(VirtualOpenDrainBus& bus, Line line) : _bus(bus), _line(line)
    {
    }

protected:
    void _writePinEncoded(uint32_t, bool v) override
    {
        if (_line == Line::SCL) {
            _bus.masterWriteScl(v);
        } else {
            _bus.masterWriteSda(v);
        }
    }
    bool _readPinEncoded(uint32_t) override
    {
        return (_line == Line::SCL) ? _bus.masterReadScl() : _bus.masterReadSda();
    }
    void _setPinModeEncoded(uint32_t, m5::hal::v1::types::gpio_mode_t) override
    {
    }
    m5::hal::v1::types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<m5::hal::v1::types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        return static_cast<uint32_t>(pin_index);
    }

private:
    VirtualOpenDrainBus& _bus;
    Line _line;
};

class VirtualI2CGPIO : public m5::hal::v1::gpio::IGPIO {
public:
    VirtualI2CGPIO(VirtualI2CPort& scl, VirtualI2CPort& sda) : _scl(scl), _sda(sda)
    {
    }

    m5::hal::v1::gpio::IPort* portForPin(m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        return (pin_index == 0) ? &_scl : &_sda;
    }
    m5::hal::v1::gpio::IPort* getPort(uint8_t port_index) const override
    {
        return (port_index == 0) ? &_scl : &_sda;
    }
    uint16_t getPinCount() const override
    {
        return 2;
    }
    uint8_t getPortCount() const override
    {
        return 2;
    }

private:
    VirtualI2CPort& _scl;
    VirtualI2CPort& _sda;
};

struct ScopedVirtualI2CGPIO {
    static constexpr m5::hal::v1::types::gpio_slot_t kSlot = 2;

    ScopedVirtualI2CGPIO(VirtualI2CPort& scl, VirtualI2CPort& sda) : _gpio(scl, sda)
    {
        auto r = m5::hal::v1::M5_Hal.Gpio.addGPIO(&_gpio, kSlot);
        assert(r.has_value() && "ScopedVirtualI2CGPIO: slot 2 add failed");
        (void)r;
    }
    ~ScopedVirtualI2CGPIO()
    {
        auto r = m5::hal::v1::M5_Hal.Gpio.removeGPIO(kSlot);
        assert(r.has_value() && "ScopedVirtualI2CGPIO: slot 2 remove failed");
        (void)r;
    }

    m5::hal::v1::types::gpio_number_t scl() const
    {
        return m5::hal::v1::types::makeGpioNumber(kSlot, 0);
    }
    m5::hal::v1::types::gpio_number_t sda() const
    {
        return m5::hal::v1::types::makeGpioNumber(kSlot, 1);
    }

private:
    VirtualI2CGPIO _gpio;
};

class I2CSlaveService : public m5::hal::v1::i2c::I2CSlaveService {
public:
    I2CSlaveService(VirtualOpenDrainBus& bus, uint8_t address) : m5::hal::v1::i2c::I2CSlaveService{}
    {
        initBus(bus, address);
    }
    I2CSlaveService(m5::hal::v1::i2c::I2CSlaveLineDriver& lines, uint8_t address) : m5::hal::v1::i2c::I2CSlaveService{}
    {
        initBus(lines, address);
    }
    I2CSlaveService(VirtualOpenDrainBus& bus, uint8_t address, const std::vector<uint8_t>& tx_bytes)
        : m5::hal::v1::i2c::I2CSlaveService{}, _tx_bytes(tx_bytes)
    {
        initBus(bus, address);
        setTxBuffer(m5::hal::v1::data::ConstDataSpan{_tx_bytes.data(), _tx_bytes.size()});
    }
    I2CSlaveService(m5::hal::v1::i2c::I2CSlaveLineDriver& lines, uint8_t address, const std::vector<uint8_t>& tx_bytes)
        : m5::hal::v1::i2c::I2CSlaveService{}, _tx_bytes(tx_bytes)
    {
        initBus(lines, address);
        setTxBuffer(m5::hal::v1::data::ConstDataSpan{_tx_bytes.data(), _tx_bytes.size()});
    }

    const std::vector<uint8_t>& received() const
    {
        const auto span = m5::hal::v1::i2c::I2CSlaveService::received();
        _received.assign(span.data, span.data + span.size);
        return _received;
    }
    const std::vector<bool>& masterAcks() const
    {
        _master_acks.clear();
        for (size_t i = 0; i < masterAckCount(); ++i) {
            _master_acks.push_back(masterAckAt(i));
        }
        return _master_acks;
    }

private:
    void initBus(m5::hal::v1::i2c::I2CSlaveLineDriver& lines, uint8_t address)
    {
        m5::hal::v1::i2c::I2CSlaveConfig cfg;
        cfg.address = address;
        auto r      = init(lines, cfg);
        assert(r.has_value() && "I2CSlaveService: init failed");
        (void)r;
        setRxBuffer(m5::hal::v1::data::DataSpan{_rx_buffer, sizeof(_rx_buffer)});
    }

    uint8_t _rx_buffer[64] = {};
    std::vector<uint8_t> _tx_bytes;
    mutable std::vector<uint8_t> _received;
    mutable std::vector<bool> _master_acks;
};

}  // namespace service_proto

#endif
