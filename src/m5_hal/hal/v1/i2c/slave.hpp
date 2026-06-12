#ifndef M5_HAL_HAL_V1_I2C_SLAVE_HPP_
#define M5_HAL_HAL_V1_I2C_SLAVE_HPP_

#include "../data.hpp"
#include "../error.hpp"
#include "../service/service.hpp"
#include "../types.hpp"

#include <M5Utility.hpp>

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v1::i2c {

struct I2CSlaveConfig {
    types::gpio_number_t pin_scl = -1;
    types::gpio_number_t pin_sda = -1;
    uint16_t address             = 0;
    bool address_is_10bit        = false;
    uint32_t timeout_ms          = 1000;
};

class I2CSlaveLineDriver {
public:
    virtual ~I2CSlaveLineDriver() = default;

    virtual bool readScl() const           = 0;
    virtual bool readSda() const           = 0;
    virtual void pullSdaLow(bool pull_low) = 0;
    virtual void pullSclLow(bool pull_low) = 0;
};

class I2CSlaveDriver {
public:
    virtual ~I2CSlaveDriver() = default;

    virtual m5::stl::expected<void, error::error_t> init(const I2CSlaveConfig& cfg) = 0;
    virtual m5::stl::expected<void, error::error_t> release()                       = 0;
    virtual service::IService* service()                                            = 0;
};

class ScopedI2CSlaveServiceRegistration {
public:
    ScopedI2CSlaveServiceRegistration() = default;
    ScopedI2CSlaveServiceRegistration(service::ServiceRunner& runner, I2CSlaveDriver& driver)
    {
        (void)registerTo(runner, driver);
    }
    ~ScopedI2CSlaveServiceRegistration()
    {
        release();
    }

    ScopedI2CSlaveServiceRegistration(const ScopedI2CSlaveServiceRegistration&)            = delete;
    ScopedI2CSlaveServiceRegistration& operator=(const ScopedI2CSlaveServiceRegistration&) = delete;

    ScopedI2CSlaveServiceRegistration(ScopedI2CSlaveServiceRegistration&& other) noexcept
        : _runner{other._runner}, _service{other._service}
    {
        other._runner  = nullptr;
        other._service = nullptr;
    }
    ScopedI2CSlaveServiceRegistration& operator=(ScopedI2CSlaveServiceRegistration&& other) noexcept
    {
        if (this != &other) {
            release();
            _runner        = other._runner;
            _service       = other._service;
            other._runner  = nullptr;
            other._service = nullptr;
        }
        return *this;
    }

    m5::stl::expected<void, error::error_t> registerTo(service::ServiceRunner& runner, I2CSlaveDriver& driver)
    {
        release();
        auto* svc = driver.service();
        if (svc == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        if (!runner.add(*svc)) {
            return m5::stl::make_unexpected(error::error_t::BUSY);
        }
        _runner  = &runner;
        _service = svc;
        return {};
    }

    void release()
    {
        if (_runner != nullptr && _service != nullptr) {
            (void)_runner->remove(*_service);
        }
        _runner  = nullptr;
        _service = nullptr;
    }

    bool registered() const
    {
        return _runner != nullptr && _service != nullptr;
    }

private:
    service::ServiceRunner* _runner = nullptr;
    service::IService* _service     = nullptr;
};

class I2CSlaveService : public service::IService {
public:
    static constexpr size_t kMaxObservedMasterAcks = 16;

    // Construction is two-phase by design: there is no (lines, config)
    // constructor because it would have to swallow init()'s expected —
    // a rejected config (10-bit / address > 0x7F) would leave a silently
    // inert service. Construct, then call init() and check its result
    // (S16 D6).
    I2CSlaveService() = default;

    m5::stl::expected<void, error::error_t> init(I2CSlaveLineDriver& lines, const I2CSlaveConfig& config)
    {
        if (config.address_is_10bit || config.address > 0x7F) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        _lines  = &lines;
        _config = config;
        resetProtocol();
        return {};
    }

    void setRxBuffer(data::DataSpan rx_buffer)
    {
        _rx_buffer = rx_buffer;
        _rx_size   = 0;
    }

    void setTxBuffer(data::ConstDataSpan tx_buffer)
    {
        _tx_buffer = tx_buffer;
        _tx_index  = 0;
    }

    void setMaxAckedWriteBytes(size_t count)
    {
        _max_acked_write_bytes = count;
    }

    size_t maxAckedWriteBytes() const
    {
        return _max_acked_write_bytes;
    }

    size_t receivedSize() const
    {
        return _rx_size;
    }

    data::ConstDataSpan received() const
    {
        return {_rx_buffer.data, _rx_size};
    }

    size_t masterAckCount() const
    {
        return _master_ack_count;
    }

    bool masterAckAt(size_t index) const
    {
        return (index < _master_ack_count) ? _master_acks[index] : false;
    }

    size_t stopCount() const
    {
        return _stop_count;
    }

    service::ServiceResult service(const service::ServiceContext& ctx) override
    {
        (void)ctx;
        if (_lines == nullptr) {
            return service::ServiceResult::Idle;
        }

        const bool scl = _lines->readScl();
        const bool sda = _lines->readSda();

        auto result = service::ServiceResult::Idle;
        if (_prev_scl && _prev_sda && !sda) {
            _state      = State::Receive;
            _byte       = 0;
            _bit_count  = 0;
            _is_address = true;
            _matched    = false;
            _read_phase = false;
            _drive_ack  = false;
            _lines->pullSdaLow(false);
            result = service::ServiceResult::Progress;
        }

        if (!_prev_scl && scl && _state == State::Receive) {
            _byte = static_cast<uint8_t>((_byte << 1) | (sda ? 1 : 0));
            ++_bit_count;
            if (_bit_count == 8) {
                if (_is_address) {
                    _matched    = ((_byte >> 1) == _config.address);
                    _read_phase = (_byte & 1) != 0;
                    _drive_ack  = _matched;
                } else {
                    const bool stored = storeReceivedByte(_byte);
                    _drive_ack        = _matched && !_read_phase && stored && (_rx_size <= _max_acked_write_bytes);
                }
                _state = State::AckSetup;
            }
            result = service::ServiceResult::Progress;
        }

        if (!_prev_scl && scl && _state == State::ReadMasterAck) {
            _master_ack = !sda;
            storeMasterAck(_master_ack);
            result = service::ServiceResult::Progress;
        }

        if (_prev_scl && !scl && _state == State::Transmit) {
            if (++_bit_count < 8) {
                driveTxBit();
            } else {
                _lines->pullSdaLow(false);
                _state = State::ReadMasterAck;
            }
            result = service::ServiceResult::Progress;
        } else if (_prev_scl && !scl && _state == State::ReadMasterAck) {
            if (_master_ack && (++_tx_index < _tx_buffer.size)) {
                _bit_count = 0;
                _state     = State::Transmit;
                driveTxBit();
            } else {
                _state = State::Ignore;
                _lines->pullSdaLow(false);
            }
            result = service::ServiceResult::Progress;
        } else if (_prev_scl && !scl && _state == State::AckSetup) {
            _lines->pullSdaLow(_drive_ack);
            _state = State::Ack;
            result = service::ServiceResult::Progress;
        } else if (_prev_scl && !scl && _state == State::Ack) {
            _lines->pullSdaLow(false);
            _byte      = 0;
            _bit_count = 0;
            if (_matched && _read_phase && _tx_buffer.data != nullptr && _tx_buffer.size > 0) {
                _tx_index = 0;
                _state    = State::Transmit;
                driveTxBit();
            } else {
                _is_address = false;
                _state      = _matched && !_read_phase ? State::Receive : State::Ignore;
            }
            result = service::ServiceResult::Progress;
        }

        if (_prev_scl && !_prev_sda && sda) {
            _state = State::Idle;
            ++_stop_count;
            _lines->pullSdaLow(false);
            result = service::ServiceResult::Progress;
        }

        _prev_scl = scl;
        _prev_sda = sda;
        return result;
    }

protected:
    void resetProtocol()
    {
        _state            = State::Idle;
        _prev_scl         = true;
        _prev_sda         = true;
        _is_address       = true;
        _matched          = false;
        _read_phase       = false;
        _drive_ack        = false;
        _master_ack       = false;
        _byte             = 0;
        _bit_count        = 0;
        _tx_index         = 0;
        _rx_size          = 0;
        _stop_count       = 0;
        _master_ack_count = 0;
        // NOTE: _max_acked_write_bytes is CONFIGURATION, not protocol
        // state — it survives init()/resetProtocol() so a value set
        // before init() is not silently discarded (S16 D6).
        for (size_t i = 0; i < kMaxObservedMasterAcks; ++i) {
            _master_acks[i] = false;
        }
        if (_lines != nullptr) {
            _lines->pullSdaLow(false);
            _lines->pullSclLow(false);
        }
    }

private:
    enum class State : uint8_t { Idle, Receive, AckSetup, Ack, Transmit, ReadMasterAck, Ignore };

    bool storeReceivedByte(uint8_t value)
    {
        if (_rx_buffer.data == nullptr || _rx_size >= _rx_buffer.size) {
            return false;
        }
        _rx_buffer.data[_rx_size++] = value;
        return true;
    }

    void storeMasterAck(bool ack)
    {
        if (_master_ack_count < kMaxObservedMasterAcks) {
            _master_acks[_master_ack_count++] = ack;
        }
    }

    void driveTxBit()
    {
        if (_lines == nullptr || _tx_buffer.data == nullptr || _tx_index >= _tx_buffer.size) {
            return;
        }
        const uint8_t b = _tx_buffer.data[_tx_index];
        const bool bit  = (b & (0x80u >> _bit_count)) != 0;
        _lines->pullSdaLow(!bit);
    }

    I2CSlaveLineDriver* _lines = nullptr;
    I2CSlaveConfig _config;
    data::DataSpan _rx_buffer;
    data::ConstDataSpan _tx_buffer;
    State _state                              = State::Idle;
    bool _prev_scl                            = true;
    bool _prev_sda                            = true;
    bool _is_address                          = true;
    bool _matched                             = false;
    bool _read_phase                          = false;
    bool _drive_ack                           = false;
    bool _master_ack                          = false;
    uint8_t _byte                             = 0;
    uint8_t _bit_count                        = 0;
    size_t _tx_index                          = 0;
    size_t _rx_size                           = 0;
    size_t _stop_count                        = 0;
    size_t _max_acked_write_bytes             = static_cast<size_t>(-1);
    size_t _master_ack_count                  = 0;
    bool _master_acks[kMaxObservedMasterAcks] = {};
};

}  // namespace m5::hal::v1::i2c

#endif  // M5_HAL_HAL_V1_I2C_SLAVE_HPP_
