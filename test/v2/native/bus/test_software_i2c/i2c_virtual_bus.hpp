// SPDX-License-Identifier: MIT
#ifndef M5HAL_TEST_V2_NATIVE_BUS_TEST_SOFTWARE_I2C_I2C_VIRTUAL_BUS_HPP
#define M5HAL_TEST_V2_NATIVE_BUS_TEST_SOFTWARE_I2C_I2C_VIRTUAL_BUS_HPP

// Test-local compat layer over m5::hal::v2::i2c::virtual_bus.hpp (the
// in-process virtual I2C wire harness, now part of the M5HAL sources).
// Keeps the `service_proto::` names this test suite was written against,
// plus two pieces the shared header intentionally does not carry:
//   - debug helpers (`SlaveEndpoint::received()` / `masterAcks()`) that
//     drain a transaction / dump the ack history for assertions
//   - `SlaveTransactionResponder` / `SlaveStreamReplyResponder`, tick-driven
//     test responders with no production use

#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/i2c/virtual_bus.hpp>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace service_proto {

using IService      = m5::hal::v2::service::IService;
using ServiceResult = m5::hal::v2::service::ServiceResult;
using ServiceRunner = m5::hal::v2::service::ServiceRunner;

using VirtualOpenDrainBus             = m5::hal::v2::i2c::VirtualOpenDrainBus;
using VirtualOpenDrainSlaveLineDriver = m5::hal::v2::i2c::VirtualOpenDrainSlaveLineDriver;
using VirtualI2CPort                  = m5::hal::v2::i2c::VirtualI2CPort;
using VirtualI2CGPIO                  = m5::hal::v2::i2c::VirtualI2CGPIO;

// Wraps m5::hal::v2::i2c::ScopedVirtualI2CGPIO bound to the M5_Hal singleton
// group, so existing call sites (`ScopedVirtualI2CGPIO gpio{scl_port,
// sda_port};`) keep working. This test suite's master side goes through
// `Bus_software`, which resolves `gpio_number_t` against the singleton
// `M5_Hal.Gpio` (not an arbitrary GPIOGroup instance) -- registering
// elsewhere would leave the master unable to find the virtual SCL/SDA pins.
class ScopedVirtualI2CGPIO {
public:
    ScopedVirtualI2CGPIO(VirtualI2CPort& scl, VirtualI2CPort& sda) : _scoped(scl, sda, m5::hal::v2::M5_Hal.Gpio)
    {
    }

    m5::hal::v2::types::gpio_number_t scl() const
    {
        return _scoped.scl();
    }
    m5::hal::v2::types::gpio_number_t sda() const
    {
        return _scoped.sda();
    }

private:
    m5::hal::v2::i2c::ScopedVirtualI2CGPIO _scoped;
};

// Adds the debug-only draining helpers this test suite asserts on back onto
// SlaveEndpoint; production code has no use for peeking at raw received
// bytes / the ack history, so they live here rather than in virtual_bus.hpp.
class SlaveEndpoint : public m5::hal::v2::i2c::SlaveEndpoint {
public:
    using Base = m5::hal::v2::i2c::SlaveEndpoint;
    using Base::Base;

    const std::vector<uint8_t>& received()
    {
        _received.clear();
        auto begin = accessor().beginTransaction(0);
        if (!begin.has_value()) {
            return _received;
        }
        uint8_t buffer[64] = {};
        auto read          = accessor().read(m5::hal::v2::data::DataSpan{buffer, sizeof(buffer)});
        if (read.has_value()) {
            _received.assign(buffer, buffer + read.value());
        }
        (void)accessor().endTransaction();
        return _received;
    }
    const std::vector<bool>& masterAcks()
    {
        _master_acks.clear();
        for (size_t i = 0; i < bus().masterAckCount(); ++i) {
            _master_acks.push_back(bus().masterAckAt(i));
        }
        return _master_acks;
    }

private:
    std::vector<uint8_t> _received;
    std::vector<bool> _master_acks;
};

class SlaveTransactionResponder : public m5::hal::v2::service::IService {
public:
    SlaveTransactionResponder(SlaveEndpoint& endpoint, const std::vector<uint8_t>& tx_bytes,
                              size_t min_rx_bytes_before_write = 0, bool repeat = false)
        : _endpoint(endpoint),
          _tx_bytes(tx_bytes),
          _min_rx_bytes_before_write(min_rx_bytes_before_write),
          _repeat(repeat)
    {
    }

    const std::vector<uint8_t>& received() const
    {
        return _received;
    }
    bool opened() const
    {
        return _opened;
    }
    bool ended() const
    {
        return _ended;
    }

    m5::hal::v2::service::ServicePoll serviceImpl(const m5::hal::v2::service::ServiceContext&) override
    {
        if (_ended && !_repeat) {
            return m5::hal::v2::service::ServiceResult::Idle;
        }
        if (!_opened) {
            auto begin = _endpoint.accessor().beginTransaction(0);
            if (!begin.has_value()) {
                return m5::hal::v2::service::ServiceResult::Idle;
            }
            _opened = true;
        }

        uint8_t buffer[64] = {};
        auto read          = _endpoint.accessor().read(m5::hal::v2::data::DataSpan{buffer, sizeof(buffer)});
        if (read.has_value() && read.value() != 0) {
            _received.insert(_received.end(), buffer, buffer + read.value());
        }

        if (!_wrote && !_tx_bytes.empty() && _received.size() >= _min_rx_bytes_before_write) {
            auto write =
                _endpoint.accessor().write(m5::hal::v2::data::ConstDataSpan{_tx_bytes.data(), _tx_bytes.size()});
            if (write.has_value() && write.value() == _tx_bytes.size()) {
                _wrote = true;
            }
        }

        auto complete = _endpoint.bus().transactionComplete(&_endpoint.accessor());
        if (complete.has_value() && complete.value()) {
            (void)_endpoint.accessor().endTransaction();
            _ended = true;
            if (_repeat) {
                _opened = false;
                _wrote  = false;
                return m5::hal::v2::service::ServiceResult::Progress;
            }
            return m5::hal::v2::service::ServiceResult::Done;
        }
        return m5::hal::v2::service::ServiceResult::Progress;
    }

private:
    SlaveEndpoint& _endpoint;
    std::vector<uint8_t> _tx_bytes;
    std::vector<uint8_t> _received;
    size_t _min_rx_bytes_before_write = 0;
    bool _repeat                      = false;
    bool _opened                      = false;
    bool _wrote                       = false;
    bool _ended                       = false;
};

// Streams a fixed reply to the master across one read transaction, writing as much
// as the tx ring accepts each tick and advancing past what was accepted, until the
// whole reply is queued. With tx_underrun=stretch the master is held whenever the
// ring momentarily empties, so a reply far larger than kTxCapacity streams out
// intact. Exercises the TX ring + the read-serve streaming contract.
class SlaveStreamReplyResponder : public m5::hal::v2::service::IService {
public:
    SlaveStreamReplyResponder(SlaveEndpoint& endpoint, std::vector<uint8_t> reply)
        : _endpoint(endpoint), _reply(std::move(reply))
    {
    }

    size_t written() const
    {
        return _off;
    }
    bool ended() const
    {
        return _ended;
    }

    m5::hal::v2::service::ServicePoll serviceImpl(const m5::hal::v2::service::ServiceContext&) override
    {
        if (_ended) {
            return m5::hal::v2::service::ServiceResult::Idle;
        }
        if (!_opened) {
            auto begin = _endpoint.accessor().beginTransaction(0);
            if (!begin.has_value()) {
                return m5::hal::v2::service::ServiceResult::Idle;
            }
            _opened = true;
        }
        if (_off < _reply.size()) {
            auto wrote = _endpoint.accessor().write(
                m5::hal::v2::data::ConstDataSpan{_reply.data() + _off, _reply.size() - _off});
            if (wrote.has_value()) {
                _off += wrote.value();
            }
        }
        auto complete = _endpoint.bus().transactionComplete(&_endpoint.accessor());
        if (complete.has_value() && complete.value()) {
            (void)_endpoint.accessor().endTransaction();
            _ended = true;
            return m5::hal::v2::service::ServiceResult::Done;
        }
        return m5::hal::v2::service::ServiceResult::Progress;
    }

private:
    SlaveEndpoint& _endpoint;
    std::vector<uint8_t> _reply;
    size_t _off  = 0;
    bool _opened = false;
    bool _ended  = false;
};

}  // namespace service_proto

#endif
