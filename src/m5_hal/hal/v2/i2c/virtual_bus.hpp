// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_I2C_VIRTUAL_BUS_HPP_
#define M5_HAL_HAL_V2_I2C_VIRTUAL_BUS_HPP_

// In-process virtual I2C wire (open-drain, wired-AND) for tick-driven
// simulation and testing -- there is no ISR and no real time here. Every
// line transition the master drives pumps the bound ServiceRunner once, so
// a single-threaded master transfer (Bus_software) completes a full
// transaction synchronously without an explicit tick loop on the caller's
// side. This is opt-in (not part of the umbrella headers): include it
// directly where an in-process master/slave harness is needed.

#include "./slave.hpp"
#include "../gpio/group.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::i2c {

// Shared open-drain SCL/SDA pair for up to 32 slave line drivers plus a
// single bit-banging master. Each slave observes the wired-AND of every
// puller (master + all slaves); a master line write / read pumps the bound
// ServiceRunner so slave-side services observe the transition immediately.
class VirtualOpenDrainBus : public SlaveLineDriver {
public:
    void setRunner(service::ServiceRunner* runner)
    {
        _runner = runner;
    }
    // Advance the bus's virtual timeline by `delta` ticks. The next pump
    // reports the accumulated advance as ctx.elapsed (relative contract);
    // the timeline value itself doubles as local_tick, which is valid here
    // because the services on this wire (software I2C master/slave) have no
    // intra-call spins — see the ServiceContext field notes.
    // The bound runner must be driven EXCLUSIVELY through this harness (no
    // auto-run, no concurrent runOnce callers). pump() intentionally discards
    // the result under that precondition; a BUSY error would mean the harness
    // contract was violated, and consuming the accumulated delta after such
    // a failed pass would silently drop elapsed time.
    void advance(service::fast_tick_t delta)
    {
        _now_v += delta;
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
            (void)_runner->runOnce(
                service::ServiceContext{static_cast<service::fast_tick_t>(_now_v - _last_pump_v), _now_v});
            _last_pump_v = _now_v;
        }
    }

    service::ServiceRunner* _runner   = nullptr;
    service::fast_tick_t _now_v       = 0;
    service::fast_tick_t _last_pump_v = 0;
    bool _master_scl_low              = false;
    bool _master_sda_low              = false;
    bool _external_slave_scl_low      = false;
    bool _external_slave_sda_low      = false;
    uint32_t _slave_scl_mask          = 0;
    uint32_t _slave_sda_mask          = 0;
};

// One slave's view of a `VirtualOpenDrainBus`: reads see the wired-AND of
// the whole bus, pulls are scoped to this slave's own bit in the mask.
class VirtualOpenDrainSlaveLineDriver : public SlaveLineDriver {
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

// GPIO port facade over one line (SCL or SDA) of a `VirtualOpenDrainBus`,
// so a software I2C master (which drives its pins through `gpio::IPort`)
// can bit-bang the virtual wire like a real pin.
class VirtualI2CPort : public gpio::IPort {
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
    void _setPinModeEncoded(uint32_t, types::gpio_mode_t) override
    {
    }
    types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(types::gpio_local_pin_t pin_index) const override
    {
        return static_cast<uint32_t>(pin_index);
    }

private:
    VirtualOpenDrainBus& _bus;
    Line _line;
};

// Two-pin IGPIO (pin 0 = SCL, pin 1 = SDA) bundling a pair of
// `VirtualI2CPort`s so they can be registered into a `GPIOGroup` slot.
class VirtualI2CGPIO : public gpio::IGPIO {
public:
    VirtualI2CGPIO(VirtualI2CPort& scl, VirtualI2CPort& sda) : _scl(scl), _sda(sda)
    {
    }

    gpio::IPort* portForPin(types::gpio_local_pin_t pin_index) const override
    {
        return (pin_index == 0) ? &_scl : &_sda;
    }
    gpio::IPort* getPort(uint8_t port_index) const override
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

// RAII registration of a `VirtualI2CGPIO` into a caller-supplied
// `GPIOGroup` slot, so a master's portable `BusConfig` can name its SCL/SDA
// as plain `gpio_number_t` pins. Takes a plain `GPIOGroup&` (not the `Hal`
// facade) so the same harness works from a native gtest with no facade
// singleton in play.
struct ScopedVirtualI2CGPIO {
    ScopedVirtualI2CGPIO(VirtualI2CPort& scl, VirtualI2CPort& sda, gpio::GPIOGroup& group, types::gpio_slot_t slot = 2)
        : _gpio(scl, sda), _group(group), _slot(slot)
    {
        auto r = _group.addGPIO(&_gpio, _slot);
        assert(r.has_value() && "ScopedVirtualI2CGPIO: slot add failed");
        (void)r;
    }
    ~ScopedVirtualI2CGPIO()
    {
        auto r = _group.removeGPIO(_slot);
        assert(r.has_value() && "ScopedVirtualI2CGPIO: slot remove failed");
        (void)r;
    }

    types::gpio_number_t scl() const
    {
        return types::makeGpioNumber(_slot, 0);
    }
    types::gpio_number_t sda() const
    {
        return types::makeGpioNumber(_slot, 1);
    }

private:
    VirtualI2CGPIO _gpio;
    gpio::GPIOGroup& _group;
    types::gpio_slot_t _slot;
};

// Owns a `SlaveBus_software` + its `SlaveStreamAccessor`. The
// `VirtualOpenDrainBus&` overload binds the bus's single un-indexed
// external puller — fine for exactly one slave. Hanging SEVERAL slaves
// off one bus needs a distinct `VirtualOpenDrainSlaveLineDriver` (indexed
// mask bit) per slave passed to the `SlaveLineDriver&` overload; sharing
// the un-indexed puller lets one slave's line release clobber another's ACK.
class SlaveEndpoint {
public:
    SlaveEndpoint(VirtualOpenDrainBus& bus, uint8_t address, TxUnderrun tx_underrun = TxUnderrun::Fill)
        : _accessor{_bus}
    {
        initBus(bus, address, tx_underrun);
    }
    SlaveEndpoint(SlaveLineDriver& lines, uint8_t address, TxUnderrun tx_underrun = TxUnderrun::Fill) : _accessor{_bus}
    {
        initBus(lines, address, tx_underrun);
    }

    SlaveBus_software& bus()
    {
        return _bus;
    }
    SlaveStreamAccessor& accessor()
    {
        return _accessor;
    }
    service::IService& service()
    {
        return _bus;
    }

    void setMaxAckedWriteBytes(size_t count)
    {
        _bus.setMaxAckedWriteBytes(count);
    }
    size_t maxAckedWriteBytes() const
    {
        return _bus.maxAckedWriteBytes();
    }
    size_t stopCount() const
    {
        return _bus.stopCount();
    }

private:
    void initBus(SlaveLineDriver& lines, uint8_t address, TxUnderrun tx_underrun = TxUnderrun::Fill)
    {
        SlaveBusConfig cfg;
        cfg.address     = address;
        cfg.timeout_ms  = 0;
        cfg.tx_underrun = tx_underrun;
        // SlaveEndpoint is the legacy wire-frame-window test helper. New
        // queue-driven endpoints leave this false and accept wire traffic only
        // between SlaveAccessor::beginAccess()/endAccess().
        cfg.legacy_wire_frame_window = true;
        auto r                       = _bus.init(lines, cfg);
        assert(r.has_value() && "SlaveEndpoint: init failed");
        (void)r;
    }

    SlaveBus_software _bus;
    SlaveStreamAccessor _accessor;
};

}  // namespace m5::hal::v2::i2c

#endif  // M5_HAL_HAL_V2_I2C_VIRTUAL_BUS_HPP_
