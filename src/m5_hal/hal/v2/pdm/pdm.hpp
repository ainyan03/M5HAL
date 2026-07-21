// SPDX-License-Identifier: MIT
#ifndef M5_HAL_PDM_PDM_HPP_
#define M5_HAL_PDM_PDM_HPP_

#include "../bus/bus.hpp"
#include "../bus/bus_view.hpp"
#include "../bus/managed_facade.hpp"
#include "../data.hpp"
#include "../data/stream.hpp"

#include <memory>
#include <new>

namespace m5::hal::v2::pdm {

/*! @brief Strong-typed PDM clock output pin. */
struct Clk {
    constexpr explicit Clk(types::gpio_number_t pin) : value{pin}
    {
    }
    types::gpio_number_t value;
};

/*! @brief Strong-typed PDM data input pin. */
struct Din {
    constexpr explicit Din(types::gpio_number_t pin) : value{pin}
    {
    }
    types::gpio_number_t value;
};

/*!
  @brief PDM RX wiring and DMA configuration.

  The initial API is intentionally one-line PCM capture: CLK and DIN are
  required, the controller is clock master, and the backend converts PDM to
  signed 16-bit mono PCM. Raw PDM, TX, and multiple DIN lines are separate
  future capabilities rather than mode fields with ambiguous semantics.
 */
struct IBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_clk = -1;
    types::gpio_number_t pin_din = -1;
    size_t rx_buffer_size        = 8192;

    constexpr IBusConfig() : bus::IBusConfig{types::bus_kind_t::PDM}
    {
    }
    constexpr IBusConfig(Clk clk, Din din)
        : bus::IBusConfig{types::bus_kind_t::PDM}, pin_clk{clk.value}, pin_din{din.value}
    {
    }
};

/*! @brief Variant-independent portable PDM bus configuration. */
using BusConfig = IBusConfig;

struct LogicalBusConfig {
    types::gpio_number_t pin_clk = -1;
    types::gpio_number_t pin_din = -1;
    types::AllocationIntent intent{};

    constexpr LogicalBusConfig() = default;
    constexpr LogicalBusConfig(Clk clk, Din din, types::AllocationIntent in = {})
        : pin_clk{clk.value}, pin_din{din.value}, intent{in}
    {
    }
};

/*! @brief Per-read PCM format and timeout. Only 16-bit mono is accepted. */
struct AccessConfig : public bus::IAccessConfig {
    uint32_t sample_rate_hz  = 16000;
    uint32_t read_timeout_ms = 1000;
    uint8_t bits_per_sample  = 16;
    uint8_t channels         = 1;

    constexpr AccessConfig() : bus::IAccessConfig{types::bus_kind_t::PDM}
    {
    }
};

struct IBus;

/*! @brief RX-only PDM PCM stream accessor. */
struct RxAccessor : public bus::IAccessor, public data::StreamReader {
    RxAccessor(IBus& bus, const AccessConfig& access_config);
    RxAccessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config);
    RxAccessor() : _context{makeOperationContext(AccessConfig{})}
    {
    }
    explicit RxAccessor(const AccessConfig& access_config) : _context{makeOperationContext(access_config)}
    {
    }

    [[nodiscard]] result_t<void> bind(IBus& bus);
    const AccessConfig& getConfig() const override
    {
        return _context.config;
    }
    IBus& getBus() const;
    [[nodiscard]] result_t<void> setConfig(const AccessConfig& cfg);
    [[nodiscard]] result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    [[nodiscard]] result_t<void> endAccess(uint32_t timeout_ms = 1000);
    bool inAccess() const
    {
        return _inOperationAccess();
    }
    result_t<bus::TransferStatus> getLastTransferStatus() const;
    result_t<size_t> read(data::DataSpan dst_bytes) override;
    result_t<size_t> read(data::Sink& dst, size_t len);
    result_t<size_t> read(uint8_t* dst, size_t len);
    result_t<size_t> readableBytes() override;

private:
    result_t<size_t> readInCurrentAccess(data::Sink& dst, size_t len);
    void recordTransferResult(const result_t<size_t>& result, size_t requested);

    bus::OperationContext<AccessConfig> _context;
    bus::TransferStatus _last_transfer_status{};
    uint32_t _next_transfer_id = 0;
};

struct IBus : public bus::IBus {
    const IBusConfig& getConfig() const override
    {
        return _config;
    }
    result_t<void> beginOperation(bus::OperationContext<AccessConfig>& context);
    result_t<void> endOperation(bus::OperationContext<AccessConfig>& context);
    result_t<size_t> read(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len);
    result_t<size_t> readableBytes(bus::OperationContext<AccessConfig>& context);

protected:
    virtual result_t<void> beginOperationBackend(bus::OperationContext<AccessConfig>& context);
    virtual result_t<void> endOperationBackend(bus::OperationContext<AccessConfig>& context);
    virtual result_t<size_t> readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len);
    virtual result_t<size_t> readableBytesBackend(bus::OperationContext<AccessConfig>& context);

    static result_t<void> beginOperationOn(IBus& backend, bus::OperationContext<AccessConfig>& context)
    {
        return backend.beginOperationBackend(context);
    }
    static result_t<void> endOperationOn(IBus& backend, bus::OperationContext<AccessConfig>& context)
    {
        return backend.endOperationBackend(context);
    }
    static result_t<size_t> readOn(IBus& backend, bus::OperationContext<AccessConfig>& context, data::Sink* dst,
                                   size_t len)
    {
        return backend.readBackend(context, dst, len);
    }
    static result_t<size_t> readableBytesOn(IBus& backend, bus::OperationContext<AccessConfig>& context)
    {
        return backend.readableBytesBackend(context);
    }

    IBusConfig _config;
    bus::OperationSlot _operation_slot;
};

inline result_t<void> RxAccessor::bind(IBus& bus)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _bindBus(bus);
    return {};
}

template <class Policy>
struct NativeProvider;
struct Bus;

struct BusTraits {
    using IBus             = pdm::IBus;
    using IBusConfig       = pdm::IBusConfig;
    using LogicalBusConfig = pdm::LogicalBusConfig;
    using BusType          = Bus;
    template <class Policy>
    using NativeProvider = pdm::NativeProvider<Policy>;

    static constexpr types::bus_kind_t KIND  = types::bus_kind_t::PDM;
    static constexpr bool MANAGED_ALLOCATION = false;

    static bus::ResourceKey identityFromConfig(const IBusConfig& cfg)
    {
        return bus::ResourceKey::fromPins(KIND, {cfg.pin_clk, cfg.pin_din});
    }
    static bus::ResourceKey identityFromLogical(const LogicalBusConfig& cfg)
    {
        return bus::ResourceKey::fromPins(KIND, {cfg.pin_clk, cfg.pin_din});
    }
    static bool configCompatible(const IBusConfig& current, const IBusConfig& requested)
    {
        return current.pin_clk == requested.pin_clk && current.pin_din == requested.pin_din &&
               current.rx_buffer_size == requested.rx_buffer_size;
    }
};

struct Bus : public bus::FacadeCore<BusTraits> {
    [[nodiscard]] result_t<void> init(const IBusConfig& config);

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<AccessConfig>& context) override
    {
        return forwardBackend([&](IBus& backend) { return beginOperationOn(backend, context); });
    }
    result_t<void> endOperationBackend(bus::OperationContext<AccessConfig>& context) override
    {
        return forwardBackend([&](IBus& backend) { return endOperationOn(backend, context); });
    }
    result_t<size_t> readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len) override
    {
        return forwardBackend([&](IBus& backend) { return readOn(backend, context, dst, len); });
    }
    result_t<size_t> readableBytesBackend(bus::OperationContext<AccessConfig>& context) override
    {
        return forwardBackend([&](IBus& backend) { return readableBytesOn(backend, context); });
    }
};

class BusView : public bus::BusViewCore<BusTraits> {
public:
    using bus::BusViewCore<BusTraits>::BusViewCore;

    LogicalBusConfig createBusConfig(Clk clk, Din din, types::AllocationIntent intent = {}) const
    {
        return {clk, din, intent};
    }
};

using BusGroup = bus::BusGroup<IBus>;

}  // namespace m5::hal::v2::pdm

#endif  // M5_HAL_PDM_PDM_HPP_
