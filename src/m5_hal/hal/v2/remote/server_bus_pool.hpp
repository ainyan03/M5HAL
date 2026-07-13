// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_BUS_POOL_HPP_
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HPP_

#include "remote.hpp"

// arduino-esp32 only (see server_bus_pool.inl's HAS_ARDUINO_BUS_CONFIG_):
// other Arduino cores (RP2040 / SAMD51) declare SPIClass inside their own
// `arduino::` namespace, not the global namespace — a bare forward-declare
// here would create a spurious, incompatible global ::SPIClass distinct
// from the real type once <SPI.h> is included elsewhere in the same TU.
#if defined(M5HAL_FRAMEWORK_HAS_ARDUINO) && M5HAL_FRAMEWORK_HAS_ARDUINO && defined(ESP_PLATFORM)
class TwoWire;
class SPIClass;
#endif

namespace m5::hal::v2::remote {

using error_t = error::error_t;

namespace detail {

inline int16_t readI16LE(const uint8_t* p);
inline void writeI16LE(uint8_t* p, int16_t value);

}  // namespace detail

static constexpr size_t kServerBusPoolSlots = bytecode::kMaxBusBindings;
static constexpr size_t kServerPinConfigMax = 16;

using pins_claimed_fn_t = void (*)(void* ctx, const types::gpio_number_t* pins, size_t count);

#if (defined(M5HAL_FRAMEWORK_HAS_ARDUINO) && M5HAL_FRAMEWORK_HAS_ARDUINO) || \
    (defined(M5HAL_FRAMEWORK_HAS_ESPIDF) && M5HAL_FRAMEWORK_HAS_ESPIDF) ||   \
    (defined(M5HAL_FRAMEWORK_HAS_POSIX) && M5HAL_FRAMEWORK_HAS_POSIX)
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ACTIVE_BUS_CONFIG_ 1
#else
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ACTIVE_BUS_CONFIG_ 0
#endif

#if defined(M5HAL_FRAMEWORK_HAS_ESPIDF) && M5HAL_FRAMEWORK_HAS_ESPIDF
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_BUS_CONFIG_ 1
#else
#define M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_BUS_CONFIG_ 0
#endif

// ---- physical pool ----------------------------------------------------------

struct PhysI2CSlot {
    bool used        = false;
    bool adopted     = false;
    uint8_t refcount = 0;
    i2c::Bus bus;
    uint8_t cfg[kServerPinConfigMax];
    uint8_t cfg_len = 0;
#if defined(M5HAL_FRAMEWORK_HAS_ARDUINO) && M5HAL_FRAMEWORK_HAS_ARDUINO && defined(ESP_PLATFORM)
    ::TwoWire* arduino_wire = nullptr;
#endif
};

struct PhysSPISlot {
    bool used        = false;
    bool adopted     = false;
    uint8_t refcount = 0;
    spi::Bus bus;
    uint8_t cfg[kServerPinConfigMax];
    uint8_t cfg_len = 0;
#if defined(M5HAL_FRAMEWORK_HAS_ARDUINO) && M5HAL_FRAMEWORK_HAS_ARDUINO && defined(ESP_PLATFORM)
    ::SPIClass* arduino_spi = nullptr;
#endif
};

struct PhysUARTSlot {
    bool used        = false;
    uint8_t refcount = 0;
    uart::Bus bus;
    uint8_t cfg[kServerPinConfigMax];
    uint8_t cfg_len = 0;
};

struct PhysI2SSlot {
    bool used        = false;
    uint8_t refcount = 0;
    i2s::Bus bus;
    uint8_t cfg[kServerPinConfigMax];
    uint8_t cfg_len = 0;
};

struct ServerPhysicalBusPool {
    PhysI2CSlot i2c[kServerBusPoolSlots];
    PhysSPISlot spi[kServerBusPoolSlots];
    PhysUARTSlot uart[kServerBusPoolSlots];
    PhysI2SSlot i2s[kServerBusPoolSlots];

#if defined(M5HAL_FRAMEWORK_HAS_ARDUINO) && M5HAL_FRAMEWORK_HAS_ARDUINO && defined(ESP_PLATFORM)
    //! Register an externally initialized TwoWire with its physical pin claim.
    //! The pool borrows it only; pin setup and begin/end stay with the caller.
    result_t<void> adoptI2C(::TwoWire& wire, types::gpio_number_t scl, types::gpio_number_t sda);
    result_t<void> adoptSPI(::SPIClass& spi, types::gpio_number_t clk, types::gpio_number_t mosi,
                            types::gpio_number_t miso);
#endif
};

// ---- I2C pool ---------------------------------------------------------------

struct I2CSlot {
    bool used      = false;
    uint8_t bus_id = 0;
    int8_t phys    = -1;
    i2c::MasterAccessConfig acc_cfg;
    alignas(i2c::MasterAccessor) uint8_t acc_buf[sizeof(i2c::MasterAccessor)];
    i2c::MasterAccessor* acc = nullptr;
};

result_t<void> createI2C(I2CSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, data::ConstDataSpan pin_config,
                         Server& server, pins_claimed_fn_t pins_claimed_fn, void* pins_claimed_ctx);

void releaseI2C(I2CSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, bytecode::BytecodeRunner& runner);

// ---- SPI pool ---------------------------------------------------------------

struct SPISlot {
    bool used      = false;
    uint8_t bus_id = 0;
    int8_t phys    = -1;
    spi::MasterAccessConfig acc_cfg;
    alignas(spi::MasterAccessor) uint8_t acc_buf[sizeof(spi::MasterAccessor)];
    spi::MasterAccessor* acc = nullptr;
};

result_t<void> createSPI(SPISlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, data::ConstDataSpan pin_config,
                         Server& server, pins_claimed_fn_t pins_claimed_fn, void* pins_claimed_ctx);

void releaseSPI(SPISlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, bytecode::BytecodeRunner& runner);

// ---- UART pool --------------------------------------------------------------

struct UARTSlot {
    bool used      = false;
    uint8_t bus_id = 0;
    int8_t phys    = -1;
    uart::AccessConfig acc_cfg;
    alignas(uart::Accessor) uint8_t acc_buf[sizeof(uart::Accessor)];
    uart::Accessor* acc = nullptr;
};

result_t<void> createUART(UARTSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, data::ConstDataSpan pin_config,
                          Server& server, pins_claimed_fn_t pins_claimed_fn, void* pins_claimed_ctx);

void releaseUART(UARTSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, bytecode::BytecodeRunner& runner);

// ---- I2S pool ---------------------------------------------------------------

struct I2SSlot {
    bool used      = false;
    uint8_t bus_id = 0;
    int8_t phys    = -1;
    i2s::AccessConfig acc_cfg;
    alignas(i2s::Accessor) uint8_t acc_buf[sizeof(i2s::Accessor)];
    alignas(i2s::TxAccessor) uint8_t tx_buf[sizeof(i2s::TxAccessor)];
    alignas(i2s::RxAccessor) uint8_t rx_buf[sizeof(i2s::RxAccessor)];
    i2s::Accessor* acc  = nullptr;
    i2s::TxAccessor* tx = nullptr;
    i2s::RxAccessor* rx = nullptr;
};

result_t<void> createI2S(I2SSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, data::ConstDataSpan pin_config,
                         Server& server, pins_claimed_fn_t pins_claimed_fn, void* pins_claimed_ctx);

void releaseI2S(I2SSlot* slots, ServerPhysicalBusPool& phys, uint8_t bus_id, bytecode::BytecodeRunner& runner);

// ---- unified pool -----------------------------------------------------------

struct ServerBusPool {
    I2CSlot i2c[kServerBusPoolSlots];
    SPISlot spi[kServerBusPoolSlots];
    UARTSlot uart[kServerBusPoolSlots];
    I2SSlot i2s[kServerBusPoolSlots];
    Server* server                    = nullptr;
    ServerPhysicalBusPool* phys       = nullptr;
    pins_claimed_fn_t pins_claimed_fn = nullptr;
    void* pins_claimed_ctx            = nullptr;

    void releaseAll();
    spi::MasterAccessor* findSPIAccessor(uint8_t bus_id) const;
    static result_t<void> handler(void* ctx, bool create, types::bus_kind_t kind, uint8_t bus_id,
                                  data::ConstDataSpan pin_config);
};

}  // namespace m5::hal::v2::remote

#undef M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ESPIDF_BUS_CONFIG_
#undef M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ACTIVE_BUS_CONFIG_

#endif  // M5_HAL_REMOTE_SERVER_BUS_POOL_HPP_
