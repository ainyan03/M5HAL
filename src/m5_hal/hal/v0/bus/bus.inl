
#include "bus.hpp"

namespace m5 {
namespace hal {
M5HAL_INLINE_V0 namespace v0 {
namespace bus {

const BusConfig& Accessor::getBusConfig(void) const
{
    return _bus.getConfig();
}

}  // namespace bus
}  // namespace v0
}  // namespace hal
}  // namespace m5
