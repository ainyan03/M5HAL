// SPDX-License-Identifier: MIT

#include "bus.hpp"

namespace m5::hal::v1::bus {

const BusConfig& Accessor::getBusConfig(void) const
{
    return _bus.getConfig();
}

}  // namespace m5::hal::v1::bus
