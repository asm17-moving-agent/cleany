#pragma once

#include <cstdint>
#include "rcl/rcl.h"

namespace cleany::sensors {
void start(uint32_t bootId);
bool open(rcl_node_t* node, const rmw_qos_profile_t& qos);
bool publish(bool clockSynchronized);
void close(rcl_node_t* node);
}  // namespace cleany::sensors
