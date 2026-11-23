#pragma once

#include "cleany_interfaces/msg/tof_frame.h"

namespace cleany::sensors {
struct TofSample {
  cleany_interfaces__msg__TofFrame frame{};
  int64_t acquisitionUs{};
};

void startTof();
bool takeTof(unsigned channel, TofSample& sample);
}  // namespace cleany::sensors
