#include <cassert>
#include <cmath>

#include "wheel_duty.hpp"

int main() {
  using namespace cleany;
  assert(dutyTicks(0) == 0);
  assert(dutyTicks(5.0F) == 102);
  assert(dutyTicks(-100.0F) == -2040);
  assert(dutyTicks(100.0F) == 2040);
  for (int i = -99000; i <= 99000; ++i) {
    const float pct = i * 0.001F;
    assert(std::abs(dutyPercent(dutyTicks(pct)) - pct) <= 50.0F/kPwmPeriodTicks + 0.00001F);
  }
}
