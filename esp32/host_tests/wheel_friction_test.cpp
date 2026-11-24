#include <cassert>
#include <cmath>
#include "wheel_velocity_controller.hpp"
#include "wheel_friction_values.hpp"

int main() {
  using namespace cleany;
  const auto& f = kRrFriction.forward;
  assert(holdingFeedForward(f, 0, 0) == 0);
  assert(holdingFeedForward(f, 0.2F, 2) == f.holdPercent.front());
  assert(holdingFeedForward(f, 1, 9) == 9);
  StartFriction start;
  assert(start.update(0.1F, 0, 60, 1000, f).floorPercent == f.startPercent);
  assert(start.update(0.2F, 0, 62, 6000, f).floorPercent == f.startPercent);
  assert(start.update(0.2F, 0, 63, 11000, f).floorPercent == 0);
  assert(start.update(0.2F, 0, 61, 16000, f).floorPercent == 0);
  start.reset();
  assert(start.update(-0.1F, 0, INT32_MIN, 21000, f).floorPercent == f.startPercent);
  assert(start.update(-0.2F, 0, INT32_MAX-2, 26000, f).floorPercent == 0);
  start.reset();
  assert(start.update(-0.1F, 0, 0, 30000, f).floorPercent > 0);
  assert(start.update(-0.2F, 0, 0, 180000, f).floorPercent == 0);  // scheduler gap
  start.reset();
  for (int us=0; us<=190000; us+=5000) {
    const auto s = start.update(0.2F, 0, 29, us, f);
    if (us == 25000) assert(s.floorPercent == f.startPercent);
    if (us == 80000) assert(s.floorPercent == f.startPercent+6.0F);
    if (us == 185000) assert(s.floorPercent == 0);
  }
  assert(start.update(0.2F, 0.2F, 32, 195000, f).floorPercent == 0);
  for (int us=200000; us<=260000; us+=5000)
    start.update(0.2F, 0, 32, us, f);
  assert(start.update(0.2F, 0, 32, 265000, f).floorPercent > f.startPercent);
  assert(start.update(0.2F, 0.2F, 33, 270000, f).floorPercent == 0);
  assert(start.update(0.6F, 0, 33, 275000, f).floorPercent == 0);
  start.reset();
  assert(start.update(0.2F, 0, 100, 0, f).anglePercent == 0);
  assert(start.update(0.2F, 0, 100, 5000, f).anglePercent > 0);
  assert(start.update(0.2F, 0, 102, 10000, f).anglePercent < 0);
  for (int us=15000; us<=1500000; us+=5000) {
    const auto output = start.update(0.2F, 0, 102, us, f);
    assert(std::fabs(output.anglePercent) <= 3.0F);
    if (us >= 1000000) assert(output.anglePercent == 0);
    if (us == 1000000) assert(output.integralTransferPercent == 3.0F);
    else assert(output.integralTransferPercent == 0);
  }
  start.reset();
  assert(start.update(-0.2F, 0, -100, 0, f).anglePercent == 0);
  assert(start.update(-0.2F, 0, -100, 5000, f).anglePercent < 0);
  assert(start.update(-0.6F, 0, -100, 10000, f).anglePercent == 0);
  WheelVelocityController c({}, &kRrFriction);
  assert(c.update(-0.1F, 0, 0.005F, 8, 0) <= -kRrFriction.reverse.startPercent);
  assert(c.update(0, 0.2F, 0.005F, 6, 5000) == 0);
  assert(c.integralError() == 0);
  assert(c.update(0.1F, 0, 0.005F, 6, 10000) >= f.startPercent);
}
