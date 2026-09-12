#include <cassert>
#include <cmath>

#include "wheel_velocity_controller.hpp"

namespace {

void testFeedForwardProvidesNominalPwm() {
  cleany::WheelVelocityController controller;
  const float output = controller.update(5.0F, 5.0F, 0.01F);
  assert(std::fabs(output - (500.0F / 11.0F)) < 0.001F);
}

void testIntegralCorrectsPersistentLoadError() {
  cleany::WheelVelocityController controller;
  const float first = controller.update(5.0F, 4.0F, 0.01F);
  float output = first;
  for (int i = 0; i < 100; ++i) {
    output = controller.update(5.0F, 4.0F, 0.01F);
  }
  assert(output > first);
}

void testSaturationDoesNotWindUpIntegral() {
  cleany::WheelVelocityController controller;
  for (int i = 0; i < 100; ++i) {
    assert(controller.update(10.0F, 0.0F, 0.01F) == 100.0F);
  }
  assert(controller.integralError() == 0.0F);
}

void testStopResetsController() {
  cleany::WheelVelocityController controller;
  controller.update(5.0F, 0.0F, 0.1F);
  assert(controller.integralError() > 0.0F);
  assert(controller.update(0.0F, 0.0F, 0.01F) == 0.0F);
  assert(controller.integralError() == 0.0F);
}

void testOutputCannotOpposeTargetDirection() {
  cleany::WheelVelocityController controller;
  assert(controller.update(0.1F, 5.0F, 0.01F) == 0.0F);
  assert(controller.integralError() == 0.0F);
  assert(controller.update(-0.1F, -5.0F, 0.01F) == 0.0F);
  assert(controller.integralError() == 0.0F);
}

}  // namespace

int main() {
  testFeedForwardProvidesNominalPwm();
  testIntegralCorrectsPersistentLoadError();
  testSaturationDoesNotWindUpIntegral();
  testStopResetsController();
  testOutputCannotOpposeTargetDirection();
}
