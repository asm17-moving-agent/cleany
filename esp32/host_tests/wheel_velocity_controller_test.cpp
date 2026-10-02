#include <array>
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

void testPerWheelConfigsAreIndependent() {
  const std::array<cleany::WheelVelocityControllerConfig, 4> configs{{
      {11.0F, 6.0F, 6.0F, 100.0F},
      {10.0F, 7.0F, 8.0F, 90.0F},
      {12.0F, 4.0F, 3.0F, 100.0F},
      {9.0F, 2.0F, 1.0F, 80.0F},
  }};
  std::array<cleany::WheelVelocityController, 4> controllers{
      cleany::WheelVelocityController{configs[0]},
      cleany::WheelVelocityController{configs[1]},
      cleany::WheelVelocityController{configs[2]},
      cleany::WheelVelocityController{configs[3]},
  };
  for (size_t i = 0; i < controllers.size(); ++i) {
    const auto& config = configs[i];
    const float expected = 2.0F / config.maximumTargetRadPerSecond *
        config.outputLimitPercent + config.proportionalGain +
        config.integralGain * 0.01F;
    assert(std::fabs(controllers[i].update(2.0F, 1.0F, 0.01F) - expected) < 0.001F);
  }
  controllers[0].update(2.0F, 1.0F, 0.01F);
  assert(std::fabs(controllers[0].integralError() - 0.02F) < 0.0001F);
  for (size_t i = 1; i < controllers.size(); ++i) {
    assert(std::fabs(controllers[i].integralError() - 0.01F) < 0.0001F);
  }
}

}  // namespace

int main() {
  testFeedForwardProvidesNominalPwm();
  testIntegralCorrectsPersistentLoadError();
  testSaturationDoesNotWindUpIntegral();
  testStopResetsController();
  testOutputCannotOpposeTargetDirection();
  testPerWheelConfigsAreIndependent();
}
