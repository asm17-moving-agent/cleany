#include <array>
#include <cassert>
#include "wheel_order.hpp"

int main() {
  constexpr std::array<size_t, 4> expected = {0, 1, 3, 2};
  assert(cleany::kWireToMotorIndex == expected);
  const std::array<int, 4> wireTarget = {11, 22, 33, 44};  // FL FR RL RR
  std::array<int, 4> pcbTarget{};                          // M1 M2 M3 M4
  for (size_t wire = 0; wire < wireTarget.size(); ++wire)
    pcbTarget[cleany::kWireToMotorIndex[wire]] = wireTarget[wire];
  assert((pcbTarget == std::array<int, 4>{11, 22, 44, 33}));
  std::array<int, 4> wireState{};
  for (size_t wire = 0; wire < wireTarget.size(); ++wire)
    wireState[wire] = pcbTarget[cleany::kWireToMotorIndex[wire]];
  assert(wireState == wireTarget);
}
