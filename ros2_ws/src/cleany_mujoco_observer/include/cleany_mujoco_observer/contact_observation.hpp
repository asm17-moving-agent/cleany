#pragma once

// 호출자가 일관된 snapshot을 소유해야 합니다. 이 reader는 physics 잠금을 잡지 않으며
// vendor가 동시에 복사하는 데이터를 전달해서는 안 됩니다.

#include <mujoco/mujoco.h>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace cleany_mujoco_observer
{
struct ContactObservation
{
  int index;
  std::array<int, 2> geom_ids;
  std::array<std::string, 2> body_names;
  std::array<std::string, 2> geom_names;
  std::array<double, 3> position;
  double distance;
  double normal_force;
  double tangential_force;
};

// 이미 계산된 snapshot만 읽으며 물리를 진행하거나 다시 계산하지 않습니다.
std::vector<ContactObservation> observeContacts(
  const mjModel* model, const mjData* data, int base_id, std::size_t maximum_contacts);
}  // namespace cleany_mujoco_observer
