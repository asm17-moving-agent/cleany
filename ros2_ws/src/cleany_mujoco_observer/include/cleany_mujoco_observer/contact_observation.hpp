#pragma once

// Callers must own a consistent snapshot: this reader does not acquire the
// physics lock and must never receive the vendor's concurrently copied data.

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

// Reads the existing solved snapshot only: never steps/recomputes physics.
std::vector<ContactObservation> observeContacts(
  const mjModel* model, const mjData* data, int base_id, std::size_t maximum_contacts);
}  // namespace cleany_mujoco_observer
