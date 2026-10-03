#include "cleany_mujoco_observer/contact_observation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cleany_mujoco_observer
{
std::vector<ContactObservation> observeContacts(
  const mjModel* model, const mjData* data, int base_id, std::size_t maximum_contacts)
{
  if (!model || !data || base_id < 0 || base_id >= model->nbody) {
    throw std::invalid_argument("Contact observation requires a valid snapshot and base body");
  }
  std::vector<ContactObservation> result;
  const auto count = std::min(maximum_contacts, static_cast<std::size_t>(data->ncon));
  result.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto& contact = data->contact[i];
    // Flex contacts may not have two geom ids; do not invent body attribution.
    if (contact.geom[0] < 0 || contact.geom[1] < 0 ||
        contact.geom[0] >= model->ngeom || contact.geom[1] >= model->ngeom) {continue;}
    ContactObservation item{};
    item.index = static_cast<int>(i);
    for (int side = 0; side < 2; ++side) {
      const int geom = contact.geom[side];
      item.geom_ids[side] = geom;
      const char* body_name = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[geom]);
      const char* geom_name = mj_id2name(model, mjOBJ_GEOM, geom);
      item.body_names[side] = body_name ? body_name : "unnamed_body";
      item.geom_names[side] = geom_name ? geom_name : "geom_" + std::to_string(geom);
    }
    mjtNum delta[3], position[3], force[6];
    mju_sub3(delta, contact.pos, data->xpos + 3 * base_id);
    mju_mulMatTVec3(position, data->xmat + 9 * base_id, delta);
    mj_contactForce(model, data, static_cast<int>(i), force);
    std::copy(position, position + 3, item.position.begin());
    item.distance = contact.dist;
    item.normal_force = force[0];
    item.tangential_force = std::hypot(force[1], force[2]);
    result.push_back(std::move(item));
  }
  return result;
}
}  // namespace cleany_mujoco_observer
