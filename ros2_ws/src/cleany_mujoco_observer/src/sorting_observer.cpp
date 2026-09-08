#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "cleany_mujoco_observer/contact_observation.hpp"
#include "cleany_mujoco_observer/geom_bounds.hpp"
#include "cleany_mujoco_observer/snapshot_observer.hpp"

#include <cmath>
#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

namespace cleany_mujoco_observer
{
// This internal reader only reads a synchronized private snapshot. It never
// assigns qpos/qvel/ctrl, applies forces, or changes an equality constraint.
class SortingObserver
  : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData*) override
  {
    base_id_ = mj_name2id(model, mjOBJ_BODY, "chassis");
    if (base_id_ < 0) {return false;}
    for (int i = 1; i < model->nbody; ++i) {
      const char* name = mj_id2name(model, mjOBJ_BODY, i);
      if (name && std::string(name).rfind("study_cafe_", 0) == 0 &&
          model->body_jntnum[i] > 0 &&
          model->jnt_type[model->body_jntadr[i]] == mjJNT_FREE)
      {
        bodies_.push_back(i);
        names_.emplace_back(name);
      }
    }
    publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/simulation/sorting_ground_truth", 10);
    bool publish_contacts = false;
    // The loader passes a sub-node. Node's templated getters prepend its
    // namespace; these parameters belong to the shared root parameter store.
    const auto parameters = node->get_node_parameters_interface();
    rclcpp::Parameter value;
    if (parameters->get_parameter("sorting_observer.publish_contacts", value)) {
      publish_contacts = value.as_bool();
    }
    int maximum_contacts = 256;
    if (parameters->get_parameter("sorting_observer.maximum_contacts", value)) {
      const auto requested = value.as_int();
      if (requested <= 0 || requested > 4096) {return false;}
      maximum_contacts = static_cast<int>(requested);
    }
    if (maximum_contacts <= 0 || maximum_contacts > 4096) {
      RCLCPP_ERROR(node->get_logger(), "maximum_contacts must be in [1, 4096]");
      return false;
    }
    maximum_contacts_ = static_cast<std::size_t>(maximum_contacts);
    if (publish_contacts) {
      contact_publisher_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/simulation/contact_diagnostics", 10);
      RCLCPP_INFO(node->get_logger(), "Read-only contact diagnostics enabled (max=%d)",
        maximum_contacts);
    }
    return !bodies_.empty();
  }

  void update(const mjModel* model, mjData* snapshot) override
  {
    const mjData* data = snapshot;
    if (data->time >= last_time_ && data->time - last_time_ < 0.1) {return;}
    last_time_ = data->time;
    visualization_msgs::msg::MarkerArray message;
    for (std::size_t n = 0; n < bodies_.size(); ++n) {
      const int id = bodies_[n];
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = rclcpp::Time(
        static_cast<int64_t>(std::llround(data->time * 1e9)));
      marker.ns = "simulation_ground_truth";
      marker.id = id;
      marker.text = names_[n];
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      std::array<double, 3> low, high;
      low.fill(std::numeric_limits<double>::infinity());
      high.fill(-std::numeric_limits<double>::infinity());
      for (int g = model->body_geomadr[id];
           g < model->body_geomadr[id] + model->body_geomnum[id]; ++g)
      {
        if (!model->geom_contype[g] && !model->geom_conaffinity[g]) {continue;}
        const auto bounds = geomBoundsInBase(model, data, g, base_id_);
        for (int row = 0; row < 3; ++row) {
          low[row] = std::min(low[row], bounds.low[row]);
          high[row] = std::max(high[row], bounds.high[row]);
        }
      }
      if (!std::isfinite(low[0])) {continue;}
      marker.pose.position.x = (low[0] + high[0]) / 2;
      marker.pose.position.y = (low[1] + high[1]) / 2;
      marker.pose.position.z = (low[2] + high[2]) / 2;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = high[0] - low[0];
      marker.scale.y = high[1] - low[1];
      marker.scale.z = high[2] - low[2];
      marker.color.r = 1.0;
      marker.color.a = 1.0;
      message.markers.push_back(marker);
    }
    publisher_->publish(message);
    if (contact_publisher_ && contact_publisher_->get_subscription_count() > 0) {
      publishContacts(model, data);
    }
  }

  void cleanup() override {publisher_.reset(); contact_publisher_.reset();}

private:
  void publishContacts(const mjModel* model, const mjData* data)
  {
    const auto contacts = observeContacts(model, data, base_id_, maximum_contacts_);
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.frame_id = "base_link";
    message.header.stamp = rclcpp::Time(
      static_cast<int64_t>(std::llround(data->time * 1e9)));
    auto add = [](auto& status, const std::string& key, const auto& value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(std::move(entry));
    };
    diagnostic_msgs::msg::DiagnosticStatus summary;
    summary.name = "simulation_contact_snapshot";
    summary.message = "Evaluation only; solved contact forces, not commanded forces";
    summary.level = contacts.size() == static_cast<std::size_t>(data->ncon) ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    add(summary, "source_contacts", std::to_string(data->ncon));
    add(summary, "published_contacts", std::to_string(contacts.size()));
    message.status.push_back(std::move(summary));
    for (const auto& contact : contacts) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "contact_" + std::to_string(contact.index);
      status.message = "Simulation contact";
      for (int side = 0; side < 2; ++side) {
        const auto suffix = std::to_string(side + 1);
        add(status, "body_" + suffix, contact.body_names[side]);
        add(status, "geom_" + suffix, contact.geom_names[side]);
        add(status, "geom_id_" + suffix, std::to_string(contact.geom_ids[side]));
      }
      for (int axis = 0; axis < 3; ++axis) {
        add(status, std::string("position_") + "xyz"[axis],
          std::to_string(contact.position[axis]));
      }
      add(status, "distance_m", std::to_string(contact.distance));
      add(status, "normal_force_N", std::to_string(contact.normal_force));
      add(status, "tangential_force_N", std::to_string(contact.tangential_force));
      message.status.push_back(std::move(status));
    }
    contact_publisher_->publish(message);
  }

  int base_id_{-1};
  double last_time_{-1.0};
  std::vector<int> bodies_;
  std::vector<std::string> names_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  std::size_t maximum_contacts_{256};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr contact_publisher_;
};

std::unique_ptr<mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase>
makeSnapshotObserver()
{
  return std::make_unique<SortingObserver>();
}
}  // namespace cleany_mujoco_observer
