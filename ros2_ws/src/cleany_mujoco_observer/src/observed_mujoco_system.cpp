#include <mujoco_ros2_control/mujoco_system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "cleany_mujoco_observer/snapshot_observer.hpp"
#include "cleany_mujoco_observer/scheduled_cameras.hpp"

#include <chrono>
#include <algorithm>
#include <memory>

namespace cleany_mujoco_observer
{
// Physics, controllers and writes remain in the installed backend; optional
// cameras render a read-only snapshot at independently scheduled rates.
// Its plugin read() data races with physics-loop mj_copyData. Instead, use the
// public get_data API which copies under that same simulation mutex.
class ObservedMujocoSystem : public mujoco_ros2_control::MujocoSystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override
  {
    auto backend_info = info;
    const bool scheduled = info.hardware_parameters.count("scheduled_cameras") &&
      info.hardware_parameters.at("scheduled_cameras") == "true";
    if (scheduled) {
      // Initialize GLFW synchronously before the backend starts its rendering
      // threads. Concurrent glfwInit calls race inside GLFW's global mutexes.
      if (!glfwInit()) {return hardware_interface::CallbackReturn::ERROR;}
      backend_info.hardware_parameters["camera_publish_rate"] = "0.0";
      // The installed backend emits one startup frame even at zero rate.
      // Quarantine it: the scheduled renderer is the only public sensor source.
      backend_info.sensors.erase(std::remove_if(backend_info.sensors.begin(), backend_info.sensors.end(),
        [](const auto& sensor) {
          return sensor.name == "head_realsense_rgb" || sensor.name == "head_realsense_depth" ||
            sensor.name == "left_wrist_rgb" || sensor.name == "right_wrist_rgb";
        }), backend_info.sensors.end());
      for(const auto& name: {"head_realsense_rgb", "head_realsense_depth", "left_wrist_rgb", "right_wrist_rgb"}) {
        hardware_interface::ComponentInfo sensor;
        sensor.name=name;
        const auto topic=std::string("/cleany/internal/disabled_vendor/")+name;
        sensor.parameters={{"frame_name",std::string(name)+"_disabled"},
          {"image_topic",topic+"/color"},{"info_topic",topic+"/camera_info"},{"depth_topic",topic+"/depth"}};
        backend_info.sensors.push_back(sensor);
      }
    }
    const auto result = MujocoSystemInterface::on_init(backend_info);
    if (result != hardware_interface::CallbackReturn::SUCCESS) {return result;}
    mjModel* model = nullptr;
    get_model(model);
    model_.reset(model);
    if (!model_) {return hardware_interface::CallbackReturn::ERROR;}
    mjData* snapshot = nullptr;
    get_data(snapshot);
    snapshot_.reset(snapshot);
    // Publisher-only node; no executor/thread or live parameter mutation.
    // Launch parameter files are inherited through global ROS arguments.
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>("sorting_observer", options);
    observer_ = makeSnapshotObserver();
    if (!snapshot_ || !observer_->init(node_, model_.get(), snapshot_.get())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Sorting oracle uses mutex-protected private snapshots");
    if (scheduled) cameras_=std::make_unique<ScheduledCameras>(model_.get(), [this](mjData*& data) {get_data(data);});
    return result;
  }

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override
  {
    const auto result = MujocoSystemInterface::read(time, period);
    if (result != hardware_interface::return_type::OK || !observer_) {return result;}
    const auto now = std::chrono::steady_clock::now();
    if (now < next_sample_) {return result;}
    next_sample_ = now + std::chrono::milliseconds(100);
    // Allocated during init, never shared with the renderer or physics thread.
    // No set_data(), mj_forward(), mj_step(), force or command changes here.
    auto* snapshot = snapshot_.get();
    get_data(snapshot);
    observer_->update(model_.get(), snapshot);
    return result;
  }

private:
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> snapshot_{nullptr, mj_deleteData};
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase> observer_;
  std::chrono::steady_clock::time_point next_sample_{};
  std::unique_ptr<ScheduledCameras> cameras_;
};
}  // namespace cleany_mujoco_observer

PLUGINLIB_EXPORT_CLASS(cleany_mujoco_observer::ObservedMujocoSystem, hardware_interface::SystemInterface)
