#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gtest/gtest.h>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_loader.hpp>
#include <hardware_interface/system_interface.hpp>
#include "cleany_mujoco_observer/snapshot_observer.hpp"

#include <chrono>
#include <memory>
#include <thread>

TEST(SortingObserverPlugin, RootOverrideWorksThroughLoaderSubNode)
{
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  const char* xml = R"(<mujoco><worldbody>
    <geom type="plane" size="4 4 .1"/>
    <body name="chassis" pos="1 2 0"/>
    <body name="study_cafe_cup" pos="1 2 .05">
      <freejoint/><geom type="sphere" size=".1" mass=".1"/>
    </body>
  </worldbody></mujoco>)";
  char error[1024];
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
    mj_parseXMLString(xml, nullptr, error, sizeof(error)), mj_deleteSpec);
  ASSERT_NE(spec, nullptr) << error;
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_compile(spec.get(), nullptr), mj_deleteModel);
  ASSERT_NE(model, nullptr);
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  mj_forward(model.get(), data.get());
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({rclcpp::Parameter("sorting_observer.publish_contacts", true)});
  auto node = std::make_shared<rclcpp::Node>("test_sorting_observer", options);
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr received;
  auto subscription = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/simulation/contact_diagnostics", 10,
    [&received](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {received = message;});
  auto plugin = cleany_mujoco_observer::makeSnapshotObserver();
  ASSERT_TRUE(plugin->init(node->create_sub_node("sorting_observer"), model.get(), data.get()));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    data->time += 0.101;
    plugin->update(model.get(), data.get());
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(received, nullptr);
  ASSERT_EQ(received->status.size(), 2U);  // Summary and one floor/cup contact.
  EXPECT_EQ(received->header.frame_id, "base_link");
  EXPECT_EQ(received->status[0].name, "simulation_contact_snapshot");
  EXPECT_EQ(received->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  bool found_force = false;
  for (const auto& entry : received->status[1].values) {
    if (entry.key == "normal_force_N") {
      found_force = true;
      EXPECT_GT(std::stod(entry.value), 0.0);
    }
  }
  EXPECT_TRUE(found_force);
  plugin->cleanup();
}

TEST(SortingObserverPlugin, ExportsSynchronizedHardwareWrapper)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  EXPECT_TRUE(loader.isClassAvailable("cleany_mujoco_observer/ObservedMujocoSystem"));
  loader.loadLibraryForClass("cleany_mujoco_observer/ObservedMujocoSystem");
}
