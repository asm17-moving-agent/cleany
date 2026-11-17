#pragma once

#include <chrono>
#include <stdexcept>
#include <thread>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace cleany_navigation::mppi::test
{

/** Wait for both background executors before a test can destroy its costmap. */
class TestCostmapROS : public nav2_costmap_2d::Costmap2DROS
{
public:
  using Costmap2DROS::Costmap2DROS;

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    const auto result = Costmap2DROS::on_configure(state);
    if (result != nav2_util::CallbackReturn::SUCCESS) {
      return result;
    }
    // Humble's executor cancel can precede spin() in a short-lived test. Receiving
    // a synthetic TF proves the private listener executor has actually started.
    auto publisher_node = rclcpp::Node::make_shared("mppi_costmap_startup");
    tf2_ros::StaticTransformBroadcaster broadcaster(publisher_node);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = publisher_node->now();
    transform.header.frame_id = "mppi_test_startup_parent";
    transform.child_frame_id = "mppi_test_startup_child";
    transform.transform.rotation.w = 1.0;
    broadcaster.sendTransform(transform);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      if (executor_->is_spinning() && tf_buffer_->canTransform(
          transform.header.frame_id, transform.child_frame_id, tf2::TimePointZero))
      {
        return result;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("Costmap test executors did not become ready");
  }
};

}  // namespace cleany_navigation::mppi::test
