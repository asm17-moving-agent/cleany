#pragma once
#include <cmath>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "behaviortree_cpp/bt_factory.h"
namespace cleany_axis_controller {
// An intentional yield is not a failed navigation recovery.
class YieldWait : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Wait> {
public:
  YieldWait(const std::string &name, const BT::NodeConfiguration &config)
  : BtActionNode(name, "wait", config) {}
  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<double>("wait_duration",15.0,"Yield duration in simulation seconds")});
  }
  void on_tick() override {
    double duration=0.;
    if (!getInput("wait_duration",duration) || !std::isfinite(duration) || duration <= 0.)
      throw BT::RuntimeError("YieldWait requires a finite positive wait_duration");
    goal_.time=rclcpp::Duration::from_seconds(duration);
  }
};
}
