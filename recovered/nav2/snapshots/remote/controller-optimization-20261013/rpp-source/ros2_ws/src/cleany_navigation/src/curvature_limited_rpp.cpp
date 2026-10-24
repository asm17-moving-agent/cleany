#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "cleany_navigation/rpp_yaw_limit.hpp"

namespace cleany_navigation
{
// Keep RPP's checked circular path when applying the platform yaw bound.
// The native check covers the original, longer arc; reducing both velocities
// follows a prefix of that arc. Collision exceptions are never suppressed.
class CurvatureLimitedRPP final : public
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    auto node = parent.lock();
    if (!node) {throw std::runtime_error("CurvatureLimitedRPP: expired node");}
    nav2_util::declare_parameter_if_not_declared(node, name + ".tracking_yaw_limit",
      rclcpp::ParameterValue(0.14));
    limit_ = node->get_parameter(name + ".tracking_yaw_limit").as_double();
    if (!std::isfinite(limit_) || limit_ <= 0.) {
      throw std::invalid_argument("tracking_yaw_limit must be finite and positive");
    }
    RegulatedPurePursuitController::configure(parent, name, tf, costmap);
    limit_handler_ = node->add_on_set_parameters_callback(
      [this, key = name + ".tracking_yaw_limit"](const std::vector<rclcpp::Parameter> & values) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & value : values) {
          if (value.get_name() == key &&
            (value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            value.as_double() != limit_))
          {
            result.successful = false;
            result.reason = "tracking_yaw_limit is configure-time only; cleanup and reconfigure";
          }
        }
        return result;
      });
  }

  void cleanup() override
  {
    limit_handler_.reset();
    RegulatedPurePursuitController::cleanup();
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * checker) override
  {
    auto command = RegulatedPurePursuitController::computeVelocityCommands(pose, speed, checker);
    const double scale = rpp_yaw_scale(command.twist.angular.z, limit_);
    command.twist.linear.x *= scale;
    command.twist.angular.z *= scale;
    return command;
  }
private:
  double limit_{0.14};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr limit_handler_;
};
}  // namespace cleany_navigation
PLUGINLIB_EXPORT_CLASS(cleany_navigation::CurvatureLimitedRPP, nav2_core::Controller)
