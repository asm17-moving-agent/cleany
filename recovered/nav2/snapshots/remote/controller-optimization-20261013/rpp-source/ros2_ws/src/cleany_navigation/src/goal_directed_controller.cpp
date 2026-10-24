#include "cleany_navigation/goal_directed_controller.hpp"
#include "cleany_navigation/holonomic_generator.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
namespace cleany_navigation {
void GoalDirectedController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) {
  dwb_core::DWBLocalPlanner::configure(parent,name,tf,costmap);
  auto node=parent.lock();const auto key=name+".continuity_timeout_s";
  if(!node->has_parameter(key))node->declare_parameter(key,.5);
  continuity_timeout_=node->get_parameter(key).as_double();
  if(!std::isfinite(continuity_timeout_) || continuity_timeout_<=0.)
    throw std::invalid_argument(key+" must be positive and finite");
  selected_.reset();
}
void GoalDirectedController::setPlan(const nav_msgs::msg::Path &path) {
  selected_.reset();
  dwb_core::DWBLocalPlanner::setPlan(path);
}
nav_2d_msgs::msg::Twist2DStamped GoalDirectedController::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped &pose,
  const nav_2d_msgs::msg::Twist2D &velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> &results) {
  auto *generator=dynamic_cast<HolonomicGenerator *>(traj_generator_.get());
  struct ClearContext {
    HolonomicGenerator *generator;
    ~ClearContext() {if(generator){generator->clearGoalContext();generator->clearContinuityContext();}}
  } clear{generator};
  if(generator) {
    generator->clearGoalContext();
    generator->clearContinuityContext();
    if(selected_ && selected_->pose.header.frame_id==pose.header.frame_id) {
      const double age=(rclcpp::Time(pose.header.stamp)-rclcpp::Time(selected_->pose.header.stamp)).seconds();
      if(age>0. && age<=continuity_timeout_) {
        const double yaw=selected_->pose.pose.theta-pose.pose.theta;
        auto v=selected_->velocity;
        v.x=std::cos(yaw)*selected_->velocity.x-std::sin(yaw)*selected_->velocity.y;
        v.y=std::sin(yaw)*selected_->velocity.x+std::cos(yaw)*selected_->velocity.y;
        generator->setContinuityContext(v);
      }
    }
    if(!global_plan_.poses.empty() && !pose.header.frame_id.empty() &&
       !global_plan_.header.frame_id.empty()) {
      nav_2d_msgs::msg::Pose2DStamped goal,transformed;
      goal.header.frame_id=global_plan_.header.frame_id;
      goal.pose=global_plan_.poses.back();
      // Same latest-transform semantics as DWB's actual global goal. A failed
      // transform adds no candidates; base DWB retains its own failure policy.
      if(nav_2d_utils::transformPose(tf_,pose.header.frame_id,goal,transformed,transform_tolerance_))
        generator->setGoalContext(pose.pose,transformed.pose);
    }
  }
  selected_.reset(); // Exceptions and failed cycles cannot retain a command.
  auto result=dwb_core::DWBLocalPlanner::computeVelocityCommands(pose,velocity,results);
  if(std::hypot(result.velocity.x,result.velocity.y)>1e-6)selected_=Selected{pose,result.velocity};
  return result;
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::GoalDirectedController,nav2_core::Controller)
