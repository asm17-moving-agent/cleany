#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include "nav2_behavior_tree/bt_conversions.hpp"
#if __has_include("behaviortree_cpp_v3/bt_factory.h")
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#else
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"
#endif
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nlohmann/json.hpp"
#include "cleany_navigation/slip_recovery_policy.hpp"

namespace cleany_navigation {
/// Halt Nav2's active child before braking; bypass generic recovery on slip.
class SlipRecovery : public BT::ControlNode {
public:
  SlipRecovery(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name,config), policy_({getInput<double>("unknown_timeout").value(),
      getInput<double>("stop_timeout").value(),getInput<double>("settle").value()}) {
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;options.callback_group=group_;
    slip_sub_=node_->create_subscription<std_msgs::msg::String>("navigation/slip",1,
      [this](std_msgs::msg::String::ConstSharedPtr msg){
        try {
          const auto value=nlohmann::json::parse(msg->data);
          const auto state=value.at("state").get<std::string>();
          const double a=value.at("t0").get<double>(), b=value.at("t1").get<double>();
          if(!std::isfinite(a+b) || a<0 || b<=a ||
            (state!="clear" && state!="suspect" && state!="confirmed" && state!="unknown")) throw std::invalid_argument("invalid observation");
          state_=state;t0_=a;t1_=b;received_=steady();
        } catch(const std::exception &) {state_="unknown";}
      },options);
    odom_sub_=node_->create_subscription<nav_msgs::msg::Odometry>("odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
        const auto & v=msg->twist.twist;
        const double stamp=rclcpp::Time(msg->header.stamp).seconds();
        const double age=node_->now().seconds()-stamp;
        stopped_=msg->header.frame_id=="odom" && msg->child_frame_id=="base_link" &&
          stamp>odom_stamp_ && age>=0 && age<=.3 && std::isfinite(v.linear.x+v.linear.y+v.angular.z) &&
          std::hypot(v.linear.x,v.linear.y)<getInput<double>("stop_linear").value() &&
          std::abs(v.angular.z)<getInput<double>("stop_angular").value();
        odom_stamp_=stamp;odom_at_=steady();
      },options);
    gate_sub_=node_->create_subscription<geometry_msgs::msg::Twist>("motion/traction_cmd_vel",1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg){
        gate_stopped_=std::isfinite(msg->linear.x+msg->linear.y+msg->angular.z) &&
          std::hypot(msg->linear.x,msg->linear.y)<1e-4 && std::abs(msg->angular.z)<1e-4;
        gate_at_=steady();
      },options);
    brake_=node_->create_publisher<std_msgs::msg::Bool>("navigation/traction_brake",1);
    status_=node_->create_publisher<std_msgs::msg::String>("navigation/traction_recovery",1);
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("unknown_timeout",3.,"Maximum unknown duration"),
      BT::InputPort<double>("stop_timeout",5.,"Stop verification deadline"),
      BT::InputPort<double>("settle",.5,"Continuous stopped duration"),
      BT::InputPort<double>("stop_linear",.01,"Stopped linear speed"),
      BT::InputPort<double>("stop_angular",.03,"Stopped angular speed"),
      BT::InputPort<double>("observation_timeout",1.5,"Maximum observation age")};
  }
  void halt() override {
    publish(true,"idle");
    fresh_cycle_=true;
    BT::ControlNode::halt();
  }
private:
  double steady() const {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void publish(bool brake, const std::string & phase) {
    std_msgs::msg::Bool message;message.data=brake;brake_->publish(message);
    std_msgs::msg::String status;
    status.data=nlohmann::json({{"phase",phase},{"reason",policy_.reason},
      {"retry",policy_.phase==SlipRecoveryPolicy::Phase::Retry}}).dump();status_->publish(status);
    if(phase!=last_phase_) {RCLCPP_INFO(node_->get_logger(),"Traction recovery: %s (%s)",phase.c_str(),policy_.reason.c_str());last_phase_=phase;}
  }
  BT::NodeStatus tick() override {
    if(childrenCount()!=2)throw BT::RuntimeError("SlipRecovery requires normal and longitudinal children");
    executor_.spin_some();const double now=steady();
    geometry_msgs::msg::PoseStamped goal;
    const bool has_goal=config().blackboard->get("goal",goal);
    const auto stamp=has_goal?rclcpp::Time(goal.header.stamp).nanoseconds():goal_stamp_;
    if(fresh_cycle_ || status()!=BT::NodeStatus::RUNNING || (has_goal && stamp!=goal_stamp_)) {
      haltChildren();policy_.reset(now);retry_started_=0.;ready_=false;fresh_cycle_=false;goal_stamp_=stamp;
    }
    const double age=node_->now().seconds()-t1_;
    bool reliable=now-received_<=.6 && age>=0 && age<=getInput<double>("observation_timeout").value() && state_!="unknown";
    if(policy_.phase==SlipRecoveryPolicy::Phase::Retry && t0_<retry_started_)reliable=false;
    if(reliable)ready_=true;
    const auto previous=policy_.phase;
    policy_.update(now,reliable,state_=="confirmed",stopped_ && now-odom_at_<=.3 && gate_stopped_ && now-gate_at_<=.15);
    if(previous!=policy_.phase && policy_.phase==SlipRecoveryPolicy::Phase::Braking) {publish(true,"braking");haltChildren();}
    if(previous!=policy_.phase && policy_.phase==SlipRecoveryPolicy::Phase::Retry)retry_started_=node_->now().seconds();
    using Phase=SlipRecoveryPolicy::Phase;
    if(policy_.phase==Phase::Braking) {publish(true,"braking");return BT::NodeStatus::RUNNING;}
    if(policy_.phase==Phase::Failed) {haltChildren();publish(true,"failed");return BT::NodeStatus::FAILURE;}
    if(!ready_) {publish(true,"warming_up");return BT::NodeStatus::RUNNING;}
    const bool retry=policy_.phase==Phase::Retry;
    publish(false,retry?"longitudinal_retry":"normal");
    const auto result=children_nodes_[retry?1:0]->executeTick();
    if(result==BT::NodeStatus::FAILURE) {
      haltChildren();policy_.fail(now,retry?"longitudinal_retry_failed":"navigation_failed");
      publish(true,"braking");return BT::NodeStatus::RUNNING;
    }
    if(result==BT::NodeStatus::SUCCESS) {haltChildren();publish(true,"succeeded");}
    return result;
  }
  SlipRecoveryPolicy policy_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr slip_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gate_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  double received_=0,t0_=0,t1_=0,odom_at_=0,odom_stamp_=0,retry_started_=0,gate_at_=0;
  bool stopped_=false,ready_=false,gate_stopped_=false,fresh_cycle_=true;
  int64_t goal_stamp_=0;
  std::string state_="unknown",last_phase_;
};
}
BT_REGISTER_NODES(factory) {factory.registerNodeType<cleany_navigation::SlipRecovery>("SlipRecovery");}
