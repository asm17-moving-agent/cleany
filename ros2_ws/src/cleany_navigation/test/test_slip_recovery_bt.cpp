#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nlohmann/json.hpp"

class Drive : public BT::ActionNodeBase {
public:
  Drive(const std::string & n,const BT::NodeConfiguration & c): BT::ActionNodeBase(n,c){}
  static BT::PortsList providedPorts(){return {};}
  BT::NodeStatus tick() override {if(name()=="normal")++normal;else ++retry;return BT::NodeStatus::RUNNING;}
  void halt() override {if(status()==BT::NodeStatus::RUNNING)++halts;setStatus(BT::NodeStatus::IDLE);}
  static int normal,retry,halts;
};
int Drive::normal=0;int Drive::retry=0;int Drive::halts=0;

TEST(SlipRecoveryBT, HaltsBrakesLatchesRetriesAndFailsOnNewSlip) {
  if(!rclcpp::ok())rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("slip_bt_test");
  auto slip=node->create_publisher<std_msgs::msg::String>("navigation/slip",1);
  auto odom=node->create_publisher<nav_msgs::msg::Odometry>("odom",rclcpp::SensorDataQoS());
  auto gate=node->create_publisher<geometry_msgs::msg::Twist>("motion/traction_cmd_vel",1);
  BT::BehaviorTreeFactory factory;factory.registerFromPlugin(CLEANY_SLIP_BT_LIBRARY);
  factory.registerNodeType<Drive>("Drive");
  auto board=BT::Blackboard::create();board->set("node",node);
  auto tree=factory.createTreeFromText(R"(<root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">
    <SlipRecovery settle="0.05" unknown_timeout="1.8"><Drive name="normal"/><Drive name="retry"/></SlipRecovery>
    </BehaviorTree></root>)",board);
  const auto start=std::chrono::steady_clock::now();
  auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();};
  auto send=[&](std::string state,double t0=0.){
    std_msgs::msg::String s;const double now=node->now().seconds();
    s.data=nlohmann::json({{"state",state},{"t0",t0>0?t0:now-1.1},{"t1",now-.05}}).dump();slip->publish(s);
    nav_msgs::msg::Odometry o;o.header.frame_id="odom";o.child_frame_id="base_link";o.header.stamp=node->now();odom->publish(o);
    gate->publish(geometry_msgs::msg::Twist());
    std::this_thread::sleep_for(std::chrono::milliseconds(15));return tree.tickRoot();
  };
  tree.tickRoot();EXPECT_EQ(Drive::normal,0); // No evidence on startup.
  while(elapsed()<.3)send("clear");
  ASSERT_GT(Drive::normal,0);const auto normal_before=Drive::normal;
  while(elapsed()<.45)send("confirmed");
  EXPECT_EQ(Drive::normal,normal_before);EXPECT_GT(Drive::halts,0);
  // Stationary clear windows cannot return the tree to the normal child.
  while(elapsed()<.65)send("clear");
  ASSERT_GT(Drive::retry,0);EXPECT_EQ(Drive::normal,normal_before);
  // A delayed pre-retry confirmation cannot consume the retry budget.
  const auto old=node->now().seconds()-1.;
  while(elapsed()<.85)send("confirmed",old);
  EXPECT_EQ(Drive::normal,normal_before);EXPECT_EQ(tree.rootNode()->status(),BT::NodeStatus::RUNNING);
  while(elapsed()<1.15)send("clear");
  const auto retry_before=Drive::retry;
  // A new post-retry comparison causes a second brake, then terminal failure.
  const auto new_start=node->now().seconds()-.1;
  BT::NodeStatus result=BT::NodeStatus::RUNNING;
  while(elapsed()<1.45 && result==BT::NodeStatus::RUNNING)result=send("confirmed",new_start);
  EXPECT_EQ(result,BT::NodeStatus::FAILURE);EXPECT_EQ(Drive::retry,retry_before);
  geometry_msgs::msg::PoseStamped next_goal;next_goal.header.stamp=node->now();board->set("goal",next_goal);
  send("clear");EXPECT_GT(Drive::normal,normal_before);
  tree.haltTree();
}
