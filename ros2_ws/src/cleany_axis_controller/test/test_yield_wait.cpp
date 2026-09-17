#include <gtest/gtest.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include "cleany_axis_controller/yield_wait.hpp"
class InspectWait : public cleany_axis_controller::YieldWait {
public:
  using YieldWait::YieldWait;
  int seconds() {return goal_.time.sec;}
};
TEST(YieldWait, IntentionalWaitPreservesExistingRecoveryCountAndRejectsInvalidDuration) {
  if (!rclcpp::ok()) rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("yield_wait_test","/yield_wait_test");
  using Wait=nav2_msgs::action::Wait;
  auto server=rclcpp_action::create_server<Wait>(node,"wait",
    [](auto, auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](auto){return rclcpp_action::CancelResponse::ACCEPT;}, [](auto){});
  BT::NodeConfiguration config;
  config.blackboard=BT::Blackboard::create();
  config.blackboard->set("node",node);
  config.blackboard->set("bt_loop_duration",std::chrono::milliseconds(10));
  config.blackboard->set("server_timeout",std::chrono::milliseconds(1000));
  config.blackboard->set("wait_for_service_timeout",std::chrono::milliseconds(2000));
  config.blackboard->set("number_recoveries",7);
  config.input_ports["wait_duration"]="15.0";
  InspectWait wait("yield",config);wait.on_tick();wait.on_tick();
  EXPECT_EQ(wait.seconds(),15);
  EXPECT_EQ(config.blackboard->get<int>("number_recoveries"),7);
  config.input_ports["wait_duration"]="-1.0";
  InspectWait bad("invalid_yield",config);
  EXPECT_THROW(bad.on_tick(),BT::RuntimeError);
}
