#include <gtest/gtest.h>
#if __has_include("behaviortree_cpp_v3/bt_factory.h")
#include "behaviortree_cpp_v3/bt_factory.h"
#else
#include "behaviortree_cpp/bt_factory.h"
#endif
static BT::NodeStatus tickTree(BT::Tree &tree) {
#if __has_include("behaviortree_cpp_v3/bt_factory.h")
  return tree.tickRoot();
#else
  return tree.tickOnce();
#endif
}
struct Counts {int plan=0,wait=0,follow=0,halt=0,failures=0;bool hold=false;};
class Backoff : public BT::StatefulActionNode {
public:
  Backoff(const std::string &name,const BT::NodeConfiguration &config,Counts &counts)
  : StatefulActionNode(name,config),counts_(counts) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<double>("wait_duration")};}
  BT::NodeStatus onStart() override {++counts_.wait;return counts_.hold?BT::NodeStatus::RUNNING:BT::NodeStatus::SUCCESS;}
  BT::NodeStatus onRunning() override {return counts_.hold?BT::NodeStatus::RUNNING:BT::NodeStatus::SUCCESS;}
  void onHalted() override {++counts_.halt;}
private:Counts &counts_;
};
static void registerMocks(BT::BehaviorTreeFactory &factory,Counts &counts) {
  // Exercise the installed Nav2 RecoveryNode and the actual shipped XML.
  // Only external planner, controller and wait actions are replaced here.
  factory.registerFromPlugin(CLEANY_TEST_RECOVERY_LIBRARY);
  factory.registerSimpleAction("ComputePathToPose",[&](BT::TreeNode &node) {
    if(++counts.plan<=counts.failures)return BT::NodeStatus::FAILURE;
    node.setOutput("path",std::string("safe_path"));return BT::NodeStatus::SUCCESS;
  },{BT::InputPort<std::string>("goal"),BT::OutputPort<std::string>("path"),
     BT::InputPort<std::string>("planner_id"),BT::OutputPort<std::string>("error_code_id")});
  factory.registerBuilder<Backoff>("YieldWait",[&](const std::string &name,const BT::NodeConfiguration &config) {
    return std::make_unique<Backoff>(name,config,counts);
  });
  factory.registerSimpleAction("TruncatePathLocal",[](BT::TreeNode &node) {
    node.setOutput("output_path",std::string("upcoming"));return BT::NodeStatus::SUCCESS;
  },{BT::InputPort<std::string>("input_path"),BT::OutputPort<std::string>("output_path"),
     BT::InputPort<double>("distance_forward"),BT::InputPort<double>("distance_backward")});
  factory.registerSimpleCondition("IsPathValid",[](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
    {BT::InputPort<std::string>("path")});
  factory.registerSimpleAction("FollowPath",[&](BT::TreeNode &) {
    ++counts.follow;return BT::NodeStatus::SUCCESS;
  },{BT::InputPort<std::string>("path"),BT::InputPort<std::string>("controller_id"),
     BT::OutputPort<std::string>("error_code_id")});
}
TEST(InitialPlanRetry, TemporaryFailureRetriesBeforeFollowing) {
  Counts counts;counts.failures=1;BT::BehaviorTreeFactory factory;registerMocks(factory,counts);
  auto tree=factory.createTreeFromFile(CLEANY_TEST_NAVIGATION_BT);
  BT::NodeStatus status=BT::NodeStatus::IDLE;
  for(int i=0;i<10;++i) {status=tickTree(tree);if(status!=BT::NodeStatus::RUNNING)break;}
  EXPECT_EQ(status,BT::NodeStatus::SUCCESS);EXPECT_EQ(counts.plan,2);EXPECT_EQ(counts.wait,1);EXPECT_EQ(counts.follow,1);
}
TEST(InitialPlanRetry, PermanentFailureIsBoundedAndNeverFollows) {
  Counts counts;counts.failures=100;BT::BehaviorTreeFactory factory;registerMocks(factory,counts);
  auto tree=factory.createTreeFromFile(CLEANY_TEST_NAVIGATION_BT);
  BT::NodeStatus status=BT::NodeStatus::IDLE;
  for(int i=0;i<20;++i) {status=tickTree(tree);if(status!=BT::NodeStatus::RUNNING)break;}
  EXPECT_EQ(status,BT::NodeStatus::FAILURE);EXPECT_EQ(counts.plan,4);EXPECT_EQ(counts.wait,3);EXPECT_EQ(counts.follow,0);
}
TEST(InitialPlanRetry, HaltDuringBackoffPreventsAnotherPlanOrFollowing) {
  Counts counts;counts.failures=1;counts.hold=true;BT::BehaviorTreeFactory factory;registerMocks(factory,counts);
  auto tree=factory.createTreeFromFile(CLEANY_TEST_NAVIGATION_BT);
  EXPECT_EQ(tickTree(tree),BT::NodeStatus::RUNNING);tree.haltTree();
  EXPECT_EQ(counts.plan,1);EXPECT_EQ(counts.follow,0);EXPECT_EQ(counts.halt,1);
}
