#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <behaviortree_cpp_v3/bt_factory.h>
using Path=std::vector<int>;
int plans=0,follows=0,backups=0;bool first_fails=false,follow_fails=false,recoverable=false;
class Plan : public BT::SyncActionNode {
public:using BT::SyncActionNode::SyncActionNode;
 static BT::PortsList providedPorts(){return {BT::OutputPort<Path>("path"),BT::InputPort<std::string>("goal"),BT::InputPort<std::string>("planner_id")};}
 BT::NodeStatus tick() override {++plans;if((plans==1&&!first_fails)||(recoverable&&backups>0)){setOutput("path",Path{42});return BT::NodeStatus::SUCCESS;}setOutput("path",Path{});return BT::NodeStatus::FAILURE;}
};
class Follow : public BT::StatefulActionNode {
public:using BT::StatefulActionNode::StatefulActionNode;
 static BT::PortsList providedPorts(){return {BT::InputPort<Path>("path"),BT::InputPort<std::string>("controller_id"),BT::InputPort<std::string>("goal_checker_id")};}
 BT::NodeStatus step(){Path p;assert(getInput("path",p));assert(p==Path{42});++follows;if(follow_fails)return BT::NodeStatus::FAILURE;return follows>=2?BT::NodeStatus::SUCCESS:BT::NodeStatus::RUNNING;}
 BT::NodeStatus onStart() override{return step();}BT::NodeStatus onRunning() override{return step();}void onHalted() override{}
};
class Backup : public BT::SyncActionNode {
public:using BT::SyncActionNode::SyncActionNode;
 static BT::PortsList providedPorts(){return {BT::InputPort<double>("backup_dist"),BT::InputPort<double>("backup_speed"),BT::InputPort<double>("time_allowance")};}
 BT::NodeStatus tick() override{++backups;return BT::NodeStatus::FAILURE;}
};
// Each unit-test tick stands for a scheduled (5-second) replanning tick.
class ScheduledTick : public BT::DecoratorNode {
public:using BT::DecoratorNode::DecoratorNode;
 static BT::PortsList providedPorts(){return {BT::InputPort<double>("hz")};}
 BT::NodeStatus tick() override{return child_node_->executeTick();}
};
class Wait : public BT::SyncActionNode {
public:using BT::SyncActionNode::SyncActionNode;static BT::PortsList providedPorts(){return {BT::InputPort<double>("wait_duration")};}BT::NodeStatus tick()override{return BT::NodeStatus::SUCCESS;}
};
BT::NodeStatus run(const std::string& xml){
 plans=follows=backups=0;BT::BehaviorTreeFactory f;
 f.registerFromPlugin("/opt/ros/humble/lib/libnav2_pipeline_sequence_bt_node.so");
 f.registerFromPlugin("/opt/ros/humble/lib/libnav2_recovery_node_bt_node.so");
 f.registerNodeType<Plan>("ComputePathToPose");f.registerNodeType<Follow>("FollowPath");f.registerNodeType<Backup>("BackUp");f.registerNodeType<ScheduledTick>("RateController");f.registerNodeType<Wait>("Wait");
 auto bb=BT::Blackboard::create();bb->set("goal",std::string("goal"));auto tree=f.createTreeFromFile(xml,bb);
 auto result=tree.tickRoot();for(int i=0;i<10&&result==BT::NodeStatus::RUNNING;++i)result=tree.tickRoot();return result;
}
int main(int argc,char**argv){assert(argc==2);
 assert(run(argv[1])==BT::NodeStatus::SUCCESS);assert(backups==0);
 first_fails=true;assert(run(argv[1])==BT::NodeStatus::FAILURE);assert(backups==4&&plans==5&&follows==0);
 first_fails=false;follow_fails=true;assert(run(argv[1])==BT::NodeStatus::FAILURE);assert(backups==4);
 first_fails=true;follow_fails=false;recoverable=true;assert(run(argv[1])==BT::NodeStatus::SUCCESS);assert(backups==1&&follows==2);
 std::cout<<"PASS: failed backup permits replan, initial/controller persistent failure terminates after four recoveries, successful controller preserved\n";
}
