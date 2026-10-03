#include "cleany_axis_controller/yield_wait.hpp"
// Nav2 exports BehaviorTree.CPP v3 on Humble and v4 on Jazzy.
#if __has_include("behaviortree_cpp_v3/bt_factory.h")
#include "behaviortree_cpp_v3/bt_factory.h"
#else
#include "behaviortree_cpp/bt_factory.h"
#endif

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<cleany_axis_controller::YieldWait>("YieldWait");
}
