#include "cleany_navigation/yield_wait.hpp"
// Nav2는 Humble에서 BehaviorTree.CPP v3, Jazzy에서 v4를 제공합니다.
#if __has_include("behaviortree_cpp_v3/bt_factory.h")
#include "behaviortree_cpp_v3/bt_factory.h"
#else
#include "behaviortree_cpp/bt_factory.h"
#endif

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<cleany_axis_controller::YieldWait>("YieldWait");
}
