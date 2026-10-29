// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <chrono>
#include <cmath>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cleany_navigation/mppi/tools/path_handler.hpp"
#include "tf2_ros/transform_broadcaster.h"

// Tests path handling

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace cleany_navigation::mppi;  // NOLINT

class PathHandlerWrapper : public PathHandler
{
public:
  PathHandlerWrapper()
  : PathHandler() {}

  void pruneGlobalPlanWrapper(nav_msgs::msg::Path & path, const PathIterator end)
  {
    return prunePlan(path, end);
  }

  double getMaxCostmapDistWrapper()
  {
    return getMaxCostmapDist();
  }

  std::pair<nav_msgs::msg::Path, PathIterator>
  getGlobalPlanConsideringBoundsInCostmapFrameWrapper(const geometry_msgs::msg::PoseStamped & pose)
  {
    return getGlobalPlanConsideringBoundsInCostmapFrame(pose);
  }

  bool transformPoseWrapper(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const
  {
    return transformPose(frame, in_pose, out_pose);
  }

  geometry_msgs::msg::PoseStamped transformToGlobalPlanFrameWrapper(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    return transformToGlobalPlanFrame(pose);
  }

  void setGlobalPlanUpToInversion(const nav_msgs::msg::Path & path)
  {
    global_plan_up_to_inversion_ = path;
  }

  bool isWithinInversionTolerancesWrapper(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    return isWithinInversionTolerances(robot_pose);
  }

  nav_msgs::msg::Path & getInvertedPath()
  {
    return global_plan_up_to_inversion_;
  }
};

TEST(PathHandlerTests, GetAndPrunePath)
{
  nav_msgs::msg::Path path;
  PathHandlerWrapper handler;

  path.header.frame_id = "fkframe";
  path.poses.resize(11);

  handler.setPath(path);
  auto & rtn_path = handler.getPath();
  EXPECT_EQ(path.header.frame_id, rtn_path.header.frame_id);
  EXPECT_EQ(path.poses.size(), rtn_path.poses.size());

  PathIterator it = rtn_path.poses.begin() + 5;
  handler.pruneGlobalPlanWrapper(rtn_path, it);
  auto rtn2_path = handler.getPath();
  EXPECT_EQ(rtn2_path.poses.size(), 6u);
}

TEST(PathHandlerTests, TestBounds)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(99999.9));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  auto results = costmap_ros->set_parameters_atomically(
    {rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("robot_base_frame", "base_link")});
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  // Test initialization and getting costmap basic metadata
  handler.initialize(node, "dummy", costmap_ros, costmap_ros->getTfBuffer(), &param_handler);
  EXPECT_EQ(handler.getMaxCostmapDistWrapper(), 2.5);

  // Set tf between map odom and base_link
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);

  // Test getting the global plans within a bounds window
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(100);
  for (unsigned int i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = i;
    path.poses[i].header.frame_id = "map";
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 25.0;

  handler.setPath(path);
  auto [transformed_plan, closest] =
    handler.getGlobalPlanConsideringBoundsInCostmapFrameWrapper(robot_pose);
  auto & path_inverted = handler.getInvertedPath();
  EXPECT_EQ(closest - path_inverted.poses.begin(), 25);
  handler.pruneGlobalPlanWrapper(path_inverted, closest);
  auto & path_pruned = handler.getInvertedPath();
  EXPECT_EQ(path_pruned.poses.size(), 75u);
}

TEST(PathHandlerTests, TestTransforms)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(99999.9));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  // Test basic transformations and path handling
  handler.initialize(node, "dummy", costmap_ros, costmap_ros->getTfBuffer(), &param_handler);

  // Set tf between map odom and base_link
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(100);
  for (unsigned int i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = i;
    path.poses[i].header.frame_id = "map";
  }

  geometry_msgs::msg::PoseStamped robot_pose, output_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 2.5;

  EXPECT_TRUE(handler.transformPoseWrapper("map", robot_pose, output_pose));
  EXPECT_EQ(output_pose.pose.position.x, 2.5);

  EXPECT_THROW(handler.transformToGlobalPlanFrameWrapper(robot_pose), std::runtime_error);
  handler.setPath(path);
  EXPECT_NO_THROW(handler.transformToGlobalPlanFrameWrapper(robot_pose));

  auto [path_out, closest] =
    handler.getGlobalPlanConsideringBoundsInCostmapFrameWrapper(robot_pose);

  // Put it all together
  auto final_path = handler.transformPath(robot_pose);
  EXPECT_EQ(final_path.poses.size(), path_out.poses.size());
}

TEST(PathHandlerTests, PreserveLatticeTurnBeforeNearbyFinalApproach)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lattice_turn_test");
  node->declare_parameter("dummy.path_pose_yaw_weight", 0.5);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "turn_costmap", "", "turn_costmap");
  costmap->set_parameter(rclcpp::Parameter("global_frame", "map"));
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ParametersHandler parameters(node);
  handler.initialize(node, "dummy", costmap, costmap->getTfBuffer(), &parameters);

  // A lattice connector backs away to turn, then passes near its start again.
  // These relative poses reproduce the pre-recovery Facility 18 path overlap.
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const std::vector<std::array<double, 3>> points{
    {1.275, 2.475, M_PI_2}, {1.225, 2.425, M_PI_2},
    {1.175, 2.375, M_PI_4}, {1.275, 2.425, 0.0},
    {1.275, 2.475, 0.0}, {1.275, 2.525, 0.0}, {1.275, 2.625, 0.0}};
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.orientation.z = std::sin(point[2] / 2.0);
    pose.pose.orientation.w = std::cos(point[2] / 2.0);
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped robot = path.poses.front();
  robot.pose.position.x = 1.277;
  robot.pose.position.y = 2.5001;
  robot.pose.orientation.z = std::sin(1.42 / 2.0);
  robot.pose.orientation.w = std::cos(1.42 / 2.0);
  handler.setPath(path);
  PathHandlerWrapper xy_handler;
  auto xy_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("xy_pruning_test");
  ParametersHandler xy_parameters(xy_node);
  xy_handler.initialize(xy_node, "dummy", costmap, costmap->getTfBuffer(), &xy_parameters);
  xy_handler.setPath(path);
  auto [xy_plan, xy_closest] =
    xy_handler.getGlobalPlanConsideringBoundsInCostmapFrameWrapper(robot);
  EXPECT_EQ(xy_closest - xy_handler.getInvertedPath().poses.begin(), 5);

  auto [before_turn, before_closest] =
    handler.getGlobalPlanConsideringBoundsInCostmapFrameWrapper(robot);
  EXPECT_EQ(before_closest - handler.getInvertedPath().poses.begin(), 0);
  ASSERT_FALSE(before_turn.poses.empty());
  EXPECT_NEAR(before_turn.poses.front().pose.orientation.z, std::sin(M_PI_4), 1e-8);

  // After completing the turn, the overlapping final segment becomes current.
  robot = path.poses[5];
  auto [after_turn, after_closest] =
    handler.getGlobalPlanConsideringBoundsInCostmapFrameWrapper(robot);
  EXPECT_EQ(after_closest - handler.getInvertedPath().poses.begin(), 5);

  path.poses.resize(2);
  path.poses[0].pose.orientation.z = std::sin((M_PI - .02) / 2.0);
  path.poses[0].pose.orientation.w = std::cos((M_PI - .02) / 2.0);
  path.poses[1].pose.orientation.z = 0.0;
  path.poses[1].pose.orientation.w = 1.0;
  robot = path.poses[1];
  robot.pose.orientation.z = std::sin((-M_PI + .02) / 2.0);
  robot.pose.orientation.w = std::cos((-M_PI + .02) / 2.0);
  handler.setPath(path);
  auto [wrapped_plan, wrapped_closest] =
    handler.getGlobalPlanConsideringBoundsInCostmapFrameWrapper(robot);
  EXPECT_EQ(wrapped_closest - handler.getInvertedPath().poses.begin(), 0);
}

TEST(PathHandlerTests, TestInversionToleranceChecks)
{
  nav_msgs::msg::Path path;
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(i);
    path.poses.push_back(pose);
  }
  path.poses.back().pose.orientation.w = 1;

  PathHandlerWrapper handler;
  handler.setGlobalPlanUpToInversion(path);

  // Not near (0,0)
  geometry_msgs::msg::PoseStamped robot_pose;
  EXPECT_FALSE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // Exactly on top of it
  robot_pose.pose.position.x = 9;
  robot_pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // Laterally of it
  robot_pose.pose.position.y = 9;
  EXPECT_FALSE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // On top but off angled
  robot_pose.pose.position.y = 0;
  robot_pose.pose.orientation.z = 0.8509035;
  robot_pose.pose.orientation.w = 0.525322;
  EXPECT_FALSE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // On top but off angled within tolerances
  robot_pose.pose.position.y = 0;
  robot_pose.pose.orientation.w = 0.9961947;
  robot_pose.pose.orientation.z = 0.0871558;
  EXPECT_TRUE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // Offset spatially + off angled but both within tolerances
  robot_pose.pose.position.x = 9.10;
  EXPECT_TRUE(handler.isWithinInversionTolerancesWrapper(robot_pose));
}
