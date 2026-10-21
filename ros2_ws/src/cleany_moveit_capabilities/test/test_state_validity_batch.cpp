#include <cleany_moveit_capabilities/state_validity_batch.hpp>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <octomap/OcTree.h>
#include <limits>
#include <algorithm>

namespace
{
class BatchValidityTest : public testing::Test
{
protected:
  void SetUp() override
  {
    auto urdf = urdf::parseURDF(R"(<robot name="probe"><link name="base"/>
      <link name="tool"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/>
      <axis xyz="1 0 0"/><limit lower="0" upper="2" effort="1" velocity="1"/></joint></robot>)");
    auto semantic = std::make_shared<srdf::Model>();
    ASSERT_TRUE(semantic->initString(*urdf,
      "<robot name='probe'><group name='arm'><joint name='slide'/></group></robot>"));
    scene = std::make_shared<planning_scene::PlanningScene>(
        std::make_shared<moveit::core::RobotModel>(urdf, semantic));
    scene->getCurrentStateNonConst().setToDefaultValues();
    obstacle.header.frame_id = "base";
    obstacle.id = "obstacle";
    obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.2, 0.2, 0.2};
    obstacle.primitives.push_back(box);
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.orientation.w = 1.0;
    obstacle.primitive_poses.push_back(pose);
    ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  }
  moveit_msgs::msg::RobotState state(double position)
  {
    moveit_msgs::msg::RobotState result;
    result.is_diff = true;
    result.joint_state.name = {"slide"};
    result.joint_state.position = {position};
    return result;
  }
  cleany::BatchValidity::Response check(std::initializer_list<double> positions)
  {
    cleany::BatchValidity::Request request;
    for (double position : positions) request.robot_states.push_back(state(position));
    cleany::BatchValidity::Response response;
    cleany::check_state_batch(*scene, request, response, builtin_interfaces::msg::Time());
    return response;
  }
  planning_scene::PlanningScenePtr scene;
  moveit_msgs::msg::CollisionObject obstacle;
};

TEST_F(BatchValidityTest, ChecksEverySampleAndStopsAtFirstCollision)
{
  auto response = check({0., 0.2, 1., 0.});
  ASSERT_TRUE(response.success) << response.message;
  EXPECT_EQ(response.first_invalid_index, 2);
  ASSERT_EQ(response.valid.size(), 3u);
  EXPECT_TRUE(response.valid[0]);
  EXPECT_TRUE(response.valid[1]);
  EXPECT_FALSE(response.valid[2]);
  ASSERT_FALSE(response.contacts.empty());
  EXPECT_EQ(response.contacts.front().header.frame_id, "base");
}

TEST_F(BatchValidityTest, LiveAcmRemainsAuthoritative)
{
  EXPECT_FALSE(check({1.}).valid.front());
  scene->getAllowedCollisionMatrixNonConst().setEntry("tool", "obstacle", true);
  EXPECT_TRUE(check({1.}).valid.front());
  scene->getAllowedCollisionMatrixNonConst().setEntry("tool", "obstacle", false);
  EXPECT_FALSE(check({1.}).valid.front());
}

TEST_F(BatchValidityTest, AttachedObjectsArePreservedForDiffStates)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "tool";
  attached.touch_links = {"tool"};
  attached.object = obstacle;
  attached.object.id = "payload";
  attached.object.header.frame_id = "tool";
  attached.object.primitive_poses.front().position.x = 0.3;
  ASSERT_TRUE(scene->processAttachedCollisionObjectMsg(attached));
  EXPECT_TRUE(check({0.}).valid.front());
  auto response = check({0., 0.7});
  ASSERT_TRUE(response.success) << response.message;
  EXPECT_EQ(response.first_invalid_index, 1);
  EXPECT_FALSE(response.valid.back());
  EXPECT_TRUE(std::any_of(response.contacts.begin(), response.contacts.end(), [](const auto& contact) {
    return contact.contact_body_1 == "payload" || contact.contact_body_2 == "payload";
  }));
}

TEST_F(BatchValidityTest, ConstraintsAndMalformedRequestsFailClosed)
{
  cleany::BatchValidity::Request request;
  request.robot_states = {state(0.), state(0.5)};
  moveit_msgs::msg::JointConstraint limit;
  limit.joint_name = "slide";
  limit.position = 0.;
  limit.tolerance_above = limit.tolerance_below = 0.1;
  limit.weight = 1.;
  request.constraints.joint_constraints.push_back(limit);
  cleany::BatchValidity::Response response;
  cleany::check_state_batch(*scene, request, response, builtin_interfaces::msg::Time());
  ASSERT_TRUE(response.success) << response.message;
  EXPECT_EQ(response.first_invalid_index, 1);
  request.group_name = "missing";
  cleany::check_state_batch(*scene, request, response, builtin_interfaces::msg::Time());
  EXPECT_FALSE(response.success);
  EXPECT_TRUE(response.valid.empty());
  request.group_name.clear();
  request.robot_states[1].joint_state.position[0] = std::numeric_limits<double>::quiet_NaN();
  cleany::check_state_batch(*scene, request, response, builtin_interfaces::msg::Time());
  EXPECT_FALSE(response.success);
  EXPECT_TRUE(response.valid.empty());
}

TEST_F(BatchValidityTest, PayloadOctomapPermissionKeepsRobotAndNamedObstacleChecks)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "tool";
  attached.touch_links = {"tool"};
  attached.object = obstacle;
  attached.object.id = "payload";
  attached.object.header.frame_id = "tool";
  attached.object.primitive_poses.front().position.x = 0.3;
  ASSERT_TRUE(scene->processAttachedCollisionObjectMsg(attached));
  auto tree = std::make_shared<octomap::OcTree>(0.01);
  tree->updateNode(octomap::point3d(0.3f, 0.f, 0.f), true);
  scene->processOctomapPtr(tree, Eigen::Isometry3d::Identity());
  const auto has_pair = [](const auto& response, const std::string& first, const std::string& second) {
    return std::any_of(response.contacts.begin(), response.contacts.end(), [&](const auto& contact) {
      return (contact.contact_body_1 == first && contact.contact_body_2 == second) ||
             (contact.contact_body_1 == second && contact.contact_body_2 == first);
    });
  };
  auto blocked = check({0.});
  ASSERT_TRUE(blocked.success) << blocked.message;
  ASSERT_FALSE(blocked.valid.front());
  ASSERT_TRUE(has_pair(blocked, "payload", "<octomap>"));
  scene->getAllowedCollisionMatrixNonConst().setEntry("payload", "<octomap>", true);
  ASSERT_TRUE(check({0.}).valid.front());

  // Ignoring payload/map contact cannot authorize payload/obstacle contact.
  blocked = check({0.7});
  ASSERT_FALSE(blocked.valid.front());
  EXPECT_TRUE(has_pair(blocked, "payload", "obstacle"));

  // Nor may it hide the robot tool inside another occupied map voxel.
  tree = std::make_shared<octomap::OcTree>(0.01);
  tree->updateNode(octomap::point3d(0.3f, 0.f, 0.f), true);
  tree->updateNode(octomap::point3d(0.f, 0.f, 0.f), true);
  scene->processOctomapPtr(tree, Eigen::Isometry3d::Identity());
  blocked = check({0.});
  ASSERT_FALSE(blocked.valid.front());
  EXPECT_TRUE(has_pair(blocked, "tool", "<octomap>"));
  EXPECT_FALSE(has_pair(blocked, "payload", "<octomap>"));

  scene->getAllowedCollisionMatrixNonConst().setEntry("payload", "<octomap>", false);
  EXPECT_TRUE(has_pair(check({0.}), "payload", "<octomap>"));
}
}  // namespace
