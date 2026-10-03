#include "cleany_scene_mapping/known_geometry.hpp"

#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <gtest/gtest.h>

#include <limits>

namespace
{
using cleany_scene_mapping::clearContainedOccupancy;

std::unique_ptr<bodies::Body> box(const Eigen::Vector3d& size,
                                const Eigen::Vector3d& center = Eigen::Vector3d::Zero(),
                                double yaw = 0.0)
{
  const shapes::Box shape(size.x(), size.y(), size.z());
  std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(&shape));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = center;
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  body->setPose(pose);
  return body;
}

TEST(KnownGeometry, ClearsOnlyExistingInteriorOccupancy)
{
  octomap::OcTree tree(0.01);
  tree.updateNode(0.005, 0.005, 0.005, true);
  tree.updateNode(0.205, 0.005, 0.005, true);
  auto body = box({0.1, 0.1, 0.1});
  const auto original_size = tree.size();
  const auto result = clearContainedOccupancy(tree, {body.get()}, 1000);
  EXPECT_EQ(result.cleared, 1U);
  EXPECT_FALSE(tree.isNodeOccupied(tree.search(0.005, 0.005, 0.005)));
  EXPECT_TRUE(tree.isNodeOccupied(tree.search(0.205, 0.005, 0.005)));
  EXPECT_EQ(tree.search(0.025, 0.025, 0.025), nullptr);
  EXPECT_EQ(tree.size(), original_size);
}

TEST(KnownGeometry, PreservesCellWhoseCenterButNotCornersIsInside)
{
  octomap::OcTree tree(0.01);
  tree.updateNode(0.005, 0.005, 0.005, true);
  auto body = box({0.009, 0.03, 0.03}, {0.005, 0.005, 0.005});
  EXPECT_EQ(clearContainedOccupancy(tree, {body.get()}, 100).cleared, 0U);
  EXPECT_TRUE(tree.isNodeOccupied(tree.search(0.005, 0.005, 0.005)));
}

TEST(KnownGeometry, DoesNotJoinSeparateShapesAcrossUnknownGap)
{
  octomap::OcTree tree(0.01);
  tree.updateNode(0.005, 0.005, 0.005, true);
  auto left = box({0.004, 0.04, 0.04}, {0.0, 0.005, 0.005});
  auto right = box({0.004, 0.04, 0.04}, {0.01, 0.005, 0.005});
  EXPECT_EQ(clearContainedOccupancy(tree, {left.get(), right.get()}, 100).cleared, 0U);
}

TEST(KnownGeometry, UsesRotatedBodyNotJustBoundingSphere)
{
  octomap::OcTree tree(0.01);
  tree.updateNode(0.005, 0.045, 0.005, true);
  tree.updateNode(0.045, 0.005, 0.005, true);
  auto body = box({0.12, 0.04, 0.04}, Eigen::Vector3d::Zero(), 1.5707963267948966);
  EXPECT_EQ(clearContainedOccupancy(tree, {body.get()}, 100).cleared, 1U);
  EXPECT_FALSE(tree.isNodeOccupied(tree.search(0.005, 0.045, 0.005)));
  EXPECT_TRUE(tree.isNodeOccupied(tree.search(0.045, 0.005, 0.005)));
}

TEST(KnownGeometry, RespectsScanBudgetAndEmptyGeometry)
{
  octomap::OcTree tree(0.01);
  tree.updateNode(0.005, 0.005, 0.005, true);
  tree.updateNode(0.105, 0.005, 0.005, true);
  auto body = box({1.0, 1.0, 1.0});
  EXPECT_EQ(clearContainedOccupancy(tree, {}, 100).cleared, 0U);
  const auto result = clearContainedOccupancy(tree, {body.get()}, 1);
  EXPECT_EQ(result.examined, 1U);
  EXPECT_EQ(result.cleared, 1U);
  EXPECT_TRUE(result.budget_exhausted);
}

TEST(KnownGeometry, PreservesAlreadyFreeCellsAndCurvedBoundary)
{
  octomap::OcTree tree(0.01);
  tree.updateNode(0.005, 0.005, 0.005, false);
  tree.updateNode(0.045, 0.025, 0.005, true);
  const auto free_odds = tree.search(0.005, 0.005, 0.005)->getLogOdds();
  const shapes::Sphere shape(0.055);
  std::unique_ptr<bodies::Body> sphere(bodies::createBodyFromShape(&shape));
  // Center is in the sphere, but the far cube corner is outside.
  EXPECT_EQ(clearContainedOccupancy(tree, {sphere.get()}, 100).cleared, 0U);
  EXPECT_FLOAT_EQ(tree.search(0.005, 0.005, 0.005)->getLogOdds(), free_odds);
  EXPECT_TRUE(tree.isNodeOccupied(tree.search(0.045, 0.025, 0.005)));
}

TEST(KnownGeometry, UsesActualLeafSizeForPrunedCells)
{
  octomap::OcTree tree(0.01);
  for (unsigned int i = 0; i < 8; ++i)
    tree.updateNode((i & 1U) ? 0.015 : 0.005, (i & 2U) ? 0.015 : 0.005,
                    (i & 4U) ? 0.015 : 0.005, true);
  tree.prune();
  ASSERT_EQ(tree.getNumLeafNodes(), 1U);
  auto partial = box({0.015, 0.03, 0.03}, {0.01, 0.01, 0.01});
  EXPECT_EQ(clearContainedOccupancy(tree, {partial.get()}, 100).cleared, 0U);
  auto full = box({0.03, 0.03, 0.03}, {0.01, 0.01, 0.01});
  EXPECT_EQ(clearContainedOccupancy(tree, {full.get()}, 100).cleared, 1U);
}

TEST(KnownGeometry, MeshFacePaddingMatchesPrimitiveAcrossScalePoseAndLocalOffset)
{
  const shapes::Box shape(0.12, 0.04, 0.04);
  std::unique_ptr<shapes::Mesh> shape_mesh(shapes::createMeshFromShape(shape));
  const Eigen::Vector3d local_offset(0.2, -0.3, 0.1);
  for (unsigned int i = 0; i < shape_mesh->vertex_count; ++i)
    for (unsigned int axis = 0; axis < 3; ++axis)
      shape_mesh->vertices[3 * i + axis] += local_offset[axis];
  for (double scale : {1.0, 1.25})
    for (double yaw : {0.0, 0.37})
    {
      auto pose = Eigen::Isometry3d::Identity();
      pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      pose.translation() = Eigen::Vector3d(0.505, 0.005, 0.005) - pose.linear() * local_offset;
      std::unique_ptr<bodies::Body> mesh(bodies::createBodyFromShape(shape_mesh.get()));
      mesh->setScale(scale);
      mesh->setPadding(0.015);
      // Frame integration clears using a clone of the capture-time body.
      auto placed_mesh = mesh->cloneAt(pose);
      auto primitive = box({0.12, 0.04, 0.04}, pose * local_offset, yaw);
      primitive->setScale(scale);
      primitive->setPadding(0.015);
      octomap::OcTree mesh_tree(0.01), primitive_tree(0.01);
      for (int x = -10; x <= 10; ++x)
        for (int y = -8; y <= 8; ++y)
          for (int z = -5; z <= 5; ++z)
          {
            mesh_tree.updateNode(0.505 + x * 0.01, 0.005 + y * 0.01, 0.005 + z * 0.01, true);
            primitive_tree.updateNode(0.505 + x * 0.01, 0.005 + y * 0.01, 0.005 + z * 0.01, true);
          }
      const auto expected = clearContainedOccupancy(primitive_tree, {primitive.get()}, 10000);
      const auto actual = clearContainedOccupancy(mesh_tree, {placed_mesh.get()}, 10000);
      ASSERT_GT(expected.cleared, 0U);
      EXPECT_EQ(actual.cleared, expected.cleared) << "scale=" << scale << " yaw=" << yaw;
      for (auto it = primitive_tree.begin_leafs(); it != primitive_tree.end_leafs(); ++it)
      {
        const auto* node = mesh_tree.search(it.getKey());
        ASSERT_NE(node, nullptr);
        EXPECT_EQ(mesh_tree.isNodeOccupied(node), primitive_tree.isNodeOccupied(*it));
      }
      // An occupied cell beyond the configured padding is preserved.
      EXPECT_TRUE(mesh_tree.isNodeOccupied(mesh_tree.search(0.505, 0.085, 0.005)));
    }
}

TEST(KnownGeometry, MissingOrNonfiniteTransformsLeaveAllBodiesUnchanged)
{
  cleany_scene_mapping::KnownBodies bodies;
  bodies[1] = box({0.1, 0.1, 0.1});
  bodies[2] = box({0.1, 0.1, 0.1});
  const auto identity = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(cleany_scene_mapping::placeKnownBodies(bodies, identity,
      [](unsigned int handle, Eigen::Isometry3d& pose) {
        pose = Eigen::Isometry3d::Identity();
        pose.translation().x() = 2.0;
        return handle == 1;
      }));
  EXPECT_TRUE(bodies[1]->getPose().matrix().isApprox(identity.matrix()));
  EXPECT_FALSE(cleany_scene_mapping::placeKnownBodies(bodies, identity,
      [](unsigned int, Eigen::Isometry3d& pose) {
        pose = Eigen::Isometry3d::Identity();
        pose.translation().x() = std::numeric_limits<double>::quiet_NaN();
        return true;
      }));
  EXPECT_TRUE(bodies[1]->getPose().matrix().isApprox(identity.matrix()));
}

TEST(KnownGeometry, ComposesCaptureTimeTransformsIntoMapFrame)
{
  cleany_scene_mapping::KnownBodies bodies;
  bodies[7] = box({0.1, 0.1, 0.1});
  Eigen::Isometry3d map_from_cloud = Eigen::Isometry3d::Identity();
  map_from_cloud.translation().y() = 3.0;
  map_from_cloud.linear() = Eigen::AngleAxisd(1.5707963267948966,
                                            Eigen::Vector3d::UnitZ()).toRotationMatrix();
  ASSERT_TRUE(cleany_scene_mapping::placeKnownBodies(bodies, map_from_cloud,
      [](unsigned int handle, Eigen::Isometry3d& pose) {
        pose = Eigen::Isometry3d::Identity();
        pose.translation().x() = 2.0;
        return handle == 7;
      }));
  EXPECT_TRUE(bodies[7]->getPose().translation().isApprox(Eigen::Vector3d(0.0, 5.0, 0.0)));
}
}  // namespace
