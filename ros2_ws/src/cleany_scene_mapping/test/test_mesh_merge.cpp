#include "cleany_scene_mapping/mesh_merge.hpp"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include <random>

namespace
{
void expectSame(const shapes::Mesh& expected, const shapes::Mesh& actual)
{
  ASSERT_EQ(expected.vertex_count, actual.vertex_count);
  ASSERT_EQ(expected.triangle_count, actual.triangle_count);
  EXPECT_EQ(0, std::memcmp(expected.vertices, actual.vertices, 3ULL * expected.vertex_count * sizeof(double)));
  EXPECT_EQ(0, std::memcmp(expected.triangles, actual.triangles, 3ULL * expected.triangle_count * sizeof(unsigned int)));
  ASSERT_EQ(expected.triangle_normals != nullptr, actual.triangle_normals != nullptr);
  ASSERT_EQ(expected.vertex_normals != nullptr, actual.vertex_normals != nullptr);
  if (expected.triangle_normals)
    EXPECT_EQ(0, std::memcmp(expected.triangle_normals, actual.triangle_normals,
                             3ULL * expected.triangle_count * sizeof(double)));
  if (expected.vertex_normals)
    EXPECT_EQ(0, std::memcmp(expected.vertex_normals, actual.vertex_normals,
                             3ULL * expected.vertex_count * sizeof(double)));
}

std::unique_ptr<shapes::Mesh> mesh(const std::vector<Eigen::Vector3d>& points, bool normals = true)
{
  auto result = std::make_unique<shapes::Mesh>(points.size(), points.size() / 3);
  for (std::size_t index = 0; index < points.size(); ++index)
    for (int axis = 0; axis < 3; ++axis) result->vertices[3 * index + axis] = points[index][axis];
  for (unsigned int index = 0; index < result->triangle_count * 3; ++index) result->triangles[index] = index;
  if (normals)
  {
    result->computeTriangleNormals();
    result->computeVertexNormals();
  }
  else
  {
    delete[] result->triangle_normals; result->triangle_normals = nullptr;
    delete[] result->vertex_normals; result->vertex_normals = nullptr;
  }
  return result;
}

void compare(const std::vector<Eigen::Vector3d>& points, double threshold, bool normals = true)
{
  auto expected = mesh(points, normals);
  std::unique_ptr<shapes::Mesh> actual(expected->clone());
  expected->mergeVertices(threshold);
  cleany_scene_mapping::mergeMeshVertices(*actual, threshold);
  expectSame(*expected, *actual);
}
}

TEST(MeshMerge, OrderedOverlappingAssignmentsAndCellBoundaries)
{
  std::vector<Eigen::Vector3d> points;
  // The third point is assigned to the first representative, then overwritten
  // by the second. Include negative and exact-radius/cell-boundary cases.
  for (int index = -12; index < 12; ++index)
  {
    const double x = index * 0.01;
    points.emplace_back(x, 0, 0);
    points.emplace_back(x + 1.5e-4, 0, 0);
    points.emplace_back(x + 0.9e-4, 0, 0);
    points.emplace_back(x + 2e-4, 0, 0);
    points.emplace_back(std::nextafter(x + 2e-4, -INFINITY), 0, 0);
    points.emplace_back(std::nextafter(x + 2e-4, INFINITY), 0, 0);
  }
  compare(points, 1e-4);
  compare(points, 1e-4, false);
  compare(points, -1e-4);
}

TEST(MeshMerge, RandomizedDifferentialAgainstInstalledLibrary)
{
  std::mt19937 random(7119);
  std::uniform_real_distribution<double> unit(-1, 1);
  for (unsigned int trial = 0; trial < 120; ++trial)
  {
    const double radius = std::pow(10., -static_cast<int>(trial % 5));
    std::vector<Eigen::Vector3d> points;
    for (unsigned int index = 0; index < 160 + trial; ++index)
    {
      Eigen::Vector3d point(unit(random), unit(random), unit(random));
      point *= radius * ((trial % 3) ? 4.0 : 100.0);
      if (!points.empty() && index % 3 == 0) point += points[index / 3];
      if (!points.empty() && index % 7 == 0) point = points[index / 7];
      points.push_back(point);
    }
    SCOPED_TRACE(trial);
    compare(points, radius, trial % 2);
  }
}

TEST(MeshMerge, NoMergePreservesArraysAndNormalPresence)
{
  std::vector<Eigen::Vector3d> points;
  for (int index = 0; index < 90; ++index) points.emplace_back(index, index % 3, index % 7);
  compare(points, 1e-4);
  compare(points, 1e-4, false);
}

TEST(MeshMerge, SmallAndUnindexableInputsRetainUpstreamBehaviour)
{
  compare({}, 1e-4, false);
  compare({{0, 0, 0}, {0, 0, 0}, {1, 1, 1}}, 1e-4);
  std::vector<Eigen::Vector3d> points(90, Eigen::Vector3d(1, -1, 0));
  compare(points, 0.0);
  compare(points, 1e-30);
  compare(points, std::numeric_limits<double>::infinity());
  compare(points, std::numeric_limits<double>::quiet_NaN());
  points[70].x() = std::numeric_limits<double>::infinity();
  compare(points, 1e-4, false);
}
