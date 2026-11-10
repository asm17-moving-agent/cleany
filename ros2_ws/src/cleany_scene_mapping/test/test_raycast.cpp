#include "cleany_scene_mapping/raycast.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <random>
#include <sstream>

namespace
{
void compare(cleany_scene_mapping::RaycastWorkspace& workspace, double resolution,
             const octomap::point3d& origin, const std::array<octomap::KeySet, 3>& endpoints)
{
  octomap::OcTree geometry(resolution);
  octomap::KeySet expected, actual;
  octomap::KeyRay ray;
  for (const auto& keys : endpoints)
    for (const auto& key : keys)
      if (geometry.computeRayKeys(origin, geometry.keyToCoord(key), ray))
        expected.insert(ray.begin(), ray.end());
  // A reused output must not retain cells from the previous frame.
  actual.insert(octomap::OcTreeKey(1, 2, 3));
  workspace.compute(geometry, origin, {&endpoints[0], &endpoints[1], &endpoints[2]}, actual);
  ASSERT_EQ(actual.size(), expected.size());
  for (const auto& key : expected) EXPECT_EQ(actual.count(key), 1U);
  octomap::OcTree reference(resolution), optimized(resolution);
  // Exercise updates over an existing, pruned map with different probabilities.
  for (int x=0; x<8; ++x)
    for (int y=0; y<8; ++y)
      for (int z=0; z<8; ++z)
        for (int repeat=0; repeat<3; ++repeat)
        {
          reference.updateNode((x+.5)*resolution, (y+.5)*resolution, (z+.5)*resolution, x<4);
          optimized.updateNode((x+.5)*resolution, (y+.5)*resolution, (z+.5)*resolution, x<4);
        }
  reference.prune(); optimized.prune();
  for (const auto& key : expected) reference.updateNode(key, false);
  for (const auto& key : actual) optimized.updateNode(key, false);
  for (const auto& key : endpoints[0])
  {
    reference.updateNode(key, true);
    optimized.updateNode(key, true);
  }
  std::ostringstream left, right;
  reference.writeData(left); optimized.writeData(right);
  EXPECT_EQ(left.str(), right.str());
}

TEST(Raycast, ReusedCachePreservesRandomRaysAndFullOccupancyData)
{
  cleany_scene_mapping::RaycastWorkspace workspace;
  std::mt19937 random(218);
  std::uniform_real_distribution<float> coordinate(-.9f, .9f);
  for (double resolution : {.01, .025, .05})
    for (const auto& origin : {octomap::point3d(.123f, -.037f, .711f), octomap::point3d(0,0,0)})
    {
      octomap::OcTree geometry(resolution);
      std::array<octomap::KeySet, 3> endpoints;
      for (unsigned int i=0; i<180; ++i)
      {
        auto key = geometry.coordToKey(coordinate(random), coordinate(random), coordinate(random));
        endpoints[i%3].insert(key);
        if (i%4==0) endpoints[(i+1)%3].insert(key);
      }
      compare(workspace, resolution, origin, endpoints);
    }
}

TEST(Raycast, BudgetFallbackAndEmptyFramesPreserveResults)
{
  std::array<octomap::KeySet, 3> endpoints;
  octomap::OcTree geometry(.01);
  endpoints[0].insert(geometry.coordToKey(.1, .2, .3));
  endpoints[1].insert(geometry.coordToKey(-.2, -.3, -.1));
  endpoints[2] = endpoints[0];
  for (std::size_t bytes : {0U, 8U, 1024U, 8U*1024U*1024U})
  {
    cleany_scene_mapping::RaycastWorkspace workspace(bytes);
    compare(workspace, .01, {0,0,0}, endpoints);
    std::array<octomap::KeySet, 3> empty;
    compare(workspace, .01, {0,0,0}, empty);
    compare(workspace, .01, {.025f,.035f,.045f}, endpoints);
  }
}

TEST(Raycast, KeyLimitsOriginEndpointsAndOutOfRangeOrigin)
{
  cleany_scene_mapping::RaycastWorkspace workspace;
  octomap::OcTree geometry(.01);
  for (const auto key : {octomap::OcTreeKey(0,0,0), octomap::OcTreeKey(65535,65535,65535)})
  {
    const auto origin = geometry.keyToCoord(key);
    std::array<octomap::KeySet,3> endpoints;
    endpoints[0].insert(key);
    endpoints[1].insert(geometry.coordToKey(origin + octomap::point3d(1,1,1) * (key[0] == 0 ? .02f : -.02f)));
    compare(workspace, .01, origin, endpoints);
  }
  std::array<octomap::KeySet,3> endpoints;
  endpoints[0].insert(geometry.coordToKey(0.,0.,0.));
  compare(workspace, .01, {400,0,0}, endpoints);
}
}  // namespace
