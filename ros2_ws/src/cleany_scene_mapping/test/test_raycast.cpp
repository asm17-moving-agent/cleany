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

TEST(Raycast, BlockWritesMatchNativeLogOddsPruningAndChangeTracking)
{
  for (bool track_changes : {false, true})
    for (std::size_t budget : {0U, 8U * 1024U * 1024U})
    {
      octomap::OcTree reference(.025), actual(.025);
      for (auto* tree : {&reference, &actual})
      {
        tree->setProbMiss(.35);
        tree->setClampingThresMin(.15);
        for (int x = -8; x < 8; ++x)
          for (int y = -4; y < 4; ++y)
            for (int z = -4; z < 4; ++z)
              tree->updateNode((x + .5) * .025, (y + .5) * .025, (z + .5) * .025, x < 0);
        tree->prune();
        tree->enableChangeDetection(track_changes);
      }
      cleany_scene_mapping::RaycastWorkspace workspace(budget);
      std::mt19937 random(819);
      std::uniform_real_distribution<float> coordinate(-.45f, .45f);
      const octomap::point3d origin(-.025f, .037f, .019f);
      octomap::KeyRay ray;
      for (unsigned int frame = 0; frame < 7; ++frame)
      {
        octomap::KeySet occupied, model, clipped, misses;
        for (unsigned int i = 0; i < 300; ++i)
        {
          const auto key = actual.coordToKey(coordinate(random), coordinate(random), coordinate(random));
          (i % 4 == 0 ? model : occupied).insert(key);
          if (i % 3 == 0)
            clipped.insert(key);
        }
        for (const auto& key : model)
          occupied.erase(key);
        misses = clipped;
        for (const auto* keys : {&occupied, &model, &clipped})
          for (const auto& key : *keys)
            if (reference.computeRayKeys(origin, reference.keyToCoord(key), ray))
              misses.insert(ray.begin(), ray.end());
        for (const auto& key : occupied)
          misses.erase(key);
        for (const auto& key : misses)
          reference.updateNode(key, false);
        const auto blocks = workspace.traceFrame(actual, origin, occupied, model, clipped);
        cleany_scene_mapping::integrateFreeBlocks(actual, blocks);
        for (auto* tree : {&reference, &actual})
        {
          for (const auto& key : occupied)
            tree->updateNode(key, true);
          for (const auto& key : model)
            tree->updateNode(key, tree->getClampingThresMinLog() - tree->getClampingThresMaxLog());
        }
        std::ostringstream expected, observed;
        reference.writeData(expected);
        actual.writeData(observed);
        ASSERT_TRUE(expected.str() == observed.str()) << "frame=" << frame << " budget=" << budget;
        EXPECT_EQ(actual.size(), reference.size());
        octomap::KeyBoolMap expected_changes(reference.changedKeysBegin(), reference.changedKeysEnd());
        for (auto it = actual.changedKeysBegin(); it != actual.changedKeysEnd(); ++it)
        {
          ASSERT_EQ(expected_changes.count(it->first), 1U);
          EXPECT_EQ(it->second, expected_changes.find(it->first)->second);
          expected_changes.erase(it->first);
        }
        EXPECT_TRUE(expected_changes.empty());
      }
    }
}

TEST(Raycast, InvalidOriginStillAppliesClippedEndpointAndHitWins)
{
  octomap::OcTree tree(.01);
  const auto hit = tree.coordToKey(.105, .005, .005);
  const auto miss = tree.coordToKey(.115, .005, .005);
  cleany_scene_mapping::RaycastWorkspace workspace;
  octomap::KeySet occupied, model, clipped;
  occupied.insert(hit);
  clipped.insert(hit);
  clipped.insert(miss);
  const auto blocks = workspace.traceFrame(tree, {400, 0, 0}, occupied, model, clipped);
  cleany_scene_mapping::integrateFreeBlocks(tree, blocks);
  EXPECT_EQ(tree.search(hit), nullptr);
  ASSERT_NE(tree.search(miss), nullptr);
  EXPECT_FLOAT_EQ(tree.search(miss)->getLogOdds(), tree.getProbMissLog());
}

TEST(Raycast, FullBlockPrunesAndSaturatedMissesPreserveCoarseEvidence)
{
  octomap::OcTree expected(.01), actual(.01);
  const octomap::OcTreeKey base(32768, 32768, 32768);
  octomap::KeySet keys;
  for (unsigned int x = 0; x < 4; ++x)
    for (unsigned int y = 0; y < 4; ++y)
      for (unsigned int z = 0; z < 4; ++z)
        keys.insert({static_cast<octomap::key_type>(base[0] + x),
                     static_cast<octomap::key_type>(base[1] + y),
                     static_cast<octomap::key_type>(base[2] + z)});
  const std::vector<cleany_scene_mapping::FreeVoxelBlock> full{{base, ~std::uint64_t{0}}};
  for (unsigned int frame = 0; frame < 12; ++frame)
  {
    for (const auto& key : keys)
      expected.updateNode(key, false);
    cleany_scene_mapping::integrateFreeBlocks(actual, full);
    EXPECT_EQ(actual.getNumLeafNodes(), 1U);
    EXPECT_FLOAT_EQ(actual.search(base)->getLogOdds(), expected.search(base)->getLogOdds());
  }
  cleany_scene_mapping::integrateFreeBlocks(actual, {{base, 1}});
  EXPECT_EQ(actual.getNumLeafNodes(), 1U);
  expected.updateNode(base, true);
  actual.updateNode(base, true);
  expected.updateNode(base, false);
  cleany_scene_mapping::integrateFreeBlocks(actual, {{base, 1}});
  std::ostringstream reference, observed;
  expected.writeData(reference);
  actual.writeData(observed);
  EXPECT_EQ(reference.str(), observed.str());
}
}  // namespace
