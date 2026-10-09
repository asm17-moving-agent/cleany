#include "cleany_mujoco_observer/geom_bounds.hpp"
#include <gtest/gtest.h>
#include <memory>
#include <vector>

TEST(GeomBounds, TiltedMeshAndCylinderUseSurfaceNotRotatedLocalBox)
{
  const char* xml = R"(<mujoco><asset>
    <mesh name="tetra" vertex="0 0 0 1 0 0 0 1 0 0 0 1"/>
    </asset><worldbody>
    <body name="base" pos="1 2 0" euler="0 0 90"><geom size=".01"/></body>
    <geom name="mesh" type="mesh" mesh="tetra" pos="1 2 3" euler="0 0 45"/>
    <geom name="cylinder" type="cylinder" size=".2 .4" pos="1 2 3" euler="0 45 0"/>
    </worldbody></mujoco>)";
  char error[1024];
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
    mj_parseXMLString(xml, nullptr, error, sizeof(error)), mj_deleteSpec);
  ASSERT_NE(spec, nullptr) << error;
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_compile(spec.get(), nullptr), mj_deleteModel);
  ASSERT_NE(model, nullptr) << mjs_getError(spec.get());
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  mj_forward(model.get(), data.get());
  const int base = mj_name2id(model.get(), mjOBJ_BODY, "base");
  const int size = mj_stateSize(model.get(), mjSTATE_INTEGRATION);
  std::vector<mjtNum> before(size), after(size);
  mj_getState(model.get(), data.get(), before.data(), mjSTATE_INTEGRATION);
  const auto mesh = cleany_mujoco_observer::geomBoundsInBase(model.get(), data.get(),
    mj_name2id(model.get(), mjOBJ_GEOM, "mesh"), base);
  EXPECT_NEAR(mesh.low[0], 0, 1e-6);
  EXPECT_NEAR(mesh.high[0], std::sqrt(.5), 1e-6);
  EXPECT_NEAR(mesh.low[1], -std::sqrt(.5), 1e-6);
  EXPECT_NEAR(mesh.high[1], std::sqrt(.5), 1e-6);
  EXPECT_NEAR(mesh.low[2], 3, 1e-6);
  EXPECT_NEAR(mesh.high[2], 4, 1e-6);
  const auto cylinder = cleany_mujoco_observer::geomBoundsInBase(model.get(), data.get(),
    mj_name2id(model.get(), mjOBJ_GEOM, "cylinder"), base);
  EXPECT_NEAR(cylinder.low[0], -.2, 1e-6);
  EXPECT_NEAR(cylinder.high[0], .2, 1e-6);
  EXPECT_NEAR(cylinder.low[1], -.6*std::sqrt(.5), 1e-6);
  EXPECT_NEAR(cylinder.low[2], 3-.6*std::sqrt(.5), 1e-6);
  mj_getState(model.get(), data.get(), after.data(), mjSTATE_INTEGRATION);
  EXPECT_EQ(before, after);
}
