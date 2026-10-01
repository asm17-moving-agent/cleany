#include "cleany_mujoco_observer/contact_observation.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>

namespace
{
class ContactTest : public testing::Test
{
protected:
  void SetUp() override
  {
    const char* xml = R"(<mujoco><worldbody>
      <geom name="floor" type="plane" size="4 4 .1"/>
      <body name="chassis" pos="1 2 0" quat=".7071067811865476 0 0 .7071067811865476">
        <geom type="sphere" size=".01" contype="0" conaffinity="0"/>
      </body>
      <body name="study_cafe_cup" pos="1.2 2 .05">
        <freejoint/><geom name="cup_collision" type="sphere" size=".1" mass=".1"/>
      </body>
    </worldbody></mujoco>)";
    char error[1024];
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXMLString(xml, nullptr, error, sizeof(error)), mj_deleteSpec);
    ASSERT_NE(spec, nullptr) << error;
    model.reset(mj_compile(spec.get(), nullptr));
    ASSERT_NE(model, nullptr) << mjs_getError(spec.get());
    data.reset(mj_makeData(model.get()));
    mj_forward(model.get(), data.get());
    base = mj_name2id(model.get(), mjOBJ_BODY, "chassis");
  }
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
  int base = -1;
};

TEST_F(ContactTest, ReportsSolvedContactInRotatedBaseFrame)
{
  const auto contacts = cleany_mujoco_observer::observeContacts(model.get(), data.get(), base, 10);
  ASSERT_EQ(contacts.size(), 1U);
  const auto& contact = contacts.front();
  EXPECT_EQ(contact.body_names[1], "study_cafe_cup");
  EXPECT_EQ(contact.geom_names[0], "floor");
  EXPECT_EQ(contact.geom_names[1], "cup_collision");
  EXPECT_NEAR(contact.position[0], 0.0, 1e-9);
  EXPECT_NEAR(contact.position[1], -0.2, 1e-9);
  EXPECT_NEAR(contact.position[2], -0.025, 1e-9);
  EXPECT_NEAR(contact.distance, -0.05, 1e-9);
  EXPECT_GT(contact.normal_force, 0.0);
}

TEST_F(ContactTest, DoesNotChangeIntegrationStateOrAppliedForces)
{
  const auto size = mj_stateSize(model.get(), mjSTATE_INTEGRATION);
  std::vector<mjtNum> before(size), after(size);
  mj_getState(model.get(), data.get(), before.data(), mjSTATE_INTEGRATION);
  cleany_mujoco_observer::observeContacts(model.get(), data.get(), base, 10);
  mj_getState(model.get(), data.get(), after.data(), mjSTATE_INTEGRATION);
  EXPECT_EQ(before, after);
}

TEST_F(ContactTest, ObeysLimitAndRejectsInvalidSnapshot)
{
  EXPECT_TRUE(cleany_mujoco_observer::observeContacts(model.get(), data.get(), base, 0).empty());
  EXPECT_THROW(cleany_mujoco_observer::observeContacts(model.get(), data.get(), -1, 10),
    std::invalid_argument);
  EXPECT_THROW(cleany_mujoco_observer::observeContacts(model.get(), nullptr, base, 10),
    std::invalid_argument);
}

TEST_F(ContactTest, PrivateCopyRemainsValidAfterSourceArenaIsReset)
{
  std::unique_ptr<mjData, decltype(&mj_deleteData)> snapshot(mj_makeData(model.get()), mj_deleteData);
  mj_copyData(snapshot.get(), model.get(), data.get());
  const auto before = cleany_mujoco_observer::observeContacts(model.get(), snapshot.get(), base, 10);
  ASSERT_EQ(before.size(), 1U);
  ASSERT_NE(snapshot->contact, data->contact);
  ASSERT_NE(snapshot->efc_force, data->efc_force);
  mj_resetData(model.get(), data.get());
  const auto after = cleany_mujoco_observer::observeContacts(model.get(), snapshot.get(), base, 10);
  ASSERT_EQ(after.size(), 1U);
  EXPECT_EQ(after[0].normal_force, before[0].normal_force);
  EXPECT_EQ(after[0].position, before[0].position);
}
}  // namespace
