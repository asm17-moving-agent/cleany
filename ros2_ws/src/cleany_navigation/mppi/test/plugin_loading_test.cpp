#include <gtest/gtest.h>
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "cleany_navigation/mppi/critic_function.hpp"

TEST(IntegratedMppiPlugins, LoadsControllerAndCriticsFromCleanyNavigation)
{
  pluginlib::ClassLoader<nav2_core::Controller> controllers(
    "nav2_core", "nav2_core::Controller");
  const std::string controller_name = "cleany_navigation::MPPIController";
  EXPECT_EQ(controllers.getClassPackage(controller_name), "cleany_navigation");
  auto controller = controllers.createUniqueInstance(controller_name);
  ASSERT_NE(controller, nullptr);

  pluginlib::ClassLoader<cleany_navigation::mppi::critics::CriticFunction> critics(
    "cleany_navigation", "cleany_navigation::mppi::critics::CriticFunction");
  for (const auto & name : {"CostCritic", "PathAlignCritic", "PathFollowCritic"}) {
    const std::string class_name = std::string("cleany_navigation::mppi::critics::") + name;
    EXPECT_EQ(critics.getClassPackage(class_name), "cleany_navigation");
    auto critic = critics.createUniqueInstance(class_name);
    ASSERT_NE(critic, nullptr);
  }
}
