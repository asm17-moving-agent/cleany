#include <gtest/gtest.h>
#include <cleany_moveit_collision/cad_linkage.hpp>
#include <moveit/robot_model/robot_model.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <yaml-cpp/yaml.h>
#include <limits>
#include <sstream>

TEST(CadLinkage, MatchesExistingPythonSolverWithoutChangingActuatorValues)
{
  const auto fixture=YAML::LoadFile(CAD_LINKAGE_FIXTURE);
  const auto config=fixture["config"];
  std::ostringstream xml;
  xml << "<robot name='pin_test'><link name='base_link'/>";
  for (const auto& group : config["grippers"])
  {
    std::vector<std::string> names{group["source_joint"].as<std::string>()};
    for (const auto& finger : group["linkage"]["fingers"])
      for (const auto& joint : finger.second["joints"]) names.push_back(joint.second.as<std::string>());
    for (const auto& name : names)
      xml << "<link name='" << name << "_link'/><joint name='" << name
          << "' type='continuous'><parent link='base_link'/><child link='" << name
          << "_link'/><axis xyz='0 0 1'/><limit effort='1' velocity='1'/></joint>";
  }
  xml << "</robot>";
  auto urdf=urdf::parseURDF(xml.str());
  auto semantic=std::make_shared<srdf::Model>();
  ASSERT_TRUE(semantic->initString(*urdf,"<robot name='pin_test'/>"));
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,semantic);
  const cleany::CadLinkage linkage(YAML::Dump(config),*model);
  for (const auto& sample : fixture["cases"])
  {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    for (const auto& input : sample["actuator_positions"])
      state.setVariablePosition(input.first.as<std::string>(),input.second.as<double>());
    ASSERT_TRUE(linkage.resolve(state));
    for (const auto& expected : sample["expected_dependent"])
      EXPECT_NEAR(state.getVariablePosition(expected.first.as<std::string>()),expected.second.as<double>(),1e-12);
    for (const auto& input : sample["actuator_positions"])
      EXPECT_DOUBLE_EQ(state.getVariablePosition(input.first.as<std::string>()),input.second.as<double>());
  }
  moveit::core::RobotState invalid(model);
  invalid.setToDefaultValues();
  invalid.setVariablePosition("left_gripper_joint",std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(linkage.resolve(invalid));
  EXPECT_THROW(cleany::CadLinkage("{}",*model),YAML::Exception);
}
