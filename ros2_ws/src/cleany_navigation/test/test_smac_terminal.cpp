#include <gtest/gtest.h>
#include "cleany_navigation/smac_lattice_planner.hpp"
#include "cleany_navigation/straight_first_planner.hpp"

TEST(SmacTerminal, SameCellRegressionExactGoalAndCollisionRejection) {
  rclcpp::init(0,nullptr);
  auto parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("smac_terminal_test");
  parent->declare_parameter("GridBased.lattice_filepath",std::string(CLEANY_TEST_LATTICE));
  parent->declare_parameter("GridBased.analytic_expansion_max_length",0.0);
  parent->declare_parameter("GridBased.smooth_path",false);
  parent->declare_parameter("GridBased.lookup_table_size",3.0);
  auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("global_frame","map"),rclcpp::Parameter("plugins",std::vector<std::string>{}),
    rclcpp::Parameter("width",10),rclcpp::Parameter("height",10),
    rclcpp::Parameter("resolution",.05),rclcpp::Parameter("origin_x",-5.),rclcpp::Parameter("origin_y",-5.),
    rclcpp::Parameter("footprint","[[-0.48,-0.40],[0.48,-0.40],[0.48,0.40],[-0.48,0.40]]"),
    rclcpp::Parameter("footprint_padding",.01)});
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  map->configure();
  auto live=map->getCostmap();
  for(unsigned int x=0;x<live->getSizeInCellsX();++x)
    for(unsigned int y=0;y<live->getSizeInCellsY();++y)live->setCost(x,y,0);
  auto tf=std::make_shared<tf2_ros::Buffer>(parent->get_clock());
  geometry_msgs::msg::PoseStamped a,b;a.header.frame_id=b.header.frame_id="map";
  a.pose.position.x=.011; a.pose.position.y=.012;
  b.pose.position.x=.035; b.pose.position.y=.038;
  a.pose.orientation.z=std::sin(.86);a.pose.orientation.w=std::cos(.86);
  b.pose.orientation.z=std::sin(M_PI/4);b.pose.orientation.w=std::cos(M_PI/4);
  {
    nav2_smac_planner::SmacPlannerLattice original;
    original.configure(parent,"GridBased",tf,map);
#ifdef CLEANY_TEST_LEGACY_SMAC
    EXPECT_TRUE(original.createPlan(a,b).poses.empty());
#endif
    original.cleanup();
  }
  {
    cleany_navigation::SmacLatticePlanner fixed;
    fixed.configure(parent,"GridBased",tf,map);
    auto plan=[&](const auto &start,const auto &goal) {
#if CLEANY_NAV2_CANCEL_CHECKER
      return fixed.createPlan(start,goal,[]{return false;});
#else
      return fixed.createPlan(start,goal);
#endif
    };
    auto path=plan(a,b);
    ASSERT_FALSE(path.poses.empty());
    EXPECT_NEAR(path.poses.back().pose.position.x,b.pose.position.x,1e-12);
    EXPECT_NEAR(path.poses.back().pose.position.y,b.pose.position.y,1e-12);
    EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation),M_PI/2,1e-12);
    EXPECT_TRUE(cleany_navigation::sweptPathCollisionFree(path,*live,map->getRobotFootprint()));
    EXPECT_FALSE(plan(b,b).poses.empty());
    auto far=b;far.pose.position.y=1.5;
    EXPECT_FALSE(plan(a,far).poses.empty());
    unsigned int x,y;ASSERT_TRUE(live->worldToMap(.025,.025,x,y));
    live->setCost(x,y,254);EXPECT_TRUE(plan(a,b).poses.empty());
    live->setCost(x,y,255);EXPECT_TRUE(plan(a,b).poses.empty());
    auto outside=a;outside.pose.position.x=100.;EXPECT_TRUE(plan(outside,b).poses.empty());
    fixed.cleanup();
  }
  map->cleanup();tf.reset();map.reset();parent.reset();rclcpp::shutdown();
}
