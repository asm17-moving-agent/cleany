#include <gtest/gtest.h>
#include "cleany_navigation/straight_first_planner.hpp"
TEST(StraightFirstPlanner, ArbitraryHeadingAndThinObstacleRespectEntireBody) {
  nav2_costmap_2d::Costmap2D map(200,200,.05,-5,-5,0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for (auto xy:std::vector<std::pair<double,double>>{{-.3,-.4},{.3,-.4},{.3,.4},{-.3,.4}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  geometry_msgs::msg::PoseStamped start,goal;start.header.frame_id=goal.header.frame_id="map";
  start.pose.orientation.w=goal.pose.orientation.w=1;goal.pose.position.x=3;goal.pose.position.y=-.4;
  auto path=cleany_navigation::checkedStraightPath(start,goal,map,footprint);
  ASSERT_FALSE(path.poses.empty());
  for (const auto &p:path.poses) EXPECT_NEAR(p.pose.position.y,-.4*p.pose.position.x/3,1e-9);
  auto behind=goal;behind.pose.position.x=-3;
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(start,behind,map,footprint).poses.empty());
  unsigned int x,y;map.worldToMap(1.5,0,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(start,goal,map,footprint).poses.empty());
  map.setCost(x,y,255);
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(start,goal,map,footprint).poses.empty());
}
TEST(StraightFirstPlanner, InitialRotationAndCancellationAreChecked) {
  nav2_costmap_2d::Costmap2D map(100,100,.05,-2.5,-2.5,0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for (auto xy:std::vector<std::pair<double,double>>{{-.1,-.5},{.1,-.5},{.1,.5},{-.1,.5}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  geometry_msgs::msg::PoseStamped a,b;a.header.frame_id=b.header.frame_id="map";
  a.pose.orientation.w=b.pose.orientation.w=1;b.pose.position.y=1;
  unsigned int x,y;map.worldToMap(.32,.32,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(a,b,map,footprint).poses.empty());
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(a,b,map,footprint,[] {return true;}).poses.empty());
}
