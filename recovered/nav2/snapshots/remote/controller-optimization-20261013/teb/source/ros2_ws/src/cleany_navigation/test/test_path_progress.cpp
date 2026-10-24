#include <gtest/gtest.h>
#include <limits>
#include <cmath>
#include "cleany_navigation/path_progress.hpp"
#include "dwb_core/exceptions.hpp"
namespace {
geometry_msgs::msg::Pose2D point(double x,double y) {geometry_msgs::msg::Pose2D p;p.x=x;p.y=y;return p;}
}
TEST(PathProgress, DiagonalProgressBeatsStandingStillAndBackwardMotion) {
  cleany_navigation::PathProgressCritic critic;nav_2d_msgs::msg::Path2D path;
  path.poses={point(0,0),point(.5,.5),point(1.,0.)};
  ASSERT_TRUE(critic.prepare(point(.1,.1),{},point(1.,0.),path));
  dwb_msgs::msg::Trajectory2D stopped,forward,backward;
  stopped.poses={point(.1,.1),point(.1,.1)};forward.poses={point(.1,.1),point(.3,.3)};backward.poses={point(.1,.1),point(0,0)};
  EXPECT_LT(critic.scoreTrajectory(forward),critic.scoreTrajectory(stopped));
  EXPECT_GT(critic.scoreTrajectory(backward),critic.scoreTrajectory(stopped));
}
TEST(PathProgress, InPlaceTurnsAndLoopCrossingsDoNotInventTravel) {
  cleany_navigation::PathProgressCritic critic;nav_2d_msgs::msg::Path2D path;
  auto turn=point(0,0);turn.theta=1.;path.poses={point(0,0),turn,point(1,0),point(1,1),point(0,0)};
  ASSERT_TRUE(critic.prepare(point(0,0),{},point(0,0),path));
  dwb_msgs::msg::Trajectory2D rotate;rotate.poses={point(0,0),turn};
  const double length=2.+std::sqrt(2.);EXPECT_NEAR(critic.scoreTrajectory(rotate),length,1e-12);
  dwb_msgs::msg::Trajectory2D loop;loop.poses={point(0,0),point(.1,.1)};
  EXPECT_GE(critic.scoreTrajectory(loop),length-std::sqrt(.02)-1e-12);
}
TEST(PathProgress, InvalidPreparationAndTrajectoriesCannotReusePriorPath) {
  cleany_navigation::PathProgressCritic critic;nav_2d_msgs::msg::Path2D path;path.poses={point(0,0),point(1,0)};
  ASSERT_TRUE(critic.prepare(point(0,0),{},point(1,0),path));
  dwb_msgs::msg::Trajectory2D trajectory;trajectory.poses={point(0,0),point(.1,0)};
  EXPECT_TRUE(std::isfinite(critic.scoreTrajectory(trajectory)));
  trajectory.poses.back().x=std::numeric_limits<double>::quiet_NaN();EXPECT_THROW(critic.scoreTrajectory(trajectory),dwb_core::IllegalTrajectoryException);
  path.poses.clear();EXPECT_FALSE(critic.prepare(point(0,0),{},point(1,0),path));
  trajectory.poses={point(0,0)};EXPECT_THROW(critic.scoreTrajectory(trajectory),dwb_core::IllegalTrajectoryException);
}
