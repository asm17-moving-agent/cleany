#include <gtest/gtest.h>
#include "cleany_navigation/mppi/path_progress.hpp"

using namespace cleany_navigation::mppi;

TEST(MppiPathProgress, PlannedSlowTurnAdvancesWithoutRelaxingTheStallDeadline)
{
  const std::vector<ProgressPoint> path{{0,0,M_PI_2},{0,0,0},{1,0,0}};
  PathProgressGate gate;
  for (int second=0; second<=20; ++second) {
    const double yaw=M_PI_2-.075*second;
    EXPECT_TRUE(gate.observe(second,projectProgress(path,0,0,yaw,.5),yaw,false,12.,.08));
  }
  const double progress=projectProgress(path,0,0,M_PI_2-1.5,.5);
  EXPECT_FALSE(gate.observe(34.,progress,.07,false,12.,.08));
  EXPECT_DOUBLE_EQ(projectProgress(path,0,0,.4,0.),0.);
}

TEST(MppiPathProgress, WrongDirectionDoesNotCreditPlannedTurn)
{
  const std::vector<ProgressPoint> path{{0,0,M_PI_2},{0,0,0},{1,0,0}};
  PathProgressGate gate;
  EXPECT_TRUE(gate.observe(0.,projectProgress(path,0,0,M_PI_2,.5),M_PI_2,false,12.,.08));
  EXPECT_FALSE(gate.observe(13.,projectProgress(path,0,0,2.,.5),2.,false,12.,.08));
}

TEST(MppiPathProgress, TurnProjectionWrapsAcrossPi)
{
  const std::vector<ProgressPoint> path{{0,0,M_PI-.1},{0,0,-M_PI+.1},{1,0,-M_PI+.1}};
  EXPECT_NEAR(projectProgress(path,0,0,M_PI,.5),.05,1e-12);
  EXPECT_NEAR(projectProgress(path,0,0,-M_PI,.5),.05,1e-12);
}

TEST(MppiPathProgress, ReplannedTurnCreditsMotionWithoutCreditingNewRouteCoordinates)
{
  PathProgressGate gate;
  double yaw=3.;
  for (int second=0; second<=24; ++second) {
    // A replan may reverse the old final-turn branch or change its arc origin.
    gate.replan();
    EXPECT_TRUE(gate.observe(second,100.*second,yaw,false,12.,.08));
    yaw-=.06;
    EXPECT_TRUE(gate.observe(second+.5,100.*second+.03,yaw,false,12.,.08));
  }
  for (int second=25; second<=35; ++second) {
    gate.replan();
    EXPECT_TRUE(gate.observe(second,100.*second,yaw,false,12.,.08));
  }
  gate.replan();
  EXPECT_FALSE(gate.observe(38.,10000.,yaw,false,12.,.08));
}

TEST(MppiPathProgress, ReplanningStationaryRobotCannotExtendDeadline)
{
  PathProgressGate gate;
  for (int second=0; second<=12; ++second) {
    gate.replan();
    EXPECT_TRUE(gate.observe(second,100.*second,1.,false,12.,.08));
  }
  gate.replan();
  EXPECT_FALSE(gate.observe(13.,1300.,1.,false,12.,.08));
}
