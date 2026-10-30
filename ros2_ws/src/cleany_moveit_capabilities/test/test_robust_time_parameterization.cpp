#include <cleany_moveit_capabilities/robust_time_parameterization.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <cmath>
#include <limits>
#include <random>
#include "retiming_regressions.hpp"

namespace
{
moveit::core::RobotModelPtr model(bool continuous=false, double upper=4., double velocity=0.)
{
  std::string xml="<robot name='timing'><link name='base'/>";
  for(int j=0;j<5;++j)
    xml+="<link name='l"+std::to_string(j)+"'/><joint name='j"+std::to_string(j)+
      "' type='"+(continuous?"continuous":"revolute")+"'><parent link='"+(j?"l"+std::to_string(j-1):"base")+
      "'/><child link='l"+std::to_string(j)+"'/><axis xyz='0 0 1'/><limit lower='-4' upper='"+std::to_string(upper)+"' effort='1' velocity='"+
      (velocity>0.?std::to_string(velocity):(j==1||j==2?"7.086":"4.245"))+"'/></joint>";
  auto urdf=urdf::parseURDF(xml+"</robot>");auto srdf=std::make_shared<srdf::Model>();
  srdf->initString(*urdf,"<robot name='timing'><group name='arm'><chain base_link='base' tip_link='l4'/></group></robot>");
  return std::make_shared<moveit::core::RobotModel>(urdf,srdf);
}
robot_trajectory::RobotTrajectory path(const moveit::core::RobotModelPtr& robot,
    const std::array<std::array<double,5>,3>& points, int subdivisions=1)
{
  robot_trajectory::RobotTrajectory result(robot,"arm");
  moveit::core::RobotState state(robot);state.setToDefaultValues();
  for(int segment=0;segment<2;++segment)
    for(int i=0;i<subdivisions;++i)
    {
      for(int j=0;j<5;++j)state.setVariablePosition("j"+std::to_string(j),
        points[segment][j]+(points[segment+1][j]-points[segment][j])*double(i)/subdivisions);
      result.addSuffixWayPoint(state,0.);
    }
  for(int j=0;j<5;++j)state.setVariablePosition("j"+std::to_string(j),points[2][j]);
  result.addSuffixWayPoint(state,0.);return result;
}
}

void expectQuinticLimits(const robot_trajectory::RobotTrajectory& timed, double velocity_scale=1., double acceleration_scale=1., bool serialized=false)
{
  if (!serialized)
  {
    moveit_msgs::msg::RobotTrajectory message;timed.getRobotTrajectoryMsg(message);
    robot_trajectory::RobotTrajectory wire(timed.getRobotModel(),timed.getGroupName());
    wire.setRobotTrajectoryMsg(timed.getFirstWayPoint(),message);
    expectQuinticLimits(wire,velocity_scale,acceleration_scale,true);
  }
  for (std::size_t i=1;i<timed.getWayPointCount();++i)
  {
    const double dt=timed.getWayPointDurationFromPrevious(i);ASSERT_GT(dt,0.);
    for (int joint=0;joint<5;++joint)
    {
      const std::string name="j"+std::to_string(joint);
      const auto& first=timed.getWayPoint(i-1);const auto& last=timed.getWayPoint(i);
      const double c0=first.getVariablePosition(name),c1=first.getVariableVelocity(name)*dt;
      const double c2=first.getVariableAcceleration(name)*dt*dt/2.;
      const double dq=last.getVariablePosition(name)-c0-c1-c2;
      const double dv=last.getVariableVelocity(name)*dt-c1-2*c2;
      const double da=last.getVariableAcceleration(name)*dt*dt-2*c2;
      const double c[]={c0,c1,c2,10*dq-4*dv+da/2,-15*dq+7*dv-da,6*dq-3*dv+da/2};
      for (int sample=0;sample<=200;++sample)
      {
        const double u=sample/200.;double velocity=0.,acceleration=0.;
        for(int k=1;k<6;++k)velocity+=k*c[k]*std::pow(u,k-1)/dt;
        for(int k=2;k<6;++k)acceleration+=k*(k-1)*c[k]*std::pow(u,k-2)/(dt*dt);
        ASSERT_LE(std::abs(velocity),timed.getRobotModel()->getVariableBounds(name).max_velocity_*velocity_scale+1e-8);
        ASSERT_LE(std::abs(acceleration),acceleration_scale+1e-8);
      }
    }
  }
}

TEST(RobustTiming, SavedBothArmFailuresUseCorrectedTotgWithoutFallback)
{
  auto robot=model();cleany::RetimingOptions options;
  for(const auto& points:REGRESSIONS)
  {
    auto raw=path(robot,points,200);robot_trajectory::RobotTrajectory old(raw,true);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(.0001);
    ASSERT_FALSE(totg.computeTimeStamps(old));
    ASSERT_EQ(cleany::retime_path(raw,1.,1.,options),cleany::RetimingResult::TOTG);
    ASSERT_GT(raw.getWayPointCount(),3u);
    for(int j=0;j<5;++j)
    {
      EXPECT_NEAR(raw.getFirstWayPoint().getVariablePosition("j"+std::to_string(j)),points[0][j],1e-12);
      EXPECT_NEAR(raw.getLastWayPoint().getVariablePosition("j"+std::to_string(j)),points[2][j],1e-12);
    }
    expectQuinticLimits(raw);
  }
}

TEST(RobustTiming, NormalTotgAndRequestScalingRemainAvailable)
{
  auto robot=model();auto points=REGRESSIONS[0];points[1]=points[0];points[2]=points[0];points[2][0]+=.05;
  auto raw=path(robot,points);cleany::RetimingOptions options;
  EXPECT_EQ(cleany::retime_path(raw,1.,1.,options),cleany::RetimingResult::TOTG);
  const double duration=raw.getDuration();raw=path(robot,points);
  EXPECT_NE(cleany::retime_path(raw,.2,.2,options),cleany::RetimingResult::FAILED);
  EXPECT_GT(raw.getDuration(),duration);
  expectQuinticLimits(raw,.2,.2);
}

TEST(RobustTiming, TinyMotionStationaryPathsAndExactReversalsArePreserved)
{
  auto robot=model();auto points=REGRESSIONS[0];points[1]=points[0];points[2]=points[0];points[1][0]+=.00001;
  auto raw=path(robot,points);cleany::RetimingOptions options;
  ASSERT_EQ(cleany::retime_path(raw,1.,1.,options),cleany::RetimingResult::TOTG);
  EXPECT_GE(raw.getWayPointCount(),3u);EXPECT_GT(raw.getDuration(),0.);
  expectQuinticLimits(raw);
  points[1]=points[0];raw=path(robot,points);
  EXPECT_EQ(cleany::retime_path(raw,1.,1.,options),cleany::RetimingResult::TOTG);
  ASSERT_EQ(raw.getWayPointCount(),1u);EXPECT_EQ(raw.getDuration(),0.);
  points[2][0]+=.00001;raw=path(robot,points);
  EXPECT_EQ(cleany::retime_path(raw,1.,1.,options),cleany::RetimingResult::TOTG);
  EXPECT_NEAR(raw.getLastWayPoint().getVariablePosition("j0"),points[2][0],1e-12);
  expectQuinticLimits(raw);
}

TEST(RobustTiming, InvalidInputIsNotMutatedOrReportedAsSuccess)
{
  auto robot=model();cleany::RetimingOptions options;auto raw=path(robot,REGRESSIONS[0]);
  const auto count=raw.getWayPointCount();
  EXPECT_EQ(cleany::retime_path(raw,-1.,1.,options),cleany::RetimingResult::FAILED);
  EXPECT_EQ(raw.getWayPointCount(),count);EXPECT_EQ(raw.getDuration(),0.);
  raw.getWayPointPtr(1)->setVariablePosition("j0",std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(cleany::retime_path(raw,1.,1.,options),cleany::RetimingResult::FAILED);
  cleany::RobustTimeParameterization adapter;
  planning_interface::MotionPlanRequest request;request.max_velocity_scaling_factor=-1.;
  planning_interface::MotionPlanResponse response;std::vector<std::size_t> added;
  auto planner=[&](const auto&,const auto&,auto& output){output.trajectory_=std::make_shared<robot_trajectory::RobotTrajectory>(path(robot,REGRESSIONS[0]));output.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;return true;};
  EXPECT_FALSE(adapter.adaptAndPlan(planner,nullptr,request,response,added));
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  EXPECT_FALSE(response.trajectory_);
}

TEST(RobustTiming, NearReversalPerturbationsAndScalingNeedNoAlternateSolver)
{
  auto robot=model();cleany::RetimingOptions options;
  for (const auto& original:REGRESSIONS)
    for (int sign:{-1,1})
      for (int joint=0;joint<5;++joint)
      {
        auto points=original;points[1][joint]+=sign*1e-5;
        auto raw=path(robot,points);
        ASSERT_EQ(cleany::retime_path(raw,.2,.3,options),cleany::RetimingResult::TOTG);
        expectQuinticLimits(raw,.2,.3);
      }
}

TEST(RobustTiming, ExcessiveFilteringFailsWithoutChangingThePath)
{
  auto robot=model(true);auto raw=path(robot,REGRESSIONS[0]);cleany::RetimingOptions options;
  options.min_angle_change=100.;
  EXPECT_EQ(cleany::retime_path(raw,1.,1.,options),cleany::RetimingResult::FAILED);
  EXPECT_EQ(raw.getWayPointCount(),3u);EXPECT_EQ(raw.getDuration(),0.);
}

TEST(RobustTiming, RandomNearReturnPathsUseTheSameSolver)
{
  auto robot=model();cleany::RetimingOptions options;std::mt19937 random(20261028);
  std::uniform_real_distribution<double> sample(-3.,3.);
  for (int i=0;i<256;++i)
  {
    SCOPED_TRACE(i);
    auto points=REGRESSIONS[i%4];
    for (double& value:points[1])value=sample(random);
    auto raw=path(robot,points);
    ASSERT_EQ(cleany::retime_path(raw,1.,1.,options),cleany::RetimingResult::TOTG);
    expectQuinticLimits(raw);
  }
}

namespace
{
void expectGeometry(const robot_trajectory::RobotTrajectory& timed,
                    const std::array<std::array<double,5>,3>& knots, double tolerance)
{
  for (std::size_t i=1; i<timed.getWayPointCount(); ++i)
  {
    const double dt=timed.getWayPointDurationFromPrevious(i);ASSERT_GT(dt,0.);
    for (int sample=0; sample<=200; ++sample)
    {
      const double u=sample/200.;Eigen::VectorXd point(5);
      for (int j=0; j<5; ++j)
      {
        const auto name="j"+std::to_string(j);const auto& a=timed.getWayPoint(i-1);const auto& b=timed.getWayPoint(i);
        const double c0=a.getVariablePosition(name),c1=a.getVariableVelocity(name)*dt,c2=a.getVariableAcceleration(name)*dt*dt/2.;
        const double dq=b.getVariablePosition(name)-c0-c1-c2,dv=b.getVariableVelocity(name)*dt-c1-2*c2,da=b.getVariableAcceleration(name)*dt*dt-2*c2;
        point[j]=c0+c1*u+c2*u*u+(10*dq-4*dv+da/2)*std::pow(u,3)+(-15*dq+7*dv-da)*std::pow(u,4)+(6*dq-3*dv+da/2)*std::pow(u,5);
        const auto& bounds=timed.getRobotModel()->getVariableBounds(name);
        ASSERT_GE(point[j],bounds.min_position_-1e-12);ASSERT_LE(point[j],bounds.max_position_+1e-12);
      }
      double distance=std::numeric_limits<double>::infinity();
      for (int segment=0;segment<2;++segment)
      {
        Eigen::VectorXd a(5),b(5);for(int j=0;j<5;++j){a[j]=knots[segment][j];b[j]=knots[segment+1][j];}
        const Eigen::VectorXd delta=b-a;const double ratio=delta.squaredNorm()>0.?std::clamp((point-a).dot(delta)/delta.squaredNorm(),0.,1.):0.;
        distance=std::min(distance,(point-a-ratio*delta).norm());
      }
      ASSERT_LE(distance,tolerance+1e-12);
    }
  }
}
}

TEST(RobustTiming, TinyTurnsAcrossScalesRetainBothEndpoints)
{
  auto robot=model();cleany::RetimingOptions options;
  for (double size:{1e-8,1e-6,1e-4})
    for (double degrees:{.01,30.,90.,150.,179.99,180.})
      for (double scale:{.05,.2,1.})
      {
        SCOPED_TRACE(::testing::Message()<<size<<" "<<degrees<<" "<<scale);
        std::array<std::array<double,5>,3> points{};points[1][0]=size;
        points[2][0]=size+size*std::cos(degrees*M_PI/180.);points[2][1]=size*std::sin(degrees*M_PI/180.);
        auto timed=path(robot,points);
        ASSERT_EQ(cleany::retime_path(timed,scale,scale,options),cleany::RetimingResult::TOTG);
        for (int j=0;j<5;++j)
        {
          EXPECT_NEAR(timed.getFirstWayPoint().getVariablePosition("j"+std::to_string(j)),points[0][j],1e-14);
          EXPECT_NEAR(timed.getLastWayPoint().getVariablePosition("j"+std::to_string(j)),points[2][j],1e-14);
        }
        expectQuinticLimits(timed,scale,scale);expectGeometry(timed,points,options.path_tolerance);
      }
}

TEST(RobustTiming, ReversalInterpolationRespectsPositionBounds)
{
  cleany::RetimingOptions options;
  for (double upper:{1.,1.005})
  for (double degrees:{179.99,180.})
  {
    auto robot=model(false,upper,1.);
    std::array<std::array<double,5>,3> points{};points[1][0]=1.;
    points[2][0]=1.+std::cos(degrees*M_PI/180.);points[2][1]=std::sin(degrees*M_PI/180.);
    auto timed=path(robot,points);
    ASSERT_EQ(cleany::retime_path(timed,1.,1.,options),cleany::RetimingResult::TOTG);
    EXPECT_LT(timed.getDuration(),12.);
    expectQuinticLimits(timed);expectGeometry(timed,points,options.path_tolerance);
  }
}

TEST(RobustTiming, ZeroToleranceRetainsSharpCorners)
{
  auto robot=model();cleany::RetimingOptions options;options.path_tolerance=0.;
  std::array<std::array<double,5>,3> points{};points[1][0]=.1;points[2][0]=.1;points[2][1]=.1;
  auto timed=path(robot,points);
  ASSERT_EQ(cleany::retime_path(timed,1.,1.,options),cleany::RetimingResult::TOTG);
  expectGeometry(timed,points,0.);expectQuinticLimits(timed);
}

TEST(RobustTiming, ShallowTurnDoesNotCreateNearDuplicateTimeSamples)
{
  auto robot=model(false,4.,1.);cleany::RetimingOptions options;
  std::array<std::array<double,5>,3> points{};points[1][0]=1.;
  points[2][0]=1.+std::cos(.01*M_PI/180.);points[2][1]=std::sin(.01*M_PI/180.);
  auto timed=path(robot,points);
  ASSERT_EQ(cleany::retime_path(timed,1.,1.,options),cleany::RetimingResult::TOTG);
  EXPECT_LT(timed.getDuration(),10.);
  expectGeometry(timed,points,options.path_tolerance);expectQuinticLimits(timed);
}
