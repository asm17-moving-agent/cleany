#include <array>
#include <limits>
#include <gtest/gtest.h>
#include "cleany_navigation/wheel_speed_limit.hpp"
#include "cleany_navigation/body_speed_limit.hpp"

TEST(WheelSpeedLimit, SmallCommandsAreNotMultipliedOrRaised) {
  cleany_navigation::WheelSpeedLimit limit(.0635, .35, .59, .7);
  EXPECT_DOUBLE_EQ(limit.ratio(.4 * .0635, 0, 0), 1.);
  EXPECT_DOUBLE_EQ(limit.ratio(0, 0, 0), 1.);
}

TEST(WheelSpeedLimit, TranslationRotationAndMixedMotionShareOneWheelCap) {
  cleany_navigation::WheelSpeedLimit limit(.0635, .35, .59, .7);
  for (auto v : std::array<std::array<double, 3>, 4>{{
      {{.127, 0, 0}}, {{0, 0, -.27}}, {{.08, -.04, .2}}, {{-.08, .04, -.2}}}}) {
    const double scale = limit.ratio(v[0], v[1], v[2]);
    const double x=v[0]*scale,y=v[1]*scale,r=.47*v[2]*scale;
    const double peak=std::max({std::abs(x-y-r),std::abs(x+y+r),
      std::abs(x+y-r),std::abs(x-y+r)})/.0635;
    EXPECT_NEAR(peak, .7, 1e-12);
    EXPECT_GT(scale, 0.);
    EXPECT_LT(scale, 1.);
  }
}

TEST(WheelSpeedLimit, InvalidConfigurationIsRejected) {
  EXPECT_THROW(cleany_navigation::WheelSpeedLimit(0., .35, .59, .7), std::invalid_argument);
  EXPECT_THROW(cleany_navigation::WheelSpeedLimit(.0635, .35, .59, -1.), std::invalid_argument);
  cleany_navigation::WheelSpeedLimit limit(.0635, .35, .59, .7);
  EXPECT_DOUBLE_EQ(limit.ratio(std::numeric_limits<double>::quiet_NaN(), 0, 0), 0.);
}

TEST(BodySpeedLimit, DiagonalUsesMagnitudeAndMixedMotionPreservesDirection) {
  cleany_navigation::BodySpeedLimit limit(.16,.3);
  EXPECT_DOUBLE_EQ(limit.ratio(.1,.1,.2),1.);
  EXPECT_NEAR(limit.ratio(.16,.16,.3),1./std::sqrt(2.),1e-12);
  EXPECT_NEAR(limit.ratio(0,0,.7),.3/.7,1e-12);
  double x=.5,y=.5,yaw=.7;
  cleany_navigation::projectBodyVelocity(x,y,yaw,.5,.7);
  EXPECT_NEAR(std::hypot(x,y),.5,1e-12);
  EXPECT_DOUBLE_EQ(yaw,.7);
  EXPECT_THROW(cleany_navigation::BodySpeedLimit(.16,-.3),std::invalid_argument);
  EXPECT_THROW(cleany_navigation::BodySpeedLimit(.16,std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
}

TEST(BodySpeedLimit, BrakingEnvelopeAccountsForLatencyAndNeverExceedsBounds) {
  constexpr double decel=.13, reaction=.3, fast=.5, slow=.16;
  const double full_distance=(fast*fast-slow*slow)/(2*decel)+fast*reaction;
  EXPECT_NEAR(cleany_navigation::approachingSpeed(fast,slow,decel,full_distance,reaction),fast,1e-12);
  EXPECT_DOUBLE_EQ(cleany_navigation::approachingSpeed(fast,slow,decel,0,reaction),slow);
  double previous=slow;
  for(double gap=0.;gap<2.;gap+=.01) {
    const double result=cleany_navigation::approachingSpeed(fast,slow,decel,gap,reaction);
    EXPECT_GE(result,previous);EXPECT_LE(result,fast);previous=result;
  }
}
