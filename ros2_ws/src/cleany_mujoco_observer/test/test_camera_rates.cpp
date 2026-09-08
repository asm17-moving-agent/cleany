#include <gtest/gtest.h>
#include <limits>
#include "cleany_mujoco_observer/camera_rates.hpp"
using cleany_mujoco_observer::CameraRates;

TEST(CameraRates, SearchOnlyRendersHead) {
  CameraRates r; r.validate();
  EXPECT_EQ(r.rate("head","head"),10);
  EXPECT_EQ(r.rate("head","left"),0); EXPECT_EQ(r.rate("head","right"),0);
}
TEST(CameraRates, BothWristModesKeepLowRateHeadAndOnlySelectedWrist) {
  CameraRates r;
  EXPECT_EQ(r.rate("left","head"),2); EXPECT_EQ(r.rate("right","head"),2);
  EXPECT_EQ(r.rate("left","left"),10); EXPECT_EQ(r.rate("right","right"),10);
  EXPECT_EQ(r.rate("left","right"),0); EXPECT_EQ(r.rate("right","left"),0);
}
TEST(CameraRates, InvalidRatesRejected) {
  for(double bad: {0.,-1.,31.,std::numeric_limits<double>::infinity()}) {
    CameraRates r; r.head_active=bad; EXPECT_THROW(r.validate(),std::invalid_argument);
  }
  CameraRates r; r.head_idle=11; EXPECT_THROW(r.validate(),std::invalid_argument);
}
TEST(CameraRates, UnknownCameraRejected) {
  CameraRates r;
  EXPECT_THROW(r.rate("unknown","head"),std::invalid_argument);
  EXPECT_THROW(r.rate("head","unknown"),std::invalid_argument);
}
