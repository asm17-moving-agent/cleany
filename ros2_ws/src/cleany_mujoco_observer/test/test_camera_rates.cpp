#include <gtest/gtest.h>
#include <limits>
#include "cleany_mujoco_observer/camera_rates.hpp"
#include "cleany_mujoco_observer/depth_conversion.hpp"
using cleany_mujoco_observer::CameraRates;
using cleany_mujoco_observer::camera_is_due;

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
TEST(CameraRates, HeadRateFollowsPipelineDemand) {
  CameraRates r;
  EXPECT_EQ(r.rate("head", "head", false, true), r.head_active);
  EXPECT_EQ(r.rate("head", "head", false, false), r.head_idle);
  EXPECT_EQ(r.rate("head", "head", true, false), r.head_active);
  EXPECT_EQ(r.rate("head", "left", false, false), 0);
}
TEST(CameraRates, InvalidRatesRejected) {
  for(double bad: {0.,-1.,31.,std::numeric_limits<double>::infinity()}) {
    CameraRates r; r.head_active=bad; EXPECT_THROW(r.validate(),std::invalid_argument);
  }
  CameraRates r; r.head_idle=11; EXPECT_THROW(r.validate(),std::invalid_argument);
}
TEST(CameraRates, DepthBoostPreservesWristSelectionAndRestoresIdleRate) {
  CameraRates r;
  for (const auto& arm: {"left", "right"}) {
    EXPECT_EQ(r.rate(arm,"head",true),r.head_active);
    EXPECT_EQ(r.rate(arm,arm,true),r.wrist_active);
    EXPECT_EQ(r.rate(arm,arm==std::string("left") ? "right" : "left",true),0);
    EXPECT_EQ(r.rate(arm,"head",false),r.head_idle);
  }
}
TEST(CameraRates, UnknownCameraRejected) {
  CameraRates r;
  EXPECT_THROW(r.rate("unknown","head"),std::invalid_argument);
  EXPECT_THROW(r.rate("head","unknown"),std::invalid_argument);
}

TEST(CameraRates, SnapshotIsRequestedOnlyAtACaptureDeadline) {
  EXPECT_TRUE(camera_is_due(0.0, -1.0, 0.0, 10.0));
  EXPECT_FALSE(camera_is_due(0.05, 0.0, 0.1, 10.0));
  EXPECT_TRUE(camera_is_due(0.1, 0.0, 0.1, 10.0));
  EXPECT_FALSE(camera_is_due(0.1, 0.0, 0.1, 0.0));
  EXPECT_FALSE(camera_is_due(0.1, 0.1, 0.1, 10.0));
}

TEST(CameraRates, InvalidScheduleValuesAreRejected) {
  EXPECT_THROW(
    camera_is_due(std::numeric_limits<double>::quiet_NaN(), -1.0, 0.0, 10.0),
    std::invalid_argument);
  EXPECT_THROW(camera_is_due(0.0, -1.0, 0.0, -1.0), std::invalid_argument);
}

TEST(CameraRates, LateFramesKeepTheCapturePhaseWithoutCatchupBursts) {
  double next = 0.1;
  for (int frame = 1; frame <= 100; ++frame) {
    const double capture = frame * 0.1 + 0.008;
    ASSERT_TRUE(camera_is_due(capture, capture - 0.1, next, 10.0));
    next = cleany_mujoco_observer::next_camera_deadline(next, capture, 10.0);
    EXPECT_NEAR(next, (frame + 1) * 0.1, 1e-9);
  }
  EXPECT_NEAR(cleany_mujoco_observer::next_camera_deadline(0.1, 0.45, 10.0), 0.5, 1e-9);
  EXPECT_FALSE(camera_is_due(0.45, 0.45, 0.5, 10.0));
}

TEST(CameraRates, ClockResetMakesTheCameraDueAgain) {
  EXPECT_TRUE(camera_is_due(0.01, 100.0, 100.1, 10.0));
  EXPECT_FALSE(camera_is_due(0.01, 100.0, 100.1, 0.0));
}

TEST(CameraRates, DepthRowConversionMatchesProjection) {
  const float source[]{0.0f, 0.5f, 1.0f};
  float result[3]{};
  cleany_mujoco_observer::convert_depth_row(source, result, 3, 0.1f, 10.0f);
  EXPECT_FLOAT_EQ(result[0], 0.1f);
  EXPECT_NEAR(result[1], 0.1f / (1.0f - 0.5f * 0.99f), 1e-6f);
  EXPECT_NEAR(result[2], 10.0f, 1e-4f);
}
