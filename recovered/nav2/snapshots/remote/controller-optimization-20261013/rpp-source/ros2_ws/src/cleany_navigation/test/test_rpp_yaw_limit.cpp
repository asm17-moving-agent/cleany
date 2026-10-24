#include <cmath>
#include <limits>
#include "gtest/gtest.h"
#include "cleany_navigation/rpp_yaw_limit.hpp"
using cleany_navigation::rpp_yaw_scale;
TEST(RppYawLimit, RejectsInvalidInputs)
{
  for (double limit : {0.0, -0.14, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(rpp_yaw_scale(0.3, limit), std::invalid_argument);
  }
  EXPECT_THROW(rpp_yaw_scale(std::numeric_limits<double>::infinity(), 0.14), std::invalid_argument);
}
TEST(RppYawLimit, PreservesCheckedArcForSignedVelocities)
{
  for (double v : {-0.15, 0.0, 0.03, 0.15}) {
    for (double w : {-0.8, -0.14, -0.01, 0.0, 0.01, 0.14, 0.8}) {
      const double scale = rpp_yaw_scale(w, 0.14);
      EXPECT_GT(scale, 0.0); EXPECT_LE(scale, 1.0);
      EXPECT_LE(std::abs(w * scale), 0.14 + 1e-12);
      // Same SE(2) pose at scaled time: lowering both components must not
      // produce the wider turning radius caused by independent yaw clipping.
      if (std::abs(w) > 1e-12) {
        const double t = 0.7;
        EXPECT_NEAR(v / w * std::sin(w * scale * t),
          (v * scale) / (w * scale) * std::sin(w * scale * t), 1e-12);
        EXPECT_NEAR(v / w * (1.0 - std::cos(w * scale * t)),
          (v * scale) / (w * scale) * (1.0 - std::cos(w * scale * t)), 1e-12);
      } else { EXPECT_DOUBLE_EQ(scale, 1.0); }
      if (std::abs(w) <= 0.14) { EXPECT_DOUBLE_EQ(scale, 1.0); }
    }
  }
}
