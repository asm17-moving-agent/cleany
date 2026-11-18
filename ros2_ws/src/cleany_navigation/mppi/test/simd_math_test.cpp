#include <cmath>

#include <gtest/gtest.h>
#include <xtensor/xmath.hpp>
#include <xtensor/xtensor.hpp>

TEST(MppiSimdMath, RoundingMatchesScalarMath)
{
  const xt::xtensor<float, 1> values = {0.3f, 0.6f, 1.4f, 1.6f, -0.3f, -0.6f, -1.4f, -1.6f};
  const xt::xtensor<float, 1> rounded = xt::nearbyint(values);
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_FLOAT_EQ(rounded(i), std::nearbyint(values(i))) << "input=" << values(i);
  }
}

TEST(MppiSimdMath, UnitPowerPreservesCriticCosts)
{
  const xt::xtensor<float, 1> values = {0.48f, 1.2f, 2.5f, 3.3f, 3.6315f, 5.0f, 15.0f, 100.0f};
  volatile unsigned int configured_power = 1;
  const unsigned int power = configured_power;
  const xt::xtensor<float, 1> powered = xt::pow(values, power);
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_NEAR(powered(i), values(i), std::abs(values(i)) * 1e-5f) << "input=" << values(i);
  }
}

TEST(MppiSimdMath, ExponentialPreservesOptimizerWeights)
{
  const xt::xtensor<float, 1> values = {-4.0f, -2.0f, -1.0f, -0.5f, 0.0f, 0.5f, 1.0f, 2.0f};
  const xt::xtensor<float, 1> weights = xt::exp(values);
  for (std::size_t i = 0; i < values.size(); ++i) {
    const float expected = std::exp(values(i));
    EXPECT_NEAR(weights(i), expected, expected * 1e-5f) << "input=" << values(i);
  }
}
