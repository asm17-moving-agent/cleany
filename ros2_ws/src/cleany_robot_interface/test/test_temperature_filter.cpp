#include "cleany_feetech_hardware/temperature_filter.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include <vector>

using cleany_feetech_hardware::TemperatureMacd;
using cleany_feetech_hardware::TemperatureMacdSettings;

namespace
{
const TemperatureMacdSettings settings{0.1, 0.5, 0.1};

TemperatureMacd::Clock::time_point at(double seconds)
{
  return TemperatureMacd::Clock::time_point{} +
         std::chrono::duration_cast<TemperatureMacd::Clock::duration>(
    std::chrono::duration<double>(seconds));
}
}  // namespace

TEST(TemperatureMacd, InitializesAtMeasuredTemperatureIncludingHotStartup)
{
  for (double raw : {30., 55., 80., 94.}) {
    TemperatureMacd filter;
    EXPECT_DOUBLE_EQ(filter.update(raw, at(0), settings), raw);
    EXPECT_DOUBLE_EQ(filter.update(raw, at(1), settings), raw);
  }
}

TEST(TemperatureMacd, HistoricalIsolatedSpikesStayBelowThe55DegreeLimit)
{
  // Observed calibration spikes at 20 Hz; stable baselines were 35-36 C.
  for (double spike : {50., 53., 55., 71., 85., 94.}) {
    TemperatureMacd filter;
    filter.update(36, at(0), settings);
    double maximum = 36;
    for (int i = 1; i <= 100; ++i) {
      const double filtered = filter.update(i == 10 ? spike : 36, at(i * .05), settings);
      maximum = std::max(maximum, filtered);
      EXPECT_LT(filtered, 55) << "spike=" << spike << " sample=" << i;
      if (i == 100) {EXPECT_NEAR(filtered, 36, .01);}
    }
    if (spike > 55) {EXPECT_GT(maximum, 36);}
  }
}

TEST(TemperatureMacd, SustainedHeatCrossesLimitAcrossRatesAndJitter)
{
  for (const auto & periods : std::vector<std::vector<double>>{
      {.01}, {.02}, {.05}, {.012, .027, .016, .025}})
  {
    TemperatureMacd filter;
    filter.update(35, at(0), settings);
    double elapsed = 0;
    double filtered = 35;
    for (size_t i = 0; elapsed < 2 && filtered < 55; ++i) {
      elapsed += periods[i % periods.size()];
      filtered = filter.update(65, at(elapsed), settings);
    }
    EXPECT_GE(filtered, 55);
    EXPECT_GT(elapsed, .05);
    EXPECT_LT(elapsed, .5);
  }
}

TEST(TemperatureMacd, RisingTemperatureIsNotMistakenForAnIsolatedSpike)
{
  TemperatureMacd filter;
  filter.update(35, at(0), settings);
  double filtered = 35;
  for (int i = 1; i <= 230; ++i) {
    const double time = i * .02;
    filtered = filter.update(35 + 5 * time, at(time), settings);
  }
  EXPECT_GE(filtered, 55);
  EXPECT_LT(filtered, 60);
}

TEST(TemperatureMacd, DiscardingAPartialFrameDoesNotAdvanceCommittedHistory)
{
  TemperatureMacd filter;
  filter.update(35, at(0), settings);
  auto partial_frame = filter;
  EXPECT_GT(partial_frame.update(94, at(.05), settings), 35);
  EXPECT_DOUBLE_EQ(filter.update(35, at(.1), settings), 35);
}

TEST(TemperatureMacd, RejectsInvalidConfigurationMeasurementsAndBackwardsTime)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const TemperatureMacdSettings bad : {
      TemperatureMacdSettings{0, .5, .1}, {.5, .1, .1}, {.1, .1, .1},
      {.1, .5, 0}, {nan, .5, .1}, {.1, nan, .1}, {.1, .5, nan}})
  {
    TemperatureMacd filter;
    EXPECT_THROW(filter.update(35, at(0), bad), std::runtime_error);
  }
  TemperatureMacd filter;
  for (double bad : {nan, -1., 256.}) {
    EXPECT_THROW(filter.update(bad, at(0), settings), std::runtime_error);
  }
  filter.update(35, at(1), settings);
  EXPECT_THROW(filter.update(35, at(0), settings), std::runtime_error);
}
