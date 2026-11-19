#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "cleany_mujoco_observer/control_cycle.hpp"

using cleany_mujoco_observer::ControlCycleGate;

TEST(ControlCycle, DelayDuringReadKeepsOneStampAndCannotProduceDuplicateFeedback)
{
  int64_t clock = 100;
  struct Manager
  {
    int64_t& clock;
    std::vector<std::array<int64_t, 3>> calls;
    std::vector<int64_t> feedback;
    void read(int64_t stamp, int64_t period) {
      calls.push_back({0, stamp, period});
      clock += 28;  // Physics advances while the hardware read is delayed.
    }
    void update(int64_t stamp, int64_t period) {
      calls.push_back({1, stamp, period});
      feedback.push_back(stamp);
    }
    void write(int64_t stamp, int64_t period) {calls.push_back({2, stamp, period});}
  } manager{clock, {}, {}};
  ControlCycleGate gate(10);
  EXPECT_FALSE(gate.take(clock));
  clock = 110;
  auto first = gate.take(clock);
  ASSERT_TRUE(first);
  first->execute(manager);
  // The old loop stamps update at 138, sleeps until 120 (already past),
  // then can stamp the next update at 138 again. Capture the cycle time once.
  auto second = gate.take(clock);
  ASSERT_TRUE(second);
  second->execute(manager);
  ASSERT_EQ(manager.feedback.size(), 2U);
  EXPECT_EQ(manager.feedback[0], 110);
  EXPECT_EQ(manager.feedback[1], 138);
  EXPECT_LT(manager.feedback[0], manager.feedback[1]);
  for (size_t cycle = 0; cycle < 2; ++cycle) {
    const auto offset = cycle * 3;
    EXPECT_EQ(manager.calls[offset][1], manager.calls[offset + 1][1]);
    EXPECT_EQ(manager.calls[offset][1], manager.calls[offset + 2][1]);
    EXPECT_EQ(manager.calls[offset][2], manager.calls[offset + 1][2]);
    EXPECT_EQ(manager.calls[offset][2], manager.calls[offset + 2][2]);
  }
  EXPECT_EQ(manager.calls[3][2], 28);  // Actual elapsed time after the overrun.
}

TEST(ControlCycle, FrozenClockDoesNotRunControllersOrRefreshFeedback)
{
  ControlCycleGate gate(10);
  EXPECT_FALSE(gate.take(100));
  for (int attempt = 0; attempt < 100; ++attempt) {EXPECT_FALSE(gate.take(100));}
  EXPECT_EQ(gate.next_stamp_ns(), 110);
  ASSERT_TRUE(gate.take(110));
  for (int attempt = 0; attempt < 100; ++attempt) {EXPECT_FALSE(gate.take(110));}
  EXPECT_EQ(gate.next_stamp_ns(), 120);
}

TEST(ControlCycle, PartialClockProgressDoesNotCreateCatchupBursts)
{
  ControlCycleGate gate(10);
  EXPECT_FALSE(gate.take(100));
  for (int64_t now = 101; now < 110; ++now) {EXPECT_FALSE(gate.take(now));}
  auto cycle = gate.take(150);
  ASSERT_TRUE(cycle);
  EXPECT_EQ(cycle->stamp_ns, 150);
  EXPECT_EQ(cycle->period_ns, 50);
  EXPECT_FALSE(gate.take(150));
  EXPECT_FALSE(gate.take(159));
  EXPECT_EQ(gate.next_stamp_ns(), 160);
  cycle = gate.take(160);
  ASSERT_TRUE(cycle);
  EXPECT_EQ(cycle->period_ns, 10);
}

TEST(ControlCycle, ClockRegressionNeverRunsANegativePeriod)
{
  ControlCycleGate gate(10);
  EXPECT_FALSE(gate.take(100));
  ASSERT_TRUE(gate.take(110));
  EXPECT_THROW(gate.take(109), std::runtime_error);
  EXPECT_THROW(gate.take(0), std::runtime_error);
}

TEST(ControlCycle, InvalidControlPeriodsAreRejected)
{
  EXPECT_THROW(ControlCycleGate(0), std::invalid_argument);
  EXPECT_THROW(ControlCycleGate(-10), std::invalid_argument);
}
