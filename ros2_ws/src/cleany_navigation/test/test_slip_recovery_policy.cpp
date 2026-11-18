#include <gtest/gtest.h>
#include "cleany_navigation/slip_recovery_policy.hpp"
using P=cleany_navigation::SlipRecoveryPolicy;
TEST(SlipRecovery, BriefSuspectDoesNotRecoverAndUnknownCannotClearLatch) {
  P p({});p.reset(0);p.update(1,true,false,false);EXPECT_EQ(p.phase,P::Phase::Normal);
  p.update(1.1,true,true,false);EXPECT_EQ(p.phase,P::Phase::Braking);
  p.update(1.2,false,false,true);p.update(1.8,false,false,true);
  EXPECT_EQ(p.phase,P::Phase::Retry);
  p.update(2,true,false,true);EXPECT_EQ(p.phase,P::Phase::Retry);
}
TEST(SlipRecovery, RepeatedSlipBrakesAndFailsWithoutAnotherRetry) {
  P p({});p.reset(0);p.update(.1,true,true,false);
  p.update(.2,true,false,true);p.update(.8,true,false,true);ASSERT_EQ(p.phase,P::Phase::Retry);
  p.update(1,true,true,false);EXPECT_EQ(p.phase,P::Phase::Braking);
  p.update(1.1,true,false,true);p.update(1.7,true,false,true);
  EXPECT_EQ(p.phase,P::Phase::Failed);EXPECT_EQ(p.reason,"slip_repeated_during_retry");
}
TEST(SlipRecovery, MissingEvidenceStopsAndFailedStopTimesOut) {
  P p({});p.reset(0);p.update(3.1,false,false,false);EXPECT_EQ(p.phase,P::Phase::Braking);
  p.update(8.2,false,false,false);EXPECT_EQ(p.phase,P::Phase::Failed);
  EXPECT_EQ(p.reason,"stop_unverified");
}
TEST(SlipRecovery, StopEvidenceMustRemainFreshAndContinuous) {
  P p({});p.reset(0);p.update(.1,true,true,false);p.update(.2,true,false,true);
  p.update(.6,true,false,false);p.update(.8,true,false,true);p.update(1.,true,false,true);
  EXPECT_EQ(p.phase,P::Phase::Braking);p.update(1.4,true,false,true);EXPECT_EQ(p.phase,P::Phase::Retry);
}
TEST(SlipRecovery, NewGoalResetsBudgetButZeroMotionDuringSameGoalDoesNot) {
  P p({});p.reset(0);p.fail(1,"navigation_failed");p.update(1.1,true,false,true);p.update(1.7,true,false,true);
  ASSERT_EQ(p.phase,P::Phase::Failed);p.reset(2);EXPECT_EQ(p.phase,P::Phase::Normal);
}
