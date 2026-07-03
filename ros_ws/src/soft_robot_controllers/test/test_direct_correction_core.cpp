#include <gtest/gtest.h>

#include "soft_robot_controllers/direct_correction_core.h"

using soft_robot_controllers::DirectCorrectionCore;
using soft_robot_controllers::DirectCorrectionParams;
using soft_robot_controllers::DirectOutput;
using soft_robot_controllers::StreamCommand;

namespace {

constexpr double kDt = 0.004;
constexpr double kNow = 100.0;  // arbitrary monotonic clock origin

DirectCorrectionParams params() {
  DirectCorrectionParams p;
  p.stream_timeout_s = 0.1;
  p.safety.max_corr_trans = 0.5;
  p.safety.max_corr_rot = 0.05;
  return p;
}

DirectCorrectionCore makeCore() {
  DirectCorrectionCore core;
  core.configure(params());
  return core;
}

StreamCommand stream(double x, double stamp_s) {
  StreamCommand c;
  c.correction.x = x;
  c.stamp_s = stamp_s;
  c.valid = true;
  return c;
}

sfc::MotionGoal goalA(double a, double max_speed_dps = 7.5) {
  sfc::MotionGoal g;
  g.a = a;
  g.max_speed_dps = max_speed_dps;
  g.p_gain = 1.0;
  g.tol_deg = 0.1;
  g.hold_s = 0.008;  // two cycles inside tolerance
  g.timeout_s = 30.0;
  return g;
}

}  // namespace

TEST(DirectCore, FreshStreamPassesThrough) {
  DirectCorrectionCore core = makeCore();
  core.setStream(stream(0.2, kNow - 0.05));
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_NEAR(out.correction.x, 0.2, 1e-12);
  EXPECT_FALSE(out.stream_stale);
  EXPECT_FALSE(out.goal_active);
}

TEST(DirectCore, StaleStreamZeros) {
  DirectCorrectionCore core = makeCore();
  core.setStream(stream(0.2, kNow - 0.2));  // older than the 0.1 s timeout
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_EQ(out.correction.x, 0.0);
  EXPECT_TRUE(out.stream_stale);
}

TEST(DirectCore, NoStreamYetReportsStale) {
  DirectCorrectionCore core = makeCore();
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_EQ(out.correction.x, 0.0);
  EXPECT_TRUE(out.stream_stale);
}

TEST(DirectCore, StreamClampedBySafetyLimiter) {
  DirectCorrectionCore core = makeCore();
  core.setStream(stream(2.0, kNow));  // way above the 0.5 mm clamp
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_NEAR(out.correction.x, 0.5, 1e-12);
  EXPECT_TRUE(out.saturated);
}

TEST(DirectCore, RunningGoalOverridesStream) {
  DirectCorrectionCore core = makeCore();
  core.setStream(stream(0.2, kNow));
  core.startGoal(goalA(10.0));
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_TRUE(out.goal_active);
  EXPECT_EQ(out.correction.x, 0.0);            // stream ignored
  EXPECT_NEAR(out.correction.a, 0.03, 1e-12);  // 7.5 deg/s * 4 ms
  EXPECT_EQ(core.goalStatus(), sfc::MotionStatus::RUNNING);
}

TEST(DirectCore, GoalConvergesThenStreamResumes) {
  DirectCorrectionCore core = makeCore();
  sfc::CartesianState s;
  s.a = 0.05;                     // already inside the 0.1 deg tolerance
  core.startGoal(goalA(0.0));
  core.update(s, kNow, kDt);                    // hold 1/2
  core.update(s, kNow + kDt, kDt);              // hold 2/2 -> CONVERGED
  EXPECT_EQ(core.goalStatus(), sfc::MotionStatus::CONVERGED);
  core.setStream(stream(0.3, kNow + 2 * kDt));
  const DirectOutput out = core.update(s, kNow + 2 * kDt, kDt);
  EXPECT_FALSE(out.goal_active);
  EXPECT_NEAR(out.correction.x, 0.3, 1e-12);
}

TEST(DirectCore, GoalTimeoutZeroesOutput) {
  DirectCorrectionCore core = makeCore();
  sfc::MotionGoal g = goalA(90.0);
  g.timeout_s = 0.01;  // expires on the third 4 ms cycle
  core.startGoal(g);
  core.update(sfc::CartesianState{}, kNow, kDt);
  core.update(sfc::CartesianState{}, kNow + kDt, kDt);
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow + 2 * kDt, kDt);
  EXPECT_EQ(core.goalStatus(), sfc::MotionStatus::TIMEOUT);
  EXPECT_EQ(out.correction.a, 0.0);
}

TEST(DirectCore, CancelGoalReturnsToStream) {
  DirectCorrectionCore core = makeCore();
  core.startGoal(goalA(10.0));
  core.update(sfc::CartesianState{}, kNow, kDt);
  core.cancelGoal();
  EXPECT_EQ(core.goalStatus(), sfc::MotionStatus::INACTIVE);
  core.setStream(stream(0.1, kNow + kDt));
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow + kDt, kDt);
  EXPECT_FALSE(out.goal_active);
  EXPECT_NEAR(out.correction.x, 0.1, 1e-12);
}

TEST(DirectCore, GoalErrorDegIsGeodesic) {
  DirectCorrectionCore core = makeCore();
  core.startGoal(goalA(10.0));
  EXPECT_NEAR(core.goalErrorDeg(sfc::CartesianState{}), 10.0, 1e-6);
}
