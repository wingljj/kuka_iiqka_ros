#include <gtest/gtest.h>
#include "soft_force_control_core/orientation_motion_core.h"

using sfc::CartesianCorrection;
using sfc::CartesianState;
using sfc::MotionGoal;
using sfc::MotionStatus;
using sfc::OrientationMotionCore;

namespace {
constexpr double kDt = 0.004;
CartesianState pose(double a, double b, double c) {
  CartesianState s;
  s.a = a;
  s.b = b;
  s.c = c;
  return s;
}
MotionGoal goalTo(double a, double b, double c) {
  MotionGoal g;
  g.a = a;
  g.b = b;
  g.c = c;
  g.max_speed_dps = 10.0;
  g.p_gain = 2.0;
  g.tol_deg = 0.05;
  g.hold_s = 0.02;
  g.timeout_s = 60.0;
  return g;
}
}  // namespace

TEST(OrientationMotion, InactiveOutputsZero) {
  OrientationMotionCore m;
  CartesianCorrection c = m.update(pose(0, 0, 0), kDt);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
  EXPECT_EQ(m.status(), MotionStatus::INACTIVE);
}

TEST(OrientationMotion, MovesTowardGoalWithClampedSpeed) {
  OrientationMotionCore m;
  m.setGoal(goalTo(90, 0, 0));
  CartesianCorrection c = m.update(pose(0, 0, 0), kDt);
  // err=90, p*err=180 dps -> clamped to 10 dps -> 0.04 deg per cycle
  EXPECT_NEAR(c.a, 10.0 * kDt, 1e-12);
  EXPECT_DOUBLE_EQ(c.x, 0.0);
  EXPECT_EQ(m.status(), MotionStatus::RUNNING);
}

TEST(OrientationMotion, TakesShortestPathAcrossWrap) {
  OrientationMotionCore m;
  m.setGoal(goalTo(-170, 0, 0));
  CartesianCorrection c = m.update(pose(170, 0, 0), kDt);
  EXPECT_GT(c.a, 0.0);  // +20 deg is shorter than -340
}

TEST(OrientationMotion, ConvergesAfterHoldTime) {
  OrientationMotionCore m;
  MotionGoal g = goalTo(1.0, 0, 0);
  m.setGoal(g);
  // Simulate the plant: integrate corrections onto the pose.
  CartesianState s = pose(0, 0, 0);
  MotionStatus st = MotionStatus::RUNNING;
  for (int i = 0; i < 20000 && st == MotionStatus::RUNNING; ++i) {
    CartesianCorrection c = m.update(s, kDt);
    s.a += c.a;
    st = m.status();
  }
  EXPECT_EQ(st, MotionStatus::CONVERGED);
  EXPECT_NEAR(s.a, 1.0, 0.1);
  // After convergence output is zero.
  CartesianCorrection c = m.update(s, kDt);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
}

TEST(OrientationMotion, TimesOutWhenStuck) {
  OrientationMotionCore m;
  MotionGoal g = goalTo(90, 0, 0);
  g.timeout_s = 0.02;  // 5 cycles
  m.setGoal(g);
  for (int i = 0; i < 6; ++i) m.update(pose(0, 0, 0), kDt);  // plant frozen
  EXPECT_EQ(m.status(), MotionStatus::TIMEOUT);
  CartesianCorrection c = m.update(pose(0, 0, 0), kDt);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
}

TEST(OrientationMotion, CancelStopsMotion) {
  OrientationMotionCore m;
  m.setGoal(goalTo(90, 0, 0));
  m.update(pose(0, 0, 0), kDt);
  m.cancel();
  EXPECT_EQ(m.status(), MotionStatus::INACTIVE);
  CartesianCorrection c = m.update(pose(0, 0, 0), kDt);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
}
