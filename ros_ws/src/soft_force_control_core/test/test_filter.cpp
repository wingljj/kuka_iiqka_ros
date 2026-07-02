#include <gtest/gtest.h>
#include "soft_force_control_core/force_torque_filter.h"

using sfc::ForceTorqueFilter;
using sfc::Wrench;

namespace {
Wrench step(double v) {
  Wrench w;
  w.fx = w.fy = w.fz = w.tx = w.ty = w.tz = v;
  return w;
}
}  // namespace

TEST(Filter, NonPositiveCutoffPassesThrough) {
  ForceTorqueFilter f(0.0);
  Wrench out = f.filter(step(7.0), 0.004);
  EXPECT_DOUBLE_EQ(out.fx, 7.0);
  EXPECT_DOUBLE_EQ(out.tz, 7.0);
}

TEST(Filter, FirstSampleInitializesState) {
  ForceTorqueFilter f(10.0);
  Wrench out = f.filter(step(5.0), 0.004);
  EXPECT_DOUBLE_EQ(out.fx, 5.0);  // no startup transient from zero
}

TEST(Filter, SmoothsStepInput) {
  ForceTorqueFilter f(10.0);
  f.filter(step(0.0), 0.004);
  Wrench out = f.filter(step(100.0), 0.004);
  EXPECT_GT(out.fx, 0.0);
  EXPECT_LT(out.fx, 100.0);
}

TEST(Filter, ConvergesToStepValue) {
  ForceTorqueFilter f(10.0);
  f.filter(step(0.0), 0.004);
  Wrench out;
  for (int i = 0; i < 5000; ++i) out = f.filter(step(100.0), 0.004);
  EXPECT_NEAR(out.fx, 100.0, 1e-6);
  EXPECT_NEAR(out.tz, 100.0, 1e-6);
}

TEST(Filter, ResetClearsState) {
  ForceTorqueFilter f(10.0);
  f.filter(step(100.0), 0.004);
  f.reset();
  Wrench out = f.filter(step(5.0), 0.004);
  EXPECT_DOUBLE_EQ(out.fx, 5.0);  // behaves like first sample again
}
