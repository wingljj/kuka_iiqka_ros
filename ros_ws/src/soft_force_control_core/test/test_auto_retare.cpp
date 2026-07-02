#include <gtest/gtest.h>
#include "soft_force_control_core/auto_retare.h"

using sfc::AutoReTare;
using sfc::AutoReTareParams;
using sfc::CartesianState;
using sfc::Wrench;

namespace {
CartesianState pose(double a, double b, double c) {
  CartesianState s;
  s.a = a;
  s.b = b;
  s.c = c;
  return s;
}
}  // namespace

TEST(AutoReTare, DisabledNeverTares) {
  AutoReTare rt;
  rt.setReference(0, 0, 0);
  AutoReTareParams p;  // enabled = false
  EXPECT_FALSE(rt.shouldTare(pose(0, 0, 0), Wrench{}, 10.0, 2.0, p));
}

TEST(AutoReTare, TaresAtReferenceWithLowWrench) {
  AutoReTare rt;
  rt.setReference(10, 20, 30);
  AutoReTareParams p;
  p.enabled = true;
  Wrench w;
  w.fx = 1.0;  // below 10 N deadband
  EXPECT_TRUE(rt.shouldTare(pose(10.2, 20.1, 29.9), w, 10.0, 2.0, p));
}

TEST(AutoReTare, RejectsWhenOrientationFar) {
  AutoReTare rt;
  rt.setReference(0, 0, 0);
  AutoReTareParams p;
  p.enabled = true;
  EXPECT_FALSE(rt.shouldTare(pose(30, 0, 0), Wrench{}, 10.0, 2.0, p));
}

TEST(AutoReTare, RejectsWhenWrenchHigh) {
  AutoReTare rt;
  rt.setReference(0, 0, 0);
  AutoReTareParams p;
  p.enabled = true;
  Wrench w;
  w.fx = 50.0;  // above deadband
  EXPECT_FALSE(rt.shouldTare(pose(0, 0, 0), w, 10.0, 2.0, p));
  Wrench t;
  t.tx = 5.0;  // above torque deadband
  EXPECT_FALSE(rt.shouldTare(pose(0, 0, 0), t, 10.0, 2.0, p));
}
