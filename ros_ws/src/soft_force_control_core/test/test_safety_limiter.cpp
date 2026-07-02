#include <gtest/gtest.h>
#include "soft_force_control_core/safety_limiter.h"

using sfc::CartesianCorrection;
using sfc::SafetyLimiter;
using sfc::SafetyParams;
using sfc::SafetyResult;
using sfc::Wrench;

TEST(SafetyLimiter, PassesSmallCorrectionUnchanged) {
  SafetyLimiter s;
  CartesianCorrection in;
  in.x = 0.1;
  in.a = 0.01;
  SafetyResult r = s.apply(in, Wrench{}, SafetyParams{});
  EXPECT_DOUBLE_EQ(r.correction.x, 0.1);
  EXPECT_DOUBLE_EQ(r.correction.a, 0.01);
  EXPECT_FALSE(r.hard_cutoff);
  EXPECT_FALSE(r.saturated);
}

TEST(SafetyLimiter, ClampsPerAxisAndFlagsSaturation) {
  SafetyLimiter s;
  CartesianCorrection in;
  in.y = 2.0;    // > 0.5 mm default
  in.b = -1.0;   // > 0.05 deg default
  SafetyResult r = s.apply(in, Wrench{}, SafetyParams{});
  EXPECT_DOUBLE_EQ(r.correction.y, 0.5);
  EXPECT_DOUBLE_EQ(r.correction.b, -0.05);
  EXPECT_TRUE(r.saturated);
  EXPECT_FALSE(r.hard_cutoff);
}

TEST(SafetyLimiter, ForceCeilingTriggersHardCutoff) {
  SafetyLimiter s;
  CartesianCorrection in;
  in.x = 0.1;
  Wrench w;
  w.fx = 300.0;
  w.fy = 400.0;  // norm = 500 -> at ceiling; use slightly above
  w.fz = 1.0;
  SafetyResult r = s.apply(in, w, SafetyParams{});
  EXPECT_TRUE(r.hard_cutoff);
  EXPECT_DOUBLE_EQ(r.correction.x, 0.0);
  EXPECT_DOUBLE_EQ(r.correction.a, 0.0);
}

TEST(SafetyLimiter, TorqueCeilingTriggersHardCutoff) {
  SafetyLimiter s;
  CartesianCorrection in;
  in.c = 0.01;
  Wrench w;
  w.tx = 60.0;  // > 50 Nm default
  SafetyResult r = s.apply(in, w, SafetyParams{});
  EXPECT_TRUE(r.hard_cutoff);
  EXPECT_DOUBLE_EQ(r.correction.c, 0.0);
}
