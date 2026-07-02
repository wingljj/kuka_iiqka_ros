#include <gtest/gtest.h>
#include "soft_force_control_core/adaptive_deadband.h"

using sfc::AdaptiveDeadband;
using sfc::Wrench;

namespace {
Wrench force(double fx, double tx = 0.0) {
  Wrench w;
  w.fx = fx;
  w.tx = tx;
  return w;
}
constexpr double kDt = 0.004;
}  // namespace

TEST(AdaptiveDeadband, InactiveByDefault) {
  AdaptiveDeadband d;
  EXPECT_FALSE(d.active());
  EXPECT_FALSE(d.update(force(1.0), kDt));
}

TEST(AdaptiveDeadband, ActiveDuringWindowThenDone) {
  AdaptiveDeadband d;
  d.start(0.02, 5.0, 1.0);  // 5 cycles at 4 ms
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(d.active());
    d.update(force(2.0, 0.5), kDt);
  }
  EXPECT_FALSE(d.active());
}

TEST(AdaptiveDeadband, DeadbandIsMaxResidualPlusMargin) {
  AdaptiveDeadband d;
  d.start(0.02, 5.0, 1.0);
  d.update(force(2.0, 0.2), kDt);
  d.update(force(3.0, 0.6), kDt);  // max
  d.update(force(1.0, 0.1), kDt);
  d.update(force(0.5, 0.0), kDt);
  d.update(force(0.5, 0.0), kDt);
  EXPECT_FALSE(d.active());
  EXPECT_NEAR(d.forceDeadband(), 3.0 + 5.0, 1e-12);
  EXPECT_NEAR(d.torqueDeadband(), 0.6 + 1.0, 1e-12);
}

TEST(AdaptiveDeadband, RestartClearsPreviousRamp) {
  AdaptiveDeadband d;
  d.start(0.008, 5.0, 1.0);
  d.update(force(10.0), kDt);
  d.update(force(10.0), kDt);
  d.start(0.008, 5.0, 1.0);
  d.update(force(1.0), kDt);
  d.update(force(1.0), kDt);
  EXPECT_NEAR(d.forceDeadband(), 6.0, 1e-12);
}
