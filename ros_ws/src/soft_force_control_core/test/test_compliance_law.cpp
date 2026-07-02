#include <gtest/gtest.h>
#include "soft_force_control_core/compliance_law.h"

using sfc::CartesianCorrection;
using sfc::ComplianceLaw;
using sfc::ComplianceParams;
using sfc::Wrench;

namespace {
ComplianceParams defaultParams() {
  ComplianceParams p;
  p.translation.gain = 1.0;       // (mm/s)/N
  p.translation.deadband = 5.0;   // N
  p.translation.max_speed = 20.0; // mm/s
  p.translation.max_accel = 0.0;  // disabled unless a test enables it
  p.rotation.gain = 0.5;          // (deg/s)/Nm
  p.rotation.deadband = 1.0;      // Nm
  p.rotation.max_speed = 10.0;    // deg/s
  p.rotation.max_accel = 0.0;
  p.speed_scale = 1.0;
  return p;
}
constexpr double kDt = 0.004;
}  // namespace

TEST(ComplianceLaw, BelowDeadbandOutputsZero) {
  ComplianceLaw law;
  Wrench w;
  w.fx = 4.9;
  w.tz = 0.9;
  CartesianCorrection c = law.compute(w, defaultParams(), kDt);
  EXPECT_DOUBLE_EQ(c.x, 0.0);
  EXPECT_DOUBLE_EQ(c.c, 0.0);
}

TEST(ComplianceLaw, DeadzoneIsContinuous) {
  // Just above deadband: e = |F| - db, so output starts from ~zero.
  ComplianceLaw law;
  Wrench w;
  w.fx = 5.001;
  CartesianCorrection c = law.compute(w, defaultParams(), kDt);
  EXPECT_NEAR(c.x, 1.0 * 0.001 * kDt, 1e-12);
}

TEST(ComplianceLaw, NegativeForceGivesNegativeCorrection) {
  ComplianceLaw law;
  Wrench w;
  w.fy = -15.0;  // e = -10 N -> v = -10 mm/s
  CartesianCorrection c = law.compute(w, defaultParams(), kDt);
  EXPECT_NEAR(c.y, -10.0 * kDt, 1e-12);
}

TEST(ComplianceLaw, SpeedClamped) {
  ComplianceLaw law;
  Wrench w;
  w.fz = 1000.0;  // e = 995 -> v would be 995 mm/s, clamp to 20
  CartesianCorrection c = law.compute(w, defaultParams(), kDt);
  EXPECT_NEAR(c.z, 20.0 * kDt, 1e-12);
}

TEST(ComplianceLaw, SpeedScaleApplies) {
  ComplianceLaw law;
  ComplianceParams p = defaultParams();
  p.speed_scale = 0.5;
  Wrench w;
  w.fx = 15.0;  // e = 10 -> v = 10 * 0.5 = 5 mm/s
  CartesianCorrection c = law.compute(w, p, kDt);
  EXPECT_NEAR(c.x, 5.0 * kDt, 1e-12);
}

TEST(ComplianceLaw, RateLimitBoundsAcceleration) {
  ComplianceLaw law;
  ComplianceParams p = defaultParams();
  p.translation.max_accel = 100.0;  // mm/s^2 -> dv per cycle = 0.4 mm/s
  Wrench w;
  w.fx = 1000.0;
  CartesianCorrection c1 = law.compute(w, p, kDt);
  EXPECT_NEAR(c1.x, 0.4 * kDt, 1e-12);  // first cycle: v limited to 0.4
  CartesianCorrection c2 = law.compute(w, p, kDt);
  EXPECT_NEAR(c2.x, 0.8 * kDt, 1e-12);  // ramps up
}

TEST(ComplianceLaw, ResetClearsVelocityState) {
  ComplianceLaw law;
  ComplianceParams p = defaultParams();
  p.translation.max_accel = 100.0;
  Wrench w;
  w.fx = 1000.0;
  law.compute(w, p, kDt);
  law.reset();
  CartesianCorrection c = law.compute(w, p, kDt);
  EXPECT_NEAR(c.x, 0.4 * kDt, 1e-12);  // ramps from zero again
}

TEST(ComplianceLaw, RotationAxesUseTorques) {
  // Mapping: fx->x, fy->y, fz->z, tz->a, ty->b, tx->c
  // (KUKA A/B/C rotate about Z/Y/X respectively, spec section 6).
  ComplianceLaw law;
  Wrench t;
  t.tx = 3.0;  // e = 2 Nm -> v = 1 deg/s on the C channel
  CartesianCorrection c = law.compute(t, defaultParams(), kDt);
  EXPECT_NEAR(c.c, 0.5 * 2.0 * kDt, 1e-12);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
}
