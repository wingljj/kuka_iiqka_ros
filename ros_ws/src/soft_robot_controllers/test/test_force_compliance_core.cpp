#include <gtest/gtest.h>

#include "soft_robot_controllers/force_compliance_core.h"

using soft_robot_controllers::ComplianceInput;
using soft_robot_controllers::ComplianceOutput;
using soft_robot_controllers::ForceComplianceCore;
using soft_robot_controllers::ForceComplianceParams;

namespace {

// PRECISION-flavoured parameters with filtering disabled and simple gains
// so expected corrections are easy to compute by hand.
ForceComplianceParams baseParams() {
  ForceComplianceParams p;
  p.filter_cutoff_hz = 0.0;  // pass-through (sfc filter semantics)
  p.compliance.translation.gain = 1.0;    // (mm/s)/N
  p.compliance.translation.max_speed = 50.0;
  p.compliance.translation.max_accel = 0.0;  // rate limiting off
  p.compliance.rotation.gain = 0.1;       // (deg/s)/Nm
  p.compliance.rotation.max_speed = 5.0;
  p.compliance.rotation.max_accel = 0.0;
  p.compliance.speed_scale = 1.0;
  p.fixed_force_deadband_n = 30.0;   // spec 7.3 PRECISION defaults
  p.fixed_torque_deadband_nm = 4.0;
  p.adaptive_deadband = false;
  p.retare.enabled = false;
  p.safety.max_corr_trans = 0.5;
  p.safety.max_corr_rot = 0.05;
  p.safety.force_ceiling = 500.0;
  p.safety.torque_ceiling = 50.0;
  p.wrench_timeout_s = 0.012;
  return p;
}

ForceComplianceCore makeCore(const ForceComplianceParams& p) {
  ForceComplianceCore core;
  core.configure(p);
  core.activate(sfc::CartesianState{});
  return core;
}

ComplianceInput freshInput() {
  ComplianceInput in;
  in.wrench_valid = true;
  in.wrench_age_s = 0.0;
  return in;
}

}  // namespace

TEST(FccCore, InvalidWrenchOutputsZeroWithTimeoutFlag) {
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.wrench_valid = false;
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_TRUE(out.wrench_timeout);
  EXPECT_EQ(out.correction.x, 0.0);
  EXPECT_EQ(out.correction.a, 0.0);
}

TEST(FccCore, StaleWrenchOutputsZeroWithTimeoutFlag) {
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.raw.fz = 100.0;
  in.wrench_age_s = 0.02;  // > 0.012 (about 3 RSI cycles, spec section 8)
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_TRUE(out.wrench_timeout);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccCore, FixedDeadbandSuppressesSmallForces) {
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.raw.fz = 20.0;  // below the 30 N PRECISION deadband
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_FALSE(out.wrench_timeout);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccCore, ForceAboveDeadbandProducesTranslation) {
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.raw.fz = 40.0;  // e = 10 N -> v = 10 mm/s -> 0.04 mm per 4 ms cycle
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_NEAR(out.correction.z, 0.04, 1e-12);
  EXPECT_EQ(out.correction.x, 0.0);
  EXPECT_FALSE(out.saturated);
}

TEST(FccCore, CorrectionScalesWithMeasuredDt) {
  // Plan 2 follow-up 1: dt comes from the caller every cycle. Doubling dt
  // must double the per-cycle correction (below all speed limits).
  ComplianceInput in = freshInput();
  in.raw.fz = 40.0;
  ForceComplianceCore c1 = makeCore(baseParams());
  ForceComplianceCore c2 = makeCore(baseParams());
  const double z1 = c1.update(in, 0.004).correction.z;
  const double z2 = c2.update(in, 0.008).correction.z;
  EXPECT_NEAR(z2, 2.0 * z1, 1e-12);
}

TEST(FccCore, HardCutoffIsStrictlyGreaterThanCeiling) {
  // Plan 1 follow-up 4: exactly 500 N does NOT trip the cutoff.
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.raw.fz = 500.0;
  ComplianceOutput out = core.update(in, 0.004);
  EXPECT_FALSE(out.hard_cutoff);
  // e = 470 N -> v clamps to 50 mm/s -> 0.2 mm per cycle.
  EXPECT_NEAR(out.correction.z, 0.2, 1e-12);

  ForceComplianceCore core2 = makeCore(baseParams());
  in.raw.fz = 500.001;
  out = core2.update(in, 0.004);
  EXPECT_TRUE(out.hard_cutoff);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccCore, SafetyLimiterClampsPerCycleCorrection) {
  ForceComplianceParams p = baseParams();
  p.compliance.translation.max_speed = 500.0;  // allow a huge velocity
  ForceComplianceCore core = makeCore(p);
  ComplianceInput in = freshInput();
  in.raw.fz = 330.0;  // e = 300 -> v = 300 mm/s -> 1.2 mm >> 0.5 mm clamp
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_NEAR(out.correction.z, 0.5, 1e-12);
  EXPECT_TRUE(out.saturated);
  EXPECT_FALSE(out.hard_cutoff);
}

TEST(FccCore, GravityCompensationCancelsToolWeight) {
  ForceComplianceParams p = baseParams();
  p.payload.gravity_n = 10.0;  // tool weighs 10 N, CoM at the sensor origin
  ForceComplianceCore core = makeCore(p);
  ComplianceInput in = freshInput();
  // Flange at identity orientation: the sensor sees the tool weight as
  // -10 N along Z. Compensation must cancel it exactly.
  in.raw.fz = -10.0;
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_NEAR(out.compensated.fz, 0.0, 1e-9);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccCore, FilterSmoothsStepInput) {
  ForceComplianceParams p = baseParams();
  p.filter_cutoff_hz = 10.0;
  p.fixed_force_deadband_n = 0.0;
  ForceComplianceCore core = makeCore(p);
  ComplianceInput zero = freshInput();
  core.update(zero, 0.004);  // first sample initialises the filter at 0
  ComplianceInput step = freshInput();
  step.raw.fz = 100.0;
  const ComplianceOutput out = core.update(step, 0.004);
  // One low-pass step must land strictly between 0 and the raw value.
  EXPECT_GT(out.correction.z, 0.0);
  EXPECT_LT(out.correction.z, 100.0 * 1.0 * 0.004);
}
