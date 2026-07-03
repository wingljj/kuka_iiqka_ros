#include <gtest/gtest.h>

#include "soft_robot_controllers/force_compliance_core.h"

using soft_robot_controllers::ComplianceInput;
using soft_robot_controllers::ComplianceOutput;
using soft_robot_controllers::ForceComplianceCore;
using soft_robot_controllers::ForceComplianceParams;

namespace {

constexpr double kDt = 0.004;

ForceComplianceParams sumParams() {
  ForceComplianceParams p;
  p.filter_cutoff_hz = 0.0;
  p.compliance.rotation.gain = 1.0;       // (deg/s)/Nm
  p.compliance.rotation.max_speed = 10.0;
  p.compliance.rotation.max_accel = 0.0;
  p.compliance.translation.gain = 1.0;
  p.compliance.translation.max_speed = 50.0;
  p.compliance.translation.max_accel = 0.0;
  p.compliance.speed_scale = 1.0;
  p.fixed_force_deadband_n = 0.0;
  p.fixed_torque_deadband_nm = 0.0;
  p.adaptive_deadband = false;
  p.retare.enabled = false;
  p.safety.max_corr_rot = 0.05;  // deg per cycle
  return p;
}

// Orientation goal producing exactly 7.5 deg/s -> 0.03 deg per cycle.
sfc::MotionGoal motionGoal() {
  sfc::MotionGoal g;
  g.a = 10.0;             // far from the start pose (a = 0)
  g.max_speed_dps = 7.5;  // p_gain * error saturates at this cap
  g.p_gain = 1.0;
  g.tol_deg = 0.1;
  g.hold_s = 0.2;
  g.timeout_s = 30.0;
  return g;
}

ComplianceInput wrenchTz(double tz) {
  ComplianceInput in;
  in.wrench_valid = true;
  in.wrench_age_s = 0.0;
  in.raw.tz = tz;  // maps to the A axis (sfc law: tz -> a)
  return in;
}

}  // namespace

TEST(FccSum, MotionAloneProducesRotation) {
  ForceComplianceCore core;
  core.configure(sumParams());
  core.activate(sfc::CartesianState{});
  core.motion().setGoal(motionGoal());
  const ComplianceOutput out = core.update(wrenchTz(0.0), kDt);
  EXPECT_NEAR(out.correction.a, 0.03, 1e-12);
  EXPECT_FALSE(out.saturated);
  EXPECT_EQ(core.motion().status(), sfc::MotionStatus::RUNNING);
}

TEST(FccSum, ComplianceAloneStaysBelowClamp) {
  ForceComplianceCore core;
  core.configure(sumParams());
  core.activate(sfc::CartesianState{});
  // 7.5 Nm * 1.0 (deg/s)/Nm = 7.5 deg/s -> 0.03 deg per cycle.
  const ComplianceOutput out = core.update(wrenchTz(7.5), kDt);
  EXPECT_NEAR(out.correction.a, 0.03, 1e-12);
  EXPECT_FALSE(out.saturated);
}

TEST(FccSum, CombinedPathsAreClampedTogether) {
  // Plan 1 follow-up 2: 0.03 (compliance) + 0.03 (motion) = 0.06 deg must
  // be clamped to 0.05 by ONE limiter pass over the sum. Clamping each
  // path separately would let 0.06 deg through unclamped.
  ForceComplianceCore core;
  core.configure(sumParams());
  core.activate(sfc::CartesianState{});
  core.motion().setGoal(motionGoal());
  const ComplianceOutput out = core.update(wrenchTz(7.5), kDt);
  EXPECT_NEAR(out.correction.a, 0.05, 1e-12);
  EXPECT_TRUE(out.saturated);
}

TEST(FccSum, HardCutoffZeroesTheCombinedOutput) {
  ForceComplianceCore core;
  core.configure(sumParams());
  core.activate(sfc::CartesianState{});
  core.motion().setGoal(motionGoal());
  ComplianceInput in = wrenchTz(7.5);
  in.raw.fz = 600.0;  // above the 500 N ceiling
  const ComplianceOutput out = core.update(in, kDt);
  EXPECT_TRUE(out.hard_cutoff);
  EXPECT_EQ(out.correction.a, 0.0);  // motion path zeroed too
  EXPECT_EQ(out.correction.z, 0.0);
}
