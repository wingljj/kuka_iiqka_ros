#include <gtest/gtest.h>

#include "soft_robot_controllers/force_compliance_core.h"

using soft_robot_controllers::ComplianceInput;
using soft_robot_controllers::ComplianceOutput;
using soft_robot_controllers::ForceComplianceCore;
using soft_robot_controllers::ForceComplianceParams;

namespace {

constexpr double kDt = 0.004;

// DRAG-flavoured parameters with a 5-cycle ramp window for fast tests.
ForceComplianceParams dragParams() {
  ForceComplianceParams p;
  p.filter_cutoff_hz = 0.0;
  p.compliance.translation.gain = 1.0;
  p.compliance.translation.max_speed = 50.0;
  p.compliance.translation.max_accel = 0.0;
  p.compliance.rotation.gain = 0.1;
  p.compliance.rotation.max_speed = 5.0;
  p.compliance.rotation.max_accel = 0.0;
  p.compliance.speed_scale = 1.0;
  p.adaptive_deadband = true;
  p.ramp_window_s = 0.02;  // 5 cycles at 4 ms
  p.ramp_force_margin_n = 5.0;   // legacy FTLimSet(5+FSum, 1+TSum)
  p.ramp_torque_margin_nm = 1.0;
  p.retare.enabled = true;
  p.retare.orientation_tol_deg = 1.0;
  p.retare_rearm_factor = 2.0;
  p.wrench_timeout_s = 0.012;
  return p;
}

// Fixed-deadband variant used by the re-tare tests (ramp off so the
// deadbands are deterministic from the first cycle).
ForceComplianceParams retareParams() {
  ForceComplianceParams p = dragParams();
  p.adaptive_deadband = false;
  p.fixed_force_deadband_n = 30.0;
  p.fixed_torque_deadband_nm = 4.0;
  return p;
}

ComplianceInput inputAt(double a_deg, double fx) {
  ComplianceInput in;
  in.wrench_valid = true;
  in.wrench_age_s = 0.0;
  in.state.a = a_deg;
  in.raw.fx = fx;
  return in;
}

}  // namespace

TEST(FccRamp, HoldsZeroForWholeWindow) {
  ForceComplianceCore core;
  core.configure(dragParams());
  core.activate(sfc::CartesianState{});
  for (int i = 0; i < 5; ++i) {
    const ComplianceOutput out = core.update(inputAt(0.0, 8.0), kDt);
    EXPECT_TRUE(out.ramp_active) << "cycle " << i;
    EXPECT_EQ(out.correction.x, 0.0) << "cycle " << i;
  }
  // Window over: residual 8 N + margin 5 N = 13 N deadband.
  const ComplianceOutput out = core.update(inputAt(0.0, 20.0), kDt);
  EXPECT_FALSE(out.ramp_active);
  EXPECT_NEAR(out.correction.x, (20.0 - 13.0) * 1.0 * kDt, 1e-12);
}

TEST(FccRamp, LearnsResidualPlusMargin) {
  ForceComplianceCore core;
  core.configure(dragParams());
  core.activate(sfc::CartesianState{});
  for (int i = 0; i < 5; ++i) core.update(inputAt(0.0, 2.0), kDt);
  EXPECT_NEAR(core.forceDeadband(), 7.0, 1e-9);   // 2 + 5
  EXPECT_NEAR(core.torqueDeadband(), 1.0, 1e-9);  // 0 + 1
  const ComplianceOutput out = core.update(inputAt(0.0, 8.0), kDt);
  EXPECT_NEAR(out.correction.x, (8.0 - 7.0) * kDt, 1e-12);
}

TEST(FccRamp, HardCutoffStaysArmedDuringRamp) {
  ForceComplianceCore core;
  core.configure(dragParams());
  core.activate(sfc::CartesianState{});
  ComplianceInput in = inputAt(0.0, 0.0);
  in.raw.fz = 600.0;
  const ComplianceOutput out = core.update(in, kDt);
  EXPECT_TRUE(out.ramp_active);
  EXPECT_TRUE(out.hard_cutoff);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccRetare, NeverFiresWithoutLeavingReference) {
  // Plan 1 follow-up 1: the stateless shouldTare predicate would be true
  // every cycle here; the edge trigger must keep it from ever firing.
  ForceComplianceCore core;
  core.configure(retareParams());
  core.activate(sfc::CartesianState{});
  for (int i = 0; i < 10; ++i) {
    const ComplianceOutput out = core.update(inputAt(0.0, 5.0), kDt);
    EXPECT_FALSE(out.tared) << "cycle " << i;
  }
  EXPECT_EQ(core.payload().bias.fx, 0.0);  // bias untouched: no drift
}

TEST(FccRetare, FiresExactlyOncePerReturn) {
  ForceComplianceCore core;
  core.configure(retareParams());
  core.activate(sfc::CartesianState{});
  for (int i = 0; i < 3; ++i) core.update(inputAt(0.0, 5.0), kDt);
  // Leave the reference beyond rearm_factor * tol = 2 deg.
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(core.update(inputAt(5.0, 5.0), kDt).tared);
  }
  // Return: exactly one tare, then quiet.
  EXPECT_TRUE(core.update(inputAt(0.0, 5.0), kDt).tared);
  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(core.update(inputAt(0.0, 5.0), kDt).tared);
  }
}

TEST(FccRetare, AbsorbsResidualIntoBias) {
  ForceComplianceCore core;
  core.configure(retareParams());
  core.activate(sfc::CartesianState{});
  core.update(inputAt(5.0, 5.0), kDt);  // arm
  const ComplianceOutput out = core.update(inputAt(0.0, 5.0), kDt);
  EXPECT_TRUE(out.tared);
  EXPECT_NEAR(core.payload().bias.fx, 5.0, 1e-9);
  EXPECT_NEAR(out.compensated.fx, 0.0, 1e-9);  // recomputed after absorb
}

TEST(FccRetare, ReArmsAfterLeavingAgain) {
  ForceComplianceCore core;
  core.configure(retareParams());
  core.activate(sfc::CartesianState{});
  core.update(inputAt(5.0, 5.0), kDt);
  ASSERT_TRUE(core.update(inputAt(0.0, 5.0), kDt).tared);  // bias.fx = 5
  core.update(inputAt(5.0, 3.0), kDt);  // leave again
  // Second return: the residual is the COMPENSATED value 3 - 5 = -2, so
  // the bias converges onto the current raw force: 5 + (-2) = 3.
  EXPECT_TRUE(core.update(inputAt(0.0, 3.0), kDt).tared);
  EXPECT_NEAR(core.payload().bias.fx, 3.0, 1e-9);
}

TEST(FccRetare, DisabledProfileNeverTares) {
  ForceComplianceParams p = retareParams();
  p.retare.enabled = false;  // PRECISION profile behavior (spec 7.3)
  ForceComplianceCore core;
  core.configure(p);
  core.activate(sfc::CartesianState{});
  core.update(inputAt(5.0, 5.0), kDt);
  EXPECT_FALSE(core.update(inputAt(0.0, 5.0), kDt).tared);
  EXPECT_EQ(core.payload().bias.fx, 0.0);
}

TEST(FccProfile, ReconfigureRebuildsAndResetsFilter) {
  // Plan 1 follow-up 3: profile switches rebuild the filter and reset its
  // state. After configure()+activate(), the first sample re-initialises
  // the low-pass (sfc filter semantics), so a step passes through whole.
  ForceComplianceParams p = retareParams();
  p.filter_cutoff_hz = 10.0;
  p.fixed_force_deadband_n = 0.0;
  // DEVIATION from the plan's verbatim code (flagged for review): without
  // this line the inherited translation.max_speed = 50 mm/s clamps the
  // 100 N step to 0.2 mm and masks the filter-reset expectation below.
  p.compliance.translation.max_speed = 500.0;  // allow a huge velocity
  ForceComplianceCore core;
  core.configure(p);
  core.activate(sfc::CartesianState{});
  core.update(inputAt(0.0, 0.0), kDt);    // filter state pinned at 0
  const double smoothed = core.update(inputAt(0.0, 100.0), kDt).correction.x;
  EXPECT_LT(smoothed, 100.0 * kDt);       // low-pass engaged

  core.configure(p);                      // profile re-entry
  core.activate(sfc::CartesianState{});
  const double fresh = core.update(inputAt(0.0, 100.0), kDt).correction.x;
  EXPECT_NEAR(fresh, 100.0 * kDt, 1e-12); // first sample after reset
}
