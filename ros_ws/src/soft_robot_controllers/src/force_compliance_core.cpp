#include "soft_robot_controllers/force_compliance_core.h"

#include "soft_force_control_core/rotation.h"

namespace soft_robot_controllers {

void ForceComplianceCore::configure(const ForceComplianceParams& p) {
  params_ = p;
  // Rebuild-and-reset: cutoff is constructor-only on the sfc filter, so a
  // profile switch replaces the object (stack assignment, no allocation).
  filter_ = sfc::ForceTorqueFilter(p.filter_cutoff_hz);
  compensator_.setParams(p.payload);
  law_.reset();
  motion_.cancel();
  ramp_ = sfc::AdaptiveDeadband{};
  tare_armed_ = false;
  force_deadband_ = p.fixed_force_deadband_n;
  torque_deadband_ = p.fixed_torque_deadband_nm;
}

void ForceComplianceCore::activate(const sfc::CartesianState& start) {
  activate(start, ToolFrameConfig{});
}

void ForceComplianceCore::activate(const sfc::CartesianState& start,
                                   const ToolFrameConfig& tf) {
  frame_.configure(tf.tool_a, tf.tool_b, tf.tool_c, tf.mount_a, tf.mount_b,
                   tf.mount_c);
  filter_.reset();
  law_.reset();
  motion_.cancel();
  ref_a_ = start.a;
  ref_b_ = start.b;
  ref_c_ = start.c;
  retare_.setReference(start.a, start.b, start.c);
  tare_armed_ = false;  // never tare before leaving the reference once
  force_deadband_ = params_.fixed_force_deadband_n;
  torque_deadband_ = params_.fixed_torque_deadband_nm;
  if (params_.adaptive_deadband) {
    ramp_.start(params_.ramp_window_s, params_.ramp_force_margin_n,
                params_.ramp_torque_margin_nm);
  }
}

ComplianceOutput ForceComplianceCore::update(const ComplianceInput& in,
                                             double dt) {
  ComplianceOutput out;
  if (!in.wrench_valid || in.wrench_age_s > params_.wrench_timeout_s) {
    out.wrench_timeout = true;
    law_.reset();  // never resume with stale rate-limiter state
    return out;    // zero correction (spec 7.1 timeout rule)
  }

  const sfc::Wrench filtered = filter_.filter(in.raw, dt);
  // Frame chain (tool-frame design): r_bt is TCP->BASE from the live RIst
  // pose; r_base_sensor extends it through the session-constant sensor leg.
  const Eigen::Matrix3d r_bt =
      sfc::kukaAbcToRotation(in.state.a, in.state.b, in.state.c);
  const Eigen::Matrix3d r_base_sensor = r_bt * frame_.tcpFromSensor();
  out.compensated = compensator_.compensate(filtered, r_base_sensor);

  if (ramp_.active()) {
    // Startup ramp (spec 7.4): hold zero output while learning the
    // residual. The hard cutoff stays armed throughout.
    out.ramp_active = true;
    const bool still_ramping = ramp_.update(out.compensated, dt);
    if (!still_ramping) {
      force_deadband_ = ramp_.forceDeadband();
      torque_deadband_ = ramp_.torqueDeadband();
    }
    const sfc::SafetyResult guard = limiter_.apply(
        sfc::CartesianCorrection{}, out.compensated, params_.safety);
    out.hard_cutoff = guard.hard_cutoff;
    return out;
  }

  // Auto re-tare, edge-triggered (Plan 1 follow-up 1): re-arm only after
  // the TCP has clearly left the reference orientation (hysteresis factor,
  // decision 9); fire at most once per return, then disarm.
  const double dist = sfc::angularDistanceDeg(
      ref_a_, ref_b_, ref_c_, in.state.a, in.state.b, in.state.c);
  if (dist >
      params_.retare.orientation_tol_deg * params_.retare_rearm_factor) {
    tare_armed_ = true;
  }
  if (tare_armed_ &&
      retare_.shouldTare(in.state, out.compensated, force_deadband_,
                         torque_deadband_, params_.retare)) {
    compensator_.absorbResidual(out.compensated);
    tare_armed_ = false;
    out.tared = true;
    // Recompute with the updated bias: the residual is now absorbed.
    out.compensated = compensator_.compensate(filtered, r_base_sensor);
  }

  sfc::ComplianceParams law_params = params_.compliance;
  law_params.translation.deadband = force_deadband_;
  law_params.rotation.deadband = torque_deadband_;
  // Admittance in the TOOL frame (deadband/speed caps are tool-frame
  // semantics per the approved design), then back to BASE for RSI.
  const sfc::Wrench w_tool = frame_.wrenchSensorToTool(out.compensated);
  const sfc::CartesianCorrection compliance_tool =
      law_.compute(w_tool, law_params, dt);
  const sfc::CartesianCorrection compliance =
      frame_.correctionToolToBase(compliance_tool, r_bt);
  const sfc::CartesianCorrection motion = motion_.update(in.state, dt);

  // Plan 1 follow-up 2: sum both correction paths BEFORE the limiter so
  // the per-cycle clamp applies to the combined magnitude.
  sfc::CartesianCorrection sum;
  sum.x = compliance.x + motion.x;
  sum.y = compliance.y + motion.y;
  sum.z = compliance.z + motion.z;
  sum.a = compliance.a + motion.a;
  sum.b = compliance.b + motion.b;
  sum.c = compliance.c + motion.c;

  const sfc::SafetyResult res =
      limiter_.apply(sum, out.compensated, params_.safety);
  out.correction = res.correction;
  out.hard_cutoff = res.hard_cutoff;
  out.saturated = res.saturated;
  if (res.hard_cutoff) law_.reset();
  return out;
}

}  // namespace soft_robot_controllers
