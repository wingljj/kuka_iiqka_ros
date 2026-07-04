#pragma once

#include "soft_force_control_core/adaptive_deadband.h"
#include "soft_force_control_core/auto_retare.h"
#include "soft_force_control_core/compliance_law.h"
#include "soft_force_control_core/force_torque_filter.h"
#include "soft_force_control_core/frame_resolver.h"
#include "soft_force_control_core/orientation_motion_core.h"
#include "soft_force_control_core/safety_limiter.h"
#include "soft_force_control_core/tool_gravity_compensator.h"
#include "soft_force_control_core/types.h"

namespace soft_robot_controllers {

// One profile's worth of FORCE_COMPLIANCE parameters (spec sections
// 7.2-7.5, 12.1). The controller shell keeps one instance per profile
// (DRAG / PRECISION) and re-configures the core on profile entry.
struct ForceComplianceParams {
  double filter_cutoff_hz{10.0};
  sfc::ComplianceParams compliance;      // gains/speed caps; deadbands below
  double fixed_force_deadband_n{30.0};   // PRECISION defaults (spec 7.3)
  double fixed_torque_deadband_nm{4.0};
  bool adaptive_deadband{false};         // DRAG startup ramp (spec 7.4)
  double ramp_window_s{2.0};
  double ramp_force_margin_n{5.0};
  double ramp_torque_margin_nm{1.0};
  sfc::AutoReTareParams retare;          // DRAG auto re-tare (spec 7.5)
  double retare_rearm_factor{2.0};       // re-arm at factor * tol (decision 9)
  sfc::SafetyParams safety;              // per-cycle clamp + hard cutoff
  sfc::PayloadParams payload;
  double wrench_timeout_s{0.012};        // ~3 RSI cycles (spec section 8)
};

// Tool-frame chain locked at servo activation (spec: session-constant).
// All-zero == identity == legacy behavior.
struct ToolFrameConfig {
  double tool_a{0}, tool_b{0}, tool_c{0};     // $TOOL A/B/C [deg], from EKI
  double mount_a{0}, mount_b{0}, mount_c{0};  // sensor_to_flange A/B/C = Rz/Ry/Rx [deg]
};

struct ComplianceInput {
  sfc::CartesianState state;   // current TCP pose from the hardware handle
  sfc::Wrench raw;             // latest SRI sample from the realtime buffer
  double wrench_age_s{1e9};    // now - sample stamp
  bool wrench_valid{false};    // false until the first sample arrives
};

struct ComplianceOutput {
  sfc::CartesianCorrection correction;  // zero-initialised
  bool ramp_active{false};
  bool wrench_timeout{false};
  bool hard_cutoff{false};
  bool saturated{false};
  bool tared{false};
  sfc::Wrench compensated;              // diagnostics (post-compensation)
};

// Realtime FORCE_COMPLIANCE pipeline (spec 7.1): filter -> gravity
// compensation -> DRAG startup ramp -> edge-triggered auto re-tare ->
// admittance law -> sum with the embedded orientation motion ->
// SafetyLimiter. Pure logic: no ROS, no allocation, no blocking. dt is
// the measured controller period, passed in every cycle.
class ForceComplianceCore {
 public:
  // Stores params, REBUILDS the low-pass filter at the profile cutoff
  // (Plan 1 follow-up 3) and loads the payload into the compensator.
  void configure(const ForceComplianceParams& p);

  // Entering servo with this profile: reset filter/law, record the auto
  // re-tare reference orientation (disarmed until the TCP leaves it),
  // start the adaptive deadband ramp if the profile uses it.
  // Legacy entry: identity tool frame (kept for existing tests/callers).
  void activate(const sfc::CartesianState& start);
  // Tool-frame entry: locks R_tcp_sensor from $TOOL + mount at activation.
  void activate(const sfc::CartesianState& start, const ToolFrameConfig& tf);

  ComplianceOutput update(const ComplianceInput& in, double dt);

  // Embedded orientation motion path; its per-cycle output is summed with
  // the compliance correction BEFORE the SafetyLimiter (Plan 1 follow-up
  // 2). No ROS entry point in v1 (decision 4).
  sfc::OrientationMotionCore& motion() { return motion_; }

  double forceDeadband() const { return force_deadband_; }
  double torqueDeadband() const { return torque_deadband_; }
  const sfc::PayloadParams& payload() const { return compensator_.params(); }

 private:
  ForceComplianceParams params_;
  sfc::ForceTorqueFilter filter_{10.0};
  sfc::ToolGravityCompensator compensator_;
  sfc::FrameResolver frame_;
  sfc::ComplianceLaw law_;
  sfc::SafetyLimiter limiter_;
  sfc::AdaptiveDeadband ramp_;
  sfc::AutoReTare retare_;
  sfc::OrientationMotionCore motion_;
  double ref_a_{0}, ref_b_{0}, ref_c_{0};  // re-tare reference copy
  bool tare_armed_{false};
  double force_deadband_{0};
  double torque_deadband_{0};
};

}  // namespace soft_robot_controllers
