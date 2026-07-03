#pragma once

#include "soft_force_control_core/orientation_motion_core.h"
#include "soft_force_control_core/safety_limiter.h"
#include "soft_force_control_core/types.h"

namespace soft_robot_controllers {

struct DirectCorrectionParams {
  double stream_timeout_s{0.1};  // zero output on stale stream (spec 7.7)
  sfc::SafetyParams safety;      // per-cycle clamp; ceilings inert here
};

// Latest stream-mode command handed from the subscriber thread to
// update() through a RealtimeBuffer.
struct StreamCommand {
  sfc::CartesianCorrection correction;
  double stamp_s{0};
  bool valid{false};
};

struct DirectOutput {
  sfc::CartesianCorrection correction;
  bool goal_active{false};
  bool stream_stale{false};
  bool saturated{false};
};

// Command-source logic of CartesianCorrectionController (spec 5.3, 7.6,
// 7.7): a RUNNING orientation goal takes priority over the stream
// (decision 11); otherwise a fresh stream command passes through, held
// until stream_timeout_s. Both paths go through the SafetyLimiter with a
// zero wrench (the hard cutoff never trips: no force feedback in this
// controller). Pure logic, allocation-free, RT-safe.
class DirectCorrectionCore {
 public:
  void configure(const DirectCorrectionParams& p) { params_ = p; }

  // Mode (re-)entry: drop any stale stream command and cancel the goal.
  void reset();

  void setStream(const StreamCommand& cmd) { stream_ = cmd; }
  void startGoal(const sfc::MotionGoal& g);
  void cancelGoal() { motion_.cancel(); }
  sfc::MotionStatus goalStatus() const { return motion_.status(); }

  // Geodesic orientation error to the active goal, for action feedback.
  double goalErrorDeg(const sfc::CartesianState& s) const;

  DirectOutput update(const sfc::CartesianState& state, double now_s,
                      double dt);

 private:
  DirectCorrectionParams params_;
  StreamCommand stream_;
  sfc::MotionGoal goal_;  // copy kept for feedback error computation
  sfc::OrientationMotionCore motion_;
  sfc::SafetyLimiter limiter_;
};

}  // namespace soft_robot_controllers
