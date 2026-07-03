#include "soft_robot_controllers/direct_correction_core.h"

#include "soft_force_control_core/rotation.h"

namespace soft_robot_controllers {

void DirectCorrectionCore::reset() {
  stream_ = StreamCommand{};
  motion_.cancel();
}

void DirectCorrectionCore::startGoal(const sfc::MotionGoal& g) {
  goal_ = g;
  motion_.setGoal(g);
}

double DirectCorrectionCore::goalErrorDeg(const sfc::CartesianState& s) const {
  return sfc::angularDistanceDeg(goal_.a, goal_.b, goal_.c, s.a, s.b, s.c);
}

DirectOutput DirectCorrectionCore::update(const sfc::CartesianState& state,
                                          double now_s, double dt) {
  DirectOutput out;
  sfc::CartesianCorrection raw;
  if (motion_.status() == sfc::MotionStatus::RUNNING) {
    raw = motion_.update(state, dt);
    out.goal_active = true;
  } else if (stream_.valid &&
             now_s - stream_.stamp_s <= params_.stream_timeout_s) {
    raw = stream_.correction;
  } else {
    out.stream_stale = true;
  }
  const sfc::SafetyResult res =
      limiter_.apply(raw, sfc::Wrench{}, params_.safety);
  out.correction = res.correction;
  out.saturated = res.saturated;
  return out;
}

}  // namespace soft_robot_controllers
