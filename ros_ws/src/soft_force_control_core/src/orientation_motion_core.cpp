#include "soft_force_control_core/orientation_motion_core.h"

#include <algorithm>
#include <cmath>

#include "soft_force_control_core/rotation.h"

namespace sfc {

void OrientationMotionCore::setGoal(const MotionGoal& g) {
  goal_ = g;
  status_ = MotionStatus::RUNNING;
  elapsed_ = 0;
  held_ = 0;
}

CartesianCorrection OrientationMotionCore::update(const CartesianState& s,
                                                  double dt) {
  CartesianCorrection out;
  if (status_ != MotionStatus::RUNNING) return out;

  elapsed_ += dt;
  if (elapsed_ > goal_.timeout_s) {
    status_ = MotionStatus::TIMEOUT;
    return out;
  }

  const double err[3] = {wrapDeg(goal_.a - s.a), wrapDeg(goal_.b - s.b),
                         wrapDeg(goal_.c - s.c)};

  const bool in_tol = std::fabs(err[0]) < goal_.tol_deg &&
                      std::fabs(err[1]) < goal_.tol_deg &&
                      std::fabs(err[2]) < goal_.tol_deg;
  if (in_tol) {
    held_ += dt;
    if (held_ >= goal_.hold_s) {
      status_ = MotionStatus::CONVERGED;
      return out;
    }
  } else {
    held_ = 0;
  }

  auto shape = [this, dt](double e) {
    const double v = std::max(-goal_.max_speed_dps,
                              std::min(goal_.max_speed_dps, goal_.p_gain * e));
    return v * dt;
  };
  out.a = shape(err[0]);
  out.b = shape(err[1]);
  out.c = shape(err[2]);
  return out;
}

}  // namespace sfc
