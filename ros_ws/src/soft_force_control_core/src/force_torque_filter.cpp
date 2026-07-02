#include "soft_force_control_core/force_torque_filter.h"

namespace sfc {

Wrench ForceTorqueFilter::filter(const Wrench& in, double dt) {
  if (cutoff_hz_ <= 0.0) return in;
  if (!initialized_) {
    state_ = in;
    initialized_ = true;
    return state_;
  }
  const double rc = 1.0 / (2.0 * M_PI * cutoff_hz_);
  const double alpha = dt / (dt + rc);
  state_.fx += alpha * (in.fx - state_.fx);
  state_.fy += alpha * (in.fy - state_.fy);
  state_.fz += alpha * (in.fz - state_.fz);
  state_.tx += alpha * (in.tx - state_.tx);
  state_.ty += alpha * (in.ty - state_.ty);
  state_.tz += alpha * (in.tz - state_.tz);
  return state_;
}

}  // namespace sfc
