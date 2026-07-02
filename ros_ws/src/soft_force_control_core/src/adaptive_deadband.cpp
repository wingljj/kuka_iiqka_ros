#include "soft_force_control_core/adaptive_deadband.h"

#include <algorithm>

namespace sfc {

void AdaptiveDeadband::start(double window_s, double force_margin_n,
                             double torque_margin_nm) {
  active_ = true;
  remaining_s_ = window_s;
  force_margin_ = force_margin_n;
  torque_margin_ = torque_margin_nm;
  max_force_ = 0;
  max_torque_ = 0;
}

bool AdaptiveDeadband::update(const Wrench& w, double dt) {
  if (!active_) return false;
  max_force_ = std::max(max_force_, w.forceNorm());
  max_torque_ = std::max(max_torque_, w.torqueNorm());
  remaining_s_ -= dt;
  if (remaining_s_ <= 0) {
    active_ = false;
    force_deadband_ = max_force_ + force_margin_;
    torque_deadband_ = max_torque_ + torque_margin_;
  }
  return active_;
}

}  // namespace sfc
