#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

// Startup adaptive deadband (legacy LIMSET, spec section 7.4). While active,
// the caller must output zero and feed every compensated wrench through
// update(). After the window, deadband = max residual norm + margin.
class AdaptiveDeadband {
 public:
  void start(double window_s, double force_margin_n, double torque_margin_nm);
  bool update(const Wrench& compensated, double dt);  // true while ramping
  bool active() const { return active_; }
  double forceDeadband() const { return force_deadband_; }
  double torqueDeadband() const { return torque_deadband_; }

 private:
  bool active_{false};
  double remaining_s_{0};
  double force_margin_{0}, torque_margin_{0};
  double max_force_{0}, max_torque_{0};
  double force_deadband_{0}, torque_deadband_{0};
};

}  // namespace sfc
