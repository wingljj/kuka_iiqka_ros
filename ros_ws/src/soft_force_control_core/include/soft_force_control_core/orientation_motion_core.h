#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

struct MotionGoal {
  double a{0}, b{0}, c{0};      // target orientation [deg]
  double max_speed_dps{5.0};    // per-axis speed clamp [deg/s]
  double p_gain{1.0};           // proportional gain [1/s]
  double tol_deg{0.1};          // convergence tolerance per axis
  double hold_s{0.2};           // time inside tolerance before CONVERGED
  double timeout_s{30.0};       // abort deadline
};

enum class MotionStatus { INACTIVE, RUNNING, CONVERGED, TIMEOUT };

// Goal-seeking orientation correction generator (legacy RotFixAng,
// spec section 7.6). P-with-clamp speed shaping; outputs a/b/c corrections
// per cycle, zero after convergence, timeout, or cancel.
class OrientationMotionCore {
 public:
  void setGoal(const MotionGoal& g);
  void cancel() { status_ = MotionStatus::INACTIVE; }
  MotionStatus status() const { return status_; }
  CartesianCorrection update(const CartesianState& current, double dt);

 private:
  MotionGoal goal_;
  MotionStatus status_{MotionStatus::INACTIVE};
  double elapsed_{0}, held_{0};
};

}  // namespace sfc
