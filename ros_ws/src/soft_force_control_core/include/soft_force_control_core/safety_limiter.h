#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

struct SafetyParams {
  double max_corr_trans{0.5};   // mm per cycle per axis
  double max_corr_rot{0.05};    // deg per cycle per axis
  double force_ceiling{500.0};  // N; legacy hard cutoff (spec section 12.1)
  double torque_ceiling{50.0};  // Nm
};

struct SafetyResult {
  CartesianCorrection correction;
  bool hard_cutoff{false};  // wrench exceeded ceiling -> zero output
  bool saturated{false};    // at least one axis was clamped
};

class SafetyLimiter {
 public:
  SafetyResult apply(const CartesianCorrection& in, const Wrench& compensated,
                     const SafetyParams& p) const;
};

}  // namespace sfc
