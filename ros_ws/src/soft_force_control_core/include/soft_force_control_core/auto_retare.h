#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

struct AutoReTareParams {
  bool enabled{false};
  double orientation_tol_deg{1.0};
};

// Return-to-start re-tare predicate (legacy isReset, spec section 7.5).
// The caller absorbs the residual via ToolGravityCompensator::absorbResidual.
class AutoReTare {
 public:
  void setReference(double a, double b, double c);
  bool shouldTare(const CartesianState& s, const Wrench& compensated,
                  double force_deadband, double torque_deadband,
                  const AutoReTareParams& p) const;

 private:
  double ref_a_{0}, ref_b_{0}, ref_c_{0};
};

}  // namespace sfc
