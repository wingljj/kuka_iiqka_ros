#include "soft_force_control_core/auto_retare.h"

#include "soft_force_control_core/rotation.h"

namespace sfc {

void AutoReTare::setReference(double a, double b, double c) {
  ref_a_ = a;
  ref_b_ = b;
  ref_c_ = c;
}

bool AutoReTare::shouldTare(const CartesianState& s, const Wrench& w,
                            double force_deadband, double torque_deadband,
                            const AutoReTareParams& p) const {
  if (!p.enabled) return false;
  if (angularDistanceDeg(ref_a_, ref_b_, ref_c_, s.a, s.b, s.c) >
      p.orientation_tol_deg) {
    return false;
  }
  return w.forceNorm() < force_deadband && w.torqueNorm() < torque_deadband;
}

}  // namespace sfc
