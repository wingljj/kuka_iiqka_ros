#include "soft_force_control_core/safety_limiter.h"

#include <algorithm>

namespace sfc {

namespace {
double clampFlag(double v, double lim, bool& saturated) {
  const double c = std::max(-lim, std::min(lim, v));
  if (c != v) saturated = true;
  return c;
}
}  // namespace

SafetyResult SafetyLimiter::apply(const CartesianCorrection& in,
                                  const Wrench& w,
                                  const SafetyParams& p) const {
  SafetyResult r;
  if (w.forceNorm() > p.force_ceiling || w.torqueNorm() > p.torque_ceiling) {
    r.hard_cutoff = true;
    return r;  // correction stays zero-initialized
  }
  r.correction.x = clampFlag(in.x, p.max_corr_trans, r.saturated);
  r.correction.y = clampFlag(in.y, p.max_corr_trans, r.saturated);
  r.correction.z = clampFlag(in.z, p.max_corr_trans, r.saturated);
  r.correction.a = clampFlag(in.a, p.max_corr_rot, r.saturated);
  r.correction.b = clampFlag(in.b, p.max_corr_rot, r.saturated);
  r.correction.c = clampFlag(in.c, p.max_corr_rot, r.saturated);
  return r;
}

}  // namespace sfc
