#include "kuka_rsi_hw_interface/command_limiter.h"

#include <algorithm>
#include <cmath>

namespace kuka_rsi {

namespace {
bool clampAxis(double& v, double lim) {
  if (!std::isfinite(v)) {
    v = 0.0;
    return true;
  }
  const double c = std::max(-lim, std::min(lim, v));
  if (c != v) {
    v = c;
    return true;
  }
  return false;
}
}  // namespace

bool CommandLimiter::clamp(double corr[6], const CommandLimits& limits) const {
  bool modified = false;
  for (int i = 0; i < 3; ++i) modified |= clampAxis(corr[i], limits.max_trans);
  for (int i = 3; i < 6; ++i) modified |= clampAxis(corr[i], limits.max_rot);
  return modified;
}

}  // namespace kuka_rsi
