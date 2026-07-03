#pragma once

namespace kuka_rsi {

// Secondary hardware-layer RKorr limits (spec section 12.2). Defaults match
// the controller-layer sfc::SafetyParams so the hardware clamp only engages
// if a controller misbehaves.
struct CommandLimits {
  double max_trans{0.5};  // mm per cycle, axes 0-2
  double max_rot{0.05};   // deg per cycle, axes 3-5
};

// Final in-place clamp before serialization. Non-finite values become zero.
// Returns true if any axis was modified. Allocation-free.
class CommandLimiter {
 public:
  bool clamp(double corr[6], const CommandLimits& limits) const;
};

}  // namespace kuka_rsi
