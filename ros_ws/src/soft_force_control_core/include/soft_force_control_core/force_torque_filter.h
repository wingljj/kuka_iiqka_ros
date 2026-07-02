#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

// First-order low-pass filter applied independently to all six channels.
// alpha = dt / (dt + 1/(2*pi*cutoff_hz)). cutoff_hz <= 0 disables filtering.
class ForceTorqueFilter {
 public:
  explicit ForceTorqueFilter(double cutoff_hz) : cutoff_hz_(cutoff_hz) {}
  Wrench filter(const Wrench& in, double dt);
  void reset() { initialized_ = false; }

 private:
  double cutoff_hz_;
  bool initialized_{false};
  Wrench state_;
};

}  // namespace sfc
