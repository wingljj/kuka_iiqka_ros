#pragma once
#include <cstddef>
#include <vector>

#include "soft_force_control_core/tool_gravity_compensator.h"
#include "soft_force_control_core/types.h"

namespace sfc {

struct PayloadFitResult {
  bool ok{false};
  PayloadParams params;
  double r2_force{0};   // coefficient of determination, force fit
  double r2_torque{0};  // coefficient of determination, torque fit
};

// Least-squares payload identification from averaged wrench samples at
// varied orientations (legacy MassCul, spec section 9). Non-realtime.
class PayloadEstimator {
 public:
  void addSample(double a_deg, double b_deg, double c_deg,
                 const Wrench& averaged);
  std::size_t sampleCount() const { return samples_.size(); }
  void clear() { samples_.clear(); }
  PayloadFitResult solve() const;

 private:
  struct Sample {
    double a, b, c;
    Wrench w;
  };
  std::vector<Sample> samples_;
};

}  // namespace sfc
