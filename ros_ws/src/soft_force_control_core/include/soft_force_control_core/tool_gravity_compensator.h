#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

// Payload/bias parameters produced by calibration (spec section 9).
struct PayloadParams {
  double gravity_n{0};                      // payload weight G [N]
  double com_x{0}, com_y{0}, com_z{0};      // center of mass [m], sensor frame
  Wrench bias;                              // sensor zero bias
};

// Subtracts sensor bias and tool gravity (force + CoM torque) from a raw
// wrench. Sensor frame is assumed aligned with the flange orientation
// given by the robot A/B/C angles (legacy FTCompensation behavior).
class ToolGravityCompensator {
 public:
  void setParams(const PayloadParams& p) { params_ = p; }
  const PayloadParams& params() const { return params_; }
  Wrench compensate(const Wrench& raw, double a_deg, double b_deg,
                    double c_deg) const;
  // Auto re-tare support: fold a measured residual into the bias.
  void absorbResidual(const Wrench& residual);

 private:
  PayloadParams params_;
};

}  // namespace sfc
