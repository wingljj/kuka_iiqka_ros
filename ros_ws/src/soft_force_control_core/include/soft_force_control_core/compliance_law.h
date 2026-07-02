#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

// Parameters for one axis group (translation or rotation).
struct AxisGroupParams {
  double gain{0};       // (mm/s)/N for translation, (deg/s)/Nm for rotation
  double deadband{0};   // N or Nm
  double max_speed{0};  // mm/s or deg/s
  double max_accel{0};  // mm/s^2 or deg/s^2; <= 0 disables rate limiting
};

struct ComplianceParams {
  AxisGroupParams translation;
  AxisGroupParams rotation;
  double speed_scale{1.0};
};

// Per-axis admittance law (spec section 7.2):
//   e = deadzone(F, db); v = clamp(gain*e*scale, max_speed);
//   v = rate_limit(v);   correction = v * dt.
// Axis mapping: fx->x, fy->y, fz->z, tx->c, ty->b, tz->a
// (KUKA A/B/C rotate about Z/Y/X).
class ComplianceLaw {
 public:
  CartesianCorrection compute(const Wrench& compensated,
                              const ComplianceParams& p, double dt);
  void reset();

 private:
  double axisVelocity(double f, const AxisGroupParams& g, double scale,
                      double dt, double& prev_v) const;
  double prev_v_[6] = {0, 0, 0, 0, 0, 0};  // x,y,z,a,b,c
};

}  // namespace sfc
