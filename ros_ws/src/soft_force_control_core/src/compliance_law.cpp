#include "soft_force_control_core/compliance_law.h"

#include <algorithm>
#include <cmath>

namespace sfc {

namespace {
double deadzone(double f, double db) {
  if (std::fabs(f) <= db) return 0.0;
  return f > 0 ? f - db : f + db;
}
double clamp(double v, double lim) { return std::max(-lim, std::min(lim, v)); }
}  // namespace

double ComplianceLaw::axisVelocity(double f, const AxisGroupParams& g,
                                   double scale, double dt,
                                   double& prev_v) const {
  double v = clamp(g.gain * deadzone(f, g.deadband) * scale, g.max_speed);
  if (g.max_accel > 0) {
    const double dv = g.max_accel * dt;
    v = std::max(prev_v - dv, std::min(prev_v + dv, v));
  }
  prev_v = v;
  return v;
}

CartesianCorrection ComplianceLaw::compute(const Wrench& w,
                                           const ComplianceParams& p,
                                           double dt) {
  CartesianCorrection c;
  c.x = axisVelocity(w.fx, p.translation, p.speed_scale, dt, prev_v_[0]) * dt;
  c.y = axisVelocity(w.fy, p.translation, p.speed_scale, dt, prev_v_[1]) * dt;
  c.z = axisVelocity(w.fz, p.translation, p.speed_scale, dt, prev_v_[2]) * dt;
  c.a = axisVelocity(w.tz, p.rotation, p.speed_scale, dt, prev_v_[3]) * dt;
  c.b = axisVelocity(w.ty, p.rotation, p.speed_scale, dt, prev_v_[4]) * dt;
  c.c = axisVelocity(w.tx, p.rotation, p.speed_scale, dt, prev_v_[5]) * dt;
  return c;
}

void ComplianceLaw::reset() {
  for (double& v : prev_v_) v = 0.0;
}

}  // namespace sfc
