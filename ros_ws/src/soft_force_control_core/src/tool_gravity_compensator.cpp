#include "soft_force_control_core/tool_gravity_compensator.h"

#include <Eigen/Dense>

#include "soft_force_control_core/rotation.h"

namespace sfc {

Wrench ToolGravityCompensator::compensate(
    const Wrench& raw, const Eigen::Matrix3d& r_base_sensor) const {
  const Eigen::Vector3d f_g =
      r_base_sensor.transpose() * Eigen::Vector3d(0, 0, -params_.gravity_n);
  const Eigen::Vector3d com(params_.com_x, params_.com_y, params_.com_z);
  const Eigen::Vector3d t_g = com.cross(f_g);

  Wrench out;
  out.fx = raw.fx - params_.bias.fx - f_g.x();
  out.fy = raw.fy - params_.bias.fy - f_g.y();
  out.fz = raw.fz - params_.bias.fz - f_g.z();
  out.tx = raw.tx - params_.bias.tx - t_g.x();
  out.ty = raw.ty - params_.bias.ty - t_g.y();
  out.tz = raw.tz - params_.bias.tz - t_g.z();
  return out;
}

Wrench ToolGravityCompensator::compensate(const Wrench& raw, double a_deg,
                                          double b_deg, double c_deg) const {
  return compensate(raw, kukaAbcToRotation(a_deg, b_deg, c_deg));
}

void ToolGravityCompensator::absorbResidual(const Wrench& residual) {
  params_.bias.fx += residual.fx;
  params_.bias.fy += residual.fy;
  params_.bias.fz += residual.fz;
  params_.bias.tx += residual.tx;
  params_.bias.ty += residual.ty;
  params_.bias.tz += residual.tz;
}

}  // namespace sfc
