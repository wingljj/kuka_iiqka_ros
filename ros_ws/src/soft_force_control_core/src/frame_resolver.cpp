#include "soft_force_control_core/frame_resolver.h"

#include "soft_force_control_core/rotation.h"

namespace sfc {

void FrameResolver::configure(double tool_a_deg, double tool_b_deg,
                              double tool_c_deg, double mount_a_deg,
                              double mount_b_deg, double mount_c_deg) {
  const Eigen::Matrix3d r_ftool =
      kukaAbcToRotation(tool_a_deg, tool_b_deg, tool_c_deg);
  const Eigen::Matrix3d r_mount =
      kukaAbcToRotation(mount_a_deg, mount_b_deg, mount_c_deg);
  r_tcp_sensor_ = r_ftool.transpose() * r_mount;
}

Wrench FrameResolver::wrenchSensorToTool(const Wrench& w) const {
  const Eigen::Vector3d f =
      r_tcp_sensor_ * Eigen::Vector3d(w.fx, w.fy, w.fz);
  const Eigen::Vector3d t =
      r_tcp_sensor_ * Eigen::Vector3d(w.tx, w.ty, w.tz);
  Wrench out;
  out.fx = f.x();
  out.fy = f.y();
  out.fz = f.z();
  out.tx = t.x();
  out.ty = t.y();
  out.tz = t.z();
  return out;
}

CartesianCorrection FrameResolver::correctionToolToBase(
    const CartesianCorrection& c, const Eigen::Matrix3d& r_bt) const {
  const Eigen::Vector3d p = r_bt * Eigen::Vector3d(c.x, c.y, c.z);
  // Rotation vector: (wx, wy, wz) = (c, b, a) -- A/B/C about Z/Y/X.
  const Eigen::Vector3d w = r_bt * Eigen::Vector3d(c.c, c.b, c.a);
  CartesianCorrection out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  out.c = w.x();
  out.b = w.y();
  out.a = w.z();
  return out;
}

}  // namespace sfc
