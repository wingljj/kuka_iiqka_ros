#pragma once
#include <Eigen/Dense>

#include "soft_force_control_core/types.h"

namespace sfc {

// Session-constant frame chain for FORCE_COMPLIANCE (tool-frame design):
//   R_tcp_sensor = R_ftool^-1 * R_mount
// where R_ftool = kukaAbcToRotation($TOOL A/B/C)   (FLANGE -> TCP)
//       R_mount = kukaAbcToRotation(mount A/B/C)   (FLANGE -> SENSOR)
// configure() runs on activate (non-RT edge); the per-cycle methods are
// constant-matrix rotations only: no allocation, no trig.
class FrameResolver {
 public:
  void configure(double tool_a_deg, double tool_b_deg, double tool_c_deg,
                 double mount_a_deg, double mount_b_deg, double mount_c_deg);

  // w_tool = R_tcp_sensor * w_sensor (force and torque rotated alike).
  Wrench wrenchSensorToTool(const Wrench& w_sensor) const;

  // Translation rotated directly; angles treated as a rotation vector
  // (a<->wz, b<->wy, c<->wx, matching KUKA A/B/C about Z/Y/X) rotated by
  // r_bt (TCP -> BASE from the current RIst pose). Small-angle: per-cycle
  // increments at 250 Hz are << 1 deg.
  CartesianCorrection correctionToolToBase(const CartesianCorrection& c_tool,
                                           const Eigen::Matrix3d& r_bt) const;

  const Eigen::Matrix3d& tcpFromSensor() const { return r_tcp_sensor_; }

 private:
  Eigen::Matrix3d r_tcp_sensor_{Eigen::Matrix3d::Identity()};
};

}  // namespace sfc
