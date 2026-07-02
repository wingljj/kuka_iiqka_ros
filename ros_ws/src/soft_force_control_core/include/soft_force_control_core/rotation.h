#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace sfc {

inline double degToRad(double d) { return d * M_PI / 180.0; }
inline double radToDeg(double r) { return r * 180.0 / M_PI; }

// Wrap angle into [-180, 180).
inline double wrapDeg(double deg) {
  double d = std::fmod(deg + 180.0, 360.0);
  if (d < 0) d += 360.0;
  return d - 180.0;
}

// KUKA A/B/C convention: R = Rz(A) * Ry(B) * Rx(C), angles in degrees.
inline Eigen::Matrix3d kukaAbcToRotation(double a_deg, double b_deg,
                                         double c_deg) {
  return (Eigen::AngleAxisd(degToRad(a_deg), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(degToRad(b_deg), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(degToRad(c_deg), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// Geodesic angle (degrees) between two KUKA A/B/C orientations.
inline double angularDistanceDeg(double a1, double b1, double c1, double a2,
                                 double b2, double c2) {
  const Eigen::Matrix3d r =
      kukaAbcToRotation(a1, b1, c1).transpose() * kukaAbcToRotation(a2, b2, c2);
  const double cos_angle = (r.trace() - 1.0) / 2.0;
  return radToDeg(std::acos(std::max(-1.0, std::min(1.0, cos_angle))));
}

}  // namespace sfc
