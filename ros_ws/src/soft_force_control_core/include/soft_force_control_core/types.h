#pragma once
#include <cmath>

namespace sfc {

// KUKA Cartesian pose: mm for x/y/z, deg for a/b/c (A=rotZ, B=rotY, C=rotX).
struct CartesianState {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};
};

// Force/torque sample: N for forces, Nm for torques.
struct Wrench {
  double fx{0}, fy{0}, fz{0}, tx{0}, ty{0}, tz{0};
  double forceNorm() const { return std::sqrt(fx * fx + fy * fy + fz * fz); }
  double torqueNorm() const { return std::sqrt(tx * tx + ty * ty + tz * tz); }
};

// Per-cycle RSI Cartesian correction (RKorr): mm and deg per cycle.
struct CartesianCorrection {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};
};

enum class ControlMode { IDLE, DIRECT_CARTESIAN, FORCE_COMPLIANCE, CALIBRATION };
enum class Profile { DRAG, PRECISION };

}  // namespace sfc
