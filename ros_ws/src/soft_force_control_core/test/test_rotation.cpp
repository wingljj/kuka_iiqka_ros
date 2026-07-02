#include <gtest/gtest.h>
#include "soft_force_control_core/rotation.h"

using sfc::angularDistanceDeg;
using sfc::kukaAbcToRotation;
using sfc::wrapDeg;

TEST(Rotation, IdentityForZeroAngles) {
  Eigen::Matrix3d r = kukaAbcToRotation(0, 0, 0);
  EXPECT_TRUE(r.isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(Rotation, AIsRotationAboutZ) {
  // A=90 deg: base x-axis maps to y-axis.
  Eigen::Vector3d v = kukaAbcToRotation(90, 0, 0) * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(v.x(), 0.0, 1e-12);
  EXPECT_NEAR(v.y(), 1.0, 1e-12);
}

TEST(Rotation, CIsAppliedFirst) {
  // R = Rz(A)*Ry(B)*Rx(C): with C=90, z-axis maps to -y before A/B.
  Eigen::Vector3d v = kukaAbcToRotation(0, 0, 90) * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(v.y(), -1.0, 1e-12);
}

TEST(Rotation, WrapDeg) {
  EXPECT_DOUBLE_EQ(wrapDeg(190.0), -170.0);
  EXPECT_DOUBLE_EQ(wrapDeg(-190.0), 170.0);
  EXPECT_DOUBLE_EQ(wrapDeg(0.0), 0.0);
}

TEST(Rotation, AngularDistanceZeroForSame) {
  EXPECT_NEAR(angularDistanceDeg(10, 20, 30, 10, 20, 30), 0.0, 1e-9);
}

TEST(Rotation, AngularDistanceKnownRotation) {
  EXPECT_NEAR(angularDistanceDeg(0, 0, 0, 90, 0, 0), 90.0, 1e-9);
}
