#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "soft_force_control_core/rotation.h"
#include "soft_force_control_core/tool_gravity_compensator.h"

using sfc::PayloadParams;
using sfc::ToolGravityCompensator;
using sfc::Wrench;

TEST(GravityComp, ZeroParamsPassesRawThrough) {
  ToolGravityCompensator g;
  Wrench raw;
  raw.fx = 1.0;
  raw.tz = 2.0;
  Wrench out = g.compensate(raw, 10, 20, 30);
  EXPECT_DOUBLE_EQ(out.fx, 1.0);
  EXPECT_DOUBLE_EQ(out.tz, 2.0);
}

TEST(GravityComp, BiasSubtracted) {
  ToolGravityCompensator g;
  PayloadParams p;
  p.bias.fx = 0.5;
  g.setParams(p);
  Wrench raw;
  raw.fx = 1.5;
  EXPECT_DOUBLE_EQ(g.compensate(raw, 0, 0, 0).fx, 1.0);
}

TEST(GravityComp, GravityCancelledAtIdentityOrientation) {
  // Sensor aligned with base, payload G=10N pulls -Z: raw reads (0,0,-10).
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  g.setParams(p);
  Wrench raw;
  raw.fz = -10.0;
  Wrench out = g.compensate(raw, 0, 0, 0);
  EXPECT_NEAR(out.fx, 0.0, 1e-12);
  EXPECT_NEAR(out.fy, 0.0, 1e-12);
  EXPECT_NEAR(out.fz, 0.0, 1e-12);
}

TEST(GravityComp, GravityRotatesWithOrientation) {
  // C=90 deg (rot about X): base -Z maps to sensor -Y... verify via cancel:
  // whatever the model predicts, feeding exactly that as raw must yield zero.
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  p.com_x = 0.01;
  p.com_y = 0.02;
  p.com_z = 0.03;
  g.setParams(p);
  Wrench zero;
  Wrench predicted = g.compensate(zero, 30, 40, 50);  // = -gravity model
  Wrench raw;
  raw.fx = -predicted.fx;
  raw.fy = -predicted.fy;
  raw.fz = -predicted.fz;
  raw.tx = -predicted.tx;
  raw.ty = -predicted.ty;
  raw.tz = -predicted.tz;
  Wrench out = g.compensate(raw, 30, 40, 50);
  EXPECT_NEAR(out.forceNorm(), 0.0, 1e-12);
  EXPECT_NEAR(out.torqueNorm(), 0.0, 1e-12);
}

TEST(GravityComp, ComProducesTorque) {
  // Identity orientation, G=10N along -Z sensor, CoM offset in +X
  // => torque = com x f = (0.1,0,0) x (0,0,-10) = (0, 1, 0).
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  p.com_x = 0.1;
  g.setParams(p);
  Wrench raw;
  raw.fz = -10.0;
  raw.ty = 1.0;
  Wrench out = g.compensate(raw, 0, 0, 0);
  EXPECT_NEAR(out.torqueNorm(), 0.0, 1e-12);
}

TEST(GravityComp, AbsorbResidualShiftsBias) {
  ToolGravityCompensator g;
  Wrench residual;
  residual.fx = 0.7;
  g.absorbResidual(residual);
  Wrench raw;
  raw.fx = 0.7;
  EXPECT_NEAR(g.compensate(raw, 0, 0, 0).fx, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(g.params().bias.fx, 0.7);
}

TEST(GravityComp, MatrixOverloadMatchesAngleOverload) {
  // 委托关系:两个重载在同一朝向下逐位一致。
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  p.com_x = 0.01;
  p.com_y = 0.02;
  p.com_z = 0.03;
  p.bias.fx = 0.5;
  g.setParams(p);
  Wrench raw;
  raw.fx = 1.0;
  raw.fz = -8.0;
  raw.ty = 0.4;
  const Wrench via_angles = g.compensate(raw, 30, 40, 50);
  const Wrench via_matrix =
      g.compensate(raw, sfc::kukaAbcToRotation(30, 40, 50));
  EXPECT_DOUBLE_EQ(via_matrix.fx, via_angles.fx);
  EXPECT_DOUBLE_EQ(via_matrix.fy, via_angles.fy);
  EXPECT_DOUBLE_EQ(via_matrix.fz, via_angles.fz);
  EXPECT_DOUBLE_EQ(via_matrix.tx, via_angles.tx);
  EXPECT_DOUBLE_EQ(via_matrix.ty, via_angles.ty);
  EXPECT_DOUBLE_EQ(via_matrix.tz, via_angles.tz);
}

TEST(GravityComp, SensorFrameGravityWithMountRotation) {
  // 传感器绕 X 转 90°(r_base_sensor = Rx(90)):BASE -Z 重力在传感器系
  // 读作 +Y 方向 -G(Rx(90)^T * (0,0,-G) = (0,-G·sin90 -> 精确 (0,-10,0)
  // 之负号按右手系:Rx(90)^T*(0,0,-10) = (0,-10,0))。
  // 静载传感器读数恰为该投影时,补偿后应为零。
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  g.setParams(p);
  const Eigen::Matrix3d r_bs = sfc::kukaAbcToRotation(0, 0, 90);
  const Eigen::Vector3d g_sensor = r_bs.transpose() * Eigen::Vector3d(0, 0, -10.0);
  Wrench raw;
  raw.fx = g_sensor.x();
  raw.fy = g_sensor.y();
  raw.fz = g_sensor.z();
  const Wrench out = g.compensate(raw, r_bs);
  EXPECT_NEAR(out.forceNorm(), 0.0, 1e-12);
}

TEST(GravityComp, UncalibratedDegradesToBiasOnly) {
  // spec 未标定退化:gravity_n == 0 时任意朝向下重力项为零,
  // compensate = raw - bias,朝向变化不引入任何项。
  ToolGravityCompensator g;
  PayloadParams p;
  p.bias.fx = 0.5;
  g.setParams(p);
  Wrench raw;
  raw.fx = 1.5;
  raw.tz = 2.0;
  const Wrench at_id = g.compensate(raw, Eigen::Matrix3d::Identity());
  const Wrench at_rot = g.compensate(raw, sfc::kukaAbcToRotation(30, 40, 50));
  EXPECT_DOUBLE_EQ(at_id.fx, 1.0);
  EXPECT_DOUBLE_EQ(at_id.tz, 2.0);
  EXPECT_DOUBLE_EQ(at_rot.fx, at_id.fx);
  EXPECT_DOUBLE_EQ(at_rot.fy, at_id.fy);
  EXPECT_DOUBLE_EQ(at_rot.fz, at_id.fz);
  EXPECT_DOUBLE_EQ(at_rot.tx, at_id.tx);
  EXPECT_DOUBLE_EQ(at_rot.ty, at_id.ty);
  EXPECT_DOUBLE_EQ(at_rot.tz, at_id.tz);
}
