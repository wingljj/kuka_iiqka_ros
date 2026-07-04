#include <gtest/gtest.h>

#include "soft_force_control_core/frame_resolver.h"
#include "soft_force_control_core/rotation.h"

using sfc::CartesianCorrection;
using sfc::FrameResolver;
using sfc::Wrench;

TEST(FrameResolver, IdentityByDefault) {
  // 未 configure(或全零 configure)= 恒等,现状等价。
  FrameResolver f;
  Wrench w;
  w.fx = 1.0;
  w.ty = 2.0;
  const Wrench out = f.wrenchSensorToTool(w);
  EXPECT_DOUBLE_EQ(out.fx, 1.0);
  EXPECT_DOUBLE_EQ(out.ty, 2.0);
  CartesianCorrection c;
  c.x = 0.5;
  c.a = 0.01;
  const CartesianCorrection cb =
      f.correctionToolToBase(c, Eigen::Matrix3d::Identity());
  EXPECT_DOUBLE_EQ(cb.x, 0.5);
  EXPECT_DOUBLE_EQ(cb.a, 0.01);
}

TEST(FrameResolver, ToolRotZ90RemapsWrench) {
  // $TOOL A=90(TCP 相对法兰绕 Z 转 90°),传感器与法兰对齐。
  // R_tcp_sensor = Rz(90)⁻¹ = Rz(-90):传感器 +X 力 → 工具 -Y。
  FrameResolver f;
  f.configure(90.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Wrench w;
  w.fx = 10.0;
  const Wrench out = f.wrenchSensorToTool(w);
  EXPECT_NEAR(out.fx, 0.0, 1e-12);
  EXPECT_NEAR(out.fy, -10.0, 1e-12);
  EXPECT_NEAR(out.fz, 0.0, 1e-12);
}

TEST(FrameResolver, MountOffsetComposes) {
  // 工具恒等、安装绕 Z 转 90:R_tcp_sensor = R_mount = Rz(90)。
  // 传感器 +X → 法兰/工具 +Y。力矩同样旋转。
  FrameResolver f;
  f.configure(0.0, 0.0, 0.0, 90.0, 0.0, 0.0);
  Wrench w;
  w.fx = 3.0;
  w.tx = 1.0;
  const Wrench out = f.wrenchSensorToTool(w);
  EXPECT_NEAR(out.fy, 3.0, 1e-12);
  EXPECT_NEAR(out.ty, 1.0, 1e-12);
  EXPECT_NEAR(out.fx, 0.0, 1e-12);
  EXPECT_NEAR(out.tx, 0.0, 1e-12);
}

TEST(FrameResolver, CorrectionRotatesTranslationAndRotationVector) {
  // r_bt = Rz(90):工具系 +X 平移 → BASE +Y;
  // 工具系绕 X 的角增量(c 分量)→ BASE 绕 Y(b 分量)。
  FrameResolver f;  // R_tcp_sensor 恒等,不影响本方法
  CartesianCorrection c_tool;
  c_tool.x = 1.0;
  c_tool.c = 0.02;  // ωx
  const Eigen::Matrix3d r_bt = sfc::kukaAbcToRotation(90.0, 0.0, 0.0);
  const CartesianCorrection cb = f.correctionToolToBase(c_tool, r_bt);
  EXPECT_NEAR(cb.x, 0.0, 1e-12);
  EXPECT_NEAR(cb.y, 1.0, 1e-12);
  EXPECT_NEAR(cb.c, 0.0, 1e-12);
  EXPECT_NEAR(cb.b, 0.02, 1e-12);  // ωx → ωy
  EXPECT_NEAR(cb.a, 0.0, 1e-12);
}

TEST(FrameResolver, RoundTripSensorToolBaseIdentity) {
  // 恒等工具/安装 + r_bt 任意:sensor→tool 是恒等,tool→base 用 r_bt,
  // 再用 r_bt⁻¹ 旋回应还原(数值往返一致性)。
  FrameResolver f;
  f.configure(0, 0, 0, 0, 0, 0);
  CartesianCorrection c;
  c.x = 0.1; c.y = -0.2; c.z = 0.3; c.a = 0.01; c.b = -0.02; c.c = 0.03;
  const Eigen::Matrix3d r_bt = sfc::kukaAbcToRotation(30.0, 40.0, 50.0);
  const CartesianCorrection fwd = f.correctionToolToBase(c, r_bt);
  const CartesianCorrection back =
      f.correctionToolToBase(fwd, r_bt.transpose());
  EXPECT_NEAR(back.x, c.x, 1e-12);
  EXPECT_NEAR(back.y, c.y, 1e-12);
  EXPECT_NEAR(back.z, c.z, 1e-12);
  EXPECT_NEAR(back.a, c.a, 1e-12);
  EXPECT_NEAR(back.b, c.b, 1e-12);
  EXPECT_NEAR(back.c, c.c, 1e-12);
}
