#include <gtest/gtest.h>

#include "soft_robot_controllers/force_compliance_core.h"

using soft_robot_controllers::ComplianceInput;
using soft_robot_controllers::ComplianceOutput;
using soft_robot_controllers::ForceComplianceCore;
using soft_robot_controllers::ForceComplianceParams;
using soft_robot_controllers::ToolFrameConfig;

namespace {

constexpr double kDt = 0.004;

// 与 test_force_compliance_core.cpp baseParams 相同的简单参数:
// 滤波关、增益 1 (mm/s)/N、死区 30 N、限幅放宽到不干扰断言。
ForceComplianceParams simpleParams() {
  ForceComplianceParams p;
  p.filter_cutoff_hz = 0.0;
  p.compliance.translation.gain = 1.0;
  p.compliance.translation.max_speed = 50.0;
  p.compliance.translation.max_accel = 0.0;
  p.compliance.rotation.gain = 0.1;
  p.compliance.rotation.max_speed = 5.0;
  p.compliance.rotation.max_accel = 0.0;
  p.compliance.speed_scale = 1.0;
  p.fixed_force_deadband_n = 30.0;
  p.fixed_torque_deadband_nm = 4.0;
  p.adaptive_deadband = false;
  p.retare.enabled = false;
  p.safety.max_corr_trans = 0.5;
  p.safety.max_corr_rot = 0.05;
  p.safety.force_ceiling = 500.0;
  p.safety.torque_ceiling = 50.0;
  p.wrench_timeout_s = 0.012;
  return p;
}

ComplianceInput freshInput() {
  ComplianceInput in;
  in.wrench_valid = true;
  in.wrench_age_s = 0.0;
  return in;
}

}  // namespace

TEST(FccToolFrame, LegacyActivateEqualsZeroConfig) {
  // 单参 activate 与全零 ToolFrameConfig 输出逐位一致(回归保护)。
  ForceComplianceCore legacy;
  legacy.configure(simpleParams());
  legacy.activate(sfc::CartesianState{});
  ForceComplianceCore zeroed;
  zeroed.configure(simpleParams());
  zeroed.activate(sfc::CartesianState{}, ToolFrameConfig{});
  ComplianceInput in = freshInput();
  in.raw.fz = 40.0;  // e = 10 N -> 0.04 mm
  const ComplianceOutput a = legacy.update(in, kDt);
  const ComplianceOutput b = zeroed.update(in, kDt);
  EXPECT_DOUBLE_EQ(a.correction.z, b.correction.z);
  EXPECT_NEAR(a.correction.z, 0.04, 1e-12);
}

TEST(FccToolFrame, ToolRotZ90RemapsForceToBase) {
  // $TOOL A=90,机器人姿态恒等(R_bt = I, TCP 与 BASE 对齐)。
  // 传感器 +X 40N:sensor->tool = Rz(-90) 得工具 -Y 40N;
  // 死区 30 -> e=10N -> 工具系 y=-0.04mm;tool->base(R_bt=I)不变。
  // 若没做工具系变换,输出会是 x=+0.04 —— 断言能区分新旧行为。
  ForceComplianceCore core;
  core.configure(simpleParams());
  ToolFrameConfig tf;
  tf.tool_a = 90.0;
  core.activate(sfc::CartesianState{}, tf);
  ComplianceInput in = freshInput();
  in.raw.fx = 40.0;
  const ComplianceOutput out = core.update(in, kDt);
  EXPECT_NEAR(out.correction.x, 0.0, 1e-12);
  EXPECT_NEAR(out.correction.y, -0.04, 1e-12);
}

TEST(FccToolFrame, CorrectionRotatedBackToBaseByRobotPose) {
  // 工具/安装恒等,但机器人 A=90(R_bt = Rz(90))。
  // 传感器系 +X 40N(即 TCP 系 +X)-> 工具系 x=0.04mm
  // -> BASE: Rz(90)*(0.04,0,0) = (0, 0.04, 0)。
  // 旧代码会直接输出 x=0.04(这正是被修的 bug)。
  ForceComplianceCore core;
  core.configure(simpleParams());
  core.activate(sfc::CartesianState{}, ToolFrameConfig{});
  ComplianceInput in = freshInput();
  in.state.a = 90.0;
  in.raw.fx = 40.0;
  const ComplianceOutput out = core.update(in, kDt);
  EXPECT_NEAR(out.correction.x, 0.0, 1e-12);
  EXPECT_NEAR(out.correction.y, 0.04, 1e-12);
}

TEST(FccToolFrame, GravityCompensatedInSensorFrameUnderRotation) {
  // G=10N 已标定;机器人 C=90(R_bt = Rx(90)),工具/安装恒等
  // => r_base_sensor = Rx(90)。传感器静载读数 = 重力投影
  // Rx(90)^T*(0,0,-10):补偿后应为零 -> 输出零(在死区内)。
  // 旧实现同样用 in.state 角旋转,此用例恒等工具下新旧一致;
  // 它锁定的是"补偿用 r_base_sensor 而非裸 TCP 角"的实现路径,
  // 与 MountOffsetChangesGravityFrame 联合构成传感器系语义。
  ForceComplianceCore core;
  ForceComplianceParams p = simpleParams();
  p.payload.gravity_n = 10.0;
  core.configure(p);
  core.activate(sfc::CartesianState{}, ToolFrameConfig{});
  ComplianceInput in = freshInput();
  in.state.c = 90.0;
  // Rx(90)^T * (0,0,-10) = (0, -10, 0)(KUKA C 绕 X)
  in.raw.fy = -10.0;
  const ComplianceOutput out = core.update(in, kDt);
  EXPECT_NEAR(out.compensated.fx, 0.0, 1e-9);
  EXPECT_NEAR(out.compensated.fy, 0.0, 1e-9);
  EXPECT_NEAR(out.compensated.fz, 0.0, 1e-9);
  EXPECT_EQ(out.correction.y, 0.0);
}

TEST(FccToolFrame, MountOffsetChangesGravityFrame) {
  // 安装偏差 mount C=90(传感器相对法兰绕 X 转 90),机器人恒等姿态,
  // 工具恒等 => R_tcp_sensor = Rx(90), r_base_sensor = Rx(90)。
  // 传感器系静载读数应为 Rx(90)^T*(0,0,-10) = (0,-10,0);
  // 旧实现(用 TCP 角=恒等)会错误地期望 (0,0,-10)。
  ForceComplianceCore core;
  ForceComplianceParams p = simpleParams();
  p.payload.gravity_n = 10.0;
  core.configure(p);
  ToolFrameConfig tf;
  tf.mount_c = 90.0;
  core.activate(sfc::CartesianState{}, tf);
  ComplianceInput in = freshInput();
  in.raw.fy = -10.0;
  const ComplianceOutput out = core.update(in, kDt);
  EXPECT_NEAR(out.compensated.forceNorm(), 0.0, 1e-9);
}
