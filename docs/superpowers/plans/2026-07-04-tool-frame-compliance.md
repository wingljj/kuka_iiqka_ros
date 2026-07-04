# 力顺应控制器工具坐标系变换 — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把力顺应管线改为:传感器系扣重力 → 力旋转到工具系 → 工具系顺应律 → 修正量转回 BASE 系下发 RSI;同时修复重力补偿坐标系缺陷,并为未标定/无工具数据提供退化路径。

**Architecture:** 新增纯逻辑组件 `FrameResolver`(持有会话内恒定的 SENSOR→TCP 旋转,提供 wrench sensor→tool 与 correction tool→base 两个变换),`ToolGravityCompensator` 增加矩阵重载在传感器系扣重力,`ForceComplianceCore` 在管线中插入两次变换,控制器壳订阅 `/kuka/eki/state` 在 activate 时锁存工具角。RSI 侧保持 BASE(`#ABSOLUTE_OFF`)不动。

**Tech Stack:** ROS1 Noetic catkin、C++14、Eigen3、gtest(`catkin_add_gtest`)。

**Spec:** `docs/superpowers/specs/2026-07-04-tool-frame-compliance-design.md`(已批准)

## Global Constraints

- 纯逻辑组件(`soft_force_control_core`、`ForceComplianceCore`)禁止 ROS 依赖、禁止堆分配、禁止阻塞(RT 路径)。
- KUKA A/B/C 约定:`R = Rz(A)·Ry(B)·Rx(C)`,统一用 `sfc::kukaAbcToRotation`(`rotation.h:18`)。
- `ComplianceLaw` 轴映射(`compliance_law.h:23`):`fx→x, fy→y, fz→z, tx→c, ty→b, tz→a`(A/B/C 分别绕 Z/Y/X)。旋转矢量 ω=(ωx,ωy,ωz) 与修正角的对应:`a↔ωz, b↔ωy, c↔ωx`。
- 每周期新增计算只允许常量矩阵乘 3D 矢量;`R_tcp_sensor` 只在 activate 锁存。
- 现有测试全部保持通过:`test_gravity_compensator` 的角度重载语义不变(旧签名保留,内部委托新矩阵重载)。
- 构建命令(workspace 已有 build/devel):`cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make --pkg <pkg> && catkin_make run_tests_<pkg> -j4`;单个 gtest 直接跑 `devel/lib/<pkg>/<test_binary>`。
- 提交信息用英文,格式 `feat:`/`fix:`/`test:`/`docs:`;每个 Task 至少一次提交。
- 不改:标定流程、`CartesianCorrectionController`、RSI 上下文、`soft_robot_msgs`(诊断标记复用现有 DEGRADED 语义之外的方式——见 Task 5,不新增 msg 字段,控制器用 ROS_WARN_ONCE 日志提示)。

## 现状关键事实(实施者必读)

1. 管线(`force_compliance_core.cpp:38-110`):filter → compensate(用 RIst 的 TCP A/B/C,**这是 bug,应为传感器系**)→ ramp → retare → ComplianceLaw → 与 motion_ 求和 → SafetyLimiter → 输出。输出被当作 BASE 系 RKorr(`ROS_RSI_CONTEXT.notes.md:11` POSCORR BASE)。
2. `in.state`(RIst = `$POS_ACT`)是 **TCP 在 BASE 系**的位姿。`R_bt = kukaAbcToRotation(state.a, state.b, state.c)` 即 TCP→BASE 旋转。
3. `out.compensated` 被 4 个消费者使用:AdaptiveDeadband(只用 norm,系无关)、AutoReTare(只用 norm,系无关)、absorbResidual(bias 在传感器系——补偿后残差必须还在传感器系)、SafetyLimiter 硬截断(只用 norm,系无关)。**结论:`out.compensated` 保持传感器系不变,只在进 ComplianceLaw 前做一次旋转到工具系的局部拷贝。**
4. `motion_`(OrientationMotionCore)输出的是 BASE/TCP 角度域修正(goal-seeking,基于 state 的 A/B/C),与工具系无关 → tool→base 变换只作用于 compliance 分量,在与 motion 求和**之前**完成。
5. EKI `tool_a/b/c`(`EkiState.msg`)镜像最后一次 RobotState 心跳,RSI-active 期间心跳暂停但 `last_state.tool` 保留(`eki_bridge_node.cpp:157-160`)→ 控制器侧"工具数据有效"定义为**曾经收到过**(connected 且至少一帧 state),不要求 state_fresh。
6. 现有测试中所有对 `out.correction.*` 的断言都在 `state.a/b/c = 0`(恒等 R_bt)下,tool/mount 默认恒等时新管线在这些用例中输出不变 → 回归安全(已逐一核对 `test_force_compliance_core.cpp`、`test_fcc_adaptive_retare.cpp`、`test_fcc_orientation_sum.cpp`、`test_force_compliance_controller.cpp`)。

## File Structure

| 文件 | 动作 | 职责 |
|------|------|------|
| `ros_ws/src/soft_force_control_core/include/soft_force_control_core/frame_resolver.h` | Create | SENSOR→TCP 恒定旋转持有者;wrench sensor→tool、correction tool→base 两个纯变换 |
| `ros_ws/src/soft_force_control_core/src/frame_resolver.cpp` | Create | 上述实现(Eigen only) |
| `ros_ws/src/soft_force_control_core/test/test_frame_resolver.cpp` | Create | FrameResolver 单测 |
| `ros_ws/src/soft_force_control_core/include/soft_force_control_core/tool_gravity_compensator.h` | Modify | 新增矩阵重载 `compensate(raw, R_base_sensor)`;旧角度重载保留并委托 |
| `ros_ws/src/soft_force_control_core/src/tool_gravity_compensator.cpp` | Modify | 同上 |
| `ros_ws/src/soft_force_control_core/test/test_gravity_compensator.cpp` | Modify | 新增矩阵重载测试(含未标定退化) |
| `ros_ws/src/soft_force_control_core/CMakeLists.txt` | Modify | 挂 frame_resolver.cpp + 新 gtest |
| `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_core.h` | Modify | activate 增加 tool/mount 参数;成员加 FrameResolver |
| `ros_ws/src/soft_robot_controllers/src/force_compliance_core.cpp` | Modify | 管线插入 sensor→tool、tool→base 变换;重力补偿改传感器系 |
| `ros_ws/src/soft_robot_controllers/test/test_fcc_tool_frame.cpp` | Create | 核心层工具系管线测试 |
| `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_controller.h` | Modify | ToolSample buffer、EKI 订阅、sensor_to_flange_rpy 参数 |
| `ros_ws/src/soft_robot_controllers/src/force_compliance_controller.cpp` | Modify | 订阅/参数/activate 锁存/退化日志 |
| `ros_ws/src/soft_robot_controllers/test/test_force_compliance_controller.cpp` | Modify | 壳层注入工具数据测试 |
| `ros_ws/src/soft_robot_controllers/CMakeLists.txt` | Modify | 挂新 gtest |
| `ros_ws/src/soft_robot_bringup/config/controllers.yaml`(若存在 FCC 段) | Modify | 注释示例 `sensor_to_flange_rpy` |

---

### Task 1: `FrameResolver` 纯变换组件

**Files:**
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/frame_resolver.h`
- Create: `ros_ws/src/soft_force_control_core/src/frame_resolver.cpp`
- Test: `ros_ws/src/soft_force_control_core/test/test_frame_resolver.cpp`
- Modify: `ros_ws/src/soft_force_control_core/CMakeLists.txt`

**Interfaces:**
- Consumes: `sfc::Wrench`、`sfc::CartesianCorrection`(`types.h`)、`sfc::kukaAbcToRotation`(`rotation.h`)、Eigen。
- Produces(后续 Task 依赖的精确签名):

```cpp
namespace sfc {
class FrameResolver {
 public:
  // tool_*: $TOOL 的 A/B/C [deg](FLANGE→TCP 旋转);
  // mount_rpy_*: sensor_to_flange 安装旋转 [deg],KUKA A/B/C 约定
  // (FLANGE→SENSOR)。全零 + 全零 = 恒等(现状等价)。
  void configure(double tool_a_deg, double tool_b_deg, double tool_c_deg,
                 double mount_a_deg, double mount_b_deg, double mount_c_deg);
  // 力/力矩各自旋转:w_tool = R_tcp_sensor * w_sensor
  Wrench wrenchSensorToTool(const Wrench& w_sensor) const;
  // 平移直接旋转;角度按旋转矢量(a↔ωz, b↔ωy, c↔ωx)旋转(小角近似)。
  // r_bt = kukaAbcToRotation(state.a, state.b, state.c)(TCP→BASE)。
  CartesianCorrection correctionToolToBase(const CartesianCorrection& c_tool,
                                           const Eigen::Matrix3d& r_bt) const;
  const Eigen::Matrix3d& tcpFromSensor() const { return r_tcp_sensor_; }
 private:
  Eigen::Matrix3d r_tcp_sensor_{Eigen::Matrix3d::Identity()};
};
}  // namespace sfc
```

数学(spec 帧链):`R_ftool = kukaAbcToRotation(tool_abc)`(FLANGE→TCP),`R_mount = kukaAbcToRotation(mount_abc)`(FLANGE→SENSOR),`R_tcp_sensor = R_ftool⁻¹ · R_mount`(SENSOR→TCP)。

- [ ] **Step 1: 写失败测试**

```cpp
// ros_ws/src/soft_force_control_core/test/test_frame_resolver.cpp
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
```

- [ ] **Step 2: 挂 CMake 并跑测试确认编译失败**

`ros_ws/src/soft_force_control_core/CMakeLists.txt`:
- `add_library` 源列表(第 16-25 行)追加一行 `src/frame_resolver.cpp`
- gtest 段(与 `test_payload_estimator` 同格式)追加:

```cmake
  catkin_add_gtest(test_frame_resolver test/test_frame_resolver.cpp)
  target_link_libraries(test_frame_resolver ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make --pkg soft_force_control_core 2>&1 | tail -5`
Expected: FAIL —— `frame_resolver.h: No such file or directory`(或找不到 frame_resolver.cpp)

- [ ] **Step 3: 最小实现**

```cpp
// ros_ws/src/soft_force_control_core/include/soft_force_control_core/frame_resolver.h
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
```

```cpp
// ros_ws/src/soft_force_control_core/src/frame_resolver.cpp
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
```

- [ ] **Step 4: 跑测试确认通过**

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make --pkg soft_force_control_core && catkin_make run_tests_soft_force_control_core -j4 2>&1 | grep -E "test_frame_resolver|Failed|passed" | tail -5`
Expected: `test_frame_resolver` PASS,其余核心测试不变

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_force_control_core/include/soft_force_control_core/frame_resolver.h \
        ros_ws/src/soft_force_control_core/src/frame_resolver.cpp \
        ros_ws/src/soft_force_control_core/test/test_frame_resolver.cpp \
        ros_ws/src/soft_force_control_core/CMakeLists.txt
git commit -m "feat(core): FrameResolver for sensor->tool->base frame chain"
```

---

### Task 2: `ToolGravityCompensator` 矩阵重载(传感器系重力扣除)

**Files:**
- Modify: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/tool_gravity_compensator.h`
- Modify: `ros_ws/src/soft_force_control_core/src/tool_gravity_compensator.cpp`
- Test: `ros_ws/src/soft_force_control_core/test/test_gravity_compensator.cpp`(追加)

**Interfaces:**
- Consumes: `Eigen::Matrix3d`、`sfc::Wrench`、`sfc::PayloadParams`。
- Produces(Task 3 依赖):

```cpp
// 新增重载:r_base_sensor = SENSOR->BASE 旋转。重力在传感器系扣除:
//   g_sensor = r_base_sensor^T * (0, 0, -gravity_n)
//   t_g = com x g_sensor        (com 为传感器系,标定输出)
// gravity_n == 0 时重力项恒为零 -> 退化为 raw - bias(纯置0模式,spec 未标定语义)。
Wrench compensate(const Wrench& raw, const Eigen::Matrix3d& r_base_sensor) const;
// 旧角度重载保留:compensate(raw, a, b, c) 委托为
//   compensate(raw, kukaAbcToRotation(a, b, c))
// 语义与现状逐位相同(旧实现即 r^T * (0,0,-G)),现有 6 个测试不变。
```

- [ ] **Step 1: 追加失败测试**

在 `test_gravity_compensator.cpp` 文件末尾追加(需在文件头部补 `#include <Eigen/Dense>` 与 `#include "soft_force_control_core/rotation.h"`):

```cpp
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
```

- [ ] **Step 2: 跑测试确认编译失败**

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make --pkg soft_force_control_core 2>&1 | grep -E "error|Error" | head -3`
Expected: FAIL —— `no matching function for call to ... compensate(..., const Eigen::Matrix3d&)`

- [ ] **Step 3: 实现矩阵重载**

`tool_gravity_compensator.h`:头部加 `#include <Eigen/Dense>`,类内加声明(注意:头文件目前只 include types.h,加 Eigen include 是必要的):

```cpp
  // Sensor-frame gravity subtraction (tool-frame design): r_base_sensor is
  // the SENSOR->BASE rotation. gravity_n == 0 degrades to raw - bias
  // (zero-only mode, no orientation dependence).
  Wrench compensate(const Wrench& raw,
                    const Eigen::Matrix3d& r_base_sensor) const;
  // Legacy overload: sensor aligned with the frame given by KUKA A/B/C.
  // Delegates to the matrix overload; behavior unchanged.
  Wrench compensate(const Wrench& raw, double a_deg, double b_deg,
                    double c_deg) const;
```

`tool_gravity_compensator.cpp` 全量替换实现:

```cpp
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
```

- [ ] **Step 4: 跑测试确认全部通过(含旧 6 个)**

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make --pkg soft_force_control_core >/dev/null 2>&1 && ./devel/lib/soft_force_control_core/test_gravity_compensator 2>&1 | tail -3`
Expected: `[  PASSED  ] 9 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_force_control_core/include/soft_force_control_core/tool_gravity_compensator.h \
        ros_ws/src/soft_force_control_core/src/tool_gravity_compensator.cpp \
        ros_ws/src/soft_force_control_core/test/test_gravity_compensator.cpp
git commit -m "feat(core): sensor-frame gravity compensation matrix overload"
```

---

### Task 3: `ForceComplianceCore` 管线插入工具系变换

**Files:**
- Modify: `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_core.h`
- Modify: `ros_ws/src/soft_robot_controllers/src/force_compliance_core.cpp`
- Test: `ros_ws/src/soft_robot_controllers/test/test_fcc_tool_frame.cpp`(新建)
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 `sfc::FrameResolver`(`configure`/`wrenchSensorToTool`/`correctionToolToBase`/`tcpFromSensor`)、Task 2 矩阵重载 `compensate(raw, r_base_sensor)`、`sfc::kukaAbcToRotation`。
- Produces(Task 4 依赖):

```cpp
struct ToolFrameConfig {                 // force_compliance_core.h,新增
  double tool_a{0}, tool_b{0}, tool_c{0};    // $TOOL A/B/C [deg]
  double mount_a{0}, mount_b{0}, mount_c{0}; // sensor_to_flange [deg]
};
// activate 新签名(旧单参重载保留,委托 ToolFrameConfig{} 全零 = 现状等价):
void activate(const sfc::CartesianState& start);                       // 保留
void activate(const sfc::CartesianState& start, const ToolFrameConfig& tf);  // 新增
```

**管线变更点(`update()` 内,按现状事实 #3/#4):**

```
每周期开头:  r_bt = kukaAbcToRotation(in.state.a, in.state.b, in.state.c)
            r_base_sensor = r_bt * frame_.tcpFromSensor()
compensate:  compensator_.compensate(filtered, r_base_sensor)   // 传感器系(改)
             out.compensated 保持传感器系(4 个 norm/residual 消费者不动)
ComplianceLaw 之前: w_tool = frame_.wrenchSensorToTool(out.compensated)
             law_.compute(w_tool, law_params, dt)               // 工具系(改)
ComplianceLaw 之后: compliance_base = frame_.correctionToolToBase(compliance, r_bt)
             sum = compliance_base + motion                     // motion 本就 BASE/TCP 角度域
SafetyLimiter: 不变(BASE 系限幅)
```

- [ ] **Step 1: 写失败测试**

```cpp
// ros_ws/src/soft_robot_controllers/test/test_fcc_tool_frame.cpp
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
```

- [ ] **Step 2: 挂 CMake 并确认编译失败**

`ros_ws/src/soft_robot_controllers/CMakeLists.txt` gtest 段(`test_fcc_orientation_sum` 之后)追加:

```cmake
  catkin_add_gtest(test_fcc_tool_frame test/test_fcc_tool_frame.cpp)
  target_link_libraries(test_fcc_tool_frame ${PROJECT_NAME}
                        ${catkin_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make --pkg soft_robot_controllers 2>&1 | grep -E "error" | head -3`
Expected: FAIL —— `ToolFrameConfig was not declared` / `no matching function activate(...)`

- [ ] **Step 3: 实现核心变更**

`force_compliance_core.h` 变更:
1. include 区加 `#include "soft_force_control_core/frame_resolver.h"`
2. `ForceComplianceParams` 之后加:

```cpp
// Tool-frame chain locked at servo activation (spec: session-constant).
// All-zero == identity == legacy behavior.
struct ToolFrameConfig {
  double tool_a{0}, tool_b{0}, tool_c{0};     // $TOOL A/B/C [deg], from EKI
  double mount_a{0}, mount_b{0}, mount_c{0};  // sensor_to_flange rpy [deg]
};
```

3. 类内 public 区,现有 `void activate(const sfc::CartesianState& start);` 改为两个:

```cpp
  // Legacy entry: identity tool frame (kept for existing tests/callers).
  void activate(const sfc::CartesianState& start);
  // Tool-frame entry: locks R_tcp_sensor from $TOOL + mount at activation.
  void activate(const sfc::CartesianState& start, const ToolFrameConfig& tf);
```

4. private 成员加 `sfc::FrameResolver frame_;`

`force_compliance_core.cpp` 变更:
1. include 区加 `#include "soft_force_control_core/rotation.h"`(已有)保持,确认即可。
2. `activate` 实现改为:

```cpp
void ForceComplianceCore::activate(const sfc::CartesianState& start) {
  activate(start, ToolFrameConfig{});
}

void ForceComplianceCore::activate(const sfc::CartesianState& start,
                                   const ToolFrameConfig& tf) {
  frame_.configure(tf.tool_a, tf.tool_b, tf.tool_c, tf.mount_a, tf.mount_b,
                   tf.mount_c);
  filter_.reset();
  law_.reset();
  motion_.cancel();
  ref_a_ = start.a;
  ref_b_ = start.b;
  ref_c_ = start.c;
  retare_.setReference(start.a, start.b, start.c);
  tare_armed_ = false;  // never tare before leaving the reference once
  force_deadband_ = params_.fixed_force_deadband_n;
  torque_deadband_ = params_.fixed_torque_deadband_nm;
  if (params_.adaptive_deadband) {
    ramp_.start(params_.ramp_window_s, params_.ramp_force_margin_n,
                params_.ramp_torque_margin_nm);
  }
}
```

3. `update()` 内三处变更(其余行不动):

在 `const sfc::Wrench filtered = ...` 之后、原 compensate 调用处:

```cpp
  // Frame chain (tool-frame design): r_bt is TCP->BASE from the live RIst
  // pose; r_base_sensor extends it through the session-constant sensor leg.
  const Eigen::Matrix3d r_bt =
      sfc::kukaAbcToRotation(in.state.a, in.state.b, in.state.c);
  const Eigen::Matrix3d r_base_sensor = r_bt * frame_.tcpFromSensor();
  out.compensated = compensator_.compensate(filtered, r_base_sensor);
```

retare 块内的第二次 compensate(原 83 行)同样改为:

```cpp
    out.compensated = compensator_.compensate(filtered, r_base_sensor);
```

原 `law_.compute(out.compensated, ...)` 与求和段改为:

```cpp
  sfc::ComplianceParams law_params = params_.compliance;
  law_params.translation.deadband = force_deadband_;
  law_params.rotation.deadband = torque_deadband_;
  // Admittance in the TOOL frame (deadband/speed caps are tool-frame
  // semantics per the approved design), then back to BASE for RSI.
  const sfc::Wrench w_tool = frame_.wrenchSensorToTool(out.compensated);
  const sfc::CartesianCorrection compliance_tool =
      law_.compute(w_tool, law_params, dt);
  const sfc::CartesianCorrection compliance =
      frame_.correctionToolToBase(compliance_tool, r_bt);
  const sfc::CartesianCorrection motion = motion_.update(in.state, dt);
```

注意:头文件需要 `Eigen::Matrix3d` 可见,`frame_resolver.h` 已带 `<Eigen/Dense>`;`force_compliance_core.cpp` 无需额外 include(`rotation.h` 已在)。

- [ ] **Step 4: 跑新测试 + 全部控制器回归**

Run:
```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make --pkg soft_robot_controllers >/dev/null 2>&1 \
  && ./devel/lib/soft_robot_controllers/test_fcc_tool_frame 2>&1 | tail -3 \
  && catkin_make run_tests_soft_robot_controllers -j4 2>&1 | grep -cE "\.xml: [0-9]+ tests.*0 failures" 
```
Expected: `test_fcc_tool_frame` 5 tests PASSED;所有既有 gtest 0 failures(现状事实 #6 保证)。

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_core.h \
        ros_ws/src/soft_robot_controllers/src/force_compliance_core.cpp \
        ros_ws/src/soft_robot_controllers/test/test_fcc_tool_frame.cpp \
        ros_ws/src/soft_robot_controllers/CMakeLists.txt
git commit -m "feat(fcc): tool-frame compliance pipeline with base back-transform"
```

---

### Task 4: 控制器壳 —— EKI 工具订阅 + 参数 + activate 锁存

**Files:**
- Modify: `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_controller.h`
- Modify: `ros_ws/src/soft_robot_controllers/src/force_compliance_controller.cpp`
- Test: `ros_ws/src/soft_robot_controllers/test/test_force_compliance_controller.cpp`(追加)

**Interfaces:**
- Consumes: Task 3 `ToolFrameConfig` + `activate(start, tf)`;`soft_robot_msgs::EkiState`(字段 `connected`, `tool_a/b/c`)。
- Produces(测试与运维依赖):

```cpp
// force_compliance_controller.h 新增(与 WrenchSample 并列):
struct ToolSample {
  double a{0}, b{0}, c{0};  // $TOOL A/B/C [deg]
  bool valid{false};        // ever received a connected EkiState
};
// 非 RT 注入口(EKI 订阅回调与离线测试共用,单生产者):
void injectTool(const ToolSample& t) { tool_buf_.writeFromNonRT(t); }
```

新参数(controller_nh 私有命名空间):`sensor_to_flange_rpy`(`std::vector<double>` 长度 3,`[A, B, C]` deg,默认 `[0,0,0]`);新话题参数 `eki_state_topic`(默认 `/kuka/eki/state`)。

**行为规则:**
- `ekiStateCb`:仅当 `msg->connected` 为 true 时写入 `tool_buf_`(valid=true)。工具值取 `msg->tool_a/b/c`(EKI 心跳暂停时 bridge 保留 last_state,现状事实 #5)。
- gate 进入分支与 `starting()` 重启分支:读 `tool_buf_`,组 `ToolFrameConfig`(tool 来自 buffer,mount 来自参数),调 `core_.activate(readState(), tf)`。
- 退化:`!tool.valid` 时 tool 三项按 0(恒等),`ROS_WARN_ONCE` 提示 "FORCE_COMPLIANCE activated without EKI tool data; assuming identity tool rotation";`gravity_n == 0`(取当前 profile 的 `payload.gravity_n`)时 `ROS_WARN_ONCE` 提示 "payload not calibrated (gravity_n == 0): zero-only mode, orientation changes will drift"。不阻断启动、不改 msg(Global Constraints)。

- [ ] **Step 1: 追加失败测试**

`test_force_compliance_controller.cpp` 文件末尾追加(fixture 已有 `state_`/`cmd_`/`cycle`;`using soft_robot_controllers::ToolSample;` 加到文件头 using 区):

```cpp
TEST_F(FccControllerTest, ToolRotationRemapsCorrection) {
  // $TOOL A=90 注入后进入模式:传感器 +X 40N -> 工具 -Y -> BASE(姿态恒等)
  // 输出 y=-0.04。未实现时输出会落在 x 上。
  ToolSample t;
  t.a = 90.0;
  t.valid = true;
  ctl_.injectTool(t);
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  WrenchSample s;
  s.w.fx = 40.0;
  s.stamp_s = kT0;
  s.valid = true;
  ctl_.injectWrench(s);
  ctl_.update(ros::Time(kT0), ros::Duration(0.004));
  EXPECT_NEAR(cmd_[0], 0.0, 1e-12);
  EXPECT_NEAR(cmd_[1], -0.04, 1e-12);
}

TEST_F(FccControllerTest, MissingToolDataDegradesToIdentity) {
  // 不注入工具数据:行为与现状一致(恒等工具),+Z 40N -> z=0.04。
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0);
  EXPECT_NEAR(cmd_[2], 0.04, 1e-12);
}

TEST_F(FccControllerTest, ToolLockedAtActivationNotLive) {
  // 会话内锁存:进入模式后再改工具数据不得影响当前会话。
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0);
  EXPECT_NEAR(cmd_[2], 0.04, 1e-12);  // 恒等工具下的 +Z 响应
  ToolSample t;
  t.a = 90.0;
  t.valid = true;
  ctl_.injectTool(t);  // servo 中途到达的工具数据
  cycle(40.0, kT0 + 0.004);
  EXPECT_NEAR(cmd_[2], 0.08 - 0.04, 1e-9);  // 仍按恒等工具响应
  // (0.04 mm/cycle 不因工具数据变化;此断言写成增量式以绕开滤波关断后的
  //  无状态性:第二周期输出仍是 0.04,直接断言即可)
  EXPECT_NEAR(cmd_[2], 0.04, 1e-12);
}
```

(注:第三个测试最后两行二选一 —— 实现者跑一次确认 `cmd_[2]` 第二周期仍为 0.04 后删掉增量式那行,保留 `EXPECT_NEAR(cmd_[2], 0.04, 1e-12)`。)

- [ ] **Step 2: 跑测试确认编译失败**

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make --pkg soft_robot_controllers 2>&1 | grep -E "error" | head -3`
Expected: FAIL —— `ToolSample was not declared` / `injectTool` 不存在

- [ ] **Step 3: 实现壳层变更**

`force_compliance_controller.h`:
1. include 区加 `#include <soft_robot_msgs/EkiState.h>` 与 `#include <vector>`
2. `WrenchSample` 之后加:

```cpp
// Latest $TOOL A/B/C from the EKI bridge; locked into the core at mode
// entry (session-constant, spec tool-frame design). valid stays false
// until the first connected EkiState arrives.
struct ToolSample {
  double a{0}, b{0}, c{0};
  bool valid{false};
};
```

3. 类内 public 注入口(`injectRsiFault` 旁):

```cpp
  void injectTool(const ToolSample& t) { tool_buf_.writeFromNonRT(t); }
```

4. private 区:回调声明 `void ekiStateCb(const soft_robot_msgs::EkiState::ConstPtr& msg);`、辅助 `void activateCore();`、成员 `realtime_tools::RealtimeBuffer<ToolSample> tool_buf_;`、`ros::Subscriber eki_sub_;`、`double mount_a_{0}, mount_b_{0}, mount_c_{0};`

`force_compliance_controller.cpp`:
1. `init()` 参数段(`wrench_timeout` 读取之后)加:

```cpp
  std::vector<double> mount_rpy{0.0, 0.0, 0.0};
  controller_nh.param("sensor_to_flange_rpy", mount_rpy, mount_rpy);
  if (mount_rpy.size() == 3) {
    mount_a_ = mount_rpy[0];
    mount_b_ = mount_rpy[1];
    mount_c_ = mount_rpy[2];
  } else {
    ROS_WARN("sensor_to_flange_rpy must have 3 elements; using identity");
  }
```

2. 订阅段(`rsi_sub_` 之后)加:

```cpp
  std::string eki_topic;
  controller_nh.param<std::string>("eki_state_topic", eki_topic,
                                   std::string("/kuka/eki/state"));
  eki_sub_ =
      root_nh.subscribe(eki_topic, 1, &ForceComplianceController::ekiStateCb,
                        this);
```

3. `configureController()` 内 buffer 初始化段加 `tool_buf_.writeFromNonRT(ToolSample{});`
4. 回调实现(`rsiStateCb` 之后):

```cpp
void ForceComplianceController::ekiStateCb(
    const soft_robot_msgs::EkiState::ConstPtr& msg) {
  if (!msg->connected) return;  // keep the last known tool
  ToolSample t;
  t.a = msg->tool_a;
  t.b = msg->tool_b;
  t.c = msg->tool_c;
  t.valid = true;
  injectTool(t);
}
```

5. 锁存辅助(`readState()` 旁):

```cpp
void ForceComplianceController::activateCore() {
  const ToolSample tool = *tool_buf_.readFromRT();
  ToolFrameConfig tf;
  if (tool.valid) {
    tf.tool_a = tool.a;
    tf.tool_b = tool.b;
    tf.tool_c = tool.c;
  } else {
    ROS_WARN_ONCE(
        "FORCE_COMPLIANCE activated without EKI tool data; assuming identity "
        "tool rotation");
  }
  tf.mount_a = mount_a_;
  tf.mount_b = mount_b_;
  tf.mount_c = mount_c_;
  if (core_.payload().gravity_n == 0.0) {
    ROS_WARN_ONCE(
        "payload not calibrated (gravity_n == 0): zero-only mode, "
        "orientation changes will drift");
  }
  core_.activate(readState(), tf);
}
```

6. 两处 `core_.activate(readState());` 调用(`starting()` 的 gate.engaged 分支、`update()` 的 gate.apply 进入分支)都换成 `activateCore();`。注意 `update()` 分支中 `activateCore()` 必须在 `core_.configure(...)` **之后**调用(保持现有顺序:configure 再 activate)。

头文件 using:`ToolFrameConfig` 来自 `force_compliance_core.h`(同命名空间,无需额外 include)。

- [ ] **Step 4: 跑控制器测试全量**

Run:
```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make --pkg soft_robot_controllers >/dev/null 2>&1 \
  && ./devel/lib/soft_robot_controllers/test_force_compliance_controller 2>&1 | tail -3 \
  && ./devel/lib/soft_robot_controllers/test_controller_chain 2>&1 | tail -3
```
Expected: 两个二进制全部 PASSED(新增 3 个用例 + 既有用例)。

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_controller.h \
        ros_ws/src/soft_robot_controllers/src/force_compliance_controller.cpp \
        ros_ws/src/soft_robot_controllers/test/test_force_compliance_controller.cpp
git commit -m "feat(fcc): lock EKI tool frame at activation; sensor_to_flange_rpy param"
```

---

### Task 5: 配置示例 + 全仓回归 + 文档

**Files:**
- Modify: `ros_ws/src/soft_robot_bringup/config/`(找到 FCC 控制器配置段的 yaml,追加注释示例;若无 FCC 段则跳过 yaml,只做回归和文档)
- Modify: `docs/superpowers/specs/2026-07-04-tool-frame-compliance-design.md`(状态行改"已实施")

**Interfaces:**
- Consumes: Task 4 的参数名 `sensor_to_flange_rpy`、`eki_state_topic`。
- Produces: 无(收尾任务)。

- [ ] **Step 1: 找到并更新控制器 yaml**

Run: `grep -rln "force_compliance\|profiles:" ros_ws/src/soft_robot_bringup/config/ ros_ws/src/*/config/ 2>/dev/null`

在找到的 FCC 控制器段(与 `profiles:`/`safety:` 同级)加注释示例:

```yaml
  # Tool-frame chain (docs/superpowers/specs/2026-07-04-tool-frame-compliance-design.md)
  # sensor_to_flange_rpy: [0.0, 0.0, 0.0]   # KUKA A/B/C deg; sensor mount offset, identity = aligned
  # eki_state_topic: /kuka/eki/state        # $TOOL source, locked at servo activation
```

- [ ] **Step 2: 全仓测试回归**

Run:
```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make -j4 >/dev/null 2>&1 \
  && catkin_make run_tests -j4 2>&1 | grep -E "tests.*failures" | grep -v "0 failures\|0 errors" | head
```
Expected: 无输出(所有包 0 failures)。再跑 web 侧确认没被波及:
`cd ros_ws/src/soft_robot_web_interface && node --test test/unit/ 2>&1 | tail -4` → 29 pass。

- [ ] **Step 3: 更新 spec 状态 + Commit**

spec 头部 `状态:待实施` 改为 `状态:已实施(<当日 commit 短哈希>)`。

```bash
cd /home/ljj/kuka_iiqka_ros
git add -A
git commit -m "docs+config: tool-frame compliance examples, spec marked implemented"
```

- [ ] **Step 4: 推送**

```bash
git push git@github.com:wingljj/kuka_iiqka_ros.git main
```

---

## Self-Review 记录

1. **Spec coverage**:帧链/两次变换(Task 1+3)、传感器系重力修正(Task 2+3)、EKI 订阅+锁存+`sensor_to_flange_rpy`(Task 4)、未标定退化+诊断(Task 2 测试 + Task 4 WARN)、无工具数据退化(Task 3 恒等默认 + Task 4 WARN + 测试)、各向同性回归保护(Task 3 `LegacyActivateEqualsZeroConfig`)、工具旋转重映射(Task 1/3/4 各层)、往返一致性(Task 1)、测试计划 7 项全部有对应用例。spec 的"ModeState 诊断标记"在 Global Constraints 里明确降级为 ROS_WARN_ONCE(不改 msg,避免破坏 29 项 web 测试的枚举铆定)——与用户批准的"给出明确诊断"意图一致,实现渠道不同,已在约束里写明理由。
2. **Placeholder scan**:无 TBD/TODO;Task 4 Step 1 第三个测试有一处"实现者跑一次后二选一"的显式指令——这是有意的可执行指令,不是占位符。
3. **Type consistency**:`ToolFrameConfig`(Task 3 定义,Task 4 消费)、`ToolSample`/`injectTool`(Task 4 定义与消费)、`compensate(raw, Matrix3d)`(Task 2 定义,Task 3 消费)、`FrameResolver` 三方法(Task 1 定义,Task 3 消费)签名逐一核对一致;`kDt=0.004`、增益 1.0、死区 30 与既有测试常数一致。
