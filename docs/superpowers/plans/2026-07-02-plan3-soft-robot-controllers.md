# Plan 3/6: `soft_robot_controllers`(ros_control 控制器插件)实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标:** 按规格 `docs/superpowers/specs/2026-07-02-kuka-rsi-ros-control-design.md` §5.3、§7、§10、§12.1,交付 ros_control 控制器包 `soft_robot_controllers`:力顺应控制器 `ForceComplianceController`(滤波 → 重力补偿 → 自适应死区 → 自动回零 → 导纳律 → 与姿态运动修正求和 → 安全限幅 → RKorr)、直通/标定控制器 `CartesianCorrectionController`(stream 直通 + RKorr goal 模式内嵌 `OrientationMotionCore`,服务 `/soft_robot/move_to_orientation` action)、模式管理集成(`ModeManagerCore` + `ModeCommand`/`ModeState` + 编译期常量对齐断言)、pluginlib 注册与离线全链路集成测试。

**架构:** 延续 Plan 1/2 的"纯逻辑类 + 薄 ROS 壳"分层。每周期算法管线封装在两个不依赖 `ros::NodeHandle` 的纯逻辑类(`ForceComplianceCore`、`DirectCorrectionCore`)与一个模式门(`ControllerModeGate`)中,全部 gtest 离线覆盖;控制器壳只做参数装载、话题↔RealtimeBuffer 搬运、action 线程与 RT 循环的原子握手。控制器壳提供免 ROS master 的 `configureController()` 入口(同 `KukaRsiRobotHW::configure` 模式),使 `update()` 全路径可在纯 gtest 中以手工 update 循环验证,直至与 Plan 2 的 `KukaRsiRobotHW` × `RsiMockServer` 闭环。

**Plan series:** ① core algorithm library(已完成)→ ② `kuka_rsi_hw_interface` + RSI mock + msgs(已完成)→ ③ `soft_robot_controllers`(本计划)→ ④ `kuka_eki_bridge` + `sri_force_torque_driver` + mocks → ⑤ manager + calibration + bringup + KUKA 模板 → ⑥ web interface。

## 范围

**包清单(1 个新 catkin 包):**

| 包 | 内容 |
|---|---|
| `soft_robot_controllers` | `ForceComplianceController`、`CartesianCorrectionController` 两个 `controller_interface::Controller<kuka_rsi::CartesianCorrectionCommandInterface>` 插件;纯逻辑层 `ForceComplianceCore`/`DirectCorrectionCore`/`ControllerModeGate`/`mode_bridge`;plugins.xml、controllers 配置 YAML、全套离线 gtest(含与 Plan 2 mock 的闭环链路测试) |

**非目标(本计划不做):**

- manager 状态机、系统级 `system_state` 权威发布、标定编排、`payload.yaml` 持久化(→ Plan 5)。本计划 `ModeState` 发布是控制器视角的过渡语义(见待确认 3)。
- EKI 桥、SRI 驱动及其 mock(→ Plan 4)。控制器只假定 `/sri_ft/wrench_raw`(`geometry_msgs/WrenchStamped`)话题存在。
- controller_manager 的 rostest/roslaunch 级测试(`ControllerManager` 构造需要 ROS master):spawner 冒烟为手动步骤,不进验收门槛。
- `MoveToOrientation` 的位置分量(`use_position=true` 直接 ABORTED,见待确认 6);`OrientationMotionCore` v1 仅姿态。
- 控制器间自动切换(`controller_manager` switch 由 Plan 5 manager 调用)。
- 参数快照的运行时热更新服务(Plan 5;本计划参数在 `init()` 装载、profile 进入时选择)。

**与其他 Plan 的接口关系:**

- 消费 Plan 1:`sfc::` 全部算法类(`ForceTorqueFilter`、`ToolGravityCompensator`、`ComplianceLaw`、`SafetyLimiter`、`AdaptiveDeadband`、`AutoReTare`、`OrientationMotionCore`、`ModeManagerCore`、`rotation.h`、`types.h`)。
- 消费 Plan 2:`kuka_rsi::CartesianStateInterface`/`CartesianCorrectionCommandInterface`(资源名 `kuka_tcp`)、`soft_robot_msgs`(`CartesianCorrectionStamped`、`ModeCommand`/`ModeState`、`RsiState`、`MoveToOrientation.action`)、`KukaRsiRobotHW` + `RsiMockServer`(Task 9 链路测试)。
- 供给 Plan 5:两个控制器插件名(`soft_robot_controllers/ForceComplianceController`、`soft_robot_controllers/CartesianCorrectionController`)、`/soft_robot/mode_command`(ModeCommand)、`/soft_robot/mode_state`(ModeState)、`/soft_robot/move_to_orientation`(action)、`config/soft_robot_controllers.yaml`(spawner 参数模板)。
- 供给 Plan 4:无直接接口;`wrench_topic` 默认 `/sri_ft/wrench_raw` 与规格 §5.6 SRI 驱动话题名对齐。
- git 基线:从 `feature/rsi-hw-interface-msgs` 新建 `feature/soft-robot-controllers`(合并顺序 Plan 1 → 2 → 3)。

## 待确认(规格未定项,本计划采用的默认决策)

1. **控制器命名**:规格 §5.3 称 `SoftComplianceController`,任务要求为 `ForceComplianceController`——采用后者(与 `FORCE_COMPLIANCE` 模式名一致),插件名 `soft_robot_controllers/ForceComplianceController`。规格文字引用处视为同一对象。
2. **故障感知(Plan 2 跟进 3)**:采用"订阅 `/kuka/rsi/state`(非 RT 话题)→ RealtimeBuffer"方案,不新增可选的 `RobotModeStateInterface`。订阅发生在 `init()`,回调运行在节点 spinner 线程,`update()` 只读缓冲——满足"update() 不做非 RT 订阅"。`fault=true` 时控制器输出零。
3. **模式分发**:两控制器各自订阅 `/soft_robot/mode_command`,各内嵌一个 `ModeManagerCore`(确定性转移规则,两实例对同一命令流结论一致)。`ModeCommand` 为电平语义而非队列:RealtimeBuffer 只保留最新请求,manager(Plan 5)负责按需重发。系统级 `system_state` 权威属 Plan 5;本计划 `ForceComplianceController` 发布的 `ModeState.system_state` 仅取 SERVOING/READY/DEGRADED 三值(控制器视角,过渡语义,Plan 5 接管后降级为诊断)。
4. **FCC 内嵌 OrientationMotionCore**:`ForceComplianceCore` 自带一个 `OrientationMotionCore` 实例,其输出与顺应修正**先求和再过 SafetyLimiter**(Plan 1 跟进 2)。v1 中该内嵌实例无 ROS 入口(供将来"顺应中回正"动作),action 服务仅由 `CartesianCorrectionController` 提供;求和路径由单测直接驱动内嵌实例验证。
5. **Action 实现样式**:用 `actionlib::SimpleActionServer` + "RealtimeBuffer 送 goal、std::atomic 回读状态"的轮询样式(不用 `RealtimeServerGoalHandle` 全套)。execute 回调(AS 线程)绝不触碰 RT 数据结构;goal 经 `requestGoal()`(带序号)进 RT,状态经 `statusSeq()`/`motionStatusValue()` 原子回读。离线 gtest 直接调 `requestGoal`/`requestCancel`(与 execute 回调完全同路径);action 通信本身属手动冒烟。`MotionStatus::INACTIVE` 不区分"未启动/被 cancel"(Plan 1 跟进 5):壳层自己记账——preempt 时壳层调用了 cancel 故回报 PREEMPTED;非 preempt 而状态回到 INACTIVE 说明 RT 侧因模式切换取消,回报 ABORTED。
6. **`use_position=true` 拒绝**:`OrientationMotionCore` v1 无位置通道,action 收到位置目标直接 `setAborted`(message 说明)。Plan 5 标定序列只用姿态,不受影响。
7. **`RsiMockCore::setJointAngles` 不补**(Plan 2 跟进 2 的条件项):本计划两控制器均不消费 `JointStateInterface`(管线只用笛卡尔状态与 wrench),测试不需要非零关节角,故不加该 setter。**但需一处勘误性小改动**:`kuka_rsi_hw_interface` 的 `catkin_package(LIBRARIES ...)` 未导出 `kuka_rsi_hw_interface_mock`,Task 9 跨包链路测试无法链接——在 Task 9 中将其加入导出列表(性质同"测试需要就补 setter"的许可,见 Task 9 勘误说明)。
8. **`speed_scale` 语义**:action goal 的 `speed_scale ∈ (0,1]` 乘到参数化的 `goal_max_speed_dps` 上;越界值(≤0 或 >1)按 1.0 处理并继续执行。
9. **AutoReTare 再武装滞回**:重新武装阈值 = `retare_rearm_factor × orientation_tol_deg`(默认 factor 2.0),避免在容差边界抖动导致重复 tare(Plan 1 跟进 1 的具体化)。
10. **硬截断语义保持严格大于**(Plan 1 跟进 4):`SafetyLimiter` 现实现为 `> ceiling` 触发,本计划不改;测试显式固化"恰等于 500 N 不触发";若安全评审改为 ≥,只改 Plan 1 的 `safety_limiter.cpp` 与参数文档。
11. **stream 与 goal 并存**:`DirectCorrectionCore` 中 goal RUNNING 期间 stream 命令被忽略(goal 优先);goal 结束(收敛/超时/取消)后 stream 自动恢复。
12. **profile 切换时机**:`ModeManagerCore` 规则强制 profile 仅在 IDLE 可改;单条 `ModeCommand{FORCE_COMPLIANCE, DRAG}` 从 IDLE 出发时,门先应用 profile(此刻仍 IDLE,成功)再切模式——一条消息即可完成"选 profile + 进入"。进入(entered 边沿)时以所选 profile 重建 `ForceComplianceCore`(含 `ForceTorqueFilter` 重建 + reset,Plan 1 跟进 3)并以当前位姿 `activate()`。

## Global Constraints

- ROS1 Noetic,catkin 工作区 `/home/ljj/kuka_iiqka_ros/ros_ws`;新包 `ros_ws/src/soft_robot_controllers/`。
- C++14,`-Wall -Wextra`,零警告;所有代码与注释英文。TDD:先失败测试(构建失败即失败态),再最小实现,再通过,再提交。
- **Noetic 的 `catkin_add_gtest` 不自动链接 gtest_main:所有测试链接行必须包含 `${GTEST_MAIN_LIBRARIES}`。**
- 单位:笛卡尔 mm/deg(KUKA A/B/C = Z-Y-X),wrench N/Nm(`geometry_msgs/WrenchStamped`),修正为 per-cycle 增量。
- **跟进事项落实(本计划的硬性验收项)**:
  1. **dt 用实测 `period`**(Plan 2 跟进 1):`update()` 一律 `period.toSec()` 传入核心,任何实现/测试不得写死 0.004;Task 3/7 各有一条"dt 加倍则修正加倍"测试固化此约束。
  2. **AutoReTare 边沿触发**(Plan 1 跟进 1):`ForceComplianceCore` 持 `tare_armed_` 标志——`activate()` 置 false(起点即参考姿态,不许立即 tare);姿态离开参考超过 `rearm_factor × tol` 才武装;`shouldTare` 为真且已武装才 `absorbResidual` 并立即解除武装。每次"离开-返回"至多 tare 一次。Task 4 逐条测试。
  3. **两路修正先求和再过 SafetyLimiter**(Plan 1 跟进 2):`ForceComplianceCore::update()` 中 `compliance + motion` 求和后进 `limiter_.apply()`,per-cycle clamp 作用于合成量。Task 5 用"各自不超限、合成超限被夹"的用例固化。
  4. **profile 切换重建滤波器**(Plan 1 跟进 3):`ForceComplianceCore::configure()` 以 `filter_ = sfc::ForceTorqueFilter(cutoff)` 重建对象(隐含全新未初始化状态);`activate()` 再显式 `filter_.reset()`。进入 FORCE_COMPLIANCE 的 entered 边沿必经 configure+activate。
  5. **硬截断严格大于**(Plan 1 跟进 4):沿用,测试固化(Task 3)。
  6. **故障感知**(Plan 2 跟进 3):方案 A(订阅 `/kuka/rsi/state`),见待确认 2。
  7. **常量对齐 static_assert**(Plan 2 跟进 4):`mode_bridge.h` 编译期断言 `soft_robot_msgs::ModeCommand` 常量 × `sfc::ControlMode`/`sfc::Profile` 枚举序号(Task 1)。
  8. **`setJointAngles` 决定不加**(Plan 2 跟进 2):理由见待确认 7;代之以导出 mock 库的勘误(Task 9)。
- `update()` 实时安全:话题输入一律经 `realtime_tools::RealtimeBuffer`(回调 `writeFromNonRT`,update `readFromRT` 后按值拷贝);action goal/cancel/状态经 RealtimeBuffer + `std::atomic`;`update()` 内无订阅/发布器创建、无参数服务器访问、无文件与 socket I/O、无阻塞锁(`RealtimePublisher::trylock` 失败即跳过)、无动态分配(核心类全为值成员;`ForceTorqueFilter` 重建是栈上赋值)。
- 纯逻辑类(`ForceComplianceCore`、`DirectCorrectionCore`、`ControllerModeGate`、`mode_bridge`)不包含 `ros/ros.h`(`mode_bridge`/`controller_mode_gate` 仅用消息头的编译期常量,无 roscpp 运行时依赖)。
- 全部 gtest 离线可跑:无 roscore、无真机;用 `ros::Time::init()` 启用壁钟时间(无 master 依赖);Task 9 仅用 127.0.0.1 loopback UDP;测试内所有窗口/超时 ≤ 0.5 s。
- 构建/运行命令沿用前两计划(仓库根 `/home/ljj/kuka_iiqka_ros`):

```bash
cd ros_ws && catkin_make tests                          # build all test binaries
./devel/lib/soft_robot_controllers/<test_binary>        # run one gtest binary
```

**`ForceComplianceController::update()` 管线伪代码(规格 §7.1 的本计划落地形态,跟进事项 1/2/3 已内嵌):**

```text
apply latest ModeCommand request (edge by sequence number):
  gate.apply(mode, profile)            # ModeManagerCore rules
  on entered FORCE_COMPLIANCE:
    core.configure(params for selected profile)   # rebuilds + resets filter
    core.activate(current pose from handle)       # retare ref, DRAG ramp start
engaged = gate.engaged() and not rsi_fault
if not engaged: setCommand(0); publish state; return
in.raw        = latest wrench from realtime buffer
in.wrench_age = time - sample.stamp
in.state      = handle X/Y/Z/A/B/C
out = core.update(in, period.toSec())            # dt = measured period
  core pipeline:
    wrench invalid or stale        -> zero + timeout flag (law reset)
    filter -> gravity compensation
    DRAG ramp active               -> zero output, deadband learning,
                                      hard cutoff stays armed
    auto re-tare (edge-triggered)  -> absorbResidual at most once per return
    compliance = law.compute(comp, deadbands, dt)
    motion     = embedded OrientationMotionCore.update(state, dt)
    sum        = compliance + motion               # BEFORE the limiter
    out        = SafetyLimiter.apply(sum, comp)    # clamp + 500 N hard cutoff
setCommand(out.correction)
publish ModeState via RealtimePublisher (throttled, trylock)
```

---

## File Structure

```text
ros_ws/src/soft_robot_controllers/
  package.xml
  CMakeLists.txt
  soft_robot_controllers_plugins.xml
  include/soft_robot_controllers/
    mode_bridge.h                       # compile-time msg-constant/enum alignment + conversions
    controller_mode_gate.h              # ModeManagerCore integration per controller
    force_compliance_core.h             # full FORCE_COMPLIANCE realtime pipeline (pure)
    direct_correction_core.h            # stream/goal command source logic (pure)
    force_compliance_controller.h       # ros_control plugin shell
    cartesian_correction_controller.h   # ros_control plugin shell + action server
  src/
    controller_mode_gate.cpp
    force_compliance_core.cpp
    direct_correction_core.cpp
    force_compliance_controller.cpp
    cartesian_correction_controller.cpp
  config/
    soft_robot_controllers.yaml
  test/
    test_mode_bridge.cpp
    test_controller_mode_gate.cpp
    test_force_compliance_core.cpp
    test_fcc_adaptive_retare.cpp
    test_fcc_orientation_sum.cpp
    test_direct_correction_core.cpp
    test_force_compliance_controller.cpp
    test_cartesian_correction_controller.cpp
    test_controller_chain.cpp

ros_ws/src/kuka_rsi_hw_interface/
  CMakeLists.txt                        # Task 9 errata: export mock library
```

---

### Task 0: 建立分支

- [ ] **Step 1: 从 Plan 2 分支创建工作分支**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git checkout feature/rsi-hw-interface-msgs && \
git checkout -b feature/soft-robot-controllers
```

预期:`Switched to a new branch 'feature/soft-robot-controllers'`。

---

### Task 1: 包骨架 + `mode_bridge.h`(编译期常量对齐 + 转换)

**Files:**
- Create: `ros_ws/src/soft_robot_controllers/package.xml`
- Create: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- Create: `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/mode_bridge.h`
- Test: `ros_ws/src/soft_robot_controllers/test/test_mode_bridge.cpp`

**Interfaces:**
- Consumes: `soft_robot_msgs/ModeCommand`(常量)、`sfc::ControlMode`/`sfc::Profile`(`soft_force_control_core/types.h`)。
- Produces(Task 2/7/8 消费):
  - 6 条 `static_assert`:msg 常量数值 × sfc 枚举序号(Plan 2 跟进 4 的落地)。
  - `bool soft_robot_controllers::toControlMode(std::uint8_t, sfc::ControlMode&)`(未知值返回 false)。
  - `bool soft_robot_controllers::toProfile(std::uint8_t, sfc::Profile&)`。
  - `std::uint8_t fromControlMode(sfc::ControlMode)` / `std::uint8_t fromProfile(sfc::Profile)`。

- [ ] **Step 1: 写包清单与构建文件**

`ros_ws/src/soft_robot_controllers/package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>soft_robot_controllers</name>
  <version>0.1.0</version>
  <description>
    ros_control controller plugins for the KUKA RSI soft force control
    system (spec sections 5.3, 7, 10, 12.1): ForceComplianceController
    (admittance pipeline over the SRI wrench) and
    CartesianCorrectionController (stream passthrough + RKorr goal mode
    hosting OrientationMotionCore). Pure-logic cores are ROS-free and
    fully covered by offline gtests.
  </description>
  <maintainer email="dev@example.com">ljj</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>controller_interface</depend>
  <depend>hardware_interface</depend>
  <depend>realtime_tools</depend>
  <depend>actionlib</depend>
  <depend>pluginlib</depend>
  <depend>geometry_msgs</depend>
  <depend>soft_force_control_core</depend>
  <depend>kuka_rsi_hw_interface</depend>
  <depend>soft_robot_msgs</depend>
  <export>
    <controller_interface plugin="${prefix}/soft_robot_controllers_plugins.xml"/>
  </export>
</package>
```

(`export` 段指向的 plugins.xml 在 Task 10 落盘;catkin 允许 export 先行,pluginlib 只有在扫描插件时才读取该文件——Task 10 前不 spawner 即可。)

`ros_ws/src/soft_robot_controllers/CMakeLists.txt`(初版,后续 Task 增量扩展):

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(soft_robot_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  realtime_tools
  actionlib
  pluginlib
  geometry_msgs
  soft_force_control_core
  kuka_rsi_hw_interface
  soft_robot_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp controller_interface hardware_interface
                 realtime_tools actionlib pluginlib geometry_msgs
                 soft_force_control_core kuka_rsi_hw_interface
                 soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mode_bridge test/test_mode_bridge.cpp)
  add_dependencies(test_mode_bridge ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_mode_bridge ${GTEST_MAIN_LIBRARIES})
endif()
```

- [ ] **Step 2: 写失败测试**

`ros_ws/src/soft_robot_controllers/test/test_mode_bridge.cpp`:

```cpp
#include <gtest/gtest.h>

#include "soft_robot_controllers/mode_bridge.h"

using soft_robot_controllers::fromControlMode;
using soft_robot_controllers::fromProfile;
using soft_robot_controllers::toControlMode;
using soft_robot_controllers::toProfile;

// The static_asserts in mode_bridge.h are the real alignment gate: this
// binary compiling at all proves msg constants match the sfc enums.
TEST(ModeBridge, ControlModeRoundTrip) {
  for (const sfc::ControlMode m :
       {sfc::ControlMode::IDLE, sfc::ControlMode::DIRECT_CARTESIAN,
        sfc::ControlMode::FORCE_COMPLIANCE, sfc::ControlMode::CALIBRATION}) {
    sfc::ControlMode back = sfc::ControlMode::IDLE;
    ASSERT_TRUE(toControlMode(fromControlMode(m), back));
    EXPECT_EQ(back, m);
  }
}

TEST(ModeBridge, ProfileRoundTrip) {
  for (const sfc::Profile p : {sfc::Profile::DRAG, sfc::Profile::PRECISION}) {
    sfc::Profile back = sfc::Profile::DRAG;
    ASSERT_TRUE(toProfile(fromProfile(p), back));
    EXPECT_EQ(back, p);
  }
}

TEST(ModeBridge, RejectsUnknownMode) {
  sfc::ControlMode out = sfc::ControlMode::FORCE_COMPLIANCE;
  EXPECT_FALSE(toControlMode(99, out));
  // A rejected conversion must not touch the output.
  EXPECT_EQ(out, sfc::ControlMode::FORCE_COMPLIANCE);
}

TEST(ModeBridge, RejectsUnknownProfile) {
  sfc::Profile out = sfc::Profile::PRECISION;
  EXPECT_FALSE(toProfile(7, out));
  EXPECT_EQ(out, sfc::Profile::PRECISION);
}
```

- [ ] **Step 3: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,`fatal error: soft_robot_controllers/mode_bridge.h: No such file or directory`。

- [ ] **Step 4: 写实现**

`ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/mode_bridge.h`:

```cpp
#pragma once

#include <soft_robot_msgs/ModeCommand.h>

#include <cstdint>

#include "soft_force_control_core/types.h"

namespace soft_robot_controllers {

// Compile-time alignment between the soft_robot_msgs wire constants and
// the sfc enums (Plan 2 follow-up 4). If either side is ever reordered,
// this header stops compiling instead of silently misrouting modes.
static_assert(soft_robot_msgs::ModeCommand::MODE_IDLE ==
                  static_cast<std::uint8_t>(sfc::ControlMode::IDLE),
              "MODE_IDLE must equal sfc::ControlMode::IDLE");
static_assert(soft_robot_msgs::ModeCommand::MODE_DIRECT_CARTESIAN ==
                  static_cast<std::uint8_t>(sfc::ControlMode::DIRECT_CARTESIAN),
              "MODE_DIRECT_CARTESIAN must equal sfc::ControlMode::DIRECT_CARTESIAN");
static_assert(soft_robot_msgs::ModeCommand::MODE_FORCE_COMPLIANCE ==
                  static_cast<std::uint8_t>(sfc::ControlMode::FORCE_COMPLIANCE),
              "MODE_FORCE_COMPLIANCE must equal sfc::ControlMode::FORCE_COMPLIANCE");
static_assert(soft_robot_msgs::ModeCommand::MODE_CALIBRATION ==
                  static_cast<std::uint8_t>(sfc::ControlMode::CALIBRATION),
              "MODE_CALIBRATION must equal sfc::ControlMode::CALIBRATION");
static_assert(soft_robot_msgs::ModeCommand::PROFILE_DRAG ==
                  static_cast<std::uint8_t>(sfc::Profile::DRAG),
              "PROFILE_DRAG must equal sfc::Profile::DRAG");
static_assert(soft_robot_msgs::ModeCommand::PROFILE_PRECISION ==
                  static_cast<std::uint8_t>(sfc::Profile::PRECISION),
              "PROFILE_PRECISION must equal sfc::Profile::PRECISION");

// Checked conversions from wire values. Unknown values are rejected (the
// caller ignores the whole request) instead of being cast blindly.
inline bool toControlMode(std::uint8_t raw, sfc::ControlMode& out) {
  switch (raw) {
    case soft_robot_msgs::ModeCommand::MODE_IDLE:
      out = sfc::ControlMode::IDLE;
      return true;
    case soft_robot_msgs::ModeCommand::MODE_DIRECT_CARTESIAN:
      out = sfc::ControlMode::DIRECT_CARTESIAN;
      return true;
    case soft_robot_msgs::ModeCommand::MODE_FORCE_COMPLIANCE:
      out = sfc::ControlMode::FORCE_COMPLIANCE;
      return true;
    case soft_robot_msgs::ModeCommand::MODE_CALIBRATION:
      out = sfc::ControlMode::CALIBRATION;
      return true;
    default:
      return false;
  }
}

inline bool toProfile(std::uint8_t raw, sfc::Profile& out) {
  switch (raw) {
    case soft_robot_msgs::ModeCommand::PROFILE_DRAG:
      out = sfc::Profile::DRAG;
      return true;
    case soft_robot_msgs::ModeCommand::PROFILE_PRECISION:
      out = sfc::Profile::PRECISION;
      return true;
    default:
      return false;
  }
}

inline std::uint8_t fromControlMode(sfc::ControlMode m) {
  return static_cast<std::uint8_t>(m);
}
inline std::uint8_t fromProfile(sfc::Profile p) {
  return static_cast<std::uint8_t>(p);
}

}  // namespace soft_robot_controllers
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_controllers/test_mode_bridge
```

预期:`[  PASSED  ] 4 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers && \
git commit -m "feat(controllers): add package skeleton and mode bridge with compile-time constant alignment (Plan 3 Task 1)"
```

---

### Task 2: `ControllerModeGate`(ModeManagerCore 集成 + 进入边沿)

**Files:**
- Create: `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/controller_mode_gate.h`
- Create: `ros_ws/src/soft_robot_controllers/src/controller_mode_gate.cpp`
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_controllers/test/test_controller_mode_gate.cpp`

**Interfaces:**
- Consumes: `sfc::ModeManagerCore`(转移规则:一切经 IDLE)、Task 1 `mode_bridge.h`。
- Produces(Task 7/8 消费):
  - `struct ModeRequest { std::uint8_t mode; std::uint8_t profile; std::uint64_t seq; }`(seq==0 表示"尚无请求";RealtimeBuffer 载荷)。
  - `class ControllerModeGate`:`ControllerModeGate(sfc::ControlMode engaged)` / `(engaged_a, engaged_b)`;`bool apply(const ModeRequest&)`(按 seq 去重,返回"本次调用进入接管模式"的上升沿);`bool engaged() const`;`void forceIdle()`;`sfc::ModeSnapshot snapshot() const`;`bool lastRequestOk() const`。
- 语义要点:profile 仅 IDLE 可改(`ModeManagerCore` 规则)。`apply` 在离开 IDLE 前先套 profile、进入 IDLE 后再套 profile,使单条 `ModeCommand` 即可同时选 profile 与切模式(待确认 12)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/soft_robot_controllers/test/test_controller_mode_gate.cpp`:

```cpp
#include <gtest/gtest.h>

#include "soft_robot_controllers/controller_mode_gate.h"

using soft_robot_controllers::ControllerModeGate;
using soft_robot_controllers::ModeRequest;
namespace msg = soft_robot_msgs;

namespace {
ModeRequest req(std::uint8_t mode, std::uint8_t profile, std::uint64_t seq) {
  ModeRequest r;
  r.mode = mode;
  r.profile = profile;
  r.seq = seq;
  return r;
}
}  // namespace

TEST(ControllerModeGate, SeqZeroIsIgnored) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_FALSE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                              msg::ModeCommand::PROFILE_DRAG, 0)));
  EXPECT_FALSE(gate.engaged());
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::IDLE);
}

TEST(ControllerModeGate, AppliesOncePerSequenceNumber) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  const ModeRequest r = req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                            msg::ModeCommand::PROFILE_PRECISION, 1);
  EXPECT_TRUE(gate.apply(r));    // entered edge
  EXPECT_FALSE(gate.apply(r));   // same seq: no-op, no edge
  EXPECT_TRUE(gate.engaged());
}

TEST(ControllerModeGate, EnterEdgeFiresOnlyOnTransition) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_TRUE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                             msg::ModeCommand::PROFILE_PRECISION, 1)));
  // Re-requesting the mode we are already in: accepted, but not an edge.
  EXPECT_FALSE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                              msg::ModeCommand::PROFILE_PRECISION, 2)));
  EXPECT_TRUE(gate.engaged());
}

TEST(ControllerModeGate, SingleMessageSelectsProfileAndMode) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_TRUE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                             msg::ModeCommand::PROFILE_DRAG, 1)));
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_EQ(gate.snapshot().profile, sfc::Profile::DRAG);
}

TEST(ControllerModeGate, RejectsDirectActiveToActiveSwitch) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  ASSERT_TRUE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                             msg::ModeCommand::PROFILE_PRECISION, 1)));
  EXPECT_FALSE(gate.apply(req(msg::ModeCommand::MODE_DIRECT_CARTESIAN,
                              msg::ModeCommand::PROFILE_PRECISION, 2)));
  EXPECT_FALSE(gate.lastRequestOk());
  EXPECT_TRUE(gate.engaged());  // still FORCE_COMPLIANCE
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::FORCE_COMPLIANCE);
}

TEST(ControllerModeGate, UnknownValuesLeaveStateUntouched) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_FALSE(gate.apply(req(99, msg::ModeCommand::PROFILE_DRAG, 1)));
  EXPECT_FALSE(gate.lastRequestOk());
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::IDLE);
  EXPECT_EQ(gate.snapshot().profile, sfc::Profile::PRECISION);
}

TEST(ControllerModeGate, TwoEngagedModesAndForceIdle) {
  ControllerModeGate gate(sfc::ControlMode::DIRECT_CARTESIAN,
                          sfc::ControlMode::CALIBRATION);
  EXPECT_TRUE(gate.apply(req(msg::ModeCommand::MODE_DIRECT_CARTESIAN,
                             msg::ModeCommand::PROFILE_PRECISION, 1)));
  EXPECT_FALSE(gate.apply(req(msg::ModeCommand::MODE_IDLE,
                              msg::ModeCommand::PROFILE_PRECISION, 2)));
  EXPECT_FALSE(gate.engaged());
  EXPECT_TRUE(gate.apply(req(msg::ModeCommand::MODE_CALIBRATION,
                             msg::ModeCommand::PROFILE_PRECISION, 3)));
  EXPECT_TRUE(gate.engaged());
  gate.forceIdle();
  EXPECT_FALSE(gate.engaged());
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::IDLE);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `controller_mode_gate.h`。

- [ ] **Step 3: 写实现**

`ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/controller_mode_gate.h`:

```cpp
#pragma once

#include <cstdint>

#include "soft_force_control_core/mode_manager_core.h"
#include "soft_robot_controllers/mode_bridge.h"

namespace soft_robot_controllers {

// Latest-value mode request handed from the subscriber thread to update()
// through a RealtimeBuffer. seq == 0 means "no request received yet";
// the producer side allocates sequence numbers starting at 1.
struct ModeRequest {
  std::uint8_t mode{soft_robot_msgs::ModeCommand::MODE_IDLE};
  std::uint8_t profile{soft_robot_msgs::ModeCommand::PROFILE_PRECISION};
  std::uint64_t seq{0};
};

// Self-initialising RealtimeBuffer payload for the /kuka/rsi/state fault
// flag shared by both controllers (a bare bool would be indeterminate in
// the buffer's default-constructed slots).
struct FaultFlag {
  bool fault{false};
};

// Per-controller mode gate: applies each ModeRequest to an embedded
// ModeManagerCore exactly once (deduplicated by sequence number) and
// reports the rising edge of entering this controller's engaged mode(s).
// Both controllers embed their own gate and see the same command stream;
// ModeManagerCore's deterministic rules keep their conclusions identical
// (decision 3). Pure logic, allocation-free, RT-safe.
class ControllerModeGate {
 public:
  ControllerModeGate(sfc::ControlMode engaged_a, sfc::ControlMode engaged_b);
  explicit ControllerModeGate(sfc::ControlMode engaged)
      : ControllerModeGate(engaged, engaged) {}

  // Applies req if its sequence number is new. Returns true exactly when
  // this call transitioned the gate from disengaged to engaged.
  bool apply(const ModeRequest& req);
  bool engaged() const;
  void forceIdle();  // controller stopping(): drop back to IDLE
  sfc::ModeSnapshot snapshot() const { return mgr_.snapshot(); }
  bool lastRequestOk() const { return last_ok_; }

 private:
  sfc::ModeManagerCore mgr_;
  sfc::ControlMode engaged_a_;
  sfc::ControlMode engaged_b_;
  std::uint64_t last_seq_{0};
  bool last_ok_{true};
};

}  // namespace soft_robot_controllers
```

`ros_ws/src/soft_robot_controllers/src/controller_mode_gate.cpp`:

```cpp
#include "soft_robot_controllers/controller_mode_gate.h"

namespace soft_robot_controllers {

ControllerModeGate::ControllerModeGate(sfc::ControlMode engaged_a,
                                       sfc::ControlMode engaged_b)
    : engaged_a_(engaged_a), engaged_b_(engaged_b) {}

bool ControllerModeGate::engaged() const {
  const sfc::ControlMode m = mgr_.snapshot().mode;
  return m == engaged_a_ || m == engaged_b_;
}

void ControllerModeGate::forceIdle() {
  mgr_.requestMode(sfc::ControlMode::IDLE);  // always allowed
}

bool ControllerModeGate::apply(const ModeRequest& req) {
  if (req.seq == 0 || req.seq == last_seq_) return false;
  last_seq_ = req.seq;

  sfc::ControlMode mode = sfc::ControlMode::IDLE;
  sfc::Profile profile = sfc::Profile::PRECISION;
  if (!toControlMode(req.mode, mode) || !toProfile(req.profile, profile)) {
    last_ok_ = false;  // unknown wire value: ignore the request entirely
    return false;
  }

  const bool was_engaged = engaged();
  // Profile changes are only legal while IDLE (ModeManagerCore rule).
  // Apply the profile before the mode switch when leaving IDLE and after
  // it when entering IDLE, so a single message can select profile and
  // mode together (decision 12).
  bool profile_applied = false;
  if (mgr_.snapshot().mode == sfc::ControlMode::IDLE) {
    profile_applied = mgr_.setProfile(profile);
  }
  last_ok_ = mgr_.requestMode(mode);
  if (last_ok_ && !profile_applied &&
      mgr_.snapshot().mode == sfc::ControlMode::IDLE) {
    mgr_.setProfile(profile);
  }
  return last_ok_ && !was_engaged && engaged();
}

}  // namespace soft_robot_controllers
```

- [ ] **Step 4: CMake 增量**

`CMakeLists.txt`:`include_directories` 之后加库目标,`catkin_package` 加 `LIBRARIES ${PROJECT_NAME}`:

```cmake
# catkin_package(...) 改为:
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp controller_interface hardware_interface
                 realtime_tools actionlib pluginlib geometry_msgs
                 soft_force_control_core kuka_rsi_hw_interface
                 soft_robot_msgs
)

# include_directories(...) 之后追加:
add_library(${PROJECT_NAME}
  src/controller_mode_gate.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
```

测试段追加:

```cmake
  catkin_add_gtest(test_controller_mode_gate test/test_controller_mode_gate.cpp)
  target_link_libraries(test_controller_mode_gate ${PROJECT_NAME}
                        ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_controllers/test_controller_mode_gate
```

预期:`[  PASSED  ] 7 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers && \
git commit -m "feat(controllers): add ControllerModeGate with per-sequence apply and entered edge (Plan 3 Task 2)"
```

---

### Task 3: `ForceComplianceCore` 基础管线(滤波 → 补偿 → 导纳 → 限幅)

**Files:**
- Create: `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_core.h`
- Create: `ros_ws/src/soft_robot_controllers/src/force_compliance_core.cpp`
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_controllers/test/test_force_compliance_core.cpp`

**Interfaces:**
- Consumes(Plan 1,签名逐一核对过):
  - `sfc::ForceTorqueFilter(double cutoff_hz)` / `Wrench filter(const Wrench&, double dt)` / `void reset()`
  - `sfc::ToolGravityCompensator::setParams(const PayloadParams&)` / `Wrench compensate(const Wrench&, double a_deg, double b_deg, double c_deg) const` / `void absorbResidual(const Wrench&)` / `const PayloadParams& params() const`
  - `sfc::ComplianceLaw::compute(const Wrench&, const ComplianceParams&, double dt)` / `void reset()`
  - `sfc::SafetyLimiter::apply(const CartesianCorrection&, const Wrench&, const SafetyParams&) const` → `SafetyResult{correction, hard_cutoff, saturated}`
  - `sfc::AdaptiveDeadband::start(window_s, force_margin_n, torque_margin_nm)` / `bool update(const Wrench&, double dt)` / `active()/forceDeadband()/torqueDeadband()`(Task 4 用)
  - `sfc::AutoReTare::setReference(a,b,c)` / `shouldTare(state, compensated, force_db, torque_db, params) const`(Task 4 用)
  - `sfc::OrientationMotionCore::setGoal/cancel/status/update(state, dt)`(Task 5 用)
  - `sfc::angularDistanceDeg`(`rotation.h`)
- Produces(Task 4/5/7/9 消费):
  - `struct ForceComplianceParams`:profile 级全参数(滤波带宽、导纳增益组、固定/自适应死区、re-tare、安全限幅、payload、wrench 超时)。
  - `struct ComplianceInput { sfc::CartesianState state; sfc::Wrench raw; double wrench_age_s; bool wrench_valid; }`
  - `struct ComplianceOutput { sfc::CartesianCorrection correction; bool ramp_active, wrench_timeout, hard_cutoff, saturated, tared; sfc::Wrench compensated; }`
  - `class ForceComplianceCore`:`configure(params)`(**重建滤波器**)、`activate(start_pose)`、`update(in, dt)`、`motion()`(内嵌 `OrientationMotionCore` 引用)、`forceDeadband()/torqueDeadband()`。
- 本 Task 实现除 ramp/retare/motion 之外的主管线(那三条路径的字段与成员在本 Task 一并落盘,行为由 Task 4/5 的测试驱动补全——本 Task 的实现即含全部代码,Task 4/5 只加测试;见各 Task 说明)。为保持"每 Task 可独立验收",本 Task 的测试仅覆盖基础管线。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/soft_robot_controllers/test/test_force_compliance_core.cpp`:

```cpp
#include <gtest/gtest.h>

#include "soft_robot_controllers/force_compliance_core.h"

using soft_robot_controllers::ComplianceInput;
using soft_robot_controllers::ComplianceOutput;
using soft_robot_controllers::ForceComplianceCore;
using soft_robot_controllers::ForceComplianceParams;

namespace {

// PRECISION-flavoured parameters with filtering disabled and simple gains
// so expected corrections are easy to compute by hand.
ForceComplianceParams baseParams() {
  ForceComplianceParams p;
  p.filter_cutoff_hz = 0.0;  // pass-through (sfc filter semantics)
  p.compliance.translation.gain = 1.0;    // (mm/s)/N
  p.compliance.translation.max_speed = 50.0;
  p.compliance.translation.max_accel = 0.0;  // rate limiting off
  p.compliance.rotation.gain = 0.1;       // (deg/s)/Nm
  p.compliance.rotation.max_speed = 5.0;
  p.compliance.rotation.max_accel = 0.0;
  p.compliance.speed_scale = 1.0;
  p.fixed_force_deadband_n = 30.0;   // spec 7.3 PRECISION defaults
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

ForceComplianceCore makeCore(const ForceComplianceParams& p) {
  ForceComplianceCore core;
  core.configure(p);
  core.activate(sfc::CartesianState{});
  return core;
}

ComplianceInput freshInput() {
  ComplianceInput in;
  in.wrench_valid = true;
  in.wrench_age_s = 0.0;
  return in;
}

}  // namespace

TEST(FccCore, InvalidWrenchOutputsZeroWithTimeoutFlag) {
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.wrench_valid = false;
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_TRUE(out.wrench_timeout);
  EXPECT_EQ(out.correction.x, 0.0);
  EXPECT_EQ(out.correction.a, 0.0);
}

TEST(FccCore, StaleWrenchOutputsZeroWithTimeoutFlag) {
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.raw.fz = 100.0;
  in.wrench_age_s = 0.02;  // > 0.012 (about 3 RSI cycles, spec section 8)
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_TRUE(out.wrench_timeout);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccCore, FixedDeadbandSuppressesSmallForces) {
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.raw.fz = 20.0;  // below the 30 N PRECISION deadband
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_FALSE(out.wrench_timeout);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccCore, ForceAboveDeadbandProducesTranslation) {
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.raw.fz = 40.0;  // e = 10 N -> v = 10 mm/s -> 0.04 mm per 4 ms cycle
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_NEAR(out.correction.z, 0.04, 1e-12);
  EXPECT_EQ(out.correction.x, 0.0);
  EXPECT_FALSE(out.saturated);
}

TEST(FccCore, CorrectionScalesWithMeasuredDt) {
  // Plan 2 follow-up 1: dt comes from the caller every cycle. Doubling dt
  // must double the per-cycle correction (below all speed limits).
  ComplianceInput in = freshInput();
  in.raw.fz = 40.0;
  ForceComplianceCore c1 = makeCore(baseParams());
  ForceComplianceCore c2 = makeCore(baseParams());
  const double z1 = c1.update(in, 0.004).correction.z;
  const double z2 = c2.update(in, 0.008).correction.z;
  EXPECT_NEAR(z2, 2.0 * z1, 1e-12);
}

TEST(FccCore, HardCutoffIsStrictlyGreaterThanCeiling) {
  // Plan 1 follow-up 4: exactly 500 N does NOT trip the cutoff.
  ForceComplianceCore core = makeCore(baseParams());
  ComplianceInput in = freshInput();
  in.raw.fz = 500.0;
  ComplianceOutput out = core.update(in, 0.004);
  EXPECT_FALSE(out.hard_cutoff);
  // e = 470 N -> v clamps to 50 mm/s -> 0.2 mm per cycle.
  EXPECT_NEAR(out.correction.z, 0.2, 1e-12);

  ForceComplianceCore core2 = makeCore(baseParams());
  in.raw.fz = 500.001;
  out = core2.update(in, 0.004);
  EXPECT_TRUE(out.hard_cutoff);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccCore, SafetyLimiterClampsPerCycleCorrection) {
  ForceComplianceParams p = baseParams();
  p.compliance.translation.max_speed = 500.0;  // allow a huge velocity
  ForceComplianceCore core = makeCore(p);
  ComplianceInput in = freshInput();
  in.raw.fz = 330.0;  // e = 300 -> v = 300 mm/s -> 1.2 mm >> 0.5 mm clamp
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_NEAR(out.correction.z, 0.5, 1e-12);
  EXPECT_TRUE(out.saturated);
  EXPECT_FALSE(out.hard_cutoff);
}

TEST(FccCore, GravityCompensationCancelsToolWeight) {
  ForceComplianceParams p = baseParams();
  p.payload.gravity_n = 10.0;  // tool weighs 10 N, CoM at the sensor origin
  ForceComplianceCore core = makeCore(p);
  ComplianceInput in = freshInput();
  // Flange at identity orientation: the sensor sees the tool weight as
  // -10 N along Z. Compensation must cancel it exactly.
  in.raw.fz = -10.0;
  const ComplianceOutput out = core.update(in, 0.004);
  EXPECT_NEAR(out.compensated.fz, 0.0, 1e-9);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccCore, FilterSmoothsStepInput) {
  ForceComplianceParams p = baseParams();
  p.filter_cutoff_hz = 10.0;
  p.fixed_force_deadband_n = 0.0;
  ForceComplianceCore core = makeCore(p);
  ComplianceInput zero = freshInput();
  core.update(zero, 0.004);  // first sample initialises the filter at 0
  ComplianceInput step = freshInput();
  step.raw.fz = 100.0;
  const ComplianceOutput out = core.update(step, 0.004);
  // One low-pass step must land strictly between 0 and the raw value.
  EXPECT_GT(out.correction.z, 0.0);
  EXPECT_LT(out.correction.z, 100.0 * 1.0 * 0.004);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `force_compliance_core.h`。

- [ ] **Step 3: 写实现(含 Task 4/5 将测试的 ramp/retare/motion 路径,一次落盘完整)**

`ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_core.h`:

```cpp
#pragma once

#include "soft_force_control_core/adaptive_deadband.h"
#include "soft_force_control_core/auto_retare.h"
#include "soft_force_control_core/compliance_law.h"
#include "soft_force_control_core/force_torque_filter.h"
#include "soft_force_control_core/orientation_motion_core.h"
#include "soft_force_control_core/safety_limiter.h"
#include "soft_force_control_core/tool_gravity_compensator.h"
#include "soft_force_control_core/types.h"

namespace soft_robot_controllers {

// One profile's worth of FORCE_COMPLIANCE parameters (spec sections
// 7.2-7.5, 12.1). The controller shell keeps one instance per profile
// (DRAG / PRECISION) and re-configures the core on profile entry.
struct ForceComplianceParams {
  double filter_cutoff_hz{10.0};
  sfc::ComplianceParams compliance;      // gains/speed caps; deadbands below
  double fixed_force_deadband_n{30.0};   // PRECISION defaults (spec 7.3)
  double fixed_torque_deadband_nm{4.0};
  bool adaptive_deadband{false};         // DRAG startup ramp (spec 7.4)
  double ramp_window_s{2.0};
  double ramp_force_margin_n{5.0};
  double ramp_torque_margin_nm{1.0};
  sfc::AutoReTareParams retare;          // DRAG auto re-tare (spec 7.5)
  double retare_rearm_factor{2.0};       // re-arm at factor * tol (decision 9)
  sfc::SafetyParams safety;              // per-cycle clamp + hard cutoff
  sfc::PayloadParams payload;
  double wrench_timeout_s{0.012};        // ~3 RSI cycles (spec section 8)
};

struct ComplianceInput {
  sfc::CartesianState state;   // current TCP pose from the hardware handle
  sfc::Wrench raw;             // latest SRI sample from the realtime buffer
  double wrench_age_s{1e9};    // now - sample stamp
  bool wrench_valid{false};    // false until the first sample arrives
};

struct ComplianceOutput {
  sfc::CartesianCorrection correction;  // zero-initialised
  bool ramp_active{false};
  bool wrench_timeout{false};
  bool hard_cutoff{false};
  bool saturated{false};
  bool tared{false};
  sfc::Wrench compensated;              // diagnostics (post-compensation)
};

// Realtime FORCE_COMPLIANCE pipeline (spec 7.1): filter -> gravity
// compensation -> DRAG startup ramp -> edge-triggered auto re-tare ->
// admittance law -> sum with the embedded orientation motion ->
// SafetyLimiter. Pure logic: no ROS, no allocation, no blocking. dt is
// the measured controller period, passed in every cycle.
class ForceComplianceCore {
 public:
  // Stores params, REBUILDS the low-pass filter at the profile cutoff
  // (Plan 1 follow-up 3) and loads the payload into the compensator.
  void configure(const ForceComplianceParams& p);

  // Entering servo with this profile: reset filter/law, record the auto
  // re-tare reference orientation (disarmed until the TCP leaves it),
  // start the adaptive deadband ramp if the profile uses it.
  void activate(const sfc::CartesianState& start);

  ComplianceOutput update(const ComplianceInput& in, double dt);

  // Embedded orientation motion path; its per-cycle output is summed with
  // the compliance correction BEFORE the SafetyLimiter (Plan 1 follow-up
  // 2). No ROS entry point in v1 (decision 4).
  sfc::OrientationMotionCore& motion() { return motion_; }

  double forceDeadband() const { return force_deadband_; }
  double torqueDeadband() const { return torque_deadband_; }
  const sfc::PayloadParams& payload() const { return compensator_.params(); }

 private:
  ForceComplianceParams params_;
  sfc::ForceTorqueFilter filter_{10.0};
  sfc::ToolGravityCompensator compensator_;
  sfc::ComplianceLaw law_;
  sfc::SafetyLimiter limiter_;
  sfc::AdaptiveDeadband ramp_;
  sfc::AutoReTare retare_;
  sfc::OrientationMotionCore motion_;
  double ref_a_{0}, ref_b_{0}, ref_c_{0};  // re-tare reference copy
  bool tare_armed_{false};
  double force_deadband_{0};
  double torque_deadband_{0};
};

}  // namespace soft_robot_controllers
```

`ros_ws/src/soft_robot_controllers/src/force_compliance_core.cpp`:

```cpp
#include "soft_robot_controllers/force_compliance_core.h"

#include "soft_force_control_core/rotation.h"

namespace soft_robot_controllers {

void ForceComplianceCore::configure(const ForceComplianceParams& p) {
  params_ = p;
  // Rebuild-and-reset: cutoff is constructor-only on the sfc filter, so a
  // profile switch replaces the object (stack assignment, no allocation).
  filter_ = sfc::ForceTorqueFilter(p.filter_cutoff_hz);
  compensator_.setParams(p.payload);
  law_.reset();
  motion_.cancel();
  ramp_ = sfc::AdaptiveDeadband{};
  tare_armed_ = false;
  force_deadband_ = p.fixed_force_deadband_n;
  torque_deadband_ = p.fixed_torque_deadband_nm;
}

void ForceComplianceCore::activate(const sfc::CartesianState& start) {
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

ComplianceOutput ForceComplianceCore::update(const ComplianceInput& in,
                                             double dt) {
  ComplianceOutput out;
  if (!in.wrench_valid || in.wrench_age_s > params_.wrench_timeout_s) {
    out.wrench_timeout = true;
    law_.reset();  // never resume with stale rate-limiter state
    return out;    // zero correction (spec 7.1 timeout rule)
  }

  const sfc::Wrench filtered = filter_.filter(in.raw, dt);
  out.compensated =
      compensator_.compensate(filtered, in.state.a, in.state.b, in.state.c);

  if (ramp_.active()) {
    // Startup ramp (spec 7.4): hold zero output while learning the
    // residual. The hard cutoff stays armed throughout.
    out.ramp_active = true;
    const bool still_ramping = ramp_.update(out.compensated, dt);
    if (!still_ramping) {
      force_deadband_ = ramp_.forceDeadband();
      torque_deadband_ = ramp_.torqueDeadband();
    }
    const sfc::SafetyResult guard = limiter_.apply(
        sfc::CartesianCorrection{}, out.compensated, params_.safety);
    out.hard_cutoff = guard.hard_cutoff;
    return out;
  }

  // Auto re-tare, edge-triggered (Plan 1 follow-up 1): re-arm only after
  // the TCP has clearly left the reference orientation (hysteresis factor,
  // decision 9); fire at most once per return, then disarm.
  const double dist = sfc::angularDistanceDeg(
      ref_a_, ref_b_, ref_c_, in.state.a, in.state.b, in.state.c);
  if (dist >
      params_.retare.orientation_tol_deg * params_.retare_rearm_factor) {
    tare_armed_ = true;
  }
  if (tare_armed_ &&
      retare_.shouldTare(in.state, out.compensated, force_deadband_,
                         torque_deadband_, params_.retare)) {
    compensator_.absorbResidual(out.compensated);
    tare_armed_ = false;
    out.tared = true;
    // Recompute with the updated bias: the residual is now absorbed.
    out.compensated =
        compensator_.compensate(filtered, in.state.a, in.state.b, in.state.c);
  }

  sfc::ComplianceParams law_params = params_.compliance;
  law_params.translation.deadband = force_deadband_;
  law_params.rotation.deadband = torque_deadband_;
  const sfc::CartesianCorrection compliance =
      law_.compute(out.compensated, law_params, dt);
  const sfc::CartesianCorrection motion = motion_.update(in.state, dt);

  // Plan 1 follow-up 2: sum both correction paths BEFORE the limiter so
  // the per-cycle clamp applies to the combined magnitude.
  sfc::CartesianCorrection sum;
  sum.x = compliance.x + motion.x;
  sum.y = compliance.y + motion.y;
  sum.z = compliance.z + motion.z;
  sum.a = compliance.a + motion.a;
  sum.b = compliance.b + motion.b;
  sum.c = compliance.c + motion.c;

  const sfc::SafetyResult res =
      limiter_.apply(sum, out.compensated, params_.safety);
  out.correction = res.correction;
  out.hard_cutoff = res.hard_cutoff;
  out.saturated = res.saturated;
  if (res.hard_cutoff) law_.reset();
  return out;
}

}  // namespace soft_robot_controllers
```

- [ ] **Step 4: CMake 增量**

库源列表加 `src/force_compliance_core.cpp`;测试段追加:

```cmake
  catkin_add_gtest(test_force_compliance_core test/test_force_compliance_core.cpp)
  target_link_libraries(test_force_compliance_core ${PROJECT_NAME}
                        ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_controllers/test_force_compliance_core
```

预期:`[  PASSED  ] 9 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers && \
git commit -m "feat(controllers): add ForceComplianceCore realtime pipeline (Plan 3 Task 3)"
```

---

### Task 4: 自适应死区 ramp + AutoReTare 边沿触发(行为固化测试)

**Files:**
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_controllers/test/test_fcc_adaptive_retare.cpp`

**Interfaces:**
- Consumes: Task 3 `ForceComplianceCore` 全量实现(ramp/retare 代码已落盘)。
- TDD 说明:同 Plan 2 Task 9 惯例——本 Task 不引入新实现代码,"失败态"=测试文件先于 CMake 条目;测试将 Plan 1 跟进 1(边沿触发)、跟进 3(滤波重建)与规格 §7.4-7.5 的行为逐条钉死。若有用例失败,修 Task 3 的实现,不修测试语义。

- [ ] **Step 1: 写测试**

`ros_ws/src/soft_robot_controllers/test/test_fcc_adaptive_retare.cpp`:

```cpp
#include <gtest/gtest.h>

#include "soft_robot_controllers/force_compliance_core.h"

using soft_robot_controllers::ComplianceInput;
using soft_robot_controllers::ComplianceOutput;
using soft_robot_controllers::ForceComplianceCore;
using soft_robot_controllers::ForceComplianceParams;

namespace {

constexpr double kDt = 0.004;

// DRAG-flavoured parameters with a 5-cycle ramp window for fast tests.
ForceComplianceParams dragParams() {
  ForceComplianceParams p;
  p.filter_cutoff_hz = 0.0;
  p.compliance.translation.gain = 1.0;
  p.compliance.translation.max_speed = 50.0;
  p.compliance.translation.max_accel = 0.0;
  p.compliance.rotation.gain = 0.1;
  p.compliance.rotation.max_speed = 5.0;
  p.compliance.rotation.max_accel = 0.0;
  p.compliance.speed_scale = 1.0;
  p.adaptive_deadband = true;
  p.ramp_window_s = 0.02;  // 5 cycles at 4 ms
  p.ramp_force_margin_n = 5.0;   // legacy FTLimSet(5+FSum, 1+TSum)
  p.ramp_torque_margin_nm = 1.0;
  p.retare.enabled = true;
  p.retare.orientation_tol_deg = 1.0;
  p.retare_rearm_factor = 2.0;
  p.wrench_timeout_s = 0.012;
  return p;
}

// Fixed-deadband variant used by the re-tare tests (ramp off so the
// deadbands are deterministic from the first cycle).
ForceComplianceParams retareParams() {
  ForceComplianceParams p = dragParams();
  p.adaptive_deadband = false;
  p.fixed_force_deadband_n = 30.0;
  p.fixed_torque_deadband_nm = 4.0;
  return p;
}

ComplianceInput inputAt(double a_deg, double fx) {
  ComplianceInput in;
  in.wrench_valid = true;
  in.wrench_age_s = 0.0;
  in.state.a = a_deg;
  in.raw.fx = fx;
  return in;
}

}  // namespace

TEST(FccRamp, HoldsZeroForWholeWindow) {
  ForceComplianceCore core;
  core.configure(dragParams());
  core.activate(sfc::CartesianState{});
  for (int i = 0; i < 5; ++i) {
    const ComplianceOutput out = core.update(inputAt(0.0, 8.0), kDt);
    EXPECT_TRUE(out.ramp_active) << "cycle " << i;
    EXPECT_EQ(out.correction.x, 0.0) << "cycle " << i;
  }
  // Window over: residual 8 N + margin 5 N = 13 N deadband.
  const ComplianceOutput out = core.update(inputAt(0.0, 20.0), kDt);
  EXPECT_FALSE(out.ramp_active);
  EXPECT_NEAR(out.correction.x, (20.0 - 13.0) * 1.0 * kDt, 1e-12);
}

TEST(FccRamp, LearnsResidualPlusMargin) {
  ForceComplianceCore core;
  core.configure(dragParams());
  core.activate(sfc::CartesianState{});
  for (int i = 0; i < 5; ++i) core.update(inputAt(0.0, 2.0), kDt);
  EXPECT_NEAR(core.forceDeadband(), 7.0, 1e-9);   // 2 + 5
  EXPECT_NEAR(core.torqueDeadband(), 1.0, 1e-9);  // 0 + 1
  const ComplianceOutput out = core.update(inputAt(0.0, 8.0), kDt);
  EXPECT_NEAR(out.correction.x, (8.0 - 7.0) * kDt, 1e-12);
}

TEST(FccRamp, HardCutoffStaysArmedDuringRamp) {
  ForceComplianceCore core;
  core.configure(dragParams());
  core.activate(sfc::CartesianState{});
  ComplianceInput in = inputAt(0.0, 0.0);
  in.raw.fz = 600.0;
  const ComplianceOutput out = core.update(in, kDt);
  EXPECT_TRUE(out.ramp_active);
  EXPECT_TRUE(out.hard_cutoff);
  EXPECT_EQ(out.correction.z, 0.0);
}

TEST(FccRetare, NeverFiresWithoutLeavingReference) {
  // Plan 1 follow-up 1: the stateless shouldTare predicate would be true
  // every cycle here; the edge trigger must keep it from ever firing.
  ForceComplianceCore core;
  core.configure(retareParams());
  core.activate(sfc::CartesianState{});
  for (int i = 0; i < 10; ++i) {
    const ComplianceOutput out = core.update(inputAt(0.0, 5.0), kDt);
    EXPECT_FALSE(out.tared) << "cycle " << i;
  }
  EXPECT_EQ(core.payload().bias.fx, 0.0);  // bias untouched: no drift
}

TEST(FccRetare, FiresExactlyOncePerReturn) {
  ForceComplianceCore core;
  core.configure(retareParams());
  core.activate(sfc::CartesianState{});
  for (int i = 0; i < 3; ++i) core.update(inputAt(0.0, 5.0), kDt);
  // Leave the reference beyond rearm_factor * tol = 2 deg.
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(core.update(inputAt(5.0, 5.0), kDt).tared);
  }
  // Return: exactly one tare, then quiet.
  EXPECT_TRUE(core.update(inputAt(0.0, 5.0), kDt).tared);
  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(core.update(inputAt(0.0, 5.0), kDt).tared);
  }
}

TEST(FccRetare, AbsorbsResidualIntoBias) {
  ForceComplianceCore core;
  core.configure(retareParams());
  core.activate(sfc::CartesianState{});
  core.update(inputAt(5.0, 5.0), kDt);  // arm
  const ComplianceOutput out = core.update(inputAt(0.0, 5.0), kDt);
  EXPECT_TRUE(out.tared);
  EXPECT_NEAR(core.payload().bias.fx, 5.0, 1e-9);
  EXPECT_NEAR(out.compensated.fx, 0.0, 1e-9);  // recomputed after absorb
}

TEST(FccRetare, ReArmsAfterLeavingAgain) {
  ForceComplianceCore core;
  core.configure(retareParams());
  core.activate(sfc::CartesianState{});
  core.update(inputAt(5.0, 5.0), kDt);
  ASSERT_TRUE(core.update(inputAt(0.0, 5.0), kDt).tared);  // bias.fx = 5
  core.update(inputAt(5.0, 3.0), kDt);  // leave again
  // Second return: the residual is the COMPENSATED value 3 - 5 = -2, so
  // the bias converges onto the current raw force: 5 + (-2) = 3.
  EXPECT_TRUE(core.update(inputAt(0.0, 3.0), kDt).tared);
  EXPECT_NEAR(core.payload().bias.fx, 3.0, 1e-9);
}

TEST(FccRetare, DisabledProfileNeverTares) {
  ForceComplianceParams p = retareParams();
  p.retare.enabled = false;  // PRECISION profile behavior (spec 7.3)
  ForceComplianceCore core;
  core.configure(p);
  core.activate(sfc::CartesianState{});
  core.update(inputAt(5.0, 5.0), kDt);
  EXPECT_FALSE(core.update(inputAt(0.0, 5.0), kDt).tared);
  EXPECT_EQ(core.payload().bias.fx, 0.0);
}

TEST(FccProfile, ReconfigureRebuildsAndResetsFilter) {
  // Plan 1 follow-up 3: profile switches rebuild the filter and reset its
  // state. After configure()+activate(), the first sample re-initialises
  // the low-pass (sfc filter semantics), so a step passes through whole.
  ForceComplianceParams p = retareParams();
  p.filter_cutoff_hz = 10.0;
  p.fixed_force_deadband_n = 0.0;
  ForceComplianceCore core;
  core.configure(p);
  core.activate(sfc::CartesianState{});
  core.update(inputAt(0.0, 0.0), kDt);    // filter state pinned at 0
  const double smoothed = core.update(inputAt(0.0, 100.0), kDt).correction.x;
  EXPECT_LT(smoothed, 100.0 * kDt);       // low-pass engaged

  core.configure(p);                      // profile re-entry
  core.activate(sfc::CartesianState{});
  const double fresh = core.update(inputAt(0.0, 100.0), kDt).correction.x;
  EXPECT_NEAR(fresh, 100.0 * kDt, 1e-12); // first sample after reset
}
```

- [ ] **Step 2: CMake 增量**

```cmake
  catkin_add_gtest(test_fcc_adaptive_retare test/test_fcc_adaptive_retare.cpp)
  target_link_libraries(test_fcc_adaptive_retare ${PROJECT_NAME}
                        ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 3: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_controllers/test_fcc_adaptive_retare
```

预期:`[  PASSED  ] 9 tests.`(失败则修 Task 3 实现。)

- [ ] **Step 4: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers && \
git commit -m "test(controllers): pin adaptive deadband ramp and edge-triggered auto re-tare behavior (Plan 3 Task 4)"
```

---

### Task 5: 顺应 + 姿态运动两路求和先于 SafetyLimiter(行为固化测试)

**Files:**
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_controllers/test/test_fcc_orientation_sum.cpp`

**Interfaces:**
- Consumes: Task 3 `ForceComplianceCore`(含内嵌 `motion()`)。
- TDD 说明:同 Task 4,测试专职固化 Plan 1 跟进 2:两路 `CartesianCorrection` **先求和再过 SafetyLimiter**——用"各自 0.03 deg 不饱和、合成 0.06 deg 被夹到 0.05"的构造证明 clamp 作用于合成量而非分量。

- [ ] **Step 1: 写测试**

`ros_ws/src/soft_robot_controllers/test/test_fcc_orientation_sum.cpp`:

```cpp
#include <gtest/gtest.h>

#include "soft_robot_controllers/force_compliance_core.h"

using soft_robot_controllers::ComplianceInput;
using soft_robot_controllers::ComplianceOutput;
using soft_robot_controllers::ForceComplianceCore;
using soft_robot_controllers::ForceComplianceParams;

namespace {

constexpr double kDt = 0.004;

ForceComplianceParams sumParams() {
  ForceComplianceParams p;
  p.filter_cutoff_hz = 0.0;
  p.compliance.rotation.gain = 1.0;       // (deg/s)/Nm
  p.compliance.rotation.max_speed = 10.0;
  p.compliance.rotation.max_accel = 0.0;
  p.compliance.translation.gain = 1.0;
  p.compliance.translation.max_speed = 50.0;
  p.compliance.translation.max_accel = 0.0;
  p.compliance.speed_scale = 1.0;
  p.fixed_force_deadband_n = 0.0;
  p.fixed_torque_deadband_nm = 0.0;
  p.adaptive_deadband = false;
  p.retare.enabled = false;
  p.safety.max_corr_rot = 0.05;  // deg per cycle
  return p;
}

// Orientation goal producing exactly 7.5 deg/s -> 0.03 deg per cycle.
sfc::MotionGoal motionGoal() {
  sfc::MotionGoal g;
  g.a = 10.0;             // far from the start pose (a = 0)
  g.max_speed_dps = 7.5;  // p_gain * error saturates at this cap
  g.p_gain = 1.0;
  g.tol_deg = 0.1;
  g.hold_s = 0.2;
  g.timeout_s = 30.0;
  return g;
}

ComplianceInput wrenchTz(double tz) {
  ComplianceInput in;
  in.wrench_valid = true;
  in.wrench_age_s = 0.0;
  in.raw.tz = tz;  // maps to the A axis (sfc law: tz -> a)
  return in;
}

}  // namespace

TEST(FccSum, MotionAloneProducesRotation) {
  ForceComplianceCore core;
  core.configure(sumParams());
  core.activate(sfc::CartesianState{});
  core.motion().setGoal(motionGoal());
  const ComplianceOutput out = core.update(wrenchTz(0.0), kDt);
  EXPECT_NEAR(out.correction.a, 0.03, 1e-12);
  EXPECT_FALSE(out.saturated);
  EXPECT_EQ(core.motion().status(), sfc::MotionStatus::RUNNING);
}

TEST(FccSum, ComplianceAloneStaysBelowClamp) {
  ForceComplianceCore core;
  core.configure(sumParams());
  core.activate(sfc::CartesianState{});
  // 7.5 Nm * 1.0 (deg/s)/Nm = 7.5 deg/s -> 0.03 deg per cycle.
  const ComplianceOutput out = core.update(wrenchTz(7.5), kDt);
  EXPECT_NEAR(out.correction.a, 0.03, 1e-12);
  EXPECT_FALSE(out.saturated);
}

TEST(FccSum, CombinedPathsAreClampedTogether) {
  // Plan 1 follow-up 2: 0.03 (compliance) + 0.03 (motion) = 0.06 deg must
  // be clamped to 0.05 by ONE limiter pass over the sum. Clamping each
  // path separately would let 0.06 deg through unclamped.
  ForceComplianceCore core;
  core.configure(sumParams());
  core.activate(sfc::CartesianState{});
  core.motion().setGoal(motionGoal());
  const ComplianceOutput out = core.update(wrenchTz(7.5), kDt);
  EXPECT_NEAR(out.correction.a, 0.05, 1e-12);
  EXPECT_TRUE(out.saturated);
}

TEST(FccSum, HardCutoffZeroesTheCombinedOutput) {
  ForceComplianceCore core;
  core.configure(sumParams());
  core.activate(sfc::CartesianState{});
  core.motion().setGoal(motionGoal());
  ComplianceInput in = wrenchTz(7.5);
  in.raw.fz = 600.0;  // above the 500 N ceiling
  const ComplianceOutput out = core.update(in, kDt);
  EXPECT_TRUE(out.hard_cutoff);
  EXPECT_EQ(out.correction.a, 0.0);  // motion path zeroed too
  EXPECT_EQ(out.correction.z, 0.0);
}
```

- [ ] **Step 2: CMake 增量**

```cmake
  catkin_add_gtest(test_fcc_orientation_sum test/test_fcc_orientation_sum.cpp)
  target_link_libraries(test_fcc_orientation_sum ${PROJECT_NAME}
                        ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 3: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_controllers/test_fcc_orientation_sum
```

预期:`[  PASSED  ] 4 tests.`

- [ ] **Step 4: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers && \
git commit -m "test(controllers): pin sum-before-limiter composition of compliance and motion corrections (Plan 3 Task 5)"
```

---

### Task 6: `DirectCorrectionCore`(stream 直通 + goal 模式命令源)

**Files:**
- Create: `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/direct_correction_core.h`
- Create: `ros_ws/src/soft_robot_controllers/src/direct_correction_core.cpp`
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_controllers/test/test_direct_correction_core.cpp`

**Interfaces:**
- Consumes: `sfc::OrientationMotionCore`、`sfc::SafetyLimiter`、`sfc::angularDistanceDeg`。
- Produces(Task 8/9 消费):
  - `struct DirectCorrectionParams { double stream_timeout_s; sfc::SafetyParams safety; }`
  - `struct StreamCommand { sfc::CartesianCorrection correction; double stamp_s; bool valid; }`(RealtimeBuffer 载荷)
  - `struct DirectOutput { sfc::CartesianCorrection correction; bool goal_active, stream_stale, saturated; }`
  - `class DirectCorrectionCore`:`configure`/`reset`/`setStream`/`startGoal`/`cancelGoal`/`goalStatus`/`goalErrorDeg(state)`/`update(state, now_s, dt)`。
- 语义:goal `RUNNING` 期间 stream 被忽略(待确认 11);无 goal 时 stream 命令**保持到超时**(发布频率低于控制频率时按周期重放,超时 `stream_timeout_s` 后归零——规格 §7.7 stale 规则);两路输出统一过 `SafetyLimiter`(传零 wrench,硬截断在此控制器天然失效,per-cycle clamp 生效)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/soft_robot_controllers/test/test_direct_correction_core.cpp`:

```cpp
#include <gtest/gtest.h>

#include "soft_robot_controllers/direct_correction_core.h"

using soft_robot_controllers::DirectCorrectionCore;
using soft_robot_controllers::DirectCorrectionParams;
using soft_robot_controllers::DirectOutput;
using soft_robot_controllers::StreamCommand;

namespace {

constexpr double kDt = 0.004;
constexpr double kNow = 100.0;  // arbitrary monotonic clock origin

DirectCorrectionParams params() {
  DirectCorrectionParams p;
  p.stream_timeout_s = 0.1;
  p.safety.max_corr_trans = 0.5;
  p.safety.max_corr_rot = 0.05;
  return p;
}

DirectCorrectionCore makeCore() {
  DirectCorrectionCore core;
  core.configure(params());
  return core;
}

StreamCommand stream(double x, double stamp_s) {
  StreamCommand c;
  c.correction.x = x;
  c.stamp_s = stamp_s;
  c.valid = true;
  return c;
}

sfc::MotionGoal goalA(double a, double max_speed_dps = 7.5) {
  sfc::MotionGoal g;
  g.a = a;
  g.max_speed_dps = max_speed_dps;
  g.p_gain = 1.0;
  g.tol_deg = 0.1;
  g.hold_s = 0.008;  // two cycles inside tolerance
  g.timeout_s = 30.0;
  return g;
}

}  // namespace

TEST(DirectCore, FreshStreamPassesThrough) {
  DirectCorrectionCore core = makeCore();
  core.setStream(stream(0.2, kNow - 0.05));
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_NEAR(out.correction.x, 0.2, 1e-12);
  EXPECT_FALSE(out.stream_stale);
  EXPECT_FALSE(out.goal_active);
}

TEST(DirectCore, StaleStreamZeros) {
  DirectCorrectionCore core = makeCore();
  core.setStream(stream(0.2, kNow - 0.2));  // older than the 0.1 s timeout
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_EQ(out.correction.x, 0.0);
  EXPECT_TRUE(out.stream_stale);
}

TEST(DirectCore, NoStreamYetReportsStale) {
  DirectCorrectionCore core = makeCore();
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_EQ(out.correction.x, 0.0);
  EXPECT_TRUE(out.stream_stale);
}

TEST(DirectCore, StreamClampedBySafetyLimiter) {
  DirectCorrectionCore core = makeCore();
  core.setStream(stream(2.0, kNow));  // way above the 0.5 mm clamp
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_NEAR(out.correction.x, 0.5, 1e-12);
  EXPECT_TRUE(out.saturated);
}

TEST(DirectCore, RunningGoalOverridesStream) {
  DirectCorrectionCore core = makeCore();
  core.setStream(stream(0.2, kNow));
  core.startGoal(goalA(10.0));
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow, kDt);
  EXPECT_TRUE(out.goal_active);
  EXPECT_EQ(out.correction.x, 0.0);            // stream ignored
  EXPECT_NEAR(out.correction.a, 0.03, 1e-12);  // 7.5 deg/s * 4 ms
  EXPECT_EQ(core.goalStatus(), sfc::MotionStatus::RUNNING);
}

TEST(DirectCore, GoalConvergesThenStreamResumes) {
  DirectCorrectionCore core = makeCore();
  sfc::CartesianState s;
  s.a = 0.05;                     // already inside the 0.1 deg tolerance
  core.startGoal(goalA(0.0));
  core.update(s, kNow, kDt);                    // hold 1/2
  core.update(s, kNow + kDt, kDt);              // hold 2/2 -> CONVERGED
  EXPECT_EQ(core.goalStatus(), sfc::MotionStatus::CONVERGED);
  core.setStream(stream(0.3, kNow + 2 * kDt));
  const DirectOutput out = core.update(s, kNow + 2 * kDt, kDt);
  EXPECT_FALSE(out.goal_active);
  EXPECT_NEAR(out.correction.x, 0.3, 1e-12);
}

TEST(DirectCore, GoalTimeoutZeroesOutput) {
  DirectCorrectionCore core = makeCore();
  sfc::MotionGoal g = goalA(90.0);
  g.timeout_s = 0.01;  // expires on the third 4 ms cycle
  core.startGoal(g);
  core.update(sfc::CartesianState{}, kNow, kDt);
  core.update(sfc::CartesianState{}, kNow + kDt, kDt);
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow + 2 * kDt, kDt);
  EXPECT_EQ(core.goalStatus(), sfc::MotionStatus::TIMEOUT);
  EXPECT_EQ(out.correction.a, 0.0);
}

TEST(DirectCore, CancelGoalReturnsToStream) {
  DirectCorrectionCore core = makeCore();
  core.startGoal(goalA(10.0));
  core.update(sfc::CartesianState{}, kNow, kDt);
  core.cancelGoal();
  EXPECT_EQ(core.goalStatus(), sfc::MotionStatus::INACTIVE);
  core.setStream(stream(0.1, kNow + kDt));
  const DirectOutput out = core.update(sfc::CartesianState{}, kNow + kDt, kDt);
  EXPECT_FALSE(out.goal_active);
  EXPECT_NEAR(out.correction.x, 0.1, 1e-12);
}

TEST(DirectCore, GoalErrorDegIsGeodesic) {
  DirectCorrectionCore core = makeCore();
  core.startGoal(goalA(10.0));
  EXPECT_NEAR(core.goalErrorDeg(sfc::CartesianState{}), 10.0, 1e-6);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `direct_correction_core.h`。

- [ ] **Step 3: 写实现**

`ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/direct_correction_core.h`:

```cpp
#pragma once

#include "soft_force_control_core/orientation_motion_core.h"
#include "soft_force_control_core/safety_limiter.h"
#include "soft_force_control_core/types.h"

namespace soft_robot_controllers {

struct DirectCorrectionParams {
  double stream_timeout_s{0.1};  // zero output on stale stream (spec 7.7)
  sfc::SafetyParams safety;      // per-cycle clamp; ceilings inert here
};

// Latest stream-mode command handed from the subscriber thread to
// update() through a RealtimeBuffer.
struct StreamCommand {
  sfc::CartesianCorrection correction;
  double stamp_s{0};
  bool valid{false};
};

struct DirectOutput {
  sfc::CartesianCorrection correction;
  bool goal_active{false};
  bool stream_stale{false};
  bool saturated{false};
};

// Command-source logic of CartesianCorrectionController (spec 5.3, 7.6,
// 7.7): a RUNNING orientation goal takes priority over the stream
// (decision 11); otherwise a fresh stream command passes through, held
// until stream_timeout_s. Both paths go through the SafetyLimiter with a
// zero wrench (the hard cutoff never trips: no force feedback in this
// controller). Pure logic, allocation-free, RT-safe.
class DirectCorrectionCore {
 public:
  void configure(const DirectCorrectionParams& p) { params_ = p; }

  // Mode (re-)entry: drop any stale stream command and cancel the goal.
  void reset();

  void setStream(const StreamCommand& cmd) { stream_ = cmd; }
  void startGoal(const sfc::MotionGoal& g);
  void cancelGoal() { motion_.cancel(); }
  sfc::MotionStatus goalStatus() const { return motion_.status(); }

  // Geodesic orientation error to the active goal, for action feedback.
  double goalErrorDeg(const sfc::CartesianState& s) const;

  DirectOutput update(const sfc::CartesianState& state, double now_s,
                      double dt);

 private:
  DirectCorrectionParams params_;
  StreamCommand stream_;
  sfc::MotionGoal goal_;  // copy kept for feedback error computation
  sfc::OrientationMotionCore motion_;
  sfc::SafetyLimiter limiter_;
};

}  // namespace soft_robot_controllers
```

`ros_ws/src/soft_robot_controllers/src/direct_correction_core.cpp`:

```cpp
#include "soft_robot_controllers/direct_correction_core.h"

#include "soft_force_control_core/rotation.h"

namespace soft_robot_controllers {

void DirectCorrectionCore::reset() {
  stream_ = StreamCommand{};
  motion_.cancel();
}

void DirectCorrectionCore::startGoal(const sfc::MotionGoal& g) {
  goal_ = g;
  motion_.setGoal(g);
}

double DirectCorrectionCore::goalErrorDeg(const sfc::CartesianState& s) const {
  return sfc::angularDistanceDeg(goal_.a, goal_.b, goal_.c, s.a, s.b, s.c);
}

DirectOutput DirectCorrectionCore::update(const sfc::CartesianState& state,
                                          double now_s, double dt) {
  DirectOutput out;
  sfc::CartesianCorrection raw;
  if (motion_.status() == sfc::MotionStatus::RUNNING) {
    raw = motion_.update(state, dt);
    out.goal_active = true;
  } else if (stream_.valid &&
             now_s - stream_.stamp_s <= params_.stream_timeout_s) {
    raw = stream_.correction;
  } else {
    out.stream_stale = true;
  }
  const sfc::SafetyResult res =
      limiter_.apply(raw, sfc::Wrench{}, params_.safety);
  out.correction = res.correction;
  out.saturated = res.saturated;
  return out;
}

}  // namespace soft_robot_controllers
```

- [ ] **Step 4: CMake 增量**

库源列表加 `src/direct_correction_core.cpp`;测试段追加:

```cmake
  catkin_add_gtest(test_direct_correction_core test/test_direct_correction_core.cpp)
  target_link_libraries(test_direct_correction_core ${PROJECT_NAME}
                        ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_controllers/test_direct_correction_core
```

预期:`[  PASSED  ] 9 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers && \
git commit -m "feat(controllers): add DirectCorrectionCore stream/goal command source (Plan 3 Task 6)"
```

---

### Task 7: `ForceComplianceController` ROS 壳(RealtimeBuffer 输入 + ModeState 发布 + 插件导出)

**Files:**
- Create: `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_controller.h`
- Create: `ros_ws/src/soft_robot_controllers/src/force_compliance_controller.cpp`
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_controllers/test/test_force_compliance_controller.cpp`

**Interfaces:**
- Consumes: Task 2/3 产物;`kuka_rsi::CartesianCorrectionCommandInterface`/`CartesianCorrectionHandle`(资源 `kuka_tcp`,`setCommand(x,y,z,a,b,c)`,state getters 继承自 `CartesianStateHandle`);`geometry_msgs/WrenchStamped`、`soft_robot_msgs/{ModeCommand,ModeState,RsiState}`;`realtime_tools::{RealtimeBuffer,RealtimePublisher}`。
- Produces(Task 9/10、Plan 5 消费):
  - 插件类 `soft_robot_controllers::ForceComplianceController : controller_interface::Controller<kuka_rsi::CartesianCorrectionCommandInterface>`(`PLUGINLIB_EXPORT_CLASS`)。
  - 免 master 测试入口 `bool configureController(const kuka_rsi::CartesianCorrectionHandle&, const ForceComplianceParams& drag, const ForceComplianceParams& precision)`;注入口 `injectWrench/injectModeCommand/injectRsiFault`(订阅回调与测试共用同一路径)。
  - `struct WrenchSample { sfc::Wrench w; double stamp_s; bool valid; }`(`FaultFlag` 复用 Task 2 `controller_mode_gate.h` 中的共享定义)。
- 实时安全:三个订阅均在 `init()` 建立,回调只 `writeFromNonRT`;`update()` 只 `readFromRT` + 值拷贝;`ModeState` 经 `RealtimePublisher::trylock` 节流 50 Hz;无 master 时(`configureController` 路径)`state_pub_` 为空指针,发布安全跳过。
- 故障感知:`/kuka/rsi/state`(Plan 2 跟进 3 方案 A);`fault=true` → 输出零。
- 注意:`init()` 的参数装载需要 param server,不做离线单测(Task 10 冒烟覆盖);`update()` 全路径离线测试。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/soft_robot_controllers/test/test_force_compliance_controller.cpp`:

```cpp
#include <gtest/gtest.h>

#include <soft_robot_msgs/ModeCommand.h>

#include "soft_robot_controllers/force_compliance_controller.h"

using soft_robot_controllers::ForceComplianceController;
using soft_robot_controllers::ForceComplianceParams;
using soft_robot_controllers::WrenchSample;
namespace msg = soft_robot_msgs;

namespace {

constexpr double kT0 = 100.0;  // synthetic clock origin [s]

ForceComplianceParams precisionParams() {
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
  p.wrench_timeout_s = 0.012;
  return p;
}

ForceComplianceParams dragParams() {
  ForceComplianceParams p = precisionParams();
  p.adaptive_deadband = true;
  p.ramp_window_s = 0.02;  // 5 cycles
  p.ramp_force_margin_n = 5.0;
  p.ramp_torque_margin_nm = 1.0;
  p.retare.enabled = true;
  p.retare.orientation_tol_deg = 1.0;
  return p;
}

class FccControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::Time::init();  // wall-clock ros::Time without a master
    state_[0] = 500.0;
    state_[1] = 0.0;
    state_[2] = 800.0;
    state_[3] = 0.0;
    state_[4] = 0.0;
    state_[5] = 0.0;
    for (double& c : cmd_) c = 0.0;
    const kuka_rsi::CartesianStateHandle sh("kuka_tcp", &state_[0], &state_[1],
                                            &state_[2], &state_[3], &state_[4],
                                            &state_[5]);
    handle_ = kuka_rsi::CartesianCorrectionHandle(sh, &cmd_[0], &cmd_[1],
                                                  &cmd_[2], &cmd_[3], &cmd_[4],
                                                  &cmd_[5]);
    ASSERT_TRUE(
        ctl_.configureController(handle_, dragParams(), precisionParams()));
    ctl_.starting(ros::Time(kT0));
  }

  // One control cycle: fresh wrench sample stamped at the update time.
  void cycle(double fz, double t, double dt = 0.004) {
    WrenchSample s;
    s.w.fz = fz;
    s.stamp_s = t;
    s.valid = true;
    ctl_.injectWrench(s);
    ctl_.update(ros::Time(t), ros::Duration(dt));
  }

  double state_[6];
  double cmd_[6];
  kuka_rsi::CartesianCorrectionHandle handle_;
  ForceComplianceController ctl_;
};

}  // namespace

TEST_F(FccControllerTest, DisengagedByDefaultOutputsZero) {
  cycle(100.0, kT0);
  for (const double c : cmd_) EXPECT_EQ(c, 0.0);
}

TEST_F(FccControllerTest, ModeEntryEnablesCompliance) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0);  // e = 10 N -> 10 mm/s -> 0.04 mm per 4 ms cycle
  EXPECT_NEAR(cmd_[2], 0.04, 1e-12);
  EXPECT_EQ(cmd_[0], 0.0);
}

TEST_F(FccControllerTest, MeasuredPeriodScalesCommand) {
  // Plan 2 follow-up 1: the controller must forward the measured period,
  // never a hardcoded 4 ms.
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0, 0.008);
  EXPECT_NEAR(cmd_[2], 0.08, 1e-12);
}

TEST_F(FccControllerTest, StaleWrenchOutputsZero) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  WrenchSample s;
  s.w.fz = 40.0;
  s.stamp_s = kT0;
  s.valid = true;
  ctl_.injectWrench(s);
  ctl_.update(ros::Time(kT0 + 0.02), ros::Duration(0.004));  // 20 ms old
  EXPECT_EQ(cmd_[2], 0.0);
}

TEST_F(FccControllerTest, RsiFaultForcesZero) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0);
  ASSERT_GT(cmd_[2], 0.0);
  ctl_.injectRsiFault(true);
  cycle(40.0, kT0 + 0.004);
  EXPECT_EQ(cmd_[2], 0.0);
}

TEST_F(FccControllerTest, IdleCommandDisengages) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0);
  ASSERT_GT(cmd_[2], 0.0);
  ctl_.injectModeCommand(msg::ModeCommand::MODE_IDLE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0 + 0.004);
  EXPECT_EQ(cmd_[2], 0.0);
}

TEST_F(FccControllerTest, DragEntryRunsStartupRamp) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_DRAG);
  // 5 ramp cycles at zero residual: output must stay zero (spec 7.4).
  for (int i = 0; i < 5; ++i) {
    cycle(0.0, kT0 + 0.004 * i);
    EXPECT_EQ(cmd_[2], 0.0) << "ramp cycle " << i;
  }
  // Learned deadband = 0 + 5 N margin; 40 N now produces output.
  cycle(40.0, kT0 + 0.004 * 5);
  EXPECT_NEAR(cmd_[2], (40.0 - 5.0) * 1.0 * 0.004, 1e-12);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `force_compliance_controller.h`。

- [ ] **Step 3: 写实现**

`ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/force_compliance_controller.h`:

```cpp
#pragma once

#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <soft_robot_msgs/ModeCommand.h>
#include <soft_robot_msgs/ModeState.h>
#include <soft_robot_msgs/RsiState.h>

#include <cstdint>
#include <memory>

#include "kuka_rsi_hw_interface/cartesian_command_interface.h"
#include "soft_robot_controllers/controller_mode_gate.h"
#include "soft_robot_controllers/force_compliance_core.h"

namespace soft_robot_controllers {

// Latest wrench sample handed from the subscriber thread to update().
struct WrenchSample {
  sfc::Wrench w;
  double stamp_s{0};
  bool valid{false};
};

// Thin ROS shell around ForceComplianceCore (spec 5.3 / 7.1). All topic
// input reaches update() through RealtimeBuffers written by subscriber
// callbacks on the node spinner threads; update() only reads buffers and
// never subscribes, allocates, or blocks. Fault awareness comes from the
// /kuka/rsi/state topic (Plan 2 follow-up 3, decision 2).
class ForceComplianceController
    : public controller_interface::Controller<
          kuka_rsi::CartesianCorrectionCommandInterface> {
 public:
  bool init(kuka_rsi::CartesianCorrectionCommandInterface* hw,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  // ROS-master-free wiring entry (same pattern as KukaRsiRobotHW::
  // configure): used by init() and directly by offline tests.
  bool configureController(const kuka_rsi::CartesianCorrectionHandle& handle,
                           const ForceComplianceParams& drag,
                           const ForceComplianceParams& precision);

  // Non-RT producers. Subscriber callbacks delegate here; offline tests
  // call them directly (single producer per buffer).
  void injectWrench(const WrenchSample& s) { wrench_buf_.writeFromNonRT(s); }
  void injectModeCommand(std::uint8_t mode, std::uint8_t profile);
  void injectRsiFault(bool fault) {
    fault_buf_.writeFromNonRT(FaultFlag{fault});
  }

  const ControllerModeGate& gate() const { return gate_; }

 private:
  void wrenchCb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void modeCb(const soft_robot_msgs::ModeCommand::ConstPtr& msg);
  void rsiStateCb(const soft_robot_msgs::RsiState::ConstPtr& msg);
  sfc::CartesianState readState() const;
  void setZero();
  void publishState(const ros::Time& time, bool degraded);

  kuka_rsi::CartesianCorrectionHandle handle_;
  ControllerModeGate gate_{sfc::ControlMode::FORCE_COMPLIANCE};
  ForceComplianceCore core_;
  ForceComplianceParams drag_params_;
  ForceComplianceParams precision_params_;

  realtime_tools::RealtimeBuffer<WrenchSample> wrench_buf_;
  realtime_tools::RealtimeBuffer<ModeRequest> mode_buf_;
  realtime_tools::RealtimeBuffer<FaultFlag> fault_buf_;
  std::uint64_t mode_seq_{0};  // producer side (subscriber thread / tests)

  ros::Subscriber wrench_sub_;
  ros::Subscriber mode_sub_;
  ros::Subscriber rsi_sub_;
  std::unique_ptr<
      realtime_tools::RealtimePublisher<soft_robot_msgs::ModeState>>
      state_pub_;
  double last_pub_s_{-1.0};
};

}  // namespace soft_robot_controllers
```

`ros_ws/src/soft_robot_controllers/src/force_compliance_controller.cpp`:

```cpp
#include "soft_robot_controllers/force_compliance_controller.h"

#include <pluginlib/class_list_macros.hpp>

#include <string>

namespace soft_robot_controllers {

namespace {

void loadSafety(ros::NodeHandle& nh, sfc::SafetyParams& s) {
  nh.param("safety/max_corr_trans", s.max_corr_trans, s.max_corr_trans);
  nh.param("safety/max_corr_rot", s.max_corr_rot, s.max_corr_rot);
  nh.param("safety/force_ceiling", s.force_ceiling, s.force_ceiling);
  nh.param("safety/torque_ceiling", s.torque_ceiling, s.torque_ceiling);
}

void loadPayload(ros::NodeHandle& nh, sfc::PayloadParams& p) {
  nh.param("payload/gravity_n", p.gravity_n, p.gravity_n);
  nh.param("payload/com_x", p.com_x, p.com_x);
  nh.param("payload/com_y", p.com_y, p.com_y);
  nh.param("payload/com_z", p.com_z, p.com_z);
  nh.param("payload/bias_fx", p.bias.fx, p.bias.fx);
  nh.param("payload/bias_fy", p.bias.fy, p.bias.fy);
  nh.param("payload/bias_fz", p.bias.fz, p.bias.fz);
  nh.param("payload/bias_tx", p.bias.tx, p.bias.tx);
  nh.param("payload/bias_ty", p.bias.ty, p.bias.ty);
  nh.param("payload/bias_tz", p.bias.tz, p.bias.tz);
}

void loadProfile(ros::NodeHandle& nh, const std::string& name,
                 bool drag_defaults, ForceComplianceParams& p) {
  p.adaptive_deadband = drag_defaults;
  p.retare.enabled = drag_defaults;
  const std::string b = "profiles/" + name + "/";
  nh.param(b + "filter_cutoff_hz", p.filter_cutoff_hz, p.filter_cutoff_hz);
  nh.param(b + "speed_scale", p.compliance.speed_scale,
           p.compliance.speed_scale);
  nh.param(b + "gain_translation", p.compliance.translation.gain,
           p.compliance.translation.gain);
  nh.param(b + "gain_rotation", p.compliance.rotation.gain,
           p.compliance.rotation.gain);
  nh.param(b + "max_speed_translation", p.compliance.translation.max_speed,
           p.compliance.translation.max_speed);
  nh.param(b + "max_speed_rotation", p.compliance.rotation.max_speed,
           p.compliance.rotation.max_speed);
  nh.param(b + "max_accel_translation", p.compliance.translation.max_accel,
           p.compliance.translation.max_accel);
  nh.param(b + "max_accel_rotation", p.compliance.rotation.max_accel,
           p.compliance.rotation.max_accel);
  nh.param(b + "deadband_force", p.fixed_force_deadband_n,
           p.fixed_force_deadband_n);
  nh.param(b + "deadband_torque", p.fixed_torque_deadband_nm,
           p.fixed_torque_deadband_nm);
  nh.param(b + "adaptive_deadband", p.adaptive_deadband, p.adaptive_deadband);
  nh.param(b + "ramp_window_s", p.ramp_window_s, p.ramp_window_s);
  nh.param(b + "ramp_force_margin_n", p.ramp_force_margin_n,
           p.ramp_force_margin_n);
  nh.param(b + "ramp_torque_margin_nm", p.ramp_torque_margin_nm,
           p.ramp_torque_margin_nm);
  nh.param(b + "auto_retare", p.retare.enabled, p.retare.enabled);
  nh.param(b + "retare_orientation_tol_deg", p.retare.orientation_tol_deg,
           p.retare.orientation_tol_deg);
  nh.param(b + "retare_rearm_factor", p.retare_rearm_factor,
           p.retare_rearm_factor);
}

}  // namespace

bool ForceComplianceController::init(
    kuka_rsi::CartesianCorrectionCommandInterface* hw, ros::NodeHandle& root_nh,
    ros::NodeHandle& controller_nh) {
  ForceComplianceParams drag;
  ForceComplianceParams precision;
  loadProfile(controller_nh, "drag", true, drag);
  loadProfile(controller_nh, "precision", false, precision);

  sfc::SafetyParams safety;
  loadSafety(controller_nh, safety);
  sfc::PayloadParams payload;
  loadPayload(controller_nh, payload);
  double wrench_timeout = 0.012;
  controller_nh.param("wrench_timeout", wrench_timeout, wrench_timeout);
  drag.safety = safety;
  precision.safety = safety;
  drag.payload = payload;
  precision.payload = payload;
  drag.wrench_timeout_s = wrench_timeout;
  precision.wrench_timeout_s = wrench_timeout;

  std::string resource;
  controller_nh.param<std::string>("cartesian_resource", resource,
                                   std::string("kuka_tcp"));
  kuka_rsi::CartesianCorrectionHandle handle;
  try {
    handle = hw->getHandle(resource);
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ForceComplianceController: " << ex.what());
    return false;
  }
  if (!configureController(handle, drag, precision)) return false;

  std::string wrench_topic;
  std::string mode_topic;
  std::string rsi_topic;
  std::string state_topic;
  controller_nh.param<std::string>("wrench_topic", wrench_topic,
                                   std::string("/sri_ft/wrench_raw"));
  controller_nh.param<std::string>("mode_command_topic", mode_topic,
                                   std::string("/soft_robot/mode_command"));
  controller_nh.param<std::string>("rsi_state_topic", rsi_topic,
                                   std::string("/kuka/rsi/state"));
  controller_nh.param<std::string>("mode_state_topic", state_topic,
                                   std::string("/soft_robot/mode_state"));
  wrench_sub_ = root_nh.subscribe(wrench_topic, 1,
                                  &ForceComplianceController::wrenchCb, this);
  mode_sub_ = root_nh.subscribe(mode_topic, 1,
                                &ForceComplianceController::modeCb, this);
  rsi_sub_ = root_nh.subscribe(rsi_topic, 1,
                               &ForceComplianceController::rsiStateCb, this);
  state_pub_.reset(
      new realtime_tools::RealtimePublisher<soft_robot_msgs::ModeState>(
          root_nh, state_topic, 4));
  return true;
}

bool ForceComplianceController::configureController(
    const kuka_rsi::CartesianCorrectionHandle& handle,
    const ForceComplianceParams& drag, const ForceComplianceParams& precision) {
  handle_ = handle;
  drag_params_ = drag;
  precision_params_ = precision;
  core_.configure(precision_params_);  // safe defaults until a mode entry
  wrench_buf_.writeFromNonRT(WrenchSample{});
  mode_buf_.writeFromNonRT(ModeRequest{});
  fault_buf_.writeFromNonRT(FaultFlag{});
  return true;
}

void ForceComplianceController::injectModeCommand(std::uint8_t mode,
                                                  std::uint8_t profile) {
  ModeRequest r;
  r.mode = mode;
  r.profile = profile;
  r.seq = ++mode_seq_;
  mode_buf_.writeFromNonRT(r);
}

void ForceComplianceController::wrenchCb(
    const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  WrenchSample s;
  s.w.fx = msg->wrench.force.x;
  s.w.fy = msg->wrench.force.y;
  s.w.fz = msg->wrench.force.z;
  s.w.tx = msg->wrench.torque.x;
  s.w.ty = msg->wrench.torque.y;
  s.w.tz = msg->wrench.torque.z;
  s.stamp_s = msg->header.stamp.isZero() ? ros::Time::now().toSec()
                                         : msg->header.stamp.toSec();
  s.valid = true;
  injectWrench(s);
}

void ForceComplianceController::modeCb(
    const soft_robot_msgs::ModeCommand::ConstPtr& msg) {
  injectModeCommand(msg->mode, msg->profile);
}

void ForceComplianceController::rsiStateCb(
    const soft_robot_msgs::RsiState::ConstPtr& msg) {
  injectRsiFault(msg->fault);
}

sfc::CartesianState ForceComplianceController::readState() const {
  sfc::CartesianState s;
  s.x = handle_.getX();
  s.y = handle_.getY();
  s.z = handle_.getZ();
  s.a = handle_.getA();
  s.b = handle_.getB();
  s.c = handle_.getC();
  return s;
}

void ForceComplianceController::setZero() {
  handle_.setCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void ForceComplianceController::starting(const ros::Time& /*time*/) {
  setZero();
  if (gate_.engaged()) {
    // Controller restarted while the mode is still active: re-activate so
    // the ramp/re-tare reference matches the current pose.
    core_.activate(readState());
  }
}

void ForceComplianceController::update(const ros::Time& time,
                                       const ros::Duration& period) {
  const ModeRequest req = *mode_buf_.readFromRT();
  if (gate_.apply(req)) {  // entered FORCE_COMPLIANCE on this cycle
    const bool drag = gate_.snapshot().profile == sfc::Profile::DRAG;
    core_.configure(drag ? drag_params_ : precision_params_);
    core_.activate(readState());
  }
  const bool fault = fault_buf_.readFromRT()->fault;
  if (!gate_.engaged() || fault) {
    setZero();
    publishState(time, fault);
    return;
  }

  const WrenchSample ws = *wrench_buf_.readFromRT();
  ComplianceInput in;
  in.state = readState();
  in.raw = ws.w;
  in.wrench_valid = ws.valid;
  in.wrench_age_s = ws.valid ? time.toSec() - ws.stamp_s : 1e9;
  // Plan 2 follow-up 1: dt is the measured period, never a constant.
  const ComplianceOutput out = core_.update(in, period.toSec());

  handle_.setCommand(out.correction.x, out.correction.y, out.correction.z,
                     out.correction.a, out.correction.b, out.correction.c);
  publishState(time, out.hard_cutoff || out.wrench_timeout);
}

void ForceComplianceController::stopping(const ros::Time& /*time*/) {
  setZero();
  gate_.forceIdle();
}

void ForceComplianceController::publishState(const ros::Time& time,
                                             bool degraded) {
  if (!state_pub_) return;  // offline tests run without a publisher
  if (last_pub_s_ >= 0.0 && time.toSec() - last_pub_s_ < 0.02) return;
  if (!state_pub_->trylock()) return;
  last_pub_s_ = time.toSec();
  const sfc::ModeSnapshot snap = gate_.snapshot();
  state_pub_->msg_.header.stamp = time;
  state_pub_->msg_.mode = fromControlMode(snap.mode);
  state_pub_->msg_.profile = fromProfile(snap.profile);
  if (!gate_.engaged()) {
    state_pub_->msg_.system_state = soft_robot_msgs::ModeState::SYSTEM_READY;
  } else if (degraded) {
    state_pub_->msg_.system_state =
        soft_robot_msgs::ModeState::SYSTEM_DEGRADED;
  } else {
    state_pub_->msg_.system_state =
        soft_robot_msgs::ModeState::SYSTEM_SERVOING;
  }
  state_pub_->unlockAndPublish();
}

}  // namespace soft_robot_controllers

PLUGINLIB_EXPORT_CLASS(soft_robot_controllers::ForceComplianceController,
                       controller_interface::ControllerBase)
```

- [ ] **Step 4: CMake 增量**

库源列表加 `src/force_compliance_controller.cpp`;测试段追加:

```cmake
  catkin_add_gtest(test_force_compliance_controller
                   test/test_force_compliance_controller.cpp)
  target_link_libraries(test_force_compliance_controller ${PROJECT_NAME}
                        ${catkin_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_controllers/test_force_compliance_controller
```

预期:`[  PASSED  ] 7 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers && \
git commit -m "feat(controllers): add ForceComplianceController ROS shell with realtime-safe topic plumbing (Plan 3 Task 7)"
```

---

### Task 8: `CartesianCorrectionController` ROS 壳(stream 话题 + MoveToOrientation action)

**Files:**
- Create: `ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/cartesian_correction_controller.h`
- Create: `ros_ws/src/soft_robot_controllers/src/cartesian_correction_controller.cpp`
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_controllers/test/test_cartesian_correction_controller.cpp`

**Interfaces:**
- Consumes: Task 2/6 产物;`soft_robot_msgs/CartesianCorrectionStamped`(stream 命令话题)、`soft_robot_msgs/MoveToOrientationAction`、`actionlib::SimpleActionServer`。
- Produces(Task 9/10、Plan 5 消费):
  - 插件类 `soft_robot_controllers::CartesianCorrectionController`(接管模式 = `DIRECT_CARTESIAN` **或** `CALIBRATION`,规格 §10)。
  - action `/soft_robot/move_to_orientation`(名字可参数化;goal → `sfc::MotionGoal`,`speed_scale` 语义见待确认 8;`use_position=true` → ABORTED,待确认 6)。
  - 免 master 入口 `configureController(handle, CartesianCorrectionControllerParams)` 与 goal 管道 `requestGoal/requestCancel/appliedGoalSeq/motionStatus/motionErrorDeg/engagedNow`(execute 回调与离线测试同路径)。
- RT↔action 握手(待确认 5):goal/cancel 经 `RealtimeBuffer<GoalRequest>`(带序号)进 RT;`update()` 每周期把 `goalStatus`/误差(double 打包进 `std::atomic<uint64_t>`)/已应用序号写回原子;execute 线程先等 `appliedGoalSeq() >= seq` 再解读状态,避免读到上一个 goal 的陈旧终态。PREEMPTED 记账由壳层完成(自己 cancel → PREEMPTED;RT 侧因模式切换 cancel → ABORTED)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/soft_robot_controllers/test/test_cartesian_correction_controller.cpp`:

```cpp
#include <gtest/gtest.h>

#include <soft_robot_msgs/ModeCommand.h>

#include "soft_robot_controllers/cartesian_correction_controller.h"

using soft_robot_controllers::CartesianCorrectionController;
using soft_robot_controllers::CartesianCorrectionControllerParams;
using soft_robot_controllers::StreamCommand;
namespace msg = soft_robot_msgs;

namespace {

constexpr double kT0 = 200.0;
constexpr double kDt = 0.004;

CartesianCorrectionControllerParams params() {
  CartesianCorrectionControllerParams p;
  p.direct.stream_timeout_s = 0.1;
  p.direct.safety.max_corr_trans = 0.5;
  p.direct.safety.max_corr_rot = 0.05;
  p.goal_defaults.max_speed_dps = 7.5;
  p.goal_defaults.p_gain = 20.0;
  p.goal_defaults.tol_deg = 0.1;
  p.goal_defaults.hold_s = 0.008;
  p.goal_defaults.timeout_s = 30.0;
  return p;
}

StreamCommand stream(double x, double stamp_s) {
  StreamCommand c;
  c.correction.x = x;
  c.stamp_s = stamp_s;
  c.valid = true;
  return c;
}

class CccControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::Time::init();
    for (double& s : state_) s = 0.0;
    for (double& c : cmd_) c = 0.0;
    const kuka_rsi::CartesianStateHandle sh("kuka_tcp", &state_[0], &state_[1],
                                            &state_[2], &state_[3], &state_[4],
                                            &state_[5]);
    handle_ = kuka_rsi::CartesianCorrectionHandle(sh, &cmd_[0], &cmd_[1],
                                                  &cmd_[2], &cmd_[3], &cmd_[4],
                                                  &cmd_[5]);
    ASSERT_TRUE(ctl_.configureController(handle_, params()));
    ctl_.starting(ros::Time(kT0));
  }

  void engage() {
    ctl_.injectModeCommand(msg::ModeCommand::MODE_DIRECT_CARTESIAN,
                           msg::ModeCommand::PROFILE_PRECISION);
  }
  void cycle(double t) { ctl_.update(ros::Time(t), ros::Duration(kDt)); }

  double state_[6];
  double cmd_[6];
  kuka_rsi::CartesianCorrectionHandle handle_;
  CartesianCorrectionController ctl_;
};

sfc::MotionGoal goalA(double a) {
  sfc::MotionGoal g = params().goal_defaults;
  g.a = a;
  return g;
}

}  // namespace

TEST_F(CccControllerTest, NotEngagedZerosStream) {
  ctl_.injectStream(stream(0.2, kT0));
  cycle(kT0);
  for (const double c : cmd_) EXPECT_EQ(c, 0.0);
  EXPECT_FALSE(ctl_.engagedNow());
}

TEST_F(CccControllerTest, StreamPassesThroughWhenEngaged) {
  engage();
  ctl_.injectStream(stream(0.2, kT0));
  cycle(kT0);
  EXPECT_TRUE(ctl_.engagedNow());
  EXPECT_NEAR(cmd_[0], 0.2, 1e-12);
}

TEST_F(CccControllerTest, StaleStreamZeros) {
  engage();
  ctl_.injectStream(stream(0.2, kT0 - 0.2));  // older than 0.1 s timeout
  cycle(kT0);
  EXPECT_EQ(cmd_[0], 0.0);
}

TEST_F(CccControllerTest, RsiFaultZeroesOutput) {
  engage();
  ctl_.injectStream(stream(0.2, kT0));
  cycle(kT0);
  ASSERT_NEAR(cmd_[0], 0.2, 1e-12);
  ctl_.injectRsiFault(true);
  ctl_.injectStream(stream(0.2, kT0 + kDt));
  cycle(kT0 + kDt);
  EXPECT_EQ(cmd_[0], 0.0);
  EXPECT_FALSE(ctl_.engagedNow());
}

TEST_F(CccControllerTest, GoalRunsToConvergenceOnIntegratedPose) {
  engage();
  const std::uint64_t seq = ctl_.requestGoal(goalA(0.5));
  int cycles = 0;
  for (; cycles < 100; ++cycles) {
    cycle(kT0 + cycles * kDt);
    state_[3] += cmd_[3];  // plant model: pose integrates the correction
    if (ctl_.appliedGoalSeq() >= seq &&
        ctl_.motionStatus() == sfc::MotionStatus::CONVERGED) {
      break;
    }
  }
  EXPECT_EQ(ctl_.motionStatus(), sfc::MotionStatus::CONVERGED);
  EXPECT_LT(cycles, 100);
  EXPECT_NEAR(state_[3], 0.5, 0.15);  // within tolerance + hold overshoot
}

TEST_F(CccControllerTest, GoalOverridesStreamThenStreamResumes) {
  engage();
  state_[3] = 0.05;  // already inside tolerance of goal a = 0
  ctl_.requestGoal(goalA(0.0));
  ctl_.injectStream(stream(0.2, kT0));
  cycle(kT0);                      // hold 1/2: goal path in charge
  EXPECT_EQ(cmd_[0], 0.0);
  cycle(kT0 + kDt);                // hold 2/2 -> CONVERGED
  EXPECT_EQ(ctl_.motionStatus(), sfc::MotionStatus::CONVERGED);
  ctl_.injectStream(stream(0.2, kT0 + 2 * kDt));
  cycle(kT0 + 2 * kDt);            // stream resumes
  EXPECT_NEAR(cmd_[0], 0.2, 1e-12);
}

TEST_F(CccControllerTest, RequestCancelStopsGoal) {
  engage();
  ctl_.requestGoal(goalA(90.0));
  cycle(kT0);
  ASSERT_EQ(ctl_.motionStatus(), sfc::MotionStatus::RUNNING);
  ctl_.requestCancel();
  cycle(kT0 + kDt);
  EXPECT_EQ(ctl_.motionStatus(), sfc::MotionStatus::INACTIVE);
  EXPECT_EQ(cmd_[3], 0.0);
}

TEST_F(CccControllerTest, ModeExitCancelsRunningGoal) {
  engage();
  ctl_.requestGoal(goalA(90.0));
  cycle(kT0);
  ASSERT_EQ(ctl_.motionStatus(), sfc::MotionStatus::RUNNING);
  ctl_.injectModeCommand(msg::ModeCommand::MODE_IDLE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(kT0 + kDt);
  EXPECT_EQ(ctl_.motionStatus(), sfc::MotionStatus::INACTIVE);
  EXPECT_EQ(cmd_[3], 0.0);
  EXPECT_FALSE(ctl_.engagedNow());
}

TEST(CccSpeedScale, ClampsOutOfRangeValues) {
  EXPECT_EQ(CartesianCorrectionController::clampSpeedScale(0.0), 1.0);
  EXPECT_EQ(CartesianCorrectionController::clampSpeedScale(-0.5), 1.0);
  EXPECT_EQ(CartesianCorrectionController::clampSpeedScale(1.5), 1.0);
  EXPECT_EQ(CartesianCorrectionController::clampSpeedScale(0.5), 0.5);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `cartesian_correction_controller.h`。

- [ ] **Step 3: 写实现**

`ros_ws/src/soft_robot_controllers/include/soft_robot_controllers/cartesian_correction_controller.h`:

```cpp
#pragma once

#include <actionlib/server/simple_action_server.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <soft_robot_msgs/CartesianCorrectionStamped.h>
#include <soft_robot_msgs/ModeCommand.h>
#include <soft_robot_msgs/MoveToOrientationAction.h>
#include <soft_robot_msgs/RsiState.h>

#include <atomic>
#include <cstdint>
#include <memory>

#include "kuka_rsi_hw_interface/cartesian_command_interface.h"
#include "soft_robot_controllers/controller_mode_gate.h"
#include "soft_robot_controllers/direct_correction_core.h"

namespace soft_robot_controllers {

struct CartesianCorrectionControllerParams {
  DirectCorrectionParams direct;
  // Template for action goals: a/b/c are overwritten per goal and
  // max_speed_dps is scaled by the goal's speed_scale (decision 8).
  sfc::MotionGoal goal_defaults;
};

// Goal/cancel request handed from the action thread to update() through a
// RealtimeBuffer. seq == 0 means "no request yet".
struct GoalRequest {
  sfc::MotionGoal goal;
  bool cancel{false};
  std::uint64_t seq{0};
};

// Direct Cartesian correction controller (spec 5.3, 7.6, 7.7): stream
// passthrough for commissioning/jog, plus the RKorr goal mode hosting
// OrientationMotionCore behind /soft_robot/move_to_orientation. Engaged
// in DIRECT_CARTESIAN and CALIBRATION modes. The action execute thread
// never touches realtime data structures: goals travel through a
// RealtimeBuffer, status travels back through atomics (decision 5).
class CartesianCorrectionController
    : public controller_interface::Controller<
          kuka_rsi::CartesianCorrectionCommandInterface> {
 public:
  bool init(kuka_rsi::CartesianCorrectionCommandInterface* hw,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  // ROS-master-free wiring entry used by init() and by offline tests.
  bool configureController(const kuka_rsi::CartesianCorrectionHandle& handle,
                           const CartesianCorrectionControllerParams& params);

  // Non-RT producers (subscriber callbacks and tests).
  void injectStream(const StreamCommand& cmd) {
    stream_buf_.writeFromNonRT(cmd);
  }
  void injectModeCommand(std::uint8_t mode, std::uint8_t profile);
  void injectRsiFault(bool fault) {
    fault_buf_.writeFromNonRT(FaultFlag{fault});
  }

  // Goal plumbing shared by the action execute thread and offline tests.
  std::uint64_t requestGoal(const sfc::MotionGoal& g);
  std::uint64_t requestCancel();
  std::uint64_t appliedGoalSeq() const { return applied_seq_pub_.load(); }
  sfc::MotionStatus motionStatus() const {
    return static_cast<sfc::MotionStatus>(status_.load());
  }
  double motionErrorDeg() const;
  bool engagedNow() const { return engaged_flag_.load(); }

  static double clampSpeedScale(double scale) {
    return (scale <= 0.0 || scale > 1.0) ? 1.0 : scale;
  }

 private:
  using ActionServer =
      actionlib::SimpleActionServer<soft_robot_msgs::MoveToOrientationAction>;

  void streamCb(
      const soft_robot_msgs::CartesianCorrectionStamped::ConstPtr& msg);
  void modeCb(const soft_robot_msgs::ModeCommand::ConstPtr& msg);
  void rsiStateCb(const soft_robot_msgs::RsiState::ConstPtr& msg);
  void executeCb(const soft_robot_msgs::MoveToOrientationGoalConstPtr& goal);
  sfc::CartesianState readState() const;
  void setZero();

  kuka_rsi::CartesianCorrectionHandle handle_;
  ControllerModeGate gate_{sfc::ControlMode::DIRECT_CARTESIAN,
                           sfc::ControlMode::CALIBRATION};
  DirectCorrectionCore core_;
  CartesianCorrectionControllerParams params_;

  realtime_tools::RealtimeBuffer<StreamCommand> stream_buf_;
  realtime_tools::RealtimeBuffer<ModeRequest> mode_buf_;
  realtime_tools::RealtimeBuffer<FaultFlag> fault_buf_;
  realtime_tools::RealtimeBuffer<GoalRequest> goal_buf_;
  std::uint64_t mode_seq_{0};                  // producer side
  std::atomic<std::uint64_t> goal_seq_{0};     // producer side (AS thread)
  std::uint64_t applied_goal_seq_{0};          // RT side only
  std::atomic<std::uint64_t> applied_seq_pub_{0};   // RT -> AS
  std::atomic<int> status_{static_cast<int>(sfc::MotionStatus::INACTIVE)};
  std::atomic<std::uint64_t> error_bits_{0};        // packed double
  std::atomic<bool> engaged_flag_{false};

  ros::Subscriber stream_sub_;
  ros::Subscriber mode_sub_;
  ros::Subscriber rsi_sub_;
  std::unique_ptr<ActionServer> action_server_;
};

}  // namespace soft_robot_controllers
```

`ros_ws/src/soft_robot_controllers/src/cartesian_correction_controller.cpp`:

```cpp
#include "soft_robot_controllers/cartesian_correction_controller.h"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cstring>
#include <string>

namespace soft_robot_controllers {

namespace {

std::uint64_t packDouble(double v) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  return bits;
}

double unpackDouble(std::uint64_t bits) {
  double v = 0.0;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

void loadSafety(ros::NodeHandle& nh, sfc::SafetyParams& s) {
  nh.param("safety/max_corr_trans", s.max_corr_trans, s.max_corr_trans);
  nh.param("safety/max_corr_rot", s.max_corr_rot, s.max_corr_rot);
  nh.param("safety/force_ceiling", s.force_ceiling, s.force_ceiling);
  nh.param("safety/torque_ceiling", s.torque_ceiling, s.torque_ceiling);
}

}  // namespace

bool CartesianCorrectionController::init(
    kuka_rsi::CartesianCorrectionCommandInterface* hw, ros::NodeHandle& root_nh,
    ros::NodeHandle& controller_nh) {
  CartesianCorrectionControllerParams p;
  controller_nh.param("stream_timeout", p.direct.stream_timeout_s,
                      p.direct.stream_timeout_s);
  loadSafety(controller_nh, p.direct.safety);
  controller_nh.param("goal/max_speed_dps", p.goal_defaults.max_speed_dps,
                      p.goal_defaults.max_speed_dps);
  controller_nh.param("goal/p_gain", p.goal_defaults.p_gain,
                      p.goal_defaults.p_gain);
  controller_nh.param("goal/tol_deg", p.goal_defaults.tol_deg,
                      p.goal_defaults.tol_deg);
  controller_nh.param("goal/hold_s", p.goal_defaults.hold_s,
                      p.goal_defaults.hold_s);
  controller_nh.param("goal/timeout_s", p.goal_defaults.timeout_s,
                      p.goal_defaults.timeout_s);

  std::string resource;
  controller_nh.param<std::string>("cartesian_resource", resource,
                                   std::string("kuka_tcp"));
  kuka_rsi::CartesianCorrectionHandle handle;
  try {
    handle = hw->getHandle(resource);
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("CartesianCorrectionController: " << ex.what());
    return false;
  }
  if (!configureController(handle, p)) return false;

  std::string command_topic;
  std::string mode_topic;
  std::string rsi_topic;
  std::string action_name;
  controller_nh.param<std::string>(
      "command_topic", command_topic,
      std::string("/soft_robot/cartesian_correction_command"));
  controller_nh.param<std::string>("mode_command_topic", mode_topic,
                                   std::string("/soft_robot/mode_command"));
  controller_nh.param<std::string>("rsi_state_topic", rsi_topic,
                                   std::string("/kuka/rsi/state"));
  controller_nh.param<std::string>(
      "action_name", action_name,
      std::string("/soft_robot/move_to_orientation"));
  stream_sub_ = root_nh.subscribe(
      command_topic, 1, &CartesianCorrectionController::streamCb, this);
  mode_sub_ = root_nh.subscribe(mode_topic, 1,
                                &CartesianCorrectionController::modeCb, this);
  rsi_sub_ = root_nh.subscribe(
      rsi_topic, 1, &CartesianCorrectionController::rsiStateCb, this);
  action_server_.reset(new ActionServer(
      root_nh, action_name,
      [this](const soft_robot_msgs::MoveToOrientationGoalConstPtr& g) {
        executeCb(g);
      },
      false));
  action_server_->start();
  return true;
}

bool CartesianCorrectionController::configureController(
    const kuka_rsi::CartesianCorrectionHandle& handle,
    const CartesianCorrectionControllerParams& params) {
  handle_ = handle;
  params_ = params;
  core_.configure(params_.direct);
  stream_buf_.writeFromNonRT(StreamCommand{});
  mode_buf_.writeFromNonRT(ModeRequest{});
  fault_buf_.writeFromNonRT(FaultFlag{});
  goal_buf_.writeFromNonRT(GoalRequest{});
  status_.store(static_cast<int>(sfc::MotionStatus::INACTIVE));
  applied_seq_pub_.store(0);
  error_bits_.store(packDouble(0.0));
  engaged_flag_.store(false);
  return true;
}

void CartesianCorrectionController::injectModeCommand(std::uint8_t mode,
                                                      std::uint8_t profile) {
  ModeRequest r;
  r.mode = mode;
  r.profile = profile;
  r.seq = ++mode_seq_;
  mode_buf_.writeFromNonRT(r);
}

std::uint64_t CartesianCorrectionController::requestGoal(
    const sfc::MotionGoal& g) {
  GoalRequest r;
  r.goal = g;
  r.cancel = false;
  r.seq = ++goal_seq_;
  goal_buf_.writeFromNonRT(r);
  return r.seq;
}

std::uint64_t CartesianCorrectionController::requestCancel() {
  GoalRequest r;
  r.cancel = true;
  r.seq = ++goal_seq_;
  goal_buf_.writeFromNonRT(r);
  return r.seq;
}

double CartesianCorrectionController::motionErrorDeg() const {
  return unpackDouble(error_bits_.load());
}

void CartesianCorrectionController::streamCb(
    const soft_robot_msgs::CartesianCorrectionStamped::ConstPtr& msg) {
  StreamCommand c;
  c.correction.x = msg->correction.x;
  c.correction.y = msg->correction.y;
  c.correction.z = msg->correction.z;
  c.correction.a = msg->correction.a;
  c.correction.b = msg->correction.b;
  c.correction.c = msg->correction.c;
  c.stamp_s = msg->header.stamp.isZero() ? ros::Time::now().toSec()
                                         : msg->header.stamp.toSec();
  c.valid = true;
  injectStream(c);
}

void CartesianCorrectionController::modeCb(
    const soft_robot_msgs::ModeCommand::ConstPtr& msg) {
  injectModeCommand(msg->mode, msg->profile);
}

void CartesianCorrectionController::rsiStateCb(
    const soft_robot_msgs::RsiState::ConstPtr& msg) {
  injectRsiFault(msg->fault);
}

sfc::CartesianState CartesianCorrectionController::readState() const {
  sfc::CartesianState s;
  s.x = handle_.getX();
  s.y = handle_.getY();
  s.z = handle_.getZ();
  s.a = handle_.getA();
  s.b = handle_.getB();
  s.c = handle_.getC();
  return s;
}

void CartesianCorrectionController::setZero() {
  handle_.setCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void CartesianCorrectionController::starting(const ros::Time& /*time*/) {
  setZero();
  core_.reset();
}

void CartesianCorrectionController::update(const ros::Time& time,
                                           const ros::Duration& period) {
  const ModeRequest req = *mode_buf_.readFromRT();
  if (gate_.apply(req)) core_.reset();  // entered: drop stale stream/goal
  const bool fault = fault_buf_.readFromRT()->fault;
  const bool engaged = gate_.engaged() && !fault;
  engaged_flag_.store(engaged);

  const GoalRequest gr = *goal_buf_.readFromRT();
  if (gr.seq != 0 && gr.seq != applied_goal_seq_) {
    applied_goal_seq_ = gr.seq;
    if (gr.cancel) {
      core_.cancelGoal();
    } else {
      core_.startGoal(gr.goal);
    }
  }

  const sfc::CartesianState state = readState();
  if (!engaged) {
    // Disengaging cancels a running goal. The action layer maps the
    // resulting INACTIVE to ABORTED unless it cancelled itself
    // (Plan 1 follow-up 5 bookkeeping, decision 5).
    if (core_.goalStatus() == sfc::MotionStatus::RUNNING) core_.cancelGoal();
    setZero();
  } else {
    core_.setStream(*stream_buf_.readFromRT());
    // Plan 2 follow-up 1: dt is the measured period, never a constant.
    const DirectOutput out =
        core_.update(state, time.toSec(), period.toSec());
    handle_.setCommand(out.correction.x, out.correction.y, out.correction.z,
                       out.correction.a, out.correction.b, out.correction.c);
  }
  status_.store(static_cast<int>(core_.goalStatus()));
  error_bits_.store(packDouble(core_.goalErrorDeg(state)));
  applied_seq_pub_.store(applied_goal_seq_);
}

void CartesianCorrectionController::stopping(const ros::Time& /*time*/) {
  setZero();
  gate_.forceIdle();
  requestCancel();  // a goal must not survive a controller stop
}

void CartesianCorrectionController::executeCb(
    const soft_robot_msgs::MoveToOrientationGoalConstPtr& goal) {
  soft_robot_msgs::MoveToOrientationResult result;
  if (goal->use_position) {
    result.result_code = soft_robot_msgs::MoveToOrientationResult::ABORTED;
    result.message =
        "position targets are not supported in v1 (orientation only)";
    action_server_->setAborted(result);
    return;
  }
  if (!engagedNow()) {
    result.result_code = soft_robot_msgs::MoveToOrientationResult::ABORTED;
    result.message = "controller is not engaged (mode gate)";
    action_server_->setAborted(result);
    return;
  }
  sfc::MotionGoal g = params_.goal_defaults;
  g.a = goal->a;
  g.b = goal->b;
  g.c = goal->c;
  g.max_speed_dps =
      params_.goal_defaults.max_speed_dps * clampSpeedScale(goal->speed_scale);
  const std::uint64_t seq = requestGoal(g);

  const ros::Time deadline =
      ros::Time::now() + ros::Duration(g.timeout_s + 5.0);
  bool cancelled = false;
  double initial_error = -1.0;
  ros::Rate rate(50);
  while (ros::ok()) {
    if (ros::Time::now() > deadline) {
      requestCancel();
      result.result_code = soft_robot_msgs::MoveToOrientationResult::ABORTED;
      result.message = "realtime loop did not process the goal in time";
      action_server_->setAborted(result);
      return;
    }
    if (!cancelled && action_server_->isPreemptRequested()) {
      requestCancel();
      cancelled = true;
    }
    if (appliedGoalSeq() >= seq) {
      const sfc::MotionStatus st = motionStatus();
      if (st == sfc::MotionStatus::CONVERGED) {
        result.result_code =
            soft_robot_msgs::MoveToOrientationResult::CONVERGED;
        result.message = "converged";
        action_server_->setSucceeded(result);
        return;
      }
      if (st == sfc::MotionStatus::TIMEOUT) {
        result.result_code = soft_robot_msgs::MoveToOrientationResult::TIMEOUT;
        result.message = "motion timed out before convergence";
        action_server_->setAborted(result);
        return;
      }
      if (st == sfc::MotionStatus::INACTIVE) {
        // INACTIVE cannot distinguish our own cancel from a realtime-side
        // cancel (Plan 1 follow-up 5): the shell keeps the bookkeeping.
        result.result_code = soft_robot_msgs::MoveToOrientationResult::ABORTED;
        if (cancelled) {
          result.message = "preempted";
          action_server_->setPreempted(result);
        } else {
          result.message = "goal cancelled by mode change or fault";
          action_server_->setAborted(result);
        }
        return;
      }
      const double err = motionErrorDeg();
      if (initial_error < 0.0 && err > 0.0) initial_error = err;
      soft_robot_msgs::MoveToOrientationFeedback fb;
      fb.error_deg = err;
      fb.error_mm = 0.0;
      fb.progress =
          initial_error > 0.0
              ? std::max(0.0, std::min(1.0, 1.0 - err / initial_error))
              : 0.0;
      action_server_->publishFeedback(fb);
    }
    rate.sleep();
  }
  requestCancel();  // node shutdown while a goal was active
}

}  // namespace soft_robot_controllers

PLUGINLIB_EXPORT_CLASS(soft_robot_controllers::CartesianCorrectionController,
                       controller_interface::ControllerBase)
```

- [ ] **Step 4: CMake 增量**

库源列表加 `src/cartesian_correction_controller.cpp`;测试段追加:

```cmake
  catkin_add_gtest(test_cartesian_correction_controller
                   test/test_cartesian_correction_controller.cpp)
  target_link_libraries(test_cartesian_correction_controller ${PROJECT_NAME}
                        ${catkin_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_controllers/test_cartesian_correction_controller
```

预期:`[  PASSED  ] 9 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers && \
git commit -m "feat(controllers): add CartesianCorrectionController with stream topic and MoveToOrientation action (Plan 3 Task 8)"
```

---

### Task 9: 离线全链路闭环(控制器 × `KukaRsiRobotHW` × `RsiMockServer`)+ mock 库导出勘误

**Files:**
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`(勘误,见下)
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_controllers/test/test_controller_chain.cpp`

**Interfaces:**
- Consumes: Task 7/8 控制器、Plan 2 `KukaRsiRobotHW::configure()`(loopback、port 0 自动分配)、`RsiMockServer`(KRC 角色)、`hw.get<CartesianCorrectionCommandInterface>()->getHandle("kuka_tcp")`。
- 无新生产接口——纯测试任务:验证 wrench → 控制器 → 硬件接口 → UDP → mock 位姿积分的整条链,以及硬截断/goal 收敛在链路级的表现。
- **勘误(性质同 Plan 2 跟进 2 的"测试需要就补"许可,待确认 7)**:`kuka_rsi_hw_interface` 的 `catkin_package(LIBRARIES ...)` 未导出 `kuka_rsi_hw_interface_mock`,跨包测试无法链接 `RsiMockServer`。本 Task 将其加入导出列表——纯构建导出变更,不改任何源代码与行为,Plan 2 的全部测试不受影响。`RsiMockCore::setJointAngles` 决定不加:本计划控制器不消费关节状态(待确认 7)。
- TDD 说明:同 Task 4/9 惯例,集成测试预期直接通过;失败则修 Task 3-8 实现,不修测试语义。

- [ ] **Step 1: 勘误——导出 mock 库**

`ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt` 中:

```cmake
# 原:
#   LIBRARIES kuka_rsi_protocol kuka_rsi_robot_hw
# 改为:
  LIBRARIES kuka_rsi_protocol kuka_rsi_robot_hw kuka_rsi_hw_interface_mock
```

- [ ] **Step 2: 写链路测试**

`ros_ws/src/soft_robot_controllers/test/test_controller_chain.cpp`:

```cpp
#include <gtest/gtest.h>

#include <soft_robot_msgs/ModeCommand.h>

#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"
#include "kuka_rsi_hw_interface/rsi_mock_server.h"
#include "soft_robot_controllers/cartesian_correction_controller.h"
#include "soft_robot_controllers/force_compliance_controller.h"

using soft_robot_controllers::CartesianCorrectionController;
using soft_robot_controllers::CartesianCorrectionControllerParams;
using soft_robot_controllers::ForceComplianceController;
using soft_robot_controllers::ForceComplianceParams;
using soft_robot_controllers::WrenchSample;
namespace msg = soft_robot_msgs;

namespace {

kuka_rsi::HwConfig hwConfig() {
  kuka_rsi::HwConfig cfg;
  cfg.listen_ip = "127.0.0.1";
  cfg.listen_port = 0;        // auto-assign; mock targets hw.listenPort()
  cfg.read_timeout_ms = 100;  // generous: mock thread scheduling jitter
  cfg.max_consecutive_timeouts = 5;
  return cfg;
}

ForceComplianceParams precisionParams() {
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
  p.wrench_timeout_s = 0.012;
  return p;
}

CartesianCorrectionControllerParams directParams() {
  CartesianCorrectionControllerParams p;
  p.direct.stream_timeout_s = 0.1;
  p.direct.safety.max_corr_trans = 0.5;
  p.direct.safety.max_corr_rot = 0.05;
  p.goal_defaults.max_speed_dps = 7.5;
  p.goal_defaults.p_gain = 20.0;
  p.goal_defaults.tol_deg = 0.1;
  p.goal_defaults.hold_s = 0.008;
  p.goal_defaults.timeout_s = 30.0;
  return p;
}

}  // namespace

TEST(ControllerChain, CompliancePushesMockPose) {
  ros::Time::init();
  kuka_rsi::KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  kuka_rsi::RsiMockServer mock(kuka_rsi::MockConfig{}, "127.0.0.1",
                               hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  ForceComplianceController ctl;
  const kuka_rsi::CartesianCorrectionHandle handle =
      hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()->getHandle(
          "kuka_tcp");
  ASSERT_TRUE(ctl.configureController(handle, precisionParams(),
                                      precisionParams()));
  ctl.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                        msg::ModeCommand::PROFILE_PRECISION);
  ctl.starting(ros::Time(0.0));

  double t = 0.0;
  for (int i = 0; i < 50; ++i) {
    t += 0.004;
    hw.read(ros::Time(t), ros::Duration(0.004));
    WrenchSample s;
    s.w.fx = 40.0;  // e = 10 N -> +0.04 mm on X per cycle
    s.stamp_s = t;
    s.valid = true;
    ctl.injectWrench(s);
    ctl.update(ros::Time(t), ros::Duration(0.004));
    hw.write(ros::Time(t), ros::Duration(0.004));
  }
  mock.stop();

  EXPECT_FALSE(hw.faulted());
  EXPECT_EQ(mock.statsSnapshot().ipoc_echo_errors, 0u);
  // ~50 * 0.04 mm; tolerate a few cycles lost to thread startup/shutdown.
  EXPECT_GE(mock.poseX(), 45 * 0.04);
}

TEST(ControllerChain, HardCutoffFreezesMockPose) {
  ros::Time::init();
  kuka_rsi::KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  kuka_rsi::RsiMockServer mock(kuka_rsi::MockConfig{}, "127.0.0.1",
                               hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  ForceComplianceController ctl;
  ASSERT_TRUE(ctl.configureController(
      hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()->getHandle(
          "kuka_tcp"),
      precisionParams(), precisionParams()));
  ctl.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                        msg::ModeCommand::PROFILE_PRECISION);
  ctl.starting(ros::Time(0.0));

  double t = 0.0;
  for (int i = 0; i < 20; ++i) {
    t += 0.004;
    hw.read(ros::Time(t), ros::Duration(0.004));
    WrenchSample s;
    s.w.fx = 600.0;  // above the 500 N ceiling: hard cutoff (spec 12.1)
    s.stamp_s = t;
    s.valid = true;
    ctl.injectWrench(s);
    ctl.update(ros::Time(t), ros::Duration(0.004));
    hw.write(ros::Time(t), ros::Duration(0.004));
  }
  mock.stop();

  EXPECT_DOUBLE_EQ(mock.poseX(), 0.0);  // zero correction throughout
}

TEST(ControllerChain, GoalMotionConvergesOnMock) {
  ros::Time::init();
  kuka_rsi::KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  kuka_rsi::RsiMockServer mock(kuka_rsi::MockConfig{}, "127.0.0.1",
                               hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  CartesianCorrectionController ctl;
  const kuka_rsi::CartesianCorrectionHandle handle =
      hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()->getHandle(
          "kuka_tcp");
  ASSERT_TRUE(ctl.configureController(handle, directParams()));
  ctl.injectModeCommand(msg::ModeCommand::MODE_DIRECT_CARTESIAN,
                        msg::ModeCommand::PROFILE_PRECISION);
  ctl.starting(ros::Time(0.0));

  sfc::MotionGoal g = directParams().goal_defaults;
  g.a = 1.0;  // +1 deg on A, driven through RKorr and the mock's pose
  const std::uint64_t seq = ctl.requestGoal(g);

  double t = 0.0;
  bool converged = false;
  for (int i = 0; i < 300 && !converged; ++i) {
    t += 0.004;
    hw.read(ros::Time(t), ros::Duration(0.004));
    ctl.update(ros::Time(t), ros::Duration(0.004));
    hw.write(ros::Time(t), ros::Duration(0.004));
    converged = ctl.appliedGoalSeq() >= seq &&
                ctl.motionStatus() == sfc::MotionStatus::CONVERGED;
  }
  mock.stop();

  EXPECT_TRUE(converged);
  // The hardware state interface tracked the mock's integrated pose.
  EXPECT_NEAR(handle.getA(), 1.0, 0.2);
}
```

- [ ] **Step 3: CMake 增量(soft_robot_controllers)**

```cmake
  catkin_add_gtest(test_controller_chain test/test_controller_chain.cpp)
  target_link_libraries(test_controller_chain ${PROJECT_NAME}
                        ${catkin_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
```

(勘误后 `kuka_rsi_hw_interface_mock` 经 `${catkin_LIBRARIES}` 进入链接。)

- [ ] **Step 4: 构建运行(含 Plan 2 回归)**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_controllers/test_controller_chain && \
  ./devel/lib/kuka_rsi_hw_interface/test_rsi_integration
```

预期:两个二进制分别 `[  PASSED  ] 3 tests.` 与 `[  PASSED  ] 4 tests.`(勘误未破坏 Plan 2)。

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "test(controllers): add offline closed-loop chain tests; export RSI mock library (Plan 3 Task 9)"
```

---

### Task 10: pluginlib 注册 + controllers 配置 + 安装规则 + 全量回归

**Files:**
- Create: `ros_ws/src/soft_robot_controllers/soft_robot_controllers_plugins.xml`
- Create: `ros_ws/src/soft_robot_controllers/config/soft_robot_controllers.yaml`
- Modify: `ros_ws/src/soft_robot_controllers/CMakeLists.txt`
- 本 Task 无 gtest(纯声明/配置,行为已被 Task 1-9 覆盖);验证 = 零警告构建 + `rospack` 插件可见性 + 全套件回归 + 手动 spawner 冒烟(需 roscore,不进验收门槛)。

- [ ] **Step 1: 写插件声明**

`ros_ws/src/soft_robot_controllers/soft_robot_controllers_plugins.xml`:

```xml
<library path="lib/libsoft_robot_controllers">
  <class name="soft_robot_controllers/ForceComplianceController"
         type="soft_robot_controllers::ForceComplianceController"
         base_class_type="controller_interface::ControllerBase">
    <description>
      SRI force-compliance controller: admittance pipeline over the
      gravity-compensated wrench, DRAG/PRECISION profiles, adaptive
      startup deadband, edge-triggered auto re-tare, 500 N hard cutoff
      (spec sections 7, 12.1).
    </description>
  </class>
  <class name="soft_robot_controllers/CartesianCorrectionController"
         type="soft_robot_controllers::CartesianCorrectionController"
         base_class_type="controller_interface::ControllerBase">
    <description>
      Direct Cartesian correction controller: stream passthrough with
      command timeout, plus RKorr goal mode hosting OrientationMotionCore
      behind the /soft_robot/move_to_orientation action (spec sections
      5.3, 7.6, 7.7).
    </description>
  </class>
</library>
```

- [ ] **Step 2: 写 controllers 配置**

`ros_ws/src/soft_robot_controllers/config/soft_robot_controllers.yaml`:

```yaml
# Controller parameters for the soft robot system (spec section 14,
# controllers.yaml + force_control.yaml migration keys). Both controllers
# claim the exclusive kuka_tcp correction resource: only one of them can
# be RUNNING at a time; the manager (Plan 5) swaps them on mode change.
force_compliance_controller:
  type: soft_robot_controllers/ForceComplianceController
  cartesian_resource: kuka_tcp
  wrench_topic: /sri_ft/wrench_raw
  mode_command_topic: /soft_robot/mode_command
  mode_state_topic: /soft_robot/mode_state
  rsi_state_topic: /kuka/rsi/state
  wrench_timeout: 0.012          # s; ~3 RSI cycles (spec section 8)
  safety:
    max_corr_trans: 0.5          # mm per cycle per axis
    max_corr_rot: 0.05           # deg per cycle per axis
    force_ceiling: 500.0         # N; legacy hard cutoff (spec 12.1)
    torque_ceiling: 50.0         # Nm
  payload:                       # fallback until calibration (Plan 5)
    gravity_n: 0.0
    com_x: 0.0
    com_y: 0.0
    com_z: 0.0
    bias_fx: 0.0
    bias_fy: 0.0
    bias_fz: 0.0
    bias_tx: 0.0
    bias_ty: 0.0
    bias_tz: 0.0
  profiles:
    drag:                        # legacy modes 1-2 (spec 7.3)
      filter_cutoff_hz: 5.0
      speed_scale: 1.0
      gain_translation: 0.4      # (mm/s)/N; commissioning refines these
      gain_rotation: 0.8         # (deg/s)/Nm
      max_speed_translation: 30.0
      max_speed_rotation: 10.0
      max_accel_translation: 200.0
      max_accel_rotation: 100.0
      adaptive_deadband: true
      ramp_window_s: 2.0         # legacy 500 x 4 ms LIMSET window
      ramp_force_margin_n: 5.0   # legacy FTLimSet(5+FSum, ...)
      ramp_torque_margin_nm: 1.0
      auto_retare: true
      retare_orientation_tol_deg: 1.0
      retare_rearm_factor: 2.0
    precision:                   # legacy modes 3-7 (spec 7.3, section 10)
      filter_cutoff_hz: 10.0
      speed_scale: 1.0
      gain_translation: 0.1
      gain_rotation: 0.2
      max_speed_translation: 10.0
      max_speed_rotation: 2.0
      max_accel_translation: 100.0
      max_accel_rotation: 50.0
      deadband_force: 30.0       # fixed (legacy LIMSET hardcoded limits)
      deadband_torque: 4.0
      adaptive_deadband: false
      auto_retare: false

cartesian_correction_controller:
  type: soft_robot_controllers/CartesianCorrectionController
  cartesian_resource: kuka_tcp
  command_topic: /soft_robot/cartesian_correction_command
  mode_command_topic: /soft_robot/mode_command
  rsi_state_topic: /kuka/rsi/state
  action_name: /soft_robot/move_to_orientation
  stream_timeout: 0.1            # s; zero output on stale command (spec 7.7)
  safety:
    max_corr_trans: 0.5
    max_corr_rot: 0.05
    force_ceiling: 500.0         # inert here (no wrench), kept for symmetry
    torque_ceiling: 50.0
  goal:                          # spec section 14 controllers.yaml keys
    max_speed_dps: 5.0
    p_gain: 1.0
    tol_deg: 0.1
    hold_s: 0.2
    timeout_s: 30.0
```

- [ ] **Step 3: CMake 安装规则**

`CMakeLists.txt` 库目标之后追加:

```cmake
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES soft_robot_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

- [ ] **Step 4: 零警告构建 + 插件可见性 + 全套件回归**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make 2>&1 | grep -E "warning|error"; \
  catkin_make tests && source devel/setup.bash && \
  rospack plugins --attrib=plugin controller_interface | grep soft_robot_controllers
```

预期:无 warning/error;`rospack` 输出含 `soft_robot_controllers .../soft_robot_controllers_plugins.xml`。

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make run_tests && \
  catkin_test_results build/test_results
```

预期:`Summary: ... 0 errors, 0 failures`(Plan 1 52 例 + Plan 2 58 例 + 本计划 61 例 = 171)。

- [ ] **Step 5: 手动冒烟(需 roscore,不进验收门槛)**

```bash
# terminal 1
roscore
# terminal 2: RSI hardware node (listens on 49152)
cd /home/ljj/kuka_iiqka_ros/ros_ws && source devel/setup.bash && \
  roslaunch kuka_rsi_hw_interface kuka_rsi.launch
# terminal 3: KRC-side mock
cd /home/ljj/kuka_iiqka_ros/ros_ws && source devel/setup.bash && \
  ./devel/lib/kuka_rsi_hw_interface/kuka_rsi_sim_server \
    --target-ip 127.0.0.1 --target-port 49152
# terminal 4: load params + spawn ONE controller (exclusive kuka_tcp claim)
cd /home/ljj/kuka_iiqka_ros/ros_ws && source devel/setup.bash && \
  rosparam load src/soft_robot_controllers/config/soft_robot_controllers.yaml && \
  rosrun controller_manager spawner force_compliance_controller
# terminal 5: engage the mode, feed a wrench, watch the state
rostopic pub -1 /soft_robot/mode_command soft_robot_msgs/ModeCommand \
  "{mode: 2, profile: 1}"
rostopic pub -r 250 /sri_ft/wrench_raw geometry_msgs/WrenchStamped \
  "{header: {stamp: now}, wrench: {force: {x: 40.0}}}" &
rostopic echo -n 3 /soft_robot/mode_state
```

预期:`mode_state` 显示 `mode: 2, system_state: 3`(SERVOING);sim server 的 pose x 递增且 `echo_err=0`。停掉 wrench 发布后(超时)`system_state: 5`(DEGRADED)且 pose 停走。两控制器同时 spawn 第二个会因 `kuka_tcp` 独占 claim 被 controller_manager 拒绝——这是规格 §5.3 的预期行为。

- [ ] **Step 6: 全量回归 + Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_controllers && \
git commit -m "feat(controllers): add pluginlib registration, controllers config, install rules (Plan 3 Task 10)"
```

---

## 验收清单

| Task | 测试二进制 | 用例数 |
|---|---|---|
| 1 | `soft_robot_controllers/test_mode_bridge` | 4 |
| 2 | `soft_robot_controllers/test_controller_mode_gate` | 7 |
| 3 | `soft_robot_controllers/test_force_compliance_core` | 9 |
| 4 | `soft_robot_controllers/test_fcc_adaptive_retare` | 9 |
| 5 | `soft_robot_controllers/test_fcc_orientation_sum` | 4 |
| 6 | `soft_robot_controllers/test_direct_correction_core` | 9 |
| 7 | `soft_robot_controllers/test_force_compliance_controller` | 7 |
| 8 | `soft_robot_controllers/test_cartesian_correction_controller` | 9 |
| 9 | `soft_robot_controllers/test_controller_chain` | 3 |
| **合计** | **9 个二进制** | **61** |

验收条件:

- `catkin_make`、`catkin_make tests` 零警告;`catkin_make run_tests` 0 errors / 0 failures(含 Plan 1/2 存量 110 例,总计 171)。
- 全部测试离线可跑(无 roscore、无真机;Task 9 仅 127.0.0.1)。
- 跟进事项逐条闭环:dt 实测 period(T3/T7 各一条测试)、AutoReTare 边沿触发(T4 四条)、两路求和先于 SafetyLimiter(T5)、profile 切换滤波器重建+reset(T4/T7)、硬截断严格大于(T3)、故障感知经 `/kuka/rsi/state`(T7/T8)、常量对齐 static_assert(T1)、`setJointAngles` 不加 + mock 库导出勘误(T9)。
- 规格对照:§5.3 两控制器 + 独占 claim;§7.1 update 管线与实时约束;§7.3-7.5 profile/ramp/re-tare;§7.6 goal 模式 + `/soft_robot/move_to_orientation`;§7.7 stream 模式 + 超时归零;§10 模式映射(gate 接管集);§12.1 控制器层安全全项(幅值/速率经 ComplianceLaw+SafetyLimiter、wrench 超时、硬截断、模式归零、饱和诊断)。
- `rospack plugins --attrib=plugin controller_interface` 可见两个插件;Task 10 Step 5 冒烟按文档可复现(手动,非门槛)。

## 遗留风险

1. **双控制器独占 `kuka_tcp`**:同时 RUNNING 会被 controller_manager 拒绝(预期);模式切换时的 stop/start 编排属 Plan 5,在此之前手动 spawner 只能跑一个。
2. **ModeCommand 电平语义**:RealtimeBuffer 只保留最新请求,连发多条会丢中间态;manager(Plan 5)需按 `ModeState` 确认后再发下一条。
3. **ModeState 过渡语义**:本计划由 `ForceComplianceController` 发布控制器视角的 READY/SERVOING/DEGRADED;Plan 5 manager 接管权威 `system_state` 后应将其降级为诊断或改话题名,避免双发布者。
4. **原子 double 打包假设 lock-free**:`std::atomic<uint64_t>` 在 x86-64/ARM64 lock-free;若换到不保证的平台,RT 侧 store 可能退化为加锁(违反 RT 约束),需改用 seqlock。
5. **action 通信路径无自动化测试**:`SimpleActionServer` execute 线程与 preempt 分支靠 Task 10 冒烟 + Plan 5 标定流程集成验证;goal 管道(requestGoal/状态回读)本身已被 T8/T9 覆盖。
6. **wrench 时戳依赖 SRI 驱动(Plan 4)**:驱动若不打戳,回调退化为接收时刻,超时检测对传输延迟不敏感;Plan 4 交付时须保证 `header.stamp` 为采样时刻。
7. **FCC 内嵌 OrientationMotionCore 未接线**:求和路径已实现并测试,但 v1 无 ROS 入口;将来接线时注意与 CALIBRATION 模式的控制器切换互斥。
8. **Euler wrap 与测地误差偏差(Plan 1 备忘)**:B≈±90° 时逐轴 wrap 误差与测地误差偏离,goal 模式大角度机动前需评估;标定序列的小角度回正不受影响。
9. **profile 只能经 IDLE 切换**:操作流程(UI/manager)须先回 IDLE 再换 profile,否则请求被拒(`lastRequestOk()=false`),仅记录不报错——Plan 5/6 需把拒绝反馈给操作者。

## 计划自查记录

- **API 签名逐一核对**(对照已交付头文件):`ForceTorqueFilter(double)`/`filter(w, dt)`/`reset()`;`ToolGravityCompensator::compensate(raw, a, b, c) const`/`absorbResidual(residual)`/`setParams`/`params()`;`ComplianceLaw::compute(w, params, dt)`(非 const,内部 prev_v_)/`reset()`;`SafetyLimiter::apply(corr, wrench, params) const` → `SafetyResult{correction, hard_cutoff, saturated}`;`AdaptiveDeadband::start(window, f_margin, t_margin)`/`update(w, dt)`→bool(ramping 中为 true)/`forceDeadband()`;`AutoReTare::setReference(a, b, c)`/`shouldTare(state, w, f_db, t_db, params) const`(无参考 getter → 核心自存 `ref_a_/b_/c_` 副本);`OrientationMotionCore::setGoal/cancel/status/update(state, dt)`,`MotionGoal{a, b, c, max_speed_dps, p_gain, tol_deg, hold_s, timeout_s}`;`ModeManagerCore::requestMode/setProfile(仅 IDLE)/snapshot`;`CartesianCorrectionHandle::setCommand(x, y, z, a, b, c)` + 继承的 `getX()..getC()`;`CartesianStateInterface::claim()` no-op 勘误已在实现中(本计划不依赖该行为);`KukaRsiRobotHW::configure/listenPort/read/write/faulted`;`RsiMockServer(cfg, ip, port, timeout)/start/stop/poseX/statsSnapshot`。
- **任务依赖顺序**:T1(bridge)→T2(gate)→T3(FCC core)→T4/T5(行为固化,仅依赖 T3)→T6(direct core,独立于 T3)→T7(依赖 T2/T3)→T8(依赖 T2/T6)→T9(依赖 T7/T8 + Plan 2)→T10(声明/配置)。无环。
- **离线可测性**:T1-T6 纯逻辑;T7/T8 经 `configureController` + `inject*`(与订阅回调同路径)绕开 master;T9 仅 loopback UDP;`ros::Time::init()` 使 `ros::Time` 可用而无需 master;唯一需要 roscore 的是 Task 10 Step 5 冒烟(明确标注,非门槛)。
- **数值自查**:T3 `40 N - 30 N 死区 → 10 mm/s → 0.04 mm@4 ms`;T4 ramp 5 周期(0.02 s)、死区=残差+5/1;T5 `0.03+0.03→clamp 0.05`(rotation gain 1.0、tz 7.5、motion 7.5 dps);T8/T9 goal `p_gain 20, max 7.5 dps → ≤0.03 deg/cycle`,1 deg 目标 <300 周期收敛。
- **单 CMake 库目标**:纯逻辑类与控制器壳同在 `libsoft_robot_controllers`(pluginlib path 一致);"纯逻辑"由头文件不含 `ros/ros.h` 保证(`mode_bridge`/`gate` 仅用消息常量头),无需拆库。
