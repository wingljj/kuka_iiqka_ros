# Plan 2/6: `soft_robot_msgs` + `kuka_rsi_hw_interface`(RSI 实时通道 + RSI Mock)实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标:** 按规格 `docs/superpowers/specs/2026-07-02-kuka-rsi-ros-control-design.md` §5.1、§6.1、§12.2、§15.2,交付全系统共享消息包 `soft_robot_msgs` 与 RSI 实时硬件抽象包 `kuka_rsi_hw_interface`(含 `hardware_interface::RobotHW` 实现、RSI UDP/XML 4 ms 实时通道、笛卡尔修正接口、看门狗/IPOC 连续性检查、二级命令限幅),以及可离线测试的 KRC 侧 RSI 行为模拟器(`RsiMockCore`/`RsiMockServer` + 独立可执行 `kuka_rsi_sim_server`)。

**架构:** 与 Plan 1 同理念的"纯逻辑类 + 薄 ROS 壳"分层:XML 帧编解码、IPOC/超时会话监控、命令限幅、UDP 传输均为不依赖 roscpp 运行时的独立类,逐个 gtest 单测;`KukaRsiRobotHW` 仅做组装,并提供免 ROS master 的 `configure()` 入口使 read()/write() 可在纯 gtest 中对着 loopback UDP mock 全链路验证。ROS 节点壳(`controller_manager` 循环 + 诊断发布)最后落盘。

**Plan series:** ① core algorithm library(已完成)→ ② `kuka_rsi_hw_interface` + RSI mock + msgs(本计划)→ ③ `soft_robot_controllers` → ④ `kuka_eki_bridge` + `sri_force_torque_driver` + mocks → ⑤ manager + calibration + bringup + KUKA 模板 → ⑥ web interface。

## 范围

**包清单(2 个新 catkin 包):**

| 包 | 内容 |
|---|---|
| `soft_robot_msgs` | 全系统共享 msg/srv/action:`CartesianState`、`CartesianCorrection(+Stamped)`、`RsiState`、`ModeCommand`、`ModeState`、`GetTool.srv`、`MoveToOrientation.action` |
| `kuka_rsi_hw_interface` | RSI 帧编解码、会话监控、命令限幅、UDP 传输、自定义 ros_control 笛卡尔接口、`KukaRsiRobotHW`、节点壳 + launch/yaml,以及 RSI mock 库与 `kuka_rsi_sim_server` 可执行 |

**非目标(本计划不做):**

- 任何控制器插件(`SoftComplianceController`/`CartesianCorrectionController` → Plan 3)。
- EKI 桥、SRI 驱动及其 mock(→ Plan 4)。
- manager 状态机、标定流程、`payload.yaml` 持久化(→ Plan 5)。
- KUKA 侧 KRL/RSI/EKI 模板文件(→ Plan 5)。
- rostest/roslaunch 级集成测试:本计划全部测试为离线 gtest(loopback UDP),节点壳只保证编译并给出手动冒烟命令。
- `AKorr`、`EKorr`、`Tech`、`DiO`、`Width` 字段(规格 §6.1 明确不迁移)。

**与其他 Plan 的接口关系:**

- 消费 Plan 1:**不消费其代码**(本计划无 `soft_force_control_core` 依赖);仅在语义上对齐 `sfc` 类型单位(mm/deg/N/Nm、A/B/C = Z-Y-X)与枚举取值(消息常量数值与 `sfc::ControlMode`/`sfc::Profile` 枚举序号一致)。
- 供给 Plan 3:`CartesianStateInterface` / `CartesianCorrectionCommandInterface`(控制器 claim 的硬件资源,资源名 `kuka_tcp`)、`soft_robot_msgs/CartesianCorrectionStamped`(stream 模式命令话题类型)、`MoveToOrientation.action`(goal 模式 action 类型)、`ModeCommand/ModeState`。
- 供给 Plan 4/5:`GetTool.srv`(EKI 工具查询)、`RsiState`(诊断聚合)、`kuka_rsi_sim_server`(无真机联调对端)。
- git 基线:从 `feature/soft-force-control-core` 分支新建 `feature/rsi-hw-interface-msgs`(工作区脚手架与 `ros_ws/.gitignore` 已在该分支上;合并顺序 Plan 1 → Plan 2)。

## 待确认(规格未定项,本计划采用的默认决策)

1. **mock 的归属**:不建独立 `kuka_rsi_mock` 包,mock 作为 `kuka_rsi_hw_interface` 的组件(独立库目标 `kuka_rsi_hw_interface_mock` + 可执行 `kuka_rsi_sim_server`,名字沿用规格 §15.2)。理由:mock 必须复用同一套帧编解码,单独成包会形成 `hw_interface --test_depend--> mock --depend--> hw_interface` 的循环依赖。
2. **Wrench 消息**:不自定义。力/力矩全链路复用 `geometry_msgs/WrenchStamped`(规格 §5.6 已指定 SRI 驱动发布该类型),避免平行类型;`soft_robot_msgs` 不包含 Wrench。
3. **关节状态单位**:`JointStateInterface` 按 ROS 惯例暴露弧度(KRC `AIPos` 的 deg 在 read() 内换算);笛卡尔状态/修正保持项目约定 mm/deg。
4. **Sen 帧字段**:按规格 §6.1 含 `RKorr` + `Stop S` + `Watchdog W` + `IPOC` 回显;`Watchdog` 为 PC 侧每周期自增的活性计数,`Stop=1` 表示 PC 侧已锁存故障、请求 KRC 停止。`Sen Type` 固定为 `"ROS"`(KRC 侧 RSI 配置须一致,列入遗留风险)。
5. **IPOC 语义**:不假设步长(旧系统/KRC 版本间步长不一致),连续性检查只要求严格递增;非递增计入 `ipoc_jumps` 但不中断会话。回显值 = 本周期收到的 IPOC。
6. **超时/故障策略**:read() 用 `poll` 带超时(默认 8 ms = 2 个 RSI 周期);连接建立后连续 `max_consecutive_timeouts`(默认 5)次超时或坏帧 → **锁存故障**,此后输出零修正 + `Stop=1`;故障复位属 Plan 4/5(EKI/manager)职责,本计划仅提供 `RsiSessionMonitor::reset()`。
7. **XML 解析器**:用系统 tinyxml2(6.2.0,pkg-config 定位)。解析发生在每周期 read() 内、存在内部分配——与 ros-industrial 参考驱动一致,作为**已声明的实时性偏差**记入遗留风险;收发缓冲区(1024 B)全部预分配。
8. **端口与坐标系**:UDP 监听端口默认 49152;修正坐标系(BASE)与相对修正模式属 KRC 侧 RSI 配置(Plan 5 交付),本计划的 PC 侧协议与其无耦合。
9. **ModeCommand/ModeState** 按最小字段定义(mode/profile/system_state + 常量);manager 级服务化(set_mode 等)留给 Plan 4/5。

## Global Constraints

- ROS1 Noetic,catkin 工作区 `/home/ljj/kuka_iiqka_ros/ros_ws`;新包位于 `ros_ws/src/soft_robot_msgs/`、`ros_ws/src/kuka_rsi_hw_interface/`。
- C++14,`-Wall -Wextra`,零警告;所有代码与注释英文。
- TDD:每 Task 先写失败测试(构建失败即失败态,同 Plan 1),再最小实现,再确认通过,再提交。
- **Noetic 的 `catkin_add_gtest` 不自动链接 gtest_main:所有测试链接行必须包含 `${GTEST_MAIN_LIBRARIES}`。**
- 单位:mm、deg(KUKA A/B/C = Z-Y-X)、N、Nm;RSI 周期 dt = 0.004 s;`JointStateInterface` 为弧度(决策 3)。
- 实时路径(read()/write() 及其调用的编解码/监控/限幅)禁止阻塞 I/O(UDP 为 poll 带超时)、磁盘 I/O、异常抛出、参数服务器访问;缓冲区预分配。唯一声明偏差:tinyxml2 解析的内部分配(决策 7)。
- 网络测试仅用 127.0.0.1,不依赖真机;测试内所有超时 ≤ 0.5 s,保证套件秒级完成。
- 构建/运行命令沿用 Plan 1(仓库根 `/home/ljj/kuka_iiqka_ros`):

```bash
cd ros_ws && catkin_make tests                       # build all test binaries
./devel/lib/soft_robot_msgs/<test_binary>            # run msgs tests
./devel/lib/kuka_rsi_hw_interface/<test_binary>      # run hw interface tests
```

- 本计划不使用 rostest;节点冒烟(需 roscore)单独标注,不进入验收门槛。

---

## File Structure

```text
ros_ws/src/soft_robot_msgs/
  package.xml
  CMakeLists.txt
  msg/
    CartesianState.msg
    CartesianCorrection.msg
    CartesianCorrectionStamped.msg
    RsiState.msg
    ModeCommand.msg
    ModeState.msg
  srv/
    GetTool.srv
  action/
    MoveToOrientation.action
  test/
    test_msgs.cpp

ros_ws/src/kuka_rsi_hw_interface/
  package.xml
  CMakeLists.txt
  include/kuka_rsi_hw_interface/
    rsi_frame.h                    # RobFrame/SenFrame parse + serialize (tinyxml2/snprintf)
    rsi_session_monitor.h          # IPOC continuity, timeout counting, latched fault
    command_limiter.h              # secondary per-axis RKorr clamp
    udp_transport.h                # non-blocking UDP endpoint (poll)
    cartesian_command_interface.h  # CartesianState/CorrectionCommand hardware interfaces
    kuka_rsi_robot_hw.h            # hardware_interface::RobotHW implementation
    rsi_mock_core.h                # deterministic KRC-side behavior model
    rsi_mock_server.h              # UDP wrapper playing the KRC role
  src/
    rsi_frame.cpp
    rsi_session_monitor.cpp
    command_limiter.cpp
    udp_transport.cpp
    kuka_rsi_robot_hw.cpp
    rsi_mock_core.cpp
    rsi_mock_server.cpp
    kuka_rsi_sim_server_main.cpp   # standalone mock executable (spec 15.2)
    kuka_rsi_hw_interface_node.cpp # ROS node shell + controller_manager loop
  config/
    kuka_rsi.yaml
  launch/
    kuka_rsi.launch
  test/
    test_rsi_frame_parse.cpp
    test_rsi_frame_serialize.cpp
    test_rsi_session_monitor.cpp
    test_command_limiter.cpp
    test_udp_transport.cpp
    test_cartesian_interfaces.cpp
    test_kuka_rsi_robot_hw.cpp
    test_rsi_mock.cpp
    test_rsi_integration.cpp
```

参考帧格式(旧系统 `ref/SoftRobot 2.0/SoftRobot/ExternalData.xml` 实测,`Sen Type="ImFree"`,含 RKorr/AKorr/EKorr/Tech/DiO/IPOC):本计划按规格 §6.1 只保留 `RKorr/Stop/Watchdog/IPOC`,`Sen Type` 改为 `"ROS"`。

---

### Task 0: 建立分支

- [ ] **Step 1: 从 Plan 1 分支创建工作分支**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git checkout feature/soft-force-control-core && \
git checkout -b feature/rsi-hw-interface-msgs
```

预期:`Switched to a new branch 'feature/rsi-hw-interface-msgs'`。

---

### Task 1: `soft_robot_msgs` 包(全部 msg/srv/action + 常量对齐测试)

**Files:**
- Create: `ros_ws/src/soft_robot_msgs/package.xml`
- Create: `ros_ws/src/soft_robot_msgs/CMakeLists.txt`
- Create: `ros_ws/src/soft_robot_msgs/msg/CartesianState.msg`
- Create: `ros_ws/src/soft_robot_msgs/msg/CartesianCorrection.msg`
- Create: `ros_ws/src/soft_robot_msgs/msg/CartesianCorrectionStamped.msg`
- Create: `ros_ws/src/soft_robot_msgs/msg/RsiState.msg`
- Create: `ros_ws/src/soft_robot_msgs/msg/ModeCommand.msg`
- Create: `ros_ws/src/soft_robot_msgs/msg/ModeState.msg`
- Create: `ros_ws/src/soft_robot_msgs/srv/GetTool.srv`
- Create: `ros_ws/src/soft_robot_msgs/action/MoveToOrientation.action`
- Test: `ros_ws/src/soft_robot_msgs/test/test_msgs.cpp`

**Interfaces:**
- Consumes: `std_msgs/Header`、`actionlib_msgs`(genaction)。
- Produces(后续所有 Plan 消费):
  - `soft_robot_msgs/CartesianState`:Header + x/y/z/a/b/c(mm/deg),语义同 `sfc::CartesianState`。
  - `soft_robot_msgs/CartesianCorrection`:x/y/z/a/b/c(mm/deg **per cycle**),语义同 `sfc::CartesianCorrection`。
  - `soft_robot_msgs/CartesianCorrectionStamped`:Header + correction(Plan 3 stream 模式命令话题)。
  - `soft_robot_msgs/RsiState`:RSI 链路诊断快照(本计划节点壳即发布)。
  - `soft_robot_msgs/ModeCommand` / `ModeState`:mode/profile/system_state,常量数值与 `sfc::ControlMode`、`sfc::Profile` 枚举序号及规格 §11 状态机一致。
  - `soft_robot_msgs/GetTool`(srv):EKI `$TOOL` 查询(Plan 4)。
  - `soft_robot_msgs/MoveToOrientation`(action):goal 模式姿态运动(Plan 3/5)。

- [ ] **Step 1: 写包清单与构建文件**

`ros_ws/src/soft_robot_msgs/package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>soft_robot_msgs</name>
  <version>0.1.0</version>
  <description>
    Shared message, service, and action definitions for the KUKA RSI soft
    force control system. Units follow the project convention: mm, deg
    (KUKA A/B/C = Z-Y-X Euler), N, Nm. Wrench data deliberately reuses
    geometry_msgs/WrenchStamped and is not redefined here.
  </description>
  <maintainer email="dev@example.com">ljj</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
</package>
```

`ros_ws/src/soft_robot_msgs/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(soft_robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  CartesianState.msg
  CartesianCorrection.msg
  CartesianCorrectionStamped.msg
  RsiState.msg
  ModeCommand.msg
  ModeState.msg
)

add_service_files(
  FILES
  GetTool.srv
)

add_action_files(
  FILES
  MoveToOrientation.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs
)

if(CATKIN_ENABLE_TESTING)
  add_compile_options(-std=c++14 -Wall -Wextra)
  include_directories(${CATKIN_DEVEL_PREFIX}/include ${catkin_INCLUDE_DIRS})
  catkin_add_gtest(test_msgs test/test_msgs.cpp)
  add_dependencies(test_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS})
  target_link_libraries(test_msgs ${GTEST_MAIN_LIBRARIES})
endif()
```

- [ ] **Step 2: 写失败测试**

`ros_ws/src/soft_robot_msgs/test/test_msgs.cpp`:

```cpp
#include <gtest/gtest.h>

#include <soft_robot_msgs/CartesianCorrectionStamped.h>
#include <soft_robot_msgs/CartesianState.h>
#include <soft_robot_msgs/GetTool.h>
#include <soft_robot_msgs/ModeCommand.h>
#include <soft_robot_msgs/ModeState.h>
#include <soft_robot_msgs/MoveToOrientationAction.h>
#include <soft_robot_msgs/RsiState.h>

// Numeric values must match the sfc enums from soft_force_control_core:
// ControlMode { IDLE, DIRECT_CARTESIAN, FORCE_COMPLIANCE, CALIBRATION }
// Profile { DRAG, PRECISION }.
TEST(Msgs, ModeConstantsMatchCoreEnums) {
  EXPECT_EQ(soft_robot_msgs::ModeCommand::MODE_IDLE, 0u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::MODE_DIRECT_CARTESIAN, 1u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::MODE_FORCE_COMPLIANCE, 2u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::MODE_CALIBRATION, 3u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::PROFILE_DRAG, 0u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::PROFILE_PRECISION, 1u);
}

// System states follow spec section 11 ordering.
TEST(Msgs, SystemStateConstantsCoverStateMachine) {
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_OFFLINE, 0u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_CONNECTED, 1u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_READY, 2u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_SERVOING, 3u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_CALIBRATING, 4u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_DEGRADED, 5u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_FAULT, 6u);
}

TEST(Msgs, MoveToOrientationResultCodes) {
  EXPECT_EQ(soft_robot_msgs::MoveToOrientationResult::CONVERGED, 0u);
  EXPECT_EQ(soft_robot_msgs::MoveToOrientationResult::TIMEOUT, 1u);
  EXPECT_EQ(soft_robot_msgs::MoveToOrientationResult::ABORTED, 2u);
}

TEST(Msgs, DefaultConstructedFieldsAreZero) {
  soft_robot_msgs::CartesianState s;
  EXPECT_EQ(s.x, 0.0);
  EXPECT_EQ(s.c, 0.0);
  soft_robot_msgs::CartesianCorrectionStamped cs;
  EXPECT_EQ(cs.correction.a, 0.0);
  soft_robot_msgs::RsiState r;
  EXPECT_FALSE(r.connected);
  EXPECT_FALSE(r.fault);
  EXPECT_EQ(r.ipoc, 0u);
  soft_robot_msgs::GetTool::Response resp;
  EXPECT_FALSE(resp.success);
  soft_robot_msgs::MoveToOrientationGoal g;
  EXPECT_FALSE(g.use_position);
  EXPECT_EQ(g.speed_scale, 0.0);
}
```

- [ ] **Step 3: 运行确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,CMake 报 `Message file .../msg/CartesianState.msg does not exist`(msg 文件尚未落盘)。

- [ ] **Step 4: 写全部消息定义**

`ros_ws/src/soft_robot_msgs/msg/CartesianState.msg`:

```text
# KUKA Cartesian pose in the configured base frame.
# Units: mm for x/y/z, deg for a/b/c (KUKA A/B/C = Z-Y-X Euler).
Header header
float64 x
float64 y
float64 z
float64 a
float64 b
float64 c
```

`ros_ws/src/soft_robot_msgs/msg/CartesianCorrection.msg`:

```text
# Per-cycle RSI Cartesian correction (RKorr).
# Units: mm and deg PER 4 ms CYCLE, not velocities.
float64 x
float64 y
float64 z
float64 a
float64 b
float64 c
```

`ros_ws/src/soft_robot_msgs/msg/CartesianCorrectionStamped.msg`:

```text
# Stamped per-cycle correction, used as the stream-mode command topic of
# CartesianCorrectionController (spec section 7.7). The controller zeroes
# its output when the stamp is older than the configured command timeout.
Header header
CartesianCorrection correction
```

`ros_ws/src/soft_robot_msgs/msg/RsiState.msg`:

```text
# Realtime RSI link diagnostics published by kuka_rsi_hw_interface
# (spec section 12.2). Counters are cumulative since node start.
Header header
bool connected                # at least one valid KRC frame received
bool fault                    # latched communication fault (zero output + Stop)
uint64 ipoc                   # last IPOC received from the KRC
uint64 total_timeouts         # read cycles that ended without a frame
uint32 consecutive_timeouts   # current run of missed cycles
uint64 bad_frames             # frames that failed XML validation
uint64 ipoc_jumps             # non-increasing IPOC events
uint64 saturation_count       # write cycles clamped by the hw command limiter
```

`ros_ws/src/soft_robot_msgs/msg/ModeCommand.msg`:

```text
# Requested control mode and parameter profile (spec section 10).
# Constant values match the sfc::ControlMode / sfc::Profile enum ordering.
uint8 MODE_IDLE=0
uint8 MODE_DIRECT_CARTESIAN=1
uint8 MODE_FORCE_COMPLIANCE=2
uint8 MODE_CALIBRATION=3
uint8 PROFILE_DRAG=0
uint8 PROFILE_PRECISION=1
uint8 mode
uint8 profile
```

`ros_ws/src/soft_robot_msgs/msg/ModeState.msg`:

```text
# Current control mode, profile, and system state (spec sections 10-11).
uint8 SYSTEM_OFFLINE=0
uint8 SYSTEM_CONNECTED=1
uint8 SYSTEM_READY=2
uint8 SYSTEM_SERVOING=3
uint8 SYSTEM_CALIBRATING=4
uint8 SYSTEM_DEGRADED=5
uint8 SYSTEM_FAULT=6
Header header
uint8 mode
uint8 profile
uint8 system_state
```

`ros_ws/src/soft_robot_msgs/srv/GetTool.srv`:

```text
# Query the currently active $TOOL frame from the KRC over EKI
# (spec section 6.4). Units: mm / deg.
---
bool success
string message
float64 x
float64 y
float64 z
float64 a
float64 b
float64 c
```

`ros_ws/src/soft_robot_msgs/action/MoveToOrientation.action`:

```text
# Goal-mode orientation motion (spec section 7.6). Executed by
# CartesianCorrectionController hosting OrientationMotionCore.
# Units: deg for a/b/c, mm for x/y/z. speed_scale in (0, 1].
float64 a
float64 b
float64 c
bool use_position
float64 x
float64 y
float64 z
float64 speed_scale
---
uint8 CONVERGED=0
uint8 TIMEOUT=1
uint8 ABORTED=2
uint8 result_code
string message
---
float64 error_deg
float64 error_mm
float64 progress
```

- [ ] **Step 5: 运行确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_msgs/test_msgs
```

预期:`[  PASSED  ] 4 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_msgs && \
git commit -m "feat(msgs): add soft_robot_msgs shared message package (Task 1)"
```

---

### Task 2: `kuka_rsi_hw_interface` 包骨架 + `RobFrame` 解析(KRC → PC)

**Files:**
- Create: `ros_ws/src/kuka_rsi_hw_interface/package.xml`
- Create: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`
- Create: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/rsi_frame.h`
- Create: `ros_ws/src/kuka_rsi_hw_interface/src/rsi_frame.cpp`
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_frame_parse.cpp`

**Interfaces:**
- Consumes: tinyxml2(系统 6.2.0)。
- Produces(命名空间 `kuka_rsi`,后续所有 Task 消费):
  - `kuka_rsi::RobFrame { double x,y,z,a,b,c; double axis_deg[6]; double delay; int mode; uint64_t ipoc; bool valid; }`
  - `bool parseRobFrame(const char* data, std::size_t len, RobFrame& out)` — 解析规格 §6.1 的 `<Rob Type="KUKA">` 帧;缺 `RIst`/`AIPos`/`IPOC` 或 XML 畸形 → 返回 false;`Delay`/`Mode` 可选(缺省 0)。

- [ ] **Step 1: 写包清单与构建文件**

`ros_ws/src/kuka_rsi_hw_interface/package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>kuka_rsi_hw_interface</name>
  <version>0.1.0</version>
  <description>
    ros_control hardware interface for the KUKA RSI realtime channel
    (spec sections 5.1, 6.1, 12.2). Owns the RSI UDP/XML communication,
    exposes Cartesian state and RKorr correction command interfaces, and
    ships a KRC-side RSI mock (kuka_rsi_sim_server) for offline testing.
  </description>
  <maintainer email="dev@example.com">ljj</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>hardware_interface</depend>
  <depend>controller_manager</depend>
  <depend>realtime_tools</depend>
  <depend>soft_robot_msgs</depend>
  <build_depend>libtinyxml2-dev</build_depend>
  <exec_depend>libtinyxml2</exec_depend>
</package>
```

`ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(kuka_rsi_hw_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  realtime_tools
  soft_robot_msgs
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kuka_rsi_protocol
  CATKIN_DEPENDS roscpp hardware_interface controller_manager realtime_tools
                 soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS} ${TINYXML2_INCLUDE_DIRS})

# Protocol / session library: no roscpp runtime dependency, fully unit-testable.
add_library(kuka_rsi_protocol
  src/rsi_frame.cpp
)
target_link_libraries(kuka_rsi_protocol ${TINYXML2_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_rsi_frame_parse test/test_rsi_frame_parse.cpp)
  target_link_libraries(test_rsi_frame_parse kuka_rsi_protocol
                        ${GTEST_MAIN_LIBRARIES})
endif()
```

- [ ] **Step 2: 写失败测试**

`ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_frame_parse.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cstring>

#include "kuka_rsi_hw_interface/rsi_frame.h"

using kuka_rsi::RobFrame;
using kuka_rsi::parseRobFrame;

namespace {
// Frame layout per spec section 6.1 (KUKA -> ROS).
const char kGoodFrame[] =
    "<Rob Type=\"KUKA\">"
    "<RIst X=\"12.5\" Y=\"-3.25\" Z=\"800.0\" A=\"90.0\" B=\"-45.5\" "
    "C=\"179.9\"/>"
    "<AIPos A1=\"1.1\" A2=\"-90.2\" A3=\"88.3\" A4=\"0.4\" A5=\"45.5\" "
    "A6=\"-0.6\"/>"
    "<Delay D=\"2\"/>"
    "<Mode M=\"1\"/>"
    "<IPOC>4894312</IPOC>"
    "</Rob>";

bool parse(const char* s, RobFrame& out) {
  return parseRobFrame(s, std::strlen(s), out);
}
}  // namespace

TEST(RobFrameParse, ParsesCartesianPose) {
  RobFrame f;
  ASSERT_TRUE(parse(kGoodFrame, f));
  EXPECT_DOUBLE_EQ(f.x, 12.5);
  EXPECT_DOUBLE_EQ(f.y, -3.25);
  EXPECT_DOUBLE_EQ(f.z, 800.0);
  EXPECT_DOUBLE_EQ(f.a, 90.0);
  EXPECT_DOUBLE_EQ(f.b, -45.5);
  EXPECT_DOUBLE_EQ(f.c, 179.9);
}

TEST(RobFrameParse, ParsesJointAnglesDelayModeIpoc) {
  RobFrame f;
  ASSERT_TRUE(parse(kGoodFrame, f));
  EXPECT_DOUBLE_EQ(f.axis_deg[0], 1.1);
  EXPECT_DOUBLE_EQ(f.axis_deg[1], -90.2);
  EXPECT_DOUBLE_EQ(f.axis_deg[5], -0.6);
  EXPECT_DOUBLE_EQ(f.delay, 2.0);
  EXPECT_EQ(f.mode, 1);
  EXPECT_EQ(f.ipoc, 4894312u);
}

TEST(RobFrameParse, DelayAndModeAreOptional) {
  const char frame[] =
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>";
  RobFrame f;
  ASSERT_TRUE(parse(frame, f));
  EXPECT_DOUBLE_EQ(f.delay, 0.0);
  EXPECT_EQ(f.mode, 0);
  EXPECT_EQ(f.ipoc, 1u);
}

TEST(RobFrameParse, RejectsMalformedXml) {
  RobFrame f;
  EXPECT_FALSE(parse("<Rob Type=\"KUKA\"><RIst X=\"1\"", f));
  EXPECT_FALSE(parse("not xml at all", f));
  EXPECT_FALSE(parseRobFrame(nullptr, 0, f));
}

TEST(RobFrameParse, RejectsMissingMandatoryElements) {
  RobFrame f;
  // Missing IPOC.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\"/>"
      "</Rob>",
      f));
  // Missing RIst.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>",
      f));
  // Missing AIPos.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>",
      f));
  // Wrong root element.
  EXPECT_FALSE(parse("<Sen Type=\"ROS\"><IPOC>1</IPOC></Sen>", f));
}

TEST(RobFrameParse, RejectsIncompleteAttributeSets) {
  RobFrame f;
  // RIst missing C attribute.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\"/>"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>",
      f));
  // AIPos missing A6 attribute.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>",
      f));
}

TEST(RobFrameParse, ValidFlagTracksResult) {
  RobFrame f;
  EXPECT_FALSE(f.valid);  // default-constructed
  ASSERT_TRUE(parse(kGoodFrame, f));
  EXPECT_TRUE(f.valid);
  EXPECT_FALSE(parse("garbage", f));
  EXPECT_FALSE(f.valid);  // reset on failed parse
}
```

- [ ] **Step 3: 运行确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,`rsi_frame.h: No such file or directory`。

- [ ] **Step 4: 写最小实现**

`ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/rsi_frame.h`:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>

namespace kuka_rsi {

// One KRC -> PC state frame (spec section 6.1, <Rob Type="KUKA">).
// Units: mm/deg for the Cartesian pose, deg for axis angles.
struct RobFrame {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};  // RIst
  double axis_deg[6] = {0, 0, 0, 0, 0, 0};    // AIPos A1..A6
  double delay{0};                            // Delay D (optional)
  int mode{0};                                // Mode M (optional)
  std::uint64_t ipoc{0};                      // IPOC (mandatory, echoed back)
  bool valid{false};
};

// One PC -> KRC correction frame (spec section 6.1, <Sen Type="ROS">).
// Correction units: mm and deg per RSI cycle.
struct SenFrame {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};  // RKorr
  int stop{0};                                // Stop S: 1 = PC requests stop
  std::uint64_t watchdog{0};                  // Watchdog W: PC liveness counter
  std::uint64_t ipoc{0};                      // echo of the received IPOC
};

// Parses a <Rob> frame. Returns false (and out.valid = false) on malformed
// XML, wrong root element, or missing RIst/AIPos/IPOC or any of their
// mandatory attributes. Note: tinyxml2 allocates internally; this is the
// declared realtime deviation of the RSI read path.
bool parseRobFrame(const char* data, std::size_t len, RobFrame& out);

// Serializes a <Sen> frame into buf (NUL-terminated). Returns the payload
// length (excluding NUL), or 0 if the buffer is too small. Allocation-free.
std::size_t serializeSenFrame(const SenFrame& frame, char* buf,
                              std::size_t buf_size);

}  // namespace kuka_rsi
```

(`SenFrame`/`serializeSenFrame` 声明在此一并落盘,实现属 Task 3——本 Task 只实现 `parseRobFrame`,头文件不再改动。)

`ros_ws/src/kuka_rsi_hw_interface/src/rsi_frame.cpp`:

```cpp
#include "kuka_rsi_hw_interface/rsi_frame.h"

#include <tinyxml2.h>

#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace kuka_rsi {

namespace {
// Reads all listed attributes as doubles; false if any is missing/invalid.
bool readAttrs(const tinyxml2::XMLElement* e, const char* const* names,
               double* out, int count) {
  for (int i = 0; i < count; ++i) {
    if (e->QueryDoubleAttribute(names[i], &out[i]) != tinyxml2::XML_SUCCESS) {
      return false;
    }
  }
  return true;
}
}  // namespace

bool parseRobFrame(const char* data, std::size_t len, RobFrame& out) {
  out.valid = false;
  if (data == nullptr || len == 0) return false;

  tinyxml2::XMLDocument doc;
  if (doc.Parse(data, len) != tinyxml2::XML_SUCCESS) return false;

  const tinyxml2::XMLElement* rob = doc.FirstChildElement("Rob");
  if (rob == nullptr) return false;

  const tinyxml2::XMLElement* rist = rob->FirstChildElement("RIst");
  const tinyxml2::XMLElement* aipos = rob->FirstChildElement("AIPos");
  const tinyxml2::XMLElement* ipoc = rob->FirstChildElement("IPOC");
  if (rist == nullptr || aipos == nullptr || ipoc == nullptr) return false;

  static const char* const kPose[] = {"X", "Y", "Z", "A", "B", "C"};
  double pose[6];
  if (!readAttrs(rist, kPose, pose, 6)) return false;

  static const char* const kAxes[] = {"A1", "A2", "A3", "A4", "A5", "A6"};
  double axes[6];
  if (!readAttrs(aipos, kAxes, axes, 6)) return false;

  const char* ipoc_text = ipoc->GetText();
  if (ipoc_text == nullptr) return false;
  std::uint64_t ipoc_value = 0;
  if (std::sscanf(ipoc_text, "%" SCNu64, &ipoc_value) != 1) return false;

  out.x = pose[0];
  out.y = pose[1];
  out.z = pose[2];
  out.a = pose[3];
  out.b = pose[4];
  out.c = pose[5];
  for (int i = 0; i < 6; ++i) out.axis_deg[i] = axes[i];
  out.delay = 0;
  out.mode = 0;
  const tinyxml2::XMLElement* delay = rob->FirstChildElement("Delay");
  if (delay != nullptr) delay->QueryDoubleAttribute("D", &out.delay);
  const tinyxml2::XMLElement* mode = rob->FirstChildElement("Mode");
  if (mode != nullptr) mode->QueryIntAttribute("M", &out.mode);
  out.ipoc = ipoc_value;
  out.valid = true;
  return true;
}

}  // namespace kuka_rsi
```

(头文件一次定稿:`SenFrame`/`serializeSenFrame` 已声明,但 **本 Task 不实现序列化**——Task 3 的失败测试将以链接错误 `undefined reference to serializeSenFrame` 呈现失败态,随后在同一 `rsi_frame.cpp` 内补实现。)

- [ ] **Step 5: 运行确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_rsi_hw_interface/test_rsi_frame_parse
```

预期:`[  PASSED  ] 7 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "feat(rsi): add package skeleton and RobFrame XML parsing (Task 2)"
```

---

### Task 3: `SenFrame` 序列化(PC → KRC,IPOC 回显)

**Files:**
- Modify: `ros_ws/src/kuka_rsi_hw_interface/src/rsi_frame.cpp`(追加 `serializeSenFrame` 实现)
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`(加测试目标)
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_frame_serialize.cpp`

**Interfaces:**
- Consumes: `kuka_rsi::SenFrame`(Task 2 已声明)。
- Produces: `serializeSenFrame` 实现——分配自由(`snprintf` 入预分配缓冲),往返一致(序列化结果可被 tinyxml2 重新解析且字段值精确到 1e-4)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_frame_serialize.cpp`:

```cpp
#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <cstring>
#include <string>

#include "kuka_rsi_hw_interface/rsi_frame.h"

using kuka_rsi::SenFrame;
using kuka_rsi::serializeSenFrame;

namespace {
SenFrame sample() {
  SenFrame f;
  f.x = 0.1234;
  f.y = -0.5;
  f.z = 0.0;
  f.a = 0.01;
  f.b = -0.02;
  f.c = 0.03;
  f.stop = 0;
  f.watchdog = 42;
  f.ipoc = 4894312;
  return f;
}
}  // namespace

TEST(SenFrameSerialize, ProducesParsableXmlWithAllFields) {
  char buf[1024];
  const std::size_t n = serializeSenFrame(sample(), buf, sizeof(buf));
  ASSERT_GT(n, 0u);
  EXPECT_EQ(n, std::strlen(buf));  // NUL-terminated, length consistent

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(buf, n), tinyxml2::XML_SUCCESS);
  const tinyxml2::XMLElement* sen = doc.FirstChildElement("Sen");
  ASSERT_NE(sen, nullptr);
  EXPECT_STREQ(sen->Attribute("Type"), "ROS");

  const tinyxml2::XMLElement* rkorr = sen->FirstChildElement("RKorr");
  ASSERT_NE(rkorr, nullptr);
  EXPECT_NEAR(rkorr->DoubleAttribute("X"), 0.1234, 1e-9);
  EXPECT_NEAR(rkorr->DoubleAttribute("Y"), -0.5, 1e-9);
  EXPECT_NEAR(rkorr->DoubleAttribute("C"), 0.03, 1e-9);

  const tinyxml2::XMLElement* stop = sen->FirstChildElement("Stop");
  ASSERT_NE(stop, nullptr);
  EXPECT_EQ(stop->IntAttribute("S"), 0);

  const tinyxml2::XMLElement* wd = sen->FirstChildElement("Watchdog");
  ASSERT_NE(wd, nullptr);
  // Int64Attribute: tinyxml2 6.2 has no Unsigned64Attribute (added in 7.0).
  EXPECT_EQ(wd->Int64Attribute("W"), 42);

  const tinyxml2::XMLElement* ipoc = sen->FirstChildElement("IPOC");
  ASSERT_NE(ipoc, nullptr);
  EXPECT_STREQ(ipoc->GetText(), "4894312");
}

TEST(SenFrameSerialize, EchoesIpocVerbatim) {
  SenFrame f = sample();
  f.ipoc = 18446744073709551615ull;  // uint64 max survives round trip
  char buf[1024];
  ASSERT_GT(serializeSenFrame(f, buf, sizeof(buf)), 0u);
  EXPECT_NE(std::strstr(buf, "<IPOC>18446744073709551615</IPOC>"), nullptr);
}

TEST(SenFrameSerialize, StopFlagSerialized) {
  SenFrame f = sample();
  f.stop = 1;
  char buf[1024];
  ASSERT_GT(serializeSenFrame(f, buf, sizeof(buf)), 0u);
  EXPECT_NE(std::strstr(buf, "<Stop S=\"1\"/>"), nullptr);
}

TEST(SenFrameSerialize, FourDecimalFixedFormat) {
  // Legacy ExternalData.xml uses fixed 4-decimal values; keep that format.
  SenFrame f;  // all zeros
  char buf[1024];
  ASSERT_GT(serializeSenFrame(f, buf, sizeof(buf)), 0u);
  EXPECT_NE(std::strstr(buf, "X=\"0.0000\""), nullptr);
  EXPECT_NE(std::strstr(buf, "C=\"0.0000\""), nullptr);
}

TEST(SenFrameSerialize, ReturnsZeroWhenBufferTooSmall) {
  char tiny[16];
  EXPECT_EQ(serializeSenFrame(sample(), tiny, sizeof(tiny)), 0u);
  char none[1];
  EXPECT_EQ(serializeSenFrame(sample(), none, sizeof(none)), 0u);
}
```

- [ ] **Step 2: 加测试目标并确认失败**

`CMakeLists.txt` 测试段追加:

```cmake
  catkin_add_gtest(test_rsi_frame_serialize test/test_rsi_frame_serialize.cpp)
  target_link_libraries(test_rsi_frame_serialize kuka_rsi_protocol
                        ${TINYXML2_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
```

运行:`cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
预期:LINK FAILS,`undefined reference to ... serializeSenFrame`。

- [ ] **Step 3: 写最小实现**

`src/rsi_frame.cpp` 在 `parseRobFrame` 之后、`}  // namespace kuka_rsi` 之前追加:

```cpp
std::size_t serializeSenFrame(const SenFrame& frame, char* buf,
                              std::size_t buf_size) {
  const int n = std::snprintf(
      buf, buf_size,
      "<Sen Type=\"ROS\">"
      "<RKorr X=\"%.4f\" Y=\"%.4f\" Z=\"%.4f\" A=\"%.4f\" B=\"%.4f\" "
      "C=\"%.4f\"/>"
      "<Stop S=\"%d\"/>"
      "<Watchdog W=\"%" PRIu64 "\"/>"
      "<IPOC>%" PRIu64 "</IPOC>"
      "</Sen>",
      frame.x, frame.y, frame.z, frame.a, frame.b, frame.c, frame.stop,
      frame.watchdog, frame.ipoc);
  if (n <= 0 || static_cast<std::size_t>(n) >= buf_size) return 0;
  return static_cast<std::size_t>(n);
}
```

- [ ] **Step 4: 运行确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_rsi_hw_interface/test_rsi_frame_serialize && \
  ./devel/lib/kuka_rsi_hw_interface/test_rsi_frame_parse
```

预期:`[  PASSED  ] 5 tests.` 与 `[  PASSED  ] 7 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "feat(rsi): add SenFrame serialization with IPOC echo (Task 3)"
```

---

### Task 4: `RsiSessionMonitor`(IPOC 连续性 + 超时计数 + 锁存故障)与 `CommandLimiter`

**Files:**
- Create: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/rsi_session_monitor.h`
- Create: `ros_ws/src/kuka_rsi_hw_interface/src/rsi_session_monitor.cpp`
- Create: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/command_limiter.h`
- Create: `ros_ws/src/kuka_rsi_hw_interface/src/command_limiter.cpp`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_session_monitor.cpp`
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_command_limiter.cpp`

**Interfaces:**
- Consumes: `kuka_rsi::RobFrame`(Task 2)。
- Produces(均分配自由、无阻塞,供 Task 7 `KukaRsiRobotHW` 消费):
  - `kuka_rsi::SessionConfig { unsigned max_consecutive_timeouts{5}; }`
  - `kuka_rsi::SessionStats { bool connected; bool fault; uint64_t last_ipoc; uint64_t total_timeouts; unsigned consecutive_timeouts; uint64_t bad_frames; uint64_t ipoc_jumps; }`
  - `kuka_rsi::RsiSessionMonitor`:`explicit RsiSessionMonitor(const SessionConfig&)`、`void onFrame(const RobFrame&)`、`void onTimeout()`、`void onBadFrame()`、`bool faulted() const`、`bool connected() const`、`const SessionStats& stats() const`、`void reset()`。
  - `kuka_rsi::CommandLimits { double max_trans{0.5}; double max_rot{0.05}; }`(每周期 mm/deg,规格 §12.2 二级限幅,默认与核心库 `SafetyParams` 一致)
  - `kuka_rsi::CommandLimiter`:`bool clamp(double corr[6], const CommandLimits&) const`(原地限幅,索引 0-2 平移 / 3-5 旋转;返回 true 表示发生了截断)。
- 语义(决策 5/6):超时/坏帧计数仅在 `connected()` 后累计连击并触发锁存;IPOC 非严格递增 → `ipoc_jumps++`(不故障);任意有效帧清零 `consecutive_timeouts`;故障锁存后 `onFrame` 不解除,唯 `reset()` 解除。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_session_monitor.cpp`:

```cpp
#include <gtest/gtest.h>

#include "kuka_rsi_hw_interface/rsi_session_monitor.h"

using kuka_rsi::RobFrame;
using kuka_rsi::RsiSessionMonitor;
using kuka_rsi::SessionConfig;

namespace {
RobFrame frameWithIpoc(std::uint64_t ipoc) {
  RobFrame f;
  f.ipoc = ipoc;
  f.valid = true;
  return f;
}
}  // namespace

TEST(SessionMonitor, StartsDisconnectedNoFault) {
  RsiSessionMonitor m{SessionConfig{}};
  EXPECT_FALSE(m.connected());
  EXPECT_FALSE(m.faulted());
  EXPECT_EQ(m.stats().last_ipoc, 0u);
}

TEST(SessionMonitor, FirstFrameConnects) {
  RsiSessionMonitor m{SessionConfig{}};
  m.onFrame(frameWithIpoc(100));
  EXPECT_TRUE(m.connected());
  EXPECT_EQ(m.stats().last_ipoc, 100u);
  EXPECT_EQ(m.stats().ipoc_jumps, 0u);
}

TEST(SessionMonitor, TimeoutsBeforeConnectionNeverFault) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 3;
  RsiSessionMonitor m{cfg};
  for (int i = 0; i < 100; ++i) m.onTimeout();
  EXPECT_FALSE(m.faulted());
  EXPECT_FALSE(m.connected());
}

TEST(SessionMonitor, ConsecutiveTimeoutsLatchFault) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 3;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onTimeout();
  m.onTimeout();
  EXPECT_FALSE(m.faulted());  // 2 < 3
  m.onTimeout();
  EXPECT_TRUE(m.faulted());  // 3rd consecutive miss
  EXPECT_EQ(m.stats().total_timeouts, 3u);
}

TEST(SessionMonitor, FrameResetsConsecutiveCount) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 3;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onTimeout();
  m.onTimeout();
  m.onFrame(frameWithIpoc(2));
  EXPECT_EQ(m.stats().consecutive_timeouts, 0u);
  m.onTimeout();
  m.onTimeout();
  EXPECT_FALSE(m.faulted());
  EXPECT_EQ(m.stats().total_timeouts, 4u);
}

TEST(SessionMonitor, BadFramesCountTowardFault) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 2;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onBadFrame();
  m.onBadFrame();
  EXPECT_TRUE(m.faulted());
  EXPECT_EQ(m.stats().bad_frames, 2u);
}

TEST(SessionMonitor, NonIncreasingIpocCountsJumpButNoFault) {
  RsiSessionMonitor m{SessionConfig{}};
  m.onFrame(frameWithIpoc(100));
  m.onFrame(frameWithIpoc(100));  // repeat
  m.onFrame(frameWithIpoc(50));   // backwards
  EXPECT_EQ(m.stats().ipoc_jumps, 2u);
  EXPECT_FALSE(m.faulted());
  EXPECT_EQ(m.stats().last_ipoc, 50u);  // tracks latest regardless
}

TEST(SessionMonitor, FaultIsLatchedUntilReset) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 1;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onTimeout();
  ASSERT_TRUE(m.faulted());
  m.onFrame(frameWithIpoc(2));  // frames do NOT clear a latched fault
  EXPECT_TRUE(m.faulted());
  m.reset();
  EXPECT_FALSE(m.faulted());
  EXPECT_FALSE(m.connected());  // reset returns to initial state
  EXPECT_EQ(m.stats().total_timeouts, 0u);
}
```

`ros_ws/src/kuka_rsi_hw_interface/test/test_command_limiter.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "kuka_rsi_hw_interface/command_limiter.h"

using kuka_rsi::CommandLimiter;
using kuka_rsi::CommandLimits;

TEST(CommandLimiter, PassesSmallCommandUnchanged) {
  CommandLimiter lim;
  double c[6] = {0.1, -0.2, 0.3, 0.01, -0.02, 0.03};
  EXPECT_FALSE(lim.clamp(c, CommandLimits{}));
  EXPECT_DOUBLE_EQ(c[0], 0.1);
  EXPECT_DOUBLE_EQ(c[5], 0.03);
}

TEST(CommandLimiter, ClampsTranslationAxes) {
  CommandLimiter lim;
  double c[6] = {2.0, -2.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_TRUE(lim.clamp(c, CommandLimits{}));
  EXPECT_DOUBLE_EQ(c[0], 0.5);
  EXPECT_DOUBLE_EQ(c[1], -0.5);
}

TEST(CommandLimiter, ClampsRotationAxes) {
  CommandLimiter lim;
  double c[6] = {0.0, 0.0, 0.0, 1.0, -1.0, 0.04};
  EXPECT_TRUE(lim.clamp(c, CommandLimits{}));
  EXPECT_DOUBLE_EQ(c[3], 0.05);
  EXPECT_DOUBLE_EQ(c[4], -0.05);
  EXPECT_DOUBLE_EQ(c[5], 0.04);  // within limit, untouched
}

TEST(CommandLimiter, NonFiniteBecomesZeroAndFlags) {
  CommandLimiter lim;
  double c[6] = {std::nan(""), 0.1, 0.0, 0.0, 0.0, 0.0};
  EXPECT_TRUE(lim.clamp(c, CommandLimits{}));
  EXPECT_DOUBLE_EQ(c[0], 0.0);
  EXPECT_DOUBLE_EQ(c[1], 0.1);
  double inf[6] = {0.0, 0.0, 0.0,
                   std::numeric_limits<double>::infinity(), 0.0, 0.0};
  EXPECT_TRUE(lim.clamp(inf, CommandLimits{}));
  EXPECT_DOUBLE_EQ(inf[3], 0.0);
}

TEST(CommandLimiter, CustomLimitsRespected) {
  CommandLimiter lim;
  CommandLimits l;
  l.max_trans = 0.1;
  l.max_rot = 0.01;
  double c[6] = {0.2, 0.0, 0.0, 0.02, 0.0, 0.0};
  EXPECT_TRUE(lim.clamp(c, l));
  EXPECT_DOUBLE_EQ(c[0], 0.1);
  EXPECT_DOUBLE_EQ(c[3], 0.01);
}
```

- [ ] **Step 2: 加构建条目并确认失败**

`CMakeLists.txt`:`add_library(kuka_rsi_protocol ...)` 源列表追加 `src/rsi_session_monitor.cpp`、`src/command_limiter.cpp`;测试段追加:

```cmake
  catkin_add_gtest(test_rsi_session_monitor test/test_rsi_session_monitor.cpp)
  target_link_libraries(test_rsi_session_monitor kuka_rsi_protocol
                        ${GTEST_MAIN_LIBRARIES})
  catkin_add_gtest(test_command_limiter test/test_command_limiter.cpp)
  target_link_libraries(test_command_limiter kuka_rsi_protocol
                        ${GTEST_MAIN_LIBRARIES})
```

运行:`cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
预期:BUILD FAILS,`rsi_session_monitor.h: No such file or directory`。

- [ ] **Step 3: 写最小实现**

`include/kuka_rsi_hw_interface/rsi_session_monitor.h`:

```cpp
#pragma once

#include <cstdint>

#include "kuka_rsi_hw_interface/rsi_frame.h"

namespace kuka_rsi {

struct SessionConfig {
  // Consecutive missed/bad cycles after connection before latching a fault.
  unsigned max_consecutive_timeouts{5};
};

struct SessionStats {
  bool connected{false};
  bool fault{false};
  std::uint64_t last_ipoc{0};
  std::uint64_t total_timeouts{0};
  unsigned consecutive_timeouts{0};
  std::uint64_t bad_frames{0};
  std::uint64_t ipoc_jumps{0};  // non-increasing IPOC events (diagnostic only)
};

// Tracks RSI link health (spec section 12.2): IPOC continuity, timeout
// runs, and a latched communication fault. Timeouts before the first valid
// frame never fault (the KRC simply has not started RSI yet). A latched
// fault is only cleared by reset() — the manager/EKI layer owns recovery.
// All methods are allocation-free and non-blocking.
class RsiSessionMonitor {
 public:
  explicit RsiSessionMonitor(const SessionConfig& cfg) : cfg_(cfg) {}

  void onFrame(const RobFrame& frame);
  void onTimeout();
  void onBadFrame();
  void reset();

  bool connected() const { return stats_.connected; }
  bool faulted() const { return stats_.fault; }
  const SessionStats& stats() const { return stats_; }

 private:
  void countMiss();
  SessionConfig cfg_;
  SessionStats stats_;
};

}  // namespace kuka_rsi
```

`src/rsi_session_monitor.cpp`:

```cpp
#include "kuka_rsi_hw_interface/rsi_session_monitor.h"

namespace kuka_rsi {

void RsiSessionMonitor::onFrame(const RobFrame& frame) {
  if (stats_.connected && frame.ipoc <= stats_.last_ipoc) {
    ++stats_.ipoc_jumps;
  }
  stats_.last_ipoc = frame.ipoc;
  stats_.connected = true;
  stats_.consecutive_timeouts = 0;
}

void RsiSessionMonitor::onTimeout() {
  if (!stats_.connected) return;  // KRC not started yet: benign
  ++stats_.total_timeouts;
  countMiss();
}

void RsiSessionMonitor::onBadFrame() {
  if (!stats_.connected) return;
  ++stats_.bad_frames;
  countMiss();
}

void RsiSessionMonitor::countMiss() {
  ++stats_.consecutive_timeouts;
  if (stats_.consecutive_timeouts >= cfg_.max_consecutive_timeouts) {
    stats_.fault = true;  // latched until reset()
  }
}

void RsiSessionMonitor::reset() { stats_ = SessionStats{}; }

}  // namespace kuka_rsi
```

`include/kuka_rsi_hw_interface/command_limiter.h`:

```cpp
#pragma once

namespace kuka_rsi {

// Secondary hardware-layer RKorr limits (spec section 12.2). Defaults match
// the controller-layer sfc::SafetyParams so the hardware clamp only engages
// if a controller misbehaves.
struct CommandLimits {
  double max_trans{0.5};  // mm per cycle, axes 0-2
  double max_rot{0.05};   // deg per cycle, axes 3-5
};

// Final in-place clamp before serialization. Non-finite values become zero.
// Returns true if any axis was modified. Allocation-free.
class CommandLimiter {
 public:
  bool clamp(double corr[6], const CommandLimits& limits) const;
};

}  // namespace kuka_rsi
```

`src/command_limiter.cpp`:

```cpp
#include "kuka_rsi_hw_interface/command_limiter.h"

#include <algorithm>
#include <cmath>

namespace kuka_rsi {

namespace {
bool clampAxis(double& v, double lim) {
  if (!std::isfinite(v)) {
    v = 0.0;
    return true;
  }
  const double c = std::max(-lim, std::min(lim, v));
  if (c != v) {
    v = c;
    return true;
  }
  return false;
}
}  // namespace

bool CommandLimiter::clamp(double corr[6], const CommandLimits& limits) const {
  bool modified = false;
  for (int i = 0; i < 3; ++i) modified |= clampAxis(corr[i], limits.max_trans);
  for (int i = 3; i < 6; ++i) modified |= clampAxis(corr[i], limits.max_rot);
  return modified;
}

}  // namespace kuka_rsi
```

- [ ] **Step 4: 运行确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_rsi_hw_interface/test_rsi_session_monitor && \
  ./devel/lib/kuka_rsi_hw_interface/test_command_limiter
```

预期:`[  PASSED  ] 8 tests.` 与 `[  PASSED  ] 5 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "feat(rsi): add session monitor and secondary command limiter (Task 4)"
```

---

### Task 5: `UdpTransport`(非阻塞 loopback UDP 端点)

**Files:**
- Create: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/udp_transport.h`
- Create: `ros_ws/src/kuka_rsi_hw_interface/src/udp_transport.cpp`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_udp_transport.cpp`

**Interfaces:**
- Consumes: POSIX socket API(`socket/bind/poll/recvfrom/sendto`)。
- Produces(Task 7/8 共用同一传输类;PC 侧与 mock 侧只是绑定端口不同):
  - `kuka_rsi::UdpTransport`:
    - `bool bind(const std::string& ip, uint16_t port)` — 绑定本地端点(非实时,init 期调用;端口 0 = 内核分配)。
    - `uint16_t boundPort() const` — 实际绑定端口(测试用内核分配端口,避免端口冲突)。
    - `int receive(char* buf, std::size_t buf_size, int timeout_ms)` — `poll` 带超时;返回字节数,0 = 超时,-1 = 错误。记录最近发送方地址。
    - `bool sendToLastSender(const char* data, std::size_t len)` — 回发至最近收包方(RSI 语义:KRC 是 UDP 客户端,PC 只应答)。
    - `bool sendTo(const std::string& ip, uint16_t port, const char* data, std::size_t len)` — 指定目标发送(mock 端主动发起用)。
    - `void close()`;析构自动 close。禁拷贝。
- 实时说明:`receive/sendToLastSender` 无堆分配、无锁;阻塞上界由 `timeout_ms` 显式控制(实时路径传 ≤ 2 周期)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/kuka_rsi_hw_interface/test/test_udp_transport.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "kuka_rsi_hw_interface/udp_transport.h"

using kuka_rsi::UdpTransport;

TEST(UdpTransport, BindsToKernelAssignedPort) {
  UdpTransport t;
  ASSERT_TRUE(t.bind("127.0.0.1", 0));
  EXPECT_GT(t.boundPort(), 0);
}

TEST(UdpTransport, ReceiveTimesOutWhenNoData) {
  UdpTransport t;
  ASSERT_TRUE(t.bind("127.0.0.1", 0));
  char buf[64];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 20), 0);  // 20 ms, no sender
}

TEST(UdpTransport, RoundTripBetweenTwoEndpoints) {
  UdpTransport server;
  UdpTransport client;
  ASSERT_TRUE(server.bind("127.0.0.1", 0));
  ASSERT_TRUE(client.bind("127.0.0.1", 0));

  const char msg[] = "<Rob Type=\"KUKA\"/>";
  ASSERT_TRUE(client.sendTo("127.0.0.1", server.boundPort(), msg,
                            std::strlen(msg)));
  char buf[128];
  const int n = server.receive(buf, sizeof(buf), 200);
  ASSERT_EQ(n, static_cast<int>(std::strlen(msg)));
  EXPECT_EQ(std::string(buf, n), msg);
}

TEST(UdpTransport, ReplyReachesLastSender) {
  UdpTransport server;
  UdpTransport client;
  ASSERT_TRUE(server.bind("127.0.0.1", 0));
  ASSERT_TRUE(client.bind("127.0.0.1", 0));

  const char ping[] = "ping";
  ASSERT_TRUE(client.sendTo("127.0.0.1", server.boundPort(), ping, 4));
  char buf[64];
  ASSERT_EQ(server.receive(buf, sizeof(buf), 200), 4);

  const char pong[] = "pong";
  ASSERT_TRUE(server.sendToLastSender(pong, 4));
  const int n = client.receive(buf, sizeof(buf), 200);
  ASSERT_EQ(n, 4);
  EXPECT_EQ(std::string(buf, n), "pong");
}

TEST(UdpTransport, SendToLastSenderFailsBeforeAnyReceive) {
  UdpTransport t;
  ASSERT_TRUE(t.bind("127.0.0.1", 0));
  EXPECT_FALSE(t.sendToLastSender("x", 1));
}

TEST(UdpTransport, OperationsFailWhenUnbound) {
  UdpTransport t;
  char buf[8];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 10), -1);
  EXPECT_FALSE(t.sendTo("127.0.0.1", 1, "x", 1));
}
```

- [ ] **Step 2: 加构建条目并确认失败**

`CMakeLists.txt`:`kuka_rsi_protocol` 源列表追加 `src/udp_transport.cpp`;测试段追加:

```cmake
  catkin_add_gtest(test_udp_transport test/test_udp_transport.cpp)
  target_link_libraries(test_udp_transport kuka_rsi_protocol
                        ${GTEST_MAIN_LIBRARIES})
```

运行:`cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
预期:BUILD FAILS,`udp_transport.h: No such file or directory`。

- [ ] **Step 3: 写最小实现**

`include/kuka_rsi_hw_interface/udp_transport.h`:

```cpp
#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace kuka_rsi {

// Minimal non-blocking UDP endpoint shared by the PC-side hardware
// interface and the KRC-side mock. receive() uses poll() with an explicit
// timeout so the realtime read path has a bounded wait. No allocation or
// locking after bind(). Not copyable.
class UdpTransport {
 public:
  UdpTransport() = default;
  ~UdpTransport() { close(); }
  UdpTransport(const UdpTransport&) = delete;
  UdpTransport& operator=(const UdpTransport&) = delete;

  // Binds the local endpoint. port 0 lets the kernel pick a free port
  // (used by tests to avoid collisions). Non-realtime; call during init.
  bool bind(const std::string& ip, std::uint16_t port);
  std::uint16_t boundPort() const { return bound_port_; }

  // Waits up to timeout_ms for a datagram. Returns byte count, 0 on
  // timeout, -1 on error/unbound. Remembers the sender for replies.
  int receive(char* buf, std::size_t buf_size, int timeout_ms);

  // Sends to the peer that sent the last received datagram (RSI role: the
  // KRC is the UDP client; the PC only ever answers).
  bool sendToLastSender(const char* data, std::size_t len);

  // Sends to an explicit destination (used by the mock to start the cycle).
  bool sendTo(const std::string& ip, std::uint16_t port, const char* data,
              std::size_t len);

  void close();

 private:
  int fd_{-1};
  std::uint16_t bound_port_{0};
  sockaddr_in last_sender_{};
  bool has_last_sender_{false};
};

}  // namespace kuka_rsi
```

`src/udp_transport.cpp`:

```cpp
#include "kuka_rsi_hw_interface/udp_transport.h"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace kuka_rsi {

bool UdpTransport::bind(const std::string& ip, std::uint16_t port) {
  close();
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) return false;

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    close();
    return false;
  }
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close();
    return false;
  }
  sockaddr_in bound{};
  socklen_t len = sizeof(bound);
  if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&bound), &len) != 0) {
    close();
    return false;
  }
  bound_port_ = ntohs(bound.sin_port);
  return true;
}

int UdpTransport::receive(char* buf, std::size_t buf_size, int timeout_ms) {
  if (fd_ < 0) return -1;
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr == 0) return 0;   // timeout
  if (pr < 0) return -1;   // error
  sockaddr_in sender{};
  socklen_t sender_len = sizeof(sender);
  const ssize_t n =
      ::recvfrom(fd_, buf, buf_size, 0,
                 reinterpret_cast<sockaddr*>(&sender), &sender_len);
  if (n < 0) return -1;
  last_sender_ = sender;
  has_last_sender_ = true;
  return static_cast<int>(n);
}

bool UdpTransport::sendToLastSender(const char* data, std::size_t len) {
  if (fd_ < 0 || !has_last_sender_) return false;
  const ssize_t n =
      ::sendto(fd_, data, len, 0,
               reinterpret_cast<const sockaddr*>(&last_sender_),
               sizeof(last_sender_));
  return n == static_cast<ssize_t>(len);
}

bool UdpTransport::sendTo(const std::string& ip, std::uint16_t port,
                          const char* data, std::size_t len) {
  if (fd_ < 0) return false;
  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &dst.sin_addr) != 1) return false;
  const ssize_t n = ::sendto(fd_, data, len, 0,
                             reinterpret_cast<const sockaddr*>(&dst),
                             sizeof(dst));
  return n == static_cast<ssize_t>(len);
}

void UdpTransport::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  bound_port_ = 0;
  has_last_sender_ = false;
}

}  // namespace kuka_rsi
```

- [ ] **Step 4: 运行确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_rsi_hw_interface/test_udp_transport
```

预期:`[  PASSED  ] 6 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "feat(rsi): add poll-based non-blocking UdpTransport (Task 5)"
```

---

### Task 6: 自定义 ros_control 笛卡尔硬件接口

**Files:**
- Create: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/cartesian_command_interface.h`(header-only)
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_cartesian_interfaces.cpp`

**Interfaces:**
- Consumes: `hardware_interface`(`HardwareResourceManager`、`ClaimResources`)。
- Produces(Plan 3 控制器 claim 的资源;资源名约定 `kuka_tcp`):
  - `kuka_rsi::CartesianStateHandle`:名字 + 6 个 `const double*`(x/y/z/a/b/c,mm/deg)。
  - `kuka_rsi::CartesianStateInterface`:只读,`HardwareResourceManager<CartesianStateHandle>`(DontClaimResources)。
  - `kuka_rsi::CartesianCorrectionHandle`:继承 state handle,另持 6 个 `double*` 命令指针,`setCommand(x,y,z,a,b,c)` / 各分量 getter。
  - `kuka_rsi::CartesianCorrectionCommandInterface`:`HardwareResourceManager<CartesianCorrectionHandle, ClaimResources>`(独占 claim,规格 §5.3"同一时刻仅一个控制器")。
- 设计对齐 `joint_command_interface.h` 惯例:空指针在句柄构造时抛 `HardwareInterfaceException`(非实时构造期,允许异常)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/kuka_rsi_hw_interface/test/test_cartesian_interfaces.cpp`:

```cpp
#include <gtest/gtest.h>

#include <hardware_interface/hardware_interface.h>

#include "kuka_rsi_hw_interface/cartesian_command_interface.h"

using kuka_rsi::CartesianCorrectionCommandInterface;
using kuka_rsi::CartesianCorrectionHandle;
using kuka_rsi::CartesianStateHandle;
using kuka_rsi::CartesianStateInterface;

class CartesianInterfacesTest : public ::testing::Test {
 protected:
  double pos_[6] = {100.0, 200.0, 300.0, 10.0, 20.0, 30.0};
  double cmd_[6] = {0, 0, 0, 0, 0, 0};
};

TEST_F(CartesianInterfacesTest, StateHandleExposesPose) {
  CartesianStateHandle h("kuka_tcp", &pos_[0], &pos_[1], &pos_[2], &pos_[3],
                         &pos_[4], &pos_[5]);
  EXPECT_EQ(h.getName(), "kuka_tcp");
  EXPECT_DOUBLE_EQ(h.getX(), 100.0);
  EXPECT_DOUBLE_EQ(h.getC(), 30.0);
  pos_[0] = 101.0;  // handle reads live data
  EXPECT_DOUBLE_EQ(h.getX(), 101.0);
}

TEST_F(CartesianInterfacesTest, StateHandleRejectsNullPointer) {
  EXPECT_THROW(CartesianStateHandle("bad", nullptr, &pos_[1], &pos_[2],
                                    &pos_[3], &pos_[4], &pos_[5]),
               hardware_interface::HardwareInterfaceException);
}

TEST_F(CartesianInterfacesTest, CorrectionHandleWritesCommand) {
  CartesianStateHandle state("kuka_tcp", &pos_[0], &pos_[1], &pos_[2],
                             &pos_[3], &pos_[4], &pos_[5]);
  CartesianCorrectionHandle h(state, &cmd_[0], &cmd_[1], &cmd_[2], &cmd_[3],
                              &cmd_[4], &cmd_[5]);
  h.setCommand(0.1, 0.2, 0.3, 0.01, 0.02, 0.03);
  EXPECT_DOUBLE_EQ(cmd_[0], 0.1);
  EXPECT_DOUBLE_EQ(cmd_[5], 0.03);
  EXPECT_DOUBLE_EQ(h.getCommandX(), 0.1);
  EXPECT_DOUBLE_EQ(h.getCommandC(), 0.03);
  EXPECT_DOUBLE_EQ(h.getX(), 100.0);  // still sees state
}

TEST_F(CartesianInterfacesTest, InterfacesRegisterAndRetrieveByName) {
  CartesianStateHandle state("kuka_tcp", &pos_[0], &pos_[1], &pos_[2],
                             &pos_[3], &pos_[4], &pos_[5]);
  CartesianStateInterface state_if;
  state_if.registerHandle(state);
  EXPECT_DOUBLE_EQ(state_if.getHandle("kuka_tcp").getY(), 200.0);

  CartesianCorrectionHandle corr(state, &cmd_[0], &cmd_[1], &cmd_[2],
                                 &cmd_[3], &cmd_[4], &cmd_[5]);
  CartesianCorrectionCommandInterface cmd_if;
  cmd_if.registerHandle(corr);
  cmd_if.getHandle("kuka_tcp").setCommand(1, 2, 3, 4, 5, 6);
  EXPECT_DOUBLE_EQ(cmd_[2], 3.0);
  EXPECT_THROW(cmd_if.getHandle("missing"),
               hardware_interface::HardwareInterfaceException);
}

TEST_F(CartesianInterfacesTest, CommandInterfaceClaimsResources) {
  CartesianStateHandle state("kuka_tcp", &pos_[0], &pos_[1], &pos_[2],
                             &pos_[3], &pos_[4], &pos_[5]);
  CartesianCorrectionHandle corr(state, &cmd_[0], &cmd_[1], &cmd_[2],
                                 &cmd_[3], &cmd_[4], &cmd_[5]);
  CartesianCorrectionCommandInterface cmd_if;
  cmd_if.registerHandle(corr);
  cmd_if.claim("kuka_tcp");
  EXPECT_EQ(cmd_if.getClaims().size(), 1u);  // exclusive-claim interface
  CartesianStateInterface state_if;
  state_if.registerHandle(state);
  state_if.claim("kuka_tcp");
  EXPECT_TRUE(state_if.getClaims().empty());  // read-only: never claims
}
```

- [ ] **Step 2: 加测试目标并确认失败**

`CMakeLists.txt` 测试段追加:

```cmake
  catkin_add_gtest(test_cartesian_interfaces test/test_cartesian_interfaces.cpp)
  target_link_libraries(test_cartesian_interfaces ${catkin_LIBRARIES}
                        ${GTEST_MAIN_LIBRARIES})
```

运行:`cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
预期:BUILD FAILS,`cartesian_command_interface.h: No such file or directory`。

- [ ] **Step 3: 写最小实现**

`include/kuka_rsi_hw_interface/cartesian_command_interface.h`:

```cpp
#pragma once

#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

#include <string>

namespace kuka_rsi {

// Read-only handle to the KUKA Cartesian TCP pose (RIst). Units: mm / deg
// (KUKA A/B/C = Z-Y-X Euler). Registered under a resource name, by
// convention "kuka_tcp".
class CartesianStateHandle {
 public:
  CartesianStateHandle() = default;
  CartesianStateHandle(const std::string& name, const double* x,
                       const double* y, const double* z, const double* a,
                       const double* b, const double* c)
      : name_(name), x_(x), y_(y), z_(z), a_(a), b_(b), c_(c) {
    if (!x || !y || !z || !a || !b || !c) {
      throw hardware_interface::HardwareInterfaceException(
          "Cannot create CartesianStateHandle '" + name +
          "': a data pointer is null.");
    }
  }

  std::string getName() const { return name_; }
  double getX() const { return *x_; }
  double getY() const { return *y_; }
  double getZ() const { return *z_; }
  double getA() const { return *a_; }
  double getB() const { return *b_; }
  double getC() const { return *c_; }

 private:
  std::string name_;
  const double* x_{nullptr};
  const double* y_{nullptr};
  const double* z_{nullptr};
  const double* a_{nullptr};
  const double* b_{nullptr};
  const double* c_{nullptr};
};

// Read-write handle: state plus the per-cycle RKorr command (mm/deg per
// 4 ms cycle). Claimed exclusively by one controller at a time (spec 5.3).
class CartesianCorrectionHandle : public CartesianStateHandle {
 public:
  CartesianCorrectionHandle() = default;
  CartesianCorrectionHandle(const CartesianStateHandle& state, double* cx,
                            double* cy, double* cz, double* ca, double* cb,
                            double* cc)
      : CartesianStateHandle(state),
        cx_(cx), cy_(cy), cz_(cz), ca_(ca), cb_(cb), cc_(cc) {
    if (!cx || !cy || !cz || !ca || !cb || !cc) {
      throw hardware_interface::HardwareInterfaceException(
          "Cannot create CartesianCorrectionHandle '" + state.getName() +
          "': a command pointer is null.");
    }
  }

  void setCommand(double x, double y, double z, double a, double b,
                  double c) {
    *cx_ = x;
    *cy_ = y;
    *cz_ = z;
    *ca_ = a;
    *cb_ = b;
    *cc_ = c;
  }
  double getCommandX() const { return *cx_; }
  double getCommandY() const { return *cy_; }
  double getCommandZ() const { return *cz_; }
  double getCommandA() const { return *ca_; }
  double getCommandB() const { return *cb_; }
  double getCommandC() const { return *cc_; }

 private:
  double* cx_{nullptr};
  double* cy_{nullptr};
  double* cz_{nullptr};
  double* ca_{nullptr};
  double* cb_{nullptr};
  double* cc_{nullptr};
};

// Read-only interface: never claims resources.
class CartesianStateInterface
    : public hardware_interface::HardwareResourceManager<
          CartesianStateHandle> {};

// Command interface: exclusive claim, enforced by controller_manager.
class CartesianCorrectionCommandInterface
    : public hardware_interface::HardwareResourceManager<
          CartesianCorrectionHandle, hardware_interface::ClaimResources> {};

}  // namespace kuka_rsi
```

- [ ] **Step 4: 运行确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_rsi_hw_interface/test_cartesian_interfaces
```

预期:`[  PASSED  ] 5 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "feat(rsi): add Cartesian state/correction hardware interfaces (Task 6)"
```

---

### Task 7: RSI Mock(`RsiMockCore` 行为模型 + `RsiMockServer` UDP 对端)

**Files:**
- Create: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/rsi_mock_core.h`
- Create: `ros_ws/src/kuka_rsi_hw_interface/src/rsi_mock_core.cpp`
- Create: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/rsi_mock_server.h`
- Create: `ros_ws/src/kuka_rsi_hw_interface/src/rsi_mock_server.cpp`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_mock.cpp`

**Interfaces:**
- Consumes: `RobFrame`/`SenFrame`/`parseRobFrame`/`serializeSenFrame`(Task 2/3)、`UdpTransport`(Task 5)。注意 mock 方向相反:**序列化 Rob 帧、解析 Sen 帧**,两者为 mock 私有,不入公共头。
- Produces(Task 8/9 测试与 `kuka_rsi_sim_server` 消费):
  - `kuka_rsi::MockConfig { double x0,y0,z0{800},a0,b0,c0; uint64_t ipoc_start{1000}; uint64_t ipoc_step{4}; }`
  - `kuka_rsi::MockStats { uint64_t frames_sent; uint64_t replies_received; uint64_t reply_timeouts; uint64_t ipoc_echo_errors; uint64_t parse_errors; int last_stop; uint64_t last_watchdog; }`
  - `kuka_rsi::RsiMockCore`(无网络、确定性;KRC 行为模型):
    - `explicit RsiMockCore(const MockConfig&)`
    - `std::size_t buildStateFrame(char* buf, std::size_t size)` — 生成当前位姿 + 当前 IPOC 的 Rob 帧,IPOC 随后自增 `ipoc_step`。
    - `bool applyReply(const char* data, std::size_t len)` — 解析 Sen 帧:IPOC 回显必须等于**刚发出的** IPOC,否则 `ipoc_echo_errors++` 且不积分;正确则把 RKorr 积分进位姿(相对修正语义),记录 stop/watchdog。解析失败 `parse_errors++`。
    - `void noteReplyTimeout()`、位姿 getter(`x() ... c()`)、`const MockStats& stats() const`。
    - 关节角固定输出 0(本计划不建运动学;`AIPos` 全零帧即可满足 PC 侧解析)。
  - `kuka_rsi::RsiMockServer`(线程封装,扮演 UDP 客户端 KRC):
    - `RsiMockServer(const MockConfig&, const std::string& target_ip, uint16_t target_port, int reply_timeout_ms = 50)`
    - `bool start()` / `void stop()`;后台线程循环:发状态帧 → 等回复(超时计 `reply_timeouts`)→ `applyReply`;**测试用途不模拟 4 ms 节拍**(全速循环,由 PC 侧 poll 驱动节奏),真实节拍留给 `kuka_rsi_sim_server` 可执行的 `--cycle-ms` 参数。
    - `MockStats statsSnapshot() const`(互斥锁保护;mock 非实时代码,允许锁)。
    - `void setPose(double x,double y,double z,double a,double b,double c)`(注入位姿轨迹/跳变,锁保护)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_mock.cpp`:

```cpp
#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "kuka_rsi_hw_interface/rsi_frame.h"
#include "kuka_rsi_hw_interface/rsi_mock_core.h"
#include "kuka_rsi_hw_interface/rsi_mock_server.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

using kuka_rsi::MockConfig;
using kuka_rsi::RobFrame;
using kuka_rsi::RsiMockCore;
using kuka_rsi::RsiMockServer;
using kuka_rsi::SenFrame;
using kuka_rsi::UdpTransport;

namespace {
std::string senReply(std::uint64_t ipoc, double x = 0.0, double a = 0.0) {
  SenFrame f;
  f.x = x;
  f.a = a;
  f.ipoc = ipoc;
  f.watchdog = 7;
  char buf[1024];
  const std::size_t n = kuka_rsi::serializeSenFrame(f, buf, sizeof(buf));
  return std::string(buf, n);
}
}  // namespace

TEST(MockCore, StateFrameIsParsableAndIpocAdvances) {
  MockConfig cfg;
  cfg.x0 = 100.0;
  cfg.ipoc_start = 1000;
  cfg.ipoc_step = 4;
  RsiMockCore core(cfg);

  char buf[1024];
  ASSERT_GT(core.buildStateFrame(buf, sizeof(buf)), 0u);
  RobFrame f;
  ASSERT_TRUE(kuka_rsi::parseRobFrame(buf, std::strlen(buf), f));
  EXPECT_DOUBLE_EQ(f.x, 100.0);
  EXPECT_DOUBLE_EQ(f.z, 800.0);
  EXPECT_EQ(f.ipoc, 1000u);

  ASSERT_GT(core.buildStateFrame(buf, sizeof(buf)), 0u);
  ASSERT_TRUE(kuka_rsi::parseRobFrame(buf, std::strlen(buf), f));
  EXPECT_EQ(f.ipoc, 1004u);
}

TEST(MockCore, CorrectEchoIntegratesCorrection) {
  RsiMockCore core(MockConfig{});
  char buf[1024];
  core.buildStateFrame(buf, sizeof(buf));  // sends ipoc_start = 1000
  const std::string reply = senReply(1000, 0.5, 0.01);
  ASSERT_TRUE(core.applyReply(reply.data(), reply.size()));
  EXPECT_DOUBLE_EQ(core.x(), 0.5);
  EXPECT_DOUBLE_EQ(core.a(), 0.01);
  EXPECT_EQ(core.stats().ipoc_echo_errors, 0u);
  EXPECT_EQ(core.stats().last_watchdog, 7u);
}

TEST(MockCore, WrongEchoCountsErrorAndDoesNotIntegrate) {
  RsiMockCore core(MockConfig{});
  char buf[1024];
  core.buildStateFrame(buf, sizeof(buf));  // ipoc 1000
  const std::string reply = senReply(999, 5.0);
  EXPECT_FALSE(core.applyReply(reply.data(), reply.size()));
  EXPECT_DOUBLE_EQ(core.x(), 0.0);
  EXPECT_EQ(core.stats().ipoc_echo_errors, 1u);
}

TEST(MockCore, MalformedReplyCountsParseError) {
  RsiMockCore core(MockConfig{});
  char buf[1024];
  core.buildStateFrame(buf, sizeof(buf));
  EXPECT_FALSE(core.applyReply("garbage", 7));
  EXPECT_EQ(core.stats().parse_errors, 1u);
}

TEST(MockCore, CorrectionsAccumulateAcrossCycles) {
  RsiMockCore core(MockConfig{});
  char buf[1024];
  for (int i = 0; i < 3; ++i) {
    core.buildStateFrame(buf, sizeof(buf));
    RobFrame f;
    ASSERT_TRUE(kuka_rsi::parseRobFrame(buf, std::strlen(buf), f));
    const std::string reply = senReply(f.ipoc, 0.1);
    ASSERT_TRUE(core.applyReply(reply.data(), reply.size()));
  }
  EXPECT_NEAR(core.x(), 0.3, 1e-12);
  EXPECT_EQ(core.stats().frames_sent, 3u);
  EXPECT_EQ(core.stats().replies_received, 3u);
}

TEST(MockServer, DrivesCycleAgainstManualPeer) {
  UdpTransport pc;  // hand-rolled PC side
  ASSERT_TRUE(pc.bind("127.0.0.1", 0));

  MockConfig cfg;
  cfg.x0 = 50.0;
  RsiMockServer server(cfg, "127.0.0.1", pc.boundPort(), 100);
  ASSERT_TRUE(server.start());

  // Answer 5 cycles with a +0.2 mm X correction each.
  char buf[1024];
  for (int i = 0; i < 5; ++i) {
    const int n = pc.receive(buf, sizeof(buf), 500);
    ASSERT_GT(n, 0);
    RobFrame f;
    ASSERT_TRUE(kuka_rsi::parseRobFrame(buf, n, f));
    const std::string reply = senReply(f.ipoc, 0.2);
    ASSERT_TRUE(pc.sendToLastSender(reply.data(), reply.size()));
  }
  server.stop();

  const kuka_rsi::MockStats s = server.statsSnapshot();
  EXPECT_GE(s.replies_received, 5u);
  EXPECT_EQ(s.ipoc_echo_errors, 0u);
}

TEST(MockServer, CountsReplyTimeouts) {
  RsiMockServer server(MockConfig{}, "127.0.0.1", 1 /* nobody listens */, 10);
  ASSERT_TRUE(server.start());
  // Bounded wait: 3 timeouts at 10 ms each should arrive well within 2 s.
  for (int i = 0; i < 200 && server.statsSnapshot().reply_timeouts < 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  server.stop();
  EXPECT_GE(server.statsSnapshot().reply_timeouts, 3u);
  EXPECT_EQ(server.statsSnapshot().replies_received, 0u);
}
```

- [ ] **Step 2: 加构建条目并确认失败**

`CMakeLists.txt`:新增 mock 库(与协议库分开,ROS 壳不链接它):

```cmake
add_library(kuka_rsi_hw_interface_mock
  src/rsi_mock_core.cpp
  src/rsi_mock_server.cpp
)
target_link_libraries(kuka_rsi_hw_interface_mock kuka_rsi_protocol pthread)
```

测试段追加:

```cmake
  catkin_add_gtest(test_rsi_mock test/test_rsi_mock.cpp)
  target_link_libraries(test_rsi_mock kuka_rsi_hw_interface_mock
                        ${GTEST_MAIN_LIBRARIES})
```

运行:`cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
预期:BUILD FAILS,`rsi_mock_core.h: No such file or directory`。

- [ ] **Step 3: 写最小实现**

`include/kuka_rsi_hw_interface/rsi_mock_core.h`:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>

namespace kuka_rsi {

// Initial pose and IPOC settings for the mock KRC.
struct MockConfig {
  double x0{0}, y0{0}, z0{800.0}, a0{0}, b0{0}, c0{0};
  std::uint64_t ipoc_start{1000};
  std::uint64_t ipoc_step{4};
};

struct MockStats {
  std::uint64_t frames_sent{0};
  std::uint64_t replies_received{0};
  std::uint64_t reply_timeouts{0};
  std::uint64_t ipoc_echo_errors{0};
  std::uint64_t parse_errors{0};
  int last_stop{0};
  std::uint64_t last_watchdog{0};
};

// Deterministic KRC-side RSI behavior model (spec section 15.2), no
// networking: emits <Rob> state frames and integrates <Sen> RKorr replies
// into its pose when the IPOC echo matches the frame just sent. Axis
// angles are reported as zero (no kinematics in this plan).
class RsiMockCore {
 public:
  explicit RsiMockCore(const MockConfig& cfg);

  // Serializes the current state frame and advances IPOC for the next
  // cycle. Returns payload length, 0 if the buffer is too small.
  std::size_t buildStateFrame(char* buf, std::size_t size);

  // Parses a <Sen> reply. On correct IPOC echo, integrates RKorr into the
  // pose and records stop/watchdog; returns true. On parse failure or
  // wrong echo, bumps the matching counter and returns false.
  bool applyReply(const char* data, std::size_t len);

  void noteReplyTimeout() { ++stats_.reply_timeouts; }

  void setPose(double x, double y, double z, double a, double b, double c) {
    x_ = x; y_ = y; z_ = z; a_ = a; b_ = b; c_ = c;
  }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double a() const { return a_; }
  double b() const { return b_; }
  double c() const { return c_; }
  const MockStats& stats() const { return stats_; }

 private:
  double x_, y_, z_, a_, b_, c_;
  std::uint64_t next_ipoc_;
  std::uint64_t ipoc_step_;
  std::uint64_t awaited_ipoc_{0};
  bool awaiting_{false};
  MockStats stats_;
};

}  // namespace kuka_rsi
```

`src/rsi_mock_core.cpp`:

```cpp
#include "kuka_rsi_hw_interface/rsi_mock_core.h"

#include <tinyxml2.h>

#include <cinttypes>
#include <cstdio>

namespace kuka_rsi {

RsiMockCore::RsiMockCore(const MockConfig& cfg)
    : x_(cfg.x0), y_(cfg.y0), z_(cfg.z0), a_(cfg.a0), b_(cfg.b0), c_(cfg.c0),
      next_ipoc_(cfg.ipoc_start), ipoc_step_(cfg.ipoc_step) {}

std::size_t RsiMockCore::buildStateFrame(char* buf, std::size_t size) {
  const int n = std::snprintf(
      buf, size,
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"%.4f\" Y=\"%.4f\" Z=\"%.4f\" A=\"%.4f\" B=\"%.4f\" "
      "C=\"%.4f\"/>"
      "<AIPos A1=\"0.0\" A2=\"0.0\" A3=\"0.0\" A4=\"0.0\" A5=\"0.0\" "
      "A6=\"0.0\"/>"
      "<Delay D=\"0\"/>"
      "<Mode M=\"1\"/>"
      "<IPOC>%" PRIu64 "</IPOC>"
      "</Rob>",
      x_, y_, z_, a_, b_, c_, next_ipoc_);
  if (n <= 0 || static_cast<std::size_t>(n) >= size) return 0;
  awaited_ipoc_ = next_ipoc_;
  awaiting_ = true;
  next_ipoc_ += ipoc_step_;
  ++stats_.frames_sent;
  return static_cast<std::size_t>(n);
}

bool RsiMockCore::applyReply(const char* data, std::size_t len) {
  tinyxml2::XMLDocument doc;
  if (data == nullptr || doc.Parse(data, len) != tinyxml2::XML_SUCCESS) {
    ++stats_.parse_errors;
    return false;
  }
  const tinyxml2::XMLElement* sen = doc.FirstChildElement("Sen");
  if (sen == nullptr) {
    ++stats_.parse_errors;
    return false;
  }
  const tinyxml2::XMLElement* rkorr = sen->FirstChildElement("RKorr");
  const tinyxml2::XMLElement* ipoc = sen->FirstChildElement("IPOC");
  if (rkorr == nullptr || ipoc == nullptr || ipoc->GetText() == nullptr) {
    ++stats_.parse_errors;
    return false;
  }
  std::uint64_t echoed = 0;
  if (std::sscanf(ipoc->GetText(), "%" SCNu64, &echoed) != 1) {
    ++stats_.parse_errors;
    return false;
  }
  ++stats_.replies_received;
  if (!awaiting_ || echoed != awaited_ipoc_) {
    ++stats_.ipoc_echo_errors;
    return false;
  }
  awaiting_ = false;

  // Relative Cartesian correction: integrate into the pose (spec 6.1).
  x_ += rkorr->DoubleAttribute("X");
  y_ += rkorr->DoubleAttribute("Y");
  z_ += rkorr->DoubleAttribute("Z");
  a_ += rkorr->DoubleAttribute("A");
  b_ += rkorr->DoubleAttribute("B");
  c_ += rkorr->DoubleAttribute("C");

  const tinyxml2::XMLElement* stop = sen->FirstChildElement("Stop");
  if (stop != nullptr) stats_.last_stop = stop->IntAttribute("S");
  const tinyxml2::XMLElement* wd = sen->FirstChildElement("Watchdog");
  if (wd != nullptr) {
    // Int64Attribute: tinyxml2 6.2 has no Unsigned64Attribute (7.0+).
    stats_.last_watchdog =
        static_cast<std::uint64_t>(wd->Int64Attribute("W"));
  }
  return true;
}

}  // namespace kuka_rsi
```

`include/kuka_rsi_hw_interface/rsi_mock_server.h`:

```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "kuka_rsi_hw_interface/rsi_mock_core.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

namespace kuka_rsi {

// Threaded UDP wrapper around RsiMockCore playing the KRC role: the mock
// is the UDP client, it sends the state frame first and waits for the
// PC's <Sen> reply each cycle (spec section 6.1). Test code: locking and
// threads are fine here, this never runs in the realtime path. Cycles run
// back-to-back (no 4 ms pacing) so tests finish fast; the standalone
// kuka_rsi_sim_server adds real pacing via --cycle-ms.
class RsiMockServer {
 public:
  RsiMockServer(const MockConfig& cfg, const std::string& target_ip,
                std::uint16_t target_port, int reply_timeout_ms = 50);
  ~RsiMockServer() { stop(); }

  bool start();
  void stop();

  MockStats statsSnapshot() const;
  void setPose(double x, double y, double z, double a, double b, double c);
  double poseX() const;

 private:
  void run();

  RsiMockCore core_;
  std::string target_ip_;
  std::uint16_t target_port_;
  int reply_timeout_ms_;
  UdpTransport udp_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex mutex_;
};

}  // namespace kuka_rsi
```

`src/rsi_mock_server.cpp`:

```cpp
#include "kuka_rsi_hw_interface/rsi_mock_server.h"

namespace kuka_rsi {

RsiMockServer::RsiMockServer(const MockConfig& cfg,
                             const std::string& target_ip,
                             std::uint16_t target_port, int reply_timeout_ms)
    : core_(cfg),
      target_ip_(target_ip),
      target_port_(target_port),
      reply_timeout_ms_(reply_timeout_ms) {}

bool RsiMockServer::start() {
  if (running_) return false;
  if (!udp_.bind("127.0.0.1", 0)) return false;
  running_ = true;
  thread_ = std::thread(&RsiMockServer::run, this);
  return true;
}

void RsiMockServer::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  udp_.close();
}

void RsiMockServer::run() {
  char tx[1024];
  char rx[1024];
  while (running_) {
    std::size_t n;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      n = core_.buildStateFrame(tx, sizeof(tx));
    }
    if (n == 0) break;
    if (!udp_.sendTo(target_ip_, target_port_, tx, n)) break;
    const int r = udp_.receive(rx, sizeof(rx), reply_timeout_ms_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (r > 0) {
      core_.applyReply(rx, static_cast<std::size_t>(r));
    } else {
      core_.noteReplyTimeout();
    }
  }
}

MockStats RsiMockServer::statsSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return core_.stats();
}

void RsiMockServer::setPose(double x, double y, double z, double a, double b,
                            double c) {
  std::lock_guard<std::mutex> lock(mutex_);
  core_.setPose(x, y, z, a, b, c);
}

double RsiMockServer::poseX() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return core_.x();
}

}  // namespace kuka_rsi
```

- [ ] **Step 4: 运行确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_rsi_hw_interface/test_rsi_mock
```

预期:`[  PASSED  ] 7 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "feat(rsi): add KRC-side RSI mock core and UDP server (Task 7)"
```

---

### Task 8: `KukaRsiRobotHW`(RobotHW 组装:read/write + 接口注册 + 故障零输出)

**Files:**
- Create: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/kuka_rsi_robot_hw.h`
- Create: `ros_ws/src/kuka_rsi_hw_interface/src/kuka_rsi_robot_hw.cpp`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_kuka_rsi_robot_hw.cpp`

**Interfaces:**
- Consumes: Task 2-6 全部(`rsi_frame`、`RsiSessionMonitor`、`CommandLimiter`、`UdpTransport`、笛卡尔接口)、`hardware_interface::{RobotHW, JointStateInterface}`。
- Produces(Plan 3 控制器与本包节点壳消费):
  - `kuka_rsi::HwConfig { std::string listen_ip{"0.0.0.0"}; uint16_t listen_port{49152}; int read_timeout_ms{8}; unsigned max_consecutive_timeouts{5}; CommandLimits limits; }`
  - `kuka_rsi::KukaRsiRobotHW : public hardware_interface::RobotHW`:
    - `bool configure(const HwConfig&)` — 免 ROS master 入口:绑定 UDP、注册接口(gtest 直用)。
    - `bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override` — 从参数服务器读 `HwConfig` 后转调 `configure`(节点壳用;本 Task 一并实现,行为验证靠 configure 路径)。
    - `void read(const ros::Time&, const ros::Duration&) override` — poll 收帧 → 解析 → 会话监控 → 更新状态缓冲(关节角 deg→rad)。
    - `void write(const ros::Time&, const ros::Duration&) override` — 组 Sen 帧回发:故障或未连接 → 零修正 + `Stop=1`(故障时);否则命令经 `CommandLimiter` 限幅后发出;IPOC 回显 = 最近收到值;watchdog 自增;**每周期结束将命令缓冲清零**(修正为增量语义,防止 stale 命令被重复积分)。
    - 注册:`JointStateInterface`(joint_a1..a6,rad,速度/力矩恒 0)、`CartesianStateInterface`、`CartesianCorrectionCommandInterface`(资源名 `kuka_tcp`)。
    - 诊断:`bool connected() const`、`bool faulted() const`、`const SessionStats& sessionStats() const`、`uint64_t saturationCount() const`、`void resetFault()`。
- `read()` 语义:一次 `receive`;超时 → `onTimeout`(保持上周期状态);坏帧 → `onBadFrame`;好帧才更新状态。`write()` 在没有可回复对象(从未收到帧)时直接返回。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/kuka_rsi_hw_interface/test/test_kuka_rsi_robot_hw.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"
#include "kuka_rsi_hw_interface/rsi_frame.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

using kuka_rsi::HwConfig;
using kuka_rsi::KukaRsiRobotHW;
using kuka_rsi::RobFrame;
using kuka_rsi::UdpTransport;

namespace {

// Hand-rolled KRC peer: gives full control over frame content and timing.
class FakeKrc {
 public:
  bool bind() { return udp_.bind("127.0.0.1", 0); }

  bool sendState(std::uint16_t hw_port, double x, double a1_deg,
                 std::uint64_t ipoc) {
    char buf[1024];
    const int n = std::snprintf(
        buf, sizeof(buf),
        "<Rob Type=\"KUKA\">"
        "<RIst X=\"%.4f\" Y=\"0.0\" Z=\"800.0\" A=\"0.0\" B=\"0.0\" "
        "C=\"0.0\"/>"
        "<AIPos A1=\"%.4f\" A2=\"0.0\" A3=\"0.0\" A4=\"0.0\" A5=\"0.0\" "
        "A6=\"0.0\"/>"
        "<IPOC>%llu</IPOC>"
        "</Rob>",
        x, a1_deg, static_cast<unsigned long long>(ipoc));
    return udp_.sendTo("127.0.0.1", hw_port, buf,
                       static_cast<std::size_t>(n));
  }

  bool sendGarbage(std::uint16_t hw_port) {
    return udp_.sendTo("127.0.0.1", hw_port, "not xml", 7);
  }

  // Receives the hw interface's <Sen> reply and parses key fields.
  bool receiveReply(double& rkorr_x, int& stop, std::uint64_t& ipoc,
                    int timeout_ms = 500) {
    char buf[1024];
    const int n = udp_.receive(buf, sizeof(buf), timeout_ms);
    if (n <= 0) return false;
    const std::string s(buf, n);
    double a, b, c, y, z;
    unsigned long long wd, ip;
    if (std::sscanf(s.c_str(),
                    "<Sen Type=\"ROS\">"
                    "<RKorr X=\"%lf\" Y=\"%lf\" Z=\"%lf\" A=\"%lf\" "
                    "B=\"%lf\" C=\"%lf\"/>"
                    "<Stop S=\"%d\"/>"
                    "<Watchdog W=\"%llu\"/>"
                    "<IPOC>%llu</IPOC>",
                    &rkorr_x, &y, &z, &a, &b, &c, &stop, &wd, &ip) != 9) {
      return false;
    }
    ipoc = ip;
    return true;
  }

 private:
  UdpTransport udp_;
};

HwConfig testConfig(unsigned max_timeouts = 3) {
  HwConfig cfg;
  cfg.listen_ip = "127.0.0.1";
  cfg.listen_port = 0;         // kernel-assigned, collision-free
  cfg.read_timeout_ms = 20;    // keep tests fast
  cfg.max_consecutive_timeouts = max_timeouts;
  return cfg;
}

constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

TEST(RobotHW, ConfigureRegistersAllInterfaces) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  auto* js = hw.get<hardware_interface::JointStateInterface>();
  ASSERT_NE(js, nullptr);
  EXPECT_NO_THROW(js->getHandle("joint_a1"));
  EXPECT_NO_THROW(js->getHandle("joint_a6"));
  auto* cs = hw.get<kuka_rsi::CartesianStateInterface>();
  ASSERT_NE(cs, nullptr);
  EXPECT_NO_THROW(cs->getHandle("kuka_tcp"));
  auto* cc = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>();
  ASSERT_NE(cc, nullptr);
  EXPECT_NO_THROW(cc->getHandle("kuka_tcp"));
}

TEST(RobotHW, ReadUpdatesStateFromFrame) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 123.5, 90.0, 1000));

  hw.read(ros::Time(), ros::Duration(0.004));
  EXPECT_TRUE(hw.connected());
  auto h = hw.get<kuka_rsi::CartesianStateInterface>()->getHandle("kuka_tcp");
  EXPECT_DOUBLE_EQ(h.getX(), 123.5);
  EXPECT_DOUBLE_EQ(h.getZ(), 800.0);
  auto j = hw.get<hardware_interface::JointStateInterface>()
               ->getHandle("joint_a1");
  EXPECT_NEAR(*j.getPosition(), 90.0 * kDegToRad, 1e-12);
}

TEST(RobotHW, WriteEchoesIpocAndClampsCommand) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 4242));
  hw.read(ros::Time(), ros::Duration(0.004));

  auto cmd = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()
                 ->getHandle("kuka_tcp");
  cmd.setCommand(9.0, 0, 0, 0, 0, 0);  // above 0.5 mm default limit
  hw.write(ros::Time(), ros::Duration(0.004));

  double x;
  int stop;
  std::uint64_t ipoc;
  ASSERT_TRUE(krc.receiveReply(x, stop, ipoc));
  EXPECT_DOUBLE_EQ(x, 0.5);  // secondary clamp engaged
  EXPECT_EQ(stop, 0);
  EXPECT_EQ(ipoc, 4242u);
  EXPECT_EQ(hw.saturationCount(), 1u);
}

TEST(RobotHW, CommandBufferClearedAfterWrite) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 1));
  hw.read(ros::Time(), ros::Duration(0.004));

  auto cmd = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()
                 ->getHandle("kuka_tcp");
  cmd.setCommand(0.1, 0, 0, 0, 0, 0);
  hw.write(ros::Time(), ros::Duration(0.004));
  double x;
  int stop;
  std::uint64_t ipoc;
  ASSERT_TRUE(krc.receiveReply(x, stop, ipoc));
  EXPECT_DOUBLE_EQ(x, 0.1);

  // Second cycle without a fresh setCommand: stale value must not repeat.
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.1, 0.0, 2));
  hw.read(ros::Time(), ros::Duration(0.004));
  hw.write(ros::Time(), ros::Duration(0.004));
  ASSERT_TRUE(krc.receiveReply(x, stop, ipoc));
  EXPECT_DOUBLE_EQ(x, 0.0);
}

TEST(RobotHW, TimeoutsLatchFaultAndForceZeroWithStop) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig(2)));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 10));
  hw.read(ros::Time(), ros::Duration(0.004));
  ASSERT_TRUE(hw.connected());

  hw.read(ros::Time(), ros::Duration(0.004));  // timeout 1
  hw.read(ros::Time(), ros::Duration(0.004));  // timeout 2 -> fault
  EXPECT_TRUE(hw.faulted());

  auto cmd = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()
                 ->getHandle("kuka_tcp");
  cmd.setCommand(0.3, 0, 0, 0, 0, 0);
  hw.write(ros::Time(), ros::Duration(0.004));
  double x;
  int stop;
  std::uint64_t ipoc;
  ASSERT_TRUE(krc.receiveReply(x, stop, ipoc));
  EXPECT_DOUBLE_EQ(x, 0.0);  // faulted: zero correction
  EXPECT_EQ(stop, 1);        // and Stop requested

  hw.resetFault();
  EXPECT_FALSE(hw.faulted());
}

TEST(RobotHW, BadFramesCountTowardFault) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig(2)));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 10));
  hw.read(ros::Time(), ros::Duration(0.004));

  ASSERT_TRUE(krc.sendGarbage(hw.listenPort()));
  hw.read(ros::Time(), ros::Duration(0.004));
  ASSERT_TRUE(krc.sendGarbage(hw.listenPort()));
  hw.read(ros::Time(), ros::Duration(0.004));
  EXPECT_TRUE(hw.faulted());
  EXPECT_EQ(hw.sessionStats().bad_frames, 2u);
}

TEST(RobotHW, WriteBeforeAnyFrameIsSilentNoop) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  // No frame ever received: nothing to reply to; must not crash.
  hw.write(ros::Time(), ros::Duration(0.004));
  EXPECT_FALSE(hw.connected());
}
```

- [ ] **Step 2: 加构建条目并确认失败**

`CMakeLists.txt`:新增 hw 库(依赖 roscpp 头,catkin 库):

```cmake
add_library(kuka_rsi_robot_hw
  src/kuka_rsi_robot_hw.cpp
)
target_link_libraries(kuka_rsi_robot_hw kuka_rsi_protocol ${catkin_LIBRARIES})
```

`catkin_package` 的 `LIBRARIES` 行改为 `LIBRARIES kuka_rsi_protocol kuka_rsi_robot_hw`。测试段追加:

```cmake
  catkin_add_gtest(test_kuka_rsi_robot_hw test/test_kuka_rsi_robot_hw.cpp)
  target_link_libraries(test_kuka_rsi_robot_hw kuka_rsi_robot_hw
                        ${GTEST_MAIN_LIBRARIES})
```

运行:`cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
预期:BUILD FAILS,`kuka_rsi_robot_hw.h: No such file or directory`。

- [ ] **Step 3: 写最小实现**

`include/kuka_rsi_hw_interface/kuka_rsi_robot_hw.h`:

```cpp
#pragma once

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <cstdint>
#include <string>

#include "kuka_rsi_hw_interface/cartesian_command_interface.h"
#include "kuka_rsi_hw_interface/command_limiter.h"
#include "kuka_rsi_hw_interface/rsi_frame.h"
#include "kuka_rsi_hw_interface/rsi_session_monitor.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

namespace kuka_rsi {

struct HwConfig {
  std::string listen_ip{"0.0.0.0"};
  std::uint16_t listen_port{49152};
  int read_timeout_ms{8};  // 2 RSI cycles
  unsigned max_consecutive_timeouts{5};
  CommandLimits limits;
};

// ros_control hardware interface for the KUKA RSI channel (spec 5.1, 12.2).
// The KRC is the UDP client: read() waits (bounded) for its state frame,
// write() answers with the RKorr correction echoing the received IPOC.
// Owns no force-control logic. configure() is the ROS-master-free entry
// used by tests and by init().
class KukaRsiRobotHW : public hardware_interface::RobotHW {
 public:
  KukaRsiRobotHW();

  bool configure(const HwConfig& cfg);
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  bool connected() const { return monitor_.connected(); }
  bool faulted() const { return monitor_.faulted(); }
  const SessionStats& sessionStats() const { return monitor_.stats(); }
  std::uint64_t saturationCount() const { return saturation_count_; }
  void resetFault() { monitor_.reset(); }
  std::uint16_t listenPort() const { return udp_.boundPort(); }

 private:
  void registerInterfaces();

  HwConfig cfg_;
  UdpTransport udp_;
  RsiSessionMonitor monitor_;
  CommandLimiter limiter_;

  // State buffers (written by read(), exposed through interfaces).
  double cart_pos_[6] = {0, 0, 0, 0, 0, 0};      // x y z a b c [mm/deg]
  double joint_pos_[6] = {0, 0, 0, 0, 0, 0};     // rad
  double joint_vel_[6] = {0, 0, 0, 0, 0, 0};     // always 0 (RSI: no velocity)
  double joint_eff_[6] = {0, 0, 0, 0, 0, 0};     // always 0
  // Command buffer (written by controllers, consumed + cleared by write()).
  double cart_cmd_[6] = {0, 0, 0, 0, 0, 0};

  std::uint64_t last_ipoc_{0};
  std::uint64_t watchdog_{0};
  std::uint64_t saturation_count_{0};
  char rx_buf_[1024];
  char tx_buf_[1024];

  hardware_interface::JointStateInterface joint_state_interface_;
  CartesianStateInterface cartesian_state_interface_;
  CartesianCorrectionCommandInterface correction_command_interface_;
};

}  // namespace kuka_rsi
```

`src/kuka_rsi_robot_hw.cpp`:

```cpp
#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"

#include <cmath>

namespace kuka_rsi {

namespace {
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

KukaRsiRobotHW::KukaRsiRobotHW() : monitor_(SessionConfig{}) {}

bool KukaRsiRobotHW::configure(const HwConfig& cfg) {
  cfg_ = cfg;
  SessionConfig sc;
  sc.max_consecutive_timeouts = cfg.max_consecutive_timeouts;
  monitor_ = RsiSessionMonitor(sc);
  if (!udp_.bind(cfg.listen_ip, cfg.listen_port)) return false;
  registerInterfaces();
  return true;
}

bool KukaRsiRobotHW::init(ros::NodeHandle& /*root_nh*/,
                          ros::NodeHandle& robot_hw_nh) {
  HwConfig cfg;
  robot_hw_nh.param<std::string>("listen_ip", cfg.listen_ip, cfg.listen_ip);
  int port = cfg.listen_port;
  robot_hw_nh.param("listen_port", port, port);
  cfg.listen_port = static_cast<std::uint16_t>(port);
  robot_hw_nh.param("read_timeout_ms", cfg.read_timeout_ms,
                    cfg.read_timeout_ms);
  int max_timeouts = static_cast<int>(cfg.max_consecutive_timeouts);
  robot_hw_nh.param("max_consecutive_timeouts", max_timeouts, max_timeouts);
  cfg.max_consecutive_timeouts = static_cast<unsigned>(max_timeouts);
  robot_hw_nh.param("max_correction_trans", cfg.limits.max_trans,
                    cfg.limits.max_trans);
  robot_hw_nh.param("max_correction_rot", cfg.limits.max_rot,
                    cfg.limits.max_rot);
  return configure(cfg);
}

void KukaRsiRobotHW::registerInterfaces() {
  static const char* const kJointNames[6] = {"joint_a1", "joint_a2",
                                             "joint_a3", "joint_a4",
                                             "joint_a5", "joint_a6"};
  for (int i = 0; i < 6; ++i) {
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
        kJointNames[i], &joint_pos_[i], &joint_vel_[i], &joint_eff_[i]));
  }
  registerInterface(&joint_state_interface_);

  CartesianStateHandle state("kuka_tcp", &cart_pos_[0], &cart_pos_[1],
                             &cart_pos_[2], &cart_pos_[3], &cart_pos_[4],
                             &cart_pos_[5]);
  cartesian_state_interface_.registerHandle(state);
  registerInterface(&cartesian_state_interface_);

  correction_command_interface_.registerHandle(CartesianCorrectionHandle(
      state, &cart_cmd_[0], &cart_cmd_[1], &cart_cmd_[2], &cart_cmd_[3],
      &cart_cmd_[4], &cart_cmd_[5]));
  registerInterface(&correction_command_interface_);
}

void KukaRsiRobotHW::read(const ros::Time& /*time*/,
                          const ros::Duration& /*period*/) {
  const int n = udp_.receive(rx_buf_, sizeof(rx_buf_) - 1, cfg_.read_timeout_ms);
  if (n <= 0) {
    monitor_.onTimeout();
    return;
  }
  rx_buf_[n] = '\0';
  RobFrame frame;
  if (!parseRobFrame(rx_buf_, static_cast<std::size_t>(n), frame)) {
    monitor_.onBadFrame();
    return;
  }
  monitor_.onFrame(frame);
  last_ipoc_ = frame.ipoc;
  cart_pos_[0] = frame.x;
  cart_pos_[1] = frame.y;
  cart_pos_[2] = frame.z;
  cart_pos_[3] = frame.a;
  cart_pos_[4] = frame.b;
  cart_pos_[5] = frame.c;
  for (int i = 0; i < 6; ++i) joint_pos_[i] = frame.axis_deg[i] * kDegToRad;
}

void KukaRsiRobotHW::write(const ros::Time& /*time*/,
                           const ros::Duration& /*period*/) {
  if (!monitor_.connected()) return;  // nobody to answer yet

  SenFrame out;
  out.ipoc = last_ipoc_;
  out.watchdog = ++watchdog_;
  if (monitor_.faulted()) {
    out.stop = 1;  // zero correction + stop request; fault is latched
  } else {
    if (limiter_.clamp(cart_cmd_, cfg_.limits)) ++saturation_count_;
    out.x = cart_cmd_[0];
    out.y = cart_cmd_[1];
    out.z = cart_cmd_[2];
    out.a = cart_cmd_[3];
    out.b = cart_cmd_[4];
    out.c = cart_cmd_[5];
  }
  // Corrections are per-cycle increments: consume-and-clear so a stale
  // command is never re-sent (spec 6.1 stale-command rule).
  for (double& v : cart_cmd_) v = 0.0;

  const std::size_t len = serializeSenFrame(out, tx_buf_, sizeof(tx_buf_));
  if (len == 0) return;
  udp_.sendToLastSender(tx_buf_, len);
}

}  // namespace kuka_rsi
```

- [ ] **Step 4: 运行确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_rsi_hw_interface/test_kuka_rsi_robot_hw
```

预期:`[  PASSED  ] 7 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "feat(rsi): add KukaRsiRobotHW read/write with fault zeroing (Task 8)"
```

---

### Task 9: 离线集成测试(`KukaRsiRobotHW` × `RsiMockServer` 全链路闭环)

**Files:**
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_integration.cpp`

**Interfaces:**
- Consumes: Task 5/7/8 全部产物。无新生产接口——本 Task 是纯测试任务,验证两端(PC 侧真实实现 × KRC 侧 mock)在 loopback 上互相驱动:IPOC 回显闭环、修正积分、mock 位姿注入反映到状态接口、故障零输出被 mock 观测到。
- TDD 说明:本 Task 的"失败态"是新测试文件先于 CMake 条目存在性——按惯例先写测试 + CMake,首次构建**预期直接通过**不成立时才修改实现;若全部直接通过,说明 Task 2-8 组合正确,这正是集成测试的验收含义(计划自查:此处不违反 TDD,因为不引入新实现代码)。

- [ ] **Step 1: 写集成测试**

`ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_integration.cpp`:

```cpp
#include <gtest/gtest.h>

#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"
#include "kuka_rsi_hw_interface/rsi_mock_server.h"

using kuka_rsi::HwConfig;
using kuka_rsi::KukaRsiRobotHW;
using kuka_rsi::MockConfig;
using kuka_rsi::MockStats;
using kuka_rsi::RsiMockServer;

namespace {

HwConfig hwConfig() {
  HwConfig cfg;
  cfg.listen_ip = "127.0.0.1";
  cfg.listen_port = 0;
  cfg.read_timeout_ms = 100;  // generous: mock thread scheduling jitter
  cfg.max_consecutive_timeouts = 5;
  return cfg;
}

// One PC-side control cycle: read state, apply a command, write reply.
void spinCycle(KukaRsiRobotHW& hw, double cmd_x, double cmd_a = 0.0) {
  hw.read(ros::Time(), ros::Duration(0.004));
  auto h = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()
               ->getHandle("kuka_tcp");
  h.setCommand(cmd_x, 0, 0, cmd_a, 0, 0);
  hw.write(ros::Time(), ros::Duration(0.004));
}

}  // namespace

TEST(RsiIntegration, ClosedLoopEchoAndIntegration) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));

  MockConfig mock_cfg;
  mock_cfg.x0 = 500.0;
  RsiMockServer mock(mock_cfg, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  // 50 cycles of +0.2 mm X corrections.
  for (int i = 0; i < 50; ++i) spinCycle(hw, 0.2);
  mock.stop();

  const MockStats s = mock.statsSnapshot();
  EXPECT_EQ(s.ipoc_echo_errors, 0u);   // every reply echoed the right IPOC
  EXPECT_EQ(s.parse_errors, 0u);       // every reply was well-formed
  EXPECT_GE(s.replies_received, 50u);
  EXPECT_TRUE(hw.connected());
  EXPECT_FALSE(hw.faulted());
  EXPECT_EQ(hw.sessionStats().ipoc_jumps, 0u);

  // The mock integrated our corrections: pose moved by ~50 * 0.2 mm.
  // (>= 45 tolerates a few cycles lost to thread startup/shutdown.)
  EXPECT_GE(mock.poseX(), 500.0 + 45 * 0.2);
}

TEST(RsiIntegration, StateInterfaceTracksInjectedPose) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  RsiMockServer mock(MockConfig{}, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  spinCycle(hw, 0.0);  // connect
  mock.setPose(111.0, 222.0, 333.0, 10.0, 20.0, 30.0);
  // A few cycles so a post-injection frame is definitely consumed.
  for (int i = 0; i < 5; ++i) spinCycle(hw, 0.0);
  mock.stop();

  auto h = hw.get<kuka_rsi::CartesianStateInterface>()->getHandle("kuka_tcp");
  EXPECT_DOUBLE_EQ(h.getX(), 111.0);
  EXPECT_DOUBLE_EQ(h.getY(), 222.0);
  EXPECT_DOUBLE_EQ(h.getA(), 10.0);
  EXPECT_DOUBLE_EQ(h.getC(), 30.0);
}

TEST(RsiIntegration, FaultZeroOutputObservedByMock) {
  HwConfig cfg = hwConfig();
  cfg.read_timeout_ms = 10;
  cfg.max_consecutive_timeouts = 2;
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(cfg));

  RsiMockServer mock(MockConfig{}, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());
  spinCycle(hw, 0.2);  // healthy cycle
  mock.stop();         // KRC dies

  hw.read(ros::Time(), ros::Duration(0.004));  // timeout 1
  hw.read(ros::Time(), ros::Duration(0.004));  // timeout 2 -> latched fault
  ASSERT_TRUE(hw.faulted());

  // KRC comes back, but the PC fault is latched: replies must be zero+Stop.
  RsiMockServer mock2(MockConfig{}, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock2.start());
  for (int i = 0; i < 5; ++i) spinCycle(hw, 0.2);
  mock2.stop();

  const MockStats s = mock2.statsSnapshot();
  EXPECT_EQ(s.last_stop, 1);           // Stop requested
  EXPECT_DOUBLE_EQ(mock2.poseX(), 0.0);  // zero correction: pose unmoved
  EXPECT_TRUE(hw.faulted());           // still latched

  hw.resetFault();
  EXPECT_FALSE(hw.faulted());
}

TEST(RsiIntegration, RotationCorrectionsIntegrateOnMock) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  RsiMockServer mock(MockConfig{}, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  for (int i = 0; i < 20; ++i) spinCycle(hw, 0.0, 0.05);  // A axis, at limit
  mock.stop();

  EXPECT_EQ(mock.statsSnapshot().ipoc_echo_errors, 0u);
  // 20 cycles * 0.05 deg = 1.0 deg total on A (tolerate a few lost cycles).
  auto h = hw.get<kuka_rsi::CartesianStateInterface>()->getHandle("kuka_tcp");
  EXPECT_GE(h.getA(), 0.7);
  EXPECT_LE(h.getA(), 1.0 + 1e-9);
}
```

- [ ] **Step 2: 加测试目标**

`CMakeLists.txt` 测试段追加:

```cmake
  catkin_add_gtest(test_rsi_integration test/test_rsi_integration.cpp)
  target_link_libraries(test_rsi_integration kuka_rsi_robot_hw
                        kuka_rsi_hw_interface_mock ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 3: 构建并运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_rsi_hw_interface/test_rsi_integration
```

预期:`[  PASSED  ] 4 tests.`(若失败,修 Task 2-8 的实现直至通过,不修测试语义。)

- [ ] **Step 4: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "test(rsi): add offline closed-loop integration tests (Task 9)"
```

---

### Task 10: `kuka_rsi_sim_server` 可执行 + ROS 节点壳 + config/launch

**Files:**
- Create: `ros_ws/src/kuka_rsi_hw_interface/src/kuka_rsi_sim_server_main.cpp`
- Create: `ros_ws/src/kuka_rsi_hw_interface/src/kuka_rsi_hw_interface_node.cpp`
- Create: `ros_ws/src/kuka_rsi_hw_interface/config/kuka_rsi.yaml`
- Create: `ros_ws/src/kuka_rsi_hw_interface/launch/kuka_rsi.launch`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 7/8 库、`controller_manager::ControllerManager`、`soft_robot_msgs/RsiState`(Task 1)。
- Produces:
  - 可执行 `kuka_rsi_sim_server`(规格 §15.2 名称):命令行 mock 对端,参数 `--target-ip --target-port --cycle-ms --reply-timeout-ms --x0/--y0/--z0/--a0/--b0/--c0`,周期打印统计;供 Plan 3-5 手动联调与 bringup 冒烟。
  - 可执行 `kuka_rsi_hw_interface_node`:`configure` → 4 ms 循环 `read → cm.update → write`,50 Hz 发布 `/kuka/rsi/state`(`soft_robot_msgs/RsiState`)。
  - `config/kuka_rsi.yaml` + `launch/kuka_rsi.launch`(参数迁移:规格 §14 `kuka_rsi.yaml` 中属 RSI 的键)。
- 本 Task 无 gtest(纯壳,行为已被 Task 2-9 覆盖);验证 = 零警告构建 + 两个可执行的 loopback 冒烟(冒烟需 roscore,标注为手动步骤,不进验收门槛)。

- [ ] **Step 1: 写 sim server main**

`src/kuka_rsi_sim_server_main.cpp`:

```cpp
// Standalone KRC-side RSI mock (spec section 15.2). Sends state frames at
// the configured cycle to the PC-side hardware interface and validates the
// returned RKorr/IPOC echo. Test/commissioning tool: not realtime code.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "kuka_rsi_hw_interface/rsi_mock_core.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

namespace {

struct Options {
  std::string target_ip{"127.0.0.1"};
  int target_port{49152};
  int cycle_ms{4};
  int reply_timeout_ms{4};
  kuka_rsi::MockConfig mock;
};

bool parseArgs(int argc, char** argv, Options& opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_value = i + 1 < argc;
    if (arg == "--help") return false;
    if (!has_value) return false;
    const char* v = argv[++i];
    if (arg == "--target-ip") opt.target_ip = v;
    else if (arg == "--target-port") opt.target_port = std::atoi(v);
    else if (arg == "--cycle-ms") opt.cycle_ms = std::atoi(v);
    else if (arg == "--reply-timeout-ms") opt.reply_timeout_ms = std::atoi(v);
    else if (arg == "--x0") opt.mock.x0 = std::atof(v);
    else if (arg == "--y0") opt.mock.y0 = std::atof(v);
    else if (arg == "--z0") opt.mock.z0 = std::atof(v);
    else if (arg == "--a0") opt.mock.a0 = std::atof(v);
    else if (arg == "--b0") opt.mock.b0 = std::atof(v);
    else if (arg == "--c0") opt.mock.c0 = std::atof(v);
    else return false;
  }
  return opt.target_port > 0 && opt.cycle_ms > 0 && opt.reply_timeout_ms > 0;
}

void printUsage() {
  std::printf(
      "Usage: kuka_rsi_sim_server [--target-ip IP] [--target-port PORT]\n"
      "  [--cycle-ms N] [--reply-timeout-ms N]\n"
      "  [--x0 V] [--y0 V] [--z0 V] [--a0 V] [--b0 V] [--c0 V]\n");
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parseArgs(argc, argv, opt)) {
    printUsage();
    return 1;
  }

  kuka_rsi::RsiMockCore core(opt.mock);
  kuka_rsi::UdpTransport udp;
  if (!udp.bind("0.0.0.0", 0)) {
    std::fprintf(stderr, "kuka_rsi_sim_server: failed to bind UDP socket\n");
    return 1;
  }
  std::printf("kuka_rsi_sim_server: -> %s:%d, cycle %d ms\n",
              opt.target_ip.c_str(), opt.target_port, opt.cycle_ms);

  char tx[1024];
  char rx[1024];
  std::uint64_t cycle = 0;
  for (;;) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(opt.cycle_ms);
    const std::size_t n = core.buildStateFrame(tx, sizeof(tx));
    if (n == 0 ||
        !udp.sendTo(opt.target_ip,
                    static_cast<std::uint16_t>(opt.target_port), tx, n)) {
      std::fprintf(stderr, "kuka_rsi_sim_server: send failed\n");
      return 1;
    }
    const int r = udp.receive(rx, sizeof(rx), opt.reply_timeout_ms);
    if (r > 0) {
      core.applyReply(rx, static_cast<std::size_t>(r));
    } else {
      core.noteReplyTimeout();
    }
    if (++cycle % 250 == 0) {  // once a second at 4 ms
      const kuka_rsi::MockStats& s = core.stats();
      std::printf(
          "sent=%llu ok=%llu tmo=%llu echo_err=%llu parse_err=%llu "
          "stop=%d pose x=%.3f y=%.3f z=%.3f a=%.3f b=%.3f c=%.3f\n",
          static_cast<unsigned long long>(s.frames_sent),
          static_cast<unsigned long long>(s.replies_received),
          static_cast<unsigned long long>(s.reply_timeouts),
          static_cast<unsigned long long>(s.ipoc_echo_errors),
          static_cast<unsigned long long>(s.parse_errors), s.last_stop,
          core.x(), core.y(), core.z(), core.a(), core.b(), core.c());
      std::fflush(stdout);
    }
    std::this_thread::sleep_until(deadline);
  }
  return 0;
}
```

- [ ] **Step 2: 写 ROS 节点壳**

`src/kuka_rsi_hw_interface_node.cpp`:

```cpp
// ROS node shell: runs KukaRsiRobotHW under controller_manager at the RSI
// cycle and publishes link diagnostics. All protocol behavior is covered by
// the offline gtests; this file only wires the pieces to ROS.
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <soft_robot_msgs/RsiState.h>

#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kuka_rsi_hw_interface");
  ros::NodeHandle root_nh;
  ros::NodeHandle robot_hw_nh("~");

  kuka_rsi::KukaRsiRobotHW hw;
  if (!hw.init(root_nh, robot_hw_nh)) {
    ROS_FATAL("kuka_rsi_hw_interface: init failed (UDP bind?)");
    return 1;
  }
  ROS_INFO("kuka_rsi_hw_interface: listening on UDP port %u", hw.listenPort());

  controller_manager::ControllerManager cm(&hw, root_nh);
  ros::Publisher state_pub =
      root_nh.advertise<soft_robot_msgs::RsiState>("kuka/rsi/state", 10);

  ros::AsyncSpinner spinner(1);  // services/topics off the control thread
  spinner.start();

  // The loop is paced by the KRC: read() blocks (bounded) on its frame.
  ros::Time last = ros::Time::now();
  ros::Time last_pub = last;
  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    const ros::Duration period = now - last;
    last = now;

    hw.read(now, period);
    cm.update(now, period);
    hw.write(now, period);

    if ((now - last_pub).toSec() >= 0.02) {  // 50 Hz diagnostics
      last_pub = now;
      const kuka_rsi::SessionStats& s = hw.sessionStats();
      soft_robot_msgs::RsiState msg;
      msg.header.stamp = now;
      msg.connected = s.connected;
      msg.fault = s.fault;
      msg.ipoc = s.last_ipoc;
      msg.total_timeouts = s.total_timeouts;
      msg.consecutive_timeouts = s.consecutive_timeouts;
      msg.bad_frames = s.bad_frames;
      msg.ipoc_jumps = s.ipoc_jumps;
      msg.saturation_count = hw.saturationCount();
      state_pub.publish(msg);
    }
  }
  return 0;
}
```

- [ ] **Step 3: 写 config 与 launch**

`config/kuka_rsi.yaml`:

```yaml
# RSI network settings (spec section 14, kuka_rsi.yaml migration keys).
# EKI settings live in kuka_eki_bridge (Plan 4).
listen_ip: "0.0.0.0"          # PC-side bind address (legacy RobIP peer)
listen_port: 49152            # RSI UDP port; must match the KRC RSI config
read_timeout_ms: 8            # 2 RSI cycles (IPOC timeout)
max_consecutive_timeouts: 5   # missed cycles before latched fault
max_correction_trans: 0.5     # secondary RKorr clamp [mm/cycle]
max_correction_rot: 0.05      # secondary RKorr clamp [deg/cycle]
```

`launch/kuka_rsi.launch`:

```xml
<launch>
  <node name="kuka_rsi_hw_interface" pkg="kuka_rsi_hw_interface"
        type="kuka_rsi_hw_interface_node" output="screen">
    <rosparam command="load"
              file="$(find kuka_rsi_hw_interface)/config/kuka_rsi.yaml" />
  </node>
</launch>
```

- [ ] **Step 4: 加构建条目并零警告构建**

`CMakeLists.txt` 追加(库目标之后):

```cmake
add_executable(kuka_rsi_sim_server src/kuka_rsi_sim_server_main.cpp)
target_link_libraries(kuka_rsi_sim_server kuka_rsi_hw_interface_mock)

add_executable(kuka_rsi_hw_interface_node src/kuka_rsi_hw_interface_node.cpp)
add_dependencies(kuka_rsi_hw_interface_node
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(kuka_rsi_hw_interface_node kuka_rsi_robot_hw
                      ${catkin_LIBRARIES})

install(TARGETS kuka_rsi_sim_server kuka_rsi_hw_interface_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make 2>&1 | grep -E "warning|error"; \
  catkin_make tests
```

预期:无 warning/error 输出;tests 构建通过。

- [ ] **Step 5: 手动冒烟(需 roscore,不进验收门槛)**

```bash
# terminal 1
roscore
# terminal 2
cd /home/ljj/kuka_iiqka_ros/ros_ws && source devel/setup.bash && \
  roslaunch kuka_rsi_hw_interface kuka_rsi.launch
# terminal 3
cd /home/ljj/kuka_iiqka_ros/ros_ws && source devel/setup.bash && \
  ./devel/lib/kuka_rsi_hw_interface/kuka_rsi_sim_server \
    --target-ip 127.0.0.1 --target-port 49152
# terminal 4: expect connected=True, fault=False, ipoc increasing
rostopic echo -n 3 /kuka/rsi/state
```

预期:sim server 每秒打印 `sent≈ok`、`echo_err=0`;`/kuka/rsi/state` 显示 `connected: True`。

- [ ] **Step 6: 全套件回归 + Commit**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make run_tests && \
  catkin_test_results build/test_results
```

预期:`Summary: ... 0 errors, 0 failures`(Plan 1 的 52 例 + 本计划全部用例)。

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_rsi_hw_interface && \
git commit -m "feat(rsi): add sim server executable, node shell, config (Task 10)"
```

---

## 验收清单

| Task | 测试二进制 | 用例数 |
|---|---|---|
| 1 | `soft_robot_msgs/test_msgs` | 4 |
| 2 | `kuka_rsi_hw_interface/test_rsi_frame_parse` | 7 |
| 3 | `kuka_rsi_hw_interface/test_rsi_frame_serialize` | 5 |
| 4 | `kuka_rsi_hw_interface/test_rsi_session_monitor` | 8 |
| 4 | `kuka_rsi_hw_interface/test_command_limiter` | 5 |
| 5 | `kuka_rsi_hw_interface/test_udp_transport` | 6 |
| 6 | `kuka_rsi_hw_interface/test_cartesian_interfaces` | 5 |
| 7 | `kuka_rsi_hw_interface/test_rsi_mock` | 7 |
| 8 | `kuka_rsi_hw_interface/test_kuka_rsi_robot_hw` | 7 |
| 9 | `kuka_rsi_hw_interface/test_rsi_integration` | 4 |
| **合计** | **10 个二进制** | **58** |

验收条件:

- `catkin_make`、`catkin_make tests` 零警告;`catkin_make run_tests` 0 errors / 0 failures(含 Plan 1 存量 52 例,总计 110)。
- 全部测试可离线运行(仅 127.0.0.1,无 roscore、无真机)。
- `kuka_rsi_sim_server --help` 可用;Task 10 Step 5 冒烟按文档可复现(手动,非门槛)。
- 规格对照:§5.1 四个注册接口中前三个(JointState/CartesianState/CartesianCorrectionCommand)已交付,`RobotModeStateInterface` 为规格标注 Optional,其内容以 `/kuka/rsi/state` 话题 + Plan 4 的 EKI 状态覆盖;§6.1 IPOC 回显/零输出/丢帧策略、§12.2 全部六条、§15.2 `kuka_rsi_sim_server` 已覆盖(延迟/丢包/畸形 XML/IPOC 跳变注入以 FakeKrc + mock 统计实现)。

## 遗留风险

1. **`Sen Type="ROS"` 与 KRC 侧配置耦合**:KRC 的 RSI XML 配置(Plan 5 交付)必须声明相同的 Sen Type 与字段集(RKorr/Stop/Watchdog/IPOC),否则 KRC 丢弃回复帧。真机联调前无法闭环验证;`kuka_rsi_sim_server` 仅验证 PC 侧自洽。
2. **tinyxml2 解析的实时性偏差**:read() 内 `XMLDocument::Parse` 有内部堆分配(决策 7)。4 ms 周期在普通 Linux + 千字节帧下经验上充裕(ros-industrial 同方案),但硬实时不达标;若联调发现超限,备选方案为手写无分配解析器(接口不变,仅替换 `parseRobFrame` 实现)。
3. **节点壳循环由 KRC 节拍驱动**:`read()` 的 poll 超时即周期上界,PC 侧无独立定时器;KRC 帧率异常(过快)时 cm.update 频率跟随。规格允许(RSI 语义即 KRC 主导节拍),但 Plan 3 控制器的 dt 必须用实测 `period` 而非常数 0.004。
4. **故障恢复链路未闭环**:`resetFault()` 已提供但无调用方(EKI/manager 属 Plan 4/5);在此之前锁存故障只能重启节点解除。
5. **IPOC 步长不校验**(决策 5):若 KRC 以非 4 ms 上下文运行,PC 侧不会告警;`ipoc_jumps` 仅捕获非递增。联调时应人工核对 `/kuka/rsi/state` 的 ipoc 增速。
6. **mock 无运动学**:`AIPos` 恒零,Plan 3 的 JointState 相关集成测试若需非零关节角,需扩展 `RsiMockCore::setJointAngles`(小改动,已预留 setter 风格)。
7. **多网卡环境**:`listen_ip: 0.0.0.0` + `sendToLastSender` 在多网卡/防火墙环境可能回包走错接口;bringup(Plan 5)应把 `listen_ip` 固定为与 KRC 同网段的网卡地址。

