# KUKA iiQKA 软体力顺应机器人系统

基于 **ROS 1 Noetic** 的 KUKA KR C5 / iiQKA.OS2 力顺应控制系统。通过 RSI 实时通道做笛卡尔修正伺服，通过 EKI 管理通道做状态/工具协商，融合 SRI 六维力/力矩传感器实现导纳（admittance）力顺应、拖动示教、姿态目标运动与负载自动标定。

> 部署到真实机器人前，请通读本文档「真实机器人部署」一节，并**严格逐阶段**执行 `docs/commissioning_checklist.md`。零输出阶段（Stage 2、5）任何时候都不能跳过。

---

## 系统架构

系统分为两条与机器人通信的物理通道，以及一条力传感通道：

| 通道 | 协议 | 端口 | 方向 | 用途 |
|------|------|------|------|------|
| **RSI** 实时通道 | UDP / XML | 49152（ROS 监听） | KRC → ROS 客户端 | 4ms 周期笛卡尔修正伺服（RKorr） |
| **EKI** 管理通道 | TCP / XML | 54600（ROS 监听） | KRC 作为 client 连入 | 状态查询、工具读取、RSI 启停、复位。**绝不用于逐周期控制** |
| **SRI** 力传感 | TCP 二进制 | 传感器盒作为 server | SRI → ROS | 250Hz 六维力/力矩流 |

数据流概览：

```
                 EKI (TCP 54600, ~100ms 心跳/命令)
   KUKA KR C5  ◄──────────────────────────────►  kuka_eki_bridge
   iiQKA.OS2       RSI (UDP 49152, 4ms 周期)          │
      │        ◄──────────────────────────────►  kuka_rsi_hw_interface ──┐
      │                                                                  │ ros_control
      │                                              soft_robot_controllers│
   SRI F/T盒 ──── TCP 250Hz ──► sri_force_torque_driver ──► WrenchStamped ┘
                                                              │
                              soft_force_control_manager (状态机 / 健康聚合 / 标定)
                                                              │
                              /soft_robot/* 服务 + 动作  ◄──── web UI (rosbridge)
```

### ROS 包一览

| 包 | 职责 |
|----|------|
| `kuka_rsi_hw_interface` | RSI 实时通道的 ros_control 硬件接口，UDP/XML 通信，暴露笛卡尔状态与 RKorr 修正命令接口。内含 KRC 端 RSI mock（`kuka_rsi_sim_server`） |
| `kuka_eki_bridge` | EKI 管理通道桥接：XML 命令/状态编解码、TCP 流拆包、seq/ack 会话、`/kuka/eki/*` 服务。内含 KRC 端 mock |
| `sri_force_torque_driver` | SRI 六维力传感器以太网驱动：M8128 二进制流解析、tare 归零、可选低通、断线重连，发布 `WrenchStamped`。内含传感器 mock |
| `soft_force_control_core` | 纯 C++ 力顺应算法库，无 ROS 依赖，复刻遗留 SoftControl/LoadMass 行为，可离线单测 |
| `soft_robot_controllers` | ros_control 控制器插件：`ForceComplianceController`（导纳管线）与 `CartesianCorrectionController`（流透传 + RKorr 目标模式，内嵌 `OrientationMotionCore`） |
| `soft_force_control_manager` | 非实时系统管理器：状态机、EKI/RSI/SRI 健康聚合、控制器切换编排、负载标定工作流、`payload.yaml` 持久化、面向操作员的服务/动作接口 |
| `soft_robot_bringup` | Launch 文件、配置加载、冒烟测试文档：真实机器人与三-mock 仿真两个入口 |
| `soft_robot_msgs` | 共享消息/服务/动作定义。单位约定：mm、deg（KUKA A/B/C = Z-Y-X 欧拉）、N、Nm |
| `soft_robot_web_interface` | 浏览器操作界面（静态 ES module + rosbridge），所有危险操作都走 manager 服务 |
| `kuka_mujoco_sim` | MuJoCo 物理验证环境，冒充 RSI/SRI 线协议，用于离线验证力顺应，不改动任何现有包 |

---

## 环境要求

**ROS 侧**
- Ubuntu 20.04 + ROS Noetic
- catkin 构建工具

**KUKA 侧（真机部署最低版本，见 spec section 6）**
- KR C5 控制器，iiQKA.OS2 ≥ 9.2
- iiQKA.RobotSensorInterface ≥ 6.2
- iiQKA.EthernetKRL ≥ 6.1

**网络**
- KRC 与 ROS 主机同一子网
- RSI UDP 49152（`kuka_rsi.yaml`），EKI TCP 54600（`kuka_eki.yaml`），SRI 盒 TCP（默认 `192.168.1.1`）

---

## 构建

```bash
cd ros_ws
catkin_make
source devel/setup.bash

# launch 文件静态检查
rosrun roslaunch roslaunch-check $(rospack find soft_robot_bringup)/launch/soft_robot.launch
```

---

## 快速验证（无硬件）

在接触真机前，先在仿真里跑通整条闭环。`sim.launch` 用三个本地 mock（RSI / EKI / SRI）替代硬件：

```bash
roslaunch soft_robot_bringup sim.launch

# manager 到达 READY（工具已同步、控制器已加载、SRI 在流）
rostopic echo -n 1 /soft_robot/manager_state
#   system_state: 2, tool_synced: True, sri_streaming: True

# 启动力顺应伺服（DRAG 拖动模式）
rosservice call /soft_robot/start_servo "{mode: 2, profile: 0}"

# 停止
rosservice call /soft_robot/stop_servo
```

更完整的仿真冒烟流程（含 fault/reset、标定、泄漏检查）见 `ros_ws/src/soft_robot_bringup/README.md` 第 3 节。若需物理级验证（真实接触力、导纳收敛），使用 `kuka_mujoco_sim`。

---

## 真实机器人部署

真机部署分为两大块：**KUKA 侧安装配置**，然后 **ROS 侧联调**。全程机器人置于 **T1（手动降速）**，急停在手边。

### 阶段 A — KUKA 侧安装（先做，且必须签核）

模板文件在 `kuka/` 目录下，它们是**文本交付物，无法在 ROS 主机上执行或单测**，因此 `docs/kuka_iiqka_rsi_eki_setup.md` 的逐字段清单才是验收凭证。请对照本地 `ref/` 手册在真实机器人上逐项 `- [ ]` 核对。

交付文件：

| 文件 | 作用 |
|------|------|
| `kuka/krl/ROS_RSI_SERVO.SRC` / `.DAT` | KRL 主程序：选工具/基座、开 EKI 通道、100ms 心跳主循环、RSI 启停 |
| `kuka/eki/ROS_EKI_CONFIG.xml` | EKI 通道 XML 配置（命令/状态 schema） |
| `kuka/rsi/ROS_RSI_ETHERNET.xml` | RSI 以太网配置 |
| `kuka/rsi/ROS_RSI_CONTEXT.notes.md` | RSI Context 信号流图搭建说明 |

Schema 权威来源（**不要**拿别处去「修正」模板）：
- EKI 通道：`kuka_eki_bridge/include/kuka_eki_bridge/eki_frame.h`
- RSI 帧：`kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/rsi_frame.h`
- 端口：`kuka_eki_bridge/config/kuka_eki.yaml`（EKI 54600）、`kuka_rsi_hw_interface/config/kuka_rsi.yaml`（RSI 49152）

**A1. KRL 语法核对（最先做，对照 `ref/` 手册）**
- 每个 `EKI_*` 调用签名匹配 EthernetKRL 6.1 手册（Load/Open/CheckBuffer/ReadNext/GetInt/GetReal/SetInt/SetReal/Send/Close/Unload）
- 每个 `RSI_*` 调用匹配 RobotSensorInterface 6.2 手册（LOAD/ACTIVATE/PROCESS_ON/MOVECORR/PROCESS_OFF/DEACTIVATE/UNLOAD），含 PROCESS_ON 相对模式参数
- `$TIMER` 与心跳定时器号 `ROS_HB_TIMER_NO` 在本 cell 空闲可用
- 有语法不符只改 KRL，**不改 schema**

**A2. EkiConfig 逐字段核对**（对照 `eki_frame.h`）— XML 路径、属性名（**大小写敏感**）、类型逐行确认；Action 码 0..6，Error 码 0/1/2；EXTERNAL TYPE=Client，INTERNAL PORT=54600/TCP；心跳 Ack.Seq=0 每 100ms。

**A3. RSI Context 搭建与核对**（对照 `rsi_frame.h`）— 在示教盘 RSI Visual 里搭有向信号流图：
- **SEND（KRC→ROS）**：RIst、AIPos、Delay、Mode、IPOC
- **RECEIVE（ROS→KRC）**：RKorr(X..C) 经 CLAMP 限幅 → POSCORR，Stop(S)、Watchdog(W)、IPOC 回显
- SENTYPE="ROS"，PORT=49152，填 ROS 主机 IP
- POSCORR：RELATIVE 模式、BASE 坐标系、设好限幅
- 详细搭建步骤见 `docs/rsi.md`，限幅建议见 `kuka/rsi/ROS_RSI_CONTEXT.notes.md`

⚠️ Watchdog 义务：ROS 必须每帧反转 W 位；心跳/W 停止超过约 20ms，KRC 判通信断开并停机。

### 阶段 B — ROS 侧联调

一条命令拉起真机侧全部节点：

```bash
roslaunch soft_robot_bringup soft_robot.launch \
    kuka_ip:=<KRC 的 IP> \
    sensor_ip:=<SRI 盒的 IP>
```

（默认 `kuka_ip:=192.168.1.10`、`sensor_ip:=192.168.1.1`）

启动后在示教盘上选中并运行 `ROS_RSI_SERVO`，然后**逐阶段**走 `docs/commissioning_checklist.md`：

| 阶段 | 内容 | 关键点 |
|------|------|--------|
| **Stage 0** | 协议现实核对（无运动） | SRI 帧头/字节序/校验和；三条链路都 fresh、干净 |
| **Stage 1** | EKI 握手 + 工具查询 | manager 自动到 READY(2)，`get_tool` 与示教盘 `$TOOL` 一致 |
| **Stage 2** | ⚠️ **RSI 零输出软泡（10 分钟）** | RSI 激活但无控制器，ROS 回全零 RKorr；机器人**绝对不动**；`ipoc_jumps: 0` |
| **Stage 3** | 小幅 DIRECT_CARTESIAN 流点动 | 单轴逐个测；停止发布后 0.1s 内停 |
| **Stage 4** | 目标姿态运动 | 保守增益（p_gain 1.0 / 5 deg/s），收敛且带超时保护 |
| **Stage 5** | ⚠️ **仅显示 SRI 力（无伺服）** | 手推工具，力方向/量级合理；松手回零 |
| **Stage 6** | PRECISION 低增益力顺应 | 30N 死区下不动，超过缓慢屈服 |
| **Stage 7** | DRAG 自适应死区拖动 | 起始 2s ramp 窗口内不动，之后自然拖动 |
| **Stage 8** | 负载自动标定 | 8 姿态采样，r2 ≥ 0.99，写 `payload.yaml` |
| **Stage 9** | 经 manager 的完整工作流 | 全程只用 `/soft_robot/*`，状态 2→3→2→…→4→2 无 FAULT |

**通用中止规则**：任何非指令运动、输入停止后修正不停、或 manager 无法解释的状态 → 立即急停，诊断后再跑。零输出阶段（2、5）永不跳过。

Stage 8 标定成功后写出的 `payload.yaml` 会在下次 `soft_robot.launch` 启动时，由 manager 的 launch-prefix `load_payload_then_exec.sh` 在控制器加载**之前**注入参数服务器，标定值覆盖默认值。

### 恢复演练（三项都要做）

- **伺服中杀 RSI** → FAULT 闭锁 → 恢复 KUKA 侧后 `rosservice call /soft_robot/reset_fault` → 回 READY（累计计数器不清零）
- **拔 EKI 网线** → 5s 内 manager OFFLINE → 插回自动重连、工具重同步、回 READY
- **伺服中调 `/soft_robot/zero_sensor`** → 被拒（decision 11 门控）

---

## 关键运行时约束

这些是从实现里固化的硬约束，改动前务必理解：

- **控制器归 manager 独占加载**：两个 soft_robot 控制器只由 manager 加载，绝不在 launch 里 spawn 或手动启动。
- **硬件接口单线程回调队列**：`kuka_rsi_hw_interface` 节点用 `AsyncSpinner(1)`，控制器的 `mode_seq_/goal_seq_` 生产者依赖它，**不要**改成多线程 spinner。
- **`/sri_ft/zero` 受门控**：只能经 `/soft_robot/zero_sensor` 在 CONNECTED/READY 调用。直接调驱动服务会绕过门控，仅限调试，**伺服中绝不可**。
- **带 stamp 的 `rostopic pub -r` 必须加 `-s`**：让 `now` 每条消息重新求值（Plan 3 follow-up 2）。
- **工具帧力解析在伺服启动时锁定**：`R_tcp_sensor` 是会话常量，从伺服激活时的 `$TOOL` A/B/C + 配置的传感器安装偏移锁定一次。中途改示教盘 `$TOOL` 要到下次 start_servo 才生效。

### 降级路径（伺服仍会启动，各 `ROS_WARN_ONCE` 一次）
- 激活时无有效 EKI 工具数据 → 用单位工具旋转（回退到 tool-frame 之前的行为）
- 负载未标定（`gravity_n == 0`）→ 仅零位模式：跳过重力项，`compensate` 退化为 `raw - bias`。在归零姿态正确，但姿态变化会把未补偿的工具重量读成外力，TCP 向其漂移；死区与 SafetyLimiter 会限幅。**依赖带载姿态变化前先标定负载。**

---

## 操作接口（manager 服务/动作）

| 接口 | 类型 | 说明 |
|------|------|------|
| `/soft_robot/start_servo` | srv `StartServo` | `{mode, profile}` 启动伺服；mode 1=DIRECT_CARTESIAN 2=FORCE_COMPLIANCE，profile 0=DRAG 1=PRECISION |
| `/soft_robot/stop_servo` | srv | 停伺服回 READY |
| `/soft_robot/zero_sensor` | srv | 力传感器归零（CONNECTED/READY 门控） |
| `/soft_robot/reset_fault` | srv | 清 FAULT 闭锁 |
| `/soft_robot/move_to_orientation` | action `MoveToOrientation` | 姿态目标运动 |
| `/soft_robot/calibrate_payload` | action `CalibratePayload` | 8 姿态负载自动标定 |
| `/soft_robot/manager_state` | topic `ManagerState`（10Hz） | 系统状态 + 健康标志 |
| `/soft_robot/diagnostics` | topic（1Hz） | 诊断 |

系统状态：0=OFFLINE 2=READY 3=SERVOING 4=CALIBRATING 6=FAULT。

Web UI（可选）：`roslaunch soft_robot_web_interface web.launch`，消费的正是上表 manager 接口，不加入实时路径。

---

## 文档索引

- `docs/commissioning_checklist.md` — **真机联调逐阶段验收清单（部署必读）**
- `docs/kuka_iiqka_rsi_eki_setup.md` — KUKA 侧 RSI/EKI 逐字段安装清单
- `docs/rsi.md` — RSI Context 信号流图搭建指南
- `docs/superpowers/specs/` — 各设计文档
- `ros_ws/src/soft_robot_bringup/README.md` — 构建与冒烟测试细则
```
