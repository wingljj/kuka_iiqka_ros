# MuJoCo 力顺应验证环境 — 设计

日期:2026-07-04
状态:待实施
新增包:`kuka_mujoco_sim`(Python,catkin)
影响现有包:**无**(零修改、零新增依赖)

## 背景与动机

力顺应控制系统(9 个包 + 工具系顺应改造)开发完毕,现有测试为纯逻辑 gtest
(321 用例)+ web node 测试(29)+ 三 mock 集成探针。这些验证的是编解码、
算法逻辑、话题接线,但**没有一个带真实物理的力-运动闭环**:现有 `RsiMockCore`
把修正量 naive 累加(`pose += RKorr`),`sri_mock_server` 发脚本化常量/正弦力,
两者之间没有物理耦合——推墙不会有反力,姿态旋转不会改变重力在传感器系的投影。

目标:用 MuJoCo 建一个带质量、接触、六维力传感器的物理仿真,驱动机械臂末端
模型,端到端验证顺应算法的准确性——尤其是刚合并的工具系重力补偿与力解算
(见 `2026-07-04-tool-frame-compliance-design.md`)。

约束(用户明确):**不修改现有项目代码,新建独立包,与现有代码保持解耦。**

## 设计决策(已与用户确认)

| 决策 | 选择 |
|------|------|
| 机械臂保真度 | **浮动末端体**(6 DOF free body,不建 KUKA 全臂、不做 IK) |
| 接入方式 | **模拟线协议**(冒充 RSI UDP 客户端 + SRI TCP 服务端,零改现有代码) |
| 修正量作用 | **弹性跟踪 + 接触反力**(真实力-运动闭环,位姿因接触让位) |
| 传感器建模 | **含质量负载 + 可配置安装偏置**(能验证工具系重力补偿与未标定漂移) |
| 实现语言 | **Python**(mujoco pip 包 + rospy,与 catkin C++ 包解耦) |
| 验证产出 | **脚本化场景 + 断言报告**(数值化、可回归、CI 友好) |

## 架构与解耦边界

新建 Python catkin 包 `kuka_mujoco_sim`,替换 `sim.launch` 中两个 naive mock
(`kuka_rsi_sim_server`、`sri_mock_server`),其余一切原样复用。它在**两个线协议
端点**上冒充硬件,与真机链路走完全相同的 UDP/TCP 栈。

```
现有代码(不改)                  kuka_mujoco_sim(新)

kuka_rsi_hw_interface ──UDP <Rob>/<Sen>──→  RSI 客户端  ┐
 (真实驱动, 指向 127.0.0.1)                              ├─→ MuJoCo 物理引擎
sri_force_torque_driver ──TCP 二进制帧────→  SRI 服务端  ┘   (浮动末端体+负载+
 (真实驱动, 指向 127.0.0.1)                                   力传感器+接触环境)
kuka_eki_bridge ──────复用现有 eki_mock_server──────────────────
 (真实驱动, 指向 127.0.0.1)
```

三个真实驱动(hw_interface / sri_ft_driver / eki_bridge)原样运行,只是连到
本机的 MuJoCo 端点而非真机。EKI 侧工具角是会话常量,沿用现有 `eki_mock_server`,
MuJoCo 无需驱动它。

**解耦保证**:不改任何现有包的源码、CMakeLists、package.xml;不向任何现有包
新增依赖;`kuka_mujoco_sim` 只依赖 rospy / geometry_msgs / soft_robot_msgs
(消费方向,不反向)。删除本包对现有系统零影响。

### 包结构

```
ros_ws/src/kuka_mujoco_sim/
  package.xml                         # catkin, rospy/geometry_msgs/soft_robot_msgs
  CMakeLists.txt                      # catkin_python_setup()
  setup.py
  models/
    kuka_tcp_scene.xml                # MJCF: 浮动末端体+负载+力传感器+接触环境
  src/kuka_mujoco_sim/
    __init__.py
    frame_conventions.py              # KUKA A/B/C(Rz·Ry·Rx, mm/deg) ↔ MuJoCo(m, quat)
    rsi_endpoint.py                   # UDP 客户端: 发 <Rob>(RIst), 收 <Sen>(RKorr)
    sri_endpoint.py                   # TCP 服务端: SRI 二进制帧, 发传感器系 6 维力
    mujoco_world.py                   # MuJoCo 封装: 弹性跟踪+接触反力+传感器读数
    sim_node.py                       # 主节点: 装配三者 + 250Hz 步进循环
    scenarios.py                      # 脚本化验证场景 + 断言
  scripts/
    run_scenarios.py                  # 场景批跑, 输出 PASS/FAIL 报告 + 退出码
  launch/
    mujoco_sim.launch                 # 第四种 bringup(替换两个 naive mock)
  test/
    test_frame_conventions.py         # A/B/C ↔ quat 往返
    test_rsi_frame_codec.py           # <Rob>/<Sen> 编解码往返
    test_sri_frame_codec.py           # SRI 二进制帧编解码往返
    test_scenarios.py                 # 场景断言逻辑(可注入合成 world)
```

## RSI 端点与力-运动闭环

### RSI 端点(`rsi_endpoint.py`)— 冒充 KRC

UDP 客户端,线协议与 `RsiMockCore` 同构(`<Rob>` 含 RIst X/Y/Z/A/B/C + AIPos +
Mode + 单调 IPOC;`<Sen>` 解析 RKorr 6 增量 + Stop + IPOC 回显),但位姿"真相"
交给物理引擎:

```
每 4ms 周期:
  1. 发 <Rob>: RIst = 末端体当前【实际】位姿(mm/deg, BASE 系, KUKA A/B/C)
  2. 收 <Sen>: 解析 RKorr(每周期相对增量) + Stop + IPOC 回显
  3. 校验 IPOC 回显(与真机一致的时序契约)
  4. pose_target += RKorr             (与硬件 RsiMockCore 累加语义一致)
  5. 把 pose_target 交给 mujoco_world 作为弹性跟踪目标
  6. mujoco 步进 → 末端体实际位姿(可能被接触推离 target)
  7. 实际位姿回填 → 下周期 RIst
```

关键:回给 RSI 的 RIst 是末端体**被接触作用后的实际位姿**,不是目标位姿。
"顺应让位"在回路里真实闭合——控制器看到指令运动因接触未完全达成,如真机。

**取舍(诚实说明)**:RKorr 是"BASE 系每周期位置增量",累加进 `pose_target`
与现有硬件 mock 一致——真机上是 KRC 的 PosCorr 对象做这件事,本仿真在 PC 侧
复刻。真机 KRC 内部积分不可见,这是仿真必然近似,但与现有 mock 同构,不引入
新失真。

### 位姿闭环物理模型(`mujoco_world.py`)

- **末端体**:6 DOF 自由浮动刚体(free joint),挂可配置质量/质心的负载。
- **弹性跟踪**:`pose_target` 作为 mocap 体,末端体经高刚度 weld 或弹簧-阻尼
  约束跟随。自由空间紧跟目标;接触时接触反力与约束力平衡,末端体偏离 target,
  偏离量即"顺应位移"。
- **刚度/阻尼可配**:调高≈刚性跟踪,调低≈更软手感,便于观察不同耦合下行为。

### 坐标系(`frame_conventions.py`)

- RSI 契约:mm、deg、BASE 系、KUKA A/B/C = `Rz(A)·Ry(B)·Rx(C)`;MuJoCo:m、
  四元数。端点做单位与表示转换,A/B/C 约定与现有 `rotation.h` 一致(**独立实现
  一份**,不依赖现有 C++ 包,保持解耦)。
- 力传感器读数在**传感器系**输出(与真机 SRI 一致),安装偏置 `sensor_to_flange_abc`
  可配。

## SRI 力传感器端点

### SRI 端点(`sri_endpoint.py`)— 冒充 M8128 采集盒

TCP **服务端**,`sri_ft_driver` 作为客户端连入。协议按 `sri_frame.h`:收到 ASCII
`AT+GSD\r\n` 后流式发送二进制帧 `AA 55 | LEN | PN | 24字节(6×f32 小端 Fx Fy Fz
Mx My Mz) | 校验和`,~250Hz。

力数据来自 MuJoCo 每周期的**真实传感器读数**(非脚本常量):

```
传感器系读数 = 接触反力(投影到传感器系)
             + 负载重力在传感器系的投影(随末端体姿态变化)
             + 可选噪声/偏置(模拟真实传感器)
```

此二项之和正是真机 SRI 读到的量,也正是控制器需靠重力补偿剥离重力项、只留
接触力驱动顺应。**姿态一变,重力项在传感器系的投影就变**——这是验证工具系
重力补偿的关键:补偿正确则自由空间任意姿态净外力趋零;补偿错误(旧 TCP 系
缺陷)则冒假力、机器人漂移。

### 传感器与负载建模(MJCF `models/kuka_tcp_scene.xml`)

- 末端体上挂有质量负载(工件),质心可配。
- 力传感器建在末端体与负载之间(MuJoCo `force`/`torque` sensor 读合力),安装
  位姿相对法兰可配 `sensor_to_flange_abc`,用于验证非零安装偏置下的变换。
- 接触环境:墙/平面/工件,供"推墙顺应"场景产生真实接触力。

### 时序

RSI 端点(4ms)与 SRI 端点(4ms)是两条独立异步网络流,与真机一致。MuJoCo 步进
用统一仿真时钟(dt=4ms 软实时),两端点从同一份 `world` 状态读数——一份物理
真相,两个传感器视角,物理上一致。

**取舍(诚实说明)**:Python 软实时 250Hz 不保证硬实时,但验证环境不进真机链路,
控制器用实测周期 `period.toSec()`(Plan 2 已确认),抖动只影响仿真流畅度,不影响
算法正确性验证。

## 脚本化验证场景与断言报告

### 场景框架(`scenarios.py` + `scripts/run_scenarios.py`)

每个场景:设定初始位姿/负载/安装偏置/接触环境 → 发模式指令和(可选)外部扰动
→ 跑 N 周期 → 对末端体轨迹与力读数断言。批跑器顺序执行,输出 PASS/FAIL 报告
(场景名、断言项、实测 vs 预期、容差)+ 退出码(CI 友好)。

### 验证场景集

| # | 场景 | 断言 | 验证目标 |
|---|------|------|---------|
| 1 | 自由空间零力 | 已标定负载,任意静止姿态,无接触 → 净外力≈0,不漂移 | 重力补偿基线 |
| 2 | 姿态旋转重力补偿 | 已标定,末端体转到多姿态 → 每姿态净外力仍≈0,无漂移 | **工具系重力补偿修正(旧 TCP 系缺陷在此暴露)** |
| 3 | 推墙顺应 | 沿某方向接触墙 → 末端体沿接触法向让位,力稳定在预期量级 | 力-运动闭环 |
| 4 | 工具系方向性 | `$TOOL` 绕 Z 转 90°,传感器 X 向接触力 → 末端体沿 BASE 下预期方向运动 | **工具系变换 sensor→tool→base** |
| 5 | 安装偏置 | `sensor_to_flange_abc` 非零 → 力解算方向正确 | 安装偏置变换 |
| 6 | 未标定漂移 | `gravity_n=0`,姿态旋转 → 末端体出现可观测漂移(证明补偿确在起作用) | 未标定退化语义 |
| 7 | 各向同性回归 | 恒等工具/安装 → 行为与"直接 BASE 增益"一致 | 保护无回归 |

场景 2/4/5/6 直接对应工具系 spec 的测试计划条目——把纯逻辑 gtest 断言搬到带
真实物理的端到端回路复验。

### 交付形态

- 默认无头批跑:文本报告 + 退出码。
- MuJoCo 可视窗口为可选开关(`gui:=true`),不参与断言——断言全部数值化、可回归。

### 包内单测(`test/`,不依赖 MuJoCo 运行时)

帧编解码往返(`<Rob>`/`<Sen>`、SRI 二进制帧)、坐标变换(A/B/C↔quat 往返)、
场景断言逻辑(可注入合成 world)。纯逻辑,遵循现有"可离线跑"原则。

## 非目标(YAGNI)

- 不建完整 KUKA 6 轴模型、不做 IK(已确认浮动末端体)。
- 不改 RSI 侧 POSCORR(保持 BASE)。
- 不修改任何现有包(源码/CMake/package.xml/依赖)。
- 不追求硬实时(验证环境不进真机链路)。
- 不替换 EKI mock(工具角是会话常量,现有 mock 足够)。

## 真机相关性说明

本环境验证的是**算法在真实物理力-运动闭环下的正确性**,不替代真机装调
(见 `2026-07-04-tool-frame-compliance-followups.md`)。仿真通过 ≠ 真机通过:
过期工具角、真实传感器噪声/标定、KRC 内部 PosCorr 行为等仍需真机确认。仿真的
价值是在装调前以可回归方式抓住算法级缺陷(坐标系错误、符号错误、重力补偿方向)。
