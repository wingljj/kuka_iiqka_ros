# 力顺应控制器工具坐标系变换 — 设计

日期:2026-07-04
状态:待实施
影响包:`soft_force_control_core`、`soft_robot_controllers`

## 背景与动机

`ForceComplianceController` 目前把 SRI 传感器测得的力/力矩直接经重力补偿、
顺应律算成 Cartesian 修正量,再原样作为 BASE 系 RKorr 写给 RSI。参考实现
(SoftRobot 2.0)的做法是:用示教器读到的工具坐标系把力解算到工具系再驱动
运动,最终转回机器人 BASE 系下发。本设计把这一坐标系处理补齐。

用户场景:力传感器安装在末端、一般与法兰轴向对齐;工具坐标系是装配面,由
示教器设定并读取,通常与传感器坐标系不重合。目标是"传感器受力解算到工具
坐标系,从而驱动机械臂运动"。

## 现状(已读代码确认)

数据流(`force_compliance_core.cpp` + `soft_force_control_core`):

```
SRI wrench(传感器系)
  → ToolGravityCompensator.compensate(wrench, a, b, c)   // a/b/c 来自 RIst
  → ComplianceLaw.compute(...)                            // 逐轴映射成修正量
  → SafetyLimiter
  → RKorr(当作 BASE 系发出)
```

RSI 修正量坐标系:`ROS_RSI_CONTEXT.notes.md` 第 11 行确认为 **BASE**
(POSCORR mode RELATIVE, correction coordinate system BASE;TOOL 是预留实验)。

RIst / `getA/B/C`:来自 `$POS_ACT`,是 **TCP**(含当前 `$TOOL`)在 BASE 系
的位姿,由 `kuka_rsi_robot_hw.cpp` 填入 `kuka_tcp` state handle。

### 两个现存缺陷

1. **重力补偿坐标系错误**:`ToolGravityCompensator` 用机器人 A/B/C(TCP 位姿)
   把重力矢量旋转后从传感器读数中扣除。但传感器测量在**传感器系**,重力应在
   传感器系扣除。仅当传感器系与 TCP 系朝向一致(工具无旋转分量)时才恰好正确;
   装配面工具通常带旋转,会产生补偿误差。
2. **修正量坐标系不匹配**:顺应律在传感器系算出的修正量被当作 BASE 系 RKorr
   下发。仅当法兰/传感器系与 BASE 对齐时才正确。

控制器目前**不订阅** EKI 工具数据;`tool_a/b/c` 发布在 `/kuka/eki/state`
但从未进入控制器。

## 设计决策(已与用户确认)

| 决策 | 选择 |
|------|------|
| 顺应增益方向性 | **各向同性**(平移/旋转两组同性值,不做逐轴扩展) |
| 传感器→法兰旋转 | **可配置** `sensor_to_flange_rpy` 参数,默认 `[0,0,0]`(轴向对齐) |
| 工具 A/B/C 数据源 | **EKI 实时订阅** `/kuka/eki/state` 的 `tool_a/b/c` |
| 死区/限速/限加速阈值所在系 | **工具系** |
| 未标定(`gravity_n == 0`)时的行为 | **退化为纯置0模式**:重力项按零处理(等价只减 SRI bias),并在 `ModeState` 诊断标记"未补偿重力,姿态变化会漂移" |

### 关于各向同性增益的数学事实(重要)

纯标量增益对坐标系旋转不变:`R·(k·f) = k·(R·f)`。因此在**仅有增益**的情况下,
"工具系算再转 BASE"与"直接 BASE 算"结果相同。真正对坐标系敏感的是**死区、
限速、限加速**这三个逐轴阈值算子——"先旋转再阈值" ≠ "先阈值再旋转"。

用户已确认这些阈值放在**工具系**。因此工具系变换的价值有二:
(a) 让死区/限速阈值在工具系下有物理意义(如沿装配方向的死区);
(b) 为将来切换各向异性增益预留结构。
而**重力补偿修正**与**输出到 BASE 的正确性**是无论增益是否各向同性都必须修的。

## 帧链

```
BASE  ──R_bt(每周期, 来自 RIst A/B/C)──→  TCP(= TOOL)
FLANGE ──R_ftool(来自 $TOOL A/B/C, EKI, 会话内恒定)──→ TCP
FLANGE ──R_mount(sensor_to_flange_rpy, 配置, 默认单位)──→ SENSOR

会话内恒定量(activate 时锁存一次):
  R_tcp_sensor = R_ftool⁻¹ · R_mount        // SENSOR → TCP

每周期:
  R_base_sensor = R_bt · R_tcp_sensor        // SENSOR → BASE
```

旋转均采用 KUKA A/B/C 约定 `R = Rz(A)·Ry(B)·Rx(C)`(与 `rotation.h`
`kukaAbcToRotation` 一致)。

## 改造后每周期管线

```
1. 读 SRI wrench(传感器系)
2. 重力补偿(修正为传感器系):
     g_sensor = R_base_sensor⁻¹ · (0, 0, -G)
     力矩重力项 = com × g_sensor(com 在传感器系, 标定输出即传感器系)
     从 raw 中扣 bias 与重力项
3. wrench 旋转到工具系:
     f_tool = R_tcp_sensor · f_comp   (force 3-vec)
     t_tool = R_tcp_sensor · t_comp   (torque 3-vec)
4. ComplianceLaw 在工具系计算(增益/死区/限速/限加速均在工具系)
5. 修正量转回 BASE:
     平移: (x,y,z)_base = R_bt · (x,y,z)_tool
     角度: 把工具系角增量当作旋转矢量 ω_tool =(c,b,a)对应(x,y,z 轴分量,
       因 A=rotZ/B=rotY/C=rotX), 旋转 ω_base = R_bt · ω_tool, 再映射回
       A/B/C 增量(a←ω_z, b←ω_y, c←ω_x)。小角近似; 250 Hz 下每周期角增量
       远小于 1°, 旋转矢量线性叠加成立。
6. SafetyLimiter 在 BASE 系限幅(与 KUKA POSCORR BASE 限幅一致)
7. 输出 RKorr
```

## 组件与落点

### `soft_force_control_core`(纯逻辑, Eigen only, gtest 可测)

- **`ToolGravityCompensator`(改)**:接收传感器系朝向而非 TCP 朝向。签名从
  `compensate(raw, a, b, c)` 改为接收 `R_base_sensor`(或等价角度/矩阵)。
  修正重力扣除坐标系缺陷。CoM 保持传感器系(标定输出即传感器系,无需改标定)。
  **未标定退化**:当 `gravity_n == 0` 时,重力力项与力矩项均为零,`compensate`
  只做 `raw - bias`——等价于"纯置0"(SRI 侧 bias 已减,这里 payload bias 为零)。
  此路径本身正确,只是在姿态变化时不再补偿工具自重(见"未标定行为语义")。
- **`FrameResolver`(新增)**:持有会话内恒定的 `R_tcp_sensor`。提供:
  - `wrenchSensorToTool(Wrench) -> Wrench`(force、torque 各旋转)
  - `correctionToolToBase(CartesianCorrection, R_bt) -> CartesianCorrection`
  - 构造:`configure(tool_abc, sensor_to_flange_rpy)`。
  纯 Eigen,无 ROS,无分配。

### `ForceComplianceCore`(改)

- 新增成员:`FrameResolver frame_`;锁存的工具/安装旋转。
- `activate(start, tool_abc)`:除现有逻辑外,用锁存的 `tool_abc` 与配置的
  `sensor_to_flange_rpy` 配置 `frame_`。
- `update()`:在重力补偿后插入 sensor→tool 变换;顺应律后插入 tool→base 变换;
  再进 SafetyLimiter。

### `ForceComplianceController`(壳, 改)

- 新增订阅 `/kuka/eki/state`(`soft_robot_msgs/EkiState`),把 `tool_a/b/c`
  写入 realtime buffer(`tool_buf_`)。
- 参数 `sensor_to_flange_rpy`(默认 `[0,0,0]`),init 时读入两个 profile。
- `activate` 路径(`update()` 内 gate 进入分支 + `starting()` 重启分支):
  从 `tool_buf_` 读一次锁存,传给 `core_.activate(state, tool_abc)`。
- 回退(无工具数据):activate 时若尚无有效 EKI 工具数据(`tool_valid=false`),
  工具旋转按单位处理(等价现状行为),并在 `ModeState` 诊断标记。
- 回退(未标定):activate 时若 `gravity_n == 0`,置一个诊断标记提示"未补偿
  重力,姿态变化会漂移"(见下节语义)。伺服照常启动,不阻断。

### 未标定行为语义(设计说明)

"传感器置0"(`/sri_ft/zero`)只在**当前姿态**下减一个静态偏置常数,不随姿态
变化补偿工具自重。因此不做标定、只置0 直接启动伺服时:

- **姿态不变**:基本正常——置0 已把当前姿态的重力投影连同零偏一起归零,
  顺应控制读到的即净外力。
- **姿态旋转**:工具自重在传感器系的投影随姿态改变,而 bias 是置0那一刻的常数;
  差值(未补偿重力)被误当外力,机器人朝假力方向漂移;负载越重、姿态变化越大
  越明显。死区、`SafetyLimiter` 硬截断让它"漂而不飞",但手感与精度变差。

设计选择:不阻断这条路径(现场到位后置0拖动是合理用法),但在 `gravity_n == 0`
时给出明确诊断,使操作员知道当前处于"未补偿"状态。

## RT 安全性

- `$TOOL` 会话内恒定,只在 servo 启动(activate,非 RT 关键路径的进入沿)
  锁存一次;**每周期不依赖 async 工具数据**。工具中途更改到下次启动生效——正确
  且安全。
- 每周期额外开销:常量 3×3 矩阵 + 每周期 ~3 次矢量旋转,可忽略。
- realtime buffer 读取 `tool_buf_` 沿用现有 `wrench_buf_` / `mode_buf_` 模式。

## 测试计划

纯逻辑 gtest(`soft_force_control_core` + `soft_robot_controllers`):

1. **往返一致性**:sensor→tool→base 组合在恒等工具/安装下为恒等。
2. **工具旋转重映射**:`$TOOL` 绕 Z 转 90° 时,传感器 X 向力驱动 BASE 下预期
   方向修正。
3. **安装偏差**:`sensor_to_flange_rpy` 非零时变换正确。
4. **各向同性回归**:恒同增益 + 恒等 R_bt 下,新管线 BASE 输出与旧"直接 BASE
   增益"数学等价(保护现有行为不回归)。
5. **重力补偿修正**:工具带旋转分量时,静态负载在传感器系的重力扣除正确
   (旧实现此处会有误差)。
6. **回退(无工具数据)**:无 EKI 工具数据时按单位工具旋转,等价现状。
7. **未标定退化**:`gravity_n == 0` 时 `compensate` 输出 = `raw - bias`(重力项零),
   且姿态变化不引入重力补偿项;诊断标记置位。

回归:现有全部 node/gtest 不破;`ComplianceLaw`、`SafetyLimiter`、`AutoReTare`
等下游组件签名不变(仍吃 `Wrench` / `CartesianCorrection`,只是所处系语义变化)。

## 非目标(YAGNI)

- 不做各向异性逐轴增益(增益结构保持平移/旋转两组)。
- 不改标定流程(CoM 仍是传感器系输出)。
- 不改 `CartesianCorrectionController`(DIRECT_CARTESIAN 模式,外部已提供 BASE
  修正,不涉及力解算)。
- 不把 RSI 侧 POSCORR 改成 TOOL 系(保持 BASE,变换在 ROS 侧完成)。
