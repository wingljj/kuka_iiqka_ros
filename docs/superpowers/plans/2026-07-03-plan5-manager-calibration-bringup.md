# Plan 5/6: `soft_force_control_manager` + 标定 + bringup + KUKA 侧模板 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标:** 按规格 `docs/superpowers/specs/2026-07-02-kuka-rsi-ros-control-design.md` §5.5、§5.8、§6、§9、§11、§14、§15.3-15.4、§16-17,交付系统编排层与部署层:`soft_force_control_manager`(系统状态机 OFFLINE→…→FAULT、EKI/RSI/SRI 健康聚合、控制器切换编排、`/soft_robot/calibrate_payload` 载荷标定动作、payload.yaml 持久化、tool 同步)、`soft_robot_bringup`(实机/仿真两套 launch + 配置装载 + 冒烟清单)、KUKA 侧模板(KRL 程序、RSI 配置、EkiConfig.xml——以 `eki_frame.h`/`rsi_frame.h` 为唯一权威逐字段生成)与联调核对单;并闭环 Plan 3/4 移交的全部跟进事项(RSI `clearFault()`、tare 中断误报、connectNow 语义、mock 固定端口等)。

**架构:** 延续 Plan 1~4 的"纯逻辑类 + 薄 ROS 壳"分层。`SystemStateCore`(状态机+健康裁决)、`CalibrationSequencer`(标定姿态序列/采样/最小二乘)、`payload_yaml`(YAML 文本生成)为不含 `ros/ros.h` 的纯逻辑,gtest 直测;`ManagerRuntime`(worker 线程 + 注入 `ManagerOps` std::function 束)组合三者并对 lambda mock 做离线闭环测试(仿 Plan 4 `EkiBridgeRuntime` 模式);`soft_robot_manager` 节点壳只做参数装载、订阅→feed 搬运、服务/action 转发。KUKA 侧模板为纯文本交付物,配"人工核对清单"任务而非假测试。

**Plan series:** ① core algorithm library(已合入 main)→ ② `kuka_rsi_hw_interface` + msgs(已合入)→ ③ `soft_robot_controllers`(已合入)→ ④ `kuka_eki_bridge` + `sri_force_torque_driver`(已合入 main,f79f387)→ ⑤ manager + calibration + bringup + KUKA 模板(本计划)→ ⑥ web interface。

## 范围

**包清单(2 个新 catkin 包 + 4 个存量包增量 + 仓库根 kuka/ 与 docs/ 交付物):**

| 位置 | 内容 |
|---|---|
| `soft_force_control_manager`(新) | `SystemStateCore`(系统状态机,纯逻辑)、`CalibrationSequencer`(标定序列,内嵌 `sfc::PayloadEstimator`)、`payload_yaml`(持久化文本)、`ManagerRuntime`(worker 线程 + ManagerOps 注入)、`soft_robot_manager` 节点(4 服务 + 1 action + 1 话题 + 诊断)、`config/manager.yaml` + `config/calibration.yaml`、全套离线 gtest |
| `soft_robot_bringup`(新) | `soft_robot.launch`(实机)、`sim.launch`(三 mock 闭环)、配置装载编排、README 冒烟清单(含 Plan 4 跟进 8 的 SRI roscore 冒烟欠账) |
| `kuka_rsi_hw_interface`(增量) | `RsiSessionMonitor::clearFault()`(保留累计计数)、`KukaRsiRobotHW::requestFaultClear()` 原子复位、节点新增 `/kuka/rsi/reset_fault` 服务(Plan 4 跟进 10) |
| `sri_force_torque_driver`(增量) | `SriStreamSession::reset()` 清 `last_zero_ok_`(Plan 4 跟进 4/N1)、`SriMockConfig::listen_port` + `sri_mock_server --port` + 注释勘误(跟进 2) |
| `kuka_eki_bridge`(增量) | `connectNow` 单次语义裁决收敛(跟进 5/N2)、`EkiMockConfig::listen_port` + `eki_mock_server --port` |
| `soft_robot_msgs`(增量) | `CalibratePayload.action`、`ManagerState.msg`、`StartServo.srv`;`test_msgs.cpp` +2 用例 |
| `kuka/`(仓库根,新) | `krl/ROS_RSI_SERVO.SRC/.DAT`、`rsi/ROS_RSI_ETHERNET.xml` + 上下文说明、`eki/ROS_EKI_CONFIG.xml`(文本模板) |
| `docs/`(新) | `kuka_iiqka_rsi_eki_setup.md`(安装 + 人工核对清单)、`commissioning_checklist.md`(真机联调核对单) |

**非目标(本计划不做):**

- Web UI(→ Plan 6)。manager 的服务/action/话题面即 Plan 6 的全部后端接口。
- 规格 §17 其余文档(architecture.md 等)与 §18 未来项(Mode 8、AKorr、TOOL 系、六维鼠标)。
- SRI mock 的 `AT+GSD=STOP` 分支(Plan 4 跟进 7,条件项):本计划集成/冒烟不走 stop 命令路径(驱动 runtime 从不发 STOP),条件未触发,明确排除。
- 控制器/驱动/桥的行为改动(上表列明的针对性小修除外);`soft_force_control_core` 与 `soft_robot_controllers` 实现代码零触碰。
- `stopping()` 内 `requestCancel()` 的锁语义改造(Plan 3 跟进 6,条件项):v1 无 RT 内核/抖动预算收紧计划,条件未触发,记入遗留风险。
- `publishState` sim-time 回跳复位(Plan 3 跟进 8,低优先):v1 无 bag 循环回放调试需求,排除,记入遗留风险。
- transport 双副本抽包重构(Plan 4 遗留风险 9):本计划无第三处 TCP client 需求(manager 全走 ROS 服务),条件未触发。

**与已交付 API 的接口关系(签名以真实头文件为准,本计划已逐一核对):**

- 消费 Plan 1 `soft_force_control_core`:`sfc::PayloadEstimator`(`addSample(a_deg,b_deg,c_deg,averaged)` / `solve() -> PayloadFitResult{ok, params{gravity_n, com_x/y/z, bias}, r2_force, r2_torque}`,n≥4 才可解)、`sfc::Wrench`、`sfc::ControlMode`/`sfc::Profile` 枚举(wire 值经 Plan 3 `mode_bridge.h` static_assert 钉死)、`sfc::kukaAbcToRotation`(测试 synth 用)。
- 消费 Plan 2 `kuka_rsi_hw_interface`:`RsiSessionMonitor`(本计划新增 `clearFault()`)、`KukaRsiRobotHW::resetFault()/faulted()`、`/kuka/rsi/state`(`RsiState`,50 Hz)、`kuka_rsi_sim_server` 可执行(`--target-port/--cycle-ms/--a0` 等,修正量积分进位姿——标定冒烟的收敛基座)。
- 消费 Plan 3 `soft_robot_controllers`:插件名 `soft_robot_controllers/ForceComplianceController` / `soft_robot_controllers/CartesianCorrectionController`(独占 `kuka_tcp`,同刻仅一个 RUNNING);`/soft_robot/mode_command`(`ModeCommand`)、`/soft_robot/mode_state`(`ModeState`)、`/soft_robot/move_to_orientation`(`MoveToOrientationAction`,goal: a/b/c/use_position/x/y/z/speed_scale;result_code CONVERGED=0/TIMEOUT=1/ABORTED=2);`config/soft_robot_controllers.yaml`(payload 块 schema 即 payload.yaml 覆写目标)。
- 消费 Plan 4:`/kuka/eki/{connect,start_rsi_program,stop_rsi_program,set_mode,reset_fault,set_tool_base,get_tool}`、`/kuka/eki/state`(`EkiState`,10 Hz)、`/kuka/diagnostics`;`/sri_ft/{wrench_raw,status,zero,set_filter}`(status 10 Hz);`sri_mock_server`/`eki_mock_server` 可执行;`eki_frame.h` schema 注释(KRL/EkiConfig.xml 模板唯一权威)。
- git 基线:Plan 1~4 均已合入 main(HEAD=f79f387);从 `main` 新建 `feature/manager-calibration-bringup`。

## 跟进清单消化对照表(计划硬性要求,执行者与评审对照用)

`docs/superpowers/plans/2026-07-02-plan4-followups.md`(11 条,按文件编号):

| # | 条目 | 归属裁决 | 闭环位置 |
|---|---|---|---|
| 1 | gtest 目标名全局冲突(已改名 `test_eki_tcp_client_transport`) | 全收(约束化) | Global Constraints:新增测试目标一律带包前缀(`test_manager_*`);引用 Plan 4 原文命令时以改名后为准 |
| 2 | `sri_mock_server_main.cpp` 顶部注释与行为矛盾 | 全收 | Task 2 Step 5:`--port` 落地时一并勘误注释 |
| 3 | T8 实施报告 2 处笔误 | 排除(计划维护类,非代码;不属本计划交付物,留 Plan 4 文档回写) | — |
| 4 | tare 捕获被重连打断时 `requestZero` 误报成功(N1) | 全收 | Task 2 Step 1-2:`reset()` 清 `last_zero_ok_` + 固化测试 |
| 5 | `connectNow` 失败后 `connect_requested_` 不清除(N2) | 全收(裁决:单次语义) | Task 2 Step 6-8 + 待确认决策 9 |
| 6 | 单线程 spin 停顿 vs manager 新鲜度阈值(N3) | 全收(裁决:容忍,不改 eki 节点) | 待确认决策 3(eki 阈值 5 s)+ 决策 13(manager 自身发布不被服务阻塞)+ Task 4/6 |
| 7 | SriMockServer 把 `AT+GSD=STOP` 也当启动命令 | 排除(条件项未触发:本计划不走 stop 命令路径,驱动从不发 STOP) | 非目标节 + 遗留风险 |
| 8 | SRI roscore 冒烟未执行 | 全收 | Task 8 冒烟清单第 2 节(逐条并入,含 `hz≈250`/`--offset`/zero 后归零/`streaming: True`) |
| 9 | 真机联调核对 stamp 与到达时刻差 | 全收 | Task 10 联调核对单(含"用 package_gaps 而非 stamp 做间隔统计"警示) |
| 10 | RSI latched fault 复位路径必补 `clearFault()` | 全收 | Task 1(保留累计计数,勿用 `reset()`) |
| 11 | KRL/EkiConfig.xml 逐字段对齐 + 100 ms 心跳义务 + `/sri_ft/zero` 仅限非 SERVOING | 全收 | Task 9(模板 + 对照表 + 心跳)/ Task 6(zero 门禁,决策 11) |

`docs/superpowers/plans/2026-07-02-plan3-followups.md`(9 条):

| # | 条目 | 归属裁决 | 闭环位置 |
|---|---|---|---|
| 1 | Task 4 期望值漏算速度夹持(教训) | 全收(约束化) | Global Constraints:期望值推导不得漏中间环节,本计划全部数值推导在任务头给出 |
| 2 | `rostopic pub -r` 含 stamp 一律带 `-s` | 全收 | Task 8 README 规则 + 所有冒烟示例照办 |
| 3 | wrench stamp = 采样/接收时刻 | Plan 4 已落实;联调抽查份额 | Task 10 联调核对单 |
| 4 | RsiState 为最新值语义,manager 自做话题新鲜度监测 | 全收 | 决策 2/3 + Task 4(`rsi_topic_fresh` 输入)+ Task 6 |
| 5 | `mode_seq_`/`goal_seq_` 依赖单线程回调队列 | 全收 | Task 8:hw 节点保持 `AsyncSpinner(1)`,launch 注释写明约束 |
| 6 | `stopping()` 内 `requestCancel()` 阻塞锁 | 排除(条件项未触发:v1 无 RT 内核) | 非目标节 + 遗留风险 |
| 7 | preempt 与收敛竞态可回报 SUCCEEDED | 全收 | 决策 15 + Task 5/7(cancel 后 SUCCEEDED 按成功终态处理) |
| 8 | `publishState` 节流依赖时钟单调 | 排除(低优先,v1 无 bag 循环回放) | 非目标节 + 遗留风险 |
| 9 | goal 默认参数为保守投运值,投运调参勿以测试数值为预期 | 全收 | Task 10 联调核对单(调参指引条目)+ Task 8 sim 冒烟时长预期按保守值推导 |

## 待确认(规格未定项,本计划采用的默认决策)

1. **manager 分层形态**:`SystemStateCore` / `CalibrationSequencer` / `payload_yaml` 为纯逻辑(无 `ros/ros.h`,时钟注入 double 秒),gtest 直测;`ManagerRuntime` 为 worker 线程 + 注入 `ManagerOps`(std::function 束:EKI 服务、控制器切换、模式发布、运动目标、持久化),lambda mock 离线闭环测试;`soft_robot_manager` 节点只做 ROS 接线。理由:编排逻辑不绑 ROS 服务即可测,延续 Plan 2/4 已验证的模式。
2. **状态机语义细则**(规格 §11 只给状态集与 SERVOING 前置):FAULT 仅由 `eki_fault` / `rsi_fault` /(SERVOING、CALIBRATING 中 `/kuka/rsi/state` 话题死亡)触发并闭锁,需显式 reset;SERVOING 中非致命健康丢失(SRI 停流/EKI 心跳失鲜/RSI 断流但未 fault)→ DEGRADED,恢复自动回 SERVOING;READY 前置 = EKI 链路新鲜 + `program_ready` + tool 已同步 + SRI streaming + 控制器已 load。**READY 不要求 RSI 帧流**(KRC 在 START_RSI 前不发 RSI 帧),RSI 有效性在 start 编排内验证(有界等待,决策 4)。OFFLINE/CONNECTED/READY 之间随健康自动迁移;进 SERVOING/CALIBRATING 仅经显式命令。
3. **新鲜度阈值**(Plan 4 跟进 6/N3 裁决):`/kuka/eki/state` 阈值默认 **5 s**(容忍 eki 节点单线程 spin 下命令期间最长 ~3 s 发布间隙 + 余量;**不给 eki 节点换多线程 spinner**——管理通道可接受,改动面最小);`/kuka/rsi/state` 0.5 s(50 Hz 话题,manager 自做监测,Plan 3 跟进 4);`/sri_ft/status` 2 s(10 Hz)。全部入 `manager.yaml`。
4. **控制器切换编排**:manager 是 `/soft_robot/mode_command` 的唯一生产者(bringup 文档明示,手动 `rostopic pub` 仅调试);切换序列 = 发 `MODE_IDLE` → `switch_controller`(STRICT)→ 发目标模式(ModeManagerCore 的"一切经 IDLE"规则);SERVOING 涵盖 FORCE_COMPLIANCE 与 DIRECT_CARTESIAN 两种 servo 模式(active mode 记录于 `ManagerState.mode`)。start 编排中 RSI 帧流的有界等待默认 5 s(`rsi_connect_wait_s`)。切换失败即命令失败返回、状态留在 READY(不自动回滚 EKI start,操作员可重试或 stop),消息注明。
5. **标定样本姿态取目标值**:`PayloadEstimator::addSample` 的 a/b/c 用 goal 目标姿态(收敛容差 0.1° ≪ 标定姿态间隔 ≥30°),不回读实际位姿——控制器无姿态回传话题,免新增通道。
6. **标定失败不回程**:MOVE_FAILED / STREAM_LOST / CANCELLED 立即终止且不再发运动目标(goal 结束后控制器本就零输出),系统回 READY;仅成功路径驱动 `return_pose`(默认 A=B=C=0,规格 §9 第 5 步)。
7. **payload 更新路径**(规格 §9 第 6-7 步 "updates the realtime parameter snapshot" 的落地形态):成功后 manager 写 `payload.yaml` → `ros::param::set` 覆写 `/force_compliance_controller/payload/*` → `unload_controller` + `load_controller`(标定期间该控制器本就 stopped;ros_control 控制器参数仅在 load/init 时读取)。bringup 启动时若 `payload.yaml` 存在则 `rosparam load`,开机即用标定值。
8. **`clearFault()` 语义**(Plan 4 跟进 10 / Plan 2 跟进 5 收尾):`RsiSessionMonitor::clearFault()` 只清 `fault` + `consecutive_timeouts`,保留 `total_timeouts/bad_frames/ipoc_jumps/connected/last_ipoc`(`RsiState` 注释 "cumulative since node start" 的承诺);`KukaRsiRobotHW::resetFault()` 改走 `clearFault()`;跨线程复位经 `requestFaultClear()` 原子标志、`read()` 开头应用(服务线程与控制线程不共触 monitor);hw 节点新增 `/kuka/rsi/reset_fault`(Trigger)。`reset()` 保留全清语义仅供测试/未来重连场景,不再接入故障路径。
9. **`connectNow` 单次语义**(Plan 4 跟进 5/N2 裁决):有界等待超时即撤回 `connect_requested_`,`auto_reconnect=false` 下不再蜕变为持续重连;撤回瞬间已在途的最后一次 connect 尝试(≤ `connect_timeout_ms`)属可接受窗口,注释注明。
10. **mock 固定端口**:`SriMockConfig` / `EkiMockConfig` 增 `listen_port`(默认 0 = 内核自选,存量测试不变);`sri_mock_server` / `eki_mock_server` 增 `--port`;顺带勘误 sri main 顶部注释(跟进 2)。sim.launch 用固定端口 4008/54600 与两 yaml 对齐。
11. **`/sri_ft/zero` 门禁在 manager**(Plan 4 遗留风险 5 / 跟进 11):`/soft_robot/zero_sensor` 仅 CONNECTED/READY 放行转发 `/sri_ft/zero`,SERVOING/CALIBRATING/DEGRADED/FAULT 拒绝;驱动服务本体不改(保持独立可用),bringup README 警告绕过 manager 直调的风险。标定期间被门禁拒绝 ⇒ 标定全程驱动侧偏置恒定,估计器解出的 bias 为"当前流的残余偏置",与驱动 tare 叠加自洽(决策 14)。
12. **manager 服务/消息面**(规格 §5.5 只给职责):`/soft_robot/start_servo` = 新 `StartServo.srv`(mode+profile,wire 值 = `ModeCommand` 常量);`/soft_robot/{stop_servo,reset_fault,zero_sensor}` = `std_srvs/Trigger`;`/soft_robot/calibrate_payload` = 新 `CalibratePayload.action`;`/soft_robot/manager_state`(新 `ManagerState.msg`,10 Hz,`system_state` 数值复用 `ModeState.SYSTEM_*`);`/soft_robot/diagnostics`(`DiagnosticArray`,1 Hz)。
13. **manager 节点线程模型**:`AsyncSpinner(3)`(订阅 + 服务 + action);命令经 `command_mutex_` 串行、耗时 ops 调用在数据锁外;`ManagerState` 由 runtime worker 线程 10 Hz 发布快照,**永不被服务阻塞**(N3 教训的 manager 侧规避)。
14. **标定 wrench 源**:`/sri_ft/wrench_raw` 原始流(规格 §9 数据通路);每姿态默认 100 样本取均值(规格 §9 第 3d 步的改进点);采样期间 SRI 停流(status 失鲜或 streaming=false)→ STREAM_LOST 终止。
15. **SUCCEEDED-after-preempt 按成功处理**(Plan 3 跟进 7):motion action done 回调把 `SUCCEEDED` 一律计成功(即使已发 cancel);sequencer 的 cancel 路径不依赖 PREEMPTED 终态。
16. **launch 分层**:`soft_robot.launch`(实机,IP 等 arg 化)与 `sim.launch`(三 mock + 127.0.0.1 参数覆写)双入口,后者不 include 前者(mock 端点/参数差异大,平铺更可读);controller_manager 宿主(hw 节点)保持 `AsyncSpinner(1)` 单线程回调队列并在 launch 注释写明(Plan 3 跟进 5);README 所有含 stamp 的 `rostopic pub -r` 示例带 `-s`(Plan 3 跟进 2)。
17. **KUKA 侧模板交付形态**:纯文本模板 + 安装文档 + 人工核对清单(无法自动测试,不造假测试);`ROS_EKI_CONFIG.xml` 逐字段以 `eki_frame.h` 注释为唯一权威生成(动作码 0~6、`<RobotCommand><Cmd Seq Action Value/><Tool/><Base/>`、`<RobotState><Ack Seq Ok Code/><Prog Ready RsiActive Fault Mode/><Tool/>`);KRL 心跳周期 **100 ms**(`EKI_Send` RobotState,与桥 `state_timeout_s=1.0` 留 10 倍余量,Plan 4 遗留风险 8);RSI 模板按 `rsi_frame.h` 的 Rob/Sen schema、`PosCorr` 相对修正、`BASE` 坐标系(规格 §6.1);模板内 KRL 函数名按 iiQKA 6.x 系(`EKI_Load/EKI_Open/RSI_LOAD/RSI_ACTIVATE/RSI_PROCESS_ON/RSI_MOVECORR`),人工核对清单要求对照 `ref/` 两本手册逐条确认语法。

## Global Constraints

- ROS1 Noetic,catkin 工作区 `/home/ljj/kuka_iiqka_ros/ros_ws`;新包 `ros_ws/src/soft_force_control_manager/`、`ros_ws/src/soft_robot_bringup/`;存量包仅做上表列明的针对性增量。
- C++14,`-Wall -Wextra`,零警告;所有代码与注释英文;计划/报告中文。TDD:先失败测试(构建失败即失败态),再最小实现,再通过,再提交。
- **Noetic 的 `catkin_add_gtest` 不自动链接 gtest_main:所有测试链接行必须包含 `${GTEST_MAIN_LIBRARIES}`。**
- **catkin_make 单工程构建,gtest 目标名在全局 CMake 命名空间:新增目标一律带包前缀**(本计划为 `test_manager_*`;Plan 4 跟进 1 的教训,其 eki transport 测试目标已改名 `test_eki_tcp_client_transport`,引用照此)。
- **Noetic 勘误照录**(Plan 2 已裁决,防复发):① `hardware_interface::HardwareInterface::claim()` 基类无条件记录资源,只读接口需 no-op 重写;② `JointStateHandle::getPosition()` 按值返回 `double`,不要写 `*handle.getPosition()`。
- **测试期望值全部在任务头手工推导,不得漏中间环节**(Plan 3 跟进 1 教训):本计划数值推导集中于 Task 2(tare 均值/偏置扣除)、Task 5(合成载荷精确回收、进度分数)、Task 6(标定闭环终值)、Task 8(sim 标定的 G≈0/bias≈(0,0,5)/r2=1 推导)。
- 统计口径:**逐包 XML 直读**(`build/test_results/<pkg>/gtest-*.xml` 的 tests/failures/errors 求和),不用 `catkin_test_results`(双计)。整仓回归基线 252,本计划完成后 **291**(+39;逐包预期表见验收清单)。
- 全部 gtest 离线可跑:无 roscore、无真机;网络测试仅 127.0.0.1 真 socket,端口默认 kernel 自选(bind 0)——本计划仅 mock 固定端口测试用"探测释放再绑"法;一切等待有界:单窗 ≤ 0.5 s,`waitFor` ≤ 2 s,重连 backoff 0.05~0.1 s;禁止无界 `while`。跑后零残留(无进程、无监听端口)。
- mock 分层:确定性原语进 gtest,连续行为(波形/节拍)只进独立可执行。
- 运行时时钟注入 double 秒(`nowS()` steady_clock);纯逻辑类(`SystemStateCore`、`CalibrationSequencer`、`payload_yaml`)不含 `ros/ros.h`、gtest 内无 `ros::Time`;runtime/节点允许 POSIX 线程,节点壳才允许 roscpp。
- 单位:笛卡尔 mm/deg(KUKA A/B/C = Z-Y-X),wrench N/Nm,com 单位 m(`sfc::PayloadParams` 注释);标定姿态表 deg。
- 含 stamp 的 `rostopic pub -r` 示例一律带 `-s`(Plan 3 跟进 2)。
- 构建/运行命令(仓库根 `/home/ljj/kuka_iiqka_ros`):

```bash
cd ros_ws && catkin_make                                   # full build
cd ros_ws && catkin_make tests                             # build all test binaries
./devel/lib/soft_force_control_manager/<test_binary>       # run one gtest binary
cd ros_ws && catkin_make run_tests                         # whole-repo regression
```

**manager 数据通路(规格 §5.5/§9/§11 的本计划落地形态):**

```text
subscriber threads (AsyncSpinner):
  /kuka/eki/state  -> feedEkiState{connected,state_fresh,ready,rsi_active,fault,tool}
  /kuka/rsi/state  -> feedRsiState{connected,fault}
  /sri_ft/status   -> feedSriStatus{streaming}
  /sri_ft/wrench_raw -> feedWrench (consumed only while calibrating/SAMPLING)
worker thread (ManagerRuntime::run, 10 Hz tick):
  build HealthInputs (msg ages vs thresholds) -> SystemStateCore::update
  tool sync edge: CONNECTED+ready & !synced -> ops.ekiGetTool (2 s retry backoff)
  CalibrationSequencer::tick -> SEND_GOAL -> ops.sendMotionGoal
                             -> FINISHED  -> success: ops.persistPayload
                                          -> switch back + calibrationFinished
  publish snapshot -> node timer-free: worker invokes ops.publishManagerState
service threads (command_mutex_ serialized, ops called outside data lock):
  start_servo: READY? -> ekiStartRsi (if !rsi_active) -> bounded wait RSI fresh
               -> switchControllers(target, other) -> mode IDLE -> mode target
               -> core.requestStart
  stop_servo:  mode IDLE -> switch stop -> ekiStopRsi -> core.requestStop
  reset_fault: FAULT? -> rsiResetFault + ekiResetFault -> core.requestReset
  zero_sensor: CONNECTED|READY? -> sriZero : reject
action thread: calibrate_payload -> beginCalibration -> poll snapshot ->
               feedback 10 Hz -> DONE/FAILED -> setSucceeded/Aborted
```

---

## File Structure

```text
ros_ws/src/kuka_rsi_hw_interface/            # Task 1 (increment)
  include/kuka_rsi_hw_interface/rsi_session_monitor.h   # + clearFault()
  src/rsi_session_monitor.cpp                           # + clearFault()
  include/kuka_rsi_hw_interface/kuka_rsi_robot_hw.h     # + requestFaultClear()
  src/kuka_rsi_robot_hw.cpp                             # apply flag in read()
  src/kuka_rsi_hw_interface_node.cpp                    # + /kuka/rsi/reset_fault
  package.xml / CMakeLists.txt                          # + std_srvs
  test/test_rsi_session_monitor.cpp                     # +2 tests
  test/test_kuka_rsi_robot_hw.cpp                       # +2 tests

ros_ws/src/sri_force_torque_driver/          # Task 2 (increment)
  src/sri_stream_session.cpp                            # reset() clears last_zero_ok_
  include/sri_force_torque_driver/sri_mock_server.h     # SriMockConfig.listen_port
  src/sri_mock_server.cpp                               # honour listen_port
  src/sri_mock_server_main.cpp                          # --port + comment erratum
  test/test_sri_stream_session.cpp                      # +1 test
  test/test_sri_mock_server.cpp                         # +1 test

ros_ws/src/kuka_eki_bridge/                  # Task 2 (increment)
  src/eki_bridge_runtime.cpp                            # connectNow single-shot
  include/kuka_eki_bridge/eki_mock_server.h             # EkiMockConfig.listen_port
  src/eki_mock_server.cpp                               # honour listen_port
  src/eki_mock_server_main.cpp                          # --port
  test/test_eki_bridge_runtime.cpp                      # +1 test
  test/test_eki_mock_server.cpp                         # +1 test

ros_ws/src/soft_robot_msgs/                  # Task 3 (increment)
  msg/ManagerState.msg
  srv/StartServo.srv
  action/CalibratePayload.action
  CMakeLists.txt                                        # register new files
  test/test_msgs.cpp                                    # +2 tests

ros_ws/src/soft_force_control_manager/       # Tasks 4-7 (new package)
  package.xml
  CMakeLists.txt
  include/soft_force_control_manager/
    system_state_core.h                     # system FSM + health verdict (pure)
    calibration_sequencer.h                 # pose sequence + estimator (pure)
    payload_yaml.h                          # payload.yaml text emitter (pure)
    manager_runtime.h                       # worker thread + ManagerOps injection
  src/
    system_state_core.cpp
    calibration_sequencer.cpp
    payload_yaml.cpp
    manager_runtime.cpp
    manager_node.cpp                        # ROS shell (soft_robot_manager)
  config/
    manager.yaml
    calibration.yaml
  test/
    test_system_state_core.cpp              # target test_manager_system_state
    test_calibration_sequencer.cpp          # target test_manager_calibration_sequencer
    test_payload_yaml.cpp                   # target test_manager_payload_yaml
    test_manager_runtime.cpp                # target test_manager_runtime

ros_ws/src/soft_robot_bringup/               # Task 8 (new package)
  package.xml
  CMakeLists.txt
  launch/soft_robot.launch                  # real robot entry
  launch/sim.launch                         # three-mock closed loop
  README.md                                 # smoke checklist (incl. Plan4 FU-8)

kuka/                                        # Task 9 (repo root, text deliverables)
  krl/ROS_RSI_SERVO.SRC
  krl/ROS_RSI_SERVO.DAT
  rsi/ROS_RSI_ETHERNET.xml
  rsi/ROS_RSI_CONTEXT.notes.md
  eki/ROS_EKI_CONFIG.xml

docs/                                        # Tasks 9-10
  kuka_iiqka_rsi_eki_setup.md               # install + manual field-by-field checklist
  commissioning_checklist.md                # real-robot commissioning (spec 15.4)
```

---

### Task 0: 建立分支

**Files:** 无代码改动。

- [ ] **Step 1: 从 main 新建特性分支**

```bash
cd /home/ljj/kuka_iiqka_ros
git checkout main && git pull
git checkout -b feature/manager-calibration-bringup
git log --oneline -1   # expect f79f387 (docs: record plan 4 ... follow-ups)
```

验收:分支基于 f79f387(或更新的 main HEAD);工作区干净。

---

### Task 1: `kuka_rsi_hw_interface` 增量 —— `clearFault()` + `/kuka/rsi/reset_fault` 服务

**目标:** 落实 Plan 4 跟进 10(= Plan 2 跟进 5 / Plan 4 待确认 10 的裁决):RSI latched fault 需要一条不清累计计数的复位路径,并把它以服务形式暴露给 manager。`RsiSessionMonitor::reset()` 全清语义保留但退出故障路径。跨线程安全:服务回调运行在 hw 节点的 AsyncSpinner 线程,monitor 只归控制线程触碰,故复位经原子标志在 `read()` 开头应用。

**Files:**
- Modify: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/rsi_session_monitor.h`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/src/rsi_session_monitor.cpp`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/kuka_rsi_robot_hw.h`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/src/kuka_rsi_robot_hw.cpp`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/src/kuka_rsi_hw_interface_node.cpp`
- Modify: `ros_ws/src/kuka_rsi_hw_interface/package.xml`(+ std_srvs)
- Modify: `ros_ws/src/kuka_rsi_hw_interface/CMakeLists.txt`(find_package + CATKIN_DEPENDS + std_srvs)
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_rsi_session_monitor.cpp`(+2)
- Test: `ros_ws/src/kuka_rsi_hw_interface/test/test_kuka_rsi_robot_hw.cpp`(+2)

**Interfaces:**
- Consumes: 既有 `RsiSessionMonitor`(`onFrame/onTimeout/onBadFrame/reset/faulted/stats`)、`KukaRsiRobotHW`(`configure/read/write/resetFault/faulted/sessionStats/listenPort`)。
- Produces(Task 4/6/8 消费):
  - `void RsiSessionMonitor::clearFault()` —— 只清 `stats_.fault` 与 `stats_.consecutive_timeouts`,保留 `total_timeouts/bad_frames/ipoc_jumps/connected/last_ipoc`。
  - `void KukaRsiRobotHW::requestFaultClear()` —— 线程安全(原子标志),下一次 `read()` 开头应用;既有 `resetFault()` 改为直调 `monitor_.clearFault()`(保留计数,签名不变——存量测试同步改期望)。
  - hw 节点服务 `/kuka/rsi/reset_fault`(`std_srvs/Trigger`):调 `requestFaultClear()`,`success=true` 恒定(应用是异步的,消息注明 "fault clear requested; applied on next RSI read cycle")。

**期望值推导(任务头,Plan 3 跟进 1 约束):**
- `ClearFaultKeepsCumulativeCounters`:max_consecutive_timeouts=2,onFrame(1) 后 2 次 onTimeout ⇒ fault=true,total_timeouts=2,consecutive=2。clearFault() ⇒ fault=false,consecutive=0,**total_timeouts 仍=2**,connected 仍 true,last_ipoc 仍 1。再 1 次 onTimeout ⇒ total=3、consecutive=1 <2 ⇒ 不再 fault(计数续增而非从故障残值继续)。
- `RobotHW ResetFaultKeepsCounters`:testConfig(max=2) 下发一帧(connected)、两次空 read(timeout)→ faulted,sessionStats().total_timeouts=2;`resetFault()` 后 faulted=false 且 total_timeouts 仍=2(旧行为是 reset() 全清=0,断言由此翻转,这就是本任务的行为变更点)。
- `RequestFaultClearAppliedOnRead`:faulted 后调 `requestFaultClear()`,此刻 `faulted()` 仍 true(未应用);mock 再发一帧后 `read()` ⇒ faulted=false 且该帧正常入 state(x 更新)。

- [ ] **Step 1: 写失败测试(monitor 层)**

在 `test/test_rsi_session_monitor.cpp` 末尾追加:

```cpp
TEST(SessionMonitor, ClearFaultKeepsCumulativeCounters) {
  // Plan 4 follow-up 10: the manager recovery path must not wipe the
  // "cumulative since node start" RsiState counters.
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 2;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onTimeout();
  m.onTimeout();
  ASSERT_TRUE(m.faulted());
  ASSERT_EQ(m.stats().total_timeouts, 2u);

  m.clearFault();
  EXPECT_FALSE(m.faulted());
  EXPECT_EQ(m.stats().consecutive_timeouts, 0u);
  EXPECT_EQ(m.stats().total_timeouts, 2u);   // kept
  EXPECT_TRUE(m.connected());                // kept
  EXPECT_EQ(m.stats().last_ipoc, 1u);        // kept
}

TEST(SessionMonitor, ClearFaultRestartsMissCounting) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 2;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onTimeout();
  m.onTimeout();
  m.clearFault();
  m.onTimeout();                             // 1 < 2: not faulted again
  EXPECT_FALSE(m.faulted());
  EXPECT_EQ(m.stats().total_timeouts, 3u);
}
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | tail -5
```

预期:编译失败,`'class kuka_rsi::RsiSessionMonitor' has no member named 'clearFault'`。

- [ ] **Step 3: 实现 `clearFault()`**

`include/kuka_rsi_hw_interface/rsi_session_monitor.h` 中在 `void reset();` 之后加:

```cpp
  // Manager-driven fault recovery (Plan 4 follow-up 10): unlatches the
  // fault and restarts the consecutive-miss run, but KEEPS the cumulative
  // counters promised as "since node start" by RsiState. reset() (full
  // wipe) stays for tests/reconnect scenarios only.
  void clearFault();
```

头部注释第 4 行 `A latched fault is only cleared by reset()` 改为 `A latched fault is only cleared by clearFault()/reset()`。

`src/rsi_session_monitor.cpp` 中在 `reset()` 定义后加:

```cpp
void RsiSessionMonitor::clearFault() {
  stats_.fault = false;
  stats_.consecutive_timeouts = 0;
}
```

- [ ] **Step 4: 跑 monitor 测试确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | tail -3
./devel/lib/kuka_rsi_hw_interface/test_rsi_session_monitor
```

预期:10 tests PASS(8 存量 + 2 新增)。

- [ ] **Step 5: 写失败测试(RobotHW 层)**

在 `test/test_kuka_rsi_robot_hw.cpp` 末尾追加(复用文件内既有 `FakeKrc`/`testConfig`;`<atomic>` 无需 include,测试不直接触 atomics):

```cpp
TEST(RobotHW, ResetFaultKeepsCounters) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig(2)));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 1));
  hw.read(ros::Time(), ros::Duration(0.004));   // connected
  hw.read(ros::Time(), ros::Duration(0.004));   // timeout 1
  hw.read(ros::Time(), ros::Duration(0.004));   // timeout 2 -> fault
  ASSERT_TRUE(hw.faulted());
  ASSERT_EQ(hw.sessionStats().total_timeouts, 2u);

  hw.resetFault();  // now clearFault semantics: counters survive
  EXPECT_FALSE(hw.faulted());
  EXPECT_EQ(hw.sessionStats().total_timeouts, 2u);
}

TEST(RobotHW, RequestFaultClearAppliedOnRead) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig(2)));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 1));
  hw.read(ros::Time(), ros::Duration(0.004));
  hw.read(ros::Time(), ros::Duration(0.004));
  hw.read(ros::Time(), ros::Duration(0.004));
  ASSERT_TRUE(hw.faulted());

  hw.requestFaultClear();
  EXPECT_TRUE(hw.faulted());  // deferred: applied at the next read()

  ASSERT_TRUE(krc.sendState(hw.listenPort(), 42.0, 0.0, 2));
  hw.read(ros::Time(), ros::Duration(0.004));
  EXPECT_FALSE(hw.faulted());
  auto h = hw.get<kuka_rsi::CartesianStateInterface>()->getHandle("kuka_tcp");
  EXPECT_DOUBLE_EQ(h.getX(), 42.0);  // the same read ingests the frame
}
```

- [ ] **Step 6: 跑测试确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | tail -5
```

预期:编译失败,`no member named 'requestFaultClear'`。

- [ ] **Step 7: 实现 RobotHW 增量**

`include/kuka_rsi_hw_interface/kuka_rsi_robot_hw.h`:头部 include 区加 `#include <atomic>`;public 区把 `void resetFault() { monitor_.reset(); }` 替换为:

```cpp
  // Immediate variant (control-thread / test use): keeps cumulative
  // counters (Plan 4 follow-up 10).
  void resetFault() { monitor_.clearFault(); }
  // Thread-safe variant for the node's service callback: applied at the
  // start of the next read() so only the control thread touches monitor_.
  void requestFaultClear() { fault_clear_requested_.store(true); }
```

private 成员区(`saturation_count_` 之后)加:

```cpp
  std::atomic<bool> fault_clear_requested_{false};
```

`src/kuka_rsi_robot_hw.cpp` 的 `read()` 开头(`const int n = ...` 之前)加:

```cpp
  if (fault_clear_requested_.exchange(false)) monitor_.clearFault();
```

- [ ] **Step 8: 节点接线 `/kuka/rsi/reset_fault`**

`package.xml`:`<depend>roscpp</depend>` 后加 `<depend>std_srvs</depend>`。
`CMakeLists.txt`:`find_package(catkin REQUIRED COMPONENTS ...)` 与 `CATKIN_DEPENDS` 两处的 `roscpp` 后各加 `std_srvs`。

`src/kuka_rsi_hw_interface_node.cpp`:include 区加 `#include <std_srvs/Trigger.h>`;`ros::Publisher state_pub = ...` 之后加:

```cpp
  // Manager-facing recovery service (Plan 4 follow-up 10). The clear is
  // deferred to the next read() cycle; cumulative RsiState counters are
  // preserved (clearFault, not reset).
  ros::ServiceServer reset_srv =
      root_nh.advertiseService<std_srvs::Trigger::Request,
                               std_srvs::Trigger::Response>(
          "/kuka/rsi/reset_fault",
          [&hw](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
            hw.requestFaultClear();
            res.success = true;
            res.message = "fault clear requested; applied on next RSI read cycle";
            return true;
          });
```

- [ ] **Step 9: 全包回归确认通过**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | grep -ci warning   # expect 0
for t in test_rsi_session_monitor test_kuka_rsi_robot_hw test_rsi_integration; do
  ./devel/lib/kuka_rsi_hw_interface/$t || break
done
```

预期:三个二进制全绿(10 + 9 + 4;`test_rsi_integration` 的 `hw.resetFault()` 用例只断言 fault 翻转,不触计数器,语义变更兼容)。零警告。

- [ ] **Step 10: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_rsi_hw_interface
git commit -m "feat(rsi): clearFault keeps cumulative counters + /kuka/rsi/reset_fault service (Plan5 T1)"
```

**验收标准:** `clearFault()` 保留累计计数(新 2 用例);`resetFault()` 走 clearFault;`requestFaultClear()` 延迟应用且线程安全;服务接线;monitor 10 / robot_hw 9 全绿;零警告。

---

### Task 2: Plan 4 跟进小修 —— tare 误报 / connectNow 单次语义 / mock 固定端口

**目标:** 闭环 Plan 4 跟进 2、4、5:① `SriStreamSession::reset()` 清 `last_zero_ok_`,杜绝"捕获被重连打断→requestZero 误报成功"(N1);② `EkiBridgeRuntime::connectNow` 超时撤回请求位,`auto_reconnect=false` 不再蜕变持续重连(N2,决策 9);③ 两个 mock 可执行支持 `--port` 固定端口(sim.launch 依赖)并勘误 sri main 顶部注释。

**Files:**
- Modify: `ros_ws/src/sri_force_torque_driver/src/sri_stream_session.cpp`
- Modify: `ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_mock_server.h`
- Modify: `ros_ws/src/sri_force_torque_driver/src/sri_mock_server.cpp`
- Modify: `ros_ws/src/sri_force_torque_driver/src/sri_mock_server_main.cpp`
- Modify: `ros_ws/src/kuka_eki_bridge/src/eki_bridge_runtime.cpp`
- Modify: `ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_bridge_runtime.h`(connectNow 注释)
- Modify: `ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_mock_server.h`
- Modify: `ros_ws/src/kuka_eki_bridge/src/eki_mock_server.cpp`
- Modify: `ros_ws/src/kuka_eki_bridge/src/eki_mock_server_main.cpp`
- Test: `ros_ws/src/sri_force_torque_driver/test/test_sri_stream_session.cpp`(+1)
- Test: `ros_ws/src/sri_force_torque_driver/test/test_sri_mock_server.cpp`(+1)
- Test: `ros_ws/src/kuka_eki_bridge/test/test_eki_bridge_runtime.cpp`(+1)
- Test: `ros_ws/src/kuka_eki_bridge/test/test_eki_mock_server.cpp`(+1)

**Interfaces(Produces,Task 8 消费):**
- `SriMockConfig::listen_port`(`std::uint16_t`,默认 0=内核自选)/ `EkiMockConfig::listen_port` 同。
- `sri_mock_server --port N` / `eki_mock_server --port N`(0 或缺省 = 原行为)。
- `connectNow(timeout_ms)`:超时返回 false 且撤回请求(单次语义)。

**期望值推导:**
- tare 用例:zero_sample_count=2。捕获 #1 输入 fx=2.0、4.0 ⇒ bias.fx=(2+4)/2=3.0,lastZeroAccepted=true。捕获 #2 startZero 后只喂 1 帧(fx=2.0,累计中)即 reset() ⇒ zeroActive=false(捕获废弃)且 lastZeroAccepted **必须为 false**(修复点;修复前为 true=误报)。reset 保偏置:再喂 fx=5.0 ⇒ 输出 5.0−3.0=2.0(滤波 cutoff=0 直通;reset 后 have_last_pn_=false,PN=1 不计 gap)。
- connectNow 用例时序(全部有界):无服务器时 `connectNow(300)`→false(ECONNREFUSED 即时失败,重试循环于 300 ms 截止,撤回请求位);等 200 ms 排空撤回瞬间可能在途的最后一次尝试(connect_timeout_ms=100);再起 mock 于同端口;观察 400 ms:`auto_reconnect=false` 且请求已撤回 ⇒ `status().connected` 恒 false(修复前:请求位残留 ⇒ backoff 0.05 s 下 400 ms 内必然连上=失败态);最后显式 `connectNow(2000)`→true。
- 固定端口用例("探测-释放-再绑"法):先起临时 mock(port 0)取得内核分配端口 P 并 stop(listen socket 无连接即关,SO_REUSEADDR 下可立即复用);再以 `listen_port=P` 起正式 mock ⇒ `port()==P` 且 transport 可连。窗口内 P 被第三方抢占的概率可忽略(测试机无并发分配方),3 连跑验证稳定性。

- [ ] **Step 1: SRI 失败测试(session)**

`test/test_sri_stream_session.cpp` 末尾追加(复用文件内 `config()/frameFx()/feedOne()` 助手):

```cpp
TEST(SriSession, ResetClearsLastZeroAccepted) {
  // Plan 4 follow-up 4 (N1): a capture killed by a reconnect must not let
  // requestZero() report the PREVIOUS capture's success.
  SriStreamSession s;
  s.configure(config());  // zero_sample_count = 2
  SriWrenchSample out[8];
  s.startZero();
  feedOne(s, frameFx(2.0f, 1), kNow, out);
  feedOne(s, frameFx(4.0f, 2), kNow, out);
  ASSERT_TRUE(s.lastZeroAccepted());  // capture #1: bias.fx = 3.0

  s.startZero();                      // capture #2 begins...
  feedOne(s, frameFx(2.0f, 3), kNow, out);
  s.reset();                          // ...and dies with the connection
  EXPECT_FALSE(s.zeroActive());
  EXPECT_FALSE(s.lastZeroAccepted());  // must NOT echo capture #1

  // The old bias survives reset (physical sensor state unchanged).
  ASSERT_EQ(feedOne(s, frameFx(5.0f, 1), kNow, out), 1);
  EXPECT_NEAR(out[0].w.fx, 2.0, 1e-6);  // 5.0 - 3.0
}
```

- [ ] **Step 2: 确认失败(RED)**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | tail -3
./devel/lib/sri_force_torque_driver/test_sri_stream_session --gtest_filter='*ResetClearsLastZeroAccepted*'
```

预期:编译通过但断言失败于 `EXPECT_FALSE(s.lastZeroAccepted())`(现行 reset() 不清该位)。

- [ ] **Step 3: 修复 session(GREEN)**

`src/sri_stream_session.cpp` 的 `reset()` 改为:

```cpp
void SriStreamSession::reset() {
  assembler_.reset();
  filter_.reset();
  have_last_pn_ = false;
  zero_remaining_ = 0;    // an interrupted capture is abandoned, bias kept
  last_zero_ok_ = false;  // Plan 4 follow-up 4 (N1): never report the
                          // previous capture's success for this one
}
```

重跑 Step 2 命令:PASS;全 session 二进制 12/12。

- [ ] **Step 4: SRI mock 固定端口(RED→GREEN)**

`test/test_sri_mock_server.cpp` 末尾追加:

```cpp
TEST(SriMock, FixedListenPortIsHonoured) {
  // Probe-release-rebind: take a kernel-chosen free port, then ask the
  // mock to bind exactly it (--port plumbing for the Plan 5 sim launch).
  std::uint16_t port = 0;
  {
    SriMockServer probe{SriMockConfig{}};
    ASSERT_TRUE(probe.start());
    port = probe.port();
    probe.stop();
  }
  SriMockConfig cfg;
  cfg.listen_port = port;
  SriMockServer mock(cfg);
  ASSERT_TRUE(mock.start());
  EXPECT_EQ(mock.port(), port);
  TcpClientTransport t;
  EXPECT_TRUE(t.connect("127.0.0.1", port, 500));
  ASSERT_TRUE(mock.waitForClient(500));
  mock.stop();
}
```

RED:编译失败 `'struct sri::SriMockConfig' has no member named 'listen_port'`。

实现:`include/sri_force_torque_driver/sri_mock_server.h` 的 `SriMockConfig` 改为:

```cpp
struct SriMockConfig {
  bool require_start_command{true};  // hold frames until AT+GSD arrives
  double rate_hz{0.0};               // 0 = frames only via sendFrames()
  std::uint16_t listen_port{0};      // 0 = kernel-chosen (default; tests)
};
```

`src/sri_mock_server.cpp` `start()` 中 `addr.sin_port = 0;` 改为 `addr.sin_port = htons(cfg_.listen_port);`。GREEN:mock 二进制 7/7。

- [ ] **Step 5: `sri_mock_server --port` + 注释勘误**

`src/sri_mock_server_main.cpp`:
1. 顶部第 13-17 行的注释块(`// This binary needs a fixed port ...`)替换为:

```cpp
// Wraps the library mock for continuous waveforms; uses rate 0 + explicit
// sendFrames pacing so the waveform can evolve per frame. Listens on
// --port when given, else on a kernel-chosen port (both printed below).
// (Plan 4 follow-up 2: the old comment claimed a fixed port was required.)
```

2. `Options` 加 `int port = 0;`;usage 文本第二行改为:

```cpp
      "usage: sri_mock_server [--port N] [--rate HZ] [--fz N] [--sine-amp N]\n"
      "                       [--sine-hz HZ] [--bad-every N]\n"
      "Listens on 127.0.0.1:--port (0/default = kernel-chosen, printed).\n");
```

3. 参数解析链加 `else if (a == "--port") opt.port = static_cast<int>(v);`
4. `sri::SriMockConfig cfg;` 之后加 `cfg.listen_port = static_cast<std::uint16_t>(opt.port);`

手动验证(非门槛):`./devel/lib/sri_force_torque_driver/sri_mock_server --port 34008 --fz 5 &`,`ss -tln | grep 34008` 命中后 kill,确认零残留。

- [ ] **Step 6: EKI 失败测试(connectNow 单次语义)**

`test/test_eki_bridge_runtime.cpp` 末尾追加(复用文件内 `config()/waitFor()`):

```cpp
TEST(EkiRuntime, ConnectNowTimeoutIsSingleShot) {
  // Plan 4 follow-up 5 (N2) ruling: a failed manual connect must not arm
  // an endless background retry loop when auto_reconnect is off.
  std::uint16_t port = 0;
  {
    EkiMockServer probe{EkiMockConfig{}};
    ASSERT_TRUE(probe.start());
    port = probe.port();
    probe.stop();
  }
  EkiBridgeConfig c = config(port);
  c.auto_reconnect = false;
  c.connect_timeout_ms = 100;
  EkiBridgeRuntime rt{c};
  ASSERT_TRUE(rt.start());

  EXPECT_FALSE(rt.connectNow(300));  // nobody listening: bounded failure
  // Drain the one attempt that may still be in flight at withdrawal time.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EkiMockConfig mock_cfg;
  mock_cfg.listen_port = port;
  EkiMockServer mock{mock_cfg};
  ASSERT_TRUE(mock.start());
  // Withdrawn request + auto_reconnect=false: must NOT connect by itself.
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_FALSE(rt.status().connected);

  EXPECT_TRUE(rt.connectNow(2000));  // an explicit new request connects
  rt.stop();
  mock.stop();
}
```

RED:编译失败(`EkiMockConfig` 无 `listen_port`);补上 Step 7 后再跑,断言失败于 `EXPECT_FALSE(rt.status().connected)`(修复前后台重试已连上)。

- [ ] **Step 7: EKI mock 固定端口**

`include/kuka_eki_bridge/eki_mock_server.h` 的 `EkiMockConfig` 改为:

```cpp
struct EkiMockConfig {
  double heartbeat_period_s{0.0};  // 0 = heartbeats only via pushHeartbeat()
  std::uint16_t listen_port{0};    // 0 = kernel-chosen (default; tests)
};
```

`src/eki_mock_server.cpp` `start()` 中 `addr.sin_port = 0;` 改为 `addr.sin_port = htons(cfg_.listen_port);`。

`test/test_eki_mock_server.cpp` 末尾追加(与 Step 4 同构,改用 `kuka_eki::TcpClientTransport`):

```cpp
TEST(EkiMock, FixedListenPortIsHonoured) {
  std::uint16_t port = 0;
  {
    EkiMockServer probe{EkiMockConfig{}};
    ASSERT_TRUE(probe.start());
    port = probe.port();
    probe.stop();
  }
  EkiMockConfig cfg;
  cfg.listen_port = port;
  EkiMockServer mock(cfg);
  ASSERT_TRUE(mock.start());
  EXPECT_EQ(mock.port(), port);
  TcpClientTransport t;
  EXPECT_TRUE(t.connect("127.0.0.1", port, 500));
  ASSERT_TRUE(mock.waitForClient(500));
  mock.stop();
}
```

`src/eki_mock_server_main.cpp`:usage 改为

```cpp
      "usage: eki_mock_server [--port N] [--heartbeat-ms N] [--start-faulted]\n"
      "Listens on 127.0.0.1:--port (0/default = kernel-chosen, printed).\n");
```

参数环加:

```cpp
    } else if (a == "--port" && i + 1 < argc) {
      cfg.listen_port = static_cast<std::uint16_t>(std::atoi(argv[++i]));
```

- [ ] **Step 8: 修复 connectNow(GREEN)**

`src/eki_bridge_runtime.cpp` 的 `connectNow()` 改为:

```cpp
bool EkiBridgeRuntime::connectNow(int timeout_ms) {
  std::unique_lock<std::mutex> lock(mutex_);
  connect_requested_ = true;
  cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
               [this] { return connected_; });
  // Plan 4 follow-up 5 (N2) ruling: the manual request is single-shot.
  // Withdraw it on timeout so auto_reconnect=false deployments never
  // degenerate into an endless background retry loop. One attempt armed
  // just before withdrawal (<= connect_timeout_ms) may still land; that
  // window is accepted and documented.
  if (!connected_) connect_requested_ = false;
  return connected_;
}
```

`include/kuka_eki_bridge/eki_bridge_runtime.h` 中 `connectNow` 的注释块替换为:

```cpp
  // Waits (bounded) until the link is up. With auto_reconnect the io
  // thread connects on its own; without it this arms a single-shot
  // request that is withdrawn again on timeout (Plan 4 follow-up 5).
```

- [ ] **Step 9: 双包回归 + 3 连跑**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | grep -ci warning   # 0
for i in 1 2 3; do
  ./devel/lib/sri_force_torque_driver/test_sri_stream_session && \
  ./devel/lib/sri_force_torque_driver/test_sri_mock_server && \
  ./devel/lib/kuka_eki_bridge/test_eki_mock_server && \
  ./devel/lib/kuka_eki_bridge/test_eki_bridge_runtime || break
done
pgrep -af 'mock_server' ; ss -tln | grep -E '4008|54600'   # both empty
```

预期:session 12 / sri mock 7 / eki mock 9 / eki runtime 9,3 连跑全绿,零残留。

- [ ] **Step 10: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/sri_force_torque_driver ros_ws/src/kuka_eki_bridge
git commit -m "fix(sri,eki): tare-interrupt misreport, single-shot connectNow, mock --port (Plan5 T2)"
```

**验收标准:** 4 个新用例落位并 GREEN;`--port` 两处可执行手动验证;注释勘误完成;两包 10 个二进制全绿、3 连跑无 flaky、零残留、零警告。

---

### Task 3: `soft_robot_msgs` 增量(ManagerState / StartServo / CalibratePayload)

**目标:** 声明式新增 manager 面的消息:`ManagerState.msg`(`/soft_robot/manager_state`)、`StartServo.srv`(`/soft_robot/start_servo`)、`CalibratePayload.action`(`/soft_robot/calibrate_payload`)。数值约定:`system_state` 复用 `ModeState.SYSTEM_*`(0~6,规格 §11 顺序),`StartServo` 的 mode/profile 复用 `ModeCommand` wire 值(经 Plan 3 `mode_bridge.h` static_assert 与 sfc 枚举钉死)。

**Files:**
- Create: `ros_ws/src/soft_robot_msgs/msg/ManagerState.msg`
- Create: `ros_ws/src/soft_robot_msgs/srv/StartServo.srv`
- Create: `ros_ws/src/soft_robot_msgs/action/CalibratePayload.action`
- Modify: `ros_ws/src/soft_robot_msgs/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_msgs/test/test_msgs.cpp`(+2)

**Interfaces(Produces,Task 4~8 与 Plan 6 消费):**
- `ManagerState`:header + system_state/mode/profile + 6 个健康布尔 + tool_synced + calibrating + active_controller(string)。
- `StartServo`:请求 mode(仅 DIRECT_CARTESIAN=1 / FORCE_COMPLIANCE=2 合法,manager 校验)+ profile;响应 success/message。
- `CalibratePayload`:goal 空(参数全在 calibration.yaml);feedback pose_index/pose_count/phase(string);result success/message/gravity_n/com_x/com_y/com_z/bias_fx..tz/r2_force/r2_torque(与 `sfc::PayloadFitResult` 逐字段对应)。

- [ ] **Step 1: 写三个定义文件**

`ros_ws/src/soft_robot_msgs/msg/ManagerState.msg`:

```text
# System-level manager state (spec sections 5.5, 11) published by
# soft_robot_manager on /soft_robot/manager_state. system_state uses the
# ModeState.SYSTEM_* constants (single source for the state numbering).
Header header
uint8 system_state          # ModeState.SYSTEM_* value
uint8 mode                  # active/target ModeCommand.MODE_* while servoing
uint8 profile               # ModeCommand.PROFILE_* selected for servo
bool eki_connected          # /kuka/eki/state: connected && state_fresh
bool eki_program_ready      # KRL program online and ready
bool rsi_connected          # /kuka/rsi/state fresh && connected
bool rsi_fault              # latched RSI communication fault
bool sri_streaming          # /sri_ft/status fresh && streaming
bool tool_synced            # $TOOL read through EKI since (re)connect
bool calibrating            # payload calibration action running
string active_controller    # running controller name; "" when none
```

`ros_ws/src/soft_robot_msgs/srv/StartServo.srv`:

```text
# Start servo operation (spec section 11 SERVOING preconditions checked by
# the manager). mode/profile use the ModeCommand constants; only
# MODE_DIRECT_CARTESIAN and MODE_FORCE_COMPLIANCE are accepted.
uint8 mode
uint8 profile
---
bool success
string message
```

`ros_ws/src/soft_robot_msgs/action/CalibratePayload.action`:

```text
# Payload calibration workflow (spec section 9), orchestrated by the
# manager: drive the calibration poses in goal mode, average wrench
# samples per pose, least-squares fit, persist payload.yaml.
# Poses/settle/sample counts come from calibration.yaml (empty goal).
---
bool success
string message              # failure phase on abort
float64 gravity_n           # fitted payload weight [N]
float64 com_x               # center of mass [m], sensor frame
float64 com_y
float64 com_z
float64 bias_fx             # fitted sensor zero bias [N / Nm]
float64 bias_fy
float64 bias_fz
float64 bias_tx
float64 bias_ty
float64 bias_tz
float64 r2_force            # fit quality (legacy R1*R2 equivalent)
float64 r2_torque
---
uint32 pose_index           # 0-based pose currently being processed
uint32 pose_count
string phase                # MOVING | SETTLING | SAMPLING | SOLVING | RETURNING
```

- [ ] **Step 2: 注册进 CMakeLists 并写失败测试**

`CMakeLists.txt`:`add_message_files` 的 `EkiState.msg` 后加 `ManagerState.msg`;`add_service_files` 的 `SetToolBase.srv` 后加 `StartServo.srv`;`add_action_files` 的 `MoveToOrientation.action` 后加 `CalibratePayload.action`。

`test/test_msgs.cpp`:include 区加

```cpp
#include <soft_robot_msgs/CalibratePayloadAction.h>
#include <soft_robot_msgs/ManagerState.h>
#include <soft_robot_msgs/StartServo.h>
```

文件末尾追加:

```cpp
// The manager republishes the spec-11 state machine through ManagerState;
// the numbering source stays ModeState.SYSTEM_* (no duplicated constants).
TEST(Msgs, ManagerStateDefaultsToOfflineShape) {
  soft_robot_msgs::ManagerState m;
  EXPECT_EQ(m.system_state, soft_robot_msgs::ModeState::SYSTEM_OFFLINE);
  EXPECT_FALSE(m.eki_connected);
  EXPECT_FALSE(m.rsi_connected);
  EXPECT_FALSE(m.sri_streaming);
  EXPECT_FALSE(m.tool_synced);
  EXPECT_FALSE(m.calibrating);
  EXPECT_TRUE(m.active_controller.empty());
}

TEST(Msgs, CalibratePayloadResultCarriesFullFit) {
  soft_robot_msgs::CalibratePayloadResult r;
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.gravity_n, 0.0);
  EXPECT_EQ(r.bias_tz, 0.0);
  EXPECT_EQ(r.r2_force, 0.0);
  soft_robot_msgs::CalibratePayloadFeedback f;
  EXPECT_EQ(f.pose_index, 0u);
  EXPECT_TRUE(f.phase.empty());
  soft_robot_msgs::StartServo::Request req;
  EXPECT_EQ(req.mode, 0u);
  soft_robot_msgs::StartServo::Response resp;
  EXPECT_FALSE(resp.success);
}
```

- [ ] **Step 3: RED→GREEN**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | tail -3
```

未注册前编译失败(header 不存在)——注册 CMakeLists 后重跑:

```bash
./devel/lib/soft_robot_msgs/test_msgs
```

预期:8 tests PASS(6 存量 + 2 新增)。

- [ ] **Step 4: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_robot_msgs
git commit -m "feat(msgs): ManagerState + StartServo + CalibratePayload action (Plan5 T3)"
```

**验收标准:** 三个新定义可编译生成;`test_msgs` 8/8;既有定义零改动(diff 只有 + 行)。

---

### Task 4: `SystemStateCore`(系统状态机,纯逻辑)+ 包骨架

**目标:** 规格 §11 状态机的纯逻辑实现:输入为健康快照(EKI/RSI/SRI/tool/控制器)与显式命令(start/stop/reset/标定开始结束),输出系统状态与命令裁决。无 ROS、无锁、无分配,时钟由调用者以 double 秒注入(仅用于事件说明,不内部计时——所有超时都在 runtime 层)。建包 `soft_force_control_manager`。

**Files:**
- Create: `ros_ws/src/soft_force_control_manager/package.xml`
- Create: `ros_ws/src/soft_force_control_manager/CMakeLists.txt`
- Create: `ros_ws/src/soft_force_control_manager/include/soft_force_control_manager/system_state_core.h`
- Create: `ros_ws/src/soft_force_control_manager/src/system_state_core.cpp`
- Test: `ros_ws/src/soft_force_control_manager/test/test_system_state_core.cpp`(目标名 `test_manager_system_state`)

**Interfaces:**
- Consumes:`soft_robot_msgs/ModeState` 的 SYSTEM_* 数值约定(核心用自有 enum,数值一致,节点侧 static_assert 对齐——同 Plan 3 `mode_bridge.h` 手法)。
- Produces(Task 6/7 消费):

```cpp
namespace sfm {
enum class SystemState : std::uint8_t {
  OFFLINE = 0, CONNECTED = 1, READY = 2, SERVOING = 3,
  CALIBRATING = 4, DEGRADED = 5, FAULT = 6,
};
struct HealthInputs {   // all freshness pre-judged by the runtime layer
  bool eki_link{false};        // connected && state_fresh
  bool eki_program_ready{false};
  bool eki_fault{false};       // KRC-side latched fault
  bool rsi_topic_fresh{false}; // /kuka/rsi/state age within threshold
  bool rsi_connected{false};
  bool rsi_fault{false};
  bool sri_streaming{false};   // status fresh && streaming
  bool tool_synced{false};
  bool controllers_loaded{false};
};
struct Verdict { bool accepted{false}; const char* reason{""}; };
class SystemStateCore {
 public:
  SystemState update(const HealthInputs& in);   // periodic re-evaluation
  Verdict requestStart(const HealthInputs& in); // -> SERVOING
  Verdict requestStop();                        // SERVOING|DEGRADED -> READY-track
  Verdict requestCalibration(const HealthInputs& in); // READY -> CALIBRATING
  void calibrationFinished();                   // CALIBRATING -> re-evaluate
  Verdict requestReset();                       // FAULT -> re-evaluate
  bool allowZeroSensor() const;                 // CONNECTED|READY only
  SystemState state() const { return state_; }
 private:
  SystemState state_{SystemState::OFFLINE};
  bool servo_requested_{false};
  bool calibrating_{false};
};
}  // namespace sfm
```

**状态迁移规则(决策 2 的精确化,实现与测试共同依据):**

```text
update(in) 优先级自上而下:
 1. eki_fault || rsi_fault                        -> FAULT (latched;
    servo_requested_/calibrating_ 同时清零)
 2. state==FAULT (已闭锁)                          -> FAULT (仅 requestReset 出)
 3. calibrating_:
      in.eki_link && in.rsi_topic_fresh && in.rsi_connected && in.sri_streaming
        ? CALIBRATING : FAULT (标定中链路死亡按致命处理,闭锁)
 4. servo_requested_:
      full = eki_link && program_ready && rsi_topic_fresh && rsi_connected
             && sri_streaming && tool_synced && controllers_loaded
      full ? SERVOING : DEGRADED (自动恢复;servo_requested_ 保持)
 5. 否则(空闲阶梯):
      !eki_link                        -> OFFLINE
      eki_link && !ready_for_servo     -> CONNECTED
      eki_link && program_ready && tool_synced && sri_streaming
        && controllers_loaded          -> READY   (RSI 帧流不做 READY 前置,决策 2)
requestStart(in): state==READY 且 full(见上,rsi 除外——start 编排先启 RSI 再置
  servo_requested_,故此处要求除 rsi_connected 外全真;rsi_topic_fresh 仍必须)
  ? {servo_requested_=true; accepted} : {reason}
requestStop(): SERVOING|DEGRADED ? {servo_requested_=false} : reject
requestCalibration(in): state==READY && sri_streaming && rsi_topic_fresh
  ? {calibrating_=true} : reject
calibrationFinished(): calibrating_=false (状态由下一次 update 重算)
requestReset(): state==FAULT ? accepted (state 改走空闲阶梯重算) : reject
allowZeroSensor(): state==CONNECTED || state==READY (决策 11)
```

**期望值推导(代表性链,测试逐步断言):**
- 空闲阶梯:全 false → OFFLINE;仅 eki_link → CONNECTED;eki_link+ready+tool+sri+ctrl → READY(注意 rsi 全 false 仍 READY——决策 2)。
- servo 链:READY 输入下 `requestStart` 需 rsi_topic_fresh=true 才 accepted;置位后 update(全真)→ SERVOING;拔 sri_streaming → DEGRADED;恢复 → SERVOING(servo_requested_ 未清);requestStop → 下一 update 回 READY。
- fault 链:SERVOING 中 rsi_fault=true → FAULT;此后全真输入仍 FAULT(闭锁);requestReset accepted → update(全真、rsi_fault=false)→ READY(servo_requested_ 已被 FAULT 清零,不回 SERVOING)。
- 标定链:READY → requestCalibration accepted → update → CALIBRATING;calibrationFinished → update → READY;标定中拔 sri_streaming → update → FAULT。
- allowZeroSensor:OFFLINE false / CONNECTED true / READY true / SERVOING false / CALIBRATING false / FAULT false。

- [ ] **Step 1: 包骨架**

`package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>soft_force_control_manager</name>
  <version>0.1.0</version>
  <description>
    Non-realtime system manager (spec sections 5.5, 9, 11): system state
    machine, EKI/RSI/SRI health aggregation, controller switching
    orchestration, payload calibration workflow, payload.yaml persistence,
    and the operator-facing service/action surface consumed by the web UI.
  </description>
  <maintainer email="dev@example.com">ljj</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>std_srvs</depend>
  <depend>actionlib</depend>
  <depend>diagnostic_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>controller_manager_msgs</depend>
  <depend>soft_robot_msgs</depend>
  <depend>soft_force_control_core</depend>
</package>
```

`CMakeLists.txt`(初版,后续 Task 增量扩展):

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(soft_force_control_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  actionlib
  diagnostic_msgs
  geometry_msgs
  controller_manager_msgs
  soft_robot_msgs
  soft_force_control_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES soft_force_control_manager_core
  CATKIN_DEPENDS roscpp std_srvs actionlib diagnostic_msgs geometry_msgs
                 controller_manager_msgs soft_robot_msgs
                 soft_force_control_core
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS})

# Pure-logic manager core: no roscpp runtime dependency.
add_library(soft_force_control_manager_core
  src/system_state_core.cpp
)
target_link_libraries(soft_force_control_manager_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # Package-prefixed gtest target names (global CMake namespace under
  # catkin_make; Plan 4 follow-up 1).
  catkin_add_gtest(test_manager_system_state test/test_system_state_core.cpp)
  target_link_libraries(test_manager_system_state
                        soft_force_control_manager_core
                        ${GTEST_MAIN_LIBRARIES})
endif()
```

- [ ] **Step 2: 写失败测试**

`test/test_system_state_core.cpp`:

```cpp
#include <gtest/gtest.h>

#include "soft_force_control_manager/system_state_core.h"

using sfm::HealthInputs;
using sfm::SystemState;
using sfm::SystemStateCore;

namespace {

HealthInputs offline() { return HealthInputs{}; }

HealthInputs connectedOnly() {
  HealthInputs in;
  in.eki_link = true;
  return in;
}

// Everything a READY verdict needs; RSI deliberately down (decision 2:
// the KRC does not stream RSI before START_RSI, READY must not need it).
HealthInputs ready() {
  HealthInputs in;
  in.eki_link = true;
  in.eki_program_ready = true;
  in.sri_streaming = true;
  in.tool_synced = true;
  in.controllers_loaded = true;
  return in;
}

HealthInputs full() {
  HealthInputs in = ready();
  in.rsi_topic_fresh = true;
  in.rsi_connected = true;
  return in;
}

}  // namespace

TEST(SystemState, IdleLadderFollowsHealth) {
  SystemStateCore c;
  EXPECT_EQ(c.update(offline()), SystemState::OFFLINE);
  EXPECT_EQ(c.update(connectedOnly()), SystemState::CONNECTED);
  EXPECT_EQ(c.update(ready()), SystemState::READY);   // no RSI needed
  EXPECT_EQ(c.update(connectedOnly()), SystemState::CONNECTED);  // back down
  EXPECT_EQ(c.update(offline()), SystemState::OFFLINE);
}

TEST(SystemState, StartRequiresFreshRsiTopic) {
  SystemStateCore c;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  EXPECT_FALSE(c.requestStart(ready()).accepted);  // rsi_topic_fresh false

  HealthInputs in = ready();
  in.rsi_topic_fresh = true;  // hw node alive; RSI frames come after START_RSI
  EXPECT_TRUE(c.requestStart(in).accepted);
  EXPECT_EQ(c.update(full()), SystemState::SERVOING);
}

TEST(SystemState, ServoDegradesAndRecovers) {
  SystemStateCore c;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  ASSERT_TRUE(c.requestStart(start_in).accepted);
  ASSERT_EQ(c.update(full()), SystemState::SERVOING);

  HealthInputs degraded = full();
  degraded.sri_streaming = false;
  EXPECT_EQ(c.update(degraded), SystemState::DEGRADED);
  EXPECT_EQ(c.update(full()), SystemState::SERVOING);  // auto-recovery

  EXPECT_TRUE(c.requestStop().accepted);
  EXPECT_EQ(c.update(full()), SystemState::READY);
}

TEST(SystemState, FaultLatchesUntilReset) {
  SystemStateCore c;
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  ASSERT_TRUE(c.requestStart(start_in).accepted);
  ASSERT_EQ(c.update(full()), SystemState::SERVOING);

  HealthInputs faulted = full();
  faulted.rsi_fault = true;
  EXPECT_EQ(c.update(faulted), SystemState::FAULT);
  EXPECT_EQ(c.update(full()), SystemState::FAULT);  // latched
  EXPECT_FALSE(c.requestStart(full()).accepted);
  EXPECT_FALSE(c.requestStop().accepted);

  EXPECT_TRUE(c.requestReset().accepted);
  // servo_requested_ was cleared by the fault: back to READY, not SERVOING.
  EXPECT_EQ(c.update(full()), SystemState::READY);
}

TEST(SystemState, CalibrationLifecycleAndLinkLoss) {
  SystemStateCore c;
  ASSERT_EQ(c.update(full()), SystemState::READY);
  EXPECT_FALSE(c.requestCalibration(ready()).accepted);  // rsi topic stale
  EXPECT_TRUE(c.requestCalibration(full()).accepted);
  EXPECT_EQ(c.update(full()), SystemState::CALIBRATING);

  c.calibrationFinished();
  EXPECT_EQ(c.update(full()), SystemState::READY);

  ASSERT_TRUE(c.requestCalibration(full()).accepted);
  ASSERT_EQ(c.update(full()), SystemState::CALIBRATING);
  HealthInputs lost = full();
  lost.sri_streaming = false;
  EXPECT_EQ(c.update(lost), SystemState::FAULT);  // fatal during calibration
  EXPECT_EQ(c.update(full()), SystemState::FAULT);
}

TEST(SystemState, CalibrationRejectedOutsideReady) {
  SystemStateCore c;
  EXPECT_FALSE(c.requestCalibration(full()).accepted);  // OFFLINE
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  ASSERT_TRUE(c.requestStart(start_in).accepted);
  ASSERT_EQ(c.update(full()), SystemState::SERVOING);
  EXPECT_FALSE(c.requestCalibration(full()).accepted);  // SERVOING
}

TEST(SystemState, ZeroSensorGateFollowsState) {
  SystemStateCore c;
  EXPECT_FALSE(c.allowZeroSensor());                    // OFFLINE
  c.update(connectedOnly());
  EXPECT_TRUE(c.allowZeroSensor());                     // CONNECTED
  c.update(ready());
  EXPECT_TRUE(c.allowZeroSensor());                     // READY
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  c.requestStart(start_in);
  c.update(full());
  EXPECT_FALSE(c.allowZeroSensor());                    // SERVOING (dec. 11)
}

TEST(SystemState, RejectionsCarryReasons) {
  SystemStateCore c;
  const sfm::Verdict v = c.requestStart(offline());
  EXPECT_FALSE(v.accepted);
  EXPECT_STRNE(v.reason, "");
}
```

- [ ] **Step 3: 确认 RED**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | tail -3
```

预期:编译失败(头文件不存在)。

- [ ] **Step 4: 实现**

`include/soft_force_control_manager/system_state_core.h`:

```cpp
#pragma once

#include <cstdint>

namespace sfm {

// System states, spec section 11. Values equal the wire constants
// soft_robot_msgs::ModeState::SYSTEM_* (static_assert in the node shell).
enum class SystemState : std::uint8_t {
  OFFLINE = 0,
  CONNECTED = 1,
  READY = 2,
  SERVOING = 3,
  CALIBRATING = 4,
  DEGRADED = 5,
  FAULT = 6,
};

// Health snapshot with all freshness ALREADY judged by the runtime layer
// (message age vs threshold); the core stays clock-free and testable.
struct HealthInputs {
  bool eki_link{false};         // TCP up && RobotState heartbeat fresh
  bool eki_program_ready{false};
  bool eki_fault{false};        // KRC-side latched fault
  bool rsi_topic_fresh{false};  // /kuka/rsi/state age within threshold
  bool rsi_connected{false};    // last RsiState.connected
  bool rsi_fault{false};        // last RsiState.fault
  bool sri_streaming{false};    // /sri_ft/status fresh && streaming
  bool tool_synced{false};      // $TOOL read through EKI since (re)connect
  bool controllers_loaded{false};
};

struct Verdict {
  bool accepted{false};
  const char* reason{""};  // static strings only (no allocation)
};

// Spec-11 state machine (decision 2 details). Pure logic: no ROS, no
// clock, no allocation. update() re-evaluates the state from the health
// snapshot; explicit operator commands arrive through the request*()
// methods. FAULT latches until requestReset(). READY deliberately does
// NOT require RSI frames (the KRC only streams after START_RSI); RSI
// validity is enforced by the start orchestration in the runtime.
class SystemStateCore {
 public:
  SystemState update(const HealthInputs& in);
  Verdict requestStart(const HealthInputs& in);
  Verdict requestStop();
  Verdict requestCalibration(const HealthInputs& in);
  void calibrationFinished();
  Verdict requestReset();
  bool allowZeroSensor() const;  // decision 11: CONNECTED/READY only
  SystemState state() const { return state_; }

 private:
  static bool readyConditions(const HealthInputs& in);
  static bool fullConditions(const HealthInputs& in);
  SystemState state_{SystemState::OFFLINE};
  bool servo_requested_{false};
  bool calibrating_{false};
};

}  // namespace sfm
```

`src/system_state_core.cpp`:

```cpp
#include "soft_force_control_manager/system_state_core.h"

namespace sfm {

bool SystemStateCore::readyConditions(const HealthInputs& in) {
  return in.eki_link && in.eki_program_ready && in.sri_streaming &&
         in.tool_synced && in.controllers_loaded;
}

bool SystemStateCore::fullConditions(const HealthInputs& in) {
  return readyConditions(in) && in.rsi_topic_fresh && in.rsi_connected;
}

SystemState SystemStateCore::update(const HealthInputs& in) {
  if (in.eki_fault || in.rsi_fault) {
    state_ = SystemState::FAULT;
    servo_requested_ = false;  // a fault always demands an explicit restart
    calibrating_ = false;
    return state_;
  }
  if (state_ == SystemState::FAULT) return state_;  // latched

  if (calibrating_) {
    // Link loss mid-calibration is fatal (the robot may be far from the
    // start pose with a half-written estimate): latch FAULT.
    const bool link_ok = in.eki_link && in.rsi_topic_fresh &&
                         in.rsi_connected && in.sri_streaming;
    state_ = link_ok ? SystemState::CALIBRATING : SystemState::FAULT;
    if (state_ == SystemState::FAULT) calibrating_ = false;
    return state_;
  }

  if (servo_requested_) {
    state_ = fullConditions(in) ? SystemState::SERVOING
                                : SystemState::DEGRADED;
    return state_;
  }

  if (!in.eki_link) {
    state_ = SystemState::OFFLINE;
  } else if (readyConditions(in)) {
    state_ = SystemState::READY;
  } else {
    state_ = SystemState::CONNECTED;
  }
  return state_;
}

Verdict SystemStateCore::requestStart(const HealthInputs& in) {
  if (state_ != SystemState::READY)
    return {false, "start requires READY"};
  // rsi_connected is NOT required here: the start orchestration issues
  // START_RSI first and waits for frames before flipping servo_requested_.
  if (!in.rsi_topic_fresh)
    return {false, "RSI hw node state topic is stale"};
  if (!readyConditions(in))
    return {false, "READY preconditions lost"};
  servo_requested_ = true;
  return {true, ""};
}

Verdict SystemStateCore::requestStop() {
  if (state_ != SystemState::SERVOING && state_ != SystemState::DEGRADED)
    return {false, "stop requires SERVOING or DEGRADED"};
  servo_requested_ = false;
  return {true, ""};
}

Verdict SystemStateCore::requestCalibration(const HealthInputs& in) {
  if (state_ != SystemState::READY)
    return {false, "calibration requires READY"};
  if (!in.rsi_topic_fresh || !in.rsi_connected)
    return {false, "RSI link not valid"};
  if (!in.sri_streaming)
    return {false, "SRI not streaming"};
  calibrating_ = true;
  return {true, ""};
}

void SystemStateCore::calibrationFinished() { calibrating_ = false; }

Verdict SystemStateCore::requestReset() {
  if (state_ != SystemState::FAULT)
    return {false, "reset requires FAULT"};
  state_ = SystemState::OFFLINE;  // re-evaluated by the next update()
  return {true, ""};
}

bool SystemStateCore::allowZeroSensor() const {
  return state_ == SystemState::CONNECTED || state_ == SystemState::READY;
}

}  // namespace sfm
```

注意 `CalibrationLifecycleAndLinkLoss` 用例的一个隐含点:`requestCalibration(full())` 在 `update(full())` 尚处 READY 时调用才被接受——测试序列已按此排列。

- [ ] **Step 5: 确认 GREEN**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | grep -ci warning   # 0
./devel/lib/soft_force_control_manager/test_manager_system_state
```

预期:8 tests PASS。

- [ ] **Step 6: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_force_control_manager
git commit -m "feat(manager): SystemStateCore spec-11 state machine + package skeleton (Plan5 T4)"
```

**验收标准:** 包骨架成型;状态机 8 用例覆盖空闲阶梯/servo 降级恢复/fault 闭锁/标定生命周期/zero 门禁/拒绝理由;纯逻辑无 ros 头;零警告。

---

### Task 5: `CalibrationSequencer` + `payload_yaml`(标定序列与持久化,纯逻辑)

**目标:** 规格 §9 标定流程的纯逻辑实现:姿态序列推进(MOVING→SETTLING→SAMPLING 循环)、每姿态 wrench 均值采样(§9 3d 的改进点)、`sfc::PayloadEstimator` 最小二乘、成功后回程(RETURNING→DONE);以及 `payload.yaml` 文本生成器。事件驱动 + 时钟注入,无 ROS、无线程;runtime(Task 6)负责把 action 结果 / wrench 话题 / 健康丢失翻译成本类事件。

**Files:**
- Create: `ros_ws/src/soft_force_control_manager/include/soft_force_control_manager/calibration_sequencer.h`
- Create: `ros_ws/src/soft_force_control_manager/src/calibration_sequencer.cpp`
- Create: `ros_ws/src/soft_force_control_manager/include/soft_force_control_manager/payload_yaml.h`
- Create: `ros_ws/src/soft_force_control_manager/src/payload_yaml.cpp`
- Modify: `ros_ws/src/soft_force_control_manager/CMakeLists.txt`(库源 + 2 测试目标 + find_package(Eigen3) 供测试 synth)
- Test: `ros_ws/src/soft_force_control_manager/test/test_calibration_sequencer.cpp`(目标 `test_manager_calibration_sequencer`)
- Test: `ros_ws/src/soft_force_control_manager/test/test_payload_yaml.cpp`(目标 `test_manager_payload_yaml`)

**Interfaces:**
- Consumes:`sfc::PayloadEstimator`(`addSample(a,b,c,mean)` / `clear()` / `solve()`,n≥4 才 ok)、`sfc::Wrench`、`sfc::PayloadFitResult`。
- Produces(Task 6/7 消费):

```cpp
namespace sfm {
struct CalPose { double a{0}, b{0}, c{0}; };            // deg
struct CalibrationConfig {
  std::vector<CalPose> poses;      // calibration.yaml pose sequence
  CalPose return_pose;             // default A=B=C=0 (spec 9 step 5)
  double settle_time_s{1.0};
  int samples_per_pose{100};
};
enum class CalPhase { IDLE, MOVING, SETTLING, SAMPLING, RETURNING, DONE, FAILED };
enum class CalFailure { NONE, MOVE_FAILED, SOLVE_FAILED, STREAM_LOST, CANCELLED };
struct CalAction { bool send_goal{false}; CalPose target; };
struct CalStatus { CalPhase phase; CalFailure failure; std::size_t pose_index;
                   std::size_t pose_count; int samples_collected;
                   bool return_move_ok; };
class CalibrationSequencer {
  void configure(const CalibrationConfig&);
  bool start(double now_s);                      // false: bad config / running
  void cancel();                                 // -> FAILED(CANCELLED)
  void onMotionResult(bool success, double now_s);
  void onWrench(const sfc::Wrench& w);           // consumed only in SAMPLING
  void onStreamLost();                           // -> FAILED(STREAM_LOST)
  CalAction tick(double now_s);                  // emits each goal exactly once
  CalStatus status() const;
  const sfc::PayloadFitResult& result() const;
};
std::string emitPayloadYaml(const sfc::PayloadFitResult& fit,
                            const std::string& timestamp);
}
```

**推进规则(实现与测试共同依据):**

```text
start: poses 非空 && samples_per_pose>0 && phase ∈ {IDLE,DONE,FAILED};
       estimator.clear(), fit={}, idx=0, phase=MOVING, goal_pending=true
tick(now):
  SETTLING && now-settle_start >= settle_time -> SAMPLING (count=0, accum=0)
  goal_pending -> 发射一次 send_goal(MOVING: poses[idx]; RETURNING: return_pose)
onMotionResult(ok, now):
  MOVING:    ok ? {SETTLING, settle_start=now} : FAILED(MOVE_FAILED)
  RETURNING: return_move_ok=ok; DONE(回程失败不作废标定,拟合结果保留)
onWrench(w) @SAMPLING: accum+=w; ++count;
  count==samples_per_pose -> mean=accum/count; estimator.addSample(pose, mean);
    idx+1<N ? {++idx, MOVING, goal_pending} :
    { fit=solve(); fit.ok ? {RETURNING, goal_pending} : FAILED(SOLVE_FAILED) }
onStreamLost/cancel @{MOVING,SETTLING,SAMPLING,RETURNING} -> FAILED(...)
FAILED 一律 goal_pending=false(决策 6:失败不回程、不再发目标)
```

**期望值推导:**
- 快乐路径用 Plan 1 `test_payload_estimator.cpp` 的已验证数据集:8 姿态 `kPoses`、G=50 N、com=(0.01,0.02,0.03) m、bias=(1,−2,0.5,0.1,−0.2,0.05),synth(w)=bias+重力项(公式与 `ToolGravityCompensator::compensate` 逆向一致)。samples_per_pose=2,两样本取 **synth±0.5(fx)**:mean=(synth+0.5+synth−0.5)/2=synth **精确**,故均值输入与 Plan 1 精确回收用例逐字节同源 ⇒ `solve()` 回收 G=50、com、bias 至 1e-8,r2=1 至 1e-9(Plan 1 已证)。若实现错拿"最后一个样本"而非均值,输入含 −0.5 偏移 ⇒ r2_force<1、G 偏离 >1e-8,断言必失败——该构造同时钉死"确实在求均值"。
- 相位走查(settle=1.0):start(t=0) → tick 发 pose0;onMotionResult(true, t=10) → SETTLING;tick(10.5) 无动作仍 SETTLING;tick(11.0):11.0−10.0=1.0 ≥ 1.0 → SAMPLING。
- SOLVE_FAILED 路径:2 姿态配置跑完采样 ⇒ estimator n=2 <4 ⇒ `solve().ok=false` ⇒ FAILED(SOLVE_FAILED)(Plan 1 `NotOkWithTooFewSamples` 已证 n=2 不可解)。
- yaml 文本:固定 fit 值 + `%.6f` 格式 ⇒ 逐字节期望串(见测试)。

- [ ] **Step 1: 写失败测试(sequencer)**

`test/test_calibration_sequencer.cpp`:

```cpp
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "soft_force_control_core/rotation.h"
#include "soft_force_control_manager/calibration_sequencer.h"

using sfm::CalAction;
using sfm::CalFailure;
using sfm::CalibrationConfig;
using sfm::CalibrationSequencer;
using sfm::CalPhase;
using sfm::CalPose;
using sfc::Wrench;

namespace {

// Same synthetic model as the Plan 1 estimator test: raw = bias + gravity.
Wrench synth(double G, const Eigen::Vector3d& com, const Wrench& bias,
             double a, double b, double c) {
  const Eigen::Matrix3d r = sfc::kukaAbcToRotation(a, b, c);
  const Eigen::Vector3d f = r.transpose() * Eigen::Vector3d(0, 0, -G);
  const Eigen::Vector3d t = com.cross(f);
  Wrench w;
  w.fx = bias.fx + f.x();
  w.fy = bias.fy + f.y();
  w.fz = bias.fz + f.z();
  w.tx = bias.tx + t.x();
  w.ty = bias.ty + t.y();
  w.tz = bias.tz + t.z();
  return w;
}

const double kPoses[8][3] = {{0, 0, 0},    {0, 45, 0},  {0, -45, 0},
                             {0, 0, 45},   {0, 0, -45}, {45, 30, 0},
                             {-45, 0, 30}, {30, -30, 30}};

CalibrationConfig config8() {
  CalibrationConfig c;
  for (const auto& p : kPoses) c.poses.push_back(CalPose{p[0], p[1], p[2]});
  c.settle_time_s = 1.0;
  c.samples_per_pose = 2;
  return c;  // return_pose defaults to A=B=C=0
}

// Drives MOVING -> SETTLING -> SAMPLING for the current pose and asserts
// the emitted goal matches. Returns the goal for further checks.
CalPose passMoveAndSettle(CalibrationSequencer& s, double& now) {
  CalAction act = s.tick(now);
  EXPECT_TRUE(act.send_goal);
  s.onMotionResult(true, now);
  EXPECT_EQ(s.status().phase, CalPhase::SETTLING);
  now += 1.0;  // == settle_time_s
  EXPECT_FALSE(s.tick(now).send_goal);
  EXPECT_EQ(s.status().phase, CalPhase::SAMPLING);
  return act.target;
}

}  // namespace

TEST(CalibrationSequencer, HappyPathRecoversPayloadExactly) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.02, 0.03);
  Wrench bias;
  bias.fx = 1.0;
  bias.fy = -2.0;
  bias.fz = 0.5;
  bias.tx = 0.1;
  bias.ty = -0.2;
  bias.tz = 0.05;

  CalibrationSequencer s;
  s.configure(config8());
  double now = 0.0;
  ASSERT_TRUE(s.start(now));
  ASSERT_EQ(s.status().pose_count, 8u);

  for (int i = 0; i < 8; ++i) {
    const CalPose goal = passMoveAndSettle(s, now);
    EXPECT_DOUBLE_EQ(goal.a, kPoses[i][0]);
    EXPECT_DOUBLE_EQ(goal.b, kPoses[i][1]);
    // Two samples at synth +/- 0.5 on fx: their mean is exactly synth, so
    // the fit must match the Plan 1 exact-recovery case. A last-sample
    // (non-averaging) bug would leave a -0.5 residual and fail below.
    Wrench w1 = synth(G, com, bias, goal.a, goal.b, goal.c);
    Wrench w2 = w1;
    w1.fx += 0.5;
    w2.fx -= 0.5;
    s.onWrench(w1);
    EXPECT_EQ(s.status().samples_collected, 1);
    s.onWrench(w2);
  }

  ASSERT_EQ(s.status().phase, CalPhase::RETURNING);
  const CalAction ret = s.tick(now);
  ASSERT_TRUE(ret.send_goal);
  EXPECT_DOUBLE_EQ(ret.target.a, 0.0);
  EXPECT_DOUBLE_EQ(ret.target.b, 0.0);
  EXPECT_DOUBLE_EQ(ret.target.c, 0.0);
  s.onMotionResult(true, now);
  ASSERT_EQ(s.status().phase, CalPhase::DONE);
  EXPECT_TRUE(s.status().return_move_ok);

  const sfc::PayloadFitResult& r = s.result();
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.params.gravity_n, G, 1e-8);
  EXPECT_NEAR(r.params.com_x, com.x(), 1e-8);
  EXPECT_NEAR(r.params.com_y, com.y(), 1e-8);
  EXPECT_NEAR(r.params.com_z, com.z(), 1e-8);
  EXPECT_NEAR(r.params.bias.fx, bias.fx, 1e-8);
  EXPECT_NEAR(r.params.bias.tz, bias.tz, 1e-8);
  EXPECT_NEAR(r.r2_force, 1.0, 1e-9);
  EXPECT_NEAR(r.r2_torque, 1.0, 1e-9);
}

TEST(CalibrationSequencer, GoalIsEmittedExactlyOnce) {
  CalibrationSequencer s;
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  EXPECT_TRUE(s.tick(0.0).send_goal);
  EXPECT_FALSE(s.tick(0.1).send_goal);  // not re-emitted while MOVING
}

TEST(CalibrationSequencer, SettleWindowIsRespected) {
  CalibrationSequencer s;
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  s.tick(0.0);
  s.onMotionResult(true, 10.0);
  s.tick(10.5);
  EXPECT_EQ(s.status().phase, CalPhase::SETTLING);  // 0.5 < 1.0
  s.tick(11.0);
  EXPECT_EQ(s.status().phase, CalPhase::SAMPLING);  // 1.0 >= 1.0
}

TEST(CalibrationSequencer, MoveFailureAbortsWithoutReturnGoal) {
  CalibrationSequencer s;
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  s.tick(0.0);
  s.onMotionResult(false, 1.0);
  EXPECT_EQ(s.status().phase, CalPhase::FAILED);
  EXPECT_EQ(s.status().failure, CalFailure::MOVE_FAILED);
  EXPECT_FALSE(s.tick(2.0).send_goal);  // decision 6: no further motion
}

TEST(CalibrationSequencer, TooFewPosesFailsAtSolve) {
  CalibrationConfig cfg;
  cfg.poses = {CalPose{0, 0, 0}, CalPose{0, 45, 0}};  // n=2 < 4: unsolvable
  cfg.settle_time_s = 0.0;
  cfg.samples_per_pose = 1;
  CalibrationSequencer s;
  s.configure(cfg);
  double now = 0.0;
  ASSERT_TRUE(s.start(now));
  for (int i = 0; i < 2; ++i) {
    ASSERT_TRUE(s.tick(now).send_goal);
    s.onMotionResult(true, now);
    s.tick(now);  // settle_time 0: straight to SAMPLING
    s.onWrench(Wrench{});
  }
  EXPECT_EQ(s.status().phase, CalPhase::FAILED);
  EXPECT_EQ(s.status().failure, CalFailure::SOLVE_FAILED);
}

TEST(CalibrationSequencer, StreamLossAndCancelAbort) {
  CalibrationSequencer s;
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  s.tick(0.0);
  s.onStreamLost();
  EXPECT_EQ(s.status().failure, CalFailure::STREAM_LOST);

  ASSERT_TRUE(s.start(0.0));  // restart after failure is allowed
  s.tick(0.0);
  s.cancel();
  EXPECT_EQ(s.status().failure, CalFailure::CANCELLED);
  EXPECT_TRUE(s.start(0.0));  // restartable again
}

TEST(CalibrationSequencer, ReturnMoveFailureKeepsFit) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.0, 0.05);
  CalibrationSequencer s;
  s.configure(config8());
  double now = 0.0;
  ASSERT_TRUE(s.start(now));
  for (int i = 0; i < 8; ++i) {
    const CalPose goal = passMoveAndSettle(s, now);
    const Wrench w = synth(G, com, Wrench{}, goal.a, goal.b, goal.c);
    s.onWrench(w);
    s.onWrench(w);
  }
  ASSERT_EQ(s.status().phase, CalPhase::RETURNING);
  s.tick(now);
  s.onMotionResult(false, now);  // return move failed
  EXPECT_EQ(s.status().phase, CalPhase::DONE);  // fit is NOT discarded
  EXPECT_FALSE(s.status().return_move_ok);
  EXPECT_TRUE(s.result().ok);
  EXPECT_NEAR(s.result().params.gravity_n, G, 1e-8);
}

TEST(CalibrationSequencer, StartRejectsBadConfigAndDoubleStart) {
  CalibrationSequencer s;
  s.configure(CalibrationConfig{});  // no poses
  EXPECT_FALSE(s.start(0.0));
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  EXPECT_FALSE(s.start(0.0));  // already running
}
```

- [ ] **Step 2: 写失败测试(payload_yaml)**

`test/test_payload_yaml.cpp`:

```cpp
#include <gtest/gtest.h>

#include <string>

#include "soft_force_control_manager/payload_yaml.h"

TEST(PayloadYaml, EmitsControllerOverrideDocument) {
  sfc::PayloadFitResult fit;
  fit.ok = true;
  fit.params.gravity_n = 50.0;
  fit.params.com_x = 0.01;
  fit.params.com_y = 0.02;
  fit.params.com_z = 0.03;
  fit.params.bias.fx = 1.0;
  fit.params.bias.fy = -2.0;
  fit.params.bias.fz = 0.5;
  fit.params.bias.tx = 0.1;
  fit.params.bias.ty = -0.2;
  fit.params.bias.tz = 0.05;
  fit.r2_force = 1.0;
  fit.r2_torque = 0.999;

  const std::string expected =
      "# payload.yaml - written by soft_force_control_manager (spec 9).\n"
      "# Loaded by soft_robot_bringup at startup to override the\n"
      "# force_compliance_controller payload block. Do not edit by hand.\n"
      "force_compliance_controller:\n"
      "  payload:\n"
      "    gravity_n: 50.000000\n"
      "    com_x: 0.010000\n"
      "    com_y: 0.020000\n"
      "    com_z: 0.030000\n"
      "    bias_fx: 1.000000\n"
      "    bias_fy: -2.000000\n"
      "    bias_fz: 0.500000\n"
      "    bias_tx: 0.100000\n"
      "    bias_ty: -0.200000\n"
      "    bias_tz: 0.050000\n"
      "soft_robot_manager:\n"
      "  payload_fit:\n"
      "    r2_force: 1.000000\n"
      "    r2_torque: 0.999000\n"
      "    timestamp: \"2026-07-03T12:00:00\"\n";
  EXPECT_EQ(sfm::emitPayloadYaml(fit, "2026-07-03T12:00:00"), expected);
}
```

- [ ] **Step 3: 确认 RED**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | tail -3
```

预期:编译失败(两个头文件不存在;CMake 增量见 Step 4)。

- [ ] **Step 4: 实现**

`CMakeLists.txt`:`find_package(catkin ...)` 后加 `find_package(Eigen3 REQUIRED)`;`include_directories` 加 `${EIGEN3_INCLUDE_DIRS}`;库源加 `src/calibration_sequencer.cpp`、`src/payload_yaml.cpp`;测试区加:

```cmake
  catkin_add_gtest(test_manager_calibration_sequencer
                   test/test_calibration_sequencer.cpp)
  target_link_libraries(test_manager_calibration_sequencer
                        soft_force_control_manager_core
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_manager_payload_yaml test/test_payload_yaml.cpp)
  target_link_libraries(test_manager_payload_yaml
                        soft_force_control_manager_core
                        ${GTEST_MAIN_LIBRARIES})
```

`include/soft_force_control_manager/calibration_sequencer.h`:

```cpp
#pragma once

#include <cstddef>
#include <vector>

#include "soft_force_control_core/payload_estimator.h"
#include "soft_force_control_core/types.h"

namespace sfm {

struct CalPose {
  double a{0}, b{0}, c{0};  // deg (KUKA A/B/C = Z-Y-X Euler)
};

struct CalibrationConfig {
  std::vector<CalPose> poses;   // calibration.yaml sequence (legacy 8-set)
  CalPose return_pose;          // spec 9 step 5; default A=B=C=0
  double settle_time_s{1.0};
  int samples_per_pose{100};
};

enum class CalPhase { IDLE, MOVING, SETTLING, SAMPLING, RETURNING, DONE,
                      FAILED };
enum class CalFailure { NONE, MOVE_FAILED, SOLVE_FAILED, STREAM_LOST,
                        CANCELLED };

// tick() output: at most one goal emission per pose (decision 5: the
// estimator uses the goal pose, not a robot readback).
struct CalAction {
  bool send_goal{false};
  CalPose target;
};

struct CalStatus {
  CalPhase phase{CalPhase::IDLE};
  CalFailure failure{CalFailure::NONE};
  std::size_t pose_index{0};
  std::size_t pose_count{0};
  int samples_collected{0};
  bool return_move_ok{true};
};

// Spec-9 calibration workflow as a pure event-driven sequencer: the
// runtime feeds motion results, wrench samples, and stream-loss events;
// tick() advances settle timing and emits goals. A failure never emits
// further goals (decision 6: no return motion on abort); a failed return
// move does NOT discard the fit (the estimate is already solved). No ROS,
// no threads; the runtime serializes access.
class CalibrationSequencer {
 public:
  void configure(const CalibrationConfig& cfg) { cfg_ = cfg; }
  bool start(double now_s);
  void cancel() { fail(CalFailure::CANCELLED); }
  void onMotionResult(bool success, double now_s);
  void onWrench(const sfc::Wrench& w);
  void onStreamLost() { fail(CalFailure::STREAM_LOST); }
  CalAction tick(double now_s);
  CalStatus status() const;
  const sfc::PayloadFitResult& result() const { return fit_; }

 private:
  bool active() const;
  void fail(CalFailure f);
  CalibrationConfig cfg_;
  CalPhase phase_{CalPhase::IDLE};
  CalFailure failure_{CalFailure::NONE};
  std::size_t idx_{0};
  bool goal_pending_{false};
  double settle_start_s_{0};
  int sample_count_{0};
  sfc::Wrench accum_;
  bool return_ok_{true};
  sfc::PayloadEstimator estimator_;
  sfc::PayloadFitResult fit_;
};

}  // namespace sfm
```

`src/calibration_sequencer.cpp`:

```cpp
#include "soft_force_control_manager/calibration_sequencer.h"

namespace sfm {

bool CalibrationSequencer::active() const {
  return phase_ == CalPhase::MOVING || phase_ == CalPhase::SETTLING ||
         phase_ == CalPhase::SAMPLING || phase_ == CalPhase::RETURNING;
}

void CalibrationSequencer::fail(CalFailure f) {
  if (!active()) return;
  phase_ = CalPhase::FAILED;
  failure_ = f;
  goal_pending_ = false;
}

bool CalibrationSequencer::start(double /*now_s*/) {
  if (active()) return false;
  if (cfg_.poses.empty() || cfg_.samples_per_pose <= 0) return false;
  estimator_.clear();
  fit_ = sfc::PayloadFitResult{};
  failure_ = CalFailure::NONE;
  idx_ = 0;
  sample_count_ = 0;
  accum_ = sfc::Wrench{};
  return_ok_ = true;
  phase_ = CalPhase::MOVING;
  goal_pending_ = true;
  return true;
}

CalAction CalibrationSequencer::tick(double now_s) {
  CalAction act;
  if (phase_ == CalPhase::SETTLING &&
      now_s - settle_start_s_ >= cfg_.settle_time_s) {
    phase_ = CalPhase::SAMPLING;
    sample_count_ = 0;
    accum_ = sfc::Wrench{};
  }
  if (goal_pending_) {
    act.send_goal = true;
    act.target = phase_ == CalPhase::RETURNING ? cfg_.return_pose
                                               : cfg_.poses[idx_];
    goal_pending_ = false;
  }
  return act;
}

void CalibrationSequencer::onMotionResult(bool success, double now_s) {
  if (phase_ == CalPhase::MOVING) {
    if (!success) {
      fail(CalFailure::MOVE_FAILED);
      return;
    }
    phase_ = CalPhase::SETTLING;
    settle_start_s_ = now_s;
  } else if (phase_ == CalPhase::RETURNING) {
    return_ok_ = success;   // a failed return move keeps the solved fit
    phase_ = CalPhase::DONE;
  }
}

void CalibrationSequencer::onWrench(const sfc::Wrench& w) {
  if (phase_ != CalPhase::SAMPLING) return;
  accum_.fx += w.fx;
  accum_.fy += w.fy;
  accum_.fz += w.fz;
  accum_.tx += w.tx;
  accum_.ty += w.ty;
  accum_.tz += w.tz;
  if (++sample_count_ < cfg_.samples_per_pose) return;

  const double n = static_cast<double>(cfg_.samples_per_pose);
  sfc::Wrench mean;
  mean.fx = accum_.fx / n;
  mean.fy = accum_.fy / n;
  mean.fz = accum_.fz / n;
  mean.tx = accum_.tx / n;
  mean.ty = accum_.ty / n;
  mean.tz = accum_.tz / n;
  const CalPose& p = cfg_.poses[idx_];
  estimator_.addSample(p.a, p.b, p.c, mean);

  if (idx_ + 1 < cfg_.poses.size()) {
    ++idx_;
    phase_ = CalPhase::MOVING;
    goal_pending_ = true;
    return;
  }
  fit_ = estimator_.solve();
  if (!fit_.ok) {
    fail(CalFailure::SOLVE_FAILED);
    return;
  }
  phase_ = CalPhase::RETURNING;
  goal_pending_ = true;
}

CalStatus CalibrationSequencer::status() const {
  CalStatus s;
  s.phase = phase_;
  s.failure = failure_;
  s.pose_index = idx_;
  s.pose_count = cfg_.poses.size();
  s.samples_collected = sample_count_;
  s.return_move_ok = return_ok_;
  return s;
}

}  // namespace sfm
```

`include/soft_force_control_manager/payload_yaml.h`:

```cpp
#pragma once

#include <string>

#include "soft_force_control_core/payload_estimator.h"

namespace sfm {

// Renders payload.yaml (spec sections 9, 14): a rosparam-loadable override
// of the force_compliance_controller payload block plus fit metadata.
// Pure text generation; the caller owns file I/O and the timestamp.
std::string emitPayloadYaml(const sfc::PayloadFitResult& fit,
                            const std::string& timestamp);

}  // namespace sfm
```

`src/payload_yaml.cpp`:

```cpp
#include "soft_force_control_manager/payload_yaml.h"

#include <cstdio>

namespace sfm {

namespace {
void line(std::string& out, const char* key, double v) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "    %s: %.6f\n", key, v);
  out += buf;
}
}  // namespace

std::string emitPayloadYaml(const sfc::PayloadFitResult& fit,
                            const std::string& timestamp) {
  std::string out;
  out.reserve(768);
  out +=
      "# payload.yaml - written by soft_force_control_manager (spec 9).\n"
      "# Loaded by soft_robot_bringup at startup to override the\n"
      "# force_compliance_controller payload block. Do not edit by hand.\n"
      "force_compliance_controller:\n"
      "  payload:\n";
  line(out, "gravity_n", fit.params.gravity_n);
  line(out, "com_x", fit.params.com_x);
  line(out, "com_y", fit.params.com_y);
  line(out, "com_z", fit.params.com_z);
  line(out, "bias_fx", fit.params.bias.fx);
  line(out, "bias_fy", fit.params.bias.fy);
  line(out, "bias_fz", fit.params.bias.fz);
  line(out, "bias_tx", fit.params.bias.tx);
  line(out, "bias_ty", fit.params.bias.ty);
  line(out, "bias_tz", fit.params.bias.tz);
  out += "soft_robot_manager:\n  payload_fit:\n";
  line(out, "r2_force", fit.r2_force);
  line(out, "r2_torque", fit.r2_torque);
  out += "    timestamp: \"" + timestamp + "\"\n";
  return out;
}

}  // namespace sfm
```

- [ ] **Step 5: 确认 GREEN**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | grep -ci warning   # 0
./devel/lib/soft_force_control_manager/test_manager_calibration_sequencer
./devel/lib/soft_force_control_manager/test_manager_payload_yaml
```

预期:sequencer 8 / yaml 1 全 PASS。

- [ ] **Step 6: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_force_control_manager
git commit -m "feat(manager): CalibrationSequencer + payload.yaml emitter (Plan5 T5)"
```

**验收标准:** 快乐路径精确回收(证均值采样)、目标只发一次、settle 时窗、四种失败路径、回程失败保留拟合、重启语义,共 8 用例;yaml 逐字节 1 用例;纯逻辑无 ros 头;零警告。

---

### Task 6: `ManagerRuntime`(worker 线程 + ManagerOps 注入)+ lambda mock 闭环测试

**目标:** 组合 Task 4/5 的两个核心:worker 线程周期评估健康→状态机、边沿触发 tool 同步、编排 start/stop/reset/zero 命令序列、驱动标定序列并在结束时持久化+切回。对外部世界的全部动作经 `ManagerOps`(std::function 束)注入——gtest 用 lambda mock 闭环,不碰 ROS。时钟 steady_clock double 秒(同 Plan 4 runtime);服务方法经 `command_mutex_` 串行、数据经 `mutex_`,耗时 ops 调用一律在数据锁外。

**Files:**
- Create: `ros_ws/src/soft_force_control_manager/include/soft_force_control_manager/manager_runtime.h`
- Create: `ros_ws/src/soft_force_control_manager/src/manager_runtime.cpp`
- Modify: `ros_ws/src/soft_force_control_manager/CMakeLists.txt`(库源 + 测试目标 + pthread)
- Test: `ros_ws/src/soft_force_control_manager/test/test_manager_runtime.cpp`(目标 `test_manager_runtime`)

**Interfaces:**
- Consumes:`sfm::SystemStateCore`(Task 4 全部签名)、`sfm::CalibrationSequencer`/`emitPayloadYaml`(Task 5)、`sfc::Wrench`。
- Produces(Task 7 节点消费,签名逐字):

```cpp
namespace sfm {
struct ToolFrame { double x{0},y{0},z{0},a{0},b{0},c{0}; bool valid{false}; };
struct EkiFeed { bool connected{false}; bool state_fresh{false};
                 bool program_ready{false}; bool rsi_active{false};
                 bool fault{false}; };
struct ManagerOps {   // all callbacks optional (null = no-op/false)
  std::function<bool()> ekiStartRsi, ekiStopRsi, ekiResetFault;
  std::function<bool()> rsiResetFault, sriZero;
  std::function<ToolFrame()> ekiGetTool;
  std::function<bool(const std::string& start_ctrl,
                     const std::string& stop_ctrl)> switchControllers;
  std::function<void(std::uint8_t mode, std::uint8_t profile)> publishMode;
  std::function<void(const CalPose& target)> sendMotionGoal;
  std::function<bool(const std::string& yaml_text,
                     const sfc::PayloadFitResult& fit)> applyPayload;
  std::function<void()> publishState;   // worker-tick hook (decision 13)
};
struct ManagerConfig {
  double tick_period_s{0.1};
  double eki_state_timeout_s{5.0};   // decision 3 (N3 tolerance)
  double rsi_state_timeout_s{0.5};
  double sri_status_timeout_s{2.0};
  double tool_sync_retry_s{2.0};
  double rsi_connect_wait_s{5.0};    // decision 4 bounded wait
  std::string compliance_controller{"force_compliance_controller"};
  std::string correction_controller{"cartesian_correction_controller"};
  CalibrationConfig calibration;
};
struct CommandResult { bool success{false}; std::string message; };
struct ManagerSnapshot {
  SystemState state{SystemState::OFFLINE};
  HealthInputs health;
  std::uint8_t mode{0}, profile{0};
  std::string active_controller;
  bool calibrating{false};
  CalStatus cal;
  sfc::PayloadFitResult fit;
};
class ManagerRuntime {
  ManagerRuntime(const ManagerConfig&, ManagerOps);
  bool start(); void stop();
  void setControllersLoaded(bool);                    // node: after load_controller
  void feedEkiState(const EkiFeed&);                  // subscriber threads
  void feedRsiState(bool connected, bool fault);
  void feedSriStatus(bool streaming);
  void feedWrench(const sfc::Wrench&);
  void onMotionResult(bool success);                  // action-client done cb
  CommandResult startServo(std::uint8_t mode, std::uint8_t profile);
  CommandResult stopServo();
  CommandResult resetFault();
  CommandResult zeroSensor();
  CommandResult beginCalibration();
  void cancelCalibration();
  ManagerSnapshot snapshot() const;
};
}
```

**编排序列(决策 4/6/7/11 的精确化,mock 顺序断言依据):**

```text
startServo(mode, profile), mode ∈ {1,2}:
  1 state==READY? else fail
  2 ops.ekiStartRsi()  (rsi_active 已真则跳过)      失败 -> fail
  3 有界等待 rsi_topic_fresh && rsi_connected (<= rsi_connect_wait_s)
                                                    超时 -> fail(留 READY,不回滚 EKI)
  4 ops.publishMode(IDLE, profile)
  5 ops.switchControllers(target_ctrl, other_ctrl)  失败 -> fail
      target = mode==2 ? compliance : correction
  6 ops.publishMode(mode, profile)
  7 core.requestStart(health)                        拒绝 -> fail
  8 active_mode/profile/controller 记账
stopServo (SERVOING|DEGRADED):
  ops.publishMode(IDLE, profile) -> ops.switchControllers("", active)
  -> ops.ekiStopRsi() -> core.requestStop()
resetFault (FAULT): ops.rsiResetFault() + ops.ekiResetFault()(都调,失败并入
  message 但不阻断)-> core.requestReset()
zeroSensor: core.allowZeroSensor() ? ops.sriZero() : reject(决策 11)
beginCalibration (READY):
  core.requestCalibration(health) -> ops.publishMode(IDLE,profile)
  -> ops.switchControllers(correction, compliance) -> ops.publishMode(CALIBRATION,profile)
  -> seq.start(now)
worker tick:
  健康快照(age vs 阈值)-> core.update;eki_link 断沿清 tool_synced
  tool 同步边沿:eki_link && program_ready && !tool_synced
    && now-last_try >= tool_sync_retry_s -> ops.ekiGetTool()(锁外)
  标定中:sri 失鲜 -> seq.onStreamLost;act=seq.tick(now);act.send_goal
    -> ops.sendMotionGoal(锁外)
  seq DONE/FAILED 边沿(一次性):DONE 且 fit.ok -> ops.applyPayload(emitPayloadYaml)
    -> 两分支同做:ops.publishMode(IDLE) + ops.switchControllers("", correction)
    -> core.calibrationFinished()
  ops.publishState()
```

**期望值推导:**
- 顺序断言 = 上表逐行(mock 把每次调用压入 `std::vector<std::string> log`,持锁)。startServo(FORCE_COMPLIANCE=2, DRAG=0) 期望 log 片段:`start_rsi`,`mode:0/0`,`switch:force_compliance_controller/cartesian_correction_controller`,`mode:2/0`。
- 标定闭环用 Task 5 同一 8 姿态合成数据(samples_per_pose=1,均值=单样本=synth 精确)⇒ `applyPayload` 收到的 fit.gravity_n≈50(1e-8)、yaml 文本含 `gravity_n: 50.000000`;teardown 后 state==READY、`calibrating=false`。
- 新鲜度:测试用 `sri_status_timeout_s=0.1`,单次 feed 后 ≥0.1 s 不喂 ⇒ DEGRADED(SERVOING 中)。tick_period_s=0.01 保证 0.2 s 内多次评估。
- 全部等待经 `waitFor(pred, ms)`(≤2 s),无裸 sleep 断言(排空型 sleep 除外)。

- [ ] **Step 1: 写失败测试**

`test/test_manager_runtime.cpp`(完整文件):

```cpp
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "soft_force_control_core/rotation.h"
#include "soft_force_control_manager/manager_runtime.h"

using sfm::CalPose;
using sfm::CommandResult;
using sfm::EkiFeed;
using sfm::ManagerConfig;
using sfm::ManagerOps;
using sfm::ManagerRuntime;
using sfm::SystemState;
using sfm::ToolFrame;
using sfc::Wrench;

namespace {

bool waitFor(const std::function<bool()>& pred, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

Wrench synth(double G, const Eigen::Vector3d& com, const Wrench& bias,
             double a, double b, double c) {
  const Eigen::Matrix3d r = sfc::kukaAbcToRotation(a, b, c);
  const Eigen::Vector3d f = r.transpose() * Eigen::Vector3d(0, 0, -G);
  const Eigen::Vector3d t = com.cross(f);
  Wrench w;
  w.fx = bias.fx + f.x();
  w.fy = bias.fy + f.y();
  w.fz = bias.fz + f.z();
  w.tx = bias.tx + t.x();
  w.ty = bias.ty + t.y();
  w.tz = bias.tz + t.z();
  return w;
}

const double kPoses[8][3] = {{0, 0, 0},    {0, 45, 0},  {0, -45, 0},
                             {0, 0, 45},   {0, 0, -45}, {45, 30, 0},
                             {-45, 0, 30}, {30, -30, 30}};

// Records every ops call (thread-safe) and lets tests script results.
struct MockOps {
  std::mutex m;
  std::vector<std::string> log;
  int get_tool_calls = 0;
  int get_tool_fail_first = 0;   // fail this many getTool calls
  bool start_rsi_ok = true;
  bool switch_ok = true;
  CalPose last_goal;
  int goals_sent = 0;
  std::string payload_yaml;
  sfc::PayloadFitResult payload_fit;
  int payload_applied = 0;
  ManagerRuntime* rt = nullptr;   // for feeding back from ekiStartRsi

  void push(const std::string& s) {
    std::lock_guard<std::mutex> lock(m);
    log.push_back(s);
  }
  ManagerOps ops() {
    ManagerOps o;
    o.ekiStartRsi = [this] {
      push("start_rsi");
      if (start_rsi_ok && rt) rt->feedRsiState(true, false);
      return start_rsi_ok;
    };
    o.ekiStopRsi = [this] { push("stop_rsi"); return true; };
    o.ekiResetFault = [this] { push("eki_reset"); return true; };
    o.rsiResetFault = [this] { push("rsi_reset"); return true; };
    o.sriZero = [this] { push("sri_zero"); return true; };
    o.ekiGetTool = [this]() -> ToolFrame {
      ToolFrame t;
      std::lock_guard<std::mutex> lock(m);
      ++get_tool_calls;
      if (get_tool_fail_first > 0) { --get_tool_fail_first; return t; }
      t.x = 10.5; t.z = 235.0; t.valid = true;
      return t;
    };
    o.switchControllers = [this](const std::string& start,
                                 const std::string& stop) {
      push("switch:" + start + "/" + stop);
      return switch_ok;
    };
    o.publishMode = [this](std::uint8_t mode, std::uint8_t profile) {
      push("mode:" + std::to_string(mode) + "/" + std::to_string(profile));
    };
    o.sendMotionGoal = [this](const CalPose& p) {
      std::lock_guard<std::mutex> lock(m);
      last_goal = p;
      ++goals_sent;
    };
    o.applyPayload = [this](const std::string& yaml,
                            const sfc::PayloadFitResult& fit) {
      std::lock_guard<std::mutex> lock(m);
      payload_yaml = yaml;
      payload_fit = fit;
      ++payload_applied;
      return true;
    };
    return o;
  }
  int goalsSent() { std::lock_guard<std::mutex> l(m); return goals_sent; }
  CalPose lastGoal() { std::lock_guard<std::mutex> l(m); return last_goal; }
  bool logged(const std::string& s) {
    std::lock_guard<std::mutex> lock(m);
    for (const auto& e : log) if (e == s) return true;
    return false;
  }
};

ManagerConfig fastConfig() {
  ManagerConfig c;
  c.tick_period_s = 0.01;
  c.eki_state_timeout_s = 5.0;
  c.rsi_state_timeout_s = 5.0;    // freshness not under test unless shrunk
  c.sri_status_timeout_s = 5.0;
  c.tool_sync_retry_s = 0.05;
  c.rsi_connect_wait_s = 0.5;
  for (const auto& p : kPoses)
    c.calibration.poses.push_back(CalPose{p[0], p[1], p[2]});
  c.calibration.settle_time_s = 0.0;
  c.calibration.samples_per_pose = 1;
  return c;
}

EkiFeed ekiUp() {
  EkiFeed f;
  f.connected = true;
  f.state_fresh = true;
  f.program_ready = true;
  return f;
}

// Brings a started runtime to READY (tool sync happens on the way).
void driveToReady(ManagerRuntime& rt) {
  rt.setControllersLoaded(true);
  rt.feedEkiState(ekiUp());
  rt.feedSriStatus(true);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
}

void driveToServoing(ManagerRuntime& rt, MockOps& mock) {
  driveToReady(rt);
  const CommandResult r = rt.startServo(2, 0);  // FORCE_COMPLIANCE + DRAG
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));
  (void)mock;
}

}  // namespace

TEST(ManagerRuntime, IdleLadderReachesReadyAndSyncsToolOnce) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  EXPECT_EQ(rt.snapshot().state, SystemState::OFFLINE);

  driveToReady(rt);
  EXPECT_TRUE(rt.snapshot().health.tool_synced);
  // Synced: no further getTool retries.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::lock_guard<std::mutex> lock(mock.m);
  EXPECT_EQ(mock.get_tool_calls, 1);
  rt.stop();
}

TEST(ManagerRuntime, ToolSyncRetriesUntilSuccess) {
  MockOps mock;
  mock.get_tool_fail_first = 2;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);  // READY implies tool_synced eventually true
  std::lock_guard<std::mutex> lock(mock.m);
  EXPECT_EQ(mock.get_tool_calls, 3);  // 2 failures + 1 success
  rt.stop();
}

TEST(ManagerRuntime, StartServoRunsOrchestrationInOrder) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToServoing(rt, mock);

  // Exact command sequence (decision 4).
  std::vector<std::string> cmd;
  {
    std::lock_guard<std::mutex> lock(mock.m);
    for (const auto& e : mock.log)
      if (e == "start_rsi" || e.rfind("mode:", 0) == 0 ||
          e.rfind("switch:", 0) == 0)
        cmd.push_back(e);
  }
  const std::vector<std::string> expected = {
      "start_rsi", "mode:0/0",
      "switch:force_compliance_controller/cartesian_correction_controller",
      "mode:2/0"};
  EXPECT_EQ(cmd, expected);
  EXPECT_EQ(rt.snapshot().active_controller, "force_compliance_controller");
  EXPECT_EQ(rt.snapshot().mode, 2u);
  rt.stop();
}

TEST(ManagerRuntime, StartServoFailsWhenRsiNeverConnects) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  // mock.rt left null: ekiStartRsi succeeds but no RSI feed ever arrives.
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  const CommandResult r = rt.startServo(2, 0);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(rt.snapshot().state, SystemState::READY);  // stays READY
  EXPECT_FALSE(mock.logged(
      "switch:force_compliance_controller/cartesian_correction_controller"));
  rt.stop();
}

TEST(ManagerRuntime, StartServoRejectsBadModeAndNotReady) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  EXPECT_FALSE(rt.startServo(2, 0).success);   // OFFLINE
  driveToReady(rt);
  EXPECT_FALSE(rt.startServo(0, 0).success);   // IDLE is not a servo mode
  EXPECT_FALSE(rt.startServo(3, 0).success);   // CALIBRATION not via start
  rt.stop();
}

TEST(ManagerRuntime, StopServoTearsDownInOrder) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToServoing(rt, mock);
  {
    std::lock_guard<std::mutex> lock(mock.m);
    mock.log.clear();
  }
  const CommandResult r = rt.stopServo();
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  std::lock_guard<std::mutex> lock(mock.m);
  const std::vector<std::string> expected = {
      "mode:0/0", "switch:/force_compliance_controller", "stop_rsi"};
  EXPECT_EQ(mock.log, expected);
  rt.stop();
}

TEST(ManagerRuntime, SriLossDegradesAndRecovers) {
  MockOps mock;
  ManagerConfig cfg = fastConfig();
  cfg.sri_status_timeout_s = 0.1;
  ManagerRuntime rt{cfg, mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  rt.setControllersLoaded(true);
  rt.feedEkiState(ekiUp());
  rt.feedSriStatus(true);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  ASSERT_TRUE(rt.startServo(2, 0).success);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));

  // Stop feeding SRI: within ~0.1 s the health judgement goes stale.
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::DEGRADED; }, 2000));
  rt.feedSriStatus(true);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));
  rt.stop();
}

TEST(ManagerRuntime, FaultThenResetRunsBothResetOps) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToServoing(rt, mock);
  rt.feedRsiState(true, true);  // latched RSI fault
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::FAULT; }, 2000));
  EXPECT_FALSE(rt.zeroSensor().success);  // gate closed in FAULT

  const CommandResult r = rt.resetFault();
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_TRUE(mock.logged("rsi_reset"));
  EXPECT_TRUE(mock.logged("eki_reset"));
  rt.feedRsiState(true, false);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  rt.stop();
}

TEST(ManagerRuntime, ZeroSensorGateFollowsDecision11) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  EXPECT_TRUE(rt.zeroSensor().success);
  EXPECT_TRUE(mock.logged("sri_zero"));

  ASSERT_TRUE(rt.startServo(2, 0).success);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));
  {
    std::lock_guard<std::mutex> lock(mock.m);
    mock.log.clear();
  }
  EXPECT_FALSE(rt.zeroSensor().success);   // SERVOING: rejected
  EXPECT_FALSE(mock.logged("sri_zero"));   // ops never touched
  rt.stop();
}

TEST(ManagerRuntime, CalibrationClosedLoopPersistsAndRestoresReady) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.02, 0.03);
  Wrench bias;
  bias.fx = 1.0; bias.fy = -2.0; bias.fz = 0.5;
  bias.tx = 0.1; bias.ty = -0.2; bias.tz = 0.05;

  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  rt.feedRsiState(true, false);   // calibration needs a valid RSI link

  const CommandResult r = rt.beginCalibration();
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_TRUE(mock.logged(
      "switch:cartesian_correction_controller/force_compliance_controller"));
  EXPECT_TRUE(mock.logged("mode:3/0"));

  for (int i = 0; i < 8; ++i) {
    ASSERT_TRUE(waitFor([&] { return mock.goalsSent() == i + 1; }, 2000))
        << "pose " << i;
    const CalPose g = mock.lastGoal();
    EXPECT_DOUBLE_EQ(g.a, kPoses[i][0]);
    rt.onMotionResult(true);
    ASSERT_TRUE(waitFor(
        [&] { return rt.snapshot().cal.phase == sfm::CalPhase::SAMPLING ||
                     mock.goalsSent() > i + 1; }, 2000));
    rt.feedWrench(synth(G, com, bias, g.a, g.b, g.c));
  }
  // Return move (goal 9), then DONE -> persist + teardown.
  ASSERT_TRUE(waitFor([&] { return mock.goalsSent() == 9; }, 2000));
  EXPECT_DOUBLE_EQ(mock.lastGoal().a, 0.0);
  rt.onMotionResult(true);

  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  std::lock_guard<std::mutex> lock(mock.m);
  ASSERT_EQ(mock.payload_applied, 1);
  EXPECT_NEAR(mock.payload_fit.params.gravity_n, G, 1e-8);
  EXPECT_NE(mock.payload_yaml.find("gravity_n: 50.000000"), std::string::npos);
  EXPECT_FALSE(rt.snapshot().calibrating);
  EXPECT_TRUE(rt.snapshot().fit.ok);
  rt.stop();
}

TEST(ManagerRuntime, CalibrationMoveFailureAbortsWithoutPersisting) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  rt.feedRsiState(true, false);
  ASSERT_TRUE(rt.beginCalibration().success);
  ASSERT_TRUE(waitFor([&] { return mock.goalsSent() == 1; }, 2000));
  rt.onMotionResult(false);   // first move fails

  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  std::lock_guard<std::mutex> lock(mock.m);
  EXPECT_EQ(mock.payload_applied, 0);
  EXPECT_EQ(mock.goals_sent, 1);   // decision 6: no return goal on abort
  rt.stop();
}

TEST(ManagerRuntime, CalibrationRejectedOutsideReady) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  EXPECT_FALSE(rt.beginCalibration().success);   // OFFLINE
  driveToServoing(rt, mock);
  EXPECT_FALSE(rt.beginCalibration().success);   // SERVOING
  rt.stop();
}
```

- [ ] **Step 2: 确认 RED**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | tail -3
```

预期:编译失败(`manager_runtime.h` 不存在)。CMake 增量先行:库源加 `src/manager_runtime.cpp`,测试区加:

```cmake
  catkin_add_gtest(test_manager_runtime test/test_manager_runtime.cpp)
  target_link_libraries(test_manager_runtime
                        soft_force_control_manager_core pthread
                        ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 3: 实现 header**

`include/soft_force_control_manager/manager_runtime.h`:

```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "soft_force_control_manager/calibration_sequencer.h"
#include "soft_force_control_manager/payload_yaml.h"
#include "soft_force_control_manager/system_state_core.h"

namespace sfm {

struct ToolFrame {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};  // mm / deg
  bool valid{false};
};

// One /kuka/eki/state message, reduced to what the manager consumes.
struct EkiFeed {
  bool connected{false};
  bool state_fresh{false};
  bool program_ready{false};
  bool rsi_active{false};
  bool fault{false};
};

// Every side effect the manager performs on the world, injected so the
// runtime is closed-loop testable with lambda mocks (Plan 4 runtime
// pattern). All callbacks are optional: a null hook no-ops (bool hooks
// report false). The node shell wires these to the real ROS clients.
struct ManagerOps {
  std::function<bool()> ekiStartRsi;
  std::function<bool()> ekiStopRsi;
  std::function<bool()> ekiResetFault;
  std::function<bool()> rsiResetFault;
  std::function<bool()> sriZero;
  std::function<ToolFrame()> ekiGetTool;
  std::function<bool(const std::string& start_ctrl,
                     const std::string& stop_ctrl)> switchControllers;
  std::function<void(std::uint8_t mode, std::uint8_t profile)> publishMode;
  std::function<void(const CalPose& target)> sendMotionGoal;
  std::function<bool(const std::string& yaml_text,
                     const sfc::PayloadFitResult& fit)> applyPayload;
  std::function<void()> publishState;  // worker-tick hook (decision 13)
};

struct ManagerConfig {
  double tick_period_s{0.1};
  double eki_state_timeout_s{5.0};  // decision 3: tolerates the eki node's
                                    // single-threaded-spin publish gaps (N3)
  double rsi_state_timeout_s{0.5};
  double sri_status_timeout_s{2.0};
  double tool_sync_retry_s{2.0};
  double rsi_connect_wait_s{5.0};   // decision 4 bounded start wait
  std::string compliance_controller{"force_compliance_controller"};
  std::string correction_controller{"cartesian_correction_controller"};
  CalibrationConfig calibration;
};

struct CommandResult {
  bool success{false};
  std::string message;
};

struct ManagerSnapshot {
  SystemState state{SystemState::OFFLINE};
  HealthInputs health;
  std::uint8_t mode{0}, profile{0};
  std::string active_controller;
  bool calibrating{false};
  CalStatus cal;
  sfc::PayloadFitResult fit;
};

// Owns the worker thread (health -> state machine, edge-triggered tool
// sync, calibration sequencing, finish handling) and the blocking command
// methods used by the service layer. Commands serialize on command_mutex_;
// mutable state hides behind mutex_; ops callbacks always run OUTSIDE
// mutex_ (they may block on ROS service calls). Not ROS-dependent.
class ManagerRuntime {
 public:
  ManagerRuntime(const ManagerConfig& cfg, ManagerOps ops);
  ~ManagerRuntime();
  ManagerRuntime(const ManagerRuntime&) = delete;
  ManagerRuntime& operator=(const ManagerRuntime&) = delete;

  bool start();
  void stop();

  void setControllersLoaded(bool loaded);
  void feedEkiState(const EkiFeed& f);
  void feedRsiState(bool connected, bool fault);
  void feedSriStatus(bool streaming);
  void feedWrench(const sfc::Wrench& w);
  void onMotionResult(bool success);  // action-client done callback

  CommandResult startServo(std::uint8_t mode, std::uint8_t profile);
  CommandResult stopServo();
  CommandResult resetFault();
  CommandResult zeroSensor();
  CommandResult beginCalibration();
  void cancelCalibration();

  ManagerSnapshot snapshot() const;

 private:
  void run();
  HealthInputs healthLocked(double now_s) const;
  static double nowS();

  ManagerConfig cfg_;
  ManagerOps ops_;
  std::thread thread_;
  std::atomic<bool> running_{false};

  std::mutex command_mutex_;  // serializes the command methods

  mutable std::mutex mutex_;  // guards everything below
  SystemStateCore core_;
  CalibrationSequencer seq_;
  EkiFeed eki_;
  double eki_rx_s_{-1.0};
  bool rsi_connected_{false};
  bool rsi_fault_{false};
  double rsi_rx_s_{-1.0};
  bool sri_streaming_{false};
  double sri_rx_s_{-1.0};
  bool controllers_loaded_{false};
  bool tool_synced_{false};
  double tool_last_try_s_{-1e9};
  std::uint8_t mode_{0}, profile_{0};
  std::string active_controller_;
  bool cal_teardown_done_{true};  // finish handling latch (edge trigger)
};

}  // namespace sfm
```

- [ ] **Step 4: 实现 runtime**

`src/manager_runtime.cpp`:

```cpp
#include "soft_force_control_manager/manager_runtime.h"

#include <chrono>

namespace sfm {

namespace {
constexpr std::uint8_t kModeIdle = 0;
constexpr std::uint8_t kModeDirect = 1;
constexpr std::uint8_t kModeCompliance = 2;
constexpr std::uint8_t kModeCalibration = 3;
}  // namespace

ManagerRuntime::ManagerRuntime(const ManagerConfig& cfg, ManagerOps ops)
    : cfg_(cfg), ops_(std::move(ops)) {
  seq_.configure(cfg_.calibration);
}

ManagerRuntime::~ManagerRuntime() { stop(); }

double ManagerRuntime::nowS() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool ManagerRuntime::start() {
  if (running_.load()) return false;
  running_ = true;
  thread_ = std::thread(&ManagerRuntime::run, this);
  return true;
}

void ManagerRuntime::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
}

void ManagerRuntime::setControllersLoaded(bool loaded) {
  std::lock_guard<std::mutex> lock(mutex_);
  controllers_loaded_ = loaded;
}

void ManagerRuntime::feedEkiState(const EkiFeed& f) {
  std::lock_guard<std::mutex> lock(mutex_);
  eki_ = f;
  eki_rx_s_ = nowS();
}

void ManagerRuntime::feedRsiState(bool connected, bool fault) {
  std::lock_guard<std::mutex> lock(mutex_);
  rsi_connected_ = connected;
  rsi_fault_ = fault;
  rsi_rx_s_ = nowS();
}

void ManagerRuntime::feedSriStatus(bool streaming) {
  std::lock_guard<std::mutex> lock(mutex_);
  sri_streaming_ = streaming;
  sri_rx_s_ = nowS();
}

void ManagerRuntime::feedWrench(const sfc::Wrench& w) {
  std::lock_guard<std::mutex> lock(mutex_);
  seq_.onWrench(w);  // no-op unless the sequencer is SAMPLING
}

void ManagerRuntime::onMotionResult(bool success) {
  std::lock_guard<std::mutex> lock(mutex_);
  seq_.onMotionResult(success, nowS());
}

HealthInputs ManagerRuntime::healthLocked(double now_s) const {
  HealthInputs in;
  const bool eki_fresh =
      eki_rx_s_ >= 0.0 && now_s - eki_rx_s_ <= cfg_.eki_state_timeout_s;
  in.eki_link = eki_fresh && eki_.connected && eki_.state_fresh;
  in.eki_program_ready = in.eki_link && eki_.program_ready;
  in.eki_fault = eki_fresh && eki_.fault;
  in.rsi_topic_fresh =
      rsi_rx_s_ >= 0.0 && now_s - rsi_rx_s_ <= cfg_.rsi_state_timeout_s;
  in.rsi_connected = in.rsi_topic_fresh && rsi_connected_;
  in.rsi_fault = in.rsi_topic_fresh && rsi_fault_;
  in.sri_streaming = sri_rx_s_ >= 0.0 &&
                     now_s - sri_rx_s_ <= cfg_.sri_status_timeout_s &&
                     sri_streaming_;
  in.tool_synced = tool_synced_;
  in.controllers_loaded = controllers_loaded_;
  return in;
}

void ManagerRuntime::run() {
  while (running_.load()) {
    const double now = nowS();
    bool want_tool_sync = false;
    bool cal_stream_ok = true;
    CalAction act;
    bool cal_finished = false;
    bool cal_success = false;
    sfc::PayloadFitResult fit;
    std::uint8_t cal_profile = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      HealthInputs in = healthLocked(now);
      if (!in.eki_link) tool_synced_ = false;  // resync after reconnect
      core_.update(in);

      want_tool_sync = in.eki_link && eki_.program_ready && !tool_synced_ &&
                       now - tool_last_try_s_ >= cfg_.tool_sync_retry_s &&
                       static_cast<bool>(ops_.ekiGetTool);
      if (want_tool_sync) tool_last_try_s_ = now;

      const CalStatus cs = seq_.status();
      const bool cal_active =
          cs.phase == CalPhase::MOVING || cs.phase == CalPhase::SETTLING ||
          cs.phase == CalPhase::SAMPLING || cs.phase == CalPhase::RETURNING;
      if (cal_active && !in.sri_streaming) cal_stream_ok = false;
      if (!cal_stream_ok) seq_.onStreamLost();
      act = seq_.tick(now);

      const CalStatus after = seq_.status();
      if ((after.phase == CalPhase::DONE || after.phase == CalPhase::FAILED) &&
          !cal_teardown_done_) {
        cal_teardown_done_ = true;
        cal_finished = true;
        cal_success = after.phase == CalPhase::DONE && seq_.result().ok;
        fit = seq_.result();
        cal_profile = profile_;
      }
    }

    // ops run outside mutex_ (they may block on ROS services).
    if (want_tool_sync) {
      const ToolFrame t = ops_.ekiGetTool();
      std::lock_guard<std::mutex> lock(mutex_);
      if (t.valid) tool_synced_ = true;
    }
    if (act.send_goal && ops_.sendMotionGoal) ops_.sendMotionGoal(act.target);
    if (cal_finished) {
      if (cal_success && ops_.applyPayload) {
        ops_.applyPayload(emitPayloadYaml(fit, ""), fit);
      }
      if (ops_.publishMode) ops_.publishMode(kModeIdle, cal_profile);
      if (ops_.switchControllers) {
        ops_.switchControllers("", cfg_.correction_controller);
      }
      std::lock_guard<std::mutex> lock(mutex_);
      core_.calibrationFinished();
      active_controller_.clear();
      mode_ = kModeIdle;
    }
    if (ops_.publishState) ops_.publishState();

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::microseconds(static_cast<long>(cfg_.tick_period_s * 1e6));
    while (running_.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

CommandResult ManagerRuntime::startServo(std::uint8_t mode,
                                         std::uint8_t profile) {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  if (mode != kModeDirect && mode != kModeCompliance)
    return {false, "mode must be DIRECT_CARTESIAN or FORCE_COMPLIANCE"};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (core_.state() != SystemState::READY)
      return {false, "start requires READY"};
  }

  bool need_start_rsi = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    need_start_rsi = !eki_.rsi_active;
  }
  if (need_start_rsi) {
    if (!ops_.ekiStartRsi || !ops_.ekiStartRsi())
      return {false, "EKI start_rsi_program rejected"};
  }

  // Bounded wait for RSI frames (decision 4). No EKI rollback on timeout:
  // the system stays READY and the operator may retry or stop.
  const double deadline = nowS() + cfg_.rsi_connect_wait_s;
  bool rsi_ok = false;
  while (nowS() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const HealthInputs in = healthLocked(nowS());
      if (in.rsi_topic_fresh && in.rsi_connected) {
        rsi_ok = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!rsi_ok) return {false, "RSI link did not come up in time"};

  const std::string target = mode == kModeCompliance
                                 ? cfg_.compliance_controller
                                 : cfg_.correction_controller;
  const std::string other = mode == kModeCompliance
                                ? cfg_.correction_controller
                                : cfg_.compliance_controller;
  if (ops_.publishMode) ops_.publishMode(kModeIdle, profile);
  if (!ops_.switchControllers || !ops_.switchControllers(target, other))
    return {false, "controller switch failed"};
  if (ops_.publishMode) ops_.publishMode(mode, profile);

  std::lock_guard<std::mutex> lock(mutex_);
  const Verdict v = core_.requestStart(healthLocked(nowS()));
  if (!v.accepted) return {false, v.reason};
  mode_ = mode;
  profile_ = profile;
  active_controller_ = target;
  return {true, ""};
}

CommandResult ManagerRuntime::stopServo() {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  std::string active;
  std::uint8_t profile = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const SystemState s = core_.state();
    if (s != SystemState::SERVOING && s != SystemState::DEGRADED)
      return {false, "stop requires SERVOING or DEGRADED"};
    active = active_controller_;
    profile = profile_;
  }
  if (ops_.publishMode) ops_.publishMode(kModeIdle, profile);
  if (ops_.switchControllers) ops_.switchControllers("", active);
  if (ops_.ekiStopRsi) ops_.ekiStopRsi();
  std::lock_guard<std::mutex> lock(mutex_);
  const Verdict v = core_.requestStop();
  if (!v.accepted) return {false, v.reason};
  mode_ = kModeIdle;
  active_controller_.clear();
  return {true, ""};
}

CommandResult ManagerRuntime::resetFault() {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (core_.state() != SystemState::FAULT)
      return {false, "reset requires FAULT"};
  }
  std::string message;
  if (ops_.rsiResetFault && !ops_.rsiResetFault())
    message += "RSI reset failed; ";
  if (ops_.ekiResetFault && !ops_.ekiResetFault())
    message += "EKI reset failed; ";
  std::lock_guard<std::mutex> lock(mutex_);
  const Verdict v = core_.requestReset();
  if (!v.accepted) return {false, v.reason};
  return {true, message};
}

CommandResult ManagerRuntime::zeroSensor() {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!core_.allowZeroSensor())
      return {false, "sensor zero allowed in CONNECTED/READY only"};
  }
  if (!ops_.sriZero || !ops_.sriZero())
    return {false, "sri zero service failed"};
  return {true, ""};
}

CommandResult ManagerRuntime::beginCalibration() {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  std::uint8_t profile = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const Verdict v = core_.requestCalibration(healthLocked(nowS()));
    if (!v.accepted) return {false, v.reason};
    profile = profile_;
  }
  if (ops_.publishMode) ops_.publishMode(kModeIdle, profile);
  if (!ops_.switchControllers ||
      !ops_.switchControllers(cfg_.correction_controller,
                              cfg_.compliance_controller)) {
    std::lock_guard<std::mutex> lock(mutex_);
    core_.calibrationFinished();  // roll the request back
    return {false, "controller switch failed"};
  }
  if (ops_.publishMode) ops_.publishMode(kModeCalibration, profile);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!seq_.start(nowS())) {
    core_.calibrationFinished();
    return {false, "calibration sequencer rejected start (bad config?)"};
  }
  cal_teardown_done_ = false;
  mode_ = kModeCalibration;
  active_controller_ = cfg_.correction_controller;
  return {true, ""};
}

void ManagerRuntime::cancelCalibration() {
  std::lock_guard<std::mutex> lock(mutex_);
  seq_.cancel();  // the worker tick performs the teardown
}

ManagerSnapshot ManagerRuntime::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  ManagerSnapshot s;
  s.health = healthLocked(nowS());
  s.state = core_.state();
  s.mode = mode_;
  s.profile = profile_;
  s.active_controller = active_controller_;
  s.cal = seq_.status();
  s.calibrating = s.cal.phase == CalPhase::MOVING ||
                  s.cal.phase == CalPhase::SETTLING ||
                  s.cal.phase == CalPhase::SAMPLING ||
                  s.cal.phase == CalPhase::RETURNING;
  s.fit = seq_.result();
  return s;
}

}  // namespace sfm
```

实现注意(评审点):
- `snapshot().state` 读的是 worker 最近一次 `core_.update` 的结果(10 Hz 刷新),测试一律 `waitFor`。
- `emitPayloadYaml(fit, "")` 的 timestamp 由节点侧 `applyPayload` 实现补齐(runtime 无 ROS 时钟;Task 7 用 `ros::Time::now()` 格式化后替换或直接把 yaml 文本重生成——节点实现选后者:忽略 runtime 文本重生成一次,见 Task 7 Step 3 注释。测试断言的是 runtime 传出的文本与 fit)。
- `beginCalibration` 的 switch 失败路径回滚 `requestCalibration`(core 的 `calibrating_` 位),已在代码内。

- [ ] **Step 5: 确认 GREEN + 3 连跑**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests 2>&1 | grep -ci warning   # 0
for i in 1 2 3; do ./devel/lib/soft_force_control_manager/test_manager_runtime || break; done
```

预期:12 tests PASS × 3,无 flaky(全部等待有界,mock 无时序敏感波形)。

- [ ] **Step 6: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_force_control_manager
git commit -m "feat(manager): ManagerRuntime orchestration + lambda-mock closed-loop tests (Plan5 T6)"
```

**验收标准:** 12 用例覆盖:阶梯+tool 同步(一次性/重试)、start 顺序断言、RSI 不上失败留 READY、非法模式、stop 顺序、SRI 失鲜降级恢复、fault+reset 双 ops、zero 门禁、标定闭环(持久化+精确拟合+teardown)、标定失败不持久化不回程、非 READY 拒绝;ops 全部锁外调用;3 连跑无 flaky;零警告。

---

### Task 7: `soft_robot_manager` 节点壳 + `manager.yaml` / `calibration.yaml`

**目标:** 薄 ROS 壳:参数装载 → `ManagerOps` 接线到真实 ROS 客户端(EKI 服务、`/kuka/rsi/reset_fault`、`/sri_ft/zero`、`controller_manager/switch_controller`、`/soft_robot/mode_command`、`/soft_robot/move_to_orientation` action client、payload.yaml 写盘+参数覆写+控制器重载)→ 服务/action server 转发到 runtime。行为已由 Task 4~6 离线覆盖,本任务无 gtest(同 Plan 4 节点壳先例),roscore 冒烟并入 Task 8。

**Files:**
- Create: `ros_ws/src/soft_force_control_manager/src/manager_node.cpp`
- Create: `ros_ws/src/soft_force_control_manager/config/manager.yaml`
- Create: `ros_ws/src/soft_force_control_manager/config/calibration.yaml`
- Modify: `ros_ws/src/soft_force_control_manager/CMakeLists.txt`(节点可执行 + install)

**Interfaces:**
- Consumes:Task 6 `ManagerRuntime` 全签名;`soft_robot_msgs`(ManagerState/StartServo/CalibratePayload/ModeCommand/ModeState/EkiState/RsiState/SriStatus/GetTool/MoveToOrientationAction);`controller_manager_msgs/SwitchController`(strictness=STRICT)+ `LoadController`/`UnloadController`;`std_srvs/Trigger`。
- Produces(Plan 6 后端接口 + Task 8 launch 消费):节点名 `soft_robot_manager`;服务 `/soft_robot/{start_servo,stop_servo,reset_fault,zero_sensor}`;action `/soft_robot/calibrate_payload`;话题 `/soft_robot/manager_state`(10 Hz)、`/soft_robot/diagnostics`(1 Hz);参数见两个 yaml。

- [ ] **Step 1: 写配置文件**

`config/manager.yaml`:

```yaml
# Manager parameters (spec sections 5.5, 11, 14; decision 3 thresholds).
# Loaded into the soft_robot_manager private namespace by the bringup.
soft_robot_manager:
  tick_period_s: 0.1
  # /kuka/eki/state freshness. Must tolerate the eki node's single-threaded
  # spin: a blocking command can gap the 10 Hz publisher by up to
  # response_timeout_s + 1 s (~3 s); 5 s adds margin (Plan 4 follow-up 6).
  eki_state_timeout_s: 5.0
  rsi_state_timeout_s: 0.5        # 50 Hz topic; manager-side watch (Plan 3 FU-4)
  sri_status_timeout_s: 2.0       # 10 Hz topic
  tool_sync_retry_s: 2.0
  rsi_connect_wait_s: 5.0         # bounded wait for RSI frames on start
  compliance_controller: force_compliance_controller
  correction_controller: cartesian_correction_controller
  payload_file: ""                # "" = <bringup share>/config/payload.yaml (Task 8 sets it)
  goal_speed_scale: 1.0           # speed_scale sent with calibration goals
```

`config/calibration.yaml`:

```yaml
# Payload calibration sequence (spec sections 9, 14): the legacy
# 8-orientation set, settle time, and per-pose averaging count.
soft_robot_manager:
  calibration:
    settle_time_s: 1.0            # legacy settle (spec 9 step 3c)
    samples_per_pose: 100         # deliberate improvement over legacy 1
    return_pose: {a: 0.0, b: 0.0, c: 0.0}
    poses:                        # deg; varied A/B/C for an observable fit
      - {a: 0.0,   b: 0.0,   c: 0.0}
      - {a: 0.0,   b: 45.0,  c: 0.0}
      - {a: 0.0,   b: -45.0, c: 0.0}
      - {a: 0.0,   b: 0.0,   c: 45.0}
      - {a: 0.0,   b: 0.0,   c: -45.0}
      - {a: 45.0,  b: 30.0,  c: 0.0}
      - {a: -45.0, b: 0.0,   c: 30.0}
      - {a: 30.0,  b: -30.0, c: 30.0}
```

- [ ] **Step 2: CMake / manifest 增量**

节点用 `ros::package::getPath`(roslib):`package.xml` 加 `<depend>roslib</depend>`;`CMakeLists.txt` 的 `find_package(catkin REQUIRED COMPONENTS ...)` 与 `CATKIN_DEPENDS` 两处各加 `roslib`。然后追加:

```cmake
add_executable(soft_robot_manager src/manager_node.cpp)
add_dependencies(soft_robot_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(soft_robot_manager soft_force_control_manager_core
                      ${catkin_LIBRARIES})

install(TARGETS soft_robot_manager
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

- [ ] **Step 3: 写节点**

`src/manager_node.cpp`(完整文件):

```cpp
// Thin ROS shell around ManagerRuntime (spec 5.5, 9, 11). All orchestration
// logic lives in the offline-tested library; this file loads parameters,
// wires ManagerOps to the real ROS clients, and forwards services/actions.
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <soft_robot_msgs/CalibratePayloadAction.h>
#include <soft_robot_msgs/EkiState.h>
#include <soft_robot_msgs/GetTool.h>
#include <soft_robot_msgs/ManagerState.h>
#include <soft_robot_msgs/ModeCommand.h>
#include <soft_robot_msgs/ModeState.h>
#include <soft_robot_msgs/MoveToOrientationAction.h>
#include <soft_robot_msgs/RsiState.h>
#include <soft_robot_msgs/SriStatus.h>
#include <soft_robot_msgs/StartServo.h>
#include <std_srvs/Trigger.h>

#include <cstdio>
#include <ctime>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "soft_force_control_manager/manager_runtime.h"

namespace {

// Wire/state numbering alignment (Plan 3 mode_bridge.h pattern).
static_assert(soft_robot_msgs::ModeState::SYSTEM_OFFLINE ==
                  static_cast<std::uint8_t>(sfm::SystemState::OFFLINE),
              "SYSTEM_OFFLINE mismatch");
static_assert(soft_robot_msgs::ModeState::SYSTEM_SERVOING ==
                  static_cast<std::uint8_t>(sfm::SystemState::SERVOING),
              "SYSTEM_SERVOING mismatch");
static_assert(soft_robot_msgs::ModeState::SYSTEM_FAULT ==
                  static_cast<std::uint8_t>(sfm::SystemState::FAULT),
              "SYSTEM_FAULT mismatch");
static_assert(soft_robot_msgs::ModeCommand::MODE_CALIBRATION == 3u,
              "MODE_CALIBRATION wire value drifted");

bool callTrigger(ros::ServiceClient& c) {
  std_srvs::Trigger srv;
  return c.call(srv) && srv.response.success;
}

const char* phaseName(sfm::CalPhase p) {
  switch (p) {
    case sfm::CalPhase::MOVING: return "MOVING";
    case sfm::CalPhase::SETTLING: return "SETTLING";
    case sfm::CalPhase::SAMPLING: return "SAMPLING";
    case sfm::CalPhase::RETURNING: return "RETURNING";
    case sfm::CalPhase::DONE: return "DONE";
    case sfm::CalPhase::FAILED: return "FAILED";
    default: return "IDLE";
  }
}

std::string isoNow() {
  char buf[32];
  const std::time_t t = static_cast<std::time_t>(ros::Time::now().sec);
  std::tm tm_utc{};
  gmtime_r(&t, &tm_utc);
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_utc);
  return buf;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "soft_robot_manager");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  sfm::ManagerConfig cfg;
  pnh.param("tick_period_s", cfg.tick_period_s, 0.1);
  pnh.param("eki_state_timeout_s", cfg.eki_state_timeout_s, 5.0);
  pnh.param("rsi_state_timeout_s", cfg.rsi_state_timeout_s, 0.5);
  pnh.param("sri_status_timeout_s", cfg.sri_status_timeout_s, 2.0);
  pnh.param("tool_sync_retry_s", cfg.tool_sync_retry_s, 2.0);
  pnh.param("rsi_connect_wait_s", cfg.rsi_connect_wait_s, 5.0);
  pnh.param<std::string>("compliance_controller", cfg.compliance_controller,
                         "force_compliance_controller");
  pnh.param<std::string>("correction_controller", cfg.correction_controller,
                         "cartesian_correction_controller");
  double goal_speed_scale = 1.0;
  pnh.param("goal_speed_scale", goal_speed_scale, 1.0);
  std::string payload_file;
  pnh.param<std::string>("payload_file", payload_file, "");
  if (payload_file.empty()) {
    payload_file =
        ros::package::getPath("soft_robot_bringup") + "/config/payload.yaml";
  }

  // calibration.yaml (spec 14): poses / settle / samples / return pose.
  pnh.param("calibration/settle_time_s", cfg.calibration.settle_time_s, 1.0);
  pnh.param("calibration/samples_per_pose", cfg.calibration.samples_per_pose,
            100);
  pnh.param("calibration/return_pose/a", cfg.calibration.return_pose.a, 0.0);
  pnh.param("calibration/return_pose/b", cfg.calibration.return_pose.b, 0.0);
  pnh.param("calibration/return_pose/c", cfg.calibration.return_pose.c, 0.0);
  XmlRpc::XmlRpcValue poses;
  if (pnh.getParam("calibration/poses", poses) &&
      poses.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < poses.size(); ++i) {
      sfm::CalPose p;
      p.a = static_cast<double>(poses[i]["a"]);
      p.b = static_cast<double>(poses[i]["b"]);
      p.c = static_cast<double>(poses[i]["c"]);
      cfg.calibration.poses.push_back(p);
    }
  }
  if (cfg.calibration.poses.empty()) {
    ROS_FATAL("soft_robot_manager: calibration/poses missing or empty");
    return 1;
  }

  // --- ROS clients backing ManagerOps ---
  ros::ServiceClient eki_start =
      nh.serviceClient<std_srvs::Trigger>("/kuka/eki/start_rsi_program");
  ros::ServiceClient eki_stop =
      nh.serviceClient<std_srvs::Trigger>("/kuka/eki/stop_rsi_program");
  ros::ServiceClient eki_reset =
      nh.serviceClient<std_srvs::Trigger>("/kuka/eki/reset_fault");
  ros::ServiceClient eki_get_tool =
      nh.serviceClient<soft_robot_msgs::GetTool>("/kuka/eki/get_tool");
  ros::ServiceClient rsi_reset =
      nh.serviceClient<std_srvs::Trigger>("/kuka/rsi/reset_fault");
  ros::ServiceClient sri_zero =
      nh.serviceClient<std_srvs::Trigger>("/sri_ft/zero");
  ros::ServiceClient cm_switch =
      nh.serviceClient<controller_manager_msgs::SwitchController>(
          "/controller_manager/switch_controller");
  ros::ServiceClient cm_load =
      nh.serviceClient<controller_manager_msgs::LoadController>(
          "/controller_manager/load_controller");
  ros::ServiceClient cm_unload =
      nh.serviceClient<controller_manager_msgs::UnloadController>(
          "/controller_manager/unload_controller");
  ros::Publisher mode_pub =
      nh.advertise<soft_robot_msgs::ModeCommand>("/soft_robot/mode_command", 10);
  ros::Publisher state_pub = nh.advertise<soft_robot_msgs::ManagerState>(
      "/soft_robot/manager_state", 10);
  ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(
      "/soft_robot/diagnostics", 10);

  actionlib::SimpleActionClient<soft_robot_msgs::MoveToOrientationAction>
      motion_client("/soft_robot/move_to_orientation", true);

  std::unique_ptr<sfm::ManagerRuntime> runtime;

  sfm::ManagerOps ops;
  ops.ekiStartRsi = [&] { return callTrigger(eki_start); };
  ops.ekiStopRsi = [&] { return callTrigger(eki_stop); };
  ops.ekiResetFault = [&] { return callTrigger(eki_reset); };
  ops.rsiResetFault = [&] { return callTrigger(rsi_reset); };
  ops.sriZero = [&] { return callTrigger(sri_zero); };
  ops.ekiGetTool = [&]() -> sfm::ToolFrame {
    sfm::ToolFrame t;
    soft_robot_msgs::GetTool srv;
    if (!eki_get_tool.call(srv) || !srv.response.success) return t;
    t.x = srv.response.x; t.y = srv.response.y; t.z = srv.response.z;
    t.a = srv.response.a; t.b = srv.response.b; t.c = srv.response.c;
    t.valid = true;
    return t;
  };
  ops.switchControllers = [&](const std::string& start,
                              const std::string& stop) {
    controller_manager_msgs::SwitchController srv;
    if (!start.empty()) srv.request.start_controllers.push_back(start);
    if (!stop.empty()) srv.request.stop_controllers.push_back(stop);
    srv.request.strictness =
        controller_manager_msgs::SwitchController::Request::STRICT;
    return cm_switch.call(srv) && srv.response.ok;
  };
  ops.publishMode = [&](std::uint8_t mode, std::uint8_t profile) {
    soft_robot_msgs::ModeCommand msg;
    msg.mode = mode;
    msg.profile = profile;
    mode_pub.publish(msg);
  };
  ops.sendMotionGoal = [&](const sfm::CalPose& p) {
    soft_robot_msgs::MoveToOrientationGoal goal;
    goal.a = p.a; goal.b = p.b; goal.c = p.c;
    goal.use_position = false;
    goal.speed_scale = goal_speed_scale;
    motion_client.sendGoal(
        goal,
        [&](const actionlib::SimpleClientGoalState& state,
            const soft_robot_msgs::MoveToOrientationResultConstPtr&) {
          // SUCCEEDED counts as success even after a preempt raced a
          // convergence (Plan 3 follow-up 7, decision 15).
          runtime->onMotionResult(
              state == actionlib::SimpleClientGoalState::SUCCEEDED);
        });
  };
  ops.applyPayload = [&](const std::string& /*yaml_from_runtime*/,
                         const sfc::PayloadFitResult& fit) {
    // Re-emit with a real timestamp (the runtime has no ROS clock).
    const std::string yaml = sfm::emitPayloadYaml(fit, isoNow());
    std::ofstream f(payload_file, std::ios::trunc);
    if (!f) {
      ROS_ERROR("payload.yaml write failed: %s", payload_file.c_str());
      return false;
    }
    f << yaml;
    f.close();
    // Decision 7: parameter override + controller reload (it is stopped
    // during calibration; ros_control reads parameters at load time).
    const std::string ns = "/" + cfg.compliance_controller + "/payload/";
    ros::param::set(ns + "gravity_n", fit.params.gravity_n);
    ros::param::set(ns + "com_x", fit.params.com_x);
    ros::param::set(ns + "com_y", fit.params.com_y);
    ros::param::set(ns + "com_z", fit.params.com_z);
    ros::param::set(ns + "bias_fx", fit.params.bias.fx);
    ros::param::set(ns + "bias_fy", fit.params.bias.fy);
    ros::param::set(ns + "bias_fz", fit.params.bias.fz);
    ros::param::set(ns + "bias_tx", fit.params.bias.tx);
    ros::param::set(ns + "bias_ty", fit.params.bias.ty);
    ros::param::set(ns + "bias_tz", fit.params.bias.tz);
    controller_manager_msgs::UnloadController u;
    u.request.name = cfg.compliance_controller;
    controller_manager_msgs::LoadController l;
    l.request.name = cfg.compliance_controller;
    if (!cm_unload.call(u) || !u.response.ok ||
        !cm_load.call(l) || !l.response.ok) {
      ROS_ERROR("payload reload of %s failed; new payload takes effect "
                "after restart", cfg.compliance_controller.c_str());
      return false;
    }
    return true;
  };
  ops.publishState = [&] {
    const sfm::ManagerSnapshot s = runtime->snapshot();
    soft_robot_msgs::ManagerState msg;
    msg.header.stamp = ros::Time::now();
    msg.system_state = static_cast<std::uint8_t>(s.state);
    msg.mode = s.mode;
    msg.profile = s.profile;
    msg.eki_connected = s.health.eki_link;
    msg.eki_program_ready = s.health.eki_program_ready;
    msg.rsi_connected = s.health.rsi_connected;
    msg.rsi_fault = s.health.rsi_fault;
    msg.sri_streaming = s.health.sri_streaming;
    msg.tool_synced = s.health.tool_synced;
    msg.calibrating = s.calibrating;
    msg.active_controller = s.active_controller;
    state_pub.publish(msg);
  };

  runtime.reset(new sfm::ManagerRuntime(cfg, ops));

  // --- inbound topics -> runtime feeds ---
  ros::Subscriber eki_sub = nh.subscribe<soft_robot_msgs::EkiState>(
      "/kuka/eki/state", 5, [&](const soft_robot_msgs::EkiState::ConstPtr& m) {
        sfm::EkiFeed f;
        f.connected = m->connected;
        f.state_fresh = m->state_fresh;
        f.program_ready = m->program_ready;
        f.rsi_active = m->rsi_active;
        f.fault = m->fault;
        runtime->feedEkiState(f);
      });
  ros::Subscriber rsi_sub = nh.subscribe<soft_robot_msgs::RsiState>(
      "/kuka/rsi/state", 5, [&](const soft_robot_msgs::RsiState::ConstPtr& m) {
        runtime->feedRsiState(m->connected, m->fault);
      });
  ros::Subscriber sri_sub = nh.subscribe<soft_robot_msgs::SriStatus>(
      "/sri_ft/status", 5, [&](const soft_robot_msgs::SriStatus::ConstPtr& m) {
        runtime->feedSriStatus(m->connected && m->streaming);
      });
  ros::Subscriber wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>(
      "/sri_ft/wrench_raw", 50,
      [&](const geometry_msgs::WrenchStamped::ConstPtr& m) {
        sfc::Wrench w;
        w.fx = m->wrench.force.x; w.fy = m->wrench.force.y;
        w.fz = m->wrench.force.z; w.tx = m->wrench.torque.x;
        w.ty = m->wrench.torque.y; w.tz = m->wrench.torque.z;
        runtime->feedWrench(w);
      });

  // --- controller preloading (READY precondition controllers_loaded) ---
  auto loadController = [&](const std::string& name) {
    controller_manager_msgs::LoadController srv;
    srv.request.name = name;
    return cm_load.call(srv) && srv.response.ok;
  };

  // --- services ---
  ros::ServiceServer start_srv =
      nh.advertiseService<soft_robot_msgs::StartServo::Request,
                          soft_robot_msgs::StartServo::Response>(
          "/soft_robot/start_servo",
          [&](soft_robot_msgs::StartServo::Request& req,
              soft_robot_msgs::StartServo::Response& res) {
            const sfm::CommandResult r =
                runtime->startServo(req.mode, req.profile);
            res.success = r.success;
            res.message = r.message;
            return true;
          });
  auto trigger = [&](sfm::CommandResult (sfm::ManagerRuntime::*fn)()) {
    return [&, fn](std_srvs::Trigger::Request&,
                   std_srvs::Trigger::Response& res) {
      const sfm::CommandResult r = ((*runtime).*fn)();
      res.success = r.success;
      res.message = r.message;
      return true;
    };
  };
  ros::ServiceServer stop_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/soft_robot/stop_servo", trigger(&sfm::ManagerRuntime::stopServo));
  ros::ServiceServer reset_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/soft_robot/reset_fault",
          trigger(&sfm::ManagerRuntime::resetFault));
  ros::ServiceServer zero_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/soft_robot/zero_sensor",
          trigger(&sfm::ManagerRuntime::zeroSensor));

  // --- calibration action server ---
  using CalServer =
      actionlib::SimpleActionServer<soft_robot_msgs::CalibratePayloadAction>;
  std::unique_ptr<CalServer> cal_server;
  cal_server.reset(new CalServer(
      nh, "/soft_robot/calibrate_payload",
      [&](const soft_robot_msgs::CalibratePayloadGoalConstPtr&) {
        soft_robot_msgs::CalibratePayloadResult result;
        const sfm::CommandResult begin = runtime->beginCalibration();
        if (!begin.success) {
          result.success = false;
          result.message = begin.message;
          cal_server->setAborted(result);
          return;
        }
        ros::Rate rate(10);
        while (ros::ok()) {
          if (cal_server->isPreemptRequested()) runtime->cancelCalibration();
          const sfm::ManagerSnapshot s = runtime->snapshot();
          soft_robot_msgs::CalibratePayloadFeedback fb;
          fb.pose_index = static_cast<std::uint32_t>(s.cal.pose_index);
          fb.pose_count = static_cast<std::uint32_t>(s.cal.pose_count);
          fb.phase = phaseName(s.cal.phase);
          cal_server->publishFeedback(fb);
          if (s.cal.phase == sfm::CalPhase::DONE ||
              s.cal.phase == sfm::CalPhase::FAILED) {
            // Wait for the runtime teardown tick to restore READY.
            if (!s.calibrating && s.state != sfm::SystemState::CALIBRATING) {
              result.success =
                  s.cal.phase == sfm::CalPhase::DONE && s.fit.ok;
              result.message =
                  result.success ? (s.cal.return_move_ok
                                        ? "ok"
                                        : "ok (return move failed)")
                                 : std::string("failed in phase ") +
                                       phaseName(s.cal.phase);
              result.gravity_n = s.fit.params.gravity_n;
              result.com_x = s.fit.params.com_x;
              result.com_y = s.fit.params.com_y;
              result.com_z = s.fit.params.com_z;
              result.bias_fx = s.fit.params.bias.fx;
              result.bias_fy = s.fit.params.bias.fy;
              result.bias_fz = s.fit.params.bias.fz;
              result.bias_tx = s.fit.params.bias.tx;
              result.bias_ty = s.fit.params.bias.ty;
              result.bias_tz = s.fit.params.bias.tz;
              result.r2_force = s.fit.r2_force;
              result.r2_torque = s.fit.r2_torque;
              if (result.success) cal_server->setSucceeded(result);
              else cal_server->setAborted(result);
              return;
            }
          }
          rate.sleep();
        }
      },
      false));

  // --- diagnostics (1 Hz) ---
  ros::Timer diag_timer =
      nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent&) {
        const sfm::ManagerSnapshot s = runtime->snapshot();
        diagnostic_msgs::DiagnosticArray arr;
        arr.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus st;
        st.name = "soft_robot_manager: system";
        st.hardware_id = "soft_robot";
        if (s.state == sfm::SystemState::FAULT) {
          st.level = diagnostic_msgs::DiagnosticStatus::ERROR;
          st.message = "FAULT latched: reset required";
        } else if (s.state == sfm::SystemState::DEGRADED) {
          st.level = diagnostic_msgs::DiagnosticStatus::WARN;
          st.message = "DEGRADED: output forced to zero";
        } else {
          st.level = diagnostic_msgs::DiagnosticStatus::OK;
          st.message = "ok";
        }
        auto kv = [&st](const std::string& k, const std::string& v) {
          diagnostic_msgs::KeyValue e;
          e.key = k;
          e.value = v;
          st.values.push_back(e);
        };
        kv("system_state", std::to_string(static_cast<int>(s.state)));
        kv("eki_link", s.health.eki_link ? "true" : "false");
        kv("rsi_connected", s.health.rsi_connected ? "true" : "false");
        kv("sri_streaming", s.health.sri_streaming ? "true" : "false");
        kv("tool_synced", s.health.tool_synced ? "true" : "false");
        kv("active_controller", s.active_controller);
        arr.status.push_back(st);
        diag_pub.publish(arr);
      });

  ros::AsyncSpinner spinner(3);  // subs + services + action (decision 13)
  spinner.start();

  if (!runtime->start()) {
    ROS_ERROR("soft_robot_manager: runtime failed to start");
    return 1;
  }
  cal_server->start();

  // Preload both controllers so the READY precondition can latch. Retry
  // in the background until the controller_manager is up.
  std::thread preload([&] {
    ros::Rate r(1.0);
    while (ros::ok()) {
      if (loadController(cfg.compliance_controller) &&
          loadController(cfg.correction_controller)) {
        runtime->setControllersLoaded(true);
        ROS_INFO("soft_robot_manager: controllers loaded");
        return;
      }
      r.sleep();
    }
  });

  ROS_INFO("soft_robot_manager: up (payload file: %s)", payload_file.c_str());
  ros::waitForShutdown();
  runtime->stop();
  preload.join();
  return 0;
}
```

实现注意(评审点):
- `load_controller` 对已加载控制器返回 ok=false——preload 线程在 bringup 中先于任何 spawner 运行,launch(Task 8)**不用** spawner 拉起这两个控制器,由 manager 独占加载权(决策 4/7 的配套;launch 注释写明)。
- action executeCb 中 `runtime->cancelCalibration()` 后继续轮询至 teardown 完成,终态 FAILED(CANCELLED)走 setAborted(preempt 语义足够,Plan 6 按 result.message 区分)。
- `ops.sendMotionGoal` 的 done-lambda 捕获 `runtime`(unique_ptr 引用),回调在 action client 的内部线程执行,`onMotionResult` 是线程安全 feed。

- [ ] **Step 4: 构建 + 静态检查**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make 2>&1 | grep -ci warning   # 0
./devel/lib/soft_force_control_manager/soft_robot_manager --help 2>&1 | head -2 || true  # links & runs (ros::init fails without master; exit is fine)
```

- [ ] **Step 5: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_force_control_manager
git commit -m "feat(manager): soft_robot_manager node shell + manager/calibration yaml (Plan5 T7)"
```

**验收标准:** 节点零警告构建、可链接;两个 yaml 键与 runtime/节点参数一一对应;static_assert 钉死状态数值;服务/action/话题名与决策 12 逐字一致;launch 侧约束(manager 独占 load)已在注释与本计划 Task 8 落实。

---

### Task 8: `soft_robot_bringup`(launch 双入口 + README 冒烟清单)

**目标:** 部署层:`soft_robot.launch`(实机)与 `sim.launch`(三 mock 闭环)、payload.yaml 开机装载、README 冒烟清单(含 Plan 4 跟进 8 的 SRI roscore 冒烟欠账,本任务执行一次并记录)。launch 无法 gtest,验证 = 构建 + `roslaunch --check`(仅静态语法,不起 master 之外的进程;后接手动冒烟)。

**Files:**
- Create: `ros_ws/src/soft_robot_bringup/package.xml`
- Create: `ros_ws/src/soft_robot_bringup/CMakeLists.txt`
- Create: `ros_ws/src/soft_robot_bringup/launch/soft_robot.launch`
- Create: `ros_ws/src/soft_robot_bringup/launch/sim.launch`
- Create: `ros_ws/src/soft_robot_bringup/README.md`

**Interfaces:**
- Consumes:全部前序节点/可执行/配置;Task 2 的 `--port`;Task 7 的 manager 独占 load 约束。
- Produces:`roslaunch soft_robot_bringup soft_robot.launch kuka_ip:=... sensor_ip:=...`、`roslaunch soft_robot_bringup sim.launch`;`config/payload.yaml`(标定产物落点,git 不入库——`.gitignore` 记入)。

- [ ] **Step 1: 包骨架**

`package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>soft_robot_bringup</name>
  <version>0.1.0</version>
  <description>
    Launch files, configuration loading, and smoke-test documentation for
    the soft robot system (spec section 5.8): real-robot and simulated
    (three-mock) bringup entries.
  </description>
  <maintainer email="dev@example.com">ljj</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>kuka_rsi_hw_interface</exec_depend>
  <exec_depend>kuka_eki_bridge</exec_depend>
  <exec_depend>sri_force_torque_driver</exec_depend>
  <exec_depend>soft_robot_controllers</exec_depend>
  <exec_depend>soft_force_control_manager</exec_depend>
</package>
```

`CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(soft_robot_bringup)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)
```

并建空目录占位 `ros_ws/src/soft_robot_bringup/config/.gitignore`,内容:

```text
# payload.yaml is a calibration product, machine-specific: never commit.
payload.yaml
```

- [ ] **Step 2: 实机入口**

`launch/soft_robot.launch`:

```xml
<launch>
  <!-- Real-robot bringup (spec 5.8). The KRC connects as RSI UDP client
       and serves EKI TCP; the SRI box serves TCP (Plan 4 decisions 1, 5). -->
  <arg name="kuka_ip"   default="192.168.1.10"/>
  <arg name="sensor_ip" default="192.168.1.1"/>

  <!-- Controller parameters + calibrated payload override (decision 7).
       payload.yaml is written by the manager after calibration; loading
       it AFTER the controller defaults makes the calibration win. -->
  <rosparam command="load"
            file="$(find soft_robot_controllers)/config/soft_robot_controllers.yaml"/>
  <rosparam command="load"
            file="$(find soft_robot_bringup)/config/payload.yaml"
            if="$(eval __import__('os').path.exists(__import__('rospkg').RosPack().get_path('soft_robot_bringup') + '/config/payload.yaml'))"/>

  <!-- RSI hardware interface. Runs the controller_manager inside; keeps a
       SINGLE-threaded callback queue (AsyncSpinner(1) in the node): the
       controllers' mode_seq_/goal_seq_ producers rely on it (Plan 3
       follow-up 5). Do NOT switch this node to a multithreaded spinner. -->
  <node pkg="kuka_rsi_hw_interface" type="kuka_rsi_hw_interface_node"
        name="kuka_rsi_hw_interface" output="screen">
    <rosparam command="load"
              file="$(find kuka_rsi_hw_interface)/config/kuka_rsi.yaml"/>
  </node>

  <!-- The Plan 4/5 package yamls carry their node name as the top-level
       key (sri_ft_driver:/kuka_eki_bridge:/soft_robot_manager:): load
       them at ROOT level so the keys land in the node's private ns.
       kuka_rsi.yaml is flat and loads inside its node tag instead. -->
  <rosparam command="load"
            file="$(find kuka_eki_bridge)/config/kuka_eki.yaml"/>
  <rosparam command="load"
            file="$(find sri_force_torque_driver)/config/sri_ft.yaml"/>
  <rosparam command="load"
            file="$(find soft_force_control_manager)/config/manager.yaml"/>
  <rosparam command="load"
            file="$(find soft_force_control_manager)/config/calibration.yaml"/>

  <!-- EKI management bridge (single-threaded spin by design; the manager
       tolerates its publish gaps through eki_state_timeout_s, decision 3). -->
  <node pkg="kuka_eki_bridge" type="eki_bridge_node"
        name="kuka_eki_bridge" output="screen">
    <param name="kuka_ip" value="$(arg kuka_ip)"/>
  </node>

  <!-- SRI force/torque driver. -->
  <node pkg="sri_force_torque_driver" type="sri_driver_node"
        name="sri_ft_driver" output="screen">
    <param name="sensor_ip" value="$(arg sensor_ip)"/>
  </node>

  <!-- System manager. Owns controller loading exclusively (Task 7): no
       spawner for the two soft_robot controllers anywhere in this file. -->
  <node pkg="soft_force_control_manager" type="soft_robot_manager"
        name="soft_robot_manager" output="screen">
    <param name="payload_file"
           value="$(find soft_robot_bringup)/config/payload.yaml"/>
  </node>
</launch>
```

注意:`rosparam ... if=$(eval ...)` 行若 `roslaunch --check` 报解析问题,回退方案(执行者按实测选择,README 记录选择):删除 `if` 行、改为 manager 启动时自读 `payload_file` 并 `ros::param::set`(Task 7 的 applyPayload 已具备全部代码路径,开机装载复用之)。两方案行为等价,验收以"标定值开机生效"为准。

- [ ] **Step 3: 仿真入口**

`launch/sim.launch`:

```xml
<launch>
  <!-- Three-mock closed loop on 127.0.0.1 (spec 15.3): RSI sim server +
       EKI KRC mock + SRI sensor mock, fixed ports from the package yamls
       (Task 2 --port). Smoke procedure: see README.md. -->
  <rosparam command="load"
            file="$(find soft_robot_controllers)/config/soft_robot_controllers.yaml"/>
  <!-- Node-keyed yamls load at ROOT level (see soft_robot.launch note). -->
  <rosparam command="load"
            file="$(find kuka_eki_bridge)/config/kuka_eki.yaml"/>
  <rosparam command="load"
            file="$(find sri_force_torque_driver)/config/sri_ft.yaml"/>
  <rosparam command="load"
            file="$(find soft_force_control_manager)/config/manager.yaml"/>
  <rosparam command="load"
            file="$(find soft_force_control_manager)/config/calibration.yaml"/>

  <node pkg="kuka_rsi_hw_interface" type="kuka_rsi_hw_interface_node"
        name="kuka_rsi_hw_interface" output="screen">
    <rosparam command="load"
              file="$(find kuka_rsi_hw_interface)/config/kuka_rsi.yaml"/>
    <param name="listen_ip" value="127.0.0.1"/>
  </node>
  <!-- KRC-side RSI mock: paces the 4 ms loop, integrates RKorr into its
       pose (goal-mode convergence base). -->
  <node pkg="kuka_rsi_hw_interface" type="kuka_rsi_sim_server"
        name="kuka_rsi_sim_server" output="screen"
        args="--target-ip 127.0.0.1 --target-port 49152 --cycle-ms 4"/>

  <node pkg="kuka_eki_bridge" type="eki_mock_server"
        name="eki_mock_server" output="screen"
        args="--port 54600 --heartbeat-ms 100"/>
  <node pkg="kuka_eki_bridge" type="eki_bridge_node"
        name="kuka_eki_bridge" output="screen">
    <param name="kuka_ip" value="127.0.0.1"/>
  </node>

  <node pkg="sri_force_torque_driver" type="sri_mock_server"
        name="sri_mock_server" output="screen"
        args="--port 4008 --fz 5"/>
  <node pkg="sri_force_torque_driver" type="sri_driver_node"
        name="sri_ft_driver" output="screen">
    <param name="sensor_ip" value="127.0.0.1"/>
  </node>

  <node pkg="soft_force_control_manager" type="soft_robot_manager"
        name="soft_robot_manager" output="screen">
    <param name="payload_file" value="/tmp/soft_robot_sim_payload.yaml"/>
    <!-- Sim speed-up: 2 poses would fail the n>=4 solve; keep all 8 but
         cut the averaging window. -->
    <param name="calibration/samples_per_pose" value="20"/>
  </node>
</launch>
```

已知仿真局限(README 照录,不是缺陷):`sri_mock_server` 输出恒定 fz=5(不随姿态变化),故 sim 标定的物理意义为"零载荷 + 恒定偏置":拟合期望 **G≈0、bias≈(0,0,5,0,0,0)、r2_force=1**(所有姿态下力向量恒等 ⇒ 力方程 `F_i = -G*r3_i + F0` 的精确解为 G=0、F0=(0,0,5);随姿态变化的项为零)。r2 的 `rSquared` 在 ss_tot=0(样本无方差)时按实现返回 1(ss_res≤1e-12)。这一推导即 sim 冒烟第 7 步的预期值。

- [ ] **Step 4: README 冒烟清单**

`README.md`(全文):

```markdown
# soft_robot_bringup

Two entries:

- `soft_robot.launch` — real robot (KR C5 / iiQKA.OS2 + SRI box).
  Args: `kuka_ip` (default 192.168.1.10), `sensor_ip` (192.168.1.1).
- `sim.launch` — full closed loop against three local mocks. No hardware.

Conventions inherited from the plans:

- Any `rostopic pub -r` example carrying a stamp uses `-s` so `now`
  re-evaluates per message (Plan 3 follow-up 2).
- The two soft_robot controllers are loaded EXCLUSIVELY by the manager;
  never spawn them from launch or by hand while the manager runs.
- The hw-interface node must keep its single-threaded callback queue
  (Plan 3 follow-up 5): do not add spinner threads to it.
- `/sri_ft/zero` is gated by the manager (`/soft_robot/zero_sensor`,
  CONNECTED/READY only). Calling the driver service directly bypasses
  the gate — debugging only, never while SERVOING.

## 1. Build & static check

    cd ros_ws && catkin_make
    roslaunch --check soft_robot_bringup sim.launch   # needs a roscore

## 2. SRI driver smoke (Plan 4 follow-up 8 backlog — run once, record)

    roscore &
    rosrun sri_force_torque_driver sri_mock_server --port 4008 --fz 5 &
    rosrun sri_force_torque_driver sri_driver_node _sensor_ip:=127.0.0.1 &

    rostopic hz /sri_ft/wrench_raw        # expect ~250 Hz
    rostopic echo -n 3 --offset /sri_ft/wrench_raw   # stamp offset: ms-level
    rostopic echo -n 1 /sri_ft/status     # streaming: True
    rosservice call /sri_ft/zero          # success: True ("tare captured")
    rostopic echo -n 1 /sri_ft/wrench_raw # force.z ~ 0 after tare

    kill %2 %3; kill %1                   # zero leftovers: pgrep -af mock

## 3. Sim closed-loop smoke

    roslaunch soft_robot_bringup sim.launch

    # a) manager reaches READY (tool sync + controllers loaded + SRI up):
    rostopic echo -n 1 /soft_robot/manager_state
    #    system_state: 2, tool_synced: True, sri_streaming: True

    # b) start compliance servo (DRAG):
    rosservice call /soft_robot/start_servo "{mode: 2, profile: 0}"
    rostopic echo -n 1 /soft_robot/manager_state   # system_state: 3
    #    kuka_rsi_sim_server log: pose x drifts (fz=5 drives +z force law)

    # c) zero gate while SERVOING (must fail):
    rosservice call /soft_robot/zero_sensor        # success: False

    # d) stop:
    rosservice call /soft_robot/stop_servo         # back to system_state: 2

    # e) fault + reset: kill the RSI sim server process, wait for
    #    system_state: 6, restart it, then:
    rosservice call /soft_robot/reset_fault        # -> READY again

    # f) calibration (expected fit for the constant-fz mock: G ~ 0,
    #    bias ~ (0,0,5), r2_force = 1 — see the derivation in sim.launch):
    rostopic pub -1 /soft_robot/calibrate_payload/goal \
        soft_robot_msgs/CalibratePayloadActionGoal '{}'
    rostopic echo /soft_robot/calibrate_payload/feedback   # phases cycle
    rostopic echo -n 1 /soft_robot/calibrate_payload/result
    cat /tmp/soft_robot_sim_payload.yaml                   # gravity_n ~ 0

    # g) shutdown; verify zero leftovers:
    pgrep -af 'mock_server|sim_server'   # empty

## 4. Real robot

Follow `docs/commissioning_checklist.md` stage by stage. Never skip the
zero-output stages (2, 5).
```

- [ ] **Step 5: 构建 + 语法检查 + 执行冒烟**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make 2>&1 | tail -2
roscore &  sleep 2
roslaunch --check soft_robot_bringup soft_robot.launch
roslaunch --check soft_robot_bringup sim.launch
kill %1
```

预期:两 launch 语法通过(若 `if=$(eval ...)` 行报错,按 Step 2 注明的回退方案调整并在 README 记录)。随后按 README §2、§3 手动执行冒烟并把结果记入任务报告(§2 是 Plan 4 欠账,必须执行;§3 a-g 全过为验收)。

- [ ] **Step 6: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/soft_robot_bringup
git commit -m "feat(bringup): real + sim launch entries and smoke checklist (Plan5 T8)"
```

**验收标准:** 双 launch 静态检查通过;sim 冒烟 a-g 全过(READY→SERVOING→zero 拒绝→stop→fault/reset→标定 G≈0/bias_fz≈5/r2_force=1→零残留);SRI roscore 冒烟(Plan 4 欠账)执行完毕并记录;README 含四条继承约束。

---

### Task 9: KUKA 侧模板(KRL / RSI 配置 / EkiConfig.xml)+ 人工核对清单

**目标:** 规格 §6 的 KUKA 侧交付物,全部为文本模板:`ROS_EKI_CONFIG.xml` 逐字段以 `eki_frame.h` 为唯一权威;RSI 配置以 `rsi_frame.h` 的 Rob/Sen schema 为准;KRL 程序含 **100 ms 周期心跳义务**(Plan 4 遗留风险 8)。无法自动测试 ⇒ 交付"人工核对清单"(`docs/kuka_iiqka_rsi_eki_setup.md`),要求装机者对照 `ref/` 两本手册与真实 WorkVisual/iiQKA 工程逐条打钩。**模板中 KRL/RSI 语法细节(函数签名、RSI 对象名)以手册为准,核对清单第一节即"语法对照",装机时允许按手册修正语法而不回改 schema。**

**Files:**
- Create: `kuka/eki/ROS_EKI_CONFIG.xml`
- Create: `kuka/krl/ROS_RSI_SERVO.SRC`
- Create: `kuka/krl/ROS_RSI_SERVO.DAT`
- Create: `kuka/rsi/ROS_RSI_ETHERNET.xml`
- Create: `kuka/rsi/ROS_RSI_CONTEXT.notes.md`
- Create: `docs/kuka_iiqka_rsi_eki_setup.md`

**Interfaces:**
- Consumes(逐字段权威,不得另行发明):`eki_frame.h` 的动作码 0~6、错误码 0/1/2、`<RobotCommand><Cmd Seq Action Value/><Tool X..C/><Base X..C/></RobotCommand>`、`<RobotState><Ack Seq Ok Code/><Prog Ready RsiActive Fault Mode/><Tool X..C/></RobotState>`、`Ack.Seq=0`=心跳;`rsi_frame.h` 的 `<Rob Type="KUKA">` RIst/AIPos/Delay/Mode/IPOC 与 `<Sen Type="ROS">` RKorr/Stop/Watchdog/IPOC;Plan 4 决策 5 连接方向(KRC=EKI TCP server 54600,EXTERNAL TYPE=Client);`kuka_rsi.yaml` 端口 49152。

- [ ] **Step 1: `ROS_EKI_CONFIG.xml`**

```xml
<ETHERNETKRL>
  <!-- EKI channel config for the ROS management link (EthernetKRL 6.1).
       GENERATED FROM ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/
       eki_frame.h - the single source of truth for this schema (Plan 4
       decision 6). Field-by-field checklist:
       docs/kuka_iiqka_rsi_eki_setup.md section 3. -->
  <CONFIGURATION>
    <EXTERNAL>
      <!-- The external system (ROS bridge) connects as TCP client
           (Plan 4 decision 5). -->
      <TYPE>Client</TYPE>
    </EXTERNAL>
    <INTERNAL>
      <ENVIRONMENT>Program</ENVIRONMENT>
      <BUFFERING Mode="FIFO" Limit="16"/>
      <ALIVE Set_Flag="1"/>
      <IP>0.0.0.0</IP>
      <PORT>54600</PORT>
      <PROTOCOL>TCP</PROTOCOL>
    </INTERNAL>
  </CONFIGURATION>
  <RECEIVE>
    <!-- ROS -> KRC: RobotCommand (eki_frame.h serializeCommand).
         Every command carries all elements; unused ones are zero. -->
    <XML>
      <ELEMENT Tag="RobotCommand/Cmd" Attribute="Seq"    Type="INT"/>
      <ELEMENT Tag="RobotCommand/Cmd" Attribute="Action" Type="INT"/>
      <ELEMENT Tag="RobotCommand/Cmd" Attribute="Value"  Type="INT"/>
      <ELEMENT Tag="RobotCommand/Tool" Attribute="X" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Tool" Attribute="Y" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Tool" Attribute="Z" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Tool" Attribute="A" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Tool" Attribute="B" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Tool" Attribute="C" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Base" Attribute="X" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Base" Attribute="Y" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Base" Attribute="Z" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Base" Attribute="A" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Base" Attribute="B" Type="REAL"/>
      <ELEMENT Tag="RobotCommand/Base" Attribute="C" Type="REAL"/>
    </XML>
  </RECEIVE>
  <SEND>
    <!-- KRC -> ROS: RobotState (eki_frame.h parseState). Ack.Seq=0 marks
         the unsolicited 100 ms heartbeat (Plan 4 decision 7). All three
         elements and every attribute are mandatory: the ROS parser
         rejects the frame otherwise. -->
    <XML>
      <ELEMENT Tag="RobotState/Ack"  Attribute="Seq"       Type="INT"/>
      <ELEMENT Tag="RobotState/Ack"  Attribute="Ok"        Type="INT"/>
      <ELEMENT Tag="RobotState/Ack"  Attribute="Code"      Type="INT"/>
      <ELEMENT Tag="RobotState/Prog" Attribute="Ready"     Type="INT"/>
      <ELEMENT Tag="RobotState/Prog" Attribute="RsiActive" Type="INT"/>
      <ELEMENT Tag="RobotState/Prog" Attribute="Fault"     Type="INT"/>
      <ELEMENT Tag="RobotState/Prog" Attribute="Mode"      Type="INT"/>
      <ELEMENT Tag="RobotState/Tool" Attribute="X" Type="REAL"/>
      <ELEMENT Tag="RobotState/Tool" Attribute="Y" Type="REAL"/>
      <ELEMENT Tag="RobotState/Tool" Attribute="Z" Type="REAL"/>
      <ELEMENT Tag="RobotState/Tool" Attribute="A" Type="REAL"/>
      <ELEMENT Tag="RobotState/Tool" Attribute="B" Type="REAL"/>
      <ELEMENT Tag="RobotState/Tool" Attribute="C" Type="REAL"/>
    </XML>
  </SEND>
</ETHERNETKRL>
```

- [ ] **Step 2: KRL 程序模板**

`kuka/krl/ROS_RSI_SERVO.SRC`:

```text
&ACCESS RVP
&REL 1
DEF ROS_RSI_SERVO()
  ; ROS soft-robot servo program (spec 6.3) for iiQKA.OS2 9.2+ with
  ; RobotSensorInterface 6.2 and EthernetKRL 6.1. TEMPLATE: verify every
  ; function signature against the local manuals before deployment
  ; (docs/kuka_iiqka_rsi_eki_setup.md section 2).
  ;
  ; Action codes / XML schema mirror kuka_eki_bridge/eki_frame.h:
  ;   0 QUERY_STATE  1 START_RSI  2 STOP_RSI  3 SET_MODE
  ;   4 RESET_FAULT  5 GET_TOOL   6 SET_TOOL_BASE
  ; Error codes: 0 OK, 1 NOT_READY, 2 FAULTED.

  DECL INT nSeq, nAction, nValue
  DECL REAL rHbTimer

  ; -- 1. select configured tool/base ------------------------------------
  $TOOL = TOOL_DATA[ROS_TOOL_NO]
  $BASE = BASE_DATA[ROS_BASE_NO]

  ; -- 2. open the EKI management channel --------------------------------
  RET = EKI_Load("ROS_EKI_CONFIG")
  RET = EKI_Open("ROS_EKI_CONFIG")
  bReady = TRUE
  bRsiActive = FALSE
  bFault = FALSE
  nMode = 0

  ; -- 3. main loop: answer commands, push the 100 ms heartbeat ----------
  ; HEARTBEAT OBLIGATION (Plan 4 risk 8): without the periodic push the
  ; ROS bridge reports "state heartbeat stale" (WARN) and the manager
  ; never leaves OFFLINE. Period 100 ms = 10x margin vs the 1 s bridge
  ; freshness threshold.
  rHbTimer = 0.0
  LOOP
    ; --- command path (non-blocking check) ---
    IF EKI_CheckBuffer("ROS_EKI_CONFIG", "RobotCommand/Cmd") > 0 THEN
      RET = EKI_ReadNext("ROS_EKI_CONFIG")
      RET = EKI_GetInt("ROS_EKI_CONFIG", "RobotCommand/Cmd@Seq", nSeq)
      RET = EKI_GetInt("ROS_EKI_CONFIG", "RobotCommand/Cmd@Action", nAction)
      RET = EKI_GetInt("ROS_EKI_CONFIG", "RobotCommand/Cmd@Value", nValue)
      SWITCH nAction
        CASE 0  ; QUERY_STATE: ack with the current state, no side effect
          ROS_SEND_STATE(nSeq, 1, 0)
        CASE 1  ; START_RSI
          IF bFault THEN
            ROS_SEND_STATE(nSeq, 0, 2)   ; FAULTED
          ELSE
            RET = RSI_LOAD("ROS_RSI_CONTEXT")
            RET = RSI_ACTIVATE()
            ; Relative Cartesian correction in the configured frame
            ; (spec 6.1: PosCorr, BASE by default).
            RET = RSI_PROCESS_ON(#ABSOLUTE_OFF)
            bRsiActive = TRUE
            ROS_SEND_STATE(nSeq, 1, 0)
            RSI_MOVECORR()               ; sensor-guided motion (blocks)
            bRsiActive = FALSE
          ENDIF
        CASE 2  ; STOP_RSI
          RET = RSI_PROCESS_OFF()
          RET = RSI_DEACTIVATE()
          RET = RSI_UNLOAD()
          bRsiActive = FALSE
          ROS_SEND_STATE(nSeq, 1, 0)
        CASE 3  ; SET_MODE: opaque to the KRC, stored and echoed
          nMode = nValue
          ROS_SEND_STATE(nSeq, 1, 0)
        CASE 4  ; RESET_FAULT: clear the program-level fault latch
          bFault = FALSE
          ROS_SEND_STATE(nSeq, 1, 0)
        CASE 5  ; GET_TOOL: current $TOOL is packed by ROS_SEND_STATE
          ROS_SEND_STATE(nSeq, 1, 0)
        CASE 6  ; SET_TOOL_BASE: read Tool/Base elements, apply
          ROS_READ_TOOL_BASE()
          ROS_SEND_STATE(nSeq, 1, 0)
        DEFAULT
          ROS_SEND_STATE(nSeq, 0, 1)     ; NOT_READY / unknown action
      ENDSWITCH
    ENDIF
    ; --- heartbeat path ---
    IF $TIMER[ROS_HB_TIMER_NO] > 100 THEN   ; ms
      $TIMER[ROS_HB_TIMER_NO] = 0
      ROS_SEND_STATE(0, 1, 0)               ; Ack.Seq=0 = heartbeat
    ENDIF
    WAIT SEC 0.01
  ENDLOOP
END

DEF ROS_SEND_STATE(nAckSeq :IN, nOk :IN, nCode :IN)
  DECL INT nAckSeq, nOk, nCode
  ; Every RobotState carries all three elements with all attributes
  ; (eki_frame.h parseState treats anything less as a bad frame).
  RET = EKI_SetInt("ROS_EKI_CONFIG", "RobotState/Ack@Seq", nAckSeq)
  RET = EKI_SetInt("ROS_EKI_CONFIG", "RobotState/Ack@Ok", nOk)
  RET = EKI_SetInt("ROS_EKI_CONFIG", "RobotState/Ack@Code", nCode)
  RET = EKI_SetInt("ROS_EKI_CONFIG", "RobotState/Prog@Ready", B_TO_I(bReady))
  RET = EKI_SetInt("ROS_EKI_CONFIG", "RobotState/Prog@RsiActive", B_TO_I(bRsiActive))
  RET = EKI_SetInt("ROS_EKI_CONFIG", "RobotState/Prog@Fault", B_TO_I(bFault))
  RET = EKI_SetInt("ROS_EKI_CONFIG", "RobotState/Prog@Mode", nMode)
  RET = EKI_SetReal("ROS_EKI_CONFIG", "RobotState/Tool@X", $TOOL.X)
  RET = EKI_SetReal("ROS_EKI_CONFIG", "RobotState/Tool@Y", $TOOL.Y)
  RET = EKI_SetReal("ROS_EKI_CONFIG", "RobotState/Tool@Z", $TOOL.Z)
  RET = EKI_SetReal("ROS_EKI_CONFIG", "RobotState/Tool@A", $TOOL.A)
  RET = EKI_SetReal("ROS_EKI_CONFIG", "RobotState/Tool@B", $TOOL.B)
  RET = EKI_SetReal("ROS_EKI_CONFIG", "RobotState/Tool@C", $TOOL.C)
  RET = EKI_Send("ROS_EKI_CONFIG", "RobotState")
END

DEF ROS_READ_TOOL_BASE()
  ; SET_TOOL_BASE payload -> $TOOL/$BASE (mm/deg, KUKA A/B/C).
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Tool@X", $TOOL.X)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Tool@Y", $TOOL.Y)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Tool@Z", $TOOL.Z)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Tool@A", $TOOL.A)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Tool@B", $TOOL.B)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Tool@C", $TOOL.C)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Base@X", $BASE.X)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Base@Y", $BASE.Y)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Base@Z", $BASE.Z)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Base@A", $BASE.A)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Base@B", $BASE.B)
  RET = EKI_GetReal("ROS_EKI_CONFIG", "RobotCommand/Base@C", $BASE.C)
END
```

`kuka/krl/ROS_RSI_SERVO.DAT`:

```text
&ACCESS RVP
&REL 1
DEFDAT ROS_RSI_SERVO
  ; Deployment-site constants. Adjust per cell; checklist section 2.
  DECL INT ROS_TOOL_NO = 1        ; TOOL_DATA index used at startup
  DECL INT ROS_BASE_NO = 0        ; BASE_DATA index (0 = $WORLD)
  DECL INT ROS_HB_TIMER_NO = 1    ; $TIMER used for the 100 ms heartbeat
  DECL BOOL bReady = FALSE
  DECL BOOL bRsiActive = FALSE
  DECL BOOL bFault = FALSE
  DECL INT nMode = 0
  DECL INT RET = 0
ENDDAT
```

- [ ] **Step 3: RSI 配置模板**

`kuka/rsi/ROS_RSI_ETHERNET.xml`:

```xml
<ROOT>
  <!-- RSI Ethernet object config (RobotSensorInterface 6.2). GENERATED
       FROM ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/
       rsi_frame.h (spec 6.1). The KRC is the UDP client: it sends <Rob>
       state at the 4 ms cycle to the ROS host and expects the <Sen>
       answer echoing IPOC within the same cycle. -->
  <CONFIG>
    <IP_NUMBER>192.168.1.2</IP_NUMBER>  <!-- ROS host IP: SITE-SPECIFIC -->
    <PORT>49152</PORT>                  <!-- must match kuka_rsi.yaml listen_port -->
    <SENTYPE>ROS</SENTYPE>              <!-- <Sen Type="ROS"> -->
    <ONLYSEND>FALSE</ONLYSEND>
  </CONFIG>
  <SEND>
    <!-- KRC -> ROS, parsed by rsi_frame.h parseRobFrame: RIst + AIPos
         + IPOC mandatory; Delay/Mode optional. -->
    <ELEMENTS>
      <ELEMENT TAG="RIst"  TYPE="DOUBLE" INDX="INTERNAL" ATTRIBUTES="X Y Z A B C"/>
      <ELEMENT TAG="AIPos" TYPE="DOUBLE" INDX="INTERNAL" ATTRIBUTES="A1 A2 A3 A4 A5 A6"/>
      <ELEMENT TAG="Delay" TYPE="LONG"   INDX="INTERNAL" ATTRIBUTES="D"/>
      <ELEMENT TAG="Mode"  TYPE="LONG"   INDX="INTERNAL" ATTRIBUTES="M"/>
      <ELEMENT TAG="IPOC"  TYPE="LONG"   INDX="INTERNAL"/>
    </ELEMENTS>
  </SEND>
  <RECEIVE>
    <!-- ROS -> KRC, produced by rsi_frame.h serializeSenFrame. RKorr is
         a per-cycle Cartesian correction consumed by a PosCorr object in
         RELATIVE mode, coordinate system BASE (spec 6.1). The KUKA-side
         context must clamp corrections (secondary limits live in ROS). -->
    <ELEMENTS>
      <ELEMENT TAG="RKorr" TYPE="DOUBLE" INDX="1 2 3 4 5 6" ATTRIBUTES="X Y Z A B C" HOLDON="1"/>
      <ELEMENT TAG="Stop"     TYPE="BOOL" INDX="7" ATTRIBUTES="S" HOLDON="1"/>
      <ELEMENT TAG="Watchdog" TYPE="LONG" INDX="8" ATTRIBUTES="W" HOLDON="0"/>
      <ELEMENT TAG="IPOC"     TYPE="LONG" INDX="INTERNAL"/>
    </ELEMENTS>
  </RECEIVE>
</ROOT>
```

`kuka/rsi/ROS_RSI_CONTEXT.notes.md`:

```markdown
# ROS_RSI_CONTEXT — signal-flow notes (build in RSI Visual / iiQKA tooling)

The .rsi context file is authored graphically; this note pins what it
must contain (checklist section 4):

1. ETHERNET object using ROS_RSI_ETHERNET.xml (4 ms cycle, timeout
   behavior = stop with zero correction after N missed answers).
2. RKorr X/Y/Z/A/B/C inputs -> per-axis clamp blocks (KUKA-side hard
   limit, spec 12.3; ROS applies its own limits before sending) ->
   POSCORR object, mode RELATIVE, correction coordinate system BASE
   (spec 6.1 default; TOOL stays a configurable experiment).
3. Stop S input -> stop/brake path (PC-requested stop, e.g. latched
   fault in the hw interface).
4. Watchdog W input: monotonically increasing PC liveness counter;
   wire to a timeout monitor if the deployment wants PC-side liveness
   enforced on the KRC too.
5. POSCORR limits: configure the maximum overall correction envelope
   (mm/deg) allowed by the cell safety concept.
```

- [ ] **Step 4: 安装 + 人工核对清单文档**

`docs/kuka_iiqka_rsi_eki_setup.md`(结构如下,执行者按此写全;每条 checklist 均为可打钩的 `- [ ]` 项):

```markdown
# KUKA iiQKA RSI/EKI Setup & Field-by-Field Checklist

## 1. Prerequisites
- [ ] KR C5, iiQKA.OS2 >= 9.2; iiQKA.RobotSensorInterface 6.2;
      iiQKA.EthernetKRL 6.1 (spec section 6 floors).
- [ ] Network plan: KRC <-> ROS host on one subnet; RSI UDP 49152
      (kuka_rsi.yaml), EKI TCP 54600 (kuka_eki.yaml).

## 2. KRL syntax verification (do FIRST, against ref/ manuals)
- [ ] Every EKI_* call in ROS_RSI_SERVO.SRC matches the EthernetKRL 6.1
      manual signature (EKI_Load/Open/CheckBuffer/ReadNext/GetInt/GetReal/
      SetInt/SetReal/Send/Close/Unload). Fix syntax here, NOT the schema.
- [ ] Every RSI_* call matches the RobotSensorInterface 6.2 manual
      (RSI_LOAD/ACTIVATE/PROCESS_ON/MOVECORR/PROCESS_OFF/DEACTIVATE/
      RSI_UNLOAD) including the PROCESS_ON relative-mode argument.
- [ ] B_TO_I helper: replace with the manual-blessed BOOL->INT idiom if
      unavailable on the target KSS.
- [ ] $TIMER usage and timer number ROS_HB_TIMER_NO free on this cell.

## 3. EkiConfig field-by-field (vs eki_frame.h — the single authority)
For EACH row confirm XML path, attribute name (case!), and type:
- [ ] RECEIVE RobotCommand/Cmd @Seq @Action @Value (INT x3)
- [ ] RECEIVE RobotCommand/Tool @X @Y @Z @A @B @C (REAL x6)
- [ ] RECEIVE RobotCommand/Base @X @Y @Z @A @B @C (REAL x6)
- [ ] SEND RobotState/Ack @Seq @Ok @Code (INT x3)
- [ ] SEND RobotState/Prog @Ready @RsiActive @Fault @Mode (INT x4)
- [ ] SEND RobotState/Tool @X @Y @Z @A @B @C (REAL x6)
- [ ] Action codes in the SRC SWITCH: 0..6 in eki_frame.h order.
- [ ] Error codes: 0 OK / 1 NOT_READY / 2 FAULTED only.
- [ ] EXTERNAL TYPE=Client, INTERNAL PORT=54600/TCP (decision 5).
- [ ] Heartbeat: Ack.Seq=0 every 100 ms while the program runs
      (bridge freshness threshold 1 s; manager threshold 5 s).

## 4. RSI context (vs rsi_frame.h)
- [ ] SEND: RIst(X..C), AIPos(A1..A6), Delay(D), Mode(M), IPOC.
- [ ] RECEIVE: RKorr(X..C), Stop(S), Watchdog(W), IPOC echo.
- [ ] SENTYPE="ROS"; PORT=49152; ROS host IP set.
- [ ] POSCORR: RELATIVE mode, coordinate system BASE, clamp limits set.
- [ ] Timeout behavior: zero correction + stop after missed answers.

## 5. First-contact smoke (with the ROS side up, robot in T1)
- [ ] eki: rostopic echo /kuka/eki/state -> connected+state_fresh true,
      heartbeat age < 0.2 s.
- [ ] rosservice call /kuka/eki/get_tool returns the pendant's $TOOL.
- [ ] START_RSI from the manager; /kuka/rsi/state connected: True,
      zero-output loop stable (commissioning checklist stage 2).
```

- [ ] **Step 5: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add kuka docs/kuka_iiqka_rsi_eki_setup.md
git commit -m "docs(kuka): KRL/RSI/EKI templates + field-by-field manual checklist (Plan5 T9)"
```

**验收标准(评审型,非测试型):** ① EkiConfig 与 `eki_frame.h` 逐字段对照零出入(元素/属性/大小写/类型/方向);② KRL 含 100 ms 心跳、7 个动作码全实现、RobotState 恒满编;③ RSI XML 与 `rsi_frame.h` 字段一致、端口/SENTYPE 与 yaml/代码一致;④ 核对清单覆盖语法核对(第一节)+ 字段核对 + 首联冒烟;⑤ 模板显式标注 GENERATED FROM 权威头文件。

---

### Task 10: 真机联调核对单 + 整仓回归收尾

**目标:** ① `docs/commissioning_checklist.md`——规格 §15.4 九阶段联调核对单,并逐条并入跟进义务(stamp `--offset` 抽查、间隔统计用 `package_gaps`、投运调参指引、DRAG deadband 解释验证、SRI 线协议真机核对);② 整仓干净重建 + 全量回归 + 新增二进制 3 连跑 + 零残留,按逐包预期表核对。

**Files:**
- Create: `docs/commissioning_checklist.md`

- [ ] **Step 1: 写联调核对单**

`docs/commissioning_checklist.md`(结构如下,执行者写全为可打钩清单;每阶段"预期/中止条件"必填):

```markdown
# Real-Robot Commissioning Checklist (spec 15.4 + plan follow-ups)

Robot in T1, reduced speed, e-stop within reach for every stage.

## Stage 0 - protocol reality checks (before any motion)
- [ ] SRI wire protocol vs the real box (Plan 4 risk 1): frame 0xAA 0x55,
      big-endian length 27, PN big-endian, float32 little-endian, checksum
      = sum of 24 data bytes & 0xFF. On mismatch fix sri_frame.h constants
      (single location) and rerun the sri package tests.
- [ ] rostopic echo --offset /sri_ft/wrench_raw: stamp-vs-arrival offset
      at ms level (Plan 4 follow-up 9). NOTE: stamps are reception times;
      use SriStatus.package_gaps for interval statistics, never stamps.
- [ ] /kuka/eki/state heartbeat: age < 0.2 s sustained for 1 min.

## Stage 1 - EKI handshake & tool query        (spec 15.4 step 1)
## Stage 2 - RSI zero-output loop              (step 2; 10 min soak,
      RsiState: total_timeouts stable, ipoc_jumps == 0)
## Stage 3 - small DIRECT_CARTESIAN stream jog (step 3; use
      `rostopic pub -s -r 100` — the -s is mandatory, Plan 3 follow-up 2)
## Stage 4 - goal-mode orientation move        (step 4; conservative yaml
      defaults p_gain 1.0 / 5 dps are intentional (Plan 3 follow-up 9);
      tune per spec 14 AFTER a clean run, record final values)
## Stage 5 - SRI wrench display only           (step 5)
## Stage 6 - PRECISION low-gain compliance     (step 6)
## Stage 7 - DRAG adaptive deadband            (step 7): VERIFY the
      FLim/TLim activation-deadband interpretation (spec 7.2 note); if
      they behaved as cutoffs on the legacy cell, only the deadband
      initialization mapping changes.
## Stage 8 - payload calibration               (step 8): after success
      check payload.yaml, r2_force/r2_torque >= 0.99 on a rigid tool;
      re-run reproducibility: gravity_n within 2%.
## Stage 9 - full workflow through the manager (step 9; web UI in Plan 6)

## Recovery drills (do all three)
- [ ] Kill RSI mid-SERVOING -> FAULT -> /soft_robot/reset_fault path.
- [ ] EKI cable pull -> manager OFFLINE within eki_state_timeout_s.
- [ ] /soft_robot/zero_sensor rejected while SERVOING (decision 11).
```

- [ ] **Step 2: 整仓干净重建 + 全量回归**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws
rm -rf build devel
catkin_make 2>&1 | grep -ci warning        # expect 0
catkin_make run_tests > /tmp/plan5_tests.log 2>&1; echo $?   # expect 0
for d in build/test_results/*/; do
  pkg=$(basename $d)
  n=$(grep -hoP 'tests="\K[0-9]+' $d/gtest-*.xml 2>/dev/null | paste -sd+ | bc)
  f=$(grep -hoP 'failures="\K[0-9]+' $d/gtest-*.xml 2>/dev/null | paste -sd+ | bc)
  echo "$pkg tests=$n failures=$f"
done
```

预期逐包(XML 直读,验收清单同表):core 52 / rsi 58 / msgs 8 / controllers 61 / sri 44 / eki 39 / manager 29,合计 **291**,failures/errors 全 0。

- [ ] **Step 3: 新增/触动二进制 3 连跑 + 零残留**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws
for i in 1 2 3; do
  ./devel/lib/kuka_rsi_hw_interface/test_rsi_session_monitor && \
  ./devel/lib/kuka_rsi_hw_interface/test_kuka_rsi_robot_hw && \
  ./devel/lib/sri_force_torque_driver/test_sri_stream_session && \
  ./devel/lib/sri_force_torque_driver/test_sri_mock_server && \
  ./devel/lib/kuka_eki_bridge/test_eki_mock_server && \
  ./devel/lib/kuka_eki_bridge/test_eki_bridge_runtime && \
  ./devel/lib/soft_robot_msgs/test_msgs && \
  ./devel/lib/soft_force_control_manager/test_manager_system_state && \
  ./devel/lib/soft_force_control_manager/test_manager_calibration_sequencer && \
  ./devel/lib/soft_force_control_manager/test_manager_payload_yaml && \
  ./devel/lib/soft_force_control_manager/test_manager_runtime || break
done
pgrep -af 'mock_server|sim_server' ; ss -tln | grep -E '4008|54600|49152'  # all empty
```

- [ ] **Step 4: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add docs/commissioning_checklist.md
git commit -m "docs: commissioning checklist + whole-repo regression record (Plan5 T10)"
```

**验收标准:** 核对单含 Stage 0~9 + 三个恢复演练,跟进义务(stamp 抽查/间隔统计口径/-s 规则/保守参数调参/deadband 解释验证/SRI 线协议核对)逐条在文;干净重建 0 警告;整仓 291/291 零失败;11 个二进制 3 连跑无 flaky;零残留。

---

## 验收清单(整计划)

**代码/测试门槛(自动):**

1. 干净重建(`rm -rf build devel && catkin_make`)零警告(-Wall -Wextra)。
2. `catkin_make run_tests` 后逐包 XML 直读(不用 catkin_test_results):

| 包 | 基线 | 本计划新增 | 预期 tests | failures/errors |
|---|---|---|---|---|
| soft_force_control_core | 52 | 0 | 52 | 0 |
| kuka_rsi_hw_interface | 54 | +4(monitor 2 + robot_hw 2) | 58 | 0 |
| soft_robot_msgs | 6 | +2 | 8 | 0 |
| soft_robot_controllers | 61 | 0 | 61 | 0 |
| sri_force_torque_driver | 42 | +2(session 1 + mock 1) | 44 | 0 |
| kuka_eki_bridge | 37 | +2(runtime 1 + mock 1) | 39 | 0 |
| soft_force_control_manager | — | +29(state 8 + sequencer 8 + yaml 1 + runtime 12;节点壳无 gtest) | 29 | 0 |
| **合计** | **252** | **+39** | **291** | **0** |

   (manager 四个二进制用例数 8/8/1/12 与任务文本内 TEST 逐一对应;执行中若增删用例,最终报告须同步更新本表。)
3. 新增/触动的 11 个 gtest 二进制 3 连跑无 flaky;跑后零残留(无 mock/sim 进程、无 4008/54600/49152 监听)。
4. 所有新 gtest 目标名带包前缀(`test_manager_*`),无全局命名冲突(Plan 4 跟进 1)。
5. 纯逻辑类(system_state_core / calibration_sequencer / payload_yaml)不含 `ros/ros.h`;gtest 无 `ros::Time`;网络类测试仅 127.0.0.1、等待有界。

**行为门槛(自动,gtest 内):**

6. `clearFault()` 保留累计计数且可重锁存(跟进 10)。
7. tare 捕获中断不误报成功(跟进 4/N1)。
8. `connectNow` 超时撤回请求位、不自发重连(跟进 5/N2)。
9. manager:READY 不要求 RSI 帧流、SERVOING 全前置、DEGRADED 自恢复、FAULT 闭锁待 reset、zero 门禁 CONNECTED/READY、标定闭环持久化精确拟合(G=50 至 1e-8)、标定失败不持久化不回程。

**冒烟门槛(手动,Task 8 执行并记录):**

10. SRI roscore 冒烟(Plan 4 跟进 8 欠账):hz≈250、`--offset` 毫秒级、zero 后 fz 归零、`streaming: True`。
11. sim.launch 闭环 a-g:OFFLINE→READY(tool 同步)→ start→SERVOING(sim 位姿漂移)→ zero 拒绝 → stop→READY → 杀 RSI sim→FAULT→reset 恢复 → 标定全程(feedback 相位循环、result G≈0 / bias_fz≈5 / r2_force=1、payload.yaml 落盘)→ 零残留。

**评审门槛(人工,Task 9/10):**

12. KUKA 模板逐字段对照零出入(EkiConfig↔eki_frame.h、RSI XML↔rsi_frame.h);KRL 心跳 100 ms 义务;核对清单三份(语法/字段/首联 + 联调九阶段)完整。
13. 跟进对照表 20 条(plan4 11 + plan3 9)全部"闭环任务可指认"或"排除且理由成立"。

## 遗留风险

1. **KRL 模板语法未经真机验证**:iiQKA 6.x 的 EKI_*/RSI_* 具体签名、`EKI_CheckBuffer` 路径写法、`B_TO_I` 习语均以 `ref/` 手册与装机核对(Task 9 清单第 2 节)为准;schema(元素/属性/动作码)不因语法修正而漂移。缓解:模板标注 GENERATED FROM + 清单第一节强制先核语法。
2. **manager 独占控制器加载**:若现场绕过 manager 用 spawner 拉起控制器,`controllers_loaded` 可能永假(load_controller 对已加载者返回 false)。缓解:launch/README 双处写明;后续可改为 `list_controllers` 探测(留 Plan 6)。
3. **`rosparam if=$(eval ...)` 的 payload.yaml 条件装载**依赖 roslaunch eval 行为,Task 8 已给等价回退方案;验收以"标定值开机生效"行为为准。
4. **标定样本姿态取目标值**(决策 5):若现场把 goal 容差调粗(>0.5°),拟合精度随之退化;联调核对单 Stage 8 的复现性检查(2%)兜底。
5. **DEGRADED 无自动停机**:SERVOING 中链路降级仅零输出+报警,不主动 STOP_RSI(控制器/HW 已各自兜底零输出);操作规程要求人工 stop。记录待 Plan 6 UI 醒目化。
6. **sim 标定的物理意义有限**(恒定 wrench ⇒ G≈0):只是编排/管线验收,不验证拟合物理正确性(gtest 的合成数据已验);真机 Stage 8 才是拟合验收。
7. **排除项跟踪**:SriMock STOP 分支(plan4-7)、`stopping()` 锁语义(plan3-6)、sim-time 回跳(plan3-8)、transport 双副本重构(Plan 4 风险 9)——条件触发时由触发方计划收编。
8. **manager 单实例假定**:命令经进程内 mutex 串行,多 manager 实例并行会竞争 controller_manager;launch 不提供多实例入口。

## 计划自查记录(writing-plans Self-Review)

1. **Spec 覆盖**:§5.5(manager 职责六项:模式切换=T6/T7、start/stop=T6、标定 action=T5/T6/T7、SRI zero 门禁=T6、tool 同步=T6、YAML 持久化=T5/T7、故障确认=T6、诊断聚合=T7)✓;§5.8 bringup=T8 ✓;§6/§6.3/§6.4 KUKA 侧=T9(tool 查询编排=T6)✓;§9 标定八步=T5/T6/T7(第 6-7 步=applyPayload)✓;§11 状态机=T4 ✓;§14 calibration.yaml/payload.yaml=T7/T5 ✓;§15.3 集成验证映射为 sim 冒烟 a-g(rostest 不在门槛,沿革自 Plan 3/4)+ runtime 闭环 gtest ✓;§15.4=T10 ✓;§16 中 manager/标定/模板相关验收项均有归属 ✓。web UI 相关(§13)明确留 Plan 6。
2. **占位符扫描**:全文无 TBD/TODO-later/"类似 Task N";Task 9 的 KRL 属"模板+人工清单"交付形态(决策 17),其中语法核对义务是交付物本身而非占位符;Task 10 核对单给出章节骨架+必填字段要求,交付判定标准明确(Stage 0~9+三演练+义务条目在文)。
3. **类型/签名一致性**:`ManagerOps`/`ManagerConfig`/`CommandResult`/`ManagerSnapshot` 在 T6 定义、T7 消费,字段逐一核对 ✓;`CalPose/CalibrationConfig/CalAction/CalStatus` T5→T6/T7 ✓;`HealthInputs/Verdict/SystemState` T4→T6 ✓;msgs 字段名 T3→T7(ManagerState 11 字段、CalibratePayload result 13 字段)✓;`clearFault/requestFaultClear` T1→T7(经 /kuka/rsi/reset_fault)✓;mock `listen_port` T2→T8 ✓。
4. **已知的两处执行时自由度**(非占位符,均给了判定标准):① T8 payload.yaml 条件装载的 roslaunch eval 回退;② manager 用例合计以落盘 TEST 计数为准:state 8 + sequencer 8 + payload_yaml 1 + runtime 12 = **29**,整仓预期 **291**;执行中增删用例须同步更新验收表。
5. **跟进对照**:plan4 11 条(1/2/4/5/6/8/9/10/11 收,3/7 排除有理由);plan3 9 条(1/2/4/5/7/9 收,3 已由 Plan 4 闭环+联调份额收,6/8 排除有理由)。
