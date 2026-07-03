# Plan 4/6: `kuka_eki_bridge` + `sri_force_torque_driver`(EKI 管理通道桥 + SRI 力传感驱动 + 各自 mock)实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标:** 按规格 `docs/superpowers/specs/2026-07-02-kuka-rsi-ros-control-design.md` §5.2、§5.6、§6.2、§6.4、§8、§15.2,交付两个非实时通信包及其 mock:`sri_force_torque_driver`(SRI 六维力/力矩传感器以太网驱动——二进制流协议解析、tare/滤波会话逻辑、断线重连、`/sri_ft/wrench_raw` WrenchStamped 发布 + `/sri_ft/status` 诊断 + `/sri_ft/zero`、`/sri_ft/set_filter` 服务、可脚本化传感器 mock)与 `kuka_eki_bridge`(EthernetKRL 6.1 管理通道——XML 帧编解码、TCP 流切分、Seq/Ack 请求响应会话、`/kuka/eki/*` 全套服务、`/kuka/eki/state` + `/kuka/diagnostics` 话题、KRC 模拟 mock),以及 `soft_robot_msgs` 的配套 msg/srv 增量。全部核心逻辑离线 gtest 覆盖,mock 闭环仅走 127.0.0.1 真 socket。

**架构:** 延续 Plan 1/2/3 的"纯逻辑类 + 薄 ROS 壳"分层。协议编解码(`SriFrameAssembler`、`eki_frame` codec、`EkiStreamSplitter`)与会话语义(`SriStreamSession`、`EkiSessionCore`)均为不含 `ros/ros.h` 的纯逻辑类,字节进、结构体出,gtest 直接喂手工推导的字节序列;传输层为 poll() 有界等待的 `TcpClientTransport`(沿用 Plan 2 `UdpTransport` 的范式,UDP 实现本身无法复用,见待确认 8);线程化 runtime(`SriDriverRuntime`、`EkiBridgeRuntime`)组合传输+会话并对 mock 闭环测试;ROS 节点壳只做参数装载、回调→publisher 搬运与服务转发,冒烟手动。**两个节点均为非实时节点**(RSI 实时环在 Plan 2/3 已闭合):wrench 的 RT 消费方是 Plan 3 控制器且其自备 RealtimeBuffer,故本计划内部线程交接一律用普通 mutex,不引入 realtime_tools(Global Constraints 有专项说明)。

**Plan series:** ① core algorithm library(已完成)→ ② `kuka_rsi_hw_interface` + RSI mock + msgs(已完成)→ ③ `soft_robot_controllers`(已完成,已合入 main)→ ④ `kuka_eki_bridge` + `sri_force_torque_driver` + mocks(本计划)→ ⑤ manager + calibration + bringup + KUKA 模板 → ⑥ web interface。

## 范围

**包清单(2 个新 catkin 包 + 1 个存量包增量):**

| 包 | 内容 |
|---|---|
| `sri_force_torque_driver` | `SriFrameAssembler`(M8128 型二进制流增量解析,校验/坏帧/重同步)、`TcpClientTransport`、`SriStreamSession`(超时/tare/偏置限幅/一阶低通/包号跳变统计)、`SriMockServer` + 独立可执行 `sri_mock_server`(波形/故障注入)、`SriDriverRuntime`(重连循环)+ `sri_driver_node`、`config/sri_ft.yaml`、全套离线 gtest |
| `kuka_eki_bridge` | `eki_frame` codec(RobotCommand/RobotState XML 双向编解码,tinyxml2)、`EkiStreamSplitter`(TCP 流→完整 XML 文档)、`TcpClientTransport`、`EkiSessionCore`(Seq/Ack 关联、响应超时、心跳新鲜度)、`EkiMockServer` + 独立可执行 `eki_mock_server`(KRC 模拟:ready/start/stop/fault/reset/tool)、`EkiBridgeRuntime` + `eki_bridge_node`(7 个服务 + 2 个话题)、`config/kuka_eki.yaml`、全套离线 gtest |
| `soft_robot_msgs`(增量) | 新增 `SriStatus.msg`、`EkiState.msg`、`SetFilter.srv`、`SetEkiMode.srv`、`SetToolBase.srv`;`test_msgs.cpp` 扩展 2 用例 |

**非目标(本计划不做):**

- KRL 程序、`EkiConfig.xml`、RSI 上下文等 KUKA 侧模板(→ Plan 5)。本计划的 XML schema(Task 7)即 Plan 5 模板的权威依据,勿在 Plan 5 中另行发明。
- launch/bringup 编排、`/kuka/eki/get_tool` 在 `CONNECTED -> READY` 转移中的自动调用(规格 §6.4 的编排属 manager → Plan 5;本计划只提供服务本体)。
- manager 状态机与系统级 `system_state`(→ Plan 5)。`/kuka/eki/state`、`/sri_ft/status` 是链路级状态,不是系统状态。
- SRI 传感器的 AT 配置命令集(采样率/量程/滤波档设置):首版假定盒端既有配置,驱动只发起/停止数据流(见待确认 3)。
- 传感器坐标系到工具坐标系的姿态变换:`/sri_ft/wrench_raw` 按传感器坐标系原样发布(`frame_id` 标注),重力补偿/坐标处理在 Plan 1 算法与 Plan 3 控制器内。
- rostest/roslaunch 级测试:两个节点壳的 roscore 冒烟为手动步骤(Task 6/10 各一节),不进验收门槛。
- RSI 侧故障复位与 `RsiSessionMonitor::reset()` 计数器语义的裁决(Plan 2 跟进 5 的后半):见待确认 10,遗留给 Plan 5。

**与其他 Plan 的接口关系:**

- 消费 Plan 1:`sfc::Wrench`(N/Nm 单位约定)、`sfc::ForceTorqueFilter(double cutoff_hz)` / `filter(w, dt)` / `reset()`(SRI 会话的可选驱动侧低通)。
- 消费 Plan 2:`soft_robot_msgs/GetTool.srv`(已交付,本计划接线其 server 端);`UdpTransport`/`RsiMockServer` 的传输与 mock 线程**范式**(代码不复用,理由见待确认 8);`kuka_rsi_hw_interface` 的 CMake/包结构惯例。
- 供给 Plan 3:`/sri_ft/wrench_raw`(`geometry_msgs/WrenchStamped`)——Plan 3 `wrench_topic` 默认值的真实上游;`header.stamp` 语义按 Plan 3 跟进 3 落实(本计划硬性验收项)。
- 供给 Plan 5:`/kuka/eki/{connect,start_rsi_program,stop_rsi_program,set_mode,reset_fault,set_tool_base,get_tool}` 服务全集、`/kuka/eki/state`(EkiState)、`/kuka/diagnostics`(DiagnosticArray)、`/sri_ft/{status,zero,set_filter}`;两个 mock 可执行文件(`sri_mock_server`、`eki_mock_server`)作为 Plan 5 集成/标定测试与冒烟的基座;`eki_frame.h` 的 XML schema 与动作码表(KRL/EkiConfig.xml 模板照此生成)。
- git 基线:Plan 1~3 均已合入 main;从 `main` 新建 `feature/eki-sri-drivers`。

## 待确认(规格未定项,本计划采用的默认决策)

1. **SRI 线协议**:规格 §5.6/§8 只定话题与职责,未给线协议。采用 SRI M8128 采集盒以太网协议族默认形态:传感器盒为 **TCP server**(默认端口 4008,`sri_ft.yaml: sensor_port`);ASCII 命令 `AT+GSD\r\n`(启动连续上传)/ `AT+GSD=STOP\r\n`(停止);数据帧 = `0xAA 0x55` 同步头 + 2 字节大端包长(=27:包号 2 + 数据 24 + 校验 1)+ 2 字节大端包号 + 6×float32 小端(Fx Fy Fz Mx My Mz,N/Nm)+ 1 字节校验(24 个数据字节之和 & 0xFF)。全部常量集中在 `sri_frame.h`,真机联调若与实际盒端配置有出入只改一处;帧长不符按坏帧计数并重同步(兼容盒端通道配置变化的显式失败)。宿主按小端解码 float(x86-64/ARM64 成立,头文件注明假定)。
2. **驱动发布策略**:每解析出一帧即发布一条 `WrenchStamped`(发布率=传感器流率,盒端默认按 250 Hz 配置,满足 RSI 4 ms 周期的"每周期有新样本");`frame_id` 参数默认 `sri_ft_link`;**`header.stamp` = 接收时刻**(回调在 rx 线程 recv 后同步执行,`ros::Time::now()` 即接收瞬间;SRI 帧内无时基,接收时刻是采样时刻的最佳可得近似),绝不发零 stamp——Plan 3 跟进 3 的落实,进验收清单。
3. **`/sri_ft/zero` 与 `/sri_ft/set_filter` 为驱动侧软件实现**:zero = 对接下来 `zero_sample_count`(默认 100)个原始样本取均值作偏置,此后发布前扣除;set_filter = 驱动侧一阶低通(复用 `sfc::ForceTorqueFilter`,重建+reset 语义同 Plan 1 跟进 3),默认 cutoff 0 = 原样直发,保持 `wrench_raw` 的 "raw" 语义(控制器仍拥有自己的滤波,规格 §8 数据通路不变)。不向传感器发校零 AT 命令(命令集非公开可验,软件 tare 行为可测且与法兰工装无关)。
4. **`FTBia` → `bias_limit_n`**(规格 §14 "bias threshold (legacy FTBia)"):语义定为 **tare 合法性上限**——捕获均值的力范数超过阈值(默认 120 N,承 `Parameter.xml`)则判定"负载下校零",拒绝该次 tare(保留旧偏置,服务返回 `success=false`),不做运行时钳制。
5. **EKI 连接方向**:KRC 为 **TCP server**、ROS 为 client,对应 EthernetKRL 6.1 配置 `<CONFIGURATION><EXTERNAL><TYPE>Client</TYPE></EXTERNAL>`(EXTERNAL=外部系统=ROS 侧为 client)+ `<INTERNAL><PORT>54600</PORT><PROTOCOL>TCP</PROTOCOL>`。Plan 5 的 `EkiConfig.xml` 模板必须与此一致;`EkiMockServer` 相应扮演 TCP server。桥支持自动重连 + `/kuka/eki/connect` 手动触发。
6. **EKI 应用层 XML schema 自定义**(规格 §5.2/§6.2 只给服务与职责清单):ROS→KRC `<RobotCommand><Cmd Seq Action Value/><Tool .../><Base .../></RobotCommand>`,KRC→ROS `<RobotState><Ack Seq Ok Code/><Prog Ready RsiActive Fault Mode/><Tool .../></RobotState>`;动作码 0=QUERY_STATE、1=START_RSI、2=STOP_RSI、3=SET_MODE、4=RESET_FAULT、5=GET_TOOL、6=SET_TOOL_BASE;每条命令固定携带全部元素(未用置零),匹配 EthernetKRL 固定结构解析习惯;`Ack.Seq=0` 表示无请求的心跳推送。错误码:0=OK、1=NOT_READY、2=FAULTED。schema 常量全部集中 `eki_frame.h`。
7. **单飞行中命令 + 心跳**:桥同一时刻只允许一条命令在途(服务层经 `execute()` 串行化,响应超时默认 2 s);KRL 侧周期推送 `RobotState` 心跳(建议 100 ms 档,mock 可配),桥以 `state_timeout_s`(默认 1 s)判定状态新鲜度——tool 查询结果、fault 等取"最近一帧状态"。
8. **`TcpClientTransport` 两包各自实现**(namespace `sri` / `kuka_eki`,代码同构):Plan 2 `UdpTransport` 是 UDP 专用(sendto/recvfrom + last-sender 语义)无法复用;做共享包会制造"传感器驱动 ↔ KUKA 桥"之间的伪依赖(规格 §5.6:driver 不得含 KUKA 特定行为),~120 行的受控重复优于错误的依赖方向。Task 8 的副本与 Task 3 逐字同构(仅命名空间/头文件保护不同),测试取子集。
9. **服务类型映射**:`/kuka/eki/{connect,start_rsi_program,stop_rsi_program,reset_fault}` 与 `/sri_ft/zero` 用 `std_srvs/Trigger`;`/kuka/eki/set_mode` = 新增 `SetEkiMode.srv`、`/kuka/eki/set_tool_base` = 新增 `SetToolBase.srv`、`/sri_ft/set_filter` = 新增 `SetFilter.srv`;`/kuka/eki/get_tool` 用已交付的 `GetTool.srv`;`/kuka/diagnostics` 用 `diagnostic_msgs/DiagnosticArray`。新定义全部落 `soft_robot_msgs`(Task 1 给全文)。
10. **`reset_fault` 只复位 KRC 侧**(Plan 2 跟进 5 的本计划份额):`/kuka/eki/reset_fault` 发 RESET_FAULT 动作,由 KRL 清 KRC 侧非实时故障;RSI 链路的 latched fault 属 `kuka_rsi_hw_interface` 节点,其复位服务与 `RsiSessionMonitor::reset()` 清计数器的矛盾("cumulative since node start" 注释)遗留 Plan 5 裁决(建议:HW 层加 `clearFault()` 保留计数,勿用 `reset()`)。写入遗留风险。
11. **mock 波形脚本化分层**:库类 `SriMockServer` 提供确定性脚本原语(`setWrench`/`sendFrames(count)`/坏帧/断链/垃圾字节)供 gtest 闭环;正弦等连续波形只在独立可执行 `sri_mock_server` 里实现(`--sine-amp/--sine-hz`),避免测试依赖时间敏感波形。EKI 同理:`EkiMockServer` 库类给确定性行为表,可执行 `eki_mock_server` 供手动冒烟/Plan 5。
12. **runtime 阻塞语义**:`SriDriverRuntime::requestZero(timeout_ms)` 有界等待捕获完成(未在流会话中则超时返回 false 并取消捕获);`EkiBridgeRuntime::execute()` 的等待上界 = `response_timeout_s + 1 s`,所有终态由 io 线程裁决(REJECTED/ACCEPTED/TIMEOUT,绝不返回 PENDING);未连接时立即 REJECTED(错误码 -1),命令在途时并发 execute 排队于 `command_mutex_`。

## Global Constraints

- ROS1 Noetic,catkin 工作区 `/home/ljj/kuka_iiqka_ros/ros_ws`;新包 `ros_ws/src/sri_force_torque_driver/`、`ros_ws/src/kuka_eki_bridge/`;存量包 `ros_ws/src/soft_robot_msgs/` 仅做声明式增量。
- C++14,`-Wall -Wextra`,零警告;所有代码与注释英文。TDD:先失败测试(构建失败即失败态),再最小实现,再通过,再提交。
- **Noetic 的 `catkin_add_gtest` 不自动链接 gtest_main:所有测试链接行必须包含 `${GTEST_MAIN_LIBRARIES}`。**
- **Noetic 勘误照录**(Plan 2 已裁决,本计划不直接触及这两处 API,照录防复发):① `hardware_interface::HardwareInterface::claim()` 基类无条件记录资源,只读接口需 no-op 重写;② `JointStateHandle::getPosition()` 按值返回 `double`,不要写 `*handle.getPosition()`。
- 单位:笛卡尔 mm/deg(KUKA A/B/C = Z-Y-X),wrench N/Nm(`sfc::Wrench` 与 `geometry_msgs/WrenchStamped` 一致);SRI 帧内 6 通道 = Fx Fy Fz Mx My Mz。
- **跟进事项落实(本计划的硬性验收项)**:
  1. **wrench `header.stamp` = 接收时刻且非零**(Plan 3 跟进 3):runtime 样本带 `stamp_s`(rx 线程 recv 后即刻取钟),节点回调在 rx 线程同步执行、`ros::Time::now()` 打戳;Task 6 有 stamp 非零且单调的固化测试;真机联调用 `rostopic echo --offset` 抽查 stamp 与到达时刻差(写入验收清单)。
  2. **`reset_fault` 接线,KRC 侧份额**(Plan 2 跟进 5):Task 9/10 覆盖 fault→reset→start 工作流;RSI 侧计数器矛盾裁决遗留 Plan 5(待确认 10)。
  3. **期望值不得漏算中间环节**(Plan 3 跟进 1 教训):本计划所有含滤波/tare/校验和的测试期望值在计划内给出完整手工推导(校验和逐字节求和、低通 alpha 取 0.5 的构造、tare 均值算式),执行者复核后才可写测试。
- **实时性定位**:本计划两个节点均为非 RT 节点,不使用 `realtime_tools`(RealtimeBuffer 仅在有 RT 消费方的进程内需要——wrench 的 RT 消费方是 Plan 3 控制器进程,其订阅侧已自备 RealtimeBuffer)。驱动/桥内部 rx/io 线程与服务线程交接用 `std::mutex`;纯逻辑类本身仍保持零分配(`SriFrameAssembler`/`SriStreamSession`/`eki_frame` codec 固定缓冲;`EkiStreamSplitter` 用 `std::function` sink,管理通道非 RT,允许)。
- 纯逻辑类(`sri_frame`、`sri_stream_session`、`eki_frame`、`eki_stream_splitter`、`eki_session_core`)不包含 `ros/ros.h`;传输/mock/runtime 允许 POSIX socket 与 `std::thread`(同 Plan 2 mock 先例),节点壳才允许 roscpp。
- 全部 gtest 离线可跑:无 roscore、无真机;网络测试仅 127.0.0.1 真 socket,端口一律 kernel 自选(bind 0)防冲突;一切等待有界——单次窗口 ≤ 0.5 s,重连类用 0.05~0.1 s backoff、整测试有界 ≤ 2 s;禁止无界 `while`。
- 构建/运行命令沿用前三计划(仓库根 `/home/ljj/kuka_iiqka_ros`):

```bash
cd ros_ws && catkin_make tests                            # build all test binaries
./devel/lib/sri_force_torque_driver/<test_binary>         # run one gtest binary
./devel/lib/kuka_eki_bridge/<test_binary>
```

**`sri_driver_node` 数据通路(规格 §5.6/§8 的本计划落地形态,跟进 1 已内嵌):**

```text
rx thread (SriDriverRuntime::run):
  not connected -> TcpClientTransport.connect (bounded) -> session.reset()
                   -> send "AT+GSD\r\n" -> (reconnects++ after first)
  receive(<=50 ms) -> -1: mark disconnected, backoff loop
                   ->  0: continue (timeout tick only)
                   -> >0: session.feed(bytes, now)
                          assembler: sync AA 55 -> len -> payload -> checksum
                            bad checksum/len -> count + resync, no sample
                          per sample: zero-capture accumulate (raw)
                                      -> bias subtract -> optional LPF -> out
       for each sample: callback(sample)      # runs IN the rx thread
         node: WrenchStamped{stamp=now(), frame_id} -> /sri_ft/wrench_raw
service threads: /sri_ft/zero -> requestZero (bounded wait)
                 /sri_ft/set_filter -> setFilterCutoff (rebuild + reset)
10 Hz timer: status() -> /sri_ft/status
```

**`eki_bridge_node` 数据通路(规格 §5.2/§6.2 的本计划落地形态):**

```text
io thread (EkiBridgeRuntime::run):
  not connected -> resolve pending request REJECTED(kErrNotConnected)
                -> if auto_reconnect or connectNow requested: bounded connect
                   -> session.reset(), splitter drained
  pending request + no in-flight command -> session.beginCommand(seq++)
                -> serializeCommand -> transport.send (io thread owns socket)
  receive(<=50 ms) -> splitter -> per doc: parseState
                -> ok:  session.onState (ack resolve / heartbeat freshness)
                -> bad: session.onBadFrame
  session.tick(now) -> pending command past response_timeout -> TIMEOUT
  terminal result -> notify execute() waiter (cv)
service threads: execute(action, ...) -> bounded wait, never PENDING
10 Hz timer: /kuka/eki/state; 1 Hz: /kuka/diagnostics
```

---

## File Structure

```text
ros_ws/src/soft_robot_msgs/
  msg/SriStatus.msg                       # NEW: /sri_ft/status payload
  msg/EkiState.msg                        # NEW: /kuka/eki/state payload
  srv/SetFilter.srv                       # NEW: /sri_ft/set_filter
  srv/SetEkiMode.srv                      # NEW: /kuka/eki/set_mode
  srv/SetToolBase.srv                     # NEW: /kuka/eki/set_tool_base
  CMakeLists.txt                          # add new files
  test/test_msgs.cpp                      # +2 tests

ros_ws/src/sri_force_torque_driver/
  package.xml
  CMakeLists.txt
  include/sri_force_torque_driver/
    sri_frame.h                           # binary stream assembler + commands (pure)
    tcp_client_transport.h                # bounded-wait TCP client (POSIX, no ROS)
    sri_stream_session.h                  # timeout/tare/filter/gap session (pure)
    sri_mock_server.h                     # scriptable sensor mock (TCP server, test code)
    sri_driver_runtime.h                  # rx thread: transport+session -> callbacks
  src/
    sri_frame.cpp
    tcp_client_transport.cpp
    sri_stream_session.cpp
    sri_mock_server.cpp
    sri_driver_runtime.cpp
    sri_mock_server_main.cpp              # standalone mock exe (waveform/fault opts)
    sri_driver_node.cpp                   # ROS shell
  config/
    sri_ft.yaml
  test/
    test_sri_frame.cpp
    test_tcp_client_transport.cpp
    test_sri_stream_session.cpp
    test_sri_mock_server.cpp
    test_sri_driver_runtime.cpp

ros_ws/src/kuka_eki_bridge/
  package.xml
  CMakeLists.txt
  include/kuka_eki_bridge/
    eki_frame.h                           # RobotCommand/RobotState codec (pure, tinyxml2)
    eki_stream_splitter.h                 # TCP byte stream -> complete XML docs (pure)
    tcp_client_transport.h                # structural twin of the sri one (decision 8)
    eki_session_core.h                    # seq/ack correlation + freshness (pure)
    eki_mock_server.h                     # KRC-side mock (TCP server, test code)
    eki_bridge_runtime.h                  # io thread + blocking execute()
  src/
    eki_frame.cpp
    eki_stream_splitter.cpp
    tcp_client_transport.cpp
    eki_session_core.cpp
    eki_mock_server.cpp
    eki_bridge_runtime.cpp
    eki_mock_server_main.cpp              # standalone KRC mock exe
    eki_bridge_node.cpp                   # ROS shell
  config/
    kuka_eki.yaml
  test/
    test_eki_frame.cpp                    # codec + splitter cases, one binary
    test_tcp_client_transport.cpp
    test_eki_session_core.cpp
    test_eki_mock_server.cpp
    test_eki_bridge_runtime.cpp
```

---

### Task 0: 建立分支

- [ ] **Step 1: 从 main 创建工作分支**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git checkout main && \
git checkout -b feature/eki-sri-drivers
```

预期:`Switched to a new branch 'feature/eki-sri-drivers'`。

---

### Task 1: `soft_robot_msgs` 增量(SriStatus / EkiState / 三个 srv)

**Files:**
- Create: `ros_ws/src/soft_robot_msgs/msg/SriStatus.msg`
- Create: `ros_ws/src/soft_robot_msgs/msg/EkiState.msg`
- Create: `ros_ws/src/soft_robot_msgs/srv/SetFilter.srv`
- Create: `ros_ws/src/soft_robot_msgs/srv/SetEkiMode.srv`
- Create: `ros_ws/src/soft_robot_msgs/srv/SetToolBase.srv`
- Modify: `ros_ws/src/soft_robot_msgs/CMakeLists.txt`
- Test: `ros_ws/src/soft_robot_msgs/test/test_msgs.cpp`(扩展,+2 用例)

**Interfaces:**
- Consumes: `std_msgs/Header`(仅 builtin,`package.xml` 无需改动)。
- Produces(Task 6/10、Plan 5/6 消费):5 个新定义,全文见 Step 2;单位/注释风格沿用存量 msg。

- [ ] **Step 1: 写失败测试(扩展 test_msgs.cpp)**

在 `ros_ws/src/soft_robot_msgs/test/test_msgs.cpp` 的 include 区追加:

```cpp
#include <soft_robot_msgs/EkiState.h>
#include <soft_robot_msgs/SetEkiMode.h>
#include <soft_robot_msgs/SetFilter.h>
#include <soft_robot_msgs/SetToolBase.h>
#include <soft_robot_msgs/SriStatus.h>
```

文件末尾追加两个用例:

```cpp
TEST(Msgs, DriverStatusMsgsDefaultToDisconnected) {
  soft_robot_msgs::SriStatus s;
  EXPECT_FALSE(s.connected);
  EXPECT_FALSE(s.streaming);
  EXPECT_EQ(s.samples, 0u);
  EXPECT_EQ(s.bad_frames, 0u);
  EXPECT_EQ(s.package_gaps, 0u);
  EXPECT_FALSE(s.zero_active);
  EXPECT_EQ(s.filter_cutoff_hz, 0.0);
  soft_robot_msgs::EkiState e;
  EXPECT_FALSE(e.connected);
  EXPECT_FALSE(e.state_fresh);
  EXPECT_FALSE(e.program_ready);
  EXPECT_FALSE(e.rsi_active);
  EXPECT_FALSE(e.fault);
  EXPECT_EQ(e.mode, 0u);
  EXPECT_EQ(e.tool_x, 0.0);
  EXPECT_EQ(e.tool_c, 0.0);
}

TEST(Msgs, ManagementSrvResponsesDefaultToFailure) {
  soft_robot_msgs::SetFilter::Response f;
  EXPECT_FALSE(f.success);
  soft_robot_msgs::SetEkiMode::Response m;
  EXPECT_FALSE(m.success);
  soft_robot_msgs::SetToolBase::Response t;
  EXPECT_FALSE(t.success);
  soft_robot_msgs::SetToolBase::Request req;
  EXPECT_EQ(req.tool_x, 0.0);
  EXPECT_EQ(req.base_c, 0.0);
}
```

- [ ] **Step 2: 构建确认失败,然后写消息定义**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `soft_robot_msgs/SriStatus.h` 等头文件。

`ros_ws/src/soft_robot_msgs/msg/SriStatus.msg`:

```text
# SRI force/torque sensor link diagnostics published by
# sri_force_torque_driver on /sri_ft/status (spec section 5.6).
# Counters are cumulative since node start.
Header header
bool connected              # TCP link to the sensor box is up
bool streaming              # a valid frame arrived within sample_timeout_s
uint32 reconnects           # completed reconnect cycles after the first connect
uint64 samples              # decoded valid frames
uint64 bad_frames           # checksum/length failures
uint64 package_gaps         # non-consecutive package-number events
uint32 zero_rejects         # tare captures rejected by bias_limit_n
float64 last_sample_age     # s since the last valid sample; -1 before the first
bool zero_active            # a tare capture is currently averaging samples
float64 filter_cutoff_hz    # driver-side low-pass; 0 = raw passthrough
```

`ros_ws/src/soft_robot_msgs/msg/EkiState.msg`:

```text
# EKI management-channel state published by kuka_eki_bridge on
# /kuka/eki/state (spec sections 5.2, 6.2). Tool frame in mm / deg
# (KUKA A/B/C = Z-Y-X Euler); tool fields are meaningful once
# state_fresh is true (they mirror the last RobotState heartbeat).
Header header
bool connected              # TCP link to the KRC EKI server is up
bool state_fresh            # a RobotState frame arrived within state_timeout_s
bool program_ready          # KRL program online and ready (spec 6.2)
bool rsi_active             # RSI context active on the KRC
bool fault                  # KRC-side non-realtime fault latched
uint8 mode                  # operating mode reported by the KRL program
uint32 reconnects           # completed reconnect cycles after the first connect
float64 state_age           # s since the last RobotState frame; -1 before first
float64 tool_x
float64 tool_y
float64 tool_z
float64 tool_a
float64 tool_b
float64 tool_c
```

`ros_ws/src/soft_robot_msgs/srv/SetFilter.srv`:

```text
# Configure the driver-side low-pass filter (/sri_ft/set_filter, spec
# section 5.6). cutoff_hz = 0 disables filtering (raw passthrough).
# Rebuilds and resets the filter state (Plan 1 follow-up 3 semantics).
float64 cutoff_hz
---
bool success
string message
```

`ros_ws/src/soft_robot_msgs/srv/SetEkiMode.srv`:

```text
# Select the KRL operating mode over EKI (/kuka/eki/set_mode, spec 5.2).
# The value is forwarded verbatim in the SET_MODE command; the KRL
# program owns its interpretation.
uint8 mode
---
bool success
string message
```

`ros_ws/src/soft_robot_msgs/srv/SetToolBase.srv`:

```text
# Push tool/base frames to the KRC over EKI (/kuka/eki/set_tool_base,
# spec sections 5.2, 6.4). Units: mm / deg (KUKA A/B/C = Z-Y-X Euler).
float64 tool_x
float64 tool_y
float64 tool_z
float64 tool_a
float64 tool_b
float64 tool_c
float64 base_x
float64 base_y
float64 base_z
float64 base_a
float64 base_b
float64 base_c
---
bool success
string message
```

- [ ] **Step 3: CMake 增量**

`ros_ws/src/soft_robot_msgs/CMakeLists.txt` 的 `add_message_files` 块追加两行、`add_service_files` 块追加三行:

```cmake
add_message_files(
  FILES
  CartesianState.msg
  CartesianCorrection.msg
  CartesianCorrectionStamped.msg
  RsiState.msg
  ModeCommand.msg
  ModeState.msg
  SriStatus.msg
  EkiState.msg
)

add_service_files(
  FILES
  GetTool.srv
  SetFilter.srv
  SetEkiMode.srv
  SetToolBase.srv
)
```

其余段落(`generate_messages`、`catkin_package`、测试段)不变——新定义只依赖 `std_msgs`。

- [ ] **Step 4: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_robot_msgs/test_msgs
```

预期:`[  PASSED  ] 6 tests.`(存量 4 + 新增 2)。

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_robot_msgs && \
git commit -m "feat(msgs): add SriStatus/EkiState msgs and SetFilter/SetEkiMode/SetToolBase srvs (Plan 4 Task 1)"
```

---

### Task 2: `sri_force_torque_driver` 包骨架 + `sri_frame`(M8128 型二进制流解析)

**Files:**
- Create: `ros_ws/src/sri_force_torque_driver/package.xml`
- Create: `ros_ws/src/sri_force_torque_driver/CMakeLists.txt`
- Create: `ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_frame.h`
- Create: `ros_ws/src/sri_force_torque_driver/src/sri_frame.cpp`
- Test: `ros_ws/src/sri_force_torque_driver/test/test_sri_frame.cpp`

**Interfaces:**
- Consumes: 无(纯字节协议层,零依赖)。
- Produces(Task 4/5/6 消费):
  - `sri::startStreamCommand()` / `sri::stopStreamCommand()`(ASCII 命令,CRLF 结尾)。
  - `struct FtSample { float ch[6]; std::uint16_t package_number; }`(通道序 Fx Fy Fz Mx My Mz)。
  - `struct AssemblerStats { frames, bad_checksum, bad_length, dropped, skipped_bytes }`。
  - `class SriFrameAssembler`:`feed(data, len, out, max_out)` 增量解析 + `stats()` + `reset()`;字节状态机,固定存储,零分配。
- 帧布局(待确认 1 的权威定义,常量集中于头文件):`AA 55 | LEN_H LEN_L (=0x001B) | PN_H PN_L | 24 data bytes (6 x float32 LE) | SUM(data)&0xFF`。

**测试期望值手工推导(校验和,执行者须复核)**:canonical 帧取 Fx=1.0、Fy=0、Fz=2.0、Mx=0、My=-1.0、Mz=0、PN=1。IEEE754 小端字节:`1.0f = 00 00 80 3F`、`2.0f = 00 00 00 40`、`-1.0f = 00 00 80 BF`、`0.0f = 00 00 00 00`。数据字节和 = (0x80+0x3F) + 0x40 + (0x80+0xBF) = 0xBF + 0x40 + 0x13F = 0x23E,& 0xFF = **0x3E**。第二帧取 Fx=0.5(`00 00 00 3F`)、其余 0、PN=2,数据和 = **0x3F**。包长恒 0x001B = 27 = 2(PN) + 24(data) + 1(sum)。

- [ ] **Step 1: 写包清单与构建文件**

`ros_ws/src/sri_force_torque_driver/package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>sri_force_torque_driver</name>
  <version>0.1.0</version>
  <description>
    Ethernet driver for the SRI six-axis force/torque sensor (spec
    sections 5.6, 8): M8128-style binary stream parsing, tare and
    optional low-pass session logic, bounded reconnect, WrenchStamped
    publishing, and a scriptable sensor mock for offline closed-loop
    tests. Contains no KUKA-specific control behavior.
  </description>
  <maintainer email="dev@example.com">ljj</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>geometry_msgs</depend>
  <depend>std_srvs</depend>
  <depend>soft_force_control_core</depend>
  <depend>soft_robot_msgs</depend>
</package>
```

`ros_ws/src/sri_force_torque_driver/CMakeLists.txt`(初版,后续 Task 增量扩展):

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(sri_force_torque_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_srvs
  soft_force_control_core
  soft_robot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sri_ft_protocol
  CATKIN_DEPENDS roscpp geometry_msgs std_srvs soft_force_control_core
                 soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS})

# Protocol / session library: no roscpp runtime dependency.
# (The sri_ft_mock library target and its catkin_package export are added
# in Task 5 together with the mock sources.)
add_library(sri_ft_protocol
  src/sri_frame.cpp
)
target_link_libraries(sri_ft_protocol ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_sri_frame test/test_sri_frame.cpp)
  target_link_libraries(test_sri_frame sri_ft_protocol ${GTEST_MAIN_LIBRARIES})
endif()
```

- [ ] **Step 2: 写失败测试**

`ros_ws/src/sri_force_torque_driver/test/test_sri_frame.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "sri_force_torque_driver/sri_frame.h"

using sri::AssemblerStats;
using sri::FtSample;
using sri::SriFrameAssembler;

namespace {

// Canonical frame, checksum hand-derived in the plan (Task 2 header):
// Fx=1.0 Fy=0 Fz=2.0 Mx=0 My=-1.0 Mz=0, PN=1.
// sum(data) = (0x80+0x3F) + 0x40 + (0x80+0xBF) = 0x23E -> 0x3E.
const std::uint8_t kFrameA[31] = {
    0xAA, 0x55, 0x00, 0x1B, 0x00, 0x01,
    0x00, 0x00, 0x80, 0x3F,   // Fx = 1.0f (LE)
    0x00, 0x00, 0x00, 0x00,   // Fy = 0.0f
    0x00, 0x00, 0x00, 0x40,   // Fz = 2.0f
    0x00, 0x00, 0x00, 0x00,   // Mx = 0.0f
    0x00, 0x00, 0x80, 0xBF,   // My = -1.0f
    0x00, 0x00, 0x00, 0x00,   // Mz = 0.0f
    0x3E};

// Second frame: Fx=0.5 (00 00 00 3F), rest 0, PN=2 -> checksum 0x3F.
const std::uint8_t kFrameB[31] = {
    0xAA, 0x55, 0x00, 0x1B, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x3F,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0x3F};

int feedAll(SriFrameAssembler& a, const std::uint8_t* data, std::size_t len,
            FtSample* out, int max_out) {
  return a.feed(data, len, out, max_out);
}

}  // namespace

TEST(SriFrame, SingleFrameDecodes) {
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(feedAll(a, kFrameA, sizeof(kFrameA), out, 4), 1);
  EXPECT_FLOAT_EQ(out[0].ch[0], 1.0f);
  EXPECT_FLOAT_EQ(out[0].ch[1], 0.0f);
  EXPECT_FLOAT_EQ(out[0].ch[2], 2.0f);
  EXPECT_FLOAT_EQ(out[0].ch[4], -1.0f);
  EXPECT_EQ(out[0].package_number, 1u);
  EXPECT_EQ(a.stats().frames, 1u);
}

TEST(SriFrame, PackageNumberIsBigEndian) {
  std::uint8_t f[31];
  std::memcpy(f, kFrameA, sizeof(f));
  f[4] = 0x01;  // PN = 0x0102 = 258
  f[5] = 0x02;
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(f, sizeof(f), out, 1), 1);
  EXPECT_EQ(out[0].package_number, 258u);
}

TEST(SriFrame, BadChecksumCountedNoSample) {
  std::uint8_t f[31];
  std::memcpy(f, kFrameA, sizeof(f));
  f[30] = 0x00;  // corrupt checksum (correct value is 0x3E)
  SriFrameAssembler a;
  FtSample out[1];
  EXPECT_EQ(a.feed(f, sizeof(f), out, 1), 0);
  EXPECT_EQ(a.stats().bad_checksum, 1u);
  EXPECT_EQ(a.stats().frames, 0u);
}

TEST(SriFrame, RecoversAfterBadFrame) {
  std::uint8_t bad[31];
  std::memcpy(bad, kFrameA, sizeof(bad));
  bad[30] = 0x00;
  SriFrameAssembler a;
  FtSample out[2];
  EXPECT_EQ(a.feed(bad, sizeof(bad), out, 2), 0);
  EXPECT_EQ(a.feed(kFrameB, sizeof(kFrameB), out, 2), 1);
  EXPECT_FLOAT_EQ(out[0].ch[0], 0.5f);
  EXPECT_EQ(out[0].package_number, 2u);
}

TEST(SriFrame, SplitAcrossFeedsByteByByte) {
  SriFrameAssembler a;
  FtSample out[1];
  int total = 0;
  for (std::size_t i = 0; i < sizeof(kFrameA); ++i)
    total += a.feed(&kFrameA[i], 1, out, 1);
  EXPECT_EQ(total, 1);
  EXPECT_FLOAT_EQ(out[0].ch[2], 2.0f);
}

TEST(SriFrame, MultipleFramesInOneFeed) {
  std::vector<std::uint8_t> buf(kFrameA, kFrameA + sizeof(kFrameA));
  buf.insert(buf.end(), kFrameB, kFrameB + sizeof(kFrameB));
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 4), 2);
  EXPECT_EQ(out[0].package_number, 1u);
  EXPECT_EQ(out[1].package_number, 2u);
}

TEST(SriFrame, AsciiAckBeforeSyncIsSkipped) {
  const char* ack = "ACK+GSD=OK\r\n";
  std::vector<std::uint8_t> buf(ack, ack + std::strlen(ack));
  buf.insert(buf.end(), kFrameA, kFrameA + sizeof(kFrameA));
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 1), 1);
  EXPECT_EQ(a.stats().skipped_bytes, std::strlen(ack));
}

TEST(SriFrame, WrongDeclaredLengthCountedAsBadLength) {
  std::uint8_t f[31];
  std::memcpy(f, kFrameA, sizeof(f));
  f[3] = 0x10;  // declared 0x0010 != 27
  SriFrameAssembler a;
  FtSample out[1];
  EXPECT_EQ(a.feed(f, sizeof(f), out, 1), 0);
  EXPECT_EQ(a.stats().bad_length, 1u);
}

TEST(SriFrame, AaNotFollowedBy55Resyncs) {
  std::vector<std::uint8_t> buf = {0xAA, 0x41};
  buf.insert(buf.end(), kFrameA, kFrameA + sizeof(kFrameA));
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 1), 1);
}

TEST(SriFrame, RepeatedAaStillSyncs) {
  // AA AA 55 ...: the second AA must be treated as a candidate sync-1.
  std::vector<std::uint8_t> buf = {0xAA};
  buf.insert(buf.end(), kFrameA, kFrameA + sizeof(kFrameA));
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 1), 1);
}

TEST(SriFrame, FramesBeyondMaxOutAreDroppedAndCounted) {
  std::vector<std::uint8_t> buf(kFrameA, kFrameA + sizeof(kFrameA));
  buf.insert(buf.end(), kFrameB, kFrameB + sizeof(kFrameB));
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 1), 1);
  EXPECT_EQ(a.stats().dropped, 1u);
  EXPECT_EQ(a.stats().frames, 2u);  // both frames were valid
}

TEST(SriFrame, CommandStringsAreCrlfTerminated) {
  EXPECT_STREQ(sri::startStreamCommand(), "AT+GSD\r\n");
  EXPECT_STREQ(sri::stopStreamCommand(), "AT+GSD=STOP\r\n");
}
```

- [ ] **Step 3: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `sri_frame.h`。

- [ ] **Step 4: 写实现**

`ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_frame.h`:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>

namespace sri {

// ASCII commands of the M8128-style acquisition box (decision 1). CRLF
// terminated; the box answers with an ASCII ACK line that the assembler
// skips while hunting for the binary sync bytes.
inline const char* startStreamCommand() { return "AT+GSD\r\n"; }
inline const char* stopStreamCommand() { return "AT+GSD=STOP\r\n"; }

// One decoded data frame: 6 float32 channels Fx Fy Fz Mx My Mz (N / Nm).
struct FtSample {
  float ch[6] = {0, 0, 0, 0, 0, 0};
  std::uint16_t package_number{0};
};

struct AssemblerStats {
  std::uint64_t frames{0};         // valid frames decoded (incl. dropped)
  std::uint64_t bad_checksum{0};
  std::uint64_t bad_length{0};     // declared length != kPayloadLen
  std::uint64_t dropped{0};        // valid frames beyond the caller's max_out
  std::uint64_t skipped_bytes{0};  // non-sync bytes (ASCII ACKs, garbage)
};

// Incremental parser for the binary stream (decision 1):
//   AA 55 | LEN_H LEN_L (=27) | PN_H PN_L | 24 data bytes (6 x f32 LE) | SUM
// SUM = sum of the 24 data bytes & 0xFF. Byte-oriented FSM with fixed
// storage, allocation-free. A corrupted frame costs at most one frame:
// the FSM drops it, counts it, and rescans for the next sync header.
// Host is assumed little-endian for the float decode (x86-64/ARM64).
class SriFrameAssembler {
 public:
  // Feeds raw bytes; writes decoded samples to out (up to max_out).
  // Returns the number of samples written. Valid frames beyond max_out
  // are counted in stats().dropped so the byte stream stays consistent.
  int feed(const std::uint8_t* data, std::size_t len, FtSample* out,
           int max_out);

  const AssemblerStats& stats() const { return stats_; }
  void reset();  // resync on a new connection; keeps counters

 private:
  static constexpr std::uint8_t kSync1 = 0xAA;
  static constexpr std::uint8_t kSync2 = 0x55;
  static constexpr std::size_t kDataLen = 24;                 // 6 x float32
  static constexpr std::size_t kPayloadLen = 2 + kDataLen + 1;  // PN+data+sum

  enum class State { SYNC1, SYNC2, LEN_HIGH, LEN_LOW, PAYLOAD };

  State state_{State::SYNC1};
  std::uint16_t declared_len_{0};
  std::uint8_t payload_[kPayloadLen];
  std::size_t payload_fill_{0};
  AssemblerStats stats_;
};

}  // namespace sri
```

`ros_ws/src/sri_force_torque_driver/src/sri_frame.cpp`:

```cpp
#include "sri_force_torque_driver/sri_frame.h"

#include <cstring>

namespace sri {

void SriFrameAssembler::reset() {
  state_ = State::SYNC1;
  declared_len_ = 0;
  payload_fill_ = 0;
}

int SriFrameAssembler::feed(const std::uint8_t* data, std::size_t len,
                            FtSample* out, int max_out) {
  int written = 0;
  for (std::size_t i = 0; i < len; ++i) {
    const std::uint8_t b = data[i];
    switch (state_) {
      case State::SYNC1:
        if (b == kSync1) {
          state_ = State::SYNC2;
        } else {
          ++stats_.skipped_bytes;
        }
        break;
      case State::SYNC2:
        if (b == kSync2) {
          state_ = State::LEN_HIGH;
        } else if (b == kSync1) {
          ++stats_.skipped_bytes;  // previous AA was noise; this one may sync
        } else {
          stats_.skipped_bytes += 2;
          state_ = State::SYNC1;
        }
        break;
      case State::LEN_HIGH:
        declared_len_ = static_cast<std::uint16_t>(b) << 8;
        state_ = State::LEN_LOW;
        break;
      case State::LEN_LOW:
        declared_len_ = static_cast<std::uint16_t>(declared_len_ | b);
        if (declared_len_ != kPayloadLen) {
          ++stats_.bad_length;
          state_ = State::SYNC1;
        } else {
          payload_fill_ = 0;
          state_ = State::PAYLOAD;
        }
        break;
      case State::PAYLOAD:
        payload_[payload_fill_++] = b;
        if (payload_fill_ == kPayloadLen) {
          state_ = State::SYNC1;
          unsigned sum = 0;
          for (std::size_t k = 2; k < 2 + kDataLen; ++k) sum += payload_[k];
          if ((sum & 0xFFu) != payload_[kPayloadLen - 1]) {
            ++stats_.bad_checksum;
            break;
          }
          ++stats_.frames;
          if (written >= max_out) {
            ++stats_.dropped;
            break;
          }
          FtSample& s = out[written++];
          s.package_number = static_cast<std::uint16_t>(
              (static_cast<std::uint16_t>(payload_[0]) << 8) | payload_[1]);
          std::memcpy(s.ch, payload_ + 2, kDataLen);  // little-endian host
        }
        break;
    }
  }
  return written;
}

}  // namespace sri
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/sri_force_torque_driver/test_sri_frame
```

预期:`[  PASSED  ] 12 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/sri_force_torque_driver && \
git commit -m "feat(sri): package skeleton + M8128-style binary frame assembler (Plan 4 Task 2)"
```

---

### Task 3: `TcpClientTransport`(sri 侧,poll 有界等待)

**Files:**
- Create: `ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/tcp_client_transport.h`
- Create: `ros_ws/src/sri_force_torque_driver/src/tcp_client_transport.cpp`
- Modify: `ros_ws/src/sri_force_torque_driver/CMakeLists.txt`
- Test: `ros_ws/src/sri_force_torque_driver/test/test_tcp_client_transport.cpp`

**Interfaces:**
- Consumes: POSIX socket API(范式对齐 Plan 2 `kuka_rsi::UdpTransport`:poll 有界等待、bind 后零分配、不可拷贝)。
- Produces(Task 5/6 消费;Task 8 复制同构副本到 `kuka_eki`):
  - `class sri::TcpClientTransport`:`connect(ip, port, timeout_ms)` 非阻塞连接 + 有界等待;`connected()`;`receive(buf, n, timeout_ms)`(>0 字节数 / 0 超时 / -1 对端关闭或错误并自关);`send(data, len, timeout_ms)`(整包发送,失败自关);`close()`。
- 语义约定:任何 `-1`/`send=false` 之后 `connected()==false`,上层据此进入重连;`close()` 后可再次 `connect()`。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/sri_force_torque_driver/test/test_tcp_client_transport.cpp`:

```cpp
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <string>

#include "sri_force_torque_driver/tcp_client_transport.h"

using sri::TcpClientTransport;

namespace {

// Minimal loopback listener used to play the sensor-box side.
class TestListener {
 public:
  bool start() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) return false;
    int on = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ::inet_addr("127.0.0.1");
    addr.sin_port = 0;  // kernel-chosen port
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
      return false;
    socklen_t len = sizeof(addr);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    return ::listen(fd_, 1) == 0;
  }
  bool acceptClient(int timeout_ms) {
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) return false;
    client_ = ::accept(fd_, nullptr, nullptr);
    return client_ >= 0;
  }
  int readClient(char* buf, std::size_t n, int timeout_ms) {
    pollfd p{client_, POLLIN, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) return 0;
    return static_cast<int>(::recv(client_, buf, n, 0));
  }
  bool writeClient(const char* data, std::size_t n) {
    return ::send(client_, data, n, 0) == static_cast<ssize_t>(n);
  }
  void closeClient() {
    if (client_ >= 0) ::close(client_);
    client_ = -1;
  }
  void stop() {
    closeClient();
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }
  ~TestListener() { stop(); }
  std::uint16_t port() const { return port_; }

 private:
  int fd_{-1};
  int client_{-1};
  std::uint16_t port_{0};
};

}  // namespace

TEST(TcpTransport, ConnectRefusedFails) {
  TestListener l;
  ASSERT_TRUE(l.start());
  const std::uint16_t dead_port = l.port();
  l.stop();  // nobody listens there anymore
  TcpClientTransport t;
  EXPECT_FALSE(t.connect("127.0.0.1", dead_port, 200));
  EXPECT_FALSE(t.connected());
}

TEST(TcpTransport, ConnectAndExchange) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  ASSERT_TRUE(t.send("ping", 4, 200));
  char buf[16] = {0};
  ASSERT_EQ(l.readClient(buf, sizeof(buf), 500), 4);
  EXPECT_EQ(std::string(buf, 4), "ping");
  ASSERT_TRUE(l.writeClient("pong!", 5));
  ASSERT_EQ(t.receive(buf, sizeof(buf), 500), 5);
  EXPECT_EQ(std::string(buf, 5), "pong!");
}

TEST(TcpTransport, ReceiveTimeoutReturnsZero) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  char buf[8];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 50), 0);
  EXPECT_TRUE(t.connected());
}

TEST(TcpTransport, PeerCloseReturnsMinusOne) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  l.closeClient();
  char buf[8];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 500), -1);
  EXPECT_FALSE(t.connected());
}

TEST(TcpTransport, SendWithoutConnectFails) {
  TcpClientTransport t;
  EXPECT_FALSE(t.send("x", 1, 100));
}

TEST(TcpTransport, ReconnectAfterClose) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  t.close();
  EXPECT_FALSE(t.connected());
  l.closeClient();
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  EXPECT_TRUE(t.connected());
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `tcp_client_transport.h`。

- [ ] **Step 3: 写实现**

`ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/tcp_client_transport.h`:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace sri {

// Minimal bounded-wait TCP client, structural twin of the UDP endpoint
// from kuka_rsi_hw_interface (poll()-based waits, no allocation after
// connect, not copyable). Any receive error / peer close / send failure
// closes the socket so connected() doubles as the reconnect trigger for
// the owning runtime. Not thread-safe: one owning thread at a time.
class TcpClientTransport {
 public:
  TcpClientTransport() = default;
  ~TcpClientTransport() { close(); }
  TcpClientTransport(const TcpClientTransport&) = delete;
  TcpClientTransport& operator=(const TcpClientTransport&) = delete;

  // Non-blocking connect with a bounded wait. false on refusal/timeout.
  bool connect(const std::string& ip, std::uint16_t port, int timeout_ms);
  bool connected() const { return fd_ >= 0; }

  // Waits up to timeout_ms. Returns byte count, 0 on timeout, -1 when the
  // peer closed or the socket errored (the transport closes itself).
  int receive(char* buf, std::size_t buf_size, int timeout_ms);

  // Sends the whole buffer within timeout_ms overall. false (and close)
  // on error or timeout.
  bool send(const char* data, std::size_t len, int timeout_ms);

  void close();

 private:
  int fd_{-1};
};

}  // namespace sri
```

`ros_ws/src/sri_force_torque_driver/src/tcp_client_transport.cpp`:

```cpp
#include "sri_force_torque_driver/tcp_client_transport.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>

namespace sri {

namespace {
int remainingMs(const std::chrono::steady_clock::time_point& deadline) {
  const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
  return left.count() > 0 ? static_cast<int>(left.count()) : 0;
}
}  // namespace

bool TcpClientTransport::connect(const std::string& ip, std::uint16_t port,
                                 int timeout_ms) {
  close();
  fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ < 0) return false;
  ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  int on = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    close();
    return false;
  }
  const int rc =
      ::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (rc != 0 && errno != EINPROGRESS) {
    close();
    return false;
  }
  if (rc != 0) {
    pollfd p{fd_, POLLOUT, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) {
      close();
      return false;
    }
    int err = 0;
    socklen_t len = sizeof(err);
    if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) {
      close();
      return false;
    }
  }
  return true;
}

int TcpClientTransport::receive(char* buf, std::size_t buf_size,
                                int timeout_ms) {
  if (fd_ < 0) return -1;
  pollfd p{fd_, POLLIN, 0};
  const int rc = ::poll(&p, 1, timeout_ms);
  if (rc == 0) return 0;
  if (rc < 0) {
    close();
    return -1;
  }
  const ssize_t n = ::recv(fd_, buf, buf_size, 0);
  if (n > 0) return static_cast<int>(n);
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
  close();  // n == 0: orderly peer close; n < 0: hard error
  return -1;
}

bool TcpClientTransport::send(const char* data, std::size_t len,
                              int timeout_ms) {
  if (fd_ < 0) return false;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  std::size_t sent = 0;
  while (sent < len) {
    pollfd p{fd_, POLLOUT, 0};
    if (::poll(&p, 1, remainingMs(deadline)) <= 0) {
      close();
      return false;
    }
#ifdef MSG_NOSIGNAL
    const ssize_t n = ::send(fd_, data + sent, len - sent, MSG_NOSIGNAL);
#else
    const ssize_t n = ::send(fd_, data + sent, len - sent, 0);
#endif
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
    if (n <= 0) {
      close();
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

void TcpClientTransport::close() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

}  // namespace sri
```

- [ ] **Step 4: CMake 增量**

`sri_ft_protocol` 源列表加 `src/tcp_client_transport.cpp`;测试段追加:

```cmake
  catkin_add_gtest(test_tcp_client_transport test/test_tcp_client_transport.cpp)
  target_link_libraries(test_tcp_client_transport sri_ft_protocol
                        ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/sri_force_torque_driver/test_tcp_client_transport
```

预期:`[  PASSED  ] 6 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/sri_force_torque_driver && \
git commit -m "feat(sri): bounded-wait TCP client transport (Plan 4 Task 3)"
```

---

### Task 4: `SriStreamSession`(超时/tare/偏置限幅/滤波/包号统计)

**Files:**
- Create: `ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_stream_session.h`
- Create: `ros_ws/src/sri_force_torque_driver/src/sri_stream_session.cpp`
- Modify: `ros_ws/src/sri_force_torque_driver/CMakeLists.txt`
- Test: `ros_ws/src/sri_force_torque_driver/test/test_sri_stream_session.cpp`

**Interfaces:**
- Consumes: Task 2 `SriFrameAssembler`/`FtSample`;Plan 1 `sfc::Wrench`、`sfc::ForceTorqueFilter(double)` / `filter(w, dt)` / `reset()`。
- Produces(Task 6 消费):
  - `struct SriSessionConfig { sample_timeout_s, nominal_rate_hz, zero_sample_count, filter_cutoff_hz, bias_limit_n }`
  - `struct SriWrenchSample { sfc::Wrench w; double stamp_s; std::uint16_t package_number; }`
  - `struct SriStatusSnapshot { streaming, last_sample_age_s, samples, bad_frames, package_gaps, zero_active, zero_rejects, filter_cutoff_hz, bias }`
  - `class SriStreamSession`:`configure` / `reset` / `feed(data, len, now_s, out, max_out)` / `startZero` / `cancelZero` / `zeroActive` / `lastZeroAccepted` / `setFilterCutoff` / `status(now_s)`。
- 处理顺序(每样本):原始值 → tare 捕获累加(**用原始值**)→ 扣偏置 → 可选低通 → 输出;滤波 dt 固定 = `1 / nominal_rate_hz`(传感器定速流,确定性优于时戳差);`setFilterCutoff` 重建 `sfc::ForceTorqueFilter`(隐含 reset,Plan 1 跟进 3 语义);`reset()`(重连时调用)重同步 assembler、reset 滤波器、**保留偏置**(传感器物理状态未变)。
- tare 语义(待确认 3/4):`startZero()` 后接下来 `zero_sample_count` 个原始样本取均值;完成时若均值力范数 > `bias_limit_n`(>0 时生效)则拒绝(保留旧偏置,`zero_rejects++`,`lastZeroAccepted()==false`);捕获期间照常发布(旧偏置)。

**测试期望值手工推导(执行者须复核)**:
- 低通:`sfc::ForceTorqueFilter` 的 `alpha = dt / (dt + 1/(2*pi*cutoff))`,首样本直通初始化。取 `nominal_rate_hz = 250`(dt = 0.004)且 `cutoff = 1/(2*pi*0.004)` ⇒ `1/(2*pi*cutoff) = 0.004` ⇒ **alpha = 0.004/(0.004+0.004) = 0.5**。序列 Fx: 0 → 2:第一样本输出 0(初始化),第二样本输出 `0 + 0.5*(2-0) = 1.0`。
- tare:`zero_sample_count = 2`,原始 Fx 序列 1.0、2.0 ⇒ 偏置 = (1.0+2.0)/2 = **1.5**;随后 Fx=2.0 的样本输出 **0.5**。
- 偏置限幅:`bias_limit_n = 1.0`,捕获 Fx=2.0、2.0 ⇒ 均值范数 2.0 > 1.0 ⇒ 拒绝,偏置仍 0,随后 Fx=2.0 输出 **2.0**。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/sri_force_torque_driver/test/test_sri_stream_session.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "sri_force_torque_driver/sri_stream_session.h"

using sri::SriSessionConfig;
using sri::SriStreamSession;
using sri::SriWrenchSample;

namespace {

constexpr double kNow = 50.0;  // arbitrary monotonic origin [s]

// Builds one valid frame for the given channels/package number. The
// checksum rule itself is pinned by the literal-byte tests of Task 2;
// this helper only reuses it.
std::vector<std::uint8_t> frame(const float ch[6], std::uint16_t pn) {
  std::vector<std::uint8_t> f = {0xAA, 0x55, 0x00, 0x1B,
                                 static_cast<std::uint8_t>(pn >> 8),
                                 static_cast<std::uint8_t>(pn & 0xFF)};
  unsigned sum = 0;
  for (int i = 0; i < 6; ++i) {
    std::uint8_t b[4];
    std::memcpy(b, &ch[i], 4);
    for (int k = 0; k < 4; ++k) {
      f.push_back(b[k]);
      sum += b[k];
    }
  }
  f.push_back(static_cast<std::uint8_t>(sum & 0xFF));
  return f;
}

std::vector<std::uint8_t> frameFx(float fx, std::uint16_t pn) {
  const float ch[6] = {fx, 0, 0, 0, 0, 0};
  return frame(ch, pn);
}

SriSessionConfig config() {
  SriSessionConfig c;
  c.sample_timeout_s = 0.1;
  c.nominal_rate_hz = 250.0;
  c.zero_sample_count = 2;
  c.filter_cutoff_hz = 0.0;
  c.bias_limit_n = 0.0;
  return c;
}

int feedOne(SriStreamSession& s, const std::vector<std::uint8_t>& bytes,
            double now, SriWrenchSample* out) {
  return s.feed(bytes.data(), bytes.size(), now, out, 8);
}

}  // namespace

TEST(SriSession, RawPassthroughByDefault) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  ASSERT_EQ(feedOne(s, frameFx(1.5f, 1), kNow, out), 1);
  EXPECT_NEAR(out[0].w.fx, 1.5, 1e-6);
  EXPECT_EQ(out[0].package_number, 1u);
}

TEST(SriSession, StampIsReceptionTime) {
  // Plan 3 follow-up 3: the stamp must be the reception instant handed
  // to feed(), never zero.
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  ASSERT_EQ(feedOne(s, frameFx(1.0f, 1), 123.456, out), 1);
  EXPECT_EQ(out[0].stamp_s, 123.456);
}

TEST(SriSession, ZeroCaptureAveragesAndSubtracts) {
  SriStreamSession s;
  s.configure(config());  // zero_sample_count = 2
  SriWrenchSample out[8];
  s.startZero();
  EXPECT_TRUE(s.zeroActive());
  feedOne(s, frameFx(1.0f, 1), kNow, out);
  feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out);
  EXPECT_FALSE(s.zeroActive());
  EXPECT_TRUE(s.lastZeroAccepted());
  // bias = (1.0 + 2.0) / 2 = 1.5; next raw 2.0 -> 0.5
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 3), kNow + 0.008, out), 1);
  EXPECT_NEAR(out[0].w.fx, 0.5, 1e-6);
  EXPECT_NEAR(s.status(kNow + 0.008).bias.fx, 1.5, 1e-6);
}

TEST(SriSession, ZeroRejectedAboveBiasLimit) {
  SriSessionConfig c = config();
  c.bias_limit_n = 1.0;  // legacy FTBia semantics (decision 4)
  SriStreamSession s;
  s.configure(c);
  SriWrenchSample out[8];
  s.startZero();
  feedOne(s, frameFx(2.0f, 1), kNow, out);
  feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out);
  EXPECT_FALSE(s.zeroActive());
  EXPECT_FALSE(s.lastZeroAccepted());
  EXPECT_EQ(s.status(kNow + 0.004).zero_rejects, 1u);
  // bias unchanged (0): raw 2.0 passes through unchanged.
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 3), kNow + 0.008, out), 1);
  EXPECT_NEAR(out[0].w.fx, 2.0, 1e-6);
}

TEST(SriSession, CancelZeroKeepsOldBias) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  s.startZero();
  feedOne(s, frameFx(1.0f, 1), kNow, out);  // capture 1/2
  s.cancelZero();
  EXPECT_FALSE(s.zeroActive());
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out), 1);
  EXPECT_NEAR(out[0].w.fx, 2.0, 1e-6);  // no bias was applied
}

TEST(SriSession, FilterHalvesStepWithConstructedAlpha) {
  SriSessionConfig c = config();
  // dt = 1/250 = 0.004; cutoff = 1/(2*pi*0.004) makes alpha exactly 0.5.
  c.filter_cutoff_hz = 1.0 / (2.0 * M_PI * 0.004);
  SriStreamSession s;
  s.configure(c);
  SriWrenchSample out[8];
  ASSERT_EQ(feedOne(s, frameFx(0.0f, 1), kNow, out), 1);
  EXPECT_NEAR(out[0].w.fx, 0.0, 1e-9);  // first sample initializes
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out), 1);
  EXPECT_NEAR(out[0].w.fx, 1.0, 1e-9);  // 0 + 0.5 * (2 - 0)
}

TEST(SriSession, SetFilterCutoffRebuildsAndResets) {
  SriSessionConfig c = config();
  c.filter_cutoff_hz = 1.0 / (2.0 * M_PI * 0.004);
  SriStreamSession s;
  s.configure(c);
  SriWrenchSample out[8];
  feedOne(s, frameFx(0.0f, 1), kNow, out);
  s.setFilterCutoff(0.0);  // back to passthrough; state discarded
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out), 1);
  EXPECT_NEAR(out[0].w.fx, 2.0, 1e-9);
  EXPECT_EQ(s.status(kNow + 0.004).filter_cutoff_hz, 0.0);
}

TEST(SriSession, PackageGapCountedWrapIsNot) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  feedOne(s, frameFx(0, 1), kNow, out);
  feedOne(s, frameFx(0, 3), kNow, out);  // 1 -> 3: one gap
  EXPECT_EQ(s.status(kNow).package_gaps, 1u);
  SriStreamSession s2;
  s2.configure(config());
  feedOne(s2, frameFx(0, 65535), kNow, out);
  feedOne(s2, frameFx(0, 0), kNow, out);  // wraparound: no gap
  EXPECT_EQ(s2.status(kNow).package_gaps, 0u);
}

TEST(SriSession, StreamingFlagFollowsSampleAge) {
  SriStreamSession s;
  s.configure(config());  // sample_timeout_s = 0.1
  SriWrenchSample out[8];
  EXPECT_FALSE(s.status(kNow).streaming);
  EXPECT_EQ(s.status(kNow).last_sample_age_s, -1.0);
  feedOne(s, frameFx(0, 1), kNow, out);
  EXPECT_TRUE(s.status(kNow + 0.05).streaming);
  EXPECT_NEAR(s.status(kNow + 0.05).last_sample_age_s, 0.05, 1e-9);
  EXPECT_FALSE(s.status(kNow + 0.2).streaming);
}

TEST(SriSession, BadFrameSurfacesInStatus) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  std::vector<std::uint8_t> bad = frameFx(1.0f, 1);
  bad.back() ^= 0xFF;  // corrupt checksum
  EXPECT_EQ(s.feed(bad.data(), bad.size(), kNow, out, 8), 0);
  EXPECT_EQ(s.status(kNow).bad_frames, 1u);
  EXPECT_EQ(s.status(kNow).samples, 0u);
}

TEST(SriSession, ResetKeepsBiasAndResyncsAssembler) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  s.startZero();
  feedOne(s, frameFx(1.0f, 1), kNow, out);
  feedOne(s, frameFx(1.0f, 2), kNow, out);  // bias = 1.0
  std::vector<std::uint8_t> half = frameFx(2.0f, 3);
  half.resize(10);  // truncated frame left in the assembler
  s.feed(half.data(), half.size(), kNow, out, 8);
  s.reset();  // reconnect: resync, keep bias
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 4), kNow + 0.1, out), 1);
  EXPECT_NEAR(out[0].w.fx, 1.0, 1e-6);  // 2.0 - kept bias 1.0
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `sri_stream_session.h`。

- [ ] **Step 3: 写实现**

`ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_stream_session.h`:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>

#include "soft_force_control_core/force_torque_filter.h"
#include "soft_force_control_core/types.h"
#include "sri_force_torque_driver/sri_frame.h"

namespace sri {

struct SriSessionConfig {
  double sample_timeout_s{0.1};   // streaming -> stalled threshold
  double nominal_rate_hz{250.0};  // fixed filter dt = 1/rate (decision 3)
  int zero_sample_count{100};     // samples averaged by one tare capture
  double filter_cutoff_hz{0.0};   // 0 = raw passthrough (default)
  double bias_limit_n{0.0};       // reject tare above this force norm; 0 = off
};

// One published-ready sample: bias-subtracted, optionally filtered.
struct SriWrenchSample {
  sfc::Wrench w;                    // N / Nm
  double stamp_s{0};                // reception time handed to feed()
  std::uint16_t package_number{0};
};

struct SriStatusSnapshot {
  bool streaming{false};
  double last_sample_age_s{-1.0};  // -1 until the first valid sample
  std::uint64_t samples{0};
  std::uint64_t bad_frames{0};     // checksum + length failures
  std::uint64_t package_gaps{0};
  bool zero_active{false};
  std::uint32_t zero_rejects{0};
  double filter_cutoff_hz{0.0};
  sfc::Wrench bias;
};

// Pure per-connection stream logic (spec 5.6 + decisions 2-4): raw TCP
// bytes in, timestamped wrench samples out. Per sample: raw value ->
// zero-capture accumulation (raw!) -> bias subtraction -> optional
// first-order low-pass (fixed dt = 1/nominal_rate_hz) -> output.
// reset() is the reconnect hook: resync assembler, reset filter state,
// KEEP the bias (the physical sensor state did not change). No ROS, no
// allocation, single-threaded (the runtime serializes access).
class SriStreamSession {
 public:
  void configure(const SriSessionConfig& cfg);
  void reset();

  int feed(const std::uint8_t* data, std::size_t len, double now_s,
           SriWrenchSample* out, int max_out);

  void startZero();
  void cancelZero();
  bool zeroActive() const { return zero_remaining_ > 0; }
  bool lastZeroAccepted() const { return last_zero_ok_; }

  // Rebuild + implicit reset (Plan 1 follow-up 3 semantics).
  void setFilterCutoff(double cutoff_hz);

  SriStatusSnapshot status(double now_s) const;

 private:
  void finishZeroCapture();

  SriSessionConfig cfg_;
  SriFrameAssembler assembler_;
  sfc::ForceTorqueFilter filter_{0.0};
  sfc::Wrench bias_;
  sfc::Wrench zero_accum_;
  int zero_remaining_{0};
  bool last_zero_ok_{false};
  std::uint32_t zero_rejects_{0};
  bool have_last_pn_{false};
  std::uint16_t last_pn_{0};
  double last_sample_s_{-1.0};
  std::uint64_t samples_{0};
  std::uint64_t gaps_{0};
};

}  // namespace sri
```

`ros_ws/src/sri_force_torque_driver/src/sri_stream_session.cpp`:

```cpp
#include "sri_force_torque_driver/sri_stream_session.h"

namespace sri {

namespace {
sfc::Wrench toWrench(const FtSample& s) {
  sfc::Wrench w;
  w.fx = s.ch[0];
  w.fy = s.ch[1];
  w.fz = s.ch[2];
  w.tx = s.ch[3];
  w.ty = s.ch[4];
  w.tz = s.ch[5];
  return w;
}
}  // namespace

void SriStreamSession::configure(const SriSessionConfig& cfg) {
  cfg_ = cfg;
  filter_ = sfc::ForceTorqueFilter(cfg_.filter_cutoff_hz);
}

void SriStreamSession::reset() {
  assembler_.reset();
  filter_.reset();
  have_last_pn_ = false;
  zero_remaining_ = 0;  // an interrupted capture is abandoned, bias kept
}

void SriStreamSession::startZero() {
  zero_remaining_ = cfg_.zero_sample_count;
  zero_accum_ = sfc::Wrench{};
}

void SriStreamSession::cancelZero() { zero_remaining_ = 0; }

void SriStreamSession::setFilterCutoff(double cutoff_hz) {
  cfg_.filter_cutoff_hz = cutoff_hz;
  filter_ = sfc::ForceTorqueFilter(cutoff_hz);
}

void SriStreamSession::finishZeroCapture() {
  const double n = static_cast<double>(cfg_.zero_sample_count);
  sfc::Wrench mean;
  mean.fx = zero_accum_.fx / n;
  mean.fy = zero_accum_.fy / n;
  mean.fz = zero_accum_.fz / n;
  mean.tx = zero_accum_.tx / n;
  mean.ty = zero_accum_.ty / n;
  mean.tz = zero_accum_.tz / n;
  if (cfg_.bias_limit_n > 0.0 && mean.forceNorm() > cfg_.bias_limit_n) {
    ++zero_rejects_;  // taring under load (legacy FTBia, decision 4)
    last_zero_ok_ = false;
    return;
  }
  bias_ = mean;
  filter_.reset();  // bias step must not smear through the filter state
  last_zero_ok_ = true;
}

int SriStreamSession::feed(const std::uint8_t* data, std::size_t len,
                           double now_s, SriWrenchSample* out, int max_out) {
  // 80 covers the worst-case burst of one 2048-byte read (66 x 31-byte
  // frames); anything beyond is counted by the assembler as dropped.
  FtSample raw[80];
  const int n = assembler_.feed(data, len, raw, 80);
  int written = 0;
  for (int i = 0; i < n; ++i) {
    const sfc::Wrench w = toWrench(raw[i]);
    if (have_last_pn_) {
      const std::uint16_t expected =
          static_cast<std::uint16_t>(last_pn_ + 1u);
      if (raw[i].package_number != expected) ++gaps_;
    }
    last_pn_ = raw[i].package_number;
    have_last_pn_ = true;
    ++samples_;
    last_sample_s_ = now_s;
    if (zero_remaining_ > 0) {
      zero_accum_.fx += w.fx;
      zero_accum_.fy += w.fy;
      zero_accum_.fz += w.fz;
      zero_accum_.tx += w.tx;
      zero_accum_.ty += w.ty;
      zero_accum_.tz += w.tz;
      if (--zero_remaining_ == 0) finishZeroCapture();
    }
    sfc::Wrench unbiased;
    unbiased.fx = w.fx - bias_.fx;
    unbiased.fy = w.fy - bias_.fy;
    unbiased.fz = w.fz - bias_.fz;
    unbiased.tx = w.tx - bias_.tx;
    unbiased.ty = w.ty - bias_.ty;
    unbiased.tz = w.tz - bias_.tz;
    const double dt = 1.0 / cfg_.nominal_rate_hz;
    const sfc::Wrench filtered = filter_.filter(unbiased, dt);
    if (written < max_out) {
      out[written].w = filtered;
      out[written].stamp_s = now_s;
      out[written].package_number = raw[i].package_number;
      ++written;
    }
  }
  return written;
}

SriStatusSnapshot SriStreamSession::status(double now_s) const {
  SriStatusSnapshot s;
  s.samples = samples_;
  s.bad_frames = assembler_.stats().bad_checksum + assembler_.stats().bad_length;
  s.package_gaps = gaps_;
  s.zero_active = zeroActive();
  s.zero_rejects = zero_rejects_;
  s.filter_cutoff_hz = cfg_.filter_cutoff_hz;
  s.bias = bias_;
  if (last_sample_s_ >= 0.0) {
    s.last_sample_age_s = now_s - last_sample_s_;
    s.streaming = s.last_sample_age_s <= cfg_.sample_timeout_s;
  }
  return s;
}

}  // namespace sri
```

- [ ] **Step 4: CMake 增量**

`sri_ft_protocol` 源列表加 `src/sri_stream_session.cpp`;测试段追加:

```cmake
  catkin_add_gtest(test_sri_stream_session test/test_sri_stream_session.cpp)
  target_link_libraries(test_sri_stream_session sri_ft_protocol
                        ${catkin_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/sri_force_torque_driver/test_sri_stream_session
```

预期:`[  PASSED  ] 11 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/sri_force_torque_driver && \
git commit -m "feat(sri): stream session with tare, bias limit, optional LPF, gap stats (Plan 4 Task 4)"
```

---

### Task 5: `SriMockServer`(可脚本化传感器 mock)+ 独立可执行 + 闭环测试

**Files:**
- Create: `ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_mock_server.h`
- Create: `ros_ws/src/sri_force_torque_driver/src/sri_mock_server.cpp`
- Create: `ros_ws/src/sri_force_torque_driver/src/sri_mock_server_main.cpp`
- Modify: `ros_ws/src/sri_force_torque_driver/CMakeLists.txt`
- Test: `ros_ws/src/sri_force_torque_driver/test/test_sri_mock_server.cpp`

**Interfaces:**
- Consumes: Task 2 帧布局常量(mock 自行编码帧,与 assembler 解码互为镜像)、POSIX socket。
- Produces(Task 6 与 Plan 5 消费):
  - `struct SriMockConfig { bool require_start_command{true}; double rate_hz{0.0}; }`(`rate_hz=0` ⇒ 只按脚本 `sendFrames` 发帧,gtest 用;>0 ⇒ 自主定速流,冒烟/Plan 5 用)。
  - `class SriMockServer`:`start()`(bind 127.0.0.1:0,kernel 选port)/ `stop()` / `port()` / `waitForClient` / `waitForStartCommand` / `setWrench(6 floats)` / `sendFrames(count)` / `sendBadChecksumFrame()` / `sendRaw(data, len)` / `dropClient()` / `framesSent()`。
  - 独立可执行 `sri_mock_server`:`--rate`(默认 250)`--fz N` `--sine-amp A --sine-hz F`(Fz 上叠加正弦)`--bad-every N`(每 N 帧注入一坏帧);监听 127.0.0.1 内核自选端口并打印(避免与真传感器端口约定耦合;固定端口需求由 Plan 5 launch 包 socat 或直接改参,首版不做)。
- 行为约定:单客户端;收到 `AT+GSD` 前(`require_start_command=true` 时)不发任何帧并回 ASCII `ACK+GSD=OK\r\n`(顺带覆盖 assembler 的 ASCII 跳过路径);`sendFrames` 在未进入流状态时静默丢弃(测试用其验证"未启动不发帧");线程/锁允许(mock 为测试代码,同 Plan 2 `RsiMockServer` 先例);`dropClient` 用 `shutdown(SHUT_RDWR)` 唤醒 mock 线程后关闭。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/sri_force_torque_driver/test/test_sri_mock_server.cpp`:

```cpp
#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <thread>

#include "sri_force_torque_driver/sri_frame.h"
#include "sri_force_torque_driver/sri_mock_server.h"
#include "sri_force_torque_driver/tcp_client_transport.h"

using sri::FtSample;
using sri::SriFrameAssembler;
using sri::SriMockConfig;
using sri::SriMockServer;
using sri::TcpClientTransport;

namespace {

// Bounded receive loop: keeps feeding the assembler until `want` samples
// arrived or the deadline passed. All waits bounded (<= 2 s total).
int receiveSamples(TcpClientTransport& t, SriFrameAssembler& a, FtSample* out,
                   int want, int deadline_ms = 1000) {
  int got = 0;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(deadline_ms);
  char buf[2048];
  while (got < want && std::chrono::steady_clock::now() < deadline) {
    const int n = t.receive(buf, sizeof(buf), 50);
    if (n < 0) break;
    if (n == 0) continue;
    got += a.feed(reinterpret_cast<const std::uint8_t*>(buf),
                  static_cast<std::size_t>(n), out + got, want - got);
  }
  return got;
}

void startStream(TcpClientTransport& t, SriMockServer& mock) {
  ASSERT_TRUE(t.send(sri::startStreamCommand(),
                     std::strlen(sri::startStreamCommand()), 200));
  ASSERT_TRUE(mock.waitForStartCommand(500));
}

}  // namespace

TEST(SriMock, ScriptedFramesReachClient) {
  SriMockServer mock{SriMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  startStream(t, mock);
  mock.setWrench(1.0f, 0.0f, 2.0f, 0.0f, -1.0f, 0.0f);
  mock.sendFrames(3);
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(receiveSamples(t, a, out, 3), 3);
  EXPECT_FLOAT_EQ(out[0].ch[0], 1.0f);
  EXPECT_FLOAT_EQ(out[2].ch[2], 2.0f);
  EXPECT_EQ(out[1].package_number, out[0].package_number + 1u);
}

TEST(SriMock, HoldsFramesUntilStartCommand) {
  SriMockServer mock{SriMockConfig{}};  // require_start_command = true
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  mock.sendFrames(2);  // silently ignored: not streaming yet
  SriFrameAssembler a;
  FtSample out[4];
  EXPECT_EQ(receiveSamples(t, a, out, 1, 150), 0);
  startStream(t, mock);
  mock.sendFrames(2);
  EXPECT_EQ(receiveSamples(t, a, out, 2), 2);
}

TEST(SriMock, BadChecksumFrameIsCountedNotDecoded) {
  SriMockServer mock{SriMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  startStream(t, mock);
  mock.sendBadChecksumFrame();
  mock.sendFrames(1);
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(receiveSamples(t, a, out, 1), 1);
  EXPECT_EQ(a.stats().bad_checksum, 1u);
}

TEST(SriMock, GarbageBytesThenValidFrameRecovers) {
  SriMockServer mock{SriMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  startStream(t, mock);
  const char garbage[] = "\x01\x02\x03garbage\xAA";
  mock.sendRaw(garbage, sizeof(garbage) - 1);
  mock.sendFrames(1);
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(receiveSamples(t, a, out, 1), 1);
  EXPECT_GT(a.stats().skipped_bytes, 0u);
}

TEST(SriMock, DropClientDetectedByTransport) {
  SriMockServer mock{SriMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  mock.dropClient();
  char buf[64];
  int rc = 0;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
  while (std::chrono::steady_clock::now() < deadline) {
    rc = t.receive(buf, sizeof(buf), 50);
    if (rc != 0) break;
  }
  EXPECT_EQ(rc, -1);
  EXPECT_FALSE(t.connected());
}

TEST(SriMock, PacedModeStreamsWithoutScripting) {
  SriMockConfig cfg;
  cfg.rate_hz = 200.0;
  SriMockServer mock{cfg};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  startStream(t, mock);
  SriFrameAssembler a;
  FtSample out[8];
  EXPECT_GE(receiveSamples(t, a, out, 5, 1000), 5);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `sri_mock_server.h`。

- [ ] **Step 3: 写实现**

`ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_mock_server.h`:

```cpp
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace sri {

struct SriMockConfig {
  bool require_start_command{true};  // hold frames until AT+GSD arrives
  double rate_hz{0.0};               // 0 = frames only via sendFrames()
};

// Scriptable stand-in for the SRI acquisition box: TCP server on
// 127.0.0.1 (kernel-chosen port), single client, answers AT+GSD with an
// ASCII ACK line, then emits binary frames on demand (scripted) or paced
// (rate_hz > 0). Test code: threads/locks are fine here, this never runs
// in a realtime path (same stance as kuka_rsi::RsiMockServer).
class SriMockServer {
 public:
  explicit SriMockServer(const SriMockConfig& cfg);
  ~SriMockServer();
  SriMockServer(const SriMockServer&) = delete;
  SriMockServer& operator=(const SriMockServer&) = delete;

  bool start();
  void stop();
  std::uint16_t port() const { return port_; }

  bool waitForClient(int timeout_ms);
  bool waitForStartCommand(int timeout_ms);

  void setWrench(float fx, float fy, float fz, float mx, float my, float mz);
  void sendFrames(int count);       // ignored while not streaming
  void sendBadChecksumFrame();
  void sendRaw(const void* data, std::size_t len);
  void dropClient();
  std::uint64_t framesSent() const { return frames_sent_.load(); }

 private:
  void run();
  void sendLocked(const void* data, std::size_t len);
  std::size_t buildFrame(std::uint8_t* buf, bool corrupt_checksum);

  SriMockConfig cfg_;
  int listen_fd_{-1};
  std::uint16_t port_{0};
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> streaming_{false};
  std::atomic<std::uint64_t> frames_sent_{0};
  mutable std::mutex mutex_;  // guards client_fd_, wrench_, pn_
  int client_fd_{-1};
  float wrench_[6] = {0, 0, 0, 0, 0, 0};
  std::uint16_t pn_{0};
};

}  // namespace sri
```

`ros_ws/src/sri_force_torque_driver/src/sri_mock_server.cpp`:

```cpp
#include "sri_force_torque_driver/sri_mock_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace sri {

namespace {
constexpr std::size_t kFrameLen = 31;  // AA 55 LL LL PN PN 24xdata SUM
constexpr int kPollMs = 10;

bool waitFlag(const std::atomic<bool>& flag, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (flag.load()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return flag.load();
}
}  // namespace

SriMockServer::SriMockServer(const SriMockConfig& cfg) : cfg_(cfg) {}

SriMockServer::~SriMockServer() { stop(); }

bool SriMockServer::start() {
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) return false;
  int on = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = ::inet_addr("127.0.0.1");
  addr.sin_port = 0;
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) !=
          0 ||
      ::listen(listen_fd_, 1) != 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  socklen_t len = sizeof(addr);
  ::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
  port_ = ntohs(addr.sin_port);
  running_ = true;
  thread_ = std::thread(&SriMockServer::run, this);
  return true;
}

void SriMockServer::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  dropClient();
  if (listen_fd_ >= 0) ::close(listen_fd_);
  listen_fd_ = -1;
}

bool SriMockServer::waitForClient(int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (client_fd_ >= 0) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return client_fd_ >= 0;
}

bool SriMockServer::waitForStartCommand(int timeout_ms) {
  return waitFlag(streaming_, timeout_ms);
}

void SriMockServer::setWrench(float fx, float fy, float fz, float mx,
                              float my, float mz) {
  std::lock_guard<std::mutex> lock(mutex_);
  wrench_[0] = fx;
  wrench_[1] = fy;
  wrench_[2] = fz;
  wrench_[3] = mx;
  wrench_[4] = my;
  wrench_[5] = mz;
}

std::size_t SriMockServer::buildFrame(std::uint8_t* buf,
                                      bool corrupt_checksum) {
  // Caller holds mutex_ (wrench_/pn_ access).
  buf[0] = 0xAA;
  buf[1] = 0x55;
  buf[2] = 0x00;
  buf[3] = 0x1B;  // payload length 27 = PN(2) + data(24) + sum(1)
  ++pn_;
  buf[4] = static_cast<std::uint8_t>(pn_ >> 8);
  buf[5] = static_cast<std::uint8_t>(pn_ & 0xFF);
  unsigned sum = 0;
  for (int i = 0; i < 6; ++i) {
    std::memcpy(buf + 6 + 4 * i, &wrench_[i], 4);  // little-endian host
  }
  for (int k = 6; k < 30; ++k) sum += buf[k];
  buf[30] = static_cast<std::uint8_t>(sum & 0xFF);
  if (corrupt_checksum) buf[30] = static_cast<std::uint8_t>(buf[30] ^ 0xFF);
  return kFrameLen;
}

void SriMockServer::sendLocked(const void* data, std::size_t len) {
  // Caller holds mutex_.
  if (client_fd_ < 0) return;
#ifdef MSG_NOSIGNAL
  ::send(client_fd_, data, len, MSG_NOSIGNAL);
#else
  ::send(client_fd_, data, len, 0);
#endif
}

void SriMockServer::sendFrames(int count) {
  if (!streaming_.load() && cfg_.require_start_command) return;
  std::lock_guard<std::mutex> lock(mutex_);
  std::uint8_t buf[kFrameLen];
  for (int i = 0; i < count; ++i) {
    buildFrame(buf, false);
    sendLocked(buf, kFrameLen);
    ++frames_sent_;
  }
}

void SriMockServer::sendBadChecksumFrame() {
  if (!streaming_.load() && cfg_.require_start_command) return;
  std::lock_guard<std::mutex> lock(mutex_);
  std::uint8_t buf[kFrameLen];
  buildFrame(buf, true);
  sendLocked(buf, kFrameLen);
}

void SriMockServer::sendRaw(const void* data, std::size_t len) {
  std::lock_guard<std::mutex> lock(mutex_);
  sendLocked(data, len);
}

void SriMockServer::dropClient() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (client_fd_ >= 0) {
    ::shutdown(client_fd_, SHUT_RDWR);
    ::close(client_fd_);
    client_fd_ = -1;
  }
  streaming_ = false;
}

void SriMockServer::run() {
  using clock = std::chrono::steady_clock;
  auto next_paced = clock::now();
  while (running_.load()) {
    int client;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      client = client_fd_;
    }
    if (client < 0) {
      pollfd p{listen_fd_, POLLIN, 0};
      if (::poll(&p, 1, kPollMs) > 0) {
        const int fd = ::accept(listen_fd_, nullptr, nullptr);
        if (fd >= 0) {
          std::lock_guard<std::mutex> lock(mutex_);
          client_fd_ = fd;
          streaming_ = !cfg_.require_start_command;
        }
      }
      continue;
    }
    pollfd p{client, POLLIN, 0};
    if (::poll(&p, 1, kPollMs) > 0) {
      char buf[256];
      const ssize_t n = ::recv(client, buf, sizeof(buf), 0);
      if (n <= 0) {
        dropClient();
        continue;
      }
      static const char kCmd[] = "AT+GSD";
      if (std::search(buf, buf + n, kCmd, kCmd + 6) != buf + n) {
        static const char kAck[] = "ACK+GSD=OK\r\n";
        std::lock_guard<std::mutex> lock(mutex_);
        sendLocked(kAck, sizeof(kAck) - 1);
        streaming_ = true;
      }
    }
    if (cfg_.rate_hz > 0.0 && streaming_.load() &&
        clock::now() >= next_paced) {
      std::lock_guard<std::mutex> lock(mutex_);
      std::uint8_t frame[kFrameLen];
      buildFrame(frame, false);
      sendLocked(frame, kFrameLen);
      ++frames_sent_;
      next_paced = clock::now() + std::chrono::microseconds(
                                      static_cast<long>(1e6 / cfg_.rate_hz));
    }
  }
}

}  // namespace sri
```

`ros_ws/src/sri_force_torque_driver/src/sri_mock_server_main.cpp`:

```cpp
// Standalone SRI sensor mock for manual smoke tests and Plan 5 bringup.
// Continuous waveform (optional sine on Fz) and periodic fault injection.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "sri_force_torque_driver/sri_mock_server.h"

namespace {

// This binary needs a fixed port (real clients configure it), so it wraps
// the library mock only for waveform scripting and uses rate 0 + explicit
// sendFrames pacing here.
struct Options {
  double rate_hz = 250.0;
  double fz = 0.0;
  double sine_amp = 0.0;
  double sine_hz = 0.0;
  int bad_every = 0;
};

int usage(int code) {
  std::printf(
      "usage: sri_mock_server [--rate HZ] [--fz N] [--sine-amp N]\n"
      "                       [--sine-hz HZ] [--bad-every N]\n"
      "Listens on a kernel-chosen 127.0.0.1 port (printed on stdout).\n");
  return code;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const bool has_val = i + 1 < argc;
    if (a == "--help" || a == "-h") return usage(0);
    if (!has_val) return usage(1);
    const double v = std::atof(argv[++i]);
    if (a == "--rate") opt.rate_hz = v;
    else if (a == "--fz") opt.fz = v;
    else if (a == "--sine-amp") opt.sine_amp = v;
    else if (a == "--sine-hz") opt.sine_hz = v;
    else if (a == "--bad-every") opt.bad_every = static_cast<int>(v);
    else return usage(1);
  }
  sri::SriMockConfig cfg;
  cfg.require_start_command = true;
  cfg.rate_hz = 0.0;  // paced manually below so the waveform can evolve
  sri::SriMockServer mock(cfg);
  if (!mock.start()) {
    std::fprintf(stderr, "bind/listen failed\n");
    return 1;
  }
  std::printf("sri_mock_server listening on 127.0.0.1:%u (rate %.0f Hz)\n",
              mock.port(), opt.rate_hz);
  std::fflush(stdout);
  const auto period = std::chrono::microseconds(
      static_cast<long>(1e6 / (opt.rate_hz > 0 ? opt.rate_hz : 250.0)));
  double t = 0.0;
  long frame_no = 0;
  for (;;) {
    const double fz =
        opt.fz + opt.sine_amp * std::sin(2.0 * M_PI * opt.sine_hz * t);
    mock.setWrench(0.0f, 0.0f, static_cast<float>(fz), 0.0f, 0.0f, 0.0f);
    ++frame_no;
    if (opt.bad_every > 0 && frame_no % opt.bad_every == 0) {
      mock.sendBadChecksumFrame();
    } else {
      mock.sendFrames(1);
    }
    t += 1e-6 * static_cast<double>(period.count());
    std::this_thread::sleep_for(period);
  }
  return 0;
}
```

- [ ] **Step 4: CMake 增量**

```cmake
# after the sri_ft_protocol block
add_library(sri_ft_mock
  src/sri_mock_server.cpp
)
target_link_libraries(sri_ft_mock sri_ft_protocol pthread)

add_executable(sri_mock_server src/sri_mock_server_main.cpp)
target_link_libraries(sri_mock_server sri_ft_mock)
```

`catkin_package(LIBRARIES ...)` 改为 `LIBRARIES sri_ft_protocol sri_ft_mock`。测试段追加:

```cmake
  catkin_add_gtest(test_sri_mock_server test/test_sri_mock_server.cpp)
  target_link_libraries(test_sri_mock_server sri_ft_mock
                        ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/sri_force_torque_driver/test_sri_mock_server
```

预期:`[  PASSED  ] 6 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/sri_force_torque_driver && \
git commit -m "feat(sri): scriptable sensor mock server + standalone waveform exe (Plan 4 Task 5)"
```

---

### Task 6: `SriDriverRuntime` + `sri_driver_node` ROS 壳 + `sri_ft.yaml`

**Files:**
- Create: `ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_driver_runtime.h`
- Create: `ros_ws/src/sri_force_torque_driver/src/sri_driver_runtime.cpp`
- Create: `ros_ws/src/sri_force_torque_driver/src/sri_driver_node.cpp`
- Create: `ros_ws/src/sri_force_torque_driver/config/sri_ft.yaml`
- Modify: `ros_ws/src/sri_force_torque_driver/CMakeLists.txt`
- Test: `ros_ws/src/sri_force_torque_driver/test/test_sri_driver_runtime.cpp`

**Interfaces:**
- Consumes: Task 3/4/5 全部产物。
- Produces(节点 + Plan 5 消费):
  - `struct SriDriverConfig { sensor_ip, sensor_port, connect_timeout_ms, receive_timeout_ms, reconnect_backoff_s, SriSessionConfig session }`
  - `struct SriDriverStatus { bool connected; std::uint32_t reconnects; SriStatusSnapshot session; }`
  - `class SriDriverRuntime`:`setSampleCallback`(start 前设置一次)/ `start` / `stop` / `requestZero(timeout_ms)`(有界,超时取消捕获)/ `setFilterCutoff` / `status()`。
  - ROS 节点 `sri_driver_node`:发布 `/sri_ft/wrench_raw`(WrenchStamped,每样本一条,**stamp = 接收时刻**)与 `/sri_ft/status`(SriStatus,10 Hz);服务 `/sri_ft/zero`(Trigger)、`/sri_ft/set_filter`(SetFilter)。
- 线程模型(待确认 12):rx 线程独占 transport 与 session 的 feed 路径;服务线程经 mutex 调 session 的 tare/filter 方法;callback 在 rx 线程同步执行(节点在其中打 `ros::Time::now()` 戳并 publish——`ros::Publisher::publish` 线程安全)。
- 离线可测性:runtime 全路径对 `SriMockServer` 闭环(无 roscore);节点壳(参数装载 + 转发)不做离线单测,Step 7 手动冒烟(非验收门槛)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/sri_force_torque_driver/test/test_sri_driver_runtime.cpp`:

```cpp
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "sri_force_torque_driver/sri_driver_runtime.h"
#include "sri_force_torque_driver/sri_mock_server.h"

using sri::SriDriverConfig;
using sri::SriDriverRuntime;
using sri::SriMockConfig;
using sri::SriMockServer;
using sri::SriWrenchSample;

namespace {

// Collects callback samples across threads.
class Sink {
 public:
  void push(const SriWrenchSample& s) {
    std::lock_guard<std::mutex> lock(m_);
    samples_.push_back(s);
  }
  std::size_t count() const {
    std::lock_guard<std::mutex> lock(m_);
    return samples_.size();
  }
  SriWrenchSample at(std::size_t i) const {
    std::lock_guard<std::mutex> lock(m_);
    return samples_.at(i);
  }
  SriWrenchSample last() const {
    std::lock_guard<std::mutex> lock(m_);
    return samples_.back();
  }

 private:
  mutable std::mutex m_;
  std::vector<SriWrenchSample> samples_;
};

bool waitFor(const std::function<bool()>& pred, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

SriDriverConfig config(std::uint16_t port) {
  SriDriverConfig c;
  c.sensor_ip = "127.0.0.1";
  c.sensor_port = port;
  c.connect_timeout_ms = 200;
  c.receive_timeout_ms = 20;
  c.reconnect_backoff_s = 0.05;
  c.session.sample_timeout_s = 0.1;
  c.session.nominal_rate_hz = 200.0;
  c.session.zero_sample_count = 5;
  c.session.filter_cutoff_hz = 0.0;
  c.session.bias_limit_n = 0.0;
  return c;
}

SriMockConfig pacedMock() {
  SriMockConfig m;
  m.rate_hz = 200.0;
  return m;
}

}  // namespace

TEST(SriRuntime, StreamsAndDeliversSamples) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  mock.setWrench(1.5f, 0, 0, 0, 0, 0);
  Sink sink;
  SriDriverRuntime rt{config(mock.port())};
  rt.setSampleCallback([&sink](const SriWrenchSample& s) { sink.push(s); });
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return sink.count() >= 5; }, 2000));
  EXPECT_NEAR(sink.last().w.fx, 1.5, 1e-6);
  EXPECT_TRUE(rt.status().connected);
  rt.stop();
}

TEST(SriRuntime, StampsAreNonZeroAndMonotonic) {
  // Plan 3 follow-up 3 hardening: reception-instant stamps, never zero.
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  Sink sink;
  SriDriverRuntime rt{config(mock.port())};
  rt.setSampleCallback([&sink](const SriWrenchSample& s) { sink.push(s); });
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return sink.count() >= 3; }, 2000));
  rt.stop();
  EXPECT_GT(sink.at(0).stamp_s, 0.0);
  EXPECT_GE(sink.at(1).stamp_s, sink.at(0).stamp_s);
  EXPECT_GE(sink.at(2).stamp_s, sink.at(1).stamp_s);
}

TEST(SriRuntime, RequestZeroBiasesSubsequentSamples) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  mock.setWrench(2.0f, 0, 0, 0, 0, 0);
  Sink sink;
  SriDriverRuntime rt{config(mock.port())};
  rt.setSampleCallback([&sink](const SriWrenchSample& s) { sink.push(s); });
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return sink.count() >= 2; }, 2000));
  ASSERT_TRUE(rt.requestZero(1000));  // averages 5 samples of constant 2.0
  const std::size_t base = sink.count();
  ASSERT_TRUE(waitFor([&] { return sink.count() >= base + 2; }, 2000));
  EXPECT_NEAR(sink.last().w.fx, 0.0, 1e-5);  // 2.0 - bias 2.0
  rt.stop();
}

TEST(SriRuntime, RequestZeroTimesOutWithoutStream) {
  SriMockConfig m;  // scripted mode + start-command gate: no frames flow
  SriMockServer mock{m};
  ASSERT_TRUE(mock.start());
  SriDriverRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 1000));
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(rt.requestZero(200));
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                .count(),
            1000);
  EXPECT_FALSE(rt.status().session.zero_active);  // capture was cancelled
  rt.stop();
}

TEST(SriRuntime, ReconnectsAfterDrop) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  Sink sink;
  SriDriverRuntime rt{config(mock.port())};
  rt.setSampleCallback([&sink](const SriWrenchSample& s) { sink.push(s); });
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return sink.count() >= 2; }, 2000));
  mock.dropClient();
  ASSERT_TRUE(waitFor([&] { return rt.status().reconnects >= 1; }, 2000));
  const std::size_t base = sink.count();
  ASSERT_TRUE(waitFor([&] { return sink.count() > base; }, 2000));
  rt.stop();
}

TEST(SriRuntime, SetFilterCutoffReflectedInStatus) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  SriDriverRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  EXPECT_TRUE(rt.setFilterCutoff(10.0));
  EXPECT_EQ(rt.status().session.filter_cutoff_hz, 10.0);
  EXPECT_FALSE(rt.setFilterCutoff(-1.0));  // negative rejected
  rt.stop();
}

TEST(SriRuntime, StopJoinsQuickly) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  SriDriverRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 1000));
  const auto t0 = std::chrono::steady_clock::now();
  rt.stop();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
  EXPECT_LT(ms, 1000);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `sri_driver_runtime.h`。

- [ ] **Step 3: 写 runtime 实现**

`ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_driver_runtime.h`:

```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "sri_force_torque_driver/sri_stream_session.h"
#include "sri_force_torque_driver/tcp_client_transport.h"

namespace sri {

struct SriDriverConfig {
  std::string sensor_ip{"127.0.0.1"};
  std::uint16_t sensor_port{4008};
  int connect_timeout_ms{1000};
  int receive_timeout_ms{50};
  double reconnect_backoff_s{0.5};
  SriSessionConfig session;
};

struct SriDriverStatus {
  bool connected{false};
  std::uint32_t reconnects{0};  // completed re-connections after the first
  SriStatusSnapshot session;
};

// Owns the rx thread: bounded connect -> AT+GSD -> receive/parse loop with
// automatic reconnect. The sample callback runs synchronously in the rx
// thread right after the socket read, so its invocation time IS the
// reception instant (the node stamps messages there; Plan 3 follow-up 3).
// Not ROS-dependent. Service-facing methods (requestZero/setFilterCutoff/
// status) synchronize with the rx thread through mutex_.
class SriDriverRuntime {
 public:
  using SampleCallback = std::function<void(const SriWrenchSample&)>;

  explicit SriDriverRuntime(const SriDriverConfig& cfg);
  ~SriDriverRuntime();
  SriDriverRuntime(const SriDriverRuntime&) = delete;
  SriDriverRuntime& operator=(const SriDriverRuntime&) = delete;

  void setSampleCallback(SampleCallback cb);  // call before start()
  bool start();
  void stop();

  // Starts a tare capture and waits (bounded) for it to finish. false on
  // timeout (capture cancelled, old bias kept) or rejected bias limit.
  bool requestZero(int timeout_ms);
  bool setFilterCutoff(double cutoff_hz);  // false for negative values
  SriDriverStatus status() const;

 private:
  void run();
  static double nowS();

  SriDriverConfig cfg_;
  SampleCallback callback_;
  TcpClientTransport transport_;  // rx thread only
  std::thread thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex mutex_;  // guards session_, connected_, reconnects_
  SriStreamSession session_;
  bool connected_{false};
  bool was_connected_once_{false};
  std::uint32_t reconnects_{0};
};

}  // namespace sri
```

`ros_ws/src/sri_force_torque_driver/src/sri_driver_runtime.cpp`:

```cpp
#include "sri_force_torque_driver/sri_driver_runtime.h"

#include <chrono>
#include <cstring>

namespace sri {

namespace {
void sleepSlices(double seconds, const std::atomic<bool>& keep_running) {
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::microseconds(static_cast<long>(seconds * 1e6));
  while (keep_running.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
}  // namespace

SriDriverRuntime::SriDriverRuntime(const SriDriverConfig& cfg) : cfg_(cfg) {
  session_.configure(cfg_.session);
}

SriDriverRuntime::~SriDriverRuntime() { stop(); }

double SriDriverRuntime::nowS() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void SriDriverRuntime::setSampleCallback(SampleCallback cb) {
  callback_ = std::move(cb);
}

bool SriDriverRuntime::start() {
  if (running_.load()) return false;
  running_ = true;
  thread_ = std::thread(&SriDriverRuntime::run, this);
  return true;
}

void SriDriverRuntime::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  transport_.close();
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = false;
}

bool SriDriverRuntime::requestZero(int timeout_ms) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    session_.startZero();
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!session_.zeroActive()) return session_.lastZeroAccepted();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!session_.zeroActive()) return session_.lastZeroAccepted();
  session_.cancelZero();
  return false;
}

bool SriDriverRuntime::setFilterCutoff(double cutoff_hz) {
  if (cutoff_hz < 0.0) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  session_.setFilterCutoff(cutoff_hz);
  return true;
}

SriDriverStatus SriDriverRuntime::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  SriDriverStatus s;
  s.connected = connected_;
  s.reconnects = reconnects_;
  s.session = session_.status(nowS());
  return s;
}

void SriDriverRuntime::run() {
  char buf[2048];
  SriWrenchSample samples[80];
  while (running_.load()) {
    if (!transport_.connected()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = false;
      }
      if (!transport_.connect(cfg_.sensor_ip, cfg_.sensor_port,
                              cfg_.connect_timeout_ms)) {
        sleepSlices(cfg_.reconnect_backoff_s, running_);
        continue;
      }
      transport_.send(startStreamCommand(),
                      std::strlen(startStreamCommand()), 200);
      std::lock_guard<std::mutex> lock(mutex_);
      session_.reset();
      connected_ = true;
      if (was_connected_once_) ++reconnects_;
      was_connected_once_ = true;
    }
    const int n = transport_.receive(buf, sizeof(buf),
                                     cfg_.receive_timeout_ms);
    if (n < 0) continue;  // next iteration handles reconnect
    if (n == 0) continue;
    int count;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      count = session_.feed(reinterpret_cast<const std::uint8_t*>(buf),
                            static_cast<std::size_t>(n), nowS(), samples, 80);
    }
    for (int i = 0; i < count; ++i) {
      if (callback_) callback_(samples[i]);
    }
  }
}

}  // namespace sri
```

- [ ] **Step 4: 写节点壳与配置**

`ros_ws/src/sri_force_torque_driver/src/sri_driver_node.cpp`:

```cpp
// Thin ROS shell around SriDriverRuntime (spec 5.6). All protocol/session
// logic lives in the offline-tested library; this file only loads
// parameters and forwards data. Topics/services use the absolute names
// fixed by the spec (/sri_ft/...).
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <soft_robot_msgs/SetFilter.h>
#include <soft_robot_msgs/SriStatus.h>
#include <std_srvs/Trigger.h>

#include <string>

#include "sri_force_torque_driver/sri_driver_runtime.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sri_ft_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  sri::SriDriverConfig cfg;
  int port = 4008;
  int zero_timeout_ms = 3000;
  std::string frame_id;
  pnh.param<std::string>("sensor_ip", cfg.sensor_ip, "192.168.1.1");
  pnh.param("sensor_port", port, 4008);
  cfg.sensor_port = static_cast<std::uint16_t>(port);
  pnh.param("connect_timeout_ms", cfg.connect_timeout_ms, 1000);
  pnh.param("receive_timeout_ms", cfg.receive_timeout_ms, 50);
  pnh.param("reconnect_backoff_s", cfg.reconnect_backoff_s, 0.5);
  pnh.param("sample_timeout_s", cfg.session.sample_timeout_s, 0.1);
  pnh.param("nominal_rate_hz", cfg.session.nominal_rate_hz, 250.0);
  pnh.param("zero_sample_count", cfg.session.zero_sample_count, 100);
  pnh.param("filter_cutoff_hz", cfg.session.filter_cutoff_hz, 0.0);
  pnh.param("bias_limit_n", cfg.session.bias_limit_n, 120.0);
  pnh.param("zero_timeout_ms", zero_timeout_ms, 3000);
  pnh.param<std::string>("frame_id", frame_id, "sri_ft_link");

  ros::Publisher wrench_pub =
      nh.advertise<geometry_msgs::WrenchStamped>("/sri_ft/wrench_raw", 10);
  ros::Publisher status_pub =
      nh.advertise<soft_robot_msgs::SriStatus>("/sri_ft/status", 10);

  sri::SriDriverRuntime runtime(cfg);
  runtime.setSampleCallback([&](const sri::SriWrenchSample& s) {
    geometry_msgs::WrenchStamped msg;
    // Reception instant: this callback runs synchronously in the rx
    // thread right after the socket read (Plan 3 follow-up 3). Never a
    // zero stamp.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.wrench.force.x = s.w.fx;
    msg.wrench.force.y = s.w.fy;
    msg.wrench.force.z = s.w.fz;
    msg.wrench.torque.x = s.w.tx;
    msg.wrench.torque.y = s.w.ty;
    msg.wrench.torque.z = s.w.tz;
    wrench_pub.publish(msg);
  });

  ros::ServiceServer zero_srv = nh.advertiseService<std_srvs::Trigger::Request,
                                                    std_srvs::Trigger::Response>(
      "/sri_ft/zero",
      [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        res.success = runtime.requestZero(zero_timeout_ms);
        res.message = res.success
                          ? "tare captured"
                          : "tare failed: no stream or bias above limit";
        return true;
      });
  ros::ServiceServer filter_srv =
      nh.advertiseService<soft_robot_msgs::SetFilter::Request,
                          soft_robot_msgs::SetFilter::Response>(
          "/sri_ft/set_filter",
          [&](soft_robot_msgs::SetFilter::Request& req,
              soft_robot_msgs::SetFilter::Response& res) {
            res.success = runtime.setFilterCutoff(req.cutoff_hz);
            res.message =
                res.success ? "filter updated" : "cutoff_hz must be >= 0";
            return true;
          });

  ros::Timer status_timer =
      nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&) {
        const sri::SriDriverStatus st = runtime.status();
        soft_robot_msgs::SriStatus msg;
        msg.header.stamp = ros::Time::now();
        msg.connected = st.connected;
        msg.streaming = st.session.streaming;
        msg.reconnects = st.reconnects;
        msg.samples = st.session.samples;
        msg.bad_frames = st.session.bad_frames;
        msg.package_gaps = st.session.package_gaps;
        msg.zero_rejects = st.session.zero_rejects;
        msg.last_sample_age = st.session.last_sample_age_s;
        msg.zero_active = st.session.zero_active;
        msg.filter_cutoff_hz = st.session.filter_cutoff_hz;
        status_pub.publish(msg);
      });

  if (!runtime.start()) {
    ROS_ERROR("sri_ft_driver: runtime failed to start");
    return 1;
  }
  ROS_INFO("sri_ft_driver: connecting to %s:%u", cfg.sensor_ip.c_str(),
           cfg.sensor_port);
  ros::spin();
  runtime.stop();
  return 0;
}
```

`ros_ws/src/sri_force_torque_driver/config/sri_ft.yaml`:

```yaml
# SRI force/torque sensor driver parameters (spec section 14, sri_ft.yaml).
# Loaded into the sri_ft_driver private namespace by the Plan 5 bringup.
sri_ft_driver:
  sensor_ip: 192.168.1.1        # legacy Parameter.xml FTIP
  sensor_port: 4008             # M8128-style box TCP port (decision 1)
  frame_id: sri_ft_link
  nominal_rate_hz: 250.0        # box stream rate; >= 1 sample per RSI cycle
  sample_timeout_s: 0.1         # streaming -> stalled threshold for /sri_ft/status
  zero_sample_count: 100        # samples averaged per /sri_ft/zero call
  zero_timeout_ms: 3000         # bounded wait for a tare capture
  filter_cutoff_hz: 0.0         # 0 = raw passthrough; controller owns filtering (spec 8)
  bias_limit_n: 120.0           # legacy FTBia: reject taring under load (decision 4)
  connect_timeout_ms: 1000
  receive_timeout_ms: 50
  reconnect_backoff_s: 0.5
```

- [ ] **Step 5: CMake 增量**

`sri_ft_protocol` 源列表加 `src/sri_driver_runtime.cpp`;节点与安装规则:

```cmake
add_executable(sri_driver_node src/sri_driver_node.cpp)
add_dependencies(sri_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sri_driver_node sri_ft_protocol ${catkin_LIBRARIES})

install(TARGETS sri_mock_server sri_driver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

测试段追加:

```cmake
  catkin_add_gtest(test_sri_driver_runtime test/test_sri_driver_runtime.cpp)
  target_link_libraries(test_sri_driver_runtime sri_ft_protocol sri_ft_mock
                        ${catkin_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 6: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/sri_force_torque_driver/test_sri_driver_runtime
```

预期:`[  PASSED  ] 7 tests.`

- [ ] **Step 7: 手动冒烟(需要 roscore,非验收门槛)**

```bash
# terminal 1
roscore
# terminal 2: mock on a fixed loopback port
/home/ljj/kuka_iiqka_ros/ros_ws/devel/lib/sri_force_torque_driver/sri_mock_server \
  --rate 250 --fz 10 --sine-amp 5 --sine-hz 0.5
# note the printed port, then in terminal 3:
rosrun sri_force_torque_driver sri_driver_node \
  _sensor_ip:=127.0.0.1 _sensor_port:=<printed port> _nominal_rate_hz:=250
# terminal 4: checks
rostopic hz /sri_ft/wrench_raw          # ~250 Hz
rostopic echo -n 3 --offset /sri_ft/wrench_raw   # stamp-vs-arrival sanity
rosservice call /sri_ft/zero            # success: true (sine mean ~10 N < 120 N limit)
rostopic echo -n 1 /sri_ft/status
```

预期:`hz` ≈ 250;`--offset` 显示的时间差为毫秒级;zero 后 `wrench_raw` 的 fz 围绕 0 波动;status 中 `streaming: True`。

- [ ] **Step 8: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/sri_force_torque_driver && \
git commit -m "feat(sri): driver runtime with reconnect + ROS node shell + sri_ft.yaml (Plan 4 Task 6)"
```

---

### Task 7: `kuka_eki_bridge` 包骨架 + `eki_frame` codec + `EkiStreamSplitter`

**Files:**
- Create: `ros_ws/src/kuka_eki_bridge/package.xml`
- Create: `ros_ws/src/kuka_eki_bridge/CMakeLists.txt`
- Create: `ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_frame.h`
- Create: `ros_ws/src/kuka_eki_bridge/src/eki_frame.cpp`
- Create: `ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_stream_splitter.h`
- Create: `ros_ws/src/kuka_eki_bridge/src/eki_stream_splitter.cpp`
- Test: `ros_ws/src/kuka_eki_bridge/test/test_eki_frame.cpp`(codec + splitter 同一二进制)

**Interfaces:**
- Consumes: tinyxml2(同 Plan 2 `rsi_frame` 先例;管理通道非 RT,解析分配无实时性顾虑)。
- Produces(Task 8/9/10 消费;Plan 5 KRL/EkiConfig.xml 模板的权威 schema):
  - 动作/错误码常量:`enum class EkiAction : int { QUERY_STATE=0, START_RSI=1, STOP_RSI=2, SET_MODE=3, RESET_FAULT=4, GET_TOOL=5, SET_TOOL_BASE=6 }`;`kErrOk=0, kErrNotReady=1, kErrFaulted=2`。
  - `struct Frame6 { double x, y, z, a, b, c; }`(mm/deg)。
  - `struct EkiCommand { std::uint32_t seq; EkiAction action; int value; Frame6 tool; Frame6 base; }`
  - `struct EkiStateFrame { ack_seq, ack_ok, ack_code, ready, rsi_active, fault, mode, tool, valid }`
  - `std::size_t serializeCommand(const EkiCommand&, char* buf, std::size_t n)`(0 = 缓冲不足);`bool parseState(const char* data, std::size_t len, EkiStateFrame& out)`。
  - `class EkiStreamSplitter`:`feed(data, len, sink)`,`sink: std::function<void(const char*, std::size_t)>` 每完整 XML 文档回调一次;`reset()`;溢出(单文档 > 8 KiB)丢弃至下一 `<RobotState`。
- XML schema(待确认 6 的权威定义,双向均为单行文档):
  - ROS→KRC:`<RobotCommand><Cmd Seq="1" Action="1" Value="0"/><Tool X="0" Y="0" Z="0" A="0" B="0" C="0"/><Base .../></RobotCommand>`
  - KRC→ROS:`<RobotState><Ack Seq="1" Ok="1" Code="0"/><Prog Ready="1" RsiActive="0" Fault="0" Mode="2"/><Tool X="10.5" .../></RobotState>`
  - 切分依据:EthernetKRL 每次 `EKI_Send` 发送一个完整 XML 文档,TCP 无消息边界 ⇒ splitter 扫描根元素闭合 `</RobotState>`(或自闭合根);splitter 只认 `<RobotState` 起始,其他字节按噪声丢弃。

- [ ] **Step 1: 写包清单与构建文件**

`ros_ws/src/kuka_eki_bridge/package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>kuka_eki_bridge</name>
  <version>0.1.0</version>
  <description>
    EKI (EthernetKRL 6.1) management-channel bridge for the KUKA KR C5 /
    iiQKA.OS2 (spec sections 5.2, 6.2, 6.4): XML command/state codec,
    TCP stream splitting, seq/ack request-response session, the
    /kuka/eki/* service surface, and a KRC-side mock for offline
    closed-loop tests. Never used for per-cycle control.
  </description>
  <maintainer email="dev@example.com">ljj</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>std_srvs</depend>
  <depend>diagnostic_msgs</depend>
  <depend>soft_robot_msgs</depend>
  <build_depend>libtinyxml2-dev</build_depend>
  <exec_depend>libtinyxml2</exec_depend>
</package>
```

`ros_ws/src/kuka_eki_bridge/CMakeLists.txt`(初版,后续 Task 增量扩展):

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(kuka_eki_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  diagnostic_msgs
  soft_robot_msgs
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kuka_eki_protocol
  CATKIN_DEPENDS roscpp std_srvs diagnostic_msgs soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS} ${TINYXML2_INCLUDE_DIRS})

# Protocol / session library: no roscpp runtime dependency.
add_library(kuka_eki_protocol
  src/eki_frame.cpp
  src/eki_stream_splitter.cpp
)
target_link_libraries(kuka_eki_protocol ${TINYXML2_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_eki_frame test/test_eki_frame.cpp)
  target_link_libraries(test_eki_frame kuka_eki_protocol
                        ${GTEST_MAIN_LIBRARIES})
endif()
```

- [ ] **Step 2: 写失败测试**

`ros_ws/src/kuka_eki_bridge/test/test_eki_frame.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "kuka_eki_bridge/eki_frame.h"
#include "kuka_eki_bridge/eki_stream_splitter.h"

using kuka_eki::EkiAction;
using kuka_eki::EkiCommand;
using kuka_eki::EkiStateFrame;
using kuka_eki::EkiStreamSplitter;

namespace {

const char kStateXml[] =
    "<RobotState>"
    "<Ack Seq=\"7\" Ok=\"1\" Code=\"0\"/>"
    "<Prog Ready=\"1\" RsiActive=\"0\" Fault=\"0\" Mode=\"2\"/>"
    "<Tool X=\"10.5\" Y=\"0\" Z=\"235.0\" A=\"0\" B=\"90.0\" C=\"0\"/>"
    "</RobotState>";

EkiCommand makeCommand() {
  EkiCommand c;
  c.seq = 42;
  c.action = EkiAction::SET_TOOL_BASE;
  c.value = 0;
  c.tool = {10.5, 0.0, 235.0, 0.0, 90.0, 0.0};
  c.base = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return c;
}

}  // namespace

TEST(EkiFrame, SerializeCommandRoundTripsThroughParser) {
  // The codec must be self-consistent: serialize, then re-parse with
  // tinyxml2 via the state parser's sibling logic. We assert on the raw
  // XML text for the schema-critical parts (Plan 5 templates depend on
  // these exact element/attribute names).
  char buf[1024];
  const std::size_t n = kuka_eki::serializeCommand(makeCommand(), buf,
                                                   sizeof(buf));
  ASSERT_GT(n, 0u);
  const std::string xml(buf, n);
  EXPECT_NE(xml.find("<RobotCommand>"), std::string::npos);
  EXPECT_NE(xml.find("<Cmd Seq=\"42\" Action=\"6\" Value=\"0\"/>"),
            std::string::npos);
  EXPECT_NE(xml.find("Z=\"235.000000\""), std::string::npos);
  EXPECT_NE(xml.find("<Base"), std::string::npos);
  EXPECT_NE(xml.find("</RobotCommand>"), std::string::npos);
}

TEST(EkiFrame, SerializeFailsOnTinyBuffer) {
  char buf[8];
  EXPECT_EQ(kuka_eki::serializeCommand(makeCommand(), buf, sizeof(buf)), 0u);
}

TEST(EkiFrame, ParseStateExtractsAllFields) {
  EkiStateFrame s;
  ASSERT_TRUE(kuka_eki::parseState(kStateXml, std::strlen(kStateXml), s));
  EXPECT_TRUE(s.valid);
  EXPECT_EQ(s.ack_seq, 7u);
  EXPECT_TRUE(s.ack_ok);
  EXPECT_EQ(s.ack_code, 0);
  EXPECT_TRUE(s.ready);
  EXPECT_FALSE(s.rsi_active);
  EXPECT_FALSE(s.fault);
  EXPECT_EQ(s.mode, 2);
  EXPECT_NEAR(s.tool.x, 10.5, 1e-9);
  EXPECT_NEAR(s.tool.z, 235.0, 1e-9);
  EXPECT_NEAR(s.tool.b, 90.0, 1e-9);
}

TEST(EkiFrame, ParseHeartbeatWithAckSeqZero) {
  const char xml[] =
      "<RobotState>"
      "<Ack Seq=\"0\" Ok=\"1\" Code=\"0\"/>"
      "<Prog Ready=\"1\" RsiActive=\"1\" Fault=\"0\" Mode=\"1\"/>"
      "<Tool X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "</RobotState>";
  EkiStateFrame s;
  ASSERT_TRUE(kuka_eki::parseState(xml, std::strlen(xml), s));
  EXPECT_EQ(s.ack_seq, 0u);  // unsolicited heartbeat marker (decision 6)
  EXPECT_TRUE(s.rsi_active);
}

TEST(EkiFrame, ParseRejectsMalformedAndWrongRoot) {
  EkiStateFrame s;
  const char broken[] = "<RobotState><Ack Seq=\"1\"";
  EXPECT_FALSE(kuka_eki::parseState(broken, std::strlen(broken), s));
  EXPECT_FALSE(s.valid);
  const char wrong[] = "<Rob><IPOC>1</IPOC></Rob>";
  EXPECT_FALSE(kuka_eki::parseState(wrong, std::strlen(wrong), s));
}

TEST(EkiFrame, ParseRejectsMissingMandatoryElements) {
  EkiStateFrame s;
  const char no_prog[] =
      "<RobotState><Ack Seq=\"1\" Ok=\"1\" Code=\"0\"/></RobotState>";
  EXPECT_FALSE(kuka_eki::parseState(no_prog, std::strlen(no_prog), s));
  const char no_ack[] =
      "<RobotState>"
      "<Prog Ready=\"1\" RsiActive=\"0\" Fault=\"0\" Mode=\"0\"/>"
      "<Tool X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "</RobotState>";
  EXPECT_FALSE(kuka_eki::parseState(no_ack, std::strlen(no_ack), s));
}

TEST(EkiSplitter, TwoDocumentsInOneChunk) {
  EkiStreamSplitter splitter;
  std::vector<std::string> docs;
  const std::string two = std::string(kStateXml) + kStateXml;
  splitter.feed(two.data(), two.size(),
                [&](const char* d, std::size_t n) { docs.emplace_back(d, n); });
  ASSERT_EQ(docs.size(), 2u);
  EXPECT_EQ(docs[0], kStateXml);
  EXPECT_EQ(docs[1], kStateXml);
}

TEST(EkiSplitter, DocumentSplitByteByByte) {
  EkiStreamSplitter splitter;
  std::vector<std::string> docs;
  for (std::size_t i = 0; i < std::strlen(kStateXml); ++i)
    splitter.feed(kStateXml + i, 1, [&](const char* d, std::size_t n) {
      docs.emplace_back(d, n);
    });
  ASSERT_EQ(docs.size(), 1u);
  EXPECT_EQ(docs[0], kStateXml);
}

TEST(EkiSplitter, NoiseBetweenDocumentsIsDropped) {
  EkiStreamSplitter splitter;
  std::vector<std::string> docs;
  const std::string noisy =
      std::string("\r\n junk ") + kStateXml + "garbage" + kStateXml;
  splitter.feed(noisy.data(), noisy.size(),
                [&](const char* d, std::size_t n) { docs.emplace_back(d, n); });
  ASSERT_EQ(docs.size(), 2u);
  EXPECT_EQ(docs[1], kStateXml);
}

TEST(EkiSplitter, OversizedDocumentIsDiscarded) {
  EkiStreamSplitter splitter;
  std::vector<std::string> docs;
  std::string huge = "<RobotState>";
  huge.append(9000, 'x');  // exceeds the 8 KiB document cap
  huge += "</RobotState>";
  splitter.feed(huge.data(), huge.size(),
                [&](const char* d, std::size_t n) { docs.emplace_back(d, n); });
  EXPECT_TRUE(docs.empty());
  // The splitter must recover on the next well-formed document.
  splitter.feed(kStateXml, std::strlen(kStateXml),
                [&](const char* d, std::size_t n) { docs.emplace_back(d, n); });
  ASSERT_EQ(docs.size(), 1u);
}
```

- [ ] **Step 3: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `eki_frame.h`。

- [ ] **Step 4: 写实现**

`ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_frame.h`:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>

namespace kuka_eki {

// Application-layer schema of the EKI management channel (decision 6).
// These action codes, element names, and attribute names are the single
// source of truth for the Plan 5 KRL program and EkiConfig.xml templates.
enum class EkiAction : int {
  QUERY_STATE = 0,
  START_RSI = 1,
  STOP_RSI = 2,
  SET_MODE = 3,
  RESET_FAULT = 4,
  GET_TOOL = 5,
  SET_TOOL_BASE = 6,
};

// Ack.Code values reported by the KRL side.
constexpr int kErrOk = 0;
constexpr int kErrNotReady = 1;
constexpr int kErrFaulted = 2;

// Cartesian frame in mm / deg (KUKA A/B/C = Z-Y-X Euler).
struct Frame6 {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};
};

// One ROS -> KRC command document:
// <RobotCommand><Cmd Seq Action Value/><Tool .../><Base .../></RobotCommand>
// Every command carries all elements (unused ones zeroed) to match the
// fixed-structure parsing habits of EthernetKRL configurations.
struct EkiCommand {
  std::uint32_t seq{0};  // starts at 1; 0 is reserved for heartbeats
  EkiAction action{EkiAction::QUERY_STATE};
  int value{0};          // SET_MODE payload; 0 otherwise
  Frame6 tool;
  Frame6 base;
};

// One KRC -> ROS state document:
// <RobotState><Ack Seq Ok Code/><Prog Ready RsiActive Fault Mode/>
//   <Tool X Y Z A B C/></RobotState>
// Ack.Seq == 0 marks an unsolicited heartbeat push.
struct EkiStateFrame {
  std::uint32_t ack_seq{0};
  bool ack_ok{false};
  int ack_code{0};
  bool ready{false};
  bool rsi_active{false};
  bool fault{false};
  int mode{0};
  Frame6 tool;
  bool valid{false};
};

// Serializes a command document into buf (NUL-terminated). Returns the
// payload length (excluding NUL), or 0 if the buffer is too small.
std::size_t serializeCommand(const EkiCommand& cmd, char* buf,
                             std::size_t buf_size);

// Parses one complete RobotState document. Returns false (out.valid =
// false) on malformed XML, wrong root, or missing Ack/Prog/Tool elements
// or any of their mandatory attributes.
bool parseState(const char* data, std::size_t len, EkiStateFrame& out);

}  // namespace kuka_eki
```

`ros_ws/src/kuka_eki_bridge/src/eki_frame.cpp`:

```cpp
#include "kuka_eki_bridge/eki_frame.h"

#include <tinyxml2.h>

#include <cstdio>

namespace kuka_eki {

std::size_t serializeCommand(const EkiCommand& cmd, char* buf,
                             std::size_t buf_size) {
  const int n = std::snprintf(
      buf, buf_size,
      "<RobotCommand>"
      "<Cmd Seq=\"%u\" Action=\"%d\" Value=\"%d\"/>"
      "<Tool X=\"%f\" Y=\"%f\" Z=\"%f\" A=\"%f\" B=\"%f\" C=\"%f\"/>"
      "<Base X=\"%f\" Y=\"%f\" Z=\"%f\" A=\"%f\" B=\"%f\" C=\"%f\"/>"
      "</RobotCommand>",
      cmd.seq, static_cast<int>(cmd.action), cmd.value, cmd.tool.x,
      cmd.tool.y, cmd.tool.z, cmd.tool.a, cmd.tool.b, cmd.tool.c, cmd.base.x,
      cmd.base.y, cmd.base.z, cmd.base.a, cmd.base.b, cmd.base.c);
  if (n <= 0 || static_cast<std::size_t>(n) >= buf_size) return 0;
  return static_cast<std::size_t>(n);
}

namespace {

bool readFrame6(const tinyxml2::XMLElement* e, Frame6& out) {
  return e->QueryDoubleAttribute("X", &out.x) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("Y", &out.y) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("Z", &out.z) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("A", &out.a) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("B", &out.b) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("C", &out.c) == tinyxml2::XML_SUCCESS;
}

}  // namespace

bool parseState(const char* data, std::size_t len, EkiStateFrame& out) {
  out = EkiStateFrame{};
  tinyxml2::XMLDocument doc;
  if (doc.Parse(data, len) != tinyxml2::XML_SUCCESS) return false;
  const tinyxml2::XMLElement* root = doc.FirstChildElement("RobotState");
  if (root == nullptr) return false;

  const tinyxml2::XMLElement* ack = root->FirstChildElement("Ack");
  const tinyxml2::XMLElement* prog = root->FirstChildElement("Prog");
  const tinyxml2::XMLElement* tool = root->FirstChildElement("Tool");
  if (ack == nullptr || prog == nullptr || tool == nullptr) return false;

  unsigned seq = 0;
  int ok = 0;
  if (ack->QueryUnsignedAttribute("Seq", &seq) != tinyxml2::XML_SUCCESS ||
      ack->QueryIntAttribute("Ok", &ok) != tinyxml2::XML_SUCCESS ||
      ack->QueryIntAttribute("Code", &out.ack_code) != tinyxml2::XML_SUCCESS)
    return false;
  out.ack_seq = seq;
  out.ack_ok = ok != 0;

  int ready = 0, rsi = 0, fault = 0;
  if (prog->QueryIntAttribute("Ready", &ready) != tinyxml2::XML_SUCCESS ||
      prog->QueryIntAttribute("RsiActive", &rsi) != tinyxml2::XML_SUCCESS ||
      prog->QueryIntAttribute("Fault", &fault) != tinyxml2::XML_SUCCESS ||
      prog->QueryIntAttribute("Mode", &out.mode) != tinyxml2::XML_SUCCESS)
    return false;
  out.ready = ready != 0;
  out.rsi_active = rsi != 0;
  out.fault = fault != 0;

  if (!readFrame6(tool, out.tool)) return false;
  out.valid = true;
  return true;
}

}  // namespace kuka_eki
```

`ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_stream_splitter.h`:

```cpp
#pragma once

#include <cstddef>
#include <functional>
#include <string>

namespace kuka_eki {

// Reassembles complete <RobotState>...</RobotState> documents from the
// TCP byte stream (EKI_Send has no message framing on the wire). Bytes
// before a document start are dropped as noise; a document larger than
// kMaxDoc is discarded and scanning resumes at the next start tag.
// Management channel, not realtime: std::string buffering is fine.
class EkiStreamSplitter {
 public:
  using Sink = std::function<void(const char* data, std::size_t len)>;

  void feed(const char* data, std::size_t len, const Sink& sink);
  void reset() { buffer_.clear(); }

 private:
  static constexpr std::size_t kMaxDoc = 8192;
  static constexpr const char* kStartTag = "<RobotState";
  static constexpr const char* kEndTag = "</RobotState>";

  std::string buffer_;
};

}  // namespace kuka_eki
```

`ros_ws/src/kuka_eki_bridge/src/eki_stream_splitter.cpp`:

```cpp
#include "kuka_eki_bridge/eki_stream_splitter.h"

#include <cstring>

namespace kuka_eki {

// C++14: odr-used constexpr static members need namespace-scope definitions.
constexpr std::size_t EkiStreamSplitter::kMaxDoc;
constexpr const char* EkiStreamSplitter::kStartTag;
constexpr const char* EkiStreamSplitter::kEndTag;

void EkiStreamSplitter::feed(const char* data, std::size_t len,
                             const Sink& sink) {
  buffer_.append(data, len);
  for (;;) {
    const std::size_t start = buffer_.find(kStartTag);
    if (start == std::string::npos) {
      // Keep a tail shorter than the start tag: it may be a split prefix.
      const std::size_t tag_len = std::strlen(kStartTag);
      if (buffer_.size() >= tag_len)
        buffer_.erase(0, buffer_.size() - (tag_len - 1));
      return;
    }
    if (start > 0) buffer_.erase(0, start);  // drop leading noise
    const std::size_t end = buffer_.find(kEndTag);
    if (end == std::string::npos) {
      if (buffer_.size() > kMaxDoc) {
        // Oversized document: discard the start tag and rescan.
        buffer_.erase(0, std::strlen(kStartTag));
        continue;
      }
      return;  // incomplete document, wait for more bytes
    }
    const std::size_t doc_len = end + std::strlen(kEndTag);
    if (doc_len <= kMaxDoc) sink(buffer_.data(), doc_len);
    buffer_.erase(0, doc_len);
  }
}

}  // namespace kuka_eki
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_eki_bridge/test_eki_frame
```

预期:`[  PASSED  ] 10 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_eki_bridge && \
git commit -m "feat(eki): package skeleton + RobotCommand/RobotState codec + TCP stream splitter (Plan 4 Task 7)"
```

---

### Task 8: EKI 侧 `TcpClientTransport`(同构副本)+ `EkiSessionCore`(Seq/Ack 会话)

**Files:**
- Create: `ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/tcp_client_transport.h`
- Create: `ros_ws/src/kuka_eki_bridge/src/tcp_client_transport.cpp`
- Create: `ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_session_core.h`
- Create: `ros_ws/src/kuka_eki_bridge/src/eki_session_core.cpp`
- Modify: `ros_ws/src/kuka_eki_bridge/CMakeLists.txt`
- Test: `ros_ws/src/kuka_eki_bridge/test/test_tcp_client_transport.cpp`
- Test: `ros_ws/src/kuka_eki_bridge/test/test_eki_session_core.cpp`

**Interfaces:**
- `kuka_eki::TcpClientTransport`:Task 3 `sri::TcpClientTransport` 的**逐字同构副本**,仅三处不同:namespace `sri` → `kuka_eki`、include 路径、头文件注释首行加 `// Structural twin of sri::TcpClientTransport (Plan 4 decision 8); keep in sync by hand.`(实现文件同样只改 include 与 namespace)。传输测试取 Task 3 用例子集(3 条:交换、超时、对端关闭),`TestListener` 辅助类照搬。
- `EkiSessionCore`(纯逻辑,Task 10 消费):
  - `struct EkiSessionConfig { double response_timeout_s{2.0}; double state_timeout_s{1.0}; }`
  - `enum class CommandOutcome { NONE, ACCEPTED, REJECTED, TIMEOUT }`
  - `struct EkiSessionSnapshot { bool state_fresh; double state_age_s; EkiStateFrame last_state; std::uint64_t states, bad_frames, timeouts; }`
  - `class EkiSessionCore`:`configure` / `reset`(断线时调,飞行中命令立即 TIMEOUT 终态)/ `beginCommand(seq, now_s)`(已有飞行中命令返回 false)/ `commandPending()` / `onState(frame, now_s)` → `CommandOutcome`(匹配 `ack_seq` 且非 0 时依 `ack_ok` 给 ACCEPTED/REJECTED 并清飞行中;心跳/不匹配返回 NONE 只刷新新鲜度)/ `onBadFrame()` / `tick(now_s)` → `CommandOutcome`(飞行中命令超时则 TIMEOUT)/ `snapshot(now_s)`。
- 语义:任何 `EkiStateFrame`(含心跳)刷新 `state_fresh`;`last_state` 为最近一帧(tool 查询结果由 runtime 从 ACCEPTED 时刻的 `last_state.tool` 取——同一帧既是 ack 也携带状态)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/kuka_eki_bridge/test/test_tcp_client_transport.cpp`(Task 3 测试文件的裁剪副本:同一 `TestListener` 辅助类,保留三条核心用例,namespace/include/测试套名机械替换):

```cpp
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <string>

#include "kuka_eki_bridge/tcp_client_transport.h"

using kuka_eki::TcpClientTransport;

namespace {

// Minimal loopback listener used to play the KRC side.
class TestListener {
 public:
  bool start() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) return false;
    int on = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ::inet_addr("127.0.0.1");
    addr.sin_port = 0;  // kernel-chosen port
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
      return false;
    socklen_t len = sizeof(addr);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    return ::listen(fd_, 1) == 0;
  }
  bool acceptClient(int timeout_ms) {
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) return false;
    client_ = ::accept(fd_, nullptr, nullptr);
    return client_ >= 0;
  }
  int readClient(char* buf, std::size_t n, int timeout_ms) {
    pollfd p{client_, POLLIN, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) return 0;
    return static_cast<int>(::recv(client_, buf, n, 0));
  }
  bool writeClient(const char* data, std::size_t n) {
    return ::send(client_, data, n, 0) == static_cast<ssize_t>(n);
  }
  void closeClient() {
    if (client_ >= 0) ::close(client_);
    client_ = -1;
  }
  void stop() {
    closeClient();
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }
  ~TestListener() { stop(); }
  std::uint16_t port() const { return port_; }

 private:
  int fd_{-1};
  int client_{-1};
  std::uint16_t port_{0};
};

}  // namespace

TEST(EkiTcpTransport, ConnectAndExchange) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  ASSERT_TRUE(t.send("ping", 4, 200));
  char buf[16] = {0};
  ASSERT_EQ(l.readClient(buf, sizeof(buf), 500), 4);
  EXPECT_EQ(std::string(buf, 4), "ping");
  ASSERT_TRUE(l.writeClient("pong!", 5));
  ASSERT_EQ(t.receive(buf, sizeof(buf), 500), 5);
  EXPECT_EQ(std::string(buf, 5), "pong!");
}

TEST(EkiTcpTransport, ReceiveTimeoutReturnsZero) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  char buf[8];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 50), 0);
  EXPECT_TRUE(t.connected());
}

TEST(EkiTcpTransport, PeerCloseReturnsMinusOne) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  l.closeClient();
  char buf[8];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 500), -1);
  EXPECT_FALSE(t.connected());
}
```

`ros_ws/src/kuka_eki_bridge/test/test_eki_session_core.cpp`:

```cpp
#include <gtest/gtest.h>

#include "kuka_eki_bridge/eki_session_core.h"

using kuka_eki::CommandOutcome;
using kuka_eki::EkiSessionConfig;
using kuka_eki::EkiSessionCore;
using kuka_eki::EkiStateFrame;

namespace {

constexpr double kNow = 200.0;  // arbitrary monotonic origin [s]

EkiSessionConfig config() {
  EkiSessionConfig c;
  c.response_timeout_s = 0.2;
  c.state_timeout_s = 0.1;
  return c;
}

EkiStateFrame state(std::uint32_t ack_seq, bool ok = true, int code = 0) {
  EkiStateFrame s;
  s.ack_seq = ack_seq;
  s.ack_ok = ok;
  s.ack_code = code;
  s.ready = true;
  s.mode = 1;
  s.tool.z = 235.0;
  s.valid = true;
  return s;
}

}  // namespace

TEST(EkiSession, HeartbeatRefreshesFreshnessOnly) {
  EkiSessionCore core;
  core.configure(config());
  EXPECT_FALSE(core.snapshot(kNow).state_fresh);
  EXPECT_EQ(core.onState(state(0), kNow), CommandOutcome::NONE);
  EXPECT_TRUE(core.snapshot(kNow + 0.05).state_fresh);
  EXPECT_NEAR(core.snapshot(kNow + 0.05).state_age_s, 0.05, 1e-9);
  EXPECT_FALSE(core.snapshot(kNow + 0.2).state_fresh);  // past 0.1 s
}

TEST(EkiSession, MatchingAckAcceptsCommand) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(5, kNow));
  EXPECT_TRUE(core.commandPending());
  EXPECT_EQ(core.onState(state(5), kNow + 0.01), CommandOutcome::ACCEPTED);
  EXPECT_FALSE(core.commandPending());
  // The ack frame doubles as a state frame: tool data is available.
  EXPECT_NEAR(core.snapshot(kNow + 0.01).last_state.tool.z, 235.0, 1e-9);
}

TEST(EkiSession, NackRejectsCommand) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(6, kNow));
  EXPECT_EQ(core.onState(state(6, false, kuka_eki::kErrNotReady), kNow),
            CommandOutcome::REJECTED);
  EXPECT_EQ(core.snapshot(kNow).last_state.ack_code, kuka_eki::kErrNotReady);
}

TEST(EkiSession, MismatchedAckIsIgnoredByPendingCommand) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(7, kNow));
  EXPECT_EQ(core.onState(state(3), kNow), CommandOutcome::NONE);  // stale ack
  EXPECT_TRUE(core.commandPending());
}

TEST(EkiSession, SecondBeginWhilePendingRefused) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(1, kNow));
  EXPECT_FALSE(core.beginCommand(2, kNow));
}

TEST(EkiSession, PendingCommandTimesOutViaTick) {
  EkiSessionCore core;
  core.configure(config());  // response_timeout_s = 0.2
  ASSERT_TRUE(core.beginCommand(8, kNow));
  EXPECT_EQ(core.tick(kNow + 0.1), CommandOutcome::NONE);
  EXPECT_EQ(core.tick(kNow + 0.25), CommandOutcome::TIMEOUT);
  EXPECT_FALSE(core.commandPending());
  EXPECT_EQ(core.snapshot(kNow + 0.25).timeouts, 1u);
}

TEST(EkiSession, ResetFailsPendingCommandAsTimeout) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(9, kNow));
  EXPECT_EQ(core.reset(), CommandOutcome::TIMEOUT);  // disconnect semantics
  EXPECT_FALSE(core.commandPending());
  EXPECT_FALSE(core.snapshot(kNow).state_fresh);
}

TEST(EkiSession, CountersAccumulate) {
  EkiSessionCore core;
  core.configure(config());
  core.onState(state(0), kNow);
  core.onState(state(0), kNow + 0.01);
  core.onBadFrame();
  const auto snap = core.snapshot(kNow + 0.01);
  EXPECT_EQ(snap.states, 2u);
  EXPECT_EQ(snap.bad_frames, 1u);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `tcp_client_transport.h`(kuka_eki_bridge)与 `eki_session_core.h`。

- [ ] **Step 3: 写实现**

`ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/tcp_client_transport.h`:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace kuka_eki {

// Structural twin of sri::TcpClientTransport (Plan 4 decision 8); keep in
// sync by hand. Minimal bounded-wait TCP client: poll()-based waits, no
// allocation after connect, not copyable. Any receive error / peer close /
// send failure closes the socket so connected() doubles as the reconnect
// trigger for the owning runtime. Not thread-safe: one owning thread.
class TcpClientTransport {
 public:
  TcpClientTransport() = default;
  ~TcpClientTransport() { close(); }
  TcpClientTransport(const TcpClientTransport&) = delete;
  TcpClientTransport& operator=(const TcpClientTransport&) = delete;

  // Non-blocking connect with a bounded wait. false on refusal/timeout.
  bool connect(const std::string& ip, std::uint16_t port, int timeout_ms);
  bool connected() const { return fd_ >= 0; }

  // Waits up to timeout_ms. Returns byte count, 0 on timeout, -1 when the
  // peer closed or the socket errored (the transport closes itself).
  int receive(char* buf, std::size_t buf_size, int timeout_ms);

  // Sends the whole buffer within timeout_ms overall. false (and close)
  // on error or timeout.
  bool send(const char* data, std::size_t len, int timeout_ms);

  void close();

 private:
  int fd_{-1};
};

}  // namespace kuka_eki
```

`ros_ws/src/kuka_eki_bridge/src/tcp_client_transport.cpp`:Task 3 的 `src/tcp_client_transport.cpp` 全文,仅两处机械替换——首行 include 改为 `#include "kuka_eki_bridge/tcp_client_transport.h"`,`namespace sri` 开闭两行改为 `namespace kuka_eki`(含尾注释)。其余与 Task 3 逐字一致(review 时 diff 校验:除这三行外零差异)。

`ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_session_core.h`:

```cpp
#pragma once

#include <cstdint>

#include "kuka_eki_bridge/eki_frame.h"

namespace kuka_eki {

struct EkiSessionConfig {
  double response_timeout_s{2.0};  // ack deadline for one command
  double state_timeout_s{1.0};     // heartbeat freshness threshold
};

enum class CommandOutcome { NONE, ACCEPTED, REJECTED, TIMEOUT };

struct EkiSessionSnapshot {
  bool state_fresh{false};
  double state_age_s{-1.0};  // -1 until the first state frame
  EkiStateFrame last_state;
  std::uint64_t states{0};
  std::uint64_t bad_frames{0};
  std::uint64_t timeouts{0};  // command response timeouts (incl. resets)
};

// Request/response bookkeeping of the management channel (decision 7):
// one in-flight command at a time, correlated by Ack.Seq; every state
// frame (ack or heartbeat) refreshes freshness and last_state. Pure
// logic, single-threaded (the runtime's io thread serializes access).
class EkiSessionCore {
 public:
  void configure(const EkiSessionConfig& cfg) { cfg_ = cfg; }

  // Disconnect hook: fails a pending command with TIMEOUT (returned so
  // the runtime can resolve its waiter) and clears freshness.
  CommandOutcome reset();

  // Registers command `seq` as in-flight. false if one is pending.
  bool beginCommand(std::uint32_t seq, double now_s);
  bool commandPending() const { return pending_; }

  // Processes one parsed state frame. ACCEPTED/REJECTED when it acks the
  // pending command; NONE for heartbeats and stale acks.
  CommandOutcome onState(const EkiStateFrame& frame, double now_s);
  void onBadFrame() { ++bad_frames_; }

  // Periodic timeout check for the pending command.
  CommandOutcome tick(double now_s);

  EkiSessionSnapshot snapshot(double now_s) const;

 private:
  EkiSessionConfig cfg_;
  bool pending_{false};
  std::uint32_t pending_seq_{0};
  double pending_since_s_{0};
  EkiStateFrame last_state_;
  double last_state_s_{-1.0};
  std::uint64_t states_{0};
  std::uint64_t bad_frames_{0};
  std::uint64_t timeouts_{0};
};

}  // namespace kuka_eki
```

`ros_ws/src/kuka_eki_bridge/src/eki_session_core.cpp`:

```cpp
#include "kuka_eki_bridge/eki_session_core.h"

namespace kuka_eki {

CommandOutcome EkiSessionCore::reset() {
  const bool had_pending = pending_;
  pending_ = false;
  last_state_s_ = -1.0;
  last_state_ = EkiStateFrame{};
  if (had_pending) {
    ++timeouts_;
    return CommandOutcome::TIMEOUT;
  }
  return CommandOutcome::NONE;
}

bool EkiSessionCore::beginCommand(std::uint32_t seq, double now_s) {
  if (pending_) return false;
  pending_ = true;
  pending_seq_ = seq;
  pending_since_s_ = now_s;
  return true;
}

CommandOutcome EkiSessionCore::onState(const EkiStateFrame& frame,
                                       double now_s) {
  ++states_;
  last_state_ = frame;
  last_state_s_ = now_s;
  if (pending_ && frame.ack_seq == pending_seq_ && frame.ack_seq != 0) {
    pending_ = false;
    return frame.ack_ok ? CommandOutcome::ACCEPTED : CommandOutcome::REJECTED;
  }
  return CommandOutcome::NONE;
}

CommandOutcome EkiSessionCore::tick(double now_s) {
  if (pending_ && now_s - pending_since_s_ > cfg_.response_timeout_s) {
    pending_ = false;
    ++timeouts_;
    return CommandOutcome::TIMEOUT;
  }
  return CommandOutcome::NONE;
}

EkiSessionSnapshot EkiSessionCore::snapshot(double now_s) const {
  EkiSessionSnapshot s;
  s.last_state = last_state_;
  s.states = states_;
  s.bad_frames = bad_frames_;
  s.timeouts = timeouts_;
  if (last_state_s_ >= 0.0) {
    s.state_age_s = now_s - last_state_s_;
    s.state_fresh = s.state_age_s <= cfg_.state_timeout_s;
  }
  return s;
}

}  // namespace kuka_eki
```

- [ ] **Step 4: CMake 增量**

`kuka_eki_protocol` 源列表加 `src/tcp_client_transport.cpp`、`src/eki_session_core.cpp`;测试段追加:

```cmake
  catkin_add_gtest(test_tcp_client_transport test/test_tcp_client_transport.cpp)
  target_link_libraries(test_tcp_client_transport kuka_eki_protocol
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_eki_session_core test/test_eki_session_core.cpp)
  target_link_libraries(test_eki_session_core kuka_eki_protocol
                        ${GTEST_MAIN_LIBRARIES})
```

(与 sri 包的 `test_tcp_client_transport` 同名不冲突:catkin gtest 目标按包隔离,二进制分处 `devel/lib/<pkg>/`。)

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_eki_bridge/test_tcp_client_transport && \
  ./devel/lib/kuka_eki_bridge/test_eki_session_core
```

预期:transport `[  PASSED  ] 3 tests.`;session `[  PASSED  ] 8 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_eki_bridge && \
git commit -m "feat(eki): TCP transport twin + seq/ack session core (Plan 4 Task 8)"
```

---

### Task 9: `EkiMockServer`(KRC 模拟端)+ 独立可执行 + 闭环测试

**Files:**
- Create: `ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_mock_server.h`
- Create: `ros_ws/src/kuka_eki_bridge/src/eki_mock_server.cpp`
- Create: `ros_ws/src/kuka_eki_bridge/src/eki_mock_server_main.cpp`
- Modify: `ros_ws/src/kuka_eki_bridge/CMakeLists.txt`
- Test: `ros_ws/src/kuka_eki_bridge/test/test_eki_mock_server.cpp`

**Interfaces:**
- Consumes: Task 7 codec(mock 用 tinyxml2 解析收到的 `RobotCommand`、用 codec 的镜像格式回 `RobotState`)、POSIX socket。
- Produces(Task 10 与 Plan 5 消费):
  - `struct EkiMockConfig { double heartbeat_period_s{0.0}; }`(0 = 无自主心跳,gtest 全确定性;>0 = 周期推送,冒烟/Plan 5 用)。
  - `class EkiMockServer`(KRC 行为表,规格 §15.2):`start()`(bind 127.0.0.1:0)/ `stop()` / `port()` / `waitForClient` / `dropClient` / `pushHeartbeat()`(手动推一帧)/ `setReady(bool)` / `injectFault()` / `setTool(x..c)` / `setRespondToNext(bool)`(false ⇒ 吞掉下一条命令不回 ack,测超时)/ `sendMalformed()` / getters:`rsiActive()`、`fault()`、`mode()`、`commandsReceived()`。
  - 行为表(收到命令时依内部状态回 ack):START_RSI:`ready && !fault` ⇒ `rsi_active=true`,Ok=1;否则 Ok=0 + Code(NOT_READY/FAULTED)。STOP_RSI:恒 Ok=1,`rsi_active=false`。SET_MODE:`!fault` ⇒ `mode=value`,Ok=1。RESET_FAULT:恒 Ok=1,`fault=false`。GET_TOOL / QUERY_STATE:恒 Ok=1(tool 数据总在 `RobotState` 中)。SET_TOOL_BASE:`!rsi_active` ⇒ 存 tool,Ok=1;否则 Ok=0 + NOT_READY(servo 中不许改工具)。
  - 独立可执行 `eki_mock_server`:`--heartbeat-ms`(默认 100)`--start-faulted`;监听 127.0.0.1 内核自选端口并打印(同 `sri_mock_server` 的端口策略)。
- mock 收侧复用 `EkiStreamSplitter` 思路但直接按 `</RobotCommand>` 切分(自有 buffer,行为表在锁内应用)。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/kuka_eki_bridge/test/test_eki_mock_server.cpp`:

```cpp
#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "kuka_eki_bridge/eki_frame.h"
#include "kuka_eki_bridge/eki_mock_server.h"
#include "kuka_eki_bridge/eki_stream_splitter.h"
#include "kuka_eki_bridge/tcp_client_transport.h"

using kuka_eki::EkiAction;
using kuka_eki::EkiCommand;
using kuka_eki::EkiMockConfig;
using kuka_eki::EkiMockServer;
using kuka_eki::EkiStateFrame;
using kuka_eki::EkiStreamSplitter;
using kuka_eki::TcpClientTransport;

namespace {

bool sendCommand(TcpClientTransport& t, std::uint32_t seq, EkiAction action,
                 int value = 0) {
  EkiCommand c;
  c.seq = seq;
  c.action = action;
  c.value = value;
  char buf[1024];
  const std::size_t n = kuka_eki::serializeCommand(c, buf, sizeof(buf));
  return n > 0 && t.send(buf, n, 200);
}

// Bounded wait for the next parsed state frame. Uses the real splitter +
// parser, so this loop is also an end-to-end schema check.
bool nextState(TcpClientTransport& t, EkiStreamSplitter& splitter,
               EkiStateFrame& out, int deadline_ms = 1000) {
  bool got = false;
  char buf[2048];
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(deadline_ms);
  while (!got && std::chrono::steady_clock::now() < deadline) {
    const int n = t.receive(buf, sizeof(buf), 50);
    if (n < 0) return false;
    if (n == 0) continue;
    splitter.feed(buf, static_cast<std::size_t>(n),
                  [&](const char* d, std::size_t len) {
                    if (!got) got = kuka_eki::parseState(d, len, out);
                  });
  }
  return got;
}

}  // namespace

TEST(EkiMock, StartRsiWorkflowAcksAndSetsActive) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 1, EkiAction::START_RSI));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_EQ(s.ack_seq, 1u);
  EXPECT_TRUE(s.ack_ok);
  EXPECT_TRUE(s.rsi_active);
  EXPECT_TRUE(mock.rsiActive());
}

TEST(EkiMock, StartRsiRefusedWhenNotReady) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setReady(false);
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 2, EkiAction::START_RSI));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_FALSE(s.ack_ok);
  EXPECT_EQ(s.ack_code, kuka_eki::kErrNotReady);
  EXPECT_FALSE(mock.rsiActive());
}

TEST(EkiMock, FaultThenResetWorkflow) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.injectFault();
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(sendCommand(t, 3, EkiAction::START_RSI));
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_FALSE(s.ack_ok);
  EXPECT_EQ(s.ack_code, kuka_eki::kErrFaulted);
  ASSERT_TRUE(sendCommand(t, 4, EkiAction::RESET_FAULT));
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_TRUE(s.ack_ok);
  EXPECT_FALSE(s.fault);
  ASSERT_TRUE(sendCommand(t, 5, EkiAction::START_RSI));
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_TRUE(s.ack_ok);
  EXPECT_TRUE(s.rsi_active);
}

TEST(EkiMock, ToolQueryReturnsConfiguredTool) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setTool(10.5, 0.0, 235.0, 0.0, 90.0, 0.0);
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 6, EkiAction::GET_TOOL));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_TRUE(s.ack_ok);
  EXPECT_NEAR(s.tool.x, 10.5, 1e-6);
  EXPECT_NEAR(s.tool.z, 235.0, 1e-6);
  EXPECT_NEAR(s.tool.b, 90.0, 1e-6);
}

TEST(EkiMock, SetModeUpdatesHeartbeatState) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 7, EkiAction::SET_MODE, 2));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_TRUE(s.ack_ok);
  EXPECT_EQ(s.mode, 2);
  EXPECT_EQ(mock.mode(), 2);
  mock.pushHeartbeat();
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_EQ(s.ack_seq, 0u);  // heartbeat marker
  EXPECT_EQ(s.mode, 2);
}

TEST(EkiMock, SwallowedCommandProducesNoAck) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setRespondToNext(false);
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 8, EkiAction::QUERY_STATE));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  EXPECT_FALSE(nextState(t, sp, s, 200));  // no answer within the window
  EXPECT_EQ(mock.commandsReceived(), 1u);
}

TEST(EkiMock, MalformedXmlIsSurvivable) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  mock.sendMalformed();  // "<RobotState><Broken></RobotState>" variant
  ASSERT_TRUE(sendCommand(t, 9, EkiAction::QUERY_STATE));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));  // parser skipped the bad doc
  EXPECT_EQ(s.ack_seq, 9u);
}

TEST(EkiMock, HeartbeatModeStreamsPeriodically) {
  EkiMockConfig cfg;
  cfg.heartbeat_period_s = 0.02;
  EkiMockServer mock{cfg};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  int got = 0;
  for (int i = 0; i < 3; ++i)
    if (nextState(t, sp, s, 500)) ++got;
  EXPECT_EQ(got, 3);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `eki_mock_server.h`。

- [ ] **Step 3: 写实现**

`ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_mock_server.h`:

```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "kuka_eki_bridge/eki_frame.h"

namespace kuka_eki {

struct EkiMockConfig {
  double heartbeat_period_s{0.0};  // 0 = heartbeats only via pushHeartbeat()
};

// KRC-side EKI mock (spec 15.2): TCP server on 127.0.0.1 (kernel-chosen
// port), single client. Parses RobotCommand documents and answers with
// RobotState acks according to a small behavior table mirroring the Plan 5
// KRL program (ready/start/stop/fault/reset/set-mode/tool). Test code:
// threads/locks are fine here (same stance as kuka_rsi::RsiMockServer).
class EkiMockServer {
 public:
  explicit EkiMockServer(const EkiMockConfig& cfg);
  ~EkiMockServer();
  EkiMockServer(const EkiMockServer&) = delete;
  EkiMockServer& operator=(const EkiMockServer&) = delete;

  bool start();
  void stop();
  std::uint16_t port() const { return port_; }

  bool waitForClient(int timeout_ms);
  void dropClient();

  // Scripting surface.
  void pushHeartbeat();                 // one unsolicited RobotState frame
  void setReady(bool ready);
  void injectFault();
  void setTool(double x, double y, double z, double a, double b, double c);
  void setRespondToNext(bool respond);  // false: swallow one command
  void sendMalformed();

  bool rsiActive() const;
  bool fault() const;
  int mode() const;
  std::uint64_t commandsReceived() const { return commands_.load(); }

 private:
  void run();
  void handleCommand(const EkiCommand& cmd);
  void sendStateLocked(std::uint32_t ack_seq, bool ok, int code);
  bool parseCommand(const char* data, std::size_t len, EkiCommand& out);

  EkiMockConfig cfg_;
  int listen_fd_{-1};
  std::uint16_t port_{0};
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> commands_{0};
  mutable std::mutex mutex_;  // guards client_fd_ and the state table
  int client_fd_{-1};
  std::string rx_buffer_;
  bool ready_{true};
  bool rsi_active_{false};
  bool fault_{false};
  int mode_{0};
  Frame6 tool_;
  bool respond_next_{true};
};

}  // namespace kuka_eki
```

`ros_ws/src/kuka_eki_bridge/src/eki_mock_server.cpp`:

```cpp
#include "kuka_eki_bridge/eki_mock_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <tinyxml2.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>

namespace kuka_eki {

namespace {
constexpr int kPollMs = 10;
constexpr const char kCmdEndTag[] = "</RobotCommand>";
}  // namespace

EkiMockServer::EkiMockServer(const EkiMockConfig& cfg) : cfg_(cfg) {}

EkiMockServer::~EkiMockServer() { stop(); }

bool EkiMockServer::start() {
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) return false;
  int on = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = ::inet_addr("127.0.0.1");
  addr.sin_port = 0;
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) !=
          0 ||
      ::listen(listen_fd_, 1) != 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  socklen_t len = sizeof(addr);
  ::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
  port_ = ntohs(addr.sin_port);
  running_ = true;
  thread_ = std::thread(&EkiMockServer::run, this);
  return true;
}

void EkiMockServer::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  dropClient();
  if (listen_fd_ >= 0) ::close(listen_fd_);
  listen_fd_ = -1;
}

bool EkiMockServer::waitForClient(int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (client_fd_ >= 0) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return client_fd_ >= 0;
}

void EkiMockServer::dropClient() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (client_fd_ >= 0) {
    ::shutdown(client_fd_, SHUT_RDWR);
    ::close(client_fd_);
    client_fd_ = -1;
  }
  rx_buffer_.clear();
}

void EkiMockServer::pushHeartbeat() {
  std::lock_guard<std::mutex> lock(mutex_);
  sendStateLocked(0, true, kErrOk);
}

void EkiMockServer::setReady(bool ready) {
  std::lock_guard<std::mutex> lock(mutex_);
  ready_ = ready;
}

void EkiMockServer::injectFault() {
  std::lock_guard<std::mutex> lock(mutex_);
  fault_ = true;
  rsi_active_ = false;
}

void EkiMockServer::setTool(double x, double y, double z, double a, double b,
                            double c) {
  std::lock_guard<std::mutex> lock(mutex_);
  tool_ = Frame6{x, y, z, a, b, c};
}

void EkiMockServer::setRespondToNext(bool respond) {
  std::lock_guard<std::mutex> lock(mutex_);
  respond_next_ = respond;
}

void EkiMockServer::sendMalformed() {
  std::lock_guard<std::mutex> lock(mutex_);
  static const char kBad[] = "<RobotState><Ack Seq=\"1\"</RobotState>";
  if (client_fd_ >= 0) {
#ifdef MSG_NOSIGNAL
    ::send(client_fd_, kBad, sizeof(kBad) - 1, MSG_NOSIGNAL);
#else
    ::send(client_fd_, kBad, sizeof(kBad) - 1, 0);
#endif
  }
}

bool EkiMockServer::rsiActive() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rsi_active_;
}

bool EkiMockServer::fault() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fault_;
}

int EkiMockServer::mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

void EkiMockServer::sendStateLocked(std::uint32_t ack_seq, bool ok,
                                    int code) {
  // Caller holds mutex_.
  if (client_fd_ < 0) return;
  char buf[1024];
  const int n = std::snprintf(
      buf, sizeof(buf),
      "<RobotState>"
      "<Ack Seq=\"%u\" Ok=\"%d\" Code=\"%d\"/>"
      "<Prog Ready=\"%d\" RsiActive=\"%d\" Fault=\"%d\" Mode=\"%d\"/>"
      "<Tool X=\"%f\" Y=\"%f\" Z=\"%f\" A=\"%f\" B=\"%f\" C=\"%f\"/>"
      "</RobotState>",
      ack_seq, ok ? 1 : 0, code, ready_ ? 1 : 0, rsi_active_ ? 1 : 0,
      fault_ ? 1 : 0, mode_, tool_.x, tool_.y, tool_.z, tool_.a, tool_.b,
      tool_.c);
  if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(buf)) return;
#ifdef MSG_NOSIGNAL
  ::send(client_fd_, buf, static_cast<std::size_t>(n), MSG_NOSIGNAL);
#else
  ::send(client_fd_, buf, static_cast<std::size_t>(n), 0);
#endif
}

bool EkiMockServer::parseCommand(const char* data, std::size_t len,
                                 EkiCommand& out) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(data, len) != tinyxml2::XML_SUCCESS) return false;
  const tinyxml2::XMLElement* root = doc.FirstChildElement("RobotCommand");
  if (root == nullptr) return false;
  const tinyxml2::XMLElement* cmd = root->FirstChildElement("Cmd");
  const tinyxml2::XMLElement* tool = root->FirstChildElement("Tool");
  if (cmd == nullptr || tool == nullptr) return false;
  unsigned seq = 0;
  int action = 0;
  if (cmd->QueryUnsignedAttribute("Seq", &seq) != tinyxml2::XML_SUCCESS ||
      cmd->QueryIntAttribute("Action", &action) != tinyxml2::XML_SUCCESS ||
      cmd->QueryIntAttribute("Value", &out.value) != tinyxml2::XML_SUCCESS)
    return false;
  out.seq = seq;
  out.action = static_cast<EkiAction>(action);
  tool->QueryDoubleAttribute("X", &out.tool.x);
  tool->QueryDoubleAttribute("Y", &out.tool.y);
  tool->QueryDoubleAttribute("Z", &out.tool.z);
  tool->QueryDoubleAttribute("A", &out.tool.a);
  tool->QueryDoubleAttribute("B", &out.tool.b);
  tool->QueryDoubleAttribute("C", &out.tool.c);
  return true;
}

void EkiMockServer::handleCommand(const EkiCommand& cmd) {
  // Caller holds mutex_. Behavior table (Task 9 header).
  ++commands_;
  if (!respond_next_) {
    respond_next_ = true;
    return;
  }
  bool ok = true;
  int code = kErrOk;
  switch (cmd.action) {
    case EkiAction::START_RSI:
      if (fault_) {
        ok = false;
        code = kErrFaulted;
      } else if (!ready_) {
        ok = false;
        code = kErrNotReady;
      } else {
        rsi_active_ = true;
      }
      break;
    case EkiAction::STOP_RSI:
      rsi_active_ = false;
      break;
    case EkiAction::SET_MODE:
      if (fault_) {
        ok = false;
        code = kErrFaulted;
      } else {
        mode_ = cmd.value;
      }
      break;
    case EkiAction::RESET_FAULT:
      fault_ = false;
      break;
    case EkiAction::SET_TOOL_BASE:
      if (rsi_active_) {
        ok = false;
        code = kErrNotReady;  // no tool change while servoing
      } else {
        tool_ = cmd.tool;
      }
      break;
    case EkiAction::GET_TOOL:
    case EkiAction::QUERY_STATE:
      break;
  }
  sendStateLocked(cmd.seq, ok, code);
}

void EkiMockServer::run() {
  using clock = std::chrono::steady_clock;
  auto next_heartbeat = clock::now();
  while (running_.load()) {
    int client;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      client = client_fd_;
    }
    if (client < 0) {
      pollfd p{listen_fd_, POLLIN, 0};
      if (::poll(&p, 1, kPollMs) > 0) {
        const int fd = ::accept(listen_fd_, nullptr, nullptr);
        if (fd >= 0) {
          std::lock_guard<std::mutex> lock(mutex_);
          client_fd_ = fd;
          rx_buffer_.clear();
        }
      }
      continue;
    }
    pollfd p{client, POLLIN, 0};
    if (::poll(&p, 1, kPollMs) > 0) {
      char buf[2048];
      const ssize_t n = ::recv(client, buf, sizeof(buf), 0);
      if (n <= 0) {
        dropClient();
        continue;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      rx_buffer_.append(buf, static_cast<std::size_t>(n));
      for (;;) {
        const std::size_t end = rx_buffer_.find(kCmdEndTag);
        if (end == std::string::npos) break;
        const std::size_t doc_len = end + sizeof(kCmdEndTag) - 1;
        EkiCommand cmd;
        if (parseCommand(rx_buffer_.data(), doc_len, cmd)) {
          handleCommand(cmd);
        }
        rx_buffer_.erase(0, doc_len);
      }
    }
    if (cfg_.heartbeat_period_s > 0.0 && clock::now() >= next_heartbeat) {
      pushHeartbeat();
      next_heartbeat =
          clock::now() + std::chrono::microseconds(static_cast<long>(
                             cfg_.heartbeat_period_s * 1e6));
    }
  }
}

}  // namespace kuka_eki
```

`ros_ws/src/kuka_eki_bridge/src/eki_mock_server_main.cpp`:

```cpp
// Standalone KRC-side EKI mock for manual smoke tests and Plan 5 bringup.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "kuka_eki_bridge/eki_mock_server.h"

namespace {
int usage(int code) {
  std::printf(
      "usage: eki_mock_server [--heartbeat-ms N] [--start-faulted]\n"
      "Listens on a kernel-chosen 127.0.0.1 port (printed on stdout).\n");
  return code;
}
}  // namespace

int main(int argc, char** argv) {
  kuka_eki::EkiMockConfig cfg;
  cfg.heartbeat_period_s = 0.1;
  bool start_faulted = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help" || a == "-h") return usage(0);
    if (a == "--start-faulted") {
      start_faulted = true;
    } else if (a == "--heartbeat-ms" && i + 1 < argc) {
      cfg.heartbeat_period_s = std::atof(argv[++i]) / 1000.0;
    } else {
      return usage(1);
    }
  }
  kuka_eki::EkiMockServer mock(cfg);
  if (!mock.start()) {
    std::fprintf(stderr, "bind/listen failed\n");
    return 1;
  }
  if (start_faulted) mock.injectFault();
  std::printf("eki_mock_server listening on 127.0.0.1:%u (heartbeat %.0f ms)\n",
              mock.port(), cfg.heartbeat_period_s * 1000.0);
  std::fflush(stdout);
  for (;;) std::this_thread::sleep_for(std::chrono::seconds(1));
  return 0;
}
```

- [ ] **Step 4: CMake 增量**

```cmake
add_library(kuka_eki_mock
  src/eki_mock_server.cpp
)
target_link_libraries(kuka_eki_mock kuka_eki_protocol pthread)

add_executable(eki_mock_server src/eki_mock_server_main.cpp)
target_link_libraries(eki_mock_server kuka_eki_mock)
```

`catkin_package(LIBRARIES ...)` 改为 `LIBRARIES kuka_eki_protocol kuka_eki_mock`。测试段追加:

```cmake
  catkin_add_gtest(test_eki_mock_server test/test_eki_mock_server.cpp)
  target_link_libraries(test_eki_mock_server kuka_eki_mock
                        ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_eki_bridge/test_eki_mock_server
```

预期:`[  PASSED  ] 8 tests.`

- [ ] **Step 6: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_eki_bridge && \
git commit -m "feat(eki): KRC-side mock server with behavior table + standalone exe (Plan 4 Task 9)"
```

---

### Task 10: `EkiBridgeRuntime` + `eki_bridge_node` ROS 壳 + `kuka_eki.yaml` + 安装规则 + 全量回归

**Files:**
- Create: `ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_bridge_runtime.h`
- Create: `ros_ws/src/kuka_eki_bridge/src/eki_bridge_runtime.cpp`
- Create: `ros_ws/src/kuka_eki_bridge/src/eki_bridge_node.cpp`
- Create: `ros_ws/src/kuka_eki_bridge/config/kuka_eki.yaml`
- Modify: `ros_ws/src/kuka_eki_bridge/CMakeLists.txt`
- Test: `ros_ws/src/kuka_eki_bridge/test/test_eki_bridge_runtime.cpp`

**Interfaces:**
- Consumes: Task 7/8/9 全部产物。
- Produces(节点 + Plan 5 消费):
  - `struct EkiBridgeConfig { kuka_ip, eki_port, connect_timeout_ms, receive_timeout_ms, reconnect_backoff_s, auto_reconnect, EkiSessionConfig session }`
  - 桥本地错误码 `kErrNotConnected = -1`(KRC 侧码非负,不冲突)。
  - `struct ExecuteResult { CommandOutcome outcome; int error_code; EkiStateFrame state; }`(`state` = 终态时刻的最近状态帧;GET_TOOL 的 tool 从此取——ack 帧本身携带状态)。
  - `struct EkiBridgeStatus { bool connected; std::uint32_t reconnects; EkiSessionSnapshot session; }`
  - `class EkiBridgeRuntime`:`start` / `stop` / `connectNow(timeout_ms)` / `execute(action, value, tool, base)`(阻塞、有界、绝不返回 PENDING/NONE 之外还会有三种终态,见待确认 12)/ `status()`。
  - ROS 节点 `eki_bridge_node`:服务 `/kuka/eki/connect`、`/kuka/eki/start_rsi_program`、`/kuka/eki/stop_rsi_program`(Trigger)、`/kuka/eki/set_mode`(SetEkiMode)、`/kuka/eki/reset_fault`(Trigger)、`/kuka/eki/set_tool_base`(SetToolBase)、`/kuka/eki/get_tool`(GetTool);话题 `/kuka/eki/state`(EkiState,10 Hz)、`/kuka/diagnostics`(DiagnosticArray,1 Hz)。
- 线程模型(待确认 12):io 线程独占 transport/splitter;`execute()` 把请求置入共享槽(`command_mutex_` 串行化多调用方),io 线程负责 begin/serialize/send/裁决,经 condition_variable 唤醒等待者;所有终态由 io 线程给出——发送失败/未连接 ⇒ `REJECTED(kErrNotConnected)`,ack ⇒ `ACCEPTED/REJECTED(ack_code)`,超时 ⇒ `TIMEOUT`;`execute()` 等待上界 = `response_timeout_s + 1 s`。

- [ ] **Step 1: 写失败测试**

`ros_ws/src/kuka_eki_bridge/test/test_eki_bridge_runtime.cpp`:

```cpp
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <thread>

#include "kuka_eki_bridge/eki_bridge_runtime.h"
#include "kuka_eki_bridge/eki_mock_server.h"

using kuka_eki::CommandOutcome;
using kuka_eki::EkiAction;
using kuka_eki::EkiBridgeConfig;
using kuka_eki::EkiBridgeRuntime;
using kuka_eki::EkiMockConfig;
using kuka_eki::EkiMockServer;
using kuka_eki::ExecuteResult;
using kuka_eki::Frame6;

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

EkiBridgeConfig config(std::uint16_t port) {
  EkiBridgeConfig c;
  c.kuka_ip = "127.0.0.1";
  c.eki_port = port;
  c.connect_timeout_ms = 200;
  c.receive_timeout_ms = 20;
  c.reconnect_backoff_s = 0.05;
  c.auto_reconnect = true;
  c.session.response_timeout_s = 0.5;
  c.session.state_timeout_s = 0.2;
  return c;
}

}  // namespace

TEST(EkiRuntime, ExecuteStartRsiAccepted) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  const ExecuteResult r = rt.execute(EkiAction::START_RSI);
  EXPECT_EQ(r.outcome, CommandOutcome::ACCEPTED);
  EXPECT_TRUE(r.state.rsi_active);
  EXPECT_TRUE(mock.rsiActive());
  rt.stop();
}

TEST(EkiRuntime, RejectedPropagatesKrcErrorCode) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.injectFault();
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  const ExecuteResult r = rt.execute(EkiAction::START_RSI);
  EXPECT_EQ(r.outcome, CommandOutcome::REJECTED);
  EXPECT_EQ(r.error_code, kuka_eki::kErrFaulted);
  rt.stop();
}

TEST(EkiRuntime, ResetFaultThenStartSucceeds) {
  // Plan 2 follow-up 5 (KRC-side share): fault -> reset -> start workflow.
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.injectFault();
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  EXPECT_EQ(rt.execute(EkiAction::RESET_FAULT).outcome,
            CommandOutcome::ACCEPTED);
  EXPECT_EQ(rt.execute(EkiAction::START_RSI).outcome,
            CommandOutcome::ACCEPTED);
  rt.stop();
}

TEST(EkiRuntime, TimeoutIsBoundedAndTerminal) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setRespondToNext(false);  // swallow the command
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  const auto t0 = std::chrono::steady_clock::now();
  const ExecuteResult r = rt.execute(EkiAction::QUERY_STATE);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
  EXPECT_EQ(r.outcome, CommandOutcome::TIMEOUT);
  EXPECT_LT(ms, 2000);  // response_timeout 0.5 s + margin, well bounded
  rt.stop();
}

TEST(EkiRuntime, GetToolDeliversToolFrame) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setTool(10.5, 0.0, 235.0, 0.0, 90.0, 0.0);
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  const ExecuteResult r = rt.execute(EkiAction::GET_TOOL);
  ASSERT_EQ(r.outcome, CommandOutcome::ACCEPTED);
  EXPECT_NEAR(r.state.tool.x, 10.5, 1e-6);
  EXPECT_NEAR(r.state.tool.z, 235.0, 1e-6);
  EXPECT_NEAR(r.state.tool.b, 90.0, 1e-6);
  rt.stop();
}

TEST(EkiRuntime, NotConnectedRejectsImmediately) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  const std::uint16_t dead_port = mock.port();
  mock.stop();  // nobody listens anymore
  EkiBridgeRuntime rt{config(dead_port)};
  ASSERT_TRUE(rt.start());
  const ExecuteResult r = rt.execute(EkiAction::QUERY_STATE);
  EXPECT_EQ(r.outcome, CommandOutcome::REJECTED);
  EXPECT_EQ(r.error_code, kuka_eki::kErrNotConnected);
  rt.stop();
}

TEST(EkiRuntime, ReconnectsAfterDropAndServesCommands) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  ASSERT_EQ(rt.execute(EkiAction::QUERY_STATE).outcome,
            CommandOutcome::ACCEPTED);
  mock.dropClient();
  ASSERT_TRUE(waitFor([&] { return rt.status().reconnects >= 1; }, 2000));
  EXPECT_EQ(rt.execute(EkiAction::QUERY_STATE).outcome,
            CommandOutcome::ACCEPTED);
  rt.stop();
}

TEST(EkiRuntime, HeartbeatFreshnessAndBoundedStop) {
  EkiMockConfig mc;
  mc.heartbeat_period_s = 0.02;
  EkiMockServer mock{mc};
  ASSERT_TRUE(mock.start());
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(
      waitFor([&] { return rt.status().session.state_fresh; }, 2000));
  const auto t0 = std::chrono::steady_clock::now();
  rt.stop();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
  EXPECT_LT(ms, 1000);
}
```

- [ ] **Step 2: 构建确认失败**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

预期:BUILD FAILS,缺 `eki_bridge_runtime.h`。

- [ ] **Step 3: 写 runtime 实现**

`ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_bridge_runtime.h`:

```cpp
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "kuka_eki_bridge/eki_frame.h"
#include "kuka_eki_bridge/eki_session_core.h"
#include "kuka_eki_bridge/eki_stream_splitter.h"
#include "kuka_eki_bridge/tcp_client_transport.h"

namespace kuka_eki {

// Bridge-local error code; KRC-side Ack codes are non-negative.
constexpr int kErrNotConnected = -1;

struct EkiBridgeConfig {
  std::string kuka_ip{"127.0.0.1"};
  std::uint16_t eki_port{54600};
  int connect_timeout_ms{1000};
  int receive_timeout_ms{50};
  double reconnect_backoff_s{1.0};
  bool auto_reconnect{true};
  EkiSessionConfig session;
};

struct ExecuteResult {
  CommandOutcome outcome{CommandOutcome::NONE};
  int error_code{0};   // Ack.Code on REJECTED; kErrNotConnected; else 0
  EkiStateFrame state; // latest state frame at resolution time
};

struct EkiBridgeStatus {
  bool connected{false};
  std::uint32_t reconnects{0};
  EkiSessionSnapshot session;
};

// Owns the io thread (connect/reconnect, receive -> splitter -> parser ->
// session, timeout ticks) and a blocking execute() used by the service
// layer (decision 12). The io thread is the only toucher of the socket;
// execute() posts a request into a shared slot and waits on a condition
// variable for the io thread's terminal verdict — never PENDING, bounded
// by response_timeout_s + 1 s. Not ROS-dependent.
class EkiBridgeRuntime {
 public:
  explicit EkiBridgeRuntime(const EkiBridgeConfig& cfg);
  ~EkiBridgeRuntime();
  EkiBridgeRuntime(const EkiBridgeRuntime&) = delete;
  EkiBridgeRuntime& operator=(const EkiBridgeRuntime&) = delete;

  bool start();
  void stop();

  // Waits (bounded) until the link is up. With auto_reconnect the io
  // thread connects on its own; this only observes.
  bool connectNow(int timeout_ms);

  ExecuteResult execute(EkiAction action, int value = 0,
                        const Frame6& tool = Frame6{},
                        const Frame6& base = Frame6{});

  EkiBridgeStatus status() const;

 private:
  void run();
  void resolveLocked(CommandOutcome outcome, int error_code);
  static double nowS();

  EkiBridgeConfig cfg_;
  TcpClientTransport transport_;  // io thread only
  EkiStreamSplitter splitter_;    // io thread only
  std::thread thread_;
  std::atomic<bool> running_{false};

  std::mutex command_mutex_;  // serializes execute() callers

  mutable std::mutex mutex_;  // guards everything below
  std::condition_variable cv_;
  EkiSessionCore session_;
  bool connected_{false};
  bool was_connected_once_{false};
  std::uint32_t reconnects_{0};
  bool connect_requested_{false};
  std::uint32_t next_seq_{1};
  bool req_active_{false};
  EkiCommand req_cmd_;
  bool result_ready_{false};
  ExecuteResult result_;
};

}  // namespace kuka_eki
```

`ros_ws/src/kuka_eki_bridge/src/eki_bridge_runtime.cpp`:

```cpp
#include "kuka_eki_bridge/eki_bridge_runtime.h"

#include <chrono>

namespace kuka_eki {

namespace {
void sleepSlices(double seconds, const std::atomic<bool>& keep_running) {
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::microseconds(static_cast<long>(seconds * 1e6));
  while (keep_running.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
}  // namespace

EkiBridgeRuntime::EkiBridgeRuntime(const EkiBridgeConfig& cfg) : cfg_(cfg) {
  session_.configure(cfg_.session);
}

EkiBridgeRuntime::~EkiBridgeRuntime() { stop(); }

double EkiBridgeRuntime::nowS() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool EkiBridgeRuntime::start() {
  if (running_.load()) return false;
  running_ = true;
  thread_ = std::thread(&EkiBridgeRuntime::run, this);
  return true;
}

void EkiBridgeRuntime::stop() {
  running_ = false;
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  transport_.close();
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = false;
}

bool EkiBridgeRuntime::connectNow(int timeout_ms) {
  std::unique_lock<std::mutex> lock(mutex_);
  connect_requested_ = true;
  cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
               [this] { return connected_; });
  return connected_;
}

void EkiBridgeRuntime::resolveLocked(CommandOutcome outcome, int error_code) {
  // Caller holds mutex_.
  result_.outcome = outcome;
  result_.error_code = error_code;
  result_.state = session_.snapshot(nowS()).last_state;
  result_ready_ = true;
  cv_.notify_all();
}

ExecuteResult EkiBridgeRuntime::execute(EkiAction action, int value,
                                        const Frame6& tool,
                                        const Frame6& base) {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  std::unique_lock<std::mutex> lock(mutex_);
  req_cmd_ = EkiCommand{};
  req_cmd_.seq = next_seq_++;
  if (next_seq_ == 0) next_seq_ = 1;  // 0 is the heartbeat marker
  req_cmd_.action = action;
  req_cmd_.value = value;
  req_cmd_.tool = tool;
  req_cmd_.base = base;
  req_active_ = true;
  result_ready_ = false;
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(
          static_cast<long>(cfg_.session.response_timeout_s * 1000.0) + 1000);
  cv_.wait_until(lock, deadline, [this] { return result_ready_; });
  req_active_ = false;
  if (!result_ready_) {
    // io thread never resolved (e.g. stop() during execute): terminal
    // timeout so callers never see PENDING semantics.
    ExecuteResult r;
    r.outcome = CommandOutcome::TIMEOUT;
    r.error_code = kErrNotConnected;
    return r;
  }
  return result_;
}

EkiBridgeStatus EkiBridgeRuntime::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  EkiBridgeStatus s;
  s.connected = connected_;
  s.reconnects = reconnects_;
  s.session = session_.snapshot(nowS());
  return s;
}

void EkiBridgeRuntime::run() {
  char buf[2048];
  while (running_.load()) {
    if (!transport_.connected()) {
      bool want_connect;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = false;
        want_connect = cfg_.auto_reconnect || connect_requested_;
        if (!want_connect && req_active_ && !result_ready_) {
          resolveLocked(CommandOutcome::REJECTED, kErrNotConnected);
        }
      }
      if (!want_connect) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (transport_.connect(cfg_.kuka_ip, cfg_.eki_port,
                             cfg_.connect_timeout_ms)) {
        std::lock_guard<std::mutex> lock(mutex_);
        splitter_.reset();
        session_.reset();  // no command can be pending while disconnected
        connected_ = true;
        connect_requested_ = false;
        if (was_connected_once_) ++reconnects_;
        was_connected_once_ = true;
        cv_.notify_all();
      } else {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (req_active_ && !result_ready_) {
            resolveLocked(CommandOutcome::REJECTED, kErrNotConnected);
          }
        }
        sleepSlices(cfg_.reconnect_backoff_s, running_);
      }
      continue;
    }

    // Service a queued request once no command is in flight.
    char cmd_buf[1024];
    std::size_t cmd_len = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (req_active_ && !result_ready_ && !session_.commandPending()) {
        if (session_.beginCommand(req_cmd_.seq, nowS())) {
          cmd_len = serializeCommand(req_cmd_, cmd_buf, sizeof(cmd_buf));
        }
      }
    }
    if (cmd_len > 0 && !transport_.send(cmd_buf, cmd_len, 200)) {
      std::lock_guard<std::mutex> lock(mutex_);
      connected_ = false;
      session_.reset();
      if (req_active_ && !result_ready_) {
        resolveLocked(CommandOutcome::REJECTED, kErrNotConnected);
      }
      continue;
    }

    const int n =
        transport_.receive(buf, sizeof(buf), cfg_.receive_timeout_ms);
    std::lock_guard<std::mutex> lock(mutex_);
    if (n < 0) {
      connected_ = false;
      const CommandOutcome oc = session_.reset();
      if (oc != CommandOutcome::NONE && req_active_ && !result_ready_) {
        resolveLocked(CommandOutcome::TIMEOUT, kErrNotConnected);
      }
      continue;
    }
    if (n > 0) {
      splitter_.feed(buf, static_cast<std::size_t>(n),
                     [this](const char* d, std::size_t len) {
                       EkiStateFrame f;
                       if (!parseState(d, len, f)) {
                         session_.onBadFrame();
                         return;
                       }
                       const CommandOutcome oc = session_.onState(f, nowS());
                       if (oc != CommandOutcome::NONE && req_active_ &&
                           !result_ready_) {
                         resolveLocked(oc, oc == CommandOutcome::REJECTED
                                               ? f.ack_code
                                               : 0);
                       }
                     });
    }
    const CommandOutcome oc = session_.tick(nowS());
    if (oc == CommandOutcome::TIMEOUT && req_active_ && !result_ready_) {
      resolveLocked(oc, 0);
    }
  }
}

}  // namespace kuka_eki
```

- [ ] **Step 4: 写节点壳与配置**

`ros_ws/src/kuka_eki_bridge/src/eki_bridge_node.cpp`:

```cpp
// Thin ROS shell around EkiBridgeRuntime (spec 5.2, 6.2, 6.4). All
// protocol/session logic lives in the offline-tested library; this file
// only loads parameters and forwards service calls. Service/topic names
// are the absolute ones fixed by the spec (/kuka/eki/...).
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <soft_robot_msgs/EkiState.h>
#include <soft_robot_msgs/GetTool.h>
#include <soft_robot_msgs/SetEkiMode.h>
#include <soft_robot_msgs/SetToolBase.h>
#include <std_srvs/Trigger.h>

#include <string>

#include "kuka_eki_bridge/eki_bridge_runtime.h"

namespace {

std::string outcomeMessage(const kuka_eki::ExecuteResult& r) {
  switch (r.outcome) {
    case kuka_eki::CommandOutcome::ACCEPTED:
      return "ok";
    case kuka_eki::CommandOutcome::REJECTED:
      return r.error_code == kuka_eki::kErrNotConnected
                 ? "not connected to the KRC"
                 : "rejected by KRC (code " + std::to_string(r.error_code) +
                       ")";
    case kuka_eki::CommandOutcome::TIMEOUT:
      return "KRC response timeout";
    default:
      return "no result";
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "kuka_eki_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  kuka_eki::EkiBridgeConfig cfg;
  int port = 54600;
  int connect_wait_ms = 3000;
  pnh.param<std::string>("kuka_ip", cfg.kuka_ip, "192.168.1.10");
  pnh.param("eki_port", port, 54600);
  cfg.eki_port = static_cast<std::uint16_t>(port);
  pnh.param("connect_timeout_ms", cfg.connect_timeout_ms, 1000);
  pnh.param("receive_timeout_ms", cfg.receive_timeout_ms, 50);
  pnh.param("reconnect_backoff_s", cfg.reconnect_backoff_s, 1.0);
  pnh.param("auto_reconnect", cfg.auto_reconnect, true);
  pnh.param("response_timeout_s", cfg.session.response_timeout_s, 2.0);
  pnh.param("state_timeout_s", cfg.session.state_timeout_s, 1.0);
  pnh.param("connect_wait_ms", connect_wait_ms, 3000);

  kuka_eki::EkiBridgeRuntime runtime(cfg);

  auto trigger = [&runtime](kuka_eki::EkiAction action) {
    return [&runtime, action](std_srvs::Trigger::Request&,
                              std_srvs::Trigger::Response& res) {
      const kuka_eki::ExecuteResult r = runtime.execute(action);
      res.success = r.outcome == kuka_eki::CommandOutcome::ACCEPTED;
      res.message = outcomeMessage(r);
      return true;
    };
  };

  ros::ServiceServer connect_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/kuka/eki/connect",
          [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
            res.success = runtime.connectNow(connect_wait_ms);
            res.message = res.success ? "connected" : "connect timeout";
            return true;
          });
  ros::ServiceServer start_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/kuka/eki/start_rsi_program",
          trigger(kuka_eki::EkiAction::START_RSI));
  ros::ServiceServer stop_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/kuka/eki/stop_rsi_program",
          trigger(kuka_eki::EkiAction::STOP_RSI));
  ros::ServiceServer reset_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/kuka/eki/reset_fault",
          trigger(kuka_eki::EkiAction::RESET_FAULT));
  ros::ServiceServer mode_srv =
      nh.advertiseService<soft_robot_msgs::SetEkiMode::Request,
                          soft_robot_msgs::SetEkiMode::Response>(
          "/kuka/eki/set_mode",
          [&](soft_robot_msgs::SetEkiMode::Request& req,
              soft_robot_msgs::SetEkiMode::Response& res) {
            const kuka_eki::ExecuteResult r = runtime.execute(
                kuka_eki::EkiAction::SET_MODE, static_cast<int>(req.mode));
            res.success = r.outcome == kuka_eki::CommandOutcome::ACCEPTED;
            res.message = outcomeMessage(r);
            return true;
          });
  ros::ServiceServer tool_base_srv =
      nh.advertiseService<soft_robot_msgs::SetToolBase::Request,
                          soft_robot_msgs::SetToolBase::Response>(
          "/kuka/eki/set_tool_base",
          [&](soft_robot_msgs::SetToolBase::Request& req,
              soft_robot_msgs::SetToolBase::Response& res) {
            kuka_eki::Frame6 tool{req.tool_x, req.tool_y, req.tool_z,
                                  req.tool_a, req.tool_b, req.tool_c};
            kuka_eki::Frame6 base{req.base_x, req.base_y, req.base_z,
                                  req.base_a, req.base_b, req.base_c};
            const kuka_eki::ExecuteResult r = runtime.execute(
                kuka_eki::EkiAction::SET_TOOL_BASE, 0, tool, base);
            res.success = r.outcome == kuka_eki::CommandOutcome::ACCEPTED;
            res.message = outcomeMessage(r);
            return true;
          });
  ros::ServiceServer get_tool_srv =
      nh.advertiseService<soft_robot_msgs::GetTool::Request,
                          soft_robot_msgs::GetTool::Response>(
          "/kuka/eki/get_tool",
          [&](soft_robot_msgs::GetTool::Request&,
              soft_robot_msgs::GetTool::Response& res) {
            const kuka_eki::ExecuteResult r =
                runtime.execute(kuka_eki::EkiAction::GET_TOOL);
            res.success = r.outcome == kuka_eki::CommandOutcome::ACCEPTED;
            res.message = outcomeMessage(r);
            res.x = r.state.tool.x;
            res.y = r.state.tool.y;
            res.z = r.state.tool.z;
            res.a = r.state.tool.a;
            res.b = r.state.tool.b;
            res.c = r.state.tool.c;
            return true;
          });

  ros::Publisher state_pub =
      nh.advertise<soft_robot_msgs::EkiState>("/kuka/eki/state", 10);
  ros::Publisher diag_pub =
      nh.advertise<diagnostic_msgs::DiagnosticArray>("/kuka/diagnostics", 10);

  ros::Timer state_timer =
      nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&) {
        const kuka_eki::EkiBridgeStatus st = runtime.status();
        soft_robot_msgs::EkiState msg;
        msg.header.stamp = ros::Time::now();
        msg.connected = st.connected;
        msg.state_fresh = st.session.state_fresh;
        msg.program_ready = st.session.last_state.ready;
        msg.rsi_active = st.session.last_state.rsi_active;
        msg.fault = st.session.last_state.fault;
        msg.mode = static_cast<std::uint8_t>(st.session.last_state.mode);
        msg.reconnects = st.reconnects;
        msg.state_age = st.session.state_age_s;
        msg.tool_x = st.session.last_state.tool.x;
        msg.tool_y = st.session.last_state.tool.y;
        msg.tool_z = st.session.last_state.tool.z;
        msg.tool_a = st.session.last_state.tool.a;
        msg.tool_b = st.session.last_state.tool.b;
        msg.tool_c = st.session.last_state.tool.c;
        state_pub.publish(msg);
      });
  ros::Timer diag_timer =
      nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent&) {
        const kuka_eki::EkiBridgeStatus st = runtime.status();
        diagnostic_msgs::DiagnosticArray arr;
        arr.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus s;
        s.name = "kuka_eki_bridge: management link";
        s.hardware_id = cfg.kuka_ip;
        if (!st.connected) {
          s.level = diagnostic_msgs::DiagnosticStatus::ERROR;
          s.message = "EKI disconnected";
        } else if (st.session.last_state.fault) {
          s.level = diagnostic_msgs::DiagnosticStatus::ERROR;
          s.message = "KRC fault latched";
        } else if (!st.session.state_fresh) {
          s.level = diagnostic_msgs::DiagnosticStatus::WARN;
          s.message = "state heartbeat stale";
        } else {
          s.level = diagnostic_msgs::DiagnosticStatus::OK;
          s.message = "ok";
        }
        auto kv = [&s](const std::string& k, const std::string& v) {
          diagnostic_msgs::KeyValue e;
          e.key = k;
          e.value = v;
          s.values.push_back(e);
        };
        kv("connected", st.connected ? "true" : "false");
        kv("state_age_s", std::to_string(st.session.state_age_s));
        kv("timeouts", std::to_string(st.session.timeouts));
        kv("bad_frames", std::to_string(st.session.bad_frames));
        kv("reconnects", std::to_string(st.reconnects));
        arr.status.push_back(s);
        diag_pub.publish(arr);
      });

  if (!runtime.start()) {
    ROS_ERROR("kuka_eki_bridge: runtime failed to start");
    return 1;
  }
  ROS_INFO("kuka_eki_bridge: KRC EKI server %s:%u (external system = client)",
           cfg.kuka_ip.c_str(), cfg.eki_port);
  ros::spin();
  runtime.stop();
  return 0;
}
```

`ros_ws/src/kuka_eki_bridge/config/kuka_eki.yaml`:

```yaml
# EKI management-channel parameters (spec section 14, kuka_rsi.yaml's EKI
# keys live here with the bridge). Loaded into the kuka_eki_bridge private
# namespace by the Plan 5 bringup. Direction (decision 5): the KRC is the
# TCP server (<EXTERNAL><TYPE>Client</TYPE> in EkiConfig.xml); this node
# is the client.
kuka_eki_bridge:
  kuka_ip: 192.168.1.10         # legacy Parameter.xml RobIP
  eki_port: 54600
  connect_timeout_ms: 1000
  receive_timeout_ms: 50
  reconnect_backoff_s: 1.0
  auto_reconnect: true
  response_timeout_s: 2.0       # one in-flight command deadline (decision 7)
  state_timeout_s: 1.0          # heartbeat freshness threshold
  connect_wait_ms: 3000         # /kuka/eki/connect bounded wait
```

- [ ] **Step 5: CMake 增量(节点 + 安装规则)**

`kuka_eki_protocol` 源列表加 `src/eki_bridge_runtime.cpp`;追加:

```cmake
add_executable(eki_bridge_node src/eki_bridge_node.cpp)
add_dependencies(eki_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(eki_bridge_node kuka_eki_protocol ${catkin_LIBRARIES})

install(TARGETS eki_mock_server eki_bridge_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

测试段追加:

```cmake
  catkin_add_gtest(test_eki_bridge_runtime test/test_eki_bridge_runtime.cpp)
  target_link_libraries(test_eki_bridge_runtime kuka_eki_protocol
                        kuka_eki_mock ${GTEST_MAIN_LIBRARIES})
```

- [ ] **Step 6: 构建运行 + 全量回归**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/kuka_eki_bridge/test_eki_bridge_runtime
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make run_tests && \
  catkin_test_results build/test_results
```

预期:runtime `[  PASSED  ] 8 tests.`;全仓回归 **0 errors / 0 failures**(存量 171 + 本计划 81 = 252 用例;逐二进制清单见验收清单)。

- [ ] **Step 7: 手动冒烟(需要 roscore,非验收门槛)**

```bash
# terminal 1
roscore
# terminal 2: KRC mock
/home/ljj/kuka_iiqka_ros/ros_ws/devel/lib/kuka_eki_bridge/eki_mock_server --heartbeat-ms 100
# note the printed port, then in terminal 3:
rosrun kuka_eki_bridge eki_bridge_node _kuka_ip:=127.0.0.1 _eki_port:=<printed port>
# terminal 4: workflow
rosservice call /kuka/eki/connect
rosservice call /kuka/eki/start_rsi_program
rosservice call /kuka/eki/get_tool
rosservice call /kuka/eki/set_mode "mode: 2"
rosservice call /kuka/eki/stop_rsi_program
rostopic echo -n 1 /kuka/eki/state
rostopic echo -n 1 /kuka/diagnostics
```

预期:connect/start/get_tool/set_mode/stop 全部 `success: True`;`/kuka/eki/state` 显示 `state_fresh: True`、mode 2;diagnostics level 0(OK)。

- [ ] **Step 8: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/kuka_eki_bridge && \
git commit -m "feat(eki): bridge runtime + /kuka/eki service surface + diagnostics + kuka_eki.yaml (Plan 4 Task 10)"
```

---

## 验收清单

| Task | 测试二进制 | 用例数 |
|---|---|---|
| 1 | `soft_robot_msgs/test_msgs`(扩展) | +2(二进制内共 6) |
| 2 | `sri_force_torque_driver/test_sri_frame` | 12 |
| 3 | `sri_force_torque_driver/test_tcp_client_transport` | 6 |
| 4 | `sri_force_torque_driver/test_sri_stream_session` | 11 |
| 5 | `sri_force_torque_driver/test_sri_mock_server` | 6 |
| 6 | `sri_force_torque_driver/test_sri_driver_runtime` | 7 |
| 7 | `kuka_eki_bridge/test_eki_frame` | 10 |
| 8 | `kuka_eki_bridge/test_tcp_client_transport` + `test_eki_session_core` | 3 + 8 |
| 9 | `kuka_eki_bridge/test_eki_mock_server` | 8 |
| 10 | `kuka_eki_bridge/test_eki_bridge_runtime` | 8 |
| **合计** | **10 个新二进制 + 1 个扩展** | **新增 81** |

验收条件:

- `catkin_make`、`catkin_make tests` 零警告;`catkin_make run_tests` 0 errors / 0 failures(存量 171 + 新增 81 = 252)。
- 全部测试离线可跑(无 roscore、无真机;网络仅 127.0.0.1、kernel 自选端口);连续 3 次全量运行无 flaky(照 Plan 3 chain 测试标准)。
- 跟进事项逐条闭环:wrench stamp = 接收时刻且非零(T4 `StampIsReceptionTime` + T6 `StampsAreNonZeroAndMonotonic` + 节点回调注释 + 冒烟 `--offset` 步骤);`reset_fault` KRC 侧接线(T9 `FaultThenResetWorkflow` + T10 `ResetFaultThenStartSucceeds`);期望值完整推导(T2 校验和、T4 低通/tare/偏置限幅,推导写入计划任务头)。
- 规格对照:§5.6 驱动话题/服务全集(`wrench_raw`/`status`/`zero`/`set_filter`)+ 无 KUKA 特定行为;§8 数据通路(驱动发原始值,控制器拥有滤波——驱动侧 LPF 默认关);§5.2 EKI 服务/话题全集 + "不用于每周期控制"(桥无 RSI/RKorr 路径);§6.2 EKI 用途边界;§6.4 tool 查询(GET_TOOL + GetTool.srv 接线;自动编排属 Plan 5);§15.2 mock 能力全项(RSI mock 已在 Plan 2:延迟/丢包/坏帧/IPOC 跳变;本计划 EKI mock:ready/start/stop/fault/reset/tool 可配/重连/坏 XML/缺字段,SRI mock:波形/坏帧/垃圾/断链/定速)。
- 服务名/话题名与规格逐字一致:`/sri_ft/wrench_raw`、`/sri_ft/status`、`/sri_ft/zero`、`/sri_ft/set_filter`、`/kuka/eki/connect`、`/kuka/eki/start_rsi_program`、`/kuka/eki/stop_rsi_program`、`/kuka/eki/set_mode`、`/kuka/eki/reset_fault`、`/kuka/eki/set_tool_base`、`/kuka/eki/get_tool`、`/kuka/eki/state`、`/kuka/diagnostics`。
- Task 6/10 冒烟按文档可复现(手动,非门槛);真机联调时用 `rostopic echo --offset /sri_ft/wrench_raw` 抽查 stamp 与到达时刻差(Plan 3 跟进 3 的联调核对项,记入 Plan 5 联调核对单)。

## 遗留风险

1. **SRI 线协议为 M8128 默认形态假定**(待确认 1):盒端固件/通道配置不同则帧长≠27,驱动会持续计 `bad_length` 而无样本——这是显式失败(status 可见),不是静默错误;联调时若不符,改 `sri_frame.h` 常量与 mock 镜像即可,会话/runtime/节点零改动。
2. **EKI XML schema 与 KRL 模板的一致性靠 Plan 5 人工对齐**:`eki_frame.h` 是权威,但 EthernetKRL 配置(EkiConfig.xml 的 RECEIVE/SEND ELEMENT 表)是手写镜像,不一致只能在联调发现。Plan 5 编写 KRL 模板时须逐字段对照 Task 7 schema 注释。
3. **`reset_fault` 只清 KRC 侧**(待确认 10):RSI 链路 latched fault 的复位路径(`kuka_rsi_hw_interface` 节点服务)尚不存在;Plan 5 manager 编排恢复流程时必须补——且勿用 `RsiSessionMonitor::reset()`(清累计计数器,违反 RsiState.msg 注释),建议 HW 层新增 `clearFault()`。
4. **wrench stamp 为接收时刻而非真采样时刻**:SRI 帧内无时基,传输/内核缓冲抖动(loopback 微秒级、真机网络亚毫秒级)会进入 stamp;对 12 ms 级的控制器超时判定影响可忽略,但不要基于 stamp 做样本间隔统计(用 `package_gaps`)。
5. **驱动侧 tare 与控制器 AutoReTare 并存**:两者偏置语义独立(驱动偏置作用于 `wrench_raw` 本身;AutoReTare 吸收进控制器补偿器)。操作规程上,`/sri_ft/zero` 只应在非 SERVOING 时调用——manager(Plan 5)须在状态机中约束,驱动本身不感知系统状态(规格 §5.6 边界)。
6. **`execute()` 与 `stopping` 语义**:节点关闭(`ros::spin` 返回)时若有服务调用在途,`runtime.stop()` 会让其以 TIMEOUT 终态返回(实现已处理),但服务响应可能已无人消费;无害,记录备查。
7. **mock 单客户端假定**:两个 mock 同时只接受一个连接,第二个连接会一直排队在 backlog——Plan 5 集成测试若并行起多个桥/驱动实例须各配一个 mock 实例。
8. **心跳周期是 KRL 侧义务**:桥的 `state_fresh` 依赖 KRC 周期 `EKI_Send`(建议 100 ms);若 Plan 5 KRL 模板只做应答不做心跳,`state_fresh` 将在空闲期恒 false(诊断 WARN)。KRL 模板必须实现周期推送,已写入 Task 7 schema 注释与本条。
9. **TcpClientTransport 双副本漂移风险**(待确认 8):两份同构代码靠注释约定"keep in sync by hand";Plan 5/6 若再需要 TCP client,应触发抽公共包的重构而不是第三份副本。

## 计划自查记录

- **API 签名逐一核对**(对照已交付头文件/消息定义):`sfc::Wrench{fx..tz}` + `forceNorm()/torqueNorm()`(types.h);`sfc::ForceTorqueFilter(double cutoff_hz)` / `filter(const Wrench&, double dt)` / `reset()`(cutoff<=0 直通、首样本初始化——T4 推导据此);`soft_robot_msgs/GetTool.srv`(request 空;response `success/message/x/y/z/a/b/c`——节点 get_tool 逐字段填);`geometry_msgs/WrenchStamped`(`wrench.force.x..torque.z`);`std_srvs/Trigger`(`success/message`);`diagnostic_msgs/DiagnosticArray/DiagnosticStatus/KeyValue`(`level/name/message/hardware_id/values`)。Plan 2 的 `UdpTransport/RsiSessionMonitor/RsiMockServer` 仅作范式引用,本计划无代码级调用。
- **任务依赖顺序**:T1(msgs)独立;T2(sri codec)→T3(transport)→T4(session,依赖 T2)→T5(mock,依赖 T2/T3)→T6(runtime,依赖 T3/T4/T5 + T1 的 SriStatus/SetFilter);T7(eki codec+splitter)→T8(transport 副本 + session core,依赖 T7 的 EkiStateFrame)→T9(mock,依赖 T7/T8)→T10(runtime+节点,依赖 T7/T8/T9 + T1 的 EkiState/SetEkiMode/SetToolBase + Plan 2 的 GetTool)。无环;两包间无依赖(待确认 8 的目的)。
- **离线可测性**:T2/T4/T7/T8(session core)纯逻辑零 socket;T3/T5/T6/T8(transport)/T9/T10 仅 127.0.0.1 + kernel 自选端口;所有等待有界(单窗 ≤0.5 s,waitFor 上限 ≤2 s,backoff 0.05 s);唯一需要 roscore 的是 T6 Step 7 / T10 Step 7 冒烟(明确标注,非门槛)。gtest 中不使用 `ros::Time`(runtime 用 steady_clock double 秒),故连 `ros::Time::init()` 都不需要。
- **数值自查**:T2 校验和 `0xBF+0x40+0x13F=0x23E→0x3E`(canonical)与 `0x3F`(frame B),包长 27=2+24+1;T4 低通 alpha=0.5 构造(cutoff=1/(2π·0.004))⇒ 0→2 序列输出 0、1.0,tare 均值 (1+2)/2=1.5 ⇒ 2.0-1.5=0.5,偏置限幅 2.0>1.0 拒绝;T9/T10 错误码 NOT_READY=1/FAULTED=2 与行为表互查。全部期望值含中间环节(Plan 3 T4 教训:无隐藏的夹持/滤波级被漏算——本计划测试里凡滤波非直通处 cutoff 均为构造值,凡 tare 处样本为常数序列)。
- **代码逐字性**:全部新文件给全文;唯一的"复制"是 Task 8 的 `src/tcp_client_transport.cpp`(头文件已给全文),复制规则精确到行(include 一行 + namespace 开闭两行),review 以 diff 校验;无 TODO/FIXME/占位符。
- **`${GTEST_MAIN_LIBRARIES}` 覆盖**:11 条 `catkin_add_gtest` 链接行(sri 5 + eki 5 + msgs 存量 1)全部包含;新增 10 条在 Task 2/3/4/5/6/7/8(×2)/9/10 的 CMake 增量中逐条给出。
- **单位与命名**:wrench N/Nm、笛卡尔 mm/deg 贯穿(msg 注释、Frame6 注释、yaml 注释);话题/参数命名沿用 `soft_robot_controllers.yaml` 风格(snake_case、单位后缀 `_s/_ms/_hz/_n`)。
