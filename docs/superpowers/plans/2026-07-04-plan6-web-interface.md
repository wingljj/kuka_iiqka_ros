# Plan 6: `soft_robot_web_interface` Web UI + Plan 5 必闭环收口 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标:** 按规格 `docs/superpowers/specs/2026-07-02-kuka-rsi-ros-control-design.md` §5.7、§13、§15(1034/1053 行)、§16,交付浏览器操作员界面 `soft_robot_web_interface`(rosbridge_suite + 无构建链静态前端):状态总览、start/stop servo、模式/profile 切换、标定向导(feedback+结果+payload 展示)、wrench 实时曲线、zero 门控按钮、故障复位、三链路连接状态;并闭环 Plan 5 followups 两条【必闭环】:I-1(真机 RSI 活动期 EKI 心跳停发导致的确定性虚报——本计划裁决为 **EKI 语义分层**,manager 纯逻辑修复 + gtest TDD + 核对单文字补丁 + UI 呈现层正确语义)与 I-2(startServo 终裁无回滚——拒绝分支完整回滚,gtest TDD)。

**架构:** 延续"纯逻辑 + 薄壳"分层,搬到前端:`ros_client.js`(手写 rosbridge v2.0 协议子集客户端,注入 WebSocket 工厂)、`action_client.js`(actionlib topic 面客户端)、`state_model.js`(视图模型推导,含 I-1 呈现语义)、`wrench_chart.js`(环形缓冲+canvas)为纯 ES Module,`node --test`(node v20 内置)离线单测;`app.js`/`index.html` 为接线壳。后端零新增 C++:全部消费 Plan 5 已交付的 manager 服务/action/话题面(commissioning_checklist Stage 9 的原话:"the web UI consumes exactly this surface in Plan 6")。manager 的 I-1/I-2 小修落在 `SystemStateCore`/`ManagerRuntime` 纯逻辑层 + 节点壳,离线 gtest 直测。

**Plan series:** ①~⑤ 已全部合入 main(整仓 296/0/0)→ ⑥ web interface + Plan 5 必闭环收口(本计划,最后一个)。

## 范围

**包清单(1 个新 catkin 包 + 2 个存量包小修 + 文档补丁):**

| 位置 | 内容 |
|---|---|
| `soft_robot_web_interface`(新) | `www/`(index.html、css、4 个纯逻辑 JS 模块、app.js)、`test/unit/`(29 个 node:test 用例)、`test/integration/probe_rosbridge.mjs`(rosbridge 冒烟探针,复用 ros_client.js)、`launch/web.launch`、`scripts/serve_www.sh`、README |
| `soft_force_control_manager`(小修) | I-1:`SystemStateCore` EKI 语义分层(+4 gtest);I-2:`ManagerRuntime::startServo` 拒绝分支回滚(+2 gtest)+ 心跳容忍闭环(+2 gtest);节点壳:preload 已加载视为成功(Minor 7)、标定 cancel 终态 `setPreempted`(Minor 5) |
| `kuka_rsi_hw_interface`(注释)| Minor 3:`countMiss` 旧注释同步(1 行,无行为) |
| `kuka_eki_bridge`(注释) | Minor 6:transport 副本 diff 口径注释(无行为) |
| `soft_robot_msgs`(注释) | Minor 4:`CalibratePayload.action` feedback 相位注释勘误(注释不入 md5,验证步骤钉死) |
| `docs/` + `kuka/`(文字补丁) | I-1 文档面:commissioning_checklist Stage 2/3 预期改写、`ROS_RSI_CONTEXT.notes.md` 增补 Stop-S-break-MOVECORR 要求、bringup README web 一节 |

**非目标(本计划不做):**

- C++ WebSocket 后端(规格 §5.7 "Industrial deployment option":留作 rosbridge 的后续替换项,本计划明确不做)。
- Web jog(规格 §7.7 提及 stream 模式服务于 web jog):250 Hz 流经 rosbridge JSON 通道无实时性保证,且属危险操作面;§16 验收清单未要求 jog。排除,记遗留风险 1。
- 参数编辑页:manager 无参数编辑服务(危险操作必须走 manager,规格 §13 末句),v1 参数页只读(rosapi get_param + EkiState tool 展示)。排除写路径,记遗留风险 2。
- correction values 实时回显:**规格-现状冲突,以现状为准**——仓库内无任何节点发布实际 RKorr(hw 不回显命令,ModeState/RsiState 均无修正量字段)。以 `saturation_count`(RsiState)+ mode/profile(ModeState)为代理展示,真实回显需 hw 增发布器(新增 C++,超范围)。记录于"规格冲突记录"与遗留风险 3。
- KRL 模板行为改动:模板逐字不动(followup 12 备案:心跳实测 100~110 ms,裕量足)。I-1 的 KUKA 侧动作仅为 context notes 文字增补(装机实测裁决项)。
- I-1 的 hw 侧"会话结束"原语(`reset()` 语义分化/新服务):经推导(Task 2 头部),manager 侧语义分层 + start 前预清已使干净停止终局回 READY,hw 改动条件未触发。记遗留风险 4。

**接口现状核对(本计划撰写时逐一读过源码,签名以现状为准):**

- `/soft_robot/manager_state`(`ManagerState`,10 Hz):header/system_state/mode/profile/eki_connected/eki_program_ready/rsi_connected/rsi_fault/sri_streaming/tool_synced/calibrating/active_controller。**无 controllers_loaded 字段**(Minor 7 的 UI 注意点→决策 11)。
- `/soft_robot/start_servo`(`StartServo`:mode+profile→success+message)、`/soft_robot/{stop_servo,reset_fault,zero_sensor}`(`std_srvs/Trigger`)、`/soft_robot/calibrate_payload`(`CalibratePayloadAction`:goal 空;result 13 字段含 r2_force/r2_torque;feedback pose_index/pose_count/phase)。
- `/kuka/eki/state`(`EkiState`):connected 与 state_fresh **分立字段**——I-1 语义分层的消息面基础已存在,manager 此前将两者合并为 eki_link 是缺口所在。
- `/kuka/rsi/state`(`RsiState`,50 Hz):connected/fault/ipoc/total_timeouts/consecutive_timeouts/bad_frames/ipoc_jumps/saturation_count。
- `/sri_ft/wrench_raw`(`WrenchStamped`,~250 Hz)、`/sri_ft/status`(`SriStatus`,10 Hz)。
- `/soft_robot/diagnostics`(`DiagnosticArray`,1 Hz)。
- 状态数值:`ModeState.SYSTEM_*` 0~6;`ModeCommand.MODE_*` 0~3 / `PROFILE_*` 0~1(test_msgs 钉死)。
- actionlib topic 面(SimpleActionServer):`<ns>/{goal,cancel,status,feedback,result}`;GoalStatus 终态 2=PREEMPTED、3=SUCCEEDED、4=ABORTED。
- 环境实测:node v20.20.2(内置 `node:test`;`--experimental-websocket` 提供全局 WebSocket);python3.8(stdlib http.server);`ros-noetic-rosbridge-server 0.11.17` apt 候选存在、**未安装**(Task 1 处理);dev 机 sudo 需密码。

**规格冲突记录(发现于本计划撰写,均以现状为准):**

1. 规格 §5.7 建议 "Frontend: Vue or React"——与任务约束"无 node 构建链优先、离线部署"冲突,裁决原生 ES Modules(决策 1)。
2. 规格 §13/§16 "view correction values"——无发布者,代理展示(见非目标)。
3. `CalibratePayload.action` feedback 注释列 SOLVING 相位,实际 `CalPhase` 无 SOLVING、`phaseName` 也不产出(求解在 DONE 迁移内完成)——Minor 4,Task 3 勘误注释。

## Plan 5 followups 消化对照表(12 条,计划硬性要求)

`docs/superpowers/plans/2026-07-04-plan5-followups.md`,按文件编号:

| # | 条目 | 裁决 | 闭环位置 |
|---|---|---|---|
| 1 | 【必闭环】I-1 真机 RSI 活动期/停止链路跨层语义缺口 | **吸收(裁决=EKI 语义分层,PC 侧修复)** | Task 2(manager 纯逻辑 3 处 + gtest TDD +8)、Task 4(核对单/notes 文字补丁)、Task 6(UI 呈现语义 + node 测试钉死)、Task 10(sim 冒烟)。裁决推导全文见 Task 2 头部 |
| 2 | 【必闭环】I-2 startServo 终裁无回滚 | **吸收(拒绝分支完整回滚)** | Task 3 Step 1-3(gtest TDD +2) |
| 3 | countMiss 旧注释未同步 | 吸收 | Task 3 Step 4(注释 1 行) |
| 4 | 标定 action feedback 相位注释与实际不符 | 吸收 | Task 3 Step 5(action 注释勘误 + md5 不变验证) |
| 5 | 标定 preempt 用 setAborted 而非 setPreempted | 吸收 | Task 3 Step 6(节点壳;UI "已取消"呈现依赖它,决策 10);sim 冒烟验证(Task 10) |
| 6 | transport 副本 diff 口径注释 | 吸收 | Task 3 Step 7(eki 侧头注释补口径,无行为) |
| 7 | manager 单独重启后 controllers_loaded 永假 | 吸收 | Task 3 Step 8(preload 先查 list_controllers)+ 决策 11(UI 启发式提示)+ Task 10 冒烟 e2 |
| 8 | bringup README 历史段述及已删除的 switch_controller_filter | **排除**:该段明言"is gone",是准确的债务回收历史记录,无误导、无引用;Plan 6 对该 README 仅追加 web 一节,不改历史段 | — |
| 9 | Plan5 Task 5 N2 机理描述勘误 | **排除**:Plan 5 计划文档维护类,非本计划交付物;followups 文档本身已完整记录勘误内容,自证 | — |
| 10 | Plan5 Task 6 勘误 A/B/C | **排除**:同上(Plan 5 文档回写类) | — |
| 11 | Plan5 Task 10 ④ grep 双计勘误 | **排除**:同上;其正确口径("逐包 XML 直读")已进入本计划 Global Constraints | — |
| 12 | KRL 心跳实测 100~110 ms 备案 | 吸收(作为 I-1 裁决输入:模板逐字不改的依据之一) | Task 2 头部推导引用;Task 4 不触碰 KRL 心跳段 |

消化统计:**吸收 8 / 排除 4**(排除四条均为 Plan 5 文档维护类或已自证的历史记录,理由如上)。

## 待确认(规格未定项,本计划采用的默认决策)

1. **前端形态 = 原生 ES Modules,零构建链、零外部依赖**:不引 Vue/React/roslibjs/图表库/CDN。理由:目标机为控制柜旁离线工控机;SDD 要求全部文件逐字入计划(roslib.min.js 无法逐字);rosbridge v2.0 协议为简单 JSON-over-WebSocket,所需子集(subscribe/unsubscribe/advertise/publish/call_service)约 160 行可控实现。影响面:自担协议客户端质量——以 8 个 node 单测 + probe 集成探针覆盖;规格 §5.7 "Vue or React" 记规格冲突 1。
2. **rosbridge_suite 为唯一后端通道**:apt 包 `ros-noetic-rosbridge-server 0.11.17`(dev 机 apt 候选已实测存在);不写任何 C++ WebSocket 后端(规格 §5.7 工业部署选项留后续)。影响面:新增运行时依赖,Task 1 安装并把离线 deb 打包清单写入 README(决策 13)。
3. **静态托管 = python3 stdlib `http.server`**,经 `scripts/serve_www.sh` 包一层(吞掉 roslaunch 隐式 remap 参数,同 bringup run_mock.sh 先例);端口默认 http 8080 / rosbridge 9090(rosbridge 上游默认)。理由:零新依赖。影响面:无缓存/无 TLS,局域网操作台可接受,README 注明。
4. **JS 单测 = `node --test`(v20 内置)**,纯逻辑模块注入 fake WebSocket/fake now;**不计入 gtest 统计**,单独汇报(目标 29/29 pass)。理由:不造假测试——DOM/rosbridge 集成面不硬套无头浏览器(离线不可装),改用 sim.launch 冒烟清单 + 可脚本化探针(Task 10)。
5. **I-1 裁决 = EKI 语义分层(PC 侧,manager 纯逻辑)**,三条子规则:(a) RSI 通道存活(话题新鲜且 connected)时容忍 EKI 心跳停鲜,管理监督换乘 50 Hz RSI 通道;(b) 陈旧 RSI fault 闩锁(非 servo/标定请求期 + 心跳新鲜 + KRC 报 rsi_active=false)判为"上次停止的残留",掩蔽不进 FAULT;(c) startServo 在 START_RSI 前预清残留闩锁(否则 hw write() 会对新会话立即发 Stop S=1)。不改 KRL 模板(followup 12:心跳 100~110 ms 实测备案);KRC 侧配套仅为 context notes 增补"Stop S 须配置为 MOVECORR break 条件"(装机实测裁决)。失效方向核对:所有掩蔽仅在输出已为零的态生效,真故障路径(servo/标定期)全部保持进 FAULT。完整推导见 Task 2 头部。
6. **I-2 裁决 = 保留终裁位置 + 拒绝分支完整回滚**:`requestStart` 被拒后依序执行 publishMode(IDLE) → switchFiltered("", target) → (本次启动过 RSI 才)ekiStopRsi,返回失败。不选"switch 前预裁"单方案:预裁与终裁间窗口仍在,回滚才是完备闭环;两者叠加无必要(回滚已覆盖)。影响面:startServo 失败路径多 2~3 次 ops 调用,均幂等。
7. **UI 危险操作全部走 manager 服务/action**(规格 §13 末句):UI 不发布 `/soft_robot/mode_command`、不发 correction 流、不直调 `/sri_ft/zero`、不直调 `/kuka/eki/*`。按钮门控在 `state_model.buttonGates` 依 manager_state 预判,服务端裁决仍是唯一权威(按钮只是减少必然失败的调用)。
8. **wrench 曲线数据面**:`/sri_ft/wrench_raw` 以 rosbridge `throttle_rate: 50`(ms)+ `queue_length: 1` 订阅(≈20 Hz);600 点环形缓冲 ≈ 30 s 窗;canvas 2D 自绘折线(力/矩两幅,各 3 通道)。推导:250 Hz 原始流全量过 JSON 通道 ≈ 每秒 250 条文本消息,无意义;20 Hz 对人眼趋势曲线足够。
9. **标定向导经 actionlib topic 面**:goal/cancel/feedback/result 四话题(rosbridge 原生 action 支持到 ROS2 才有);goal_id 客户端生成 `web_<epoch_ms>_<rand>`,feedback/result 按 goal_id 过滤;单飞行 goal(与 SimpleActionServer 语义一致)。
10. **PREEMPTED(2) 呈现"cancelled"**:依赖 Task 3 Step 6 的 setPreempted 修复;修复前 cancel 会回 ABORTED(4)显示为 aborted——顺序上 Task 3 先于 Task 6,无窗口。
11. **controllers_loaded 不加消息字段**:`ManagerState` 增字段将改 md5 波及全链(Plan 5 刚定稿),收益仅为一个提示;UI 以启发式替代——state=CONNECTED 且 eki_program_ready/sri_streaming/tool_synced 全真时提示 "controllers not loaded yet (manager preload pending)"。配合 Task 3 Step 8(preload 修复)该态基本只在 controller_manager 未起时出现。
12. **`web.launch` 独立入口**,不 include 进 `sim.launch`/`soft_robot.launch`,也不被其 include。理由:rosbridge 未安装的机器上既有两 launch 的 roslaunch-check 必须继续通过;UI 属可选层。README 给三种组合用法。
13. **离线部署路径**:README 记录 `apt-get download ros-noetic-rosbridge-server $(apt-cache depends ...)` 的离线 deb 束制作步骤;工控机浏览器要求:支持 ES Modules 的常青浏览器(Chromium/Firefox ≥2020)。
14. **UI 文案英文**:与"代码与注释英文"约定、pendant 文案、commissioning_checklist 术语一致(操作员对照核对单执行)。
15. **stop 后宽限呈现窗 10 s(仅 UI 层)**:用户按下 Stop 后 10 s 内出现的 FAULT 呈现为"RSI session ended after stop (expected) — press Reset Fault"而非红色告警(I-1 停止链路在 KRC 侧 Stop-S-break 未配置/未实测前仍可能以 FAULT 终局);窗外 FAULT 恢复标准告警。纯呈现逻辑,进 state_model 单测。

## Global Constraints(全任务适用)

- 工作区 `/home/ljj/kuka_iiqka_ros/ros_ws`;新包 `ros_ws/src/soft_robot_web_interface/`;存量包仅做上表列明的针对性小修。子代理不做任何 git 操作;报告写 `.superpowers/sdd/plan6-task-N-report.md`。
- C++ 侧:C++14,`-Wall -Wextra` 零警告;TDD 先 RED(构建失败即失败态)后 GREEN。**Noetic `catkin_add_gtest` 不链 gtest_main → 测试链接行必含 `${GTEST_MAIN_LIBRARIES}`**;gtest 目标名一律带包前缀(既有 `test_manager_*` 目标只增用例不改名)。
- JS 侧:ES2020 模块,浏览器/`node --test` 双跑;单测禁网络禁 DOM(注入 fake);等待有界(单窗 ≤0.5 s,整测 ≤2 s)。测试命令:`cd ros_ws/src/soft_robot_web_interface && node --test test/unit/`。
- 统计口径:gtest 逐包 XML 直读(`build/test_results/<pkg>/gtest-*.xml` 求和,锚定 `<testsuites>` 根元素属性,勿元素级双计——followup 11 教训);基线 **296**,本计划完成后 **304**(soft_force_control_manager 33→41,其余包不变);node 测试单列 **29**,不并入。
- 网络测试仅 127.0.0.1;冒烟跑后零残留(`pgrep -af 'mock_server|sim_server|rosbridge|http.server'` 为空);kill 不经 subshell。
- 所有代码与注释英文;计划/报告中文。全部数值期望在任务头手工推导。
- 计划-现实冲突走三步裁决(查源头/复现/最小性);发现规范与现状冲突以现状为准并记录。
- 构建/回归命令(仓库根):

```bash
cd ros_ws && catkin_make                          # full build
cd ros_ws && catkin_make tests                    # build test binaries
cd ros_ws && catkin_make run_tests                # whole-repo regression
cd ros_ws/src/soft_robot_web_interface && node --test test/unit/   # JS tests
```

**Web 数据通路(本计划落地形态):**

```text
browser (index.html, ES modules)
  app.js
    RosClient ---- ws://host:9090 ----> rosbridge_websocket ----> ROS graph
      subscribe /soft_robot/manager_state (10 Hz)   -> state_model -> header/buttons
      subscribe /kuka/eki/state    (throttle 200ms) -> dashboard + heartbeat note
      subscribe /kuka/rsi/state    (throttle 200ms) -> dashboard + diagnostics
      subscribe /sri_ft/status     (throttle 200ms) -> dashboard
      subscribe /sri_ft/wrench_raw (throttle 50ms)  -> WrenchChart x2
      subscribe /soft_robot/diagnostics             -> diagnostics table
      call /soft_robot/start_servo|stop_servo|reset_fault|zero_sensor
      call /rosapi/get_param (read-only parameters panel)
      ActionClient /soft_robot/calibrate_payload (goal/cancel/feedback/result)
  static files <- http://host:8080 <- python3 http.server (serve_www.sh)
```

---

## File Structure

```text
ros_ws/src/soft_force_control_manager/       # Tasks 2-3 (increment)
  include/soft_force_control_manager/system_state_core.h    # +2 inputs, +2 helpers
  src/system_state_core.cpp                                 # I-1 layering
  include/soft_force_control_manager/manager_runtime.h      # (comment only)
  src/manager_runtime.cpp                                   # healthLocked split,
                                                            # start pre-clear, I-2 rollback
  src/manager_node.cpp                                      # preload fix, setPreempted
  test/test_system_state_core.cpp                           # +4 tests (12)
  test/test_manager_runtime.cpp                             # +4 tests (20)

ros_ws/src/kuka_rsi_hw_interface/src/rsi_session_monitor.cpp  # Task 3 comment
ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/tcp_client_transport.h  # Task 3 comment
ros_ws/src/soft_robot_msgs/action/CalibratePayload.action     # Task 3 comment

docs/commissioning_checklist.md              # Task 4 (Stage 2/3 text patch)
kuka/rsi/ROS_RSI_CONTEXT.notes.md            # Task 4 (+ Stop-S break item)
ros_ws/src/soft_robot_bringup/README.md      # Task 4 (+ web section pointer)

ros_ws/src/soft_robot_web_interface/         # Tasks 5-9 (new package)
  package.xml
  CMakeLists.txt
  launch/web.launch
  scripts/serve_www.sh
  www/
    index.html
    css/app.css
    js/ros_client.js                         # Task 5
    js/action_client.js                      # Task 6
    js/state_model.js                        # Task 6
    js/wrench_chart.js                       # Task 7
    js/app.js                                # Task 8
  test/
    unit/test_ros_client.test.mjs            # 8 tests
    unit/test_action_client.test.mjs         # 6 tests
    unit/test_state_model.test.mjs           # 10 tests
    unit/test_wrench_chart.test.mjs          # 5 tests
    integration/probe_rosbridge.mjs          # scripted smoke probe (Task 10)
  README.md
```

---

## Task 0: 建分支(主流程直接执行,非子代理)

- [ ] `git -C /home/ljj/kuka_iiqka_ros checkout main && git pull`(确认 HEAD 含 Plan 5 合入与 followups 提交)
- [ ] `git checkout -b feature/web-interface`
- [ ] 本计划文件随首个提交入库

**Commit 点:** `plan6: add web interface implementation plan`

---

## Task 1: 环境预检与 rosbridge 依赖安装

**目标:** 固化 Task 5~10 的环境前提;安装 `ros-noetic-rosbridge-server`;记录离线部署清单素材。**无代码交付物**,产物为报告中的核对记录。

**Steps:**

- [ ] Step 1 预检(全部记录实测输出到报告):

```bash
node --version                        # 期望 v20.x(实测 v20.20.2)
node -e "console.log(typeof (await import('node:test')))" || node --test --help >/dev/null && echo node-test-ok
node --experimental-websocket -e "console.log(typeof WebSocket)"   # 期望 function
python3 -c "import http.server; print('http.server ok')"
apt-cache policy ros-noetic-rosbridge-server                       # 候选 0.11.17
```

- [ ] Step 2 安装(dev 机需 sudo 密码——**此步请求主流程/用户执行**,子代理不持凭据;安装完成前不得进入 Task 9):

```bash
sudo apt-get install -y ros-noetic-rosbridge-server
```

- [ ] Step 3 安装后验证:

```bash
source /opt/ros/noetic/setup.bash
rospack find rosbridge_server                    # 有路径
ls $(rospack find rosbridge_server)/launch/rosbridge_websocket.launch
python3 -c "import tornado; print(tornado.version)"   # rosbridge 依赖随包带入
```

- [ ] Step 4 离线部署素材(README 用,Task 9 引用):

```bash
mkdir -p /tmp/rosbridge_debs && cd /tmp/rosbridge_debs
apt-get download ros-noetic-rosbridge-server ros-noetic-rosbridge-library \
  ros-noetic-rosapi ros-noetic-rosbridge-msgs ros-noetic-rosapi-msgs 2>&1 | tail -5
ls   # 记录 deb 清单与体积;完成后 rm -rf /tmp/rosbridge_debs(零残留)
```

若 Step 4 因镜像不含某 deb 失败:记录失败项,README 改述为"用 `apt-get install --download-only` 于同发行版机器制作离线束",不阻塞后续任务。

**验证:** Step 3 三条命令全部成功。
**Commit 点:** 无代码变更则无 commit;报告 `.superpowers/sdd/plan6-task-1-report.md`。

---

## Task 2: I-1 裁决落地 —— EKI 语义分层(SystemStateCore + ManagerRuntime)

**目标:** 闭环 followup I-1。manager 在 RSI 通道存活时容忍 EKI 心跳停鲜;把"上次 RSI 会话终止残留的陈旧 fault"与"活动会话真故障"区分开;startServo 预清残留闩锁。纯逻辑修改 + gtest TDD(+4 core,+2 runtime),零 ROS 面变更、零消息变更。

### 裁决推导(I-1 全链,评审对照用)

**缺口链(followup 原文 + 源码核对):**

1. KRL 模板 `ROS_RSI_SERVO.SRC`:START_RSI 分支内 `RSI_MOVECORR()` 阻塞 KRL 主循环 → 心跳 `ROS_SEND_STATE(0,1,0)` 停发 → 桥侧 `state_timeout_s=1.0` 判 `state_fresh=false`。
2. manager `healthLocked()`(manager_runtime.cpp 79 行):`in.eki_link = eki_fresh && eki_.connected && eki_.state_fresh` —— state_fresh=false 直接拉倒 eki_link。
3. `SystemStateCore::update`:SERVOING 中 `fullConditions` 含 `readyConditions` 含 `eki_link` → 真机 SERVOING 确定性落 DEGRADED(虚报);闲态(Stage 2 直调 start_rsi_program)落 OFFLINE。
4. 停止链路:MOVECORR 阻塞期 KRL 不读 EKI 命令 → STOP_RSI 不可达;PC 侧唯一"打断 MOVECORR"的原语是 RSI 信号流内 `Stop S=1`(hw 仅在 latched fault 时置位,见 ROS_RSI_CONTEXT.notes.md 第 3 条)与 KRC 侧断流检测;RSI 流终止后 hw 5 拍闩锁 fault(`max_consecutive_timeouts=5`)→ manager 进 FAULT。核对单 Stage 2/3 书面预期 "clean stop / clean return to READY" 与此矛盾。

**三方案裁决(followup 给出的方向):**

- 改 KRL 心跳(如移入 RSI context 旁路):**否**。RSI 6.2 context 无 EKI 发送原语,心跳只能出自 KRL 主循环;重构 KRL(如 SPS 子程序层)超出模板逐字交付形态且不可离线验证;followup 12 已备案模板逐字不改。
- 仅改核对单预期:**不充分**。文字对齐消虚假矛盾,但 SERVOING 虚报 DEGRADED 使 DEGRADED 语义失去信息量(真机上永远亮),Web UI 无法呈现正确语义——与本计划 §13 交付直接冲突。
- **EKI 语义分层(选定)**:RSI 活动期管理监督换乘 50 Hz `/kuka/rsi/state`(核对单 Stage 2 观察项本已写明此意图:"RSI-phase supervision rides the 50 Hz /kuka/rsi/state channel");EKI 心跳停鲜在 RSI 存活时不降级。配合停止链路的"陈旧 fault 掩蔽 + start 预清",干净停止终局回 READY。PC 侧纯逻辑,离线 gtest 可钉死,失效方向可逐条论证安全。

**新语义(三条子规则,即决策 5):**

- **R1(活动期容忍)**:`rsi_topic_fresh && rsi_connected` 为真时,`eki_link` 的 `state_fresh` 要求放宽为 `connected`(TCP 在即可)。理由:RSI 帧流本身是 KRL 程序活着且在 MOVECORR 的直接证据,比心跳更强。失效方向:EKI TCP 断链仍即时拉倒 eki_link;RSI 流一停(≤0.5 s)容忍即撤。
- **R2(陈旧 fault 掩蔽)**:`servo_requested_==false && !calibrating_` 且 `eki_state_fresh && !eki_rsi_active` 时,`rsi_fault` 判为上次会话残留,不触发 FAULT(仍在 ManagerState.rsi_fault 原样上报,UI 可见)。理由:心跳已恢复新鲜 + KRC 报 RSI 不活动 = KRL 已退出 MOVECORR 回到主循环,流终止是会话结束而非链路故障。失效方向:servo/标定期 rsi_fault 照旧进 FAULT;心跳未恢复(state_fresh=false)不掩蔽(信息不足,保守进 FAULT)。
- **R3(start 预清)**:startServo 在 ekiStartRsi 之前,若 hw 报 fault 且满足 R2 掩蔽条件,先调 `/kuka/rsi/reset_fault`(既有服务,Plan 5 Task 1)。理由:hw `write()` 对 latched fault 发 `Stop S=1`,不清则新 RSI 会话被立即打断。幂等,失败则 start 拒绝(fail-closed)。

**终局核对(新语义下的真机行为推演):**

| 场景 | 旧终局 | 新终局 |
|---|---|---|
| SERVOING 中 MOVECORR 心跳停发 | DEGRADED(虚报) | SERVOING(R1) |
| Stage 2 闲态 RSI 环路(直调 start_rsi_program) | OFFLINE(虚报) | READY(R1;servo_requested=false 走 idle 阶梯) |
| stop_servo(EKI STOP_RSI 不可达→失败被忽略)→流终止→hw 5 拍闩锁 | FAULT(需手动 reset) | READY(R2 掩蔽;下次 start R3 预清) |
| SERVOING 中真断网(RSI+EKI 全死) | FAULT | FAULT(R1 撤销→eki_link 依 state_fresh→rsi_fault 活动期不掩蔽) |
| SERVOING 中仅 SRI 停流 | DEGRADED | DEGRADED(不变) |
| 标定期任意链路丢失 | FAULT | FAULT(不变,R2 明确排除 calibrating) |

**File Structure:**

```text
ros_ws/src/soft_force_control_manager/
  include/soft_force_control_manager/system_state_core.h   # HealthInputs +2 字段、私有 helper +2
  src/system_state_core.cpp                                # R1/R2 落地
  src/manager_runtime.cpp                                  # healthLocked 供料 + R3 预清
  test/test_system_state_core.cpp                          # +4
  test/test_manager_runtime.cpp                            # +2
```

**Steps:**

- [ ] Step 1(RED): `test/test_system_state_core.cpp` 末尾追加 4 个用例,逐字:

```cpp

// ---- Plan 6 Task 2: EKI semantic layering (Plan 5 followup I-1) ----
// While RSI_MOVECORR blocks the KRL loop the EKI heartbeat stops by
// design (Task 9 review observation). RSI liveness supersedes the
// heartbeat for supervision during any RSI-active phase (rule R1), and
// a latched hw fault observed while idle with a fresh heartbeat and
// rsi_active=false is the residue of the previous RSI session, not a
// live failure (rule R2).

namespace {

// Real-robot SERVOING during MOVECORR: heartbeat stale, RSI stream alive.
HealthInputs servoingHeartbeatStale() {
  HealthInputs in = full();
  in.eki_heartbeat_fresh = false;   // state_fresh dropped by the bridge
  in.eki_tcp_connected = true;      // TCP still up
  in.eki_link = false;              // legacy aggregate (what R1 relaxes)
  return in;
}

}  // namespace

TEST(SystemState, ServoingToleratesStaleHeartbeatWhileRsiAlive) {
  SystemStateCore c;
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  ASSERT_TRUE(c.requestStart(start_in).accepted);
  ASSERT_EQ(c.update(full()), SystemState::SERVOING);

  // MOVECORR phase: heartbeat stale but the 50 Hz RSI channel is alive.
  EXPECT_EQ(c.update(servoingHeartbeatStale()), SystemState::SERVOING);

  // TCP loss is NOT tolerated even with RSI alive.
  HealthInputs tcp_down = servoingHeartbeatStale();
  tcp_down.eki_tcp_connected = false;
  EXPECT_EQ(c.update(tcp_down), SystemState::DEGRADED);

  // RSI stream death revokes the tolerance: nothing proves the KRL is
  // alive any more -> DEGRADED (and the hw fault path follows separately).
  HealthInputs rsi_dead = servoingHeartbeatStale();
  rsi_dead.rsi_topic_fresh = false;
  rsi_dead.rsi_connected = false;
  EXPECT_EQ(c.update(rsi_dead), SystemState::DEGRADED);
}

TEST(SystemState, IdleRsiActiveRidesRsiChannel) {
  // Commissioning Stage 2: RSI zero-output loop started by a direct
  // bridge call; the manager stays READY on the strength of the RSI
  // stream even though the heartbeat pauses during MOVECORR.
  SystemStateCore c;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  HealthInputs in = servoingHeartbeatStale();  // full() variant, no servo req
  EXPECT_EQ(c.update(in), SystemState::READY);
}

TEST(SystemState, StaleSessionFaultIsMaskedWhenIdle) {
  // Clean-stop endgame: the RSI stream ended, the hw latched its 5-miss
  // fault, the heartbeat recovered and the KRC reports rsi_active=false.
  // That fault is residue of the finished session: stay READY.
  SystemStateCore c;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  HealthInputs residue = ready();
  residue.rsi_topic_fresh = true;   // hw node alive, publishing fault=true
  residue.rsi_fault = true;
  residue.eki_heartbeat_fresh = true;
  residue.eki_rsi_active = false;   // KRC: no RSI context active
  EXPECT_EQ(c.update(residue), SystemState::READY);

  // Without heartbeat recovery there is no evidence: conservative FAULT.
  HealthInputs unknown = residue;
  unknown.eki_heartbeat_fresh = false;
  unknown.eki_link = false;
  unknown.eki_tcp_connected = true;
  EXPECT_EQ(c.update(unknown), SystemState::FAULT);
}

TEST(SystemState, ActiveSessionFaultStillLatches) {
  // R2 must never mask a fault during a requested servo or calibration.
  SystemStateCore c;
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  ASSERT_TRUE(c.requestStart(start_in).accepted);
  ASSERT_EQ(c.update(full()), SystemState::SERVOING);

  HealthInputs faulted = full();
  faulted.rsi_fault = true;
  faulted.eki_rsi_active = false;   // even if the KRC already dropped it
  EXPECT_EQ(c.update(faulted), SystemState::FAULT);
  EXPECT_EQ(c.update(full()), SystemState::FAULT);  // latched
}
```

- [ ] Step 2(RED 确认): `cd ros_ws && catkin_make tests` —— 期望编译失败(`eki_heartbeat_fresh` 等字段不存在)。RED 达成。
- [ ] Step 3(GREEN – core): `system_state_core.h` 两处修改,逐字:

`HealthInputs` 结构体替换为:

```cpp
// Health snapshot with all freshness ALREADY judged by the runtime layer
// (message age vs threshold); the core stays clock-free and testable.
struct HealthInputs {
  bool eki_link{false};         // TCP up && RobotState heartbeat fresh
  // I-1 layering inputs (Plan 6 Task 2): the raw components of eki_link,
  // plus the KRC's own view of the RSI context. During RSI_MOVECORR the
  // KRL heartbeat pauses by design, so eki_link alone cannot supervise
  // an RSI-active phase (rule R1), and a latched hw fault seen while
  // idle with a fresh heartbeat and rsi_active=false is residue of the
  // finished RSI session (rule R2).
  bool eki_tcp_connected{false};   // TCP link up (heartbeat not required)
  bool eki_heartbeat_fresh{false}; // topic fresh && msg.state_fresh
  bool eki_rsi_active{false};      // last EkiState.rsi_active
  bool eki_program_ready{false};
  bool eki_fault{false};        // KRC-side latched fault
  bool rsi_topic_fresh{false};  // /kuka/rsi/state age within threshold
  bool rsi_connected{false};    // last RsiState.connected
  bool rsi_fault{false};        // last RsiState.fault
  bool sri_streaming{false};    // /sri_ft/status fresh && streaming
  bool tool_synced{false};      // $TOOL read through EKI since (re)connect
  bool controllers_loaded{false};
};
```

私有段 `static bool fullConditions(const HealthInputs& in);` 之后追加:

```cpp
  // I-1 rules (Plan 6 Task 2). R1: with the RSI stream alive, EKI is
  // considered linked as long as TCP is up. R2: a latched hw fault is
  // session residue (maskable) only when no servo/calibration is
  // requested AND the recovered heartbeat shows rsi_active=false.
  static bool ekiLinkLayered(const HealthInputs& in);
  bool faultIsSessionResidue(const HealthInputs& in) const;
```

`system_state_core.cpp` 全文替换为:

```cpp
#include "soft_force_control_manager/system_state_core.h"

namespace sfm {

// R1 (Plan 6 Task 2, followup I-1): the KRL heartbeat pauses during
// RSI_MOVECORR, so while the 50 Hz RSI stream proves the program alive
// the EKI link requirement relaxes from "heartbeat fresh" to "TCP up".
bool SystemStateCore::ekiLinkLayered(const HealthInputs& in) {
  if (in.eki_link) return true;
  const bool rsi_alive = in.rsi_topic_fresh && in.rsi_connected;
  return rsi_alive && in.eki_tcp_connected;
}

// R2: residue of a finished RSI session. Only maskable when idle (no
// servo request, no calibration) and the recovered heartbeat reports
// rsi_active=false; anything less stays a real fault (fail-closed).
bool SystemStateCore::faultIsSessionResidue(const HealthInputs& in) const {
  if (servo_requested_ || calibrating_) return false;
  return in.eki_heartbeat_fresh && !in.eki_rsi_active;
}

bool SystemStateCore::readyConditions(const HealthInputs& in) {
  return ekiLinkLayered(in) && in.eki_program_ready && in.sri_streaming &&
         in.tool_synced && in.controllers_loaded;
}

bool SystemStateCore::fullConditions(const HealthInputs& in) {
  return readyConditions(in) && in.rsi_topic_fresh && in.rsi_connected;
}

SystemState SystemStateCore::update(const HealthInputs& in) {
  const bool live_rsi_fault = in.rsi_fault && !faultIsSessionResidue(in);
  if (in.eki_fault || live_rsi_fault) {
    state_ = SystemState::FAULT;
    servo_requested_ = false;  // a fault always demands an explicit restart
    calibrating_ = false;
    return state_;
  }
  if (state_ == SystemState::FAULT) return state_;  // latched

  if (calibrating_) {
    // Link loss mid-calibration is fatal (the robot may be far from the
    // start pose with a half-written estimate): latch FAULT.
    const bool link_ok = ekiLinkLayered(in) && in.rsi_topic_fresh &&
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

  if (!ekiLinkLayered(in)) {
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

与旧文件的差异面(评审比对锚点):新增 `ekiLinkLayered`/`faultIsSessionResidue` 两函数;`update()` 开头 fault 判定改经 `live_rsi_fault`;`readyConditions`/标定 link_ok/idle 阶梯三处 `in.eki_link` → `ekiLinkLayered(in)`。其余逐字不变。

- [ ] Step 4(GREEN – runtime 供料,三处): `manager_runtime.cpp`

(4a)`healthLocked()` 中 `in.eki_link = ...` 行之后插入:

```cpp
  // I-1 layering inputs (Plan 6 Task 2): raw components + KRC RSI view.
  // eki_.connected is the bridge's TCP view from the last message; while
  // MOVECORR gaps the heartbeat the bridge keeps publishing, so the last
  // received value stays authoritative for the TCP link.
  in.eki_tcp_connected = eki_.connected;
  in.eki_heartbeat_fresh = eki_fresh && eki_.state_fresh;
  in.eki_rsi_active = eki_.rsi_active;
```

(4b)`healthLocked()` 末尾 `return in;` 之前插入(program_ready 的旧供料被 eki_link 门死,MOVECORR 期会连带拉倒 READY 阶梯——R1 需同步分层):

```cpp
  // R1 companion: during an RSI-active heartbeat gap the last heartbeat
  // before MOVECORR reported program_ready; carry it while TCP is up and
  // the RSI stream proves the program alive.
  if (!in.eki_program_ready && in.eki_tcp_connected &&
      in.rsi_topic_fresh && in.rsi_connected)
    in.eki_program_ready = eki_.program_ready;
```

(4c)`run()` 中 `if (!in.eki_link) tool_synced_ = false;  // resync after reconnect` 替换为(否则 MOVECORR 期 tool_synced 被误清,回程后还要重同步):

```cpp
      const bool eki_gone =
          !in.eki_link && !(in.eki_tcp_connected && in.rsi_topic_fresh &&
                            in.rsi_connected);
      if (eki_gone) tool_synced_ = false;  // resync after real reconnect
```

- [ ] Step 5(RED – R3 预清): `test/test_manager_runtime.cpp` 末尾追加 2 个用例,逐字:

```cpp

// ---- Plan 6 Task 2: I-1 stop-chain endgame (rules R2 + R3) ----

TEST(ManagerRuntime, CleanStopResidueFaultKeepsReady) {
  // After stop_servo the RSI stream dies and the hw latches fault=true
  // within 5 cycles; with a fresh heartbeat and rsi_active=false the
  // manager must treat it as session residue and stay READY (R2).
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToServoing(rt, mock);
  ASSERT_TRUE(rt.stopServo().success);
  // Post-stop world: KRC ended the session, hw latched the miss fault.
  EkiFeed f = ekiUp();
  f.rsi_active = false;
  rt.feedEkiState(f);
  rt.feedRsiState(false, true);   // connected=false, fault=true (latched)
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  EXPECT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 100));
  rt.stop();
}

TEST(ManagerRuntime, StartServoPreClearsResidueFault) {
  // R3: a start over a residue-latched hw fault must call the hw reset
  // first (otherwise write() sends Stop S=1 into the new session), then
  // proceed normally.
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  rt.setControllersLoaded(true);
  EkiFeed f = ekiUp();
  f.rsi_active = false;
  rt.feedEkiState(f);
  rt.feedSriStatus(true);
  rt.feedRsiState(false, true);   // residue: hw fault latched, stream dead
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  const CommandResult r = rt.startServo(2, 0);
  EXPECT_TRUE(r.success) << r.message;
  EXPECT_TRUE(mock.logged("rsi_reset"));
  // rsi_reset must have been requested before start_rsi.
  {
    std::lock_guard<std::mutex> lock(mock.m);
    std::size_t i_reset = mock.log.size(), i_start = mock.log.size();
    for (std::size_t i = 0; i < mock.log.size(); ++i) {
      if (mock.log[i] == "rsi_reset" && i_reset == mock.log.size())
        i_reset = i;
      if (mock.log[i] == "start_rsi" && i_start == mock.log.size())
        i_start = i;
    }
    EXPECT_LT(i_reset, i_start);
  }
  rt.stop();
}
```

配套:`MockOps::ops()` 中 `o.rsiResetFault` 的 lambda 替换为(mock 侧同步撤 fault,模拟 hw `clearFault()`):

```cpp
    o.rsiResetFault = [this] {
      push("rsi_reset");
      if (rt) rt->feedRsiState(false, false);  // hw fault unlatched
      return true;
    };
```

同时 `o.ekiStartRsi` 的 lambda 替换为(START_RSI 后 KRC 报 rsi_active=true——为 Step 7 的 startServo 期望自洽):

```cpp
    o.ekiStartRsi = [this] {
      push("start_rsi");
      if (start_rsi_ok && rt) {
        rt->feedRsiState(true, false);
        EkiFeed f;
        f.connected = true;
        f.state_fresh = true;
        f.program_ready = true;
        f.rsi_active = true;
        rt->feedEkiState(f);
      }
      return start_rsi_ok;
    };
```

- [ ] Step 6(RED 确认): `catkin_make tests && ./devel/lib/soft_force_control_manager/test_manager_runtime` —— `StartServoPreClearsResidueFault` FAIL(无 rsi_reset 记录);`CleanStopResidueFaultKeepsReady` 在 Step 3/4 已 GREEN 的 core 下应 PASS(它验证的 R2 已落地——如 FAIL 按三步裁决查供料)。
- [ ] Step 7(GREEN – R3): `manager_runtime.cpp` `startServo()` 中,`bool need_start_rsi = true;` 块之前插入:

```cpp
  // R3 (Plan 6 Task 2): a residue-latched hw fault from the previous RSI
  // session must be cleared before START_RSI, or write() would answer the
  // new session with Stop S=1. Only the masked-residue case qualifies;
  // a live fault would have latched FAULT above and never reach here.
  bool need_fault_clear = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const HealthInputs in = healthLocked(nowS());
    need_fault_clear = rsi_fault_ && in.eki_heartbeat_fresh &&
                       !in.eki_rsi_active;
  }
  if (need_fault_clear) {
    if (!ops_.rsiResetFault || !ops_.rsiResetFault())
      return {false, "residual RSI fault clear failed"};
  }
```

(注:此处读 `rsi_fault_` 原始值而非 `in.rsi_fault`——后者含 topic 新鲜门,残留场景下话题是新鲜的,两者等价;用原始值使意图更直白。)

- [ ] Step 8(GREEN 确认 + 回归):

```bash
cd ros_ws && catkin_make tests
./devel/lib/soft_force_control_manager/test_manager_system_state    # 12/12
./devel/lib/soft_force_control_manager/test_manager_runtime         # 19/19(17+2)
for i in 1 2 3; do ./devel/lib/soft_force_control_manager/test_manager_runtime || echo RUN_$i_FAIL; done   # 3 连跑
catkin_make run_tests && 逐包 XML 直读                                # 302(296+6),0 fail
```

用例数推导:core 8+4=12;runtime 17+2=19;包合计 33+6=39;整仓 296+6=302。

**Commit 点:** `manager: layer EKI semantics for RSI-active phases (I-1, R1-R3)`

---

## Task 3: I-2 startServo 回滚 + Minor 3-7 逐项小修

**目标:** 闭环 followup I-2(gtest TDD +2)与 Minor 3/4/5/6/7(注释 3 处 + 节点壳 2 处)。

**File Structure:**

```text
ros_ws/src/soft_force_control_manager/src/manager_runtime.cpp   # I-2 rollback
ros_ws/src/soft_force_control_manager/test/test_manager_runtime.cpp  # +2
ros_ws/src/soft_force_control_manager/src/manager_node.cpp      # Minor 5, 7
ros_ws/src/kuka_rsi_hw_interface/src/rsi_session_monitor.cpp    # Minor 3 comment
ros_ws/src/soft_robot_msgs/action/CalibratePayload.action       # Minor 4 comment
ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/tcp_client_transport.h  # Minor 6 comment
```

**Steps:**

- [ ] Step 1(RED – I-2): `test/test_manager_runtime.cpp` 末尾追加,逐字:

```cpp

// ---- Plan 6 Task 3: I-2 startServo final-verdict rollback ----

TEST(ManagerRuntime, StartVerdictRejectionRollsBackFully) {
  // requestStart's final verdict sits after START_RSI + controller
  // switch + mode publish. If READY was lost inside that window the
  // rejection branch must undo all three: mode back to IDLE, target
  // controller stopped, RSI program stopped (it was started here).
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  // Sabotage the verdict window: the switch hook drops SRI streaming
  // right before requestStart re-checks readyConditions.
  mock.sabotage_on_switch = [&] { rt.feedSriStatus(false); };
  const CommandResult r = rt.startServo(2, 0);
  EXPECT_FALSE(r.success);
  // Rollback trail: IDLE republished after the target-mode publish,
  // the started controller stopped again, and stop_rsi issued.
  EXPECT_TRUE(mock.logged("mode:0/0"));
  EXPECT_TRUE(mock.logged("switch:/force_compliance_controller"));
  EXPECT_TRUE(mock.logged("stop_rsi"));
  // And the runtime is not left half-armed:
  const sfm::ManagerSnapshot s = rt.snapshot();
  EXPECT_TRUE(s.active_controller.empty());
  EXPECT_EQ(s.mode, 0u);
  EXPECT_NE(s.state, SystemState::SERVOING);
  rt.stop();
}

TEST(ManagerRuntime, StartVerdictRollbackSkipsStopRsiWhenPreActive) {
  // When RSI was already active before this start (rsi_active=true), the
  // rollback must NOT stop the RSI program it did not start.
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  rt.setControllersLoaded(true);
  EkiFeed f = ekiUp();
  f.rsi_active = true;            // pre-existing RSI session
  rt.feedEkiState(f);
  rt.feedSriStatus(true);
  rt.feedRsiState(true, false);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  mock.sabotage_on_switch = [&] { rt.feedSriStatus(false); };
  const CommandResult r = rt.startServo(2, 0);
  EXPECT_FALSE(r.success);
  EXPECT_FALSE(mock.logged("start_rsi"));
  EXPECT_FALSE(mock.logged("stop_rsi"));
  EXPECT_TRUE(mock.logged("switch:/force_compliance_controller"));
  rt.stop();
}
```

配套:`MockOps` 结构体成员区(`ManagerRuntime* rt = nullptr;` 行后)追加:

```cpp
  // Plan 6 Task 3 (I-2): invoked from the switch hook AFTER a successful
  // switch, to sabotage the verdict window deterministically.
  std::function<void()> sabotage_on_switch;
```

`o.switchControllers` 的 lambda 完整替换为下面这版(sabotage 必须在 mock 锁外回调 runtime,避免与 `feedSriStatus` 的锁交叉):

```cpp
    o.switchControllers = [this](const std::string& start,
                                 const std::string& stop) {
      push("switch:" + start + "/" + stop);
      std::function<void()> sab;
      {
        std::lock_guard<std::mutex> lock(m);
        if (!switch_ok) return false;
        if (strict) {  // controller_manager STRICT: no-op entries are errors
          if (!stop.empty() && running.count(stop) == 0) return false;
          if (!start.empty() && running.count(start) != 0) return false;
        }
        if (!stop.empty()) running.erase(stop);
        if (!start.empty()) running.insert(start);
        if (!start.empty()) {   // sabotage fires on the arming switch only
          sab = sabotage_on_switch;
          sabotage_on_switch = nullptr;  // one-shot
        }
      }
      if (sab) sab();   // outside the mock mutex (calls back into rt)
      return true;
    };
```

期望轨迹推导(第一用例):start→READY→startServo:`rsi_reset?`(否,无 fault)→`start_rsi`→RSI 等待成立→`mode:0/0`→`switch:force_compliance_controller/`(成功,sabotage 触发,SRI 停流)→`mode:2/0`→`requestStart`:`readyConditions` 因 `sri_streaming=false` 拒绝→回滚:`mode:0/0`→`switch:/force_compliance_controller`→`stop_rsi`→返回失败。注意 `mode:0/0` 在轨迹中出现两次,`logged()` 只验存在性,足够。

- [ ] Step 2(RED 确认): `catkin_make tests && ./devel/lib/soft_force_control_manager/test_manager_runtime` —— 两用例 FAIL(现无回滚:无第二次 switch、无 stop_rsi;snapshot 也非空 active——实际现状拒绝时 `active_controller_` 未写,但 mode 已发目标值且控制器在跑,断言 `switch:/force_compliance_controller` 与 `stop_rsi` 必 FAIL)。
- [ ] Step 3(GREEN): `manager_runtime.cpp` `startServo()` 末段(自 `std::lock_guard<std::mutex> lock(mutex_);` 起到函数尾)替换为:

```cpp
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const Verdict v = core_.requestStart(healthLocked(nowS()));
    if (v.accepted) {
      mode_ = mode;
      profile_ = profile;
      active_controller_ = target;
      return {true, ""};
    }
    // fallthrough to rollback with the reason captured
    rollback_reason_ = v.reason;
  }
  // I-2 rollback (Plan 6 Task 3): the final verdict rejected after the
  // world was already armed. Undo in reverse order; every op is
  // idempotent and best-effort (the system re-evaluates health anyway).
  if (ops_.publishMode) ops_.publishMode(kModeIdle, profile);
  switchFiltered("", target);
  if (need_start_rsi && ops_.ekiStopRsi) ops_.ekiStopRsi();
  return {false, rollback_reason_};
```

配套 `manager_runtime.h` 私有成员区(`bool cal_teardown_done_{true};` 行后)追加:

```cpp
  const char* rollback_reason_{""};  // I-2: verdict reason across unlock
```

(`need_start_rsi` 在函数前段已有定义,天然区分"本次启动的 RSI"与"预先已活动",覆盖第二用例。)

- [ ] Step 4(Minor 3): `rsi_session_monitor.cpp` 中 `stats_.fault = true;  // latched until reset()` 的注释改为 `// latched until clearFault()/reset()`。无行为。
- [ ] Step 5(Minor 4): `CalibratePayload.action` 的 feedback `phase` 注释行 `string phase                # MOVING | SETTLING | SAMPLING | SOLVING | RETURNING` 改为:

```text
string phase                # MOVING | SETTLING | SAMPLING | RETURNING
                            # (solve runs inside the DONE transition; no
                            # SOLVING phase is ever published)
```

验证 md5 不变(注释不入 md5):改前后各跑 `rosmsg md5 soft_robot_msgs/CalibratePayloadAction` 比对相同 ⇒ 零重编译依赖。若实测 md5 变化(裁决错误)则回退本步、改记 followup。
- [ ] Step 6(Minor 5): `manager_node.cpp` 标定 action 终态分支,`if (result.success) cal_server->setSucceeded(result); else cal_server->setAborted(result);` 替换为:

```cpp
              if (result.success) {
                cal_server->setSucceeded(result);
              } else if (s.cal.failure == sfm::CalFailure::CANCELLED) {
                // Minor 5 (Plan 5 followup): a preempted goal must end
                // PREEMPTED, not ABORTED — actionlib clients and the web
                // UI (Plan 6 decision 10) key "cancelled" off this code.
                cal_server->setPreempted(result);
              } else {
                cal_server->setAborted(result);
              }
```

前置核对:`CalStatus` 已含 `failure` 字段、`CalFailure::CANCELLED` 已存在(本计划撰写时读过 calibration_sequencer.h 22-41 行),`ManagerSnapshot.cal` 即 `CalStatus`,无需改纯逻辑层。
- [ ] Step 7(Minor 6): `kuka_eki_bridge/include/kuka_eki_bridge/tcp_client_transport.h` 头注释 `keep in sync by hand.` 后追加一句:

```cpp
// Normalized diff vs the sri twin: guard/include/namespace plus comment
// wording only (Plan 5 followup Minor 6); re-check both after any edit.
```

- [ ] Step 8(Minor 7): `manager_node.cpp` preload 线程体内,`if (loadController(...) && loadController(...))` 判定替换为:

```cpp
      // Minor 7 (Plan 5 followup): after a manager-only restart the
      // controllers are already loaded and load_controller fails; an
      // already-loaded controller satisfies the READY precondition, so
      // check list_controllers first and treat presence as success.
      auto loadedOrLoads = [&](const std::string& name) {
        controller_manager_msgs::ListControllers srv;
        if (cm_list.call(srv)) {
          for (const auto& c : srv.response.controller)
            if (c.name == name) return true;
        }
        return loadController(name);
      };
      if (loadedOrLoads(cfg.compliance_controller) &&
          loadedOrLoads(cfg.correction_controller)) {
```

- [ ] Step 9(GREEN 确认 + 回归):

```bash
cd ros_ws && catkin_make && catkin_make tests
./devel/lib/soft_force_control_manager/test_manager_runtime   # 21/21,3 连跑
catkin_make run_tests                                          # 整仓 304,0 fail
```

用例数推导:runtime 19+2=21;包 39+2=41;整仓 302+2=**304**。

**Commit 点:** 两个——`manager: roll back armed state on start verdict rejection (I-2)`;`manager/rsi/eki/msgs: close out Plan 5 minor followups 3-7`

---

## Task 4: I-1 文档面补丁(核对单 + KRC notes + bringup README 指针)

**目标:** 把 Task 2 的新语义写回三份文档,消除核对单 Stage 2/3 书面矛盾(followup I-1 的"装机前至少打文字补丁"要求)。纯文档,无测试。

**Steps:**

- [ ] Step 1: `docs/commissioning_checklist.md` Stage 2 的 "Task 9 review observation item" 条目(78-84 行)整条替换为:

```text
- [ ] I-1 semantic layering check (Plan 6 Task 2): while RSI is active,
      /kuka/eki/state `state_fresh` is EXPECTED to go false (the KRL
      heartbeat pauses inside RSI_MOVECORR by design). The manager now
      rides the 50 Hz /kuka/rsi/state channel during any RSI-active
      phase (rule R1), so `manager_state` must stay READY here (idle
      RSI loop, no servo requested) — record the observed state: ____.
      DEGRADED/OFFLINE during this soak is a regression of rule R1.
```

- [ ] Step 2: Stage 2 的 stop 条目(85-86 行)替换为:

```text
- [ ] `rosservice call /kuka/eki/stop_rsi_program` ends the loop. NOTE
      (I-1): while MOVECORR blocks the KRL loop this command cannot be
      served until the KRC leaves MOVECORR; if the deployment wired
      Stop S as a MOVECORR break condition (ROS_RSI_CONTEXT.notes.md
      item 3) a PC-side stop is available, otherwise stop the RSI loop
      from the pendant. After the stream ends the hw latches its 5-miss
      fault; with the heartbeat back and rsi_active=false the manager
      masks it as session residue (rule R2) and stays READY — record
      the post-stop manager state: ____ (expected READY, not FAULT).
      A later start_servo pre-clears the latched hw fault (rule R3).
```

同段末尾 "Expected:" 行的 `timeout counters flat, clean stop.` 改为 `timeout counters flat, post-stop state READY (rules R2/R3).`
- [ ] Step 3: Stage 3 末条(118-119 行)的预期文字后追加一句:

```text
      Post-stop the same R2/R3 endgame as Stage 2 applies: READY with a
      masked residual hw fault is the clean outcome.
```

- [ ] Step 4: `kuka/rsi/ROS_RSI_CONTEXT.notes.md` 第 3 条(Stop S 段)末尾追加:

```text
   COMMISSIONING DECISION (Plan 6 / followup I-1): wiring Stop S as a
   MOVECORR break condition is the only PC-side path that can end an
   RSI-active phase while the KRL EKI loop is blocked; without it the
   operator must stop from the pendant. Verify on the real cell and
   record the choice in the commissioning checklist Stage 2.
```

- [ ] Step 5: `ros_ws/src/soft_robot_bringup/README.md` 末尾追加:

```markdown

## 5. Web UI (Plan 6)

Optional browser interface: see
`ros_ws/src/soft_robot_web_interface/README.md`. Start it alongside
either entry with `roslaunch soft_robot_web_interface web.launch`;
it consumes exactly the manager surface used in Stage 9 of
`docs/commissioning_checklist.md` and adds nothing to the realtime path.
```

- [ ] Step 6 验证:`grep -n "R1\|R2\|R3" docs/commissioning_checklist.md`(≥3 处);`grep -c "clean READY" docs/commissioning_checklist.md`(0——矛盾文字已不存);Stage 2 无残留 "record the observed manager behavior (READY/DEGRADED)" 旧句。

**Commit 点:** `docs/kuka: align stop-chain expectations with I-1 layering (R1-R3)`

---

## Task 5: `soft_robot_web_interface` 包骨架 + `ros_client.js`(rosbridge 协议客户端)

**目标:** 新 catkin 包(纯资源包,无 C++ 目标);手写 rosbridge v2.0 协议子集客户端(纯 ES Module,WebSocket 工厂注入);`node --test` 单测 8 个。

**协议子集依据(rosbridge v2.0,包版 0.11.17):** 出站 op:`subscribe`(topic/type/throttle_rate/queue_length/id)、`unsubscribe`(topic/id)、`advertise`(topic/type/id)、`publish`(topic/msg)、`call_service`(service/args/id);入站 op:`publish`(topic/msg)、`service_response`(id/values/result)。`result=false` 时 `values` 为错误字符串。

**File Structure:**

```text
ros_ws/src/soft_robot_web_interface/
  package.xml
  CMakeLists.txt
  www/js/ros_client.js
  test/unit/test_ros_client.test.mjs
```

**Steps:**

- [ ] Step 1: `package.xml`,逐字:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>soft_robot_web_interface</name>
  <version>0.1.0</version>
  <description>
    Browser operator interface (spec 5.7, 13): static ES-module frontend
    served over plain HTTP, talking to the ROS graph through
    rosbridge_websocket. No build chain, no external JS dependencies;
    all dangerous operations go through the manager services/actions.
  </description>
  <maintainer email="dev@example.com">Soft Robot Team</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <!-- exec-only: the package ships static files and a launch file. -->
  <exec_depend>rosbridge_server</exec_depend>
  <exec_depend>soft_robot_msgs</exec_depend>
</package>
```

- [ ] Step 2: `CMakeLists.txt`,逐字:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(soft_robot_web_interface)

find_package(catkin REQUIRED)
catkin_package()

# Static frontend + launch + serving shim. No compiled targets; the JS
# unit tests run through `node --test test/unit/` (not part of gtest).
install(DIRECTORY www launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/serve_www.sh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
```

- [ ] Step 3(RED): `test/unit/test_ros_client.test.mjs`,逐字:

```js
// node --test unit tests for the rosbridge protocol client. A FakeSocket
// stands in for WebSocket: tests inspect the frames the client sends and
// inject inbound frames; no network, no timers left running.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RosClient } from '../../www/js/ros_client.js';

class FakeSocket {
  constructor() {
    this.sent = [];
    this.readyState = 0; // CONNECTING
    this.onopen = null; this.onclose = null;
    this.onerror = null; this.onmessage = null;
  }
  send(text) { this.sent.push(JSON.parse(text)); }
  close() {
    this.readyState = 3;
    if (this.onclose) this.onclose({});
  }
  // test helpers
  open() { this.readyState = 1; if (this.onopen) this.onopen({}); }
  inject(obj) {
    if (this.onmessage) this.onmessage({ data: JSON.stringify(obj) });
  }
}

function makeClient() {
  const sockets = [];
  const timers = [];
  const client = new RosClient('ws://test:9090', () => {
    const s = new FakeSocket();
    sockets.push(s);
    return s;
  }, {
    reconnectMs: 10,
    setTimeoutFn: (fn, ms) => { timers.push({ fn, ms }); return timers.length; },
    clearTimeoutFn: () => {},
  });
  return { client, sockets, timers };
}

test('connect reports status and flushes queued operations', () => {
  const { client, sockets } = makeClient();
  const status = [];
  client.onStatus = (up) => status.push(up);
  client.subscribe('/soft_robot/manager_state',
                   'soft_robot_msgs/ManagerState', () => {});
  client.connect();
  assert.equal(sockets.length, 1);
  assert.equal(sockets[0].sent.length, 0);   // nothing before open
  sockets[0].open();
  assert.deepEqual(status, [true]);
  const sub = sockets[0].sent.find((m) => m.op === 'subscribe');
  assert.ok(sub);
  assert.equal(sub.topic, '/soft_robot/manager_state');
  assert.equal(sub.type, 'soft_robot_msgs/ManagerState');
});

test('subscribe options map to throttle_rate and queue_length', () => {
  const { client, sockets } = makeClient();
  client.connect();
  sockets[0].open();
  client.subscribe('/sri_ft/wrench_raw', 'geometry_msgs/WrenchStamped',
                   () => {}, { throttleMs: 50, queueLength: 1 });
  const sub = sockets[0].sent.find((m) => m.topic === '/sri_ft/wrench_raw');
  assert.equal(sub.throttle_rate, 50);
  assert.equal(sub.queue_length, 1);
});

test('inbound publish frames fan out to topic callbacks', () => {
  const { client, sockets } = makeClient();
  const got = [];
  client.connect();
  sockets[0].open();
  client.subscribe('/kuka/rsi/state', 'soft_robot_msgs/RsiState',
                   (m) => got.push(m));
  sockets[0].inject({ op: 'publish', topic: '/kuka/rsi/state',
                      msg: { connected: true, ipoc: 7 } });
  sockets[0].inject({ op: 'publish', topic: '/other', msg: {} });
  assert.equal(got.length, 1);
  assert.equal(got[0].ipoc, 7);
});

test('callService resolves on matching service_response', async () => {
  const { client, sockets } = makeClient();
  client.connect();
  sockets[0].open();
  const p = client.callService('/soft_robot/stop_servo', {});
  const call = sockets[0].sent.find((m) => m.op === 'call_service');
  assert.ok(call.id);
  sockets[0].inject({ op: 'service_response', id: call.id,
                      result: true, values: { success: true, message: '' } });
  const res = await p;
  assert.equal(res.success, true);
});

test('callService rejects on result=false', async () => {
  const { client, sockets } = makeClient();
  client.connect();
  sockets[0].open();
  const p = client.callService('/soft_robot/start_servo',
                               { mode: 2, profile: 0 });
  const call = sockets[0].sent.find((m) => m.op === 'call_service');
  sockets[0].inject({ op: 'service_response', id: call.id,
                      result: false, values: 'service failed' });
  await assert.rejects(p, /service failed/);
});

test('publish sends advertise once then publish frames', () => {
  const { client, sockets } = makeClient();
  client.connect();
  sockets[0].open();
  client.publish('/x', 'std_msgs/Empty', {});
  client.publish('/x', 'std_msgs/Empty', {});
  const adv = sockets[0].sent.filter((m) => m.op === 'advertise');
  const pub = sockets[0].sent.filter((m) => m.op === 'publish');
  assert.equal(adv.length, 1);
  assert.equal(pub.length, 2);
});

test('socket loss schedules reconnect and re-subscribes', () => {
  const { client, sockets, timers } = makeClient();
  const status = [];
  client.onStatus = (up) => status.push(up);
  client.connect();
  sockets[0].open();
  client.subscribe('/kuka/eki/state', 'soft_robot_msgs/EkiState', () => {});
  sockets[0].close();                       // dropped
  assert.deepEqual(status, [true, false]);
  assert.equal(timers.length >= 1, true);   // reconnect scheduled
  timers[timers.length - 1].fn();           // fire it
  assert.equal(sockets.length, 2);
  sockets[1].open();
  const sub = sockets[1].sent.find((m) => m.op === 'subscribe');
  assert.equal(sub.topic, '/kuka/eki/state');
  assert.deepEqual(status, [true, false, true]);
});

test('close() is final: no reconnect after explicit close', () => {
  const { client, sockets, timers } = makeClient();
  client.connect();
  sockets[0].open();
  const before = timers.length;
  client.close();
  assert.equal(timers.length, before);      // nothing scheduled
  assert.equal(sockets.length, 1);
});
```

- [ ] Step 4(RED 确认): `cd ros_ws/src/soft_robot_web_interface && node --test test/unit/` —— 期望 8 fail(模块不存在即失败态)。
- [ ] Step 5(GREEN): `www/js/ros_client.js`,逐字:

```js
// Minimal rosbridge v2.0 protocol client (Plan 6 decision 1: no external
// dependencies; this subset — subscribe/unsubscribe/advertise/publish/
// call_service — is everything the operator UI needs). Pure logic: the
// WebSocket implementation and timer functions are injected so node:test
// drives it with fakes. Reconnects forever with a fixed bounded delay;
// close() is final. Not a general roslib replacement.
export class RosClient {
  constructor(url, wsFactory, opts = {}) {
    this._url = url;
    this._wsFactory = wsFactory;
    this._reconnectMs = opts.reconnectMs ?? 1000;
    this._callTimeoutMs = opts.callTimeoutMs ?? 3000;
    this._setTimeout = opts.setTimeoutFn ?? ((fn, ms) => setTimeout(fn, ms));
    this._clearTimeout = opts.clearTimeoutFn ?? ((h) => clearTimeout(h));
    this._ws = null;
    this._up = false;
    this._closed = false;
    this._nextId = 1;
    this._subs = new Map();       // topic -> {type, opts, cbs:Set}
    this._advertised = new Set(); // topics advertised on current socket
    this._pending = new Map();    // call id -> {resolve, reject, timer}
    this.onStatus = null;         // (connected: boolean) => void
  }

  connect() {
    if (this._closed || this._ws) return;
    const ws = this._wsFactory(this._url);
    this._ws = ws;
    ws.onopen = () => {
      this._up = true;
      this._advertised.clear();
      for (const [topic, s] of this._subs) this._sendSubscribe(topic, s);
      if (this.onStatus) this.onStatus(true);
    };
    ws.onclose = () => this._dropped();
    ws.onerror = () => {};        // onclose always follows
    ws.onmessage = (ev) => this._handle(ev.data);
  }

  close() {
    this._closed = true;
    const ws = this._ws;
    this._ws = null;              // _dropped() sees the explicit close
    if (ws) ws.close();
  }

  _dropped() {
    const wasUp = this._up;
    this._up = false;
    this._ws = null;
    for (const [, p] of this._pending) {
      this._clearTimeout(p.timer);
      p.reject(new Error('rosbridge connection lost'));
    }
    this._pending.clear();
    if (wasUp && this.onStatus) this.onStatus(false);
    if (!this._closed) {
      this._setTimeout(() => this.connect(), this._reconnectMs);
    }
  }

  _send(obj) {
    if (this._up && this._ws) this._ws.send(JSON.stringify(obj));
  }

  _sendSubscribe(topic, s) {
    const frame = { op: 'subscribe', id: 'sub:' + topic, topic, type: s.type };
    if (s.opts.throttleMs !== undefined) frame.throttle_rate = s.opts.throttleMs;
    if (s.opts.queueLength !== undefined) frame.queue_length = s.opts.queueLength;
    this._send(frame);
  }

  subscribe(topic, type, cb, opts = {}) {
    let s = this._subs.get(topic);
    if (!s) {
      s = { type, opts, cbs: new Set() };
      this._subs.set(topic, s);
      this._sendSubscribe(topic, s);
    }
    s.cbs.add(cb);
  }

  unsubscribe(topic, cb) {
    const s = this._subs.get(topic);
    if (!s) return;
    s.cbs.delete(cb);
    if (s.cbs.size === 0) {
      this._subs.delete(topic);
      this._send({ op: 'unsubscribe', id: 'sub:' + topic, topic });
    }
  }

  publish(topic, type, msg) {
    if (!this._advertised.has(topic)) {
      this._advertised.add(topic);
      this._send({ op: 'advertise', id: 'adv:' + topic, topic, type });
    }
    this._send({ op: 'publish', topic, msg });
  }

  callService(service, args = {}) {
    const id = 'call:' + service + ':' + this._nextId++;
    return new Promise((resolve, reject) => {
      const timer = this._setTimeout(() => {
        this._pending.delete(id);
        reject(new Error('service call timed out: ' + service));
      }, this._callTimeoutMs);
      this._pending.set(id, { resolve, reject, timer });
      this._send({ op: 'call_service', id, service, args });
    });
  }

  _handle(text) {
    let m;
    try { m = JSON.parse(text); } catch { return; }
    if (m.op === 'publish') {
      const s = this._subs.get(m.topic);
      if (s) for (const cb of s.cbs) cb(m.msg);
    } else if (m.op === 'service_response') {
      const p = this._pending.get(m.id);
      if (!p) return;
      this._pending.delete(m.id);
      this._clearTimeout(p.timer);
      if (m.result) p.resolve(m.values ?? {});
      else p.reject(new Error(String(m.values ?? 'service call failed')));
    }
  }
}
```

- [ ] Step 6(GREEN 确认): `node --test test/unit/` —— 8/8 pass,3 连跑。
- [ ] Step 7: `cd ros_ws && catkin_make`(新包入构建,无目标,应零警告通过);`rospack find soft_robot_web_interface` 有路径。

**验证:** node 8/8;catkin_make 通过;gtest 面无变化(304)。
**Commit 点:** `web: package skeleton + rosbridge protocol client (8 node tests)`

---

## Task 6: `state_model.js` + `action_client.js`(视图模型 + 标定 action 客户端)

**目标:** 纯逻辑视图模型(状态名、I-1 呈现语义、按钮门控、停止宽限窗、controllers 启发式)与 actionlib topic 面客户端;node 单测 10+6。

**File Structure:**

```text
ros_ws/src/soft_robot_web_interface/
  www/js/state_model.js
  www/js/action_client.js
  test/unit/test_state_model.test.mjs
  test/unit/test_action_client.test.mjs
```

**Steps:**

- [ ] Step 1(RED): `test/unit/test_state_model.test.mjs`,逐字:

```js
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  STATE, STATE_NAMES, MODE_NAMES, PROFILE_NAMES,
  linkBadges, buttonGates, presentState, controllersHint, calPhaseLabel,
} from '../../www/js/state_model.js';

function mgr(over = {}) {
  return Object.assign({
    system_state: STATE.READY, mode: 0, profile: 0,
    eki_connected: true, eki_program_ready: true,
    rsi_connected: false, rsi_fault: false,
    sri_streaming: true, tool_synced: true,
    calibrating: false, active_controller: '',
  }, over);
}

test('name tables cover the full wire ranges', () => {
  assert.equal(STATE_NAMES.length, 7);
  assert.equal(STATE_NAMES[0], 'OFFLINE');
  assert.equal(STATE_NAMES[6], 'FAULT');
  assert.deepEqual(MODE_NAMES,
    ['IDLE', 'DIRECT_CARTESIAN', 'FORCE_COMPLIANCE', 'CALIBRATION']);
  assert.deepEqual(PROFILE_NAMES, ['DRAG', 'PRECISION']);
});

test('linkBadges reflects the three links', () => {
  const b = linkBadges(mgr(), { connected: true, state_fresh: true,
                               rsi_active: false });
  assert.equal(b.eki.ok, true);
  assert.equal(b.rsi.ok, false);
  assert.equal(b.sri.ok, true);
});

test('I-1 presentation: heartbeat pause during RSI phase is expected', () => {
  // EkiState.state_fresh false + manager rsi_connected true = MOVECORR
  // heartbeat pause (rule R1): informational, never an error badge.
  const b = linkBadges(mgr({ rsi_connected: true }),
                       { connected: true, state_fresh: false,
                         rsi_active: true });
  assert.equal(b.eki.ok, true);
  assert.match(b.eki.note, /heartbeat paused/i);
  // Without RSI alive the stale heartbeat IS a problem.
  const b2 = linkBadges(mgr({ eki_connected: false }),
                        { connected: true, state_fresh: false,
                          rsi_active: false });
  assert.equal(b2.eki.ok, false);
});

test('buttonGates in READY', () => {
  const g = buttonGates(mgr());
  assert.deepEqual(g, { startServo: true, stopServo: false,
                        resetFault: false, zeroSensor: true,
                        calibrate: true });
});

test('buttonGates in SERVOING and DEGRADED', () => {
  const s = buttonGates(mgr({ system_state: STATE.SERVOING }));
  assert.equal(s.startServo, false);
  assert.equal(s.stopServo, true);
  assert.equal(s.zeroSensor, false);   // decision 11 gate mirrored
  assert.equal(s.calibrate, false);
  const d = buttonGates(mgr({ system_state: STATE.DEGRADED }));
  assert.equal(d.stopServo, true);
});

test('buttonGates in FAULT and CALIBRATING', () => {
  const f = buttonGates(mgr({ system_state: STATE.FAULT }));
  assert.deepEqual(f, { startServo: false, stopServo: false,
                        resetFault: true, zeroSensor: false,
                        calibrate: false });
  const c = buttonGates(mgr({ system_state: STATE.CALIBRATING }));
  assert.equal(c.calibrate, false);    // cancel goes through the action
  assert.equal(c.zeroSensor, false);
});

test('zeroSensor also open in CONNECTED (decision 11)', () => {
  const g = buttonGates(mgr({ system_state: STATE.CONNECTED }));
  assert.equal(g.zeroSensor, true);
  assert.equal(g.startServo, false);
});

test('presentState grace window after stop (decision 15)', () => {
  const now = 100000;
  const f = mgr({ system_state: STATE.FAULT, rsi_fault: true });
  const inWindow = presentState(f, now - 5000, now);
  assert.equal(inWindow.level, 'info');
  assert.match(inWindow.text, /after stop/i);
  const outWindow = presentState(f, now - 15000, now);
  assert.equal(outWindow.level, 'error');
  const neverStopped = presentState(f, null, now);
  assert.equal(neverStopped.level, 'error');
});

test('presentState normal levels', () => {
  assert.equal(presentState(mgr(), null, 0).level, 'ok');
  assert.equal(presentState(mgr({ system_state: STATE.DEGRADED }),
                            null, 0).level, 'warn');
  assert.equal(presentState(mgr({ system_state: STATE.SERVOING }),
                            null, 0).level, 'ok');
});

test('controllersHint heuristic (decision 11 / followup Minor 7)', () => {
  // CONNECTED with every visible READY precondition true -> the missing
  // one is controllers_loaded (not a ManagerState field).
  const hinted = controllersHint(mgr({ system_state: STATE.CONNECTED }));
  assert.match(hinted, /controllers/i);
  assert.equal(controllersHint(mgr()), '');           // READY: no hint
  assert.equal(controllersHint(mgr({ system_state: STATE.CONNECTED,
                                     tool_synced: false })), '');
  assert.equal(calPhaseLabel('SAMPLING', 3, 8), 'SAMPLING pose 4/8');
});
```

- [ ] Step 2(RED): `test/unit/test_action_client.test.mjs`,逐字:

```js
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ActionClient, GOAL_STATUS } from '../../www/js/action_client.js';

// Fake RosClient: records publishes, exposes subscribe callbacks.
class FakeRos {
  constructor() { this.pubs = []; this.subs = new Map(); }
  publish(topic, type, msg) { this.pubs.push({ topic, type, msg }); }
  subscribe(topic, type, cb) { this.subs.set(topic, cb); }
  unsubscribe() {}
  inject(topic, msg) { this.subs.get(topic)(msg); }
}

const NS = '/soft_robot/calibrate_payload';
const TYPE = 'soft_robot_msgs/CalibratePayload';

test('sendGoal publishes an ActionGoal with a fresh goal_id', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 42 });
  ac.sendGoal({});
  const g = ros.pubs.find((p) => p.topic === NS + '/goal');
  assert.ok(g);
  assert.equal(g.type, TYPE + 'ActionGoal');
  assert.match(g.msg.goal_id.id, /^web_42_/);
  assert.deepEqual(g.msg.goal, {});
});

test('feedback for the active goal fans out, others filtered', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  const fbs = [];
  ac.onFeedback = (fb) => fbs.push(fb);
  ac.sendGoal({});
  const id = ac.activeGoalId();
  ros.inject(NS + '/feedback', { status: { goal_id: { id } },
                                 feedback: { phase: 'MOVING', pose_index: 0 } });
  ros.inject(NS + '/feedback', { status: { goal_id: { id: 'other' } },
                                 feedback: { phase: 'X' } });
  assert.equal(fbs.length, 1);
  assert.equal(fbs[0].phase, 'MOVING');
});

test('result resolves with terminal status and payload', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  const results = [];
  ac.onResult = (status, res) => results.push({ status, res });
  ac.sendGoal({});
  const id = ac.activeGoalId();
  ros.inject(NS + '/result', {
    status: { goal_id: { id }, status: GOAL_STATUS.SUCCEEDED },
    result: { success: true, gravity_n: 42.5, r2_force: 1.0 },
  });
  assert.equal(results.length, 1);
  assert.equal(results[0].status, GOAL_STATUS.SUCCEEDED);
  assert.equal(results[0].res.gravity_n, 42.5);
  assert.equal(ac.activeGoalId(), null);   // terminal clears the goal
});

test('cancel publishes GoalID for the active goal', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  ac.sendGoal({});
  const id = ac.activeGoalId();
  ac.cancel();
  const c = ros.pubs.find((p) => p.topic === NS + '/cancel');
  assert.equal(c.type, 'actionlib_msgs/GoalID');
  assert.equal(c.msg.id, id);
});

test('PREEMPTED terminal status is distinguishable (decision 10)', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  const results = [];
  ac.onResult = (status) => results.push(status);
  ac.sendGoal({});
  const id = ac.activeGoalId();
  ros.inject(NS + '/result', {
    status: { goal_id: { id }, status: GOAL_STATUS.PREEMPTED },
    result: { success: false },
  });
  assert.deepEqual(results, [GOAL_STATUS.PREEMPTED]);
});

test('second sendGoal while active is refused (single in-flight)', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  assert.equal(ac.sendGoal({}), true);
  assert.equal(ac.sendGoal({}), false);
  const goals = ros.pubs.filter((p) => p.topic === NS + '/goal');
  assert.equal(goals.length, 1);
});
```

- [ ] Step 3(RED 确认): `node --test test/unit/` —— 8 pass + 16 fail。
- [ ] Step 4(GREEN): `www/js/state_model.js`,逐字:

```js
// Pure view-model derivations for the operator UI. No DOM, no network:
// everything here is testable with plain objects (node:test). Wire
// values mirror soft_robot_msgs (test_msgs.cpp pins them on the C++
// side; test_state_model pins the copies here).
export const STATE = Object.freeze({
  OFFLINE: 0, CONNECTED: 1, READY: 2, SERVOING: 3,
  CALIBRATING: 4, DEGRADED: 5, FAULT: 6,
});
export const STATE_NAMES = Object.freeze(['OFFLINE', 'CONNECTED', 'READY',
  'SERVOING', 'CALIBRATING', 'DEGRADED', 'FAULT']);
export const MODE_NAMES = Object.freeze(['IDLE', 'DIRECT_CARTESIAN',
  'FORCE_COMPLIANCE', 'CALIBRATION']);
export const PROFILE_NAMES = Object.freeze(['DRAG', 'PRECISION']);

// Grace window (decision 15): a FAULT appearing this soon after the
// operator's own stop is the expected I-1 stop-chain endgame on cells
// without a Stop-S MOVECORR break, not an alarm.
export const STOP_GRACE_MS = 10000;

// Three-link badges. I-1 presentation rule: a stale EKI heartbeat while
// the RSI stream is alive is the documented MOVECORR pause (rule R1) —
// show the EKI link as OK with an informational note, never as an error.
export function linkBadges(m, eki) {
  const rsiAlive = !!m.rsi_connected;
  const hbStale = !!(eki && eki.connected && !eki.state_fresh);
  const ekiOk = !!m.eki_connected || (hbStale && rsiAlive);
  return {
    eki: {
      ok: ekiOk,
      note: (hbStale && rsiAlive)
        ? 'heartbeat paused (RSI active, expected)' : '',
    },
    rsi: { ok: rsiAlive, note: m.rsi_fault ? 'fault latched' : '' },
    sri: { ok: !!m.sri_streaming, note: '' },
  };
}

// Client-side pre-judgement of the manager gates. The manager remains
// the sole authority (spec 13: dangerous operations go through manager
// services with state checks); these only suppress calls that would
// certainly be rejected.
export function buttonGates(m) {
  const s = m.system_state;
  return {
    startServo: s === STATE.READY,
    stopServo: s === STATE.SERVOING || s === STATE.DEGRADED,
    resetFault: s === STATE.FAULT,
    zeroSensor: s === STATE.CONNECTED || s === STATE.READY, // decision 11
    calibrate: s === STATE.READY,
  };
}

// Header banner: level 'ok' | 'info' | 'warn' | 'error' + text.
export function presentState(m, lastStopAtMs, nowMs) {
  const s = m.system_state;
  if (s === STATE.FAULT) {
    if (lastStopAtMs !== null && lastStopAtMs !== undefined &&
        nowMs - lastStopAtMs <= STOP_GRACE_MS) {
      return { level: 'info',
               text: 'RSI session ended after stop (expected) — ' +
                     'press Reset Fault' };
    }
    return { level: 'error', text: 'FAULT latched: reset required' };
  }
  if (s === STATE.DEGRADED) {
    return { level: 'warn', text: 'DEGRADED: output forced to zero' };
  }
  return { level: 'ok', text: STATE_NAMES[s] ?? ('state ' + s) };
}

// Decision 11 heuristic for the missing controllers_loaded field: stuck
// in CONNECTED with every visible READY precondition true means the
// invisible one (controller preload) is the blocker.
export function controllersHint(m) {
  if (m.system_state !== STATE.CONNECTED) return '';
  if (m.eki_connected && m.eki_program_ready && m.sri_streaming &&
      m.tool_synced) {
    return 'controllers not loaded yet (manager preload pending — is ' +
           'the controller_manager up?)';
  }
  return '';
}

export function calPhaseLabel(phase, poseIndex, poseCount) {
  return phase + ' pose ' + (poseIndex + 1) + '/' + poseCount;
}
```

- [ ] Step 5(GREEN): `www/js/action_client.js`,逐字:

```js
// actionlib client over the rosbridge topic surface (goal/cancel/
// feedback/result). rosbridge 0.11 has no first-class ROS1 action op,
// so this speaks the SimpleActionServer wire protocol directly.
// Single in-flight goal (matches the server's SimpleActionServer
// semantics). Injected clock for tests.
export const GOAL_STATUS = Object.freeze({
  PENDING: 0, ACTIVE: 1, PREEMPTED: 2, SUCCEEDED: 3, ABORTED: 4,
});

export class ActionClient {
  constructor(ros, ns, actionType, opts = {}) {
    this._ros = ros;
    this._ns = ns;
    this._type = actionType;
    this._nowMs = opts.nowMs ?? (() => Date.now());
    this._goalId = null;
    this.onFeedback = null;   // (feedback) => void
    this.onResult = null;     // (status, result) => void
    ros.subscribe(ns + '/feedback', actionType + 'ActionFeedback', (m) => {
      if (this._goalId && m.status.goal_id.id === this._goalId &&
          this.onFeedback) {
        this.onFeedback(m.feedback);
      }
    });
    ros.subscribe(ns + '/result', actionType + 'ActionResult', (m) => {
      if (!this._goalId || m.status.goal_id.id !== this._goalId) return;
      this._goalId = null;
      if (this.onResult) this.onResult(m.status.status, m.result);
    });
  }

  activeGoalId() { return this._goalId; }

  sendGoal(goal) {
    if (this._goalId) return false;   // single in-flight goal
    const id = 'web_' + this._nowMs() + '_' +
               Math.floor(Math.random() * 0xffff).toString(16);
    this._goalId = id;
    this._ros.publish(this._ns + '/goal', this._type + 'ActionGoal', {
      goal_id: { stamp: { secs: 0, nsecs: 0 }, id },
      goal,
    });
    return true;
  }

  cancel() {
    if (!this._goalId) return;
    this._ros.publish(this._ns + '/cancel', 'actionlib_msgs/GoalID',
                      { stamp: { secs: 0, nsecs: 0 }, id: this._goalId });
  }
}
```

- [ ] Step 6(GREEN 确认): `node --test test/unit/` —— 24/24 pass,3 连跑。

**验证:** node 24/24;gtest 面无变化。
**Commit 点:** `web: state model + actionlib client (16 node tests)`

---

## Task 7: `wrench_chart.js`(环形缓冲 + canvas 趋势曲线)

**目标:** 纯逻辑数据模型(环形缓冲、极值、抽稀映射)与 canvas 绘制分离;node 单测 5 个(只测数据模型,绘制函数以"接受任何实现 2D context 面的对象"设计、冒烟里人工看)。

**数值推导:** 决策 8:throttle 50 ms ⇒ ~20 Hz;容量 600 点 ⇒ 600/20 = **30 s** 窗。`yRange` 对称扩展:数据极值 |max|=7.3 ⇒ ceil(7.3/5)*5 = ±10(5 的倍数取整,坐标轴整洁);全零数据退化 ⇒ 默认 ±5。

**File Structure:**

```text
ros_ws/src/soft_robot_web_interface/
  www/js/wrench_chart.js
  test/unit/test_wrench_chart.test.mjs
```

**Steps:**

- [ ] Step 1(RED): `test/unit/test_wrench_chart.test.mjs`,逐字:

```js
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { TraceBuffer, yRange, toPolyline } from '../../www/js/wrench_chart.js';

test('TraceBuffer keeps the last capacity samples in order', () => {
  const b = new TraceBuffer(3, 4);           // 3 channels, capacity 4
  for (let i = 1; i <= 6; i += 1) b.push([i, i * 10, i * 100]);
  assert.equal(b.length, 4);
  assert.deepEqual(b.channel(0), [3, 4, 5, 6]);
  assert.deepEqual(b.channel(2), [300, 400, 500, 600]);
});

test('TraceBuffer clear resets to empty', () => {
  const b = new TraceBuffer(2, 8);
  b.push([1, 2]);
  b.clear();
  assert.equal(b.length, 0);
  assert.deepEqual(b.channel(0), []);
});

test('yRange expands to a symmetric multiple of 5', () => {
  assert.deepEqual(yRange([[0.5, -7.3, 2.0]]), [-10, 10]);
  assert.deepEqual(yRange([[1, 2], [-4.9, 0]]), [-5, 5]);
  assert.deepEqual(yRange([[12.1]]), [-15, 15]);
});

test('yRange degenerates to ±5 on empty/zero data', () => {
  assert.deepEqual(yRange([]), [-5, 5]);
  assert.deepEqual(yRange([[0, 0, 0]]), [-5, 5]);
});

test('toPolyline maps samples into pixel space', () => {
  // width 100, height 50, range [-10, 10]: value +10 -> y 0,
  // value -10 -> y 50, value 0 -> y 25. Four points span x 0..100.
  const pts = toPolyline([-10, 0, 10, 0], 100, 50, [-10, 10]);
  assert.deepEqual(pts, [[0, 50], [100 / 3, 25], [200 / 3, 0], [100, 25]]);
});
```

- [ ] Step 2(RED 确认): `node --test test/unit/` —— 24 pass + 5 fail。
- [ ] Step 3(GREEN): `www/js/wrench_chart.js`,逐字:

```js
// Wrench trend chart split into a pure data model (ring buffer + scaling,
// node-tested) and a thin canvas painter (exercised by the sim smoke).
// Decision 8: ~20 Hz throttled samples, capacity 600 = a 30 s window.

export class TraceBuffer {
  constructor(channels, capacity) {
    this._n = channels;
    this._cap = capacity;
    this._data = [];            // array of sample arrays, oldest first
  }
  get length() { return this._data.length; }
  push(sample) {
    this._data.push(sample.slice(0, this._n));
    if (this._data.length > this._cap) this._data.shift();
  }
  clear() { this._data = []; }
  channel(i) { return this._data.map((s) => s[i]); }
}

// Symmetric range snapped up to a multiple of 5 so axis labels stay
// clean; ±5 floor for flat/empty data.
export function yRange(samples) {
  let peak = 0;
  for (const s of samples) {
    for (const v of s) peak = Math.max(peak, Math.abs(v));
  }
  const m = Math.max(5, Math.ceil(peak / 5) * 5);
  return [-m, m];
}

// Maps a single channel into pixel coordinates (x spread over width,
// y inverted: +range at the top).
export function toPolyline(values, width, height, range) {
  const n = values.length;
  if (n === 0) return [];
  const [lo, hi] = range;
  return values.map((v, i) => [
    n === 1 ? 0 : (i * width) / (n - 1),
    height - ((v - lo) / (hi - lo)) * height,
  ]);
}

// Canvas painter: ctx is anything implementing the 2D context calls used
// here (the browser CanvasRenderingContext2D in production).
const COLORS = ['#e63946', '#2a9d8f', '#457b9d'];

export function drawChart(ctx, width, height, buffer, labels) {
  ctx.clearRect(0, 0, width, height);
  const chans = labels.length;
  const samples = [];
  for (let i = 0; i < buffer.length; i += 1) samples.push([]);
  const range = yRange(
    Array.from({ length: chans }, (_, c) => buffer.channel(c))
      .reduce((acc, ch) => {
        ch.forEach((v, i) => { (acc[i] = acc[i] || []).push(v); });
        return acc;
      }, samples));
  // zero line + range labels
  ctx.strokeStyle = '#888';
  ctx.beginPath();
  ctx.moveTo(0, height / 2);
  ctx.lineTo(width, height / 2);
  ctx.stroke();
  ctx.fillStyle = '#888';
  ctx.font = '10px sans-serif';
  ctx.fillText(String(range[1]), 2, 10);
  ctx.fillText(String(range[0]), 2, height - 2);
  // traces
  for (let c = 0; c < chans; c += 1) {
    const pts = toPolyline(buffer.channel(c), width, height, range);
    if (pts.length < 2) continue;
    ctx.strokeStyle = COLORS[c % COLORS.length];
    ctx.beginPath();
    ctx.moveTo(pts[0][0], pts[0][1]);
    for (let i = 1; i < pts.length; i += 1) ctx.lineTo(pts[i][0], pts[i][1]);
    ctx.stroke();
  }
}
```

- [ ] Step 4(GREEN 确认): `node --test test/unit/` —— 29/29 pass,3 连跑。

**验证:** node 29/29。
**Commit 点:** `web: wrench trend chart data model + painter (5 node tests)`

---

## Task 8: `index.html` + `app.css` + `app.js`(接线壳与页面)

**目标:** 单页五区(Dashboard / Servo Control / Force Monitor / Calibration / Parameters+Diagnostics,对应规格 §13 六页合并——Parameters 只读并入 Diagnostics 区,见决策与非目标);app.js 只做 DOM 接线,不含可测逻辑(逻辑全在 Task 5-7 模块)。无单测(壳),验证走 Task 10 冒烟。

**File Structure:**

```text
ros_ws/src/soft_robot_web_interface/www/
  index.html
  css/app.css
  js/app.js
```

**Steps:**

- [ ] Step 1: `www/index.html`,逐字:

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Soft Robot Operator Interface</title>
  <link rel="stylesheet" href="css/app.css">
</head>
<body>
  <header id="banner" class="ok">
    <span id="banner-state">CONNECTING…</span>
    <span id="banner-note"></span>
    <span id="ws-status" class="badge off">rosbridge</span>
  </header>

  <main>
    <!-- Dashboard (spec 13): links, mode/profile, active faults -->
    <section id="dashboard">
      <h2>Dashboard</h2>
      <div class="badges">
        <span id="badge-eki" class="badge off">EKI</span>
        <span id="badge-rsi" class="badge off">RSI</span>
        <span id="badge-sri" class="badge off">SRI</span>
      </div>
      <table class="kv">
        <tr><th>System state</th><td id="kv-state">—</td></tr>
        <tr><th>Mode / profile</th><td id="kv-mode">—</td></tr>
        <tr><th>Active controller</th><td id="kv-controller">—</td></tr>
        <tr><th>Tool synced</th><td id="kv-tool">—</td></tr>
        <tr><th>EKI note</th><td id="kv-ekinote">—</td></tr>
        <tr><th>Hint</th><td id="kv-hint">—</td></tr>
      </table>
    </section>

    <!-- Servo Control (spec 13): start/stop, mode & profile, limits -->
    <section id="servo">
      <h2>Servo Control</h2>
      <label>Mode
        <select id="sel-mode">
          <option value="1">DIRECT_CARTESIAN</option>
          <option value="2" selected>FORCE_COMPLIANCE</option>
        </select>
      </label>
      <label>Profile
        <select id="sel-profile">
          <option value="0" selected>DRAG</option>
          <option value="1">PRECISION</option>
        </select>
      </label>
      <div class="buttons">
        <button id="btn-start">Start Servo</button>
        <button id="btn-stop">Stop Servo</button>
        <button id="btn-zero">Zero Sensor</button>
        <button id="btn-reset">Reset Fault</button>
      </div>
      <p id="cmd-result" class="cmd-result"></p>
      <table class="kv">
        <tr><th>Saturation count</th><td id="kv-sat">—</td></tr>
        <tr><th>RSI fault</th><td id="kv-rsifault">—</td></tr>
      </table>
      <p class="note">Correction values are not published by any node
        (plan 6 spec-conflict record 2); saturation counts stand in as
        the limit-status indicator.</p>
    </section>

    <!-- Force Monitor (spec 13): trend curves -->
    <section id="force">
      <h2>Force Monitor</h2>
      <div class="chart-block">
        <h3>Force [N] <span class="legend">
          <i class="c0">Fx</i> <i class="c1">Fy</i> <i class="c2">Fz</i>
        </span></h3>
        <canvas id="chart-force" width="560" height="160"></canvas>
      </div>
      <div class="chart-block">
        <h3>Torque [Nm] <span class="legend">
          <i class="c0">Tx</i> <i class="c1">Ty</i> <i class="c2">Tz</i>
        </span></h3>
        <canvas id="chart-torque" width="560" height="160"></canvas>
      </div>
      <table class="kv">
        <tr><th>Latest wrench</th><td id="kv-wrench">—</td></tr>
      </table>
    </section>

    <!-- Calibration (spec 13): wizard with feedback + result preview -->
    <section id="calibration">
      <h2>Calibration</h2>
      <div class="buttons">
        <button id="btn-cal-start">Start Calibration</button>
        <button id="btn-cal-cancel">Cancel</button>
      </div>
      <p id="cal-phase" class="cal-phase">idle</p>
      <progress id="cal-progress" max="8" value="0"></progress>
      <table class="kv" id="cal-result-table">
        <tr><th>Result</th><td id="cal-outcome">—</td></tr>
        <tr><th>gravity_n [N]</th><td id="cal-g">—</td></tr>
        <tr><th>com [m]</th><td id="cal-com">—</td></tr>
        <tr><th>bias F [N]</th><td id="cal-biasf">—</td></tr>
        <tr><th>bias T [Nm]</th><td id="cal-biast">—</td></tr>
        <tr><th>r2 force / torque</th><td id="cal-r2">—</td></tr>
      </table>
    </section>

    <!-- Diagnostics + read-only parameters (spec 13, merged) -->
    <section id="diagnostics">
      <h2>Diagnostics</h2>
      <table class="kv">
        <tr><th>RSI ipoc</th><td id="kv-ipoc">—</td></tr>
        <tr><th>RSI timeouts (total/consec)</th><td id="kv-timeouts">—</td></tr>
        <tr><th>RSI bad frames / ipoc jumps</th><td id="kv-badframes">—</td></tr>
        <tr><th>SRI samples / gaps</th><td id="kv-sri-samples">—</td></tr>
        <tr><th>SRI last sample age [s]</th><td id="kv-sri-age">—</td></tr>
        <tr><th>EKI reconnects / state age</th><td id="kv-eki-age">—</td></tr>
        <tr><th>Tool (EKI, read-only)</th><td id="kv-tool-frame">—</td></tr>
        <tr><th>Manager diagnostics</th><td id="kv-diag">—</td></tr>
      </table>
    </section>
  </main>

  <script type="module" src="js/app.js"></script>
</body>
</html>
```

- [ ] Step 2: `www/css/app.css`,逐字:

```css
/* Operator UI: dark, high-contrast, large hit targets (shop floor). */
* { box-sizing: border-box; }
body {
  margin: 0; font-family: system-ui, sans-serif;
  background: #14171c; color: #e8e8e8;
}
header {
  position: sticky; top: 0; padding: 10px 16px; font-size: 1.2rem;
  display: flex; gap: 16px; align-items: center;
}
header.ok    { background: #1d3524; }
header.info  { background: #1c3350; }
header.warn  { background: #57430e; }
header.error { background: #5a1717; }
#banner-note { font-size: 0.9rem; opacity: 0.85; }
#ws-status { margin-left: auto; }

main {
  display: grid; gap: 14px; padding: 14px;
  grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
}
section { background: #1d2129; border-radius: 8px; padding: 12px 16px; }
h2 { margin: 0 0 10px; font-size: 1rem; color: #9ecbff; }
h3 { margin: 8px 0 4px; font-size: 0.9rem; }

.badge {
  display: inline-block; padding: 3px 10px; border-radius: 12px;
  font-size: 0.8rem; background: #444;
}
.badge.on  { background: #2a9d4a; }
.badge.off { background: #8a2b2b; }

table.kv { width: 100%; border-collapse: collapse; font-size: 0.9rem; }
table.kv th {
  text-align: left; font-weight: normal; color: #9aa4b2;
  padding: 3px 8px 3px 0; width: 45%;
}
table.kv td { padding: 3px 0; font-variant-numeric: tabular-nums; }

label { display: inline-block; margin: 0 12px 8px 0; font-size: 0.9rem; }
select, button {
  font-size: 1rem; padding: 8px 14px; border-radius: 6px; border: 0;
}
select { background: #2b3140; color: #e8e8e8; }
.buttons { display: flex; gap: 10px; flex-wrap: wrap; margin: 8px 0; }
button { background: #33518d; color: #fff; cursor: pointer; }
button:disabled { background: #3a3f4a; color: #777; cursor: not-allowed; }
#btn-stop { background: #8d3333; }
.cmd-result { min-height: 1.2em; font-size: 0.85rem; color: #ffd166; }
.note { font-size: 0.78rem; color: #8a93a2; }

canvas { background: #12151a; border-radius: 4px; width: 100%; }
.legend i { font-style: normal; margin-left: 8px; font-size: 0.8rem; }
.legend .c0 { color: #e63946; } .legend .c1 { color: #2a9d8f; }
.legend .c2 { color: #457b9d; }
.cal-phase { font-size: 1rem; color: #9ecbff; }
progress { width: 100%; height: 14px; }
```

- [ ] Step 3: `www/js/app.js`,逐字:

```js
// DOM wiring shell: everything testable lives in the imported modules.
// Dangerous operations go exclusively through the manager services and
// the calibrate_payload action (spec 13; decision 7).
import { RosClient } from './ros_client.js';
import { ActionClient, GOAL_STATUS } from './action_client.js';
import {
  STATE_NAMES, MODE_NAMES, PROFILE_NAMES,
  linkBadges, buttonGates, presentState, controllersHint, calPhaseLabel,
} from './state_model.js';
import { TraceBuffer, drawChart } from './wrench_chart.js';

const $ = (id) => document.getElementById(id);
const wsUrl = 'ws://' + location.hostname + ':9090';
const ros = new RosClient(wsUrl, (url) => new WebSocket(url));

let lastMgr = null;
let lastEki = null;
let lastStopAtMs = null;

function setBadge(el, ok) {
  el.classList.toggle('on', ok);
  el.classList.toggle('off', !ok);
}

function cmdResult(text) { $('cmd-result').textContent = text; }

function render() {
  if (!lastMgr) return;
  const m = lastMgr;
  const banner = presentState(m, lastStopAtMs, Date.now());
  const header = document.querySelector('header');
  header.className = banner.level;
  $('banner-state').textContent = STATE_NAMES[m.system_state] ?? '?';
  $('banner-note').textContent = banner.text === STATE_NAMES[m.system_state]
    ? '' : banner.text;

  const badges = linkBadges(m, lastEki);
  setBadge($('badge-eki'), badges.eki.ok);
  setBadge($('badge-rsi'), badges.rsi.ok);
  setBadge($('badge-sri'), badges.sri.ok);
  $('kv-ekinote').textContent = badges.eki.note || '—';

  $('kv-state').textContent =
    m.system_state + ' (' + (STATE_NAMES[m.system_state] ?? '?') + ')';
  $('kv-mode').textContent = (MODE_NAMES[m.mode] ?? m.mode) + ' / ' +
    (PROFILE_NAMES[m.profile] ?? m.profile);
  $('kv-controller').textContent = m.active_controller || '(none)';
  $('kv-tool').textContent = m.tool_synced ? 'yes' : 'no';
  $('kv-hint').textContent = controllersHint(m) || '—';
  $('kv-rsifault').textContent = m.rsi_fault ? 'LATCHED' : 'no';

  const g = buttonGates(m);
  $('btn-start').disabled = !g.startServo;
  $('btn-stop').disabled = !g.stopServo;
  $('btn-zero').disabled = !g.zeroSensor;
  $('btn-reset').disabled = !g.resetFault;
  $('btn-cal-start').disabled = !g.calibrate;
  $('btn-cal-cancel').disabled = !m.calibrating;
}

// --- rosbridge link status ---
ros.onStatus = (up) => {
  setBadge($('ws-status'), up);
  if (!up) cmdResult('rosbridge connection lost — reconnecting');
};
ros.connect();

// --- subscriptions ---
ros.subscribe('/soft_robot/manager_state', 'soft_robot_msgs/ManagerState',
  (m) => { lastMgr = m; render(); });
ros.subscribe('/kuka/eki/state', 'soft_robot_msgs/EkiState', (m) => {
  lastEki = m;
  $('kv-eki-age').textContent =
    m.reconnects + ' / ' + m.state_age.toFixed(2);
  $('kv-tool-frame').textContent =
    [m.tool_x, m.tool_y, m.tool_z, m.tool_a, m.tool_b, m.tool_c]
      .map((v) => v.toFixed(1)).join(', ');
  render();
}, { throttleMs: 200 });
ros.subscribe('/kuka/rsi/state', 'soft_robot_msgs/RsiState', (m) => {
  $('kv-ipoc').textContent = m.ipoc;
  $('kv-timeouts').textContent =
    m.total_timeouts + ' / ' + m.consecutive_timeouts;
  $('kv-badframes').textContent = m.bad_frames + ' / ' + m.ipoc_jumps;
  $('kv-sat').textContent = m.saturation_count;
}, { throttleMs: 200 });
ros.subscribe('/sri_ft/status', 'soft_robot_msgs/SriStatus', (m) => {
  $('kv-sri-samples').textContent = m.samples + ' / ' + m.package_gaps;
  $('kv-sri-age').textContent = m.last_sample_age.toFixed(3);
}, { throttleMs: 200 });
ros.subscribe('/soft_robot/diagnostics',
  'diagnostic_msgs/DiagnosticArray', (m) => {
    const st = m.status && m.status[0];
    $('kv-diag').textContent = st ? st.message : '—';
  });

// --- wrench charts (decision 8: 50 ms throttle, 600-sample window) ---
const forceBuf = new TraceBuffer(3, 600);
const torqueBuf = new TraceBuffer(3, 600);
ros.subscribe('/sri_ft/wrench_raw', 'geometry_msgs/WrenchStamped', (m) => {
  const f = m.wrench.force;
  const t = m.wrench.torque;
  forceBuf.push([f.x, f.y, f.z]);
  torqueBuf.push([t.x, t.y, t.z]);
  $('kv-wrench').textContent =
    'F(' + f.x.toFixed(1) + ', ' + f.y.toFixed(1) + ', ' + f.z.toFixed(1) +
    ') T(' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ', ' +
    t.z.toFixed(2) + ')';
}, { throttleMs: 50, queueLength: 1 });

function paint() {
  const fc = $('chart-force');
  const tc = $('chart-torque');
  drawChart(fc.getContext('2d'), fc.width, fc.height, forceBuf,
            ['Fx', 'Fy', 'Fz']);
  drawChart(tc.getContext('2d'), tc.width, tc.height, torqueBuf,
            ['Tx', 'Ty', 'Tz']);
  requestAnimationFrame(paint);
}
requestAnimationFrame(paint);

// --- manager service buttons ---
function call(service, args, label) {
  cmdResult(label + '…');
  ros.callService(service, args).then((res) => {
    cmdResult(label + ': ' + (res.success ? 'ok' : 'REJECTED') +
              (res.message ? ' — ' + res.message : ''));
  }).catch((err) => cmdResult(label + ': ' + err.message));
}
$('btn-start').onclick = () => call('/soft_robot/start_servo', {
  mode: Number($('sel-mode').value),
  profile: Number($('sel-profile').value),
}, 'start_servo');
$('btn-stop').onclick = () => {
  lastStopAtMs = Date.now();   // decision 15 grace window anchor
  call('/soft_robot/stop_servo', {}, 'stop_servo');
};
$('btn-zero').onclick = () => call('/soft_robot/zero_sensor', {},
                                   'zero_sensor');
$('btn-reset').onclick = () => call('/soft_robot/reset_fault', {},
                                    'reset_fault');

// --- calibration wizard ---
const cal = new ActionClient(ros, '/soft_robot/calibrate_payload',
                             'soft_robot_msgs/CalibratePayload');
cal.onFeedback = (fb) => {
  $('cal-phase').textContent =
    calPhaseLabel(fb.phase, fb.pose_index, fb.pose_count);
  $('cal-progress').max = fb.pose_count || 8;
  $('cal-progress').value = fb.pose_index;
};
cal.onResult = (status, res) => {
  const name = status === GOAL_STATUS.SUCCEEDED ? 'SUCCEEDED'
    : status === GOAL_STATUS.PREEMPTED ? 'CANCELLED' : 'ABORTED';
  $('cal-phase').textContent = 'finished: ' + name;
  $('cal-outcome').textContent = name + (res.message
    ? ' — ' + res.message : '');
  $('cal-g').textContent = res.gravity_n.toFixed(2);
  $('cal-com').textContent = [res.com_x, res.com_y, res.com_z]
    .map((v) => v.toFixed(4)).join(', ');
  $('cal-biasf').textContent = [res.bias_fx, res.bias_fy, res.bias_fz]
    .map((v) => v.toFixed(2)).join(', ');
  $('cal-biast').textContent = [res.bias_tx, res.bias_ty, res.bias_tz]
    .map((v) => v.toFixed(3)).join(', ');
  $('cal-r2').textContent =
    res.r2_force.toFixed(4) + ' / ' + res.r2_torque.toFixed(4);
};
$('btn-cal-start').onclick = () => {
  if (cal.sendGoal({})) $('cal-phase').textContent = 'goal sent…';
};
$('btn-cal-cancel').onclick = () => cal.cancel();
```

- [ ] Step 4 验证(静态): `node --check` 不适用于 module DOM 代码——改用语法检查:

```bash
cd ros_ws/src/soft_robot_web_interface
node --input-type=module -e "await import('./www/js/state_model.js'); await import('./www/js/wrench_chart.js'); await import('./www/js/ros_client.js'); await import('./www/js/action_client.js'); console.log('modules ok')"
node --experimental-websocket --input-type=module -e "
globalThis.document={getElementById:()=>new Proxy({classList:{toggle(){}},style:{}},{get:(t,k)=>t[k]??((k==='getContext')?()=>new Proxy({},{get:()=>()=>{}}):t[k]),set:()=>true}),querySelector:()=>({className:''})};
globalThis.location={hostname:'127.0.0.1'};
globalThis.requestAnimationFrame=()=>{};
await import('./www/js/app.js'); console.log('app wiring ok')" 
node --test test/unit/    # 29/29 不回归
```

(第二条为 app.js 的"能加载不炸"探针:stub 掉 DOM/location/RAF;WebSocket 由 --experimental-websocket 提供,连不上 9090 只会走 onclose 重连路径,不影响加载判定。若该探针在实施中因 DOM stub 边角失败,允许简化为仅第一条 import 检查并在报告记录——app.js 的行为验证本就归 Task 10 冒烟。)

**Commit 点:** `web: operator page, styles, and DOM wiring shell`

---

## Task 9: `web.launch` + `serve_www.sh` + README(部署面)

**目标:** 一键启动 rosbridge + 静态托管;README 记录使用、离线部署(Task 1 素材)、浏览器要求、smoke 入口。

**File Structure:**

```text
ros_ws/src/soft_robot_web_interface/
  launch/web.launch
  scripts/serve_www.sh
  README.md
```

**Steps:**

- [ ] Step 1: `scripts/serve_www.sh`,逐字(可执行位):

```bash
#!/usr/bin/env bash
# Serves the static frontend with the python3 stdlib server. Shim layer
# so roslaunch can own the process (it appends __name/__log remapping
# args that http.server would choke on — same pattern as the bringup
# run_mock.sh). Usage: serve_www.sh [port]
set -eu
PORT="${1:-8080}"
WWW_DIR="$(cd "$(dirname "$0")/../www" && pwd)"
exec python3 -m http.server "$PORT" --directory "$WWW_DIR" --bind 0.0.0.0
```

- [ ] Step 2: `launch/web.launch`,逐字:

```xml
<launch>
  <!-- Optional web UI layer (spec 5.7/13). Independent entry point: run
       it NEXT TO soft_robot.launch or sim.launch (decision 12); neither
       includes the other so the core stacks stay launchable on machines
       without rosbridge installed. -->
  <arg name="rosbridge_port" default="9090"/>
  <arg name="http_port" default="8080"/>

  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch">
    <arg name="port" value="$(arg rosbridge_port)"/>
  </include>

  <node pkg="soft_robot_web_interface" type="serve_www.sh"
        name="soft_robot_web_server" output="screen"
        args="$(arg http_port)"/>
</launch>
```

- [ ] Step 3: `README.md`,逐字:

```markdown
# soft_robot_web_interface

Browser operator interface (spec 5.7/13). Static ES-module frontend +
`rosbridge_websocket`; no build chain, no external JS dependencies, no
new C++. All dangerous operations go through the manager services and
the `/soft_robot/calibrate_payload` action — the UI never publishes
`/soft_robot/mode_command`, correction streams, or calls `/sri_ft/zero`
directly (spec 13; manager remains the sole authority, buttons only
pre-judge the gates).

## Run

    # next to either core stack (decision 12):
    roslaunch soft_robot_bringup sim.launch          # or soft_robot.launch
    roslaunch soft_robot_web_interface web.launch    # rosbridge 9090 + http 8080

Open `http://<host>:8080`. The `ws-status` badge turns green when the
rosbridge socket is up; the client reconnects automatically every 1 s.

## Requirements

- `ros-noetic-rosbridge-server` (apt). Offline cells: build a deb bundle
  on any machine of the same distro with
  `apt-get download ros-noetic-rosbridge-server ros-noetic-rosbridge-library
  ros-noetic-rosapi ros-noetic-rosbridge-msgs ros-noetic-rosapi-msgs`
  (list verified in plan Task 1), copy, `sudo dpkg -i *.deb`.
- Browser: any evergreen browser with ES-module support (Chromium /
  Firefox from 2020 on). No internet access needed at runtime.

## Tests

    node --test test/unit/          # 29 pure-logic tests, no ROS/DOM/network

Integration (rosbridge + sim stack) is a scripted probe plus a manual
smoke checklist — see plan Task 10 and `test/integration/`.

## I-1 semantics shown by the UI

While RSI is active the KRL EKI heartbeat pauses inside `RSI_MOVECORR`
by design; the manager supervises through the 50 Hz RSI channel (rule
R1) and the UI shows the EKI badge OK with the note "heartbeat paused
(RSI active, expected)". A FAULT within 10 s of the operator's own stop
is presented as the expected stop-chain endgame with a Reset hint
(rules R2/R3, `docs/commissioning_checklist.md` Stage 2).
```

- [ ] Step 4 验证:

```bash
chmod +x ros_ws/src/soft_robot_web_interface/scripts/serve_www.sh
cd ros_ws && catkin_make
rosrun roslaunch roslaunch-check src/soft_robot_web_interface/launch/web.launch
# 静态服务冒烟(无 ROS):
./src/soft_robot_web_interface/scripts/serve_www.sh 8090 &
sleep 0.5
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/index.html   # 200
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/js/app.js    # 200
kill %1
pgrep -f "http.server 8090" || echo no-leftover
```

**Commit 点:** `web: launch entry, static server shim, README`

---

## Task 10: 集成探针 + sim 冒烟清单 + 整仓回归

**目标:** 可脚本化 rosbridge 探针(node 复用 ros_client.js 直连 9090,验证订阅/服务/action 三通道);人工冒烟清单(浏览器全流程,规格 15.4 步 9 的 sim 版);整仓回归定格 304。

**File Structure:**

```text
ros_ws/src/soft_robot_web_interface/test/integration/probe_rosbridge.mjs
ros_ws/src/soft_robot_web_interface/README.md        # + smoke 附录
```

**Steps:**

- [ ] Step 1: `test/integration/probe_rosbridge.mjs`,逐字:

```js
// Scripted rosbridge probe: drives the REAL websocket (node
// --experimental-websocket) against a running sim stack + web.launch.
// Exercises the three transport patterns the UI uses — subscribe,
// service call, action goal/feedback/result — and prints PASS/FAIL
// lines. Exit 0 only if every check passed. Bounded: hard 90 s cap.
//
// Usage:
//   roslaunch soft_robot_bringup sim.launch        (terminal 1)
//   roslaunch soft_robot_web_interface web.launch  (terminal 2)
//   node --experimental-websocket test/integration/probe_rosbridge.mjs
import { RosClient } from '../../www/js/ros_client.js';
import { ActionClient, GOAL_STATUS } from '../../www/js/action_client.js';

const checks = [];
function check(name, ok, detail = '') {
  checks.push(ok);
  console.log((ok ? 'PASS' : 'FAIL') + ' ' + name +
              (detail ? ' — ' + detail : ''));
}
function waitFor(pred, ms) {
  return new Promise((resolve) => {
    const t0 = Date.now();
    const h = setInterval(() => {
      if (pred() || Date.now() - t0 > ms) {
        clearInterval(h);
        resolve(pred());
      }
    }, 100);
  });
}

setTimeout(() => { console.log('FAIL global 90 s cap'); process.exit(2); },
           90000).unref();

const ros = new RosClient('ws://127.0.0.1:9090', (u) => new WebSocket(u));
let wsUp = false;
ros.onStatus = (up) => { wsUp = up; };
ros.connect();
check('rosbridge socket', await waitFor(() => wsUp, 5000));

// 1. subscribe: manager_state arrives and reaches READY (sim stack).
let mgr = null;
ros.subscribe('/soft_robot/manager_state', 'soft_robot_msgs/ManagerState',
              (m) => { mgr = m; });
check('manager_state received', await waitFor(() => mgr !== null, 5000));
check('manager READY (state 2)',
      await waitFor(() => mgr && mgr.system_state === 2, 20000),
      mgr ? 'state=' + mgr.system_state : 'no msg');

// 2. wrench stream through the UI's own throttle settings.
let wrenches = 0;
ros.subscribe('/sri_ft/wrench_raw', 'geometry_msgs/WrenchStamped',
              () => { wrenches += 1; }, { throttleMs: 50, queueLength: 1 });
check('throttled wrench stream', await waitFor(() => wrenches >= 10, 5000),
      wrenches + ' msgs');

// 3. service round-trips: gate rejection then accepted start/stop.
const zeroInReady = await ros.callService('/soft_robot/zero_sensor', {});
check('zero_sensor in READY ok', zeroInReady.success === true,
      zeroInReady.message);
const start = await ros.callService('/soft_robot/start_servo',
                                    { mode: 2, profile: 0 });
check('start_servo accepted', start.success === true, start.message);
check('SERVOING (state 3)',
      await waitFor(() => mgr && mgr.system_state === 3, 5000));
const zeroInServo = await ros.callService('/soft_robot/zero_sensor', {});
check('zero_sensor REJECTED while SERVOING', zeroInServo.success === false,
      zeroInServo.message);
const stop = await ros.callService('/soft_robot/stop_servo', {});
check('stop_servo accepted', stop.success === true, stop.message);
check('back to READY',
      await waitFor(() => mgr && mgr.system_state === 2, 10000));

// 4. action channel: full calibration against the sim mocks
//    (sim.launch cuts samples_per_pose to 20; expected fit for the
//    constant fz=5 mock: gravity ~ 0, bias ~ (0,0,5), r2_force = 1 —
//    derivation in the bringup README / sim.launch comments).
const cal = new ActionClient(ros, '/soft_robot/calibrate_payload',
                             'soft_robot_msgs/CalibratePayload');
let feedbacks = 0;
let calDone = null;
cal.onFeedback = () => { feedbacks += 1; };
cal.onResult = (status, res) => { calDone = { status, res }; };
check('calibration goal sent', cal.sendGoal({}) === true);
check('calibration finished', await waitFor(() => calDone !== null, 60000));
if (calDone) {
  check('calibration SUCCEEDED',
        calDone.status === GOAL_STATUS.SUCCEEDED,
        'status=' + calDone.status + ' msg=' + calDone.res.message);
  check('feedback flowed', feedbacks >= 5, feedbacks + ' frames');
  check('fit sane (|G|<1, bias_fz≈5, r2_force>0.999)',
        Math.abs(calDone.res.gravity_n) < 1.0 &&
        Math.abs(calDone.res.bias_fz - 5.0) < 0.5 &&
        calDone.res.r2_force > 0.999,
        'G=' + calDone.res.gravity_n.toFixed(3) +
        ' bias_fz=' + calDone.res.bias_fz.toFixed(3) +
        ' r2=' + calDone.res.r2_force.toFixed(5));
}

ros.close();
const failed = checks.filter((c) => !c).length;
console.log(checks.length + ' checks, ' + failed + ' failed');
process.exit(failed === 0 ? 0 : 1);
```

期望检查数推导:14 项固定(socket、received、READY、wrench、zero-ok、start、SERVOING、zero-reject、stop、READY、goal-sent、finished、SUCCEEDED+feedback+fit 3 项)= **15 checks**(calDone 分支 3 项),全 PASS。

- [ ] Step 2 执行探针(三终端,记录输出全文到报告):

```bash
roslaunch soft_robot_bringup sim.launch                    # T1
roslaunch soft_robot_web_interface web.launch              # T2
cd ros_ws/src/soft_robot_web_interface && \
  node --experimental-websocket test/integration/probe_rosbridge.mjs   # T3
# 期望:15 checks, 0 failed;exit 0
```

- [ ] Step 3 人工浏览器冒烟(sim 栈 + web.launch 均在跑;逐条记录):

```text
a) http://127.0.0.1:8080:ws-status 绿;banner READY;EKI/RSI/SRI 徽章
   依 sim 状态(EKI 绿、SRI 绿;RSI 在首次 start 前灰属正常——KRC 未推流)。
b) Force Monitor:两幅曲线滚动;fz 通道恒 ~5(mock --fz 5),其余 ~0;
   30 s 后窗口平移(600 点环形缓冲)。
c) Start Servo(FORCE_COMPLIANCE/DRAG):banner SERVOING;active_controller
   = force_compliance_controller;Start 灰、Stop 亮;Zero 灰(门控镜像)。
d) Zero Sensor 按钮灰(SERVOING);cmd-result 无误报。
e) Stop Servo:回 READY;若 10 s 内出现 FAULT(sim 下通常不出现——mock
   持续推流不断流;此项主要为真机语义),banner 应为蓝色 info "after stop"。
e2) Minor 7 启发式:kill soft_robot_manager 进程后单独重启它(sim 栈
   其余不动):UI 短暂 CONNECTED 期间 Hint 行不出现误导性 controllers
   提示(preload 已修复,应快速回 READY)。
f) 标定向导:Start Calibration → phase 走 MOVING→SETTLING→SAMPLING×8
   → RETURNING → finished: SUCCEEDED;进度条 0→8;结果表 gravity≈0、
   bias F≈(0,0,5)、r2 1.0000(与 bringup README 3f 推导一致)。
g) 标定取消:再次 Start 后在第 2~3 姿态 Cancel → finished: CANCELLED
   (Minor 5 修复的 PREEMPTED 终态;修复前会显示 ABORTED)。
h) 故障复位:kill RSI sim server 进程 → banner FAULT(红,超宽限窗语义
   仅对操作员 stop 生效,此处非 stop 引发,立即红)→ 重启 sim server →
   Reset Fault → READY。
i) 断桥重连:Ctrl-C web.launch → ws-status 红 + "connection lost";重启
   web.launch → 自动重连绿,订阅恢复(无需刷新页面)。
j) 关停全部;pgrep -af 'mock_server|sim_server|rosbridge|http.server'
   为空(零残留)。
```

- [ ] Step 4: 把 Step 3 清单(a~j)追加到 `soft_robot_web_interface/README.md` 末尾,标题 `## Sim smoke checklist (plan Task 10)`,逐字照录。
- [ ] Step 5 整仓回归定格:

```bash
cd ros_ws && catkin_make run_tests
# 逐包 XML 直读(锚定 <testsuites> 根元素,勿元素级双计):
for d in build/test_results/*/gtest-*.xml; do echo "$d"; grep -o '<testsuites[^>]*' "$d" | head -1; done
# 求和期望:304 tests / 0 failures / 0 errors
#   52+59+8+61+44+39+41 = 304(仅 soft_force_control_manager 33→41)
cd src/soft_robot_web_interface && node --test test/unit/   # 29/29
```

**Commit 点:** `web: rosbridge integration probe + smoke checklist; repo regression 304`

---

## 验收清单(整计划完成判定)

- [ ] followups 消化:I-1(Task 2+4+6)、I-2(Task 3)闭环;Minor 3-7 落地;8-11 排除有记录;12 作为裁决输入引用。
- [ ] `catkin_make` 全仓零警告;`catkin_make run_tests` 逐包 XML 直读 **304/0/0**(52+59+8+61+44+39+41)。
- [ ] `node --test test/unit/` **29/29**(8 ros_client + 6 action_client + 10 state_model + 5 wrench_chart),3 连跑稳定。
- [ ] `probe_rosbridge.mjs` 15 checks 全 PASS(sim 栈)。
- [ ] 浏览器冒烟 a~j 全过,README 附录收录。
- [ ] 规格 §16 web 相关验收位:show status ✓ / start-stop ✓ / switch modes+profiles ✓ / view wrench ✓ / view correction values → 以 saturation+mode 代理并记录冲突 ✓ / run calibration ✓。
- [ ] `roslaunch-check` 三 launch 全过(web.launch 新增;sim/soft_robot 不回归)。
- [ ] 核对单 Stage 2/3 无 "clean READY" 矛盾文字;R1/R2/R3 语义三处文档一致。
- [ ] 零残留:所有冒烟后 `pgrep -af 'mock_server|sim_server|rosbridge|http.server'` 为空。

## 遗留风险

1. **Web jog 未做**(非目标):规格 §7.7 的 web jog 消费面存在(stream 话题),但 rosbridge JSON 通道对 100 Hz 命令流的时延/抖动未验证;若未来做,须先量化 rosbridge 往返延迟并评估 stream_timeout 余量。
2. **参数页只读**:gains/limits 的在线编辑需要 manager 侧校验服务(规格 §14 "Manager validates updates"),v1 无此服务,UI 不提供写路径。
3. **correction values 无真实回显**(规格冲突 2):需要 hw/controller 增发布器才能实现,留给后续计划;当前以 saturation_count 代理。
4. **I-1 的 KRC 侧终局依赖装机实测**:R2/R3 使 PC 侧终局回 READY,但"PC 侧干净停止触发器"(Stop S break MOVECORR)仍未配置——装机时按 context notes 第 3 条增补项裁决并回填核对单。
5. **rosbridge 0.11 安全面**:无鉴权、明文 WS;部署限定隔离的设备网段(README 已注明浏览器/网段前提,未做 TLS/auth)。
6. **stop 宽限窗为 UI 呈现层启发式**:10 s 常数未经真机 5 拍闩锁时序标定(4 ms×5=20 ms 闩锁 + 心跳恢复时间 ≪ 10 s,方向安全——只可能把真故障多显示为 info 10 秒,状态机本体不受影响)。
7. **node 单测不等于浏览器行为**:ES module 在 node 与浏览器的运行差异(如 WebSocket 事件时序)由冒烟 i 项兜底,未做自动化浏览器测试。

## 自查表(撰写者已核对)

| 项 | 结果 |
|---|---|
| 全部必读输入读毕 | ✓(规范 §5.7/13/11/10/9/5.5/15/14;followups 12 条;msgs 3 定义;manager.yaml/calibration.yaml;manager_node.cpp 全文;两 launch+README;commissioning_checklist 全文;plan5-progress conventions;另加:system_state_core.h/.cpp、manager_runtime.h/.cpp、test_system_state_core.cpp、test_manager_runtime.cpp 60-199、KRL 模板、RSI context notes、rsi_session_monitor、kuka_rsi_robot_hw.h、test_msgs.cpp、CMakeLists) |
| 接口名与现状一致 | ✓(服务/话题/字段逐一自源码抄录;ManagerState 无 controllers_loaded 已注意;CalPhase 无 SOLVING 已入 Minor 4) |
| 规范-现状冲突记录 | ✓(3 条,均以现状为准) |
| followups 12 条逐项裁决 | ✓(吸收 8/排除 4,理由在表) |
| 数值期望可手工推导 | ✓(304=296+6+2;29=8+6+10+5;15 checks;30 s 窗=600/20;±10=ceil(7.3/5)*5;I-1 终局 6 场景表) |
| 逐字代码完整 | ✓(C++ 增量含插入锚点与替换范围;JS/HTML/CSS/launch/sh/md 全文) |
| TDD RED→GREEN 每任务成立 | ✓(Task 2/3 gtest 先失败;Task 5/6/7 node 先失败;Task 8/9 为壳/部署面,验证走静态检查与探针,已注明不造假测试) |
| 有界等待/零残留/127.0.0.1 | ✓(probe 90 s 硬帽+每检查独立超时;冒烟 j 项;全部 127.0.0.1) |
| 不做 git 操作(子代理) | ✓(commit 点仅标注给主流程) |
