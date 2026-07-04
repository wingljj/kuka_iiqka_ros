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

If 9090 is taken on the host (e.g. a local proxy), start with
`web.launch rosbridge_port:=9091` and open `http://<host>:8080/?ws=9091`
— the `?ws=<port>` query overrides the frontend's rosbridge port (the
integration probe takes the same override via `ROSBRIDGE_PORT`).

## Requirements

- `ros-noetic-rosbridge-server` (apt). Offline cells: build a deb bundle
  on any machine of the same distro with
  `apt-get download ros-noetic-rosbridge-server ros-noetic-rosbridge-library
  ros-noetic-rosapi ros-noetic-rosbridge-msgs`
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

## Sim smoke checklist (plan Task 10)

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
