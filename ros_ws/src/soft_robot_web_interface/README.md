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
