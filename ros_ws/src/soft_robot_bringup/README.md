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

Boot-time payload loading (Task 8 Step 2 fallback choice, recorded):
`config/payload.yaml` cannot be loaded through a conditional
`<rosparam if="$(eval ...)"/>` — roslaunch rejects double-underscore
eval expressions and offers no file-existence primitive — so
`soft_robot.launch` runs the manager under the launch-prefix
`scripts/load_payload_then_exec.sh`, which `rosparam load`s the file
(if present) and then execs the node. The parameters are therefore on
the server before the manager loads any controller, and the calibrated
values win over the `soft_robot_controllers.yaml` defaults at boot.

Retired Task 8 debt (Task 8b): the temporary
`switch_controller_filter.py` proxy (and its `<remap>` in both launch
files) is gone. The manager now queries
`/controller_manager/list_controllers` itself and drops no-op entries
before every STRICT `switch_controller` request, so starts from READY,
mode changes, and the calibration entry work against the real
controller_manager without any middleman.

## Tool-frame force resolution (FORCE_COMPLIANCE)

Design: `docs/superpowers/specs/2026-07-04-tool-frame-compliance-design.md`.
The compliance pipeline resolves the SRI wrench through the frame chain
SENSOR -> TOOL -> BASE instead of treating sensor readings as BASE:

- Gravity compensation runs in the SENSOR frame using
  `R_base_sensor = R_bt * R_tcp_sensor` (R_bt from the live RIst pose).
- The admittance law (gains, deadbands, speed/accel caps) runs in the
  TOOL frame; the correction is rotated back to BASE for RSI RKorr.
- `R_tcp_sensor = R_ftool^-1 * R_mount` is SESSION-CONSTANT: it is
  locked once at servo activation from the latest `$TOOL` A/B/C on the
  EKI state topic plus the configured sensor mount offset. Changing the
  pendant `$TOOL` mid-servo takes effect at the NEXT servo start.
- All rotations use the KUKA convention `R = Rz(A)*Ry(B)*Rx(C)`, degrees.

Controller parameters (`soft_robot_controllers.yaml`, commented
examples next to `wrench_timeout`):

- `sensor_to_flange_abc` (default `[0, 0, 0]`): sensor mount offset on
  the flange as KUKA `[A, B, C] = [Rz, Ry, Rx]` in degrees. The default
  identity means the sensor is axis-aligned with the flange.
- `eki_state_topic` (default `/kuka/eki/state`): source of the `$TOOL`
  A/B/C angles.

Degrade paths (servo starts either way, ROS_WARN_ONCE each):

- No valid EKI tool data at activation -> identity tool rotation
  (pre-tool-frame behavior).
- Payload not calibrated (`gravity_n == 0`) -> zero-only mode: gravity
  terms are skipped, `compensate` reduces to `raw - bias`. Correct at
  the taring pose, but orientation changes make the uncompensated tool
  weight read as external force and the TCP drifts toward it; deadbands
  and the SafetyLimiter keep it bounded. Calibrate the payload before
  relying on orientation changes under load.

## 1. Build & static check

    cd ros_ws && catkin_make
    rosrun roslaunch roslaunch-check \
        $(rospack find soft_robot_bringup)/launch/sim.launch   # and soft_robot.launch

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
    #    Note (observed in the Task 8 smoke): the pose does NOT drift.
    #    The DRAG profile's adaptive deadband ramps to the standing force
    #    plus ramp_force_margin_n, so the mock's constant fz=5 stays
    #    inside the deadband. State/mode assertions above are the check.

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

## 5. Web UI (Plan 6)

Optional browser interface: see
`ros_ws/src/soft_robot_web_interface/README.md`. Start it alongside
either entry with `roslaunch soft_robot_web_interface web.launch`;
it consumes exactly the manager surface used in Stage 9 of
`docs/commissioning_checklist.md` and adds nothing to the realtime path.
