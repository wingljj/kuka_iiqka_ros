# Real-Robot Commissioning Checklist (spec 15.4 + plan follow-ups)

Robot in T1, reduced speed, e-stop within reach for every stage.

Prerequisites for every stage below:

- ROS side launched with `roslaunch soft_robot_bringup soft_robot.launch
  kuka_ip:=<KRC IP> sensor_ip:=<SRI box IP>`.
- KUKA side installed and verified per
  `docs/kuka_iiqka_rsi_eki_setup.md` (KRL syntax section signed off
  first, then the EkiConfig / RSI-context field-by-field sections).
- `ROS_RSI_SERVO` selected and running on the pendant.
- Record every measured number next to its checkbox; this document is
  the commissioning record.

General abort rule (applies to all stages): any uncommanded motion,
any correction that does not stop when its input stops, or any state
the manager cannot explain -> e-stop, stop the stage, diagnose before
re-running. Never skip the zero-output stages (2 and 5).

## Stage 0 - protocol reality checks (before any motion)

- [ ] SRI wire protocol vs the real box (Plan 4 risk 1): frame 0xAA 0x55,
      big-endian length 27, PN big-endian, float32 little-endian, checksum
      = sum of 24 data bytes & 0xFF. On mismatch fix sri_frame.h constants
      (single location:
      `ros_ws/src/sri_force_torque_driver/include/sri_force_torque_driver/sri_frame.h`)
      and rerun the sri package tests.
- [ ] `rostopic echo --offset /sri_ft/wrench_raw`: stamp-vs-arrival offset
      at ms level (Plan 4 follow-up 9). NOTE: stamps are reception times;
      use SriStatus.package_gaps for interval statistics, never stamps.
- [ ] `rostopic hz /sri_ft/wrench_raw` ~ 250 Hz;
      `rostopic echo -n 1 /sri_ft/status` shows `streaming: True`,
      `bad_frames: 0`, and `package_gaps` not growing over 1 min.
- [ ] `/kuka/eki/state` heartbeat: age < 0.2 s sustained for 1 min
      (`rostopic echo --offset /kuka/eki/state`; the KRL pushes a
      RobotState every ~100 ms while no command is being served).

Expected: all three links report fresh and clean without any motion
command ever having been sent.
Abort: any checksum/endianness mismatch (fix `sri_frame.h`, rerun
`sri_force_torque_driver` tests, restart Stage 0); heartbeat gaps
> 1 s (KRL loop not running or EKI channel misconfigured).

## Stage 1 - EKI handshake & tool query (spec 15.4 step 1)

- [ ] `rostopic echo -n 1 /kuka/eki/state`: `connected: True`,
      `state_fresh: True`, `program_ready: True`, `fault: False`.
- [ ] `rosservice call /kuka/eki/get_tool` returns the values shown on
      the pendant for the active `$TOOL` (X/Y/Z/A/B/C, mm/deg).
- [ ] `rostopic echo -n 1 /soft_robot/manager_state`: `system_state: 2`
      (READY), `tool_synced: True`, `eki_connected: True`,
      `sri_streaming: True` (manager auto-syncs the tool after connect;
      retry period `tool_sync_retry_s: 2.0`).

Expected: manager reaches READY with no manual parameter pushing;
tool values match the pendant exactly.
Abort: tool mismatch (wrong tool selected on the pendant, or
Tool attribute mapping broken -> re-run the EkiConfig field checklist
in `docs/kuka_iiqka_rsi_eki_setup.md`); manager stuck in
OFFLINE/CONNECTED (check which `ManagerState` health flag is false).

## Stage 2 - RSI zero-output loop (spec 15.4 step 2)

10 min soak. RSI active, NO controller running, so the ROS side answers
every RSI frame with an all-zero RKorr.

- [ ] Start RSI without a servo mode:
      `rosservice call /kuka/eki/start_rsi_program` (direct bridge call;
      the manager is bypassed on purpose in this stage only).
- [ ] `rostopic echo -n 1 /kuka/rsi/state`: `connected: True`,
      `fault: False`.
- [ ] Robot does not move at all for 10 min (pendant position display
      frozen; no drift).
- [ ] After 10 min `rostopic echo -n 1 /kuka/rsi/state`:
      `total_timeouts` stable (not steadily climbing), `ipoc_jumps: 0`,
      `bad_frames: 0`.
- [ ] Task 9 review observation item (RSI_MOVECORR blocks the KRL EKI
      loop by design): while RSI is active, watch
      `/kuka/eki/state` — `state_fresh` may go false because the KRL
      heartbeat pauses during `RSI_MOVECORR()`. Confirm the manager's
      tolerance matches expectations: RSI-phase supervision rides the
      50 Hz `/kuka/rsi/state` channel (`rsi_state_timeout_s: 0.5`);
      record the observed manager behavior (READY/DEGRADED) here.
- [ ] `rosservice call /kuka/eki/stop_rsi_program` ends the loop cleanly;
      `/kuka/eki/state` heartbeat resumes (age < 0.2 s again).

Expected: 10 min of RSI traffic, zero motion, zero ipoc_jumps,
timeout counters flat, clean stop.
Abort: any motion during the soak (e-stop immediately — correction
path is not zero: check RSI context POSCORR limits and RKorr mapping
before ANY later stage); `ipoc_jumps > 0` or climbing
`total_timeouts` (network/cycle-time problem: fix cabling/QoS first).

## Stage 3 - small DIRECT_CARTESIAN stream jog (spec 15.4 step 3)

- [ ] Start stream mode through the manager:
      `rosservice call /soft_robot/start_servo "{mode: 1, profile: 1}"`
      (MODE_DIRECT_CARTESIAN, PROFILE_PRECISION);
      `manager_state.system_state: 3`,
      `active_controller: cartesian_correction_controller`.
- [ ] Jog +X with a tiny per-cycle correction (use
      `rostopic pub -s -r 100` — the `-s` is mandatory, Plan 3
      follow-up 2, so the stamp re-evaluates per message):

      rostopic pub -s -r 100 /soft_robot/cartesian_correction_command \
          soft_robot_msgs/CartesianCorrectionStamped \
          "{header: {stamp: now}, correction: {x: 0.05}}"

      Values are mm/deg PER 4 ms RSI CYCLE (~12.5 mm/s here), clamped by
      `safety.max_corr_trans: 0.5` mm/cycle.
- [ ] Robot creeps in +X of the RSI POSCORR frame (BASE); direction and
      magnitude match expectation.
- [ ] Ctrl-C the `rostopic pub`: motion stops within
      `stream_timeout: 0.1` s (stale stamp -> zero output).
- [ ] Repeat one axis at a time for Y/Z (and optionally a/b/c with
      correction <= 0.02 deg/cycle) — one axis per run.
- [ ] `rosservice call /soft_robot/stop_servo`: back to
      `system_state: 2` (READY), RSI program stopped.

Expected: motion only while messages flow, correct axis/sign/speed,
instant stop on publisher kill, clean return to READY.
Abort: wrong direction or frame (fix BASE/tool setup before
continuing); motion continues after the publisher stops (stream
timeout broken — do not proceed to any force stage).

## Stage 4 - goal-mode orientation move (spec 15.4 step 4)

Conservative yaml defaults `p_gain: 1.0` / `max_speed_dps: 5.0` are
intentional (Plan 3 follow-up 9); tune per spec 14 AFTER a clean run,
record final values.

- [ ] Start stream mode as in Stage 3 (goal mode runs on
      `cartesian_correction_controller`).
- [ ] Send a small orientation goal (~10 deg away from current A, B/C
      unchanged, `use_position: false`) on
      `/soft_robot/move_to_orientation` (e.g. via `axclient.py` or
      `rostopic pub -1 .../goal`), `speed_scale: 1.0`.
- [ ] Motion is smooth, <= ~5 deg/s, converges and holds; result
      `result_code: 0` (CONVERGED), reported final error <= `tol_deg: 0.1`.
- [ ] Timeout guard: send a goal with `speed_scale: 0.1` and a far pose
      so it cannot converge within `timeout_s: 30.0`; expect
      `result_code: 1` (TIMEOUT) and zero output afterwards.
- [ ] Tuning (only after a clean first run): raise `goal.p_gain` /
      `goal.max_speed_dps` per spec section 14; do NOT treat plan/test
      numbers as expected process values (Plan 3 follow-up 9). Record
      final tuned values: p_gain = ____, max_speed_dps = ____.
- [ ] `rosservice call /soft_robot/stop_servo` -> READY.

Expected: converged goal at conservative speed, clean timeout
behavior, tuned values recorded.
Abort: overshoot/oscillation at defaults (mechanical or frame issue —
do not "fix" by raising gains); ABORTED results (`result_code: 2`)
with RSI healthy.

## Stage 5 - SRI wrench display only (spec 15.4 step 5)

No servo mode. Manager in READY.

- [ ] `rosservice call /soft_robot/zero_sensor` in READY: succeeds
      (gate open in CONNECTED/READY, decision 11).
- [ ] `rostopic echo /sri_ft/wrench_raw`: push/pull the tool by hand in
      +X/+Y/+Z and torque it about each axis; sign and rough magnitude
      match the sensor datasheet frame.
- [ ] Release the tool: wrench returns to ~0 (post-tare) with only
      noise.
- [ ] Record noise floor (1 min, tool untouched): |F| < ____ N,
      |T| < ____ Nm (feeds deadband sanity in Stages 6-7).

Expected: physically plausible, correctly signed wrench; stable
zero after tare.
Abort: axis swap/sign errors (fix sensor mounting convention or
document the mapping before any compliance stage).

## Stage 6 - PRECISION low-gain compliance (spec 15.4 step 6)

- [ ] Tare first (READY): `rosservice call /soft_robot/zero_sensor`.
- [ ] `rosservice call /soft_robot/start_servo "{mode: 2, profile: 1}"`
      (MODE_FORCE_COMPLIANCE, PROFILE_PRECISION: fixed deadband
      `deadband_force: 30.0` N / `deadband_torque: 4.0` Nm, low gains,
      `max_speed_translation: 10.0` mm/s).
- [ ] Hands off: robot stands still (noise floor from Stage 5 is far
      below the 30 N deadband).
- [ ] Push steadily above 30 N on one axis: robot yields slowly in the
      push direction; release: motion stops.
- [ ] While SERVOING: `rosservice call /soft_robot/zero_sensor` is
      REJECTED (`success: False` — gate closed, decision 11).
- [ ] `rosservice call /soft_robot/stop_servo` -> READY.

Expected: dead below 30 N, gentle bounded yield above it, no drift,
zero gate closed while servoing.
Abort: motion with hands off (tare/payload wrong — run Stage 8 before
retrying); runaway acceleration (gain/sign error — e-stop).

## Stage 7 - DRAG adaptive deadband (spec 15.4 step 7)

VERIFY the FLim/TLim activation-deadband interpretation (spec 7.2
note); if they behaved as cutoffs on the legacy cell, only the
deadband initialization mapping changes.

- [ ] Tare in READY, then
      `rosservice call /soft_robot/start_servo "{mode: 2, profile: 0}"`
      (PROFILE_DRAG: `adaptive_deadband: true`,
      `ramp_window_s: 2.0`, `ramp_force_margin_n: 5.0`).
- [ ] During the first ~2 s ramp window the robot must NOT move even
      with a hand resting on the tool (deadband ramps to standing
      force + 5 N margin).
- [ ] After the ramp: light guided pull moves the robot smoothly
      (drag feel); stopping the pull stops the motion.
- [ ] Interpretation check (spec 7.2): confirm forces just under
      (standing + margin) produce NO motion and forces above it
      produce motion — i.e. FLim/TLim act as an activation deadband.
      If the legacy cell behavior turns out to be a safety cutoff
      instead, change only the deadband initialization mapping
      (spec 7.2 note), not the controller structure. Record verdict:
      deadband / cutoff = ____.
- [ ] Auto re-tare sanity (`auto_retare: true`): reorient the tool by
      > `retare_orientation_tol_deg: 1.0` deg via a light drag, let it
      settle; no force drift accumulates afterwards.
- [ ] `rosservice call /soft_robot/stop_servo` -> READY.

Expected: zero output through the ramp window, natural drag
afterwards, interpretation verdict recorded.
Abort: motion during the ramp window (adaptive deadband not armed —
stop, this is the spec 7.4 safety property); drift after re-tare.

## Stage 8 - payload calibration (spec 15.4 step 8)

After success check payload.yaml, r2_force/r2_torque >= 0.99 on a
rigid tool; re-run reproducibility: gravity_n within 2%.

- [ ] Rigid, known tool mounted; manager in READY; clear space for the
      8 calibration poses of `calibration.yaml` (A/B/C up to +-45 deg,
      goal speed <= 5 deg/s).
- [ ] Send the action goal:

      rostopic pub -1 /soft_robot/calibrate_payload/goal \
          soft_robot_msgs/CalibratePayloadActionGoal '{}'

- [ ] `system_state: 4` (CALIBRATING); feedback phases cycle
      MOVE -> SETTLE -> SAMPLE per pose (8 poses, 100 samples each,
      `settle_time_s: 1.0`); robot returns to `return_pose`
      (A=B=C=0) at the end.
- [ ] While CALIBRATING: `rosservice call /soft_robot/zero_sensor`
      REJECTED (gate, decision 11 — driver-side bias stays constant
      through the whole capture).
- [ ] Result: `success: True`, `r2_force >= 0.99`,
      `r2_torque >= 0.99` (rigid tool); `gravity_n` within ~10% of
      m*g for the known tool mass; com plausible (m, in the sensor
      frame). Record: gravity_n = ____ N, com = (____, ____, ____) m,
      r2_force = ____, r2_torque = ____.
- [ ] `payload.yaml` written under
      `ros_ws/src/soft_robot_bringup/config/payload.yaml` and its
      values match the action result
      (`/force_compliance_controller/payload/*` keys).
- [ ] Reproducibility: run the action a second time; `gravity_n`
      within 2% of the first run. Record both values.
- [ ] Boot persistence: restart `soft_robot.launch`; the calibrated
      payload parameters are on the parameter server before the
      controllers load (launch-prefix `load_payload_then_exec.sh`),
      and Stage 6 hands-off stillness now holds with the calibrated
      values.
- [ ] Failure path (optional but recommended): cancel one run
      mid-sequence; sequence terminates without a return move,
      system back to READY, `payload.yaml` NOT rewritten.

Expected: r2 >= 0.99 both channels, gravity_n reproducible within
2%, payload.yaml persisted and effective after restart.
Abort: r2 < 0.99 on a rigid tool (mounting slack, goal tolerance
loosened > 0.5 deg, or wrench axis mapping wrong — see plan risk 4);
MOVE_FAILED / STREAM_LOST results.

## Stage 9 - full workflow through the manager (spec 15.4 step 9; web UI in Plan 6)

One uninterrupted operator session driven ONLY by the manager
interface (`/soft_robot/*` services + `manager_state`; the web UI
consumes exactly this surface in Plan 6):

- [ ] Cold start -> READY (tool synced, payload loaded from Stage 8).
- [ ] zero_sensor (READY) -> start_servo DRAG -> guided drag work ->
      stop_servo.
- [ ] start_servo PRECISION -> contact task sample -> stop_servo.
- [ ] start_servo DIRECT_CARTESIAN -> short stream jog -> goal-mode
      orientation move -> stop_servo.
- [ ] calibrate_payload once more (end-to-end sanity) -> READY.
- [ ] Throughout: `/soft_robot/manager_state` (10 Hz) and
      `/soft_robot/diagnostics` (1 Hz) stay consistent with every
      transition; no manual `rostopic pub /soft_robot/mode_command`
      used anywhere (manager is the only mode-command producer).

Expected: entire session through manager services only; states
observed: 2 -> 3 -> 2 -> 3 -> 2 -> 3 -> 2 -> 4 -> 2 with no FAULT.
Abort: any transition the manager refuses that an operator would
need (record message verbatim — it is a Plan 6 UX input).

## Recovery drills (do all three)

- [ ] Kill RSI mid-SERVOING -> FAULT -> /soft_robot/reset_fault path.
      Procedure: in a Stage 3 jog, deselect/stop the KRL program (or
      unplug the RSI segment) -> `/kuka/rsi/state` `fault: True`,
      manager `system_state: 6` (FAULT, latched even after the link
      is restored). Restore the KUKA side, then
      `rosservice call /soft_robot/reset_fault` (clears the latch via
      `/kuka/rsi/reset_fault` + `/kuka/eki/reset_fault`; cumulative
      counters must survive: `total_timeouts`/`bad_frames` NOT reset)
      -> back to READY; a new start_servo works.
- [ ] EKI cable pull -> manager OFFLINE within `eki_state_timeout_s`
      (5.0 s, manager.yaml). Manager in READY (not servoing), pull
      the EKI network path: `system_state: 0` within ~5 s. Replug:
      bridge auto-reconnects, tool re-syncs, manager returns to READY
      without operator action.
- [ ] /soft_robot/zero_sensor rejected while SERVOING (decision 11).
      In any Stage 6/7 run: call it, expect `success: False` with a
      gate message; confirm `/sri_ft/zero` was NOT forwarded
      (`zero_active` never goes true in `/sri_ft/status`). Reminder:
      calling `/sri_ft/zero` directly bypasses the gate — debugging
      only, never while SERVOING (bringup README warning).

Expected: all three drills recover to READY without restarting any
ROS node.
Abort: FAULT does not latch, reset does not clear, or reset wipes
cumulative counters (Plan 4 follow-up 10 regression).
