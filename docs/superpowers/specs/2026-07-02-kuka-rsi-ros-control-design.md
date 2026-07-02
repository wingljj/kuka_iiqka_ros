# KUKA RSI + EKI + ROS Control Design

Date: 2026-07-02
Status: Approved design (supersedes `ref/2026-07-02-kuka-rsi-ros-control-design.md`)

## 1. Background

This design is based on the existing `SoftRobot 2.0` MFC project. The original
system controls a KUKA robot through RSI by sending Cartesian correction values
through `RKorr X/Y/Z/A/B/C`. It also reads a force/torque sensor, applies force
compensation and soft compliance logic, and writes the resulting correction back
to KUKA in a realtime loop.

The new system will reimplement the same business behavior with ROS1 Noetic and
`ros_control`, while splitting the old monolithic application into maintainable
modules.

Confirmed target:

- Robot controller: KR C5.
- KUKA software: iiQKA.OS2 9.x, version 9.2 or newer.
- KUKA options: iiQKA.RobotSensorInterface 6.2 and iiQKA.EthernetKRL 6.1,
  based on the local manuals in `ref/`.
- KUKA communication: RSI for realtime servo, EKI for non-realtime management.
- ROS: ROS1 Noetic + `ros_control`.
- Main command semantic: Cartesian RSI correction through `RKorr X/Y/Z/A/B/C`.
- Sensor: SRI six-axis force/torque sensor.
- Development language: C++ for ROS/backend modules.
- Operator interface: web interface is allowed and preferred.
- Six-dimensional mouse: not required. The legacy mouse input path
  (`Mouse`, `SLine` scaling) is removed entirely.
- KUKA side and ROS side must both be designed.

## 2. Existing Project Findings

CodeGraph analysis of the current project shows:

- `CSoftRobotDlg` is the main UI and control hub.
- `SOFTMOVE` is the central control thread.
- `SOFTMOVE` reads mouse input, force/torque data, and KUKA Cartesian state.
- It calls force compensation and soft motion logic.
- It sends output through:

```cpp
KukaRobot::GetInstance()->SetRKorr(
    SoftRob.SpeedX,
    SoftRob.SpeedY,
    SoftRob.SpeedZ,
    SoftRob.SpeedA,
    SoftRob.SpeedB,
    SoftRob.SpeedC);
```

The current KUKA interface exposes both Cartesian pose and joint angle feedback:

- Cartesian feedback: `x_pos/y_pos/z_pos/a_pos/b_pos/c_pos`.
- Joint feedback: `A1_ang` to `A6_ang`.
- Command output: `SetRKorr(x, y, z, a, b, c)`.

The project contains `ExternalData.xml` with both `RKorr` and `AKorr`, but the
visible application code only uses `RKorr`. Therefore the ROS system keeps the
same Cartesian correction interface and does not command joint angles in the
first version.

### 2.1 Legacy Behavior Inventory

Source analysis of `SoftRobotDlg.cpp` and the DLL headers identified these
business behaviors. Each is marked with its disposition in the new system:

| Legacy behavior | Location | Disposition |
|---|---|---|
| Force-compliance control law `FT2Move` | `SoftControl.dll` (no source, header only) | Re-designed as a standard admittance law (Section 7.2) |
| Adaptive threshold ramp at startup | `LIMSET` thread | Kept: startup adaptive deadband (Section 7.4) |
| Auto re-zero on return to start orientation | `SOFTMOVE` + `isReset`/`SetZeroPoint` | Kept: auto re-tare (Section 7.5) |
| Hard cutoff at 500 N total force | `SOFTMOVE` | Kept: configurable global hard cutoff (Section 12.1) |
| 8-pose payload calibration driven by RSI corrections | `MASSDATA` + `RotFixAng` | Kept: ROS drives poses through `RKorr` goal mode (Section 9) |
| Tool parameters read from robot at startup | `GETTOOL` + `KukaRobot::GetTool` | Kept: read through EKI (Section 6.4) |
| Grease-application mode (Mode 8, `GuiZhiWidth/GuiZhiSpeed`, `Width`) | `SOFTMOVE`, `SoftMove` | Not in first version; future work |
| Six-D mouse input | `Mouse`, `SLine` | Removed |

Key constraint: the core compliance algorithm `FT2Move` exists only as a
compiled DLL. Its header (`SoftControl.h`) exposes parameter names
(`MRC/MRX/MRP`, `LRC/LRX/LRP`, `XRC/XRX/XRP`, `XZSPD/MZSPD`, mode selectors),
but not the algorithm. The confirmed decision is to design a clean, standard
admittance control law whose parameter structure mirrors the legacy grouping,
rather than reverse-engineering the DLL.

## 3. Architecture Choice

The selected approach is:

> Use KUKA-side RSI/EKI configuration appropriate for iiQKA.OS2 9.x, and implement the
> ROS1 Noetic upper computer side as a custom `ros_control` system.

This keeps the KUKA side aligned with standard KUKA RSI/EKI practice while
allowing the ROS side to preserve the current application's Cartesian correction
behavior and force-control workflow.

Rejected alternatives:

- Full self-contained custom stack without following KUKA-side conventions:
  flexible, but higher risk on KR C5/iiQKA.OS2 deployment.
- ROS2/KUKA external-control stack bridged into ROS1:
  reusable in some cases, but adds an extra middleware layer and does not match
  the ROS1 `ros_control` requirement.
- Upper-computer joint-angle control:
  useful for advanced planning and null-space control, but not aligned with the
  original RSI `RKorr` behavior.
- Reverse-engineering `SoftControl.dll` by input/output recording:
  highest fidelity to legacy behavior, but large effort and the resulting
  algorithm would remain unexplainable. Rejected in favor of a standard
  admittance law.

## 4. System Overview

The system is split into a realtime path and a management path.

Realtime path:

```text
SRI FT Sensor
  -> sri_force_torque_driver
  -> SoftComplianceController
  -> kuka_rsi_hw_interface
  -> RSI RKorr
  -> KUKA KR C5 / iiQKA.OS2
```

Management path:

```text
Web UI / CLI
  -> soft_force_control_manager
  -> kuka_eki_bridge
  -> EKI
  -> KUKA KRL program
```

High-level data flow:

```text
SRI FT Sensor
   |
   v
sri_force_torque_driver
   |
   v
SoftComplianceController <---- RSI robot state ---- kuka_rsi_hw_interface
   |
   v
CartesianCorrectionCommandInterface
   |
   v
kuka_rsi_hw_interface
   |
   v
RSI RKorr X/Y/Z/A/B/C
   |
   v
KUKA

soft_robot_web_interface --> soft_force_control_manager --> kuka_eki_bridge --> KUKA EKI
```

Core rules:

- RSI is the realtime channel.
- EKI is the management channel.
- Web/UI actions never enter the realtime loop directly.
- Force-control computation is performed in a `ros_control` controller.
- Algorithm code is reusable C++ and testable without ROS.
- KUKA-side safety remains authoritative.

## 5. ROS Package Layout

The workspace should use a catkin layout:

```text
ros_ws/src/
  kuka_rsi_hw_interface/
  kuka_eki_bridge/
  soft_robot_controllers/
  soft_force_control_core/
  soft_force_control_manager/
  sri_force_torque_driver/
  soft_robot_web_interface/
  soft_robot_bringup/
  soft_robot_msgs/
```

### 5.1 `kuka_rsi_hw_interface`

Purpose:

- Implement `hardware_interface::RobotHW`.
- Own RSI UDP communication.
- Receive KUKA state and `IPOC`.
- Send `RKorr X/Y/Z/A/B/C`.
- Enforce communication watchdogs and final command limits.

Main class:

```cpp
class KukaRsiRobotHW : public hardware_interface::RobotHW {
public:
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);
};
```

Registered interfaces:

- `JointStateInterface`: read-only `A1` to `A6`.
- `CartesianStateInterface`: read-only `X/Y/Z/A/B/C`.
- `CartesianCorrectionCommandInterface`: write-only `RKorr X/Y/Z/A/B/C`.
- Optional `RobotModeStateInterface`: RSI, EKI, watchdog, and KUKA program state.

The hardware interface must not contain force-control logic.

### 5.2 `kuka_eki_bridge`

Purpose:

- Own EKI TCP/XML management communication.
- Handle KUKA program handshake.
- Start/stop external-control workflow.
- Set operating mode.
- Reset non-realtime faults.
- Synchronize tool/base/configuration state.

Services:

- `/kuka/eki/connect`
- `/kuka/eki/start_rsi_program`
- `/kuka/eki/stop_rsi_program`
- `/kuka/eki/set_mode`
- `/kuka/eki/reset_fault`
- `/kuka/eki/set_tool_base`
- `/kuka/eki/get_tool` — query the currently active `$TOOL` data from the
  controller (see Section 6.4).

Topics:

- `/kuka/eki/state`
- `/kuka/diagnostics`

The EKI bridge must not be used for per-cycle control.

### 5.3 `soft_robot_controllers`

Purpose:

- Provide `ros_control` controller plugins.

Controllers:

1. `SoftComplianceController`

   This is the main realtime controller. It reads:

   - KUKA Cartesian state.
   - KUKA joint state.
   - Latest SRI wrench sample through a realtime-safe buffer.
   - Current parameter snapshot.
   - Current mode snapshot.

   It computes `RKorr X/Y/Z/A/B/C` inside `update()`.

2. `CartesianCorrectionController`

   A direct Cartesian correction controller with two command sources:

   - **Stream mode**: external per-cycle correction commands, used for
     debugging, web jog, and low-level RSI verification. Supports command
     timeout and zero output on stale command.
   - **Goal mode**: the manager sends a target orientation (and optionally
     position) through an action. The controller uses `OrientationMotionCore`
     (Section 7.6) to generate rate-limited corrections each cycle until the
     target converges, then reports completion. This is the motion engine for
     payload calibration and replaces the legacy `RotFixAng`/`MoveToTarget`
     behavior.

   Only one controller may claim the `CartesianCorrectionCommandInterface`
   at a time; the manager swaps controllers on mode change through the
   `controller_manager`.

### 5.4 `soft_force_control_core`

Purpose:

- Pure C++ algorithm library.
- No ROS dependency.
- No socket access.
- No file I/O in realtime functions.

Components:

- `ForceTorqueFilter`
- `ToolGravityCompensator`
- `PayloadEstimator`
- `ComplianceLaw`
- `SafetyLimiter`
- `ModeManagerCore`
- `OrientationMotionCore` — rate-limited goal-seeking correction generator
  (Section 7.6).

This package is where the original `SoftControl` and `LoadMass` behavior is
reimplemented in testable form.

### 5.5 `soft_force_control_manager`

Purpose:

- Non-realtime process manager.
- Own state machine.
- Own parameter loading/saving.
- Own calibration workflow.
- Serve web UI requests through ROS service/action APIs.

Responsibilities:

- Mode switching.
- Start/stop requests.
- Payload calibration action (orchestration; Section 9).
- SRI zero/tare requests.
- Tool parameter synchronization from EKI (Section 6.4).
- YAML persistence.
- Fault acknowledgement.
- Logging and diagnostic aggregation.

### 5.6 `sri_force_torque_driver`

Purpose:

- Communicate with the SRI six-axis force/torque sensor.
- Publish standard ROS wrench data.
- Provide sensor zero/tare services.

Topics:

- `/sri_ft/wrench_raw`: `geometry_msgs/WrenchStamped`
- `/sri_ft/status`

Services:

- `/sri_ft/zero`
- `/sri_ft/set_filter`

The driver should not implement KUKA-specific control behavior.

### 5.7 `soft_robot_web_interface`

Purpose:

- Provide a browser-based operator interface.

Recommended first version:

- Frontend: Vue or React.
- ROS communication: `rosbridge_suite`.

Industrial deployment option:

- Replace rosbridge with a controlled C++ WebSocket backend.

The web UI must call manager services/actions. It must not directly write RSI
commands.

### 5.8 `soft_robot_bringup`

Purpose:

- Launch files.
- YAML parameters.
- URDF/xacro if needed.
- Controller configuration.
- Network configuration examples.
- KUKA deployment files.
- Diagnostic startup scripts.

## 6. KUKA iiQKA RSI/EKI/KRL Design

The authoritative local manuals for this design are:

- `ref/iiQKA_RobotSensorInterface_62_zh.pdf`
- `ref/iiQKA_EthernetKRL_61_zh.pdf`

Important constraints from these manuals:

- iiQKA.RobotSensorInterface 6.2 targets iiQKA.OS2.
- iiQKA.EthernetKRL 6.1 targets iiQKA.OS2.
- The controller-side software prerequisite is iiQKA.OS2 9.x, version 9.2 or
  newer.
- RSI Ethernet communication uses UDP/IP and custom XML data groups configured
  in an XML file.
- RSI signal processing is calculated at a 4 ms sensor cycle.
- EKI does not provide deterministic timing and is not suitable for cyclic
  realtime communication at the robot controller cycle. It is therefore only a
  management channel in this design.
- iiQKA.RobotSensorInterface 6.x uses the newer KRL function names:
  `RSI_LOAD`, `RSI_UNLOAD`, `RSI_ACTIVATE`, `RSI_DEACTIVATE`,
  `RSI_PROCESS_ON`, `RSI_PROCESS_OFF`, `RSI_MOVECORR`,
  `RSI_GETPARAMETER`, and `RSI_SETPARAMETER`.
- iiQKA.EthernetKRL 6.x uses the newer EKI function style:
  `EKI_Load`, `EKI_Open`, `EKI_Close`, `EKI_Unload`,
  `EKI_ReadNext`, `EKI_Get*`, `EKI_Set*`, and `EKI_Send`.
  Old `EKI_Init` and `EKI_Clear` behavior is only retained as migration
  forwarding and should not be used in new code.

Cartesian correction coordinate semantics:

- RSI distinguishes axis correction from Cartesian correction.
- Axis correction uses `AxisCorr` / `AxisCorrExt`.
- Cartesian correction uses `PosCorr`.
- For Cartesian correction, the correction coordinate system origin is always
  the active TCP.
- The correction coordinate system orientation can be based on `BASE`,
  `ROBROOT`, `TOOL`, `WORLD`, or `TTS`.
- This design should make the integration coordinate system explicit in
  configuration. The default should be `BASE` to preserve the current project's
  observed `RKorr X/Y/Z/A/B/C` behavior, but `TOOL` should remain a configured
  option for future experiments.
- Relative correction should be the default integration mode.

KUKA-side files:

```text
kuka/
  krl/
    ROS_RSI_SERVO.SRC
    ROS_RSI_SERVO.DAT
  rsi/
    ROS_RSI_CONTEXT.rsi
    ROS_RSI_CONFIG.xml
  eki/
    ROS_EKI_CONFIG.xml
```

### 6.1 RSI Realtime Channel

KUKA to ROS:

```xml
<Rob Type="KUKA">
  <RIst X="" Y="" Z="" A="" B="" C="" />
  <AIPos A1="" A2="" A3="" A4="" A5="" A6="" />
  <Delay D="" />
  <Mode M="" />
  <IPOC></IPOC>
</Rob>
```

ROS to KUKA:

```xml
<Sen Type="ROS">
  <RKorr X="" Y="" Z="" A="" B="" C="" />
  <Stop S="" />
  <Watchdog W="" />
  <IPOC></IPOC>
</Sen>
```

Rules:

- ROS must echo the received `IPOC`.
- `RKorr` is a per-cycle Cartesian correction represented by the ROS side.
  In the iiQKA RSI context, the actual correction object should be `PosCorr`
  and should map the XML receive fields to Cartesian correction values.
- `AKorr` is not used in the first version.
- The legacy `EKorr`, `Tech`, `DiO`, and `Width` fields from
  `ExternalData.xml` are not carried over.
- Missing or stale `IPOC` causes zero output and fault handling.
- KUKA-side RSI configuration must clamp or reject invalid correction values.

### 6.2 EKI Management Channel

EKI is used for:

- Program ready/active/fault state.
- Start/stop handshake.
- Mode selection.
- Tool/base configuration and tool query (Section 6.4).
- Fault reset.
- Non-realtime diagnostic text.

EKI is not used for:

- Per-cycle `RKorr`.
- Force-control law calculation.
- Sensor streaming.

### 6.3 KRL Program Responsibilities

`ROS_RSI_SERVO.SRC` should:

1. Select the configured tool and base.
2. Load and open the EKI management channel with `EKI_Load` and `EKI_Open`.
3. Wait for ROS manager ready.
4. Answer management queries (including the tool query, Section 6.4).
5. Load the configured RSI context with `RSI_LOAD`.
6. Activate the RSI context with `RSI_ACTIVATE`.
7. Start RSI signal processing with `RSI_PROCESS_ON`, using relative Cartesian
   correction and the configured integration coordinate system.
8. Enter sensor-guided motion with `RSI_MOVECORR` when pure sensor-guided
   motion is required.
9. Handle stop/fault/watchdog transitions.
10. Exit safely with zero correction.

The KRL program should not contain the soft compliance algorithm.

### 6.4 Tool Parameter Synchronization (legacy `GetTool`)

The legacy system read the active tool data from the robot at startup and
pushed it into both the sensor gravity compensation and the compliance module.
The new system keeps this behavior through EKI:

- KRL side: on receiving a tool-query command over EKI, the KRL program
  packs the currently active `$TOOL` frame (X/Y/Z/A/B/C) with `EKI_Set*`
  and returns it with `EKI_Send`.
- ROS side: `kuka_eki_bridge` exposes `/kuka/eki/get_tool`. The manager calls
  it automatically during the `CONNECTED -> READY` transition and whenever the
  operator changes the tool, then updates the realtime parameter snapshot used
  by `ToolGravityCompensator` and `SoftComplianceController`.
- YAML tool parameters exist only as a simulation/offline fallback. On real
  hardware the EKI-read values are authoritative; if YAML and EKI values
  disagree beyond a tolerance, the manager publishes a warning diagnostic.

## 7. Realtime Controller Design

### 7.1 `SoftComplianceController`

The controller update loop:

```text
read KUKA Cartesian state
read KUKA joint state
read latest SRI wrench sample from realtime buffer
read parameter snapshot
read mode snapshot
if mode is not SERVOING: output zero
if wrench timeout: output zero and set degraded/fault flag
filter wrench
apply tool gravity compensation
update adaptive deadband if in startup ramp (7.4)
check auto re-tare condition (7.5)
compute compliance law (7.2)
apply correction limits
apply correction rate limits
apply global hard cutoff (12.1)
write RKorr command
publish realtime-safe status snapshot
```

No blocking calls are allowed in `update()`:

- No file I/O.
- No parameter server calls.
- No service calls.
- No dynamic allocation in the hot path.
- No socket operations except through `RobotHW` read/write.

### 7.2 `ComplianceLaw`: Standard Admittance Law

The legacy `FT2Move` algorithm is unavailable (DLL only). The confirmed
decision is a clean per-axis admittance law whose parameter structure mirrors
the legacy grouping so behavior can be tuned close to the old system:

```text
for each axis i in {X, Y, Z, A, B, C}:
  e_i = deadzone(F_comp_i, deadband_i)
  v_i = K_i * e_i * speed_scale
  v_i = clamp(v_i, -v_max_i, +v_max_i)
  v_i = rate_limit(v_i, prev_v_i, dv_max_i)
```

- `F_comp` is the filtered, gravity-compensated wrench.
- Translation axes (X/Y/Z) are driven by forces; rotation axes (A/B/C) by
  torques.
- Gains, deadbands, and limits are per-axis-group (translation/rotation),
  matching the legacy `*RC/*RX/*RP` gain grouping and `XZSPD/MZSPD` speed caps.

Interpretation note (to verify during commissioning): the legacy `FLim/TLim`
values are interpreted as **activation deadbands** (motion starts only when
force exceeds residual + margin), based on `LIMSET` setting them to
`5 + FSum` / `1 + TSum` and the auto re-zero condition requiring
`FSum < FLim`. If commissioning shows they acted as safety cutoffs instead,
only the deadband initialization mapping changes; the law structure is
unaffected.

### 7.3 Parameter Profiles

`FORCE_COMPLIANCE` supports two named parameter profiles, reproducing the
legacy Mode 1-2 vs Mode 3-6 split:

- **DRAG** (legacy Modes 1-2): startup adaptive deadband (7.4), auto re-tare
  enabled (7.5). Intended for hand-guiding.
- **PRECISION** (legacy Modes 3-6): fixed deadband, default 30 N / 4 Nm
  (values taken from the legacy `LIMSET` hardcoded limits). Adaptive startup
  and auto re-tare disabled.

Profiles live in `force_control.yaml` and are selected through the manager.
The exact numeric mapping from the legacy `M/L/X` gain groups to the two
profiles is finalized during commissioning and then fixed as YAML defaults.

### 7.4 Startup Adaptive Deadband (legacy `LIMSET`)

When entering `FORCE_COMPLIANCE` with the DRAG profile:

1. For a configurable ramp window (default 2 s, matching the legacy
   500 x 4 ms loop), the controller outputs zero and measures the residual
   compensated wrench.
2. At the end of the window, the deadband is set to
   `residual + margin` (default margin 5 N force / 1 Nm torque, from the
   legacy `FTLimSet(5+FSum, 1+TSum)`).
3. Normal compliance output then begins.

The PRECISION profile skips the ramp and uses its fixed deadband directly.

### 7.5 Auto Re-Tare (legacy `isReset` / `SetZeroPoint`)

DRAG profile only, configurable on/off:

- On entering `SERVOING`, the controller records the start orientation.
- When the orientation returns to the start orientation (within a configurable
  tolerance) and the compensated wrench magnitude is below the deadband, the
  current residual wrench is absorbed into the runtime zero bias.
- This is a pure in-memory state update inside the realtime-safe core; it
  never writes files or calls services from `update()`.

### 7.6 `OrientationMotionCore` and Goal Mode

`OrientationMotionCore` (in `soft_force_control_core`) reimplements the legacy
`RotFixAng`/`MoveToTarget` behavior as a deterministic, testable component:

- Input: current pose, target orientation A/B/C (optionally position X/Y/Z).
- Output: a rate-limited per-cycle Cartesian correction toward the target,
  reproducing the legacy `SpeedV`/`SpeedR` trapezoidal speed shaping.
- Convergence: pose error below tolerance held for a configurable time.
- Timeout: if the target is not reached within a configurable duration, the
  motion aborts with an error result and zero output.

`CartesianCorrectionController` hosts this core in its goal mode (Section 5.3).
The manager commands goals through an action:

```text
/soft_robot/move_to_orientation  (action)
  goal:    target A/B/C (+ optional X/Y/Z), speed scale
  feedback: pose error, progress
  result:  converged | timeout | aborted
```

### 7.7 Direct Cartesian Stream Mode

The stream mode of `CartesianCorrectionController` is retained for:

- Commissioning.
- Web jog.
- Low-level RSI verification.
- Replay tests.

It supports command timeout and zero output on stale command.

## 8. SRI Force/Torque Sensor and Compensation

The SRI driver publishes raw data. The controller performs realtime-safe use of
the latest data.

Data path:

```text
sri_force_torque_driver
  -> /sri_ft/wrench_raw
  -> realtime buffer in SoftComplianceController
  -> filter
  -> gravity compensation
  -> compliance law
```

Timeout rule:

- If wrench data is older than the configured threshold, output zero `RKorr`.
- Default threshold should be 2 to 3 RSI cycles.

The compensation model should support:

- Sensor zero bias (static from calibration + runtime re-tare updates).
- Tool gravity vector.
- Tool center-of-mass parameters.
- Force/torque deadband.
- Low-pass filtering.
- Maximum force/torque threshold.

## 9. Payload Calibration Workflow

Payload calibration is a non-realtime action managed by
`soft_force_control_manager`. The robot is driven through the calibration
poses by the ROS side via `RKorr` goal mode, faithful to the legacy
`MASSDATA`/`RotFixAng` approach.

Action name:

```text
/soft_robot/calibrate_payload
```

Flow:

1. Manager verifies system is in `READY`.
2. Manager requests `CALIBRATING`; `controller_manager` switches from
   `SoftComplianceController` to `CartesianCorrectionController` (goal mode).
3. For each of the calibration poses (default: the legacy 8-orientation
   sequence, stored in `calibration.yaml`):
   a. Manager sends the target orientation through
      `/soft_robot/move_to_orientation`.
   b. Waits for convergence (timeout aborts the calibration safely).
   c. Waits a settle time (default 1 s, from legacy).
   d. Collects ~100 wrench + robot-state samples and averages them.
      (Deliberate improvement: the legacy system took a single sample per
      pose, which is noise-sensitive.)
4. `PayloadEstimator` solves a least-squares fit for payload gravity `G`,
   center of mass `Gx/Gy/Gz`, and sensor zero bias `Fx0..Tz0`, and reports a
   fit-quality index (equivalent to the legacy `R1*R2` indicator) so the web
   UI can judge calibration quality.
5. Manager returns the robot to the initial orientation (legacy behavior:
   `RotFixAng(0, 0, 5)` equivalent, configurable).
6. Manager writes `payload.yaml`.
7. Manager updates the realtime parameter snapshot.
8. System returns to `READY`.

Calibration must not run while `SERVOING`. Compliance output is disabled for
the entire calibration.

## 10. Modes

Supported first-version modes:

- `IDLE`: always zero output.
- `DIRECT_CARTESIAN`: direct Cartesian correction (stream or goal mode) for
  commissioning and jog. (`CALIBRATION` reuses the same controller in goal
  mode, but is its own system mode.)
- `FORCE_COMPLIANCE`: main SRI force-compliance mode, with the DRAG or
  PRECISION parameter profile (Section 7.3).
- `CALIBRATION`: payload calibration workflow; compliance disabled; motion
  through goal mode only.

Legacy mode mapping:

| Legacy mode | New system |
|---|---|
| 1-2 | `FORCE_COMPLIANCE` + DRAG profile |
| 3-6 | `FORCE_COMPLIANCE` + PRECISION profile |
| 7 | `FORCE_COMPLIANCE` + PRECISION profile (the legacy code itself remapped 7 to 4) |
| 8 (grease application) | Not implemented in the first version; future work. The `Width` field is not part of the RSI XML. |

Mode changes are requested through the manager. The controller only consumes
the current mode snapshot.

## 11. State Machine

System states:

```text
OFFLINE
CONNECTED
READY
SERVOING
CALIBRATING
DEGRADED
FAULT
```

State meanings:

- `OFFLINE`: KUKA/EKI/RSI not connected.
- `CONNECTED`: EKI connected, KUKA program online, RSI not servoing.
- `READY`: RSI, sensor, controller, and parameters are ready; output is zero.
  Tool parameters have been synchronized through EKI (Section 6.4).
- `SERVOING`: `SoftComplianceController` is actively producing `RKorr`.
- `CALIBRATING`: payload calibration is running; compliance output is disabled.
- `DEGRADED`: nonfatal problem; output policy depends on the specific fault.
- `FAULT`: fatal problem; output zero and require explicit reset.

Entering `SERVOING` requires:

- EKI connected.
- KUKA program ready.
- RSI communication valid.
- SRI wrench valid.
- Controller loaded and running.
- Safety parameters loaded.
- Tool parameters synchronized.
- Explicit user start command.

## 12. Safety Design

Safety is layered.

### 12.1 Controller Layer

The controller must enforce:

- `RKorr` magnitude limits.
- `RKorr` rate limits.
- Wrench timeout zeroing.
- Force/torque threshold zeroing.
- Global hard cutoff (legacy 500 N behavior): if the compensated total force
  or torque magnitude exceeds a configurable ceiling (default 500 N), output
  zero immediately, set the `DEGRADED` flag, and publish a diagnostic.
- Mode-based zero output.
- Saturation diagnostics.

### 12.2 Hardware Interface Layer

`kuka_rsi_hw_interface` must enforce:

- RSI packet timeout detection.
- `IPOC` continuity checks.
- Command watchdog.
- Secondary command limiting.
- Zero command on socket failure.
- Fault publication to diagnostics.

### 12.3 KUKA Layer

KUKA KRL/RSI must enforce:

- No RSI unless EKI/ROS ready.
- Stop on watchdog error.
- Stop or zero correction on invalid `RKorr`.
- Native KUKA safety, limit, drive, and emergency-stop behavior.

## 13. Web Interface

Pages:

- Dashboard
  - EKI state
  - RSI state
  - SRI state
  - current mode and profile
  - active faults

- Servo Control
  - start/stop
  - mode and profile selection
  - correction values
  - limit status

- Force Monitor
  - raw wrench
  - filtered wrench
  - compensated wrench
  - force/torque trend curves

- Calibration
  - calibration action start/stop
  - pose progress
  - result preview including fit-quality index
  - save parameters

- Parameters
  - RSI network settings
  - force-control gains and profiles
  - filter settings
  - safety limits
  - payload parameters
  - tool parameters (EKI-read values shown read-only; YAML fallback marked)

- Diagnostics
  - update loop time
  - RSI packet loss
  - IPOC delay
  - wrench timestamp
  - saturation count

Dangerous operations must be routed through manager services/actions with state
checks.

## 14. Parameters

Parameter files:

```text
config/
  kuka_rsi.yaml
  controllers.yaml
  force_control.yaml
  payload.yaml
  calibration.yaml
  sri_ft.yaml
```

`kuka_rsi.yaml`:

- ROS IP.
- KUKA IP.
- RSI UDP port.
- EKI TCP port.
- RSI cycle.
- `IPOC` timeout.
- maximum `RKorr`.

`controllers.yaml`:

- controller names.
- command timeout.
- smoothing settings.
- rate limits.
- goal-mode convergence tolerance and timeout.

`force_control.yaml`:

- filter cutoff.
- profiles (`drag`, `precision`), each with:
  - per-axis-group gains (translation/rotation).
  - deadband (fixed value, or adaptive ramp settings: window, margin).
  - speed scale and per-axis max speed.
  - response mode.
- global hard cutoff (force/torque ceilings).
- auto re-tare enable + orientation tolerance.

`payload.yaml`:

- payload mass or equivalent gravity parameter.
- center-of-mass parameters.
- zero bias.
- fit-quality index.
- calibration timestamp.

`calibration.yaml`:

- calibration pose sequence (default: legacy 8-orientation set).
- settle time.
- samples per pose.
- return pose.

`sri_ft.yaml`:

- sensor IP and port.
- bias threshold (legacy `FTBia`).
- publish rate and filter defaults.

Legacy `Parameter.xml` migration map:

| Legacy parameter | New location |
|---|---|
| `RobIP` | `kuka_rsi.yaml: kuka_ip` |
| `FTIP` | `sri_ft.yaml: sensor_ip` |
| `MSPER/LSPER/XSPER` | `force_control.yaml: profiles.*.speed_scale` |
| `MRC/MRX/MRP`, `LRC/LRX/LRP`, `XRC/XRX/XRP` | `force_control.yaml: profiles.*.gains` (translation/rotation groups) |
| `XZSPD/MZSPD` | `force_control.yaml: profiles.*.max_speed` |
| `FTBia` | `sri_ft.yaml: bias_threshold` |
| `MRMode/LRMode/XRMode` | `force_control.yaml: profiles.*.response_mode` |
| `MousePID/MouseVID/Width/GuiZhi*` | Not migrated |

Runtime parameter updates:

- Manager validates updates.
- Manager creates an immutable snapshot.
- Snapshot is passed to the controller through a realtime-safe buffer.
- The controller never reads YAML or the ROS parameter server in `update()`.

## 15. Testing Strategy

### 15.1 Unit Tests

Use `gtest` for:

- XML parser/serializer.
- `ForceTorqueFilter`.
- `ToolGravityCompensator`.
- `PayloadEstimator` (including fit-quality output against synthetic data).
- `ComplianceLaw` (deadzone, gains, saturation, rate limits, profiles).
- `SafetyLimiter` (including global hard cutoff).
- `OrientationMotionCore` (convergence, speed shaping, timeout).
- Adaptive deadband ramp logic.
- Auto re-tare condition logic.

### 15.2 Communication Simulation

Create `kuka_rsi_sim_server`:

- Sends mock RSI state packets.
- Validates returned `RKorr`.
- Injects delay, packet loss, malformed XML, and `IPOC` jumps.

Create `eki_mock_server`:

- Simulates ready/start/stop/fault/reset.
- Answers tool queries with configurable `$TOOL` data.
- Tests reconnect behavior.
- Tests malformed XML and missing fields.

### 15.3 ROS Integration Tests

Launch:

- RSI mock server.
- EKI mock server.
- `kuka_rsi_hw_interface`.
- `controller_manager`.
- `SoftComplianceController`.
- SRI wrench mock.
- `soft_force_control_manager`.

Verify:

- Controller load/start/stop and controller switching on mode change.
- Wrench input generates bounded `RKorr`.
- `IDLE`, `FAULT`, and `CALIBRATING` force zero compliance output.
- Wrench timeout zeros output.
- RSI timeout enters fault.
- Adaptive deadband ramp holds zero output during the ramp window.
- Goal-mode motion converges on the RSI mock and reports completion.
- Full calibration action completes against mocks and writes `payload.yaml`.
- Tool query result propagates into the parameter snapshot.
- Parameter snapshot updates do not block realtime control.

### 15.4 Commissioning Tests

Real robot commissioning stages:

1. EKI connection, state handshake, and tool query.
2. RSI zero-output loop.
3. Small `DIRECT_CARTESIAN` correction (stream mode).
4. Goal-mode orientation move at low speed.
5. SRI wrench display only.
6. Low-gain force compliance (PRECISION profile).
7. DRAG profile with adaptive deadband; verify the `FLim/TLim` deadband
   interpretation (Section 7.2 note).
8. Payload calibration.
9. Full workflow through the web interface.

## 16. Acceptance Criteria

The first complete implementation is accepted when:

- The catkin workspace builds on ROS1 Noetic.
- Core ROS/backend modules are C++.
- `SoftComplianceController` loads as a `ros_control` plugin.
- `KukaRsiRobotHW` runs against the RSI mock server.
- RSI command output is `RKorr X/Y/Z/A/B/C`.
- All dangerous states default to zero correction.
- SRI wrench mock can drive bounded compliance output.
- Wrench timeout zeros output within the configured threshold.
- The global hard cutoff zeros output and raises `DEGRADED`.
- The adaptive deadband ramp and auto re-tare behave per Sections 7.4-7.5
  in integration tests.
- Goal-mode motion and the full calibration action complete against mocks.
- EKI mock can complete start/stop/fault/reset/tool-query workflows.
- Web UI can show status, start/stop, switch modes/profiles, view wrench data,
  view correction values, and run calibration.
- KUKA KRL/RSI/EKI templates and setup documentation are delivered.
- The implementation does not depend on the old MFC project's DLLs.

## 17. Documentation Deliverables

Required documentation:

```text
docs/
  architecture.md
  kuka_iiqka_rsi_eki_setup.md
  rsi_protocol.md
  eki_protocol.md
  safety_state_machine.md
  calibration_workflow.md
  web_interface.md
  test_plan.md
```

## 18. Future Work (explicitly out of scope for v1)

- Grease-application process mode (legacy Mode 8: `GuiZhiWidth/GuiZhiSpeed`,
  `Width` handshake).
- `AKorr` joint-space correction.
- `TOOL`-frame correction coordinate system experiments.
- Six-D mouse or other manual input devices.

## 19. Reference Links

- Local manual: `ref/iiQKA_RobotSensorInterface_62_zh.pdf`
- Local manual: `ref/iiQKA_EthernetKRL_61_zh.pdf`
- Legacy source: `ref/SoftRobot 2.0/`
- Brainstorm draft: `ref/2026-07-02-kuka-rsi-ros-control-design.md`
- KUKA RobotSensorInterface:
  https://www.kuka.com/en-us/products/robotics-systems/software/hub-technologies/kuka_robotsensorinterface
