# kuka_mujoco_sim

A **MuJoCo physics validation environment** for the KUKA force-compliance stack.
It stands up a rigid-body world with a floating tool flange, a payload with a
force/torque sensor, and a contact wall, then speaks the **real wire protocols**
(RSI over UDP, SRI over TCP) so the *unmodified* production controllers can run
against physics instead of the two naive numeric mocks.

## Zero coupling with the existing system

This package adds **only** a new directory (`ros_ws/src/kuka_mujoco_sim/`). It
does **not** touch any existing package, header, launch, or config. It is a
Python package (`catkin` metadata only — no C++, no gtest) and MuJoCo is a
developer-machine dependency, **not** a rosdep/`package.xml` build dependency.

At runtime `mujoco_sim.launch` reuses every real node unchanged — the RSI
`hw_interface`, the EKI bridge, the SRI driver, and the force-control manager —
and swaps *only* the two mocks (`kuka_rsi_sim_server` + `sri_mock_server`) for
one MuJoCo node that impersonates both wire endpoints. Deleting this directory
returns the tree to its exact prior state.

## Dependencies

```bash
pip install mujoco numpy      # developer machine only; not in rosdep/catkin
```

Developed and validated against **MuJoCo 3.2.3 / NumPy 1.17+**. If MuJoCo is
absent, the MuJoCo-dependent tests `pytest.importorskip` cleanly (offline unit
tests still run).

## Architecture

```
        ┌──────────────────────────── mujoco_sim.launch ────────────────────────────┐
        │                                                                            │
  real RSI hw_interface ──UDP──▶ ┌───────────────────────┐ ◀──TCP── real SRI driver │
  (unchanged)          RKorr/pose│   kuka_mujoco_sim node │  wrench stream           │
                                 │  ┌─────────────────┐   │                          │
   real EKI bridge + mock        │  │  MujocoWorld    │   │   real force-control     │
   real force-control manager    │  │  (physics)      │   │   manager (unchanged)    │
   (all unchanged)               │  └─────────────────┘   │                          │
                                 └───────────────────────┘                           │
        └────────────────────────────────────────────────────────────────────────────┘

  RsiEndpoint  : UDP *client* impersonating the RSI KRC — IPOC handshake,
                 consumes RKorr corrections, integrates them onto the pose,
                 streams back the flange pose as a <Rob> frame.
  SriEndpoint  : TCP *server* impersonating the SRI FT sensor — streams the
                 MuJoCo sensor wrench as SRI frames after the START handshake.
  MujocoWorld  : mocap 'target' driven to the corrected TCP pose; a free-joint
                 'flange' tracks it elastically through a weld equality; a
                 payload body carries the FT-sensor site; a toggleable wall
                 provides contact reaction.
```

The MuJoCo node replaces the two numeric mocks from `sim.launch`; everything
else in the loop is the production stack.

## The three validation layers

Validation is deliberately split into three layers with **different scopes** —
read this before trusting any single command.

| Layer | Command | Needs MuJoCo | Scope / what it proves |
|-------|---------|:---:|------------------------|
| **1. Unit (offline)** | `pytest test/` | no | Pure logic: RSI/SRI frame encode-decode, `frame_conventions` coordinate transforms, endpoint handshake/integration logic, the scenario report framework. Deterministic, fast, no physics. |
| **2. Physics scenarios** | `scripts/run_scenarios.py` | yes | **World-level physical observables**: gravity projection changes with orientation, contact reaction appears, mount offset redistributes the wrench, uncompensated gravity is observable. Asserts on force / displacement / direction of the *physics world only*. |
| **3. End-to-end closed loop** | `roslaunch kuka_mujoco_sim mujoco_sim.launch` | yes + ROS | The **real controller in the loop**: RSI does not fault, the force-control manager consumes the wrench and produces corrections that move the flange. Observed live / via GUI. This is the only layer that exercises the full force→controller→motion chain. |

### Honest note on what layer 2 does *not* prove

`run_scenarios.py` (layer 2) asserts **world-level observables** — the force a
sensor would read, the displacement under contact, the direction of a reaction.
It deliberately keeps the controller *out of the loop*. Scenarios such as
`tool_frame_directionality` and the isotropy regression check the *physics* of
the wrench, **not** the full `sensor-wrench → controller-correction → base-motion`
transform chain. That complete chain — a real force producing a correctly
oriented robot motion — is validated by **layer 3** (the Task 7 launch smoke,
real controller in-loop). Do not read a green `run_scenarios.py` as proof that
the entire transform chain is correct; it proves the world behaves physically
and the controller layer is validated separately in-loop.

## Running

```bash
# Layer 1 — offline unit tests (no MuJoCo needed)
cd ros_ws/src/kuka_mujoco_sim
PYTHONPATH=src python3 -m pytest test/ -v

# Layer 2 — physics scenarios (needs MuJoCo); exits non-zero on FAIL
python3 scripts/run_scenarios.py

# Layer 3 — end-to-end closed loop (needs MuJoCo + ROS)
roslaunch kuka_mujoco_sim mujoco_sim.launch
# with options:
roslaunch kuka_mujoco_sim mujoco_sim.launch gui:=true payload_mass:=2.0 mount_abc:="[0,90,0]"
```

Launch args: `gui` (open the MuJoCo viewer), `payload_mass` (kg, default 1.0),
`mount_abc` (sensor mount A/B/C in degrees, default `[0,0,0]`), `wall_enabled`
(contact wall on/off, default true).

## Assertion thresholds and their rationale

Thresholds live in `src/kuka_mujoco_sim/scenarios.py` (and mirrored in
`test/test_mount_override.py`). They are chosen to be physically meaningful with
a wide margin over solver/settling noise, so they stay regression-stable.

| Threshold | Value | Rationale |
|-----------|-------|-----------|
| `NET_FORCE_ZERO_N` | **0.5 N** | "Effectively zero" residual force. Below solver noise floor for a settled free-space hang; a real uncompensated gravity load (~19.6 N) sits far above it, so it cleanly separates "compensated" from "not". |
| `DRIFT_MM` | **5 mm** | Free-space tracking must not wander. The stiff weld holds sub-mm; 5 mm flags a real tracking failure, not settling jitter. |
| `CONTACT_FORCE_N` | **2 N** | A contact reaction must be unmistakably present, not sensor ripple. Wall pushes here build tens of N. |
| mount / orientation delta | **> 3 N** | A 90° tilt of a ~19.6 N (2 kg) payload load moves the *entire* load onto another axis (measured Δ ≈ 27.7 N). 3 N is a large-margin floor that still rejects a no-op mount edit (the T8 `site_sameframe` bug produced ≈ 0 N). |
| gravity magnitude band | **15–25 N** | 2 kg × 9.81 ≈ 19.6 N; the band absorbs frame projection and settling without admitting a wrong mass. |

## Simulation pass ≠ real-robot pass

A green run here means the **physics-world observables and the wire protocols**
behave correctly. It does **not** certify the real robot. In particular the
following require confirmation on hardware and are tracked in
[`docs/superpowers/plans/2026-07-04-tool-frame-compliance-followups.md`](../../../docs/superpowers/plans/2026-07-04-tool-frame-compliance-followups.md):

- **Stale tool angle risk** — `$TOOL` changed on the pendant during an EKI
  disconnect can silently latch an old value; verify the `FORCE_COMPLIANCE frame
  locked` A/B/C echo before every servo start.
- **Real sensor noise / calibration** — MuJoCo reports a clean wrench; real FT
  bias, drift, and the 8-pose calibration must be validated on hardware.
- **Behaviour differs from old builds** under non-identity tool poses (the
  corrected RKorr direction) — brief operators before the first drag test.
- **Published wrench frame** — the compensated wrench is in the *sensor* frame,
  not tool/BASE.

Soft-real-time simulation is not the controller's hard-real-time path.

## MJCF tunable points (`models/kuka_tcp_scene.xml`)

- **Weld stiffness** `equality/weld solref="0.008 1"` `solimp="0.99 0.999 0.001"`
  — governs elastic tracking. Stiffer keeps free-space error sub-mm under
  payload gravity; softer yields more compliance displacement on contact.
- **Wall position** `body name="wall" pos="0.075 0 0.75"` — sits just past the
  tip's home so a small `push_into_wall` makes contact; move it to change the
  contact standoff. Toggled off via `wall_enabled` (contype/conaffinity → 0).
- **Payload mass / COM** — `body name="payload"` default `mass="1.0"`; overridden
  at load time by `payload_mass` (→ `body_mass`) and `payload_com_m`
  (→ `body_ipos`). Launch default is 1.0 kg; scenarios use 2.0 kg.
- **Sensor mount pose** — `site name="ft_site"` default identity; `mount_abc_deg`
  edits `site_quat` at load time (and clears `site_sameframe` so the edit is not
  a no-op — see `mujoco_world._load_spec`).
- **Timestep / gravity / integrator** — `option timestep="0.004"
  gravity="0 0 -9.81" integrator="implicitfast"`.

## Layout

```
models/kuka_tcp_scene.xml     MJCF world (flange + payload + FT site + wall)
src/kuka_mujoco_sim/
  frame_conventions.py        mm/deg ↔ m/quat, ABC↔quat helpers
  rsi_codec.py / sri_codec.py wire frame encode/decode
  rsi_endpoint.py             UDP client impersonating the RSI KRC
  sri_endpoint.py             TCP server impersonating the SRI FT sensor
  mujoco_world.py             MuJoCo wrapper (tracking + contact + FT readout)
  scenarios.py                world-level physics scenarios + report
  sim_node.py                 ROS node wiring endpoints ↔ MujocoWorld
scripts/run_scenarios.py      batch-run scenarios (layer 2)
scripts/sim_node              ROS entry point
launch/mujoco_sim.launch      end-to-end closed loop (layer 3)
test/                         offline unit + MuJoCo world/mount regressions
```
