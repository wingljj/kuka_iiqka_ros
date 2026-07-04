"""Physics-direction equivalence between the MuJoCo sensor and the controller.

Root fix (force-loop-fix branch): mujoco_sim.launch must inject the payload
gravity and mount angle into the CONTROLLER too, not only the MuJoCo node, so
the controller's gravity compensation cancels the standing payload load. For
that passthrough to be correct, the MuJoCo site-frame reading and the
controller's compensation term must be expressed in the SAME frame for the
SAME mount angle.

This test proves they are, so the launch may inject
``sensor_to_flange_abc = mount_abc`` VERBATIM (no transpose / sign flip).

Controller math (soft_force_control_core/force_compliance_core.cpp):
    r_bt          = kukaAbcToRotation(state.a, b, c)   # TCP->BASE, live pose
    r_base_sensor = r_bt * frame.tcpFromSensor()
    tcpFromSensor = r_ftool^T * r_mount                # no EKI tool -> r_mount
    f_g           = r_base_sensor^T * (0, 0, -gravity_n)
    compensated   = raw - bias - f_g

At the home pose r_bt = I and (no tool) r_ftool = I, so
    r_base_sensor = r_mount = abc_deg_to_matrix(mount)
    f_g           = r_mount^T * (0, 0, -gravity_n)

If raw (MuJoCo, site frame) == f_g, then compensated ~= 0 and the payload
gravity stops feeding the adaptive deadband. We assert |raw - f_g| is tiny for
identity and several non-trivial mounts (incl. the [0,90,0] the user hit).
"""
import os

import numpy as np
import pytest

pytest.importorskip("mujoco")  # skip cleanly if MuJoCo absent
from kuka_mujoco_sim.mujoco_world import MujocoWorld
import kuka_mujoco_sim.frame_conventions as fc

MODEL = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'models', 'kuka_tcp_scene.xml'))

GRAVITY = 9.81  # matches models/kuka_tcp_scene.xml <option gravity="0 0 -9.81"/>


def _controller_expected_fg(mount_abc, gravity_n):
    """f_g the controller subtracts, at the home pose with no EKI tool."""
    r_mount = fc.abc_deg_to_matrix(*mount_abc)      # r_base_sensor at home
    return r_mount.T @ np.array([0.0, 0.0, -gravity_n])


def _mujoco_raw_gravity(mount_abc, payload_mass):
    """Settle the payload under gravity and read the sensor force (site frame)."""
    world = MujocoWorld(MODEL, mount_abc_deg=tuple(mount_abc),
                        payload_mass=payload_mass, wall_enabled=False)
    world.step(400)  # settle the elastic weld + payload
    return np.array(world.get_sensor_wrench()[:3])


@pytest.mark.parametrize("mount_abc", [
    [0, 0, 0],      # identity: full load on -Z
    [0, 90, 0],     # the case the user launched (payload swings load onto +X)
    [0, -90, 0],
    [90, 0, 0],
    [0, 45, 30],    # compound tilt: all three axes non-trivial
])
def test_sensor_matches_controller_compensation(mount_abc):
    payload_mass = 2.0
    gravity_n = payload_mass * GRAVITY

    raw = _mujoco_raw_gravity(mount_abc, payload_mass)
    f_g = _controller_expected_fg(mount_abc, gravity_n)
    net = raw - f_g

    # Verbatim passthrough is correct only if the residual is ~0 in EVERY
    # component. 0.5 N leaves a wide margin over solver/settling noise while
    # still failing hard if a transpose/sign were needed (those errors are
    # O(gravity_n) ~= 19.6 N, not < 0.5 N).
    assert np.linalg.norm(net) < 0.5, (
        f"mount={mount_abc}: raw={raw} vs controller f_g={f_g}, "
        f"net={net} (|net|={np.linalg.norm(net):.3f}N). If this fails, the "
        f"launch passthrough of sensor_to_flange_abc needs a transform.")


def test_tilt_actually_redistributes_load():
    """Guard the test above isn't trivially passing on a zero reading: a 90deg
    mount must move the load OFF -Z onto +X (else raw==f_g==(0,0,-g) would pass
    even if both were stuck at identity)."""
    raw = _mujoco_raw_gravity([0, 90, 0], 2.0)
    assert raw[0] > 15.0, f"expected load on +X under B=90 mount, got {raw}"
    assert abs(raw[2]) < 3.0, f"expected little load on Z under B=90, got {raw}"
