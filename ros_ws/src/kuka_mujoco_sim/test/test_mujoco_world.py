# test/test_mujoco_world.py
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")  # skip cleanly if MuJoCo absent
import os
from kuka_mujoco_sim.mujoco_world import MujocoWorld

MODEL = os.path.join(os.path.dirname(__file__), '..', 'models', 'kuka_tcp_scene.xml')


def make_world(**kw):
    return MujocoWorld(os.path.abspath(MODEL), **kw)


def test_free_space_tracks_target():
    w = make_world(wall_enabled=False)
    start = w.home_pose6
    target = (start[0], start[1], start[2] - 20.0, start[3], start[4], start[5])
    w.set_target_pose(target)
    w.step(400)  # settle
    actual = w.get_actual_pose()
    # end-body follows target closely in free space (< 1 mm error)
    assert abs(actual[2] - target[2]) < 1.0


def test_free_space_gravity_only_wrench_matches_payload():
    # No contact: sensor reads payload gravity projected into sensor frame.
    w = make_world(wall_enabled=False, payload_mass=2.0)
    w.set_target_pose(w.home_pose6)
    w.step(400)
    fx, fy, fz, mx, my, mz = w.get_sensor_wrench()
    fmag = (fx**2 + fy**2 + fz**2) ** 0.5
    # 2 kg * 9.81 ~ 19.6 N gravity load magnitude (sign/frame per mount)
    assert 15.0 < fmag < 25.0


def test_orientation_changes_gravity_projection():
    # Rotating the end-body changes how gravity projects into sensor axes.
    w = make_world(wall_enabled=False, payload_mass=2.0)
    s = w.home_pose6
    w.set_target_pose(s)
    w.step(400)
    f_upright = np.array(w.get_sensor_wrench()[:3])
    w.set_target_pose((s[0], s[1], s[2], s[3], s[4] + 90.0, s[5]))
    w.step(600)
    f_tilted = np.array(w.get_sensor_wrench()[:3])
    # the per-axis distribution must differ after a 90 deg tilt
    assert np.linalg.norm(f_upright - f_tilted) > 3.0


def test_contact_produces_reaction_force():
    w = make_world(wall_enabled=True, payload_mass=0.0)
    s = w.home_pose6
    # push target well past the wall so contact builds up
    w.set_target_pose((s[0], s[1], s[2], s[3], s[4], s[5]))
    w.step(200)
    f0 = np.linalg.norm(w.get_sensor_wrench()[:3])
    # drive target into the wall (wall placed along +something in the MJCF)
    w.push_into_wall(depth_mm=10.0)
    w.step(400)
    f1 = np.linalg.norm(w.get_sensor_wrench()[:3])
    assert f1 > f0 + 1.0  # contact force appeared


def test_external_force_shows_on_sensor_same_axis():
    # A programmatic hand-drag (external +X force on the payload) must show up
    # on the FT sensor's X axis so the force-compliance loop can react to it.
    # Identity mount keeps gravity on -Z, isolating the drag on X.
    w = make_world(wall_enabled=False, payload_mass=2.0)
    w.set_target_pose(w.home_pose6)
    w.step(200)
    fx0 = w.get_sensor_wrench()[0]
    w.apply_external_force(40.0, 0.0, 0.0)
    w.step(200)
    fx1 = w.get_sensor_wrench()[0]
    # The +40 N drag dominates the X reading with the SAME sign (gravity is on
    # -Z under identity mount, leaving X ~0 before the drag). Same-sign matters:
    # after gravity compensation the controller moves WITH the +X force, so the
    # sensor must report +X, not -X.
    assert (fx1 - fx0) > 30.0, f"drag not seen on sensor +X: {fx0}->{fx1}"


def test_clear_external_force_removes_drag():
    w = make_world(wall_enabled=False, payload_mass=2.0)
    w.set_target_pose(w.home_pose6)
    w.apply_external_force(40.0, 0.0, 0.0)
    w.step(200)
    fx_dragged = w.get_sensor_wrench()[0]
    w.clear_external_force()
    w.step(200)
    fx_released = w.get_sensor_wrench()[0]
    assert abs(fx_dragged) > 30.0
    assert abs(fx_released) < 5.0, f"drag not cleared: {fx_released}"
