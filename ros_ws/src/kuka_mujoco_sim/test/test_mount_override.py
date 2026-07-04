# test/test_mount_override.py
"""Regression for the runtime override paths in MujocoWorld._load_spec.

T4 shipped mount_abc_deg / payload_com_m overrides without dedicated
regression coverage; T8 then proved mount_abc_deg works end-to-end and fixed
the site_sameframe no-op bug (see mujoco_world._load_spec). These tests lock
that behaviour in at the world level:

  (a) a B=90deg mount tilts the sensor frame off the gravity axis, so the
      free-space gravity wrench is redistributed away from -Z (delta > 3N,
      matching T8's finding);
  (b) an identity mount keeps the pure-gravity load on -Z (sanity);
  (c) a payload_com_m override actually moves the COM: an off-origin COM puts
      the gravity load on a lever arm, producing a sensor torque that is ~0
      when the COM is centered (guards the body_sameframe no-op trap).

Thresholds are physical: 2 kg payload -> ~19.6 N gravity load. A 90deg tilt
moves the full load onto another axis (delta ~= 27.7 N here); 3N leaves a wide
margin over solver/settling noise.
"""
import os

import numpy as np
import pytest

pytest.importorskip("mujoco")  # skip cleanly if MuJoCo absent
from kuka_mujoco_sim.mujoco_world import MujocoWorld

MODEL = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'models', 'kuka_tcp_scene.xml'))


def make_world(**kw):
    return MujocoWorld(MODEL, **kw)


def _settled_wrench(**kw):
    w = make_world(wall_enabled=False, payload_mass=2.0, **kw)
    w.set_target_pose(w.home_pose6)
    w.step(400)
    return np.array(w.get_sensor_wrench()[:3])


def test_mount_b90_redistributes_gravity_off_z():
    # Identity mount: gravity load sits on -Z. A B=90 (Ry) mount rotates the
    # sensor frame so the same load projects onto another axis -> large delta.
    f_id = _settled_wrench()
    f_rot = _settled_wrench(mount_abc_deg=(0.0, 90.0, 0.0))
    delta = np.linalg.norm(f_id - f_rot)
    assert delta > 3.0, 'mount B90 should redistribute the wrench, delta={:.2f}N'.format(delta)
    # and the rotated reading must actually leave the vertical axis: the
    # off-Z (horizontal) magnitude becomes the dominant component.
    horiz = np.hypot(f_rot[0], f_rot[1])
    assert horiz > abs(f_rot[2]) + 3.0, \
        'rotated wrench should be dominated by an off-Z axis, got {}'.format(np.round(f_rot, 2))


def test_identity_mount_keeps_wrench_on_minus_z():
    # Sanity: with no mount rotation the 2 kg payload weight is a pure -Z load
    # (~19.6 N), and Z dominates the horizontal axes.
    f_id = _settled_wrench(mount_abc_deg=(0.0, 0.0, 0.0))
    horiz = np.hypot(f_id[0], f_id[1])
    assert f_id[2] < 0.0, 'gravity load should point along -Z, got {}'.format(np.round(f_id, 2))
    assert abs(f_id[2]) > horiz + 3.0, 'Z should dominate, got {}'.format(np.round(f_id, 2))
    assert 15.0 < abs(f_id[2]) < 25.0, 'magnitude should match ~2kg*g, got {:.2f}N'.format(f_id[2])


def test_payload_com_override_shifts_gravity_torque():
    # The payload_com_m path edits body_ipos on the compiled model and clears
    # body_sameframe so the edit is not silently ignored (same trap class as
    # site_sameframe). Physical guard, NOT a tautology: a COM offset from the
    # sensor origin puts the gravity load on a lever arm, so a static hang
    # develops a gravity torque about the sensor that is ~0 when the COM is
    # centered. If body_sameframe were left at 1 the edit would no-op and both
    # readings would be identical (~0), failing the delta assertion.
    def settled_torque(**kw):
        w = make_world(wall_enabled=False, payload_mass=2.0, **kw)
        w.set_target_pose(w.home_pose6)
        w.step(400)
        return np.array(w.get_sensor_wrench()[3:])  # (mx, my, mz) Nm

    t_centered = settled_torque()                        # COM at body origin
    t_offset = settled_torque(payload_com_m=(0.03, 0.0, 0.0))
    # centered COM -> negligible gravity torque
    assert np.linalg.norm(t_centered) < 0.1, \
        'centered COM should read ~0 torque, got {}'.format(np.round(t_centered, 3))
    # 0.03 m lever * ~19.6 N ~= 0.59 Nm must appear -> proves the edit took effect
    assert np.linalg.norm(t_offset - t_centered) > 0.3, \
        'COM offset must produce a gravity torque, delta={:.3f}Nm'.format(
            np.linalg.norm(t_offset - t_centered))
    # force magnitude unchanged by a pure COM shift (still ~2kg*g)
    w = make_world(wall_enabled=False, payload_mass=2.0,
                   payload_com_m=(0.03, 0.0, 0.0))
    bid = w.model.body('payload').id
    assert np.allclose(w.model.body_ipos[bid], (0.03, 0.0, 0.0)), \
        'payload_com_m should be written to body_ipos'
