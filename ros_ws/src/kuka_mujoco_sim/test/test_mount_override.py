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
  (c) a payload_com_m override is accepted and integrates without error.

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


def test_payload_com_override_accepted():
    # The payload_com_m path edits body_ipos on the compiled model. In a static
    # free-space hang it need not change the (site-frame) reading, but it must
    # be accepted and integrate stably while still reading the gravity load.
    w = make_world(wall_enabled=False, payload_mass=2.0,
                   payload_com_m=(0.03, 0.0, 0.0))
    bid = w.model.body('payload').id
    assert np.allclose(w.model.body_ipos[bid], (0.03, 0.0, 0.0)), \
        'payload_com_m should be written to body_ipos'
    w.set_target_pose(w.home_pose6)
    w.step(400)
    fmag = np.linalg.norm(w.get_sensor_wrench()[:3])
    assert 15.0 < fmag < 25.0, 'gravity load still read after com shift, got {:.2f}N'.format(fmag)
