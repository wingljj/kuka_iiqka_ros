# test/test_frame_conventions.py
import numpy as np
import pytest
from kuka_mujoco_sim.frame_conventions import (
    abc_deg_to_matrix, matrix_to_abc_deg, abc_deg_to_quat, quat_to_abc_deg,
)

def test_identity():
    R = abc_deg_to_matrix(0, 0, 0)
    assert np.allclose(R, np.eye(3), atol=1e-12)

def test_rz90_convention():
    # Rz(90): x-axis -> y-axis. R @ [1,0,0] = [0,1,0].
    R = abc_deg_to_matrix(90, 0, 0)
    assert np.allclose(R @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-9)

def test_ry90_convention():
    # Ry(90): x-axis -> -z. R @ [1,0,0] = [0,0,-1].
    R = abc_deg_to_matrix(0, 90, 0)
    assert np.allclose(R @ np.array([1, 0, 0]), [0, 0, -1], atol=1e-9)

def test_rx90_convention():
    # Rx(90): y-axis -> z. R @ [0,1,0] = [0,0,1].
    R = abc_deg_to_matrix(0, 0, 90)
    assert np.allclose(R @ np.array([0, 1, 0]), [0, 0, 1], atol=1e-9)

def test_composed_order_zyx():
    # R = Rz·Ry·Rx must match explicit product.
    a, b, c = 30.0, 40.0, 50.0
    ra = np.deg2rad(a); rb = np.deg2rad(b); rc = np.deg2rad(c)
    Rz = np.array([[np.cos(ra), -np.sin(ra), 0], [np.sin(ra), np.cos(ra), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(rb), 0, np.sin(rb)], [0, 1, 0], [-np.sin(rb), 0, np.cos(rb)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(rc), -np.sin(rc)], [0, np.sin(rc), np.cos(rc)]])
    assert np.allclose(abc_deg_to_matrix(a, b, c), Rz @ Ry @ Rx, atol=1e-12)

def test_matrix_abc_roundtrip():
    for abc in [(10, 20, 30), (-45, 15, 80), (170, -10, 25)]:
        R = abc_deg_to_matrix(*abc)
        a2, b2, c2 = matrix_to_abc_deg(R)
        assert np.allclose(abc_deg_to_matrix(a2, b2, c2), R, atol=1e-9)

def test_quat_abc_roundtrip():
    for abc in [(0, 0, 0), (30, 40, 50), (-60, 20, 10)]:
        q = abc_deg_to_quat(*abc)
        assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-9)
        a2, b2, c2 = quat_to_abc_deg(q)
        R1 = abc_deg_to_matrix(*abc)
        R2 = abc_deg_to_matrix(a2, b2, c2)
        assert np.allclose(R1, R2, atol=1e-9)
