# src/kuka_mujoco_sim/frame_conventions.py
"""KUKA A/B/C (R = Rz(A)·Ry(B)·Rx(C), degrees) <-> rotation matrix / quaternion.

Independent reimplementation of the C++ rotation.h convention, kept here so
this package has zero dependency on the existing C++ packages.
"""
import numpy as np

MM_PER_M = 1000.0


def _rz(r):
    c, s = np.cos(r), np.sin(r)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _ry(r):
    c, s = np.cos(r), np.sin(r)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rx(r):
    c, s = np.cos(r), np.sin(r)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def abc_deg_to_matrix(a, b, c):
    """R = Rz(A)·Ry(B)·Rx(C), inputs in degrees."""
    return _rz(np.deg2rad(a)) @ _ry(np.deg2rad(b)) @ _rx(np.deg2rad(c))


def matrix_to_abc_deg(R):
    """Inverse of abc_deg_to_matrix. Returns (a, b, c) in degrees.

    Z-Y-X Euler extraction. B = asin(-R[2,0]); at |R[2,0]|~1 (gimbal lock)
    fall back to C=0.
    """
    R = np.asarray(R, dtype=float)
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    b = np.arcsin(sy)
    if np.cos(b) > 1e-9:
        a = np.arctan2(R[1, 0], R[0, 0])
        c = np.arctan2(R[2, 1], R[2, 2])
    else:
        a = np.arctan2(-R[0, 1], R[1, 1])
        c = 0.0
    return np.rad2deg(a), np.rad2deg(b), np.rad2deg(c)


def abc_deg_to_quat(a, b, c):
    """Return MuJoCo-order quaternion [w, x, y, z]."""
    return matrix_to_quat(abc_deg_to_matrix(a, b, c))


def matrix_to_quat(R):
    """Rotation matrix -> [w, x, y, z], normalized."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """[w, x, y, z] -> rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_abc_deg(q):
    return matrix_to_abc_deg(quat_to_matrix(q))
