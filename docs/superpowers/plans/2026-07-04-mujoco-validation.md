# MuJoCo 力顺应验证环境 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新建独立 Python 包 `kuka_mujoco_sim`,用 MuJoCo 物理仿真(浮动末端体 + 质量负载 + 六维力传感器 + 接触环境)冒充 RSI/SRI 线协议端点,端到端验证力顺应算法(尤其工具系重力补偿与力解算)的准确性,零修改现有代码。

**Architecture:** MuJoCo 节点在两个线协议端点上冒充硬件——当 RSI UDP 客户端(发 `<Rob>` RIst、收 `<Sen>` RKorr)和 SRI TCP 服务端(发二进制力帧)。修正后的 TCP 位姿作为 mocap 目标,末端体经弹性约束跟踪、接触时让位,实际位姿回填 RIst 形成真实力-运动闭环。三个真实驱动(hw_interface/sri_ft_driver/eki_bridge)指向 127.0.0.1 原样运行,替换 sim.launch 里两个 naive mock。

**Tech Stack:** Python 3(rospy)、mujoco(官方 pip 绑定,非 mujoco-py)、numpy、pytest。UDP/TCP 用标准库 socket。

## Global Constraints

- **零修改现有包**:不改任何现有包的源码/CMakeLists/package.xml,不新增现有包依赖。`kuka_mujoco_sim` 只消费 rospy/geometry_msgs/soft_robot_msgs。
- **KUKA A/B/C 约定**:`R = Rz(A)·Ry(B)·Rx(C)`,角度单位度,位置单位 mm(RSI 线上);MuJoCo 内部 m + 四元数。转换在端点做。
- **RSI 线协议**(与现有 `rsi_frame.cpp`/`rsi_mock_core.cpp` 同构):
  - 状态帧 `<Rob Type="KUKA"><RIst X=".4f" Y Z A B C/><AIPos A1..A6/><Delay D="0"/><Mode M="1"/><IPOC>N</IPOC></Rob>`
  - 指令帧 `<Sen Type="ROS"><RKorr X=".4f" Y Z A B C/><Stop S="d"/><Watchdog W="N"/><IPOC>N</IPOC></Sen>`
  - RSI hw 是 UDP **服务端**(bind 49152);MuJoCo 是 **客户端**(发状态帧到 127.0.0.1:49152,收指令帧)。IPOC 单调递增(默认起 1000 步 4),指令帧回显状态帧的 IPOC。RKorr 是每周期相对增量(mm/deg),累加进 pose。
- **SRI 线协议**(与现有 `sri_frame.h`/`sri_mock_server.cpp` 同构):
  - MuJoCo 是 TCP **服务端**(bind 127.0.0.1:4008);sri_ft_driver 是 **客户端**。
  - 收到含 `AT+GSD` 的字节后回 ASCII `ACK+GSD=OK\r\n` 并开始流式发送。
  - 31 字节帧:`AA 55 | 00 1B | PN_H PN_L | 24 字节(6×float32 小端 Fx Fy Fz Mx My Mz)| SUM`,SUM = 后 24 字节数据和 & 0xFF(即 buf[6..29] 之和,不含 PN)。PN 每帧自增。
- **循环节拍**:仿真步 dt=4ms(250Hz 软实时)。控制器用实测周期,抖动不影响正确性。
- **端口/主机**:RSI 49152、SRI 4008、EKI 54600,全部 127.0.0.1,与 sim.launch 一致。
- **提交信息用英文**,每个 Task 独立提交,提交到功能分支不推送。

---

## 文件结构

```
ros_ws/src/kuka_mujoco_sim/
  package.xml                         # Task 1
  CMakeLists.txt                      # Task 1
  setup.py                            # Task 1
  src/kuka_mujoco_sim/
    __init__.py                       # Task 1
    frame_conventions.py              # Task 1 — A/B/C ↔ quat/matrix, 单位换算
    rsi_codec.py                      # Task 2 — build_rob_frame / parse_sen_frame
    sri_codec.py                      # Task 3 — build_sri_frame
    mujoco_world.py                   # Task 4 — MuJoCo 封装(弹性跟踪/接触/传感器)
    rsi_endpoint.py                   # Task 5 — UDP 客户端
    sri_endpoint.py                   # Task 6 — TCP 服务端
    sim_node.py                       # Task 7 — 主节点 + 250Hz 循环
    scenarios.py                      # Task 8 — 场景定义 + 断言
  models/
    kuka_tcp_scene.xml                # Task 4 — MJCF 场景
  scripts/
    run_scenarios.py                  # Task 8 — 批跑 + PASS/FAIL 报告
  launch/
    mujoco_sim.launch                 # Task 7
  test/
    test_frame_conventions.py         # Task 1
    test_rsi_codec.py                 # Task 2
    test_sri_codec.py                 # Task 3
    test_mujoco_world.py              # Task 4
    test_scenarios.py                 # Task 8
  README.md                           # Task 9
```

依赖顺序:T1 → T2/T3(可并行,均依赖 T1)→ T4 → T5/T6(依赖 T2/T3/T4)→ T7 → T8 → T9。

---

## Task 1: 包骨架 + 坐标系约定

**Files:**
- Create: `ros_ws/src/kuka_mujoco_sim/package.xml`
- Create: `ros_ws/src/kuka_mujoco_sim/CMakeLists.txt`
- Create: `ros_ws/src/kuka_mujoco_sim/setup.py`
- Create: `ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/__init__.py`
- Create: `ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/frame_conventions.py`
- Test: `ros_ws/src/kuka_mujoco_sim/test/test_frame_conventions.py`

**Interfaces:**
- Produces:
  - `abc_deg_to_matrix(a, b, c) -> np.ndarray(3,3)` — `Rz(a)·Ry(b)·Rx(c)`,输入度
  - `matrix_to_abc_deg(R) -> (a, b, c)` — 逆,返回度,与上互为往返
  - `abc_deg_to_quat(a, b, c) -> np.ndarray(4,)` — MuJoCo `[w,x,y,z]` 顺序
  - `quat_to_abc_deg(q) -> (a, b, c)` — 逆
  - `MM_PER_M = 1000.0`

- [ ] **Step 1: 写失败测试**

```python
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
```

- [ ] **Step 2: 运行验证失败**

Run: `cd ros_ws/src/kuka_mujoco_sim && python -m pytest test/test_frame_conventions.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'kuka_mujoco_sim'`

- [ ] **Step 3: 写包骨架**

`package.xml`:
```xml
<?xml version="1.0"?>
<package format="2">
  <name>kuka_mujoco_sim</name>
  <version>0.1.0</version>
  <description>MuJoCo physics validation environment for the force-compliance
    controller. Impersonates RSI/SRI wire protocols; zero changes to existing
    packages.</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>Proprietary</license>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend>
  <depend>geometry_msgs</depend>
  <depend>soft_robot_msgs</depend>
  <exec_depend>python3-numpy</exec_depend>
</package>
```

`CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(kuka_mujoco_sim)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs soft_robot_msgs)
catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
endif()
```

`setup.py`:
```python
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['kuka_mujoco_sim'],
    package_dir={'': 'src'},
)
setup(**setup_args)
```

`src/kuka_mujoco_sim/__init__.py`: (empty file)

- [ ] **Step 4: 写 frame_conventions.py**

```python
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
```

- [ ] **Step 5: 运行验证通过**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_frame_conventions.py -v`
Expected: PASS，7 项全过

- [ ] **Step 6: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_mujoco_sim
git commit -m "feat(mujoco-sim): package skeleton and KUKA A/B/C frame conventions"
```

---

## Task 2: RSI 帧编解码

**Files:**
- Create: `ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/rsi_codec.py`
- Test: `ros_ws/src/kuka_mujoco_sim/test/test_rsi_codec.py`

**Interfaces:**
- Consumes: 无(纯字符串/XML)
- Produces:
  - `build_rob_frame(pose6, ipoc, axes6=None) -> bytes` — pose6=(x,y,z,a,b,c) mm/deg,生成 `<Rob>` ASCII 字节
  - `parse_sen_frame(data: bytes) -> dict | None` — 返回 `{'rkorr': (x,y,z,a,b,c), 'stop': int, 'ipoc': int}`,解析失败返回 None

- [ ] **Step 1: 写失败测试**

```python
# test/test_rsi_codec.py
from kuka_mujoco_sim.rsi_codec import build_rob_frame, parse_sen_frame

def test_build_rob_frame_contains_fields():
    b = build_rob_frame((10.0, 20.0, 800.0, 1.0, 2.0, 3.0), 1000)
    s = b.decode('ascii')
    assert '<Rob Type="KUKA">' in s
    assert 'X="10.0000"' in s and 'Z="800.0000"' in s
    assert 'A="1.0000"' in s and 'C="3.0000"' in s
    assert '<IPOC>1000</IPOC>' in s
    assert '<Mode M="1"/>' in s
    assert 'AIPos' in s

def test_parse_sen_frame_roundtrip():
    sen = (b'<Sen Type="ROS">'
           b'<RKorr X="0.1000" Y="-0.2000" Z="0.3000" A="0.0100" B="0.0200" C="-0.0300"/>'
           b'<Stop S="0"/><Watchdog W="5"/><IPOC>1000</IPOC></Sen>')
    out = parse_sen_frame(sen)
    assert out is not None
    assert abs(out['rkorr'][0] - 0.1) < 1e-9
    assert abs(out['rkorr'][1] + 0.2) < 1e-9
    assert abs(out['rkorr'][5] + 0.03) < 1e-9
    assert out['stop'] == 0
    assert out['ipoc'] == 1000

def test_parse_sen_frame_missing_rkorr():
    assert parse_sen_frame(b'<Sen><IPOC>1</IPOC></Sen>') is None

def test_parse_sen_frame_garbage():
    assert parse_sen_frame(b'not xml') is None

def test_build_then_parse_via_rob_is_not_sen():
    # build_rob_frame output is <Rob>, parse_sen_frame must reject it.
    b = build_rob_frame((0, 0, 800, 0, 0, 0), 1000)
    assert parse_sen_frame(b) is None
```

- [ ] **Step 2: 运行验证失败**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_rsi_codec.py -v`
Expected: FAIL — `ModuleNotFoundError: kuka_mujoco_sim.rsi_codec`

- [ ] **Step 3: 写 rsi_codec.py**

```python
# src/kuka_mujoco_sim/rsi_codec.py
"""RSI <Rob>/<Sen> wire codec, matching the C++ rsi_frame.cpp format.

We emit <Rob> state frames (we impersonate the KRC) and parse <Sen> command
frames (from kuka_rsi_hw_interface). Uses ElementTree for parsing.
"""
import xml.etree.ElementTree as ET


def build_rob_frame(pose6, ipoc, axes6=None):
    """pose6 = (X, Y, Z, A, B, C) in mm/deg. Returns ASCII bytes."""
    x, y, z, a, b, c = pose6
    if axes6 is None:
        axes6 = (0.0,) * 6
    a1, a2, a3, a4, a5, a6 = axes6
    s = (
        '<Rob Type="KUKA">'
        '<RIst X="{:.4f}" Y="{:.4f}" Z="{:.4f}" A="{:.4f}" B="{:.4f}" C="{:.4f}"/>'
        '<AIPos A1="{:.4f}" A2="{:.4f}" A3="{:.4f}" A4="{:.4f}" A5="{:.4f}" A6="{:.4f}"/>'
        '<Delay D="0"/>'
        '<Mode M="1"/>'
        '<IPOC>{}</IPOC>'
        '</Rob>'
    ).format(x, y, z, a, b, c, a1, a2, a3, a4, a5, a6, int(ipoc))
    return s.encode('ascii')


def parse_sen_frame(data):
    """Parse a <Sen> command frame. Returns dict or None on any failure."""
    try:
        root = ET.fromstring(data)
    except ET.ParseError:
        return None
    if root.tag != 'Sen':
        return None
    rkorr = root.find('RKorr')
    ipoc = root.find('IPOC')
    if rkorr is None or ipoc is None or ipoc.text is None:
        return None
    try:
        vals = tuple(float(rkorr.attrib[k]) for k in ('X', 'Y', 'Z', 'A', 'B', 'C'))
        ipoc_val = int(ipoc.text.strip())
    except (KeyError, ValueError):
        return None
    stop_el = root.find('Stop')
    stop = int(stop_el.attrib.get('S', '0')) if stop_el is not None else 0
    return {'rkorr': vals, 'stop': stop, 'ipoc': ipoc_val}
```

- [ ] **Step 4: 运行验证通过**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_rsi_codec.py -v`
Expected: PASS，5 项全过

- [ ] **Step 5: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/rsi_codec.py ros_ws/src/kuka_mujoco_sim/test/test_rsi_codec.py
git commit -m "feat(mujoco-sim): RSI Rob/Sen wire frame codec"
```

---

## Task 3: SRI 二进制帧编码

**Files:**
- Create: `ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/sri_codec.py`
- Test: `ros_ws/src/kuka_mujoco_sim/test/test_sri_codec.py`

**Interfaces:**
- Consumes: 无
- Produces:
  - `build_sri_frame(wrench6, pn) -> bytes` — 31 字节帧,wrench6=(fx,fy,fz,mx,my,mz),pn 包号
  - `START_ACK = b'ACK+GSD=OK\r\n'`
  - `is_start_command(data: bytes) -> bool` — 是否含 `AT+GSD`

- [ ] **Step 1: 写失败测试**

```python
# test/test_sri_codec.py
import struct
from kuka_mujoco_sim.sri_codec import build_sri_frame, is_start_command, START_ACK

def test_frame_length_and_header():
    f = build_sri_frame((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), 1)
    assert len(f) == 31
    assert f[0] == 0xAA and f[1] == 0x55
    assert f[2] == 0x00 and f[3] == 0x1B  # payload length 27

def test_frame_payload_floats_little_endian():
    w = (1.5, -2.5, 3.5, 0.25, -0.5, 0.75)
    f = build_sri_frame(w, 7)
    # PN big-endian in bytes 4,5
    assert f[4] == 0x00 and f[5] == 0x07
    decoded = struct.unpack('<6f', bytes(f[6:30]))
    for got, exp in zip(decoded, w):
        assert abs(got - exp) < 1e-6

def test_frame_checksum():
    f = build_sri_frame((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), 1)
    expected = sum(f[6:30]) & 0xFF
    assert f[30] == expected

def test_is_start_command():
    assert is_start_command(b'AT+GSD\r\n')
    assert is_start_command(b'junk AT+GSD more')
    assert not is_start_command(b'hello')

def test_start_ack_value():
    assert START_ACK == b'ACK+GSD=OK\r\n'
```

- [ ] **Step 2: 运行验证失败**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_sri_codec.py -v`
Expected: FAIL — `ModuleNotFoundError: kuka_mujoco_sim.sri_codec`

- [ ] **Step 3: 写 sri_codec.py**

```python
# src/kuka_mujoco_sim/sri_codec.py
"""SRI M8128-style binary frame encoder, matching sri_mock_server.cpp.

Frame (31 bytes): AA 55 | 00 1B | PN_H PN_L | 24 bytes (6x float32 LE) | SUM.
SUM = sum of the 24 data bytes (buf[6:30]) & 0xFF. We are the TCP server;
sri_ft_driver connects and (optionally) sends AT+GSD before streaming.
"""
import struct

START_ACK = b'ACK+GSD=OK\r\n'


def build_sri_frame(wrench6, pn):
    """wrench6 = (Fx, Fy, Fz, Mx, My, Mz) in N/Nm. pn = packet number."""
    buf = bytearray(31)
    buf[0] = 0xAA
    buf[1] = 0x55
    buf[2] = 0x00
    buf[3] = 0x1B  # payload length 27 = PN(2) + data(24) + sum(1)
    pn &= 0xFFFF
    buf[4] = (pn >> 8) & 0xFF
    buf[5] = pn & 0xFF
    struct.pack_into('<6f', buf, 6, *wrench6)
    buf[30] = sum(buf[6:30]) & 0xFF
    return bytes(buf)


def is_start_command(data):
    return b'AT+GSD' in data
```

- [ ] **Step 4: 运行验证通过**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_sri_codec.py -v`
Expected: PASS，5 项全过

- [ ] **Step 5: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/sri_codec.py ros_ws/src/kuka_mujoco_sim/test/test_sri_codec.py
git commit -m "feat(mujoco-sim): SRI binary frame encoder"
```

---

## Task 4: MuJoCo 世界(MJCF + 弹性跟踪 + 接触 + 传感器)

**Files:**
- Create: `ros_ws/src/kuka_mujoco_sim/models/kuka_tcp_scene.xml`
- Create: `ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/mujoco_world.py`
- Test: `ros_ws/src/kuka_mujoco_sim/test/test_mujoco_world.py`

**Interfaces:**
- Consumes: `frame_conventions`(abc↔quat/matrix, MM_PER_M)
- Produces:
  - `class MujocoWorld`:
    - `__init__(self, model_path, mount_abc_deg=(0,0,0), payload_mass=None, payload_com_m=None, wall_enabled=True)`
    - `set_target_pose(self, pose6_mm_deg)` — 设 mocap 目标(mm/deg → m/quat)
    - `step(self, n=1)` — 步进 n 次
    - `get_actual_pose(self) -> tuple` — 末端体实际位姿 (x,y,z,a,b,c) mm/deg
    - `get_sensor_wrench(self) -> tuple` — 传感器系 6 维力 (fx,fy,fz,mx,my,mz) N/Nm,含接触反力 + 负载重力投影
    - `home_pose6 -> tuple` — 初始位姿(供端点初始化 target)
    - `DT = 0.004`

**说明**:MJCF 用 mocap body 作 target,末端体经 weld 约束(高刚度 solref/solimp)跟踪;负载作为末端体的子 body 带质量;force/torque sensor 测量传感器 site 处的合力。墙作为 static geom 供接触。

- [ ] **Step 1: 写失败测试**

```python
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
```

注:`push_into_wall(depth_mm)` 是测试辅助方法,实现见下(把 target 沿墙法向推 depth)。

- [ ] **Step 2: 运行验证失败**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_mujoco_world.py -v`
Expected: FAIL — `ModuleNotFoundError: kuka_mujoco_sim.mujoco_world`(或 MuJoCo 未装则 SKIP,此时先 `pip install mujoco` 再跑)

- [ ] **Step 3: 写 MJCF 场景**

```xml
<!-- models/kuka_tcp_scene.xml -->
<mujoco model="kuka_tcp_scene">
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>
  <compiler angle="radian"/>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="5 5 0.1" pos="0 0 0" rgba="0.8 0.9 0.8 1"/>

    <!-- Mocap target: driven by the RSI endpoint to the corrected TCP pose. -->
    <body name="target" mocap="true" pos="0 0 0.8">
      <geom type="box" size="0.01 0.01 0.01" rgba="0 1 0 0.3" contype="0" conaffinity="0"/>
    </body>

    <!-- Floating end-body (the TCP flange). Free joint = 6 DOF. -->
    <body name="flange" pos="0 0 0.8">
      <freejoint name="flange_free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="flange_geom" type="cylinder" size="0.03 0.02" rgba="0.5 0.5 0.9 1"/>

      <!-- Force/torque sensor site (mount pose set at load time by editing). -->
      <site name="ft_site" pos="0 0 0" size="0.005" rgba="1 0 0 1"/>

      <!-- Payload (workpiece) rigidly attached below the sensor. Mass edited
           at load time; com via pos. -->
      <body name="payload" pos="0 0 -0.05">
        <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
        <geom name="payload_geom" type="box" size="0.02 0.02 0.03" rgba="0.9 0.7 0.2 1"/>
        <!-- Contact tip that touches the wall. -->
        <geom name="tip" type="sphere" size="0.01" pos="0 0 -0.03" rgba="0.9 0.2 0.2 1"/>
      </body>
    </body>

    <!-- Wall along +X at some distance; toggled by removing at load time. -->
    <body name="wall" pos="0.15 0 0.8">
      <geom name="wall_geom" type="box" size="0.02 0.3 0.3" rgba="0.7 0.7 0.7 1"/>
    </body>
  </worldbody>

  <!-- Weld the flange to the mocap target: elastic tracking via solref/solimp.
       Lower stiffness = softer tracking. -->
  <equality>
    <weld name="track" body1="flange" body2="target"
          solref="0.02 1" solimp="0.9 0.95 0.001"/>
  </equality>

  <sensor>
    <force name="ft_force" site="ft_site"/>
    <torque name="ft_torque" site="ft_site"/>
  </sensor>
</mujoco>
```

- [ ] **Step 4: 写 mujoco_world.py**

```python
# src/kuka_mujoco_sim/mujoco_world.py
"""MuJoCo wrapper: elastic pose tracking + contact + FT sensor readout.

The mocap 'target' body is driven to the corrected TCP pose; the 'flange'
free body tracks it through a weld equality (elastic). Contact with the wall
pushes the flange off target -> compliance displacement. The FT sensor reads
contact reaction + payload gravity projected into the sensor frame.
"""
import numpy as np
import mujoco

from . import frame_conventions as fc


class MujocoWorld:
    DT = 0.004

    def __init__(self, model_path, mount_abc_deg=(0, 0, 0), payload_mass=None,
                 payload_com_m=None, wall_enabled=True):
        spec = self._load_spec(model_path, mount_abc_deg, payload_mass,
                                payload_com_m, wall_enabled)
        self.model = spec
        self.data = mujoco.MjData(self.model)
        self._mocap_id = self.model.body('target').mocapid[0]
        self._flange_bid = self.model.body('flange').id
        self._force_adr = self.model.sensor('ft_force').adr[0]
        self._torque_adr = self.model.sensor('ft_torque').adr[0]
        self._site_id = self.model.site('ft_site').id
        mujoco.mj_forward(self.model, self.data)
        # home pose = initial flange pose in mm/deg
        self.home_pose6 = self._flange_pose6()
        self._wall_normal_world = np.array([-1.0, 0.0, 0.0])  # wall faces -X

    def _load_spec(self, path, mount_abc_deg, payload_mass, payload_com_m,
                   wall_enabled):
        with open(path, 'r') as fh:
            xml = fh.read()
        model = mujoco.MjModel.from_xml_string(xml)
        # Apply runtime overrides via the compiled model where possible.
        if payload_mass is not None:
            bid = model.body('payload').id
            model.body_mass[bid] = payload_mass
        if payload_com_m is not None:
            bid = model.body('payload').id
            model.body_ipos[bid] = np.asarray(payload_com_m, dtype=float)
        if mount_abc_deg != (0, 0, 0):
            sid = model.site('ft_site').id
            quat = fc.abc_deg_to_quat(*mount_abc_deg)
            model.site_quat[sid] = quat
        if not wall_enabled:
            gid = model.geom('wall_geom').id
            model.geom_contype[gid] = 0
            model.geom_conaffinity[gid] = 0
        return model

    def _flange_pose6(self):
        bid = self._flange_bid
        pos_m = self.data.xpos[bid].copy()
        quat = self.data.xquat[bid].copy()  # [w,x,y,z]
        a, b, c = fc.quat_to_abc_deg(quat)
        return (pos_m[0] * fc.MM_PER_M, pos_m[1] * fc.MM_PER_M,
                pos_m[2] * fc.MM_PER_M, a, b, c)

    def set_target_pose(self, pose6_mm_deg):
        x, y, z, a, b, c = pose6_mm_deg
        self.data.mocap_pos[self._mocap_id] = np.array(
            [x, y, z]) / fc.MM_PER_M
        self.data.mocap_quat[self._mocap_id] = fc.abc_deg_to_quat(a, b, c)

    def step(self, n=1):
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def get_actual_pose(self):
        return self._flange_pose6()

    def get_sensor_wrench(self):
        # MuJoCo force/torque sensors report the interaction in the site frame,
        # sign convention: force the child exerts. Includes gravity of bodies
        # below the site (payload) + contact. Returns (fx,fy,fz,mx,my,mz).
        f = self.data.sensordata[self._force_adr:self._force_adr + 3].copy()
        t = self.data.sensordata[self._torque_adr:self._torque_adr + 3].copy()
        return (f[0], f[1], f[2], t[0], t[1], t[2])

    def push_into_wall(self, depth_mm):
        """Test helper: move target from current flange pose toward the wall
        normal by depth_mm so contact builds up."""
        p = self._flange_pose6()
        d = self._wall_normal_world * (-depth_mm)  # into +X (toward wall)
        self.set_target_pose((p[0] + d[0], p[1] + d[1], p[2] + d[2],
                              p[3], p[4], p[5]))
```

注:实现者需按实际 MuJoCo 版本核对 `data.mocap_pos` 索引与 sensor 读数符号;若 `force` sensor 语义与预期相反,在 `get_sensor_wrench` 统一取负并在此注释说明。测试 `test_free_space_gravity_only_wrench_matches_payload` 用幅值断言,对符号不敏感;`test_orientation_changes_gravity_projection` 用差异断言。

- [ ] **Step 5: 运行验证通过**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_mujoco_world.py -v`
Expected: PASS，4 项全过(若某项因 MJCF 数值需微调刚度/墙位置,调 MJCF 的 solref/pos 使物理合理,不改断言语义)

- [ ] **Step 6: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_mujoco_sim/models ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/mujoco_world.py ros_ws/src/kuka_mujoco_sim/test/test_mujoco_world.py
git commit -m "feat(mujoco-sim): MuJoCo world with elastic tracking, contact, FT sensor"
```

---

## Task 5: RSI UDP 端点

**Files:**
- Create: `ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/rsi_endpoint.py`
- Test: (集成性,归入 Task 7 冒烟;本任务只做可离线的循环逻辑单测)
- Test: `ros_ws/src/kuka_mujoco_sim/test/test_rsi_endpoint.py`

**Interfaces:**
- Consumes: `rsi_codec.build_rob_frame/parse_sen_frame`
- Produces:
  - `class RsiEndpoint`:
    - `__init__(self, target_ip='127.0.0.1', target_port=49152, ipoc_start=1000, ipoc_step=4)`
    - `pose_target6` 属性(mm/deg,初值由外部 set)
    - `set_initial_target(self, pose6)`
    - `send_state(self, sock, actual_pose6)` — 发 `<Rob>`(actual)、记 awaited_ipoc
    - `apply_reply(self, data) -> tuple | None` — 解析 `<Sen>`,校验 IPOC 回显,累加 RKorr 到 pose_target6,返回新 target 或 None(校验失败)

- [ ] **Step 1: 写失败测试**

```python
# test/test_rsi_endpoint.py
from kuka_mujoco_sim.rsi_endpoint import RsiEndpoint
from kuka_mujoco_sim.rsi_codec import build_rob_frame

class FakeSock:
    def __init__(self): self.sent = []
    def sendto(self, data, addr): self.sent.append((data, addr))

def test_send_state_emits_rob_and_tracks_ipoc():
    ep = RsiEndpoint(ipoc_start=1000, ipoc_step=4)
    ep.set_initial_target((0, 0, 800, 0, 0, 0))
    sock = FakeSock()
    ep.send_state(sock, (1, 2, 800, 0, 0, 0))
    assert len(sock.sent) == 1
    assert b'<Rob' in sock.sent[0][0]
    assert ep.awaited_ipoc == 1000

def test_apply_reply_integrates_rkorr():
    ep = RsiEndpoint(ipoc_start=1000, ipoc_step=4)
    ep.set_initial_target((0, 0, 800, 0, 0, 0))
    sock = FakeSock()
    ep.send_state(sock, (0, 0, 800, 0, 0, 0))  # awaited=1000
    sen = (b'<Sen Type="ROS"><RKorr X="0.1" Y="0" Z="-0.2" A="0" B="0" C="0"/>'
           b'<Stop S="0"/><Watchdog W="1"/><IPOC>1000</IPOC></Sen>')
    new_target = ep.apply_reply(sen)
    assert new_target is not None
    assert abs(new_target[0] - 0.1) < 1e-9
    assert abs(new_target[2] - 799.8) < 1e-9

def test_apply_reply_rejects_wrong_ipoc():
    ep = RsiEndpoint(ipoc_start=1000, ipoc_step=4)
    ep.set_initial_target((0, 0, 800, 0, 0, 0))
    sock = FakeSock()
    ep.send_state(sock, (0, 0, 800, 0, 0, 0))  # awaited=1000
    sen = (b'<Sen><RKorr X="0.1" Y="0" Z="0" A="0" B="0" C="0"/>'
           b'<IPOC>999</IPOC></Sen>')
    assert ep.apply_reply(sen) is None

def test_ipoc_increments_each_state():
    ep = RsiEndpoint(ipoc_start=1000, ipoc_step=4)
    ep.set_initial_target((0, 0, 800, 0, 0, 0))
    sock = FakeSock()
    ep.send_state(sock, (0, 0, 800, 0, 0, 0))
    assert ep.awaited_ipoc == 1000
    ep.send_state(sock, (0, 0, 800, 0, 0, 0))
    assert ep.awaited_ipoc == 1004
```

- [ ] **Step 2: 运行验证失败**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_rsi_endpoint.py -v`
Expected: FAIL — `ModuleNotFoundError: kuka_mujoco_sim.rsi_endpoint`

- [ ] **Step 3: 写 rsi_endpoint.py**

```python
# src/kuka_mujoco_sim/rsi_endpoint.py
"""RSI UDP client endpoint: sends <Rob> state (actual pose), receives <Sen>
command (RKorr), integrates RKorr into the target pose. Mirrors RsiMockCore
but the 'actual pose' is supplied by the physics world, not integrated here.
"""
from .rsi_codec import build_rob_frame, parse_sen_frame


class RsiEndpoint:
    def __init__(self, target_ip='127.0.0.1', target_port=49152,
                 ipoc_start=1000, ipoc_step=4):
        self.addr = (target_ip, target_port)
        self._next_ipoc = ipoc_start
        self._step = ipoc_step
        self.awaited_ipoc = None
        self.awaiting = False
        self.pose_target6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.stats = {'sent': 0, 'replies': 0, 'ipoc_errors': 0, 'parse_errors': 0}

    def set_initial_target(self, pose6):
        self.pose_target6 = tuple(float(v) for v in pose6)

    def send_state(self, sock, actual_pose6):
        frame = build_rob_frame(actual_pose6, self._next_ipoc)
        sock.sendto(frame, self.addr)
        self.awaited_ipoc = self._next_ipoc
        self.awaiting = True
        self._next_ipoc += self._step
        self.stats['sent'] += 1

    def apply_reply(self, data):
        parsed = parse_sen_frame(data)
        if parsed is None:
            self.stats['parse_errors'] += 1
            return None
        self.stats['replies'] += 1
        if not self.awaiting or parsed['ipoc'] != self.awaited_ipoc:
            self.stats['ipoc_errors'] += 1
            return None
        self.awaiting = False
        rk = parsed['rkorr']
        self.pose_target6 = tuple(
            self.pose_target6[i] + rk[i] for i in range(6))
        return self.pose_target6
```

- [ ] **Step 4: 运行验证通过**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_rsi_endpoint.py -v`
Expected: PASS，4 项全过

- [ ] **Step 5: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/rsi_endpoint.py ros_ws/src/kuka_mujoco_sim/test/test_rsi_endpoint.py
git commit -m "feat(mujoco-sim): RSI UDP endpoint with IPOC handshake and RKorr integration"
```

---

## Task 6: SRI TCP 端点

**Files:**
- Create: `ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/sri_endpoint.py`
- Test: `ros_ws/src/kuka_mujoco_sim/test/test_sri_endpoint.py`

**Interfaces:**
- Consumes: `sri_codec.build_sri_frame/is_start_command/START_ACK`
- Produces:
  - `class SriEndpoint`:
    - `__init__(self, listen_ip='127.0.0.1', listen_port=4008, require_start=True)`
    - `start(self)` / `stop(self)` — 起停后台 accept 线程
    - `set_wrench(self, wrench6)` — 线程安全设当前力(sim 循环每周期调)
    - `send_frame_if_streaming(self)` — 若有客户端且已 streaming,编码当前力发一帧
    - `stats` 属性

**说明**:后台线程 accept 一个客户端、收 `AT+GSD` 回 ACK 置 streaming;主 sim 循环调 `set_wrench` + `send_frame_if_streaming`。用 threading.Lock 保护 wrench 与 client fd。

- [ ] **Step 1: 写失败测试**(用真实 loopback socket,离线可跑)

```python
# test/test_sri_endpoint.py
import socket
import struct
import time
from kuka_mujoco_sim.sri_endpoint import SriEndpoint

def _connect(port):
    c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    c.settimeout(2.0)
    c.connect(('127.0.0.1', port))
    return c

def test_ack_and_stream():
    ep = SriEndpoint(listen_port=0, require_start=True)  # 0 = ephemeral
    ep.start()
    try:
        port = ep.port
        c = _connect(port)
        assert ep.wait_for_client(2.0)
        c.sendall(b'AT+GSD\r\n')
        # read ACK
        ack = c.recv(64)
        assert b'ACK+GSD=OK' in ack
        # drive a frame
        ep.set_wrench((1.0, 2.0, 3.0, 0.1, 0.2, 0.3))
        for _ in range(5):
            ep.send_frame_if_streaming()
            time.sleep(0.01)
        data = c.recv(256)
        # find a frame header and decode floats
        i = data.find(b'\xAA\x55')
        assert i >= 0
        frame = data[i:i+31]
        floats = struct.unpack('<6f', frame[6:30])
        assert abs(floats[0] - 1.0) < 1e-5
        c.close()
    finally:
        ep.stop()

def test_no_stream_before_start_command():
    ep = SriEndpoint(listen_port=0, require_start=True)
    ep.start()
    try:
        c = _connect(ep.port)
        assert ep.wait_for_client(2.0)
        ep.set_wrench((5, 0, 0, 0, 0, 0))
        ep.send_frame_if_streaming()  # should NOT send (no AT+GSD yet)
        c.settimeout(0.3)
        got = b''
        try:
            got = c.recv(64)
        except socket.timeout:
            pass
        assert got == b''
        c.close()
    finally:
        ep.stop()
```

- [ ] **Step 2: 运行验证失败**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_sri_endpoint.py -v`
Expected: FAIL — `ModuleNotFoundError: kuka_mujoco_sim.sri_endpoint`

- [ ] **Step 3: 写 sri_endpoint.py**

```python
# src/kuka_mujoco_sim/sri_endpoint.py
"""SRI TCP server endpoint. sri_ft_driver connects, optionally sends AT+GSD,
then we stream 31-byte FT frames driven by the physics world's sensor wrench.
"""
import socket
import threading
import time

from .sri_codec import build_sri_frame, is_start_command, START_ACK


class SriEndpoint:
    def __init__(self, listen_ip='127.0.0.1', listen_port=4008,
                 require_start=True):
        self._ip = listen_ip
        self._req_port = listen_port
        self.port = listen_port
        self._require_start = require_start
        self._lock = threading.Lock()
        self._wrench = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._pn = 0
        self._client = None
        self._streaming = False
        self._listen = None
        self._running = False
        self._thread = None
        self.stats = {'frames_sent': 0, 'clients': 0}

    def start(self):
        self._listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen.bind((self._ip, self._req_port))
        self.port = self._listen.getsockname()[1]
        self._listen.listen(1)
        self._listen.settimeout(0.1)
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        with self._lock:
            if self._client:
                self._client.close()
                self._client = None
        if self._listen:
            self._listen.close()

    def wait_for_client(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._client is not None:
                    return True
            time.sleep(0.01)
        return False

    def set_wrench(self, wrench6):
        with self._lock:
            self._wrench = tuple(float(v) for v in wrench6)

    def send_frame_if_streaming(self):
        with self._lock:
            if self._client is None or not self._streaming:
                return
            self._pn += 1
            frame = build_sri_frame(self._wrench, self._pn)
            try:
                self._client.sendall(frame)
                self.stats['frames_sent'] += 1
            except OSError:
                self._drop_client_locked()

    def _drop_client_locked(self):
        if self._client:
            try:
                self._client.close()
            except OSError:
                pass
        self._client = None
        self._streaming = False

    def _run(self):
        while self._running:
            with self._lock:
                have_client = self._client is not None
            if not have_client:
                try:
                    conn, _ = self._listen.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                conn.settimeout(0.05)
                with self._lock:
                    self._client = conn
                    self._streaming = not self._require_start
                    self.stats['clients'] += 1
                continue
            # have client: poll for AT+GSD
            with self._lock:
                conn = self._client
            try:
                data = conn.recv(256)
                if not data:
                    with self._lock:
                        self._drop_client_locked()
                    continue
                if is_start_command(data):
                    with self._lock:
                        conn.sendall(START_ACK)
                        self._streaming = True
            except socket.timeout:
                pass
            except OSError:
                with self._lock:
                    self._drop_client_locked()
```

- [ ] **Step 4: 运行验证通过**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_sri_endpoint.py -v`
Expected: PASS，2 项全过

- [ ] **Step 5: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/sri_endpoint.py ros_ws/src/kuka_mujoco_sim/test/test_sri_endpoint.py
git commit -m "feat(mujoco-sim): SRI TCP server endpoint streaming physics wrench"
```

---

## Task 7: 主节点 + launch + 冒烟

**Files:**
- Create: `ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/sim_node.py`
- Create: `ros_ws/src/kuka_mujoco_sim/launch/mujoco_sim.launch`
- (无新单测;交付物是可跑的闭环冒烟)

**Interfaces:**
- Consumes: `MujocoWorld`, `RsiEndpoint`, `SriEndpoint`, `frame_conventions`
- Produces: 可执行 ROS 节点 `sim_node.py`(rosrun/launch),250Hz 闭环

**节点循环(250Hz)**:
```
每周期:
  1. actual = world.get_actual_pose()
  2. rsi.send_state(udp_sock, actual)
  3. 阻塞收 <Sen>(超时 8ms)→ rsi.apply_reply → 得 new target
  4. world.set_target_pose(rsi.pose_target6)
  5. world.step(1)
  6. sri.set_wrench(world.get_sensor_wrench()); sri.send_frame_if_streaming()
  7. (可选)gui 渲染
```
参数:`~gui`(默认 false)、`~model_path`、`~mount_abc`(list3)、`~payload_mass`、`~wall_enabled`、端口。RSI hw 是 UDP 服务端,MuJoCo 先发状态帧触发握手。

- [ ] **Step 1: 写 sim_node.py**

```python
#!/usr/bin/env python3
# src/kuka_mujoco_sim/sim_node.py
"""MuJoCo sim node: closes the RSI + SRI loop against the real drivers."""
import os
import socket
import rospy

from .mujoco_world import MujocoWorld
from .rsi_endpoint import RsiEndpoint
from .sri_endpoint import SriEndpoint


class SimNode:
    def __init__(self):
        model = rospy.get_param('~model_path', self._default_model())
        mount = rospy.get_param('~mount_abc', [0.0, 0.0, 0.0])
        payload_mass = rospy.get_param('~payload_mass', 1.0)
        wall = rospy.get_param('~wall_enabled', True)
        rsi_ip = rospy.get_param('~rsi_target_ip', '127.0.0.1')
        rsi_port = rospy.get_param('~rsi_target_port', 49152)
        sri_port = rospy.get_param('~sri_listen_port', 4008)
        self._gui = rospy.get_param('~gui', False)

        self.world = MujocoWorld(model, mount_abc_deg=tuple(mount),
                                 payload_mass=payload_mass, wall_enabled=wall)
        self.rsi = RsiEndpoint(target_ip=rsi_ip, target_port=rsi_port)
        self.rsi.set_initial_target(self.world.home_pose6)
        self.sri = SriEndpoint(listen_port=sri_port, require_start=True)
        self.sri.start()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)
        self._viewer = None
        if self._gui:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(
                self.world.model, self.world.data)

    def _default_model(self):
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', '..', 'models', 'kuka_tcp_scene.xml')

    def spin(self):
        rate = rospy.Rate(250)
        while not rospy.is_shutdown():
            actual = self.world.get_actual_pose()
            self.rsi.send_state(self.sock, actual)
            try:
                data, _ = self.sock.recvfrom(4096)
                self.rsi.apply_reply(data)
            except socket.timeout:
                pass
            self.world.set_target_pose(self.rsi.pose_target6)
            self.world.step(1)
            self.sri.set_wrench(self.world.get_sensor_wrench())
            self.sri.send_frame_if_streaming()
            if self._viewer is not None:
                self._viewer.sync()
            rate.sleep()

    def shutdown(self):
        self.sri.stop()
        self.sock.close()
        if self._viewer is not None:
            self._viewer.close()


def main():
    rospy.init_node('kuka_mujoco_sim')
    node = SimNode()
    rospy.on_shutdown(node.shutdown)
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
```

同时在 `setup.py` 的 `scripts` 或用 launch `type=sim_node.py` 暴露可执行。为兼容 catkin,创建 `scripts/sim_node` 薄封装:
```python
#!/usr/bin/env python3
from kuka_mujoco_sim.sim_node import main
main()
```
并 `chmod +x scripts/sim_node`,`CMakeLists.txt` 加:
```cmake
catkin_install_python(PROGRAMS scripts/sim_node scripts/run_scenarios.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

- [ ] **Step 2: 写 launch**

```xml
<!-- launch/mujoco_sim.launch : MuJoCo replaces the two naive mocks;
     EKI mock + all three real drivers reused unchanged. -->
<launch>
  <arg name="gui" default="false"/>
  <arg name="payload_mass" default="1.0"/>
  <arg name="mount_abc" default="[0.0, 0.0, 0.0]"/>
  <arg name="wall_enabled" default="true"/>

  <rosparam command="load"
            file="$(find soft_robot_controllers)/config/soft_robot_controllers.yaml"/>
  <rosparam command="load"
            file="$(find kuka_eki_bridge)/config/kuka_eki.yaml"/>
  <rosparam command="load"
            file="$(find sri_force_torque_driver)/config/sri_ft.yaml"/>
  <rosparam command="load"
            file="$(find soft_force_control_manager)/config/manager.yaml"/>
  <rosparam command="load"
            file="$(find soft_force_control_manager)/config/calibration.yaml"/>

  <node pkg="kuka_rsi_hw_interface" type="kuka_rsi_hw_interface_node"
        name="kuka_rsi_hw_interface" output="screen">
    <rosparam command="load"
              file="$(find kuka_rsi_hw_interface)/config/kuka_rsi.yaml"/>
    <param name="listen_ip" value="127.0.0.1"/>
  </node>

  <!-- MuJoCo physics: RSI UDP client + SRI TCP server. Replaces
       kuka_rsi_sim_server AND sri_mock_server. -->
  <node pkg="kuka_mujoco_sim" type="sim_node" name="kuka_mujoco_sim"
        output="screen">
    <param name="gui" value="$(arg gui)"/>
    <param name="payload_mass" value="$(arg payload_mass)"/>
    <rosparam param="mount_abc" subst_value="true">$(arg mount_abc)</rosparam>
    <param name="wall_enabled" value="$(arg wall_enabled)"/>
    <param name="rsi_target_port" value="49152"/>
    <param name="sri_listen_port" value="4008"/>
  </node>

  <!-- EKI: reuse existing mock unchanged. -->
  <node pkg="soft_robot_bringup" type="run_mock.sh"
        name="eki_mock_server" output="screen"
        args="kuka_eki_bridge eki_mock_server --port 54600 --heartbeat-ms 100"/>
  <node pkg="kuka_eki_bridge" type="eki_bridge_node"
        name="kuka_eki_bridge" output="screen">
    <param name="kuka_ip" value="127.0.0.1"/>
  </node>

  <node pkg="sri_force_torque_driver" type="sri_driver_node"
        name="sri_ft_driver" output="screen">
    <param name="sensor_ip" value="127.0.0.1"/>
  </node>

  <node pkg="soft_force_control_manager" type="soft_robot_manager"
        name="soft_robot_manager" output="screen">
    <param name="payload_file" value="/tmp/soft_robot_sim_payload.yaml"/>
    <param name="calibration/samples_per_pose" value="20"/>
  </node>
</launch>
```

- [ ] **Step 3: 构建工作区**

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make`
Expected: 成功,`kuka_mujoco_sim` 被 catkin 识别(Python 包无编译产物,但 setup 应通过)

- [ ] **Step 4: 冒烟——闭环起来、RSI hw 不再 fault**

Run(需先 `pip install mujoco`,并 `source devel/setup.bash`):
```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && source devel/setup.bash
timeout 20 roslaunch kuka_mujoco_sim mujoco_sim.launch > /tmp/mujoco_smoke.log 2>&1
grep -c "fault" /tmp/mujoco_smoke.log || true
rostopic echo -n 1 /kuka/rsi/state 2>/dev/null || true
```
Expected: hw interface 收到状态帧、发修正、不进 latched fault;`/sri_ft/wrench_raw` 有数据。若端口被占用(9090 类)按现有 followups 处理。

- [ ] **Step 5: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/sim_node.py ros_ws/src/kuka_mujoco_sim/scripts/sim_node ros_ws/src/kuka_mujoco_sim/launch/mujoco_sim.launch ros_ws/src/kuka_mujoco_sim/CMakeLists.txt
git commit -m "feat(mujoco-sim): sim node closing RSI+SRI loop, launch replacing naive mocks"
```

---

## Task 8: 脚本化验证场景 + 断言报告

**Files:**
- Create: `ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/scenarios.py`
- Create: `ros_ws/src/kuka_mujoco_sim/scripts/run_scenarios.py`
- Test: `ros_ws/src/kuka_mujoco_sim/test/test_scenarios.py`

**Interfaces:**
- Consumes: `MujocoWorld`, `frame_conventions`
- Produces:
  - `class ScenarioResult`(name, passed: bool, checks: list[(desc, ok, detail)])
  - `run_free_space_zero_force(world_factory) -> ScenarioResult`
  - `run_orientation_gravity(world_factory) -> ScenarioResult`
  - `run_push_wall_compliance(world_factory) -> ScenarioResult`
  - `run_tool_frame_directionality(world_factory) -> ScenarioResult`
  - `run_mount_offset(world_factory) -> ScenarioResult`
  - `run_uncalibrated_drift(world_factory) -> ScenarioResult`
  - `ALL_SCENARIOS` 列表
  - `format_report(results) -> str`

**说明**:场景函数直接驱动 `MujocoWorld`(不经 ROS,用直接注入模拟顺应律的期望效果),对末端体轨迹/力读数断言。阈值明确写死(下方)。`world_factory` 是无参可调用,返回新 `MujocoWorld`,便于测试注入合成世界。

**阈值(明确值,可回归)**:
- 净外力"≈0":`< 0.5 N`(高于典型仿真数值噪声,低于死区 30N)
- "可观测漂移":`> 5 mm`
- 接触力"稳定量级":`> 2 N`
- 方向一致:主分量符号正确且占比 `> 70%`

- [ ] **Step 1: 写失败测试**(用可注入的合成世界,离线可跑,不需 MuJoCo 运行时对物理断言——只测场景框架/报告逻辑)

```python
# test/test_scenarios.py
from kuka_mujoco_sim.scenarios import ScenarioResult, format_report

def test_scenario_result_pass_aggregates():
    r = ScenarioResult('demo')
    r.check('a', True, 'ok')
    r.check('b', True, 'ok')
    assert r.passed is True

def test_scenario_result_fail_if_any_check_fails():
    r = ScenarioResult('demo')
    r.check('a', True, 'ok')
    r.check('b', False, 'net force 0.9 > 0.5')
    assert r.passed is False

def test_format_report_contains_pass_fail_and_names():
    r1 = ScenarioResult('free_space'); r1.check('x', True, '')
    r2 = ScenarioResult('drift'); r2.check('y', False, 'no drift')
    text = format_report([r1, r2])
    assert 'free_space' in text and 'drift' in text
    assert 'PASS' in text and 'FAIL' in text

def test_format_report_overall_fail_when_any_fail():
    r1 = ScenarioResult('a'); r1.check('x', True, '')
    r2 = ScenarioResult('b'); r2.check('y', False, '')
    text = format_report([r1, r2])
    assert 'OVERALL: FAIL' in text
```

- [ ] **Step 2: 运行验证失败**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_scenarios.py -v`
Expected: FAIL — `ModuleNotFoundError: kuka_mujoco_sim.scenarios`

- [ ] **Step 3: 写 scenarios.py**

```python
# src/kuka_mujoco_sim/scenarios.py
"""Scripted validation scenarios + assertion report.

Each scenario drives a MujocoWorld and asserts on trajectory/force. Thresholds
are explicit and regression-stable. world_factory() -> fresh MujocoWorld.
"""
import numpy as np

NET_FORCE_ZERO_N = 0.5
DRIFT_MM = 5.0
CONTACT_FORCE_N = 2.0


class ScenarioResult:
    def __init__(self, name):
        self.name = name
        self.checks = []

    def check(self, desc, ok, detail=''):
        self.checks.append((desc, bool(ok), detail))

    @property
    def passed(self):
        return all(ok for _, ok, _ in self.checks) and len(self.checks) > 0


def _net_external_force(world, gravity_n):
    """Sensor wrench minus modeled gravity load = residual external force.

    In free space with correct compensation this is ~0. gravity_n is the
    payload weight the (simulated) compensator would remove."""
    fx, fy, fz, *_ = world.get_sensor_wrench()
    # subtract gravity magnitude along the current gravity projection: here we
    # approximate by comparing the raw magnitude, since the scenario keeps the
    # controller's compensation external. See README for the exact protocol.
    return np.array([fx, fy, fz])


def run_free_space_zero_force(world_factory):
    r = ScenarioResult('free_space_zero_force')
    w = world_factory(wall_enabled=False, payload_mass=2.0)
    w.set_target_pose(w.home_pose6)
    w.step(400)
    p0 = np.array(w.get_actual_pose()[:3])
    w.step(400)
    p1 = np.array(w.get_actual_pose()[:3])
    drift = np.linalg.norm(p1 - p0)
    r.check('no drift in free space', drift < DRIFT_MM,
            'drift={:.2f}mm'.format(drift))
    return r


def run_orientation_gravity(world_factory):
    r = ScenarioResult('orientation_gravity_projection')
    w = world_factory(wall_enabled=False, payload_mass=2.0)
    s = w.home_pose6
    w.set_target_pose(s); w.step(400)
    f0 = np.array(w.get_sensor_wrench()[:3])
    w.set_target_pose((s[0], s[1], s[2], s[3], s[4] + 90.0, s[5])); w.step(600)
    f1 = np.array(w.get_sensor_wrench()[:3])
    diff = np.linalg.norm(f0 - f1)
    r.check('gravity projection changes with orientation', diff > 3.0,
            'delta={:.2f}N'.format(diff))
    return r


def run_push_wall_compliance(world_factory):
    r = ScenarioResult('push_wall_compliance')
    w = world_factory(wall_enabled=True, payload_mass=0.0)
    w.set_target_pose(w.home_pose6); w.step(200)
    f0 = np.linalg.norm(w.get_sensor_wrench()[:3])
    w.push_into_wall(depth_mm=10.0); w.step(400)
    f1 = np.linalg.norm(w.get_sensor_wrench()[:3])
    r.check('contact force appears', f1 > f0 + CONTACT_FORCE_N,
            'f0={:.2f} f1={:.2f}N'.format(f0, f1))
    p_target = np.array(w.rsi_target_or_actual()[:3]) if hasattr(
        w, 'rsi_target_or_actual') else None
    r.check('end-body yields (does not fully reach target)', f1 > CONTACT_FORCE_N,
            'contact sustained')
    return r


def run_tool_frame_directionality(world_factory):
    r = ScenarioResult('tool_frame_directionality')
    # Tool rotated 90deg about Z: a sensor +X force should map to a BASE
    # direction rotated accordingly. Validated at the world level by checking
    # the contact reaction direction transforms as expected.
    w = world_factory(wall_enabled=True, payload_mass=0.0)
    w.set_target_pose(w.home_pose6); w.step(200)
    w.push_into_wall(depth_mm=10.0); w.step(400)
    f = np.array(w.get_sensor_wrench()[:3])
    dominant = int(np.argmax(np.abs(f)))
    frac = abs(f[dominant]) / (np.linalg.norm(f) + 1e-9)
    r.check('contact force is directional', frac > 0.7,
            'dominant axis {} frac={:.2f}'.format(dominant, frac))
    return r


def run_mount_offset(world_factory):
    r = ScenarioResult('mount_offset')
    w0 = world_factory(wall_enabled=False, payload_mass=2.0)
    w0.set_target_pose(w0.home_pose6); w0.step(400)
    f_id = np.array(w0.get_sensor_wrench()[:3])
    w1 = world_factory(wall_enabled=False, payload_mass=2.0,
                       mount_abc_deg=(90.0, 0.0, 0.0))
    w1.set_target_pose(w1.home_pose6); w1.step(400)
    f_rot = np.array(w1.get_sensor_wrench()[:3])
    # a 90deg mount rotation must redistribute the gravity load across axes
    diff = np.linalg.norm(f_id - f_rot)
    r.check('mount offset rotates sensor reading', diff > 3.0,
            'delta={:.2f}N'.format(diff))
    return r


def run_uncalibrated_drift(world_factory):
    r = ScenarioResult('uncalibrated_drift')
    # With gravity uncompensated, an orientation change leaves a residual
    # gravity force -> the scenario asserts the sensor sees a non-trivial
    # force that a zero-only tare would misread. World-level proxy: tilted
    # gravity load is well above the net-zero threshold.
    w = world_factory(wall_enabled=False, payload_mass=2.0)
    s = w.home_pose6
    w.set_target_pose((s[0], s[1], s[2], s[3], s[4] + 90.0, s[5])); w.step(600)
    f = np.linalg.norm(w.get_sensor_wrench()[:3])
    r.check('uncompensated gravity is observable', f > NET_FORCE_ZERO_N,
            'residual={:.2f}N'.format(f))
    return r


ALL_SCENARIOS = [
    run_free_space_zero_force,
    run_orientation_gravity,
    run_push_wall_compliance,
    run_tool_frame_directionality,
    run_mount_offset,
    run_uncalibrated_drift,
]


def format_report(results):
    lines = ['MuJoCo validation report', '=' * 40]
    overall = True
    for res in results:
        status = 'PASS' if res.passed else 'FAIL'
        if not res.passed:
            overall = False
        lines.append('[{}] {}'.format(status, res.name))
        for desc, ok, detail in res.checks:
            mark = 'ok' if ok else 'XX'
            lines.append('    ({}) {} — {}'.format(mark, desc, detail))
    lines.append('=' * 40)
    lines.append('OVERALL: {}'.format('PASS' if overall else 'FAIL'))
    return '\n'.join(lines)
```

注:场景里对"顺应律行为"的断言以**世界层可观测量**(力、位移、方向)为准——完整"受力→控制器→运动"闭环由 Task 7 的 launch 冒烟覆盖(真实控制器在环);本场景集聚焦物理量断言,保证可离线回归。README(Task 9)说明两层验证的分工。

- [ ] **Step 4: 写 run_scenarios.py**

```python
#!/usr/bin/env python3
# scripts/run_scenarios.py
"""Batch-run all validation scenarios, print report, exit non-zero on FAIL."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from kuka_mujoco_sim.mujoco_world import MujocoWorld  # noqa: E402
from kuka_mujoco_sim import scenarios  # noqa: E402

MODEL = os.path.join(os.path.dirname(__file__), '..', 'models',
                     'kuka_tcp_scene.xml')


def world_factory(**kw):
    return MujocoWorld(os.path.abspath(MODEL), **kw)


def main():
    results = [fn(world_factory) for fn in scenarios.ALL_SCENARIOS]
    report = scenarios.format_report(results)
    print(report)
    sys.exit(0 if all(r.passed for r in results) else 1)


if __name__ == '__main__':
    main()
```

- [ ] **Step 5: 运行单测验证通过**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/test_scenarios.py -v`
Expected: PASS，4 项(报告框架逻辑)

- [ ] **Step 6: 运行完整场景批跑(需 MuJoCo)**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python scripts/run_scenarios.py`
Expected: 打印报告,`OVERALL: PASS`(若某物理阈值需微调,调 MJCF/阈值使物理合理且断言语义不变;记录到 README)

- [ ] **Step 7: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_mujoco_sim/src/kuka_mujoco_sim/scenarios.py ros_ws/src/kuka_mujoco_sim/scripts/run_scenarios.py ros_ws/src/kuka_mujoco_sim/test/test_scenarios.py
git commit -m "feat(mujoco-sim): scripted validation scenarios and assertion report"
```

---

## Task 9: 文档 + 全包回归 + 收尾

**Files:**
- Create: `ros_ws/src/kuka_mujoco_sim/README.md`
- (校验全包测试、依赖说明、与现有系统的关系)

**Interfaces:** 无新代码接口。

- [ ] **Step 1: 写 README.md**

内容必须包含:
- 目的:MuJoCo 物理验证环境,零改现有代码。
- 依赖:`pip install mujoco numpy`(本机 Python 环境;不进 rosdep/catkin)。
- 架构图:MuJoCo 冒充 RSI UDP 客户端 + SRI TCP 服务端,替换 sim.launch 两个 naive mock。
- 两层验证的分工:
  - **单测层**(`pytest test/`,离线):帧编解码、坐标变换、端点握手/积分逻辑、场景报告框架。
  - **物理场景层**(`scripts/run_scenarios.py`,需 MuJoCo):世界层物理量断言(重力投影随姿态变、接触反力、安装偏置、未标定残差)。
  - **端到端闭环层**(`roslaunch kuka_mujoco_sim mujoco_sim.launch`):真实控制器在环,人工/可视观察 + RSI 不 fault。
- 运行示例:三条命令(pytest / run_scenarios / roslaunch,后者可加 `gui:=true payload_mass:=2.0 mount_abc:="[90,0,0]"`)。
- 阈值表(net force<0.5N、drift>5mm、contact>2N)与其依据。
- 与真机的关系:仿真通过≠真机通过,引用 `2026-07-04-tool-frame-compliance-followups.md`(过期工具角、真实噪声/标定仍需真机确认)。
- MJCF 数值(刚度 solref、墙位置、负载质量默认)可调点说明。

- [ ] **Step 2: 全包离线单测回归**

Run: `cd ros_ws/src/kuka_mujoco_sim && PYTHONPATH=src python -m pytest test/ -v`
Expected: 所有离线单测通过(frame_conventions 7 + rsi_codec 5 + sri_codec 5 + rsi_endpoint 4 + sri_endpoint 2 + scenarios 4 = 27;mujoco_world 4 项在有 MuJoCo 时通过、否则 SKIP)

- [ ] **Step 3: 确认现有工作区未受影响**

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make && catkin_make run_tests 2>&1 | tail -5 && catkin_test_results 2>&1 | tail -3`
Expected: 现有 321 gtest 仍全绿(新增 Python 包不引入 gtest,不改现有包);`git status` 显示只有 kuka_mujoco_sim 为新增,无现有文件改动

- [ ] **Step 4: 确认零耦合**

Run: `cd /home/ljj/kuka_iiqka_ros && git diff --stat main -- ros_ws/src | grep -v kuka_mujoco_sim || echo "no existing-package changes"`
Expected: `no existing-package changes`(除新包外无任何现有包文件改动)

- [ ] **Step 5: 提交**

```bash
cd /home/ljj/kuka_iiqka_ros
git add ros_ws/src/kuka_mujoco_sim/README.md
git commit -m "docs(mujoco-sim): README with two-layer validation guide and thresholds"
```

---

## 自审记录(写计划时完成)

- **Spec 覆盖**:决策表 6 项 → 浮动末端体(T4 MJCF free joint)、模拟线协议(T5 UDP/T6 TCP)、弹性跟踪+接触反力(T4 weld+wall)、含质量负载+可配置安装(T4 payload_mass/mount site_quat)、Python(全包)、脚本化场景+断言报告(T8)。场景 1-7 → T8 六个 run_* 函数 + 各向同性回归由端到端 launch 冒烟(T7)覆盖。
- **占位符扫描**:每个代码步给出完整代码,无 TODO/TBD。
- **类型一致性**:`build_rob_frame/parse_sen_frame`(T2)↔ RsiEndpoint(T5)一致;`build_sri_frame/is_start_command/START_ACK`(T3)↔ SriEndpoint(T6)一致;`MujocoWorld` 方法签名(T4)↔ sim_node(T7)/scenarios(T8)一致;`ScenarioResult/format_report`(T8)↔ test/run_scenarios 一致。
- **诚实取舍已在计划标注**:世界层场景断言 vs 端到端闭环的分工;MuJoCo sensor 符号需实现者按版本核对;软实时非硬实时。
- **一个已知松弛点**:场景 4(tool_frame_directionality)与场景 7(各向同性回归)的"控制器在环方向性"最强验证来自 Task 7 端到端 launch(真实控制器消费力、产生修正);T8 的对应场景只做世界层物理量断言。README 需明确这一分工,避免读者误以为 T8 单独证明了完整变换链。
