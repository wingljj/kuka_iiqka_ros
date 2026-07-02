# Plan 1/6: `soft_force_control_core` Pure Algorithm Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the ROS-free C++ algorithm library (`soft_force_control_core`) that reimplements the legacy SoftRobot compliance/compensation behavior in testable form, per spec `docs/superpowers/specs/2026-07-02-kuka-rsi-ros-control-design.md` Sections 5.4 and 7.

**Architecture:** A catkin package containing a plain C++ static/shared library with zero ROS runtime dependencies (catkin used only for build/test plumbing). Each algorithm is a small class with deterministic, allocation-free compute methods, unit-tested with gtest. Later plans (2-6) wrap this library in `ros_control` controllers and the manager.

**Tech Stack:** C++14, Eigen3, gtest (via `catkin_add_gtest`), catkin/ROS1 Noetic build tooling.

**Plan series:** ① core algorithm library (this plan) → ② `kuka_rsi_hw_interface` + RSI mock + msgs → ③ `soft_robot_controllers` → ④ `kuka_eki_bridge` + `sri_force_torque_driver` + mocks → ⑤ manager + calibration + bringup + KUKA-side templates → ⑥ web interface.

## Global Constraints

- ROS1 Noetic, catkin workspace at `ros_ws/` in the repo root; this package: `ros_ws/src/soft_force_control_core/`.
- Core library has **no ROS dependency**: no `roscpp`, no sockets, no file I/O in realtime functions (spec §5.4).
- No dynamic allocation, blocking calls, or exceptions thrown in compute/hot-path methods (spec §7.1).
- Units: positions mm, angles deg (KUKA A/B/C = Z-Y-X Euler), forces N, torques Nm, CoM meters, time seconds. RSI cycle dt = 0.004 s (spec §6).
- Defaults copied from legacy/spec verbatim: adaptive deadband margin **5 N / 1 Nm**, ramp window **2 s**; PRECISION fixed deadband **30 N / 4 Nm**; global hard cutoff **500 N** (spec §7.3-7.4, §12.1).
- Language: C++14, `-Wall -Wextra`, all code and comments in English.
- Every task follows TDD: failing test → minimal implementation → pass → commit.

---

## File Structure

```text
ros_ws/src/soft_force_control_core/
  package.xml
  CMakeLists.txt
  include/soft_force_control_core/
    types.h                     # CartesianState, Wrench, CartesianCorrection, enums
    mode_manager_core.h         # ModeSnapshot + transition rules
    force_torque_filter.h       # first-order low-pass
    rotation.h                  # KUKA ABC Euler helpers (header-only, Eigen)
    tool_gravity_compensator.h  # bias + gravity/CoM compensation
    compliance_law.h            # admittance law (deadzone/gain/clamp/rate-limit)
    safety_limiter.h            # per-axis clamp + global hard cutoff
    adaptive_deadband.h         # startup ramp (legacy LIMSET)
    auto_retare.h               # return-to-start re-tare predicate (legacy isReset)
    orientation_motion_core.h   # goal-seeking correction generator (legacy RotFixAng)
    payload_estimator.h         # least-squares payload fit (legacy MassCul)
  src/
    mode_manager_core.cpp
    force_torque_filter.cpp
    tool_gravity_compensator.cpp
    compliance_law.cpp
    safety_limiter.cpp
    adaptive_deadband.cpp
    auto_retare.cpp
    orientation_motion_core.cpp
    payload_estimator.cpp
  test/
    test_mode_manager.cpp
    test_filter.cpp
    test_rotation.cpp
    test_gravity_compensator.cpp
    test_compliance_law.cpp
    test_safety_limiter.cpp
    test_adaptive_deadband.cpp
    test_auto_retare.cpp
    test_orientation_motion.cpp
    test_payload_estimator.cpp
```

Build/test commands used throughout (run from repo root `/home/ljj/kuka_iiqka_ros`):

```bash
cd ros_ws && catkin_make tests            # build library + all test binaries
./devel/lib/soft_force_control_core/<test_binary>   # run one gtest binary
```

---

### Task 1: Package skeleton, core types, ModeManagerCore

**Files:**
- Create: `ros_ws/src/soft_force_control_core/package.xml`
- Create: `ros_ws/src/soft_force_control_core/CMakeLists.txt`
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/types.h`
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/mode_manager_core.h`
- Create: `ros_ws/src/soft_force_control_core/src/mode_manager_core.cpp`
- Create: `ros_ws/.gitignore`
- Test: `ros_ws/src/soft_force_control_core/test/test_mode_manager.cpp`

**Interfaces:**
- Consumes: nothing (first task).
- Produces (used by every later task):
  - `sfc::CartesianState {x,y,z,a,b,c}` (mm/deg)
  - `sfc::Wrench {fx,fy,fz,tx,ty,tz}` with `double forceNorm() const`, `double torqueNorm() const`
  - `sfc::CartesianCorrection {x,y,z,a,b,c}` (mm/deg per cycle)
  - `enum class sfc::ControlMode { IDLE, DIRECT_CARTESIAN, FORCE_COMPLIANCE, CALIBRATION }`
  - `enum class sfc::Profile { DRAG, PRECISION }`
  - `sfc::ModeSnapshot {ControlMode mode; Profile profile;}`
  - `sfc::ModeManagerCore` with `bool requestMode(ControlMode)`, `bool setProfile(Profile)`, `ModeSnapshot snapshot() const`

- [ ] **Step 1: Create workspace ignore file and package manifest**

`ros_ws/.gitignore`:

```gitignore
build/
devel/
.catkin_workspace
```

`ros_ws/src/soft_force_control_core/package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>soft_force_control_core</name>
  <version>0.1.0</version>
  <description>
    Pure C++ force-compliance algorithm library. No ROS runtime dependency.
    Reimplements the legacy SoftRobot SoftControl/LoadMass behavior in
    deterministic, unit-testable form.
  </description>
  <maintainer email="dev@example.com">ljj</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>eigen</depend>
</package>
```

`ros_ws/src/soft_force_control_core/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(soft_force_control_core)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/mode_manager_core.cpp
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mode_manager test/test_mode_manager.cpp)
  target_link_libraries(test_mode_manager ${PROJECT_NAME})
endif()
```

- [ ] **Step 2: Write types header**

`include/soft_force_control_core/types.h`:

```cpp
#pragma once
#include <cmath>

namespace sfc {

// KUKA Cartesian pose: mm for x/y/z, deg for a/b/c (A=rotZ, B=rotY, C=rotX).
struct CartesianState {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};
};

// Force/torque sample: N for forces, Nm for torques.
struct Wrench {
  double fx{0}, fy{0}, fz{0}, tx{0}, ty{0}, tz{0};
  double forceNorm() const { return std::sqrt(fx * fx + fy * fy + fz * fz); }
  double torqueNorm() const { return std::sqrt(tx * tx + ty * ty + tz * tz); }
};

// Per-cycle RSI Cartesian correction (RKorr): mm and deg per cycle.
struct CartesianCorrection {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};
};

enum class ControlMode { IDLE, DIRECT_CARTESIAN, FORCE_COMPLIANCE, CALIBRATION };
enum class Profile { DRAG, PRECISION };

}  // namespace sfc
```

- [ ] **Step 3: Write the failing test for ModeManagerCore**

`test/test_mode_manager.cpp`:

```cpp
#include <gtest/gtest.h>
#include "soft_force_control_core/mode_manager_core.h"

using sfc::ControlMode;
using sfc::ModeManagerCore;
using sfc::Profile;

TEST(ModeManager, StartsIdlePrecision) {
  ModeManagerCore m;
  EXPECT_EQ(m.snapshot().mode, ControlMode::IDLE);
  EXPECT_EQ(m.snapshot().profile, Profile::PRECISION);
}

TEST(ModeManager, IdleCanEnterAnyMode) {
  for (auto target : {ControlMode::DIRECT_CARTESIAN,
                      ControlMode::FORCE_COMPLIANCE,
                      ControlMode::CALIBRATION}) {
    ModeManagerCore m;
    EXPECT_TRUE(m.requestMode(target));
    EXPECT_EQ(m.snapshot().mode, target);
  }
}

TEST(ModeManager, AnyModeCanReturnToIdle) {
  ModeManagerCore m;
  ASSERT_TRUE(m.requestMode(ControlMode::FORCE_COMPLIANCE));
  EXPECT_TRUE(m.requestMode(ControlMode::IDLE));
  EXPECT_EQ(m.snapshot().mode, ControlMode::IDLE);
}

TEST(ModeManager, DirectSwitchBetweenActiveModesRejected) {
  ModeManagerCore m;
  ASSERT_TRUE(m.requestMode(ControlMode::FORCE_COMPLIANCE));
  EXPECT_FALSE(m.requestMode(ControlMode::DIRECT_CARTESIAN));
  EXPECT_FALSE(m.requestMode(ControlMode::CALIBRATION));
  EXPECT_EQ(m.snapshot().mode, ControlMode::FORCE_COMPLIANCE);
}

TEST(ModeManager, ProfileChangeOnlyInIdle) {
  ModeManagerCore m;
  EXPECT_TRUE(m.setProfile(Profile::DRAG));
  ASSERT_TRUE(m.requestMode(ControlMode::FORCE_COMPLIANCE));
  EXPECT_FALSE(m.setProfile(Profile::PRECISION));
  EXPECT_EQ(m.snapshot().profile, Profile::DRAG);
}
```

- [ ] **Step 4: Run test to verify it fails**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests
```

Expected: BUILD FAILS with `mode_manager_core.h: No such file or directory`.

- [ ] **Step 5: Write minimal implementation**

`include/soft_force_control_core/mode_manager_core.h`:

```cpp
#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

struct ModeSnapshot {
  ControlMode mode{ControlMode::IDLE};
  Profile profile{Profile::PRECISION};
};

// Validates mode transitions. Rule (spec section 10): all mode changes pass
// through IDLE; direct switches between active modes are rejected.
class ModeManagerCore {
 public:
  bool requestMode(ControlMode target);
  bool setProfile(Profile profile);  // only allowed while IDLE
  ModeSnapshot snapshot() const { return snap_; }

 private:
  ModeSnapshot snap_;
};

}  // namespace sfc
```

`src/mode_manager_core.cpp`:

```cpp
#include "soft_force_control_core/mode_manager_core.h"

namespace sfc {

bool ModeManagerCore::requestMode(ControlMode target) {
  if (target == snap_.mode) return true;
  const bool allowed =
      target == ControlMode::IDLE || snap_.mode == ControlMode::IDLE;
  if (allowed) snap_.mode = target;
  return allowed;
}

bool ModeManagerCore::setProfile(Profile profile) {
  if (snap_.mode != ControlMode::IDLE) return false;
  snap_.profile = profile;
  return true;
}

}  // namespace sfc
```

- [ ] **Step 6: Run test to verify it passes**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_force_control_core/test_mode_manager
```

Expected: `[  PASSED  ] 5 tests.`

- [ ] **Step 7: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/.gitignore ros_ws/src/soft_force_control_core && \
git commit -m "feat(core): add package skeleton, core types, ModeManagerCore"
```

---

### Task 2: ForceTorqueFilter

**Files:**
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/force_torque_filter.h`
- Create: `ros_ws/src/soft_force_control_core/src/force_torque_filter.cpp`
- Modify: `ros_ws/src/soft_force_control_core/CMakeLists.txt` (add source + test target)
- Test: `ros_ws/src/soft_force_control_core/test/test_filter.cpp`

**Interfaces:**
- Consumes: `sfc::Wrench` (Task 1).
- Produces: `sfc::ForceTorqueFilter` with
  `explicit ForceTorqueFilter(double cutoff_hz)`,
  `Wrench filter(const Wrench& in, double dt)`,
  `void reset()`. `cutoff_hz <= 0` means pass-through.

- [ ] **Step 1: Write the failing test**

`test/test_filter.cpp`:

```cpp
#include <gtest/gtest.h>
#include "soft_force_control_core/force_torque_filter.h"

using sfc::ForceTorqueFilter;
using sfc::Wrench;

namespace {
Wrench step(double v) {
  Wrench w;
  w.fx = w.fy = w.fz = w.tx = w.ty = w.tz = v;
  return w;
}
}  // namespace

TEST(Filter, NonPositiveCutoffPassesThrough) {
  ForceTorqueFilter f(0.0);
  Wrench out = f.filter(step(7.0), 0.004);
  EXPECT_DOUBLE_EQ(out.fx, 7.0);
  EXPECT_DOUBLE_EQ(out.tz, 7.0);
}

TEST(Filter, FirstSampleInitializesState) {
  ForceTorqueFilter f(10.0);
  Wrench out = f.filter(step(5.0), 0.004);
  EXPECT_DOUBLE_EQ(out.fx, 5.0);  // no startup transient from zero
}

TEST(Filter, SmoothsStepInput) {
  ForceTorqueFilter f(10.0);
  f.filter(step(0.0), 0.004);
  Wrench out = f.filter(step(100.0), 0.004);
  EXPECT_GT(out.fx, 0.0);
  EXPECT_LT(out.fx, 100.0);
}

TEST(Filter, ConvergesToStepValue) {
  ForceTorqueFilter f(10.0);
  f.filter(step(0.0), 0.004);
  Wrench out;
  for (int i = 0; i < 5000; ++i) out = f.filter(step(100.0), 0.004);
  EXPECT_NEAR(out.fx, 100.0, 1e-6);
  EXPECT_NEAR(out.tz, 100.0, 1e-6);
}

TEST(Filter, ResetClearsState) {
  ForceTorqueFilter f(10.0);
  f.filter(step(100.0), 0.004);
  f.reset();
  Wrench out = f.filter(step(5.0), 0.004);
  EXPECT_DOUBLE_EQ(out.fx, 5.0);  // behaves like first sample again
}
```

- [ ] **Step 2: Add test target and run to verify it fails**

In `CMakeLists.txt`, extend the library and test section:

```cmake
add_library(${PROJECT_NAME}
  src/mode_manager_core.cpp
  src/force_torque_filter.cpp
)
```

and inside `if(CATKIN_ENABLE_TESTING)`:

```cmake
  catkin_add_gtest(test_filter test/test_filter.cpp)
  target_link_libraries(test_filter ${PROJECT_NAME})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
Expected: BUILD FAILS with `force_torque_filter.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

`include/soft_force_control_core/force_torque_filter.h`:

```cpp
#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

// First-order low-pass filter applied independently to all six channels.
// alpha = dt / (dt + 1/(2*pi*cutoff_hz)). cutoff_hz <= 0 disables filtering.
class ForceTorqueFilter {
 public:
  explicit ForceTorqueFilter(double cutoff_hz) : cutoff_hz_(cutoff_hz) {}
  Wrench filter(const Wrench& in, double dt);
  void reset() { initialized_ = false; }

 private:
  double cutoff_hz_;
  bool initialized_{false};
  Wrench state_;
};

}  // namespace sfc
```

`src/force_torque_filter.cpp`:

```cpp
#include "soft_force_control_core/force_torque_filter.h"

namespace sfc {

Wrench ForceTorqueFilter::filter(const Wrench& in, double dt) {
  if (cutoff_hz_ <= 0.0) return in;
  if (!initialized_) {
    state_ = in;
    initialized_ = true;
    return state_;
  }
  const double rc = 1.0 / (2.0 * M_PI * cutoff_hz_);
  const double alpha = dt / (dt + rc);
  state_.fx += alpha * (in.fx - state_.fx);
  state_.fy += alpha * (in.fy - state_.fy);
  state_.fz += alpha * (in.fz - state_.fz);
  state_.tx += alpha * (in.tx - state_.tx);
  state_.ty += alpha * (in.ty - state_.ty);
  state_.tz += alpha * (in.tz - state_.tz);
  return state_;
}

}  // namespace sfc
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_force_control_core/test_filter
```

Expected: `[  PASSED  ] 5 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_force_control_core && \
git commit -m "feat(core): add first-order low-pass ForceTorqueFilter"
```

---

### Task 3: Rotation utilities (KUKA A/B/C Euler)

**Files:**
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/rotation.h` (header-only)
- Modify: `ros_ws/src/soft_force_control_core/CMakeLists.txt` (add test target only)
- Test: `ros_ws/src/soft_force_control_core/test/test_rotation.cpp`

**Interfaces:**
- Consumes: Eigen3.
- Produces (header-only, namespace `sfc`):
  - `Eigen::Matrix3d kukaAbcToRotation(double a_deg, double b_deg, double c_deg)` — `R = Rz(A) * Ry(B) * Rx(C)`.
  - `double wrapDeg(double deg)` — wraps into `[-180, 180)`.
  - `double angularDistanceDeg(double a1, double b1, double c1, double a2, double b2, double c2)` — geodesic angle between the two orientations.

- [ ] **Step 1: Write the failing test**

`test/test_rotation.cpp`:

```cpp
#include <gtest/gtest.h>
#include "soft_force_control_core/rotation.h"

using sfc::angularDistanceDeg;
using sfc::kukaAbcToRotation;
using sfc::wrapDeg;

TEST(Rotation, IdentityForZeroAngles) {
  Eigen::Matrix3d r = kukaAbcToRotation(0, 0, 0);
  EXPECT_TRUE(r.isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(Rotation, AIsRotationAboutZ) {
  // A=90 deg: base x-axis maps to y-axis.
  Eigen::Vector3d v = kukaAbcToRotation(90, 0, 0) * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(v.x(), 0.0, 1e-12);
  EXPECT_NEAR(v.y(), 1.0, 1e-12);
}

TEST(Rotation, CIsAppliedFirst) {
  // R = Rz(A)*Ry(B)*Rx(C): with C=90, z-axis maps to -y before A/B.
  Eigen::Vector3d v = kukaAbcToRotation(0, 0, 90) * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(v.y(), -1.0, 1e-12);
}

TEST(Rotation, WrapDeg) {
  EXPECT_DOUBLE_EQ(wrapDeg(190.0), -170.0);
  EXPECT_DOUBLE_EQ(wrapDeg(-190.0), 170.0);
  EXPECT_DOUBLE_EQ(wrapDeg(0.0), 0.0);
}

TEST(Rotation, AngularDistanceZeroForSame) {
  EXPECT_NEAR(angularDistanceDeg(10, 20, 30, 10, 20, 30), 0.0, 1e-9);
}

TEST(Rotation, AngularDistanceKnownRotation) {
  EXPECT_NEAR(angularDistanceDeg(0, 0, 0, 90, 0, 0), 90.0, 1e-9);
}
```

- [ ] **Step 2: Add test target and run to verify it fails**

In `CMakeLists.txt` testing section add:

```cmake
  catkin_add_gtest(test_rotation test/test_rotation.cpp)
  target_link_libraries(test_rotation ${PROJECT_NAME})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
Expected: BUILD FAILS with `rotation.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

`include/soft_force_control_core/rotation.h`:

```cpp
#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace sfc {

inline double degToRad(double d) { return d * M_PI / 180.0; }
inline double radToDeg(double r) { return r * 180.0 / M_PI; }

// Wrap angle into [-180, 180).
inline double wrapDeg(double deg) {
  double d = std::fmod(deg + 180.0, 360.0);
  if (d < 0) d += 360.0;
  return d - 180.0;
}

// KUKA A/B/C convention: R = Rz(A) * Ry(B) * Rx(C), angles in degrees.
inline Eigen::Matrix3d kukaAbcToRotation(double a_deg, double b_deg,
                                         double c_deg) {
  return (Eigen::AngleAxisd(degToRad(a_deg), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(degToRad(b_deg), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(degToRad(c_deg), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// Geodesic angle (degrees) between two KUKA A/B/C orientations.
inline double angularDistanceDeg(double a1, double b1, double c1, double a2,
                                 double b2, double c2) {
  const Eigen::Matrix3d r =
      kukaAbcToRotation(a1, b1, c1).transpose() * kukaAbcToRotation(a2, b2, c2);
  const double cos_angle = (r.trace() - 1.0) / 2.0;
  return radToDeg(std::acos(std::max(-1.0, std::min(1.0, cos_angle))));
}

}  // namespace sfc
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_force_control_core/test_rotation
```

Expected: `[  PASSED  ] 6 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_force_control_core && \
git commit -m "feat(core): add KUKA ABC rotation utilities"
```

---

### Task 4: ToolGravityCompensator

**Files:**
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/tool_gravity_compensator.h`
- Create: `ros_ws/src/soft_force_control_core/src/tool_gravity_compensator.cpp`
- Modify: `ros_ws/src/soft_force_control_core/CMakeLists.txt`
- Test: `ros_ws/src/soft_force_control_core/test/test_gravity_compensator.cpp`

**Interfaces:**
- Consumes: `sfc::Wrench` (Task 1), `sfc::kukaAbcToRotation` (Task 3).
- Produces:
  - `sfc::PayloadParams { double gravity_n; double com_x, com_y, com_z; Wrench bias; }` (CoM in meters, sensor frame)
  - `sfc::ToolGravityCompensator` with `void setParams(const PayloadParams&)`, `const PayloadParams& params() const`, `Wrench compensate(const Wrench& raw, double a_deg, double b_deg, double c_deg) const`, `void absorbResidual(const Wrench& residual)` (adds residual into bias — used by auto re-tare, Task 7).
- Model: sensor frame assumed aligned with the flange orientation given by robot A/B/C (legacy behavior). Gravity in base frame is `(0, 0, -G)`; in sensor frame `f_g = R^T * (0,0,-G)`; gravity torque `t_g = com x f_g`. `compensated = raw - bias - gravity_terms`.

- [ ] **Step 1: Write the failing test**

`test/test_gravity_compensator.cpp`:

```cpp
#include <gtest/gtest.h>
#include "soft_force_control_core/tool_gravity_compensator.h"

using sfc::PayloadParams;
using sfc::ToolGravityCompensator;
using sfc::Wrench;

TEST(GravityComp, ZeroParamsPassesRawThrough) {
  ToolGravityCompensator g;
  Wrench raw;
  raw.fx = 1.0;
  raw.tz = 2.0;
  Wrench out = g.compensate(raw, 10, 20, 30);
  EXPECT_DOUBLE_EQ(out.fx, 1.0);
  EXPECT_DOUBLE_EQ(out.tz, 2.0);
}

TEST(GravityComp, BiasSubtracted) {
  ToolGravityCompensator g;
  PayloadParams p;
  p.bias.fx = 0.5;
  g.setParams(p);
  Wrench raw;
  raw.fx = 1.5;
  EXPECT_DOUBLE_EQ(g.compensate(raw, 0, 0, 0).fx, 1.0);
}

TEST(GravityComp, GravityCancelledAtIdentityOrientation) {
  // Sensor aligned with base, payload G=10N pulls -Z: raw reads (0,0,-10).
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  g.setParams(p);
  Wrench raw;
  raw.fz = -10.0;
  Wrench out = g.compensate(raw, 0, 0, 0);
  EXPECT_NEAR(out.fx, 0.0, 1e-12);
  EXPECT_NEAR(out.fy, 0.0, 1e-12);
  EXPECT_NEAR(out.fz, 0.0, 1e-12);
}

TEST(GravityComp, GravityRotatesWithOrientation) {
  // C=90 deg (rot about X): base -Z maps to sensor -Y... verify via cancel:
  // whatever the model predicts, feeding exactly that as raw must yield zero.
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  p.com_x = 0.01;
  p.com_y = 0.02;
  p.com_z = 0.03;
  g.setParams(p);
  Wrench zero;
  Wrench predicted = g.compensate(zero, 30, 40, 50);  // = -gravity model
  Wrench raw;
  raw.fx = -predicted.fx;
  raw.fy = -predicted.fy;
  raw.fz = -predicted.fz;
  raw.tx = -predicted.tx;
  raw.ty = -predicted.ty;
  raw.tz = -predicted.tz;
  Wrench out = g.compensate(raw, 30, 40, 50);
  EXPECT_NEAR(out.forceNorm(), 0.0, 1e-12);
  EXPECT_NEAR(out.torqueNorm(), 0.0, 1e-12);
}

TEST(GravityComp, ComProducesTorque) {
  // Identity orientation, G=10N along -Z sensor, CoM offset in +X
  // => torque = com x f = (0.1,0,0) x (0,0,-10) = (0, 1, 0).
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  p.com_x = 0.1;
  g.setParams(p);
  Wrench raw;
  raw.fz = -10.0;
  raw.ty = 1.0;
  Wrench out = g.compensate(raw, 0, 0, 0);
  EXPECT_NEAR(out.torqueNorm(), 0.0, 1e-12);
}

TEST(GravityComp, AbsorbResidualShiftsBias) {
  ToolGravityCompensator g;
  Wrench residual;
  residual.fx = 0.7;
  g.absorbResidual(residual);
  Wrench raw;
  raw.fx = 0.7;
  EXPECT_NEAR(g.compensate(raw, 0, 0, 0).fx, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(g.params().bias.fx, 0.7);
}
```

- [ ] **Step 2: Add build entries and run to verify it fails**

In `CMakeLists.txt`: add `src/tool_gravity_compensator.cpp` to `add_library`, and in the testing section:

```cmake
  catkin_add_gtest(test_gravity_compensator test/test_gravity_compensator.cpp)
  target_link_libraries(test_gravity_compensator ${PROJECT_NAME})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
Expected: BUILD FAILS with `tool_gravity_compensator.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

`include/soft_force_control_core/tool_gravity_compensator.h`:

```cpp
#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

// Payload/bias parameters produced by calibration (spec section 9).
struct PayloadParams {
  double gravity_n{0};                      // payload weight G [N]
  double com_x{0}, com_y{0}, com_z{0};      // center of mass [m], sensor frame
  Wrench bias;                              // sensor zero bias
};

// Subtracts sensor bias and tool gravity (force + CoM torque) from a raw
// wrench. Sensor frame is assumed aligned with the flange orientation
// given by the robot A/B/C angles (legacy FTCompensation behavior).
class ToolGravityCompensator {
 public:
  void setParams(const PayloadParams& p) { params_ = p; }
  const PayloadParams& params() const { return params_; }
  Wrench compensate(const Wrench& raw, double a_deg, double b_deg,
                    double c_deg) const;
  // Auto re-tare support: fold a measured residual into the bias.
  void absorbResidual(const Wrench& residual);

 private:
  PayloadParams params_;
};

}  // namespace sfc
```

`src/tool_gravity_compensator.cpp`:

```cpp
#include "soft_force_control_core/tool_gravity_compensator.h"

#include <Eigen/Dense>

#include "soft_force_control_core/rotation.h"

namespace sfc {

Wrench ToolGravityCompensator::compensate(const Wrench& raw, double a_deg,
                                          double b_deg, double c_deg) const {
  const Eigen::Matrix3d r = kukaAbcToRotation(a_deg, b_deg, c_deg);
  const Eigen::Vector3d f_g =
      r.transpose() * Eigen::Vector3d(0, 0, -params_.gravity_n);
  const Eigen::Vector3d com(params_.com_x, params_.com_y, params_.com_z);
  const Eigen::Vector3d t_g = com.cross(f_g);

  Wrench out;
  out.fx = raw.fx - params_.bias.fx - f_g.x();
  out.fy = raw.fy - params_.bias.fy - f_g.y();
  out.fz = raw.fz - params_.bias.fz - f_g.z();
  out.tx = raw.tx - params_.bias.tx - t_g.x();
  out.ty = raw.ty - params_.bias.ty - t_g.y();
  out.tz = raw.tz - params_.bias.tz - t_g.z();
  return out;
}

void ToolGravityCompensator::absorbResidual(const Wrench& residual) {
  params_.bias.fx += residual.fx;
  params_.bias.fy += residual.fy;
  params_.bias.fz += residual.fz;
  params_.bias.tx += residual.tx;
  params_.bias.ty += residual.ty;
  params_.bias.tz += residual.tz;
}

}  // namespace sfc
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_force_control_core/test_gravity_compensator
```

Expected: `[  PASSED  ] 6 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_force_control_core && \
git commit -m "feat(core): add ToolGravityCompensator with bias absorb"
```

---

### Task 5: ComplianceLaw (admittance: deadzone → gain → clamp → rate limit)

**Files:**
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/compliance_law.h`
- Create: `ros_ws/src/soft_force_control_core/src/compliance_law.cpp`
- Modify: `ros_ws/src/soft_force_control_core/CMakeLists.txt`
- Test: `ros_ws/src/soft_force_control_core/test/test_compliance_law.cpp`

**Interfaces:**
- Consumes: `sfc::Wrench`, `sfc::CartesianCorrection` (Task 1).
- Produces:
  - `sfc::AxisGroupParams { double gain; double deadband; double max_speed; double max_accel; }` — gain in (mm/s)/N or (deg/s)/Nm; `max_accel <= 0` disables rate limiting.
  - `sfc::ComplianceParams { AxisGroupParams translation; AxisGroupParams rotation; double speed_scale; }`
  - `sfc::ComplianceLaw` with `CartesianCorrection compute(const Wrench& compensated, const ComplianceParams& p, double dt)` and `void reset()`.
- Law (spec §7.2): per axis `e = deadzone(F, db)` (subtract deadband, keep sign, for continuity), `v = gain * e * speed_scale`, clamp to `±max_speed`, rate-limit `|v - v_prev| <= max_accel * dt`, output `correction = v * dt`.

- [ ] **Step 1: Write the failing test**

`test/test_compliance_law.cpp`:

```cpp
#include <gtest/gtest.h>
#include "soft_force_control_core/compliance_law.h"

using sfc::CartesianCorrection;
using sfc::ComplianceLaw;
using sfc::ComplianceParams;
using sfc::Wrench;

namespace {
ComplianceParams defaultParams() {
  ComplianceParams p;
  p.translation.gain = 1.0;       // (mm/s)/N
  p.translation.deadband = 5.0;   // N
  p.translation.max_speed = 20.0; // mm/s
  p.translation.max_accel = 0.0;  // disabled unless a test enables it
  p.rotation.gain = 0.5;          // (deg/s)/Nm
  p.rotation.deadband = 1.0;      // Nm
  p.rotation.max_speed = 10.0;    // deg/s
  p.rotation.max_accel = 0.0;
  p.speed_scale = 1.0;
  return p;
}
constexpr double kDt = 0.004;
}  // namespace

TEST(ComplianceLaw, BelowDeadbandOutputsZero) {
  ComplianceLaw law;
  Wrench w;
  w.fx = 4.9;
  w.tz = 0.9;
  CartesianCorrection c = law.compute(w, defaultParams(), kDt);
  EXPECT_DOUBLE_EQ(c.x, 0.0);
  EXPECT_DOUBLE_EQ(c.c, 0.0);
}

TEST(ComplianceLaw, DeadzoneIsContinuous) {
  // Just above deadband: e = |F| - db, so output starts from ~zero.
  ComplianceLaw law;
  Wrench w;
  w.fx = 5.001;
  CartesianCorrection c = law.compute(w, defaultParams(), kDt);
  EXPECT_NEAR(c.x, 1.0 * 0.001 * kDt, 1e-12);
}

TEST(ComplianceLaw, NegativeForceGivesNegativeCorrection) {
  ComplianceLaw law;
  Wrench w;
  w.fy = -15.0;  // e = -10 N -> v = -10 mm/s
  CartesianCorrection c = law.compute(w, defaultParams(), kDt);
  EXPECT_NEAR(c.y, -10.0 * kDt, 1e-12);
}

TEST(ComplianceLaw, SpeedClamped) {
  ComplianceLaw law;
  Wrench w;
  w.fz = 1000.0;  // e = 995 -> v would be 995 mm/s, clamp to 20
  CartesianCorrection c = law.compute(w, defaultParams(), kDt);
  EXPECT_NEAR(c.z, 20.0 * kDt, 1e-12);
}

TEST(ComplianceLaw, SpeedScaleApplies) {
  ComplianceLaw law;
  ComplianceParams p = defaultParams();
  p.speed_scale = 0.5;
  Wrench w;
  w.fx = 15.0;  // e = 10 -> v = 10 * 0.5 = 5 mm/s
  CartesianCorrection c = law.compute(w, p, kDt);
  EXPECT_NEAR(c.x, 5.0 * kDt, 1e-12);
}

TEST(ComplianceLaw, RateLimitBoundsAcceleration) {
  ComplianceLaw law;
  ComplianceParams p = defaultParams();
  p.translation.max_accel = 100.0;  // mm/s^2 -> dv per cycle = 0.4 mm/s
  Wrench w;
  w.fx = 1000.0;
  CartesianCorrection c1 = law.compute(w, p, kDt);
  EXPECT_NEAR(c1.x, 0.4 * kDt, 1e-12);  // first cycle: v limited to 0.4
  CartesianCorrection c2 = law.compute(w, p, kDt);
  EXPECT_NEAR(c2.x, 0.8 * kDt, 1e-12);  // ramps up
}

TEST(ComplianceLaw, ResetClearsVelocityState) {
  ComplianceLaw law;
  ComplianceParams p = defaultParams();
  p.translation.max_accel = 100.0;
  Wrench w;
  w.fx = 1000.0;
  law.compute(w, p, kDt);
  law.reset();
  CartesianCorrection c = law.compute(w, p, kDt);
  EXPECT_NEAR(c.x, 0.4 * kDt, 1e-12);  // ramps from zero again
}

TEST(ComplianceLaw, RotationAxesUseTorques) {
  // Mapping: fx->x, fy->y, fz->z, tz->a, ty->b, tx->c
  // (KUKA A/B/C rotate about Z/Y/X respectively, spec section 6).
  ComplianceLaw law;
  Wrench t;
  t.tx = 3.0;  // e = 2 Nm -> v = 1 deg/s on the C channel
  CartesianCorrection c = law.compute(t, defaultParams(), kDt);
  EXPECT_NEAR(c.c, 0.5 * 2.0 * kDt, 1e-12);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
}
```

- [ ] **Step 2: Add build entries and run to verify it fails**

`CMakeLists.txt`: add `src/compliance_law.cpp` to `add_library`; add test target:

```cmake
  catkin_add_gtest(test_compliance_law test/test_compliance_law.cpp)
  target_link_libraries(test_compliance_law ${PROJECT_NAME})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
Expected: BUILD FAILS with `compliance_law.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

`include/soft_force_control_core/compliance_law.h`:

```cpp
#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

// Parameters for one axis group (translation or rotation).
struct AxisGroupParams {
  double gain{0};       // (mm/s)/N for translation, (deg/s)/Nm for rotation
  double deadband{0};   // N or Nm
  double max_speed{0};  // mm/s or deg/s
  double max_accel{0};  // mm/s^2 or deg/s^2; <= 0 disables rate limiting
};

struct ComplianceParams {
  AxisGroupParams translation;
  AxisGroupParams rotation;
  double speed_scale{1.0};
};

// Per-axis admittance law (spec section 7.2):
//   e = deadzone(F, db); v = clamp(gain*e*scale, max_speed);
//   v = rate_limit(v);   correction = v * dt.
// Axis mapping: fx->x, fy->y, fz->z, tx->c, ty->b, tz->a
// (KUKA A/B/C rotate about Z/Y/X).
class ComplianceLaw {
 public:
  CartesianCorrection compute(const Wrench& compensated,
                              const ComplianceParams& p, double dt);
  void reset();

 private:
  double axisVelocity(double f, const AxisGroupParams& g, double scale,
                      double dt, double& prev_v) const;
  double prev_v_[6] = {0, 0, 0, 0, 0, 0};  // x,y,z,a,b,c
};

}  // namespace sfc
```

`src/compliance_law.cpp`:

```cpp
#include "soft_force_control_core/compliance_law.h"

#include <algorithm>
#include <cmath>

namespace sfc {

namespace {
double deadzone(double f, double db) {
  if (std::fabs(f) <= db) return 0.0;
  return f > 0 ? f - db : f + db;
}
double clamp(double v, double lim) { return std::max(-lim, std::min(lim, v)); }
}  // namespace

double ComplianceLaw::axisVelocity(double f, const AxisGroupParams& g,
                                   double scale, double dt,
                                   double& prev_v) const {
  double v = clamp(g.gain * deadzone(f, g.deadband) * scale, g.max_speed);
  if (g.max_accel > 0) {
    const double dv = g.max_accel * dt;
    v = std::max(prev_v - dv, std::min(prev_v + dv, v));
  }
  prev_v = v;
  return v;
}

CartesianCorrection ComplianceLaw::compute(const Wrench& w,
                                           const ComplianceParams& p,
                                           double dt) {
  CartesianCorrection c;
  c.x = axisVelocity(w.fx, p.translation, p.speed_scale, dt, prev_v_[0]) * dt;
  c.y = axisVelocity(w.fy, p.translation, p.speed_scale, dt, prev_v_[1]) * dt;
  c.z = axisVelocity(w.fz, p.translation, p.speed_scale, dt, prev_v_[2]) * dt;
  c.a = axisVelocity(w.tz, p.rotation, p.speed_scale, dt, prev_v_[3]) * dt;
  c.b = axisVelocity(w.ty, p.rotation, p.speed_scale, dt, prev_v_[4]) * dt;
  c.c = axisVelocity(w.tx, p.rotation, p.speed_scale, dt, prev_v_[5]) * dt;
  return c;
}

void ComplianceLaw::reset() {
  for (double& v : prev_v_) v = 0.0;
}

}  // namespace sfc
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_force_control_core/test_compliance_law
```

Expected: `[  PASSED  ] 8 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_force_control_core && \
git commit -m "feat(core): add admittance ComplianceLaw"
```

---

### Task 6: SafetyLimiter (per-axis clamp + global hard cutoff)

**Files:**
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/safety_limiter.h`
- Create: `ros_ws/src/soft_force_control_core/src/safety_limiter.cpp`
- Modify: `ros_ws/src/soft_force_control_core/CMakeLists.txt`
- Test: `ros_ws/src/soft_force_control_core/test/test_safety_limiter.cpp`

**Interfaces:**
- Consumes: `sfc::Wrench`, `sfc::CartesianCorrection` (Task 1).
- Produces:
  - `sfc::SafetyParams { double max_corr_trans{0.5}; double max_corr_rot{0.05}; double force_ceiling{500.0}; double torque_ceiling{50.0}; }` (per-cycle mm / deg limits; ceilings in N / Nm; legacy 500 N default, spec §12.1)
  - `sfc::SafetyResult { CartesianCorrection correction; bool hard_cutoff; bool saturated; }`
  - `sfc::SafetyLimiter` with `SafetyResult apply(const CartesianCorrection& in, const Wrench& compensated, const SafetyParams& p) const`.

- [ ] **Step 1: Write the failing test**

`test/test_safety_limiter.cpp`:

```cpp
#include <gtest/gtest.h>
#include "soft_force_control_core/safety_limiter.h"

using sfc::CartesianCorrection;
using sfc::SafetyLimiter;
using sfc::SafetyParams;
using sfc::SafetyResult;
using sfc::Wrench;

TEST(SafetyLimiter, PassesSmallCorrectionUnchanged) {
  SafetyLimiter s;
  CartesianCorrection in;
  in.x = 0.1;
  in.a = 0.01;
  SafetyResult r = s.apply(in, Wrench{}, SafetyParams{});
  EXPECT_DOUBLE_EQ(r.correction.x, 0.1);
  EXPECT_DOUBLE_EQ(r.correction.a, 0.01);
  EXPECT_FALSE(r.hard_cutoff);
  EXPECT_FALSE(r.saturated);
}

TEST(SafetyLimiter, ClampsPerAxisAndFlagsSaturation) {
  SafetyLimiter s;
  CartesianCorrection in;
  in.y = 2.0;    // > 0.5 mm default
  in.b = -1.0;   // > 0.05 deg default
  SafetyResult r = s.apply(in, Wrench{}, SafetyParams{});
  EXPECT_DOUBLE_EQ(r.correction.y, 0.5);
  EXPECT_DOUBLE_EQ(r.correction.b, -0.05);
  EXPECT_TRUE(r.saturated);
  EXPECT_FALSE(r.hard_cutoff);
}

TEST(SafetyLimiter, ForceCeilingTriggersHardCutoff) {
  SafetyLimiter s;
  CartesianCorrection in;
  in.x = 0.1;
  Wrench w;
  w.fx = 300.0;
  w.fy = 400.0;  // norm = 500 -> at ceiling; use slightly above
  w.fz = 1.0;
  SafetyResult r = s.apply(in, w, SafetyParams{});
  EXPECT_TRUE(r.hard_cutoff);
  EXPECT_DOUBLE_EQ(r.correction.x, 0.0);
  EXPECT_DOUBLE_EQ(r.correction.a, 0.0);
}

TEST(SafetyLimiter, TorqueCeilingTriggersHardCutoff) {
  SafetyLimiter s;
  CartesianCorrection in;
  in.c = 0.01;
  Wrench w;
  w.tx = 60.0;  // > 50 Nm default
  SafetyResult r = s.apply(in, w, SafetyParams{});
  EXPECT_TRUE(r.hard_cutoff);
  EXPECT_DOUBLE_EQ(r.correction.c, 0.0);
}
```

- [ ] **Step 2: Add build entries and run to verify it fails**

`CMakeLists.txt`: add `src/safety_limiter.cpp` to `add_library`; add test target:

```cmake
  catkin_add_gtest(test_safety_limiter test/test_safety_limiter.cpp)
  target_link_libraries(test_safety_limiter ${PROJECT_NAME})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
Expected: BUILD FAILS with `safety_limiter.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

`include/soft_force_control_core/safety_limiter.h`:

```cpp
#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

struct SafetyParams {
  double max_corr_trans{0.5};   // mm per cycle per axis
  double max_corr_rot{0.05};    // deg per cycle per axis
  double force_ceiling{500.0};  // N; legacy hard cutoff (spec section 12.1)
  double torque_ceiling{50.0};  // Nm
};

struct SafetyResult {
  CartesianCorrection correction;
  bool hard_cutoff{false};  // wrench exceeded ceiling -> zero output
  bool saturated{false};    // at least one axis was clamped
};

class SafetyLimiter {
 public:
  SafetyResult apply(const CartesianCorrection& in, const Wrench& compensated,
                     const SafetyParams& p) const;
};

}  // namespace sfc
```

`src/safety_limiter.cpp`:

```cpp
#include "soft_force_control_core/safety_limiter.h"

#include <algorithm>

namespace sfc {

namespace {
double clampFlag(double v, double lim, bool& saturated) {
  const double c = std::max(-lim, std::min(lim, v));
  if (c != v) saturated = true;
  return c;
}
}  // namespace

SafetyResult SafetyLimiter::apply(const CartesianCorrection& in,
                                  const Wrench& w,
                                  const SafetyParams& p) const {
  SafetyResult r;
  if (w.forceNorm() > p.force_ceiling || w.torqueNorm() > p.torque_ceiling) {
    r.hard_cutoff = true;
    return r;  // correction stays zero-initialized
  }
  r.correction.x = clampFlag(in.x, p.max_corr_trans, r.saturated);
  r.correction.y = clampFlag(in.y, p.max_corr_trans, r.saturated);
  r.correction.z = clampFlag(in.z, p.max_corr_trans, r.saturated);
  r.correction.a = clampFlag(in.a, p.max_corr_rot, r.saturated);
  r.correction.b = clampFlag(in.b, p.max_corr_rot, r.saturated);
  r.correction.c = clampFlag(in.c, p.max_corr_rot, r.saturated);
  return r;
}

}  // namespace sfc
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_force_control_core/test_safety_limiter
```

Expected: `[  PASSED  ] 4 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_force_control_core && \
git commit -m "feat(core): add SafetyLimiter with global hard cutoff"
```

---

### Task 7: AdaptiveDeadband + AutoReTare (legacy LIMSET / isReset)

**Files:**
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/adaptive_deadband.h`
- Create: `ros_ws/src/soft_force_control_core/src/adaptive_deadband.cpp`
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/auto_retare.h`
- Create: `ros_ws/src/soft_force_control_core/src/auto_retare.cpp`
- Modify: `ros_ws/src/soft_force_control_core/CMakeLists.txt`
- Test: `ros_ws/src/soft_force_control_core/test/test_adaptive_deadband.cpp`
- Test: `ros_ws/src/soft_force_control_core/test/test_auto_retare.cpp`

**Interfaces:**
- Consumes: `sfc::Wrench`, `sfc::CartesianState` (Task 1), `sfc::angularDistanceDeg` (Task 3).
- Produces:
  - `sfc::AdaptiveDeadband` with `void start(double window_s, double force_margin_n, double torque_margin_nm)`, `bool update(const Wrench& compensated, double dt)` (returns true while ramping — caller must output zero), `bool active() const`, `double forceDeadband() const`, `double torqueDeadband() const`.
  - `sfc::AutoReTareParams { bool enabled{false}; double orientation_tol_deg{1.0}; }`
  - `sfc::AutoReTare` with `void setReference(double a, double b, double c)`, `bool shouldTare(const CartesianState& s, const Wrench& compensated, double force_deadband, double torque_deadband, const AutoReTareParams& p) const`.
- Behavior notes: deadband after ramp = max residual norm during window + margin (deliberate hardening of the legacy last-sample behavior; spec §7.4). `AutoReTare` is a pure predicate; the caller performs the absorb via `ToolGravityCompensator::absorbResidual` (Task 4).

- [ ] **Step 1: Write the failing tests**

`test/test_adaptive_deadband.cpp`:

```cpp
#include <gtest/gtest.h>
#include "soft_force_control_core/adaptive_deadband.h"

using sfc::AdaptiveDeadband;
using sfc::Wrench;

namespace {
Wrench force(double fx, double tx = 0.0) {
  Wrench w;
  w.fx = fx;
  w.tx = tx;
  return w;
}
constexpr double kDt = 0.004;
}  // namespace

TEST(AdaptiveDeadband, InactiveByDefault) {
  AdaptiveDeadband d;
  EXPECT_FALSE(d.active());
  EXPECT_FALSE(d.update(force(1.0), kDt));
}

TEST(AdaptiveDeadband, ActiveDuringWindowThenDone) {
  AdaptiveDeadband d;
  d.start(0.02, 5.0, 1.0);  // 5 cycles at 4 ms
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(d.active());
    d.update(force(2.0, 0.5), kDt);
  }
  EXPECT_FALSE(d.active());
}

TEST(AdaptiveDeadband, DeadbandIsMaxResidualPlusMargin) {
  AdaptiveDeadband d;
  d.start(0.02, 5.0, 1.0);
  d.update(force(2.0, 0.2), kDt);
  d.update(force(3.0, 0.6), kDt);  // max
  d.update(force(1.0, 0.1), kDt);
  d.update(force(0.5, 0.0), kDt);
  d.update(force(0.5, 0.0), kDt);
  EXPECT_FALSE(d.active());
  EXPECT_NEAR(d.forceDeadband(), 3.0 + 5.0, 1e-12);
  EXPECT_NEAR(d.torqueDeadband(), 0.6 + 1.0, 1e-12);
}

TEST(AdaptiveDeadband, RestartClearsPreviousRamp) {
  AdaptiveDeadband d;
  d.start(0.008, 5.0, 1.0);
  d.update(force(10.0), kDt);
  d.update(force(10.0), kDt);
  d.start(0.008, 5.0, 1.0);
  d.update(force(1.0), kDt);
  d.update(force(1.0), kDt);
  EXPECT_NEAR(d.forceDeadband(), 6.0, 1e-12);
}
```

`test/test_auto_retare.cpp`:

```cpp
#include <gtest/gtest.h>
#include "soft_force_control_core/auto_retare.h"

using sfc::AutoReTare;
using sfc::AutoReTareParams;
using sfc::CartesianState;
using sfc::Wrench;

namespace {
CartesianState pose(double a, double b, double c) {
  CartesianState s;
  s.a = a;
  s.b = b;
  s.c = c;
  return s;
}
}  // namespace

TEST(AutoReTare, DisabledNeverTares) {
  AutoReTare rt;
  rt.setReference(0, 0, 0);
  AutoReTareParams p;  // enabled = false
  EXPECT_FALSE(rt.shouldTare(pose(0, 0, 0), Wrench{}, 10.0, 2.0, p));
}

TEST(AutoReTare, TaresAtReferenceWithLowWrench) {
  AutoReTare rt;
  rt.setReference(10, 20, 30);
  AutoReTareParams p;
  p.enabled = true;
  Wrench w;
  w.fx = 1.0;  // below 10 N deadband
  EXPECT_TRUE(rt.shouldTare(pose(10.2, 20.1, 29.9), w, 10.0, 2.0, p));
}

TEST(AutoReTare, RejectsWhenOrientationFar) {
  AutoReTare rt;
  rt.setReference(0, 0, 0);
  AutoReTareParams p;
  p.enabled = true;
  EXPECT_FALSE(rt.shouldTare(pose(30, 0, 0), Wrench{}, 10.0, 2.0, p));
}

TEST(AutoReTare, RejectsWhenWrenchHigh) {
  AutoReTare rt;
  rt.setReference(0, 0, 0);
  AutoReTareParams p;
  p.enabled = true;
  Wrench w;
  w.fx = 50.0;  // above deadband
  EXPECT_FALSE(rt.shouldTare(pose(0, 0, 0), w, 10.0, 2.0, p));
  Wrench t;
  t.tx = 5.0;  // above torque deadband
  EXPECT_FALSE(rt.shouldTare(pose(0, 0, 0), t, 10.0, 2.0, p));
}
```

- [ ] **Step 2: Add build entries and run to verify it fails**

`CMakeLists.txt`: add `src/adaptive_deadband.cpp` and `src/auto_retare.cpp` to `add_library`; add test targets:

```cmake
  catkin_add_gtest(test_adaptive_deadband test/test_adaptive_deadband.cpp)
  target_link_libraries(test_adaptive_deadband ${PROJECT_NAME})
  catkin_add_gtest(test_auto_retare test/test_auto_retare.cpp)
  target_link_libraries(test_auto_retare ${PROJECT_NAME})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
Expected: BUILD FAILS with `adaptive_deadband.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementations**

`include/soft_force_control_core/adaptive_deadband.h`:

```cpp
#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

// Startup adaptive deadband (legacy LIMSET, spec section 7.4). While active,
// the caller must output zero and feed every compensated wrench through
// update(). After the window, deadband = max residual norm + margin.
class AdaptiveDeadband {
 public:
  void start(double window_s, double force_margin_n, double torque_margin_nm);
  bool update(const Wrench& compensated, double dt);  // true while ramping
  bool active() const { return active_; }
  double forceDeadband() const { return force_deadband_; }
  double torqueDeadband() const { return torque_deadband_; }

 private:
  bool active_{false};
  double remaining_s_{0};
  double force_margin_{0}, torque_margin_{0};
  double max_force_{0}, max_torque_{0};
  double force_deadband_{0}, torque_deadband_{0};
};

}  // namespace sfc
```

`src/adaptive_deadband.cpp`:

```cpp
#include "soft_force_control_core/adaptive_deadband.h"

#include <algorithm>

namespace sfc {

void AdaptiveDeadband::start(double window_s, double force_margin_n,
                             double torque_margin_nm) {
  active_ = true;
  remaining_s_ = window_s;
  force_margin_ = force_margin_n;
  torque_margin_ = torque_margin_nm;
  max_force_ = 0;
  max_torque_ = 0;
}

bool AdaptiveDeadband::update(const Wrench& w, double dt) {
  if (!active_) return false;
  max_force_ = std::max(max_force_, w.forceNorm());
  max_torque_ = std::max(max_torque_, w.torqueNorm());
  remaining_s_ -= dt;
  if (remaining_s_ <= 0) {
    active_ = false;
    force_deadband_ = max_force_ + force_margin_;
    torque_deadband_ = max_torque_ + torque_margin_;
  }
  return active_;
}

}  // namespace sfc
```

`include/soft_force_control_core/auto_retare.h`:

```cpp
#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

struct AutoReTareParams {
  bool enabled{false};
  double orientation_tol_deg{1.0};
};

// Return-to-start re-tare predicate (legacy isReset, spec section 7.5).
// The caller absorbs the residual via ToolGravityCompensator::absorbResidual.
class AutoReTare {
 public:
  void setReference(double a, double b, double c);
  bool shouldTare(const CartesianState& s, const Wrench& compensated,
                  double force_deadband, double torque_deadband,
                  const AutoReTareParams& p) const;

 private:
  double ref_a_{0}, ref_b_{0}, ref_c_{0};
};

}  // namespace sfc
```

`src/auto_retare.cpp`:

```cpp
#include "soft_force_control_core/auto_retare.h"

#include "soft_force_control_core/rotation.h"

namespace sfc {

void AutoReTare::setReference(double a, double b, double c) {
  ref_a_ = a;
  ref_b_ = b;
  ref_c_ = c;
}

bool AutoReTare::shouldTare(const CartesianState& s, const Wrench& w,
                            double force_deadband, double torque_deadband,
                            const AutoReTareParams& p) const {
  if (!p.enabled) return false;
  if (angularDistanceDeg(ref_a_, ref_b_, ref_c_, s.a, s.b, s.c) >
      p.orientation_tol_deg) {
    return false;
  }
  return w.forceNorm() < force_deadband && w.torqueNorm() < torque_deadband;
}

}  // namespace sfc
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_force_control_core/test_adaptive_deadband && \
  ./devel/lib/soft_force_control_core/test_auto_retare
```

Expected: `[  PASSED  ] 4 tests.` for both binaries.

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_force_control_core && \
git commit -m "feat(core): add AdaptiveDeadband ramp and AutoReTare predicate"
```

---

### Task 8: OrientationMotionCore (legacy RotFixAng goal-seeking)

**Files:**
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/orientation_motion_core.h`
- Create: `ros_ws/src/soft_force_control_core/src/orientation_motion_core.cpp`
- Modify: `ros_ws/src/soft_force_control_core/CMakeLists.txt`
- Test: `ros_ws/src/soft_force_control_core/test/test_orientation_motion.cpp`

**Interfaces:**
- Consumes: `sfc::CartesianState`, `sfc::CartesianCorrection` (Task 1), `sfc::wrapDeg` (Task 3).
- Produces:
  - `sfc::MotionGoal { double a, b, c; double max_speed_dps{5.0}; double p_gain{1.0}; double tol_deg{0.1}; double hold_s{0.2}; double timeout_s{30.0}; }`
  - `enum class sfc::MotionStatus { INACTIVE, RUNNING, CONVERGED, TIMEOUT }`
  - `sfc::OrientationMotionCore` with `void setGoal(const MotionGoal&)`, `void cancel()`, `MotionStatus status() const`, `CartesianCorrection update(const CartesianState& current, double dt)`.
- Law: per axis `err = wrapDeg(goal - current)`; `v = clamp(p_gain * err, ±max_speed)`; correction = `v * dt` on a/b/c only (x/y/z stay 0). P-with-clamp reproduces the legacy `SpeedR` trapezoid: constant max speed far away, proportional deceleration near the goal. Converged when all `|err| < tol` held for `hold_s`; `timeout_s` exceeded → TIMEOUT and zero output.

- [ ] **Step 1: Write the failing test**

`test/test_orientation_motion.cpp`:

```cpp
#include <gtest/gtest.h>
#include "soft_force_control_core/orientation_motion_core.h"

using sfc::CartesianCorrection;
using sfc::CartesianState;
using sfc::MotionGoal;
using sfc::MotionStatus;
using sfc::OrientationMotionCore;

namespace {
constexpr double kDt = 0.004;
CartesianState pose(double a, double b, double c) {
  CartesianState s;
  s.a = a;
  s.b = b;
  s.c = c;
  return s;
}
MotionGoal goalTo(double a, double b, double c) {
  MotionGoal g;
  g.a = a;
  g.b = b;
  g.c = c;
  g.max_speed_dps = 10.0;
  g.p_gain = 2.0;
  g.tol_deg = 0.05;
  g.hold_s = 0.02;
  g.timeout_s = 60.0;
  return g;
}
}  // namespace

TEST(OrientationMotion, InactiveOutputsZero) {
  OrientationMotionCore m;
  CartesianCorrection c = m.update(pose(0, 0, 0), kDt);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
  EXPECT_EQ(m.status(), MotionStatus::INACTIVE);
}

TEST(OrientationMotion, MovesTowardGoalWithClampedSpeed) {
  OrientationMotionCore m;
  m.setGoal(goalTo(90, 0, 0));
  CartesianCorrection c = m.update(pose(0, 0, 0), kDt);
  // err=90, p*err=180 dps -> clamped to 10 dps -> 0.04 deg per cycle
  EXPECT_NEAR(c.a, 10.0 * kDt, 1e-12);
  EXPECT_DOUBLE_EQ(c.x, 0.0);
  EXPECT_EQ(m.status(), MotionStatus::RUNNING);
}

TEST(OrientationMotion, TakesShortestPathAcrossWrap) {
  OrientationMotionCore m;
  m.setGoal(goalTo(-170, 0, 0));
  CartesianCorrection c = m.update(pose(170, 0, 0), kDt);
  EXPECT_GT(c.a, 0.0);  // +20 deg is shorter than -340
}

TEST(OrientationMotion, ConvergesAfterHoldTime) {
  OrientationMotionCore m;
  MotionGoal g = goalTo(1.0, 0, 0);
  m.setGoal(g);
  // Simulate the plant: integrate corrections onto the pose.
  CartesianState s = pose(0, 0, 0);
  MotionStatus st = MotionStatus::RUNNING;
  for (int i = 0; i < 20000 && st == MotionStatus::RUNNING; ++i) {
    CartesianCorrection c = m.update(s, kDt);
    s.a += c.a;
    st = m.status();
  }
  EXPECT_EQ(st, MotionStatus::CONVERGED);
  EXPECT_NEAR(s.a, 1.0, 0.1);
  // After convergence output is zero.
  CartesianCorrection c = m.update(s, kDt);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
}

TEST(OrientationMotion, TimesOutWhenStuck) {
  OrientationMotionCore m;
  MotionGoal g = goalTo(90, 0, 0);
  g.timeout_s = 0.02;  // 5 cycles
  m.setGoal(g);
  for (int i = 0; i < 6; ++i) m.update(pose(0, 0, 0), kDt);  // plant frozen
  EXPECT_EQ(m.status(), MotionStatus::TIMEOUT);
  CartesianCorrection c = m.update(pose(0, 0, 0), kDt);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
}

TEST(OrientationMotion, CancelStopsMotion) {
  OrientationMotionCore m;
  m.setGoal(goalTo(90, 0, 0));
  m.update(pose(0, 0, 0), kDt);
  m.cancel();
  EXPECT_EQ(m.status(), MotionStatus::INACTIVE);
  CartesianCorrection c = m.update(pose(0, 0, 0), kDt);
  EXPECT_DOUBLE_EQ(c.a, 0.0);
}
```

- [ ] **Step 2: Add build entries and run to verify it fails**

`CMakeLists.txt`: add `src/orientation_motion_core.cpp` to `add_library`; add test target:

```cmake
  catkin_add_gtest(test_orientation_motion test/test_orientation_motion.cpp)
  target_link_libraries(test_orientation_motion ${PROJECT_NAME})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
Expected: BUILD FAILS with `orientation_motion_core.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

`include/soft_force_control_core/orientation_motion_core.h`:

```cpp
#pragma once
#include "soft_force_control_core/types.h"

namespace sfc {

struct MotionGoal {
  double a{0}, b{0}, c{0};      // target orientation [deg]
  double max_speed_dps{5.0};    // per-axis speed clamp [deg/s]
  double p_gain{1.0};           // proportional gain [1/s]
  double tol_deg{0.1};          // convergence tolerance per axis
  double hold_s{0.2};           // time inside tolerance before CONVERGED
  double timeout_s{30.0};       // abort deadline
};

enum class MotionStatus { INACTIVE, RUNNING, CONVERGED, TIMEOUT };

// Goal-seeking orientation correction generator (legacy RotFixAng,
// spec section 7.6). P-with-clamp speed shaping; outputs a/b/c corrections
// per cycle, zero after convergence, timeout, or cancel.
class OrientationMotionCore {
 public:
  void setGoal(const MotionGoal& g);
  void cancel() { status_ = MotionStatus::INACTIVE; }
  MotionStatus status() const { return status_; }
  CartesianCorrection update(const CartesianState& current, double dt);

 private:
  MotionGoal goal_;
  MotionStatus status_{MotionStatus::INACTIVE};
  double elapsed_{0}, held_{0};
};

}  // namespace sfc
```

`src/orientation_motion_core.cpp`:

```cpp
#include "soft_force_control_core/orientation_motion_core.h"

#include <algorithm>
#include <cmath>

#include "soft_force_control_core/rotation.h"

namespace sfc {

void OrientationMotionCore::setGoal(const MotionGoal& g) {
  goal_ = g;
  status_ = MotionStatus::RUNNING;
  elapsed_ = 0;
  held_ = 0;
}

CartesianCorrection OrientationMotionCore::update(const CartesianState& s,
                                                  double dt) {
  CartesianCorrection out;
  if (status_ != MotionStatus::RUNNING) return out;

  elapsed_ += dt;
  if (elapsed_ > goal_.timeout_s) {
    status_ = MotionStatus::TIMEOUT;
    return out;
  }

  const double err[3] = {wrapDeg(goal_.a - s.a), wrapDeg(goal_.b - s.b),
                         wrapDeg(goal_.c - s.c)};

  const bool in_tol = std::fabs(err[0]) < goal_.tol_deg &&
                      std::fabs(err[1]) < goal_.tol_deg &&
                      std::fabs(err[2]) < goal_.tol_deg;
  if (in_tol) {
    held_ += dt;
    if (held_ >= goal_.hold_s) {
      status_ = MotionStatus::CONVERGED;
      return out;
    }
  } else {
    held_ = 0;
  }

  auto shape = [this, dt](double e) {
    const double v = std::max(-goal_.max_speed_dps,
                              std::min(goal_.max_speed_dps, goal_.p_gain * e));
    return v * dt;
  };
  out.a = shape(err[0]);
  out.b = shape(err[1]);
  out.c = shape(err[2]);
  return out;
}

}  // namespace sfc
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_force_control_core/test_orientation_motion
```

Expected: `[  PASSED  ] 6 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_force_control_core && \
git commit -m "feat(core): add OrientationMotionCore goal-seeking generator"
```

---

### Task 9: PayloadEstimator (legacy MassCul least-squares fit)

**Files:**
- Create: `ros_ws/src/soft_force_control_core/include/soft_force_control_core/payload_estimator.h`
- Create: `ros_ws/src/soft_force_control_core/src/payload_estimator.cpp`
- Modify: `ros_ws/src/soft_force_control_core/CMakeLists.txt`
- Test: `ros_ws/src/soft_force_control_core/test/test_payload_estimator.cpp`

**Interfaces:**
- Consumes: `sfc::Wrench` (Task 1), `sfc::kukaAbcToRotation` (Task 3), `sfc::PayloadParams` (Task 4).
- Produces:
  - `sfc::PayloadFitResult { bool ok; PayloadParams params; double r2_force; double r2_torque; }`
  - `sfc::PayloadEstimator` with `void addSample(double a_deg, double b_deg, double c_deg, const Wrench& averaged)`, `size_t sampleCount() const`, `void clear()`, `PayloadFitResult solve() const`.
- Math: with `r3_i = third row of R_i` (gravity direction in sensor frame), force model `F_i = -G * r3_i + F0` → linear LS for `[G, F0]`. Torque model `T_i = com x (F_i - F0) + T0` → `T_i = -skew(f_i) * com + T0`, linear LS for `[com, T0]`. `ok` requires ≥ 4 samples and finite solution. `r2_*` are coefficient-of-determination values (legacy `R1*R2` quality index equivalent).

- [ ] **Step 1: Write the failing test**

`test/test_payload_estimator.cpp`:

```cpp
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "soft_force_control_core/payload_estimator.h"
#include "soft_force_control_core/rotation.h"

using sfc::PayloadEstimator;
using sfc::PayloadFitResult;
using sfc::Wrench;

namespace {
// Synthesize the exact wrench a sensor would read for payload (G, com, bias)
// at orientation (a, b, c): raw = bias + gravity terms.
Wrench synth(double G, const Eigen::Vector3d& com, const Wrench& bias,
             double a, double b, double c) {
  const Eigen::Matrix3d r = sfc::kukaAbcToRotation(a, b, c);
  const Eigen::Vector3d f = r.transpose() * Eigen::Vector3d(0, 0, -G);
  const Eigen::Vector3d t = com.cross(f);
  Wrench w;
  w.fx = bias.fx + f.x();
  w.fy = bias.fy + f.y();
  w.fz = bias.fz + f.z();
  w.tx = bias.tx + t.x();
  w.ty = bias.ty + t.y();
  w.tz = bias.tz + t.z();
  return w;
}

// Legacy-style 8-orientation calibration set (varied A/B/C).
const double kPoses[8][3] = {{0, 0, 0},    {0, 45, 0},  {0, -45, 0},
                             {0, 0, 45},   {0, 0, -45}, {45, 30, 0},
                             {-45, 0, 30}, {30, -30, 30}};
}  // namespace

TEST(PayloadEstimator, NotOkWithTooFewSamples) {
  PayloadEstimator e;
  e.addSample(0, 0, 0, Wrench{});
  e.addSample(0, 45, 0, Wrench{});
  EXPECT_FALSE(e.solve().ok);
}

TEST(PayloadEstimator, RecoversSyntheticPayloadExactly) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.02, 0.03);
  Wrench bias;
  bias.fx = 1.0;
  bias.fy = -2.0;
  bias.fz = 0.5;
  bias.tx = 0.1;
  bias.ty = -0.2;
  bias.tz = 0.05;

  PayloadEstimator e;
  for (const auto& p : kPoses) {
    e.addSample(p[0], p[1], p[2], synth(G, com, bias, p[0], p[1], p[2]));
  }
  PayloadFitResult r = e.solve();
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.params.gravity_n, G, 1e-8);
  EXPECT_NEAR(r.params.com_x, com.x(), 1e-8);
  EXPECT_NEAR(r.params.com_y, com.y(), 1e-8);
  EXPECT_NEAR(r.params.com_z, com.z(), 1e-8);
  EXPECT_NEAR(r.params.bias.fx, bias.fx, 1e-8);
  EXPECT_NEAR(r.params.bias.tz, bias.tz, 1e-8);
  EXPECT_NEAR(r.r2_force, 1.0, 1e-9);
  EXPECT_NEAR(r.r2_torque, 1.0, 1e-9);
}

TEST(PayloadEstimator, NoisyDataReducesR2ButStillFits) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.0, 0.05);
  PayloadEstimator e;
  int sign = 1;
  for (const auto& p : kPoses) {
    Wrench w = synth(G, com, Wrench{}, p[0], p[1], p[2]);
    w.fx += 0.5 * sign;  // deterministic "noise"
    w.tx += 0.01 * sign;
    sign = -sign;
    e.addSample(p[0], p[1], p[2], w);
  }
  PayloadFitResult r = e.solve();
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.params.gravity_n, G, 1.0);
  EXPECT_LT(r.r2_force, 1.0);
  EXPECT_GT(r.r2_force, 0.99);
}

TEST(PayloadEstimator, ClearResetsSamples) {
  PayloadEstimator e;
  for (const auto& p : kPoses) e.addSample(p[0], p[1], p[2], Wrench{});
  EXPECT_EQ(e.sampleCount(), 8u);
  e.clear();
  EXPECT_EQ(e.sampleCount(), 0u);
  EXPECT_FALSE(e.solve().ok);
}
```

- [ ] **Step 2: Add build entries and run to verify it fails**

`CMakeLists.txt`: add `src/payload_estimator.cpp` to `add_library`; add test target:

```cmake
  catkin_add_gtest(test_payload_estimator test/test_payload_estimator.cpp)
  target_link_libraries(test_payload_estimator ${PROJECT_NAME})
```

Run: `cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests`
Expected: BUILD FAILS with `payload_estimator.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

`include/soft_force_control_core/payload_estimator.h`:

```cpp
#pragma once
#include <cstddef>
#include <vector>

#include "soft_force_control_core/tool_gravity_compensator.h"
#include "soft_force_control_core/types.h"

namespace sfc {

struct PayloadFitResult {
  bool ok{false};
  PayloadParams params;
  double r2_force{0};   // coefficient of determination, force fit
  double r2_torque{0};  // coefficient of determination, torque fit
};

// Least-squares payload identification from averaged wrench samples at
// varied orientations (legacy MassCul, spec section 9). Non-realtime.
class PayloadEstimator {
 public:
  void addSample(double a_deg, double b_deg, double c_deg,
                 const Wrench& averaged);
  std::size_t sampleCount() const { return samples_.size(); }
  void clear() { samples_.clear(); }
  PayloadFitResult solve() const;

 private:
  struct Sample {
    double a, b, c;
    Wrench w;
  };
  std::vector<Sample> samples_;
};

}  // namespace sfc
```

`src/payload_estimator.cpp`:

```cpp
#include "soft_force_control_core/payload_estimator.h"

#include <Eigen/Dense>
#include <cmath>

#include "soft_force_control_core/rotation.h"

namespace sfc {

namespace {
// R^2 = 1 - SS_res / SS_tot over all residual components.
double rSquared(const Eigen::VectorXd& y, const Eigen::VectorXd& y_fit) {
  const double ss_res = (y - y_fit).squaredNorm();
  const double ss_tot = (y.array() - y.mean()).matrix().squaredNorm();
  if (ss_tot <= 0) return ss_res <= 1e-12 ? 1.0 : 0.0;
  return 1.0 - ss_res / ss_tot;
}
}  // namespace

void PayloadEstimator::addSample(double a_deg, double b_deg, double c_deg,
                                 const Wrench& averaged) {
  samples_.push_back({a_deg, b_deg, c_deg, averaged});
}

PayloadFitResult PayloadEstimator::solve() const {
  PayloadFitResult res;
  const std::size_t n = samples_.size();
  if (n < 4) return res;

  // Force fit: F_i = -G * r3_i + F0, unknowns x = [G, F0x, F0y, F0z].
  Eigen::MatrixXd af(3 * n, 4);
  Eigen::VectorXd bf(3 * n);
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Matrix3d r =
        kukaAbcToRotation(samples_[i].a, samples_[i].b, samples_[i].c);
    const Eigen::Vector3d r3 = r.row(2).transpose();  // R^T * e_z
    af.block<3, 1>(3 * i, 0) = -r3;
    af.block<3, 3>(3 * i, 1) = Eigen::Matrix3d::Identity();
    bf.segment<3>(3 * i) = Eigen::Vector3d(samples_[i].w.fx, samples_[i].w.fy,
                                           samples_[i].w.fz);
  }
  const Eigen::VectorXd xf = af.colPivHouseholderQr().solve(bf);
  if (!xf.allFinite()) return res;

  // Torque fit: T_i = -skew(f_i) * com + T0, f_i = F_meas_i - F0.
  Eigen::MatrixXd at(3 * n, 6);
  Eigen::VectorXd bt(3 * n);
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d f(samples_[i].w.fx - xf(1), samples_[i].w.fy - xf(2),
                            samples_[i].w.fz - xf(3));
    Eigen::Matrix3d skew;
    skew << 0, -f.z(), f.y(), f.z(), 0, -f.x(), -f.y(), f.x(), 0;
    at.block<3, 3>(3 * i, 0) = -skew;
    at.block<3, 3>(3 * i, 3) = Eigen::Matrix3d::Identity();
    bt.segment<3>(3 * i) = Eigen::Vector3d(samples_[i].w.tx, samples_[i].w.ty,
                                           samples_[i].w.tz);
  }
  const Eigen::VectorXd xt = at.colPivHouseholderQr().solve(bt);
  if (!xt.allFinite()) return res;

  res.ok = true;
  res.params.gravity_n = xf(0);
  res.params.bias.fx = xf(1);
  res.params.bias.fy = xf(2);
  res.params.bias.fz = xf(3);
  res.params.com_x = xt(0);
  res.params.com_y = xt(1);
  res.params.com_z = xt(2);
  res.params.bias.tx = xt(3);
  res.params.bias.ty = xt(4);
  res.params.bias.tz = xt(5);
  res.r2_force = rSquared(bf, af * xf);
  res.r2_torque = rSquared(bt, at * xt);
  return res;
}

}  // namespace sfc
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make tests && \
  ./devel/lib/soft_force_control_core/test_payload_estimator
```

Expected: `[  PASSED  ] 4 tests.`

- [ ] **Step 5: Run the full suite and commit**

```bash
cd /home/ljj/kuka_iiqka_ros/ros_ws && catkin_make run_tests && \
  catkin_test_results build/test_results
```

Expected: `Summary: ... 0 errors, 0 failures`.

```bash
cd /home/ljj/kuka_iiqka_ros && \
git add ros_ws/src/soft_force_control_core && \
git commit -m "feat(core): add PayloadEstimator least-squares fit"
```

---

## Spec Coverage Notes (self-review record)

- Spec §5.4 components: `ForceTorqueFilter` (T2), `ToolGravityCompensator` (T4), `PayloadEstimator` (T9), `ComplianceLaw` (T5), `SafetyLimiter` (T6), `ModeManagerCore` (T1), `OrientationMotionCore` (T8). Adaptive deadband + auto re-tare (spec §7.4-7.5) are T7.
- Spec §15.1 unit-test list is fully covered by T1-T9. The "XML parser/serializer" item in §15.1 belongs to `kuka_rsi_hw_interface` (Plan 2), not this package.
- Realtime constraints (spec §7.1): all compute methods here are allocation-free except `PayloadEstimator` (explicitly non-realtime, manager-side) — matches the spec.
- The `FLim/TLim` deadband interpretation note (spec §7.2) is encoded in the deadzone semantics of T5 and the adaptive initialization of T7; commissioning verification stays in Plan 5's scope.
