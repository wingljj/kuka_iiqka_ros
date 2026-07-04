#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <chrono>
#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "soft_force_control_core/rotation.h"
#include "soft_force_control_manager/manager_runtime.h"

using sfm::CalPose;
using sfm::CommandResult;
using sfm::EkiFeed;
using sfm::ManagerConfig;
using sfm::ManagerOps;
using sfm::ManagerRuntime;
using sfm::SystemState;
using sfm::ToolFrame;
using sfc::Wrench;

namespace {

bool waitFor(const std::function<bool()>& pred, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

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

const double kPoses[8][3] = {{0, 0, 0},    {0, 45, 0},  {0, -45, 0},
                             {0, 0, 45},   {0, 0, -45}, {45, 30, 0},
                             {-45, 0, 30}, {30, -30, 30}};

// Records every ops call (thread-safe) and lets tests script results.
struct MockOps {
  std::mutex m;
  std::vector<std::string> log;
  int get_tool_calls = 0;
  int get_tool_fail_first = 0;   // fail this many getTool calls
  bool start_rsi_ok = true;
  bool switch_ok = true;
  // Task 8b STRICT emulation: switchControllers fails on no-op entries
  // (stop of a non-running / start of a running controller), exactly as
  // the real controller_manager does under STRICT. `running` mirrors the
  // controller_manager's running set; `provide_list` wires the
  // listRunningControllers hook; `list_ok` scripts a query failure.
  bool strict = false;
  bool provide_list = false;
  bool list_ok = true;
  std::set<std::string> running;
  CalPose last_goal;
  int goals_sent = 0;
  std::string payload_yaml;
  sfc::PayloadFitResult payload_fit;
  int payload_applied = 0;
  ManagerRuntime* rt = nullptr;   // for feeding back from ekiStartRsi

  void push(const std::string& s) {
    std::lock_guard<std::mutex> lock(m);
    log.push_back(s);
  }
  ManagerOps ops() {
    ManagerOps o;
    o.ekiStartRsi = [this] {
      push("start_rsi");
      if (start_rsi_ok && rt) {
        rt->feedRsiState(true, false);
        EkiFeed f;
        f.connected = true;
        f.state_fresh = true;
        f.program_ready = true;
        f.rsi_active = true;
        rt->feedEkiState(f);
      }
      return start_rsi_ok;
    };
    o.ekiStopRsi = [this] { push("stop_rsi"); return true; };
    o.ekiResetFault = [this] { push("eki_reset"); return true; };
    o.rsiResetFault = [this] {
      push("rsi_reset");
      if (rt) rt->feedRsiState(false, false);  // hw fault unlatched
      return true;
    };
    o.sriZero = [this] { push("sri_zero"); return true; };
    o.ekiGetTool = [this]() -> ToolFrame {
      ToolFrame t;
      std::lock_guard<std::mutex> lock(m);
      ++get_tool_calls;
      if (get_tool_fail_first > 0) { --get_tool_fail_first; return t; }
      t.x = 10.5; t.z = 235.0; t.valid = true;
      return t;
    };
    o.switchControllers = [this](const std::string& start,
                                 const std::string& stop) {
      push("switch:" + start + "/" + stop);
      std::lock_guard<std::mutex> lock(m);
      if (!switch_ok) return false;
      if (strict) {  // controller_manager STRICT: no-op entries are errors
        if (!stop.empty() && running.count(stop) == 0) return false;
        if (!start.empty() && running.count(start) != 0) return false;
      }
      if (!stop.empty()) running.erase(stop);
      if (!start.empty()) running.insert(start);
      return true;
    };
    o.publishMode = [this](std::uint8_t mode, std::uint8_t profile) {
      push("mode:" + std::to_string(mode) + "/" + std::to_string(profile));
    };
    o.sendMotionGoal = [this](const CalPose& p) {
      std::lock_guard<std::mutex> lock(m);
      last_goal = p;
      ++goals_sent;
    };
    o.applyPayload = [this](const std::string& yaml,
                            const sfc::PayloadFitResult& fit) {
      std::lock_guard<std::mutex> lock(m);
      payload_yaml = yaml;
      payload_fit = fit;
      ++payload_applied;
      return true;
    };
    if (provide_list) {
      o.listRunningControllers = [this](std::vector<std::string>& out) {
        std::lock_guard<std::mutex> lock(m);
        if (!list_ok) return false;
        out.assign(running.begin(), running.end());
        return true;
      };
    }
    return o;
  }
  bool switchLogged() {
    std::lock_guard<std::mutex> lock(m);
    for (const auto& e : log)
      if (e.rfind("switch:", 0) == 0) return true;
    return false;
  }
  int goalsSent() { std::lock_guard<std::mutex> l(m); return goals_sent; }
  CalPose lastGoal() { std::lock_guard<std::mutex> l(m); return last_goal; }
  bool logged(const std::string& s) {
    std::lock_guard<std::mutex> lock(m);
    for (const auto& e : log) if (e == s) return true;
    return false;
  }
};

ManagerConfig fastConfig() {
  ManagerConfig c;
  c.tick_period_s = 0.01;
  c.eki_state_timeout_s = 5.0;
  c.rsi_state_timeout_s = 5.0;    // freshness not under test unless shrunk
  c.sri_status_timeout_s = 5.0;
  c.tool_sync_retry_s = 0.05;
  c.rsi_connect_wait_s = 0.5;
  for (const auto& p : kPoses)
    c.calibration.poses.push_back(CalPose{p[0], p[1], p[2]});
  c.calibration.settle_time_s = 0.0;
  c.calibration.samples_per_pose = 1;
  return c;
}

EkiFeed ekiUp() {
  EkiFeed f;
  f.connected = true;
  f.state_fresh = true;
  f.program_ready = true;
  return f;
}

// Brings a started runtime to READY (tool sync happens on the way).
void driveToReady(ManagerRuntime& rt) {
  rt.setControllersLoaded(true);
  rt.feedEkiState(ekiUp());
  rt.feedSriStatus(true);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
}

void driveToServoing(ManagerRuntime& rt, MockOps& mock) {
  driveToReady(rt);
  const CommandResult r = rt.startServo(2, 0);  // FORCE_COMPLIANCE + DRAG
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));
  (void)mock;
}

}  // namespace

TEST(ManagerRuntime, IdleLadderReachesReadyAndSyncsToolOnce) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  EXPECT_EQ(rt.snapshot().state, SystemState::OFFLINE);

  driveToReady(rt);
  EXPECT_TRUE(rt.snapshot().health.tool_synced);
  // Synced: no further getTool retries.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::lock_guard<std::mutex> lock(mock.m);
  EXPECT_EQ(mock.get_tool_calls, 1);
  rt.stop();
}

TEST(ManagerRuntime, ToolSyncRetriesUntilSuccess) {
  MockOps mock;
  mock.get_tool_fail_first = 2;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);  // READY implies tool_synced eventually true
  std::lock_guard<std::mutex> lock(mock.m);
  EXPECT_EQ(mock.get_tool_calls, 3);  // 2 failures + 1 success
  rt.stop();
}

TEST(ManagerRuntime, StartServoRunsOrchestrationInOrder) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToServoing(rt, mock);

  // Exact command sequence (decision 4).
  std::vector<std::string> cmd;
  {
    std::lock_guard<std::mutex> lock(mock.m);
    for (const auto& e : mock.log)
      if (e == "start_rsi" || e.rfind("mode:", 0) == 0 ||
          e.rfind("switch:", 0) == 0)
        cmd.push_back(e);
  }
  const std::vector<std::string> expected = {
      "start_rsi", "mode:0/0",
      "switch:force_compliance_controller/cartesian_correction_controller",
      "mode:2/0"};
  EXPECT_EQ(cmd, expected);
  EXPECT_EQ(rt.snapshot().active_controller, "force_compliance_controller");
  EXPECT_EQ(rt.snapshot().mode, 2u);
  rt.stop();
}

TEST(ManagerRuntime, StartServoFailsWhenRsiNeverConnects) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  // mock.rt left null: ekiStartRsi succeeds but no RSI feed ever arrives.
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  const CommandResult r = rt.startServo(2, 0);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(rt.snapshot().state, SystemState::READY);  // stays READY
  EXPECT_FALSE(mock.logged(
      "switch:force_compliance_controller/cartesian_correction_controller"));
  rt.stop();
}

TEST(ManagerRuntime, StartServoRejectsBadModeAndNotReady) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  EXPECT_FALSE(rt.startServo(2, 0).success);   // OFFLINE
  driveToReady(rt);
  EXPECT_FALSE(rt.startServo(0, 0).success);   // IDLE is not a servo mode
  EXPECT_FALSE(rt.startServo(3, 0).success);   // CALIBRATION not via start
  rt.stop();
}

TEST(ManagerRuntime, StopServoTearsDownInOrder) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToServoing(rt, mock);
  {
    std::lock_guard<std::mutex> lock(mock.m);
    mock.log.clear();
  }
  const CommandResult r = rt.stopServo();
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  std::lock_guard<std::mutex> lock(mock.m);
  const std::vector<std::string> expected = {
      "mode:0/0", "switch:/force_compliance_controller", "stop_rsi"};
  EXPECT_EQ(mock.log, expected);
  rt.stop();
}

TEST(ManagerRuntime, SriLossDegradesAndRecovers) {
  MockOps mock;
  ManagerConfig cfg = fastConfig();
  cfg.sri_status_timeout_s = 0.1;
  ManagerRuntime rt{cfg, mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  rt.setControllersLoaded(true);
  rt.feedEkiState(ekiUp());
  rt.feedSriStatus(true);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  ASSERT_TRUE(rt.startServo(2, 0).success);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));

  // Stop feeding SRI: within ~0.1 s the health judgement goes stale.
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::DEGRADED; }, 2000));
  rt.feedSriStatus(true);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));
  rt.stop();
}

TEST(ManagerRuntime, FaultThenResetRunsBothResetOps) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToServoing(rt, mock);
  rt.feedRsiState(true, true);  // latched RSI fault
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::FAULT; }, 2000));
  EXPECT_FALSE(rt.zeroSensor().success);  // gate closed in FAULT

  const CommandResult r = rt.resetFault();
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_TRUE(mock.logged("rsi_reset"));
  EXPECT_TRUE(mock.logged("eki_reset"));
  rt.feedRsiState(true, false);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  rt.stop();
}

TEST(ManagerRuntime, ZeroSensorGateFollowsDecision11) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  EXPECT_TRUE(rt.zeroSensor().success);
  EXPECT_TRUE(mock.logged("sri_zero"));

  ASSERT_TRUE(rt.startServo(2, 0).success);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));
  {
    std::lock_guard<std::mutex> lock(mock.m);
    mock.log.clear();
  }
  EXPECT_FALSE(rt.zeroSensor().success);   // SERVOING: rejected
  EXPECT_FALSE(mock.logged("sri_zero"));   // ops never touched
  rt.stop();
}

TEST(ManagerRuntime, CalibrationClosedLoopPersistsAndRestoresReady) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.02, 0.03);
  Wrench bias;
  bias.fx = 1.0; bias.fy = -2.0; bias.fz = 0.5;
  bias.tx = 0.1; bias.ty = -0.2; bias.tz = 0.05;

  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  rt.feedRsiState(true, false);   // calibration needs a valid RSI link

  const CommandResult r = rt.beginCalibration();
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_TRUE(mock.logged(
      "switch:cartesian_correction_controller/force_compliance_controller"));
  EXPECT_TRUE(mock.logged("mode:3/0"));

  for (int i = 0; i < 8; ++i) {
    ASSERT_TRUE(waitFor([&] { return mock.goalsSent() == i + 1; }, 2000))
        << "pose " << i;
    const CalPose g = mock.lastGoal();
    EXPECT_DOUBLE_EQ(g.a, kPoses[i][0]);
    rt.onMotionResult(true);
    ASSERT_TRUE(waitFor(
        [&] { return rt.snapshot().cal.phase == sfm::CalPhase::SAMPLING ||
                     mock.goalsSent() > i + 1; }, 2000));
    rt.feedWrench(synth(G, com, bias, g.a, g.b, g.c));
  }
  // Return move (goal 9), then DONE -> persist + teardown.
  ASSERT_TRUE(waitFor([&] { return mock.goalsSent() == 9; }, 2000));
  EXPECT_DOUBLE_EQ(mock.lastGoal().a, 0.0);
  rt.onMotionResult(true);

  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  std::lock_guard<std::mutex> lock(mock.m);
  ASSERT_EQ(mock.payload_applied, 1);
  EXPECT_NEAR(mock.payload_fit.params.gravity_n, G, 1e-8);
  EXPECT_NE(mock.payload_yaml.find("gravity_n: 50.000000"), std::string::npos);
  EXPECT_FALSE(rt.snapshot().calibrating);
  EXPECT_TRUE(rt.snapshot().fit.ok);
  rt.stop();
}

TEST(ManagerRuntime, CalibrationMoveFailureAbortsWithoutPersisting) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  rt.feedRsiState(true, false);
  ASSERT_TRUE(rt.beginCalibration().success);
  ASSERT_TRUE(waitFor([&] { return mock.goalsSent() == 1; }, 2000));
  rt.onMotionResult(false);   // first move fails

  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  std::lock_guard<std::mutex> lock(mock.m);
  EXPECT_EQ(mock.payload_applied, 0);
  EXPECT_EQ(mock.goals_sent, 1);   // decision 6: no return goal on abort
  rt.stop();
}

TEST(ManagerRuntime, CalibrationRejectedOutsideReady) {
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  EXPECT_FALSE(rt.beginCalibration().success);   // OFFLINE
  driveToServoing(rt, mock);
  EXPECT_FALSE(rt.beginCalibration().success);   // SERVOING
  rt.stop();
}

// --- Task 8b: STRICT switch semantics (debt reclaim of the Task 8
// bringup proxy). From READY neither controller is running, and the real
// controller_manager under STRICT refuses any stop of a non-running
// controller (or start of a running one). The runtime must therefore
// query the running set and never emit no-op entries. ---

TEST(ManagerRuntime, StartServoUnderStrictOmitsNonRunningStop) {
  MockOps mock;
  mock.strict = true;
  mock.provide_list = true;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);

  const CommandResult r = rt.startServo(2, 0);
  EXPECT_TRUE(r.success) << r.message;
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));
  // The sibling is not running: it must not appear as a stop entry.
  EXPECT_TRUE(mock.logged("switch:force_compliance_controller/"));
  EXPECT_FALSE(mock.logged(
      "switch:force_compliance_controller/cartesian_correction_controller"));
  std::lock_guard<std::mutex> lock(mock.m);
  EXPECT_EQ(mock.running.count("force_compliance_controller"), 1u);
  rt.stop();
}

TEST(ManagerRuntime, CalibrationEntryUnderStrictOmitsNonRunningStop) {
  MockOps mock;
  mock.strict = true;
  mock.provide_list = true;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  rt.feedRsiState(true, false);

  const CommandResult r = rt.beginCalibration();
  EXPECT_TRUE(r.success) << r.message;
  EXPECT_TRUE(mock.logged("switch:cartesian_correction_controller/"));
  EXPECT_FALSE(mock.logged(
      "switch:cartesian_correction_controller/force_compliance_controller"));

  // Abort via a failed first move: the teardown stop of the now-RUNNING
  // correction controller must pass STRICT and restore READY.
  ASSERT_TRUE(waitFor([&] { return mock.goalsSent() == 1; }, 2000));
  rt.onMotionResult(false);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  EXPECT_TRUE(mock.logged("switch:/cartesian_correction_controller"));
  std::lock_guard<std::mutex> lock(mock.m);
  EXPECT_EQ(mock.running.count("cartesian_correction_controller"), 0u);
  rt.stop();
}

TEST(ManagerRuntime, StartServoAfterFaultResetSkipsFullNoOpSwitch) {
  // FAULT does not stop the active controller; after reset the target is
  // still running, so a restart of the same mode is a full no-op switch:
  // nothing may be forwarded to the controller_manager, and the command
  // must still succeed.
  MockOps mock;
  mock.strict = true;
  mock.provide_list = true;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToServoing(rt, mock);
  rt.feedRsiState(true, true);   // latched RSI fault
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::FAULT; }, 2000));
  ASSERT_TRUE(rt.resetFault().success);
  rt.feedRsiState(true, false);
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  {
    std::lock_guard<std::mutex> lock(mock.m);
    ASSERT_EQ(mock.running.count("force_compliance_controller"), 1u);
    mock.log.clear();
  }

  const CommandResult r = rt.startServo(2, 0);
  EXPECT_TRUE(r.success) << r.message;
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::SERVOING; }, 2000));
  EXPECT_EQ(rt.snapshot().mode, 2u);
  EXPECT_FALSE(mock.switchLogged());   // no request reached the cm
  rt.stop();
}

TEST(ManagerRuntime, StartServoFailsClosedWhenRunningQueryFails) {
  MockOps mock;
  mock.strict = true;
  mock.provide_list = true;
  mock.list_ok = false;          // running-set query itself fails
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToReady(rt);
  {
    std::lock_guard<std::mutex> lock(mock.m);
    mock.log.clear();
  }

  const CommandResult r = rt.startServo(2, 0);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(rt.snapshot().state, SystemState::READY);  // stays READY
  EXPECT_FALSE(mock.switchLogged());   // fail-closed: nothing forwarded
  rt.stop();
}

// ---- Plan 6 Task 2: I-1 stop-chain endgame (rules R2 + R3) ----

TEST(ManagerRuntime, CleanStopResidueFaultKeepsReady) {
  // After stop_servo the RSI stream dies and the hw latches fault=true
  // within 5 cycles; with a fresh heartbeat and rsi_active=false the
  // manager must treat it as session residue and stay READY (R2).
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  driveToServoing(rt, mock);
  ASSERT_TRUE(rt.stopServo().success);
  // Post-stop world: KRC ended the session, hw latched the miss fault.
  EkiFeed f = ekiUp();
  f.rsi_active = false;
  rt.feedEkiState(f);
  rt.feedRsiState(false, true);   // connected=false, fault=true (latched)
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  EXPECT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 100));
  rt.stop();
}

TEST(ManagerRuntime, StartServoPreClearsResidueFault) {
  // R3: a start over a residue-latched hw fault must call the hw reset
  // first (otherwise write() sends Stop S=1 into the new session), then
  // proceed normally.
  MockOps mock;
  ManagerRuntime rt{fastConfig(), mock.ops()};
  mock.rt = &rt;
  ASSERT_TRUE(rt.start());
  rt.setControllersLoaded(true);
  EkiFeed f = ekiUp();
  f.rsi_active = false;
  rt.feedEkiState(f);
  rt.feedSriStatus(true);
  rt.feedRsiState(false, true);   // residue: hw fault latched, stream dead
  ASSERT_TRUE(waitFor(
      [&] { return rt.snapshot().state == SystemState::READY; }, 2000));
  const CommandResult r = rt.startServo(2, 0);
  EXPECT_TRUE(r.success) << r.message;
  EXPECT_TRUE(mock.logged("rsi_reset"));
  // rsi_reset must have been requested before start_rsi.
  {
    std::lock_guard<std::mutex> lock(mock.m);
    std::size_t i_reset = mock.log.size(), i_start = mock.log.size();
    for (std::size_t i = 0; i < mock.log.size(); ++i) {
      if (mock.log[i] == "rsi_reset" && i_reset == mock.log.size())
        i_reset = i;
      if (mock.log[i] == "start_rsi" && i_start == mock.log.size())
        i_start = i;
    }
    EXPECT_LT(i_reset, i_start);
  }
  rt.stop();
}
