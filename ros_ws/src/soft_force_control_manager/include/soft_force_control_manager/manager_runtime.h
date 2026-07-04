#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "soft_force_control_manager/calibration_sequencer.h"
#include "soft_force_control_manager/payload_yaml.h"
#include "soft_force_control_manager/system_state_core.h"

namespace sfm {

struct ToolFrame {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};  // mm / deg
  bool valid{false};
};

// One /kuka/eki/state message, reduced to what the manager consumes.
struct EkiFeed {
  bool connected{false};
  bool state_fresh{false};
  bool program_ready{false};
  bool rsi_active{false};
  bool fault{false};
};

// Every side effect the manager performs on the world, injected so the
// runtime is closed-loop testable with lambda mocks (Plan 4 runtime
// pattern). All callbacks are optional: a null hook no-ops (bool hooks
// report false). The node shell wires these to the real ROS clients.
struct ManagerOps {
  std::function<bool()> ekiStartRsi;
  std::function<bool()> ekiStopRsi;
  std::function<bool()> ekiResetFault;
  std::function<bool()> rsiResetFault;
  std::function<bool()> sriZero;
  std::function<ToolFrame()> ekiGetTool;
  std::function<bool(const std::string& start_ctrl,
                     const std::string& stop_ctrl)> switchControllers;
  // Fills `running` with the names of the currently running controllers
  // (Task 8b). Queried right before every switch so a STRICT request
  // never carries no-op entries: controller_manager refuses to stop a
  // controller that is not running (and to start one that already is).
  // Returns false when the query itself failed; the switch is then
  // refused (fail-closed, mirroring the retired bringup proxy). A null
  // hook keeps the legacy unfiltered passthrough.
  std::function<bool(std::vector<std::string>& running)>
      listRunningControllers;
  std::function<void(std::uint8_t mode, std::uint8_t profile)> publishMode;
  std::function<void(const CalPose& target)> sendMotionGoal;
  std::function<bool(const std::string& yaml_text,
                     const sfc::PayloadFitResult& fit)> applyPayload;
  std::function<void()> publishState;  // worker-tick hook (decision 13)
};

struct ManagerConfig {
  double tick_period_s{0.1};
  double eki_state_timeout_s{5.0};  // decision 3: tolerates the eki node's
                                    // single-threaded-spin publish gaps (N3)
  double rsi_state_timeout_s{0.5};
  double sri_status_timeout_s{2.0};
  double tool_sync_retry_s{2.0};
  double rsi_connect_wait_s{5.0};   // decision 4 bounded start wait
  std::string compliance_controller{"force_compliance_controller"};
  std::string correction_controller{"cartesian_correction_controller"};
  CalibrationConfig calibration;
};

struct CommandResult {
  bool success{false};
  std::string message;
};

struct ManagerSnapshot {
  SystemState state{SystemState::OFFLINE};
  HealthInputs health;
  std::uint8_t mode{0}, profile{0};
  std::string active_controller;
  bool calibrating{false};
  CalStatus cal;
  sfc::PayloadFitResult fit;
};

// Owns the worker thread (health -> state machine, edge-triggered tool
// sync, calibration sequencing, finish handling) and the blocking command
// methods used by the service layer. Commands serialize on command_mutex_;
// mutable state hides behind mutex_; ops callbacks always run OUTSIDE
// mutex_ (they may block on ROS service calls). Not ROS-dependent.
class ManagerRuntime {
 public:
  ManagerRuntime(const ManagerConfig& cfg, ManagerOps ops);
  ~ManagerRuntime();
  ManagerRuntime(const ManagerRuntime&) = delete;
  ManagerRuntime& operator=(const ManagerRuntime&) = delete;

  bool start();
  void stop();

  void setControllersLoaded(bool loaded);
  void feedEkiState(const EkiFeed& f);
  void feedRsiState(bool connected, bool fault);
  void feedSriStatus(bool streaming);
  void feedWrench(const sfc::Wrench& w);
  void onMotionResult(bool success);  // action-client done callback

  CommandResult startServo(std::uint8_t mode, std::uint8_t profile);
  CommandResult stopServo();
  CommandResult resetFault();
  CommandResult zeroSensor();
  CommandResult beginCalibration();
  void cancelCalibration();

  ManagerSnapshot snapshot() const;

 private:
  void run();
  HealthInputs healthLocked(double now_s) const;
  // Issues a controller switch with no-op entries removed (Task 8b): the
  // start entry is dropped when already running, the stop entry when not
  // running, and a fully emptied request succeeds without any call. See
  // ManagerOps::listRunningControllers for the null-hook/failure rules.
  bool switchFiltered(const std::string& start, const std::string& stop);
  static double nowS();

  ManagerConfig cfg_;
  ManagerOps ops_;
  std::thread thread_;
  std::atomic<bool> running_{false};

  std::mutex command_mutex_;  // serializes the command methods

  mutable std::mutex mutex_;  // guards everything below
  SystemStateCore core_;
  CalibrationSequencer seq_;
  EkiFeed eki_;
  double eki_rx_s_{-1.0};
  bool rsi_connected_{false};
  bool rsi_fault_{false};
  double rsi_rx_s_{-1.0};
  bool sri_streaming_{false};
  double sri_rx_s_{-1.0};
  bool controllers_loaded_{false};
  bool tool_synced_{false};
  double tool_last_try_s_{-1e9};
  std::uint8_t mode_{0}, profile_{0};
  std::string active_controller_;
  bool cal_teardown_done_{true};  // finish handling latch (edge trigger)
  const char* rollback_reason_{""};  // I-2: verdict reason across unlock
};

}  // namespace sfm
