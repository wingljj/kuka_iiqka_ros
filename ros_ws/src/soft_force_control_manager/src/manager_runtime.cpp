#include "soft_force_control_manager/manager_runtime.h"

#include <algorithm>
#include <chrono>
#include <vector>

namespace sfm {

namespace {
constexpr std::uint8_t kModeIdle = 0;
constexpr std::uint8_t kModeDirect = 1;
constexpr std::uint8_t kModeCompliance = 2;
constexpr std::uint8_t kModeCalibration = 3;
}  // namespace

ManagerRuntime::ManagerRuntime(const ManagerConfig& cfg, ManagerOps ops)
    : cfg_(cfg), ops_(std::move(ops)) {
  seq_.configure(cfg_.calibration);
}

ManagerRuntime::~ManagerRuntime() { stop(); }

double ManagerRuntime::nowS() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool ManagerRuntime::start() {
  if (running_.load()) return false;
  running_ = true;
  thread_ = std::thread(&ManagerRuntime::run, this);
  return true;
}

void ManagerRuntime::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
}

void ManagerRuntime::setControllersLoaded(bool loaded) {
  std::lock_guard<std::mutex> lock(mutex_);
  controllers_loaded_ = loaded;
}

void ManagerRuntime::feedEkiState(const EkiFeed& f) {
  std::lock_guard<std::mutex> lock(mutex_);
  eki_ = f;
  eki_rx_s_ = nowS();
}

void ManagerRuntime::feedRsiState(bool connected, bool fault) {
  std::lock_guard<std::mutex> lock(mutex_);
  rsi_connected_ = connected;
  rsi_fault_ = fault;
  rsi_rx_s_ = nowS();
}

void ManagerRuntime::feedSriStatus(bool streaming) {
  std::lock_guard<std::mutex> lock(mutex_);
  sri_streaming_ = streaming;
  sri_rx_s_ = nowS();
}

void ManagerRuntime::feedWrench(const sfc::Wrench& w) {
  std::lock_guard<std::mutex> lock(mutex_);
  seq_.onWrench(w);  // no-op unless the sequencer is SAMPLING
}

void ManagerRuntime::onMotionResult(bool success) {
  std::lock_guard<std::mutex> lock(mutex_);
  seq_.onMotionResult(success, nowS());
}

HealthInputs ManagerRuntime::healthLocked(double now_s) const {
  HealthInputs in;
  const bool eki_fresh =
      eki_rx_s_ >= 0.0 && now_s - eki_rx_s_ <= cfg_.eki_state_timeout_s;
  in.eki_link = eki_fresh && eki_.connected && eki_.state_fresh;
  in.eki_program_ready = in.eki_link && eki_.program_ready;
  in.eki_fault = eki_fresh && eki_.fault;
  in.rsi_topic_fresh =
      rsi_rx_s_ >= 0.0 && now_s - rsi_rx_s_ <= cfg_.rsi_state_timeout_s;
  in.rsi_connected = in.rsi_topic_fresh && rsi_connected_;
  in.rsi_fault = in.rsi_topic_fresh && rsi_fault_;
  in.sri_streaming = sri_rx_s_ >= 0.0 &&
                     now_s - sri_rx_s_ <= cfg_.sri_status_timeout_s &&
                     sri_streaming_;
  in.tool_synced = tool_synced_;
  in.controllers_loaded = controllers_loaded_;
  return in;
}

// Task 8b: every controller switch goes through here so a STRICT request
// never carries no-op entries (controller_manager refuses to stop a
// non-running controller and to start a running one; from READY neither
// controller runs, which made every first start fail). Runs OUTSIDE
// mutex_ like all ops. Null query hook = legacy unfiltered passthrough;
// a failed query refuses the switch (fail-closed).
bool ManagerRuntime::switchFiltered(const std::string& start,
                                    const std::string& stop) {
  if (!ops_.switchControllers) return false;
  std::string s = start;
  std::string t = stop;
  if (ops_.listRunningControllers) {
    std::vector<std::string> running;
    if (!ops_.listRunningControllers(running)) return false;
    const auto runs = [&running](const std::string& n) {
      return std::find(running.begin(), running.end(), n) != running.end();
    };
    if (!s.empty() && runs(s)) s.clear();   // already running: no-op
    if (!t.empty() && !runs(t)) t.clear();  // not running: no-op
    if (s.empty() && t.empty()) return true;  // nothing left to do
  }
  return ops_.switchControllers(s, t);
}

void ManagerRuntime::run() {
  while (running_.load()) {
    const double now = nowS();
    bool want_tool_sync = false;
    bool cal_stream_ok = true;
    CalAction act;
    bool cal_finished = false;
    bool cal_success = false;
    sfc::PayloadFitResult fit;
    std::uint8_t cal_profile = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      HealthInputs in = healthLocked(now);
      if (!in.eki_link) tool_synced_ = false;  // resync after reconnect
      core_.update(in);

      want_tool_sync = in.eki_link && eki_.program_ready && !tool_synced_ &&
                       now - tool_last_try_s_ >= cfg_.tool_sync_retry_s &&
                       static_cast<bool>(ops_.ekiGetTool);
      if (want_tool_sync) tool_last_try_s_ = now;

      const CalStatus cs = seq_.status();
      const bool cal_active =
          cs.phase == CalPhase::MOVING || cs.phase == CalPhase::SETTLING ||
          cs.phase == CalPhase::SAMPLING || cs.phase == CalPhase::RETURNING;
      if (cal_active && !in.sri_streaming) cal_stream_ok = false;
      if (!cal_stream_ok) seq_.onStreamLost();
      act = seq_.tick(now);

      const CalStatus after = seq_.status();
      if ((after.phase == CalPhase::DONE || after.phase == CalPhase::FAILED) &&
          !cal_teardown_done_) {
        cal_teardown_done_ = true;
        cal_finished = true;
        cal_success = after.phase == CalPhase::DONE && seq_.result().ok;
        fit = seq_.result();
        cal_profile = profile_;
      }
    }

    // ops run outside mutex_ (they may block on ROS services).
    if (want_tool_sync) {
      const ToolFrame t = ops_.ekiGetTool();
      std::lock_guard<std::mutex> lock(mutex_);
      if (t.valid) tool_synced_ = true;
    }
    if (act.send_goal && ops_.sendMotionGoal) ops_.sendMotionGoal(act.target);
    if (cal_finished) {
      if (cal_success && ops_.applyPayload) {
        ops_.applyPayload(emitPayloadYaml(fit, ""), fit);
      }
      if (ops_.publishMode) ops_.publishMode(kModeIdle, cal_profile);
      switchFiltered("", cfg_.correction_controller);
      std::lock_guard<std::mutex> lock(mutex_);
      core_.calibrationFinished();
      active_controller_.clear();
      mode_ = kModeIdle;
    }
    if (ops_.publishState) ops_.publishState();

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::microseconds(static_cast<long>(cfg_.tick_period_s * 1e6));
    while (running_.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

CommandResult ManagerRuntime::startServo(std::uint8_t mode,
                                         std::uint8_t profile) {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  if (mode != kModeDirect && mode != kModeCompliance)
    return {false, "mode must be DIRECT_CARTESIAN or FORCE_COMPLIANCE"};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (core_.state() != SystemState::READY)
      return {false, "start requires READY"};
  }

  bool need_start_rsi = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    need_start_rsi = !eki_.rsi_active;
  }
  if (need_start_rsi) {
    if (!ops_.ekiStartRsi || !ops_.ekiStartRsi())
      return {false, "EKI start_rsi_program rejected"};
  }

  // Bounded wait for RSI frames (decision 4). No EKI rollback on timeout:
  // the system stays READY and the operator may retry or stop.
  const double deadline = nowS() + cfg_.rsi_connect_wait_s;
  bool rsi_ok = false;
  while (nowS() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const HealthInputs in = healthLocked(nowS());
      if (in.rsi_topic_fresh && in.rsi_connected) {
        rsi_ok = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!rsi_ok) return {false, "RSI link did not come up in time"};

  const std::string target = mode == kModeCompliance
                                 ? cfg_.compliance_controller
                                 : cfg_.correction_controller;
  const std::string other = mode == kModeCompliance
                                ? cfg_.correction_controller
                                : cfg_.compliance_controller;
  if (ops_.publishMode) ops_.publishMode(kModeIdle, profile);
  if (!switchFiltered(target, other))
    return {false, "controller switch failed"};
  if (ops_.publishMode) ops_.publishMode(mode, profile);

  std::lock_guard<std::mutex> lock(mutex_);
  const Verdict v = core_.requestStart(healthLocked(nowS()));
  if (!v.accepted) return {false, v.reason};
  mode_ = mode;
  profile_ = profile;
  active_controller_ = target;
  return {true, ""};
}

CommandResult ManagerRuntime::stopServo() {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  std::string active;
  std::uint8_t profile = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const SystemState s = core_.state();
    if (s != SystemState::SERVOING && s != SystemState::DEGRADED)
      return {false, "stop requires SERVOING or DEGRADED"};
    active = active_controller_;
    profile = profile_;
  }
  if (ops_.publishMode) ops_.publishMode(kModeIdle, profile);
  switchFiltered("", active);
  if (ops_.ekiStopRsi) ops_.ekiStopRsi();
  std::lock_guard<std::mutex> lock(mutex_);
  const Verdict v = core_.requestStop();
  if (!v.accepted) return {false, v.reason};
  mode_ = kModeIdle;
  active_controller_.clear();
  return {true, ""};
}

CommandResult ManagerRuntime::resetFault() {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (core_.state() != SystemState::FAULT)
      return {false, "reset requires FAULT"};
  }
  std::string message;
  if (ops_.rsiResetFault && !ops_.rsiResetFault())
    message += "RSI reset failed; ";
  if (ops_.ekiResetFault && !ops_.ekiResetFault())
    message += "EKI reset failed; ";
  std::lock_guard<std::mutex> lock(mutex_);
  const Verdict v = core_.requestReset();
  if (!v.accepted) return {false, v.reason};
  return {true, message};
}

CommandResult ManagerRuntime::zeroSensor() {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!core_.allowZeroSensor())
      return {false, "sensor zero allowed in CONNECTED/READY only"};
  }
  if (!ops_.sriZero || !ops_.sriZero())
    return {false, "sri zero service failed"};
  return {true, ""};
}

CommandResult ManagerRuntime::beginCalibration() {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  std::uint8_t profile = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const Verdict v = core_.requestCalibration(healthLocked(nowS()));
    if (!v.accepted) return {false, v.reason};
    profile = profile_;
  }
  if (ops_.publishMode) ops_.publishMode(kModeIdle, profile);
  if (!switchFiltered(cfg_.correction_controller,
                      cfg_.compliance_controller)) {
    std::lock_guard<std::mutex> lock(mutex_);
    core_.calibrationFinished();  // roll the request back
    return {false, "controller switch failed"};
  }
  if (ops_.publishMode) ops_.publishMode(kModeCalibration, profile);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!seq_.start(nowS())) {
    core_.calibrationFinished();
    return {false, "calibration sequencer rejected start (bad config?)"};
  }
  cal_teardown_done_ = false;
  mode_ = kModeCalibration;
  active_controller_ = cfg_.correction_controller;
  return {true, ""};
}

void ManagerRuntime::cancelCalibration() {
  std::lock_guard<std::mutex> lock(mutex_);
  seq_.cancel();  // the worker tick performs the teardown
}

ManagerSnapshot ManagerRuntime::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  ManagerSnapshot s;
  s.health = healthLocked(nowS());
  s.state = core_.state();
  s.mode = mode_;
  s.profile = profile_;
  s.active_controller = active_controller_;
  s.cal = seq_.status();
  s.calibrating = s.cal.phase == CalPhase::MOVING ||
                  s.cal.phase == CalPhase::SETTLING ||
                  s.cal.phase == CalPhase::SAMPLING ||
                  s.cal.phase == CalPhase::RETURNING;
  s.fit = seq_.result();
  return s;
}

}  // namespace sfm
