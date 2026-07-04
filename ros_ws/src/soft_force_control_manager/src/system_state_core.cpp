#include "soft_force_control_manager/system_state_core.h"

namespace sfm {

// R1 (Plan 6 Task 2, followup I-1): the KRL heartbeat pauses during
// RSI_MOVECORR, so while the 50 Hz RSI stream proves the program alive
// the EKI link requirement relaxes from "heartbeat fresh" to "TCP up".
bool SystemStateCore::ekiLinkLayered(const HealthInputs& in) {
  if (in.eki_link) return true;
  const bool rsi_alive = in.rsi_topic_fresh && in.rsi_connected;
  return rsi_alive && in.eki_tcp_connected;
}

// R2: residue of a finished RSI session. Only maskable when idle (no
// servo request, no calibration) and the recovered heartbeat reports
// rsi_active=false; anything less stays a real fault (fail-closed).
bool SystemStateCore::faultIsSessionResidue(const HealthInputs& in) const {
  if (servo_requested_ || calibrating_) return false;
  return in.eki_heartbeat_fresh && !in.eki_rsi_active;
}

bool SystemStateCore::readyConditions(const HealthInputs& in) {
  return ekiLinkLayered(in) && in.eki_program_ready && in.sri_streaming &&
         in.tool_synced && in.controllers_loaded;
}

bool SystemStateCore::fullConditions(const HealthInputs& in) {
  return readyConditions(in) && in.rsi_topic_fresh && in.rsi_connected;
}

SystemState SystemStateCore::update(const HealthInputs& in) {
  const bool live_rsi_fault = in.rsi_fault && !faultIsSessionResidue(in);
  if (in.eki_fault || live_rsi_fault) {
    state_ = SystemState::FAULT;
    servo_requested_ = false;  // a fault always demands an explicit restart
    calibrating_ = false;
    return state_;
  }
  if (state_ == SystemState::FAULT) return state_;  // latched

  if (calibrating_) {
    // Link loss mid-calibration is fatal (the robot may be far from the
    // start pose with a half-written estimate): latch FAULT.
    const bool link_ok = ekiLinkLayered(in) && in.rsi_topic_fresh &&
                         in.rsi_connected && in.sri_streaming;
    state_ = link_ok ? SystemState::CALIBRATING : SystemState::FAULT;
    if (state_ == SystemState::FAULT) calibrating_ = false;
    return state_;
  }

  if (servo_requested_) {
    state_ = fullConditions(in) ? SystemState::SERVOING
                                : SystemState::DEGRADED;
    return state_;
  }

  if (!ekiLinkLayered(in)) {
    state_ = SystemState::OFFLINE;
  } else if (readyConditions(in)) {
    state_ = SystemState::READY;
  } else {
    state_ = SystemState::CONNECTED;
  }
  return state_;
}

Verdict SystemStateCore::requestStart(const HealthInputs& in) {
  if (state_ != SystemState::READY)
    return {false, "start requires READY"};
  // rsi_connected is NOT required here: the start orchestration issues
  // START_RSI first and waits for frames before flipping servo_requested_.
  if (!in.rsi_topic_fresh)
    return {false, "RSI hw node state topic is stale"};
  if (!readyConditions(in))
    return {false, "READY preconditions lost"};
  servo_requested_ = true;
  return {true, ""};
}

Verdict SystemStateCore::requestStop() {
  if (state_ != SystemState::SERVOING && state_ != SystemState::DEGRADED)
    return {false, "stop requires SERVOING or DEGRADED"};
  servo_requested_ = false;
  return {true, ""};
}

Verdict SystemStateCore::requestCalibration(const HealthInputs& in) {
  if (state_ != SystemState::READY)
    return {false, "calibration requires READY"};
  if (!in.rsi_topic_fresh || !in.rsi_connected)
    return {false, "RSI link not valid"};
  if (!in.sri_streaming)
    return {false, "SRI not streaming"};
  calibrating_ = true;
  return {true, ""};
}

void SystemStateCore::calibrationFinished() { calibrating_ = false; }

Verdict SystemStateCore::requestReset() {
  if (state_ != SystemState::FAULT)
    return {false, "reset requires FAULT"};
  state_ = SystemState::OFFLINE;  // re-evaluated by the next update()
  return {true, ""};
}

bool SystemStateCore::allowZeroSensor() const {
  return state_ == SystemState::CONNECTED || state_ == SystemState::READY;
}

}  // namespace sfm
