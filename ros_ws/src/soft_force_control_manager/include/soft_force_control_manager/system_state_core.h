#pragma once

#include <cstdint>

namespace sfm {

// System states, spec section 11. Values equal the wire constants
// soft_robot_msgs::ModeState::SYSTEM_* (static_assert in the node shell).
enum class SystemState : std::uint8_t {
  OFFLINE = 0,
  CONNECTED = 1,
  READY = 2,
  SERVOING = 3,
  CALIBRATING = 4,
  DEGRADED = 5,
  FAULT = 6,
};

// Health snapshot with all freshness ALREADY judged by the runtime layer
// (message age vs threshold); the core stays clock-free and testable.
struct HealthInputs {
  bool eki_link{false};         // TCP up && RobotState heartbeat fresh
  // I-1 layering inputs (Plan 6 Task 2): the raw components of eki_link,
  // plus the KRC's own view of the RSI context. During RSI_MOVECORR the
  // KRL heartbeat pauses by design, so eki_link alone cannot supervise
  // an RSI-active phase (rule R1), and a latched hw fault seen while
  // idle with a fresh heartbeat and rsi_active=false is residue of the
  // finished RSI session (rule R2).
  bool eki_tcp_connected{false};   // TCP link up (heartbeat not required)
  bool eki_heartbeat_fresh{false}; // topic fresh && msg.state_fresh
  bool eki_rsi_active{false};      // last EkiState.rsi_active
  bool eki_program_ready{false};
  bool eki_fault{false};        // KRC-side latched fault
  bool rsi_topic_fresh{false};  // /kuka/rsi/state age within threshold
  bool rsi_connected{false};    // last RsiState.connected
  bool rsi_fault{false};        // last RsiState.fault
  bool sri_streaming{false};    // /sri_ft/status fresh && streaming
  bool tool_synced{false};      // $TOOL read through EKI since (re)connect
  bool controllers_loaded{false};
};

struct Verdict {
  bool accepted{false};
  const char* reason{""};  // static strings only (no allocation)
};

// Spec-11 state machine (decision 2 details). Pure logic: no ROS, no
// clock, no allocation. update() re-evaluates the state from the health
// snapshot; explicit operator commands arrive through the request*()
// methods. FAULT latches until requestReset(). READY deliberately does
// NOT require RSI frames (the KRC only streams after START_RSI); RSI
// validity is enforced by the start orchestration in the runtime.
class SystemStateCore {
 public:
  SystemState update(const HealthInputs& in);
  Verdict requestStart(const HealthInputs& in);
  Verdict requestStop();
  Verdict requestCalibration(const HealthInputs& in);
  void calibrationFinished();
  Verdict requestReset();
  bool allowZeroSensor() const;  // decision 11: CONNECTED/READY only
  SystemState state() const { return state_; }

 private:
  static bool readyConditions(const HealthInputs& in);
  static bool fullConditions(const HealthInputs& in);
  // I-1 rules (Plan 6 Task 2). R1: with the RSI stream alive, EKI is
  // considered linked as long as TCP is up. R2: a latched hw fault is
  // session residue (maskable) only when no servo/calibration is
  // requested AND the recovered heartbeat shows rsi_active=false.
  static bool ekiLinkLayered(const HealthInputs& in);
  bool faultIsSessionResidue(const HealthInputs& in) const;
  SystemState state_{SystemState::OFFLINE};
  bool servo_requested_{false};
  bool calibrating_{false};
};

}  // namespace sfm
