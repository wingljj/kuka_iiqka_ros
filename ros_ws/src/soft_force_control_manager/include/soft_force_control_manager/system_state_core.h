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
  SystemState state_{SystemState::OFFLINE};
  bool servo_requested_{false};
  bool calibrating_{false};
};

}  // namespace sfm
