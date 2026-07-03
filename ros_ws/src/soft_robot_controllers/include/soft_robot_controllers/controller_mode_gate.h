#pragma once

#include <cstdint>

#include "soft_force_control_core/mode_manager_core.h"
#include "soft_robot_controllers/mode_bridge.h"

namespace soft_robot_controllers {

// Latest-value mode request handed from the subscriber thread to update()
// through a RealtimeBuffer. seq == 0 means "no request received yet";
// the producer side allocates sequence numbers starting at 1.
struct ModeRequest {
  std::uint8_t mode{soft_robot_msgs::ModeCommand::MODE_IDLE};
  std::uint8_t profile{soft_robot_msgs::ModeCommand::PROFILE_PRECISION};
  std::uint64_t seq{0};
};

// Self-initialising RealtimeBuffer payload for the /kuka/rsi/state fault
// flag shared by both controllers (a bare bool would be indeterminate in
// the buffer's default-constructed slots).
struct FaultFlag {
  bool fault{false};
};

// Per-controller mode gate: applies each ModeRequest to an embedded
// ModeManagerCore exactly once (deduplicated by sequence number) and
// reports the rising edge of entering this controller's engaged mode(s).
// Both controllers embed their own gate and see the same command stream;
// ModeManagerCore's deterministic rules keep their conclusions identical
// (decision 3). Pure logic, allocation-free, RT-safe.
class ControllerModeGate {
 public:
  ControllerModeGate(sfc::ControlMode engaged_a, sfc::ControlMode engaged_b);
  explicit ControllerModeGate(sfc::ControlMode engaged)
      : ControllerModeGate(engaged, engaged) {}

  // Applies req if its sequence number is new. Returns true exactly when
  // this call transitioned the gate from disengaged to engaged.
  bool apply(const ModeRequest& req);
  bool engaged() const;
  void forceIdle();  // controller stopping(): drop back to IDLE
  sfc::ModeSnapshot snapshot() const { return mgr_.snapshot(); }
  bool lastRequestOk() const { return last_ok_; }

 private:
  sfc::ModeManagerCore mgr_;
  sfc::ControlMode engaged_a_;
  sfc::ControlMode engaged_b_;
  std::uint64_t last_seq_{0};
  bool last_ok_{true};
};

}  // namespace soft_robot_controllers
