#include "soft_robot_controllers/controller_mode_gate.h"

namespace soft_robot_controllers {

ControllerModeGate::ControllerModeGate(sfc::ControlMode engaged_a,
                                       sfc::ControlMode engaged_b)
    : engaged_a_(engaged_a), engaged_b_(engaged_b) {}

bool ControllerModeGate::engaged() const {
  const sfc::ControlMode m = mgr_.snapshot().mode;
  return m == engaged_a_ || m == engaged_b_;
}

void ControllerModeGate::forceIdle() {
  mgr_.requestMode(sfc::ControlMode::IDLE);  // always allowed
}

bool ControllerModeGate::apply(const ModeRequest& req) {
  if (req.seq == 0 || req.seq == last_seq_) return false;
  last_seq_ = req.seq;

  sfc::ControlMode mode = sfc::ControlMode::IDLE;
  sfc::Profile profile = sfc::Profile::PRECISION;
  if (!toControlMode(req.mode, mode) || !toProfile(req.profile, profile)) {
    last_ok_ = false;  // unknown wire value: ignore the request entirely
    return false;
  }

  const bool was_engaged = engaged();
  // Profile changes are only legal while IDLE (ModeManagerCore rule).
  // Apply the profile before the mode switch when leaving IDLE and after
  // it when entering IDLE, so a single message can select profile and
  // mode together (decision 12).
  bool profile_applied = false;
  if (mgr_.snapshot().mode == sfc::ControlMode::IDLE) {
    profile_applied = mgr_.setProfile(profile);
  }
  last_ok_ = mgr_.requestMode(mode);
  if (last_ok_ && !profile_applied &&
      mgr_.snapshot().mode == sfc::ControlMode::IDLE) {
    mgr_.setProfile(profile);
  }
  return last_ok_ && !was_engaged && engaged();
}

}  // namespace soft_robot_controllers
