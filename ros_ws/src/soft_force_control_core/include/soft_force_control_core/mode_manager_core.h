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
