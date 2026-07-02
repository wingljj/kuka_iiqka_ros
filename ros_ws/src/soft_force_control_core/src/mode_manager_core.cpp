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
