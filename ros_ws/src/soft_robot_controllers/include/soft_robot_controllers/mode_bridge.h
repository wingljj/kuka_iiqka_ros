#pragma once

#include <soft_robot_msgs/ModeCommand.h>

#include <cstdint>

#include "soft_force_control_core/types.h"

namespace soft_robot_controllers {

// Compile-time alignment between the soft_robot_msgs wire constants and
// the sfc enums (Plan 2 follow-up 4). If either side is ever reordered,
// this header stops compiling instead of silently misrouting modes.
static_assert(soft_robot_msgs::ModeCommand::MODE_IDLE ==
                  static_cast<std::uint8_t>(sfc::ControlMode::IDLE),
              "MODE_IDLE must equal sfc::ControlMode::IDLE");
static_assert(soft_robot_msgs::ModeCommand::MODE_DIRECT_CARTESIAN ==
                  static_cast<std::uint8_t>(sfc::ControlMode::DIRECT_CARTESIAN),
              "MODE_DIRECT_CARTESIAN must equal sfc::ControlMode::DIRECT_CARTESIAN");
static_assert(soft_robot_msgs::ModeCommand::MODE_FORCE_COMPLIANCE ==
                  static_cast<std::uint8_t>(sfc::ControlMode::FORCE_COMPLIANCE),
              "MODE_FORCE_COMPLIANCE must equal sfc::ControlMode::FORCE_COMPLIANCE");
static_assert(soft_robot_msgs::ModeCommand::MODE_CALIBRATION ==
                  static_cast<std::uint8_t>(sfc::ControlMode::CALIBRATION),
              "MODE_CALIBRATION must equal sfc::ControlMode::CALIBRATION");
static_assert(soft_robot_msgs::ModeCommand::PROFILE_DRAG ==
                  static_cast<std::uint8_t>(sfc::Profile::DRAG),
              "PROFILE_DRAG must equal sfc::Profile::DRAG");
static_assert(soft_robot_msgs::ModeCommand::PROFILE_PRECISION ==
                  static_cast<std::uint8_t>(sfc::Profile::PRECISION),
              "PROFILE_PRECISION must equal sfc::Profile::PRECISION");

// Checked conversions from wire values. Unknown values are rejected (the
// caller ignores the whole request) instead of being cast blindly.
inline bool toControlMode(std::uint8_t raw, sfc::ControlMode& out) {
  switch (raw) {
    case soft_robot_msgs::ModeCommand::MODE_IDLE:
      out = sfc::ControlMode::IDLE;
      return true;
    case soft_robot_msgs::ModeCommand::MODE_DIRECT_CARTESIAN:
      out = sfc::ControlMode::DIRECT_CARTESIAN;
      return true;
    case soft_robot_msgs::ModeCommand::MODE_FORCE_COMPLIANCE:
      out = sfc::ControlMode::FORCE_COMPLIANCE;
      return true;
    case soft_robot_msgs::ModeCommand::MODE_CALIBRATION:
      out = sfc::ControlMode::CALIBRATION;
      return true;
    default:
      return false;
  }
}

inline bool toProfile(std::uint8_t raw, sfc::Profile& out) {
  switch (raw) {
    case soft_robot_msgs::ModeCommand::PROFILE_DRAG:
      out = sfc::Profile::DRAG;
      return true;
    case soft_robot_msgs::ModeCommand::PROFILE_PRECISION:
      out = sfc::Profile::PRECISION;
      return true;
    default:
      return false;
  }
}

inline std::uint8_t fromControlMode(sfc::ControlMode m) {
  return static_cast<std::uint8_t>(m);
}
inline std::uint8_t fromProfile(sfc::Profile p) {
  return static_cast<std::uint8_t>(p);
}

}  // namespace soft_robot_controllers
