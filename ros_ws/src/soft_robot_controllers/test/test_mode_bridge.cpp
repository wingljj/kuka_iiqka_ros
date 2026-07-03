#include <gtest/gtest.h>

#include "soft_robot_controllers/mode_bridge.h"

using soft_robot_controllers::fromControlMode;
using soft_robot_controllers::fromProfile;
using soft_robot_controllers::toControlMode;
using soft_robot_controllers::toProfile;

// The static_asserts in mode_bridge.h are the real alignment gate: this
// binary compiling at all proves msg constants match the sfc enums.
TEST(ModeBridge, ControlModeRoundTrip) {
  for (const sfc::ControlMode m :
       {sfc::ControlMode::IDLE, sfc::ControlMode::DIRECT_CARTESIAN,
        sfc::ControlMode::FORCE_COMPLIANCE, sfc::ControlMode::CALIBRATION}) {
    sfc::ControlMode back = sfc::ControlMode::IDLE;
    ASSERT_TRUE(toControlMode(fromControlMode(m), back));
    EXPECT_EQ(back, m);
  }
}

TEST(ModeBridge, ProfileRoundTrip) {
  for (const sfc::Profile p : {sfc::Profile::DRAG, sfc::Profile::PRECISION}) {
    sfc::Profile back = sfc::Profile::DRAG;
    ASSERT_TRUE(toProfile(fromProfile(p), back));
    EXPECT_EQ(back, p);
  }
}

TEST(ModeBridge, RejectsUnknownMode) {
  sfc::ControlMode out = sfc::ControlMode::FORCE_COMPLIANCE;
  EXPECT_FALSE(toControlMode(99, out));
  // A rejected conversion must not touch the output.
  EXPECT_EQ(out, sfc::ControlMode::FORCE_COMPLIANCE);
}

TEST(ModeBridge, RejectsUnknownProfile) {
  sfc::Profile out = sfc::Profile::PRECISION;
  EXPECT_FALSE(toProfile(7, out));
  EXPECT_EQ(out, sfc::Profile::PRECISION);
}
