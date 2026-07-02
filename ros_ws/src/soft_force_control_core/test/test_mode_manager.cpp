#include <gtest/gtest.h>
#include "soft_force_control_core/mode_manager_core.h"

using sfc::ControlMode;
using sfc::ModeManagerCore;
using sfc::Profile;

TEST(ModeManager, StartsIdlePrecision) {
  ModeManagerCore m;
  EXPECT_EQ(m.snapshot().mode, ControlMode::IDLE);
  EXPECT_EQ(m.snapshot().profile, Profile::PRECISION);
}

TEST(ModeManager, IdleCanEnterAnyMode) {
  for (auto target : {ControlMode::DIRECT_CARTESIAN,
                      ControlMode::FORCE_COMPLIANCE,
                      ControlMode::CALIBRATION}) {
    ModeManagerCore m;
    EXPECT_TRUE(m.requestMode(target));
    EXPECT_EQ(m.snapshot().mode, target);
  }
}

TEST(ModeManager, AnyModeCanReturnToIdle) {
  ModeManagerCore m;
  ASSERT_TRUE(m.requestMode(ControlMode::FORCE_COMPLIANCE));
  EXPECT_TRUE(m.requestMode(ControlMode::IDLE));
  EXPECT_EQ(m.snapshot().mode, ControlMode::IDLE);
}

TEST(ModeManager, DirectSwitchBetweenActiveModesRejected) {
  ModeManagerCore m;
  ASSERT_TRUE(m.requestMode(ControlMode::FORCE_COMPLIANCE));
  EXPECT_FALSE(m.requestMode(ControlMode::DIRECT_CARTESIAN));
  EXPECT_FALSE(m.requestMode(ControlMode::CALIBRATION));
  EXPECT_EQ(m.snapshot().mode, ControlMode::FORCE_COMPLIANCE);
}

TEST(ModeManager, ProfileChangeOnlyInIdle) {
  ModeManagerCore m;
  EXPECT_TRUE(m.setProfile(Profile::DRAG));
  ASSERT_TRUE(m.requestMode(ControlMode::FORCE_COMPLIANCE));
  EXPECT_FALSE(m.setProfile(Profile::PRECISION));
  EXPECT_EQ(m.snapshot().profile, Profile::DRAG);
}
