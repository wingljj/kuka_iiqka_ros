#include <gtest/gtest.h>

#include <soft_robot_msgs/CartesianCorrectionStamped.h>
#include <soft_robot_msgs/CartesianState.h>
#include <soft_robot_msgs/GetTool.h>
#include <soft_robot_msgs/ModeCommand.h>
#include <soft_robot_msgs/ModeState.h>
#include <soft_robot_msgs/MoveToOrientationAction.h>
#include <soft_robot_msgs/RsiState.h>

// Numeric values must match the sfc enums from soft_force_control_core:
// ControlMode { IDLE, DIRECT_CARTESIAN, FORCE_COMPLIANCE, CALIBRATION }
// Profile { DRAG, PRECISION }.
TEST(Msgs, ModeConstantsMatchCoreEnums) {
  EXPECT_EQ(soft_robot_msgs::ModeCommand::MODE_IDLE, 0u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::MODE_DIRECT_CARTESIAN, 1u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::MODE_FORCE_COMPLIANCE, 2u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::MODE_CALIBRATION, 3u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::PROFILE_DRAG, 0u);
  EXPECT_EQ(soft_robot_msgs::ModeCommand::PROFILE_PRECISION, 1u);
}

// System states follow spec section 11 ordering.
TEST(Msgs, SystemStateConstantsCoverStateMachine) {
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_OFFLINE, 0u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_CONNECTED, 1u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_READY, 2u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_SERVOING, 3u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_CALIBRATING, 4u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_DEGRADED, 5u);
  EXPECT_EQ(soft_robot_msgs::ModeState::SYSTEM_FAULT, 6u);
}

TEST(Msgs, MoveToOrientationResultCodes) {
  EXPECT_EQ(soft_robot_msgs::MoveToOrientationResult::CONVERGED, 0u);
  EXPECT_EQ(soft_robot_msgs::MoveToOrientationResult::TIMEOUT, 1u);
  EXPECT_EQ(soft_robot_msgs::MoveToOrientationResult::ABORTED, 2u);
}

TEST(Msgs, DefaultConstructedFieldsAreZero) {
  soft_robot_msgs::CartesianState s;
  EXPECT_EQ(s.x, 0.0);
  EXPECT_EQ(s.c, 0.0);
  soft_robot_msgs::CartesianCorrectionStamped cs;
  EXPECT_EQ(cs.correction.a, 0.0);
  soft_robot_msgs::RsiState r;
  EXPECT_FALSE(r.connected);
  EXPECT_FALSE(r.fault);
  EXPECT_EQ(r.ipoc, 0u);
  soft_robot_msgs::GetTool::Response resp;
  EXPECT_FALSE(resp.success);
  soft_robot_msgs::MoveToOrientationGoal g;
  EXPECT_FALSE(g.use_position);
  EXPECT_EQ(g.speed_scale, 0.0);
}
