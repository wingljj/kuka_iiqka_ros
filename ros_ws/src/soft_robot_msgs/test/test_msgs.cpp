#include <gtest/gtest.h>

#include <soft_robot_msgs/CalibratePayloadAction.h>
#include <soft_robot_msgs/ManagerState.h>
#include <soft_robot_msgs/StartServo.h>
#include <soft_robot_msgs/CartesianCorrectionStamped.h>
#include <soft_robot_msgs/CartesianState.h>
#include <soft_robot_msgs/GetTool.h>
#include <soft_robot_msgs/ModeCommand.h>
#include <soft_robot_msgs/ModeState.h>
#include <soft_robot_msgs/MoveToOrientationAction.h>
#include <soft_robot_msgs/RsiState.h>
#include <soft_robot_msgs/EkiState.h>
#include <soft_robot_msgs/SetEkiMode.h>
#include <soft_robot_msgs/SetFilter.h>
#include <soft_robot_msgs/SetToolBase.h>
#include <soft_robot_msgs/SriStatus.h>

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

TEST(Msgs, DriverStatusMsgsDefaultToDisconnected) {
  soft_robot_msgs::SriStatus s;
  EXPECT_FALSE(s.connected);
  EXPECT_FALSE(s.streaming);
  EXPECT_EQ(s.samples, 0u);
  EXPECT_EQ(s.bad_frames, 0u);
  EXPECT_EQ(s.package_gaps, 0u);
  EXPECT_FALSE(s.zero_active);
  EXPECT_EQ(s.filter_cutoff_hz, 0.0);
  soft_robot_msgs::EkiState e;
  EXPECT_FALSE(e.connected);
  EXPECT_FALSE(e.state_fresh);
  EXPECT_FALSE(e.program_ready);
  EXPECT_FALSE(e.rsi_active);
  EXPECT_FALSE(e.fault);
  EXPECT_EQ(e.mode, 0u);
  EXPECT_EQ(e.tool_x, 0.0);
  EXPECT_EQ(e.tool_c, 0.0);
}

TEST(Msgs, ManagementSrvResponsesDefaultToFailure) {
  soft_robot_msgs::SetFilter::Response f;
  EXPECT_FALSE(f.success);
  soft_robot_msgs::SetEkiMode::Response m;
  EXPECT_FALSE(m.success);
  soft_robot_msgs::SetToolBase::Response t;
  EXPECT_FALSE(t.success);
  soft_robot_msgs::SetToolBase::Request req;
  EXPECT_EQ(req.tool_x, 0.0);
  EXPECT_EQ(req.base_c, 0.0);
}

// The manager republishes the spec-11 state machine through ManagerState;
// the numbering source stays ModeState.SYSTEM_* (no duplicated constants).
TEST(Msgs, ManagerStateDefaultsToOfflineShape) {
  soft_robot_msgs::ManagerState m;
  EXPECT_EQ(m.system_state, soft_robot_msgs::ModeState::SYSTEM_OFFLINE);
  EXPECT_FALSE(m.eki_connected);
  EXPECT_FALSE(m.rsi_connected);
  EXPECT_FALSE(m.sri_streaming);
  EXPECT_FALSE(m.tool_synced);
  EXPECT_FALSE(m.calibrating);
  EXPECT_TRUE(m.active_controller.empty());
}

TEST(Msgs, CalibratePayloadResultCarriesFullFit) {
  soft_robot_msgs::CalibratePayloadResult r;
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.gravity_n, 0.0);
  EXPECT_EQ(r.bias_tz, 0.0);
  EXPECT_EQ(r.r2_force, 0.0);
  soft_robot_msgs::CalibratePayloadFeedback f;
  EXPECT_EQ(f.pose_index, 0u);
  EXPECT_TRUE(f.phase.empty());
  soft_robot_msgs::StartServo::Request req;
  EXPECT_EQ(req.mode, 0u);
  soft_robot_msgs::StartServo::Response resp;
  EXPECT_FALSE(resp.success);
}
