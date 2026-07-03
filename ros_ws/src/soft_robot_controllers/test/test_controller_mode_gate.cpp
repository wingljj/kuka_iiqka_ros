#include <gtest/gtest.h>

#include "soft_robot_controllers/controller_mode_gate.h"

using soft_robot_controllers::ControllerModeGate;
using soft_robot_controllers::ModeRequest;
namespace msg = soft_robot_msgs;

namespace {
ModeRequest req(std::uint8_t mode, std::uint8_t profile, std::uint64_t seq) {
  ModeRequest r;
  r.mode = mode;
  r.profile = profile;
  r.seq = seq;
  return r;
}
}  // namespace

TEST(ControllerModeGate, SeqZeroIsIgnored) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_FALSE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                              msg::ModeCommand::PROFILE_DRAG, 0)));
  EXPECT_FALSE(gate.engaged());
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::IDLE);
}

TEST(ControllerModeGate, AppliesOncePerSequenceNumber) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  const ModeRequest r = req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                            msg::ModeCommand::PROFILE_PRECISION, 1);
  EXPECT_TRUE(gate.apply(r));    // entered edge
  EXPECT_FALSE(gate.apply(r));   // same seq: no-op, no edge
  EXPECT_TRUE(gate.engaged());
}

TEST(ControllerModeGate, EnterEdgeFiresOnlyOnTransition) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_TRUE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                             msg::ModeCommand::PROFILE_PRECISION, 1)));
  // Re-requesting the mode we are already in: accepted, but not an edge.
  EXPECT_FALSE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                              msg::ModeCommand::PROFILE_PRECISION, 2)));
  EXPECT_TRUE(gate.engaged());
}

TEST(ControllerModeGate, SingleMessageSelectsProfileAndMode) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_TRUE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                             msg::ModeCommand::PROFILE_DRAG, 1)));
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_EQ(gate.snapshot().profile, sfc::Profile::DRAG);
}

TEST(ControllerModeGate, RejectsDirectActiveToActiveSwitch) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  ASSERT_TRUE(gate.apply(req(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                             msg::ModeCommand::PROFILE_PRECISION, 1)));
  EXPECT_FALSE(gate.apply(req(msg::ModeCommand::MODE_DIRECT_CARTESIAN,
                              msg::ModeCommand::PROFILE_PRECISION, 2)));
  EXPECT_FALSE(gate.lastRequestOk());
  EXPECT_TRUE(gate.engaged());  // still FORCE_COMPLIANCE
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::FORCE_COMPLIANCE);
}

TEST(ControllerModeGate, UnknownValuesLeaveStateUntouched) {
  ControllerModeGate gate(sfc::ControlMode::FORCE_COMPLIANCE);
  EXPECT_FALSE(gate.apply(req(99, msg::ModeCommand::PROFILE_DRAG, 1)));
  EXPECT_FALSE(gate.lastRequestOk());
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::IDLE);
  EXPECT_EQ(gate.snapshot().profile, sfc::Profile::PRECISION);
}

TEST(ControllerModeGate, TwoEngagedModesAndForceIdle) {
  ControllerModeGate gate(sfc::ControlMode::DIRECT_CARTESIAN,
                          sfc::ControlMode::CALIBRATION);
  EXPECT_TRUE(gate.apply(req(msg::ModeCommand::MODE_DIRECT_CARTESIAN,
                             msg::ModeCommand::PROFILE_PRECISION, 1)));
  EXPECT_FALSE(gate.apply(req(msg::ModeCommand::MODE_IDLE,
                              msg::ModeCommand::PROFILE_PRECISION, 2)));
  EXPECT_FALSE(gate.engaged());
  EXPECT_TRUE(gate.apply(req(msg::ModeCommand::MODE_CALIBRATION,
                             msg::ModeCommand::PROFILE_PRECISION, 3)));
  EXPECT_TRUE(gate.engaged());
  gate.forceIdle();
  EXPECT_FALSE(gate.engaged());
  EXPECT_EQ(gate.snapshot().mode, sfc::ControlMode::IDLE);
}
