#include <gtest/gtest.h>

#include <soft_robot_msgs/ModeCommand.h>

#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"
#include "kuka_rsi_hw_interface/rsi_mock_server.h"
#include "soft_robot_controllers/cartesian_correction_controller.h"
#include "soft_robot_controllers/force_compliance_controller.h"

using soft_robot_controllers::CartesianCorrectionController;
using soft_robot_controllers::CartesianCorrectionControllerParams;
using soft_robot_controllers::ForceComplianceController;
using soft_robot_controllers::ForceComplianceParams;
using soft_robot_controllers::WrenchSample;
namespace msg = soft_robot_msgs;

namespace {

kuka_rsi::HwConfig hwConfig() {
  kuka_rsi::HwConfig cfg;
  cfg.listen_ip = "127.0.0.1";
  cfg.listen_port = 0;        // auto-assign; mock targets hw.listenPort()
  cfg.read_timeout_ms = 100;  // generous: mock thread scheduling jitter
  cfg.max_consecutive_timeouts = 5;
  return cfg;
}

ForceComplianceParams precisionParams() {
  ForceComplianceParams p;
  p.filter_cutoff_hz = 0.0;
  p.compliance.translation.gain = 1.0;
  p.compliance.translation.max_speed = 50.0;
  p.compliance.translation.max_accel = 0.0;
  p.compliance.rotation.gain = 0.1;
  p.compliance.rotation.max_speed = 5.0;
  p.compliance.rotation.max_accel = 0.0;
  p.compliance.speed_scale = 1.0;
  p.fixed_force_deadband_n = 30.0;
  p.fixed_torque_deadband_nm = 4.0;
  p.adaptive_deadband = false;
  p.retare.enabled = false;
  p.wrench_timeout_s = 0.012;
  return p;
}

CartesianCorrectionControllerParams directParams() {
  CartesianCorrectionControllerParams p;
  p.direct.stream_timeout_s = 0.1;
  p.direct.safety.max_corr_trans = 0.5;
  p.direct.safety.max_corr_rot = 0.05;
  p.goal_defaults.max_speed_dps = 7.5;
  p.goal_defaults.p_gain = 20.0;
  p.goal_defaults.tol_deg = 0.1;
  p.goal_defaults.hold_s = 0.008;
  p.goal_defaults.timeout_s = 30.0;
  return p;
}

}  // namespace

TEST(ControllerChain, CompliancePushesMockPose) {
  ros::Time::init();
  kuka_rsi::KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  kuka_rsi::RsiMockServer mock(kuka_rsi::MockConfig{}, "127.0.0.1",
                               hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  ForceComplianceController ctl;
  const kuka_rsi::CartesianCorrectionHandle handle =
      hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()->getHandle(
          "kuka_tcp");
  ASSERT_TRUE(ctl.configureController(handle, precisionParams(),
                                      precisionParams()));
  ctl.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                        msg::ModeCommand::PROFILE_PRECISION);
  ctl.starting(ros::Time(0.0));

  double t = 0.0;
  for (int i = 0; i < 50; ++i) {
    t += 0.004;
    hw.read(ros::Time(t), ros::Duration(0.004));
    WrenchSample s;
    s.w.fx = 40.0;  // e = 10 N -> +0.04 mm on X per cycle
    s.stamp_s = t;
    s.valid = true;
    ctl.injectWrench(s);
    ctl.update(ros::Time(t), ros::Duration(0.004));
    hw.write(ros::Time(t), ros::Duration(0.004));
  }
  mock.stop();

  EXPECT_FALSE(hw.faulted());
  EXPECT_EQ(mock.statsSnapshot().ipoc_echo_errors, 0u);
  // ~50 * 0.04 mm; tolerate a few cycles lost to thread startup/shutdown.
  EXPECT_GE(mock.poseX(), 45 * 0.04);
}

TEST(ControllerChain, HardCutoffFreezesMockPose) {
  ros::Time::init();
  kuka_rsi::KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  kuka_rsi::RsiMockServer mock(kuka_rsi::MockConfig{}, "127.0.0.1",
                               hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  ForceComplianceController ctl;
  ASSERT_TRUE(ctl.configureController(
      hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()->getHandle(
          "kuka_tcp"),
      precisionParams(), precisionParams()));
  ctl.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                        msg::ModeCommand::PROFILE_PRECISION);
  ctl.starting(ros::Time(0.0));

  double t = 0.0;
  for (int i = 0; i < 20; ++i) {
    t += 0.004;
    hw.read(ros::Time(t), ros::Duration(0.004));
    WrenchSample s;
    s.w.fx = 600.0;  // above the 500 N ceiling: hard cutoff (spec 12.1)
    s.stamp_s = t;
    s.valid = true;
    ctl.injectWrench(s);
    ctl.update(ros::Time(t), ros::Duration(0.004));
    hw.write(ros::Time(t), ros::Duration(0.004));
  }
  mock.stop();

  EXPECT_DOUBLE_EQ(mock.poseX(), 0.0);  // zero correction throughout
}

TEST(ControllerChain, GoalMotionConvergesOnMock) {
  ros::Time::init();
  kuka_rsi::KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  kuka_rsi::RsiMockServer mock(kuka_rsi::MockConfig{}, "127.0.0.1",
                               hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  CartesianCorrectionController ctl;
  const kuka_rsi::CartesianCorrectionHandle handle =
      hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()->getHandle(
          "kuka_tcp");
  ASSERT_TRUE(ctl.configureController(handle, directParams()));
  ctl.injectModeCommand(msg::ModeCommand::MODE_DIRECT_CARTESIAN,
                        msg::ModeCommand::PROFILE_PRECISION);
  ctl.starting(ros::Time(0.0));

  sfc::MotionGoal g = directParams().goal_defaults;
  g.a = 1.0;  // +1 deg on A, driven through RKorr and the mock's pose
  const std::uint64_t seq = ctl.requestGoal(g);

  double t = 0.0;
  bool converged = false;
  for (int i = 0; i < 300 && !converged; ++i) {
    t += 0.004;
    hw.read(ros::Time(t), ros::Duration(0.004));
    ctl.update(ros::Time(t), ros::Duration(0.004));
    hw.write(ros::Time(t), ros::Duration(0.004));
    converged = ctl.appliedGoalSeq() >= seq &&
                ctl.motionStatus() == sfc::MotionStatus::CONVERGED;
  }
  mock.stop();

  EXPECT_TRUE(converged);
  // The hardware state interface tracked the mock's integrated pose.
  EXPECT_NEAR(handle.getA(), 1.0, 0.2);
}
