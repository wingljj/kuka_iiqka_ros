#include <gtest/gtest.h>

#include <soft_robot_msgs/ModeCommand.h>

#include "soft_robot_controllers/force_compliance_controller.h"

using soft_robot_controllers::ForceComplianceController;
using soft_robot_controllers::ForceComplianceParams;
using soft_robot_controllers::WrenchSample;
namespace msg = soft_robot_msgs;

namespace {

constexpr double kT0 = 100.0;  // synthetic clock origin [s]

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

ForceComplianceParams dragParams() {
  ForceComplianceParams p = precisionParams();
  p.adaptive_deadband = true;
  p.ramp_window_s = 0.02;  // 5 cycles
  p.ramp_force_margin_n = 5.0;
  p.ramp_torque_margin_nm = 1.0;
  p.retare.enabled = true;
  p.retare.orientation_tol_deg = 1.0;
  return p;
}

class FccControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::Time::init();  // wall-clock ros::Time without a master
    state_[0] = 500.0;
    state_[1] = 0.0;
    state_[2] = 800.0;
    state_[3] = 0.0;
    state_[4] = 0.0;
    state_[5] = 0.0;
    for (double& c : cmd_) c = 0.0;
    const kuka_rsi::CartesianStateHandle sh("kuka_tcp", &state_[0], &state_[1],
                                            &state_[2], &state_[3], &state_[4],
                                            &state_[5]);
    handle_ = kuka_rsi::CartesianCorrectionHandle(sh, &cmd_[0], &cmd_[1],
                                                  &cmd_[2], &cmd_[3], &cmd_[4],
                                                  &cmd_[5]);
    ASSERT_TRUE(
        ctl_.configureController(handle_, dragParams(), precisionParams()));
    ctl_.starting(ros::Time(kT0));
  }

  // One control cycle: fresh wrench sample stamped at the update time.
  void cycle(double fz, double t, double dt = 0.004) {
    WrenchSample s;
    s.w.fz = fz;
    s.stamp_s = t;
    s.valid = true;
    ctl_.injectWrench(s);
    ctl_.update(ros::Time(t), ros::Duration(dt));
  }

  double state_[6];
  double cmd_[6];
  kuka_rsi::CartesianCorrectionHandle handle_;
  ForceComplianceController ctl_;
};

}  // namespace

TEST_F(FccControllerTest, DisengagedByDefaultOutputsZero) {
  cycle(100.0, kT0);
  for (const double c : cmd_) EXPECT_EQ(c, 0.0);
}

TEST_F(FccControllerTest, ModeEntryEnablesCompliance) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0);  // e = 10 N -> 10 mm/s -> 0.04 mm per 4 ms cycle
  EXPECT_NEAR(cmd_[2], 0.04, 1e-12);
  EXPECT_EQ(cmd_[0], 0.0);
}

TEST_F(FccControllerTest, MeasuredPeriodScalesCommand) {
  // Plan 2 follow-up 1: the controller must forward the measured period,
  // never a hardcoded 4 ms.
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0, 0.008);
  EXPECT_NEAR(cmd_[2], 0.08, 1e-12);
}

TEST_F(FccControllerTest, StaleWrenchOutputsZero) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  WrenchSample s;
  s.w.fz = 40.0;
  s.stamp_s = kT0;
  s.valid = true;
  ctl_.injectWrench(s);
  ctl_.update(ros::Time(kT0 + 0.02), ros::Duration(0.004));  // 20 ms old
  EXPECT_EQ(cmd_[2], 0.0);
}

TEST_F(FccControllerTest, RsiFaultForcesZero) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0);
  ASSERT_GT(cmd_[2], 0.0);
  ctl_.injectRsiFault(true);
  cycle(40.0, kT0 + 0.004);
  EXPECT_EQ(cmd_[2], 0.0);
}

TEST_F(FccControllerTest, IdleCommandDisengages) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0);
  ASSERT_GT(cmd_[2], 0.0);
  ctl_.injectModeCommand(msg::ModeCommand::MODE_IDLE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(40.0, kT0 + 0.004);
  EXPECT_EQ(cmd_[2], 0.0);
}

TEST_F(FccControllerTest, DragEntryRunsStartupRamp) {
  ctl_.injectModeCommand(msg::ModeCommand::MODE_FORCE_COMPLIANCE,
                         msg::ModeCommand::PROFILE_DRAG);
  // 5 ramp cycles at zero residual: output must stay zero (spec 7.4).
  for (int i = 0; i < 5; ++i) {
    cycle(0.0, kT0 + 0.004 * i);
    EXPECT_EQ(cmd_[2], 0.0) << "ramp cycle " << i;
  }
  // Learned deadband = 0 + 5 N margin; 40 N now produces output.
  cycle(40.0, kT0 + 0.004 * 5);
  EXPECT_NEAR(cmd_[2], (40.0 - 5.0) * 1.0 * 0.004, 1e-12);
}
