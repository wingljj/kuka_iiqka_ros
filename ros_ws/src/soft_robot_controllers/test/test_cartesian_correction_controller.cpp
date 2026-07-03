#include <gtest/gtest.h>

#include <soft_robot_msgs/ModeCommand.h>

#include "soft_robot_controllers/cartesian_correction_controller.h"

using soft_robot_controllers::CartesianCorrectionController;
using soft_robot_controllers::CartesianCorrectionControllerParams;
using soft_robot_controllers::StreamCommand;
namespace msg = soft_robot_msgs;

namespace {

constexpr double kT0 = 200.0;
constexpr double kDt = 0.004;

CartesianCorrectionControllerParams params() {
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

StreamCommand stream(double x, double stamp_s) {
  StreamCommand c;
  c.correction.x = x;
  c.stamp_s = stamp_s;
  c.valid = true;
  return c;
}

class CccControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::Time::init();
    for (double& s : state_) s = 0.0;
    for (double& c : cmd_) c = 0.0;
    const kuka_rsi::CartesianStateHandle sh("kuka_tcp", &state_[0], &state_[1],
                                            &state_[2], &state_[3], &state_[4],
                                            &state_[5]);
    handle_ = kuka_rsi::CartesianCorrectionHandle(sh, &cmd_[0], &cmd_[1],
                                                  &cmd_[2], &cmd_[3], &cmd_[4],
                                                  &cmd_[5]);
    ASSERT_TRUE(ctl_.configureController(handle_, params()));
    ctl_.starting(ros::Time(kT0));
  }

  void engage() {
    ctl_.injectModeCommand(msg::ModeCommand::MODE_DIRECT_CARTESIAN,
                           msg::ModeCommand::PROFILE_PRECISION);
  }
  void cycle(double t) { ctl_.update(ros::Time(t), ros::Duration(kDt)); }

  double state_[6];
  double cmd_[6];
  kuka_rsi::CartesianCorrectionHandle handle_;
  CartesianCorrectionController ctl_;
};

sfc::MotionGoal goalA(double a) {
  sfc::MotionGoal g = params().goal_defaults;
  g.a = a;
  return g;
}

}  // namespace

TEST_F(CccControllerTest, NotEngagedZerosStream) {
  ctl_.injectStream(stream(0.2, kT0));
  cycle(kT0);
  for (const double c : cmd_) EXPECT_EQ(c, 0.0);
  EXPECT_FALSE(ctl_.engagedNow());
}

TEST_F(CccControllerTest, StreamPassesThroughWhenEngaged) {
  engage();
  ctl_.injectStream(stream(0.2, kT0));
  cycle(kT0);
  EXPECT_TRUE(ctl_.engagedNow());
  EXPECT_NEAR(cmd_[0], 0.2, 1e-12);
}

TEST_F(CccControllerTest, StaleStreamZeros) {
  engage();
  ctl_.injectStream(stream(0.2, kT0 - 0.2));  // older than 0.1 s timeout
  cycle(kT0);
  EXPECT_EQ(cmd_[0], 0.0);
}

TEST_F(CccControllerTest, RsiFaultZeroesOutput) {
  engage();
  ctl_.injectStream(stream(0.2, kT0));
  cycle(kT0);
  ASSERT_NEAR(cmd_[0], 0.2, 1e-12);
  ctl_.injectRsiFault(true);
  ctl_.injectStream(stream(0.2, kT0 + kDt));
  cycle(kT0 + kDt);
  EXPECT_EQ(cmd_[0], 0.0);
  EXPECT_FALSE(ctl_.engagedNow());
}

TEST_F(CccControllerTest, GoalRunsToConvergenceOnIntegratedPose) {
  engage();
  const std::uint64_t seq = ctl_.requestGoal(goalA(0.5));
  int cycles = 0;
  for (; cycles < 100; ++cycles) {
    cycle(kT0 + cycles * kDt);
    state_[3] += cmd_[3];  // plant model: pose integrates the correction
    if (ctl_.appliedGoalSeq() >= seq &&
        ctl_.motionStatus() == sfc::MotionStatus::CONVERGED) {
      break;
    }
  }
  EXPECT_EQ(ctl_.motionStatus(), sfc::MotionStatus::CONVERGED);
  EXPECT_LT(cycles, 100);
  EXPECT_NEAR(state_[3], 0.5, 0.15);  // within tolerance + hold overshoot
}

TEST_F(CccControllerTest, GoalOverridesStreamThenStreamResumes) {
  engage();
  state_[3] = 0.05;  // already inside tolerance of goal a = 0
  ctl_.requestGoal(goalA(0.0));
  ctl_.injectStream(stream(0.2, kT0));
  cycle(kT0);                      // hold 1/2: goal path in charge
  EXPECT_EQ(cmd_[0], 0.0);
  cycle(kT0 + kDt);                // hold 2/2 -> CONVERGED
  EXPECT_EQ(ctl_.motionStatus(), sfc::MotionStatus::CONVERGED);
  ctl_.injectStream(stream(0.2, kT0 + 2 * kDt));
  cycle(kT0 + 2 * kDt);            // stream resumes
  EXPECT_NEAR(cmd_[0], 0.2, 1e-12);
}

TEST_F(CccControllerTest, RequestCancelStopsGoal) {
  engage();
  ctl_.requestGoal(goalA(90.0));
  cycle(kT0);
  ASSERT_EQ(ctl_.motionStatus(), sfc::MotionStatus::RUNNING);
  ctl_.requestCancel();
  cycle(kT0 + kDt);
  EXPECT_EQ(ctl_.motionStatus(), sfc::MotionStatus::INACTIVE);
  EXPECT_EQ(cmd_[3], 0.0);
}

TEST_F(CccControllerTest, ModeExitCancelsRunningGoal) {
  engage();
  ctl_.requestGoal(goalA(90.0));
  cycle(kT0);
  ASSERT_EQ(ctl_.motionStatus(), sfc::MotionStatus::RUNNING);
  ctl_.injectModeCommand(msg::ModeCommand::MODE_IDLE,
                         msg::ModeCommand::PROFILE_PRECISION);
  cycle(kT0 + kDt);
  EXPECT_EQ(ctl_.motionStatus(), sfc::MotionStatus::INACTIVE);
  EXPECT_EQ(cmd_[3], 0.0);
  EXPECT_FALSE(ctl_.engagedNow());
}

TEST(CccSpeedScale, ClampsOutOfRangeValues) {
  EXPECT_EQ(CartesianCorrectionController::clampSpeedScale(0.0), 1.0);
  EXPECT_EQ(CartesianCorrectionController::clampSpeedScale(-0.5), 1.0);
  EXPECT_EQ(CartesianCorrectionController::clampSpeedScale(1.5), 1.0);
  EXPECT_EQ(CartesianCorrectionController::clampSpeedScale(0.5), 0.5);
}
