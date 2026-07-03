#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "soft_force_control_core/rotation.h"
#include "soft_force_control_manager/calibration_sequencer.h"

using sfm::CalAction;
using sfm::CalFailure;
using sfm::CalibrationConfig;
using sfm::CalibrationSequencer;
using sfm::CalPhase;
using sfm::CalPose;
using sfc::Wrench;

namespace {

// Same synthetic model as the Plan 1 estimator test: raw = bias + gravity.
Wrench synth(double G, const Eigen::Vector3d& com, const Wrench& bias,
             double a, double b, double c) {
  const Eigen::Matrix3d r = sfc::kukaAbcToRotation(a, b, c);
  const Eigen::Vector3d f = r.transpose() * Eigen::Vector3d(0, 0, -G);
  const Eigen::Vector3d t = com.cross(f);
  Wrench w;
  w.fx = bias.fx + f.x();
  w.fy = bias.fy + f.y();
  w.fz = bias.fz + f.z();
  w.tx = bias.tx + t.x();
  w.ty = bias.ty + t.y();
  w.tz = bias.tz + t.z();
  return w;
}

const double kPoses[8][3] = {{0, 0, 0},    {0, 45, 0},  {0, -45, 0},
                             {0, 0, 45},   {0, 0, -45}, {45, 30, 0},
                             {-45, 0, 30}, {30, -30, 30}};

CalibrationConfig config8() {
  CalibrationConfig c;
  for (const auto& p : kPoses) c.poses.push_back(CalPose{p[0], p[1], p[2]});
  c.settle_time_s = 1.0;
  c.samples_per_pose = 2;
  return c;  // return_pose defaults to A=B=C=0
}

// Drives MOVING -> SETTLING -> SAMPLING for the current pose and asserts
// the emitted goal matches. Returns the goal for further checks.
CalPose passMoveAndSettle(CalibrationSequencer& s, double& now) {
  CalAction act = s.tick(now);
  EXPECT_TRUE(act.send_goal);
  s.onMotionResult(true, now);
  EXPECT_EQ(s.status().phase, CalPhase::SETTLING);
  now += 1.0;  // == settle_time_s
  EXPECT_FALSE(s.tick(now).send_goal);
  EXPECT_EQ(s.status().phase, CalPhase::SAMPLING);
  return act.target;
}

}  // namespace

TEST(CalibrationSequencer, HappyPathRecoversPayloadExactly) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.02, 0.03);
  Wrench bias;
  bias.fx = 1.0;
  bias.fy = -2.0;
  bias.fz = 0.5;
  bias.tx = 0.1;
  bias.ty = -0.2;
  bias.tz = 0.05;

  CalibrationSequencer s;
  s.configure(config8());
  double now = 0.0;
  ASSERT_TRUE(s.start(now));
  ASSERT_EQ(s.status().pose_count, 8u);

  for (int i = 0; i < 8; ++i) {
    const CalPose goal = passMoveAndSettle(s, now);
    EXPECT_DOUBLE_EQ(goal.a, kPoses[i][0]);
    EXPECT_DOUBLE_EQ(goal.b, kPoses[i][1]);
    // Two samples at synth +/- 0.5 on fx: their mean is exactly synth, so
    // the fit must match the Plan 1 exact-recovery case. A last-sample
    // (non-averaging) bug would leave a -0.5 residual and fail below.
    Wrench w1 = synth(G, com, bias, goal.a, goal.b, goal.c);
    Wrench w2 = w1;
    w1.fx += 0.5;
    w2.fx -= 0.5;
    s.onWrench(w1);
    EXPECT_EQ(s.status().samples_collected, 1);
    s.onWrench(w2);
  }

  ASSERT_EQ(s.status().phase, CalPhase::RETURNING);
  const CalAction ret = s.tick(now);
  ASSERT_TRUE(ret.send_goal);
  EXPECT_DOUBLE_EQ(ret.target.a, 0.0);
  EXPECT_DOUBLE_EQ(ret.target.b, 0.0);
  EXPECT_DOUBLE_EQ(ret.target.c, 0.0);
  s.onMotionResult(true, now);
  ASSERT_EQ(s.status().phase, CalPhase::DONE);
  EXPECT_TRUE(s.status().return_move_ok);

  const sfc::PayloadFitResult& r = s.result();
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.params.gravity_n, G, 1e-8);
  EXPECT_NEAR(r.params.com_x, com.x(), 1e-8);
  EXPECT_NEAR(r.params.com_y, com.y(), 1e-8);
  EXPECT_NEAR(r.params.com_z, com.z(), 1e-8);
  EXPECT_NEAR(r.params.bias.fx, bias.fx, 1e-8);
  EXPECT_NEAR(r.params.bias.tz, bias.tz, 1e-8);
  EXPECT_NEAR(r.r2_force, 1.0, 1e-9);
  EXPECT_NEAR(r.r2_torque, 1.0, 1e-9);
}

TEST(CalibrationSequencer, GoalIsEmittedExactlyOnce) {
  CalibrationSequencer s;
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  EXPECT_TRUE(s.tick(0.0).send_goal);
  EXPECT_FALSE(s.tick(0.1).send_goal);  // not re-emitted while MOVING
}

TEST(CalibrationSequencer, SettleWindowIsRespected) {
  CalibrationSequencer s;
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  s.tick(0.0);
  s.onMotionResult(true, 10.0);
  s.tick(10.5);
  EXPECT_EQ(s.status().phase, CalPhase::SETTLING);  // 0.5 < 1.0
  s.tick(11.0);
  EXPECT_EQ(s.status().phase, CalPhase::SAMPLING);  // 1.0 >= 1.0
}

TEST(CalibrationSequencer, MoveFailureAbortsWithoutReturnGoal) {
  CalibrationSequencer s;
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  s.tick(0.0);
  s.onMotionResult(false, 1.0);
  EXPECT_EQ(s.status().phase, CalPhase::FAILED);
  EXPECT_EQ(s.status().failure, CalFailure::MOVE_FAILED);
  EXPECT_FALSE(s.tick(2.0).send_goal);  // decision 6: no further motion
}

TEST(CalibrationSequencer, TooFewPosesFailsAtSolve) {
  CalibrationConfig cfg;
  cfg.poses = {CalPose{0, 0, 0}, CalPose{0, 45, 0}};  // n=2 < 4: unsolvable
  cfg.settle_time_s = 0.0;
  cfg.samples_per_pose = 1;
  CalibrationSequencer s;
  s.configure(cfg);
  double now = 0.0;
  ASSERT_TRUE(s.start(now));
  for (int i = 0; i < 2; ++i) {
    ASSERT_TRUE(s.tick(now).send_goal);
    s.onMotionResult(true, now);
    s.tick(now);  // settle_time 0: straight to SAMPLING
    s.onWrench(Wrench{});
  }
  EXPECT_EQ(s.status().phase, CalPhase::FAILED);
  EXPECT_EQ(s.status().failure, CalFailure::SOLVE_FAILED);
}

TEST(CalibrationSequencer, StreamLossAndCancelAbort) {
  CalibrationSequencer s;
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  s.tick(0.0);
  s.onStreamLost();
  EXPECT_EQ(s.status().failure, CalFailure::STREAM_LOST);

  ASSERT_TRUE(s.start(0.0));  // restart after failure is allowed
  s.tick(0.0);
  s.cancel();
  EXPECT_EQ(s.status().failure, CalFailure::CANCELLED);
  EXPECT_TRUE(s.start(0.0));  // restartable again
}

TEST(CalibrationSequencer, ReturnMoveFailureKeepsFit) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.0, 0.05);
  CalibrationSequencer s;
  s.configure(config8());
  double now = 0.0;
  ASSERT_TRUE(s.start(now));
  for (int i = 0; i < 8; ++i) {
    const CalPose goal = passMoveAndSettle(s, now);
    const Wrench w = synth(G, com, Wrench{}, goal.a, goal.b, goal.c);
    s.onWrench(w);
    s.onWrench(w);
  }
  ASSERT_EQ(s.status().phase, CalPhase::RETURNING);
  s.tick(now);
  s.onMotionResult(false, now);  // return move failed
  EXPECT_EQ(s.status().phase, CalPhase::DONE);  // fit is NOT discarded
  EXPECT_FALSE(s.status().return_move_ok);
  EXPECT_TRUE(s.result().ok);
  EXPECT_NEAR(s.result().params.gravity_n, G, 1e-8);
}

TEST(CalibrationSequencer, StartRejectsBadConfigAndDoubleStart) {
  CalibrationSequencer s;
  s.configure(CalibrationConfig{});  // no poses
  EXPECT_FALSE(s.start(0.0));
  s.configure(config8());
  ASSERT_TRUE(s.start(0.0));
  EXPECT_FALSE(s.start(0.0));  // already running
}
