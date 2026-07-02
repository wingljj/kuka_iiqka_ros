#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "soft_force_control_core/payload_estimator.h"
#include "soft_force_control_core/rotation.h"

using sfc::PayloadEstimator;
using sfc::PayloadFitResult;
using sfc::Wrench;

namespace {
// Synthesize the exact wrench a sensor would read for payload (G, com, bias)
// at orientation (a, b, c): raw = bias + gravity terms.
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

// Legacy-style 8-orientation calibration set (varied A/B/C).
const double kPoses[8][3] = {{0, 0, 0},    {0, 45, 0},  {0, -45, 0},
                             {0, 0, 45},   {0, 0, -45}, {45, 30, 0},
                             {-45, 0, 30}, {30, -30, 30}};
}  // namespace

TEST(PayloadEstimator, NotOkWithTooFewSamples) {
  PayloadEstimator e;
  e.addSample(0, 0, 0, Wrench{});
  e.addSample(0, 45, 0, Wrench{});
  EXPECT_FALSE(e.solve().ok);
}

TEST(PayloadEstimator, RecoversSyntheticPayloadExactly) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.02, 0.03);
  Wrench bias;
  bias.fx = 1.0;
  bias.fy = -2.0;
  bias.fz = 0.5;
  bias.tx = 0.1;
  bias.ty = -0.2;
  bias.tz = 0.05;

  PayloadEstimator e;
  for (const auto& p : kPoses) {
    e.addSample(p[0], p[1], p[2], synth(G, com, bias, p[0], p[1], p[2]));
  }
  PayloadFitResult r = e.solve();
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

TEST(PayloadEstimator, NoisyDataReducesR2ButStillFits) {
  const double G = 50.0;
  const Eigen::Vector3d com(0.01, 0.0, 0.05);
  PayloadEstimator e;
  int sign = 1;
  for (const auto& p : kPoses) {
    Wrench w = synth(G, com, Wrench{}, p[0], p[1], p[2]);
    w.fx += 0.5 * sign;  // deterministic "noise"
    w.tx += 0.01 * sign;
    sign = -sign;
    e.addSample(p[0], p[1], p[2], w);
  }
  PayloadFitResult r = e.solve();
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.params.gravity_n, G, 1.0);
  EXPECT_LT(r.r2_force, 1.0);
  EXPECT_GT(r.r2_force, 0.99);
}

TEST(PayloadEstimator, ClearResetsSamples) {
  PayloadEstimator e;
  for (const auto& p : kPoses) e.addSample(p[0], p[1], p[2], Wrench{});
  EXPECT_EQ(e.sampleCount(), 8u);
  e.clear();
  EXPECT_EQ(e.sampleCount(), 0u);
  EXPECT_FALSE(e.solve().ok);
}
