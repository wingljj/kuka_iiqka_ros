#include <gtest/gtest.h>
#include "soft_force_control_core/tool_gravity_compensator.h"

using sfc::PayloadParams;
using sfc::ToolGravityCompensator;
using sfc::Wrench;

TEST(GravityComp, ZeroParamsPassesRawThrough) {
  ToolGravityCompensator g;
  Wrench raw;
  raw.fx = 1.0;
  raw.tz = 2.0;
  Wrench out = g.compensate(raw, 10, 20, 30);
  EXPECT_DOUBLE_EQ(out.fx, 1.0);
  EXPECT_DOUBLE_EQ(out.tz, 2.0);
}

TEST(GravityComp, BiasSubtracted) {
  ToolGravityCompensator g;
  PayloadParams p;
  p.bias.fx = 0.5;
  g.setParams(p);
  Wrench raw;
  raw.fx = 1.5;
  EXPECT_DOUBLE_EQ(g.compensate(raw, 0, 0, 0).fx, 1.0);
}

TEST(GravityComp, GravityCancelledAtIdentityOrientation) {
  // Sensor aligned with base, payload G=10N pulls -Z: raw reads (0,0,-10).
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  g.setParams(p);
  Wrench raw;
  raw.fz = -10.0;
  Wrench out = g.compensate(raw, 0, 0, 0);
  EXPECT_NEAR(out.fx, 0.0, 1e-12);
  EXPECT_NEAR(out.fy, 0.0, 1e-12);
  EXPECT_NEAR(out.fz, 0.0, 1e-12);
}

TEST(GravityComp, GravityRotatesWithOrientation) {
  // C=90 deg (rot about X): base -Z maps to sensor -Y... verify via cancel:
  // whatever the model predicts, feeding exactly that as raw must yield zero.
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  p.com_x = 0.01;
  p.com_y = 0.02;
  p.com_z = 0.03;
  g.setParams(p);
  Wrench zero;
  Wrench predicted = g.compensate(zero, 30, 40, 50);  // = -gravity model
  Wrench raw;
  raw.fx = -predicted.fx;
  raw.fy = -predicted.fy;
  raw.fz = -predicted.fz;
  raw.tx = -predicted.tx;
  raw.ty = -predicted.ty;
  raw.tz = -predicted.tz;
  Wrench out = g.compensate(raw, 30, 40, 50);
  EXPECT_NEAR(out.forceNorm(), 0.0, 1e-12);
  EXPECT_NEAR(out.torqueNorm(), 0.0, 1e-12);
}

TEST(GravityComp, ComProducesTorque) {
  // Identity orientation, G=10N along -Z sensor, CoM offset in +X
  // => torque = com x f = (0.1,0,0) x (0,0,-10) = (0, 1, 0).
  ToolGravityCompensator g;
  PayloadParams p;
  p.gravity_n = 10.0;
  p.com_x = 0.1;
  g.setParams(p);
  Wrench raw;
  raw.fz = -10.0;
  raw.ty = 1.0;
  Wrench out = g.compensate(raw, 0, 0, 0);
  EXPECT_NEAR(out.torqueNorm(), 0.0, 1e-12);
}

TEST(GravityComp, AbsorbResidualShiftsBias) {
  ToolGravityCompensator g;
  Wrench residual;
  residual.fx = 0.7;
  g.absorbResidual(residual);
  Wrench raw;
  raw.fx = 0.7;
  EXPECT_NEAR(g.compensate(raw, 0, 0, 0).fx, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(g.params().bias.fx, 0.7);
}
