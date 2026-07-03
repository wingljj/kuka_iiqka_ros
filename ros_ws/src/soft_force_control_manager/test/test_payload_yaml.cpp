#include <gtest/gtest.h>

#include <string>

#include "soft_force_control_manager/payload_yaml.h"

TEST(PayloadYaml, EmitsControllerOverrideDocument) {
  sfc::PayloadFitResult fit;
  fit.ok = true;
  fit.params.gravity_n = 50.0;
  fit.params.com_x = 0.01;
  fit.params.com_y = 0.02;
  fit.params.com_z = 0.03;
  fit.params.bias.fx = 1.0;
  fit.params.bias.fy = -2.0;
  fit.params.bias.fz = 0.5;
  fit.params.bias.tx = 0.1;
  fit.params.bias.ty = -0.2;
  fit.params.bias.tz = 0.05;
  fit.r2_force = 1.0;
  fit.r2_torque = 0.999;

  const std::string expected =
      "# payload.yaml - written by soft_force_control_manager (spec 9).\n"
      "# Loaded by soft_robot_bringup at startup to override the\n"
      "# force_compliance_controller payload block. Do not edit by hand.\n"
      "force_compliance_controller:\n"
      "  payload:\n"
      "    gravity_n: 50.000000\n"
      "    com_x: 0.010000\n"
      "    com_y: 0.020000\n"
      "    com_z: 0.030000\n"
      "    bias_fx: 1.000000\n"
      "    bias_fy: -2.000000\n"
      "    bias_fz: 0.500000\n"
      "    bias_tx: 0.100000\n"
      "    bias_ty: -0.200000\n"
      "    bias_tz: 0.050000\n"
      "soft_robot_manager:\n"
      "  payload_fit:\n"
      "    r2_force: 1.000000\n"
      "    r2_torque: 0.999000\n"
      "    timestamp: \"2026-07-03T12:00:00\"\n";
  EXPECT_EQ(sfm::emitPayloadYaml(fit, "2026-07-03T12:00:00"), expected);
}
