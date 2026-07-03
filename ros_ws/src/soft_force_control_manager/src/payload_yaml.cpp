#include "soft_force_control_manager/payload_yaml.h"

#include <cstdio>

namespace sfm {

namespace {
void line(std::string& out, const char* key, double v) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "    %s: %.6f\n", key, v);
  out += buf;
}
}  // namespace

std::string emitPayloadYaml(const sfc::PayloadFitResult& fit,
                            const std::string& timestamp) {
  std::string out;
  out.reserve(768);
  out +=
      "# payload.yaml - written by soft_force_control_manager (spec 9).\n"
      "# Loaded by soft_robot_bringup at startup to override the\n"
      "# force_compliance_controller payload block. Do not edit by hand.\n"
      "force_compliance_controller:\n"
      "  payload:\n";
  line(out, "gravity_n", fit.params.gravity_n);
  line(out, "com_x", fit.params.com_x);
  line(out, "com_y", fit.params.com_y);
  line(out, "com_z", fit.params.com_z);
  line(out, "bias_fx", fit.params.bias.fx);
  line(out, "bias_fy", fit.params.bias.fy);
  line(out, "bias_fz", fit.params.bias.fz);
  line(out, "bias_tx", fit.params.bias.tx);
  line(out, "bias_ty", fit.params.bias.ty);
  line(out, "bias_tz", fit.params.bias.tz);
  out += "soft_robot_manager:\n  payload_fit:\n";
  line(out, "r2_force", fit.r2_force);
  line(out, "r2_torque", fit.r2_torque);
  out += "    timestamp: \"" + timestamp + "\"\n";
  return out;
}

}  // namespace sfm
