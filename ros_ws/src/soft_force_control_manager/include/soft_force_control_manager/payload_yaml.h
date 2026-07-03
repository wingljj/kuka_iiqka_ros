#pragma once

#include <string>

#include "soft_force_control_core/payload_estimator.h"

namespace sfm {

// Renders payload.yaml (spec sections 9, 14): a rosparam-loadable override
// of the force_compliance_controller payload block plus fit metadata.
// Pure text generation; the caller owns file I/O and the timestamp.
std::string emitPayloadYaml(const sfc::PayloadFitResult& fit,
                            const std::string& timestamp);

}  // namespace sfm
