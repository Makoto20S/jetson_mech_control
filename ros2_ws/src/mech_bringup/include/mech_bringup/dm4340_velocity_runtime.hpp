#pragma once

// Compatibility names for earlier bench integrations. New code uses the
// mode-neutral runtime, shared by velocity and effort deployments.
#include "mech_bringup/dm4340_runtime.hpp"

namespace mech::mech_bringup {
using Dm4340VelocityRuntime = Dm4340Runtime;
using Dm4340VelocityConfig = Dm4340RuntimeConfig;
}  // namespace mech::mech_bringup
