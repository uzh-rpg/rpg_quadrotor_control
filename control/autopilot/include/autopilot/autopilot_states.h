#pragma once

namespace autopilot {

enum class States {
  OFF,
  START,
  HOVER,
  LAND,
  EMERGENCY_LAND,
  BREAKING,
  GO_TO_POSE,
  VELOCITY_CONTROL,
  REFERENCE_CONTROL,
  TRAJECTORY_CONTROL,
  COMMAND_FEEDTHROUGH,
  RC_MANUAL
};

}  // namespace autopilot
