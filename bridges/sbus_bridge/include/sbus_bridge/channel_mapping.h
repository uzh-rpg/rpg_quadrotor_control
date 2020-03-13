#pragma once

namespace sbus_bridge {

namespace channel_mapping {

static constexpr uint8_t kThrottle = 0;
static constexpr uint8_t kRoll = 1;
static constexpr uint8_t kPitch = 2;
static constexpr uint8_t kYaw = 3;
static constexpr uint8_t kArming = 4;
static constexpr uint8_t kControlMode = 5;
static constexpr uint8_t kGamepadMode = 6;

}  // namespace channel_mapping

}  // namespace sbus_bridge
