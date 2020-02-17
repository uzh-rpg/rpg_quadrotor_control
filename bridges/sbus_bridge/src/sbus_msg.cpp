#include "sbus_bridge/sbus_msg.h"

#include "sbus_bridge/channel_mapping.h"

namespace sbus_bridge {

SBusMsg::SBusMsg()
    : timestamp(ros::Time::now()),
      digital_channel_1(false),
      digital_channel_2(false),
      frame_lost(false),
      failsafe(false) {
  for (int i = 0; i < kNChannels; i++) {
    channels[i] = kMeanCmd;
  }
}

SBusMsg::SBusMsg(const sbus_bridge::SbusRosMessage& sbus_ros_msg) {
  timestamp = sbus_ros_msg.header.stamp;
  for (uint8_t i; i < kNChannels; i++) {
    channels[i] = sbus_ros_msg.channels[i];
  }
  digital_channel_1 = sbus_ros_msg.digital_channel_1;
  digital_channel_2 = sbus_ros_msg.digital_channel_2;
  frame_lost = sbus_ros_msg.frame_lost;
  failsafe = sbus_ros_msg.failsafe;
}

SBusMsg::~SBusMsg() {}

sbus_bridge::SbusRosMessage SBusMsg::toRosMessage() const {
  sbus_bridge::SbusRosMessage sbus_ros_msg;
  sbus_ros_msg.header.stamp = timestamp;
  for (uint8_t i; i < kNChannels; i++) {
    sbus_ros_msg.channels[i] = channels[i];
  }
  sbus_ros_msg.digital_channel_1 = digital_channel_1;
  sbus_ros_msg.digital_channel_2 = digital_channel_2;
  sbus_ros_msg.frame_lost = frame_lost;
  sbus_ros_msg.failsafe = failsafe;

  return sbus_ros_msg;
}

void SBusMsg::limitAllChannelsFeasible() {
  for (uint8_t i = 0; i < kNChannels; i++) {
    limitSbusChannelFeasible(i);
  }
}

void SBusMsg::limitSbusChannelFeasible(const int channel_idx) {
  if (channel_idx < 0 || channel_idx >= kNChannels) {
    return;
  }

  if (channels[channel_idx] > kMaxCmd) {
    channels[channel_idx] = kMaxCmd;
  }
  if (channels[channel_idx] < kMinCmd) {
    channels[channel_idx] = kMinCmd;
  }
}

void SBusMsg::setThrottleCommand(const uint16_t throttle_cmd) {
  channels[channel_mapping::kThrottle] = throttle_cmd;
}

void SBusMsg::setRollCommand(const uint16_t roll_cmd) {
  channels[channel_mapping::kRoll] = roll_cmd;
}

void SBusMsg::setPitchCommand(const uint16_t pitch_cmd) {
  channels[channel_mapping::kPitch] = pitch_cmd;
}

void SBusMsg::setYawCommand(const uint16_t yaw_cmd) {
  channels[channel_mapping::kYaw] = yaw_cmd;
}

void SBusMsg::setControlMode(const ControlMode& control_mode) {
  if (control_mode == ControlMode::ATTITUDE) {
    channels[channel_mapping::kControlMode] = kMinCmd;
  } else if (control_mode == ControlMode::BODY_RATES) {
    channels[channel_mapping::kControlMode] = kMaxCmd;
  }
}

void SBusMsg::setControlModeAttitude() {
  setControlMode(ControlMode::ATTITUDE);
}

void SBusMsg::setControlModeBodyRates() {
  setControlMode(ControlMode::BODY_RATES);
}

void SBusMsg::setArmState(const ArmState& arm_state) {
  if (arm_state == ArmState::ARMED) {
    channels[channel_mapping::kArming] = kMaxCmd;
  } else {
    channels[channel_mapping::kArming] = kMinCmd;
  }
}

void SBusMsg::setArmStateArmed() { setArmState(ArmState::ARMED); }

void SBusMsg::setArmStateDisarmed() {
  setArmState(ArmState::DISARMED);
  // Should not be necessary but for safety we also set the throttle command
  // to the minimum
  setThrottleCommand(kMinCmd);
}

bool SBusMsg::isArmed() const {
  if (channels[channel_mapping::kArming] <= kMeanCmd) {
    return false;
  }

  return true;
}

ControlMode SBusMsg::getControlMode() const {
  if (channels[channel_mapping::kControlMode] > kMeanCmd) {
    return ControlMode::BODY_RATES;
  }

  return ControlMode::ATTITUDE;
}

}  // namespace sbus_bridge
