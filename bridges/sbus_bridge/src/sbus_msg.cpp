#include "sbus_bridge/sbus_msg.h"

namespace sbus_bridge
{

SBusMsg::SBusMsg() :
    timestamp(ros::Time::now()), digital_channel_1(false), digital_channel_2(false), frame_lost(false), failsafe(false)
{
  for (int i = 0; i < kSbusNChannels; i++)
  {
    channels[i] = kSbusMeanCmd;
  }
}

SBusMsg::SBusMsg(const sbus_bridge::SbusRosMessage& sbus_ros_msg)
{
  timestamp = sbus_ros_msg.header.stamp;
  for (uint8_t i; i < kSbusNChannels; i++)
  {
    channels[i] = sbus_ros_msg.channels[i];
  }
  digital_channel_1 = sbus_ros_msg.digital_channel_1;
  digital_channel_2 = sbus_ros_msg.digital_channel_2;
  frame_lost = sbus_ros_msg.frame_lost;
  failsafe = sbus_ros_msg.failsafe;
}

SBusMsg::~SBusMsg()
{
}

sbus_bridge::SbusRosMessage SBusMsg::toRosMessage() const
{
  sbus_bridge::SbusRosMessage sbus_ros_msg;
  sbus_ros_msg.header.stamp = timestamp;
  for (uint8_t i; i < kSbusNChannels; i++)
  {
    sbus_ros_msg.channels[i] = channels[i];
  }
  sbus_ros_msg.digital_channel_1 = digital_channel_1;
  sbus_ros_msg.digital_channel_2 = digital_channel_2;
  sbus_ros_msg.frame_lost = frame_lost;
  sbus_ros_msg.failsafe = failsafe;

  return sbus_ros_msg;
}

} // namespace sbus_bridge
