#pragma once

#include <stdint.h>

#include "sbus_bridge/SbusRosMessage.h"

namespace sbus_bridge
{

static constexpr int kSbusNChannels = 16;
static constexpr uint16_t kSbusMinCmd = 192; // corresponds to 1000 on FC
static constexpr uint16_t kSbusMinSpinCmd = 288; // corresponds to 1060 on FC
static constexpr uint16_t kSbusMeanCmd = 992; // corresponds to 1500 on FC
static constexpr uint16_t kSbusMaxCmd = 1792; // corresponds to 2000 on FC

class SBusMsg
{
public:
  SBusMsg();
  SBusMsg(const sbus_bridge::SbusRosMessage& sbus_ros_msg);
  ~SBusMsg();

  sbus_bridge::SbusRosMessage toRosMessage() const;

  ros::Time timestamp;

  // Normal 11 bit channels
  uint16_t channels[kSbusNChannels];

  // Digital channels (ch17 and ch18)
  bool digital_channel_1;
  bool digital_channel_2;

  // Flags
  bool frame_lost;
  bool failsafe;
};

} // namespace sbus_bridge
