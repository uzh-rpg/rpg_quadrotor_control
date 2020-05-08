#pragma once

#include "ctrl_bridge/msgs_defs.hpp"

namespace ctrl_bridge {

int SerialContainerChecksum(
  const uint8_t* const buffer,
  const uint8_t length,
  uint16_t* const checksum);

int SerialMessageEncode(
  const msgs::serial_message_t& message,
  uint8_t* const buffer,
  uint8_t* const length);

int SerialMessageDecode(
  const uint8_t* const buffer,
  const uint8_t length,
  msgs::serial_message_t* const serial_message);

int SerialContainerEncode(
  const msgs::serial_container_t* const serial_container,
  uint8_t* const buffer,
  uint8_t* const length);

int SerialContainerDecode(
  const uint8_t* const buffer,
  const uint8_t length,
  msgs::serial_container_t* serial_container);

int SerialContainerStuffing(
  const uint8_t* const src,
  const uint8_t src_len,
  uint8_t* const dst,
  uint8_t* const dst_len);

int SerialContainerUnstuffing(
  const uint8_t* const src,
  const uint8_t src_len,
  uint8_t* const dst,
  uint8_t* const dst_len);

static uint8_t getPayloadLength(const msgs::CTRLMODE mode)
{
  switch(mode)
  {
    case msgs::CTRLMODE::OFF:
    case msgs::CTRLMODE::ARM:
    case msgs::CTRLMODE::EMERGENCY:
      return 0u;
    case msgs::CTRLMODE::ROTOR_THROTTLE:
      return sizeof(msgs::rotor_throttle_payload_t);
    case msgs::CTRLMODE::ROTOR_SPEED:
      return sizeof(msgs::rotor_speed_payload_t);
    case msgs::CTRLMODE::ROTOR_THRUST:
      return sizeof(msgs::rotor_thrust_payload_t);
    case msgs::CTRLMODE::BODY_RATE:
      return sizeof(msgs::body_rate_payload_t);
    case msgs::CTRLMODE::ATTITUDE:
      return sizeof(msgs::attitude_payload_t);
    case msgs::CTRLMODE::VELOCITY:
      return sizeof(msgs::velocity_payload_t);
    case msgs::CTRLMODE::SET_PARAM:
      return sizeof(msgs::parameter_payload_t);
    case msgs::CTRLMODE::FEEDBACK:
      return sizeof(msgs::upstream_payload_t);
    default:
      return 0u;
  }
}

static uint8_t getPayloadLength(const uint8_t mode)
{
  return getPayloadLength((msgs::CTRLMODE)mode);
}


static uint8_t getMsgLength(const msgs::CTRLMODE mode)
{
  return 1 + msgs::SERIAL_MESSAGE_HEADER_LENGTH + getPayloadLength(mode);
}

static uint8_t getMsgLength(const uint8_t mode)
{
  return getMsgLength((msgs::CTRLMODE)mode);
}
}