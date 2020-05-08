#include "ctrl_bridge/msg_encoding.hpp"
#include <cstring>

namespace ctrl_bridge {

int SerialMessageEncode(
  const msgs::serial_message_t& message,
  uint8_t* const buffer,
  uint8_t* const length)
{
  if(message.payload == nullptr) return 1;
  if(buffer == nullptr) return 1;
  if(length == nullptr) return 1;
  
  const uint8_t msg_length = getMsgLength(message.mode);
  if(msg_length > msgs::SERIAL_MESSAGE_MAX_LENGTH) return 1;
  if(msg_length > *length) return 1;
  *length = msg_length;

  uint8_t* ptr = buffer;
  *ptr++ = *length;
  *ptr++ = message.mode;
  *ptr++ = message.flags;
  memcpy(ptr, &message.time, sizeof(uint64_t)); 
  ptr += sizeof(uint64_t);
  memcpy(ptr, &message.payload, getPayloadLength(message.mode));
  return 0;
}

int SerialMessageDecode(
  const uint8_t* const buffer,
  const uint8_t length,
  msgs::serial_message_t* const serial_message)
{
  if(buffer == nullptr) return 1;
  if(length < msgs::SERIAL_MESSAGE_MIN_LENGTH) return 1;
  if(serial_message == nullptr) return 1;
  
  const uint8_t* ptr = buffer;

  const uint8_t msg_length = *ptr++;

  if(msg_length < msgs::SERIAL_MESSAGE_MIN_LENGTH) return 1;

  serial_message->mode = *ptr++;
  serial_message->flags = *ptr++;

  memcpy(&serial_message->time, ptr, sizeof(uint64_t));
  ptr += sizeof(uint64_t);

  const uint8_t payload_length = getPayloadLength(serial_message->mode);
  const uint8_t msg_length_expected = getMsgLength(serial_message->mode);

  if(length < msg_length_expected) return 1;
  if(msg_length < msg_length_expected) return 1;
  
  memcpy(&serial_message->payload, ptr, payload_length);
  
  return 0;
}

int SerialContainerEncode(
  const msgs::serial_container_t* const serial_container,
  uint8_t* const buffer,
  uint8_t* const length)
{
  if (serial_container == nullptr) return 1;
  if (buffer == nullptr) return 1;
  if (length == NULL) return 1;
  
  if (serial_container->num_messages < 1) return 1;
  if (serial_container->num_messages > msgs::SERIAL_MESSAGE_MAX_COUNT) return 1;
  
  uint8_t container_buffer[msgs::SERIAL_CONTAINER_MAX_LENGTH];
  
  uint8_t pos = 1u;

  container_buffer[pos++] = serial_container->num_messages;

  for(int i = 0; i < serial_container->num_messages; ++i)
  {
    uint8_t len = *length - pos;
    const int ret = SerialMessageEncode(
      serial_container->messages[i],
      &container_buffer[pos], &len);
    if (ret) return ret;
    pos += len;
  }
  
  container_buffer[0] = pos;

  uint16_t checksum = 0;
  const int crcret = SerialContainerChecksum(container_buffer, pos, &checksum);
  if (crcret) return crcret;
  container_buffer[pos++] = checksum >> 8;
  container_buffer[pos++] = checksum;
  
  const int stuffret = SerialContainerStuffing(
    container_buffer, pos, buffer, length);
  if (stuffret) return stuffret;
  
  return 0;
}

int SerialContainerDecode(
  const uint8_t* const buffer,
  const uint8_t length,
  msgs::serial_container_t* const serial_container)
{
  if (buffer == nullptr) return 1;
  if (serial_container == nullptr) return 1;
  
  if (length == 0) return 1;
  
  uint8_t unstuffed_length = msgs::SERIAL_CONTAINER_MAX_LENGTH;
  uint8_t container_buffer[msgs::SERIAL_CONTAINER_MAX_LENGTH];
  
  const int stuffret = SerialContainerUnstuffing(
    buffer, length, container_buffer, &unstuffed_length);
  if (stuffret) return stuffret;
  
  if (unstuffed_length - 2 != container_buffer[0]) return 1;
  
  // Compute checksum
  uint16_t checksum;
  const int crcret = SerialContainerChecksum(
    container_buffer, unstuffed_length, &checksum);
  if (crcret) return crcret;

  // Compare checksum
  if(checksum != 0) return 1;

  // Read all messags    
  serial_container->num_messages = container_buffer[1];
  
  if (serial_container->num_messages < 1) return 1;
  if (serial_container->num_messages > msgs::SERIAL_MESSAGE_MAX_COUNT) return 1;
  
  uint8_t pos = 2u;
  
  for(int i = 0; i < serial_container->num_messages; ++i)
  {
    uint8_t msg_length = container_buffer[pos];
    uint8_t len = length - pos;
    const int ret = SerialMessageDecode(&container_buffer[pos], len, &serial_container->messages[i]);
    if(ret) return ret;
    pos += msg_length;
  }
  
  return 0;
}

int SerialContainerStuffing(
  const uint8_t* const src,
  const uint8_t src_len,
  uint8_t* const dst,
  uint8_t* const dst_len)
{
  if(src == nullptr) return 1;
  if(dst == nullptr) return 1;
  if(dst_len == nullptr) return 1;
  
  if (src_len == 0) return 1;
  if (*dst_len < src_len + 1) return 1;
  
  const uint8_t* src_ptr = src;
  uint8_t* dst_ptr = dst;

  const uint8_t* const end = src + src_len;
  uint8_t* last = dst_ptr++;
  uint8_t count = 1u;
  
  do{
    if(*src_ptr != msgs::SERIAL_CONTAINER_DELIMITER) {
      *dst_ptr++ = *src_ptr;
      ++count;
    }else {
      if (count == msgs::SERIAL_CONTAINER_DELIMITER)
        count = 0;
      *last = count;
      last = dst_ptr++;
      count = 1u;
    }
  }while(++src_ptr < end);
  if (count == msgs::SERIAL_CONTAINER_DELIMITER)
    count = 0;
  *last = count;
  *dst_len = src_len + 1;
  
  return 0;
}

int SerialContainerUnstuffing(
  const uint8_t* const src,
  const uint8_t src_len,
  uint8_t* const dst,
  uint8_t* const dst_len)
{
  if(src == nullptr) return 1;
  if(dst == nullptr) return 1;
  if(dst_len == nullptr) return 1;
  
  if (src_len == 0) return 1;
  if (*dst_len < src_len - 1) return 1;
  
  const uint8_t* src_ptr = src;
  uint8_t* dst_ptr = dst;
  
  const uint8_t* end = src + src_len;
  uint8_t count = *src_ptr++;
  if (count == 0)
    count = msgs::SERIAL_CONTAINER_DELIMITER;
  --count;
  
  do {
    if (*src_ptr == msgs::SERIAL_CONTAINER_DELIMITER) return 1;
    if (count != 0) {
      *dst_ptr++ = *src_ptr;
      --count;
    } else {
      count = *src_ptr;
      *dst_ptr++ = msgs::SERIAL_CONTAINER_DELIMITER;
      if (count == 0)
        count = msgs::SERIAL_CONTAINER_DELIMITER;
      --count;
    }
  } while(++src_ptr < end);
  *dst_len = src_len - 1;
  
  if (count != 0) return 1;
  
  return 0;
}

static const uint16_t crc16_ccitt_table[256] = {
  0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
  0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
  0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
  0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
  0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
  0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
  0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
  0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
  0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
  0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
  0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
  0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
  0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
  0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
  0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
  0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
  0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
  0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
  0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
  0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
  0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
  0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
  0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
  0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
  0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
  0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
  0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
  0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
  0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
  0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
  0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
  0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U
};

int SerialContainerChecksum(
  const uint8_t* const buffer,
  const uint8_t length,
  uint16_t* const checksum)
{
  if (buffer == nullptr) return 1;
  if (checksum == nullptr) return 1;
  
  if (length == 0) return 1;
  
  const uint8_t* ptr = buffer;
  const uint8_t* const end = buffer + length;
  
  *checksum = 0xFFFF;
  do {
      const uint8_t index = (*checksum >> 8) ^ *ptr;
      *checksum = (*checksum << 8) ^ crc16_ccitt_table[index];
  } while(++ptr < end);
  
  return 0;
}

} // namespace ctrl_bridge