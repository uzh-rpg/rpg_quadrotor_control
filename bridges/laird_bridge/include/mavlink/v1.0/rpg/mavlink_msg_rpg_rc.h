// MESSAGE RPG_RC PACKING

#define MAVLINK_MSG_ID_RPG_RC 156

typedef struct __mavlink_rpg_rc_t
{
 uint64_t timestamp; ///<  Time stamp [microseconds].
 float channel_1; ///< Channel 1 input [].
 float channel_2; ///< Channel 2 input [].
 float channel_3; ///< Channel 3 input [].
 float channel_4; ///< Channel 4 input [].
 uint8_t switch_1; ///< Switch 1 [].
 uint8_t switch_2; ///< Switch 2 [].
 uint8_t switch_3; ///< Switch 2 [].
 uint8_t switch_4; ///< Switch 2 [].
} mavlink_rpg_rc_t;

#define MAVLINK_MSG_ID_RPG_RC_LEN 28
#define MAVLINK_MSG_ID_156_LEN 28

#define MAVLINK_MSG_ID_RPG_RC_CRC 116
#define MAVLINK_MSG_ID_156_CRC 116



#define MAVLINK_MESSAGE_INFO_RPG_RC { \
	"RPG_RC", \
	9, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rpg_rc_t, timestamp) }, \
         { "channel_1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rpg_rc_t, channel_1) }, \
         { "channel_2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rpg_rc_t, channel_2) }, \
         { "channel_3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rpg_rc_t, channel_3) }, \
         { "channel_4", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rpg_rc_t, channel_4) }, \
         { "switch_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_rpg_rc_t, switch_1) }, \
         { "switch_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_rpg_rc_t, switch_2) }, \
         { "switch_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_rpg_rc_t, switch_3) }, \
         { "switch_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_rpg_rc_t, switch_4) }, \
         } \
}


/**
 * @brief Pack a rpg_rc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param channel_1 Channel 1 input [].
 * @param channel_2 Channel 2 input [].
 * @param channel_3 Channel 3 input [].
 * @param channel_4 Channel 4 input [].
 * @param switch_1 Switch 1 [].
 * @param switch_2 Switch 2 [].
 * @param switch_3 Switch 2 [].
 * @param switch_4 Switch 2 [].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpg_rc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float channel_1, float channel_2, float channel_3, float channel_4, uint8_t switch_1, uint8_t switch_2, uint8_t switch_3, uint8_t switch_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPG_RC_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, channel_1);
	_mav_put_float(buf, 12, channel_2);
	_mav_put_float(buf, 16, channel_3);
	_mav_put_float(buf, 20, channel_4);
	_mav_put_uint8_t(buf, 24, switch_1);
	_mav_put_uint8_t(buf, 25, switch_2);
	_mav_put_uint8_t(buf, 26, switch_3);
	_mav_put_uint8_t(buf, 27, switch_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPG_RC_LEN);
#else
	mavlink_rpg_rc_t packet;
	packet.timestamp = timestamp;
	packet.channel_1 = channel_1;
	packet.channel_2 = channel_2;
	packet.channel_3 = channel_3;
	packet.channel_4 = channel_4;
	packet.switch_1 = switch_1;
	packet.switch_2 = switch_2;
	packet.switch_3 = switch_3;
	packet.switch_4 = switch_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPG_RC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RPG_RC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPG_RC_LEN, MAVLINK_MSG_ID_RPG_RC_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPG_RC_LEN);
#endif
}

/**
 * @brief Pack a rpg_rc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param channel_1 Channel 1 input [].
 * @param channel_2 Channel 2 input [].
 * @param channel_3 Channel 3 input [].
 * @param channel_4 Channel 4 input [].
 * @param switch_1 Switch 1 [].
 * @param switch_2 Switch 2 [].
 * @param switch_3 Switch 2 [].
 * @param switch_4 Switch 2 [].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpg_rc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float channel_1,float channel_2,float channel_3,float channel_4,uint8_t switch_1,uint8_t switch_2,uint8_t switch_3,uint8_t switch_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPG_RC_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, channel_1);
	_mav_put_float(buf, 12, channel_2);
	_mav_put_float(buf, 16, channel_3);
	_mav_put_float(buf, 20, channel_4);
	_mav_put_uint8_t(buf, 24, switch_1);
	_mav_put_uint8_t(buf, 25, switch_2);
	_mav_put_uint8_t(buf, 26, switch_3);
	_mav_put_uint8_t(buf, 27, switch_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPG_RC_LEN);
#else
	mavlink_rpg_rc_t packet;
	packet.timestamp = timestamp;
	packet.channel_1 = channel_1;
	packet.channel_2 = channel_2;
	packet.channel_3 = channel_3;
	packet.channel_4 = channel_4;
	packet.switch_1 = switch_1;
	packet.switch_2 = switch_2;
	packet.switch_3 = switch_3;
	packet.switch_4 = switch_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPG_RC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RPG_RC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPG_RC_LEN, MAVLINK_MSG_ID_RPG_RC_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPG_RC_LEN);
#endif
}

/**
 * @brief Encode a rpg_rc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rpg_rc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpg_rc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rpg_rc_t* rpg_rc)
{
	return mavlink_msg_rpg_rc_pack(system_id, component_id, msg, rpg_rc->timestamp, rpg_rc->channel_1, rpg_rc->channel_2, rpg_rc->channel_3, rpg_rc->channel_4, rpg_rc->switch_1, rpg_rc->switch_2, rpg_rc->switch_3, rpg_rc->switch_4);
}

/**
 * @brief Encode a rpg_rc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rpg_rc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpg_rc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rpg_rc_t* rpg_rc)
{
	return mavlink_msg_rpg_rc_pack_chan(system_id, component_id, chan, msg, rpg_rc->timestamp, rpg_rc->channel_1, rpg_rc->channel_2, rpg_rc->channel_3, rpg_rc->channel_4, rpg_rc->switch_1, rpg_rc->switch_2, rpg_rc->switch_3, rpg_rc->switch_4);
}

/**
 * @brief Send a rpg_rc message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param channel_1 Channel 1 input [].
 * @param channel_2 Channel 2 input [].
 * @param channel_3 Channel 3 input [].
 * @param channel_4 Channel 4 input [].
 * @param switch_1 Switch 1 [].
 * @param switch_2 Switch 2 [].
 * @param switch_3 Switch 2 [].
 * @param switch_4 Switch 2 [].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rpg_rc_send(mavlink_channel_t chan, uint64_t timestamp, float channel_1, float channel_2, float channel_3, float channel_4, uint8_t switch_1, uint8_t switch_2, uint8_t switch_3, uint8_t switch_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPG_RC_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, channel_1);
	_mav_put_float(buf, 12, channel_2);
	_mav_put_float(buf, 16, channel_3);
	_mav_put_float(buf, 20, channel_4);
	_mav_put_uint8_t(buf, 24, switch_1);
	_mav_put_uint8_t(buf, 25, switch_2);
	_mav_put_uint8_t(buf, 26, switch_3);
	_mav_put_uint8_t(buf, 27, switch_4);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_RC, buf, MAVLINK_MSG_ID_RPG_RC_LEN, MAVLINK_MSG_ID_RPG_RC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_RC, buf, MAVLINK_MSG_ID_RPG_RC_LEN);
#endif
#else
	mavlink_rpg_rc_t packet;
	packet.timestamp = timestamp;
	packet.channel_1 = channel_1;
	packet.channel_2 = channel_2;
	packet.channel_3 = channel_3;
	packet.channel_4 = channel_4;
	packet.switch_1 = switch_1;
	packet.switch_2 = switch_2;
	packet.switch_3 = switch_3;
	packet.switch_4 = switch_4;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_RC, (const char *)&packet, MAVLINK_MSG_ID_RPG_RC_LEN, MAVLINK_MSG_ID_RPG_RC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_RC, (const char *)&packet, MAVLINK_MSG_ID_RPG_RC_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RPG_RC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rpg_rc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float channel_1, float channel_2, float channel_3, float channel_4, uint8_t switch_1, uint8_t switch_2, uint8_t switch_3, uint8_t switch_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, channel_1);
	_mav_put_float(buf, 12, channel_2);
	_mav_put_float(buf, 16, channel_3);
	_mav_put_float(buf, 20, channel_4);
	_mav_put_uint8_t(buf, 24, switch_1);
	_mav_put_uint8_t(buf, 25, switch_2);
	_mav_put_uint8_t(buf, 26, switch_3);
	_mav_put_uint8_t(buf, 27, switch_4);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_RC, buf, MAVLINK_MSG_ID_RPG_RC_LEN, MAVLINK_MSG_ID_RPG_RC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_RC, buf, MAVLINK_MSG_ID_RPG_RC_LEN);
#endif
#else
	mavlink_rpg_rc_t *packet = (mavlink_rpg_rc_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->channel_1 = channel_1;
	packet->channel_2 = channel_2;
	packet->channel_3 = channel_3;
	packet->channel_4 = channel_4;
	packet->switch_1 = switch_1;
	packet->switch_2 = switch_2;
	packet->switch_3 = switch_3;
	packet->switch_4 = switch_4;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_RC, (const char *)packet, MAVLINK_MSG_ID_RPG_RC_LEN, MAVLINK_MSG_ID_RPG_RC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_RC, (const char *)packet, MAVLINK_MSG_ID_RPG_RC_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RPG_RC UNPACKING


/**
 * @brief Get field timestamp from rpg_rc message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_rpg_rc_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field channel_1 from rpg_rc message
 *
 * @return Channel 1 input [].
 */
static inline float mavlink_msg_rpg_rc_get_channel_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field channel_2 from rpg_rc message
 *
 * @return Channel 2 input [].
 */
static inline float mavlink_msg_rpg_rc_get_channel_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field channel_3 from rpg_rc message
 *
 * @return Channel 3 input [].
 */
static inline float mavlink_msg_rpg_rc_get_channel_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field channel_4 from rpg_rc message
 *
 * @return Channel 4 input [].
 */
static inline float mavlink_msg_rpg_rc_get_channel_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field switch_1 from rpg_rc message
 *
 * @return Switch 1 [].
 */
static inline uint8_t mavlink_msg_rpg_rc_get_switch_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field switch_2 from rpg_rc message
 *
 * @return Switch 2 [].
 */
static inline uint8_t mavlink_msg_rpg_rc_get_switch_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field switch_3 from rpg_rc message
 *
 * @return Switch 2 [].
 */
static inline uint8_t mavlink_msg_rpg_rc_get_switch_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field switch_4 from rpg_rc message
 *
 * @return Switch 2 [].
 */
static inline uint8_t mavlink_msg_rpg_rc_get_switch_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Decode a rpg_rc message into a struct
 *
 * @param msg The message to decode
 * @param rpg_rc C-struct to decode the message contents into
 */
static inline void mavlink_msg_rpg_rc_decode(const mavlink_message_t* msg, mavlink_rpg_rc_t* rpg_rc)
{
#if MAVLINK_NEED_BYTE_SWAP
	rpg_rc->timestamp = mavlink_msg_rpg_rc_get_timestamp(msg);
	rpg_rc->channel_1 = mavlink_msg_rpg_rc_get_channel_1(msg);
	rpg_rc->channel_2 = mavlink_msg_rpg_rc_get_channel_2(msg);
	rpg_rc->channel_3 = mavlink_msg_rpg_rc_get_channel_3(msg);
	rpg_rc->channel_4 = mavlink_msg_rpg_rc_get_channel_4(msg);
	rpg_rc->switch_1 = mavlink_msg_rpg_rc_get_switch_1(msg);
	rpg_rc->switch_2 = mavlink_msg_rpg_rc_get_switch_2(msg);
	rpg_rc->switch_3 = mavlink_msg_rpg_rc_get_switch_3(msg);
	rpg_rc->switch_4 = mavlink_msg_rpg_rc_get_switch_4(msg);
#else
	memcpy(rpg_rc, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RPG_RC_LEN);
#endif
}
