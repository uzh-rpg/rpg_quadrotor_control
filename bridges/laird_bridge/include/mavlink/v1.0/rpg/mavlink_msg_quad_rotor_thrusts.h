// MESSAGE QUAD_ROTOR_THRUSTS PACKING

#define MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS 151

typedef struct __mavlink_quad_rotor_thrusts_t
{
 uint64_t timestamp; ///<  Time stamp [microseconds].
 float thrust_1; ///< Thrust of Rotor 1 [N].
 float thrust_2; ///< Thrust of Rotor 2 [N].
 float thrust_3; ///< Thrust of Rotor 3 [N].
 float thrust_4; ///< Thrust of Rotor 4 [N].
} mavlink_quad_rotor_thrusts_t;

#define MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN 24
#define MAVLINK_MSG_ID_151_LEN 24

#define MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_CRC 44
#define MAVLINK_MSG_ID_151_CRC 44



#define MAVLINK_MESSAGE_INFO_QUAD_ROTOR_THRUSTS { \
	"QUAD_ROTOR_THRUSTS", \
	5, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_quad_rotor_thrusts_t, timestamp) }, \
         { "thrust_1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_quad_rotor_thrusts_t, thrust_1) }, \
         { "thrust_2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_quad_rotor_thrusts_t, thrust_2) }, \
         { "thrust_3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_quad_rotor_thrusts_t, thrust_3) }, \
         { "thrust_4", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_quad_rotor_thrusts_t, thrust_4) }, \
         } \
}


/**
 * @brief Pack a quad_rotor_thrusts message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param thrust_1 Thrust of Rotor 1 [N].
 * @param thrust_2 Thrust of Rotor 2 [N].
 * @param thrust_3 Thrust of Rotor 3 [N].
 * @param thrust_4 Thrust of Rotor 4 [N].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quad_rotor_thrusts_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float thrust_1, float thrust_2, float thrust_3, float thrust_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, thrust_1);
	_mav_put_float(buf, 12, thrust_2);
	_mav_put_float(buf, 16, thrust_3);
	_mav_put_float(buf, 20, thrust_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#else
	mavlink_quad_rotor_thrusts_t packet;
	packet.timestamp = timestamp;
	packet.thrust_1 = thrust_1;
	packet.thrust_2 = thrust_2;
	packet.thrust_3 = thrust_3;
	packet.thrust_4 = thrust_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#endif
}

/**
 * @brief Pack a quad_rotor_thrusts message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param thrust_1 Thrust of Rotor 1 [N].
 * @param thrust_2 Thrust of Rotor 2 [N].
 * @param thrust_3 Thrust of Rotor 3 [N].
 * @param thrust_4 Thrust of Rotor 4 [N].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quad_rotor_thrusts_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float thrust_1,float thrust_2,float thrust_3,float thrust_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, thrust_1);
	_mav_put_float(buf, 12, thrust_2);
	_mav_put_float(buf, 16, thrust_3);
	_mav_put_float(buf, 20, thrust_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#else
	mavlink_quad_rotor_thrusts_t packet;
	packet.timestamp = timestamp;
	packet.thrust_1 = thrust_1;
	packet.thrust_2 = thrust_2;
	packet.thrust_3 = thrust_3;
	packet.thrust_4 = thrust_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#endif
}

/**
 * @brief Encode a quad_rotor_thrusts struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param quad_rotor_thrusts C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quad_rotor_thrusts_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_quad_rotor_thrusts_t* quad_rotor_thrusts)
{
	return mavlink_msg_quad_rotor_thrusts_pack(system_id, component_id, msg, quad_rotor_thrusts->timestamp, quad_rotor_thrusts->thrust_1, quad_rotor_thrusts->thrust_2, quad_rotor_thrusts->thrust_3, quad_rotor_thrusts->thrust_4);
}

/**
 * @brief Encode a quad_rotor_thrusts struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param quad_rotor_thrusts C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quad_rotor_thrusts_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_quad_rotor_thrusts_t* quad_rotor_thrusts)
{
	return mavlink_msg_quad_rotor_thrusts_pack_chan(system_id, component_id, chan, msg, quad_rotor_thrusts->timestamp, quad_rotor_thrusts->thrust_1, quad_rotor_thrusts->thrust_2, quad_rotor_thrusts->thrust_3, quad_rotor_thrusts->thrust_4);
}

/**
 * @brief Send a quad_rotor_thrusts message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param thrust_1 Thrust of Rotor 1 [N].
 * @param thrust_2 Thrust of Rotor 2 [N].
 * @param thrust_3 Thrust of Rotor 3 [N].
 * @param thrust_4 Thrust of Rotor 4 [N].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_quad_rotor_thrusts_send(mavlink_channel_t chan, uint64_t timestamp, float thrust_1, float thrust_2, float thrust_3, float thrust_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, thrust_1);
	_mav_put_float(buf, 12, thrust_2);
	_mav_put_float(buf, 16, thrust_3);
	_mav_put_float(buf, 20, thrust_4);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS, buf, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS, buf, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#endif
#else
	mavlink_quad_rotor_thrusts_t packet;
	packet.timestamp = timestamp;
	packet.thrust_1 = thrust_1;
	packet.thrust_2 = thrust_2;
	packet.thrust_3 = thrust_3;
	packet.thrust_4 = thrust_4;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS, (const char *)&packet, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS, (const char *)&packet, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_quad_rotor_thrusts_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float thrust_1, float thrust_2, float thrust_3, float thrust_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, thrust_1);
	_mav_put_float(buf, 12, thrust_2);
	_mav_put_float(buf, 16, thrust_3);
	_mav_put_float(buf, 20, thrust_4);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS, buf, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS, buf, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#endif
#else
	mavlink_quad_rotor_thrusts_t *packet = (mavlink_quad_rotor_thrusts_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->thrust_1 = thrust_1;
	packet->thrust_2 = thrust_2;
	packet->thrust_3 = thrust_3;
	packet->thrust_4 = thrust_4;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS, (const char *)packet, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS, (const char *)packet, MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE QUAD_ROTOR_THRUSTS UNPACKING


/**
 * @brief Get field timestamp from quad_rotor_thrusts message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_quad_rotor_thrusts_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field thrust_1 from quad_rotor_thrusts message
 *
 * @return Thrust of Rotor 1 [N].
 */
static inline float mavlink_msg_quad_rotor_thrusts_get_thrust_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field thrust_2 from quad_rotor_thrusts message
 *
 * @return Thrust of Rotor 2 [N].
 */
static inline float mavlink_msg_quad_rotor_thrusts_get_thrust_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field thrust_3 from quad_rotor_thrusts message
 *
 * @return Thrust of Rotor 3 [N].
 */
static inline float mavlink_msg_quad_rotor_thrusts_get_thrust_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field thrust_4 from quad_rotor_thrusts message
 *
 * @return Thrust of Rotor 4 [N].
 */
static inline float mavlink_msg_quad_rotor_thrusts_get_thrust_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a quad_rotor_thrusts message into a struct
 *
 * @param msg The message to decode
 * @param quad_rotor_thrusts C-struct to decode the message contents into
 */
static inline void mavlink_msg_quad_rotor_thrusts_decode(const mavlink_message_t* msg, mavlink_quad_rotor_thrusts_t* quad_rotor_thrusts)
{
#if MAVLINK_NEED_BYTE_SWAP
	quad_rotor_thrusts->timestamp = mavlink_msg_quad_rotor_thrusts_get_timestamp(msg);
	quad_rotor_thrusts->thrust_1 = mavlink_msg_quad_rotor_thrusts_get_thrust_1(msg);
	quad_rotor_thrusts->thrust_2 = mavlink_msg_quad_rotor_thrusts_get_thrust_2(msg);
	quad_rotor_thrusts->thrust_3 = mavlink_msg_quad_rotor_thrusts_get_thrust_3(msg);
	quad_rotor_thrusts->thrust_4 = mavlink_msg_quad_rotor_thrusts_get_thrust_4(msg);
#else
	memcpy(quad_rotor_thrusts, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS_LEN);
#endif
}
