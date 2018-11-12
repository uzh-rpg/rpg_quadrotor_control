// MESSAGE RPG_DESIRED_TORQUES_AND_THRUST PACKING

#define MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST 155

typedef struct __mavlink_rpg_desired_torques_and_thrust_t
{
 uint64_t timestamp; ///<  Time stamp [microseconds].
 float roll_torque; ///< Roll torque [Nm].
 float pitch_torque; ///< Pitch torque [Nm].
 float yaw_torque; ///< Yaw rate torque [Nm].
 float normalized_thrust; ///< Normalized thrust [N].
} mavlink_rpg_desired_torques_and_thrust_t;

#define MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN 24
#define MAVLINK_MSG_ID_155_LEN 24

#define MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_CRC 124
#define MAVLINK_MSG_ID_155_CRC 124



#define MAVLINK_MESSAGE_INFO_RPG_DESIRED_TORQUES_AND_THRUST { \
	"RPG_DESIRED_TORQUES_AND_THRUST", \
	5, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rpg_desired_torques_and_thrust_t, timestamp) }, \
		 { "roll_torque", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rpg_desired_torques_and_thrust_t, roll_torque) }, \
		 { "pitch_torque", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rpg_desired_torques_and_thrust_t, pitch_torque) }, \
		 { "yaw_torque", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rpg_desired_torques_and_thrust_t, yaw_torque) }, \
		 { "normalized_thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rpg_desired_torques_and_thrust_t, normalized_thrust) }, \
	} \
}


/**
 * @brief Pack a rpg_desired_torques_and_thrust message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param roll_torque Roll torque [Nm].
 * @param pitch_torque Pitch torque [Nm].
 * @param yaw_torque Yaw rate torque [Nm].
 * @param normalized_thrust Normalized thrust [N].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpg_desired_torques_and_thrust_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float roll_torque, float pitch_torque, float yaw_torque, float normalized_thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, roll_torque);
	_mav_put_float(buf, 12, pitch_torque);
	_mav_put_float(buf, 16, yaw_torque);
	_mav_put_float(buf, 20, normalized_thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#else
	mavlink_rpg_desired_torques_and_thrust_t packet;
	packet.timestamp = timestamp;
	packet.roll_torque = roll_torque;
	packet.pitch_torque = pitch_torque;
	packet.yaw_torque = yaw_torque;
	packet.normalized_thrust = normalized_thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#endif
}

/**
 * @brief Pack a rpg_desired_torques_and_thrust message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param roll_torque Roll torque [Nm].
 * @param pitch_torque Pitch torque [Nm].
 * @param yaw_torque Yaw rate torque [Nm].
 * @param normalized_thrust Normalized thrust [N].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpg_desired_torques_and_thrust_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float roll_torque,float pitch_torque,float yaw_torque,float normalized_thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, roll_torque);
	_mav_put_float(buf, 12, pitch_torque);
	_mav_put_float(buf, 16, yaw_torque);
	_mav_put_float(buf, 20, normalized_thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#else
	mavlink_rpg_desired_torques_and_thrust_t packet;
	packet.timestamp = timestamp;
	packet.roll_torque = roll_torque;
	packet.pitch_torque = pitch_torque;
	packet.yaw_torque = yaw_torque;
	packet.normalized_thrust = normalized_thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#endif
}

/**
 * @brief Encode a rpg_desired_torques_and_thrust struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rpg_desired_torques_and_thrust C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpg_desired_torques_and_thrust_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rpg_desired_torques_and_thrust_t* rpg_desired_torques_and_thrust)
{
	return mavlink_msg_rpg_desired_torques_and_thrust_pack(system_id, component_id, msg, rpg_desired_torques_and_thrust->timestamp, rpg_desired_torques_and_thrust->roll_torque, rpg_desired_torques_and_thrust->pitch_torque, rpg_desired_torques_and_thrust->yaw_torque, rpg_desired_torques_and_thrust->normalized_thrust);
}

/**
 * @brief Encode a rpg_desired_torques_and_thrust struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rpg_desired_torques_and_thrust C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpg_desired_torques_and_thrust_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rpg_desired_torques_and_thrust_t* rpg_desired_torques_and_thrust)
{
	return mavlink_msg_rpg_desired_torques_and_thrust_pack_chan(system_id, component_id, chan, msg, rpg_desired_torques_and_thrust->timestamp, rpg_desired_torques_and_thrust->roll_torque, rpg_desired_torques_and_thrust->pitch_torque, rpg_desired_torques_and_thrust->yaw_torque, rpg_desired_torques_and_thrust->normalized_thrust);
}

/**
 * @brief Send a rpg_desired_torques_and_thrust message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param roll_torque Roll torque [Nm].
 * @param pitch_torque Pitch torque [Nm].
 * @param yaw_torque Yaw rate torque [Nm].
 * @param normalized_thrust Normalized thrust [N].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rpg_desired_torques_and_thrust_send(mavlink_channel_t chan, uint64_t timestamp, float roll_torque, float pitch_torque, float yaw_torque, float normalized_thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, roll_torque);
	_mav_put_float(buf, 12, pitch_torque);
	_mav_put_float(buf, 16, yaw_torque);
	_mav_put_float(buf, 20, normalized_thrust);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST, buf, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST, buf, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#endif
#else
	mavlink_rpg_desired_torques_and_thrust_t packet;
	packet.timestamp = timestamp;
	packet.roll_torque = roll_torque;
	packet.pitch_torque = pitch_torque;
	packet.yaw_torque = yaw_torque;
	packet.normalized_thrust = normalized_thrust;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST, (const char *)&packet, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST, (const char *)&packet, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rpg_desired_torques_and_thrust_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float roll_torque, float pitch_torque, float yaw_torque, float normalized_thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, roll_torque);
	_mav_put_float(buf, 12, pitch_torque);
	_mav_put_float(buf, 16, yaw_torque);
	_mav_put_float(buf, 20, normalized_thrust);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST, buf, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST, buf, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#endif
#else
	mavlink_rpg_desired_torques_and_thrust_t *packet = (mavlink_rpg_desired_torques_and_thrust_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->roll_torque = roll_torque;
	packet->pitch_torque = pitch_torque;
	packet->yaw_torque = yaw_torque;
	packet->normalized_thrust = normalized_thrust;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST, (const char *)packet, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST, (const char *)packet, MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RPG_DESIRED_TORQUES_AND_THRUST UNPACKING


/**
 * @brief Get field timestamp from rpg_desired_torques_and_thrust message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_rpg_desired_torques_and_thrust_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field roll_torque from rpg_desired_torques_and_thrust message
 *
 * @return Roll torque [Nm].
 */
static inline float mavlink_msg_rpg_desired_torques_and_thrust_get_roll_torque(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch_torque from rpg_desired_torques_and_thrust message
 *
 * @return Pitch torque [Nm].
 */
static inline float mavlink_msg_rpg_desired_torques_and_thrust_get_pitch_torque(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw_torque from rpg_desired_torques_and_thrust message
 *
 * @return Yaw rate torque [Nm].
 */
static inline float mavlink_msg_rpg_desired_torques_and_thrust_get_yaw_torque(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field normalized_thrust from rpg_desired_torques_and_thrust message
 *
 * @return Normalized thrust [N].
 */
static inline float mavlink_msg_rpg_desired_torques_and_thrust_get_normalized_thrust(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a rpg_desired_torques_and_thrust message into a struct
 *
 * @param msg The message to decode
 * @param rpg_desired_torques_and_thrust C-struct to decode the message contents into
 */
static inline void mavlink_msg_rpg_desired_torques_and_thrust_decode(const mavlink_message_t* msg, mavlink_rpg_desired_torques_and_thrust_t* rpg_desired_torques_and_thrust)
{
#if MAVLINK_NEED_BYTE_SWAP
	rpg_desired_torques_and_thrust->timestamp = mavlink_msg_rpg_desired_torques_and_thrust_get_timestamp(msg);
	rpg_desired_torques_and_thrust->roll_torque = mavlink_msg_rpg_desired_torques_and_thrust_get_roll_torque(msg);
	rpg_desired_torques_and_thrust->pitch_torque = mavlink_msg_rpg_desired_torques_and_thrust_get_pitch_torque(msg);
	rpg_desired_torques_and_thrust->yaw_torque = mavlink_msg_rpg_desired_torques_and_thrust_get_yaw_torque(msg);
	rpg_desired_torques_and_thrust->normalized_thrust = mavlink_msg_rpg_desired_torques_and_thrust_get_normalized_thrust(msg);
#else
	memcpy(rpg_desired_torques_and_thrust, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST_LEN);
#endif
}
