// MESSAGE RESET_ATTITUDE_INTEGRATOR PACKING

#define MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR 154

typedef struct __mavlink_reset_attitude_integrator_t
{
 uint64_t timestamp; ///<  Time stamp [microseconds].
} mavlink_reset_attitude_integrator_t;

#define MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN 8
#define MAVLINK_MSG_ID_154_LEN 8

#define MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_CRC 242
#define MAVLINK_MSG_ID_154_CRC 242



#define MAVLINK_MESSAGE_INFO_RESET_ATTITUDE_INTEGRATOR { \
	"RESET_ATTITUDE_INTEGRATOR", \
	1, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_reset_attitude_integrator_t, timestamp) }, \
         } \
}


/**
 * @brief Pack a reset_attitude_integrator message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reset_attitude_integrator_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#else
	mavlink_reset_attitude_integrator_t packet;
	packet.timestamp = timestamp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#endif
}

/**
 * @brief Pack a reset_attitude_integrator message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reset_attitude_integrator_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#else
	mavlink_reset_attitude_integrator_t packet;
	packet.timestamp = timestamp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#endif
}

/**
 * @brief Encode a reset_attitude_integrator struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param reset_attitude_integrator C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reset_attitude_integrator_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_reset_attitude_integrator_t* reset_attitude_integrator)
{
	return mavlink_msg_reset_attitude_integrator_pack(system_id, component_id, msg, reset_attitude_integrator->timestamp);
}

/**
 * @brief Encode a reset_attitude_integrator struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reset_attitude_integrator C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reset_attitude_integrator_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_reset_attitude_integrator_t* reset_attitude_integrator)
{
	return mavlink_msg_reset_attitude_integrator_pack_chan(system_id, component_id, chan, msg, reset_attitude_integrator->timestamp);
}

/**
 * @brief Send a reset_attitude_integrator message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_reset_attitude_integrator_send(mavlink_channel_t chan, uint64_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR, buf, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR, buf, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#endif
#else
	mavlink_reset_attitude_integrator_t packet;
	packet.timestamp = timestamp;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR, (const char *)&packet, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR, (const char *)&packet, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_reset_attitude_integrator_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR, buf, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR, buf, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#endif
#else
	mavlink_reset_attitude_integrator_t *packet = (mavlink_reset_attitude_integrator_t *)msgbuf;
	packet->timestamp = timestamp;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR, (const char *)packet, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR, (const char *)packet, MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RESET_ATTITUDE_INTEGRATOR UNPACKING


/**
 * @brief Get field timestamp from reset_attitude_integrator message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_reset_attitude_integrator_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a reset_attitude_integrator message into a struct
 *
 * @param msg The message to decode
 * @param reset_attitude_integrator C-struct to decode the message contents into
 */
static inline void mavlink_msg_reset_attitude_integrator_decode(const mavlink_message_t* msg, mavlink_reset_attitude_integrator_t* reset_attitude_integrator)
{
#if MAVLINK_NEED_BYTE_SWAP
	reset_attitude_integrator->timestamp = mavlink_msg_reset_attitude_integrator_get_timestamp(msg);
#else
	memcpy(reset_attitude_integrator, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RESET_ATTITUDE_INTEGRATOR_LEN);
#endif
}
