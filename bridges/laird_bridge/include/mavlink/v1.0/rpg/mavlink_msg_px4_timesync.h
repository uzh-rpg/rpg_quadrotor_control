// MESSAGE PX4_TIMESYNC PACKING

#define MAVLINK_MSG_ID_PX4_TIMESYNC 160

typedef struct __mavlink_px4_timesync_t
{
 uint64_t timestamp; ///<  Time stamp [microseconds].
 int64_t sync_id; ///< Trigger bumber
} mavlink_px4_timesync_t;

#define MAVLINK_MSG_ID_PX4_TIMESYNC_LEN 16
#define MAVLINK_MSG_ID_160_LEN 16

#define MAVLINK_MSG_ID_PX4_TIMESYNC_CRC 223
#define MAVLINK_MSG_ID_160_CRC 223



#define MAVLINK_MESSAGE_INFO_PX4_TIMESYNC { \
	"PX4_TIMESYNC", \
	2, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_px4_timesync_t, timestamp) }, \
         { "sync_id", NULL, MAVLINK_TYPE_INT64_T, 0, 8, offsetof(mavlink_px4_timesync_t, sync_id) }, \
         } \
}


/**
 * @brief Pack a px4_timesync message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param sync_id Trigger bumber
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_timesync_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, int64_t sync_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PX4_TIMESYNC_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int64_t(buf, 8, sync_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#else
	mavlink_px4_timesync_t packet;
	packet.timestamp = timestamp;
	packet.sync_id = sync_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PX4_TIMESYNC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN, MAVLINK_MSG_ID_PX4_TIMESYNC_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#endif
}

/**
 * @brief Pack a px4_timesync message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param sync_id Trigger bumber
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_timesync_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,int64_t sync_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PX4_TIMESYNC_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int64_t(buf, 8, sync_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#else
	mavlink_px4_timesync_t packet;
	packet.timestamp = timestamp;
	packet.sync_id = sync_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PX4_TIMESYNC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN, MAVLINK_MSG_ID_PX4_TIMESYNC_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#endif
}

/**
 * @brief Encode a px4_timesync struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4_timesync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_timesync_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4_timesync_t* px4_timesync)
{
	return mavlink_msg_px4_timesync_pack(system_id, component_id, msg, px4_timesync->timestamp, px4_timesync->sync_id);
}

/**
 * @brief Encode a px4_timesync struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4_timesync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_timesync_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4_timesync_t* px4_timesync)
{
	return mavlink_msg_px4_timesync_pack_chan(system_id, component_id, chan, msg, px4_timesync->timestamp, px4_timesync->sync_id);
}

/**
 * @brief Send a px4_timesync message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param sync_id Trigger bumber
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4_timesync_send(mavlink_channel_t chan, uint64_t timestamp, int64_t sync_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PX4_TIMESYNC_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int64_t(buf, 8, sync_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_TIMESYNC, buf, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN, MAVLINK_MSG_ID_PX4_TIMESYNC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_TIMESYNC, buf, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#endif
#else
	mavlink_px4_timesync_t packet;
	packet.timestamp = timestamp;
	packet.sync_id = sync_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_TIMESYNC, (const char *)&packet, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN, MAVLINK_MSG_ID_PX4_TIMESYNC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_TIMESYNC, (const char *)&packet, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PX4_TIMESYNC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4_timesync_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, int64_t sync_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int64_t(buf, 8, sync_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_TIMESYNC, buf, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN, MAVLINK_MSG_ID_PX4_TIMESYNC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_TIMESYNC, buf, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#endif
#else
	mavlink_px4_timesync_t *packet = (mavlink_px4_timesync_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->sync_id = sync_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_TIMESYNC, (const char *)packet, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN, MAVLINK_MSG_ID_PX4_TIMESYNC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_TIMESYNC, (const char *)packet, MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PX4_TIMESYNC UNPACKING


/**
 * @brief Get field timestamp from px4_timesync message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_px4_timesync_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sync_id from px4_timesync message
 *
 * @return Trigger bumber
 */
static inline int64_t mavlink_msg_px4_timesync_get_sync_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int64_t(msg,  8);
}

/**
 * @brief Decode a px4_timesync message into a struct
 *
 * @param msg The message to decode
 * @param px4_timesync C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4_timesync_decode(const mavlink_message_t* msg, mavlink_px4_timesync_t* px4_timesync)
{
#if MAVLINK_NEED_BYTE_SWAP
	px4_timesync->timestamp = mavlink_msg_px4_timesync_get_timestamp(msg);
	px4_timesync->sync_id = mavlink_msg_px4_timesync_get_sync_id(msg);
#else
	memcpy(px4_timesync, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PX4_TIMESYNC_LEN);
#endif
}
