// MESSAGE MAGNETIC_FIELD PACKING

#define MAVLINK_MSG_ID_MAGNETIC_FIELD 152

typedef struct __mavlink_magnetic_field_t
{
 uint64_t timestamp; ///<  Time stamp [microseconds].
 float mag_x; ///< X magnetic field [Gauss].
 float mag_y; ///< Y magnetic field [Gauss].
 float mag_z; ///< Z magnetic field [Gauss].
} mavlink_magnetic_field_t;

#define MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN 20
#define MAVLINK_MSG_ID_152_LEN 20

#define MAVLINK_MSG_ID_MAGNETIC_FIELD_CRC 122
#define MAVLINK_MSG_ID_152_CRC 122



#define MAVLINK_MESSAGE_INFO_MAGNETIC_FIELD { \
	"MAGNETIC_FIELD", \
	4, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_magnetic_field_t, timestamp) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_magnetic_field_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_magnetic_field_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_magnetic_field_t, mag_z) }, \
         } \
}


/**
 * @brief Pack a magnetic_field message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param mag_x X magnetic field [Gauss].
 * @param mag_y Y magnetic field [Gauss].
 * @param mag_z Z magnetic field [Gauss].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magnetic_field_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float mag_x, float mag_y, float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, mag_x);
	_mav_put_float(buf, 12, mag_y);
	_mav_put_float(buf, 16, mag_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#else
	mavlink_magnetic_field_t packet;
	packet.timestamp = timestamp;
	packet.mag_x = mag_x;
	packet.mag_y = mag_y;
	packet.mag_z = mag_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAGNETIC_FIELD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN, MAVLINK_MSG_ID_MAGNETIC_FIELD_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#endif
}

/**
 * @brief Pack a magnetic_field message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param mag_x X magnetic field [Gauss].
 * @param mag_y Y magnetic field [Gauss].
 * @param mag_z Z magnetic field [Gauss].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magnetic_field_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float mag_x,float mag_y,float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, mag_x);
	_mav_put_float(buf, 12, mag_y);
	_mav_put_float(buf, 16, mag_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#else
	mavlink_magnetic_field_t packet;
	packet.timestamp = timestamp;
	packet.mag_x = mag_x;
	packet.mag_y = mag_y;
	packet.mag_z = mag_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAGNETIC_FIELD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN, MAVLINK_MSG_ID_MAGNETIC_FIELD_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#endif
}

/**
 * @brief Encode a magnetic_field struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magnetic_field C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magnetic_field_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magnetic_field_t* magnetic_field)
{
	return mavlink_msg_magnetic_field_pack(system_id, component_id, msg, magnetic_field->timestamp, magnetic_field->mag_x, magnetic_field->mag_y, magnetic_field->mag_z);
}

/**
 * @brief Encode a magnetic_field struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magnetic_field C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magnetic_field_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magnetic_field_t* magnetic_field)
{
	return mavlink_msg_magnetic_field_pack_chan(system_id, component_id, chan, msg, magnetic_field->timestamp, magnetic_field->mag_x, magnetic_field->mag_y, magnetic_field->mag_z);
}

/**
 * @brief Send a magnetic_field message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param mag_x X magnetic field [Gauss].
 * @param mag_y Y magnetic field [Gauss].
 * @param mag_z Z magnetic field [Gauss].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magnetic_field_send(mavlink_channel_t chan, uint64_t timestamp, float mag_x, float mag_y, float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, mag_x);
	_mav_put_float(buf, 12, mag_y);
	_mav_put_float(buf, 16, mag_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGNETIC_FIELD, buf, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN, MAVLINK_MSG_ID_MAGNETIC_FIELD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGNETIC_FIELD, buf, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#endif
#else
	mavlink_magnetic_field_t packet;
	packet.timestamp = timestamp;
	packet.mag_x = mag_x;
	packet.mag_y = mag_y;
	packet.mag_z = mag_z;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGNETIC_FIELD, (const char *)&packet, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN, MAVLINK_MSG_ID_MAGNETIC_FIELD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGNETIC_FIELD, (const char *)&packet, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magnetic_field_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float mag_x, float mag_y, float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, mag_x);
	_mav_put_float(buf, 12, mag_y);
	_mav_put_float(buf, 16, mag_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGNETIC_FIELD, buf, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN, MAVLINK_MSG_ID_MAGNETIC_FIELD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGNETIC_FIELD, buf, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#endif
#else
	mavlink_magnetic_field_t *packet = (mavlink_magnetic_field_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->mag_x = mag_x;
	packet->mag_y = mag_y;
	packet->mag_z = mag_z;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGNETIC_FIELD, (const char *)packet, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN, MAVLINK_MSG_ID_MAGNETIC_FIELD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGNETIC_FIELD, (const char *)packet, MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MAGNETIC_FIELD UNPACKING


/**
 * @brief Get field timestamp from magnetic_field message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_magnetic_field_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field mag_x from magnetic_field message
 *
 * @return X magnetic field [Gauss].
 */
static inline float mavlink_msg_magnetic_field_get_mag_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field mag_y from magnetic_field message
 *
 * @return Y magnetic field [Gauss].
 */
static inline float mavlink_msg_magnetic_field_get_mag_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field mag_z from magnetic_field message
 *
 * @return Z magnetic field [Gauss].
 */
static inline float mavlink_msg_magnetic_field_get_mag_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a magnetic_field message into a struct
 *
 * @param msg The message to decode
 * @param magnetic_field C-struct to decode the message contents into
 */
static inline void mavlink_msg_magnetic_field_decode(const mavlink_message_t* msg, mavlink_magnetic_field_t* magnetic_field)
{
#if MAVLINK_NEED_BYTE_SWAP
	magnetic_field->timestamp = mavlink_msg_magnetic_field_get_timestamp(msg);
	magnetic_field->mag_x = mavlink_msg_magnetic_field_get_mag_x(msg);
	magnetic_field->mag_y = mavlink_msg_magnetic_field_get_mag_y(msg);
	magnetic_field->mag_z = mavlink_msg_magnetic_field_get_mag_z(msg);
#else
	memcpy(magnetic_field, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAGNETIC_FIELD_LEN);
#endif
}
