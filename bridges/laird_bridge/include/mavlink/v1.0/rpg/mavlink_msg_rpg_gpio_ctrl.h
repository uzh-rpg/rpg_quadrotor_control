// MESSAGE RPG_GPIO_CTRL PACKING

#define MAVLINK_MSG_ID_RPG_GPIO_CTRL 161

typedef struct __mavlink_rpg_gpio_ctrl_t
{
 uint64_t timestamp; /*<  Time stamp [microseconds].*/
 float value; /*< GPIO level {0.0, 1.0}; Servo pitch [10.0, 11.0]; Duty cycle [20.0, 21.0].*/
 uint8_t device; /*< Device number [].*/
 uint8_t num_gpio; /*< Output number [].*/
} mavlink_rpg_gpio_ctrl_t;

#define MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN 14
#define MAVLINK_MSG_ID_161_LEN 14

#define MAVLINK_MSG_ID_RPG_GPIO_CTRL_CRC 91
#define MAVLINK_MSG_ID_161_CRC 91



#define MAVLINK_MESSAGE_INFO_RPG_GPIO_CTRL { \
	"RPG_GPIO_CTRL", \
	4, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rpg_gpio_ctrl_t, timestamp) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rpg_gpio_ctrl_t, value) }, \
         { "device", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_rpg_gpio_ctrl_t, device) }, \
         { "num_gpio", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_rpg_gpio_ctrl_t, num_gpio) }, \
         } \
}


/**
 * @brief Pack a rpg_gpio_ctrl message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param device Device number [].
 * @param num_gpio Output number [].
 * @param value GPIO level {0.0, 1.0}; Servo pitch [10.0, 11.0]; Duty cycle [20.0, 21.0].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpg_gpio_ctrl_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint8_t device, uint8_t num_gpio, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, value);
	_mav_put_uint8_t(buf, 12, device);
	_mav_put_uint8_t(buf, 13, num_gpio);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#else
	mavlink_rpg_gpio_ctrl_t packet;
	packet.timestamp = timestamp;
	packet.value = value;
	packet.device = device;
	packet.num_gpio = num_gpio;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RPG_GPIO_CTRL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN, MAVLINK_MSG_ID_RPG_GPIO_CTRL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#endif
}

/**
 * @brief Pack a rpg_gpio_ctrl message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param device Device number [].
 * @param num_gpio Output number [].
 * @param value GPIO level {0.0, 1.0}; Servo pitch [10.0, 11.0]; Duty cycle [20.0, 21.0].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpg_gpio_ctrl_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint8_t device,uint8_t num_gpio,float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, value);
	_mav_put_uint8_t(buf, 12, device);
	_mav_put_uint8_t(buf, 13, num_gpio);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#else
	mavlink_rpg_gpio_ctrl_t packet;
	packet.timestamp = timestamp;
	packet.value = value;
	packet.device = device;
	packet.num_gpio = num_gpio;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RPG_GPIO_CTRL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN, MAVLINK_MSG_ID_RPG_GPIO_CTRL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#endif
}

/**
 * @brief Encode a rpg_gpio_ctrl struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rpg_gpio_ctrl C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpg_gpio_ctrl_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rpg_gpio_ctrl_t* rpg_gpio_ctrl)
{
	return mavlink_msg_rpg_gpio_ctrl_pack(system_id, component_id, msg, rpg_gpio_ctrl->timestamp, rpg_gpio_ctrl->device, rpg_gpio_ctrl->num_gpio, rpg_gpio_ctrl->value);
}

/**
 * @brief Encode a rpg_gpio_ctrl struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rpg_gpio_ctrl C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpg_gpio_ctrl_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rpg_gpio_ctrl_t* rpg_gpio_ctrl)
{
	return mavlink_msg_rpg_gpio_ctrl_pack_chan(system_id, component_id, chan, msg, rpg_gpio_ctrl->timestamp, rpg_gpio_ctrl->device, rpg_gpio_ctrl->num_gpio, rpg_gpio_ctrl->value);
}

/**
 * @brief Send a rpg_gpio_ctrl message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param device Device number [].
 * @param num_gpio Output number [].
 * @param value GPIO level {0.0, 1.0}; Servo pitch [10.0, 11.0]; Duty cycle [20.0, 21.0].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rpg_gpio_ctrl_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t device, uint8_t num_gpio, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, value);
	_mav_put_uint8_t(buf, 12, device);
	_mav_put_uint8_t(buf, 13, num_gpio);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL, buf, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN, MAVLINK_MSG_ID_RPG_GPIO_CTRL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL, buf, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#endif
#else
	mavlink_rpg_gpio_ctrl_t packet;
	packet.timestamp = timestamp;
	packet.value = value;
	packet.device = device;
	packet.num_gpio = num_gpio;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL, (const char *)&packet, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN, MAVLINK_MSG_ID_RPG_GPIO_CTRL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL, (const char *)&packet, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rpg_gpio_ctrl_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t device, uint8_t num_gpio, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, value);
	_mav_put_uint8_t(buf, 12, device);
	_mav_put_uint8_t(buf, 13, num_gpio);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL, buf, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN, MAVLINK_MSG_ID_RPG_GPIO_CTRL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL, buf, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#endif
#else
	mavlink_rpg_gpio_ctrl_t *packet = (mavlink_rpg_gpio_ctrl_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->value = value;
	packet->device = device;
	packet->num_gpio = num_gpio;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL, (const char *)packet, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN, MAVLINK_MSG_ID_RPG_GPIO_CTRL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPG_GPIO_CTRL, (const char *)packet, MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RPG_GPIO_CTRL UNPACKING


/**
 * @brief Get field timestamp from rpg_gpio_ctrl message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_rpg_gpio_ctrl_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field device from rpg_gpio_ctrl message
 *
 * @return Device number [].
 */
static inline uint8_t mavlink_msg_rpg_gpio_ctrl_get_device(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field num_gpio from rpg_gpio_ctrl message
 *
 * @return Output number [].
 */
static inline uint8_t mavlink_msg_rpg_gpio_ctrl_get_num_gpio(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field value from rpg_gpio_ctrl message
 *
 * @return GPIO level {0.0, 1.0}; Servo pitch [10.0, 11.0]; Duty cycle [20.0, 21.0].
 */
static inline float mavlink_msg_rpg_gpio_ctrl_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a rpg_gpio_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param rpg_gpio_ctrl C-struct to decode the message contents into
 */
static inline void mavlink_msg_rpg_gpio_ctrl_decode(const mavlink_message_t* msg, mavlink_rpg_gpio_ctrl_t* rpg_gpio_ctrl)
{
#if MAVLINK_NEED_BYTE_SWAP
	rpg_gpio_ctrl->timestamp = mavlink_msg_rpg_gpio_ctrl_get_timestamp(msg);
	rpg_gpio_ctrl->value = mavlink_msg_rpg_gpio_ctrl_get_value(msg);
	rpg_gpio_ctrl->device = mavlink_msg_rpg_gpio_ctrl_get_device(msg);
	rpg_gpio_ctrl->num_gpio = mavlink_msg_rpg_gpio_ctrl_get_num_gpio(msg);
#else
	memcpy(rpg_gpio_ctrl, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RPG_GPIO_CTRL_LEN);
#endif
}
