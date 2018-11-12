// MESSAGE EMERGENCY_EKF PACKING

#define MAVLINK_MSG_ID_EMERGENCY_EKF 153

typedef struct __mavlink_emergency_ekf_t
{
 uint64_t timestamp; ///<  Time stamp [microseconds].
 float height_world; ///< Height in world frame [m].
 float vel_body_x; ///< X velocity in body frame [m/s].
 float vel_body_y; ///< Y velocity in body frame [m/s].
 float vel_body_z; ///< Z velocity in body frame [m/s].
 float q_w; ///< Orientation quaternion w component [-].
 float q_x; ///< Orientation quaternion x component [-].
 float q_y; ///< Orientation quaternion y component [-].
 float q_z; ///< Orientation quaternion z component [-].
 float reference_pressure; ///< Reference pressure [mbar].
 float phi; ///<  [rad].
 float theta; ///<  [rad].
 float psi; ///<  [rad].
 float reference_height; ///< Reference height h_0 [m].
 float terrain_bias; ///< Terrain bias [m].
} mavlink_emergency_ekf_t;

#define MAVLINK_MSG_ID_EMERGENCY_EKF_LEN 64
#define MAVLINK_MSG_ID_153_LEN 64

#define MAVLINK_MSG_ID_EMERGENCY_EKF_CRC 212
#define MAVLINK_MSG_ID_153_CRC 212



#define MAVLINK_MESSAGE_INFO_EMERGENCY_EKF { \
	"EMERGENCY_EKF", \
	15, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_emergency_ekf_t, timestamp) }, \
         { "height_world", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_emergency_ekf_t, height_world) }, \
         { "vel_body_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_emergency_ekf_t, vel_body_x) }, \
         { "vel_body_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_emergency_ekf_t, vel_body_y) }, \
         { "vel_body_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_emergency_ekf_t, vel_body_z) }, \
         { "q_w", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_emergency_ekf_t, q_w) }, \
         { "q_x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_emergency_ekf_t, q_x) }, \
         { "q_y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_emergency_ekf_t, q_y) }, \
         { "q_z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_emergency_ekf_t, q_z) }, \
         { "reference_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_emergency_ekf_t, reference_pressure) }, \
         { "phi", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_emergency_ekf_t, phi) }, \
         { "theta", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_emergency_ekf_t, theta) }, \
         { "psi", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_emergency_ekf_t, psi) }, \
         { "reference_height", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_emergency_ekf_t, reference_height) }, \
         { "terrain_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_emergency_ekf_t, terrain_bias) }, \
         } \
}


/**
 * @brief Pack a emergency_ekf message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time stamp [microseconds].
 * @param height_world Height in world frame [m].
 * @param vel_body_x X velocity in body frame [m/s].
 * @param vel_body_y Y velocity in body frame [m/s].
 * @param vel_body_z Z velocity in body frame [m/s].
 * @param q_w Orientation quaternion w component [-].
 * @param q_x Orientation quaternion x component [-].
 * @param q_y Orientation quaternion y component [-].
 * @param q_z Orientation quaternion z component [-].
 * @param reference_pressure Reference pressure [mbar].
 * @param phi  [rad].
 * @param theta  [rad].
 * @param psi  [rad].
 * @param reference_height Reference height h_0 [m].
 * @param terrain_bias Terrain bias [m].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_emergency_ekf_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float height_world, float vel_body_x, float vel_body_y, float vel_body_z, float q_w, float q_x, float q_y, float q_z, float reference_pressure, float phi, float theta, float psi, float reference_height, float terrain_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EMERGENCY_EKF_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, height_world);
	_mav_put_float(buf, 12, vel_body_x);
	_mav_put_float(buf, 16, vel_body_y);
	_mav_put_float(buf, 20, vel_body_z);
	_mav_put_float(buf, 24, q_w);
	_mav_put_float(buf, 28, q_x);
	_mav_put_float(buf, 32, q_y);
	_mav_put_float(buf, 36, q_z);
	_mav_put_float(buf, 40, reference_pressure);
	_mav_put_float(buf, 44, phi);
	_mav_put_float(buf, 48, theta);
	_mav_put_float(buf, 52, psi);
	_mav_put_float(buf, 56, reference_height);
	_mav_put_float(buf, 60, terrain_bias);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#else
	mavlink_emergency_ekf_t packet;
	packet.timestamp = timestamp;
	packet.height_world = height_world;
	packet.vel_body_x = vel_body_x;
	packet.vel_body_y = vel_body_y;
	packet.vel_body_z = vel_body_z;
	packet.q_w = q_w;
	packet.q_x = q_x;
	packet.q_y = q_y;
	packet.q_z = q_z;
	packet.reference_pressure = reference_pressure;
	packet.phi = phi;
	packet.theta = theta;
	packet.psi = psi;
	packet.reference_height = reference_height;
	packet.terrain_bias = terrain_bias;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EMERGENCY_EKF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
}

/**
 * @brief Pack a emergency_ekf message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time stamp [microseconds].
 * @param height_world Height in world frame [m].
 * @param vel_body_x X velocity in body frame [m/s].
 * @param vel_body_y Y velocity in body frame [m/s].
 * @param vel_body_z Z velocity in body frame [m/s].
 * @param q_w Orientation quaternion w component [-].
 * @param q_x Orientation quaternion x component [-].
 * @param q_y Orientation quaternion y component [-].
 * @param q_z Orientation quaternion z component [-].
 * @param reference_pressure Reference pressure [mbar].
 * @param phi  [rad].
 * @param theta  [rad].
 * @param psi  [rad].
 * @param reference_height Reference height h_0 [m].
 * @param terrain_bias Terrain bias [m].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_emergency_ekf_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float height_world,float vel_body_x,float vel_body_y,float vel_body_z,float q_w,float q_x,float q_y,float q_z,float reference_pressure,float phi,float theta,float psi,float reference_height,float terrain_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EMERGENCY_EKF_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, height_world);
	_mav_put_float(buf, 12, vel_body_x);
	_mav_put_float(buf, 16, vel_body_y);
	_mav_put_float(buf, 20, vel_body_z);
	_mav_put_float(buf, 24, q_w);
	_mav_put_float(buf, 28, q_x);
	_mav_put_float(buf, 32, q_y);
	_mav_put_float(buf, 36, q_z);
	_mav_put_float(buf, 40, reference_pressure);
	_mav_put_float(buf, 44, phi);
	_mav_put_float(buf, 48, theta);
	_mav_put_float(buf, 52, psi);
	_mav_put_float(buf, 56, reference_height);
	_mav_put_float(buf, 60, terrain_bias);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#else
	mavlink_emergency_ekf_t packet;
	packet.timestamp = timestamp;
	packet.height_world = height_world;
	packet.vel_body_x = vel_body_x;
	packet.vel_body_y = vel_body_y;
	packet.vel_body_z = vel_body_z;
	packet.q_w = q_w;
	packet.q_x = q_x;
	packet.q_y = q_y;
	packet.q_z = q_z;
	packet.reference_pressure = reference_pressure;
	packet.phi = phi;
	packet.theta = theta;
	packet.psi = psi;
	packet.reference_height = reference_height;
	packet.terrain_bias = terrain_bias;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EMERGENCY_EKF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
}

/**
 * @brief Encode a emergency_ekf struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param emergency_ekf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_emergency_ekf_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_emergency_ekf_t* emergency_ekf)
{
	return mavlink_msg_emergency_ekf_pack(system_id, component_id, msg, emergency_ekf->timestamp, emergency_ekf->height_world, emergency_ekf->vel_body_x, emergency_ekf->vel_body_y, emergency_ekf->vel_body_z, emergency_ekf->q_w, emergency_ekf->q_x, emergency_ekf->q_y, emergency_ekf->q_z, emergency_ekf->reference_pressure, emergency_ekf->phi, emergency_ekf->theta, emergency_ekf->psi, emergency_ekf->reference_height, emergency_ekf->terrain_bias);
}

/**
 * @brief Encode a emergency_ekf struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param emergency_ekf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_emergency_ekf_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_emergency_ekf_t* emergency_ekf)
{
	return mavlink_msg_emergency_ekf_pack_chan(system_id, component_id, chan, msg, emergency_ekf->timestamp, emergency_ekf->height_world, emergency_ekf->vel_body_x, emergency_ekf->vel_body_y, emergency_ekf->vel_body_z, emergency_ekf->q_w, emergency_ekf->q_x, emergency_ekf->q_y, emergency_ekf->q_z, emergency_ekf->reference_pressure, emergency_ekf->phi, emergency_ekf->theta, emergency_ekf->psi, emergency_ekf->reference_height, emergency_ekf->terrain_bias);
}

/**
 * @brief Send a emergency_ekf message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time stamp [microseconds].
 * @param height_world Height in world frame [m].
 * @param vel_body_x X velocity in body frame [m/s].
 * @param vel_body_y Y velocity in body frame [m/s].
 * @param vel_body_z Z velocity in body frame [m/s].
 * @param q_w Orientation quaternion w component [-].
 * @param q_x Orientation quaternion x component [-].
 * @param q_y Orientation quaternion y component [-].
 * @param q_z Orientation quaternion z component [-].
 * @param reference_pressure Reference pressure [mbar].
 * @param phi  [rad].
 * @param theta  [rad].
 * @param psi  [rad].
 * @param reference_height Reference height h_0 [m].
 * @param terrain_bias Terrain bias [m].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_emergency_ekf_send(mavlink_channel_t chan, uint64_t timestamp, float height_world, float vel_body_x, float vel_body_y, float vel_body_z, float q_w, float q_x, float q_y, float q_z, float reference_pressure, float phi, float theta, float psi, float reference_height, float terrain_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EMERGENCY_EKF_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, height_world);
	_mav_put_float(buf, 12, vel_body_x);
	_mav_put_float(buf, 16, vel_body_y);
	_mav_put_float(buf, 20, vel_body_z);
	_mav_put_float(buf, 24, q_w);
	_mav_put_float(buf, 28, q_x);
	_mav_put_float(buf, 32, q_y);
	_mav_put_float(buf, 36, q_z);
	_mav_put_float(buf, 40, reference_pressure);
	_mav_put_float(buf, 44, phi);
	_mav_put_float(buf, 48, theta);
	_mav_put_float(buf, 52, psi);
	_mav_put_float(buf, 56, reference_height);
	_mav_put_float(buf, 60, terrain_bias);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
#else
	mavlink_emergency_ekf_t packet;
	packet.timestamp = timestamp;
	packet.height_world = height_world;
	packet.vel_body_x = vel_body_x;
	packet.vel_body_y = vel_body_y;
	packet.vel_body_z = vel_body_z;
	packet.q_w = q_w;
	packet.q_x = q_x;
	packet.q_y = q_y;
	packet.q_z = q_z;
	packet.reference_pressure = reference_pressure;
	packet.phi = phi;
	packet.theta = theta;
	packet.psi = psi;
	packet.reference_height = reference_height;
	packet.terrain_bias = terrain_bias;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, (const char *)&packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, (const char *)&packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_EMERGENCY_EKF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_emergency_ekf_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float height_world, float vel_body_x, float vel_body_y, float vel_body_z, float q_w, float q_x, float q_y, float q_z, float reference_pressure, float phi, float theta, float psi, float reference_height, float terrain_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, height_world);
	_mav_put_float(buf, 12, vel_body_x);
	_mav_put_float(buf, 16, vel_body_y);
	_mav_put_float(buf, 20, vel_body_z);
	_mav_put_float(buf, 24, q_w);
	_mav_put_float(buf, 28, q_x);
	_mav_put_float(buf, 32, q_y);
	_mav_put_float(buf, 36, q_z);
	_mav_put_float(buf, 40, reference_pressure);
	_mav_put_float(buf, 44, phi);
	_mav_put_float(buf, 48, theta);
	_mav_put_float(buf, 52, psi);
	_mav_put_float(buf, 56, reference_height);
	_mav_put_float(buf, 60, terrain_bias);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
#else
	mavlink_emergency_ekf_t *packet = (mavlink_emergency_ekf_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->height_world = height_world;
	packet->vel_body_x = vel_body_x;
	packet->vel_body_y = vel_body_y;
	packet->vel_body_z = vel_body_z;
	packet->q_w = q_w;
	packet->q_x = q_x;
	packet->q_y = q_y;
	packet->q_z = q_z;
	packet->reference_pressure = reference_pressure;
	packet->phi = phi;
	packet->theta = theta;
	packet->psi = psi;
	packet->reference_height = reference_height;
	packet->terrain_bias = terrain_bias;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, (const char *)packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, (const char *)packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE EMERGENCY_EKF UNPACKING


/**
 * @brief Get field timestamp from emergency_ekf message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_emergency_ekf_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field height_world from emergency_ekf message
 *
 * @return Height in world frame [m].
 */
static inline float mavlink_msg_emergency_ekf_get_height_world(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vel_body_x from emergency_ekf message
 *
 * @return X velocity in body frame [m/s].
 */
static inline float mavlink_msg_emergency_ekf_get_vel_body_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vel_body_y from emergency_ekf message
 *
 * @return Y velocity in body frame [m/s].
 */
static inline float mavlink_msg_emergency_ekf_get_vel_body_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_body_z from emergency_ekf message
 *
 * @return Z velocity in body frame [m/s].
 */
static inline float mavlink_msg_emergency_ekf_get_vel_body_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field q_w from emergency_ekf message
 *
 * @return Orientation quaternion w component [-].
 */
static inline float mavlink_msg_emergency_ekf_get_q_w(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field q_x from emergency_ekf message
 *
 * @return Orientation quaternion x component [-].
 */
static inline float mavlink_msg_emergency_ekf_get_q_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field q_y from emergency_ekf message
 *
 * @return Orientation quaternion y component [-].
 */
static inline float mavlink_msg_emergency_ekf_get_q_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field q_z from emergency_ekf message
 *
 * @return Orientation quaternion z component [-].
 */
static inline float mavlink_msg_emergency_ekf_get_q_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field reference_pressure from emergency_ekf message
 *
 * @return Reference pressure [mbar].
 */
static inline float mavlink_msg_emergency_ekf_get_reference_pressure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field phi from emergency_ekf message
 *
 * @return  [rad].
 */
static inline float mavlink_msg_emergency_ekf_get_phi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field theta from emergency_ekf message
 *
 * @return  [rad].
 */
static inline float mavlink_msg_emergency_ekf_get_theta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field psi from emergency_ekf message
 *
 * @return  [rad].
 */
static inline float mavlink_msg_emergency_ekf_get_psi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field reference_height from emergency_ekf message
 *
 * @return Reference height h_0 [m].
 */
static inline float mavlink_msg_emergency_ekf_get_reference_height(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field terrain_bias from emergency_ekf message
 *
 * @return Terrain bias [m].
 */
static inline float mavlink_msg_emergency_ekf_get_terrain_bias(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Decode a emergency_ekf message into a struct
 *
 * @param msg The message to decode
 * @param emergency_ekf C-struct to decode the message contents into
 */
static inline void mavlink_msg_emergency_ekf_decode(const mavlink_message_t* msg, mavlink_emergency_ekf_t* emergency_ekf)
{
#if MAVLINK_NEED_BYTE_SWAP
	emergency_ekf->timestamp = mavlink_msg_emergency_ekf_get_timestamp(msg);
	emergency_ekf->height_world = mavlink_msg_emergency_ekf_get_height_world(msg);
	emergency_ekf->vel_body_x = mavlink_msg_emergency_ekf_get_vel_body_x(msg);
	emergency_ekf->vel_body_y = mavlink_msg_emergency_ekf_get_vel_body_y(msg);
	emergency_ekf->vel_body_z = mavlink_msg_emergency_ekf_get_vel_body_z(msg);
	emergency_ekf->q_w = mavlink_msg_emergency_ekf_get_q_w(msg);
	emergency_ekf->q_x = mavlink_msg_emergency_ekf_get_q_x(msg);
	emergency_ekf->q_y = mavlink_msg_emergency_ekf_get_q_y(msg);
	emergency_ekf->q_z = mavlink_msg_emergency_ekf_get_q_z(msg);
	emergency_ekf->reference_pressure = mavlink_msg_emergency_ekf_get_reference_pressure(msg);
	emergency_ekf->phi = mavlink_msg_emergency_ekf_get_phi(msg);
	emergency_ekf->theta = mavlink_msg_emergency_ekf_get_theta(msg);
	emergency_ekf->psi = mavlink_msg_emergency_ekf_get_psi(msg);
	emergency_ekf->reference_height = mavlink_msg_emergency_ekf_get_reference_height(msg);
	emergency_ekf->terrain_bias = mavlink_msg_emergency_ekf_get_terrain_bias(msg);
#else
	memcpy(emergency_ekf, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
}
