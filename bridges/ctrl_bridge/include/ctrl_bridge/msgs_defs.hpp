#pragma once

#include <stdint.h>

namespace ctrl_bridge {

namespace msgs {

static constexpr uint8_t SERIAL_MESSAGE_HEADER_LENGTH = 10;
static constexpr uint8_t SERIAL_MESSAGE_MAX_PAYLOAD_LENGTH = 66;
static constexpr uint8_t SERIAL_MESSAGE_MIN_LENGTH = SERIAL_MESSAGE_HEADER_LENGTH + 1;
static constexpr uint8_t SERIAL_MESSAGE_MAX_LENGTH = SERIAL_MESSAGE_MIN_LENGTH + SERIAL_MESSAGE_MAX_PAYLOAD_LENGTH;
static constexpr uint8_t SERIAL_MESSAGE_MAX_COUNT = 3;
static constexpr uint8_t SERIAL_CONTAINER_MAX_LENGTH =
  SERIAL_MESSAGE_MAX_COUNT * SERIAL_MESSAGE_MAX_LENGTH + 7; // Less than 256
static constexpr uint8_t SERIAL_CONTAINER_DELIMITER = 0x7c;

static constexpr int RAW_RANGE = 1;                     // no conversion!
static constexpr double UNITLESS_RANGE = 32.767;        // [1]
static constexpr double TIME_RANGE = 32.767;            // [s]
static constexpr double FREQUENCY_RANGE = 32767;        // [Hz]
static constexpr double ANGLE_RANGE = 32.767;           // [rad]
static constexpr double ANGULAR_VELOCITY_RANGE = 32.767;// [rad/s]
static constexpr double DISTANCE_RANGE = 32.767;        // [m]
static constexpr double VELOCITY_RANGE = 32.767;        // [m/s]
static constexpr double ACCELERATION_RANGE = 327.67;    // [m/s^2]
static constexpr double FORCE_RANGE = 327.67;           // [N]
static constexpr double VOLTAGE_RANGE = 327.67;         // [V]
static constexpr double CURRENT_RANGE = 327.67;         // [A]

enum CTRLMODE:uint8_t {
  INVALID = 0,
  OFF = 1,                    /* disarmed */
  SET_PARAM = 2,              /* change a single specified parameter */
  ARM = 3,                    /* armed and rotors spin at min speed */
  ROTOR_THROTTLE = 4,         /* set rotor throttle for every rotor [0-2000] */
  ROTOR_SPEED = 5,            /* set rotor speed for every rotor [rad/s] */
  ROTOR_THRUST = 6,           /* set rotor thrust for every rotor [N] */
  BODY_RATE = 7,              /* set body rate [vector rad/s] and total thrust [N] */
  ATTITUDE = 8,               /* set attitude as [unit vector], yaw body rate [rad/s] and total thrust [N] */
  VELOCITY = 9,               /* set velocity [vector m/s] and yaw body rate [rad/s] */
  EMERGENCY = 10,             /* hold position and land */
  FEEDBACK = 255              /* upstream flight data */
};

enum COMMANDFLAGS:uint8_t {
  SET_BIAS = 0x01,            /* use current low pass filtered IMU data as bias */
  SAVE_TO_FLASH = 0x02        /* save all parameters to flash (includes bias) */
};

enum FEEDBACKTYPE:uint8_t {
  NOTHING = 0,
  THROTTLE = 1,
  SPEED = 2,
  THRUST = 3
};

struct __attribute__((__packed__)) serial_message_t
{
  uint8_t mode{1};           /* use control_mode_t */
  uint8_t flags{0};          /* use command_flag_t */
  uint64_t time{0};          /* time since startup [ns] */
  char payload[SERIAL_MESSAGE_MAX_PAYLOAD_LENGTH];
};

struct __attribute__((__packed__)) serial_container_t
{
  uint8_t num_messages{1};
  serial_message_t messages[SERIAL_MESSAGE_MAX_COUNT];
};

struct __attribute__((__packed__)) parameter_payload_t
{
  uint16_t parameter{0};     /* use parameter_t */
  float value{0};            /* range dependent on parameter */
};

struct __attribute__((__packed__)) rotor_throttle_payload_t
{
  int16_t throttle[4] = {0};    /* RAW_RANGE */
};

struct __attribute__((__packed__)) rotor_speed_payload_t
{
  int16_t speed[4] = {0};       /* ANGULAR_VELOCITY_RANGE */
};

struct __attribute__((__packed__)) rotor_thrust_payload_t
{
  int16_t thrust[4] = {0};      /* FORCE_RANGE */
};

struct __attribute__((__packed__)) body_rate_payload_t
{
  int16_t body_rate[3] = {0};   /* ANGULAR_VELOCITY_RANGE */
  int16_t thrust{0};            /* FORCE_RANGE */
};

struct __attribute__((__packed__)) attitude_payload_t
{
  int16_t attitude[3] = {0};    /* UNITLESS_RANGE */
  int16_t yaw_body_rate{0};     /* ANGULAR_VELOCITY_RANGE */
  int16_t thrust{0};            /* FORCE_RANGE */
};

struct __attribute__((__packed__)) velocity_payload_t
{
  int16_t velocity[3] = {0};    /* VELOCITY_RANGE */
  int16_t yaw_body_rate{0};     /* ANGULAR_VELOCITY_RANGE */
};

struct __attribute__((__packed__)) upstream_payload_t
{
  /* IMU upstream */
  float angular_vel_x;
  float angular_vel_y;
  float angular_vel_z;
  float linear_accel_x;
  float linear_accel_y;
  float linear_accel_z;
  
  /* rotor feedback */
  float feedback_0;
  float feedback_1;
  float feedback_2;
  float feedback_3;  
  uint8_t feedback_type;    /* use feedback_type_t */
  
  /* estimated attitude feedback */
  float quaternion_w; 
  float quaternion_x;
  float quaternion_y;
  float quaternion_z;
  
  /* battery feedback */
  float battery_voltage;
  float battery_current;
  
  uint8_t control_mode;           /* use control_mode_t */
};

} // namespace msgs

} // namespace ctrl_bridge