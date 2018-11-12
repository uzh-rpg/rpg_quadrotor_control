/*
 * LairdBridge.cpp
 *
 *  Created on: Jan 15, 2014
 *      Author: ffontana
 */

#include <laird_bridge/laird_bridge.h>          

#include "laird_bridge/serial_port.h"
#include "laird_bridge/parameter_handler.h"

namespace laird_bridge
{

using namespace quad_common;

LairdBridge::LairdBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) :
    nh_(nh), pnh_(pnh), control_active_(false), min_timesync_offset_(1e-4)
{
  // Fetch parameters
  if (!reloadParameters())
  {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Publishers
  if (publish_sensors_)
  {
    ROS_INFO("[%s] publishes sensor messages", pnh_.getNamespace().c_str());

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
    onboard_attitude_est_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>("onboard_attitude_est", 1);
    rotor_thrusts_input_pub_ = nh_.advertise<quad_msgs::QuadRotorThrusts>("rotor_thrusts_input", 1);
    torques_and_thrust_input_pub_ = nh_.advertise<quad_msgs::QuadDesiredTorquesAndThrust>("torques_and_thrust_input",
                                                                                          1);
    onboard_ranger_pub_ = nh_.advertise<sensor_msgs::Range>("onboard_ranger", 1);
    barometer_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("barometer", 1);
    baro_temperature_pub_ = nh_.advertise<sensor_msgs::Temperature>("baro_temperature", 1);
    onboard_status_pub_ = nh_.advertise<quad_msgs::OnboardStatus>("onboard_status", 1);
    magnetometer_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magnetometer", 1);

    px4_timesync_msg_pub_ = nh_.advertise<quad_msgs::Px4Timesync>("px4_timesync_msg", 1);
    rc_pub_ = nh_.advertise<sensor_msgs::Joy>("rc_commands", 1);
  }

  if (publish_commands_)
  {
    ROS_INFO("[%s] Receiving control commands from serial port (Used for FPV Quad)", pnh_.getNamespace().c_str());
    control_command_pub_ = nh_.advertise<quad_msgs::ControlCommand>("control_command", 1);
  }

  // Subscribers
  if (accept_commands_)
  {
    ROS_INFO("[%s] accepts commands", pnh_.getNamespace().c_str());
    set_control_active_sub_ = nh_.subscribe("laird_bridge/control_active", 1,
                                            &LairdBridge::setControlActiveCallback, this);
    control_command_sub_ = nh_.subscribe("control_command", 1, &LairdBridge::controlCommandCallback, this);

    gpio_pwm_control_sub_ = nh_.subscribe("gpio_control", 1, &LairdBridge::gpioPwmControlCallback, this);
  }

  // Init time offset topic
  px4_time_offset_sub_ = nh_.subscribe("px4_time_offset", 1, &LairdBridge::px4TimeOffsetCallback, this);
  px4_time_offset_ = ros::Duration(0.0);

  // Open serial port
  serial_port_ = new SerialPort(this);
  if (!serial_port_->connect(port_name_, baud_rate_))
  {
    ros::shutdown();
    return;
  }

  if (handle_parameters_)
  {
    ROS_INFO("[%s] handles parameters", pnh_.getNamespace().c_str());

    parameter_handler_ = new ParameterHandler(this, basecomputer_sys_id_, basecomputer_comp_id_, px4_sys_id_,
                                              px4_comp_id_);
  }
}

LairdBridge::~LairdBridge()
{
  //  serial_port_.disconnect();
}

// Publisher Functions
void LairdBridge::handleMavlinkMessage(const mavlink_message_t & mavlink_msg)
{
  if (mavlink_msg.msgid == MAVLINK_MSG_ID_PARAM_VALUE && handle_parameters_)
  {
    parameter_handler_->handleOnboardParameterMsg(mavlink_msg);
  }

  if (publish_sensors_)
  {
    switch (mavlink_msg.msgid)
    {
      case MAVLINK_MSG_ID_RPG_IMU:
        publishImuMsg(mavlink_msg);
        break;
      case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        publishOnboardAttitudeMsg(mavlink_msg);
        break;
      case MAVLINK_MSG_ID_QUAD_ROTOR_THRUSTS:
        publishRotorThrustsInputMsg(mavlink_msg);
        break;
      case MAVLINK_MSG_ID_RPG_DESIRED_TORQUES_AND_THRUST:
        publishTorquesAndThrustInputMsg(mavlink_msg);
        break;
      case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        publishOnboardTeraRangerMsg(mavlink_msg);
        break;
      case MAVLINK_MSG_ID_SCALED_PRESSURE:
        publishBarometerMsg(mavlink_msg);
        break;
      case MAVLINK_MSG_ID_RPG_ONBOARD_STATUS:
        publishOnboardStatusMsg(mavlink_msg);
        break;
      case MAVLINK_MSG_ID_MAGNETIC_FIELD:
        publishMagnetometerMsg(mavlink_msg);
        break;
      case MAVLINK_MSG_ID_PX4_TIMESYNC:
        publishPx4TimesyncMsg(mavlink_msg);
        break;
      case MAVLINK_MSG_ID_RPG_RC:
        publishRCMsg(mavlink_msg);
        break;

      default:
        break;
    }
  }

  if (publish_commands_)
  {
    if (mavlink_msg.msgid == MAVLINK_MSG_ID_RPG_CONTROL_COMMAND)
    {
      publishControlCommandMsg(mavlink_msg);
    }
  }
}

void LairdBridge::sendMavlinkMessage(const mavlink_message_t & mavlink_msg)
{
  serial_port_->sendMavlinkMessage(mavlink_msg);
}

void LairdBridge::publishImuMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  // Convert to a Imu ROS message
  sensor_msgs::Imu imu_ros_msg;
  imu_ros_msg.header.frame_id = ""; //TODO
  imu_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    imu_ros_msg.header.stamp = ros::Time(mavlink_msg_rpg_imu_get_timestamp(&mavlink_msg) / 1000000.0)
        + px4_time_offset_;
  }
  else
  {
    imu_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  imu_ros_msg.angular_velocity.x = mavlink_msg_rpg_imu_get_gyro_x(&mavlink_msg);
  imu_ros_msg.angular_velocity.y = mavlink_msg_rpg_imu_get_gyro_y(&mavlink_msg);
  imu_ros_msg.angular_velocity.z = mavlink_msg_rpg_imu_get_gyro_z(&mavlink_msg);
  imu_ros_msg.linear_acceleration.x = mavlink_msg_rpg_imu_get_acc_x(&mavlink_msg);
  imu_ros_msg.linear_acceleration.y = mavlink_msg_rpg_imu_get_acc_y(&mavlink_msg);
  imu_ros_msg.linear_acceleration.z = mavlink_msg_rpg_imu_get_acc_z(&mavlink_msg);

  // Publish ROS message
  imu_pub_.publish(imu_ros_msg);
}

void LairdBridge::publishRotorThrustsInputMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  quad_msgs::QuadRotorThrusts rotor_thrusts_input_ros_msg;
  rotor_thrusts_input_ros_msg.header.frame_id = ""; //TODO
  rotor_thrusts_input_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    rotor_thrusts_input_ros_msg.header.stamp = ros::Time(
        mavlink_msg_quad_rotor_thrusts_get_timestamp(&mavlink_msg) / 1000000.0) + px4_time_offset_;
  }
  else
  {
    rotor_thrusts_input_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  rotor_thrusts_input_ros_msg.thrust_1 = mavlink_msg_quad_rotor_thrusts_get_thrust_1(&mavlink_msg);
  rotor_thrusts_input_ros_msg.thrust_2 = mavlink_msg_quad_rotor_thrusts_get_thrust_2(&mavlink_msg);
  rotor_thrusts_input_ros_msg.thrust_3 = mavlink_msg_quad_rotor_thrusts_get_thrust_3(&mavlink_msg);
  rotor_thrusts_input_ros_msg.thrust_4 = mavlink_msg_quad_rotor_thrusts_get_thrust_4(&mavlink_msg);

  rotor_thrusts_input_pub_.publish(rotor_thrusts_input_ros_msg);
}

void LairdBridge::publishTorquesAndThrustInputMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  // Convert to a Imu ROS message
  quad_msgs::QuadDesiredTorquesAndThrust torques_and_thrust_input_ros_msg;
  torques_and_thrust_input_ros_msg.header.frame_id = ""; //TODO
  torques_and_thrust_input_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    torques_and_thrust_input_ros_msg.header.stamp = ros::Time(
        mavlink_msg_rpg_desired_torques_and_thrust_get_timestamp(&mavlink_msg) / 1000000.0) + px4_time_offset_;
  }
  else
  {
    torques_and_thrust_input_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  torques_and_thrust_input_ros_msg.pitch_torque = mavlink_msg_rpg_desired_torques_and_thrust_get_pitch_torque(
      &mavlink_msg);
  torques_and_thrust_input_ros_msg.roll_torque = mavlink_msg_rpg_desired_torques_and_thrust_get_roll_torque(
      &mavlink_msg);
  torques_and_thrust_input_ros_msg.yaw_torque = mavlink_msg_rpg_desired_torques_and_thrust_get_yaw_torque(&mavlink_msg);
  torques_and_thrust_input_ros_msg.normalized_thrust = mavlink_msg_rpg_desired_torques_and_thrust_get_normalized_thrust(
      &mavlink_msg);

  // Publish ROS message
  torques_and_thrust_input_pub_.publish(torques_and_thrust_input_ros_msg);
}

void LairdBridge::publishOnboardTeraRangerMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  // Convert to a Range ROS message
  sensor_msgs::Range onboard_teraranger_ros_msg;
  onboard_teraranger_ros_msg.header.frame_id = ""; //TODO
  onboard_teraranger_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    onboard_teraranger_ros_msg.header.stamp = ros::Time(
        mavlink_msg_distance_sensor_get_time_boot_ms(&mavlink_msg) / 1000000.0) + px4_time_offset_;
  }
  else
  {
    onboard_teraranger_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  onboard_teraranger_ros_msg.field_of_view = 0.0593;
  onboard_teraranger_ros_msg.max_range = 14.0; // mavlink_msg_distance_sensor_get_max_distance(&mavlink_msg);
  onboard_teraranger_ros_msg.min_range = 0.2; // mavlink_msg_distance_sensor_get_min_distance(&mavlink_msg);
  onboard_teraranger_ros_msg.radiation_type = sensor_msgs::Range::INFRARED;
  onboard_teraranger_ros_msg.range = mavlink_msg_distance_sensor_get_current_distance(&mavlink_msg) / 1000.0;

  // Publish ROS message
  onboard_ranger_pub_.publish(onboard_teraranger_ros_msg);
}

void LairdBridge::publishBarometerMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  // Publish barometer measurement
  sensor_msgs::FluidPressure barometer_ros_msg;
  barometer_ros_msg.header.frame_id = ""; //TODO
  barometer_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    barometer_ros_msg.header.stamp = ros::Time(mavlink_msg_scaled_pressure_get_time_boot_ms(&mavlink_msg) / 1000000.0)
        + px4_time_offset_;
  }
  else
  {
    barometer_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  barometer_ros_msg.fluid_pressure = mavlink_msg_scaled_pressure_get_press_abs(&mavlink_msg);
  barometer_pub_.publish(barometer_ros_msg);

  // Publish temperature measurement (this is done here since the temperature is measured on the barometer)
  sensor_msgs::Temperature temperature_ros_msg;
  temperature_ros_msg.header.frame_id = ""; //TODO
  temperature_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    temperature_ros_msg.header.stamp = ros::Time(mavlink_msg_scaled_pressure_get_time_boot_ms(&mavlink_msg) / 1000000.0)
        + px4_time_offset_;
  }
  else
  {
    temperature_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  temperature_ros_msg.temperature = mavlink_msg_scaled_pressure_get_temperature(&mavlink_msg) / 100.0;
  baro_temperature_pub_.publish(temperature_ros_msg);
}

void LairdBridge::publishOnboardStatusMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  // Publish onboard status
  quad_msgs::OnboardStatus onboard_status_ros_msg;
  onboard_status_ros_msg.header.frame_id = ""; //TODO
  onboard_status_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    onboard_status_ros_msg.header.stamp = ros::Time(
        mavlink_msg_rpg_onboard_status_get_timestamp(&mavlink_msg) / 1000000.0) + px4_time_offset_;
  }
  else
  {
    onboard_status_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  onboard_status_ros_msg.commander_state = mavlink_msg_rpg_onboard_status_get_commander_state(&mavlink_msg);
  onboard_status_ros_msg.battery_state = mavlink_msg_rpg_onboard_status_get_battery_state(&mavlink_msg);
  onboard_status_ros_msg.control_mode = mavlink_msg_rpg_onboard_status_get_control_mode(&mavlink_msg);
  onboard_status_ros_msg.battery_voltage = mavlink_msg_rpg_onboard_status_get_battery_voltage(&mavlink_msg);
  onboard_status_ros_msg.pcb_temperature = mavlink_msg_rpg_onboard_status_get_temperature_pwr(&mavlink_msg);

  onboard_status_pub_.publish(onboard_status_ros_msg);
}

void LairdBridge::publishOnboardAttitudeMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  geometry_msgs::QuaternionStamped onboard_attitude_ros_msg;
  onboard_attitude_ros_msg.header.frame_id = ""; //TODO
  onboard_attitude_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    onboard_attitude_ros_msg.header.stamp = ros::Time(
        mavlink_msg_attitude_quaternion_get_time_boot_ms(&mavlink_msg) / 1000000.0) + px4_time_offset_;
  }
  else
  {
    onboard_attitude_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  onboard_attitude_ros_msg.quaternion.w = mavlink_msg_attitude_quaternion_get_q1(&mavlink_msg);
  onboard_attitude_ros_msg.quaternion.x = mavlink_msg_attitude_quaternion_get_q2(&mavlink_msg);
  onboard_attitude_ros_msg.quaternion.y = mavlink_msg_attitude_quaternion_get_q3(&mavlink_msg);
  onboard_attitude_ros_msg.quaternion.z = mavlink_msg_attitude_quaternion_get_q4(&mavlink_msg);

  onboard_attitude_est_pub_.publish(onboard_attitude_ros_msg);
}

void LairdBridge::publishMagnetometerMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  // Convert to a MagneticField ROS message
  sensor_msgs::MagneticField magnetometer_ros_msg;
  magnetometer_ros_msg.header.frame_id = ""; //TODO
  magnetometer_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    magnetometer_ros_msg.header.stamp = ros::Time(mavlink_msg_magnetic_field_get_timestamp(&mavlink_msg) / 1000000.0)
        + px4_time_offset_;
  }
  else
  {
    magnetometer_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  magnetometer_ros_msg.magnetic_field.x = mavlink_msg_magnetic_field_get_mag_x(&mavlink_msg);
  magnetometer_ros_msg.magnetic_field.y = mavlink_msg_magnetic_field_get_mag_y(&mavlink_msg);
  magnetometer_ros_msg.magnetic_field.z = mavlink_msg_magnetic_field_get_mag_z(&mavlink_msg);

  // Publish ROS message
  magnetometer_pub_.publish(magnetometer_ros_msg);
}

void LairdBridge::publishPx4TimesyncMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  quad_msgs::Px4Timesync sync_msg;

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    sync_msg.header.stamp = ros::Time(mavlink_msg_px4_timesync_get_timestamp(&mavlink_msg) / 1000000.0)
        + px4_time_offset_;
  }
  else
  {
    sync_msg.header.stamp = time_mavlink_msg_received;
  }

  sync_msg.sync_id = mavlink_msg_px4_timesync_get_sync_id(&mavlink_msg);
  sync_msg.time_px4 = mavlink_msg_px4_timesync_get_timestamp(&mavlink_msg);

  // Publish ROS message
  px4_timesync_msg_pub_.publish(sync_msg);
}

void LairdBridge::publishRCMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  // Convert to a Joy ROS message
  sensor_msgs::Joy rc_ros_msg;
  rc_ros_msg.header.frame_id = ""; //TODO
  rc_ros_msg.header.seq = 0; // auto

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    rc_ros_msg.header.stamp = ros::Time(mavlink_msg_rpg_rc_get_timestamp(&mavlink_msg) / 1000000.0) + px4_time_offset_;
  }
  else
  {
    rc_ros_msg.header.stamp = time_mavlink_msg_received;
  }

  rc_ros_msg.axes.push_back(mavlink_msg_rpg_rc_get_channel_1(&mavlink_msg));
  rc_ros_msg.axes.push_back(mavlink_msg_rpg_rc_get_channel_2(&mavlink_msg));
  rc_ros_msg.axes.push_back(mavlink_msg_rpg_rc_get_channel_3(&mavlink_msg));
  rc_ros_msg.axes.push_back(mavlink_msg_rpg_rc_get_channel_4(&mavlink_msg));
  rc_ros_msg.buttons.push_back(mavlink_msg_rpg_rc_get_switch_1(&mavlink_msg));
  rc_ros_msg.buttons.push_back(mavlink_msg_rpg_rc_get_switch_2(&mavlink_msg));
  rc_ros_msg.buttons.push_back(mavlink_msg_rpg_rc_get_switch_3(&mavlink_msg));
  rc_ros_msg.buttons.push_back(mavlink_msg_rpg_rc_get_switch_4(&mavlink_msg));

  // Publish ROS message
  rc_pub_.publish(rc_ros_msg);
}

void LairdBridge::publishControlCommandMsg(const mavlink_message_t& mavlink_msg)
{
  ros::Time time_mavlink_msg_received = ros::Time().now();

  // Convert to a ROS message
  quad_msgs::ControlCommand control_command_msg;

  if (fabs(px4_time_offset_.toSec()) > min_timesync_offset_)
  {
    control_command_msg.header.stamp = ros::Time(
        mavlink_msg_rpg_control_command_get_timestamp(&mavlink_msg) / 1000000.0) + px4_time_offset_;
  }
  else
  {
    control_command_msg.header.stamp = time_mavlink_msg_received;
  }

  control_command_msg.execution_time = ros::Time(
      mavlink_msg_rpg_control_command_get_execution_time(&mavlink_msg) / 1000000.0) + px4_time_offset_;

  int control_mode = mavlink_msg_rpg_control_command_get_control_mode(&mavlink_msg);
  control_command_msg.control_mode = control_command_msg.NONE;
  if (control_mode == 1)
  {
    control_command_msg.control_mode = control_command_msg.ANGLE;
  }
  else if (control_mode == 2)
  {
    control_command_msg.control_mode = control_command_msg.ANGLERATE;
  }

  control_command_msg.off = true;
  if (mavlink_msg_rpg_control_command_get_off(&mavlink_msg) == 0)
  {
    control_command_msg.off = false;
  }

  // Transform thrust direction into quaternion with zero yaw
  Eigen::Vector3d thrust_direction;
  thrust_direction.x() = mavlink_msg_rpg_control_command_get_thrust_dir_x(&mavlink_msg);
  thrust_direction.y() = mavlink_msg_rpg_control_command_get_thrust_dir_y(&mavlink_msg);
  thrust_direction.z() = mavlink_msg_rpg_control_command_get_thrust_dir_z(&mavlink_msg);

  const Eigen::Vector3d z_B = thrust_direction.normalized();
  Eigen::Vector3d x_B = Eigen::Vector3d::UnitY().cross(z_B);
  if (x_B.norm() < 0.001)
  {
    // This can still lead to jumps but should never occur in reasonable flight conditions (at least in angle mode)
    x_B = Eigen::Vector3d::UnitX();
  }
  x_B.normalize();
  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  const Eigen::Quaterniond desired_attitude(R_W_B);

  control_command_msg.orientation = quad_common::eigenToGeometry(Eigen::Quaterniond(R_W_B));
  control_command_msg.bodyrates.x = mavlink_msg_rpg_control_command_get_bodyrates_x(&mavlink_msg);
  control_command_msg.bodyrates.y = mavlink_msg_rpg_control_command_get_bodyrates_y(&mavlink_msg);
  control_command_msg.bodyrates.z = mavlink_msg_rpg_control_command_get_bodyrates_z(&mavlink_msg);
  control_command_msg.angular_accelerations.x = mavlink_msg_rpg_control_command_get_angular_acc_x(&mavlink_msg);
  control_command_msg.angular_accelerations.y = mavlink_msg_rpg_control_command_get_angular_acc_y(&mavlink_msg);
  control_command_msg.angular_accelerations.z = mavlink_msg_rpg_control_command_get_angular_acc_z(&mavlink_msg);
  control_command_msg.thrust = mavlink_msg_rpg_control_command_get_thrust(&mavlink_msg);

  control_command_pub_.publish(control_command_msg);
}

// Subscriber Callbacks
void LairdBridge::controlCommandCallback(const quad_msgs::ControlCommand::ConstPtr& msg)
{
  if (control_active_)
  {
    mavlink_message_t mavlink_msg;

    uint8_t control_mode = 0;
    if (msg->control_mode == msg->ANGLE)
    {
      control_mode = 1;
    }
    else if (msg->control_mode == msg->ANGLERATE)
    {
      control_mode = 2;
    }
    uint8_t control_off = 1;
    if (!msg->off)
    {
      control_off = 0;
    }

    const Eigen::Quaterniond q_W_B = quad_common::geometryToEigen(msg->orientation);
    Eigen::Vector3d thrust_direction = q_W_B * Eigen::Vector3d::UnitZ();

    // Un-rotate thrust direction by heading quaternion
    const double heading = quad_common::quaternionToEulerAnglesZYX(q_W_B).z();
    const Eigen::Quaterniond q_heading = Eigen::Quaterniond(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
    thrust_direction = q_heading.inverse() * thrust_direction;

    mavlink_msg_rpg_control_command_pack(basecomputer_sys_id_, basecomputer_comp_id_, &mavlink_msg,
                                         (msg->header.stamp.toNSec() - px4_time_offset_.toNSec()) / 1000,
                                         (msg->execution_time.toNSec() - px4_time_offset_.toNSec()) / 1000,
                                         control_mode, control_off, thrust_direction.x(), thrust_direction.y(),
                                         thrust_direction.z(), msg->bodyrates.x, msg->bodyrates.y, msg->bodyrates.z,
                                         msg->angular_accelerations.x, msg->angular_accelerations.y,
                                         msg->angular_accelerations.z, msg->thrust);

    sendMavlinkMessage(mavlink_msg);
  }
  else if (!msg->off)
  {
    ROS_WARN_THROTTLE(1, "[%s] Control not activated!", pnh_.getNamespace().c_str());
  }
}

void LairdBridge::gpioPwmControlCallback(const quad_msgs::QuadGpioPwmCtrlConstPtr& msg)
{
  if (control_active_)
  {
    mavlink_message_t mavlink_msg;

    mavlink_msg_rpg_gpio_ctrl_pack(basecomputer_sys_id_, basecomputer_comp_id_, &mavlink_msg, 0, msg->device,
                                   msg->channel, msg->value);

    // Only print message for GPIO operation:
    if (msg->value >= 0.0f && msg->value <= 1.0f)
    {
      ROS_INFO("[%s] Set channel %i of device %i to %f", pnh_.getNamespace().c_str(), msg->channel, msg->device,
               msg->value);
    }

    sendMavlinkMessage(mavlink_msg);
  }
  else
  {
    ROS_WARN("[%s] Control not activated!", pnh_.getNamespace().c_str());
  }
}

void LairdBridge::setControlActiveCallback(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data == false)
  {
    control_active_ = false;
    ROS_INFO("[%s] Control inactive", pnh_.getNamespace().c_str());
  }
  else
  {
    control_active_ = true;
    ROS_INFO("[%s] Control active", pnh_.getNamespace().c_str());
  }
}

void LairdBridge::px4TimeOffsetCallback(const std_msgs::Duration::ConstPtr& msg)
{
  px4_time_offset_ = msg->data;
}

bool LairdBridge::reloadParameters()
{
  if (!getParam("port_name", port_name_, pnh_))
    return false;
  if (!getParam("baud_rate", baud_rate_, pnh_))
    return false;
  if (!getParam("basecomputer_comp_id", basecomputer_comp_id_, pnh_))
    return false;
  if (!getParam("basecomputer_sys_id", basecomputer_sys_id_, pnh_))
    return false;
  if (!getParam("px4_comp_id", px4_comp_id_, pnh_))
    return false;
  if (!getParam("px4_sys_id", px4_sys_id_, pnh_))
    return false;

  getParam("publish_sensors", publish_sensors_, true);
  getParam("accept_commands", accept_commands_, true);
  getParam("handle_parameters", handle_parameters_, true);
  getParam("publish_commands", publish_commands_, false);
  if (accept_commands_)
  {
    // If we accept commands, there is no way we also want to receive commands since they would be published
    // on the same topic otherwise
    publish_commands_ = false;
  }

  return true;
}

} // namespace laird_bridge

