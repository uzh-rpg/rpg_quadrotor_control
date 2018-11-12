/*
 * PixHawkBridge.h
 *
 *  Created on: Jan 15, 2014
 *      Author: ffontana
 */

#pragma once

#include "ros/ros.h"
#include <Eigen/Dense>

#include "mavlink/v1.0/rpg/mavlink.h"

#include "quad_msgs/ControlCommand.h"
#include "quad_msgs/QuadStateEstimate.h"
#include "quad_msgs/QuadRotorThrusts.h"
#include "quad_msgs/QuadDesiredTorquesAndThrust.h"
#include "quad_msgs/Px4Timesync.h"
#include "quad_msgs/OnboardStatus.h"
#include "quad_msgs/QuadGpioPwmCtrl.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Duration.h>

#include "geometry_msgs/QuaternionStamped.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Joy.h"

#include <quad_common/math_common.h>
#include <quad_common/parameter_helper.h>

#include <quad_common/geometry_eigen_conversions.h>

#include "geometry_msgs/PointStamped.h"

namespace pixhawk_bridge
{

class SerialPort;
class ParameterHandler;

class PixHawkBridge
{
public:
  PixHawkBridge(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  PixHawkBridge() :
      PixHawkBridge(ros::NodeHandle(), ros::NodeHandle("~"))
  {
  }

  virtual ~PixHawkBridge();

  void handleMavlinkMessage(const mavlink_message_t & mavlink_msg);
  void sendMavlinkMessage(const mavlink_message_t & mavlink_msg);

private:
  // Publisher Functions
  void publishImuMsg(const mavlink_message_t& mavlink_msg);
  void publishOnboardAttitudeMsg(const mavlink_message_t& mavlink_msg);

  void publishRotorThrustsInputMsg(const mavlink_message_t& mavlink_msg);
  void publishTorquesAndThrustInputMsg(const mavlink_message_t& mavling_msg);
  void publishOnboardTeraRangerMsg(const mavlink_message_t& mavling_msg);
  void publishBarometerMsg(const mavlink_message_t& mavlink_msg);
  void publishOnboardStatusMsg(const mavlink_message_t & mavlink_msg);
  void publishMagnetometerMsg(const mavlink_message_t& mavlink_msg);
  void publishPx4TimesyncMsg(const mavlink_message_t& mavlink_msg);
  void publishRCMsg(const mavlink_message_t& mavlink_msg);
  void publishControlCommandMsg(const mavlink_message_t& mavlink_msg);

  // Subscriber Callbacks
  void gpioPwmControlCallback(const quad_msgs::QuadGpioPwmCtrlConstPtr& msg);
  void setControlActiveCallback(const std_msgs::BoolConstPtr &msg);
  void controlCommandCallback(const quad_msgs::ControlCommand::ConstPtr& msg);
  void px4TimeOffsetCallback(const std_msgs::Duration::ConstPtr& msg);

  // Helpers
  bool reloadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  SerialPort * serial_port_;
  ParameterHandler * parameter_handler_;

  // Publishers
  ros::Publisher imu_pub_;
  ros::Publisher onboard_attitude_est_pub_;

  ros::Publisher rotor_thrusts_input_pub_;
  ros::Publisher torques_and_thrust_input_pub_;
  ros::Publisher onboard_ranger_pub_;
  ros::Publisher barometer_pub_;
  ros::Publisher baro_temperature_pub_;
  ros::Publisher onboard_status_pub_;
  ros::Publisher magnetometer_pub_;
  ros::Publisher rc_pub_;
  ros::Publisher px4_timesync_msg_pub_;
  ros::Publisher control_command_pub_;

  // Subscribers
  ros::Subscriber control_command_sub_;
  ros::Subscriber set_control_active_sub_;
  ros::Subscriber px4_time_offset_sub_;
  ros::Subscriber gpio_pwm_control_sub_;

  std::string port_name_;
  int baud_rate_;
  int basecomputer_sys_id_;
  int basecomputer_comp_id_;
  int px4_sys_id_;
  int px4_comp_id_;
  bool control_active_;

  // PX4 Timesync
  ros::Duration px4_time_offset_;
  const double min_timesync_offset_;

  bool publish_sensors_;
  bool accept_commands_;
  bool handle_parameters_;
  bool publish_commands_;
};

} // namespace pixhawk_bridge
