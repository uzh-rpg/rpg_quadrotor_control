#pragma once

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sbus_bridge/SbusRosMessage.h>
#include <sensor_msgs/Joy.h>

namespace manual_flight_assistant {

class ManualFlightAssistant {
 public:
  ManualFlightAssistant(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ManualFlightAssistant()
      : ManualFlightAssistant(ros::NodeHandle(), ros::NodeHandle("~")) {}
  virtual ~ManualFlightAssistant();

 private:
  void mainLoop(const ros::TimerEvent& time);

  void rcSbusCallback(const sbus_bridge::SbusRosMessage::ConstPtr& msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  bool joypadAvailable();
  bool rcSbusAvailable();

  void publishVelocityCommand(
      const geometry_msgs::TwistStamped& velocity_command);

  bool loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher manual_desired_velocity_pub_;
  ros::Publisher start_pub_;
  ros::Publisher land_pub_;

  ros::Subscriber joypad_sub_;
  ros::Subscriber rc_sbus_sub_;

  ros::Timer main_loop_timer_;

  sensor_msgs::Joy joypad_command_;
  sensor_msgs::Joy previous_joypad_command_;
  ros::Time time_last_joypad_msg_;

  sbus_bridge::SbusRosMessage sbus_command_;
  sbus_bridge::SbusRosMessage previous_sbus_command_;
  ros::Time time_last_sbus_msg_;
  bool sbus_needs_to_go_through_zero_;
  bool velocity_command_is_zero_;

  // Parameters
  double joypad_timeout_;
  double sbus_timeout_;
  double joypad_axes_zero_tolerance_;
  int sbus_axes_zero_tolerance_;
  double vmax_xy_;
  double vmax_z_;
  double rmax_yaw_;

  // Constants
  static constexpr double kLoopFrequency_ = 50.0;
  static constexpr double kVelocityCommandZeroThreshold_ = 0.03;
};

}  // namespace manual_flight_assistant
