#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include <quadrotor_msgs/ControlCommand.h>
#include <ros/ros.h>
#include <sbus_bridge/sbus_serial_port.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "sbus_bridge/sbus_msg.h"
#include "sbus_bridge/thrust_mapping.h"

namespace sbus_bridge {

enum class BridgeState { OFF, ARMING, AUTONOMOUS_FLIGHT, RC_FLIGHT };

class SBusBridge : public SBusSerialPort {
 public:
  SBusBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  SBusBridge() : SBusBridge(ros::NodeHandle(), ros::NodeHandle("~")) {}

  virtual ~SBusBridge();

 private:
  void watchdogThread();

  void handleReceivedSbusMessage(const SBusMsg& received_sbus_msg) override;
  void controlCommandCallback(
      const quadrotor_msgs::ControlCommand::ConstPtr& msg);
  void sendSBusMessageToSerialPort(const SBusMsg& sbus_msg);

  SBusMsg generateSBusMessageFromControlCommand(
      const quadrotor_msgs::ControlCommand::ConstPtr& control_command) const;

  void setBridgeState(const BridgeState& desired_bridge_state);

  void armBridgeCallback(const std_msgs::Bool::ConstPtr& msg);
  void batteryVoltageCallback(const std_msgs::Float32::ConstPtr& msg);
  void publishLowLevelFeedback(const ros::TimerEvent& time) const;

  bool loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Mutex for:
  // - bridge_state_
  // - control_mode_
  // - bridge_armed_
  // - time_last_active_control_command_received_
  // - time_last_rc_msg_received_
  // - arming_counter_
  // - time_last_sbus_msg_sent_
  // Also "setBridgeState" and "sendSBusMessageToSerialPort" should only be
  // called when "main_mutex_" is locked
  mutable std::mutex main_mutex_;
  // Mutex for:
  // - battery_voltage_
  // - time_last_battery_voltage_received_
  // Also "generateSBusMessageFromControlCommand" should only be called when
  // "battery_voltage_mutex_" is locked
  mutable std::mutex battery_voltage_mutex_;

  // Publishers
  ros::Publisher low_level_feedback_pub_;
  ros::Publisher received_sbus_msg_pub_;

  // Subscribers
  ros::Subscriber control_command_sub_;
  ros::Subscriber arm_bridge_sub_;
  ros::Subscriber battery_voltage_sub_;

  // Timer
  ros::Timer low_level_feedback_pub_timer_;

  // Watchdog
  std::thread watchdog_thread_;
  std::atomic_bool stop_watchdog_thread_;
  ros::Time time_last_rc_msg_received_;
  ros::Time time_last_sbus_msg_sent_;
  ros::Time time_last_battery_voltage_received_;
  ros::Time time_last_active_control_command_received_;

  BridgeState bridge_state_;
  ControlMode control_mode_;
  int arming_counter_;
  double battery_voltage_;

  // Safety flags
  bool bridge_armed_;
  bool rc_was_disarmed_once_;

  std::atomic_bool destructor_invoked_;

  thrust_mapping::CollectiveThrustMapping thrust_mapping_;

  // Parameters
  std::string port_name_;
  bool enable_receiving_sbus_messages_;

  double control_command_timeout_;
  double rc_timeout_;

  double mass_;

  bool disable_thrust_mapping_;

  double max_roll_rate_;
  double max_pitch_rate_;
  double max_yaw_rate_;

  double max_roll_angle_;
  double max_pitch_angle_;

  double alpha_vbat_filter_;
  bool perform_thrust_voltage_compensation_;
  int n_lipo_cells_;

  // Constants
  static constexpr double kLowLevelFeedbackPublishFrequency_ = 50.0;

  static constexpr int kSmoothingFailRepetitions_ = 5;

  static constexpr double kBatteryLowVoltagePerCell_ = 3.6;
  static constexpr double kBatteryCriticalVoltagePerCell_ = 3.4;
  static constexpr double kBatteryInvalidVoltagePerCell_ = 3.0;
  static constexpr double kBatteryVoltageTimeout_ = 1.0;
};

}  // namespace sbus_bridge
