#pragma once

#include <mutex>
#include <thread>

#include <quad_msgs/ControlCommand.h>
#include <sbus_bridge/sbus_serial_port.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "sbus_bridge/sbus_msg.h"

namespace sbus_bridge
{

enum class BridgeState
{
  OFF, ARMING, AUTONOMOUS_FLIGHT, RC_FLIGHT
};

enum class ControlMode
{
  RATE, ANGLE
};

enum class ArmState
{
  DISARMED, ARMED
};

class SBusBridge : public SBusSerialPort
{
public:

  SBusBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  SBusBridge() :
      SBusBridge(ros::NodeHandle(), ros::NodeHandle("~"))
  {
  }

  virtual ~SBusBridge();

private:

  void watchdogThread();

  void handleReceivedSbusMessage(const SBusMsg& received_sbus_msg);
  void controlCommandCallback(const quad_msgs::ControlCommand::ConstPtr& msg);
  void sendSBusMessageToSerialPort(const SBusMsg& sbus_msg);

  void armBridgeCallback(const std_msgs::BoolConstPtr& msg);
  void batteryVoltageCallback(const std_msgs::Float32::ConstPtr& msg);
  void publishOnboardStatus(const ros::TimerEvent& time) const;

  uint16_t inverseThrustMapping(const double thrust) const;
  void limitAllChannelsFeasible(SBusMsg* sbus_msg) const;
  void limitSbusChannel(uint16_t* channel_value) const;

  // Setting sbus command helpers
  void setThrottleCommand(const uint16_t throttle_cmd, SBusMsg* sbus_msg) const;
  void enforceSpinningThrottleCommand(SBusMsg* sbus_msg) const;
  void setRollCommand(const uint16_t roll_cmd, SBusMsg* sbus_msg) const;
  void setPitchCommand(const uint16_t pitch_cmd, SBusMsg* sbus_msg) const;
  void setYawCommand(const uint16_t yaw_cmd, SBusMsg* sbus_msg) const;
  void setControlMode(const ControlMode& mode, SBusMsg* sbus_msg) const;
  void setArmState(const ArmState& arm_state, SBusMsg* sbus_msg) const;

  // Sbus message check helpers
  bool isArmed(const SBusMsg& sbus_msg) const;
  ControlMode getControlModeFromSBusMessage(const SBusMsg& sbus_msg) const;

  bool loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  mutable std::mutex mutex_;

  // Publishers
  ros::Publisher onboard_status_pub_;
  ros::Publisher received_sbus_msg_pub_;

  // Subscribers
  ros::Subscriber control_command_sub_;
  ros::Subscriber arm_bridge_sub_;
  ros::Subscriber battery_voltage_sub_;

  // Timer
  ros::Timer onboard_status_pub_timer_;

  // Watchdog
  std::thread watchdog_thread_;
  bool stop_watchdog_thread_;
  ros::Time time_last_rc_msg_received_;
  ros::Time time_last_sbus_msg_sent_;
  ros::Time time_last_battery_voltage_received_;
  ros::Time time_last_active_control_command_received_;

  BridgeState bridge_state_;
  bool bridge_armed_;
  ControlMode control_mode_;
  int arming_counter_;
  double battery_voltage_;

  // Parameters
  std::string port_name_;
  bool enable_receiving_sbus_messages_;

  double control_command_timeout_;
  double rc_timeout_;

  double mass_;

  bool disable_thrust_mapping_;
  double thrust_map_a_;
  double thrust_map_b_;
  double thrust_map_c_;

  double max_roll_rate_;
  double max_pitch_rate_;
  double max_yaw_rate_;

  double max_roll_angle_;
  double max_pitch_angle_;

  double alpha_vbat_filter_;
  bool perform_thrust_voltage_compensation_;
  double thrust_ratio_voltage_map_a_;
  double thrust_ratio_voltage_map_b_;

  // Constants
  static constexpr double kOnboardStatusPublishFrequency_ = 50.0;

  static constexpr int kSmoothingFailRepetitions_ = 5;

  static constexpr double kBatteryLowVoltage_ = 10.7;
  static constexpr double kBatteryCriticalVoltage_ = 10.5;
  static constexpr double kBatteryInvalidVoltage_ = 9.0;
  static constexpr double kMinBatteryCompensationVoltage_ = 10.6;
  static constexpr double kMaxBatteryCompensationVoltage_ = 12.6;
  static constexpr double kBatteryVoltageTimeout_ = 1.0;
};

} // namespace sbus_bridge
