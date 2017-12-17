#include "sbus_bridge/sbus_bridge.h"

#include <quad_common/geometry_eigen_conversions.h>
#include <quad_common/math_common.h>
#include <quad_common/parameter_helper.h>
#include <quad_msgs/OnboardStatus.h>

#include "sbus_bridge/channel_mapping.h"
#include "sbus_bridge/SbusRosMessage.h"

namespace sbus_bridge
{

SBusBridge::SBusBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) :
    nh_(nh), pnh_(pnh), stop_watchdog_thread_(false), bridge_state_(BridgeState::OFF), bridge_armed_(false), control_mode_(
        ControlMode::RATE), arming_counter_(0), battery_voltage_(0.0)
{
  if (!loadParameters())
  {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  if (disable_thrust_mapping_)
  {
    ROS_WARN("[%s] Thrust mapping disabled!", pnh_.getNamespace().c_str());
  }

  // Publishers
  onboard_status_pub_ = nh_.advertise<quad_msgs::OnboardStatus>("onboard_status", 1);
  if (enable_receiving_sbus_messages_)
  {
    received_sbus_msg_pub_ = nh_.advertise<sbus_bridge::SbusRosMessage>("received_sbus_message", 1);
  }

  if (!setUpSBusSerialPort(port_name_, enable_receiving_sbus_messages_))
  {
    ros::shutdown();
  }

  // Subscribers
  arm_bridge_sub_ = nh_.subscribe("sbus_bridge/arm", 1, &SBusBridge::armBridgeCallback, this);
  control_command_sub_ = nh_.subscribe("control_command", 1, &SBusBridge::controlCommandCallback, this);
  battery_voltage_sub_ = nh_.subscribe("battery_voltage", 1, &SBusBridge::batteryVoltageCallback, this);

  onboard_status_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / kOnboardStatusPublishFrequency_),
                                              &SBusBridge::publishOnboardStatus, this);

  // Start watchdog thread
  watchdog_thread_ = std::thread(&SBusBridge::watchdogThread, this);

  if (!watchdog_thread_.joinable())
  {
    ROS_ERROR("[%s] Could not successfully start watchdog thread.", pnh_.getNamespace().c_str());
    ros::shutdown();
  }
}

SBusBridge::~SBusBridge()
{
  // Stop SBus receiver thread
  if (enable_receiving_sbus_messages_)
  {
    stopReceiverThread();
  }

  bridge_state_ = BridgeState::OFF;

  // Send disarming SBus message for safety
  // We repeat it to prevent any possible smoothing of commands on the flight controller to interfere with this
  SBusMsg shut_down_message;
  setThrottleCommand(kSbusMinCmd, &shut_down_message);
  setArmState(ArmState::DISARMED, &shut_down_message);
  ros::Rate loop_rate(110.0);
  for (int i = 0; i < kSmoothingFailRepetitions_; i++)
  {
    transmitSerialSBusMessage(shut_down_message);
    loop_rate.sleep();
  }

  // Close serial port
  disconnectSerialPort();

  // Stop watchdog thread
  stop_watchdog_thread_ = true;
  // Wait for watchdog thread to finish
  watchdog_thread_.join();
}

void SBusBridge::watchdogThread()
{
  ros::Rate watchdog_rate(100.0);
  while (ros::ok() && !stop_watchdog_thread_)
  {
    watchdog_rate.sleep();

    std::unique_lock<std::mutex> lock(mutex_);

    const ros::Time time_now = ros::Time::now();

    if (bridge_state_ == BridgeState::RC_FLIGHT && time_now - time_last_rc_msg_received_ > ros::Duration(rc_timeout_))
    {
      // If the last received RC command was armed but was received longer than rc_timeout ago we switch the bridge
      // state to AUTONOMOUS_FLIGHT. In case there are no valid control commands the bridge state is set to OFF
      // in the next check below
      bridge_state_ = BridgeState::AUTONOMOUS_FLIGHT;
    }

    if (bridge_state_ == BridgeState::ARMING || bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT)
    {
      if (time_now - time_last_active_control_command_received_ > ros::Duration(control_command_timeout_))
      {
        bridge_state_ = BridgeState::OFF;
        // Make sure off message is sent
        SBusMsg off_msg;
        setThrottleCommand(kSbusMinCmd, &off_msg);
        setArmState(ArmState::DISARMED, &off_msg);
        sendSBusMessageToSerialPort(off_msg);
      }
    }

    // Check battery voltage timeout
    if (time_now - time_last_battery_voltage_received_ > ros::Duration(kBatteryVoltageTimeout_))
    {
      battery_voltage_ = 0.0;
      if (perform_thrust_voltage_compensation_)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "[%s] Can not perform battery voltage compensation because there is no recent battery voltage measurement",
            pnh_.getNamespace().c_str());
      }
    }

    // mutex is unlocked because unique_lock goes out of scope here
  }
}

void SBusBridge::handleReceivedSbusMessage(const SBusMsg& received_sbus_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  time_last_rc_msg_received_ = ros::Time::now();

  if (isArmed(received_sbus_msg))
  {
    // Immediately go into RC_FLIGHT state since RC always has priority
    bridge_state_ = BridgeState::RC_FLIGHT;
    sendSBusMessageToSerialPort(received_sbus_msg);
    control_mode_ = getControlModeFromSBusMessage(received_sbus_msg);
  }
  else if (bridge_state_ == BridgeState::RC_FLIGHT)
  {
    // If the bridge was in state RC_FLIGHT and the RC is disarmed we set the state to AUTONOMOUS_FLIGHT
    // In case there are valid control commands, the bridge will stay in AUTONOMOUS_FLIGHT, otherwise the watchdog
    // will set the state to OFF
    bridge_state_ = BridgeState::AUTONOMOUS_FLIGHT;
  }

  received_sbus_msg_pub_.publish(received_sbus_msg.toRosMessage());
}

void SBusBridge::controlCommandCallback(const quad_msgs::ControlCommand::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!msg->off)
  {
    // We only need to know when the last active control command was received. If it is not active, the bridge state
    // will go to off and we keep sending an off command with every new control command received.
    // This prevents the flight controller from going into failsafe
    time_last_active_control_command_received_ = ros::Time::now();
  }

  if (!bridge_armed_ || bridge_state_ == BridgeState::RC_FLIGHT)
  {
    // If bridge is not armed we do not allow control commands to be sent
    // RC has priority over control commands for autonomous flying
    return;
  }

  // Set commands
  SBusMsg sbus_msg_to_send;
  setArmState(ArmState::ARMED, &sbus_msg_to_send);
  if (disable_thrust_mapping_)
  {
    setThrottleCommand(int(msg->thrust), &sbus_msg_to_send);
    setRollCommand(kSbusMeanCmd, &sbus_msg_to_send);
    setPitchCommand(kSbusMeanCmd, &sbus_msg_to_send);
    setYawCommand(kSbusMeanCmd, &sbus_msg_to_send);
    setControlMode(ControlMode::RATE, &sbus_msg_to_send);
  }
  else
  {
    if (msg->control_mode == msg->ANGLE)
    {
      setControlMode(ControlMode::ANGLE, &sbus_msg_to_send);
      setThrottleCommand(inverseThrustMapping(msg->thrust * mass_), &sbus_msg_to_send);

      Eigen::Vector3d desired_euler_angles = quad_common::quaternionToEulerAnglesZYX(
          quad_common::geometryToEigen(msg->orientation));

      quad_common::limit(desired_euler_angles(0), -max_roll_angle_, max_roll_angle_);
      quad_common::limit(desired_euler_angles(1), -max_pitch_angle_, max_pitch_angle_);

      setRollCommand(round((desired_euler_angles(0) / max_roll_angle_) * (kSbusMaxCmd - kSbusMeanCmd) + kSbusMeanCmd),
                     &sbus_msg_to_send);
      setPitchCommand(round((desired_euler_angles(1) / max_pitch_angle_) * (kSbusMaxCmd - kSbusMeanCmd) + kSbusMeanCmd),
                      &sbus_msg_to_send);
      setYawCommand(round((-msg->bodyrates.z / max_yaw_rate_) * (kSbusMaxCmd - kSbusMeanCmd) + kSbusMeanCmd),
                    &sbus_msg_to_send);
    }
    else
    {
      setControlMode(ControlMode::RATE, &sbus_msg_to_send);
      setThrottleCommand(inverseThrustMapping(msg->thrust * mass_), &sbus_msg_to_send);
      setRollCommand(round((msg->bodyrates.x / max_roll_rate_) * (kSbusMaxCmd - kSbusMeanCmd) + kSbusMeanCmd),
                     &sbus_msg_to_send);
      setPitchCommand(round((msg->bodyrates.y / max_pitch_rate_) * (kSbusMaxCmd - kSbusMeanCmd) + kSbusMeanCmd),
                      &sbus_msg_to_send);
      setYawCommand(round((-msg->bodyrates.z / max_yaw_rate_) * (kSbusMaxCmd - kSbusMeanCmd) + kSbusMeanCmd),
                    &sbus_msg_to_send);
    }
  }

  // Set to arming state or ensure disarmed command if necessary
  if (!msg->off)
  {
    if (bridge_state_ != BridgeState::ARMING && bridge_state_ != BridgeState::AUTONOMOUS_FLIGHT)
    {
      bridge_state_ = BridgeState::ARMING;
      arming_counter_ = 0;
    }
  }
  else
  {
    // Make sure vehicle is disarmed
    setThrottleCommand(kSbusMinCmd, &sbus_msg_to_send);
    setArmState(ArmState::DISARMED, &sbus_msg_to_send);
  }

  // Limit channels
  limitAllChannelsFeasible(&sbus_msg_to_send);

  // Immediately send SBus message
  sendSBusMessageToSerialPort(sbus_msg_to_send);

  // Set control mode for onboard status message to be published
  if (msg->control_mode == msg->ANGLE)
  {
    control_mode_ = ControlMode::ANGLE;
  }
  else
  {
    control_mode_ = ControlMode::RATE;
  }
}

void SBusBridge::sendSBusMessageToSerialPort(const SBusMsg& sbus_msg)
{
  SBusMsg sbus_message_to_send = sbus_msg;

  switch (bridge_state_)
  {
    case BridgeState::OFF:
      // Disarm vehicle
      setThrottleCommand(kSbusMinCmd, &sbus_message_to_send);
      setArmState(ArmState::DISARMED, &sbus_message_to_send);
      break;

    case BridgeState::ARMING:
      // Since there might be some RC commands smoothing and checks on multiple messages before arming, we repeat
      // the messages with minimum throttle and arming command multiple times. 5 times seems to work robustly.
      if (arming_counter_ >= kSmoothingFailRepetitions_)
      {
        bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT;
        arming_counter_ = 0;
      }
      else
      {
        // Set thrust to minimum command to make sure FC arms
        setThrottleCommand(kSbusMinCmd, &sbus_message_to_send);
        setArmState(ArmState::ARMED, &sbus_message_to_send);
        arming_counter_++;
      }
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      setArmState(ArmState::ARMED, &sbus_message_to_send);
      // If vehicle is armed force thrust command to be at least minimum spinning command
      enforceSpinningThrottleCommand(&sbus_message_to_send);
      break;

    case BridgeState::RC_FLIGHT:
      // Passing RC command straight through
      break;
  }

  if ((ros::Time::now() - time_last_sbus_msg_sent_).toSec() <= 0.006)
  {
    // An SBUS message takes 3ms to be transmitted by the serial port so let's not stress it too much
    // This should only happen in case of switching between control commands and rc commands
    return;
  }

  sbus_message_to_send.timestamp = ros::Time::now();
  transmitSerialSBusMessage(sbus_message_to_send);
  time_last_sbus_msg_sent_ = ros::Time::now();
}

void SBusBridge::armBridgeCallback(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data)
  {
    bridge_armed_ = true;
    ROS_INFO("[%s] Bridge armed", pnh_.getNamespace().c_str());
  }
  else
  {
    bridge_armed_ = false;
    if (bridge_state_ == BridgeState::ARMING || bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT)
    {
      bridge_state_ == BridgeState::OFF;
    }
    ROS_INFO("[%s] Bridge disarmed", pnh_.getNamespace().c_str());
  }
}

void SBusBridge::batteryVoltageCallback(const std_msgs::Float32::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (battery_voltage_ != 0.0)
  {
    battery_voltage_ = alpha_vbat_filter_ * msg->data + (1.0 - alpha_vbat_filter_) * battery_voltage_;
  }
  else
  {
    battery_voltage_ = msg->data;
  }
  time_last_battery_voltage_received_ = ros::Time::now();
}

void SBusBridge::publishOnboardStatus(const ros::TimerEvent& time) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Publish an onboard status message
  quad_msgs::OnboardStatus onboard_status_msg;
  onboard_status_msg.header.stamp = ros::Time::now();
  onboard_status_msg.battery_voltage = battery_voltage_;
  if (battery_voltage_ > kBatteryLowVoltage_)
  {
    onboard_status_msg.battery_state = onboard_status_msg.GOOD;
  }
  else if (battery_voltage_ > kBatteryCriticalVoltage_)
  {
    onboard_status_msg.battery_state = onboard_status_msg.LOW;
  }
  else if (battery_voltage_ > kBatteryInvalidVoltage_)
  {
    onboard_status_msg.battery_state = onboard_status_msg.CRITICAL;
  }
  else
  {
    onboard_status_msg.battery_state = onboard_status_msg.INVALID;
  }

  onboard_status_msg.commander_state = onboard_status_msg.LANDED;
  if (bridge_state_ != BridgeState::RC_FLIGHT)
  {
    onboard_status_msg.commander_state = onboard_status_msg.MANUAL_FLYING;
  }
  else if (bridge_state_ != BridgeState::AUTONOMOUS_FLIGHT)
  {
    onboard_status_msg.commander_state = onboard_status_msg.AUTONOMOUS_FLYING;
  }

  onboard_status_msg.control_mode = onboard_status_msg.RATE_MODE;
  if (control_mode_ == ControlMode::ANGLE)
  {
    onboard_status_msg.control_mode = onboard_status_msg.ATTITUDE_MODE;
  }

  onboard_status_pub_.publish(onboard_status_msg);
}

uint16_t SBusBridge::inverseThrustMapping(const double thrust) const
{
  double thrust_applied = thrust;
  if (perform_thrust_voltage_compensation_)
  {
    if (battery_voltage_ >= kMinBatteryCompensationVoltage_ && battery_voltage_ <= kMaxBatteryCompensationVoltage_)
    {
      const double thrust_cmd_voltage_ratio = thrust_ratio_voltage_map_a_ * battery_voltage_
          + thrust_ratio_voltage_map_b_;
      thrust_applied *= thrust_cmd_voltage_ratio;
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "[%s] Battery voltage out of range for compensation", pnh_.getNamespace().c_str());
    }
  }

  const uint16_t cmd = (-thrust_map_b_
      + sqrt(thrust_map_b_ * thrust_map_b_ - 4.0 * thrust_map_a_ * (thrust_map_c_ - thrust_applied)))
      / (2.0 * thrust_map_a_);

  return cmd;
}

void SBusBridge::limitAllChannelsFeasible(SBusMsg* sbus_msg) const
{
  for (uint8_t i = 0; i < kSbusNChannels; i++)
  {
    limitSbusChannel(&sbus_msg->channels[i]);
  }
}

void SBusBridge::limitSbusChannel(uint16_t* channel_value) const
{
  if (*channel_value > kSbusMaxCmd)
  {
    *channel_value = kSbusMaxCmd;
  }
  if (*channel_value < kSbusMinCmd)
  {
    *channel_value = kSbusMinCmd;
  }
}

void SBusBridge::setThrottleCommand(const uint16_t throttle_cmd, SBusMsg* sbus_msg) const
{
  sbus_msg->channels[channel_mapping::kThrottleChannel] = throttle_cmd;
}

void SBusBridge::enforceSpinningThrottleCommand(SBusMsg* sbus_msg) const
{
  if (sbus_msg->channels[channel_mapping::kThrottleChannel] < kSbusMinSpinCmd)
  {
    sbus_msg->channels[channel_mapping::kThrottleChannel] = kSbusMinSpinCmd;
  }
}

void SBusBridge::setRollCommand(const uint16_t roll_cmd, SBusMsg* sbus_msg) const
{
  sbus_msg->channels[channel_mapping::kRollChannel] = roll_cmd;
}

void SBusBridge::setPitchCommand(const uint16_t pitch_cmd, SBusMsg* sbus_msg) const
{
  sbus_msg->channels[channel_mapping::kPitchChannel] = pitch_cmd;
}

void SBusBridge::setYawCommand(const uint16_t yaw_cmd, SBusMsg* sbus_msg) const
{
  sbus_msg->channels[channel_mapping::kYawChannel] = yaw_cmd;
}

void SBusBridge::setControlMode(const ControlMode& mode, SBusMsg* sbus_msg) const
{
  if (mode == ControlMode::ANGLE)
  {
    sbus_msg->channels[channel_mapping::kControlModeChannel] = kSbusMinCmd; // Set angle control mode
  }
  else
  {
    sbus_msg->channels[channel_mapping::kControlModeChannel] = kSbusMaxCmd; // Set rate control mode
  }
}

void SBusBridge::setArmState(const ArmState& arm_state, SBusMsg* sbus_msg) const
{
  if (arm_state == ArmState::ARMED)
  {
    sbus_msg->channels[channel_mapping::kArmingChannel] = kSbusMaxCmd; // Arm the vehicle
  }
  else
  {
    sbus_msg->channels[channel_mapping::kArmingChannel] = kSbusMinCmd; // Disarm the vehicle
  }
}

bool SBusBridge::isArmed(const SBusMsg& sbus_msg) const
{
  if (sbus_msg.channels[channel_mapping::kArmingChannel] <= kSbusMeanCmd)
  {
    return false;
  }

  return true;
}

ControlMode SBusBridge::getControlModeFromSBusMessage(const SBusMsg& sbus_msg) const
{
  if (sbus_msg.channels[channel_mapping::kControlModeChannel] > kSbusMeanCmd)
  {
    return ControlMode::RATE;
  }

  return ControlMode::ANGLE;
}

bool SBusBridge::loadParameters()
{
  if (!quad_common::getParam("port_name", port_name_, pnh_))
    return false;
  if (!quad_common::getParam("enable_receiving_sbus_messages", enable_receiving_sbus_messages_, pnh_))
    return false;

  if (!quad_common::getParam("control_command_timeout", control_command_timeout_, pnh_))
    return false;
  if (!quad_common::getParam("rc_timeout", rc_timeout_, pnh_))
    return false;

  if (!quad_common::getParam("mass", mass_, pnh_))
    return false;

  quad_common::getParam("disable_thrust_mapping", disable_thrust_mapping_, pnh_);
  if (!disable_thrust_mapping_)
  {
    if (!quad_common::getParam("thrust_map_a", thrust_map_a_, pnh_))
      return false;
    if (!quad_common::getParam("thrust_map_b", thrust_map_b_, pnh_))
      return false;
    if (!quad_common::getParam("thrust_map_c", thrust_map_c_, pnh_))
      return false;
  }

  double max_roll_rate_deg, max_pitch_rate_deg, max_yaw_rate_deg;
  if (!quad_common::getParam("max_roll_rate_deg", max_roll_rate_deg, pnh_))
    return false;
  if (!quad_common::getParam("max_pitch_rate_deg", max_pitch_rate_deg, pnh_))
    return false;
  if (!quad_common::getParam("max_yaw_rate_deg", max_yaw_rate_deg, pnh_))
    return false;
  max_roll_rate_ = max_roll_rate_deg / 180.0 * M_PI;
  max_pitch_rate_ = max_pitch_rate_deg / 180.0 * M_PI;
  max_yaw_rate_ = max_yaw_rate_deg / 180.0 * M_PI;

  double max_roll_angle_deg, max_pitch_angle_deg;
  if (!quad_common::getParam("max_roll_angle_deg", max_roll_angle_deg, pnh_))
    return false;
  if (!quad_common::getParam("max_pitch_angle_deg", max_pitch_angle_deg, pnh_))
    return false;
  max_roll_angle_ = max_roll_angle_deg / 180.0 * M_PI;
  max_pitch_angle_ = max_pitch_angle_deg / 180.0 * M_PI;

  if (!quad_common::getParam("alpha_vbat_filter", alpha_vbat_filter_, pnh_))
    return false;
  if (!quad_common::getParam("perform_thrust_voltage_compensation", perform_thrust_voltage_compensation_, pnh_))
    return false;
  if (!quad_common::getParam("thrust_ratio_voltage_map_a", thrust_ratio_voltage_map_a_, pnh_))
    return false;
  if (!quad_common::getParam("thrust_ratio_voltage_map_b", thrust_ratio_voltage_map_b_, pnh_))
    return false;

  return true;
}

} // namespace sbus_bridge

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbus_bridge");
  sbus_bridge::SBusBridge sbus_bridge;

  ros::spin();

  return 0;
}
