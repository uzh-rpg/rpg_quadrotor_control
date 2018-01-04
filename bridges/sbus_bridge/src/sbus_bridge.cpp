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
        ControlMode::RATE), arming_counter_(0), battery_voltage_(0.0), destructor_invoked_(false)
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

  // Subscribers
  arm_bridge_sub_ = nh_.subscribe("sbus_bridge/arm", 1, &SBusBridge::armBridgeCallback, this);
  control_command_sub_ = nh_.subscribe("control_command", 1, &SBusBridge::controlCommandCallback, this);
  battery_voltage_sub_ = nh_.subscribe("battery_voltage", 1, &SBusBridge::batteryVoltageCallback, this);

  onboard_status_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / kOnboardStatusPublishFrequency_),
                                              &SBusBridge::publishOnboardStatus, this);

  // Start serial port with receiver thread if receiving sbus messages is enabled
  if (!setUpSBusSerialPort(port_name_, enable_receiving_sbus_messages_))
  {
    ros::shutdown();
    return;
  }

  // Start watchdog thread
  try
  {
    watchdog_thread_ = std::thread(&SBusBridge::watchdogThread, this);
  }
  catch (...)
  {
    ROS_ERROR("[%s] Could not successfully start watchdog thread.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
}

SBusBridge::~SBusBridge()
{
  destructor_invoked_ = true;

  // Stop SBus receiver thread
  if (enable_receiving_sbus_messages_)
  {
    stopReceiverThread();
  }

  // Stop watchdog thread
  stop_watchdog_thread_ = true;
  // Wait for watchdog thread to finish
  watchdog_thread_.join();

  // Now only one thread (the ROS thread) is remaining

  setBridgeState(BridgeState::OFF);

  // Send disarming SBus message for safety
  // We repeat it to prevent any possible smoothing of commands on the flight controller to interfere with this
  SBusMsg shut_down_message;
  shut_down_message.setArmStateDisarmed();
  ros::Rate loop_rate(110.0);
  for (int i = 0; i < kSmoothingFailRepetitions_; i++)
  {
    transmitSerialSBusMessage(shut_down_message);
    loop_rate.sleep();
  }

  // Close serial port
  disconnectSerialPort();
}

void SBusBridge::watchdogThread()
{
  ros::Rate watchdog_rate(110.0);
  while (ros::ok() && !stop_watchdog_thread_)
  {
    watchdog_rate.sleep();

    std::lock_guard<std::mutex> main_lock(main_mutex_);

    const ros::Time time_now = ros::Time::now();

    if (bridge_state_ == BridgeState::RC_FLIGHT && time_now - time_last_rc_msg_received_ > ros::Duration(rc_timeout_))
    {
      // If the last received RC command was armed but was received longer than rc_timeout ago we switch the bridge
      // state to AUTONOMOUS_FLIGHT. In case there are no valid control commands the bridge state is set to OFF
      // in the next check below
      ROS_WARN("[%s] Remote control was active but no message from it was received within timeout (%f s).",
               pnh_.getNamespace().c_str(), rc_timeout_);
      setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
    }

    if (bridge_state_ == BridgeState::ARMING || bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT)
    {
      if (time_now - time_last_active_control_command_received_ > ros::Duration(control_command_timeout_))
      {
        // When switching the bridge state to off, our watchdog ensures that a disarming off message is sent
        // kSmoothingFailRepetitions_ times.
        setBridgeState(BridgeState::OFF);
        // Note: Control could theoretically still be taken over by RC but if this happened in flight it might
        // require super human reaction since in this case the quad can not be armed with non zero throttle
        // by the remote.
      }
    }

    if (bridge_state_ == BridgeState::OFF)
    {
      // Send off message that disarms the vehicle
      // We repeat it to prevent any weird behavior that occurs if the flight controller is not receiving
      // commands for a while
      SBusMsg off_msg;
      off_msg.setArmStateDisarmed();
      sendSBusMessageToSerialPort(off_msg);
    }

    // Check battery voltage timeout
    std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);
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

    // Mutexes are unlocked because they go out of scope here
  }
}

void SBusBridge::handleReceivedSbusMessage(const SBusMsg& received_sbus_msg)
{
  {
    std::lock_guard<std::mutex> main_lock(main_mutex_);

    time_last_rc_msg_received_ = ros::Time::now();

    if (received_sbus_msg.isArmed())
    {
      // Immediately go into RC_FLIGHT state since RC always has priority
      if (bridge_state_ != BridgeState::RC_FLIGHT)
      {
        setBridgeState(BridgeState::RC_FLIGHT);
        ROS_INFO("[%s] Control authority taken over by remote control.", pnh_.getNamespace().c_str());
      }
      sendSBusMessageToSerialPort(received_sbus_msg);
      control_mode_ = received_sbus_msg.getControlMode();
    }
    else if (bridge_state_ == BridgeState::RC_FLIGHT)
    {
      // If the bridge was in state RC_FLIGHT and the RC is disarmed we set the state to AUTONOMOUS_FLIGHT
      // In case there are valid control commands, the bridge will stay in AUTONOMOUS_FLIGHT, otherwise the watchdog
      // will set the state to OFF
      ROS_INFO("[%s] Control authority returned by remote control.", pnh_.getNamespace().c_str());
      if (bridge_armed_)
      {
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      }
      else
      {
        // When switching the bridge state to off, our watchdog ensures that a disarming off message is sent
        // kSmoothingFailRepetitions_ times.
        setBridgeState(BridgeState::OFF);
      }
    }

    // Main mutex is unlocked here because it goes out of scope
  }

  received_sbus_msg_pub_.publish(received_sbus_msg.toRosMessage());
}

void SBusBridge::controlCommandCallback(const quad_msgs::ControlCommand::ConstPtr& msg)
{
  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (destructor_invoked_)
  {
    // This ensures that if the destructor was invoked we do not try to write to the serial port anymore
    // because of receiving a control command
    return;
  }

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

  SBusMsg sbus_msg_to_send;
  {
    std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);
    // Set commands
    sbus_msg_to_send = generateSBusMessageFromControlCommand(msg);
    // Battery voltage mutex is unlocked because it goes out of scope here
  }

  // Set to arming state or ensure disarmed command if necessary
  if (!msg->off)
  {
    if (bridge_state_ != BridgeState::ARMING && bridge_state_ != BridgeState::AUTONOMOUS_FLIGHT)
    {
      setBridgeState(BridgeState::ARMING);
    }
  }
  else
  {
    // Make sure vehicle is disarmed
    sbus_msg_to_send.setArmStateDisarmed();
  }

  // Limit channels
  sbus_msg_to_send.limitAllChannelsFeasible();

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

  // Main mutex is unlocked because it goes out of scope here
}

void SBusBridge::sendSBusMessageToSerialPort(const SBusMsg& sbus_msg)
{
  SBusMsg sbus_message_to_send = sbus_msg;

  switch (bridge_state_)
  {
    case BridgeState::OFF:
      // Disarm vehicle
      sbus_message_to_send.setArmStateDisarmed();
      break;

    case BridgeState::ARMING:
      // Since there might be some RC commands smoothing and checks on multiple messages before arming, we repeat
      // the messages with minimum throttle and arming command multiple times. 5 times seems to work robustly.
      if (arming_counter_ >= kSmoothingFailRepetitions_)
      {
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      }
      else
      {
        // Set thrust to minimum command to make sure FC arms
        sbus_message_to_send.setThrottleCommand(SBusMsg::kMinCmd);
        sbus_message_to_send.setArmStateArmed();
        arming_counter_++;
      }
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      sbus_message_to_send.setArmStateArmed();
      break;

    case BridgeState::RC_FLIGHT:
      // Passing RC command straight through
      break;

    default:
      // Disarm the vehicle because this code must be terribly wrong
      sbus_message_to_send.setArmStateDisarmed();
      ROS_WARN("[%s] Bridge is in unknown state, vehicle will be disarmed", pnh_.getNamespace().c_str());
      break;
  }

  if ((ros::Time::now() - time_last_sbus_msg_sent_).toSec() <= 0.006)
  {
    // An SBUS message takes 3ms to be transmitted by the serial port so let's not stress it too much
    // This should only happen in case of switching between control commands and rc commands
    if (bridge_state_ == BridgeState::ARMING)
    {
      // In case of arming we want to send kSmoothingFailRepetitions_ messages with minimum throttle to the
      // flight controller. Since this check prevents the message from being sent out we reduce the counter that
      // was incremented above assuming the message would actually be sent.
      arming_counter_--;
    }
    return;
  }

  sbus_message_to_send.timestamp = ros::Time::now();
  transmitSerialSBusMessage(sbus_message_to_send);
  time_last_sbus_msg_sent_ = ros::Time::now();
}

SBusMsg SBusBridge::generateSBusMessageFromControlCommand(
    const quad_msgs::ControlCommand::ConstPtr& control_command) const
{
  SBusMsg sbus_msg;

  sbus_msg.setArmStateArmed();
  if (disable_thrust_mapping_)
  {
    sbus_msg.setThrottleCommand(int(control_command->thrust));
    sbus_msg.setRollCommand(SBusMsg::kMeanCmd);
    sbus_msg.setPitchCommand(SBusMsg::kMeanCmd);
    sbus_msg.setYawCommand(SBusMsg::kMeanCmd);
    sbus_msg.setControlModeRate();
  }
  else
  {
    if (control_command->control_mode == control_command->ANGLE)
    {
      sbus_msg.setControlModeAngle();
      sbus_msg.setThrottleCommand(
          thrust_mapping_.inverseThrustMapping(control_command->thrust * mass_, battery_voltage_));

      Eigen::Vector3d desired_euler_angles = quad_common::quaternionToEulerAnglesZYX(
          quad_common::geometryToEigen(control_command->orientation));

      quad_common::limit(desired_euler_angles(0), -max_roll_angle_, max_roll_angle_);
      quad_common::limit(desired_euler_angles(1), -max_pitch_angle_, max_pitch_angle_);

      sbus_msg.setRollCommand(
          round(
              (desired_euler_angles(0) / max_roll_angle_) * (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd)
                  + SBusMsg::kMeanCmd));
      sbus_msg.setPitchCommand(
          round(
              (desired_euler_angles(1) / max_pitch_angle_) * (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd)
                  + SBusMsg::kMeanCmd));
      sbus_msg.setYawCommand(
          round(
              (-control_command->bodyrates.z / max_yaw_rate_) * (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd)
                  + SBusMsg::kMeanCmd));
    }
    else
    {
      sbus_msg.setControlModeRate();
      sbus_msg.setThrottleCommand(
          thrust_mapping_.inverseThrustMapping(control_command->thrust * mass_, battery_voltage_));
      sbus_msg.setRollCommand(
          round(
              (control_command->bodyrates.x / max_roll_rate_) * (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd)
                  + SBusMsg::kMeanCmd));
      sbus_msg.setPitchCommand(
          round(
              (control_command->bodyrates.y / max_pitch_rate_) * (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd)
                  + SBusMsg::kMeanCmd));
      sbus_msg.setYawCommand(
          round(
              (-control_command->bodyrates.z / max_yaw_rate_) * (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd)
                  + SBusMsg::kMeanCmd));
    }
  }

  return sbus_msg;
}

void SBusBridge::setBridgeState(const BridgeState& desired_bridge_state)
{
  switch (desired_bridge_state)
  {
    case BridgeState::OFF:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::ARMING:
      bridge_state_ = desired_bridge_state;
      arming_counter_ = 0;
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::RC_FLIGHT:
      bridge_state_ = desired_bridge_state;
      break;

    default:
      ROS_WARN("[%s] Wanted to switch to unknown bridge state", pnh_.getNamespace().c_str());
  }
}

void SBusBridge::armBridgeCallback(const std_msgs::BoolConstPtr& msg)
{
  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (destructor_invoked_)
  {
    // On shut down we do not allow to arm (or disarm) the bridge anymore
    return;
  }

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
      setBridgeState(BridgeState::OFF);
    }
    ROS_INFO("[%s] Bridge disarmed", pnh_.getNamespace().c_str());
  }
}

void SBusBridge::batteryVoltageCallback(const std_msgs::Float32::ConstPtr& msg)
{
  std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);

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
  quad_msgs::OnboardStatus onboard_status_msg;

  {
    std::lock_guard<std::mutex> main_lock(main_mutex_);
    std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);

    // Publish an onboard status message
    onboard_status_msg.header.stamp = ros::Time::now();
    onboard_status_msg.battery_voltage = battery_voltage_;
    if (battery_voltage_ > n_lipo_cells_ * kBatteryLowVoltagePerCell_)
    {
      onboard_status_msg.battery_state = onboard_status_msg.GOOD;
    }
    else if (battery_voltage_ > n_lipo_cells_ * kBatteryCriticalVoltagePerCell_)
    {
      onboard_status_msg.battery_state = onboard_status_msg.LOW;
    }
    else if (battery_voltage_ > n_lipo_cells_ * kBatteryInvalidVoltagePerCell_)
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

    // Mutexes are unlocked here since they go out of scope
  }

  onboard_status_pub_.publish(onboard_status_msg);
}

bool SBusBridge::loadParameters()
{
#define GET_PARAM(name) \
if (!quad_common::getParam(#name, name ## _, pnh_)) \
  return false

  GET_PARAM(port_name);
  GET_PARAM(enable_receiving_sbus_messages);

  GET_PARAM(control_command_timeout);
  GET_PARAM(rc_timeout);

  GET_PARAM(mass);

  GET_PARAM(disable_thrust_mapping);

  GET_PARAM(max_roll_rate);
  GET_PARAM(max_pitch_rate);
  GET_PARAM(max_yaw_rate);
  max_roll_rate_ /= 180.0 * M_PI;
  max_pitch_rate_ /= 180.0 * M_PI;
  max_yaw_rate_ /= 180.0 * M_PI;

  GET_PARAM(max_roll_angle);
  GET_PARAM(max_pitch_angle);
  max_roll_angle_ /= 180.0 * M_PI;
  max_pitch_angle_ /= 180.0 * M_PI;

  GET_PARAM(alpha_vbat_filter);
  GET_PARAM(perform_thrust_voltage_compensation);
  GET_PARAM(n_lipo_cells);

  if (!thrust_mapping_.loadParameters())
  {
    return false;
  }

  return true;

#undef GET_PARAM
}

} // namespace sbus_bridge
