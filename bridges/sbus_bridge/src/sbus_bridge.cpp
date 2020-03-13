#include "sbus_bridge/sbus_bridge.h"

#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/LowLevelFeedback.h>
#include <Eigen/Dense>

#include "sbus_bridge/SbusRosMessage.h"
#include "sbus_bridge/channel_mapping.h"

namespace sbus_bridge {

SBusBridge::SBusBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      stop_watchdog_thread_(false),
      time_last_rc_msg_received_(),
      time_last_sbus_msg_sent_(ros::Time::now()),
      time_last_battery_voltage_received_(ros::Time::now()),
      time_last_active_control_command_received_(),
      bridge_state_(BridgeState::OFF),
      control_mode_(ControlMode::NONE),
      arming_counter_(0),
      battery_voltage_(0.0),
      bridge_armed_(false),
      rc_was_disarmed_once_(false),
      destructor_invoked_(false) {
  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  if (disable_thrust_mapping_) {
    ROS_WARN("[%s] Thrust mapping disabled!", pnh_.getNamespace().c_str());
  }

  // Publishers
  low_level_feedback_pub_ =
      nh_.advertise<quadrotor_msgs::LowLevelFeedback>("low_level_feedback", 1);
  if (enable_receiving_sbus_messages_) {
    received_sbus_msg_pub_ =
        nh_.advertise<sbus_bridge::SbusRosMessage>("received_sbus_message", 1);
  }

  // Subscribers
  arm_bridge_sub_ =
      nh_.subscribe("sbus_bridge/arm", 1, &SBusBridge::armBridgeCallback, this);
  control_command_sub_ = nh_.subscribe(
      "control_command", 1, &SBusBridge::controlCommandCallback, this);
  battery_voltage_sub_ = nh_.subscribe(
      "battery_voltage", 1, &SBusBridge::batteryVoltageCallback, this);

  low_level_feedback_pub_timer_ =
      nh_.createTimer(ros::Duration(1.0 / kLowLevelFeedbackPublishFrequency_),
                      &SBusBridge::publishLowLevelFeedback, this);

  // Start serial port with receiver thread if receiving sbus messages is
  // enabled
  if (!setUpSBusSerialPort(port_name_, enable_receiving_sbus_messages_)) {
    ros::shutdown();
    return;
  }

  // Start watchdog thread
  try {
    watchdog_thread_ = std::thread(&SBusBridge::watchdogThread, this);
  } catch (...) {
    ROS_ERROR("[%s] Could not successfully start watchdog thread.",
              pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
}

SBusBridge::~SBusBridge() {
  destructor_invoked_ = true;

  // Stop SBus receiver thread
  if (enable_receiving_sbus_messages_) {
    stopReceiverThread();
  }

  // Stop watchdog thread
  stop_watchdog_thread_ = true;
  // Wait for watchdog thread to finish
  watchdog_thread_.join();

  // Now only one thread (the ROS thread) is remaining

  setBridgeState(BridgeState::OFF);

  // Send disarming SBus message for safety
  // We repeat it to prevent any possible smoothing of commands on the flight
  // controller to interfere with this
  SBusMsg shut_down_message;
  shut_down_message.setArmStateDisarmed();
  ros::Rate loop_rate(110.0);
  for (int i = 0; i < kSmoothingFailRepetitions_; i++) {
    transmitSerialSBusMessage(shut_down_message);
    loop_rate.sleep();
  }

  // Close serial port
  disconnectSerialPort();
}

void SBusBridge::watchdogThread() {
  ros::Rate watchdog_rate(110.0);
  while (ros::ok() && !stop_watchdog_thread_) {
    watchdog_rate.sleep();

    std::lock_guard<std::mutex> main_lock(main_mutex_);

    const ros::Time time_now = ros::Time::now();

    if (bridge_state_ == BridgeState::RC_FLIGHT &&
        time_now - time_last_rc_msg_received_ > ros::Duration(rc_timeout_)) {
      // If the last received RC command was armed but was received longer than
      // rc_timeout ago we switch the bridge state to AUTONOMOUS_FLIGHT.
      // In case there are no valid control commands the bridge state is set to
      // OFF in the next check below
      ROS_WARN(
          "[%s] Remote control was active but no message from it was received "
          "within timeout (%f s).",
          pnh_.getNamespace().c_str(), rc_timeout_);
      setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
    }

    if (bridge_state_ == BridgeState::ARMING ||
        bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT) {
      if (time_now - time_last_active_control_command_received_ >
          ros::Duration(control_command_timeout_)) {
        // When switching the bridge state to off, our watchdog ensures that a
        // disarming off message is repeated.
        setBridgeState(BridgeState::OFF);
        // Note: Control could theoretically still be taken over by RC but if
        // this happened in flight it might require super human reaction since
        // in this case the quad can not be armed with non zero throttle by
        // the remote.
      }
    }

    if (bridge_state_ == BridgeState::OFF) {
      // Send off message that disarms the vehicle
      // We repeat it to prevent any weird behavior that occurs if the flight
      // controller is not receiving commands for a while
      SBusMsg off_msg;
      off_msg.setArmStateDisarmed();
      sendSBusMessageToSerialPort(off_msg);
    }

    // Check battery voltage timeout
    std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);
    if (time_now - time_last_battery_voltage_received_ >
        ros::Duration(kBatteryVoltageTimeout_)) {
      battery_voltage_ = 0.0;
      if (perform_thrust_voltage_compensation_) {
        ROS_WARN_THROTTLE(
            1.0,
            "[%s] Can not perform battery voltage compensation because there "
            "is no recent battery voltage measurement",
            pnh_.getNamespace().c_str());
      }
    }

    // Mutexes are unlocked because they go out of scope here
  }
}

void SBusBridge::handleReceivedSbusMessage(const SBusMsg& received_sbus_msg) {
  {
    std::lock_guard<std::mutex> main_lock(main_mutex_);

    time_last_rc_msg_received_ = ros::Time::now();

    if (received_sbus_msg.isArmed()) {
      if (!rc_was_disarmed_once_) {
        // This flag prevents that the vehicle can be armed if the RC is armed
        // on startup of the bridge
        ROS_WARN_THROTTLE(
            1.0,
            "[%s] RC needs to be disarmed once before it can take over control",
            pnh_.getNamespace().c_str());
        return;
      }

      // Immediately go into RC_FLIGHT state since RC always has priority
      if (bridge_state_ != BridgeState::RC_FLIGHT) {
        setBridgeState(BridgeState::RC_FLIGHT);
        ROS_INFO("[%s] Control authority taken over by remote control.",
                 pnh_.getNamespace().c_str());
      }
      sendSBusMessageToSerialPort(received_sbus_msg);
      control_mode_ = received_sbus_msg.getControlMode();
    } else if (bridge_state_ == BridgeState::RC_FLIGHT) {
      // If the bridge was in state RC_FLIGHT and the RC is disarmed we set the
      // state to AUTONOMOUS_FLIGHT
      // In case there are valid control commands, the bridge will stay in
      // AUTONOMOUS_FLIGHT, otherwise the watchdog will set the state to OFF
      ROS_INFO("[%s] Control authority returned by remote control.",
               pnh_.getNamespace().c_str());
      if (bridge_armed_) {
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      } else {
        // When switching the bridge state to off, our watchdog ensures that a
        // disarming off message is sent
        setBridgeState(BridgeState::OFF);
      }
    } else if (!rc_was_disarmed_once_) {
      ROS_INFO(
          "[%s] RC was disarmed once, now it is allowed to take over control",
          pnh_.getNamespace().c_str());
      rc_was_disarmed_once_ = true;
    }

    // Main mutex is unlocked here because it goes out of scope
  }

  received_sbus_msg_pub_.publish(received_sbus_msg.toRosMessage());
}

void SBusBridge::controlCommandCallback(
    const quadrotor_msgs::ControlCommand::ConstPtr& msg) {
  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (destructor_invoked_) {
    // This ensures that if the destructor was invoked we do not try to write
    // to the serial port anymore because of receiving a control command
    return;
  }

  if (msg->armed) {
    // We only need to know when the last active control command was received.
    // If it is not active, the bridge state will go to off and we keep sending
    // an off command with every new control command received.
    // This prevents the flight controller from going into failsafe
    time_last_active_control_command_received_ = ros::Time::now();
  }

  if (!bridge_armed_ || bridge_state_ == BridgeState::RC_FLIGHT) {
    // If bridge is not armed we do not allow control commands to be sent
    // RC has priority over control commands for autonomous flying
    if (!bridge_armed_ && msg->armed &&
        bridge_state_ != BridgeState::RC_FLIGHT) {
      ROS_WARN_THROTTLE(
          1.0,
          "[%s] Received active control command but sbus bridge is not armed.",
          pnh_.getNamespace().c_str());
    }
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
  if (msg->armed) {
    if (bridge_state_ != BridgeState::ARMING &&
        bridge_state_ != BridgeState::AUTONOMOUS_FLIGHT) {
      if (msg->control_mode == msg->ATTITUDE ||
          msg->control_mode == msg->BODY_RATES) {
        setBridgeState(BridgeState::ARMING);
      } else {
        ROS_WARN_THROTTLE(
            1.0,
            "[%s] Received active control command with unsupported control "
            "mode, sbus bridge will not arm. Supported control modes are "
            "ATTITUDE and BODY_RATES",
            pnh_.getNamespace().c_str());
      }
    }
  } else {
    if (bridge_state_ == BridgeState::ARMING ||
        bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT) {
      setBridgeState(BridgeState::OFF);
    }
    // Make sure vehicle is disarmed to immediately switch it off
    sbus_msg_to_send.setArmStateDisarmed();
  }

  // Limit channels
  sbus_msg_to_send.limitAllChannelsFeasible();

  // Immediately send SBus message
  sendSBusMessageToSerialPort(sbus_msg_to_send);

  // Set control mode for low level feedback message to be published
  if (msg->control_mode == msg->ATTITUDE) {
    control_mode_ = ControlMode::ATTITUDE;
  } else if (msg->control_mode == msg->BODY_RATES) {
    control_mode_ = ControlMode::BODY_RATES;
  } else {
    control_mode_ = ControlMode::NONE;
  }

  // Main mutex is unlocked because it goes out of scope here
}

void SBusBridge::sendSBusMessageToSerialPort(const SBusMsg& sbus_msg) {
  SBusMsg sbus_message_to_send = sbus_msg;

  switch (bridge_state_) {
    case BridgeState::OFF:
      // Disarm vehicle
      sbus_message_to_send.setArmStateDisarmed();
      break;

    case BridgeState::ARMING:
      // Since there might be some RC commands smoothing and checks on multiple
      // messages before arming, we repeat the messages with minimum throttle
      // and arming command multiple times. 5 times seems to work robustly.
      if (arming_counter_ >= kSmoothingFailRepetitions_) {
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      } else {
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
      ROS_WARN("[%s] Bridge is in unknown state, vehicle will be disarmed",
               pnh_.getNamespace().c_str());
      break;
  }

  if ((ros::Time::now() - time_last_sbus_msg_sent_).toSec() <= 0.006) {
    // An SBUS message takes 3ms to be transmitted by the serial port so let's
    // not stress it too much. This should only happen in case of switching
    // between control commands and rc commands
    if (bridge_state_ == BridgeState::ARMING) {
      // In case of arming we want to send kSmoothingFailRepetitions_ messages
      // with minimum throttle to the flight controller. Since this check
      // prevents the message from being sent out we reduce the counter that
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
    const quadrotor_msgs::ControlCommand::ConstPtr& control_command) const {
  SBusMsg sbus_msg;

  sbus_msg.setArmStateArmed();
  if (disable_thrust_mapping_) {
    sbus_msg.setThrottleCommand(int(control_command->collective_thrust));
    sbus_msg.setRollCommand(SBusMsg::kMeanCmd);
    sbus_msg.setPitchCommand(SBusMsg::kMeanCmd);
    sbus_msg.setYawCommand(SBusMsg::kMeanCmd);
    sbus_msg.setControlModeBodyRates();
  } else {
    if (control_command->control_mode == control_command->ATTITUDE) {
      sbus_msg.setControlModeAttitude();
      sbus_msg.setThrottleCommand(thrust_mapping_.inverseThrustMapping(
          control_command->collective_thrust * mass_, battery_voltage_));

      Eigen::Vector3d desired_euler_angles =
          quadrotor_common::quaternionToEulerAnglesZYX(
              quadrotor_common::geometryToEigen(control_command->orientation));

      quadrotor_common::limit(&desired_euler_angles(0), -max_roll_angle_,
                              max_roll_angle_);
      quadrotor_common::limit(&desired_euler_angles(1), -max_pitch_angle_,
                              max_pitch_angle_);

      sbus_msg.setRollCommand(
          round((desired_euler_angles(0) / max_roll_angle_) *
                    (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd) +
                SBusMsg::kMeanCmd));
      sbus_msg.setPitchCommand(
          round((desired_euler_angles(1) / max_pitch_angle_) *
                    (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd) +
                SBusMsg::kMeanCmd));
      sbus_msg.setYawCommand(
          round((-control_command->bodyrates.z / max_yaw_rate_) *
                    (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd) +
                SBusMsg::kMeanCmd));
    } else if (control_command->control_mode == control_command->BODY_RATES) {
      sbus_msg.setControlModeBodyRates();
      sbus_msg.setThrottleCommand(thrust_mapping_.inverseThrustMapping(
          control_command->collective_thrust * mass_, battery_voltage_));
      sbus_msg.setRollCommand(
          round((control_command->bodyrates.x / max_roll_rate_) *
                    (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd) +
                SBusMsg::kMeanCmd));
      sbus_msg.setPitchCommand(
          round((control_command->bodyrates.y / max_pitch_rate_) *
                    (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd) +
                SBusMsg::kMeanCmd));
      sbus_msg.setYawCommand(
          round((-control_command->bodyrates.z / max_yaw_rate_) *
                    (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd) +
                SBusMsg::kMeanCmd));
    } else {
      // Not supported control mode
      sbus_msg.setArmStateDisarmed();
    }
  }

  return sbus_msg;
}

void SBusBridge::setBridgeState(const BridgeState& desired_bridge_state) {
  switch (desired_bridge_state) {
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
      ROS_WARN("[%s] Wanted to switch to unknown bridge state",
               pnh_.getNamespace().c_str());
  }
}

void SBusBridge::armBridgeCallback(const std_msgs::Bool::ConstPtr& msg) {
  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (destructor_invoked_) {
    // On shut down we do not allow to arm (or disarm) the bridge anymore
    return;
  }

  if (msg->data) {
    bridge_armed_ = true;
    ROS_INFO("[%s] Bridge armed", pnh_.getNamespace().c_str());
  } else {
    bridge_armed_ = false;
    if (bridge_state_ == BridgeState::ARMING ||
        bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT) {
      setBridgeState(BridgeState::OFF);
    }
    ROS_INFO("[%s] Bridge disarmed", pnh_.getNamespace().c_str());
  }
}

void SBusBridge::batteryVoltageCallback(
    const std_msgs::Float32::ConstPtr& msg) {
  std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);

  if (battery_voltage_ != 0.0) {
    battery_voltage_ = alpha_vbat_filter_ * msg->data +
                       (1.0 - alpha_vbat_filter_) * battery_voltage_;
  } else {
    battery_voltage_ = msg->data;
  }
  time_last_battery_voltage_received_ = ros::Time::now();
}

void SBusBridge::publishLowLevelFeedback(const ros::TimerEvent& time) const {
  quadrotor_msgs::LowLevelFeedback low_level_feedback_msg;

  {
    std::lock_guard<std::mutex> main_lock(main_mutex_);
    std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);

    // Publish a low level feedback message
    low_level_feedback_msg.header.stamp = ros::Time::now();
    low_level_feedback_msg.battery_voltage = battery_voltage_;
    if (battery_voltage_ > n_lipo_cells_ * kBatteryLowVoltagePerCell_) {
      low_level_feedback_msg.battery_state = low_level_feedback_msg.BAT_GOOD;
    } else if (battery_voltage_ >
               n_lipo_cells_ * kBatteryCriticalVoltagePerCell_) {
      low_level_feedback_msg.battery_state = low_level_feedback_msg.BAT_LOW;
    } else if (battery_voltage_ >
               n_lipo_cells_ * kBatteryInvalidVoltagePerCell_) {
      low_level_feedback_msg.battery_state =
          low_level_feedback_msg.BAT_CRITICAL;
    } else {
      low_level_feedback_msg.battery_state = low_level_feedback_msg.BAT_INVALID;
    }

    if (bridge_state_ == BridgeState::RC_FLIGHT) {
      low_level_feedback_msg.control_mode = low_level_feedback_msg.RC_MANUAL;
    } else {
      if (control_mode_ == ControlMode::ATTITUDE) {
        low_level_feedback_msg.control_mode = low_level_feedback_msg.ATTITUDE;
      } else if (control_mode_ == ControlMode::BODY_RATES) {
        low_level_feedback_msg.control_mode = low_level_feedback_msg.BODY_RATES;
      } else {
        low_level_feedback_msg.control_mode = low_level_feedback_msg.NONE;
      }
    }

    // Mutexes are unlocked here since they go out of scope
  }

  low_level_feedback_pub_.publish(low_level_feedback_msg);
}

bool SBusBridge::loadParameters() {
#define GET_PARAM(name) \
  if (!quadrotor_common::getParam(#name, name##_, pnh_)) return false

  GET_PARAM(port_name);
  GET_PARAM(enable_receiving_sbus_messages);

  GET_PARAM(control_command_timeout);
  GET_PARAM(rc_timeout);

  GET_PARAM(mass);

  GET_PARAM(disable_thrust_mapping);

  GET_PARAM(max_roll_rate);
  GET_PARAM(max_pitch_rate);
  GET_PARAM(max_yaw_rate);
  max_roll_rate_ /= (180.0 / M_PI);
  max_pitch_rate_ /= (180.0 / M_PI);
  max_yaw_rate_ /= (180.0 / M_PI);

  GET_PARAM(max_roll_angle);
  GET_PARAM(max_pitch_angle);
  max_roll_angle_ /= (180.0 / M_PI);
  max_pitch_angle_ /= (180.0 / M_PI);

  GET_PARAM(alpha_vbat_filter);
  GET_PARAM(perform_thrust_voltage_compensation);
  GET_PARAM(n_lipo_cells);

  if (!thrust_mapping_.loadParameters()) {
    return false;
  }

  return true;

#undef GET_PARAM
}

}  // namespace sbus_bridge
