#include "manual_flight_assistant/manual_flight_assistant.h"

#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/parameter_helper.h>
#include <sbus_bridge/channel_mapping.h>
#include <sbus_bridge/sbus_msg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "manual_flight_assistant/joypad_axes_buttons.h"
#include "manual_flight_assistant/rc_channels_switches.h"

namespace manual_flight_assistant {

ManualFlightAssistant::ManualFlightAssistant(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      time_last_joypad_msg_(),
      sbus_command_(),
      previous_sbus_command_(),
      time_last_sbus_msg_(),
      sbus_needs_to_go_through_zero_(true),
      velocity_command_is_zero_(true) {
  loadParameters();

  joypad_command_ = sensor_msgs::Joy();
  joypad_command_.axes = std::vector<float>(8, 0);
  joypad_command_.buttons = std::vector<int32_t>(8, 0);
  previous_joypad_command_ = joypad_command_;

  manual_desired_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "autopilot/velocity_command", 1);
  start_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/land", 1);

  joypad_sub_ =
      nh_.subscribe("joy", 1, &ManualFlightAssistant::joyCallback, this);
  rc_sbus_sub_ = nh_.subscribe("received_sbus_message", 1,
                               &ManualFlightAssistant::rcSbusCallback, this);

  main_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / kLoopFrequency_),
                                     &ManualFlightAssistant::mainLoop, this);
}

ManualFlightAssistant::~ManualFlightAssistant() {}

void ManualFlightAssistant::mainLoop(const ros::TimerEvent& time) {
  if (rcSbusAvailable()) {
    geometry_msgs::TwistStamped velocity_command;
    velocity_command.header.stamp = ros::Time::now();

    // Coefficients to linearly map sbus values to an interval of [-1, 1]
    const double value_map_a =
        2.0 / (sbus_bridge::SBusMsg::kMaxCmd - sbus_bridge::SBusMsg::kMinCmd);
    const double value_map_b =
        1.0 - (value_map_a * sbus_bridge::SBusMsg::kMaxCmd);

    if (abs(sbus_command_.channels[sbus_bridge::channel_mapping::kPitch] -
            sbus_bridge::SBusMsg::kMeanCmd) > sbus_axes_zero_tolerance_) {
      const double normalized_command = std::max(
          std::min(value_map_a *
                           sbus_command_
                               .channels[sbus_bridge::channel_mapping::kPitch] +
                       value_map_b,
                   1.0),
          -1.0);
      velocity_command.twist.linear.x = vmax_xy_ * normalized_command;
    }
    if (abs(sbus_command_.channels[sbus_bridge::channel_mapping::kRoll] -
            sbus_bridge::SBusMsg::kMeanCmd) > sbus_axes_zero_tolerance_) {
      const double normalized_command = std::max(
          std::min(
              value_map_a * sbus_command_
                                .channels[sbus_bridge::channel_mapping::kRoll] +
                  value_map_b,
              1.0),
          -1.0);
      velocity_command.twist.linear.y = -vmax_xy_ * normalized_command;
    }
    if (abs(sbus_command_.channels[sbus_bridge::channel_mapping::kThrottle] -
            sbus_bridge::SBusMsg::kMeanCmd) > 3 * sbus_axes_zero_tolerance_) {
      const double normalized_command = std::max(
          std::min(
              value_map_a *
                      sbus_command_
                          .channels[sbus_bridge::channel_mapping::kThrottle] +
                  value_map_b,
              1.0),
          -1.0);
      velocity_command.twist.linear.z = vmax_z_ * normalized_command;
    }
    if (abs(sbus_command_.channels[sbus_bridge::channel_mapping::kYaw] -
            sbus_bridge::SBusMsg::kMeanCmd) > sbus_axes_zero_tolerance_) {
      const double normalized_command = std::max(
          std::min(
              value_map_a * sbus_command_
                                .channels[sbus_bridge::channel_mapping::kYaw] +
                  value_map_b,
              1.0),
          -1.0);
      velocity_command.twist.angular.z = -rmax_yaw_ * normalized_command;
    }

    publishVelocityCommand(velocity_command);
  } else if (joypadAvailable()) {
    geometry_msgs::TwistStamped velocity_command;
    velocity_command.header.stamp = ros::Time::now();

    if (fabs(joypad_command_.axes[joypad::axes::kX]) >
        joypad_axes_zero_tolerance_) {
      velocity_command.twist.linear.x =
          vmax_xy_ * joypad_command_.axes[joypad::axes::kX];
    }
    if (fabs(joypad_command_.axes[joypad::axes::kY]) >
        joypad_axes_zero_tolerance_) {
      velocity_command.twist.linear.y =
          vmax_xy_ * joypad_command_.axes[joypad::axes::kY];
    }
    if (fabs(joypad_command_.axes[joypad::axes::kZ]) >
        joypad_axes_zero_tolerance_) {
      velocity_command.twist.linear.z =
          vmax_z_ * joypad_command_.axes[joypad::axes::kZ];
    }
    if (fabs(joypad_command_.axes[joypad::axes::kYaw]) >
        joypad_axes_zero_tolerance_) {
      velocity_command.twist.angular.z =
          rmax_yaw_ * joypad_command_.axes[joypad::axes::kYaw];
    }

    publishVelocityCommand(velocity_command);

    // Start and Land Buttons
    if (!previous_joypad_command_.buttons.empty()) {
      if (joypad_command_.buttons[joypad::buttons::kGreen] &&
          !previous_joypad_command_.buttons[joypad::buttons::kGreen]) {
        start_pub_.publish(std_msgs::Empty());
      }
      if (joypad_command_.buttons[joypad::buttons::kBlue] &&
          !previous_joypad_command_.buttons[joypad::buttons::kBlue]) {
        land_pub_.publish(std_msgs::Empty());
      }
    }
  } else {
    // Publish zero velocity command
    geometry_msgs::TwistStamped velocity_command;
    velocity_command.header.stamp = ros::Time::now();
    publishVelocityCommand(velocity_command);
  }

  // Check whether sbus passes zero if it needs to
  if (sbus_needs_to_go_through_zero_ &&
      (ros::Time::now() - time_last_sbus_msg_) <=
          ros::Duration(sbus_timeout_) &&
      abs(sbus_command_.channels[sbus_bridge::channel_mapping::kPitch] -
          sbus_bridge::SBusMsg::kMeanCmd) < sbus_axes_zero_tolerance_ &&
      abs(sbus_command_.channels[sbus_bridge::channel_mapping::kRoll] -
          sbus_bridge::SBusMsg::kMeanCmd) < sbus_axes_zero_tolerance_ &&
      abs(sbus_command_.channels[sbus_bridge::channel_mapping::kThrottle] -
          sbus_bridge::SBusMsg::kMeanCmd) < 3 * sbus_axes_zero_tolerance_ &&
      abs(sbus_command_.channels[sbus_bridge::channel_mapping::kYaw] -
          sbus_bridge::SBusMsg::kMeanCmd) < sbus_axes_zero_tolerance_) {
    sbus_needs_to_go_through_zero_ = false;
  }
}

void ManualFlightAssistant::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  previous_joypad_command_ = joypad_command_;
  joypad_command_ = *msg;
  time_last_joypad_msg_ = ros::Time::now();
}

void ManualFlightAssistant::rcSbusCallback(
    const sbus_bridge::SbusRosMessage::ConstPtr& msg) {
  previous_sbus_command_ = sbus_command_;
  sbus_command_ = *msg;
  if (!sbus_command_.frame_lost && !sbus_command_.failsafe) {
    // This also accounts for the fact that the sbus message should not be
    // invalid for too long
    time_last_sbus_msg_ = ros::Time::now();
  }
}

bool ManualFlightAssistant::joypadAvailable() {
  if ((ros::Time::now() - time_last_joypad_msg_) >
      ros::Duration(joypad_timeout_)) {
    return false;
  }

  return true;
}

bool ManualFlightAssistant::rcSbusAvailable() {
  bool available = true;

  if (sbus_command_.channels[sbus_bridge::channel_mapping::kArming] >
      sbus_bridge::SBusMsg::kMeanCmd) {
    // Pure manual flight
    available = false;
  } else if (sbus_command_
                 .channels[sbus_bridge::channel_mapping::kGamepadMode] <
             (sbus_bridge::SBusMsg::kMeanCmd + sbus_axes_zero_tolerance_)) {
    // RC is not in gamepad flight mode
    available = false;
  } else if (sbus_needs_to_go_through_zero_) {
    available = false;
  } else if ((ros::Time::now() - time_last_sbus_msg_) >
             ros::Duration(sbus_timeout_)) {
    available = false;
  }

  if (!available) {
    sbus_needs_to_go_through_zero_ = true;
  }

  return available;
}

void ManualFlightAssistant::publishVelocityCommand(
    const geometry_msgs::TwistStamped& velocity_command) {
  if (quadrotor_common::geometryToEigen(velocity_command.twist.linear).norm() <=
          kVelocityCommandZeroThreshold_ &&
      fabs(velocity_command.twist.angular.z) <=
          kVelocityCommandZeroThreshold_) {
    if (!velocity_command_is_zero_) {
      velocity_command_is_zero_ = true;
      // Publish one zero velocity command afterwards stop publishing
      const geometry_msgs::TwistStamped zero_command;
      manual_desired_velocity_pub_.publish(zero_command);
    }
  } else if (velocity_command_is_zero_) {
    velocity_command_is_zero_ = false;
  }

  if (!velocity_command_is_zero_) {
    manual_desired_velocity_pub_.publish(velocity_command);
  }
}

bool ManualFlightAssistant::loadParameters() {
  if (!quadrotor_common::getParam("joypad_timeout", joypad_timeout_, pnh_)) {
    return false;
  }
  if (!quadrotor_common::getParam("sbus_timeout", sbus_timeout_, pnh_)) {
    return false;
  }
  if (!quadrotor_common::getParam("joypad_axes_zero_tolerance",
                                  joypad_axes_zero_tolerance_, pnh_)) {
    return false;
  }
  if (!quadrotor_common::getParam("sbus_axes_zero_tolerance",
                                  sbus_axes_zero_tolerance_, pnh_)) {
    return false;
  }
  if (!quadrotor_common::getParam("vmax_xy", vmax_xy_, pnh_)) {
    return false;
  }
  if (!quadrotor_common::getParam("vmax_z", vmax_z_, pnh_)) {
    return false;
  }
  if (!quadrotor_common::getParam("rmax_yaw", rmax_yaw_, pnh_)) {
    return false;
  }

  return true;
}

}  // namespace manual_flight_assistant

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_flight_assistant");
  manual_flight_assistant::ManualFlightAssistant manual_flight_assistant;

  ros::spin();
  return 0;
}
