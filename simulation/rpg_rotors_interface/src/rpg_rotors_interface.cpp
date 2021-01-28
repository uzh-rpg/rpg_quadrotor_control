#include "rpg_rotors_interface/rpg_rotors_interface.h"

#include <chrono>
#include <thread>

#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/parameter_helper.h>
#include <std_srvs/Empty.h>

namespace rpg_rotors_interface {

RPGRotorsInterface::RPGRotorsInterface(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      interface_armed_(false),
      torques_and_thrust_estimate_(),
      control_command_() {
  loadParameters();

  rotors_desired_motor_speed_pub_ =
      nh_.advertise<mav_msgs::Actuators>("command/motor_speed", 1);

  rpg_control_command_sub_ =
      nh_.subscribe("control_command", 1,
                    &RPGRotorsInterface::rpgControlCommandCallback, this);
  rotors_odometry_sub_ = nh_.subscribe(
      "odometry", 1, &RPGRotorsInterface::rotorsOdometryCallback, this);
  motor_speed_sub_ = nh_.subscribe(
      "motor_speed", 1, &RPGRotorsInterface::motorSpeedCallback, this);
  arm_interface_sub_ =
      nh_.subscribe("rpg_rotors_interface/arm", 1,
                    &RPGRotorsInterface::armInterfaceCallback, this);

  low_level_control_loop_timer_ =
      nh_.createTimer(ros::Duration(1.0 / low_level_control_frequency_),
                      &RPGRotorsInterface::lowLevelControlLoop, this);
}

RPGRotorsInterface::~RPGRotorsInterface() {}

void RPGRotorsInterface::lowLevelControlLoop(const ros::TimerEvent& time) {
  mav_msgs::Actuators desired_motor_speed;
  if (!interface_armed_ || !control_command_.armed) {
    for (int i = 0; i < 4; i++) {
      desired_motor_speed.angular_velocities.push_back(0.0);
    }
  } else {
    if (control_command_.control_mode == control_command_.ATTITUDE) {
      const quadrotor_common::ControlCommand rate_cmd = attitudeControl(
          quadrotor_common::ControlCommand(control_command_),
          quadrotor_common::geometryToEigen(quad_state_.pose.pose.orientation));

      const TorquesAndThrust torques_and_thrust = bodyRateControl(
          rate_cmd,
          quadrotor_common::geometryToEigen(quad_state_.twist.twist.angular));

      desired_motor_speed = mixer(torques_and_thrust);
    } else if (control_command_.control_mode == control_command_.BODY_RATES) {
      const quadrotor_common::ControlCommand rate_cmd =
          quadrotor_common::ControlCommand(control_command_);

      const TorquesAndThrust torques_and_thrust = bodyRateControl(
          rate_cmd,
          quadrotor_common::geometryToEigen(quad_state_.twist.twist.angular));

      desired_motor_speed = mixer(torques_and_thrust);
    } else if (control_command_.control_mode ==
               control_command_.ANGULAR_ACCELERATIONS) {
      TorquesAndThrust torques_and_thrust;
      torques_and_thrust.body_torques =
          inertia_ * quadrotor_common::geometryToEigen(
                         control_command_.angular_accelerations) +
          quadrotor_common::geometryToEigen(control_command_.bodyrates)
              .cross(inertia_ * quadrotor_common::geometryToEigen(
                                    control_command_.bodyrates));
      torques_and_thrust.collective_thrust = control_command_.collective_thrust;

      desired_motor_speed = mixer(torques_and_thrust);
    } else if (control_command_.control_mode ==
               control_command_.ROTOR_THRUSTS) {
      if (control_command_.rotor_thrusts.size() != 4) {
        ROS_ERROR_THROTTLE(
            1,
            "[%s] Require exactly 4 rotor thrusts in case of ROTOR_THRUSTS "
            "control mode.",
            ros::this_node::getName().c_str());
        return;
      }
      for (int i = 0; i < 4; i++) {
        double motor_speed_squared =
            control_command_.rotor_thrusts[i] / rotor_thrust_coeff_;
        quadrotor_common::limit(&motor_speed_squared, 0.0,
                                pow(max_rotor_speed_, 2.0));
        desired_motor_speed.angular_velocities[i] = sqrt(motor_speed_squared);
      }
    } else {
      ROS_ERROR_THROTTLE(1,
                         "[%s] Undefined contol mode, will not apply command.",
                         ros::this_node::getName().c_str());
      return;
    }
  }

  rotors_desired_motor_speed_pub_.publish(desired_motor_speed);
}

void RPGRotorsInterface::rotorsOdometryCallback(
    const nav_msgs::Odometry::ConstPtr& msg) {
  quad_state_ = *msg;
}

void RPGRotorsInterface::rpgControlCommandCallback(
    const quadrotor_msgs::ControlCommand::ConstPtr& msg) {
  control_command_ = *msg;
}

quadrotor_common::ControlCommand RPGRotorsInterface::attitudeControl(
    const quadrotor_common::ControlCommand& attitude_cmd,
    const Eigen::Quaterniond& attitude_estimate) {
  Eigen::Quaterniond q_e =
      attitude_estimate.inverse() * attitude_cmd.orientation;

  quadrotor_common::ControlCommand body_rate_command = attitude_cmd;
  body_rate_command.control_mode = quadrotor_common::ControlMode::BODY_RATES;

  // pitch and roll control
  if (q_e.w() >= 0) {
    body_rate_command.bodyrates.x() =
        attitude_cmd.bodyrates.x() + 2.0 * roll_pitch_cont_gain_ * q_e.x();
    body_rate_command.bodyrates.y() =
        attitude_cmd.bodyrates.y() + 2.0 * roll_pitch_cont_gain_ * q_e.y();
  } else {
    body_rate_command.bodyrates.x() =
        attitude_cmd.bodyrates.x() - 2.0 * roll_pitch_cont_gain_ * q_e.x();
    body_rate_command.bodyrates.y() =
        attitude_cmd.bodyrates.y() - 2.0 * roll_pitch_cont_gain_ * q_e.y();
  }

  return body_rate_command;
}

TorquesAndThrust RPGRotorsInterface::bodyRateControl(
    const quadrotor_common::ControlCommand& rate_cmd,
    const Eigen::Vector3d& body_rate_estimate) {
  //  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 3);
  Eigen::VectorXd control_error = Eigen::VectorXd::Zero(6);
  control_error.segment(0, 3) = rate_cmd.bodyrates - body_rate_estimate;
//  control_error.segment(3, 3) = Eigen::Vector3d::Zero();
  control_error.segment(3, 3) =
            rate_cmd.bodyrates.cross(inertia_ * rate_cmd.bodyrates) +
        inertia_ * rate_cmd.angular_accelerations -
        torques_and_thrust_estimate_.body_torques;

  TorquesAndThrust torques_and_thrust;
  torques_and_thrust.body_torques =
      K_lqr_ * control_error +
      body_rate_estimate.cross(inertia_ * body_rate_estimate) +
      inertia_ * rate_cmd.angular_accelerations;
  torques_and_thrust.collective_thrust = rate_cmd.collective_thrust;

  return torques_and_thrust;
}

mav_msgs::Actuators RPGRotorsInterface::mixer(
    const TorquesAndThrust& torques_and_thrust) {
  // Using Rotor Convention of the Hummingbird
  mav_msgs::Actuators rotor_speed_cmds;
  for (int i = 0; i < 4; i++) {
    rotor_speed_cmds.angular_velocities.push_back(0.0);
  }

  // Compute square of single rotor speed commands
  if (torques_and_thrust.collective_thrust < 0.05) {
    return rotor_speed_cmds;
  }

  // Compute the square of the single rotor speeds
  rotor_speed_cmds.angular_velocities[0] =
      ((arm_length_ * torques_and_thrust.body_torques.z() -
        2.0 * rotor_drag_coeff_ * torques_and_thrust.body_torques.y() +
        rotor_drag_coeff_ * arm_length_ * mass_ *
            torques_and_thrust.collective_thrust) /
       (4.0 * rotor_drag_coeff_ * arm_length_)) /
      rotor_thrust_coeff_;
  rotor_speed_cmds.angular_velocities[1] =
      ((2.0 * rotor_drag_coeff_ * torques_and_thrust.body_torques.x() -
        arm_length_ * torques_and_thrust.body_torques.z() +
        rotor_drag_coeff_ * arm_length_ * mass_ *
            torques_and_thrust.collective_thrust) /
       (4.0 * rotor_drag_coeff_ * arm_length_)) /
      rotor_thrust_coeff_;
  rotor_speed_cmds.angular_velocities[2] =
      ((2.0 * rotor_drag_coeff_ * torques_and_thrust.body_torques.y() +
        arm_length_ * torques_and_thrust.body_torques.z() +
        rotor_drag_coeff_ * arm_length_ * mass_ *
            torques_and_thrust.collective_thrust) /
       (4.0 * rotor_drag_coeff_ * arm_length_)) /
      rotor_thrust_coeff_;
  rotor_speed_cmds.angular_velocities[3] =
      (-(2.0 * rotor_drag_coeff_ * torques_and_thrust.body_torques.x() +
         arm_length_ * torques_and_thrust.body_torques.z() -
         rotor_drag_coeff_ * arm_length_ * mass_ *
             torques_and_thrust.collective_thrust) /
       (4.0 * rotor_drag_coeff_ * arm_length_)) /
      rotor_thrust_coeff_;

  // TODO: Implement RPG saturation from PX4FMU

  // Apply limits and take square root
  for (int i = 0; i < 4; i++) {
    quadrotor_common::limit(&rotor_speed_cmds.angular_velocities[i], 0.0,
                            pow(max_rotor_speed_, 2.0));
    rotor_speed_cmds.angular_velocities[i] =
        sqrt(rotor_speed_cmds.angular_velocities[i]);
  }

  rotor_speed_cmds.header.stamp = ros::Time::now();
  return rotor_speed_cmds;
}

void RPGRotorsInterface::motorSpeedCallback(
    const mav_msgs::Actuators::ConstPtr& msg) {
  // The hummingbird that we use in our simulations has the following
  // rotor configuration
  // Rotor 0 spins clockwise
  //                 x
  //    0            ^
  //    |            |
  // 1--+--3    y <--+
  //    |
  //    2

  const double f0 = rotor_thrust_coeff_ * pow(msg->angular_velocities[0], 2.0);
  const double f1 = rotor_thrust_coeff_ * pow(msg->angular_velocities[1], 2.0);
  const double f2 = rotor_thrust_coeff_ * pow(msg->angular_velocities[2], 2.0);
  const double f3 = rotor_thrust_coeff_ * pow(msg->angular_velocities[3], 2.0);

  torques_and_thrust_estimate_.body_torques.x() = arm_length_ * (f1 - f3);
  torques_and_thrust_estimate_.body_torques.y() = arm_length_ * (f2 - f0);
  torques_and_thrust_estimate_.body_torques.z() =
      rotor_drag_coeff_ * (f0 - f1 + f2 - f3);
  torques_and_thrust_estimate_.collective_thrust = f0 + f1 + f2 + f3;
}

void RPGRotorsInterface::armInterfaceCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    interface_armed_ = true;
    ROS_INFO("[%s] Interface armed", pnh_.getNamespace().c_str());
  } else {
    interface_armed_ = false;
    ROS_INFO("[%s] Interface disarmed", pnh_.getNamespace().c_str());
  }
}

void RPGRotorsInterface::loadParameters() {
  inertia_ = Eigen::Matrix3d::Zero();

  quadrotor_common::getParam("inertia_x", inertia_(0, 0), 0.007, pnh_);
  quadrotor_common::getParam("inertia_y", inertia_(1, 1), 0.007, pnh_);
  quadrotor_common::getParam("inertia_z", inertia_(2, 2), 0.012, pnh_);

  K_lqr_ = Eigen::MatrixXd::Zero(3, 6);

  double body_rates_p_xy;
  double body_rates_d_xy;
  double body_rates_p_z;
  double body_rates_d_z;
  quadrotor_common::getParam("body_rates_p_xy", body_rates_p_xy, 0.15, pnh_);
  quadrotor_common::getParam("body_rates_d_xy", body_rates_d_xy, 0.5, pnh_);
  quadrotor_common::getParam("body_rates_p_z", body_rates_p_z, 0.03, pnh_);
  quadrotor_common::getParam("body_rates_d_z", body_rates_d_z, 0.1, pnh_);
  K_lqr_(0, 0) = body_rates_p_xy;
  K_lqr_(1, 1) = body_rates_p_xy;
  K_lqr_(2, 2) = body_rates_p_z;
  K_lqr_(0, 3) = body_rates_d_xy;
  K_lqr_(1, 4) = body_rates_d_xy;
  K_lqr_(2, 5) = body_rates_d_z;

  quadrotor_common::getParam("low_level_control_frequency",
                             low_level_control_frequency_, 200.0, pnh_);

  quadrotor_common::getParam("roll_pitch_cont_gain", roll_pitch_cont_gain_, 6.0,
                             pnh_);

  quadrotor_common::getParam("arm_length", arm_length_, 0.17, pnh_);

  quadrotor_common::getParam("rotor_drag_coeff", rotor_drag_coeff_, 0.016,
                             pnh_);
  quadrotor_common::getParam("rotor_thrust_coeff", rotor_thrust_coeff_,
                             8.54858e-06, pnh_);
  quadrotor_common::getParam("mass", mass_, 0.68 + 0.009 * 4.0, pnh_);
  quadrotor_common::getParam("max_rotor_speed", max_rotor_speed_, 838.0, pnh_);
}

}  // namespace rpg_rotors_interface

int main(int argc, char** argv) {
  ros::init(argc, argv, "rpg_rotors_interface");

  rpg_rotors_interface::RPGRotorsInterface rpg_rotors_interface;
  ros::spin();

  return 0;
}
