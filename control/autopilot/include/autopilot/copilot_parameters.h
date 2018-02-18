/*
 * copilot_parameters.h
 *
 *  Created on: Mar 27, 2014
 *      Author: ffontana
 */
#pragma once

namespace copilot
{

class CopilotSettings
{
public:
  CopilotSettings() :
      user_command_timeout_(0.0), start_land_velocity_(0.0), start_height_(0.0), propeller_ramp_down_timeout_(0.0), idle_thrust_(
          0.0), start_idle_duration_(0.0), optitrack_start_land_timeout_(0.0), optitrack_land_drop_height_(0.0), open_loop_takeoff_timeout_(
          0.0), vision_init_timeout_(0.0), open_loop_takeoff_thrust_(0.0), vision_land_drop_height_(0.0), emergency_land_duration_(
          0.0), emergency_land_thrust_(0.0), breaking_velocity_threshold_(0.0), breaking_timeout_(0.0), manual_velocity_input_timeout_(
          0.0), tau_manual_velocity_(0.0)
  {
  }
  ;

  ~CopilotSettings()
  {
  }
  ;

  double user_command_timeout_; // [s]

  double start_land_velocity_; // [m/s]
  double start_height_; // [m]

  double propeller_ramp_down_timeout_; // [s]
  double idle_thrust_; // [m/s]
  double start_idle_duration_; // [s]

  double optitrack_start_land_timeout_; // [s]
  double optitrack_land_drop_height_; // [m]

  double open_loop_takeoff_timeout_; // [s]
  double vision_init_timeout_; // [s]
  double open_loop_takeoff_thrust_; // [m/s^2]
  double vision_land_drop_height_; // [m]

  double emergency_land_duration_; // [s]
  double emergency_land_thrust_; // [m/s^2]

  double breaking_velocity_threshold_; // [m/s]
  double breaking_timeout_; // [s]

  double manual_velocity_input_timeout_; // [s]
  double tau_manual_velocity_; // []
};

} //namespace copilot
