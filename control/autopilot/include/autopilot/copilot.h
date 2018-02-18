/*
 * Copilot.h
 *
 *  Created on: Apr 1, 2014
 *      Author: ffontana
 */

#pragma once

#include <copilot/controller_with_primitives.h>
#include "std_msgs/Bool.h"
#include "quad_msgs/ControlCommand.h"

#include "flyingspace_bounds/flyingspace_bounds.h"
#include "flight_controller/flight_controller_helper.h"

namespace copilot
{

class Copilot : public copilot::ControllerWithPrimitives
{
public:
  Copilot(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);
  Copilot() :
      Copilot(ros::NodeHandle(), ros::NodeHandle("~"))
  {
  }
  virtual ~Copilot();

private:
  void checkUserControllerLoop(const ros::TimerEvent& time);
  bool userControllerIsOk();
  void enableUserController();
  void disableUserController();

  void feedthroughCallback(const std_msgs::Bool::ConstPtr &msg);
  void flightControllerCommandCallback(const quad_msgs::ControlCommand::ConstPtr &cmd);

  void updateDesiredState(const quad_common::QuadDesiredState& desired_state);
  bool PositionIsWithinArena();
  bool userControllerIsAlive();
  
  const double kLoopRateHz_ = 100;
  const double kFlightcontrollerTimeout = 5.0;

  ros::Subscriber flight_controller_command_sub_;
  ros::Subscriber feedthrough_sub_;

  ros::Timer check_flight_controller_loop_timer_;

  flyingspace_bounds::FlyingSpace flyingroom_;
  flight_controller::FlightControllerHelper flightControllerHelper_;
};

} /* namespace copilot */
