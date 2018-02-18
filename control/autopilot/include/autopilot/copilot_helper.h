/*
 * copilot_helper.h
 *
 *  Created on: Oct 13, 2016
 *      Author: Alessandro
 */

#pragma once

#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include "controller_common/flight_controller_base_helper.h"
#include "flight_controller/flight_controller_helper.h"
#include "quad_common/quad_desired_state.h"
#include "quad_msgs/QuadDesiredState.h"

namespace copilot
{
class CopilotHelper : public controller_common::FlightControllerBaseHelper
{
public:
  CopilotHelper();
  ~CopilotHelper();

  // Disable feedthrough, letting Copilot take over control
  bool disableFeedthrough(const double loop_rate_hz) const;

  // Enabling feedthrough passes control to the flight_controller. The Copilot's
  // current desired state is commanded to the flight_controller as its new
  // desired state.
  bool enableFeedthrough(const double loop_rate_hz,
                         const bool copilot_must_hover = true) const;

  // User should never publish desired state to copilot. Use FlightController
  // instead!
  void publishDesiredState(const quad_common::QuadDesiredState& state) const;

private:
  ros::Publisher feedthrough_pub_;
  flight_controller::FlightControllerHelper flightControllerHelper_;
};

}  // namespace copilot
