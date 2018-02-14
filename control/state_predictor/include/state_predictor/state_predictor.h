#pragma once

#include <Eigen/Dense>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_common/control_command.h>
#include <ros/ros.h>

#include "state_predictor/quad_ext_state_estimate.h"

namespace state_predictor
{

class StatePredictor
{
public:
  StatePredictor(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  StatePredictor() :
      StatePredictor(ros::NodeHandle(), ros::NodeHandle("~"))
  {
  }
  ~StatePredictor();

  void initialize(const quadrotor_common::QuadStateEstimate& state_estimate);
  void updateWithStateEstimate(
      const quadrotor_common::QuadStateEstimate& state_estimate);
  quadrotor_common::QuadStateEstimate predictState(const ros::Time& time) const;
  bool isInitialized();

  void pushCommandToQueue(const quadrotor_common::ControlCommand& cmd);

private:
  QuadExtStateEstimate predictExtendedState(const ros::Time& time) const;
  QuadExtStateEstimate predictWithCommand(
      const QuadExtStateEstimate& old_state,
      const quadrotor_common::ControlCommand& cmd,
      const double total_integration_time) const;
  QuadExtStateEstimate dynamics(const QuadExtStateEstimate& old_state,
                                const quadrotor_common::ControlCommand& cmd,
                                const double dt) const;
  QuadExtStateEstimate predictWithoutCommand(
      const QuadExtStateEstimate& old_state,
      const double total_integration_time) const;
  QuadExtStateEstimate dynamics(const QuadExtStateEstimate& old_state,
                                const double dt) const;

  void cleanUpCommandQueue();

  bool loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  state_predictor::QuadExtStateEstimate ext_state_estimate_;

  std::list<quadrotor_common::ControlCommand> command_queue_;

  bool initialization_called_;

  // Parameters
  double integration_step_size_;
  double maximum_prediction_horizon_;

  // Time constants for first order system approximations of thrust and body
  // rates dynamics
  double tau_thrust_up_dynamics_;
  double tau_thrust_down_dynamics_;
  double tau_body_rates_dynamics_;
};

} // namespace state_predictor

