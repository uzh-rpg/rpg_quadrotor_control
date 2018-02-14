#include "state_predictor/state_predictor.h"

#include <quadrotor_common/quaternion_functions.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/parameter_helper.h>

namespace state_predictor
{

StatePredictor::StatePredictor(const ros::NodeHandle& nh,
                               const ros::NodeHandle& pnh) :
    nh_(nh), pnh_(pnh), ext_state_estimate_(), command_queue_(),
    initialization_called_(false), integration_step_size_(0.0),
    maximum_prediction_horizon_(0.0), tau_thrust_up_dynamics_(0.0),
    tau_thrust_down_dynamics_(0.0), tau_body_rates_dynamics_(0.0)
{
  if (!loadParameters())
  {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
}

StatePredictor::~StatePredictor()
{
}

void StatePredictor::initialize(
    const quadrotor_common::QuadStateEstimate& state_estimate)
{
  ext_state_estimate_.timestamp = state_estimate.timestamp;
  ext_state_estimate_.coordinate_frame = state_estimate.coordinate_frame;
  ext_state_estimate_.position = state_estimate.position;
  ext_state_estimate_.velocity = state_estimate.velocity;
  ext_state_estimate_.orientation = state_estimate.orientation;
  ext_state_estimate_.bodyrates = state_estimate.bodyrates;
  ext_state_estimate_.thrust = 0.0;

  // Initialize thrust with command if available
  if (!command_queue_.empty())
  {
    if (command_queue_.back().expected_execution_time
        < state_estimate.timestamp)
    {
      std::list<quadrotor_common::ControlCommand>::reverse_iterator
          reverse_queue_it = command_queue_.rbegin();
      // Find the last command applied before the time of the current
      // state estimate
      while (reverse_queue_it != command_queue_.rend())
      {
        if (reverse_queue_it->expected_execution_time
            > state_estimate.timestamp)
        {
          reverse_queue_it--;
          break;
        }
        reverse_queue_it++;
      }
      if (state_estimate.timestamp - reverse_queue_it->expected_execution_time
          < ros::Duration(maximum_prediction_horizon_))
      {
        ext_state_estimate_.thrust = reverse_queue_it->collective_thrust;
      }
    }
  }
  initialization_called_ = true;
}

bool StatePredictor::isInitialized()
{
  if ((ros::Time::now() - ext_state_estimate_.timestamp)
      > ros::Duration(maximum_prediction_horizon_))
  {
    // The current ext state estimate is too old so we have to reinitialize
    // the predictor
    initialization_called_ = false;
    return false;
  }
  return initialization_called_;
}

void StatePredictor::updateWithStateEstimate(
    const quadrotor_common::QuadStateEstimate& state_estimate)
{
  if (state_estimate.coordinate_frame
      == quadrotor_common::CoordinateFrame::INVALID)
  {
    // If the state estimate does not have a valid coordinate frame we do not
    // consider it in the predictor
    return;
  }

  // If the received state estimate is older than the current
  // ext_state_estimate_, we do nothing.
  if (state_estimate.timestamp < ext_state_estimate_.timestamp)
  {
    return;
  }

  // To get the extended state at the time of the received state_estimate,
  // we predict the ext_state_estimate_ to the time of this state_estimate
  // but use all normal states from the received state_estimate
  if (isInitialized())
  {
    QuadExtStateEstimate predicted_state = predictExtendedState(
        state_estimate.timestamp);

    ext_state_estimate_ = QuadExtStateEstimate(state_estimate);
    ext_state_estimate_.thrust = predicted_state.thrust;

    cleanUpCommandQueue();
  }
  else
  {
    initialize(state_estimate);
  }
}

quadrotor_common::QuadStateEstimate StatePredictor::predictState(
    const ros::Time& time) const
{
  double prediction_horizon = (time - ext_state_estimate_.timestamp).toSec();

  if (prediction_horizon < 0.0)
  {
    if (prediction_horizon < -0.001)
    {
      // Only warn for real problems, not for e.g. optitrack jitter.
      ROS_WARN_THROTTLE(
          2, "[%s] Requested a state estimate that is %.3fs before my last "
          "available state estimate.\n\tI'm a predictor not Captain Hindsight! "
          "Returning last available state estimate instead.",
          pnh_.getNamespace().c_str(), -prediction_horizon);
    }
    QuadExtStateEstimate last_available_state_estimate = ext_state_estimate_;
    return last_available_state_estimate.getQuadStateEstimate();
  }

  // Check if the prediction horizon is too large
  if (prediction_horizon > maximum_prediction_horizon_)
  {
    ROS_WARN_THROTTLE(
        2, "[%s] Too large state prediction horizon. Requested: %.3fs Max "
        "allowed: %.3fs.\n\tPredictions are hard to make, you know, especially "
        "about the future!",
        pnh_.getNamespace().c_str(), prediction_horizon,
        maximum_prediction_horizon_);
    QuadExtStateEstimate predicted_ext_state = predictExtendedState(
        ext_state_estimate_.timestamp
            + ros::Duration(maximum_prediction_horizon_));

    return predicted_ext_state.getQuadStateEstimate();
  }

  QuadExtStateEstimate predicted_ext_state = predictExtendedState(time);

  return predicted_ext_state.getQuadStateEstimate();
}

QuadExtStateEstimate StatePredictor::predictExtendedState(
    const ros::Time& time) const
{
  // Check if there are no commands in the queue
  if (command_queue_.empty())
  {
    return predictWithoutCommand(
        ext_state_estimate_, (time - ext_state_estimate_.timestamp).toSec());
  }

  QuadExtStateEstimate predicted_ext_state = ext_state_estimate_;
  if (ext_state_estimate_.timestamp
      < command_queue_.back().expected_execution_time)
  {
    // The oldest command is newer than our time
    // We need to push the state forward to the command time
    double time_left_to_predict =
        (time - predicted_ext_state.timestamp).toSec();
    double command_execution_time =
        (command_queue_.back().expected_execution_time
            - predicted_ext_state.timestamp).toSec();
    quadrotor_common::limit(&command_execution_time, 0.0, time_left_to_predict);

    predicted_ext_state = predictWithoutCommand(predicted_ext_state,
                                                command_execution_time);
  }

  // From now on we have commands and I am sure there is a command older than
  // the ext_state_estimate
  for (auto command_it = command_queue_.rbegin();
      command_it != command_queue_.rend(); command_it++)
  {
    // Check if there is a next command
    auto next_command = std::next(command_it);
    if (next_command != command_queue_.rend())
    {
      // There is a next command
      double time_left_to_predict =
          (time - predicted_ext_state.timestamp).toSec();
      // Time to next command can be negative then set it zero (no integration)
      // because command and next_command are behind us
      double command_execution_time = (next_command->expected_execution_time
          - predicted_ext_state.timestamp).toSec();
      quadrotor_common::limit(&command_execution_time, 0.0,
                              time_left_to_predict);
      predicted_ext_state = predictWithCommand(predicted_ext_state, *command_it,
                                               command_execution_time);
    }
    else
    {
      // There are no more commands
      // Predict to the end of the horizon
      double time_left_to_predict =
          (time - predicted_ext_state.timestamp).toSec();
      predicted_ext_state = predictWithCommand(predicted_ext_state, *command_it,
                                               time_left_to_predict);

      return predicted_ext_state;
    }

    // Are we done?
    if (predicted_ext_state.timestamp >= time)
    {
      return predicted_ext_state;
    }
  }

  ROS_ERROR(
      "[%s] Prediction reached the last command without being able to predict "
      "to the end",
      pnh_.getNamespace().c_str());
  return predicted_ext_state;
}

void StatePredictor::pushCommandToQueue(
    const quadrotor_common::ControlCommand& cmd)
{
  // Note that when we use ANGLE control mode, the bodyrates contain the feed
  // forward terms from a reference trajectory so we can use both type of
  // control modes for prediction.
  command_queue_.push_front(cmd);
}

QuadExtStateEstimate StatePredictor::predictWithCommand(
    const QuadExtStateEstimate& old_state,
    const quadrotor_common::ControlCommand& cmd,
    const double total_integration_time) const
{
  QuadExtStateEstimate new_state = old_state;
  double current_integration_step = 0.0;
  double integrated_time = 0.0;

  while (integrated_time < total_integration_time)
  {
    current_integration_step = std::min(
        integration_step_size_, total_integration_time - integrated_time);
    new_state = dynamics(new_state, cmd, current_integration_step);
    integrated_time += current_integration_step;
  }

  return new_state;
}

QuadExtStateEstimate StatePredictor::dynamics(
    const QuadExtStateEstimate& old_state,
    const quadrotor_common::ControlCommand& cmd, const double dt) const
{
  QuadExtStateEstimate new_state;
  if (cmd.collective_thrust == 0.0)
  {
    new_state = old_state;
  }
  else
  {
    double alpha_thrust = 1.0 - exp(-dt / tau_thrust_up_dynamics_);
    if (cmd.collective_thrust < old_state.thrust)
    {
      alpha_thrust = 1.0 - exp(-dt / tau_thrust_down_dynamics_);
    }
    double alpha_angle_rate = 1.0 - exp(-dt / tau_body_rates_dynamics_);

    new_state.thrust = (1.0 - alpha_thrust) * old_state.thrust
        + alpha_thrust * cmd.collective_thrust;
    new_state.bodyrates = (1.0 - alpha_angle_rate) * old_state.bodyrates
        + alpha_angle_rate * cmd.bodyrates;

    new_state.orientation = quadrotor_common::integrateQuaternion(
        old_state.orientation, new_state.bodyrates, dt);
    new_state.velocity = old_state.velocity
        + (new_state.orientation * Eigen::Vector3d(0.0, 0.0, new_state.thrust)
            - Eigen::Vector3d(0.0, 0.0, 9.81)) * dt;
    new_state.position = old_state.position + new_state.velocity * dt;

    new_state.timestamp = old_state.timestamp + ros::Duration(dt);
    new_state.coordinate_frame = old_state.coordinate_frame;
  }
  return new_state;
}

QuadExtStateEstimate StatePredictor::predictWithoutCommand(
    const QuadExtStateEstimate& old_state,
    const double total_integration_time) const
{
  QuadExtStateEstimate new_state = old_state;
  double current_integration_step = 0.0;
  double integrated_time = 0.0;

  while (integrated_time < total_integration_time)
  {
    current_integration_step = std::min(
        integration_step_size_, total_integration_time - integrated_time);
    new_state = dynamics(new_state, current_integration_step);
    integrated_time += current_integration_step;
  }

  return new_state;
}

QuadExtStateEstimate StatePredictor::dynamics(
    const QuadExtStateEstimate& old_state, const double dt) const
{
  QuadExtStateEstimate new_state;

  new_state.timestamp = old_state.timestamp + ros::Duration(dt);
  new_state.coordinate_frame = old_state.coordinate_frame;
  new_state.position = old_state.position + old_state.velocity * dt;
  new_state.velocity = old_state.velocity;
  new_state.orientation = quadrotor_common::integrateQuaternion(
      old_state.orientation, old_state.bodyrates, dt);
  new_state.bodyrates = old_state.bodyrates;
  new_state.thrust = old_state.thrust;

  return new_state;
}

void StatePredictor::cleanUpCommandQueue()
{
  if (command_queue_.size() <= 1)
  {
    return;
  }

  for (;;)
  {
    auto oldest_command = command_queue_.rbegin();
    auto second_oldest_command = std::next(oldest_command);

    if (second_oldest_command->expected_execution_time
        < ext_state_estimate_.timestamp)
    {
      command_queue_.pop_back();
    }
    else
    {
      break;
    }
    if (command_queue_.size() == 1)
    {
      break;
    }
  }
}

bool StatePredictor::loadParameters()
{
  quadrotor_common::getParam("state_predictor/integration_step_size",
                             integration_step_size_, 0.001, pnh_);
  quadrotor_common::getParam("state_predictor/maximum_prediction_horizon",
                             maximum_prediction_horizon_, 0.1, pnh_);

  if (!quadrotor_common::getParam("state_predictor/tau_thrust_up_dynamics",
                                  tau_thrust_up_dynamics_, pnh_))
  {
    return false;
  }
  if (!quadrotor_common::getParam("state_predictor/tau_thrust_down_dynamics",
                                  tau_thrust_down_dynamics_, pnh_))
  {
    return false;
  }
  if (!quadrotor_common::getParam("state_predictor/tau_angle_rate_dynamics",
                                  tau_body_rates_dynamics_, pnh_))
  {
    return false;
  }

  return true;
}

} // namespace state_predictor

