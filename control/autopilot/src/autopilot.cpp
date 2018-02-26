#include "autopilot/autopilot.h"

#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

namespace autopilot
{

AutoPilot::AutoPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) :
    nh_(nh), pnh_(pnh), state_predictor_(nh_, pnh_), reference_state_(),
    received_state_est_(), desired_velocity_command_(),
    reference_state_input_(), received_low_level_feedback_(),
    autopilot_state_(States::OFF), state_before_rc_manual_flight_(States::OFF),
    state_estimate_available_(false), time_of_switch_to_current_state_(),
    first_time_in_new_state_(true), initial_start_position_(),
    initial_land_position_(), time_to_ramp_down_(false),
    time_started_ramping_down_(), initial_drop_thrust_(0.0),
    time_last_velocity_command_handled_(),
    time_last_reference_state_input_received_(),
    desired_state_after_breaking_(States::HOVER), requested_go_to_pose_(),
    received_go_to_pose_command_(false), stop_go_to_pose_thread_(false),
    stop_watchdog_thread_(false), time_last_state_estimate_received_(),
    time_started_emergency_landing_(), destructor_invoked_(false),
    time_last_autopilot_feedback_published_()

{
  if (!loadParameters())
  {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Publishers
  control_command_pub_ = nh_.advertise<quadrotor_msgs::ControlCommand>(
      "control_command", 1);
  autopilot_feedback_pub_ = nh_.advertise<quadrotor_msgs::AutopilotFeedback>(
      "autopilot/feedback", 1);

  // Subscribers
  state_estimate_sub_ = nh_.subscribe("autopilot/state_estimate", 1,
                                      &AutoPilot::stateEstimateCallback, this);
  low_level_feedback_sub_ = nh_.subscribe("low_level_feedback", 1,
                                          &AutoPilot::lowLevelFeedbackCallback,
                                          this);

  pose_command_sub_ = nh_.subscribe("autopilot/pose_command", 1,
                                    &AutoPilot::poseCommandCallback, this);
  velocity_command_sub_ = nh_.subscribe("autopilot/velocity_command", 1,
                                        &AutoPilot::velocityCommandCallback,
                                        this);
  reference_state_sub_ = nh_.subscribe("autopilot/reference_state", 1,
                                       &AutoPilot::referenceStateCallback,
                                       this);
  trajectory_sub_ = nh_.subscribe("autopilot/trajectory", 1,
                                  &AutoPilot::trajectoryCallback, this);
  control_command_input_sub_ = nh_.subscribe(
      "autopilot/control_command_input", 1,
      &AutoPilot::controlCommandInputCallback, this);

  start_sub_ = nh_.subscribe("autopilot/start", 1, &AutoPilot::startCallback,
                             this);
  force_hover_sub_ = nh_.subscribe("autopilot/force_hover", 1,
                                   &AutoPilot::forceHoverCallback, this);
  land_sub_ = nh_.subscribe("autopilot/land", 1, &AutoPilot::landCallback,
                            this);
  off_sub_ = nh_.subscribe("autopilot/off", 1, &AutoPilot::offCallback, this);

  // Start watchdog thread
  try
  {
    watchdog_thread_ = std::thread(&AutoPilot::watchdogThread, this);
  }
  catch (...)
  {
    ROS_ERROR("[%s] Could not successfully start watchdog thread.",
              pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Start go to pose thread
  try
  {
    go_to_pose_thread_ = std::thread(&AutoPilot::goToPoseThread, this);
  }
  catch (...)
  {
    ROS_ERROR("[%s] Could not successfully start go to pose thread.",
              pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
}

AutoPilot::~AutoPilot()
{
  destructor_invoked_ = true;

  // Stop go to pose thread
  stop_go_to_pose_thread_ = true;
  // Wait for go to pose thread to finish
  go_to_pose_thread_.join();

  // Stop watchdog thread
  stop_watchdog_thread_ = true;
  // Wait for watchdog thread to finish
  watchdog_thread_.join();

  // Send out an off command to ensure quadrotor is off
  quadrotor_common::ControlCommand control_cmd;
  control_cmd.zero();
  publishControlCommand(control_cmd);
}

// Watchdog thread to check when the last state estimate was received
// -> trigger emergency landing if necessary
// -> set state_estimate_available_ false
void AutoPilot::watchdogThread()
{
  ros::Rate watchdog_rate(1.0 / kWatchdogFrequency_);
  while (ros::ok() && !stop_watchdog_thread_)
  {
    watchdog_rate.sleep();

    std::lock_guard<std::mutex> main_lock(main_mutex_);

    const ros::Time time_now = ros::Time::now();

    if (autopilot_state_ != States::OFF
        && autopilot_state_ != States::EMERGENCY_LAND
        && time_now - time_last_state_estimate_received_
            > ros::Duration(state_estimate_timeout_))
    {
      state_estimate_available_ = false;
      setAutoPilotStateForced(States::EMERGENCY_LAND);
    }

    if (autopilot_state_ == States::EMERGENCY_LAND)
    {
      // Check timeout to switch to OFF
      if (time_now - time_started_emergency_landing_
          > ros::Duration(emergency_land_duration_))
      {
        setAutoPilotStateForced(States::OFF);
      }

      // Send emergency landing control command
      quadrotor_common::ControlCommand control_cmd;
      control_cmd.armed = true;
      control_cmd.control_mode = quadrotor_common::ControlMode::ATTITUDE;
      control_cmd.collective_thrust = emergency_land_thrust_;
      control_cmd.timestamp = time_now;
      control_cmd.expected_execution_time = control_cmd.timestamp
          + ros::Duration(control_command_delay_);
      publishControlCommand(control_cmd);
    }

    // Mutex is unlocked because it goes out of scope here
  }
}

// Planning thread for planning GO_TO_POSE actions
// -> when done call trajectoryCallback with computed trajectory
void AutoPilot::goToPoseThread()
{
  ros::Rate idle_rate(1.0 / kGoToPoseIdleFrequency_);
  while (ros::ok() && !stop_go_to_pose_thread_)
  {
    idle_rate.sleep();

    std::lock_guard<std::mutex> go_to_pose_lock(go_to_pose_mutex_);

    if (received_go_to_pose_command_)
    {
      quadrotor_common::TrajectoryPoint start_state;
      {
        // Store current reference state as a start state for trajectory
        // planning
        std::lock_guard<std::mutex> main_lock(main_mutex_);

        // Note that since we only allow go to pose actions starting from hover
        // state, the derivatives of reference state are all zeros
        start_state = reference_state_;

        // Main mutex is unlocked because it goes out of scope here
      }

      // Compose desired end state
      quadrotor_common::TrajectoryPoint end_state;
      end_state.position = quadrotor_common::geometryToEigen(
          requested_go_to_pose_.pose.position);
      end_state.orientation = quadrotor_common::geometryToEigen(
          requested_go_to_pose_.pose.orientation);
      end_state.heading = quadrotor_common::quaternionToEulerAnglesZYX(
          end_state.orientation).z();

      quadrotor_common::Trajectory go_to_pose_traj =
          trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
              start_state, end_state, kGoToPosePolynomialOrderOfContinuity_,
              go_to_pose_max_velocity_, go_to_pose_max_normalized_thrust_,
              go_to_pose_max_roll_pitch_rate_,
              kGoToPoseTrajectorySamplingFrequency_);

      trajectory_generation_helper::heading::addConstantHeadingRate(
          start_state.heading, end_state.heading, &go_to_pose_traj);

      quadrotor_msgs::Trajectory::Ptr go_to_pose_traj_ptr(
                new quadrotor_msgs::Trajectory);
      *go_to_pose_traj_ptr = go_to_pose_traj.toRosMessage();

      received_go_to_pose_command_ = false;
      trajectoryCallback(go_to_pose_traj_ptr);
    }

    // Mutex is unlocked because it goes out of scope here
  }
}

void AutoPilot::stateEstimateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  received_state_est_ = quadrotor_common::QuadStateEstimate(*msg);
  if (!received_state_est_.isValid())
  {
    state_estimate_available_ = false;
    if (autopilot_state_ != States::OFF)
    {
      // Do not run control loop if state estimate is not valid
      // Only allow OFF state without a valid state estimate
      return;
    }
  }
  else
  {
    state_estimate_available_ = true;
    time_last_state_estimate_received_ = ros::Time::now();
  }

  if (!velocity_estimate_in_world_frame_)
  {
    received_state_est_.transformVelocityToWorldFrame();
  }

  // Push received state estimate into predictor
  state_predictor_.updateWithStateEstimate(received_state_est_);

  quadrotor_common::ControlCommand control_cmd;

  ros::Time wall_time_now = ros::Time::now();
  ros::Time command_execution_time = wall_time_now
      + ros::Duration(control_command_delay_);

  quadrotor_common::QuadStateEstimate predicted_state =
      getPredictedStateEstimate(command_execution_time);

  ros::Duration trajectory_execution_left_duration(0.0);
  int trajectories_left_in_queues = 0;
  const ros::Time start_control_command_computation = ros::Time::now();
  // Compute control command depending on autopilot state
  switch (autopilot_state_)
  {
    case States::OFF:
      control_cmd.zero();
      // Reset refence_state so we don't have random values in our autopilot
      // feedback message
      reference_state_ = quadrotor_common::TrajectoryPoint();
      break;
    case States::START:
      control_cmd = start(predicted_state);
      break;
    case States::HOVER:
      control_cmd = hover(predicted_state);
      break;
    case States::LAND:
      control_cmd = land(predicted_state);
      break;
    case States::EMERGENCY_LAND:
      if (state_estimate_available_)
      {
        // If we end up here it means that we have regained a valid state
        // estimate, so lets go back to HOVER state
        setAutoPilotState(States::HOVER);
        control_cmd = hover(predicted_state);
      }
      break;
    case States::BREAKING:
      control_cmd = breakVelocity(predicted_state);
      break;
    case States::GO_TO_POSE:
      // Currently not necessary but might be useful for MPC control
      break;
    case States::VELOCITY_CONTROL:
      control_cmd = velocityControl(predicted_state);
      break;
    case States::REFERENCE_CONTROL:
      control_cmd = followReference(predicted_state);
      break;
    case States::TRAJECTORY_CONTROL:
      // TODO: Set trajectory_execution_left_duration
      // TODO: Set trajectories_left_in_queues
      // NOTE: This function also needs to set the reference_state_ variable
      // according to the currently used trajectory sample
      break;
    case States::COMMAND_FEEDTHROUGH:
      // Do nothing here, command is being published in the command input
      // callback directly
      break;
    case States::RC_MANUAL:
      // Send an armed command with zero body rates and hover thrust to avoid
      // letting the low level part switch off when the remote control gives
      // the authority back to our autonomous controller
      control_cmd.zero();
      control_cmd.armed = true;
      control_cmd.collective_thrust = kGravityAcc_;
      break;
  }
  const ros::Duration control_computation_time = ros::Time::now()
      - start_control_command_computation;

  if (autopilot_state_ != States::COMMAND_FEEDTHROUGH)
  {
    control_cmd.timestamp = wall_time_now;
    control_cmd.expected_execution_time = command_execution_time;
    publishControlCommand(control_cmd);
  }

  // Publish autopilot feedback throttled down to a maximum frequency
  if ((ros::Time::now() - time_last_autopilot_feedback_published_)
      >= ros::Duration(1.0 / kMaxAutopilotFeedbackPublishFrequency_))
  {
    publishAutopilotFeedback(autopilot_state_,
                             ros::Duration(control_command_delay_),
                             control_computation_time,
                             trajectory_execution_left_duration,
                             trajectories_left_in_queues,
                             received_low_level_feedback_, reference_state_,
                             predicted_state);
  }

  // Mutex is unlocked because it goes out of scope here
}

void AutoPilot::lowLevelFeedbackCallback(
    const quadrotor_msgs::LowLevelFeedback::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  received_low_level_feedback_ = *msg;

  if (msg->control_mode == msg->RC_MANUAL
      && autopilot_state_ != States::RC_MANUAL)
  {
    setAutoPilotState(States::RC_MANUAL);
  }
  if (msg->control_mode != msg->RC_MANUAL
      && autopilot_state_ == States::RC_MANUAL)
  {
    if (state_before_rc_manual_flight_ == States::OFF)
    {
      setAutoPilotState(States::OFF);
    }
    else
    {
      setAutoPilotState(States::HOVER);
    }
  }

  // Mutex is unlocked because it goes out of scope here
}

void AutoPilot::poseCommandCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  // We need to lock both the go to pose mutex and the main mutex here.
  // The order of locking has to match the one in goToPoseThread() to prevent
  // deadlocks. So the go to pose mutex has to be locked first.
  std::lock_guard<std::mutex> go_to_pose_lock(go_to_pose_mutex_);
  std::lock_guard<std::mutex> main_lock(main_mutex_);

  // Idea: A trajectory is planned to the desired pose in a separate
  // thread. Once the thread is done it calls the trajectoryCallback with
  // the computed trajectory
  if (autopilot_state_ == States::HOVER)
  {
    requested_go_to_pose_ = *msg;
    received_go_to_pose_command_ = true;
  }

  // Mutexes are unlocked because they go out of scope here
}

void AutoPilot::velocityCommandCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (quadrotor_common::geometryToEigen(msg->twist.linear).norm()
      <= kVelocityCommandZeroThreshold_
      && fabs(msg->twist.angular.z) <= kVelocityCommandZeroThreshold_)
  {
    // Only consider commands with non negligible velocities
    return;
  }
  if (autopilot_state_ != States::HOVER
      && autopilot_state_ != States::VELOCITY_CONTROL)
  {
    return;
  }
  if (autopilot_state_ != States::VELOCITY_CONTROL)
  {
    setAutoPilotState(States::VELOCITY_CONTROL);
  }

  desired_velocity_command_ = *msg;
  desired_velocity_command_.header.stamp = ros::Time::now();

  // Mutex is unlocked because it goes out of scope here
}

void AutoPilot::referenceStateCallback(
    const quadrotor_msgs::TrajectoryPoint::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (autopilot_state_ != States::HOVER
      && autopilot_state_ != States::REFERENCE_CONTROL)
  {
    return;
  }
  if (autopilot_state_ != States::REFERENCE_CONTROL)
  {
    if ((reference_state_.position
        - quadrotor_common::geometryToEigen(msg->pose.position)).norm()
        < kPositionJumpTolerance_)
    {
      setAutoPilotState(States::REFERENCE_CONTROL);
    }
    else
    {
      ROS_WARN("[%s] Received first reference state that is more than %fm away "
               "from current position, will not go to REFERENCE_CONTROL mode.",
               pnh_.getNamespace().c_str(), kPositionJumpTolerance_);
    }
  }

  time_last_reference_state_input_received_ = ros::Time::now();
  reference_state_input_ = *msg;

  // Mutex is unlocked because it goes out of scope here
}

void AutoPilot::trajectoryCallback(
    const quadrotor_msgs::Trajectory::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  // TODO: Idea: trajectories are being pushed into a queue and consecutively
  // executed if there are no jumps in the beginning and between them

  // Mutex is unlocked because it goes out of scope here
}

void AutoPilot::controlCommandInputCallback(
    const quadrotor_msgs::ControlCommand::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (autopilot_state_ != States::OFF && autopilot_state_ != States::HOVER)
  {
    // Only allow this if the current state is OFF or HOVER
    return;
  }

  if (autopilot_state_ != States::COMMAND_FEEDTHROUGH)
  {
    setAutoPilotState(States::COMMAND_FEEDTHROUGH);
  }

  control_command_pub_.publish(*msg);

  // Mutex is unlocked because it goes out of scope here
}

void AutoPilot::startCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  ROS_INFO_THROTTLE(0.5, "[%s] START command received",
                    pnh_.getNamespace().c_str());
  if (autopilot_state_ == States::OFF)
  {
    if (state_estimate_available_)
    {
      if (received_state_est_.coordinate_frame
          == quadrotor_common::QuadStateEstimate::CoordinateFrame::WORLD
          || received_state_est_.coordinate_frame
              == quadrotor_common::QuadStateEstimate::CoordinateFrame::OPTITRACK)
      {
        ROS_INFO(
            "[%s] Absolute state estimate available, taking off based on it",
            pnh_.getNamespace().c_str());
        setAutoPilotState(States::START);
      }
      else if (received_state_est_.coordinate_frame
          == quadrotor_common::QuadStateEstimate::CoordinateFrame::VISION
          || received_state_est_.coordinate_frame
              == quadrotor_common::QuadStateEstimate::CoordinateFrame::LOCAL)
      {
        ROS_INFO("[%s] Relative state estimate available, switch to hover",
                 pnh_.getNamespace().c_str());
        setAutoPilotState(States::HOVER);
      }
    }
    else
    {
      ROS_ERROR("[%s] No state estimate available, will not start",
                pnh_.getNamespace().c_str());
    }
  }
  else
  {
    ROS_WARN("[%s] Autopilot is not OFF, will not switch to START",
             pnh_.getNamespace().c_str());
  }

  // Mutex is unlocked because it goes out of scope here
}

void AutoPilot::forceHoverCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  ROS_INFO_THROTTLE(0.5, "[%s] FORCE HOVER command received",
                    pnh_.getNamespace().c_str());

  if (autopilot_state_ == States::OFF || autopilot_state_ == States::HOVER
      || autopilot_state_ == States::EMERGENCY_LAND
      || autopilot_state_ == States::RC_MANUAL)
  {
    return;
  }

  setAutoPilotState(States::HOVER);

  // Mutex is unlocked because it goes out of scope here
}

void AutoPilot::landCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  ROS_INFO_THROTTLE(0.5, "[%s] LAND command received",
                    pnh_.getNamespace().c_str());
  if (autopilot_state_ == States::OFF || autopilot_state_ == States::LAND
      || autopilot_state_ == States::EMERGENCY_LAND
      || autopilot_state_ == States::RC_MANUAL)
  {
    return;
  }

  setAutoPilotState(States::LAND);

  // Mutex is unlocked because it goes out of scope here
}

void AutoPilot::offCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if (destructor_invoked_)
  {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (autopilot_state_ != States::OFF)
  {
    ROS_INFO("[%s] OFF command received", pnh_.getNamespace().c_str());
    setAutoPilotStateForced(States::OFF);
    // Allow user to take over manually and land the vehicle, then off the
    // controller and disable the RC without the vehicle going back to hover
    state_before_rc_manual_flight_ = States::OFF;
  }

  // Mutex is unlocked because it goes out of scope here
}

quadrotor_common::ControlCommand AutoPilot::start(
    const quadrotor_common::QuadStateEstimate& state_estimate)
{
  quadrotor_common::ControlCommand command;

  if (first_time_in_new_state_)
  {
    first_time_in_new_state_ = false;
    initial_start_position_ = state_estimate.position;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = state_estimate.position;
    reference_state_.heading = quadrotor_common::quaternionToEulerAnglesZYX(
        state_estimate.orientation).z();
    if (state_estimate.position.z() >= optitrack_land_drop_height_)
    {
      setAutoPilotState(States::HOVER);
    }
  }

  if (timeInCurrentState() > optitrack_start_land_timeout_
      || reference_state_.position.z() >= optitrack_start_height_)
  {
    setAutoPilotState(States::HOVER);
  }
  else
  {
    if (timeInCurrentState() < start_idle_duration_)
    {
      command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
      command.armed = true;
      command.bodyrates = Eigen::Vector3d::Zero();
      command.collective_thrust = idle_thrust_;
      return command;
    }
    else
    {
      reference_state_.position.z() = initial_start_position_.z()
          + start_land_velocity_
              * (timeInCurrentState() - start_idle_duration_);
      reference_state_.velocity.z() = start_land_velocity_;
    }
  }

  command = base_controller_.run(state_estimate, reference_state_,
                                 base_controller_params_);

  return command;
}

quadrotor_common::ControlCommand AutoPilot::hover(
    const quadrotor_common::QuadStateEstimate& state_estimate)
{
  if (first_time_in_new_state_)
  {
    first_time_in_new_state_ = false;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = state_estimate.position;
    reference_state_.heading = quadrotor_common::quaternionToEulerAnglesZYX(
        state_estimate.orientation).z();
  }

  const quadrotor_common::ControlCommand command = base_controller_.run(
      state_estimate, reference_state_, base_controller_params_);

  return command;
}

quadrotor_common::ControlCommand AutoPilot::land(
    const quadrotor_common::QuadStateEstimate& state_estimate)
{
  quadrotor_common::ControlCommand command;

  if (first_time_in_new_state_)
  {
    first_time_in_new_state_ = false;
    initial_land_position_ = state_estimate.position;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = state_estimate.position;
    reference_state_.heading = quadrotor_common::quaternionToEulerAnglesZYX(
        state_estimate.orientation).z();
    // Reset ramp down flag
    time_to_ramp_down_ = false;
  }

  reference_state_.position.z() = fmax(
      0.0,
      initial_land_position_.z() - start_land_velocity_ * timeInCurrentState());
  reference_state_.velocity.z() = -start_land_velocity_;

  command = base_controller_.run(state_estimate, reference_state_,
                                 base_controller_params_);

  if (received_state_est_.coordinate_frame
      == quadrotor_common::QuadStateEstimate::CoordinateFrame::WORLD
      || received_state_est_.coordinate_frame
          == quadrotor_common::QuadStateEstimate::CoordinateFrame::OPTITRACK)
  {
    // We only allow ramping down the propellers if we have an absolute stat
    // estimate available, otherwise we just keep going down "forever"
    if (!time_to_ramp_down_
        && (state_estimate.position.z() < optitrack_land_drop_height_
            || timeInCurrentState() > optitrack_start_land_timeout_))
    {
      time_to_ramp_down_ = true;
      time_started_ramping_down_ = ros::Time::now();
    }
  }

  if (time_to_ramp_down_)
  {
    // we are low enough -> ramp down the thrust
    // we timed out on landing -> ramp down the thrust
    ROS_INFO_THROTTLE(2, "[%s] Ramping propeller down",
                      pnh_.getNamespace().c_str());
    command.collective_thrust = initial_drop_thrust_
        - initial_drop_thrust_ / propeller_ramp_down_timeout_
            * (ros::Time::now() - time_started_ramping_down_).toSec();
  }

  if (command.collective_thrust <= 0.0)
  {
    setAutoPilotState(States::OFF);
    command.zero();
  }

  return command;
}

quadrotor_common::ControlCommand AutoPilot::breakVelocity(
    const quadrotor_common::QuadStateEstimate& state_estimate)
{
  quadrotor_common::ControlCommand command;

  if (first_time_in_new_state_)
  {
    first_time_in_new_state_ = false;
    initial_land_position_ = state_estimate.position;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = state_estimate.position;
    reference_state_.velocity = state_estimate.velocity;
    reference_state_.heading = quadrotor_common::quaternionToEulerAnglesZYX(
        state_estimate.orientation).z();
  }

  if (state_estimate.velocity.norm() < breaking_velocity_threshold_
      || timeInCurrentState() > breaking_timeout_)
  {
    const double current_heading = reference_state_.heading;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = state_estimate.position;
    reference_state_.heading = current_heading;
    setAutoPilotStateForced(desired_state_after_breaking_);
  }

  command = base_controller_.run(state_estimate, reference_state_,
                                 base_controller_params_);
  return command;
}

quadrotor_common::ControlCommand AutoPilot::velocityControl(
    const quadrotor_common::QuadStateEstimate& state_estimate)
{
  quadrotor_common::ControlCommand command;

  if (first_time_in_new_state_)
  {
    first_time_in_new_state_ = false;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = state_estimate.position;
    reference_state_.heading = quadrotor_common::quaternionToEulerAnglesZYX(
        state_estimate.orientation).z();
    time_last_velocity_command_handled_ = ros::Time::now();
  }

  if ((ros::Time::now() - desired_velocity_command_.header.stamp)
      > ros::Duration(velocity_command_input_timeout_))
  {
    desired_velocity_command_.twist.linear.x = 0.0;
    desired_velocity_command_.twist.linear.y = 0.0;
    desired_velocity_command_.twist.linear.z = 0.0;
    desired_velocity_command_.twist.angular.z = 0.0;
  }

  const double dt =
      (ros::Time::now() - time_last_velocity_command_handled_).toSec();
  const double alpha_velocity = 1 - exp(-dt / tau_velocity_command_);

  const Eigen::Vector3d commanded_velocity = quadrotor_common::geometryToEigen(
      desired_velocity_command_.twist.linear);
  reference_state_.velocity = (1.0 - alpha_velocity) * reference_state_.velocity
      + alpha_velocity * commanded_velocity;

  if (reference_state_.velocity.norm() < kVelocityCommandZeroThreshold_
      && commanded_velocity.norm() < kVelocityCommandZeroThreshold_)
  {
    reference_state_.velocity = Eigen::Vector3d::Zero();
    if (fabs(desired_velocity_command_.twist.angular.z)
        < kVelocityCommandZeroThreshold_)
    {
      setAutoPilotState(States::HOVER);
    }
  }
  reference_state_.position += reference_state_.velocity * dt;

  reference_state_.heading += desired_velocity_command_.twist.angular.z * dt;
  reference_state_.heading = quadrotor_common::wrapMinusPiToPi(
      reference_state_.heading);
  reference_state_.heading_rate = desired_velocity_command_.twist.angular.z;

  time_last_velocity_command_handled_ = ros::Time::now();

  command = base_controller_.run(state_estimate, reference_state_,
                                 base_controller_params_);

  return command;
}

quadrotor_common::ControlCommand AutoPilot::followReference(
    const quadrotor_common::QuadStateEstimate& state_estimate)
{
  quadrotor_common::ControlCommand command;

  if (first_time_in_new_state_)
  {
    first_time_in_new_state_ = false;
  }

  if ((ros::Time::now() - time_last_reference_state_input_received_)
      > ros::Duration(reference_state_input_timeout_))
  {
    setAutoPilotState(States::HOVER);
  }

  command = base_controller_.run(state_estimate, reference_state_,
                                 base_controller_params_);

  return command;
}

void AutoPilot::setAutoPilotState(const States& new_state)
{
  time_of_switch_to_current_state_ = ros::Time::now();
  first_time_in_new_state_ = true;

  if (!state_estimate_available_ && new_state != States::OFF
      && new_state != States::EMERGENCY_LAND
      && new_state != States::COMMAND_FEEDTHROUGH
      && new_state != States::RC_MANUAL)
  {
    autopilot_state_ = States::EMERGENCY_LAND;
    time_started_emergency_landing_ = ros::Time::now();
    return;
  }

  if (new_state == States::HOVER || new_state == States::LAND
      || new_state == States::GO_TO_POSE)
  {
    desired_state_after_breaking_ = new_state;
    autopilot_state_ = States::BREAKING;
    return;
  }
  if (new_state == States::RC_MANUAL)
  {
    if (autopilot_state_ == States::OFF)
    {
      state_before_rc_manual_flight_ = States::OFF;
    }
    else
    {
      state_before_rc_manual_flight_ = States::HOVER;
    }
  }
  if (new_state == States::EMERGENCY_LAND)
  {
    time_started_emergency_landing_ = ros::Time::now();
  }

  autopilot_state_ = new_state;
}

void AutoPilot::setAutoPilotStateForced(const States& new_state)
{
  time_of_switch_to_current_state_ = ros::Time::now();
  first_time_in_new_state_ = true;
  autopilot_state_ = new_state;
  if (new_state == States::EMERGENCY_LAND)
  {
    time_started_emergency_landing_ = ros::Time::now();
  }
}

double AutoPilot::timeInCurrentState() const
{
  return (ros::Time::now() - time_of_switch_to_current_state_).toSec();
}

quadrotor_common::QuadStateEstimate AutoPilot::getPredictedStateEstimate(
    const ros::Time& time) const
{
  return state_predictor_.predictState(time);
}

void AutoPilot::publishControlCommand(
    const quadrotor_common::ControlCommand& control_cmd)
{
  if (control_cmd.control_mode == quadrotor_common::ControlMode::NONE)
  {
    ROS_ERROR("[%s] Control mode is NONE, will not publish ControlCommand",
              pnh_.getNamespace().c_str());
  }
  else
  {
    quadrotor_msgs::ControlCommand control_cmd_msg;

    control_cmd_msg = control_cmd.toRosMessage();

    control_command_pub_.publish(control_cmd_msg);
    state_predictor_.pushCommandToQueue(control_cmd);
    // Save applied thrust to initialize propeller ramping down if necessary
    initial_drop_thrust_ = control_cmd.collective_thrust;
  }
}

void AutoPilot::publishAutopilotFeedback(
    const States& autopilot_state, const ros::Duration& control_command_delay,
    const ros::Duration& control_computation_time,
    const ros::Duration& trajectory_execution_left_duration,
    const int trajectories_left_in_queues,
    const quadrotor_msgs::LowLevelFeedback& low_level_feedback,
    const quadrotor_common::TrajectoryPoint& reference_state,
    const quadrotor_common::QuadStateEstimate& state_estimate)
{
  quadrotor_msgs::AutopilotFeedback fb_msg;

  fb_msg.header.stamp = ros::Time::now();
  switch (autopilot_state)
  {
    case States::OFF:
      fb_msg.autopilot_state = fb_msg.OFF;
      break;
    case States::START:
      fb_msg.autopilot_state = fb_msg.START;
      break;
    case States::HOVER:
      fb_msg.autopilot_state = fb_msg.HOVER;
      break;
    case States::LAND:
      fb_msg.autopilot_state = fb_msg.LAND;
      break;
    case States::EMERGENCY_LAND:
      fb_msg.autopilot_state = fb_msg.EMERGENCY_LAND;
      break;
    case States::BREAKING:
      fb_msg.autopilot_state = fb_msg.BREAKING;
      break;
    case States::GO_TO_POSE:
      fb_msg.autopilot_state = fb_msg.GO_TO_POSE;
      break;
    case States::VELOCITY_CONTROL:
      fb_msg.autopilot_state = fb_msg.VELOCITY_CONTROL;
      break;
    case States::REFERENCE_CONTROL:
      fb_msg.autopilot_state = fb_msg.REFERENCE_CONTROL;
      break;
    case States::TRAJECTORY_CONTROL:
      fb_msg.autopilot_state = fb_msg.TRAJECTORY_CONTROL;
      break;
    case States::COMMAND_FEEDTHROUGH:
      fb_msg.autopilot_state = fb_msg.COMMAND_FEEDTHROUGH;
      break;
    case States::RC_MANUAL:
      fb_msg.autopilot_state = fb_msg.RC_MANUAL;
      break;
  }
  fb_msg.control_command_delay = control_command_delay;
  fb_msg.control_computation_time = control_computation_time;
  fb_msg.trajectory_execution_left_duration =
      trajectory_execution_left_duration;
  fb_msg.low_level_feedback = low_level_feedback;
  fb_msg.reference_state = reference_state.toRosMessage();
  fb_msg.state_estimate = state_estimate.toRosMessage();

  autopilot_feedback_pub_.publish(fb_msg);

  time_last_autopilot_feedback_published_ = ros::Time::now();
}

bool AutoPilot::loadParameters()
{
#define GET_PARAM(name) \
if (!quadrotor_common::getParam(#name, name ## _, pnh_)) \
  return false

  GET_PARAM(state_estimate_timeout);
  GET_PARAM(velocity_estimate_in_world_frame);
  GET_PARAM(control_command_delay);
  GET_PARAM(start_land_velocity);
  GET_PARAM(start_idle_duration);
  GET_PARAM(idle_thrust);
  GET_PARAM(optitrack_start_height);
  GET_PARAM(optitrack_start_land_timeout);
  GET_PARAM(optitrack_land_drop_height);
  GET_PARAM(propeller_ramp_down_timeout);
  GET_PARAM(breaking_velocity_threshold);
  GET_PARAM(breaking_timeout);
  GET_PARAM(go_to_pose_max_velocity);
  GET_PARAM(go_to_pose_max_normalized_thrust);
  GET_PARAM(go_to_pose_max_roll_pitch_rate);
  GET_PARAM(velocity_command_input_timeout);
  GET_PARAM(tau_velocity_command);
  GET_PARAM(reference_state_input_timeout);
  GET_PARAM(emergency_land_duration);
  GET_PARAM(emergency_land_thrust);

  if (!base_controller_params_.loadParameters(pnh_))
  {
    return false;
  }

  return true;

#undef GET_PARAM
}

} // namespace autopilot
