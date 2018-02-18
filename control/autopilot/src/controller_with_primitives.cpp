#include "copilot/controller_with_primitives.h"
#include "estimator_common/estimator_id.h"

namespace copilot
{

ControllerWithPrimitives::ControllerWithPrimitives(const ros::NodeHandle & nh, const ros::NodeHandle & pnh) :
    FlightControllerBase(nh, pnh), settings_(), controller_(), controller_parameters_(), vision_pipeline_helper_(), height_estimate_(), manual_desired_velocity_command_(), state_machine_before_emg_landing_(
        states::INVALID), state_before_rc_manual_flight_(states::INVALID), desired_state_after_breaking_(
        states::INVALID), first_time_in_new_mode_(true), time_of_switch_to_current_state_(), optitrack_start_desired_state_(), optitrack_land_desired_state_(), vision_land_desired_state_(), drop_thrust_(
        0.0), most_recent_flight_controller_command_stamp_(), time_first_height_measurement_during_start_(), received_first_height_measurement_during_start_(
        false), initialization_height_(), land_using_vision_(false), time_to_ramp_down_(false), allow_open_loop_start_(
        false), time_last_manual_input_handled_()
{

  if (!loadParameters())
    ros::shutdown();

  state_machine_ = states::OFF;
  // Subscribers
  manual_desired_velocity_sub_ = nh_.subscribe(node_name_ + "/manual_desired_velocity", 1,
                                               &ControllerWithPrimitives::manualDesiredVelocityCallback, this);
  start_sub_ = nh_.subscribe(node_name_ + "/start", 1, &ControllerWithPrimitives::startCallback, this);
  land_sub_ = nh_.subscribe(node_name_ + "/land", 1, &ControllerWithPrimitives::landCallback, this);
  off_sub_ = nh_.subscribe(node_name_ + "/off", 1, &ControllerWithPrimitives::offCallback, this);
  switch_rc_manual_mode_sub_ = nh_.subscribe(node_name_ + "/switch_rc_manual_mode", 1,
                                             &ControllerWithPrimitives::switchRCManualModeCallback, this);
  height_estimate_sub_ = nh_.subscribe("height_estimate", 1, &ControllerWithPrimitives::heightEstimateCallback, this);
  reload_param_ = nh_.subscribe(node_name_ + "/reload_param", 1, &ControllerWithPrimitives::reloadParamCallback, this);
  flightcontroller_fdb_sub_ = nh_.subscribe("flight_controller/feedback", 1,
                                            &ControllerWithPrimitives::flightcontrollerFdbCallback, this);
  desired_state_pub_ = nh_.advertise<quad_msgs::QuadDesiredState>("desired_state", 1);

  vision_pipeline_helper_.reset();
}

ControllerWithPrimitives::~ControllerWithPrimitives()
{

}

void ControllerWithPrimitives::flightcontrollerFdbCallback(
    const quad_msgs::ControllerFeedbackConstPtr& msg)
{
  if (!received_flight_controller_feedback_)
  {
    received_flight_controller_feedback_ = true;
  }

  flightcontroller_feedback_msg_ = *msg;
  flightcontroller_feedback_msg_.header.stamp = ros::Time::now();  // TODO: Is this even necessary?
}

void ControllerWithPrimitives::reloadParamCallback(const std_msgs::EmptyConstPtr)
{
  controller_parameters_.loadParameters(pnh_);
}

void ControllerWithPrimitives::mainloop(const ros::TimerEvent& time)
{
  // Handle lost tracking
  if (!stateEstimateAvailable() && state_machine_ != states::INVALID && state_machine_ != states::OFF
      && state_machine_ != states::VISION_START && state_machine_ != states::VISION_LAND
      && state_machine_ != states::EMERGENCYLAND && state_machine_ != states::WAIT_FOR_USER_CONTROLLER
      && state_machine_ != states::FEEDTHROUGH && state_machine_ != states::RC_MANUAL)
  {
    if (isHeightEstimateAvailable())
    {
      ROS_WARN("[%s] Only height estimate available, will land with teraranger now", pnh_.getNamespace().c_str());
      setNewState(states::VISION_LAND);
    }
    else
    {
      if ((state_machine_ == states::OPTITRACK_LAND && time_to_ramp_down_)
          || (state_machine_ == states::VISION_LAND && time_to_ramp_down_))
      {
        // Not go into emergency landing anymore since propellers are ramping down already
      }
      else
      {
        ROS_ERROR("[%s] No state estimate available, will emergency land now", pnh_.getNamespace().c_str());
        setNewState(states::EMERGENCYLAND);
      }
    }
  }

  ControlCommand command;

  ros::Time wall_time_now = ros::Time::now();
  ros::Time command_execution_time = wall_time_now + ros::Duration(control_command_delay_);

  QuadState predicted_state;

  if (state_machine_ == states::INVALID)
  {
    // Nothing to be done here
  }

  if (state_machine_ == states::OFF)
  {
    command = controller_.off();
    desired_state_ = QuadDesiredState();
  }

  if (state_machine_ == states::OPTITRACK_START)
  {
    predicted_state = getStateEstimate(command_execution_time);
    command = optitrack_start(predicted_state);
  }

  if (state_machine_ == states::VISION_START)
  {
    command = vision_start();
  }

  if (state_machine_ == states::HOVER)
  {
    handleManualDesiredVelocityCommand();
    predicted_state = getStateEstimate(command_execution_time);
    command = hover(predicted_state);
  }

  if (state_machine_ == states::OPTITRACK_LAND)
  {
    predicted_state = getStateEstimate(command_execution_time);
    command = optitrack_land(predicted_state);
  }

  if (state_machine_ == states::VISION_LAND)
  {
    if (stateEstimateAvailable())
    {
      predicted_state = getStateEstimate(command_execution_time);
    }
    command = vision_land(predicted_state);
  }

  if (state_machine_ == states::WAIT_FOR_USER_CONTROLLER)
  {
    predicted_state = getStateEstimate(command_execution_time);
    command = waitForUserControllerInput(predicted_state);
  }

  if (state_machine_ == states::FEEDTHROUGH)
  {
    // Nothing to be done here
  }

  if (state_machine_ == states::BREAKING)
  {
    predicted_state = getStateEstimate(command_execution_time);
    command = breakVelocity(predicted_state);
  }

  if (state_machine_ == states::RC_MANUAL)
  {
    // Nothing to be done here
  }

  if (state_machine_ == states::EMERGENCYLAND)
  {
    command = emergencyLand();
  }

  if (state_machine_ != states::INVALID && state_machine_ != states::FEEDTHROUGH && state_machine_ != states::RC_MANUAL)
  {
    command.timestamp = wall_time_now;
    command.execution_time = command_execution_time; // predicted_state.timestamp = wall_time_now + ros::Duration(control_command_delay_);
    publishCommand(command);
  }
  publishControllerFeedback(command_execution_time, desired_state_, predicted_state);
}

ControlCommand ControllerWithPrimitives::optitrack_start(const QuadState & predicted_state)
{
  ControlCommand command;

  if (first_time_in_new_mode_)
  {
    first_time_in_new_mode_ = false;
    optitrack_start_desired_state_.position = predicted_state.position;
    optitrack_start_desired_state_.velocity = Eigen::Vector3d::Zero();
    optitrack_start_desired_state_.acceleration = Eigen::Vector3d::Zero();
    optitrack_start_desired_state_.yaw = quaternionToEulerAnglesZYX(predicted_state.orientation).z();
    if (predicted_state.position.z() >= settings_.optitrack_land_drop_height_)
    {
      setNewState(states::HOVER);
    }
  }

  if (timeInCurrentState() > settings_.optitrack_start_land_timeout_
      || optitrack_start_desired_state_.position.z() >= settings_.start_height_)
  {
    setNewState(states::HOVER);
  }
  else
  {
    if (timeInCurrentState() < settings_.start_idle_duration_)
    {
      command.mode = ControllerMode::ANGLERATE;
      command.off = false;
      command.bodyrates = Eigen::Vector3d::Zero();
      command.thrust = settings_.idle_thrust_;
      return command;
    }
    else
    {
      optitrack_start_desired_state_.position.z() += settings_.start_land_velocity_ / controller_frequency_;
      optitrack_start_desired_state_.velocity.z() = settings_.start_land_velocity_;
    }
  }

  desired_state_.timestamp = ros::Time::now();
  desired_state_ = optitrack_start_desired_state_;
  command = controller_.run(predicted_state, desired_state_, controller_parameters_);

  return command;
}

ControlCommand ControllerWithPrimitives::vision_start()
{
  ControlCommand command;

  if (first_time_in_new_mode_)
  {
    first_time_in_new_mode_ = false;
    received_first_height_measurement_during_start_ = false;
    vision_pipeline_helper_.reset();
    ROS_INFO("[%s] start ideling", pnh_.getNamespace().c_str());
  }

  if (!received_first_height_measurement_during_start_)
  {
    if (timeInCurrentState() < settings_.start_idle_duration_)
    {
      command.mode = ControllerMode::ANGLERATE;
      command.off = false;
      command.bodyrates = Eigen::Vector3d::Zero();
      command.thrust = settings_.idle_thrust_;
      return command;
    }
    else
    {
      if (timeInCurrentState() < settings_.start_idle_duration_ + settings_.open_loop_takeoff_timeout_)
      {
        if (height_estimate_.position.z > settings_.vision_land_drop_height_ && isHeightEstimateAvailable())
        {
          ROS_INFO("[%s] received first height estimate, start to height control", pnh_.getNamespace().c_str());
          received_first_height_measurement_during_start_ = true;
          // store height measruement
          initialization_height_ = height_estimate_.position.z + 0.1;
          // store current time stamp
          time_first_height_measurement_during_start_ = ros::Time::now();
          // Trigger Vision Pipeline
          ROS_INFO("[%s] initialize vision pipeline", pnh_.getNamespace().c_str());
          vision_pipeline_helper_.start();
        }
      }
      else
      {
        ROS_INFO("[%s] start up timed out, going to vision land", pnh_.getNamespace().c_str());
        setNewState(states::VISION_LAND);
      }
      command.mode = ControllerMode::ANGLE;
      command.off = false;
      command.thrust = settings_.open_loop_takeoff_thrust_;
      return command;
    }
  }
  else
  {
    // height control and wait for SVO
    if (vision_pipeline_helper_.IsOk())
    {
      ROS_INFO("[%s] vision pipeline initialized, going to hover", pnh_.getNamespace().c_str());
      setNewState(states::HOVER);
    }
    else if (ros::Time::now() - time_first_height_measurement_during_start_
        > ros::Duration(settings_.vision_init_timeout_))
    {
      ROS_INFO("[%s] init vision timeout, going to vision land", pnh_.getNamespace().c_str());
      setNewState(states::VISION_LAND);
    }
    large_angle_controller::ControllerParameters config = controller_parameters_;
    config.use_rate_mode = false;
    config.kpxy = 0.0;
    config.kdxy = 0.0;
    config.kyaw = 0.0;

    desired_state_ = quad_common::QuadDesiredState();
    desired_state_.position.z() = initialization_height_;

    command = controller_.run(height_estimate_, desired_state_, config);
  }

  return command;
}

ControlCommand ControllerWithPrimitives::hover(const QuadState & predicted_state)
{
  ControlCommand command;

  if (first_time_in_new_mode_)
  {
    first_time_in_new_mode_ = false;
    desired_state_.timestamp = ros::Time::now();
    desired_state_.position = predicted_state.position;
    desired_state_.velocity = Eigen::Vector3d::Zero();
    desired_state_.acceleration = Eigen::Vector3d::Zero();
    desired_state_.yaw = quaternionToEulerAnglesZYX(predicted_state.orientation).z();
    time_last_manual_input_handled_ = ros::Time::now();
  }

  command = controller_.run(predicted_state, desired_state_, controller_parameters_);

  return command;
}

ControlCommand ControllerWithPrimitives::optitrack_land(const QuadState & predicted_state)
{
  ControlCommand command;

  if (first_time_in_new_mode_)
  {
    first_time_in_new_mode_ = false;
    optitrack_land_desired_state_.timestamp = ros::Time::now();
    optitrack_land_desired_state_.position = predicted_state.position;
    optitrack_land_desired_state_.velocity = Eigen::Vector3d::Zero();
    optitrack_land_desired_state_.acceleration = Eigen::Vector3d::Zero();
    optitrack_land_desired_state_.yaw = quaternionToEulerAnglesZYX(predicted_state.orientation).z();
    // initialize drop thrust for the case quad is already below drop height
    time_to_ramp_down_ = false;
    drop_thrust_ = 9.81;
  }

  optitrack_land_desired_state_.position.z() = fmax(
      0.0, optitrack_land_desired_state_.position.z() - settings_.start_land_velocity_ / controller_frequency_);
  optitrack_land_desired_state_.velocity.z() = -settings_.start_land_velocity_;

  desired_state_.timestamp = ros::Time::now();
  desired_state_ = optitrack_land_desired_state_;
  command = controller_.run(predicted_state, desired_state_, controller_parameters_);

  time_to_ramp_down_ = predicted_state.position.z() < settings_.optitrack_land_drop_height_
      || timeInCurrentState() > settings_.optitrack_start_land_timeout_;

  if (time_to_ramp_down_)
  {
    // we are low enough -> ramp down the thrust
    // we timed out on landing -> ramp down the thrust
    ROS_INFO_THROTTLE(2, "[%s] ramping propeller down", pnh_.getNamespace().c_str());
    drop_thrust_ -= 9.81 / (controller_frequency_ * settings_.propeller_ramp_down_timeout_);
    command.thrust = drop_thrust_;
  }
  else
  {
    // ramp down from the last given command
    drop_thrust_ = command.thrust;
  }

  if (command.thrust <= 0.0)
  {
    setNewState(states::OFF);
    command.zero();
  }

  return command;
}

ControlCommand ControllerWithPrimitives::vision_land(const QuadState & predicted_state)
{
  ControlCommand command;

  if (first_time_in_new_mode_)
  {
    // distinguish whether state estimate or height estimate is available
    if (stateEstimateAvailable() || isHeightEstimateAvailable())
    {
      if (getEstimatorID() == estimator_common::estimator_id::MSF)
      {
        ROS_INFO("[%s] start vision landing based on MSF", pnh_.getNamespace().c_str());
        land_using_vision_ = true;
        vision_land_desired_state_.timestamp = ros::Time::now();
        vision_land_desired_state_.position = predicted_state.position;
        vision_land_desired_state_.velocity = Eigen::Vector3d::Zero();
        vision_land_desired_state_.acceleration = Eigen::Vector3d::Zero();
        vision_land_desired_state_.yaw = quaternionToEulerAnglesZYX(predicted_state.orientation).z();
        // initialize drop thrust for the case quad is already below drop height
        drop_thrust_ = 9.81;
      }
      else
      {
        ROS_INFO("[%s] start vision landing based on teraranger", pnh_.getNamespace().c_str());
        land_using_vision_ = false;
        vision_land_desired_state_.timestamp = ros::Time::now();
        vision_land_desired_state_.position = geometryToEigen(height_estimate_.position);
        vision_land_desired_state_.velocity = Eigen::Vector3d::Zero();
        vision_land_desired_state_.acceleration = Eigen::Vector3d::Zero();
        vision_land_desired_state_.yaw = 0.0;
        // initialize drop thrust for the case quad is already below drop height
        drop_thrust_ = 9.81;
      }

      time_to_ramp_down_ = false;
      if (isHeightEstimateAvailable())
      {
        time_to_ramp_down_ = height_estimate_.position.z < settings_.vision_land_drop_height_;
        if (time_to_ramp_down_)
        {
          ROS_INFO("[%s] already below height threshold, going to immediate ramp down", pnh_.getNamespace().c_str());
        }
      }
    }

    first_time_in_new_mode_ = false;
  }

  vision_land_desired_state_.position.z() -= settings_.start_land_velocity_ / controller_frequency_;
  vision_land_desired_state_.velocity.z() = -settings_.start_land_velocity_;

  if (!time_to_ramp_down_)
  {
    if (stateEstimateAvailable() && land_using_vision_)
    {
      desired_state_.timestamp = ros::Time::now();
      desired_state_ = vision_land_desired_state_;
      command = controller_.run(predicted_state, desired_state_, controller_parameters_);

      // check whether it is time to ramp down or not
      if (isHeightEstimateAvailable())
      {
        time_to_ramp_down_ = height_estimate_.position.z < settings_.vision_land_drop_height_;
      }
    }
    else if (isHeightEstimateAvailable())
    {
      if (land_using_vision_)
      {
        ROS_INFO("[%s] switched from msf landing to teraranger landing", pnh_.getNamespace().c_str());
        // Store the state where the switch from vision-based to Teraranger-based landing happens
        land_using_vision_ = false;
        vision_land_desired_state_.timestamp = ros::Time::now();
        vision_land_desired_state_.position = geometryToEigen(height_estimate_.position);
        vision_land_desired_state_.velocity = Eigen::Vector3d::Zero();
        vision_land_desired_state_.acceleration = Eigen::Vector3d::Zero();
        vision_land_desired_state_.yaw = 0.0;
      }

      large_angle_controller::ControllerParameters config = controller_parameters_;
      config.use_rate_mode = false;
      config.kpxy = 0.0;
      config.kdxy = 0.0;
      config.kyaw = 0.0;

      desired_state_.timestamp = ros::Time::now();
      desired_state_ = vision_land_desired_state_;

      command = controller_.run(height_estimate_, desired_state_, config);

      // check whether is time to ramp down or not
      time_to_ramp_down_ = height_estimate_.position.z < settings_.vision_land_drop_height_;
    }
    else
    {
      ROS_INFO("[%s] no state or height estimate during vision landing, going to emergency land",
               pnh_.getNamespace().c_str());
      setNewState(states::EMERGENCYLAND);
      command.mode = ControllerMode::ANGLE;
      command.off = false;
      command.thrust = settings_.emergency_land_thrust_;
    }

    drop_thrust_ = command.thrust;
  }
  else // ramp down (we are low enough or timed out on landing)
  {
    ROS_INFO_THROTTLE(2, "[%s] ramping propeller down", pnh_.getNamespace().c_str());
    drop_thrust_ -= 9.81 / (controller_frequency_ * settings_.propeller_ramp_down_timeout_);
    command.mode = ControllerMode::ANGLE;
    command.off = false;
    command.thrust = drop_thrust_;
  }

  if (command.thrust <= 0.0)
  {
    vision_pipeline_helper_.reset();
    setNewState(states::OFF);
    command.zero();
  }

  return command;
}

ControlCommand ControllerWithPrimitives::emergencyLand()
{
  ControlCommand command;
  if (first_time_in_new_mode_)
  {
    first_time_in_new_mode_ = false;
  }

  command.mode = ControllerMode::ANGLE;
  command.thrust = settings_.emergency_land_thrust_;
  command.off = false;

  if (timeInCurrentState() > settings_.emergency_land_duration_)
  {
    command.zero();
    setNewState(states::OFF);
    vision_pipeline_helper_.reset();
  }

  if (stateEstimateAvailable() && state_machine_before_emg_landing_ != states::INVALID)
  {
    if (getEstimatorID() == estimator_common::estimator_id::OPTITRACK
        || getEstimatorID() == estimator_common::estimator_id::SIMULATOR)
    {
      // regained tracking, reset to previous state
      ROS_ERROR("[%s] Regained tracking", pnh_.getNamespace().c_str());
      setNewState(state_machine_before_emg_landing_);
    }
  }

  return command;
}

ControlCommand ControllerWithPrimitives::waitForUserControllerInput(
    const QuadState& predicted_state)
{
  // Prepare next control command
  ControlCommand command;
  switch (state_machine_before_waiting_for_user_controller_)
  {
    case states::OFF:
      command = controller_.off();
      break;

    case states::HOVER:
      command = hover(predicted_state);
      break;
  }

  if (first_time_in_new_mode_)
  {
    first_time_in_new_mode_ = false;
  }

  // Check if flightcontroller is alive
  if (!received_flight_controller_feedback_ ||
      (ros::Time::now() - flightcontroller_feedback_msg_.header.stamp)
              .toSec() >= settings_.user_command_timeout_)
  {
    ROS_ERROR("[%s] Not receiving flightcontroller feedback or message is too "
              "old. Will not enable feedthrough!",
              node_name_.c_str());
    setNewState(state_machine_before_waiting_for_user_controller_);
    return command;
  }

  // Check timeout
  if (timeInCurrentState() > settings_.user_command_timeout_)
  {
    ROS_ERROR("[%s] Timeout reached when trying to enable feedthrough mode.",
              ros::this_node::getName().c_str());
    setNewState(state_machine_before_waiting_for_user_controller_);
    return command;
  }

  // Publish current desired state to flightcontroller
  quad_msgs::QuadDesiredState desired_state_msg;
  desired_state_msg = desired_state_.toRosMessage();
  desired_state_msg.header.stamp = ros::Time::now();
  desired_state_pub_.publish(desired_state_msg);

  // Check if flightcontroller accepted the desired state
  // For that compare desired state's individual data entries
  const double kDiffNormThreshold = 1e-10;
  quad_common::QuadDesiredState fc_desired_state =
      flightcontroller_feedback_msg_.desired_state;
  if ((fc_desired_state.position - desired_state_.position).norm() <=
          kDiffNormThreshold &&
      (fc_desired_state.velocity - desired_state_.velocity).norm() <=
          kDiffNormThreshold &&
      (fc_desired_state.acceleration - desired_state_.acceleration).norm() <=
          kDiffNormThreshold &&
      (fc_desired_state.jerk - desired_state_.jerk).norm() <=
          kDiffNormThreshold)
  {
    // Reset copilot's desired state
    desired_state_.timestamp = ros::Time::now();
    desired_state_.position = Eigen::Vector3d::Zero();
    desired_state_.velocity = Eigen::Vector3d::Zero();
    desired_state_.acceleration = Eigen::Vector3d::Zero();
    desired_state_.yaw = 0.0;

    ROS_INFO("[%s] Feedthrough mode succesfully enabled", node_name_.c_str());

    // State machine transition
    setNewState(states::FEEDTHROUGH);
  }

  // return appropriate ControlCommand dependnig on the previous state
  return command;
}

ControlCommand ControllerWithPrimitives::breakVelocity(const QuadState & predicted_state)
{
  ControlCommand command;

  if (first_time_in_new_mode_)
  {
    first_time_in_new_mode_ = false;
    desired_state_ = QuadDesiredState();
    desired_state_.position = predicted_state.position;
    desired_state_.velocity = predicted_state.velocity;
    desired_state_.yaw = quaternionToEulerAnglesZYX(predicted_state.orientation).z();
  }

  if (predicted_state.velocity.norm() < settings_.breaking_velocity_threshold_
      || timeInCurrentState() > settings_.breaking_timeout_)
  {
    double current_yaw = desired_state_.yaw;
    desired_state_ = QuadDesiredState();
    desired_state_.position = predicted_state.position;
    desired_state_.yaw = current_yaw;
    setNewState(desired_state_after_breaking_);
  }

  command = controller_.run(predicted_state, desired_state_, controller_parameters_);

  return command;
}

void ControllerWithPrimitives::handleManualDesiredVelocityCommand()
{
  // TODO: This if-check is unnecessary. Function only called from hover state!
  if (state_machine_ == states::HOVER)
  {
    double dt = (ros::Time::now() - time_last_manual_input_handled_).toSec();
    if ((ros::Time::now() - manual_desired_velocity_command_.header.stamp)
        > ros::Duration(settings_.manual_velocity_input_timeout_))
    {
      manual_desired_velocity_command_.twist.linear.x = 0.0;
      manual_desired_velocity_command_.twist.linear.y = 0.0;
      manual_desired_velocity_command_.twist.linear.z = 0.0;
      manual_desired_velocity_command_.twist.angular.z = 0.0;
    }

    double alpha_velocity = 1 - exp(-dt / settings_.tau_manual_velocity_);

    Eigen::Vector3d commanded_velocity = geometryToEigen(manual_desired_velocity_command_.twist.linear);
    desired_state_.velocity = (1.0 - alpha_velocity) * desired_state_.velocity + alpha_velocity * commanded_velocity;
    if (desired_state_.velocity.norm() < 0.01 && commanded_velocity.norm() < 0.01)
    {
      desired_state_.velocity = Eigen::Vector3d::Zero();
    }
    desired_state_.position += desired_state_.velocity * dt;

    desired_state_.yaw += manual_desired_velocity_command_.twist.angular.z * dt;
    desired_state_.yaw = wrapMinusPiToPi(desired_state_.yaw);
    desired_state_.yaw_rate = manual_desired_velocity_command_.twist.angular.z;

    time_last_manual_input_handled_ = ros::Time::now();
  }
}

void ControllerWithPrimitives::manualDesiredVelocityCallback(const geometry_msgs::TwistStampedConstPtr msg)
{
  manual_desired_velocity_command_ = *msg;
}

void ControllerWithPrimitives::startCallback(const std_msgs::EmptyConstPtr msg)
{
  if (state_machine_ == states::OFF || state_machine_ == states::OPTITRACK_LAND
      || state_machine_ == states::VISION_LAND)
  {
    ROS_INFO("[%s] START command received", pnh_.getNamespace().c_str());

    if (stateEstimateAvailable())
    {
      if (getEstimatorID() == estimator_common::estimator_id::OPTITRACK
          || getEstimatorID() == estimator_common::estimator_id::SIMULATOR)
      {
        ROS_INFO("[%s] Taking off based on OptiTrack", pnh_.getNamespace().c_str());
        setNewState(states::OPTITRACK_START);
      }
      else if (getEstimatorID() == estimator_common::estimator_id::MSF)
      {
        ROS_INFO("[%s] Vision based state estimate available, switch to hover", pnh_.getNamespace().c_str());
        setNewState(states::HOVER);
      }
      else
      {
        if (allow_open_loop_start_)
        {
          ROS_INFO("[%s] Taking off open loop", pnh_.getNamespace().c_str());
          setNewState(states::VISION_START);
        }
        else
        {
          ROS_INFO("[%s] Wanted to start open loop but it is not enabled, will not start", pnh_.getNamespace().c_str());
        }
      }
    }
    else
    {
      if (allow_open_loop_start_)
      {
        ROS_INFO("[%s] Taking off open loop", pnh_.getNamespace().c_str());
        setNewState(states::VISION_START);
      }
      else
      {
        ROS_INFO("[%s] Wanted to start open loop but it is not enabled, will not start", pnh_.getNamespace().c_str());
      }
    }
  }
}

void ControllerWithPrimitives::offCallback(const std_msgs::EmptyConstPtr msg)
{
  if (state_machine_ != states::OFF)
  {
    ROS_INFO("[%s] OFF command received", pnh_.getNamespace().c_str());
    setNewState(states::OFF);
    // Allow user to take over manually and land the vehicle, then off the controller and disable the RC without
    // the vehicle going back to hover
    state_before_rc_manual_flight_ = states::OFF;
  }
}

void ControllerWithPrimitives::switchRCManualModeCallback(const std_msgs::BoolConstPtr msg)
{
  if (msg->data && state_machine_ != states::RC_MANUAL)
  {
    state_before_rc_manual_flight_ = state_machine_;
    setNewState(states::RC_MANUAL);
  }
  else if (!msg->data)
  {
    if (stateEstimateAvailable())
    {
      if (state_before_rc_manual_flight_ == states::INVALID || state_before_rc_manual_flight_ == states::OFF)
      {
        setNewState(state_before_rc_manual_flight_);
      }
      else
      {
        setNewState(states::HOVER);
      }
    }
    else
    {
      setNewState(states::OFF);
    }
  }
}

void ControllerWithPrimitives::landCallback(const std_msgs::EmptyConstPtr msg)
{
  if (state_machine_ != states::OPTITRACK_LAND && state_machine_ != states::VISION_LAND
      && state_machine_ != states::EMERGENCYLAND
      && !(state_machine_ == states::BREAKING
          && (desired_state_after_breaking_ == states::OPTITRACK_LAND
              || desired_state_after_breaking_ == states::VISION_LAND)))
  {
    ROS_INFO("[%s] LAND command received", pnh_.getNamespace().c_str());

    if (stateEstimateAvailable())
    {
      if (getEstimatorID() == estimator_common::estimator_id::OPTITRACK
          || getEstimatorID() == estimator_common::estimator_id::SIMULATOR)
      {
        setNewState(states::OPTITRACK_LAND);
      }
      else if (getEstimatorID() == estimator_common::estimator_id::MSF
          || getEstimatorID() == estimator_common::estimator_id::RANGER)
      {
        setNewState(states::VISION_LAND);
      }
      else
      {
        setNewState(states::EMERGENCYLAND);
      }
    }
    else
    {
      setNewState(states::EMERGENCYLAND);
    }
  }
}

void ControllerWithPrimitives::heightEstimateCallback(const quad_msgs::QuadStateEstimateConstPtr msg)
{
  height_estimate_ = *msg;
}

bool ControllerWithPrimitives::isHeightEstimateAvailable()
{
  if (ros::Time::now() - height_estimate_.header.stamp < ros::Duration(0.3))
  {
    return true;
  }
  return false;
}

bool ControllerWithPrimitives::loadParameters()
{
  if (!getParam("user_command_timeout", settings_.user_command_timeout_, pnh_))
    return false;
  if (!getParam("start_land_velocity", settings_.start_land_velocity_, pnh_))
    return false;
  if (!getParam("start_height", settings_.start_height_, pnh_))
    return false;
  if (!getParam("propeller_ramp_down_timeout", settings_.propeller_ramp_down_timeout_, pnh_))
    return false;
  if (!getParam("idle_thrust", settings_.idle_thrust_, pnh_))
    return false;
  if (!getParam("start_idle_duration", settings_.start_idle_duration_, pnh_))
    return false;
  if (!getParam("optitrack_start_land_timeout", settings_.optitrack_start_land_timeout_, pnh_))
    return false;
  if (!getParam("optitrack_land_drop_height", settings_.optitrack_land_drop_height_, pnh_))
    return false;
  if (!getParam("open_loop_takeoff_timeout", settings_.open_loop_takeoff_timeout_, pnh_))
    return false;
  if (!getParam("vision_init_timeout", settings_.vision_init_timeout_, pnh_))
    return false;
  if (!getParam("open_loop_takeoff_thrust", settings_.open_loop_takeoff_thrust_, pnh_))
    return false;
  if (!getParam("vision_land_drop_height", settings_.vision_land_drop_height_, pnh_))
    return false;
  if (!getParam("emergency_land_duration", settings_.emergency_land_duration_, pnh_))
    return false;
  if (!getParam("emergency_land_thrust", settings_.emergency_land_thrust_, pnh_))
    return false;
  if (!getParam("breaking_velocity_threshold", settings_.breaking_velocity_threshold_, pnh_))
    return false;
  if (!getParam("breaking_timeout", settings_.breaking_timeout_, pnh_))
    return false;
  if (!getParam("manual_velocity_input_timeout", settings_.manual_velocity_input_timeout_, pnh_))
    return false;
  if (!getParam("tau_manual_velocity", settings_.tau_manual_velocity_, pnh_))
    return false;
  
  // Dangerzone
  getParam("allow_open_loop_start", allow_open_loop_start_, false, pnh_);
  getParam("feedthrough_safety_checks", feedthrough_safety_checks_, true, pnh_);

  // load the parameters of the controller
  if (!controller_parameters_.loadParameters(pnh_))
    return false;

  return true;
}

double ControllerWithPrimitives::timeInCurrentState() const
{
  return (ros::Time::now() - time_of_switch_to_current_state_).toSec();
}

bool ControllerWithPrimitives::setNewState(const int desired_new_state)
{
  //  const int INVALID = -1;
  //  const int OFF = 0;
  //  const int OPTITRACK_START = 1;
  //  const int VISION_START = 2;
  //  const int HOVER = 3;
  //  const int OPTITRACK_LAND = 4;
  //  const int VISION_LAND = 5;
  //  const int EMERGENCYLAND = 6;
  //  const int WAIT_FOR_USER_CONTROLLER = 7;
  //  const int FEEDTHROUGH = 8;
  //  const int BREAKING = 9;
  //  const int RC_MANUAL = 10;

  int new_state = states::INVALID;

  if (desired_new_state == states::OFF)
  {
    // Not Available
  }

  // switch to OFF
  if (desired_new_state == states::OFF)
  {
    new_state = states::OFF;
    desired_state_ = QuadDesiredState();
  }

  // switch to OPTITRACK_START
  if (desired_new_state == states::OPTITRACK_START)
  {
    if (getEstimatorID() == estimator_common::estimator_id::OPTITRACK
        || getEstimatorID() == estimator_common::estimator_id::SIMULATOR)
    {
      if (state_machine_ == states::OFF)
      {
        new_state = states::OPTITRACK_START;
      }
      if (state_machine_ == states::EMERGENCYLAND && state_machine_before_emg_landing_ == states::OPTITRACK_START)
      {
        new_state = states::OPTITRACK_START;
      }
    }
  }

  // switch to VISION_START
  if (desired_new_state == states::VISION_START)
  {
    if (allow_open_loop_start_)
    {
      if (state_machine_ == states::OFF)
      {
        if (getEstimatorID() != estimator_common::estimator_id::OPTITRACK
            && getEstimatorID() != estimator_common::estimator_id::SIMULATOR
            && getEstimatorID() != estimator_common::estimator_id::MSF)
        {
          new_state = states::VISION_START;
        }
      }
    }
  }

  // switch to HOVER
  if (desired_new_state == states::HOVER)
  {
    if (state_machine_ == states::OFF)
    {
      if (getEstimatorID() == estimator_common::estimator_id::MSF)
      {
        new_state = states::HOVER;
      }
    }
    if (state_machine_ == states::WAIT_FOR_USER_CONTROLLER || state_machine_ == states::BREAKING)
    {
      new_state = states::HOVER;
    }
    if (state_machine_ == states::OPTITRACK_START || state_machine_ == states::VISION_START
        || state_machine_ == states::OPTITRACK_LAND || state_machine_ == states::VISION_LAND
        || state_machine_ == states::FEEDTHROUGH)
    {
      new_state = states::BREAKING;
      desired_state_after_breaking_ = states::HOVER;
    }
    if (state_machine_ == states::EMERGENCYLAND && state_machine_before_emg_landing_ == states::HOVER)
    {
      if (getEstimatorID() == estimator_common::estimator_id::OPTITRACK
          || getEstimatorID() == estimator_common::estimator_id::SIMULATOR)
      {
        new_state = states::HOVER;
      }
    }
    if (state_machine_ == states::RC_MANUAL)
    {
      new_state = states::BREAKING;
      desired_state_after_breaking_ = states::HOVER;
    }
  }

  // switch to OPTITRACK_LAND
  if (desired_new_state == states::OPTITRACK_LAND)
  {
    if (state_machine_ == states::BREAKING)
    {
      if ((getEstimatorID() == estimator_common::estimator_id::OPTITRACK
          || getEstimatorID() == estimator_common::estimator_id::SIMULATOR))
      {
        new_state = states::OPTITRACK_LAND;
      }
    }
    if (state_machine_ == states::OPTITRACK_START || state_machine_ == states::HOVER
        || state_machine_ == states::WAIT_FOR_USER_CONTROLLER || state_machine_ == states::FEEDTHROUGH)
    {
      new_state = states::BREAKING;
      desired_state_after_breaking_ = states::OPTITRACK_LAND;
    }
    if (state_machine_ == states::EMERGENCYLAND && state_machine_before_emg_landing_ == states::OPTITRACK_LAND)
    {
      if (getEstimatorID() == estimator_common::estimator_id::OPTITRACK
          || getEstimatorID() == estimator_common::estimator_id::SIMULATOR)
      {
        new_state = states::OPTITRACK_LAND;
      }
    }
  }

  // switch to VISION_LAND
  if (desired_new_state == states::VISION_LAND)
  {
    if (state_machine_ == states::VISION_START || state_machine_ == states::HOVER
        || state_machine_ == states::WAIT_FOR_USER_CONTROLLER || state_machine_ == states::FEEDTHROUGH)
    {
      if (getEstimatorID() == estimator_common::estimator_id::MSF)
      {
        new_state = states::BREAKING;
        desired_state_after_breaking_ = states::VISION_LAND;
      }
      else
      {
        new_state = states::VISION_LAND;
      }
    }
    if (state_machine_ == states::BREAKING)
    {
      new_state = states::VISION_LAND;
    }
  }

  // switch to EMERGENCYLAND
  if (desired_new_state == states::EMERGENCYLAND)
  {
    if (state_machine_ != states::INVALID && state_machine_ != states::OFF && state_machine_ != states::EMERGENCYLAND)
    {
      if ((state_machine_ == states::OPTITRACK_LAND && time_to_ramp_down_)
          || (state_machine_ == states::VISION_LAND && time_to_ramp_down_))
      {
        // Not go into emergency landing anymore since propellers are ramping down already
      }
      else
      {
        state_machine_before_emg_landing_ = state_machine_;
        new_state = states::EMERGENCYLAND;
      }
    }
  }

  // switch to WAIT_FOR_USER_CONTROLLER
  if (desired_new_state == states::WAIT_FOR_USER_CONTROLLER)
  {
    if (state_machine_ == states::OFF || state_machine_ == states::HOVER)
    {
      state_machine_before_waiting_for_user_controller_ = state_machine_;
      new_state = states::WAIT_FOR_USER_CONTROLLER;
    }
  }

  // switch to FEEDTHROUGH
  if (desired_new_state == states::FEEDTHROUGH)
  {
    if (state_machine_ == states::WAIT_FOR_USER_CONTROLLER)
    {
      new_state = states::FEEDTHROUGH;
    }
  }

  // switch to BREAKING
  if (desired_new_state == states::BREAKING)
  {
    if (state_machine_ == states::OPTITRACK_START || state_machine_ == states::VISION_START
        || state_machine_ == states::HOVER || state_machine_ == states::OPTITRACK_LAND
        || state_machine_ == states::VISION_LAND || state_machine_ == states::WAIT_FOR_USER_CONTROLLER
        || state_machine_ == states::FEEDTHROUGH || state_machine_ == states::RC_MANUAL)
    {
      new_state = states::BREAKING;
      desired_state_after_breaking_ = state_machine_;
    }
    if (state_machine_ == states::EMERGENCYLAND && state_machine_before_emg_landing_ == states::BREAKING)
    {
      if (getEstimatorID() == estimator_common::estimator_id::OPTITRACK
          || getEstimatorID() == estimator_common::estimator_id::SIMULATOR)
      {
        new_state = states::BREAKING;
        desired_state_after_breaking_ = states::HOVER;
      }
    }
  }

  // switch to RC_MANUAL
  if (desired_new_state == states::RC_MANUAL)
  {
    new_state = states::RC_MANUAL;
  }

  if (new_state != states::INVALID && new_state != state_machine_)
  {
    ROS_INFO("[%s] New State: %d", pnh_.getNamespace().c_str(), new_state);
    first_time_in_new_mode_ = true;
    time_of_switch_to_current_state_ = ros::Time::now();
    state_machine_ = new_state;
  }

  if (new_state == desired_new_state && new_state != states::INVALID)
  {
    return true;
  }

  return false;
}

} // namespace copilot
