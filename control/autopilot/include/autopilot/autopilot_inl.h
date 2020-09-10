#pragma once

#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <visualization_msgs/MarkerArray.h>

namespace autopilot {

template <typename Tcontroller, typename Tparams>
AutoPilot<Tcontroller, Tparams>::AutoPilot(const ros::NodeHandle& nh,
                                           const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      state_predictor_(nh_, pnh_),
      reference_state_(),
      received_state_est_(),
      desired_velocity_command_(),
      reference_state_input_(),
      received_low_level_feedback_(),
      autopilot_state_(States::OFF),
      state_before_rc_manual_flight_(States::OFF),
      state_estimate_available_(false),
      time_of_switch_to_current_state_(),
      first_time_in_new_state_(true),
      initial_start_position_(),
      initial_land_position_(),
      time_to_ramp_down_(false),
      time_started_ramping_down_(),
      initial_drop_thrust_(0.0),
      time_last_velocity_command_handled_(),
      time_last_reference_state_input_received_(),
      desired_state_after_breaking_(States::HOVER),
      state_before_emergency_landing_(States::OFF),
      force_breaking_(false),
      requested_go_to_pose_(),
      received_go_to_pose_command_(false),
      stop_go_to_pose_thread_(false),
      trajectory_queue_(),
      time_start_trajectory_execution_(),
      time_last_control_command_input_received_(),
      last_control_command_input_thrust_high_(false),
      stop_watchdog_thread_(false),
      time_last_state_estimate_received_(),
      time_started_emergency_landing_(),
      destructor_invoked_(false),
      time_last_autopilot_feedback_published_(),
      time_last_control_command_published_(),
      time_last_control_command_computed_() {
  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Publishers
  control_command_pub_ =
      nh_.advertise<quadrotor_msgs::ControlCommand>("control_command", 1);
  autopilot_feedback_pub_ =
      nh_.advertise<quadrotor_msgs::AutopilotFeedback>("autopilot/feedback", 1);

  // Subscribers
  state_estimate_sub_ =
      nh_.subscribe("autopilot/state_estimate", 1,
                    &AutoPilot<Tcontroller, Tparams>::stateEstimateCallback,
                    this, ros::TransportHints().tcpNoDelay());
  low_level_feedback_sub_ = nh_.subscribe(
      "low_level_feedback", 1,
      &AutoPilot<Tcontroller, Tparams>::lowLevelFeedbackCallback, this);

  pose_command_sub_ = nh_.subscribe(
      "autopilot/pose_command", 1,
      &AutoPilot<Tcontroller, Tparams>::poseCommandCallback, this);
  velocity_command_sub_ = nh_.subscribe(
      "autopilot/velocity_command", 1,
      &AutoPilot<Tcontroller, Tparams>::velocityCommandCallback, this);
  reference_state_sub_ = nh_.subscribe(
      "autopilot/reference_state", 1,
      &AutoPilot<Tcontroller, Tparams>::referenceStateCallback, this);
  trajectory_sub_ =
      nh_.subscribe("autopilot/trajectory", 1,
                    &AutoPilot<Tcontroller, Tparams>::trajectoryCallback, this);
  control_command_input_sub_ = nh_.subscribe(
      "autopilot/control_command_input", 1,
      &AutoPilot<Tcontroller, Tparams>::controlCommandInputCallback, this);

  start_sub_ =
      nh_.subscribe("autopilot/start", 1,
                    &AutoPilot<Tcontroller, Tparams>::startCallback, this);
  force_hover_sub_ =
      nh_.subscribe("autopilot/force_hover", 1,
                    &AutoPilot<Tcontroller, Tparams>::forceHoverCallback, this);
  land_sub_ =
      nh_.subscribe("autopilot/land", 1,
                    &AutoPilot<Tcontroller, Tparams>::landCallback, this);
  off_sub_ = nh_.subscribe("autopilot/off", 1,
                           &AutoPilot<Tcontroller, Tparams>::offCallback, this);

  reload_param_sub_ = nh_.subscribe(
      "autopilot/reload_parameters", 1,
      &AutoPilot<Tcontroller, Tparams>::reloadParamsCallback, this);

  // Start watchdog thread
  try {
    watchdog_thread_ =
        std::thread(&AutoPilot<Tcontroller, Tparams>::watchdogThread, this);
  } catch (...) {
    ROS_ERROR("[%s] Could not successfully start watchdog thread.",
              pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Start go to pose thread
  try {
    go_to_pose_thread_ =
        std::thread(&AutoPilot<Tcontroller, Tparams>::goToPoseThread, this);
  } catch (...) {
    ROS_ERROR("[%s] Could not successfully start go to pose thread.",
              pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
}

template <typename Tcontroller, typename Tparams>
AutoPilot<Tcontroller, Tparams>::~AutoPilot() {
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
template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::watchdogThread() {
  ros::Rate watchdog_rate(kWatchdogFrequency_);
  while (ros::ok() && !stop_watchdog_thread_) {
    watchdog_rate.sleep();

    std::lock_guard<std::mutex> main_lock(main_mutex_);

    const ros::Time time_now = ros::Time::now();

    if (state_estimate_available_ &&
        time_now - time_last_state_estimate_received_ >
            ros::Duration(state_estimate_timeout_)) {
      ROS_ERROR("[%s] Lost state estimate", pnh_.getNamespace().c_str());
      state_estimate_available_ = false;
    }

    if (!state_estimate_available_ && autopilot_state_ != States::OFF &&
        autopilot_state_ != States::EMERGENCY_LAND &&
        autopilot_state_ != States::COMMAND_FEEDTHROUGH &&
        autopilot_state_ != States::RC_MANUAL) {
      ROS_INFO("[%s] No state estimate available, switching to emergency land.",
               pnh_.getNamespace().c_str());
      setAutoPilotStateForced(States::EMERGENCY_LAND);
    }

    if (autopilot_state_ == States::EMERGENCY_LAND) {
      // Check timeout to switch to OFF
      if (time_now - time_started_emergency_landing_ >
          ros::Duration(emergency_land_duration_)) {
        setAutoPilotStateForced(States::OFF);
      }

      // Send emergency landing control command
      quadrotor_common::ControlCommand control_cmd;
      control_cmd.armed = true;
      control_cmd.control_mode = quadrotor_common::ControlMode::ATTITUDE;
      control_cmd.collective_thrust = emergency_land_thrust_;
      control_cmd.timestamp = time_now;
      control_cmd.expected_execution_time =
          control_cmd.timestamp + ros::Duration(control_command_delay_);
      publishControlCommand(control_cmd);
    }

    if (autopilot_state_ == States::COMMAND_FEEDTHROUGH &&
        (time_now - time_last_control_command_input_received_) >
            ros::Duration(control_command_input_timeout_)) {
      if (last_control_command_input_thrust_high_) {
        ROS_WARN(
            "[%s] Did not receive control command inputs anymore but last "
            "thrust command was high, will switch to hover",
            pnh_.getNamespace().c_str());
        setAutoPilotState(States::HOVER);
      } else {
        ROS_WARN(
            "[%s] Did not receive control command inputs anymore but last "
            "thrust command was low, will switch to off",
            pnh_.getNamespace().c_str());
        setAutoPilotState(States::OFF);
      }
    }

    if (!state_estimate_available_) {
      // Publish autopilot feedback throttled down to a maximum frequency
      // If there is no state estimate no feedback would be published otherwise
      if ((ros::Time::now() - time_last_autopilot_feedback_published_) >=
          ros::Duration(1.0 / kMaxAutopilotFeedbackPublishFrequency_)) {
        publishAutopilotFeedback(autopilot_state_,
                                 ros::Duration(control_command_delay_),
                                 ros::Duration(0.0), ros::Duration(0.0), 0,
                                 received_low_level_feedback_, reference_state_,
                                 quadrotor_common::QuadStateEstimate());
      }
    }

    // Mutex is unlocked because it goes out of scope here
  }
}

// Planning thread for planning GO_TO_POSE actions
template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::goToPoseThread() {
  ros::Rate idle_rate(kGoToPoseIdleFrequency_);
  while (ros::ok() && !stop_go_to_pose_thread_) {
    idle_rate.sleep();

    std::lock_guard<std::mutex> go_to_pose_lock(go_to_pose_mutex_);

    if (received_go_to_pose_command_) {
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
      end_state.heading =
          quadrotor_common::quaternionToEulerAnglesZYX(end_state.orientation)
              .z();

      if ((start_state.position - end_state.position).norm() <=
          kGoToPoseNeglectThreshold_) {
        // If the requested position is very close to the current reference
        // position we do not compute a trajectory but just change the
        // reference position
        std::lock_guard<std::mutex> main_lock(main_mutex_);

        reference_state_.position = end_state.position;
        reference_state_.heading = end_state.heading;
        // TODO: Do something smarter if we want to rotate without translation
        setAutoPilotState(States::HOVER);

        // Main mutex is unlocked because it goes out of scope here
      } else {
        quadrotor_common::Trajectory go_to_pose_traj =
            trajectory_generation_helper::polynomials::
                computeTimeOptimalTrajectory(
                    start_state, end_state,
                    kGoToPosePolynomialOrderOfContinuity_,
                    go_to_pose_max_velocity_, go_to_pose_max_normalized_thrust_,
                    go_to_pose_max_roll_pitch_rate_,
                    kGoToPoseTrajectorySamplingFrequency_);

        trajectory_generation_helper::heading::addConstantHeadingRate(
            start_state.heading, end_state.heading, &go_to_pose_traj);

        if (go_to_pose_traj.trajectory_type !=
            quadrotor_common::Trajectory::TrajectoryType::UNDEFINED) {
          // Push computed trajectory into the queue and set the autopilot
          // to the TRAJECTORY_CONTROL state
          std::lock_guard<std::mutex> main_lock(main_mutex_);

          if (autopilot_state_ == States::GO_TO_POSE) {
            trajectory_queue_.clear();
            trajectory_queue_.push_back(go_to_pose_traj);
            setAutoPilotState(States::TRAJECTORY_CONTROL);
          } else {
            ROS_WARN(
                "[%s] Autopilot state switched to another state from "
                "GO_TO_POSE while computing a go to pose trajectory. "
                "Therefore, trajectory will not be executed.",
                pnh_.getNamespace().c_str());
          }

          // Main mutex is unlocked because it goes out of scope here
        } else {
          ROS_WARN(
              "[%s] Failed to compute valid trajectory, will not execute "
              "go to pose action",
              pnh_.getNamespace().c_str());
        }
      }

      received_go_to_pose_command_ = false;
    }

    // Go to pose mutex is unlocked because it goes out of scope here
  }
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::stateEstimateCallback(
    const nav_msgs::Odometry::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  if ((ros::Time::now() - time_last_control_command_computed_).toSec() <
      min_control_period_comp_) {
    return;
  }

  time_last_control_command_computed_ = ros::Time::now();

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  received_state_est_ = quadrotor_common::QuadStateEstimate(*msg);
  if (!received_state_est_.isValid()) {
    ROS_INFO("[%s] Received invalid state estimate",
             pnh_.getNamespace().c_str());
    ROS_INFO(
        "[%s] x: %.2f,  y: %.2f,  z: %.2f,  qw: %.2f,  qx: %.2f,  qy: %.2f,  "
        "qz: %.2f,  vx: %.2f,  vy: %.2f,  vz: %.2f,  brx: %.2f,  bry: %.2f,  "
        "brz: %.2f",
        pnh_.getNamespace().c_str(), received_state_est_.position.x(),
        received_state_est_.position.y(), received_state_est_.position.z(),
        received_state_est_.orientation.w(),
        received_state_est_.orientation.x(),
        received_state_est_.orientation.y(),
        received_state_est_.orientation.z(), received_state_est_.velocity.x(),
        received_state_est_.velocity.y(), received_state_est_.velocity.z(),
        received_state_est_.bodyrates.x(), received_state_est_.bodyrates.y(),
        received_state_est_.bodyrates.z());
    state_estimate_available_ = false;
    if (autopilot_state_ != States::OFF) {
      // Do not run control loop if state estimate is not valid
      // Only allow OFF state without a valid state estimate
      return;
    }
  } else {
    state_estimate_available_ = true;
    time_last_state_estimate_received_ = ros::Time::now();
  }

  if (!velocity_estimate_in_world_frame_) {
    received_state_est_.transformVelocityToWorldFrame();
  }

  // Push received state estimate into predictor
  state_predictor_.updateWithStateEstimate(received_state_est_);

  quadrotor_common::ControlCommand control_cmd;

  ros::Time wall_time_now = ros::Time::now();
  ros::Time command_execution_time =
      wall_time_now + ros::Duration(control_command_delay_);

  quadrotor_common::QuadStateEstimate predicted_state = received_state_est_;
  if (autopilot_state_ != States::OFF) {
    // If the autopilot is OFF we don't need to predict
    predicted_state = getPredictedStateEstimate(command_execution_time);
  }

  ros::Duration trajectory_execution_left_duration(0.0);
  int trajectories_left_in_queue = 0;
  const ros::Time start_control_command_computation = ros::Time::now();
  // Compute control command depending on autopilot state
  switch (autopilot_state_) {
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
      if (state_estimate_available_) {
        // If we end up here it means that we have regained a valid state
        // estimate, so lets go back to HOVER state unless we were about
        // to land before the emergency landing happened
        ROS_INFO("[%s] Regained state estimate", pnh_.getNamespace().c_str());
        if (state_before_emergency_landing_ == States::LAND) {
          setAutoPilotState(States::LAND);
        } else {
          setAutoPilotState(States::HOVER);
        }
        control_cmd = hover(predicted_state);
      }
      break;
    case States::BREAKING:
      control_cmd = breakVelocity(predicted_state);
      break;
    case States::GO_TO_POSE:
      control_cmd = waitForGoToPoseAction(predicted_state);
      break;
    case States::VELOCITY_CONTROL:
      control_cmd = velocityControl(predicted_state);
      break;
    case States::REFERENCE_CONTROL:
      control_cmd = followReference(predicted_state);
      break;
    case States::TRAJECTORY_CONTROL:
      control_cmd = executeTrajectory(predicted_state,
                                      &trajectory_execution_left_duration,
                                      &trajectories_left_in_queue);
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
  const ros::Duration control_computation_time =
      ros::Time::now() - start_control_command_computation;

  if (autopilot_state_ != States::COMMAND_FEEDTHROUGH) {
    control_cmd.timestamp = wall_time_now;
    control_cmd.expected_execution_time = command_execution_time;
    publishControlCommand(control_cmd);
  }

  // Publish autopilot feedback throttled down to a maximum frequency
  if ((ros::Time::now() - time_last_autopilot_feedback_published_) >=
      ros::Duration(1.0 / kMaxAutopilotFeedbackPublishFrequency_)) {
    publishAutopilotFeedback(
        autopilot_state_, ros::Duration(control_command_delay_),
        control_computation_time, trajectory_execution_left_duration,
        trajectories_left_in_queue, received_low_level_feedback_,
        reference_state_, predicted_state);
  }

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::lowLevelFeedbackCallback(
    const quadrotor_msgs::LowLevelFeedback::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  received_low_level_feedback_ = *msg;

  if (msg->control_mode == msg->RC_MANUAL &&
      autopilot_state_ != States::RC_MANUAL) {
    setAutoPilotState(States::RC_MANUAL);
  }
  if (msg->control_mode != msg->RC_MANUAL &&
      autopilot_state_ == States::RC_MANUAL) {
    if (state_before_rc_manual_flight_ == States::OFF) {
      setAutoPilotState(States::OFF);
    } else {
      force_breaking_ = true;  // Ensure reference state is reset
      setAutoPilotState(States::HOVER);
    }
  }

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::poseCommandCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  // We need to lock both the go to pose mutex and the main mutex here.
  // The order of locking has to match the one in goToPoseThread() to prevent
  // deadlocks. So the go to pose mutex has to be locked first.
  std::lock_guard<std::mutex> go_to_pose_lock(go_to_pose_mutex_);
  std::lock_guard<std::mutex> main_lock(main_mutex_);

  // Idea: A trajectory is planned to the desired pose in a separate
  // thread. Once the thread is done it pushes the computed trajectory into the
  // trajectory queue and switches to TRAJECTORY_CONTROL mode
  if (autopilot_state_ == States::HOVER) {
    setAutoPilotState(States::GO_TO_POSE);
    requested_go_to_pose_ = *msg;
    received_go_to_pose_command_ = true;
  } else {
    ROS_WARN(
        "[%s] Will not execute go to pose action since autopilot is "
        "not in HOVER",
        pnh_.getNamespace().c_str());
  }

  // Mutexes are unlocked because they go out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::velocityCommandCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (quadrotor_common::geometryToEigen(msg->twist.linear).norm() <=
          kVelocityCommandZeroThreshold_ &&
      fabs(msg->twist.angular.z) <= kVelocityCommandZeroThreshold_) {
    // Only consider commands with non negligible velocities
    return;
  }
  if (autopilot_state_ != States::HOVER &&
      autopilot_state_ != States::VELOCITY_CONTROL) {
    return;
  }
  if (autopilot_state_ != States::VELOCITY_CONTROL) {
    setAutoPilotState(States::VELOCITY_CONTROL);
  }

  desired_velocity_command_ = *msg;
  desired_velocity_command_.header.stamp = ros::Time::now();

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::referenceStateCallback(
    const quadrotor_msgs::TrajectoryPoint::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (autopilot_state_ != States::HOVER &&
      autopilot_state_ != States::REFERENCE_CONTROL) {
    return;
  }
  if (autopilot_state_ != States::REFERENCE_CONTROL) {
    if ((reference_state_.position -
         quadrotor_common::geometryToEigen(msg->pose.position))
            .norm() < kPositionJumpTolerance_) {
      setAutoPilotState(States::REFERENCE_CONTROL);
    } else {
      ROS_WARN(
          "[%s] Received first reference state that is more than %fm away "
          "from current position, will not go to REFERENCE_CONTROL mode.",
          pnh_.getNamespace().c_str(), kPositionJumpTolerance_);
    }
  } else if ((reference_state_.position -
              quadrotor_common::geometryToEigen(msg->pose.position))
                 .norm() > kPositionJumpTolerance_) {
    ROS_WARN_THROTTLE(
        0.5,
        "[%s] Received reference state that is more than %fm away "
        "from current reference position and is therefore rejected.",
        pnh_.getNamespace().c_str(), kPositionJumpTolerance_);
    return;
  }

  time_last_reference_state_input_received_ = ros::Time::now();
  reference_state_input_ = *msg;

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::trajectoryCallback(
    const quadrotor_msgs::Trajectory::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  // Idea: trajectories are being pushed into a queue and consecutively
  // executed if there are no jumps in the beginning and between them

  if (autopilot_state_ != States::HOVER &&
      autopilot_state_ != States::TRAJECTORY_CONTROL) {
    return;
  }
  if (msg->type == msg->UNDEFINED || msg->points.size() == 0) {
    ROS_WARN("[%s] Received invalid trajectory, will ignore trajectory",
             pnh_.getNamespace().c_str());
    return;
  }
  if (trajectory_queue_.empty()) {
    // Before executing the first trajectory segment, the autopilot must be in
    // HOVER state
    if (autopilot_state_ != States::HOVER) {
      ROS_WARN(
          "[%s] Received first trajectory but autopilot is not in HOVER, "
          "will ignore trajectory",
          pnh_.getNamespace().c_str());
      return;
    }
    // Check if there is a jump in the beginning of the trajectory
    if ((reference_state_.position -
         quadrotor_common::geometryToEigen(msg->points[0].pose.position))
            .norm() > kPositionJumpTolerance_) {
      ROS_WARN(
          "[%s] First received trajectory segment does not start at current "
          "position, will ignore trajectory",
          pnh_.getNamespace().c_str());
      return;
    }
  } else {
    // Check that there is no jump from the last trajectory in the queue to the
    // newly received one
    if ((trajectory_queue_.back().points.back().position -
         quadrotor_common::geometryToEigen(msg->points.front().pose.position))
            .norm() > kPositionJumpTolerance_) {
      ROS_WARN(
          "[%s] Received trajectory has a too large jump from the last "
          "trajectory in the queue, will ignore trajectory",
          pnh_.getNamespace().c_str());
      return;
    }
  }

  trajectory_queue_.push_back(quadrotor_common::Trajectory(*msg));

  if (autopilot_state_ != States::TRAJECTORY_CONTROL) {
    setAutoPilotState(States::TRAJECTORY_CONTROL);
  }

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::controlCommandInputCallback(
    const quadrotor_msgs::ControlCommand::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  if (!enable_command_feedthrough_ || !msg->armed) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (autopilot_state_ != States::OFF && autopilot_state_ != States::HOVER &&
      autopilot_state_ != States::COMMAND_FEEDTHROUGH) {
    // Only allow this if the current state is OFF or HOVER
    // or already in COMMAND_FEEDTHROUGH
    return;
  }

  if (autopilot_state_ != States::COMMAND_FEEDTHROUGH) {
    setAutoPilotState(States::COMMAND_FEEDTHROUGH);
  }

  control_command_pub_.publish(*msg);

  time_last_control_command_input_received_ = ros::Time::now();
  if (msg->collective_thrust > kThrustHighThreshold_) {
    last_control_command_input_thrust_high_ = true;
  } else {
    last_control_command_input_thrust_high_ = false;
  }

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::startCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  ROS_INFO_THROTTLE(0.5, "[%s] START command received",
                    pnh_.getNamespace().c_str());
  if (autopilot_state_ == States::OFF) {
    if (state_estimate_available_) {
      if (received_state_est_.coordinate_frame ==
              quadrotor_common::QuadStateEstimate::CoordinateFrame::WORLD ||
          received_state_est_.coordinate_frame ==
              quadrotor_common::QuadStateEstimate::CoordinateFrame::OPTITRACK) {
        ROS_INFO(
            "[%s] Absolute state estimate available, taking off based on it",
            pnh_.getNamespace().c_str());
        setAutoPilotState(States::START);
      } else if (
          received_state_est_.coordinate_frame ==
              quadrotor_common::QuadStateEstimate::CoordinateFrame::VISION ||
          received_state_est_.coordinate_frame ==
              quadrotor_common::QuadStateEstimate::CoordinateFrame::LOCAL) {
        ROS_INFO("[%s] Relative state estimate available, switch to hover",
                 pnh_.getNamespace().c_str());
        force_breaking_ = true;  // Ensure reference state is reset
        setAutoPilotState(States::HOVER);
      }
    } else {
      ROS_ERROR("[%s] No state estimate available, will not start",
                pnh_.getNamespace().c_str());
    }
  } else {
    ROS_WARN_THROTTLE(1.0,
                      "[%s] Autopilot is not OFF, will not switch to START",
                      pnh_.getNamespace().c_str());
  }

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::forceHoverCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  ROS_INFO_THROTTLE(0.5, "[%s] FORCE HOVER command received",
                    pnh_.getNamespace().c_str());

  if (autopilot_state_ == States::OFF || autopilot_state_ == States::HOVER ||
      autopilot_state_ == States::EMERGENCY_LAND ||
      autopilot_state_ == States::RC_MANUAL) {
    return;
  }

  force_breaking_ = true;
  setAutoPilotState(States::HOVER);

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::landCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  ROS_INFO_THROTTLE(0.5, "[%s] LAND command received",
                    pnh_.getNamespace().c_str());
  if (autopilot_state_ == States::OFF || autopilot_state_ == States::LAND ||
      autopilot_state_ == States::EMERGENCY_LAND ||
      autopilot_state_ == States::RC_MANUAL) {
    return;
  }

  setAutoPilotState(States::LAND);

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::offCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (autopilot_state_ != States::OFF) {
    ROS_INFO("[%s] OFF command received", pnh_.getNamespace().c_str());
    setAutoPilotStateForced(States::OFF);
    // Allow user to take over manually and land the vehicle, then off the
    // controller and disable the RC without the vehicle going back to hover
    state_before_rc_manual_flight_ = States::OFF;
  }

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::reloadParamsCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  if (destructor_invoked_) {
    return;
  }

  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (autopilot_state_ != States::OFF) {
    ROS_INFO("[%s] Setting autopilot to OFF before reloading parameters",
             pnh_.getNamespace().c_str());
    setAutoPilotStateForced(States::OFF);
    // Allow user to take over manually and land the vehicle, then off the
    // controller and disable the RC without the vehicle going back to hover
    state_before_rc_manual_flight_ = States::OFF;
  }
  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Mutex is unlocked because it goes out of scope here
}

template <typename Tcontroller, typename Tparams>
quadrotor_common::ControlCommand AutoPilot<Tcontroller, Tparams>::start(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  quadrotor_common::ControlCommand command;
  if (first_time_in_new_state_) {
    first_time_in_new_state_ = false;
    initial_start_position_ = state_estimate.position;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = state_estimate.position;
    reference_state_.heading =
        quadrotor_common::quaternionToEulerAnglesZYX(state_estimate.orientation)
            .z();
    if (state_estimate.position.z() >= optitrack_land_drop_height_ + 0.5) {
      // assuming start from handheld configuration, go directly to hover
      ROS_INFO(
          "Assuming handheld start, since state_estimate.position.z() > %.2f",
          optitrack_land_drop_height_);
      setAutoPilotState(States::HOVER);
    }
  }

  if (timeInCurrentState() > optitrack_start_land_timeout_ ||
      reference_state_.position.z() >= optitrack_start_height_) {
    setAutoPilotState(States::HOVER);
  } else {
    if (timeInCurrentState() < start_idle_duration_) {
      command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
      command.armed = true;
      command.bodyrates = Eigen::Vector3d::Zero();
      command.collective_thrust = idle_thrust_;
      return command;
    } else {
      reference_state_.position.z() =
          initial_start_position_.z() +
          start_land_velocity_ * (timeInCurrentState() - start_idle_duration_);
      reference_state_.velocity.z() = start_land_velocity_;
      if (timeInCurrentState() <
          start_idle_duration_ +
              start_land_velocity_ / start_land_acceleration_) {
        reference_state_.acceleration.z() = start_land_acceleration_;
        reference_state_.velocity.z() =
            start_land_acceleration_ *
            (timeInCurrentState() - start_idle_duration_);
      } else {
        reference_state_.acceleration.setZero();
      }
    }
  }

  reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
  command = base_controller_.run(state_estimate, reference_trajectory_,
                                 base_controller_params_);

  return command;
}

template <typename Tcontroller, typename Tparams>
quadrotor_common::ControlCommand AutoPilot<Tcontroller, Tparams>::hover(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  if (first_time_in_new_state_) {
    first_time_in_new_state_ = false;
    // We can only enter HOVER mode from breaking unless breaking is not
    // necessary. So we keep the reference position and heading and only
    // set the derivatives to zero to avoid jumps due to setting the reference
    // to the current estimate
    const Eigen::Vector3d current_position = reference_state_.position;
    const double current_heading = reference_state_.heading;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = current_position;
    reference_state_.heading = current_heading;
  }

  reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
  const quadrotor_common::ControlCommand command = base_controller_.run(
      state_estimate, reference_trajectory_, base_controller_params_);

  return command;
}

template <typename Tcontroller, typename Tparams>
quadrotor_common::ControlCommand AutoPilot<Tcontroller, Tparams>::land(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  quadrotor_common::ControlCommand command;

  if (first_time_in_new_state_) {
    first_time_in_new_state_ = false;
    initial_land_position_ = state_estimate.position;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = state_estimate.position;
    reference_state_.heading =
        quadrotor_common::quaternionToEulerAnglesZYX(state_estimate.orientation)
            .z();
    // Reset ramp down flag
    time_to_ramp_down_ = false;
  }

  reference_state_.position.z() =
      initial_land_position_.z() - start_land_velocity_ * timeInCurrentState();
  reference_state_.velocity.z() = -start_land_velocity_;

  reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
  command = base_controller_.run(state_estimate, reference_trajectory_,
                                 base_controller_params_);
  /*
  if (received_state_est_.coordinate_frame ==
          quadrotor_common::QuadStateEstimate::CoordinateFrame::WORLD ||
      received_state_est_.coordinate_frame ==
          quadrotor_common::QuadStateEstimate::CoordinateFrame::OPTITRACK) {
    // We only allow ramping down the propellers if we have an absolute state
    // estimate available, otherwise we just keep going down "forever"
    if (!time_to_ramp_down_ &&
        (state_estimate.position.z() < optitrack_land_drop_height_ ||
         timeInCurrentState() > optitrack_start_land_timeout_)) {
      time_to_ramp_down_ = true;
      time_started_ramping_down_ = ros::Time::now();
    }
  }

  if (time_to_ramp_down_) {
    // we are low enough -> ramp down the thrust
    // we timed out on landing -> ramp down the thrust
    ROS_INFO_THROTTLE(2, "[%s] Ramping propeller down",
                      pnh_.getNamespace().c_str());
    command.collective_thrust =
        initial_drop_thrust_ -
        initial_drop_thrust_ / propeller_ramp_down_timeout_ *
            (ros::Time::now() - time_started_ramping_down_).toSec();
  }
  */

  if (command.collective_thrust <= 0.2) {
    setAutoPilotState(States::OFF);
    command.zero();
  }

  return command;
}

template <typename Tcontroller, typename Tparams>
quadrotor_common::ControlCommand AutoPilot<Tcontroller, Tparams>::breakVelocity(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  if (first_time_in_new_state_) {
    first_time_in_new_state_ = false;
    if (force_breaking_ ||
        state_estimate.velocity.norm() > breaking_velocity_threshold_) {
      force_breaking_ = false;
      reference_state_ = quadrotor_common::TrajectoryPoint();
      reference_state_.position = state_estimate.position;
      reference_state_.velocity = state_estimate.velocity;
      reference_state_.heading = quadrotor_common::quaternionToEulerAnglesZYX(
                                     state_estimate.orientation)
                                     .z();
    } else {
      // Breaking is not necessary so we do not update the reference position
      // but set all derivatives to zero
      const Eigen::Vector3d current_position = state_estimate.position;
      const double current_heading =
          quadrotor_common::quaternionToEulerAnglesZYX(
              state_estimate.orientation)
              .z();
      reference_state_ = quadrotor_common::TrajectoryPoint();
      reference_state_.position = current_position;
      reference_state_.heading = current_heading;
      setAutoPilotStateForced(desired_state_after_breaking_);

      reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
      return base_controller_.run(state_estimate, reference_trajectory_,
                                  base_controller_params_);
    }
  }

  if (state_estimate.velocity.norm() < breaking_velocity_threshold_ ||
      timeInCurrentState() > breaking_timeout_) {
    const double current_heading = reference_state_.heading;
    reference_state_ = quadrotor_common::TrajectoryPoint();
    reference_state_.position = state_estimate.position;
    reference_state_.heading = current_heading;
    setAutoPilotStateForced(desired_state_after_breaking_);
  }

  reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
  const quadrotor_common::ControlCommand command = base_controller_.run(
      state_estimate, reference_trajectory_, base_controller_params_);

  return command;
}

template <typename Tcontroller, typename Tparams>
quadrotor_common::ControlCommand
AutoPilot<Tcontroller, Tparams>::waitForGoToPoseAction(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  if (first_time_in_new_state_) {
    first_time_in_new_state_ = false;
    // We do not reset the reference state since we are only allowed to
    // transition to the GO_TO_POSE state from HOVER
  }

  // This effectively just hovers since the actual go to pose action happens
  // by computing a trajectory in a separate thread and then call the trajectory
  // callback which executes the go to pose action.
  // Going into this state instead of remaining in hover prevents the autopilot
  // to go to TRAJECTORY_CONTROL mode in the time where a go to pose trajectory
  // is planned

  reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
  const quadrotor_common::ControlCommand command = base_controller_.run(
      state_estimate, reference_trajectory_, base_controller_params_);

  return command;
}

template <typename Tcontroller, typename Tparams>
quadrotor_common::ControlCommand
AutoPilot<Tcontroller, Tparams>::velocityControl(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  if (first_time_in_new_state_) {
    first_time_in_new_state_ = false;
    time_last_velocity_command_handled_ = ros::Time::now();
  }

  if ((ros::Time::now() - desired_velocity_command_.header.stamp) >
      ros::Duration(velocity_command_input_timeout_)) {
    desired_velocity_command_.twist.linear.x = 0.0;
    desired_velocity_command_.twist.linear.y = 0.0;
    desired_velocity_command_.twist.linear.z = 0.0;
    desired_velocity_command_.twist.angular.z = 0.0;
  }

  const double dt =
      (ros::Time::now() - time_last_velocity_command_handled_).toSec();
  const double alpha_velocity = 1 - exp(-dt / tau_velocity_command_);

  const Eigen::Vector3d commanded_velocity =
      quadrotor_common::geometryToEigen(desired_velocity_command_.twist.linear);
  reference_state_.velocity =
      (1.0 - alpha_velocity) * reference_state_.velocity +
      alpha_velocity * commanded_velocity;

  if (reference_state_.velocity.norm() < kVelocityCommandZeroThreshold_ &&
      commanded_velocity.norm() < kVelocityCommandZeroThreshold_) {
    reference_state_.velocity = Eigen::Vector3d::Zero();
    if (fabs(desired_velocity_command_.twist.angular.z) <
        kVelocityCommandZeroThreshold_) {
      reference_state_.heading_rate = 0.0;
      setAutoPilotState(States::HOVER);
    }
  }
  reference_state_.position += reference_state_.velocity * dt;

  reference_state_.heading += desired_velocity_command_.twist.angular.z * dt;
  reference_state_.heading =
      quadrotor_common::wrapMinusPiToPi(reference_state_.heading);
  reference_state_.heading_rate = desired_velocity_command_.twist.angular.z;

  time_last_velocity_command_handled_ = ros::Time::now();

  reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
  const quadrotor_common::ControlCommand command = base_controller_.run(
      state_estimate, reference_trajectory_, base_controller_params_);

  return command;
}

template <typename Tcontroller, typename Tparams>
quadrotor_common::ControlCommand
AutoPilot<Tcontroller, Tparams>::followReference(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  if (first_time_in_new_state_) {
    first_time_in_new_state_ = false;
  }

  if ((ros::Time::now() - time_last_reference_state_input_received_) >
      ros::Duration(reference_state_input_timeout_)) {
    setAutoPilotState(States::HOVER);
  }

  reference_state_ = quadrotor_common::TrajectoryPoint(reference_state_input_);

  reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
  const quadrotor_common::ControlCommand command = base_controller_.run(
      state_estimate, reference_trajectory_, base_controller_params_);

  return command;
}

template <typename Tcontroller, typename Tparams>
quadrotor_common::ControlCommand
AutoPilot<Tcontroller, Tparams>::executeTrajectory(
    const quadrotor_common::QuadStateEstimate& state_estimate,
    ros::Duration* trajectory_execution_left_duration,
    int* trajectories_left_in_queue) {
  const ros::Time time_now = ros::Time::now();
  if (first_time_in_new_state_) {
    first_time_in_new_state_ = false;
    time_start_trajectory_execution_ = time_now;
  }

  if (trajectory_queue_.empty()) {
    ROS_ERROR(
        "[%s] Trajectory queue was unexpectedly emptied, going back to HOVER",
        pnh_.getNamespace().c_str());
    *trajectory_execution_left_duration = ros::Duration(0.0);
    *trajectories_left_in_queue = 0;
    setAutoPilotState(States::HOVER);

    reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
    return base_controller_.run(state_estimate, reference_trajectory_,
                                base_controller_params_);
  }

  if ((time_now - time_start_trajectory_execution_) >
      trajectory_queue_.front().points.back().time_from_start) {
    if (trajectory_queue_.size() == 1) {
      // This was the last trajectory in the queue -> go back to hover
      reference_state_ = trajectory_queue_.back().points.back();
      *trajectory_execution_left_duration = ros::Duration(0.0);
      *trajectories_left_in_queue = 0;
      trajectory_queue_.pop_front();
      setAutoPilotStateForced(States::HOVER);
      reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
      return base_controller_.run(state_estimate, reference_trajectory_,
                                  base_controller_params_);
    } else {
      time_start_trajectory_execution_ +=
          trajectory_queue_.front().points.back().time_from_start;
      trajectory_queue_.pop_front();
    }
  }

  // Time from trajectory start and corresponding reference state.
  const ros::Duration dt = time_now - time_start_trajectory_execution_;
  reference_state_ = trajectory_queue_.front().getStateAtTime(dt);

  // New trajectory where we fill in our lookahead horizon.
  reference_trajectory_ = quadrotor_common::Trajectory();
  reference_trajectory_.trajectory_type =
      quadrotor_common::Trajectory::TrajectoryType::GENERAL;

  bool lookahead_reached(false);  // Boolean break flag
  // Time wrap if lookahead spans multiple trajectories:
  double time_wrapover(0.0);

  for (auto trajectory : trajectory_queue_) {
    for (auto point : trajectory.points) {
      // Check wether we reached our lookahead.
      // Use boolean flag to also break the outer loop.
      if (point.time_from_start.toSec() >
          (dt.toSec() - time_wrapover + predictive_control_lookahead_)) {
        lookahead_reached = true;
        break;
      }
      // Add a point if the time corresponds to a sample on the lookahead.
      if (point.time_from_start.toSec() > (dt.toSec() - time_wrapover)) {
        // check if two trajectory points are the same...
        if (reference_trajectory_.points.size() > 1) {
          point.time_from_start += ros::Duration(time_wrapover);
          reference_trajectory_.points.push_back(point);
        } else {
          // this is the first point of the reference trajectory
          reference_trajectory_.points.push_back(point);
        }
      }
    }
    if (lookahead_reached) break;  // Break on boolean flag.
    // Sum up the wrap-over time if lookahead spans multiple trajectories.
    time_wrapover += trajectory.points.back().time_from_start.toSec();
  }

  *trajectory_execution_left_duration =
      trajectory_queue_.front().points.back().time_from_start -
      reference_state_.time_from_start;
  if (trajectory_queue_.size() > 1) {
    std::list<quadrotor_common::Trajectory>::const_iterator it;
    for (it = std::next(trajectory_queue_.begin(), 1);
         it != trajectory_queue_.end(); it++) {
      *trajectory_execution_left_duration += it->points.back().time_from_start;
    }
  }
  *trajectories_left_in_queue = trajectory_queue_.size();

  // handle case of empty reference_trajectory
  if (reference_trajectory_.points.empty()) {
    ROS_WARN("Empty reference trajectory!");
    *trajectory_execution_left_duration = ros::Duration(0.0);
    *trajectories_left_in_queue = 0;
    setAutoPilotState(States::HOVER);

    reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
    return base_controller_.run(state_estimate, reference_trajectory_,
                                base_controller_params_);
  }

  const quadrotor_common::ControlCommand command = base_controller_.run(
      state_estimate, reference_trajectory_, base_controller_params_);

  return command;
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::setAutoPilotState(
    const States& new_state) {
  if (!state_estimate_available_ && new_state != States::OFF &&
      new_state != States::EMERGENCY_LAND &&
      new_state != States::COMMAND_FEEDTHROUGH &&
      new_state != States::RC_MANUAL) {
    ROS_INFO("[%s] No state estimate available, switching to emergency land.",
             pnh_.getNamespace().c_str());
    setAutoPilotStateForced(States::EMERGENCY_LAND);
    return;
  }

  if (new_state == States::HOVER || new_state == States::LAND) {
    desired_state_after_breaking_ = new_state;
    setAutoPilotStateForced(States::BREAKING);
    return;
  }
  if (new_state == States::RC_MANUAL) {
    if (autopilot_state_ == States::OFF) {
      state_before_rc_manual_flight_ = States::OFF;
    } else {
      state_before_rc_manual_flight_ = States::HOVER;
    }
  }
  if (new_state == States::EMERGENCY_LAND) {
    time_started_emergency_landing_ = ros::Time::now();
  }

  setAutoPilotStateForced(new_state);
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::setAutoPilotStateForced(
    const States& new_state) {
  const ros::Time time_now = ros::Time::now();
  if (new_state == States::EMERGENCY_LAND) {
    time_started_emergency_landing_ = time_now;
    if (autopilot_state_ == States::BREAKING) {
      state_before_emergency_landing_ = desired_state_after_breaking_;
    } else {
      state_before_emergency_landing_ = autopilot_state_;
    }
  }
  if (new_state != States::TRAJECTORY_CONTROL && !trajectory_queue_.empty()) {
    trajectory_queue_.clear();
  }
  time_of_switch_to_current_state_ = time_now;
  first_time_in_new_state_ = true;
  autopilot_state_ = new_state;

  std::string state_name;
  switch (autopilot_state_) {
    case States::OFF:
      state_name = "OFF";
      break;
    case States::START:
      state_name = "START";
      break;
    case States::HOVER:
      state_name = "HOVER";
      break;
    case States::LAND:
      state_name = "LAND";
      break;
    case States::EMERGENCY_LAND:
      state_name = "EMERGENCY_LAND";
      break;
    case States::BREAKING:
      state_name = "BREAKING";
      break;
    case States::GO_TO_POSE:
      state_name = "GO_TO_POSE";
      break;
    case States::VELOCITY_CONTROL:
      state_name = "VELOCITY_CONTROL";
      break;
    case States::REFERENCE_CONTROL:
      state_name = "REFERENCE_CONTROL";
      break;
    case States::TRAJECTORY_CONTROL:
      state_name = "TRAJECTORY_CONTROL";
      break;
    case States::COMMAND_FEEDTHROUGH:
      state_name = "COMMAND_FEEDTHROUGH";
      break;
    case States::RC_MANUAL:
      state_name = "RC_MANUAL";
      break;
  }
  ROS_INFO("[%s] Switched to %s state", pnh_.getNamespace().c_str(),
           state_name.c_str());
}

template <typename Tcontroller, typename Tparams>
double AutoPilot<Tcontroller, Tparams>::timeInCurrentState() const {
  return (ros::Time::now() - time_of_switch_to_current_state_).toSec();
}

template <typename Tcontroller, typename Tparams>
quadrotor_common::QuadStateEstimate
AutoPilot<Tcontroller, Tparams>::getPredictedStateEstimate(
    const ros::Time& time) const {
  return state_predictor_.predictState(time);
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::publishControlCommand(
    const quadrotor_common::ControlCommand& control_cmd) {
  if (control_cmd.control_mode == quadrotor_common::ControlMode::NONE) {
    ROS_ERROR("[%s] Control mode is NONE, will not publish ControlCommand",
              pnh_.getNamespace().c_str());
  } else {
    quadrotor_msgs::ControlCommand control_cmd_msg;

    control_cmd_msg = control_cmd.toRosMessage();

    // in optitrack flight, the laird module can only handle control commands at
    // 50-60Hz. Limit publishing frequency here
    if ((ros::Time::now() - time_last_control_command_published_).toSec() >
        min_control_period_pub_) {
      control_command_pub_.publish(control_cmd_msg);
      time_last_control_command_published_ = ros::Time::now();
    }
    state_predictor_.pushCommandToQueue(control_cmd);
    // Save applied thrust to initialize propeller ramping down if necessary
    initial_drop_thrust_ = control_cmd.collective_thrust;
  }
}

template <typename Tcontroller, typename Tparams>
void AutoPilot<Tcontroller, Tparams>::publishAutopilotFeedback(
    const States& autopilot_state, const ros::Duration& control_command_delay,
    const ros::Duration& control_computation_time,
    const ros::Duration& trajectory_execution_left_duration,
    const int trajectories_left_in_queue,
    const quadrotor_msgs::LowLevelFeedback& low_level_feedback,
    const quadrotor_common::TrajectoryPoint& reference_state,
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  quadrotor_msgs::AutopilotFeedback fb_msg;

  fb_msg.header.stamp = ros::Time::now();
  switch (autopilot_state) {
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
  fb_msg.trajectories_left_in_queue = trajectories_left_in_queue;
  fb_msg.low_level_feedback = low_level_feedback;
  fb_msg.reference_state = reference_state.toRosMessage();
  fb_msg.state_estimate = state_estimate.toRosMessage();

  autopilot_feedback_pub_.publish(fb_msg);

  time_last_autopilot_feedback_published_ = ros::Time::now();
}

template <typename Tcontroller, typename Tparams>
bool AutoPilot<Tcontroller, Tparams>::loadParameters() {
#define GET_PARAM(name) \
  if (!quadrotor_common::getParam(#name, name##_, pnh_)) return false

  GET_PARAM(state_estimate_timeout);
  GET_PARAM(velocity_estimate_in_world_frame);
  GET_PARAM(control_command_delay);
  GET_PARAM(start_land_velocity);
  GET_PARAM(start_land_acceleration);
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
  GET_PARAM(control_command_input_timeout);
  GET_PARAM(enable_command_feedthrough);
  GET_PARAM(predictive_control_lookahead);
  GET_PARAM(min_control_period_comp);
  GET_PARAM(min_control_period_pub);

  if (!base_controller_params_.loadParameters(pnh_)) {
    return false;
  }

  return true;

#undef GET_PARAM
}

template class AutoPilot<position_controller::PositionController,
                         position_controller::PositionControllerParams>;

}  // namespace autopilot
