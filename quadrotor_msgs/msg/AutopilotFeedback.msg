Header header

# TODO: TBD: Info to know what the copilot is doing, e.g. control goal
int8 state

# Time at which the resulting control command should be executed
duration control_command_delay

# Controller computation time [s]
duration control_computation_time

# Low level feedback
quadrotor_msgs/LowLevelFeedback low_level_feedback

# Desired state used to compute the control command
quadrotor_msgs/TrajectoryPoint reference_state

# State estimate used to compute the control command
nav_msgs/Odometry state_estimate
