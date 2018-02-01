Header header

# State machine of the copilot
int8 state

# Time at which the resulting control command should be executed
time command_execution_time

# Desired state used to compute the control command
quad_msgs/QuadDesiredState desired_state

# State estimate used to compute the control command
nav_msgs/Odometry state_estimate
