Header header

int8 tracking_ok

# State machine of the controller
int8 controller_state

# Time at which the resulting control command should be executed
time command_execution_time

# Desired state used to compute the control command
quad_msgs/QuadDesiredState desired_state

# State estimate used to compute the control command
quad_msgs/QuadStateEstimate state_estimate
