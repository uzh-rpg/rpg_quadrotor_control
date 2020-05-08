# Control Command
# This message is sent to a low level controller on the quadrotor

# control mode enums
uint8 NONE=0
uint8 ANGLE=1
uint8 ANGLERATE=2
uint8 ROTTHROTTLE=3


Header header

# Control mode as defined above
uint8 control_mode

# Flag whether controller was set to OFF
bool off

# Time at which this command should be executed
time execution_time

# Orientation of the body frame with respect to the world frame [-]
geometry_msgs/Quaternion orientation

# Body rates in body frame [rad/s]
# Note that in ANGLE mode the x-y-bodyrates are only feed forward terms computed from a reference trajectory
# Also in ANGLE mode, the z-bodyrate has to be from feedback control
geometry_msgs/Vector3 bodyrates

# Angular accelerations in body frame [rad/s^2]
geometry_msgs/Vector3 angular_accelerations

# Collective mass normalized thrust [m/s^2]
float64 thrust

# Single rotor throttle [0-2000]
float64[4] mot_throttle