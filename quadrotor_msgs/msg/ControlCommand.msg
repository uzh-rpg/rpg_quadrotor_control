# Quadrotor control command

# control mode enums
uint8 NONE=0
uint8 ATTITUDE=1
uint8 BODY_RATES=2
uint8 ANGULAR_ACCELERATIONS=3
uint8 ROTOR_THRUSTS=4

Header header

# Control mode as defined above
uint8 control_mode

# Flag whether controller is allowed to arm
bool armed

# Time at which this command should be executed
time expected_execution_time

# Orientation of the body frame with respect to the world frame [-]
geometry_msgs/Quaternion orientation

# Body rates in body frame [rad/s]
# Note that in ATTITUDE mode the x-y-bodyrates are only feed forward terms 
# computed from a reference trajectory
# Also in ATTITUDE mode, the z-bodyrate has to be from feedback control
geometry_msgs/Vector3 bodyrates

# Angular accelerations in body frame [rad/s^2]
geometry_msgs/Vector3 angular_accelerations

# Collective mass normalized thrust [m/s^2]
float64 collective_thrust

# Single rotor thrusts [N]
# These are only considered in the ROTOR_THRUSTS control mode
float64[] rotor_thrusts
