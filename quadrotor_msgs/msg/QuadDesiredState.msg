# Quadrotor Desired State 

Header header

# 3D position in world frame [m]
geometry_msgs/Vector3 position

# 3D velocity in world frame [m/s]
geometry_msgs/Vector3 velocity

# 3D acceleration in world frame [m/s^2]
geometry_msgs/Vector3 acceleration

# 3D jerk in world frame [m/s^3]
geometry_msgs/Vector3 jerk

# 3D snap in world frame [m/s^4]
geometry_msgs/Vector3 snap

# Heading angle with respect to world frame [rad]
float64 heading

# First derivative of the heading angle [rad/s]
float64 heading_rate

# Second derivative of the heading angle [rad/s^2]
float64 heading_acceleration
