# Quadrotor State Estimate 
# This Message is published by an estimator

Header header

#
int8 estimator_id

# 3D position in world frame [m]
geometry_msgs/Vector3 position

# 3D velocity in world frame [m/s]
geometry_msgs/Vector3 velocity

# Orientation with respect to world frame []
geometry_msgs/Quaternion orientation

# Body rates in body frame [rad/s]
geometry_msgs/Vector3 bodyrates
