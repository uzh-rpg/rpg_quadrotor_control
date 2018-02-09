# Trajectory type enums

# General trajectory type that considers orientation from the pose and
# neglects heading values
uint8 GENERAL=0

# Trajectory types that compute orientation from acceleration and heading
# values and consider derivatives up to what is indicated by the name
uint8 ACCELERATION=1
uint8 JERK=2
uint8 SNAP=3

Header header

# Trajectory type as defined above
uint8 type

quadrotor_msgs/TrajectoryPoint[] points
