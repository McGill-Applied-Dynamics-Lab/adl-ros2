# Cartesian trajectory message for end-effector motion
# Defines a sequence of poses with timing and velocity information for smooth trajectory execution

std_msgs/Header header

# Waypoints to follow in Cartesian space
geometry_msgs/PoseStamped[] points

# Time from trajectory start to reach each waypoint
# Must be same length as points array
# Times must be monotonically increasing
builtin_interfaces/Duration[] time_from_start

# Optional: desired velocity at each waypoint for smooth interpolation
# If provided, must be same length as points array
# If empty, velocities will be computed from positions and times
geometry_msgs/Twist[] velocities

# Optional velocity limits (override controller defaults)
# Set to -1.0 to use controller default values
float64 max_linear_velocity    # m/s
float64 max_angular_velocity   # rad/s
