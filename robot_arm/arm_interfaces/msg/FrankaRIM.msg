# FrankaRIM.msg
# Message for publishing Franka Reduced Impedance Model parameters

std_msgs/Header header

# Interface dimension (for reshaping)
uint32 m

# Interface stiffness and damping
float64 interface_stiffness
float64 interface_damping

# Effective mass matrix (flattened row-major)
float64[] effective_mass

# Effective force vector (flattened row-major)
float64[] effective_force

# RIM position
float64[] rim_position

# RIM velocity (flattened row-major)
float64[] rim_velocity
