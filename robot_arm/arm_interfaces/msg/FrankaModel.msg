# FrankaModel.msg
# Message for publishing Franka model matrices

std_msgs/Header header

# Mass matrix (flattened row-major)
float64[] mass_matrix

# Coriolis and nonlinear terms
float64[] coriolis

# Joint torques
float64[] tau

# Interaction Jacobian (flattened row-major)
float64[] ai

# Ai_dot * q_dot (vector)
float64[] ai_dot_q_dot

# Applied forces
float64[] fa

# Dimensions (for reshaping)
uint32 n
