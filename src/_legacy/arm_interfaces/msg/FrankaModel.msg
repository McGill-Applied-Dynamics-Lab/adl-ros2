# FrankaModel.msg
# Message for publishing Franka model matrices

std_msgs/Header header

# System coordinates (joint positions)
float64[] q

# System velocity (joint velocities)
float64[] q_dot

# Interface position
float64[] x_ee

# Interface velocity
float64[] v_ee

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
