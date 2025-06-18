# FrankaModel.msg
# Message for publishing Franka model matrices

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

# Contact force estimation
float64[] f_ext_estimated  # Estimated external forces (6D wrench: fx, fy, fz, mx, my, mz)
float64[] f_ext_robot      # Robot's own force estimates (6D wrench)

# Dimensions (for reshaping)
uint32 n
