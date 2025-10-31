# FrankaModel.msg
# Message for publishing Franka model matrices

std_msgs/Header header

# --- Kinematics
# System coordinates (joint positions)
float64[] q

# System velocity (joint velocities)
float64[] q_dot

# Interface position
float64[] x_ee

# Interface velocity
float64[] v_ee

# --- Dynamics
# Mass matrix (flattened row-major)
float64[] mass_matrix

# Coriolis and nonlinear terms
float64[] coriolis

# Joint torques
float64[] tau

# Driving joint torques
float64[] tau_d

# External joint torques
float64[] tau_ext

# Interaction Jacobian (flattened row-major)
float64[] ai

# Ai_dot * q_dot (vector)
float64[] ai_dot_q_dot

# External forces, at end-effector
float64[] f_ext_ee 

# Driving end-effector forces (from controller)
float64[] f_d_ee

# Applied forces
float64[] f_a

# Dimensions (for reshaping)
uint32 n
