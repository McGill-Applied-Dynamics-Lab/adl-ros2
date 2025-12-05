# Teleoperation message for either position or velocity command of the robot end-effector

std_msgs/Header header

uint8 CONTROL_MODE_POSITION=0
uint8 CONTROL_MODE_VEL=1
uint8 control_mode # 0: position, 1: velocity

geometry_msgs/Pose ee_des  # Desired position of the end-effector
geometry_msgs/Twist ee_vel_des  # Desired velocity of the end-effector
