# robot_arm
Collection of ros2 packages to control robotic arms using python.

Supported arms:
- Franka Research 3
- Vectis?


## Packages
- `robot_arm_interface`: High level controllers
- `robot_arm_client`: To call the commands of the interface
- `robot_arm_bringup`: Launch files
- `fr3_controllers`: `ros_control` controllers 
- `arm_interfaces`: Message, action and service files
- `isaac_sim_ros`: To convert joint names between Isaac Sim and Franka URDF model
- `franka_rim`: To compute the RIM of the FR3

## Installation




## Controllers
### OSC PD Controller
To test goal
```bash
ros2 topic pub --once /osc_pd_controller/goal geometry_msgs/msg/PointStamped "{point: {x: 0.309, y: 0.0, z: 0.488}}"
```