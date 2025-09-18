# Franka Server

The *Franka Server* consists of group of ROS2 nodes sending the low level torque commands to the *Franka Controller*. 
The main component is the ros2 controller.

> HINT: **Resources.**
>
> - [franka-server: GitHub Repo](https://github.com/McGill-Applied-Dynamics-Group/franka-server)
> - [IsaacSim Documentation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/index.html)
> - [IsaacSim API](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/index.html)

## Testing `libfranka`
Libfranka is the low level c++ api to send commands to the robot. 
Test that you are able to connect to the robot:
```
cd ~/git/libfranka/build
./examples/echo_robot_state $FR3_IP
```

To check the realtime is working:
```
# This moves the robot home
./examples/communication_test $FR3_IP
```

Other examples are available. You can take a look with 
```
ls examples
```

## 


## Simulation Setup
1. [Install Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/index.html) (Only version 4.5.0 was tested)


## Moving the robot
*Should this be moved to the *

1. launch the server
```
ros2 launch franka_server franka.launch.py robot_ip:=$FR3_IP
```
TODO: Launch with Docker (create the .env file, docker up ...)

This starts the ros2 controller (*osc_pd_controller* by default) and the franka_hand_server. 

2. Should be in impedance mode, you can try pushing it around you you'll feel the force of the 'control spring'


3. We provide a few utilities scripts to move the robot

- Move with RVIZ    
- Go Home
- Change Controller
- Set collision behaviour
- Gripper

## Controllers Validation

