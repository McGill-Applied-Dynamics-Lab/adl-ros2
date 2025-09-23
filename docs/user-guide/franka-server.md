# Franka Server

The *Franka Server* consists of group of ROS2 nodes sending the low level torque commands to the *Franka Controller*. 
The main component is the ros2 controller.

> HINT: **Resources.**
>
> - [franka-server: GitHub Repo](https://github.com/McGill-Applied-Dynamics-Group/franka-server)
> - [IsaacSim Documentation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/index.html)
> - [IsaacSim API](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/index.html)

The following steps assume you have access to the shell and files of the FrankaPC, either directly or via ssh.

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


## Starting the Server
1. Launch the server

```
fr3-launch
# Alias for: ros2 launch franka_server franka.launch.py robot_ip:=$FR3_IP
```

TODO: Launch with Docker (create the .env file, docker up ...) (*when we get the new PC*)

This starts the ros2 controller (*osc_pd_controller* by default) and the franka_hand_server. 

2. Should be in impedance mode, you can try pushing it around you you'll feel the force of the 'control spring'

3. You can look at the available topics and robot information
    ```
    ros2 topic list
    ros2 topic echo /f3/...
    ```

    > WARNING: **Communication Constraints**
    >
    > When too many packets to the controller are dropped, the server is shut down.. just restart it!

4. You can directly send position commands with:

    ```
    ros2 topic pub --once /osc_pd_controller/goal geometry_msgs/msg/PointStamped "{position: {x: 0.3085, y: 0.0, z: 0.4854}}"
    ```

    > TIP: Big position steps
    > Watch out sending big position steps, the gains are set pretty high. Safeties will shut it down though if the joint velocities/torques, end-effector position or even contact forces reach certain threshold


5. We provide a few utilities scripts to move the robot

    TODO: Add scripts and how to start them

    - Move with RVIZ    
    - Go Home
    - Change Controller
    - Set collision behaviour
    - Gripper


## Accessing the Source Code
To modify/read the source code, we recommend using 
[remote development using SSH VsCode functionality](https://code.visualstudio.com/docs/remote/ssh).

This way, you can open, modify and build the code on the FrankaPC from your machine. 


## Available Controllers
It's in the Franka Server that the low level controllers are implemented using 
[ros2 control](https://control.ros.org/rolling/index.html).

We also have the [CRISP PY](https://utiasdsl.github.io/crisp_controllers/getting_started/) controllers. 

It's currently hard to modify add new controllers because of the desktop limitations. 

TODO: Add available controllers, math, their configurations, ... to the franka-server package doc


## Simulation Setup
*Eventually, it would be nice to have a simulated environment of the arm in IsaacSim*

1. [Install Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/index.html)
...