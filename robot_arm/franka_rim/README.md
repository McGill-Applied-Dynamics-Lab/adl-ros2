# franka_rim
ROS 2 package to compute the Reduced Interface Model (RIM) for the Franka Research 3 arm.

## Nodes
- franka_model_node: Computes and publishes the robot model (mass matrix, coriolis, etc.)
- franka_rim_node: Computes and publishes the RIM based on the robot model

```
ros2 launch franka_rim franka_rim.launch.py delay:=5g
```


