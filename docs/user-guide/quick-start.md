# Quick Start

Get up and running with ADG ROS2 in 5 minutes! This guide assumes you have already completed the [installation](installation.md).

## Launch Your First Robot Simulation

### 1. Start the Robot Simulation

Open a terminal and launch the robot arm simulation:

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Launch robot simulation with Gazebo
ros2 launch robot_arm_bringup gazebo.launch.py
```

This will start:
- Gazebo simulator with the Franka FR3 robot
- Robot state publisher
- Joint controllers
- Basic teleoperation interface

### 2. Control the Robot

In a new terminal, try some basic controls:

```bash
# Move robot to home position
ros2 service call /robot_arm_controller/home std_srvs/srv/Trigger

# Check robot status
ros2 topic echo /robot_arm_controller/status
```

### 3. Use Teleoperation

Start the teleoperation interface:

```bash
# Launch keyboard teleoperation
ros2 run teleop teleop_keyboard

# Follow the on-screen instructions to control the robot
```

## Run a Simple Task

### 1. Launch Task Environment

```bash
# Start the lift task environment
ros2 launch robot_tasks lift_task.launch.py
```

### 2. Run Manual Task Execution

```bash
# Execute a pre-programmed lift sequence
ros2 run robot_tasks execute_lift_task.py
```

### 3. Run RL Agent (if trained)

```bash
# Run a trained reinforcement learning agent
ros2 run robot_tasks run_trained_agent.py --task=lift
```

## Explore the System

### Check Available Topics

```bash
# List all active topics
ros2 topic list

# Key topics to watch:
ros2 topic echo /joint_states                    # Joint positions
ros2 topic echo /robot_arm_controller/status     # Controller status
ros2 topic echo /task_manager/current_task       # Active tasks
```

### Monitor Robot State

```bash
# View robot in RViz
ros2 launch robot_arm_bringup rviz.launch.py

# Check diagnostics
ros2 run rqt_robot_monitor rqt_robot_monitor
```

## Example Workflows

### Basic Robot Movement

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
    def move_to_position(self, positions):
        """Move robot to specified joint positions."""
        msg = JointTrajectory()
        msg.joint_names = [f'joint_{i+1}' for i in range(7)]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2
        
        msg.points = [point]
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = SimpleRobotController()
    
    # Move to a safe position
    safe_position = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    controller.move_to_position(safe_position)
    
    rclpy.spin_once(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save this as `simple_move.py` and run:
```bash
python3 simple_move.py
```

### Task Execution

```python
#!/usr/bin/env python3
import rclpy
from robot_tasks.task_executor import TaskExecutor

def main():
    rclpy.init()
    
    # Create task executor
    executor = TaskExecutor()
    
    # Define a simple lift task
    task_config = {
        'task_type': 'lift',
        'object_position': [0.5, 0.0, 0.02],
        'lift_height': 0.2,
        'approach_distance': 0.1
    }
    
    # Execute the task
    success = executor.execute_task('lift_object', task_config)
    
    if success:
        print("Task completed successfully!")
    else:
        print("Task failed!")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Debugging Tips

### Use Debug Utilities

The ADG ROS2 utils package provides debugging tools:

```bash
# Enable debug mode for a specific node
export DEBUG_NODE="robot_arm_controller"
export DEBUG_PORT=5678

# Launch with debugging
ros2 run robot_arm_interface controller_node
```

### Common Commands

```bash
# Check node status
ros2 node list
ros2 node info /robot_arm_controller

# Monitor system performance
ros2 run rqt_top rqt_top

# View computation graph
ros2 run rqt_graph rqt_graph
```

## What's Next?

Now that you have the basic system running:

1. **Learn the Architecture**: Read about the [system architecture](../developer-guide/architecture.md)
2. **Explore Packages**: Check out individual [package documentation](../packages/index.md)
3. **Advanced Tutorials**: Try more complex [tutorials](../tutorials/index.md)
4. **API Reference**: Dive into the [API documentation](../reference/)

## Need Help?

- **Common Issues**: Check the [troubleshooting section](installation.md#troubleshooting)
- **Examples**: Browse more examples in the `/scripts/examples/` directory
- **Community**: Join our [GitHub Discussions](https://github.com/McGill-Applied-Dynamics-Group/adg-ros2/discussions)

Happy robot programming! 🤖
