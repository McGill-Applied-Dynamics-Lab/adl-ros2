# ADG ROS2 Utils

Core utilities and debugging tools for the ADG ROS2 ecosystem.

For now, mainly for remote debugging of ROS2 nodes

## Package Structure
```
adg_ros2_utils/
├── adg_ros2_utils/
│   ├── __init__.py
│   └── debug_utils.py      # Debug utilities and remote debugging
├── package.xml             # Package metadata
├── setup.py               # Python package setup
└── setup.cfg              # Package configuration
```

## Key Components

### Debug Utils (`debug_utils.py`)

Provides debugging utilities for ROS2 nodes, including remote debugging support.

#### Functions

::: adg_ros2_utils.debug_utils.wait_for_debugger


## Usage

### Remote Debugging
To enable remote debugging for a specific node:

```bash
# Set environment variables
export DEBUG_NODE="your_node_name"
export DEBUG_PORT=5678

# Launch your node
ros2 run your_package your_node
```

In your Python code:

```python
from adg_ros2_utils.debug_utils import wait_for_debugger

def main():
    rclpy.init()
    node = YourNode()
    
    # Wait for debugger if this node matches DEBUG_NODE
    wait_for_debugger(node.get_name())
    
    rclpy.spin(node)
    rclpy.shutdown()
```

#### In a VsCode Launch Configuration
```json
{
    "configurations": [
        {
                    "name": "Python Debugger: Remote Attach",
                    "type": "debugpy",
                    "request": "attach",
                    "connect": {
                        "host": "localhost",
                        "port": 5678
                    },
                    "pathMappings": [
                        {
                            "localRoot": "${workspaceFolder}",
                            "remoteRoot": "."
                        }
                    ]
        },
        {
                    "name": "Launch: franka_rim.launch.py",
                    "type": "debugpy",
                    "request": "launch",
                    "program": "/opt/ros/humble/bin/ros2",
                    "args": [
                        "launch",
                        "franka_rim",
                        "franka_rim.launch.py",
                        "fake_i3:=false",
                        "save_data:=false",
                    ],
                    "console": "integratedTerminal",
                    "env": {
                        "DEBUG_NODE": "franka_rim_node", // Node name to debug
                    }
        },
    ]
}
```