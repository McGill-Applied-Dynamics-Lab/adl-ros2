# ADG ROS2 Utils

Core utilities and debugging tools for the ADG ROS2 ecosystem.

## Overview

The `adg_ros2_utils` package provides essential utilities and debugging tools that are shared across all other packages in the ADG ROS2 system. This package serves as the foundation for common functionality.

## Features

- **Debug Utilities**: Remote debugging support for ROS2 nodes
- **Common Interfaces**: Shared data structures and utilities
- **Logging Helpers**: Enhanced logging capabilities

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

## Installation

The package is automatically installed when you build the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select adg_ros2_utils
```

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

## Dependencies

- **ROS2 Humble**: Core ROS2 functionality
- **debugpy**: Python debugging support
- **rclpy**: ROS2 Python client library

## Contributing

This package follows the project-wide contributing guidelines. See the [contributing guide](../developer-guide/contributing.md) for details.

## License

Licensed under the same terms as the main ADG ROS2 project.
