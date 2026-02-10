# API Reference

This section contains the API documentation automatically generated from Python docstrings.

## Available Packages

The following packages provide Python APIs with docstring documentation:

### Core Utilities

#### ADG ROS2 Utils
Core debugging and utility functions.

::: adg_ros2_utils.debug_utils

### Robot Arm System

The robot arm system includes several packages with Python interfaces. Due to the current package structure, most of the robot arm functionality is implemented in C++ controllers, but Python interfaces are provided for:

- Robot arm client libraries
- Configuration management
- Task integration

### Robot Tasks

High-level task execution and reinforcement learning capabilities.

#### Core Task Framework

```python
# Example usage of task framework
from robot_tasks.task_executor import TaskExecutor

executor = TaskExecutor()
result = executor.execute_task('lift_object', task_config)
```

### Teleoperation

Teleoperation interfaces and utilities.

#### Core Teleop Functionality

The teleoperation system provides Python APIs for:

- Input device management
- Command processing
- Safety monitoring
- Network communication

## Using the API Documentation

This documentation is automatically generated from docstrings in the Python source code. Each function, class, and method includes:

- **Purpose**: What the function/class does
- **Parameters**: Input parameters and their types
- **Returns**: Return values and types
- **Examples**: Usage examples where available
- **Raises**: Exceptions that may be raised

## Docstring Format

All Python code follows Google-style docstrings:

```python
def example_function(param1: str, param2: int = 0) -> bool:
    """
    Short description of the function.
    
    Longer description if needed. This can span multiple lines
    and provide more detailed information about the function's
    behavior and use cases.
    
    Args:
        param1: Description of the first parameter
        param2: Description of the second parameter with default value
        
    Returns:
        Description of the return value
        
    Raises:
        ValueError: When param1 is empty
        RuntimeError: When operation fails
        
    Example:
        >>> result = example_function("hello", 42)
        >>> print(result)
        True
    """
    if not param1:
        raise ValueError("param1 cannot be empty")
    
    return len(param1) > param2
```

## Package Structure

For detailed information about package organization and architecture, see:

- [Package Overview](../packages/index.md)
- [System Architecture](../developer-guide/architecture.md)
- [Contributing Guide](../developer-guide/contributing.md)

## Need More Examples?

Check out the [Tutorials](../tutorials/index.md) section for step-by-step examples of using these APIs in real applications.
