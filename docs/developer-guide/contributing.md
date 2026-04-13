# Contributing

Thank you for your interest in contributing to the project! 
This guide will help you get started with contributing to the code, documentation, and other improvements.

## Getting Started

### Development Setup

1. Follow the [getting started guide](getting-started.md)

*Any other setup required for devs?*


## Contribution Workflow

### 1. Create a Feature Branch

```bash
# Update your main branch
git checkout main
git pull upstream main

# Create feature branch
git checkout -b feature/your-feature-name
```

### 2. Make Your Changes

Follow our coding standards (see below) and make your changes. Make sure to:

- Update documentation. Look at the [documentation guide](documentation.md#development).
- Add tests for new functionality


### 3. Test Your Changes
*Coming soon ...*

??? warning "outdated"
    ```bash
    # Build and test
    colcon build --packages-select YOUR_PACKAGE
    colcon test --packages-select YOUR_PACKAGE

    # Run static analysis
    pre-commit run --all-files

    # Test documentation build
    cd docs && mkdocs build
    ```

### 4. Submit a Pull Request

```bash
# Push your changes
git push origin feature/your-feature-name

# Create PR on GitHub
# Include detailed description of changes
# Reference any related issues
```

## Coding Standards

### Python Code Style

We follow PEP 8 with some modifications:

```python
# Use type hints
def process_joint_states(states: List[float]) -> bool:
    """
    Process joint states and return success status.
    
    Args:
        states: List of joint positions in radians
        
    Returns:
        True if processing succeeded, False otherwise
    """
    pass

# Use docstrings for all public functions/classes
class RobotController:
    """Robot controller for arm manipulation.
    
    This class provides high-level control interface for robot arms,
    including trajectory planning and execution monitoring.
    
    Attributes:
        joint_names: List of joint names
        current_state: Current robot state
    """
    
    def __init__(self, joint_names: List[str]):
        """Initialize controller with joint configuration."""
        self.joint_names = joint_names
```

### C++ Code Style

We follow the ROS2 C++ style guide:

```cpp
// Use snake_case for functions and variables
// Use PascalCase for classes
class TrajectoryController : public rclcpp::Node
{
public:
  explicit TrajectoryController(const rclcpp::NodeOptions & options);
  
  /// @brief Execute trajectory command
  /// @param trajectory Joint trajectory to execute
  /// @return True if trajectory was accepted
  bool execute_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory);

private:
  // Member variables with trailing underscore
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::vector<std::string> joint_names_;
};
```

### Documentation Standards

#### Python Docstrings

Use Google-style docstrings:

```python
def calculate_inverse_kinematics(target_pose: Pose, current_joints: List[float]) -> Optional[List[float]]:
    """Calculate inverse kinematics solution.
    
    Args:
        target_pose: Desired end-effector pose
        current_joints: Current joint configuration for warm start
        
    Returns:
        Joint angles that achieve target pose, or None if no solution exists
        
    Raises:
        ValueError: If target pose is unreachable
        RuntimeError: If kinematics solver fails
        
    Example:
        >>> joints = calculate_inverse_kinematics(target, current)
        >>> if joints is not None:
        ...     controller.move_to_joints(joints)
    """
    pass
```

#### Markdown Documentation
See [Zensical authoring guide](https://zensical.org/docs/authoring/markdown/) for tips on how to write in markdown. 

## Testing Guidelines
*Coming soon...*


## Getting Help

<!-- ### Communication Channels

- **GitHub Issues**: Bug reports and feature requests
- **GitHub Discussions**: Questions and general discussion
- **Pull Request Comments**: Code-specific discussions -->

### Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/)
- [Python Style Guide](https://pep8.org/)
- [Git Best Practices](https://git-scm.com/book)

