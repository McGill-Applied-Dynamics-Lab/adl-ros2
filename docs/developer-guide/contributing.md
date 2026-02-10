# Contributing

Thank you for your interest in contributing to the ADG ROS2 project! This guide will help you get started with contributing code, documentation, and other improvements.

## Getting Started

### Development Setup

1. **Fork the Repository**
   ```bash
   # Fork on GitHub, then clone your fork
   git clone https://github.com/YOUR_USERNAME/adg-ros2.git
   cd adg-ros2
   
   # Add upstream remote
   git remote add upstream https://github.com/McGill-Applied-Dynamics-Group/adg-ros2.git
   ```

2. **Set Up Development Environment**
   ```bash
   # Create development workspace
   mkdir -p ~/dev_ws/src
   cd ~/dev_ws/src
   ln -s /path/to/your/adg-ros2 ./
   
   # Install dependencies
   cd ~/dev_ws
   rosdep install --from-paths src --ignore-src -r -y
   
   # Build in debug mode
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
   ```

3. **Install Development Tools**
   ```bash
   # Code formatting and linting
   pip install pre-commit black flake8 mypy
   
   # C++ tools
   sudo apt install clang-format cppcheck
   
   # Documentation tools
   pip install sphinx sphinx-rtd-theme
   ```

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

- Write clear, descriptive commit messages
- Add tests for new functionality
- Update documentation as needed
- Follow the existing code style

### 3. Test Your Changes

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

- Use clear headings and structure
- Include code examples with syntax highlighting
- Add cross-references to related sections
- Use diagrams where helpful (Mermaid preferred)

### Commit Message Format

Use conventional commit format:

```
type(scope): short description

Longer description if needed.

- List specific changes
- Reference issues: Fixes #123
- Co-authored-by: Name <email>
```

Types: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

Examples:
```
feat(robot_arm): add trajectory smoothing filter

fix(teleop): resolve joystick connection timeout

docs(tutorials): add RL integration example
```

## Testing Guidelines

### Unit Tests

Write comprehensive unit tests:

```python
# Python tests using pytest
import pytest
from robot_arm_interface.controller import RobotController

class TestRobotController:
    @pytest.fixture
    def controller(self):
        return RobotController(['joint_1', 'joint_2'])
    
    def test_joint_validation(self, controller):
        """Test that invalid joint positions are rejected."""
        invalid_positions = [10.0, 5.0]  # Outside limits
        assert not controller.validate_joint_positions(invalid_positions)
    
    def test_trajectory_execution(self, controller):
        """Test trajectory execution with valid inputs."""
        trajectory = create_test_trajectory()
        assert controller.execute_trajectory(trajectory)
```

```cpp
// C++ tests using gtest
#include <gtest/gtest.h>
#include "robot_arm_interface/controller.hpp"

class ControllerTest : public ::testing::Test {
protected:
  void SetUp() override {
    controller_ = std::make_unique<RobotController>(joint_names_);
  }
  
  std::vector<std::string> joint_names_{"joint_1", "joint_2"};
  std::unique_ptr<RobotController> controller_;
};

TEST_F(ControllerTest, ValidateJointLimits) {
  std::vector<double> invalid_positions{10.0, 5.0};
  EXPECT_FALSE(controller_->validate_joint_positions(invalid_positions));
}
```

### Integration Tests

Test package interactions:

```bash
# Launch test for integration testing
ros2 launch robot_arm_testing integration_test.launch.py
```

## Review Process

### What We Look For

1. **Functionality**: Does it work as intended?
2. **Code Quality**: Is it readable, maintainable, and efficient?
3. **Testing**: Are there adequate tests?
4. **Documentation**: Is it well documented?
5. **Compatibility**: Does it work with existing code?

### Review Timeline

- **Initial Response**: Within 2 business days
- **Full Review**: Within 1 week for most PRs
- **Large Features**: May take longer, will provide timeline

### Addressing Feedback

- Respond to all review comments
- Make requested changes in new commits (don't squash during review)
- Re-request review after addressing feedback

## Release Process

### Version Numbering

We use semantic versioning (MAJOR.MINOR.PATCH):

- **MAJOR**: Breaking changes
- **MINOR**: New features, backward compatible
- **PATCH**: Bug fixes, backward compatible

### Release Branches

- `main`: Development branch
- `release/vX.Y`: Release preparation
- `hotfix/vX.Y.Z`: Critical fixes

## Getting Help

### Communication Channels

- **GitHub Issues**: Bug reports and feature requests
- **GitHub Discussions**: Questions and general discussion
- **Pull Request Comments**: Code-specific discussions

### Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/)
- [Python Style Guide](https://pep8.org/)
- [Git Best Practices](https://git-scm.com/book)

## Recognition

Contributors will be:
- Listed in `CONTRIBUTORS.md`
- Credited in release notes
- Invited to join the maintainers team for significant contributions

Thank you for contributing to ADG ROS2! 🚀
