# Testing Guide

Comprehensive testing strategies and tools for the ADG ROS2 system.

## Testing Philosophy

Our testing approach follows the testing pyramid with emphasis on:

- **Unit Tests**: Fast, isolated tests for individual components
- **Integration Tests**: Tests for package interactions
- **System Tests**: End-to-end testing with real or simulated hardware
- **Performance Tests**: Benchmarks and load testing

## Test Organization

### Directory Structure

```
package_name/
├── src/                    # Source code
├── test/                   # All test files
│   ├── unit/              # Unit tests
│   ├── integration/       # Integration tests
│   ├── system/            # System tests
│   ├── performance/       # Performance tests
│   └── fixtures/          # Test data and fixtures
├── launch/
│   └── test/              # Test launch files
└── config/
    └── test/              # Test configurations
```

### Test Naming Convention

```
test_<functionality>_<test_type>.py
```

Examples:
- `test_robot_controller_unit.py`
- `test_task_execution_integration.py`
- `test_teleoperation_system.py`

## Unit Testing

### Python Unit Tests

We use `pytest` for Python unit testing:

#### Basic Test Structure

```python
# test/unit/test_robot_controller_unit.py
import pytest
from unittest.mock import Mock, patch
from robot_arm_interface.controller import RobotController

class TestRobotController:
    
    @pytest.fixture
    def controller(self):
        """Create controller instance for testing."""
        joint_names = ['joint_1', 'joint_2', 'joint_3']
        return RobotController(joint_names)
    
    @pytest.fixture
    def mock_hardware(self):
        """Mock hardware interface."""
        return Mock()
    
    def test_initialization(self, controller):
        """Test controller initialization."""
        assert len(controller.joint_names) == 3
        assert controller.is_initialized() is True
    
    def test_joint_command_validation(self, controller):
        """Test joint command validation."""
        valid_command = [0.1, -0.5, 0.8]
        assert controller.validate_joint_command(valid_command) is True
        
        invalid_command = [10.0, 5.0, -8.0]  # Outside limits
        assert controller.validate_joint_command(invalid_command) is False
    
    @patch('robot_arm_interface.hardware.HardwareInterface')
    def test_command_execution(self, mock_hardware_class, controller):
        """Test command execution with mocked hardware."""
        mock_hardware = Mock()
        mock_hardware_class.return_value = mock_hardware
        
        controller.connect_hardware()
        controller.execute_joint_command([0.1, -0.2, 0.3])
        
        mock_hardware.send_joint_command.assert_called_once()
```

#### Testing Async Code

```python
import pytest
import asyncio
from robot_arm_interface.async_controller import AsyncController

class TestAsyncController:
    
    @pytest.mark.asyncio
    async def test_async_trajectory_execution(self):
        """Test asynchronous trajectory execution."""
        controller = AsyncController()
        trajectory = create_test_trajectory()
        
        result = await controller.execute_trajectory_async(trajectory)
        
        assert result.success is True
        assert result.execution_time > 0
```

#### Parameterized Tests

```python
@pytest.mark.parametrize("joint_angles,expected_valid", [
    ([0.0, 0.0, 0.0], True),
    ([1.0, -1.0, 0.5], True),
    ([10.0, 5.0, -8.0], False),  # Outside limits
    ([float('nan'), 0.0, 0.0], False),  # NaN values
])
def test_joint_validation(controller, joint_angles, expected_valid):
    """Test joint validation with various inputs."""
    assert controller.validate_joints(joint_angles) == expected_valid
```

### C++ Unit Tests

We use Google Test (gtest) for C++ unit testing:

```cpp
// test/unit/test_trajectory_controller_unit.cpp
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "fr3_controllers/trajectory_controller.hpp"

class TrajectoryControllerTest : public ::testing::Test 
{
protected:
    void SetUp() override 
    {
        joint_names_ = {"joint_1", "joint_2", "joint_3"};
        controller_ = std::make_unique<TrajectoryController>(joint_names_);
    }
    
    std::vector<std::string> joint_names_;
    std::unique_ptr<TrajectoryController> controller_;
};

TEST_F(TrajectoryControllerTest, InitializationTest) 
{
    EXPECT_EQ(controller_->get_joint_names().size(), 3);
    EXPECT_TRUE(controller_->is_configured());
}

TEST_F(TrajectoryControllerTest, TrajectoryValidation) 
{
    trajectory_msgs::msg::JointTrajectory valid_trajectory;
    valid_trajectory.joint_names = joint_names_;
    
    // Add valid trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.1, -0.2, 0.3};
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
    valid_trajectory.points.push_back(point);
    
    EXPECT_TRUE(controller_->validate_trajectory(valid_trajectory));
}
```

### Running Unit Tests

```bash
# Python tests
cd ~/ros2_ws/src/package_name
python3 -m pytest test/unit/ -v

# C++ tests (run through colcon)
cd ~/ros2_ws
colcon build --packages-select package_name
colcon test --packages-select package_name --event-handlers console_direct+
```

## Integration Testing

### ROS2 Integration Tests

Test interactions between multiple nodes:

```python
# test/integration/test_robot_arm_integration.py
import unittest
import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
import launch_testing

class TestRobotArmIntegration(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment."""
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment."""
        rclpy.shutdown()
    
    def test_controller_communication(self):
        """Test communication between controller and interface."""
        node = rclpy.create_node('test_node')
        
        # Create publishers/subscribers
        cmd_pub = node.create_publisher(
            trajectory_msgs.msg.JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        status_sub = node.create_subscription(
            control_msgs.msg.JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self.status_callback,
            10
        )
        
        # Send test command
        test_trajectory = self.create_test_trajectory()
        cmd_pub.publish(test_trajectory)
        
        # Wait for response
        rclpy.spin_once(node, timeout_sec=1.0)
        
        # Verify response received
        self.assertIsNotNone(self.last_status)
        node.destroy_node()
    
    def status_callback(self, msg):
        """Callback for status messages."""
        self.last_status = msg

# Launch description for integration test
@launch_testing.parametrize("robot_type", ["gazebo", "mock"])
def generate_test_description(robot_type):
    """Generate launch description for test."""
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robot_arm_bringup', f'{robot_type}.launch.py'],
            output='screen'
        ),
        ReadyToTest()
    ])
```

### Service Integration Tests

```python
import rclpy
from rclpy.node import Node
import pytest
from std_srvs.srv import Trigger

class TestServiceIntegration:
    
    @pytest.fixture
    def test_node(self):
        """Create test node."""
        rclpy.init()
        node = rclpy.create_node('test_client')
        yield node
        node.destroy_node()
        rclpy.shutdown()
    
    def test_emergency_stop_service(self, test_node):
        """Test emergency stop service call."""
        client = test_node.create_client(Trigger, '/emergency_stop')
        
        # Wait for service
        assert client.wait_for_service(timeout_sec=5.0)
        
        # Call service
        request = Trigger.Request()
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(test_node, future)
        
        # Verify response
        response = future.result()
        assert response.success is True
```

## System Testing

### End-to-End Tests

Test complete workflows from user input to robot action:

```python
# test/system/test_complete_workflow.py
import pytest
import rclpy
from robot_arm_client.client import RobotArmClient
from robot_tasks.task_executor import TaskExecutor

@pytest.mark.system
class TestCompleteWorkflow:
    
    def setup_method(self):
        """Set up for each test method."""
        rclpy.init()
        self.client = RobotArmClient()
        self.task_executor = TaskExecutor()
    
    def teardown_method(self):
        """Clean up after each test method."""
        rclpy.shutdown()
    
    def test_pick_and_place_workflow(self):
        """Test complete pick and place operation."""
        # Connect to robot
        assert self.client.connect(timeout=10.0)
        
        # Home robot
        result = self.client.home_robot()
        assert result.success
        
        # Execute pick and place task
        task_config = {
            'object_pose': [0.5, 0.0, 0.02],
            'target_pose': [0.3, 0.3, 0.1]
        }
        
        result = self.task_executor.execute_task('pick_and_place', task_config)
        assert result.success
        
        # Verify final position
        final_pose = self.client.get_end_effector_pose()
        expected_pose = task_config['target_pose']
        
        assert self.poses_close(final_pose, expected_pose, tolerance=0.01)
```

### Hardware-in-the-Loop Tests

```python
@pytest.mark.hardware
@pytest.mark.skipif(not hardware_available(), reason="Hardware not available")
class TestHardwareIntegration:
    
    def test_real_robot_connection(self):
        """Test connection to real robot hardware."""
        from robot_arm_interface.hardware_interface import FrankaInterface
        
        interface = FrankaInterface()
        assert interface.connect(robot_ip="192.168.1.100")
        assert interface.is_connected()
        
        # Test basic communication
        joint_states = interface.get_joint_states()
        assert len(joint_states.position) == 7
```

## Performance Testing

### Latency Testing

```python
# test/performance/test_control_latency.py
import time
import statistics
import pytest
from robot_arm_interface.controller import RobotController

class TestControlLatency:
    
    def test_command_latency(self):
        """Test command processing latency."""
        controller = RobotController()
        latencies = []
        
        for _ in range(100):
            start_time = time.perf_counter()
            
            # Send command
            controller.send_joint_command([0.1, -0.2, 0.3])
            
            # Wait for acknowledgment
            controller.wait_for_command_ack()
            
            end_time = time.perf_counter()
            latencies.append((end_time - start_time) * 1000)  # ms
        
        avg_latency = statistics.mean(latencies)
        max_latency = max(latencies)
        
        # Assert performance requirements
        assert avg_latency < 10.0  # ms
        assert max_latency < 50.0  # ms
```

### Throughput Testing

```python
def test_command_throughput():
    """Test maximum command throughput."""
    controller = RobotController()
    
    num_commands = 1000
    start_time = time.time()
    
    for i in range(num_commands):
        command = [0.1 * i, -0.1 * i, 0.05 * i]
        controller.send_joint_command(command)
    
    end_time = time.time()
    duration = end_time - start_time
    throughput = num_commands / duration
    
    # Should handle at least 100 commands per second
    assert throughput >= 100.0
```

### Memory and Resource Testing

```python
import psutil
import gc

def test_memory_usage():
    """Test memory usage during operation."""
    process = psutil.Process()
    initial_memory = process.memory_info().rss
    
    # Run intensive operation
    controller = RobotController()
    for _ in range(1000):
        controller.complex_operation()
    
    # Force garbage collection
    gc.collect()
    
    final_memory = process.memory_info().rss
    memory_increase = final_memory - initial_memory
    
    # Memory increase should be reasonable (< 50MB)
    assert memory_increase < 50 * 1024 * 1024
```

## Test Configuration

### pytest Configuration

```ini
# pytest.ini
[tool:pytest]
testpaths = test
python_files = test_*.py
python_classes = Test*
python_functions = test_*
addopts = 
    -v
    --tb=short
    --strict-markers
markers =
    unit: Unit tests
    integration: Integration tests
    system: System tests
    performance: Performance tests
    hardware: Tests requiring real hardware
    slow: Slow running tests
```

### Test Environment Setup

```yaml
# test/config/test_environment.yaml
test_robot:
  type: "mock"
  joint_names: ["joint_1", "joint_2", "joint_3"]
  joint_limits:
    joint_1: [-3.14, 3.14]
    joint_2: [-1.57, 1.57] 
    joint_3: [-2.09, 2.09]

mock_hardware:
  simulate_latency: true
  latency_ms: 1.0
  add_noise: true
  noise_std: 0.001
```

## Continuous Integration

### GitHub Actions Workflow

```yaml
# .github/workflows/test.yml
name: Test Suite
on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    strategy:
      matrix:
        ros_distro: [humble]
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Setup ROS2
      uses: ros-tooling/setup-ros@v0.3
      with:
        required-ros-distributions: ${{ matrix.ros_distro }}
    
    - name: Install dependencies
      run: |
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
    
    - name: Build packages
      run: colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
    
    - name: Run tests
      run: |
        source install/setup.bash
        colcon test --event-handlers console_direct+
        colcon test-result
```

## Test Data Management

### Test Fixtures

```python
# test/fixtures/trajectory_fixtures.py
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryFixtures:
    
    @staticmethod
    def simple_trajectory():
        """Create simple test trajectory."""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']
        
        point = JointTrajectoryPoint()
        point.positions = [0.1, -0.2, 0.3]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 1
        
        trajectory.points = [point]
        return trajectory
    
    @staticmethod
    def complex_trajectory():
        """Create complex multi-point trajectory."""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']
        
        # Generate sinusoidal trajectory
        for i in range(10):
            point = JointTrajectoryPoint()
            t = i * 0.1
            point.positions = [
                0.5 * np.sin(2 * np.pi * t),
                0.3 * np.cos(2 * np.pi * t),
                0.2 * np.sin(4 * np.pi * t)
            ]
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)
        
        return trajectory
```

## Mock and Simulation

### Mock Hardware Interface

```python
# test/mocks/mock_hardware.py
class MockHardwareInterface:
    """Mock hardware interface for testing."""
    
    def __init__(self):
        self.joint_positions = [0.0] * 7
        self.is_connected = False
        self.command_history = []
    
    def connect(self, robot_ip=None):
        """Simulate connection to robot."""
        self.is_connected = True
        return True
    
    def send_joint_command(self, positions):
        """Simulate sending joint command."""
        if not self.is_connected:
            raise RuntimeError("Not connected to hardware")
        
        self.command_history.append(positions)
        # Simulate command execution
        self.joint_positions = positions.copy()
    
    def get_joint_states(self):
        """Get current joint states."""
        return {
            'positions': self.joint_positions.copy(),
            'velocities': [0.0] * 7,
            'efforts': [0.0] * 7
        }
```

## Test Utilities

### Custom Assertions

```python
# test/utils/test_assertions.py
def assert_poses_close(pose1, pose2, position_tolerance=0.01, 
                      orientation_tolerance=0.1):
    """Assert that two poses are close within tolerance."""
    import numpy as np
    
    # Check position
    pos_diff = np.linalg.norm(
        np.array(pose1.position) - np.array(pose2.position)
    )
    assert pos_diff < position_tolerance, \
        f"Position difference {pos_diff} > {position_tolerance}"
    
    # Check orientation (quaternion distance)
    q1 = np.array([pose1.orientation.x, pose1.orientation.y, 
                   pose1.orientation.z, pose1.orientation.w])
    q2 = np.array([pose2.orientation.x, pose2.orientation.y,
                   pose2.orientation.z, pose2.orientation.w])
    
    dot_product = np.abs(np.dot(q1, q2))
    angle_diff = 2 * np.arccos(np.clip(dot_product, 0, 1))
    
    assert angle_diff < orientation_tolerance, \
        f"Orientation difference {angle_diff} > {orientation_tolerance}"
```

## Running Tests

### Local Testing

```bash
# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select robot_arm_interface

# Run with output
colcon test --event-handlers console_direct+

# Run specific test types
python3 -m pytest -m unit
python3 -m pytest -m integration
python3 -m pytest -m "not slow"

# Run with coverage
python3 -m pytest --cov=robot_arm_interface test/
```

### Test Reports

```bash
# Generate test results summary
colcon test-result --verbose

# Generate coverage report
coverage html
open htmlcov/index.html
```

## Best Practices

### Test Writing Guidelines

1. **Test Isolation**: Each test should be independent
2. **Clear Test Names**: Names should describe what is being tested
3. **Arrange-Act-Assert**: Structure tests clearly
4. **Test Edge Cases**: Include boundary conditions and error cases
5. **Mock External Dependencies**: Use mocks for external systems

### Performance Testing Guidelines

1. **Baseline Measurements**: Establish performance baselines
2. **Consistent Environment**: Use consistent test environments
3. **Statistical Significance**: Run multiple iterations
4. **Resource Monitoring**: Monitor CPU, memory, and network usage

### CI/CD Integration

1. **Fail Fast**: Run fast tests first
2. **Parallel Execution**: Run tests in parallel where possible
3. **Test Reports**: Generate comprehensive test reports
4. **Quality Gates**: Set quality thresholds for merge approval

This comprehensive testing strategy ensures the reliability, performance, and maintainability of the ADG ROS2 system.
