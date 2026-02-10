import pytest
import numpy as np
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from arm_interfaces.msg import Teleop
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaRobotState

from robot_arm_interface.fr3_interface import FR3Interface, GripperState


@pytest.fixture(scope="module")
def ros_context():
    """Initialize and shutdown ROS context"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def fr3_interface(ros_context):
    """Create an instance of FR3Interface for testing"""
    interface = FR3Interface(hw_type="fake")
    
    # Start spinning in a separate thread
    spin_thread = Thread(target=lambda: rclpy.spin(interface))
    spin_thread.daemon = True
    spin_thread.start()
    
    # Wait for initialization
    time.sleep(1.0)
    
    # Return the interface for testing
    yield interface
    
    # Clean up
    interface.destroy_node()
    time.sleep(0.5)


def test_pose_msg_to_array(fr3_interface):
    # Test the static utility method
    test_pose = Pose()
    test_pose.position.x = 1.0
    test_pose.position.y = 2.0
    test_pose.position.z = 3.0
    test_pose.orientation.x = 0.0
    test_pose.orientation.y = 0.0
    test_pose.orientation.z = 0.0
    test_pose.orientation.w = 1.0
    
    position, orientation = fr3_interface.pose_msg_to_array(test_pose)
    
    np.testing.assert_array_equal(position, np.array([1.0, 2.0, 3.0]))
    np.testing.assert_array_equal(orientation, np.array([0.0, 0.0, 0.0, 1.0]))


def test_twist_msg_to_array(fr3_interface):
    # Test the static utility method
    test_twist = Twist()
    test_twist.linear.x = 0.1
    test_twist.linear.y = 0.2
    test_twist.linear.z = 0.3
    test_twist.angular.x = 0.4
    test_twist.angular.y = 0.5
    test_twist.angular.z = 0.6
    
    linear, angular = fr3_interface.twist_msg_to_array(test_twist)
    
    np.testing.assert_array_equal(linear, np.array([0.1, 0.2, 0.3]))
    np.testing.assert_array_equal(angular, np.array([0.4, 0.5, 0.6]))


@pytest.mark.unit
def test_teleop_command_position_mode(fr3_interface):
    # Set up a teleop message in position control mode
    teleop_msg = Teleop()
    teleop_msg.control_mode = Teleop.CONTROL_MODE_POSITION
    teleop_msg.ee_des.position.x = 0.5
    teleop_msg.ee_des.position.y = 0.1
    teleop_msg.ee_des.position.z = 0.3
    teleop_msg.ee_des.orientation.w = 1.0
    
    # Manually set the current EE pose
    fr3_interface.o_t_ee = teleop_msg._create_default_instance()  # Create a default instance
    fr3_interface.o_t_ee.pose.position.x = 0.45  # Slightly different
    fr3_interface.o_t_ee.pose.position.y = 0.09
    fr3_interface.o_t_ee.pose.position.z = 0.32
    fr3_interface.o_t_ee.pose.orientation.w = 1.0
    
    # Set desired end effector twist as zero
    fr3_interface.o_dp_ee_d = teleop_msg._create_default_instance()
    
    # Set the teleop message
    fr3_interface._teleop_goal_msg = teleop_msg
    
    # Compute desired velocity based on teleop command
    desired_vel = fr3_interface._compute_teleop_cmd()
    
    # Verify desired velocity is in the right direction
    assert desired_vel[0] > 0, "X velocity should be positive"
    assert desired_vel[1] > 0, "Y velocity should be positive"
    assert desired_vel[2] < 0, "Z velocity should be negative"


@pytest.mark.unit
def test_gripper_state_toggle(fr3_interface):
    # Test toggling gripper state
    initial_state = fr3_interface._gripper_state
    
    if initial_state == GripperState.OPEN:
        expected_state = GripperState.CLOSED
    else:
        expected_state = GripperState.OPEN
    
    # Create method to simulate gripper callbacks
    def simulate_gripper_action_complete():
        time.sleep(0.5)
        fr3_interface._gripper_in_action = False
        
    # Start thread to simulate action completion
    action_thread = Thread(target=simulate_gripper_action_complete)
    action_thread.daemon = True
    action_thread.start()
    
    # Call toggle action (but don't actually send to robot in test)
    if initial_state == GripperState.OPEN:
        fr3_interface.gripper_close()
    else:
        fr3_interface.gripper_open()
    
    # Wait for "action" to complete
    action_thread.join()
    
    # Check state has changed appropriately
    assert fr3_interface._gripper_state == expected_state, "Gripper state should have toggled"
import unittest
import numpy as np
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from arm_interfaces.msg import Teleop
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaRobotState

from robot_arm_interface.fr3_interface import FR3Interface, GripperState


class TestFr3Interface(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize ROS
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        # Shutdown ROS
        rclpy.shutdown()

    def setUp(self):
        # Create a mock interface instance for testing
        self.interface = FR3Interface(hw_type="fake")
        
        # Create a separate thread for spinning the node
        self.spin_thread = Thread(target=self.spin_node)
        self.spin_thread.daemon = True
        self.spin_thread.start()
        
        # Wait for node to initialize properly
        time.sleep(1.0)
        
    def tearDown(self):
        self.interface.destroy_node()
        time.sleep(0.5)  # Give time for clean shutdown
        
    def spin_node(self):
        rclpy.spin(self.interface)

    def test_pose_msg_to_array(self):
        # Test the static utility method
        test_pose = Pose()
        test_pose.position.x = 1.0
        test_pose.position.y = 2.0
        test_pose.position.z = 3.0
        test_pose.orientation.x = 0.0
        test_pose.orientation.y = 0.0
        test_pose.orientation.z = 0.0
        test_pose.orientation.w = 1.0
        
        position, orientation = self.interface.pose_msg_to_array(test_pose)
        
        np.testing.assert_array_equal(position, np.array([1.0, 2.0, 3.0]))
        np.testing.assert_array_equal(orientation, np.array([0.0, 0.0, 0.0, 1.0]))

    def test_twist_msg_to_array(self):
        # Test the static utility method
        test_twist = Twist()
        test_twist.linear.x = 0.1
        test_twist.linear.y = 0.2
        test_twist.linear.z = 0.3
        test_twist.angular.x = 0.4
        test_twist.angular.y = 0.5
        test_twist.angular.z = 0.6
        
        linear, angular = self.interface.twist_msg_to_array(test_twist)
        
        np.testing.assert_array_equal(linear, np.array([0.1, 0.2, 0.3]))
        np.testing.assert_array_equal(angular, np.array([0.4, 0.5, 0.6]))
    
    def test_teleop_command_position_mode(self):
        # Set up a teleop message in position control mode
        teleop_msg = Teleop()
        teleop_msg.control_mode = Teleop.CONTROL_MODE_POSITION
        teleop_msg.ee_des.position.x = 0.5
        teleop_msg.ee_des.position.y = 0.1
        teleop_msg.ee_des.position.z = 0.3
        teleop_msg.ee_des.orientation.w = 1.0
        
        # Manually set the current EE pose
        self.interface.o_t_ee = teleop_msg._create_default_instance()  # Create a default instance
        self.interface.o_t_ee.pose.position.x = 0.45  # Slightly different
        self.interface.o_t_ee.pose.position.y = 0.09
        self.interface.o_t_ee.pose.position.z = 0.32
        self.interface.o_t_ee.pose.orientation.w = 1.0
        
        # Set desired end effector twist as zero
        self.interface.o_dp_ee_d = teleop_msg._create_default_instance()
        
        # Set the teleop message
        self.interface._teleop_goal_msg = teleop_msg
        
        # Compute desired velocity based on teleop command
        desired_vel = self.interface._compute_teleop_cmd()
        
        # Verify desired velocity is in the right direction
        self.assertGreater(desired_vel[0], 0)  # X should be positive
        self.assertGreater(desired_vel[1], 0)  # Y should be positive
        self.assertLess(desired_vel[2], 0)     # Z should be negative
    
    def test_gripper_state_toggle(self):
        # Test toggling gripper state
        initial_state = self.interface._gripper_state
        
        if initial_state == GripperState.OPEN:
            expected_state = GripperState.CLOSED
        else:
            expected_state = GripperState.OPEN
        
        # Create method to simulate gripper callbacks
        def simulate_gripper_action_complete():
            time.sleep(0.5)
            self.interface._gripper_in_action = False
            
        # Start thread to simulate action completion
        action_thread = Thread(target=simulate_gripper_action_complete)
        action_thread.daemon = True
        action_thread.start()
        
        # Call toggle action (but don't actually send to robot in test)
        if initial_state == GripperState.OPEN:
            self.interface.gripper_close()
        else:
            self.interface.gripper_open()
        
        # Wait for "action" to complete
        action_thread.join()
        
        # Check state has changed appropriately
        self.assertEqual(self.interface._gripper_state, expected_state)


if __name__ == '__main__':
    unittest.main()
