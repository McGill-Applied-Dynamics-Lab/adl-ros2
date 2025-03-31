import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy
from turtlesim.msg import Pose


def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package="turtlesim",
                    namespace="",
                    executable="turtlesim_node",
                    name="turtle1",
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ),
        {},
    )


# Active tests
class TestTurtleSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_turtlesim")

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_pose(self, proc_output):
        """Check whether pose messages published"""
        msgs_rx = []
        sub = self.node.create_subscription(Pose, "turtle1/pose", lambda msg: msgs_rx.append(msg), 100)
        try:
            # Listen to the pose topic for 10 s
            end_time = time.time() + 10
            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
            # There should have been 100 messages received
            assert len(msgs_rx) > 100
        finally:
            self.node.destroy_subscription(sub)

    def test_logs_spawning(self, proc_output):
        """Check whether logging properly"""
        proc_output.assertWaitFor("Spawning turtle [turtle1] at x=", timeout=5, stream="stderr")

    # def test_fail(self, proc_output):
    #     """Check whether test fails"""
    #     self.fail("This test should fail")


# Post-shutdown tests
@launch_testing.post_shutdown_test()
class TestTurtleSimShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
