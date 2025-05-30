# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from rclpy.action import ActionClient
from arm_interfaces.action import (
    GotoJoints,
    GotoPose,
    GotoJointVelocities,
    GotoEEVelocity,
    GripperHoming,
    GripperToggle,
    GripperOpen,
    GripperClose,
)
from geometry_msgs.msg import PoseStamped

import numpy as np
from typing import List, Literal
import pinocchio as pin

# from robot_arm_interface.utils import SE32PoseStamped, PoseStamped2SE3, array2pose
from robot_arm_interface.utils import se32rospose

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters


class FrankaArm(Node):
    def __init__(self):
        super().__init__("franka_arm_client")
        self.get_logger().info("Starting robot arm client")

        #! Action clients
        self._action_client_list: list[ActionClient] = []
        self._init_action_clients()
        self._wait_for_action_servers()

    def _init_action_clients(self):
        # # goto_joints
        # self._goto_joints_ac = ActionClient(self, GotoJoints, "goto_joints")
        # self._action_client_list.append(self._goto_joints_ac)

        # goto_pose
        self._goto_pose_ac = ActionClient(self, GotoPose, "goto_pose")
        self._action_client_list.append(self._goto_pose_ac)

        # goto_joint_vels
        self._goto_joint_vels_ac = ActionClient(self, GotoJointVelocities, "goto_joint_vels")
        self._action_client_list.append(self._goto_joint_vels_ac)

        # goto_ee_vels
        self._goto_ee_vel_ac = ActionClient(self, GotoEEVelocity, "goto_ee_vels")
        self._action_client_list.append(self._goto_ee_vel_ac)

        # gripper
        self._gripper_homing_ac = ActionClient(self, GripperHoming, "gripper_homing")
        self._action_client_list.append(self._gripper_homing_ac)

        self._gripper_toggle_ac = ActionClient(self, GripperToggle, "gripper_toggle")
        self._action_client_list.append(self._gripper_toggle_ac)

        self._gripper_open_ac = ActionClient(self, GripperOpen, "gripper_open")
        self._action_client_list.append(self._gripper_open_ac)

        self._gripper_close_ac = ActionClient(self, GripperClose, "gripper_close")
        self._action_client_list.append(self._gripper_close_ac)

    def _wait_for_action_servers(self):
        self.get_logger().info("Waiting for action servers...")
        for ac in self._action_client_list:
            # ac_found = ac.wait_for_server(timeout_sec=1)
            # if not ac_found:
            #     self.get_logger().error(f"Action server {ac._action_name} not found!")

            while not ac.wait_for_server(timeout_sec=1):
                self.get_logger().warn(f"Action server {ac._action_name} not up...")

        self.get_logger().info("Action servers are up!")

    #! Action clients
    def _wait_for_action(self, future):
        """
        To wait until `future` is completed and return the result.
        """
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal was rejected by the action server!")
                return False

            self.get_logger().info("Goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            if result_future.done():
                result = result_future.result()
                self.get_logger().info(f"Result received: {result.result.success}")
                return result.result.success

            else:
                self.get_logger().error("Failed to get result!")
                return False
        else:
            self.get_logger().error("Failed to send goal!")
            return False

    def goto_joints(self, joint_goal: np.ndarray, duration: Duration):
        self.get_logger().info(f"Moving joints to goal: {joint_goal}")

        raise NotImplementedError("goto_joints not implemented yet")

        goal_msg = GotoJoints.Goal()
        goal_msg.joints_goal = joint_goal.tolist()
        goal_msg.duration = duration.to_msg()

        future = self._goto_joints_ac.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal was rejected by the action server!")
                return False

            self.get_logger().info("Goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            if result_future.done():
                result = result_future.result()
                self.get_logger().info(f"Result received: {result.result.success}")
                return result.result.success

            else:
                self.get_logger().error("Failed to get result!")
                return False
        else:
            self.get_logger().error("Failed to send goal!")
            return False

    def home(self):
        self.get_logger().info("Moving to home position")

        # joint_home_position = np.deg2rad([0, -45, 0, -135, 0, 90, 45])
        # self.goto_joints(joint_home_position, Duration(seconds=3))

        home_t = np.array([0.30, 0.00, 0.5])
        home_pose = pin.SE3(pin.rpy.rpyToMatrix(np.array([np.pi, 0, 0])), home_t)
        self.goto_pose(home_pose, Duration(seconds=10))

    def goto_pose(self, pose_goal: pin.SE3, duration: Duration, Kp=None, Kd=None):
        self.get_logger().info(f"Moving to cartesian goal:\n {pose_goal}")

        if Kp is not None:
            self.get_logger().info(f"Setting Kp: {Kp}")
            self.set_robot_parameter("/fr3_interface", "Kp_gripper_trans", Kp)

        if Kd is not None:
            self.get_logger().info(f"Setting Kd: {Kd}")
            self.set_robot_parameter("/fr3_interface", "Kd_gripper_trans", Kd)

        pose_goal_msg = PoseStamped()
        pose_goal_msg.header.stamp = self.get_clock().now().to_msg()
        pose_goal_msg.pose = se32rospose(pose_goal)

        # T = PoseStamped2SE3(pose_goal_msg)

        goal_msg = GotoPose.Goal()
        goal_msg.pose_goal = pose_goal_msg
        goal_msg.duration = duration.to_msg()

        future = self._goto_pose_ac.send_goal_async(goal_msg)
        return self._wait_for_action(future)

    def goto_ee_vel(self, ee_vel: List | np.ndarray, duration: Duration):
        """
        Move the end effector with a given velocity.

        Args:
            ee_vel (np.ndarray): End effector velocity [vx, vy, vz, wx, wy, wz] in m/s and rad/s
            duration (Duration): Duration of the movement in seconds
        """
        if type(ee_vel) is list:
            ee_vel = np.array(ee_vel)

        self.get_logger().info(f"Moving end effector with velocity: {ee_vel}")

        if len(ee_vel) != 6:
            self.get_logger().error("End effector velocity must have 6 elements!")
            return False

        # Build the goal message
        goal_msg = GotoEEVelocity.Goal()

        goal_msg.ee_velocity = ee_vel.tolist()
        goal_msg.duration = duration.to_msg()

        future = self._goto_ee_vel_ac.send_goal_async(goal_msg)
        return self._wait_for_action(future)

    def goto_joint_vels(
        self,
        joint_vels: List | np.ndarray,
        duration: Duration,
        unit: str = Literal["rad", "deg"],
    ):
        # Process the joint vels
        if unit == "deg":
            joint_vels = np.deg2rad(joint_vels)

        if type(joint_vels) is list:
            joint_vels = np.array(joint_vels)

        self.get_logger().info(f"Moving joints with velocities: {np.rad2deg(joint_vels)}")

        # Build the goal message
        goal_msg = GotoJointVelocities.Goal()
        goal_msg.joint_velocities = joint_vels.tolist()
        goal_msg.duration = duration.to_msg()

        future = self._goto_joint_vels_ac.send_goal_async(goal_msg)
        return self._wait_for_action(future)

    def gripper_homing(self):
        self.get_logger().info("Homing gripper")
        goal_msg = GripperHoming.Goal()
        future = self._gripper_homing_ac.send_goal_async(goal_msg)
        return self._wait_for_action(future)

    def gripper_toggle(self):
        self.get_logger().info("Toggling gripper")
        goal_msg = GripperToggle.Goal()
        future = self._gripper_toggle_ac.send_goal_async(goal_msg)
        return self._wait_for_action(future)

    def gripper_open(self):
        self.get_logger().info("Opening gripper")
        goal_msg = GripperOpen.Goal()
        future = self._gripper_open_ac.send_goal_async(goal_msg)
        return self._wait_for_action(future)

    def gripper_close(self):
        self.get_logger().info("Closing gripper")
        goal_msg = GripperClose.Goal()
        future = self._gripper_close_ac.send_goal_async(goal_msg)
        return self._wait_for_action(future)

    #! Services
    def get_robot_state(self): ...

    def get_joints(self): ...

    def get_pose(self): ...

    #! Utility functions
    def set_robot_parameter(self, target_node_name: str, param_name: str, param_value: float):
        """
        Update a parameter on another ROS2 node.

        Args:
            target_node_name: The name of the target node (e.g., '/fr3_interface').
            param: The name of the parameter (str).
            value: The new value for the parameter.
        Returns:
            True if successful, False otherwise.
        """
        client = self.create_client(SetParameters, f"{target_node_name}/set_parameters")
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {target_node_name}/set_parameters not available.")
            return False

        param_msg = Parameter(param_name, Parameter.Type.DOUBLE, float(param_value)).to_parameter_msg()
        req = SetParameters.Request()
        req.parameters = [param_msg]

        future = client.call_async(req)
        import rclpy

        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return result.results[0].successful if result and result.results else False


def main(args=None):
    rclpy.init(args=args)
    node = FrankaArm()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    # executor = MultiThreadedExecutor()
    # try:
    #     rclpy.spin(node, executor=executor)
    # except KeyboardInterrupt:
    #     pass

    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == "__main__":
    main()
