import rclpy
from robot_arm_client.FrankaArm import FrankaArm
from robot_arm_client.teleop_node import TeleopNode
from rclpy.duration import Duration
import argparse
from rclpy.action import ActionClient
from arm_interfaces.action import PegInHole, RlAgent

import numpy as np
from numpy import pi as PI

import time
import pinocchio as pin
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Experiment parameters
OBJECT_POSITIONS = [
    np.array([-0.0, 0.0, -np.deg2rad(0)]),  # x, y, theta
    # np.array([-0.01, 0.02, np.deg2rad(45)]),  # x, y, theta
]

GRIPPER_TYPE = ["rigid", "soft"]

OBJECTS = {
    "cube": 0.035,  # 3.5cm cube
    "cube_big": 0.05,  # 5cm cube
    "sphere": 0.03,
}


def call_lift_action(node):
    """Send the insert action goal and wait for the result."""
    # Create action client
    action_client = ActionClient(node, RlAgent, "/lift_action")

    # Wait for action server
    print("Waiting for insert action server...")
    if not action_client.wait_for_server(timeout_sec=10.0):
        print("Action server not available after waiting 10 seconds")
        return False, "Action server not available"

    # Create goal message
    goal_msg = RlAgent.Goal()

    # Send goal
    print("Sending goal to insert action server")
    future = action_client.send_goal_async(goal_msg)

    # Wait for goal acceptance
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        print("Goal rejected by action server")
        return False, "Goal rejected"

    # Get result
    result_future = goal_handle.get_result_async()
    print("Waiting for insertion action to complete...")
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result
    return result.success, result.message


def publish_static_transform(static_broadcaster, node, X: pin.SE3, parent_frame="base", child_frame="cube_frame"):
    """Publish the transform `X` from `parent_frame` to `child_frame` as a static transform to tf2."""
    tf_msg = TransformStamped()
    tf_msg.header.stamp = node.get_clock().now().to_msg()
    tf_msg.header.frame_id = parent_frame
    tf_msg.child_frame_id = child_frame
    tf_msg.transform.translation.x = X.translation[0]
    tf_msg.transform.translation.y = X.translation[1]
    tf_msg.transform.translation.z = X.translation[2]

    quat = pin.Quaternion(X.rotation)
    tf_msg.transform.rotation.x = quat.x
    tf_msg.transform.rotation.y = quat.y
    tf_msg.transform.rotation.z = quat.z
    tf_msg.transform.rotation.w = quat.w
    static_broadcaster.sendTransform(tf_msg)


def main(args=None):
    parser = argparse.ArgumentParser(description="Robot arm control test")
    # parser.add_argument("--demo", action="store_true", help="Run the demo sequence")
    # parser.add_argument("--no-teleop", action="store_true", help="Run without teleop capabilities")
    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)

    print("----- Starting lift insert task -----")
    franka_arm = FrankaArm()

    # Create tf2 broadcasters
    node = rclpy.create_node("fr3_lift_tf_broadcaster")
    # tf_broadcaster = TransformBroadcaster(node)
    static_broadcaster = StaticTransformBroadcaster(node)

    #! Transforms
    # Gripper to tool transform (fingers tips)
    # TODO: Change this for soft gripper
    p_Toffset = np.array([0.0, 0.0, 0.0])  # Tool offset from gripper (z-axis)

    X_GT = pin.SE3(pin.rpy.rpyToMatrix(np.zeros(3)), p_Toffset)  # Gripper to tool transform
    publish_static_transform(static_broadcaster, node, X_GT, parent_frame="fr3_hand_tcp", child_frame="fr3_tool")

    # World to sim base and real base
    p_WBsim = np.array([-0.56, 0.0, 0.912])  # World to simulation base

    # ? Change `p_BsimB` in `Lift.py` to match this
    p_BsimB = np.array(
        [0.0, 0.0, -0.1095]
    )  # Sim base to real base (-0.142 if object's top, -0.117 for middle, -0.1095 for 3.5cm cube)

    rpy_WB = np.array([0.0, 0.0, 0.0])  # World frame to base frame rpy angles

    X_WBsim = pin.SE3(pin.rpy.rpyToMatrix(rpy_WB), p_WBsim)  # World to base transform
    X_BsimB = pin.SE3(pin.rpy.rpyToMatrix(rpy_WB), p_BsimB)  # Base to base simulation transform
    X_WB = X_WBsim * X_BsimB  # World to base transform in simulation

    # Gripper start pose (G is at the tip of the rigid fingers)
    gripper_start_pose_rpy = np.array([PI, 0, 0])  # Orientation
    p_WTstart_W = np.array([0.0, 0.0, 1.01])  # World to tool
    p_BTstart_B = X_WB.inverse() * p_WTstart_W  # Base frame position of tool start pose

    p_BGstart = p_BTstart_B + p_Toffset  # Base frame position of gripper start pose

    p_BOzero = np.array([0.56, 0.0, 0.0])  # Base to object zero position

    #! Agent
    # Moving to Start pose
    print("Moving to start pose...")

    X_G_start = pin.SE3(pin.rpy.rpyToMatrix(gripper_start_pose_rpy), p_BGstart)
    franka_arm.goto_pose(X_G_start, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

    franka_arm.gripper_open()

    # Test functions
    # franka_arm.gripper_close()
    # X_test = pin.SE3(pin.rpy.rpyToMatrix(gripper_start_pose_rpy), np.array([0.56, -0.4, 0.2]))
    # franka_arm.goto_pose(X_test, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

    # Start a trial! Call the insert action
    print("--- Starting lift trials ---")
    trial_n = 1

    while True:
        object = "cube_big"  # Example object, can be parameterized
        object_size = OBJECTS[object]

        for each_object_pose in OBJECT_POSITIONS:
            print(f"\n\n--- Trial {trial_n} ---")
            dx = each_object_pose[0]
            dy = each_object_pose[1]
            theta = each_object_pose[2]
            print(f"Object position: dx={dx}, dy={dy}, theta={theta}")

            # Adjust object position based on trial parameters
            cube_start_pose = p_BOzero + np.array([dx, dy, object_size / 2.0])
            cube_start_rpy = np.array([0, 0.0, theta])
            X_BC = pin.SE3(pin.rpy.rpyToMatrix(cube_start_rpy), cube_start_pose)

            # Publish object static transform (only once per trial)
            publish_static_transform(static_broadcaster, node, X_BC, parent_frame="base", child_frame="cube_frame")

            print("Starting lift action")
            success, message = call_lift_action(franka_arm)

            # Print result
            if success:
                print(f"Lift successful! Message: {message}")
            else:
                print(f"Lift failed! Message: {message}")

            time.sleep(1.0)

            franka_arm.gripper_open()
            franka_arm.goto_pose(X_G_start, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

            trial_n += 1

    # Clean up
    rclpy.shutdown()


if __name__ == "__main__":
    main()
