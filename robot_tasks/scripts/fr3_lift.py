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
import pandas as pd
from datetime import datetime
import signal

# Experiment parameters
OBJECT_POSITIONS = [
    # x, y, theta
    np.array([0.0, 0.0, 0.0]),
    # np.array([0.01, 0.01, 0.0]),
    # np.array([0.02, -0.02, 0.0]),
    # np.array([-0.03, -0.03, 0.0]),
    # np.array([-0.04, 0.04, 0.0]),
    # np.array([0.05, 0.05, 0.0]),
    # np.array([0.06, 0.06, 0.0]),
    # # np.array([0.0, 0.0, np.deg2rad(30)]),
]

GRIPPER_TYPE = ["rigid", "soft"]


OBJECTS = {
    "cube": 0.035,  # 3.5cm cube
    "cube_big": 0.05,  # 5cm cube
    "cube_small": 0.02,  # 2cm cube
    "sphere": 0.05,  # 5cm sphere
    "sphere_big": 0.07,  # 7cm sphere
    "cube_side": 0.035 + 0.005,  # 3.5cm cube
    "cube_big_side": 0.05 + 0.01,  # 5cm cube
}

# Experiment settings
OBJECT = "cube_side"  # Example object, can be parameterized
GRIPPER = "rigid"  # Example gripper type, can be parameterized
AGENT = "agent_04"


def call_lift_action(node):
    """Send the insert action goal and wait for the result, allowing KeyboardInterrupt to cancel the action."""
    # Create action client
    action_client = ActionClient(node, RlAgent, "/lift_action")

    # Wait for action server
    print("Waiting for lift action server...")
    if not action_client.wait_for_server(timeout_sec=10.0):
        print("Action server not available after waiting 10 seconds")
        return False, "Action server not available"

    # Create goal message
    goal_msg = RlAgent.Goal()

    # Send goal
    print("Sending goal to lift action server")
    future = action_client.send_goal_async(goal_msg)

    # Wait for goal acceptance (non-blocking, interruptible)
    try:
        while not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nInterrupted while waiting for goal acceptance.")
        raise
    goal_handle = future.result()

    if not goal_handle.accepted:
        print("Goal rejected by action server")
        return False, "Goal rejected"

    # Get result
    result_future = goal_handle.get_result_async()
    print("Waiting for lift action to complete...")
    try:
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt as e:
        print("\nCtrl+C detected: Cancelling lift action...")

        cancel_future = goal_handle.cancel_goal_async()

        # # Wait for cancellation to complete
        # while not cancel_future.done():
        #     rclpy.spin_once(node, timeout_sec=0.1)

        print("Lift action cancelled.")
        # return False, "Action cancelled by user"
        raise e  # Re-raise to exit gracefully

    except Exception as e:
        print(f"An error occurred while waiting for the result: {e}")
        return False, "Error while waiting for result"

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

    print("\n\n----- Starting lift insert task -----")
    franka_arm = FrankaArm()

    # Create tf2 broadcasters
    node = rclpy.create_node("fr3_lift_tf_broadcaster")
    # tf_broadcaster = TransformBroadcaster(node)
    static_broadcaster = StaticTransformBroadcaster(node)

    #! Transforms
    # Gripper to tool transform (fingers tips)
    if GRIPPER == "rigid":
        p_Toffset = np.array([0.0, 0.0, 0.005])
    elif GRIPPER == "soft":
        p_Toffset = np.array([0.0, 0.0, 0.04])  # Tool offset from gripper (z-axis)
    else:
        raise ValueError(f"Unknown gripper type: {GRIPPER}. Supported types are 'rigid' and 'soft'.")

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
    # p_WTstart_W = np.array([0.0, 0.0, 0.95])  # World to tool
    p_BTstart_B = X_WB.inverse() * p_WTstart_W  # Base frame position of tool start pose

    p_BGstart = p_BTstart_B + p_Toffset  # Base frame position of gripper start pose

    p_BOzero = np.array([0.56, 0.0, 0.0])  # Base to object zero position

    #! Agent
    # Moving to Start pose
    print("Moving to start pose...")

    X_G_start = pin.SE3(pin.rpy.rpyToMatrix(gripper_start_pose_rpy), p_BGstart)
    franka_arm.goto_pose(X_G_start, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

    franka_arm.gripper_open()

    # # Test functions
    # franka_arm.gripper_close()
    # X_test = pin.SE3(pin.rpy.rpyToMatrix(gripper_start_pose_rpy), np.array([0.56, 0.0, 0.11]))
    # franka_arm.goto_pose(X_test, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

    # Start a trial! Call the insert action
    print("--- Starting lift trials ---")
    position_n = 1

    print(f"Selected gripper type: {GRIPPER}")
    print(f"Selected object: {OBJECT}")

    # Prepare results DataFrame
    in_action = False  # Flag to indicate if we are currently in an action
    results = []
    experiment_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    out_folder = Path("results")
    out_folder.mkdir(exist_ok=True)
    out_name = Path(f"lift_results_{experiment_time.replace(' ', '_').replace(':', '-')}.csv")
    out_path = out_folder / out_name

    def save_results():
        if results:
            df = pd.DataFrame(results)
            df.to_csv(out_path.as_posix(), index=False)
            print(f"Results saved to {out_path}")
        else:
            print("No results to save.")

    def signal_handler(sig, frame):
        print("\nCtrl+C detected. Saving results before exit...")

        save_results()
        exit(0)

    # signal.signal(signal.SIGINT, signal_handler)

    try:
        while True:
            object_size = OBJECTS[OBJECT]
            N = 3  # Number of repetitions per position
            for each_object_pose in OBJECT_POSITIONS:
                for repeat_idx in range(N):
                    print(f"\n--- Position {position_n}/{len(OBJECT_POSITIONS)} - repeat {repeat_idx + 1}/{N} ---")
                    dx = each_object_pose[0]
                    dy = each_object_pose[1]
                    theta = each_object_pose[2]
                    print(f"Object position: dx={dx}, dy={dy}, theta={np.rad2deg(theta)}")

                    # Adjust object position based on trial parameters
                    cube_start_pose = p_BOzero + np.array([dx, dy, object_size / 2.0])
                    cube_start_rpy = np.array([0, 0.0, theta])
                    X_BC = pin.SE3(pin.rpy.rpyToMatrix(cube_start_rpy), cube_start_pose)

                    # Publish object static transform (only once per trial)
                    publish_static_transform(
                        static_broadcaster, node, X_BC, parent_frame="base", child_frame="cube_frame"
                    )

                    input("Press Enter to start the lift action...")
                    print("Starting lift action")
                    call_lift_action(franka_arm)
                    # Prompt user for success/failure
                    user_result = input("Did the lift succeed? [Y/n]: ").strip().lower()
                    if user_result in ("", "y", "yes"):
                        success = True
                    else:
                        success = False
                    message = "SUCCESS" if success else "FAILED"

                    # Save result for this trial
                    results.append(
                        {
                            "experiment_time": experiment_time,
                            "agent": AGENT,
                            "gripper_type": GRIPPER,
                            "object_type": OBJECT,
                            "object_pose_x": float(dx),
                            "object_pose_y": float(dy),
                            "object_pose_theta_deg": float(np.rad2deg(theta)),
                            "success": success,
                            "message": message,
                        }
                    )

                    if success:
                        print(f"Lift successful! Message: {message}")
                    else:
                        print(f"Lift failed! Message: {message}")

                    time.sleep(1.0)

                    franka_arm.gripper_open()
                    franka_arm.goto_pose(X_G_start, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

                position_n += 1

            print("All trials completed. Exiting...")
            break

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Saving results before exit...")

    finally:
        save_results()
        # Clean up
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
