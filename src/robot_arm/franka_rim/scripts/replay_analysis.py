#!/usr/bin/env python3
"""
Script to replay DelayRIM bag data with analysis nodes
"""

import subprocess
import sys
import argparse
import time
from pathlib import Path

FILES = {
    "sin1": {
        "file": "franka_rim_data_20250801_183142",
        "desc": "Sin input                       ",
    },
    "sin2": {
        "file": "franka_rim_data_20250801_183837",
        "desc": "Sin, x3, fast                   ",
    },
    "sin3": {
        "file": "franka_rim_data_20250801_184008",
        "desc": "Sin x3, slow                    ",
    },
    "teleop1": {
        "file": "franka_rim_data_20250801_184356",
        "desc": "Teleop, no interaction, 3D      ",
    },
    "teleop2": {
        "file": "franka_rim_data_20250801_184721",
        "desc": "Teleop, no interaction, $x$ only",
    },
    "teleop3": {
        "file": "franka_rim_data_20250801_185047",
        "desc": "Teleop, no movement, interaction",
    },
    "teleop4": {
        "file": "franka_rim_data_20250801_185149",
        "desc": "Teleop, no movement, interaction",
    },
    "contact1": {
        "file": "franka_rim_data_20250801_185623",
        "desc": "Nic wall                        ",
    },
    "contact2": {
        "file": "franka_rim_data_20250801_192149",
        "desc": "Sliding, one way only           ",
    },
    "contact3": {
        "file": "franka_rim_data_20250801_192407",
        "desc": "Sliding                         ",
    },
    "sliding1": {
        "file": "franka_rim_data_20250801_192514",
        "desc": "Sliding                         ",
    },
}


def replay_delayrim_data(bag_path: str, rate: float = 1.0, analysis_mode: bool = False):
    """Replay DelayRIM bag data with optional analysis"""

    # Base replay command with simulation time enabled
    cmd = [
        "ros2",
        "bag",
        "play",
        bag_path,
        "--rate",
        str(rate),
        "--clock",  # Publish simulation time from bag
        # "--loop",
    ]

    analysis_topics = [
        "/clicked_point",
        # "/clock",
        # "/delayrim_visualization",
        "/dynamic_joint_states",
        "/events/read_split",
        "/events/write_split",
        # "/f_ext_est",
        # "/f_ext_robot",
        # "/fr3_model",
        # "/fr3_rim",
        # "/fr3_rim_delayed",
        "/franka/joint_states",
        "/franka_gripper/joint_states",
        "/franka_robot_state_broadcaster/current_pose",
        "/franka_robot_state_broadcaster/desired_end_effector_twist",
        "/franka_robot_state_broadcaster/desired_joint_states",
        "/franka_robot_state_broadcaster/external_joint_torques",
        "/franka_robot_state_broadcaster/external_wrench_in_base_frame",
        "/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame",
        "/franka_robot_state_broadcaster/last_desired_pose",
        "/franka_robot_state_broadcaster/measured_joint_states",
        "/franka_robot_state_broadcaster/robot_state",
        "/franka_robot_state_broadcaster/transition_event",
        "/goal_pose",
        "/initialpose",
        "/inverse3/state",
        # "/inverse3/wrench_des",
        "/joint_state_broadcaster/transition_event",
        "/joint_states",
        "/osc_pd_controller/cartesian_force",
        # "/osc_pd_controller/goal",
        # "/osc_pd_controller/goal_pre_delay",
        "/osc_pd_controller/transition_event",
        "/parameter_events",
        # "/rim_interface_force",
        # "/rim_state",
        "/robot_description",
        "/rosout",
        "/tf",
        "/tf_static",
    ]
    cmd.extend(["--topics"] + analysis_topics)

    print(f"Playing bag: {bag_path}")
    print(f"Command: {' '.join(cmd)}")

    # Start replay
    process = subprocess.Popen(cmd)

    try:
        process.wait()
    except KeyboardInterrupt:
        print("\nStopping replay...")
        process.terminate()
        process.wait()


def main():
    parser = argparse.ArgumentParser(description="Replay DelayRIM experiment data")
    parser.add_argument("bag_file", help="Bag file to replay")
    parser.add_argument("--rate", "-r", type=float, default=1.0, help="Playback rate (default: 1.0)")
    parser.add_argument("--analysis", "-a", action="store_true", help="Analysis mode - only play relevant topics")

    args = parser.parse_args()

    # Verify bag exists
    bag_file = FILES[args.bag_file]
    bag_path = Path("data") / bag_file["file"]
    if not bag_path.exists():
        print(f"Error: Bag file {bag_path} does not exist")
        sys.exit(1)

    else:
        print(f"Using bag file: {bag_path}")
        print(f"Description: {bag_file['desc']}")

    replay_delayrim_data(str(bag_path), args.rate, args.analysis)


if __name__ == "__main__":
    main()
