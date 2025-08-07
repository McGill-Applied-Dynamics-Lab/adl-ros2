#!/usr/bin/env python3
"""
Script to replay DelayRIM bag data with analysis nodes
"""

import subprocess
import sys
import argparse
import time
from pathlib import Path


def replay_delayrim_data(bag_path: str, rate: float = 1.0, analysis_mode: bool = False):
    """Replay DelayRIM bag data with optional analysis"""

    # Base replay command
    cmd = ["ros2", "bag", "play", bag_path, "--rate", str(rate)]

    if analysis_mode:
        # Only play relevant topics for analysis
        analysis_topics = [
            "/inverse3/state",
            "/fr3_rim",
            "/fr3_rim_delayed",
            "/rim_state",
            "/rim_interface_force",
            "/osc_pd_controller/cartesian_force",
            "/franka_robot_state_broadcaster/robot_state",
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
    parser.add_argument("bag_path", help="Path to bag file")
    parser.add_argument("--rate", "-r", type=float, default=1.0, help="Playback rate (default: 1.0)")
    parser.add_argument("--analysis", "-a", action="store_true", help="Analysis mode - only play relevant topics")

    args = parser.parse_args()

    # Verify bag exists
    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f"Error: Bag file {bag_path} does not exist")
        sys.exit(1)

    replay_delayrim_data(str(bag_path), args.rate, args.analysis)


if __name__ == "__main__":
    main()
