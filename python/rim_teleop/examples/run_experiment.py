"""
Launcher template for a RIM teleoperation experiment with a custom setup sequence.

Usage:
    pixi run -e humble python python/rim_teleop/examples/run_experiment.py
    pixi run -e humble python python/rim_teleop/examples/run_experiment.py --config path/to/config.yaml
"""

from __future__ import annotations

import argparse
import signal
import threading

import numpy as np
from arm_client.robot import Robot
from scipy.spatial.transform import Rotation

from rim_teleop import RIMTeleopOrchestrator, RIMTeleopConfig, load_config_from_yaml


def setup(robot: Robot) -> None:
    """Run before the teleop loops start. Robot is ready; JTC is active."""
    robot.home()

    # Example: move to a custom start pose
    # start_pose = robot.end_effector_pose.copy()
    # start_pose.position[2] = 0.45  # lower z
    # robot.move_to(start_pose)

    # Example: close gripper
    # robot.gripper.move(width=0.0, speed=0.1, force=10.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args = parser.parse_args()

    cfg: RIMTeleopConfig = load_config_from_yaml(args.config) if args.config else RIMTeleopConfig()

    orchestrator = RIMTeleopOrchestrator(cfg, setup_fn=setup)

    stop_evt = threading.Event()

    def _stop(signum, frame):  # noqa: ANN001
        del signum, frame
        stop_evt.set()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    orchestrator.start()
    stop_evt.wait()
    orchestrator.stop()


if __name__ == "__main__":
    main()
