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
from arm_client.robot import Pose, Robot
from rim_teleop import RIMTeleopConfig, RIMTeleopOrchestrator, load_config_from_yaml
from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR, robot


def experiment_setup(robot: Robot) -> None:
    """Run before the teleop loops start."""
    print("Setting up experiment...")
    # robot.home()
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    start_position = np.array([0.6, 0.15, 0.10])  # Same as granular bed
    # start_position = np.array([0.3, 0.0, 0.15])

    start_orientation = Rotation.from_euler("xyz", [-180, 0, 0], degrees=True)
    start_pose = Pose(position=start_position, orientation=start_orientation)

    robot.move_to(pose=start_pose)

    print("Experiment setup complete.")


def main() -> None:
    print("========================================")
    print("RIM Teleop Experiment")
    print("========================================")

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args = parser.parse_args()

    cfg: RIMTeleopConfig = load_config_from_yaml(args.config) if args.config else RIMTeleopConfig()

    orchestrator = RIMTeleopOrchestrator(cfg, setup_fn=experiment_setup)

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
