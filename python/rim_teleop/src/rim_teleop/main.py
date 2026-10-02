"""CLI entrypoint: run RIM bilateral teleoperation."""

from __future__ import annotations

import argparse
import signal
import threading

from rim_teleop.config import RIMTeleopConfig, load_config_from_yaml
from rim_teleop.orchestrator import RIMTeleopOrchestrator


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run RIM bilateral teleoperation with arm_client")
    parser.add_argument("--config", type=str, default=None, help="Path to YAML config file")
    parser.add_argument("--websocket-uri", default=None, help="Inverse3 websocket URI")
    parser.add_argument("--robot-ns", default=None, help="ROS namespace used by arm_client")
    parser.add_argument("--control-rate", type=float, default=None, help="Robot command loop rate (Hz)")
    parser.add_argument("--rim-rate", type=float, default=None, help="RIM integration loop rate (Hz)")
    parser.add_argument("--haptic-rate", type=float, default=None, help="Haptic I/O loop rate (Hz)")
    parser.add_argument("--dry-run", action="store_true", help="Skip robot init; test loops, logging, and I3 without arm")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    cfg: RIMTeleopConfig = load_config_from_yaml(args.config) if args.config else RIMTeleopConfig()

    if args.websocket_uri is not None:
        cfg.inverse3.uri = args.websocket_uri
    if args.robot_ns is not None:
        cfg.robot.namespace = args.robot_ns
    if args.control_rate is not None:
        cfg.rates.control_rate_hz = args.control_rate
        cfg.rates.model_rate_hz = args.control_rate
    if args.rim_rate is not None:
        cfg.rates.rim_rate_hz = args.rim_rate
    if args.haptic_rate is not None:
        cfg.rates.haptic_rate_hz = args.haptic_rate
    if args.dry_run:
        cfg.dry_run = True

    orchestrator = RIMTeleopOrchestrator(cfg)

    stop_evt = threading.Event()

    def _stop_handler(signum, frame):  # noqa: ANN001
        del signum, frame
        stop_evt.set()

    signal.signal(signal.SIGINT, _stop_handler)
    signal.signal(signal.SIGTERM, _stop_handler)

    orchestrator.start()
    stop_evt.wait()
    orchestrator.stop()


if __name__ == "__main__":
    main()
