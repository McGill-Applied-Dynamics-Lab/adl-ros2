from pathlib import Path

from arm_client_rim.config import load_config_from_yaml


def test_load_config_from_yaml_defaults_file():
    repo_root = Path(__file__).resolve().parents[4]
    config_path = repo_root / "src" / "robot_arm" / "arm_client_rim" / "config" / "rim_teleop_default.yaml"

    cfg = load_config_from_yaml(str(config_path))

    assert cfg.interface.axis == "z"
    assert cfg.inverse3.interface_axis == "z"
    assert cfg.rates.rim_rate_hz == 1000.0
    assert cfg.robot.namespace == "fr3"
