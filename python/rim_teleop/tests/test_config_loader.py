from rim_teleop.config import default_config_path, load_config_from_yaml


def test_load_config_from_yaml_defaults_file():
    cfg = load_config_from_yaml(default_config_path())

    assert cfg.interface.rim_direction == [0.0, 0.0, 1.0]
    assert cfg.interface.free_space == "leader"
    assert cfg.inverse3.interface_axis == "z"
    assert cfg.rates.rim_rate_hz == 1000.0
    assert cfg.robot.namespace == "fr3"
