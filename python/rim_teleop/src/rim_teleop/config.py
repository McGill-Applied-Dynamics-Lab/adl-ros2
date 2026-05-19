"""Configuration dataclasses for RIM teleoperation runtime."""

from dataclasses import dataclass, field, is_dataclass
from importlib.resources import files

import yaml

from arm_client.teleop.inverse3_teleop import Inverse3Config


@dataclass(kw_only=True)
class RateConfig:
    """Multi-rate loop settings in Hz."""

    control_rate_hz: float = 50.0
    model_rate_hz: float = 50.0
    rim_rate_hz: float = 1000.0
    haptic_rate_hz: float = 1000.0


@dataclass(kw_only=True)
class InterfaceConfig:
    """RIM interface and force rendering settings."""

    axis: str = "z"
    stiffness: float = 1000.0
    damping: float = 90.0
    contact_surface: float = 0.0
    contact_stiffness: float = 10000.0
    contact_damping: float = 100.0
    force_scale: float = 1.0
    force_cap: float = 12.0
    rim_enabled: bool = True
    force_feedback: str = "rim"  # "none" | "robot" | "rim"


@dataclass(kw_only=True)
class SafetyConfig:
    """Safety and stale data gates."""

    stale_state_timeout_s: float = 0.2
    stale_model_timeout_s: float = 0.25
    deadman_required: bool = False


@dataclass(kw_only=True)
class RobotRuntimeConfig:
    """Robot-facing runtime settings specific to RIM orchestration."""

    namespace: str = "fr3"
    controller_name: str = "joint_trajectory_controller"
    trajectory_length: int = 5
    trajectory_dt: float = 0.1
    wrench_filter_alpha: float | None = None  # IIR alpha [0,1]; None disables filtering


@dataclass(kw_only=True)
class ModelConfig:
    """Local model estimator settings."""

    urdf_package: str = "franka_rim"
    urdf_relative_path: str = "models/fr3_franka_hand.urdf"
    ee_frame_name: str = "fr3_hand_tcp"
    filter_alpha_q: float = 0.3
    filter_alpha_q_dot: float = 0.2
    filter_alpha_tau: float = 0.2


@dataclass(kw_only=True)
class FileSinkConfig:
    """Local file sink configuration."""

    enabled: bool = True
    output_dir: str = "data/rim_runs"
    run_name: str | None = None
    flush_hz: float = 20.0


@dataclass(kw_only=True)
class FoxgloveSinkConfig:
    """Foxglove live sink configuration."""

    enabled: bool = True
    host: str = "0.0.0.0"
    port: int = 8765
    topic_prefix: str = "/rim"


@dataclass(kw_only=True)
class LoggingConfig:
    """Experiment logger with pluggable sinks."""

    enabled: bool = True
    sinks: list[str] = field(default_factory=lambda: ["file_sink", "foxglove_sink"])
    file_sink: FileSinkConfig = field(default_factory=FileSinkConfig)
    foxglove_sink: FoxgloveSinkConfig = field(default_factory=FoxgloveSinkConfig)


@dataclass(kw_only=True)
class RIMTeleopConfig:
    """Top-level configuration for the full orchestrator."""

    rates: RateConfig = field(default_factory=RateConfig)
    interface: InterfaceConfig = field(default_factory=InterfaceConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    inverse3: Inverse3Config = field(default_factory=Inverse3Config)
    robot: RobotRuntimeConfig = field(default_factory=RobotRuntimeConfig)
    model: ModelConfig = field(default_factory=ModelConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    dry_run: bool = False


def _apply_nested_config(instance, values: dict) -> None:
    """Apply nested dict values to nested dataclass instances in place."""
    for key, value in values.items():
        if not hasattr(instance, key):
            continue
        current = getattr(instance, key)
        if is_dataclass(current) and isinstance(value, dict):
            _apply_nested_config(current, value)
        else:
            setattr(instance, key, value)


def default_config_path() -> str:
    """Return the path to the bundled default config YAML."""
    return str(files("rim_teleop") / "configs" / "rim_teleop_default.yaml")


def load_config_from_yaml(file_path: str) -> RIMTeleopConfig:
    """Load RIM teleoperation config from a YAML file."""
    with open(file_path, "r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    config = RIMTeleopConfig()
    if not isinstance(loaded, dict):
        raise ValueError("Expected YAML root object to be a dictionary")
    _apply_nested_config(config, loaded)
    return config
