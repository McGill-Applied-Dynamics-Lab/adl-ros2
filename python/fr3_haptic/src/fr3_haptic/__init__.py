"""RIM teleoperation package integrated with arm_client.

The top-level names are imported on first use (PEP 562), so the ROS-free submodules
(``fr3_haptic.scripted``, ``fr3_haptic.guide``) import without arm_client, e.g. in the
analysis environment.
"""

__all__ = ["RIMTeleopConfig", "RIMTeleopOrchestrator", "load_config_from_yaml"]


def __getattr__(name: str):
    if name in ("RIMTeleopConfig", "load_config_from_yaml"):
        from . import config

        return getattr(config, name)
    if name == "RIMTeleopOrchestrator":
        from .orchestrator import RIMTeleopOrchestrator

        return RIMTeleopOrchestrator
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
