from pathlib import Path

"""Initialize crisp_py."""

import ctypes
import importlib.metadata


def _preload_workspace_typesupport() -> None:
    """Load locally-built ROS interface libraries for direct Python execution.

    When examples are launched with the Pixi Python interpreter directly instead of
    sourcing ``install_humble/setup.bash`` first, ROS can't discover the generated
    typesupport libraries via ``LD_LIBRARY_PATH``. Preloading the local arm interface
    libraries keeps the examples usable in that mode.
    """

    package_root = Path(__file__).resolve().parents[2]
    workspace_root = package_root.parent.parent
    install_root = workspace_root / "install_humble" / "arm_interfaces" / "lib"
    if not install_root.exists():
        return

    for lib_path in sorted(install_root.glob("libarm_interfaces__rosidl_typesupport_*.so")):
        ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)


_preload_workspace_typesupport()

try:
    __version__ = importlib.metadata.version("arm_client")
except importlib.metadata.PackageNotFoundError:
    __version__ = "0.0.0"
# CONFIG_DIR = Path(__file__).parent.parent.parent.parent / "src" /"configs"
CONFIG_DIR = Path("configs")

try:
    import rclpy  # noqa: F401
except ImportError:
    print("ROS2 should be installed and sourced!")

try:
    import control_msgs  # noqa: F401

except ImportError:
    print("Could not import control_msgs. Make sure that you installed ros-<ROS_DISTRO>-ros2-control(lers).")
    print("If you are using robostack with pixi, add the following lines to your pixi.toml dependencies:")
    print('ros-<ROS_DISTRO>-ros2-control = "*"\nros-<ROS_DISTRO>-ros2-controllers = "*"')
