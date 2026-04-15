import ctypes
import importlib.metadata
import os
from pathlib import Path

"""Initialize crisp_py."""


def _preload_workspace_typesupport() -> None:
    """Load locally-built ROS interface libraries for direct Python execution.

    When examples are launched with the Pixi Python interpreter directly instead of
    sourcing ``install_humble/setup.bash`` first, ROS can't discover the generated
    typesupport libraries via ``LD_LIBRARY_PATH``. Preloading the local arm interface
    libraries keeps the examples usable in that mode.
    """

    install_root = None
    current_file = Path(__file__).resolve()
    for candidate_root in current_file.parents:
        candidate_install_root = candidate_root / "install_humble" / "arm_interfaces" / "lib"
        if candidate_install_root.exists():
            install_root = candidate_install_root
            break
    if install_root is None:
        return

    existing_ld_library_path = os.environ.get("LD_LIBRARY_PATH")
    install_root_str = str(install_root)
    if existing_ld_library_path:
        if install_root_str not in existing_ld_library_path.split(":"):
            os.environ["LD_LIBRARY_PATH"] = f"{install_root_str}:{existing_ld_library_path}"
    else:
        os.environ["LD_LIBRARY_PATH"] = install_root_str

    for lib_path in sorted(install_root.glob("libarm_interfaces__rosidl_typesupport_*.so")):
        ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)


_preload_workspace_typesupport()

try:
    __version__ = importlib.metadata.version("arm_client")
except importlib.metadata.PackageNotFoundError:
    __version__ = "0.0.0"
# CONFIG_DIR = Path(__file__).parent.parent.parent.parent / "src" /"configs"

root_dir = Path(__file__).parent.parent.parent.parent

CONFIG_DIR = root_dir / "configs"

if not CONFIG_DIR.exists():
    Warning(f"Config directory {CONFIG_DIR} not found.")

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
