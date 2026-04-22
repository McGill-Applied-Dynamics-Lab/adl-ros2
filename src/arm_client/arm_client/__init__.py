from pathlib import Path

import importlib.metadata

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
