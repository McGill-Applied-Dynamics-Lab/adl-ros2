"""
Launch file for RIM Stepper Node - validates RIM dynamics by stepping forward in time.

This launch file runs:
- Inverse3 device (real or simulated)
- Franka model node (computes robot dynamics)
- Franka RIM node (computes reduced interface model)
- RIM Stepper node (steps RIM forward for validation)
- Visualization nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Single launch argument for parameter file
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="franka_rim_default.yaml",
        description="YAML configuration file to load (in franka_rim/config/)",
    )

    # Build path to config file
    config_file_path = PathJoinSubstitution(
        [FindPackageShare("franka_rim"), "config", LaunchConfiguration("config_file")]
    )

    # Additional override arguments (optional)
    fake_i3_arg = DeclareLaunchArgument(
        "fake_i3", default_value="true", description="Override fake_i3 setting (true/false)"
    )

    #! MARK: Nodes
    rviz_config_file = PathJoinSubstitution([FindPackageShare("franka_rim"), "rviz", "fr3_rim.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[config_file_path],
    )

    visualization_node = Node(
        package="franka_rim",
        executable="rim_vis_node",
        name="rim_vis_node",
        parameters=[config_file_path],
        output="screen",
    )

    # -- Inverse3Node (real and fake)
    inverse3_node = Node(
        package="inverse3_ros2",
        executable="inverse3_node",
        name="inverse3_node",
        output="screen",
        parameters=[config_file_path],
        condition=UnlessCondition(LaunchConfiguration("fake_i3")),
    )

    i3_sim_node = Node(
        package="franka_rim",
        executable="i3_sim_node",
        name="i3_sim_node",
        output="screen",
        parameters=[config_file_path],
        condition=IfCondition(LaunchConfiguration("fake_i3")),
    )

    # -- Teleop Node
    fr3_teleop_node = Node(
        package="teleop",
        executable="i3_teleop",
        name="i3_teleop",
        output="screen",
        parameters=[config_file_path],
    )

    # --- FR3 model node
    franka_model_node = Node(
        package="franka_rim",
        executable="franka_model_node",
        name="franka_model_node",
        parameters=[config_file_path],
        output="screen",
    )

    # --- FR3 RIM node
    franka_rim_node = Node(
        package="franka_rim",
        executable="franka_rim_node",
        name="franka_rim_node",
        parameters=[config_file_path],
        output="screen",
    )

    # --- RIM Stepper node (for validation)
    rim_stepper_node = Node(
        package="franka_rim",
        executable="rim_stepper_node",
        name="rim_stepper_node",
        parameters=[config_file_path],
        output="screen",
    )

    return LaunchDescription(
        [
            # -- Args
            config_file_arg,
            fake_i3_arg,
            # -- Nodes
            inverse3_node,
            i3_sim_node,
            franka_model_node,
            franka_rim_node,
            rim_stepper_node,
            fr3_teleop_node,
            # -- Visualization
            rviz_node,
            visualization_node,
        ]
    )
