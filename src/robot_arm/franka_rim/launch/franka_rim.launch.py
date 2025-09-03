import os
from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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

    save_data_arg = DeclareLaunchArgument(
        "save_data", default_value="false", description="Record all data to a ROS 2 bag file"
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

    # -- Network delay nodes
    netsim_rim_msg_node = Node(
        package="network_sim",
        executable="network_sim_node",
        name="rim_msg_delay",
        parameters=[config_file_path],
        output="screen",
    )

    netsim_node_ee_cmd_node = Node(
        package="network_sim",
        executable="network_sim_node",
        name="ee_cmd_delay",
        parameters=[config_file_path],
        output="screen",
    )

    # -- DelayRIMNode
    delay_rim_node = Node(
        package="franka_rim",
        executable="delay_rim_node",
        name="delay_rim_node",
        parameters=[config_file_path],
        output="screen",
    )

    # ROS 2 bag recording
    # Generate timestamped bag file name
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_filename = f"franka_rim_data_{timestamp}"
    bag_filepath = Path("logs") / bag_filename

    bag_record_node = ExecuteProcess(  # noqa
        cmd=[
            "ros2",
            "bag",
            "record",
            "--all",  # Record all topics
            "--output",
            bag_filepath.as_posix(),  # Use the generated bag file path
            "--storage",
            "sqlite3",
            "--max-bag-size",
            "0",  # No size limit
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("save_data")),
    )

    return LaunchDescription(
        [
            # -- Args
            config_file_arg,
            fake_i3_arg,
            save_data_arg,
            # -- Nodes
            # inverse3_node,  # Add real inverse3 node back
            i3_sim_node,
            franka_model_node,
            franka_rim_node,
            delay_rim_node,
            # -- Network sim nodes
            netsim_rim_msg_node,
            netsim_node_ee_cmd_node,
            # -- Data recording
            bag_record_node,
            # -- Visualization
            rviz_node,
            visualization_node,
        ]
    )
