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
    inverse3_node = Node(  # noqa
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
    bag_filepath = Path("data") / bag_filename  # Store in data directory

    # Define topics to record (add/remove as needed)
    topics_to_record = [
        "/delayrim_visualization",
        "/f_ext_est",
        "/f_ext_robot",
        "/fr3/current_pose",
        "/fr3/franka/joint_states",
        "/fr3/franka_gripper/joint_states",
        "/fr3/franka_robot_state_broadcaster/current_pose",
        "/fr3/franka_robot_state_broadcaster/desired_end_effector_twist",
        "/fr3/franka_robot_state_broadcaster/desired_joint_states",
        "/fr3/franka_robot_state_broadcaster/external_joint_torques",
        "/fr3/franka_robot_state_broadcaster/external_wrench_in_base_frame",
        "/fr3/franka_robot_state_broadcaster/last_desired_pose",
        "/fr3/franka_robot_state_broadcaster/measured_joint_states",
        "/fr3/franka_robot_state_broadcaster/robot_state",
        "/fr3/interface_force",
        "/fr3/interface_torques",
        "/fr3/joint_states",
        "/fr3/robot_description",
        "/fr3/target_joint",
        "/fr3/target_pose",
        "/fr3/target_wrench",
        "/fr3_model",
        "/fr3_rim",
        "/fr3_rim_delayed",
        "/goal_pose",
        "/haptic_pose",
        "/haptic_twist",
        "/i3/pose",
        "/i3/twist",
        "/i3/wrench",
        "/rim/interface_force",
        "/rim/pose",
        "/rim/twist",
        "/tau_grav_est",
        "/tf",
        "/tf_static",
    ]

    # ROS 2 bag recording (uncompressed for easier analysis)
    bag_record_node = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            *topics_to_record,
            "--output",
            bag_filepath.as_posix(),
            "--storage",
            "sqlite3",
            "--max-bag-size",
            "0",  # No size limit
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
            inverse3_node,  # Add real inverse3 node back
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
