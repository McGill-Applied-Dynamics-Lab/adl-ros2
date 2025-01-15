from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import xacro
import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # ------- Config ------
    # Get URDF via xacro
    franka_xacro_filepath = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3" + ".urdf.xacro",
    )

    robot_description_content = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            "hw_type": "fake",
            "ros2_control": "true",
            "arm_id": "fr3",
            "robot_ip": "dont-care",
            "hand": "false",
            "use_fake_hardware": "false",
            "fake_sensor_commands": "false",
        },
    ).toprettyxml(indent="  ")

    robot_description = {"robot_description": robot_description_content}

    franka_controllers = PathJoinSubstitution(
        [FindPackageShare("franka_bringup"), "config", "controllers.yaml"]
    )

    # rviz_file = os.path.join(
    #     get_package_share_directory("franka_description"),
    #     "rviz",
    #     "visualize_franka.rviz",
    # )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("franka_description"), "rviz", "visualize_franka.rviz"]
    )

    # -------- Nodes --------
    # To publish the state of the robot joints. Subscribes to the topics in `source_list`
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "franka/joint_states",
                    "franka_gripper/joint_states",
                ],
                "rate": 30,
            }
        ],
    )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[franka_controllers],
    #     output="both",
    # )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            franka_controllers,
            robot_description,
            # {"robot_description": robot_description},
            {"arm_id": "dont-care"},
            {"load_gripper": "false"},
        ],
        remappings=[
            ("joint_states", "franka/joint_states"),
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    franka_robot_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_robot_state_broadcaster"],
        parameters=[{"arm_id": "fr3"}],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", franka_controllers],
    )

    # # Delay start of joint_state_broadcaster after `robot_controller`
    # delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_config_file],
        condition=IfCondition(gui),
    )

    # # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    nodes = [
        joint_state_publisher_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        franka_robot_state_broadcaster_spawner,
        robot_controller_spawner,
        # delay_joint_state_broadcaster_after_robot_controller_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
