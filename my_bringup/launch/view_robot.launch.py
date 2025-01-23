#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def robot_description_dependent_nodes_spawner(
    context: LaunchContext,
    robot_ip,
    arm_id,
    # use_fake_hardware,
    # fake_sensor_commands,
    load_gripper,
    hw_type,
):
    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = "false"  # context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = (
        "false"  # context.perform_substitution(fake_sensor_commands)
    )
    load_gripper_str = context.perform_substitution(load_gripper)
    hw_type_str = context.perform_substitution(hw_type)

    franka_xacro_filepath = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        arm_id_str,
        arm_id_str + ".urdf.xacro",
    )

    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            "hw_type": hw_type_str,
            "ros2_control": "true",
            "arm_id": arm_id_str,
            "robot_ip": robot_ip_str,
            "hand": load_gripper_str,
            "use_fake_hardware": use_fake_hardware_str,
            "fake_sensor_commands": fake_sensor_commands_str,
        },
    ).toprettyxml(indent="  ")

    franka_controllers = PathJoinSubstitution(
        [FindPackageShare("my_bringup"), "config", "controllers.yaml"]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                franka_controllers,
                # {"robot_description": robot_description},
                {"arm_id": arm_id},
                {"load_gripper": load_gripper},
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
        ),
    ]


def generate_launch_description():
    arm_id_parameter_name = "arm_id"
    robot_ip_parameter_name = "robot_ip"
    load_gripper_parameter_name = "load_gripper"
    hw_type_parameter_name = "hw_type"
    use_rviz_parameter_name = "use_rviz"

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    hw_type = LaunchConfiguration(hw_type_parameter_name)  # fake, real, or isaac
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    #! Configs
    rviz_file = os.path.join(
        get_package_share_directory("franka_description"),
        "rviz",
        "visualize_franka.rviz",
    )

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ip,
            arm_id,
            # use_fake_hardware,
            # fake_sensor_commands,
            load_gripper,
            hw_type,
        ],
    )

    # MARK: Launch Arguments
    robot_ip_launch_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        default_value="dont-care",
        description="Hostname or IP address of the robot.",
    )

    arm_id_launch_arg = DeclareLaunchArgument(
        arm_id_parameter_name,
        default_value="fr3",
        description="ID of the type of arm used. Supported values: fer, fr3, fp3",
    )

    use_rviz_launch_arg = DeclareLaunchArgument(
        use_rviz_parameter_name,
        default_value="true",
        description="Visualize the robot in Rviz",
    )

    load_gripper_launch_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value="true",
        description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded "
        "without an end-effector.",
    )

    hw_type_launch_arg = DeclareLaunchArgument(
        hw_type_parameter_name,
        default_value="fake",
        description="Which hardware to use: 'real', 'fake', or 'isaac'",
        choices=["real", "fake", "isaac", "gazebo"],
    )

    # MARK: Nodes
    # To publish the state of the robot joints. Subscribes to the topics in `source_list`
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        # name="joint_state_publisher",
        # parameters=[
        #     {
        #         "source_list": [
        #             "franka/joint_states",
        #             "franka_gripper/joint_states",
        #         ],
        #         "rate": 30,
        #     }
        # ],
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
        parameters=[{"arm_id": arm_id}],
        output="screen",
        condition=IfCondition(PythonExpression(["'", hw_type, "' == 'real'"])),
    )

    # gripper_launch_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("franka_gripper"),
    #                     "launch",
    #                     "gripper.launch.py",
    #                 ]
    #             )
    #         ]
    #     ),
    #     launch_arguments={
    #         robot_ip_parameter_name: robot_ip,
    #         use_fake_hardware_parameter_name: use_fake_hardware,
    #     }.items(),
    #     condition=IfCondition(load_gripper),
    # )

    # # To remap the joints names
    # isaac_topics_remapper_node = Node(
    #     package="isaac_joint_state_remapper",
    #     executable="isaac_joint_state_remapper",
    #     name="isaac_joint_state_remapper",
    #     condition=IfCondition(PythonExpression(["'", hw_type, "' == 'isaac'"])),
    # )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_file],
        condition=IfCondition(use_rviz),
    )

    launch_description = LaunchDescription(
        [
            # -- Launch args
            robot_ip_launch_arg,
            arm_id_launch_arg,
            use_rviz_launch_arg,
            # use_fake_hw_launch_arg,
            # fake_sensor_commands_launch_arg,
            load_gripper_launch_arg,
            hw_type_launch_arg,
            # -- Nodes
            joint_state_publisher_node,
            robot_description_dependent_nodes_spawner_opaque_function,
            joint_state_broadcaster_spawner,
            franka_robot_state_broadcaster_spawner,
            # gripper_launch_description,
            rviz2_node,
            # isaac_topics_remapper_node,
        ]
    )

    return launch_description
