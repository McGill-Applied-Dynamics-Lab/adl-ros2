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
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    # OpaqueFunction,
    ExecuteProcess,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# import xacro


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

    #! MARK: Configs
    rviz_file = os.path.join(
        get_package_share_directory("franka_description"),
        "rviz",
        "visualize_franka.rviz",
    )

    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.urdf.xacro",
    )

    robot_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_xacro_file,
            " hand:=true",
            " robot_ip:=",
            robot_ip,
            " use_fake_hardware:=false",
            " fake_sensor_commands:=false",
            " ros2_control:=true",
            " hw_type:=",
            hw_type,
            " arm_id:=",
            arm_id,
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_config, value_type=str)}

    franka_controllers = PathJoinSubstitution([FindPackageShare("robot_arm_bringup"), "config", "controllers.yaml"])

    #! MARK: Launch Arguments
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
        description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded without an end-effector.",
    )

    hw_type_launch_arg = DeclareLaunchArgument(
        hw_type_parameter_name,
        default_value="isaac",
        description="Which hardware to use: 'real', 'fake', or 'isaac'",
        choices=["real", "fake", "isaac", "gazebo"],
    )

    #! MARK: Nodes
    # ** Publish TFs **

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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ** Gripper **
    gripper_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_gripper"),
                        "launch",
                        "gripper.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            robot_ip_parameter_name: robot_ip,
            # use_fake_hardware_parameter_name: use_fake_hardware,
        }.items(),
        condition=IfCondition(load_gripper),
    )

    # To remap the joints names
    isaac_topics_remapper_node = Node(
        package="isaac_sim_ros",
        executable="isaac_joint_state_remapper",
        name="isaac_joint_state_remapper",
        condition=IfCondition(PythonExpression(["'", hw_type, "' == 'isaac'"])),
    )

    # ** Rviz **
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_file],
        condition=IfCondition(use_rviz),
    )

    # ** Controllers **
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            franka_controllers,
            robot_description,
            {"arm_id": arm_id},
            {"load_gripper": load_gripper},
        ],
        remappings=[
            ("joint_states", "franka/joint_states"),
            # ("/controller_manager/robot_description", "/robot_description"),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    # Spawn the ros2_control controllers
    controllers_list = [
        "joint_state_broadcaster",
        "cartesian_vel_controller",
        # "model_example_controller",
    ]

    ros_controllers_nodes = []
    for controller in controllers_list:
        ros_controllers_nodes += [
            ExecuteProcess(
                cmd=[f"ros2 run controller_manager spawner {controller}"],
                shell=True,
                output="screen",
            )
        ]

    # Node to read Franka data and publish it (e.g. joint states, pose, ...)
    franka_robot_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_robot_state_broadcaster"],
        parameters=[{"arm_id": arm_id}],
        output={
            "stdout": "log",
            "stderr": "log",
        },
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        condition=IfCondition(PythonExpression(["'", hw_type, "' == 'real'"])),
    )

    franka_interface_node = Node(
        package="robot_arm_interface",
        executable="fr3_interface",
        name="fr3_interface",
        output="screen",
        # prefix=["gdbserver :3000"],
    )

    # delayed_franka_interface_node = TimerAction(
    #     period=1.0,  # Delay in seconds
    #     actions=[franka_interface_node],
    # )

    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     name="joy_node",
    #     output="screen",
    # )

    #! MARK: Launch Description
    launch_description = LaunchDescription(
        [
            #! --- Launch args ---
            robot_ip_launch_arg,
            arm_id_launch_arg,
            use_rviz_launch_arg,
            # use_fake_hw_launch_arg,
            # fake_sensor_commands_launch_arg,
            load_gripper_launch_arg,
            hw_type_launch_arg,
            #! --- Nodes ---
            joint_state_publisher_node,
            robot_state_publisher,
            gripper_launch_description,
            ros2_control_node,
            franka_robot_state_broadcaster_spawner,
            rviz2_node,
            isaac_topics_remapper_node,
            # joy_node,
            # delayed_franka_interface_node,
        ]
        + ros_controllers_nodes
    )

    return launch_description
