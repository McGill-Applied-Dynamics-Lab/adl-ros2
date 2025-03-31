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
        default_value="false",
        description="Visualize the robot in Rviz",
    )

    load_gripper_launch_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value="true",
        description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded without an end-effector.",
    )

    hw_type_launch_arg = DeclareLaunchArgument(
        hw_type_parameter_name,
        default_value="real",
        description="Which hardware to use: 'real', 'fake', or 'isaac'",
        choices=["real", "fake", "isaac", "gazebo"],
    )

    #! MARK: Include fr3.launch.py
    fr3_launch_file = PathJoinSubstitution([FindPackageShare("robot_arm_bringup"), "launch", "fr3.launch.py"])

    fr3_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([fr3_launch_file]),
        launch_arguments={
            robot_ip_parameter_name: robot_ip,
            arm_id_parameter_name: arm_id,
            load_gripper_parameter_name: load_gripper,
            hw_type_parameter_name: hw_type,
            use_rviz_parameter_name: use_rviz,
        }.items(),
    )

    #! MARK: Additional Controllers
    # Spawn controllers specific to fr3_interface
    controllers_list = [  # They are moved to the `fr3.launch.py`
        # "joint_trajectory_controller --inactive",
        # "joint_velocity_controller",
        # "joint_velocity_example_controller",
        # "my_vel_controller",
        # "move_to_start_example_controller",
        # "cartesian_pose_controller",
        # "cartesian_vel_controller",
        # "move_to_start_controller --inactive",
        # "move_to_start_example_controller --inactive",
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

    franka_interface_node = Node(
        package="robot_arm_interface",
        executable="fr3_interface",
        name="fr3_interface",
        output="screen",
        # prefix=["gdbserver :3000"],
    )

    delayed_franka_interface_node = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[franka_interface_node],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    #! MARK: Launch Description
    launch_description = LaunchDescription(
        [
            #! --- Launch args ---
            robot_ip_launch_arg,
            arm_id_launch_arg,
            use_rviz_launch_arg,
            load_gripper_launch_arg,
            hw_type_launch_arg,
            #! --- Include fr3.launch.py ---
            fr3_launch_include,
            #! --- Additional nodes ---
            joy_node,
            delayed_franka_interface_node,
        ]
        + ros_controllers_nodes
    )

    return launch_description
