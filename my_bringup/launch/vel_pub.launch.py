import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "/velocity_controller/commands",
                    "std_msgs/msg/Float64MultiArray",
                    "data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                ],
                output="screen",
            )
        ]
    )
