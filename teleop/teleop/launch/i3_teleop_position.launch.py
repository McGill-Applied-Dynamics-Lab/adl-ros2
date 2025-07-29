from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    #! Launch arguments
    fake_i3_arg = DeclareLaunchArgument(
        "fake_i3", default_value="false", description="Use haptic simulator instead of real I3 device"
    )

    #! MARK: Nodes
    i3_teleop_node = Node(
        package="teleop",
        executable="i3_teleop_position",
        name="i3_teleop",
        output="screen",
        parameters=[
            {
                "command_topic": "/osc_pd_controller/goal",
                "pos_radius": 0.3,  # Radius for the position control region
                "position_scale": 1.0,  # Scale for the position of the i3
            }
        ],
    )

    haptic_simulator_node = Node(
        package="franka_rim",
        executable="haptic_simulator_node",
        name="i3_simulator_node",
        output="screen",
        parameters=[
            {
                "publish_frequency": 100.0,  # 100Hz
                "sine_frequency": 2.0,  # 2 cycles per second
                "amplitude": 0.05,  # 5cm amplitude
                "duration": 1.0,  # 1 second duration (completes 2 cycles)
                "center_x": 0.0,
                "center_y": 0.0,
                "center_z": 0.0,
            }
        ],
        condition=IfCondition(LaunchConfiguration("fake_i3")),
    )

    inverse3_node = Node(
        package="inverse3_ros2",
        executable="inverse3_node",
        name="inverse3_node",
        output="screen",
        parameters=[
            {
                "restitution_stiffness": 0.1,  # 20.0
            }
        ],
        condition=UnlessCondition(LaunchConfiguration("fake_i3")),
    )

    #! MARK: Launch Description
    launch_description = LaunchDescription(
        [
            #! --- Launch args ---
            fake_i3_arg,
            #! --- Nodes ---
            i3_teleop_node,
            haptic_simulator_node,
            inverse3_node,
        ]
    )
    return launch_description
