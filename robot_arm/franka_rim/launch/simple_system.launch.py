from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

K_INT = 3000.0
D_INT = 100.0

K_SPRING = 500.0
D_SPRING = 200.0
MASS = 10.0
# tau = 2 * MASS / D_SPRING

F_SCALE = 0.005


def generate_launch_description():
    #! Launch arguments
    delay_arg = DeclareLaunchArgument("delay", default_value="100", description="5g or fixed delay value in ms")

    rviz_config_file = os.path.join(
        get_package_share_directory("franka_rim"),
        "rviz",
        "simple_system.rviz",
    )

    #! Nodes
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    # Inverse3 Node
    inverse3_node = Node(
        package="inverse3_ros2",
        executable="inverse3_node",
        name="inverse3_node",
        output="screen",
        parameters=[
            {
                "restitution_stiffness": 1.0,
            }
        ],
    )

    # Simple Mass System
    simple_mass_system_node = Node(
        package="franka_rim",
        executable="simple_mass_system_node",
        name="simple_mass_system_node",
        output="screen",
        parameters=[
            {
                "mass": MASS,
                "spring_constant": K_SPRING,
                "damping_coefficient": D_SPRING,
                "simulation_frequency": 1000.0,
                "rim_publish_frequency": 500.0,
                "interface_stiffness": K_INT,
                "interface_damping": D_INT,
                "i3_state_topic": "/inverse3/state_delayed",  # Topic for Inverse3State messages
            }
        ],
    )

    # Network delay simulator for RIM messages
    rim_msg_delay_node = Node(
        package="network_sim",
        executable="network_sim_node",
        name="rim_delay_sim",
        parameters=[
            {
                "input_topic": "/rim_msg",
                "output_topic": "/rim_msg_delayed",
                "delay": LaunchConfiguration("delay"),
                "message_type": "arm_interfaces/msg/FrankaRIM",
            }
        ],
        output="screen",
    )

    i3_msg_delay_node = Node(
        package="network_sim",
        executable="network_sim_node",
        name="i3_cmd_delay",
        parameters=[
            {
                "input_topic": "/inverse3/state",
                "output_topic": "/inverse3/state_delayed",
                "delay": 0.0,  # LaunchConfiguration("delay"),
                "message_type": "teleop_interfaces/msg/Inverse3State",
            }
        ],
        output="screen",
    )

    # DelayRIM Node
    delay_rim_node = Node(
        package="franka_rim",
        executable="delay_rim_node",
        name="delay_rim_node",
        parameters=[
            {
                "rim_topic": "/rim_msg_delayed",
                "cmd_topic": "/simple_system/cmd",
                "control_period": 0.001,  # 1kHz
                "delay_compensation_method": "DelayRIM",  # Enable DelayRIM for debugging
                "interface_stiffness": 3000.0,
                "interface_damping": 2.0,
                "force_scaling": 1.0,  # No scaling for simple system
                "enable_debug": True,  # Enable debugging
                "debug_csv": True,  # Enable CSV logging
                "debug_rviz": True,  # Enable RViz visualization
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # Args
            delay_arg,
            # Nodes
            rviz_node,
            # inverse3_node,
            simple_mass_system_node,
            rim_msg_delay_node,
            i3_msg_delay_node,
            delay_rim_node,
        ]
    )
