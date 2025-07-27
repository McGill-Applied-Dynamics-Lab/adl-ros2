import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

K_INT = 0.1
D_INT = 0.0

F_SCALE = 0.005


def generate_launch_description():
    #! Launch arguments
    # Franka model parameters
    model_update_freq_arg = DeclareLaunchArgument(
        "model_update_freq", default_value="100.0", description="Frequency for model computation in Hz"
    )

    robot_urdf_filename_arg = DeclareLaunchArgument(
        "robot_urdf_filename", default_value="fr3_franka_hand.urdf", description="URDF filename for the robot model"
    )

    # Delay parameters
    delay_arg = DeclareLaunchArgument("delay", default_value="100", description="5g or fixed delay value in ms")

    delay_compensation = DeclareLaunchArgument(
        "compensation", default_value="delay_rim", description="Delay compensation method: delay_rim, zoh, or zoh_phi"
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("franka_rim"),
        "rviz",
        "fr3_rim.rviz",
    )

    #! Nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    visualization_node = Node(
        package="franka_rim",
        executable="rim_vis_node",
        name="rim_vis_node",
        parameters=[
            {
                "i3_sphere_size": 0.03,
                "rim_sphere_size": 0.03,
            }
        ],
        output="screen",
    )

    # Inverse3Node
    inverse3_node = Node(
        package="inverse3_ros2",
        executable="inverse3_node",
        name="inverse3_node",
        output="screen",
        parameters=[
            {
                # "input_topic": "/robot_state_delayed",
                "restitution_stiffness": 1.0,
            }
        ],
    )

    # FrankaModelNode
    franka_model_node = Node(
        package="franka_rim",
        executable="franka_model_node",
        name="franka_model_node",
        parameters=[
            {
                # "input_topic": "/robot_state_delayed",
                "input_topic": "/franka_robot_state_broadcaster/robot_state",
                "output_topic": "/fr3_model",
                "model_update_freq": LaunchConfiguration("model_update_freq"),
                "robot_urdf_filename": LaunchConfiguration("robot_urdf_filename"),
            }
        ],
        output="screen",
    )

    # FrankaRimNode
    franka_rim_node = Node(
        package="franka_rim",
        executable="franka_rim_node",
        name="franka_rim_node",
        output="screen",
        parameters=[
            {
                "rim_period": 1.0,
            }
        ],
    )

    # Network delay simulator for robot state
    delay_node_rim_msg = Node(
        package="network_sim",
        executable="network_sim_node",
        name="rim_msg_delay",
        parameters=[
            {
                "input_topic": "/fr3_rim",
                "output_topic": "/fr3_rim_delayed",
                "delay": LaunchConfiguration("delay"),
                "message_type": "arm_interfaces/msg/FrankaRIM",
            }
        ],
        output="screen",
    )

    delay_node_ee_cmd = Node(
        package="network_sim",
        executable="network_sim_node",
        name="ee_cmd_delay",
        parameters=[
            {
                "input_topic": "/teleop/ee_cmd_no_delay",
                "output_topic": "/teleop/ee_cmd",
                "delay": 0,  # LaunchConfiguration("delay"),
                "message_type": "arm_interfaces/msg/Teleop",
            }
        ],
        output="screen",
    )

    # DelayRIMNode
    delay_rim_node = Node(
        package="franka_rim",
        executable="delay_rim_node",
        name="delay_rim_node",
        parameters=[
            {
                "rim_topic": "/fr3_rim_delayed",
                "cmd_topic": "/teleop/ee_cmd_no_delay",
                "control_period": 0.001,  # 1kHz control rate
                "delay_compensation_method": LaunchConfiguration(
                    "compensation"
                ),  # "DelayRIM",  # 'DelayRIM', 'ZOH', or 'ZOHPhi
                "interface_stiffness": K_INT,
                "interface_damping": D_INT,
                "force_scaling": F_SCALE,  # No scaling for simple system
                "max_workers": 8,  # Threading parameter
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # -- Args
            model_update_freq_arg,
            robot_urdf_filename_arg,
            delay_arg,
            delay_compensation,
            # -- Nodes
            # inverse3_node,
            franka_model_node,
            franka_rim_node,
            delay_rim_node,
            # -- Network sim nodes
            delay_node_rim_msg,
            delay_node_ee_cmd,
            # -- Visualization
            rviz_node,
            visualization_node,
        ]
    )
