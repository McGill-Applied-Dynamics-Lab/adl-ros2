from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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
    delay_type_arg = DeclareLaunchArgument(
        "delay_type", default_value="5g", description="Type of delay simulation: 5g or fixed"
    )

    fixed_delay_arg = DeclareLaunchArgument(
        "fixed_delay", default_value="0.05", description="Fixed delay value in seconds (used when delay_type=fixed)"
    )

    #! Nodes
    # FrankaModelNode
    franka_model_node = Node(
        package="franka_rim",
        executable="franka_model_node",
        name="franka_model_node",
        parameters=[
            {
                "input_topic": "/robot_state_delayed",
                "output_topic": "/fr3_model",
                "model_update_freq": LaunchConfiguration("model_update_freq"),
                "robot_urdf_filename": LaunchConfiguration("robot_urdf_filename"),
            }
        ],
        output="screen",
    )

    # FrankaRimNode
    franka_rim_node = Node(package="franka_rim", executable="franka_rim_node", name="franka_rim_node", output="screen")

    # Network delay simulator for robot state
    network_delay_node = Node(
        package="network_sim",
        executable="network_sim_5g",
        name="robot_state_delay_sim",
        parameters=[
            {
                "input_topic": "/franka_robot_state_broadcaster/robot_state",
                "output_topic": "/robot_state_delayed",
                "delay": LaunchConfiguration("fixed_delay"),
                "delay_type": LaunchConfiguration("delay_type"),
                "message_type": "franka_msgs/msg/FrankaRobotState",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # Args
            model_update_freq_arg,
            robot_urdf_filename_arg,
            delay_type_arg,
            fixed_delay_arg,
            # Nodes
            network_delay_node,
            franka_model_node,
            franka_rim_node,
        ]
    )
