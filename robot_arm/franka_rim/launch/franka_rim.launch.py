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
    delay_arg = DeclareLaunchArgument("delay", default_value="100", description="5g or fixed delay value in ms")

    #! Nodes
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
    franka_rim_node = Node(package="franka_rim", executable="franka_rim_node", name="franka_rim_node", output="screen")

    # Network delay simulator for robot state
    rim_msg_delay_node = Node(
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

    ee_cmd_delay_node = Node(
        package="network_sim",
        executable="network_sim_node",
        name="ee_cmd_delay",
        parameters=[
            {
                "input_topic": "/teleop/ee_cmd_no_delay",
                "output_topic": "/teleop/ee_cmd",
                "delay": LaunchConfiguration("delay"),
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
                # "delay_compensation_method": "DelayRIM",
                "interface_stiffness": 3000.0,
                "interface_damping": 2.0,
                # "force_scaling": 0.02,  # Force scaling factor
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # Args
            model_update_freq_arg,
            robot_urdf_filename_arg,
            delay_arg,
            # Nodes
            inverse3_node,
            rim_msg_delay_node,
            ee_cmd_delay_node,
            franka_model_node,
            franka_rim_node,
            delay_rim_node,
        ]
    )
