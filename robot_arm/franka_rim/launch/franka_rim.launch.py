import os
from datetime import datetime

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory

from pathlib import Path

K_INT = 500.0
D_INT = 41.71

F_SCALE = 0.1


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

    fake_i3_arg = DeclareLaunchArgument(
        "fake_i3", default_value="false", description="Use haptic simulator instead of real Inverse3 device"
    )

    save_data_arg = DeclareLaunchArgument(
        "save_data", default_value="false", description="Record all data to a ROS 2 bag file"
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

    # Inverse3Node (real device)
    inverse3_node = Node(  # noqa
        package="inverse3_ros2",
        executable="inverse3_node",
        name="inverse3_node",
        output="screen",
        parameters=[
            {
                "restitution_stiffness": 1.0,
            }
        ],
        condition=UnlessCondition(LaunchConfiguration("fake_i3")),
    )

    # Haptic Simulator Node (fake device)
    i3_sim_node = Node(
        package="franka_rim",
        executable="i3_sim_node",
        name="i3_sim_node",
        output="screen",
        parameters=[
            {
                "publish_frequency": 1000.0,
                "sine_frequency": 0.2,
                "amplitude": 0.1,
                "n_cycles": 3,
                "wait_to_start": True,
            }
        ],
        condition=IfCondition(LaunchConfiguration("fake_i3")),
    )

    # Franka Model Node
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

    # Franka Rim Node
    franka_rim_node = Node(
        package="franka_rim",
        executable="franka_rim_node",
        name="franka_rim_node",
        output="screen",
        parameters=[
            {
                "rim_period": 0.05,  # 20 Hz
                "interface_stiffness": K_INT,
                "interface_damping": D_INT,
            }
        ],
    )

    # Network delay simulator for robot state
    delay_node_rim_msg = Node(  # rim msg delay
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

    delay_node_ee_cmd = Node(  # teleop cmd delay
        package="network_sim",
        executable="network_sim_node",
        name="ee_cmd_delay",
        parameters=[
            {
                "input_topic": "/osc_pd_controller/goal_pre_delay",
                "output_topic": "/osc_pd_controller/goal",
                "delay": 0,  # LaunchConfiguration("delay"),
                "message_type": "geometry_msgs/msg/PointStamped",  # "arm_interfaces/msg/Teleop",
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
                # "cmd_topic": "/teleop/ee_cmd_no_delay",
                "cmd_topic": "/osc_pd_controller/goal_pre_delay",  # For teleop commands
                "control_period": 0.001,  # 1kHz control rate
                "delay_compensation_method": LaunchConfiguration(
                    "compensation"
                ),  # "DelayRIM",  # 'DelayRIM', 'ZOH', or 'ZOHPhi
                "interface_stiffness": K_INT,
                "interface_damping": D_INT,
                "force_scaling": F_SCALE,  # No scaling for simple system
                "max_workers": 8,  # Threading parameter
                "i3_position_scale": 1.5,  # Mapping between I3 and end-effector position
            }
        ],
        output="screen",
    )

    # ROS 2 bag recording
    # Generate timestamped bag file name
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_filename = f"franka_rim_data_{timestamp}"
    bag_filepath = Path("logs") / bag_filename

    # # Topics to record - comprehensive list of all DelayRIM experiment data
    # topics_to_record = [
    #     "/inverse3/state",  # Haptic device state
    #     "/fr3_model",  # Robot model data
    #     "/fr3_rim",  # RIM messages (before delay)
    #     "/fr3_rim_delayed",  # RIM messages (after delay simulation)
    #     "/rim_state",  # Estimated RIM state from DelayRIM
    #     "/rim_interface_force",  # Interface forces computed by DelayRIM
    #     "/inverse3/wrench_des",  # Force feedback to haptic device
    #     "/osc_pd_controller/goal_pre_delay",  # Robot commands (before delay)
    #     "/osc_pd_controller/goal",  # Robot commands (after delay)
    #     "/osc_pd_controller/cartesian_force",  # Robot controller forces
    #     "/franka_robot_state_broadcaster/robot_state",  # Raw robot state
    #     "/delayrim_visualization",  # Visualization markers
    #     "/f_ext_est",  # External force estimates
    #     "/f_ext_robot",  # Robot's force estimates
    # ]

    bag_record_node = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--all",  # Record all topics
            "--output",
            bag_filepath.as_posix(),  # Use the generated bag file path
            "--storage",
            "sqlite3",
            "--max-bag-size",
            "0",  # No size limit
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("save_data")),
    )

    return LaunchDescription(
        [
            # -- Args
            model_update_freq_arg,
            robot_urdf_filename_arg,
            delay_arg,
            delay_compensation,
            fake_i3_arg,
            save_data_arg,
            # -- Nodes
            # inverse3_node,
            i3_sim_node,
            franka_model_node,
            franka_rim_node,
            delay_rim_node,
            # -- Network sim nodes
            delay_node_rim_msg,
            delay_node_ee_cmd,
            # -- Data recording
            bag_record_node,
            # -- Visualization
            rviz_node,
            visualization_node,
        ]
    )
