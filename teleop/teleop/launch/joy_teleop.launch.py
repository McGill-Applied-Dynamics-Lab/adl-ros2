from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    # arm_id_parameter_name = "arm_id"
    # device_id = "device_id"  # Maybe to add?

    # arm_id = LaunchConfiguration(arm_id_parameter_name)

    #! MARK: Configs
    # rviz_file = os.path.join(
    #     get_package_share_directory("franka_description"),
    #     "rviz",
    #     "visualize_franka.rviz",
    # )

    # franka_controllers = PathJoinSubstitution([FindPackageShare("robot_arm_bringup"), "config", "controllers.yaml"])

    #! MARK: Launch Arguments

    # arm_id_launch_arg = DeclareLaunchArgument(
    #     arm_id_parameter_name,
    #     default_value="fr3",
    #     description="ID of the type of arm used. Supported values: fer, fr3, fp3",
    # )

    #! MARK: Nodes
    joy_teleop_node = Node(package="teleop", executable="joy_teleop", name="joy_teleop", output="screen", parameters=[])

    #! MARK: Launch Description
    launch_description = LaunchDescription(
        [
            #! --- Launch args ---
            # ...,
            #! --- Nodes ---
            joy_teleop_node,
        ]
    )
    return launch_description
