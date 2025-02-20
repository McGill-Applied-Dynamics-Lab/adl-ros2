from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    # IncludeLaunchDescription,
    # OpaqueFunction,
    ExecuteProcess,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition

# from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    i3_teleop_node = Node(package="teleop", executable="i3_teleop", name="i3_teleop", output="screen", parameters=[])

    inverse3_node = Node(
        package="inverse3_ros2", executable="inverse3_node", name="inverse3_node", output="screen", parameters=[]
    )

    #! MARK: Launch Description
    launch_description = LaunchDescription(
        [
            #! --- Launch args ---
            # ...,
            #! --- Nodes ---
            i3_teleop_node,
            inverse3_node,
        ]
    )
    return launch_description
