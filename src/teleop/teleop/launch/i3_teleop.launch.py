from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
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
