from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='acoustic_sensing',
            executable='acoustic_receiver_node',
            name='acoustic_receiver_node',
            output='screen',
            parameters=[
                {"port": 1234},
                {"two_rf_mode": True},   # set False if using serverSocket.c
                {"buf_size": 4000},
                {"frame_id": "beaglebone_acoustic"}
            ]
        )
    ])
