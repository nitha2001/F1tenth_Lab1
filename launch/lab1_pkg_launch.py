from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker_node',
            parameters=[
            {'steering_angle': 20.0},
            {'speed': 25.0}
            ],


        ),
        Node(
            package='lab1_pkg',
            executable='relay',
            name='relay_node'
        ),
    ])
