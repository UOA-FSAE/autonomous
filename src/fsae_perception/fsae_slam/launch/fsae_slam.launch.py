from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fsae_slam',
            executable='cone_landmark_mapper',
            name='cone_landmark_mapper',
            output='screen',
        ),
    ])
