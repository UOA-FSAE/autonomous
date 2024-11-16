from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    package_name = 'aruco_detection'

    launch_descriptions = [
        Node(
            package=package_name,
            executable='aruco_detection',
            name=package_name,
        )
    ]


    return LaunchDescription(launch_descriptions)