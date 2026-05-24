import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    custom_camera_node = Node(
        package='zed_perception',
        executable='zed_launch_node',
        name='zed_camera',
        output='screen',
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne'),
                'launch',
                'velodyne-all-nodes-VLP16-launch.py'
            )
        )
    )

    return LaunchDescription([
        custom_camera_node,
        velodyne_launch,
    ])
