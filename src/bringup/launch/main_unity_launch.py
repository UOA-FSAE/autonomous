import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('bringup'), 'launch'),
                            '/agent_parameterised.py']), 
            #TODO make the arguments be based off of a config file
            launch_arguments={
                'ns': 'agent0',
                'sim': 'True'
                }.items()),
    ])