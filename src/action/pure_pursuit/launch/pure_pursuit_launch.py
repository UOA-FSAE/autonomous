#!/usr/bin/env python3

# no parameters, run ros2 launch pure_pursuit pure_pursuit_launch.py 


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_prefix
from launch.actions import OpaqueFunction
import os


def generate_launch_description():
    return LaunchDescription([
        # Declare the 'vis' parameter
        DeclareLaunchArgument(
            'vis',
            default_value='false',
            description='Enable visualization?'
        ),

        Node(
            package='pure_pursuit',
            executable='pure_pursuit',
            name='pure_pursuit_controller',
            parameters=[{'vis': LaunchConfiguration('vis')}],
            output='screen',
        ),
    ])