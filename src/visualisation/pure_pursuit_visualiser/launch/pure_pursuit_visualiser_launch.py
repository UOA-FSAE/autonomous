#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit_visualiser', 
            executable='visualiser',  
            name='publish_pure_pursuit_msgs',  
            output='screen'
        ),
    ])
