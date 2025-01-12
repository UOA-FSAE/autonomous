#!/usr/bin/env python3


# no parameters, run ros2 launch head_to_goal_control head_to_goal_launch.py 

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='head_to_goal_control', 
            executable='controller',  
            name='Head_To_Goal_Controller',  
            output='screen'
        ),
    ])
