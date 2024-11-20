#!/usr/bin/env python3

# no parameters, run ros2 launch pure_pursuit pure_pursuit_launch.py 


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit', 
            executable='pure_pursuit',  
            name='Pure_Pursuit_Controller',  
            output='screen'
        ),
    ])
