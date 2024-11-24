from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planning_visualiser',
            executable='visualiser2',
            name='visualise_trajectories_demo'
        ),
        Node(
            package='path_planning_visualiser',
            executable='visualiser',
            name='visualise_trajectories'
        )
    ])