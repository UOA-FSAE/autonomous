from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        package_name = 'path_planning_visualiser'
        Node(
            package=package_name,
            executable='visualise_action_demo',
            name='visualise_action_demo'
        ),
        Node(
            package=package_name,
            executable='visualise_trajectories_demo',
            name='visualise_trajectories_demo'
        ),
        Node(
            package=package_name,
            executable='visualise_trajectories',
            name='visualise_trajectories'
        )
    ])