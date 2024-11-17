from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    stanley_controller = Node(
        package='stanley_controller',
        executable='stanley_controller.py',
        name='stanley_controller',
        output='screen'
    )
   

    return LaunchDescription([
        stanley_controller
    ])