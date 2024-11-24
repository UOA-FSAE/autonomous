from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='steer_torque_from_ackermann',
            executable='steer_torque_from_ackermann',
            name='steer_torque_converter'
        )
    ])