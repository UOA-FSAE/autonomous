from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    talker = Node(
        package='virtual_sensor',
        executable='talker',
        name='virtual_sensor_talker_node',
        output='screen'
    )
    
    listener = Node(
        package='virtual_sensor',
        executable='listener',
        name='virtual_sensor_listener_node',
        output='screen'
    )

    return LaunchDescription([
        talker,
        listener
    ])
