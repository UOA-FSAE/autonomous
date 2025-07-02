from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    can_decoder = Node(
        package='moa_driver',
        executable='can_decoder_jnano',
        name='can_decoder_node',
        output='screen'
    )
    
    can_interface = Node(
        package='moa_driver',
        executable='can_interface_jnano',
        name='can_interface_node',
        output='screen'
    )

    return LaunchDescription([
        can_decoder,
        can_interface
    ])