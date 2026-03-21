from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    can_decoder = Node(
        package='gocart_driver',
        executable='can_decoder_jnano',
        name='can_decoder_node',
        output='screen'
    )
    
    return LaunchDescription([
        can_decoder,
    ])