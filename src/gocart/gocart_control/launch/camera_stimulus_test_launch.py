from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python import get_package_share_directory
import os

def generate_launch_description():  
    return LaunchDescription([
        DeclareLaunchArgument(
        'can_id',
        default_value='0x300',
        description='The frame ID for the CAN messages containing Ackermann commands that are sent to the car'
        ),

        DeclareLaunchArgument(
            'candapter_topic',
            default_value='pub_raw_can',
            description='The subscriber and publisher topic for the Can Adapter node'
        ),
        
        Node(
            package='gocart_control',
            executable='ack_to_can_node',
            name='ack_to_can_node',
            parameters=[{'can_id': LaunchConfiguration('can_id')}],
        ),

        Node(
            namespace='moa',
            package='zed_perception',
            executable='zed_launch_node',
            name='perception'
        ),

        Node(
            namespace='moa',
            package='cone_mapping',
            executable='kalman_filter',
            name='kalman_filter'
        ),
        
        Node(
            namespace='moa',
            package='fsae_planning',
            executable='fasttube',
            name='centerline_planner'
        ),
        
        # # uncomment when CAN interface is completed
        # Node(
        #     package='gocart_driver',
        #     executable='can_interface_jnano',
        #     name='can_interface_jnano'),
        
        Node(
            package='CanTalk',
            executable='candapter_node',
            name='candapter_node',
            remappings=[('can', LaunchConfiguration('candapter_topic'))],
        ),
        Node(
            package='gocart_control',
            executable='mock_stimulus',
            name='mock_stimulus',
        ),
  ])
